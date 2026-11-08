--[[
    ArduPilot Lua - NavPy watchdog for ArduPlane (docs/design/navpy_recovery.md, step 7).

    NavPy's in-process recovery holds the UAV while NavPy runs. This script covers a NavPy that is dead or whose
    link is gone: it receives NavPy's companion HEARTBEAT (this aircraft's system id, component AAS_NWD_COMP,
    MAV_TYPE_ONBOARD_CONTROLLER) and, in a GUIDED that NavPy was alive during:

      WATCH --no NavPy heartbeat for AAS_NWD_TMO s--> HOLD (LOITER)
      HOLD  --a NavPy heartbeat again, mode still its LOITER--> AUTO, WATCH (the mission resumes where it was)
      HOLD  --AAS_NWD_RTL s without one--> RTL (not undone; the operator decides)

    It does nothing in any other mode, and stands down when the mode leaves its LOITER or RTL, so pilot and
    failsafe modes always win. Times are the autopilot's own clock (aircraft time). The loop runs under pcall,
    and its state is sent once a second as NAMED_VALUE_FLOAT "NAVWD" (STATE_* below), so NavPy and the GCS can
    tell when the watchdog is not running.
--]]

local PARAM_TABLE_KEY = 88   -- the AAS_ table (aas_params.lua); this script owns indices 20-23
assert(param:add_table(PARAM_TABLE_KEY, "AAS_", 30), "nav_watchdog: failed to add AAS_ parameter table (key 88)")
assert(param:add_param(PARAM_TABLE_KEY, 20, "NWD_ENABLE", 1), "nav_watchdog: add_param NWD_ENABLE")
assert(param:add_param(PARAM_TABLE_KEY, 21, "NWD_TMO", 1.5), "nav_watchdog: add_param NWD_TMO")   -- s, NAV_HOLD_TRIGGER_S
assert(param:add_param(PARAM_TABLE_KEY, 22, "NWD_RTL", 30), "nav_watchdog: add_param NWD_RTL")    -- s, NAV_HOLD_RECOVERY_S
assert(param:add_param(PARAM_TABLE_KEY, 23, "NWD_COMP", 191), "nav_watchdog: add_param NWD_COMP") -- MAV_COMP_ID_ONBOARD_COMPUTER

local TICK_MS = 100
local REPORT_MS = 1000
local RX_QUEUE = 10
local MAX_RX_PER_TICK = RX_QUEUE    -- a full queue per call, within SCR_VM_I_COUNT even if every frame needs a CRC
local MSG_HEARTBEAT = 0
local HEARTBEAT_CRC_EXTRA = 50
local MAV_TYPE_ONBOARD_CONTROLLER = 18
local MODE_AUTO, MODE_RTL, MODE_LOITER, MODE_GUIDED = 10, 11, 12, 15   -- ArduPlane mode numbers
local STATE_OFF, STATE_WATCH, STATE_HOLD, STATE_RTL = -1, 0, 1, 2

local state = STATE_WATCH
local last_hb_ms = nil              -- millis of the last NavPy heartbeat
local navpy_guided = false          -- in GUIDED, and NavPy was alive during it
local hold_since_ms, hold_hb_ms = 0, nil
local last_report_ms = nil
local last_text = nil

local function text(severity, msg)
    if msg ~= last_text then
        gcs:send_text(severity, msg)
        last_text = msg
    end
end

local function own_sysid()
    return param:get("MAV_SYSID") or param:get("SYSID_THISMAV")
end

-- mavlink_message_t: checksum(2) magic len incompat compat seq sysid compid msgid(3), payload from byte 13;
-- HEARTBEAT's payload is custom_mode(4) type autopilot base_mode system_status mavlink_version. ArduPilot also
-- hands scripts frames that failed their CRC (GCS_Common.cpp, MAVLINK_FRAMING_BAD_CRC), so the CRC is checked.
local function crc_ok(msg)
    local len = string.byte(msg, 4)
    local covered
    if string.byte(msg, 3) == 0xFD then
        covered = string.sub(msg, 4, 12 + len)
    else                                                     -- MAVLink 1: len seq sysid compid msgid payload
        covered = string.char(len) .. string.sub(msg, 7, 10) .. string.sub(msg, 13, 12 + len)
    end
    local crc = 0xFFFF
    for i = 1, #covered + 1 do
        local byte = (i <= #covered) and string.byte(covered, i) or HEARTBEAT_CRC_EXTRA
        local tmp = (byte ~ (crc & 0xFF)) & 0xFF
        tmp = (tmp ~ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ~ (tmp << 8) ~ (tmp << 3) ~ (tmp >> 4)) & 0xFFFF
    end
    return crc == string.unpack("<H", msg, 1)
end

local function receive_heartbeats(now)
    local sysid, comp = own_sysid(), param:get("AAS_NWD_COMP")
    local seen = false                  -- once NavPy's heartbeat is seen, the rest are only drained
    for _ = 1, MAX_RX_PER_TICK do
        local msg = mavlink:receive_chan()
        if msg == nil then
            return
        end
        if not seen then
            local _, src_sys, src_comp, msgid = string.unpack("<BBBI3", msg, 7)
            if msgid == MSG_HEARTBEAT and src_sys == sysid and src_comp == comp
                and string.unpack("<B", msg, 17) == MAV_TYPE_ONBOARD_CONTROLLER and crc_ok(msg) then
                last_hb_ms, seen = now, true
            end
        end
    end
end

local function enter(new_state, severity, msg)
    state = new_state
    text(severity, msg)
end

local function watch(now, mode, fresh)
    if mode ~= MODE_GUIDED or not arming:is_armed() then
        navpy_guided = false
    elseif fresh then
        navpy_guided = true
    elseif navpy_guided then
        if vehicle:set_mode(MODE_LOITER) then
            hold_since_ms, hold_hb_ms = now, last_hb_ms
            enter(STATE_HOLD, 2, "NavPy watchdog: no NavPy heartbeat, LOITER")
        else
            text(3, "NavPy watchdog: LOITER refused, retrying")
        end
    end
end

local function hold(now, mode, fresh)
    if mode ~= MODE_LOITER then
        navpy_guided = false
        enter(STATE_WATCH, 4, "NavPy watchdog: mode changed, standing down")
        return
    end
    if fresh and last_hb_ms ~= hold_hb_ms then
        if vehicle:set_mode(MODE_AUTO) then
            navpy_guided = false
            enter(STATE_WATCH, 4, "NavPy watchdog: NavPy back, AUTO")
            return
        end
        text(3, "NavPy watchdog: AUTO refused, retrying")
    end
    if now - hold_since_ms >= param:get("AAS_NWD_RTL") * 1000 then
        if vehicle:set_mode(MODE_RTL) then
            enter(STATE_RTL, 2, "NavPy watchdog: no NavPy for the hold, RTL")
        else
            text(3, "NavPy watchdog: RTL refused, retrying")
        end
    end
end

local function step(now)
    receive_heartbeats(now)
    if param:get("AAS_NWD_ENABLE") ~= 1 then
        if state ~= STATE_OFF then
            navpy_guided = false
            enter(STATE_OFF, 4, "NavPy watchdog disabled")
        end
        return
    end
    if state == STATE_OFF then
        enter(STATE_WATCH, 6, "NavPy watchdog enabled")
    end
    local mode = vehicle:get_mode()
    local fresh = last_hb_ms ~= nil and now - last_hb_ms < param:get("AAS_NWD_TMO") * 1000
    if state == STATE_WATCH then
        watch(now, mode, fresh)
    elseif state == STATE_HOLD then
        hold(now, mode, fresh)
    elseif mode ~= MODE_RTL then
        navpy_guided = false
        enter(STATE_WATCH, 4, "NavPy watchdog: mode changed, standing down")
    end
end

local function report(now)
    if last_report_ms == nil or now - last_report_ms >= REPORT_MS then
        gcs:send_named_float("NAVWD", state)
        last_report_ms = now
    end
end

local function update()
    local now = millis():toint()
    local ok, err = pcall(step, now)
    if not ok then
        text(3, "NavPy watchdog error: " .. string.sub(tostring(err), 1, 26))
    end
    report(now)
    return update, TICK_MS
end

-- (queue length, message ids): the order lua_bindings.cpp reads, not docs.lua's. The receive queue is the
-- scripting engine's, shared by every script; no other script here receives MAVLink.
mavlink:init(RX_QUEUE, 1)
mavlink:register_rx_msgid(MSG_HEARTBEAT)
gcs:send_text(6, "NavPy watchdog loaded")
return update, TICK_MS
