--[[
    ArduPilot Lua - NavPy watchdog for ArduPlane (docs/design/navpy_recovery.md, step 7).

    NavPy's in-process recovery holds the UAV while NavPy runs. This script covers a NavPy that is dead or whose
    link is gone: it receives NavPy's companion HEARTBEAT (this aircraft's system id, component AAS_NWD_COMP,
    MAV_TYPE_ONBOARD_CONTROLLER) and, in an armed GUIDED that NavPy was alive during:

      WATCH --no NavPy heartbeat for the timeout--> HOLD (LOITER)
      HOLD  --a NavPy heartbeat received after the hold began, mode still its LOITER--> AUTO, WATCH
      HOLD  --AAS_NWD_RTL s without one--> RTL (not undone; the operator decides)

    The timeout is AAS_NWD_TMO, or AAS_NWD_MISS heartbeat periods if that is longer. The period is the shortest
    recent spacing of NavPy's heartbeats in aircraft time, so neither one lost beat nor a sim speed-up (the
    companion paces its heartbeat in wall time) reads as a dead NavPy. Times are the autopilot's own clock;
    heartbeats are timed when the autopilot received them, not when this script got to them.

    It does nothing in any other mode or disarmed, and stands down when the mode leaves its LOITER or RTL, so
    pilot and failsafe modes win. Each mode write is guarded by a mode read just before it: bindings lock one call
    at a time, so this is as close as a script gets. The loop runs under pcall, and its state is sent once a
    second as NAMED_VALUE_FLOAT "NAVWD" (STATE_* below), so NavPy and the GCS can tell when it is not running.
--]]

local PARAM_TABLE_KEY = 88   -- the AAS_ table (aas_params.lua); this script owns indices 20-24
assert(param:add_table(PARAM_TABLE_KEY, "AAS_", 30), "nav_watchdog: failed to add AAS_ parameter table (key 88)")
assert(param:add_param(PARAM_TABLE_KEY, 20, "NWD_ENABLE", 1), "nav_watchdog: add_param NWD_ENABLE")
assert(param:add_param(PARAM_TABLE_KEY, 21, "NWD_TMO", 1.5), "nav_watchdog: add_param NWD_TMO")   -- s, NAV_HOLD_TRIGGER_S
assert(param:add_param(PARAM_TABLE_KEY, 22, "NWD_RTL", 30), "nav_watchdog: add_param NWD_RTL")    -- s, NAV_HOLD_RECOVERY_S
assert(param:add_param(PARAM_TABLE_KEY, 23, "NWD_COMP", 191), "nav_watchdog: add_param NWD_COMP") -- MAV_COMP_ID_ONBOARD_COMPUTER
-- Heartbeat periods without one before the hold: one beat lost whole, plus half a period of jitter.
assert(param:add_param(PARAM_TABLE_KEY, 24, "NWD_MISS", 2.5), "nav_watchdog: add_param NWD_MISS")

local TICK_MS = 100
local REPORT_MS = 1000
local RX_QUEUE = 10
local MAX_RX_PER_TICK = RX_QUEUE    -- a full queue per call, within SCR_VM_I_COUNT even if every frame needs a CRC
local MSG_HEARTBEAT = 0
local HEARTBEAT_CRC_EXTRA = 50
local HEARTBEAT_LEN = 9             -- its payload length; MAVLink 2 may trim trailing zeros, never lengthen it
local PERIOD_SAMPLES = 8            -- heartbeat spacings kept; the shortest is the period (a lost beat is longer)
local MAV_TYPE_ONBOARD_CONTROLLER = 18
local MODE_AUTO, MODE_RTL, MODE_LOITER, MODE_GUIDED = 10, 11, 12, 15   -- ArduPlane mode numbers
local STATE_OFF, STATE_WATCH, STATE_HOLD, STATE_RTL = -1, 0, 1, 2
local STANDING_DOWN = "NavPy watchdog: mode changed, standing down"

local state = STATE_WATCH
local last_hb_ms = nil              -- receive time (autopilot millis) of the latest NavPy heartbeat
local spacings, next_spacing = {}, 1
local navpy_guided = false          -- in an armed GUIDED, and NavPy was alive during it
local hold_since_ms = 0
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
    if len > HEARTBEAT_LEN then         -- before the CRC: a corrupt length must not run it past SCR_VM_I_COUNT
        return false
    end
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

local function heard(received_ms)
    if last_hb_ms ~= nil and received_ms <= last_hb_ms then
        return
    end
    if last_hb_ms ~= nil then
        spacings[next_spacing] = received_ms - last_hb_ms
        next_spacing = next_spacing % PERIOD_SAMPLES + 1
    end
    last_hb_ms = received_ms
end

local function receive_heartbeats()
    local sysid, comp = own_sysid(), param:get("AAS_NWD_COMP")
    for _ = 1, MAX_RX_PER_TICK do
        local msg, _, received = mavlink:receive_chan()
        if msg == nil then
            return
        end
        local _, src_sys, src_comp, msgid = string.unpack("<BBBI3", msg, 7)
        if msgid == MSG_HEARTBEAT and src_sys == sysid and src_comp == comp
            and string.unpack("<B", msg, 17) == MAV_TYPE_ONBOARD_CONTROLLER and crc_ok(msg) then
            heard(received:toint())
        end
    end
end

-- The timeout in ms, or nil until two heartbeats gave a period.
local function timeout_ms()
    local period = nil
    for _, spacing in pairs(spacings) do
        if period == nil or spacing < period then
            period = spacing
        end
    end
    if period == nil then
        return nil
    end
    return math.max(param:get("AAS_NWD_TMO") * 1000, param:get("AAS_NWD_MISS") * period)
end

-- nil if the mode is no longer ``from`` (someone else changed it), else whether ``to`` was accepted. The read is
-- the last binding call before the write.
local function switch(from, to)
    if vehicle:get_mode() ~= from then
        return nil
    end
    return vehicle:set_mode(to)
end

local function enter(new_state, severity, msg)
    state = new_state
    text(severity, msg)
end

local function stand_down(msg)
    navpy_guided = false
    enter(STATE_WATCH, 4, msg)
end

local function watch(now, mode, fresh)
    if mode ~= MODE_GUIDED or not arming:is_armed() then
        navpy_guided = false
    elseif fresh then
        navpy_guided = true
    elseif navpy_guided then
        local switched = switch(MODE_GUIDED, MODE_LOITER)
        if switched then
            hold_since_ms = now
            enter(STATE_HOLD, 2, "NavPy watchdog: no NavPy heartbeat, LOITER")
        elseif switched == nil then
            navpy_guided = false
        else
            text(3, "NavPy watchdog: LOITER refused, retrying")
        end
    end
end

local function hold(now, mode, fresh)
    if mode ~= MODE_LOITER then
        stand_down(STANDING_DOWN)
        return
    end
    if fresh and last_hb_ms > hold_since_ms then
        local switched = switch(MODE_LOITER, MODE_AUTO)
        if switched then
            stand_down("NavPy watchdog: NavPy back, AUTO")
            return
        elseif switched == nil then
            stand_down(STANDING_DOWN)
            return
        end
        text(3, "NavPy watchdog: AUTO refused, retrying")
    end
    if now - hold_since_ms >= param:get("AAS_NWD_RTL") * 1000 then
        local switched = switch(MODE_LOITER, MODE_RTL)
        if switched then
            enter(STATE_RTL, 2, "NavPy watchdog: no NavPy for the hold, RTL")
        elseif switched == nil then
            stand_down(STANDING_DOWN)
        else
            text(3, "NavPy watchdog: RTL refused, retrying")
        end
    end
end

local function step(now)
    receive_heartbeats()
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
    local timeout = timeout_ms()
    local fresh = timeout ~= nil and now - last_hb_ms < timeout
    if state == STATE_WATCH then
        watch(now, mode, fresh)
    elseif not arming:is_armed() then
        stand_down("NavPy watchdog: disarmed, standing down")
    elseif state == STATE_HOLD then
        hold(now, mode, fresh)
    elseif mode ~= MODE_RTL then
        stand_down(STANDING_DOWN)
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
