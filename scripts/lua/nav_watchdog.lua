--[[
    ArduPilot Lua - NavPy watchdog for ArduPlane (docs/design/navpy_recovery.md, step 7).

    NavPy's in-process recovery holds the UAV while NavPy runs. This script covers a NavPy that is dead or whose
    link is gone: it receives NavPy's companion HEARTBEAT (this aircraft's system id, component AAS_NWD_COMP,
    MAV_TYPE_ONBOARD_CONTROLLER) and, in an armed GUIDED that NavPy was alive during:

      WATCH --no NavPy heartbeat for AAS_NWD_TMO s--> HOLD (LOITER)
      HOLD  --a NavPy heartbeat received after the hold began, mode still its LOITER--> AUTO, WATCH
      HOLD  --AAS_NWD_RTL s without one--> RTL (not undone; the operator decides)

    NavPy's own recovery LOITER is guarded too (step 10, item 9, owner ruling 2026-11-09; Codex round 2): from its
    LOITER request on, NavPy's heartbeat claims the hold (custom_mode: bit 0 HOLD_CLAIM, bits 1-7 the request's
    token, bits 8-23 the incident's hold budget left in tenths of a second; companion_hold_claim.py). A claim is
    taken only at a switch into an armed LOITER, received within CLAIM_WINDOW_MS of it, and each token once:

      WATCH --switch into LOITER with a claim--> GUARD (no mode write: the mode is LOITER already)
      GUARD --claim's receipt + min(budget, AAS_NWD_RTL) s, whatever the heartbeat does--> RTL
      GUARD --any mode change or a disarm--> WATCH (the token stays spent)

    The deadline is this script's: NavPy dead, restarted (not knowing it held) or alive, its hold ends by NavPy
    changing the mode (AUTO, RTL) or by RTL here, at the summed-hold bound NavPy itself keeps. A pilot's LOITER is
    never taken: NavPy claims only a LOITER it requested or owns, a claim received later than the window around a
    switch is ignored, and one already taken never counts again, so a pilot who leaves NavPy's LOITER and comes
    back to LOITER while NavPy still claims it is not guarded.

    Times are the autopilot's own clock (aircraft time); heartbeats are timed when the autopilot received them,
    not when this script got to them. NavPy spaces its heartbeat evenly in aircraft time at any sim speed-up
    (heartbeat_pacing.py), so a fixed timeout holds there too.

    It does nothing in any other mode (but NavPy's claimed LOITER) or disarmed, and stands down when the mode
    leaves its LOITER or RTL, so pilot and failsafe modes win. Each mode write is guarded by a mode read just
    before it: bindings lock one call at a time, so this is as close as a script gets. The loop runs under pcall,
    and its state is sent once a second as NAMED_VALUE_FLOAT "NAVWD" (STATE_* below), so NavPy and the GCS can
    tell when it is not running.
--]]

-- A table of its own, as every script here: ArduPilot's add_table on a key another script already added re-blanks
-- every entry of that table (AP_Param.cpp, "fill in footer for all entries"), so sharing aas_params.lua's key 88
-- wiped one script's params. The prefix keeps the names AAS_NWD_*; AP_Param::find looks on past the AAS_ table.
local PARAM_TABLE_KEY = 89
assert(param:add_table(PARAM_TABLE_KEY, "AAS_NWD_", 8), "nav_watchdog: failed to add AAS_NWD_ parameter table (key 89)")
assert(param:add_param(PARAM_TABLE_KEY, 1, "ENABLE", 1), "nav_watchdog: add_param ENABLE")
assert(param:add_param(PARAM_TABLE_KEY, 2, "TMO", 1.5), "nav_watchdog: add_param TMO")    -- s, NAV_HOLD_TRIGGER_S
assert(param:add_param(PARAM_TABLE_KEY, 3, "RTL", 30), "nav_watchdog: add_param RTL")     -- s, NAV_HOLD_RECOVERY_S
assert(param:add_param(PARAM_TABLE_KEY, 4, "COMP", 191), "nav_watchdog: add_param COMP")  -- MAV_COMP_ID_ONBOARD_COMPUTER

local TICK_MS = 100
local REPORT_MS = 1000
local RX_QUEUE = 10
local MAX_RX_PER_TICK = RX_QUEUE    -- a full queue per call, within SCR_VM_I_COUNT even if every frame needs a CRC
local MSG_HEARTBEAT = 0
local HEARTBEAT_CRC_EXTRA = 50
local HEARTBEAT_LEN = 9             -- its payload length; MAVLink 2 may trim trailing zeros, never lengthen it
local MAV_TYPE_ONBOARD_CONTROLLER = 18
local HOLD_CLAIM = 1                -- NavPy's heartbeat custom_mode bit 0: its recovery holds in its own LOITER
-- A claim counts only if received this close to the switch into LOITER, either side: NavPy claims from its request
-- on and its 4 Hz heartbeat repeats the claim, so two of its periods hold one heartbeat lost or sent late.
local CLAIM_WINDOW_MS = 500
local MODE_AUTO, MODE_RTL, MODE_LOITER, MODE_GUIDED = 10, 11, 12, 15   -- ArduPlane mode numbers
local STATE_OFF, STATE_WATCH, STATE_HOLD, STATE_RTL, STATE_GUARD = -1, 0, 1, 2, 3
local STANDING_DOWN = "NavPy watchdog: mode changed, standing down"

local state = STATE_WATCH
local last_hb_ms = nil              -- receive time (autopilot millis) of the latest NavPy heartbeat
local navpy_guided = false          -- in an armed GUIDED, and NavPy was alive during it
local claim = nil                   -- the latest claimed NavPy heartbeat: { ms, token, budget_ms }
local spent_token = nil             -- the token taken last: a claim is taken once
local was_in_loiter = false         -- armed in LOITER at the previous tick
local loiter_since_ms = nil         -- when this armed LOITER began (nil: not in one)
local guard_deadline_ms = nil
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

local function heard(received_ms, custom_mode)
    if last_hb_ms == nil or received_ms > last_hb_ms then
        last_hb_ms = received_ms
    end
    if custom_mode & HOLD_CLAIM ~= 0 and (claim == nil or received_ms >= claim.ms) then
        claim = { ms = received_ms, token = (custom_mode >> 1) & 0x7F, budget_ms = ((custom_mode >> 8) & 0xFFFF) * 100 }
    end
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
            heard(received:toint(), (string.unpack("<I4", msg, 13)))
        end
    end
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
    guard_deadline_ms = nil
    enter(STATE_WATCH, 4, msg)
end

-- In WATCH, armed in LOITER: take NavPy's claim if it was received within CLAIM_WINDOW_MS of the switch into this
-- LOITER (before it: sent with NavPy's request; after it: a heartbeat later) and its token was not taken before.
local function take_claim()
    if loiter_since_ms == nil or claim == nil or claim.token == spent_token
        or math.abs(claim.ms - loiter_since_ms) > CLAIM_WINDOW_MS then
        return
    end
    spent_token = claim.token
    guard_deadline_ms = claim.ms + math.min(claim.budget_ms, param:get("AAS_NWD_RTL") * 1000)
    enter(STATE_GUARD, 6, "NavPy watchdog: guarding NavPy's LOITER")
end

local function guard(now, armed, in_loiter)
    if not in_loiter then
        stand_down(armed and STANDING_DOWN or "NavPy watchdog: disarmed, standing down")
    elseif now >= guard_deadline_ms then
        local switched = switch(MODE_LOITER, MODE_RTL)
        if switched then
            enter(STATE_RTL, 2, "NavPy watchdog: NavPy's hold over, RTL")
        elseif switched == nil then
            stand_down(STANDING_DOWN)
        else
            text(3, "NavPy watchdog: RTL refused, retrying")
        end
    end
end

local function watch(now, mode, armed, fresh)
    if mode ~= MODE_GUIDED or not armed then
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
            guard_deadline_ms = nil
            enter(STATE_OFF, 4, "NavPy watchdog disabled")
        end
        return
    end
    if state == STATE_OFF then
        enter(STATE_WATCH, 6, "NavPy watchdog enabled")
    end
    local mode = vehicle:get_mode()
    local armed = arming:is_armed()
    local in_loiter = armed and mode == MODE_LOITER
    if not in_loiter then
        loiter_since_ms = nil
    elseif not was_in_loiter then
        loiter_since_ms = now
    end
    was_in_loiter = in_loiter
    local fresh = last_hb_ms ~= nil and now - last_hb_ms < param:get("AAS_NWD_TMO") * 1000
    if state == STATE_WATCH then
        watch(now, mode, armed, fresh)
        if state == STATE_WATCH and in_loiter then
            take_claim()
        end
    elseif state == STATE_GUARD then
        guard(now, armed, in_loiter)
    elseif not armed then
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
