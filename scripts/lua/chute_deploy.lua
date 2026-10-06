--[[
    ArduPilot Lua - Simple parachute deployment for ArduPlane.

    Trigger flow:
      SAFE -> PITCHUP -> DEPLOYED

    A valid trigger first tries to climb by switching to FBWA, pitching up, and
    commanding full throttle. Once pitch reaches CHUTE_PITCH_DEG, the script
    cuts throttle, keeps pitch-up commanded, restarts the setup timer, and
    deploys after a short post-cut hold. Low airspeed can deploy earlier, but
    it is not required.
--]]

local PARAM_TABLE_KEY = 90
local PARAM_PREFIX    = "CHUTE_"

assert(param:add_table(PARAM_TABLE_KEY, PARAM_PREFIX, 30),
       "chute_deploy: failed to add CHUTE_ parameter table (key 90)")
assert(param:add_param(PARAM_TABLE_KEY, 1, "RC_CHAN",   7),    "chute_deploy: add_param RC_CHAN")
assert(param:add_param(PARAM_TABLE_KEY, 2, "SCR_FN",    1),    "chute_deploy: add_param SCR_FN")
assert(param:add_param(PARAM_TABLE_KEY, 3, "MIN_ALT",   80),   "chute_deploy: add_param MIN_ALT")
assert(param:add_param(PARAM_TABLE_KEY, 4, "DESC_RATE", 5),    "chute_deploy: add_param DESC_RATE")
assert(param:add_param(PARAM_TABLE_KEY, 5, "POST_MODE", 5),    "chute_deploy: add_param POST_MODE")
assert(param:add_param(PARAM_TABLE_KEY, 6, "SETUP_MS",  2000), "chute_deploy: add_param SETUP_MS")
assert(param:add_param(PARAM_TABLE_KEY, 7, "MAX_AS",    12),   "chute_deploy: add_param MAX_AS")
assert(param:add_param(PARAM_TABLE_KEY, 8, "CLIMB_MS",  2000), "chute_deploy: add_param CLIMB_MS")
assert(param:add_param(PARAM_TABLE_KEY, 9, "PITCH_DEG", 20),   "chute_deploy: add_param PITCH_DEG")

-- ---------- Constants ----------
local TICK_MS                    = 50
local RC_LOW_PWM                 = 1300
local RC_HIGH_PWM                = 1700
local PWM_FIRE                   = 2000
local PWM_SAFE                   = 1000
local OUTPUT_WATCHDOG_MS         = 200
local ABORT_FLOOR_M              = 30
local THROTTLE_FN                = 70   -- SERVOx_FUNCTION == k_throttle
local SCRIPTING_FN_BASE          = 93   -- k_scripting1 = 94, ..., k_scripting16 = 109
local FBWA_MODE                  = 5    -- ArduPlane mode number
local RECOVERY_HOLD_MS           = 2000
local RECOVERY_THROTTLE_EPSILON  = 50
local RECOVERY_LOG_INTERVAL_MS   = 5000

-- ---------- Log dedup ----------
local last_log_by_cat = {}

local function log(level, category, msg)
    if last_log_by_cat[category] ~= msg then
        gcs:send_text(level, msg)
        last_log_by_cat[category] = msg
    end
end

local function log_once(level, category, msg)
    gcs:send_text(level, msg)
    last_log_by_cat[category] = msg
end

-- ---------- Live state (refreshed in SAFE) ----------
local p_rc_chan, p_scr_fn, p_min_alt, p_desc_rate, p_post_mode, p_setup_ms
local p_max_airspeed, p_climb_ms, p_pitch_deg
local live_chute_chan_0idx, live_throttle_chan_0idx, live_throttle_min_pwm
local inhibit_reason

-- ---------- Cross-phase state ----------
local edge_armed = false
local prev_high  = false
local last_missing_alt_ms = 0
local last_inhibit_log_ms = 0
local last_channel_map_logged
local last_recovery_valid_rc_log_ms = 0

-- ---------- Per-sequence state ----------
local state = "SAFE"
local seq_rc_chan, seq_post_mode, seq_setup_ms, seq_max_airspeed, seq_climb_ms, seq_pitch_deg
local seq_chute_chan_0idx, seq_throttle_chan_0idx, seq_throttle_min_pwm
local seq_pitch_rc, seq_throttle_rc
local seq_pitch_max_pwm, seq_throttle_max_pwm
local setup_phase
local setup_start_ms
local deploy_reason, deploy_logged
local recovery_hold_start_ms

local function clear_seq_snapshot()
    seq_rc_chan, seq_post_mode, seq_setup_ms, seq_max_airspeed, seq_climb_ms, seq_pitch_deg = nil, nil, nil, nil, nil, nil
    seq_chute_chan_0idx, seq_throttle_chan_0idx, seq_throttle_min_pwm = nil, nil, nil
    seq_pitch_rc, seq_throttle_rc = nil, nil
    seq_pitch_max_pwm, seq_throttle_max_pwm = nil, nil
    setup_phase = nil
    setup_start_ms = nil
    deploy_reason, deploy_logged = nil, nil
    recovery_hold_start_ms = nil
end

clear_seq_snapshot()

-- ---------- Parameters ----------
local function refresh_params()
    p_rc_chan      = param:get(PARAM_PREFIX .. "RC_CHAN")
    p_scr_fn       = param:get(PARAM_PREFIX .. "SCR_FN")
    p_min_alt      = param:get(PARAM_PREFIX .. "MIN_ALT")
    p_desc_rate    = param:get(PARAM_PREFIX .. "DESC_RATE") -- retained for parameter compatibility
    p_post_mode    = param:get(PARAM_PREFIX .. "POST_MODE")
    p_setup_ms     = param:get(PARAM_PREFIX .. "SETUP_MS")
    p_max_airspeed = param:get(PARAM_PREFIX .. "MAX_AS")
    p_climb_ms     = param:get(PARAM_PREFIX .. "CLIMB_MS")
    p_pitch_deg    = param:get(PARAM_PREFIX .. "PITCH_DEG")

    if not (p_rc_chan and p_scr_fn and p_min_alt and p_desc_rate and p_post_mode
            and p_setup_ms and p_max_airspeed and p_climb_ms and p_pitch_deg) then
        inhibit_reason = "CHUTE_ params not yet readable"
        return
    end

    if p_rc_chan < 1 or p_rc_chan > 16 or math.floor(p_rc_chan) ~= p_rc_chan then
        inhibit_reason = "CHUTE_RC_CHAN must be integer 1..16"; return
    end
    if p_scr_fn < 1 or p_scr_fn > 16 or math.floor(p_scr_fn) ~= p_scr_fn then
        inhibit_reason = "CHUTE_SCR_FN must be integer 1..16"; return
    end
    if p_min_alt < ABORT_FLOOR_M then
        inhibit_reason = string.format("CHUTE_MIN_ALT must be >= %d", ABORT_FLOOR_M); return
    end
    if p_desc_rate <= 0 then
        inhibit_reason = "CHUTE_DESC_RATE must be > 0"; return
    end
    if p_setup_ms <= 0 then
        inhibit_reason = "CHUTE_SETUP_MS must be > 0"; return
    end
    if p_max_airspeed <= 0 then
        inhibit_reason = "CHUTE_MAX_AS must be > 0"; return
    end
    if p_climb_ms <= 0 then
        inhibit_reason = "CHUTE_CLIMB_MS must be > 0"; return
    end
    if p_pitch_deg <= 0 then
        inhibit_reason = "CHUTE_PITCH_DEG must be > 0"; return
    end

    inhibit_reason = nil
end

-- ---------- Channel discovery ----------
local function find_chan_for_fn(fn_num)
    for i = 1, 16 do
        local v = param:get(string.format("SERVO%d_FUNCTION", i))
        if v and math.floor(v + 0.5) == fn_num then
            return i - 1, i
        end
    end
    return nil, nil
end

local function refresh_channel_cache()
    live_chute_chan_0idx    = nil
    live_throttle_chan_0idx = nil
    live_throttle_min_pwm   = nil

    if inhibit_reason ~= nil then return end

    local chute_fn_num = SCRIPTING_FN_BASE + p_scr_fn
    local chute_0, _ = find_chan_for_fn(chute_fn_num)
    local throttle_0, throttle_1 = find_chan_for_fn(THROTTLE_FN)

    if chute_0 == nil then
        inhibit_reason = string.format("No SERVOx_FUNCTION=%d (CHUTE_SCR_FN=%d)",
                                       chute_fn_num, p_scr_fn)
        return
    end
    if throttle_0 == nil then
        inhibit_reason = "No SERVOx_FUNCTION=70 (throttle)"
        return
    end

    local thr_min = param:get(string.format("SERVO%d_MIN", throttle_1))
    if thr_min == nil then
        inhibit_reason = string.format("SERVO%d_MIN missing (throttle cut needs it)", throttle_1)
        return
    end

    live_chute_chan_0idx    = chute_0
    live_throttle_chan_0idx = throttle_0
    live_throttle_min_pwm   = thr_min

    local map_sig = string.format("chute=SERVO%d throttle=SERVO%d thr_min=%d",
                                  chute_0 + 1, throttle_1, math.floor(thr_min + 0.5))
    if last_channel_map_logged ~= map_sig then
        log_once(6, "chan_map", "Chute: channel map " .. map_sig)
        last_channel_map_logged = map_sig
    end
end

-- ---------- Vehicle data ----------
local function get_agl_m()
    local hagl = ahrs:get_hagl()
    if hagl ~= nil then return hagl, "hagl" end
    local pos  = ahrs:get_position()
    local home = ahrs:get_home()
    if pos == nil or home == nil then return nil, nil end
    return (pos:alt() - home:alt()) * 0.01, "home_relative"
end

local function get_airspeed_ms()
    return ahrs:airspeed_estimate()
end

local function get_pitch_deg()
    local pitch = ahrs:get_pitch()
    if pitch == nil then return nil end
    return math.deg(pitch)
end

-- ---------- Servo / RC helpers ----------
local function drive_servo_pwm(chan_0idx, pwm)
    if chan_0idx == nil then return end
    SRV_Channels:set_output_pwm_chan_timeout(chan_0idx, pwm, OUTPUT_WATCHDOG_MS)
end

local function clear_setup_overrides()
    if seq_pitch_rc ~= nil then seq_pitch_rc:set_override(0) end
    if seq_throttle_rc ~= nil then seq_throttle_rc:set_override(0) end
end

local function valid_rc_chan(chan)
    return type(chan) == "number"
        and chan >= 1 and chan <= 16
        and math.floor(chan) == chan
end

local function rc_switch_tick_using(chan)
    if not rc:has_valid_input() then
        return "rc_lost"
    end
    local pwm = rc:get_pwm(chan)
    if not pwm then
        return "rc_lost"
    end
    local cur_high = pwm > RC_HIGH_PWM
    if not cur_high then
        edge_armed = true
    end
    local was_high = prev_high
    prev_high = cur_high
    if edge_armed and cur_high and not was_high then return "rising"  end
    if was_high and not cur_high                    then return "falling" end
    return false, "unknown setup phase"
end

local function get_rc_pwm_param(chan, suffix)
    if not valid_rc_chan(chan) then return nil end
    return param:get(string.format("RC%d_%s", chan, suffix))
end

local function setup_control_snapshot()
    local pitch_chan = param:get("RCMAP_PITCH")
    if not valid_rc_chan(pitch_chan) then return false, "RCMAP_PITCH missing" end
    local throttle_chan = param:get("RCMAP_THROTTLE")
    if not valid_rc_chan(throttle_chan) then return false, "RCMAP_THROTTLE missing" end

    local pitch_max = get_rc_pwm_param(pitch_chan, "MAX")
    if pitch_max == nil then return false, string.format("RC%d_MAX missing", pitch_chan) end
    local throttle_max = get_rc_pwm_param(throttle_chan, "MAX")
    if throttle_max == nil then return false, string.format("RC%d_MAX missing", throttle_chan) end

    local pitch_rc = rc:get_channel(pitch_chan)
    if pitch_rc == nil then return false, string.format("RC%d channel unavailable", pitch_chan) end
    local throttle_rc = rc:get_channel(throttle_chan)
    if throttle_rc == nil then return false, string.format("RC%d channel unavailable", throttle_chan) end

    seq_pitch_rc = pitch_rc
    seq_throttle_rc = throttle_rc
    seq_pitch_max_pwm = pitch_max
    seq_throttle_max_pwm = throttle_max
    return true, nil
end

local function apply_pitchup_overrides()
    if not rc:has_valid_input() then
        return false, "RC lost during setup"
    end
    if seq_pitch_rc == nil or seq_throttle_rc == nil
            or seq_pitch_max_pwm == nil or seq_throttle_max_pwm == nil then
        return false, "setup controls not initialized"
    end
    seq_pitch_rc:set_override(seq_pitch_max_pwm)
    seq_throttle_rc:set_override(seq_throttle_max_pwm)
    return true, nil
end

local function apply_slowdown_overrides()
    if not rc:has_valid_input() then
        return false, "RC lost during setup"
    end
    if seq_pitch_rc == nil or seq_pitch_max_pwm == nil then
        return false, "setup controls not initialized"
    end
    seq_pitch_rc:set_override(seq_pitch_max_pwm)
    if seq_throttle_rc ~= nil then seq_throttle_rc:set_override(0) end
    drive_servo_pwm(seq_throttle_chan_0idx, seq_throttle_min_pwm)
    return true, nil
end

-- ---------- Transitions ----------
local function release_to_safe()
    drive_servo_pwm(seq_chute_chan_0idx or live_chute_chan_0idx, PWM_SAFE)
    clear_setup_overrides()
    edge_armed = false
    prev_high  = false
    clear_seq_snapshot()
    state = "SAFE"
end

local function fire_chute(reason)
    clear_setup_overrides()
    deploy_reason = reason
    deploy_logged = false
    recovery_hold_start_ms = nil
    state = "DEPLOYED"
    drive_servo_pwm(seq_throttle_chan_0idx, seq_throttle_min_pwm)
    drive_servo_pwm(seq_chute_chan_0idx, PWM_FIRE)
end

local function enter_slowdown()
    setup_phase = "SLOWDOWN"
    setup_start_ms = millis():toint()
    if seq_pitch_rc ~= nil and seq_pitch_max_pwm ~= nil then seq_pitch_rc:set_override(seq_pitch_max_pwm) end
    if seq_throttle_rc ~= nil then seq_throttle_rc:set_override(0) end
    drive_servo_pwm(seq_throttle_chan_0idx, seq_throttle_min_pwm)
    log_once(5, "slowdown", "Chute: SLOWDOWN (pitch hold, throttle cut)")
end

local function pitchup_finish_reason()
    if setup_phase == "CLIMB" then
        local now = millis():toint()
        local elapsed = now - (setup_start_ms or now)
        local pitch_deg = get_pitch_deg()

        if pitch_deg == nil then
            return "setup_data_missing"
        end

        if pitch_deg >= seq_pitch_deg then
            enter_slowdown()
            return nil
        end

        if elapsed < seq_climb_ms then
            return nil
        end

        return "pitch_timeout"
    end

    if setup_phase == "SLOWDOWN" then
        local airspeed = get_airspeed_ms()
        if airspeed == nil then return "setup_data_missing" end
        if airspeed <= seq_max_airspeed then return "airspeed" end

        local now = millis():toint()
        if now - (setup_start_ms or now) >= seq_setup_ms then
            return "post_cut_hold"
        end
        return nil
    end

    return "setup_data_missing"
end

local function apply_current_setup_outputs()
    if setup_phase == "SLOWDOWN" then
        return apply_slowdown_overrides()
    end
    if setup_phase == "CLIMB" then
        return apply_pitchup_overrides()
    end
    return nil
end

local function enter_pitchup()
    seq_rc_chan            = p_rc_chan
    seq_post_mode          = p_post_mode
    seq_setup_ms           = p_setup_ms
    seq_max_airspeed       = p_max_airspeed
    seq_climb_ms           = p_climb_ms
    seq_pitch_deg          = p_pitch_deg
    seq_chute_chan_0idx    = live_chute_chan_0idx
    seq_throttle_chan_0idx = live_throttle_chan_0idx
    seq_throttle_min_pwm   = live_throttle_min_pwm
    setup_phase            = "CLIMB"
    setup_start_ms         = millis():toint()
    deploy_reason          = nil
    deploy_logged          = nil
    recovery_hold_start_ms = nil

    if not vehicle:set_mode(FBWA_MODE) then
        gcs:send_text(3, "Chute: cannot enter FBWA setup - deploying")
        fire_chute("setup_fbwa_failed")
        return
    end

    local ok, reason = setup_control_snapshot()
    if not ok then
        gcs:send_text(4, "Chute: setup unavailable (" .. reason .. ") - deploying")
        fire_chute("setup_unavailable: " .. reason)
        return
    end

    state = "PITCHUP"
    log_once(5, "pitchup", "Chute: PITCHUP (full throttle)")
    drive_servo_pwm(seq_chute_chan_0idx, PWM_SAFE)

    ok, reason = apply_current_setup_outputs()
    if not ok then
        gcs:send_text(4, "Chute: setup unavailable (" .. reason .. ") - deploying")
        fire_chute("setup_unavailable: " .. reason)
        return
    end

    local finish = pitchup_finish_reason()
    if finish ~= nil then
        fire_chute(finish)
    end
end

-- ---------- SAFE ----------
local function tick_safe()
    refresh_params()
    refresh_channel_cache()

    drive_servo_pwm(live_chute_chan_0idx, PWM_SAFE)

    if inhibit_reason ~= nil then
        local now = millis():toint()
        if now - last_inhibit_log_ms > 5000 then
            gcs:send_text(4, "Chute inhibited: " .. inhibit_reason)
            last_inhibit_log_ms = now
        end
        if valid_rc_chan(p_rc_chan) then
            local ev = rc_switch_tick_using(p_rc_chan)
            if ev == "rc_lost" then
                edge_armed = false
                prev_high  = false
            end
        else
            edge_armed = false
            prev_high  = false
        end
        return
    end

    local entry_agl, _ = get_agl_m()
    local ev = rc_switch_tick_using(p_rc_chan)

    if ev == "rc_lost" then
        edge_armed = false
        prev_high  = false
        return
    end
    if ev ~= "rising" then return end

    if not arming:is_armed() then
        gcs:send_text(4, "Chute: trigger while disarmed - ignored")
        return
    end
    if entry_agl == nil then
        local now = millis():toint()
        if now - last_missing_alt_ms > 2000 then
            gcs:send_text(4, "Chute: trigger with AGL unavailable - ignored")
            last_missing_alt_ms = now
        end
        return
    end
    if entry_agl < p_min_alt then
        gcs:send_text(4, string.format("Chute inhibited: altitude %.0f < CHUTE_MIN_ALT %d",
                                       entry_agl, math.floor(p_min_alt + 0.5)))
        return
    end

    enter_pitchup()
end

-- ---------- PITCHUP ----------
local function tick_pitchup()
    drive_servo_pwm(seq_chute_chan_0idx, PWM_SAFE)

    local ok, reason = apply_current_setup_outputs()
    if not ok then
        gcs:send_text(4, "Chute: setup unavailable (" .. reason .. ") - deploying")
        fire_chute("setup_unavailable: " .. reason)
        return
    end

    local finish = pitchup_finish_reason()
    if finish ~= nil then
        fire_chute(finish)
    end
end

-- ---------- DEPLOYED / recovery ----------
local function recovery_gates_true()
    if not rc:has_valid_input() then
        local now = millis():toint()
        if now - last_recovery_valid_rc_log_ms > RECOVERY_LOG_INTERVAL_MS then
            gcs:send_text(4, "Chute: recovery requires valid RC")
            last_recovery_valid_rc_log_ms = now
        end
        return false
    end

    local chute_pwm = rc:get_pwm(seq_rc_chan)
    if chute_pwm == nil or chute_pwm > RC_LOW_PWM then
        return false
    end

    local throttle_chan = param:get("RCMAP_THROTTLE")
    if not valid_rc_chan(throttle_chan) then
        log(4, "recovery_param", "Chute: recovery blocked - RCMAP_THROTTLE missing")
        return false
    end
    local throttle_min = get_rc_pwm_param(throttle_chan, "MIN")
    if throttle_min == nil then
        log(4, "recovery_param", string.format("Chute: recovery blocked - RC%d_MIN missing", throttle_chan))
        return false
    end
    local throttle_pwm = rc:get_pwm(throttle_chan)
    if throttle_pwm == nil then
        return false
    end

    return throttle_pwm <= throttle_min + RECOVERY_THROTTLE_EPSILON
end

local function maybe_release_after_deploy()
    local now = millis():toint()

    if not recovery_gates_true() then
        recovery_hold_start_ms = nil
        return false
    end

    if recovery_hold_start_ms == nil then
        recovery_hold_start_ms = now
        return false
    end
    if now - recovery_hold_start_ms < RECOVERY_HOLD_MS then
        return false
    end

    if not vehicle:set_mode(FBWA_MODE) then
        log(4, "recovery_mode", "Chute: set_mode(FBWA) failed; retrying")
        return false
    end

    release_to_safe()
    gcs:send_text(3, "Chute: post-deploy lock released by operator -> FBWA")
    return true
end

local function tick_deployed()
    if maybe_release_after_deploy() then
        return
    end

    if not deploy_logged then
        if seq_post_mode ~= nil then
            if not vehicle:set_mode(seq_post_mode) then
                gcs:send_text(4, "Chute: set_mode(post_mode) failed")
            end
        end
        gcs:send_text(3, "Chute deployed (" .. (deploy_reason or "?") .. ")")
        deploy_logged = true
    end

    drive_servo_pwm(seq_chute_chan_0idx, PWM_FIRE)
    drive_servo_pwm(seq_throttle_chan_0idx, seq_throttle_min_pwm)
end

-- ---------- Dispatch ----------
local function update()
    if     state == "SAFE"     then tick_safe()
    elseif state == "PITCHUP"  then tick_pitchup()
    elseif state == "DEPLOYED" then tick_deployed()
    else
        gcs:send_text(3, "Chute: unknown state " .. tostring(state) .. " - resetting to SAFE")
        release_to_safe()
    end
    return update, TICK_MS
end

local function start()
    gcs:send_text(6, "Chute deploy script loaded")
    edge_armed = false
    prev_high  = false
    return update, TICK_MS
end

return start()
