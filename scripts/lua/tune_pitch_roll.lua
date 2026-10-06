--[[
    Refined Lua Script for ArduPlane Roll and Pitch Autotuning
    Features:
    - Alternates PWM signals for roll and pitch channels based on tuning mode.
    - Adjustable toggle interval via RC channel with robust handling.
    - Pitch tuning continues until original altitude is regained.
    - Ensures pitch tuning only initiates if current position is valid and altitude is safe.
    - Maintains simplicity by avoiding override clearing.
    - Modular functions for clarity and maintainability.
--]]

-- Constants for tuning modes
local PARAM_TABLE_KEY = 99 -- Key for parameter table
assert(param:add_table(PARAM_TABLE_KEY, "MTUNE_", 7), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'BTMSK', 3), 'could not add param1')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'MIN_PTCH_ALT', 100), 'could not add param2')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'MIN_RLL_ALT', -10), 'could not add param3')
assert(param:add_param(PARAM_TABLE_KEY, 4, 'TRG_CH', 7), 'could not add param4')
assert(param:add_param(PARAM_TABLE_KEY, 5, 'TGL_INT_CH', 8), 'could not add param5')
assert(param:add_param(PARAM_TABLE_KEY, 6, 'TGL_INT_MIN', 500), 'could not add param6')
assert(param:add_param(PARAM_TABLE_KEY, 7, 'TGL_INT_MAX', 5000), 'could not add param7')

-- Safety parameters
local ALLOWED_MODE_BITMASK = 3    -- Bitmask for allowed tuning modes
local MIN_PITCH_ALT = 100         -- Minimum altitude for pitch tuning in meters
local MIN_ROLL_ALT = -10          -- Minimum altitude for roll tuning in meters
local TRIGGER_CHANNEL = 7         -- Trigger channel to select tuning mode
local TOGGLE_INTERVAL_CHANNEL = 8 -- RC Channel to adjust the toggle interval
local TOGGLE_INTERVAL_MIN = 500   -- Minimum toggle interval in milliseconds
local TOGGLE_INTERVAL_MAX = 5000  -- Maximum toggle interval in milliseconds
local toggle_interval_pwm_max = 1900
local toggle_interval_pwm_min = 1000

-- Constants for tuning modes
local FBWA_TUNE_MODE_NUMBER = 5 -- PLANE_MODE_FBWA
local AUTO_TUNE_MODE_NUMBER = 8 -- PLANE_MODE_AUTOTUNE
local AUTO_TUNE_MODE_MASK = 1
local FBWA_MODE_MASK = 2

-- Constants for tuning modes using enum-like table
local TUNING_MODE = {
    DISABLED = 0,
    ROLL = 1,
    PITCH = 2,
    [0] = "Disabled",
    [1] = "Roll",
    [2] = "Pitch",
}

-- RC Channels
local ROLL_CHANNEL = 1  -- Aileron
local PITCH_CHANNEL = 2 -- Elevator

-- PWM Thresholds for trigger channel
local TRIGGER_THRESHOLD_ROLL = 1400  -- Threshold for roll tuning
local TRIGGER_THRESHOLD_PITCH = 1700 -- Threshold for pitch tuning

-- Variables for PWM values
local roll_pwm_min = 0
local roll_pwm_max = 0
local pitch_pwm_min = 0
local pitch_pwm_max = 0

-- Tuning state variables
local MAX_ALTITUDE_LOSS = 20 -- Maximum allowed altitude loss in meters during pitch down

local last_toggle_time = 0
local is_rolling_right = true
local is_pitching_up = true
local original_altitude = nil

-- References to RC channels (initialized later)
local RC1
local RC2

-- Variables for tracking modes and intervals
local LOG_CATEGORY_MODE = 1
local LOG_CATEGORY_INTERVAL = 2
local LOG_CATEGORY_DATA = 3
local last_log = {}

-- Tuning control variables
local last_tune_mode = nil
local tuning_start_time = 0
local TUNE_START_DELAY = 500 -- milliseconds delay when switching modes

-- New variables to store the current toggle interval
local old_toggle_interval = 2000
local current_toggle_interval = 2000 -- Initial value

local roll_left_duration = -10
local roll_right_duration = -10

local function abs_diff(a, b)
    if a >= b then
        return a - b
    else
        return b - a
    end
end

local function log(level, category, message)
    if not last_log[category] or last_log[category] ~= message then
        gcs:send_text(level, message)
        last_log[category] = message
    end
end

-- Function to clear RC overrides
local function clear_rc_overrides()
    if RC1 then RC1:set_override(0) end
    if RC2 then RC2:set_override(0) end
end

local function refresh_params()
    ALLOWED_MODE_BITMASK = param:get("MTUNE_BTMSK")
    MIN_PITCH_ALT = param:get("MTUNE_MIN_PTCH_ALT")
    MIN_ROLL_ALT = param:get("MTUNE_MIN_RLL_ALT")
    TRIGGER_CHANNEL = param:get("MTUNE_TRG_CH")
    TOGGLE_INTERVAL_CHANNEL = param:get("MTUNE_TGL_INT_CH")
    TOGGLE_INTERVAL_MIN = param:get("MTUNE_TGL_INT_MIN")
    TOGGLE_INTERVAL_MAX = param:get("MTUNE_TGL_INT_MAX")

    toggle_interval_pwm_max = param:get(string.format("RC%d_MAX", TOGGLE_INTERVAL_CHANNEL))
    toggle_interval_pwm_min = param:get(string.format("RC%d_MIN", TOGGLE_INTERVAL_CHANNEL))
end

-- Function to retrieve the current tuning mode based on trigger channel PWM
local function get_tune_mode()
    local rc_input = rc:get_pwm(TRIGGER_CHANNEL) -- Get PWM value from trigger channel
    if rc_input then
        if rc_input > TRIGGER_THRESHOLD_PITCH then
            return TUNING_MODE.PITCH
        elseif rc_input > TRIGGER_THRESHOLD_ROLL then
            return TUNING_MODE.ROLL
        end
    end
    return TUNING_MODE.DISABLED
end

-- Function to retrieve the current altitude
local function get_current_altitude()
    if not ahrs:healthy() then
        log(5, LOG_CATEGORY_DATA, "ERROR: AHRS not healthy")
        return 0
    end

    local current_pos = ahrs:get_position()
    local home_pos = ahrs:get_home()
    if not current_pos or not home_pos then
        log(5, LOG_CATEGORY_DATA, "ERROR: Unable to retrieve current position or home position")
        return 0
    end
    local current_alt = (current_pos:alt() - home_pos:alt()) * 0.01
    return current_alt
end

-- Function to calculate and update the toggle interval based on RC input
local function calculate_toggle_interval()
    -- Retrieve toggle interval parameters
    local toggle_interval_pwm = rc:get_pwm(TOGGLE_INTERVAL_CHANNEL)

    -- Validate toggle interval parameters
    if not (toggle_interval_pwm_max and toggle_interval_pwm_min and toggle_interval_pwm) then
        log(5, LOG_CATEGORY_INTERVAL, "Error: Unable to retrieve toggle interval parameters")
        return
    end

    if toggle_interval_pwm_max == toggle_interval_pwm_min then
        log(5, LOG_CATEGORY_INTERVAL, "Error: Toggle interval PWM max and min are equal")
        return
    end

    -- Simple linear interpolation for toggle interval
    current_toggle_interval = TOGGLE_INTERVAL_MIN + (TOGGLE_INTERVAL_MAX - TOGGLE_INTERVAL_MIN) *
        (toggle_interval_pwm - toggle_interval_pwm_min) /
        (toggle_interval_pwm_max - toggle_interval_pwm_min)

    old_toggle_interval = current_toggle_interval
    -- Clamp current_toggle_interval to prevent out-of-bound values
    current_toggle_interval = math.max(TOGGLE_INTERVAL_MIN, math.min(current_toggle_interval, TOGGLE_INTERVAL_MAX))

    if abs_diff(old_toggle_interval, current_toggle_interval) >= 10 then
        param:set_and_save("RC_OVERRIDE_TIME", (current_toggle_interval + 50) / 1000)
    end
end

-- Function to handle roll tuning
local function roll_tuning(current_time)
    current_time = millis()
    if current_time < tuning_start_time then
        -- Waiting for initial delay
        clear_rc_overrides()
        return
    end

    local duration = current_time - last_toggle_time
    local should_switch = duration >= current_toggle_interval

    if is_rolling_right then
        -- Roll right (using max roll PWM)
        RC1:set_override(roll_pwm_max)
        if should_switch then
            --gcs:send_text(6, string.format("Rolling right to max PWM: %d", roll_pwm_max))
            log(5, LOG_CATEGORY_MODE,
                string.format("R: " .. tostring(roll_left_duration) .. ", R: " .. tostring(roll_right_duration)))
            is_rolling_right = false
            roll_right_duration = duration
            last_toggle_time = current_time
            calculate_toggle_interval()
        end
    else
        -- Roll left (using min roll PWM)
        RC1:set_override(roll_pwm_min)
        if should_switch then
            --gcs:send_text(6, string.format("Rolling left to min PWM: %d", roll_pwm_min))
            log(5, LOG_CATEGORY_MODE,
                string.format("L: " .. tostring(roll_left_duration) .. ", R: " .. tostring(roll_right_duration)))
            is_rolling_right = true
            roll_left_duration = duration
            last_toggle_time = current_time
            calculate_toggle_interval()
        end
    end
end

local function pitch_tuning(current_time)
    current_time = millis()
    if current_time < tuning_start_time then
        -- Waiting for initial delay
        clear_rc_overrides()
        return
    end

    -- Retrieve current altitude
    local current_alt = get_current_altitude()

    local duration = current_time - last_toggle_time
    local should_switch = duration >= current_toggle_interval

    if is_pitching_up then
        -- Pitch up (using max pitch PWM)
        RC2:set_override(pitch_pwm_max)
        if should_switch and current_alt > original_altitude then
            gcs:send_text(6, string.format("Pitching up to max PWM: %d", pitch_pwm_max))
            is_pitching_up = false
            last_toggle_time = current_time
            calculate_toggle_interval()
        end
    else
        -- Pitch down (using min pitch PWM)
        RC2:set_override(pitch_pwm_min)
        -- Switch back to pitching up if conditions are met
        if should_switch
            or current_alt < (original_altitude - MAX_ALTITUDE_LOSS)
            or current_alt < MIN_PITCH_ALT then
            gcs:send_text(6, string.format("Pitching down to min PWM: %d", pitch_pwm_min))
            is_pitching_up = true
            last_toggle_time = current_time
            calculate_toggle_interval()
        end
    end
end

-- Main update loop function
local function update()
    local current_time = millis()
    local current_flight_mode = vehicle:get_mode()
    local current_mode = get_tune_mode()

    -- Detect mode change and reset variables
    if current_mode ~= last_tune_mode then
        refresh_params()
        last_tune_mode = current_mode
        last_toggle_time = current_time
        is_rolling_right = true
        is_pitching_up = true
        original_altitude = nil
        tuning_start_time = current_time + TUNE_START_DELAY
        -- Clear any RC overrides
        clear_rc_overrides()
        -- Initialize current toggle interval
        calculate_toggle_interval()
        log(6, LOG_CATEGORY_MODE, "Tuning mode changed to " .. (TUNING_MODE[current_mode] or "Unknown"))
    end

    -- Verify if the aircraft is in AUTO_TUNE mode or FBWA
    local autotune_bitmask = ALLOWED_MODE_BITMASK & AUTO_TUNE_MODE_MASK ~= 0
    local autotune_active = current_flight_mode == AUTO_TUNE_MODE_NUMBER and autotune_bitmask
    local fbwa_bitmask = ALLOWED_MODE_BITMASK & FBWA_MODE_MASK ~= 0
    local fbwa_active = current_flight_mode == FBWA_TUNE_MODE_NUMBER and fbwa_bitmask

    if not (autotune_active or fbwa_active) then
        log(6, LOG_CATEGORY_MODE,
            "AUTOTUNE or FBWA mode not active: Current tuning is " .. (TUNING_MODE[current_mode] or "Disabled"))

        original_altitude = nil
        -- Clear overrides
        clear_rc_overrides()
        return update, 10
    end

    -- Set original altitude when entering tuning mode
    if not original_altitude then
        local current_altitude = get_current_altitude()

        original_altitude = current_altitude

        if current_mode == TUNING_MODE.PITCH then
            if current_altitude < MIN_PITCH_ALT then
                log(5, LOG_CATEGORY_MODE,
                    string.format("Altitude %.1f m is below minimum %.1f.", current_altitude, MIN_PITCH_ALT))
                return update, 10
            end

            log(6, LOG_CATEGORY_MODE,
                string.format("Pitch tuning mode enabled. Original altitude: %.1f m", original_altitude))
        elseif current_mode == TUNING_MODE.ROLL then
            if current_altitude < MIN_ROLL_ALT then
                log(5, LOG_CATEGORY_MODE,
                    string.format("Altitude %.1f m is below minimum %.1f.", current_altitude, MIN_ROLL_ALT))
                return update, 10
            end

            log(6, LOG_CATEGORY_MODE,
                string.format("Roll tuning mode enabled. Original altitude: %.1f m", original_altitude))
        end
    end

    -- If a valid tuning mode is active, perform tuning
    if current_mode == TUNING_MODE.ROLL then
        roll_tuning(current_time)
    elseif current_mode == TUNING_MODE.PITCH then
        pitch_tuning(current_time)
    else
        -- Clear overrides if tuning mode is disabled
        clear_rc_overrides()
    end

    return update, 10
end

-- Function to initialize the script
local function start()
    log(6, LOG_CATEGORY_MODE, "Starting pitch and roll tuning script")

    roll_pwm_max = param:get(string.format("RC%d_MAX", ROLL_CHANNEL))
    roll_pwm_min = param:get(string.format("RC%d_MIN", ROLL_CHANNEL))
    pitch_pwm_max = param:get(string.format("RC%d_MAX", PITCH_CHANNEL))
    pitch_pwm_min = param:get(string.format("RC%d_MIN", PITCH_CHANNEL))

    refresh_params()

    if not (roll_pwm_max and roll_pwm_min and pitch_pwm_max and pitch_pwm_min) then
        log(5, LOG_CATEGORY_DATA, "Error: Unable to retrieve roll or pitch PWM parameters")
        return start, 1000
    end

    RC1 = rc:get_channel(ROLL_CHANNEL)
    RC2 = rc:get_channel(PITCH_CHANNEL)

    if not (RC1 and RC2) then
        log(5, LOG_CATEGORY_DATA, "Error: Unable to access Roll or Pitch RC channels")
        return start, 1000
    end

    gcs:send_text(6, string.format("Roll PWM: Max = %d, Min = %d", roll_pwm_max, roll_pwm_min))
    gcs:send_text(6, string.format("Pitch PWM: Max = %d, Min = %d", pitch_pwm_max, pitch_pwm_min))
    gcs:send_text(6, "Initialization complete. Starting autotuning...")

    return update()
end

-- Start the script
return start()
