-- Mocks ArduPilot Lua scripting globals so chute_deploy.lua can run under
-- a plain Lua 5.3 interpreter for tests. Step 2a covers SAFE-state mocks
-- only; AHRS/Location/Vector mocks are added in 2b.

local M = {}

local function default_state()
    return {
        params = {
            CHUTE_RC_CHAN   = 7,
            CHUTE_SCR_FN    = 1,
            CHUTE_MIN_ALT   = 80,
            CHUTE_DESC_RATE = 5,
            CHUTE_POST_MODE = 0,
            CHUTE_SETUP_MS  = 2000,
            CHUTE_MAX_AS    = 12,
            CHUTE_CLIMB_MS  = 2000,
            CHUTE_PITCH_DEG = 20,
            RCMAP_PITCH     = 2,
            RC3_MIN         = 1100,
            RC2_MAX         = 1900,
            RC3_MAX         = 1900,
            RCMAP_THROTTLE  = 3,
            AIRSPEED_MIN    = 8,
            ARSPD_FBW_MIN   = 8,
            WP_LOITER_RAD   = 60,
        },
        rc_pwms        = { [7] = 1100, [2] = 1500, [3] = 1100 },
        rc_overrides   = {},
        servo_funcs    = {},
        servo_mins     = {},
        servo_pwm      = {},
        time_ms        = 0,
        valid_rc       = true,
        armed          = false,
        rpm            = nil,
        mode           = 0,
        mode_change_ok = true,
        mode_fail_for  = {},   -- per-mode override: { [5] = true } makes set_mode(5) return false

        -- AHRS / position / attitude state. Values are deliberately
        -- in the "happy path" range so quality gates clear cleanly when
        -- the test exercises the all-gates deploy.
        hagl           = 100,
        airspeed       = 10,
        velocity_d     = -2,        -- m/s, positive = down (sink rate)
        roll           = 0,         -- rad
        pitch          = 0,         -- rad
        wind_n         = 0,
        wind_e         = 0,
        position       = { alt_cm = 12500, lat = 403117414, lng = 444552111 },
        home           = { alt_cm = 12500, lat = 403117414, lng = 444552111 },
        distance_m     = 0,         -- mock cur:get_distance(release_loc) return value
    }
end

-- Vector mock for ahrs:wind_estimate / get_velocity_NED.
local function mock_vector(x, y, z)
    return {
        _x = x, _y = y, _z = z,
        x = function(self) return self._x end,
        y = function(self) return self._y end,
        z = function(self) return self._z end,
    }
end

-- Location mock. Independent copies on :copy(); offset_bearing is a
-- no-op (distance is controlled by state.distance_m). alt() returns cm
-- like the real ArduPilot Lua binding.
local function mock_location(alt_cm, lat, lng)
    local loc
    loc = {
        _alt_cm = alt_cm or 0,
        _lat    = lat or 0,
        _lng    = lng or 0,
        alt = function(self) return self._alt_cm end,
        copy = function(self)
            return mock_location(self._alt_cm, self._lat, self._lng)
        end,
        offset_bearing = function(_, _brg, _dist) return end,
        get_distance = function(_, _other) return M.state.distance_m end,
    }
    return loc
end

M.state = default_state()
M.messages = {}
M.servo_writes = {}
M.rc_overrides = {}
M.mode_attempts = {}

function M.set_state(patch)
    for k, v in pairs(patch) do M.state[k] = v end
end

function M.reset()
    M.state = default_state()
    M.messages = {}
    M.servo_writes = {}
    M.rc_overrides = {}
    M.mode_attempts = {}
end

local function install_globals()
    -- Mocks dereference M.state at call time so M.reset() (which rebinds
    -- M.state to a fresh table) takes effect even after install_globals().
    -- param:get() synthesizes SERVO%d_FUNCTION / SERVO%d_MIN from
    -- state.servo_funcs / state.servo_mins so tests can mutate them
    -- between ticks without an explicit sync step.

    _G.param = {
        get = function(_, name)
            local v = M.state.params[name]
            if v ~= nil then return v end
            local ch = name:match("^SERVO(%d+)_FUNCTION$")
            if ch then return M.state.servo_funcs[tonumber(ch)] end
            ch = name:match("^SERVO(%d+)_MIN$")
            if ch then
                local n = tonumber(ch)
                return (M.state.servo_mins[n] ~= nil) and M.state.servo_mins[n] or 1100
            end
            return nil
        end,
        add_table = function() return true end,
        add_param = function() return true end,
    }
    _G.rc = {
        get_pwm         = function(_, chan) return M.state.rc_pwms[chan] end,
        has_valid_input = function() return M.state.valid_rc end,
        get_channel     = function(_, chan)
            if chan == nil or chan < 1 or chan > 16 then return nil end
            return {
                set_override = function(_, pwm)
                    table.insert(M.rc_overrides, { chan = chan, pwm = pwm, t = M.state.time_ms })
                    M.state.rc_overrides[chan] = pwm
                end,
            }
        end,
    }
    _G.vehicle = {
        get_mode = function() return M.state.mode end,
        set_mode = function(_, m)
            local fail_for_this_mode = M.state.mode_fail_for[m] == true
            local ok = M.state.mode_change_ok and not fail_for_this_mode
            table.insert(M.mode_attempts, { mode = m, ok = ok, t = M.state.time_ms })
            if ok then M.state.mode = m end
            return ok
        end,
        set_poi_location = function() return true end,
    }
    _G.SRV_Channels = {
        set_output_pwm_chan_timeout = function(_, chan, pwm, timeout)
            table.insert(M.servo_writes, { chan = chan, pwm = pwm, timeout = timeout, t = M.state.time_ms })
            M.state.servo_pwm[chan] = pwm
        end,
        get_output_pwm_chan = function(_, chan) return M.state.servo_pwm[chan] end,
    }
    _G.gcs = {
        send_text = function(_, sev, msg)
            table.insert(M.messages, { sev = sev, msg = msg, t = M.state.time_ms })
        end,
    }
    _G.millis = function()
        local t = M.state.time_ms
        return { toint = function() return t end }
    end
    _G.ahrs = {
        get_hagl     = function() return M.state.hagl end,
        get_position = function()
            local p = M.state.position
            if p == nil then return nil end
            return mock_location(p.alt_cm, p.lat, p.lng)
        end,
        get_home = function()
            local h = M.state.home
            if h == nil then return nil end
            return mock_location(h.alt_cm, h.lat, h.lng)
        end,
        wind_estimate     = function() return mock_vector(M.state.wind_n, M.state.wind_e, 0) end,
        airspeed_estimate = function() return M.state.airspeed end,
        get_velocity_NED  = function() return mock_vector(0, 0, M.state.velocity_d) end,
        get_roll          = function() return M.state.roll end,
        get_pitch         = function() return M.state.pitch end,
    }
    _G.arming = { is_armed = function() return M.state.armed end }
    _G.RPM    = { get_rpm = function() return M.state.rpm end }
end

function M.load_script(path)
    install_globals()
    local update_fn, tick_ms = dofile(path)
    M.update = update_fn
    M.tick_ms = tick_ms
    return update_fn, tick_ms
end

function M.tick(n)
    n = n or 1
    for _ = 1, n do
        M.state.time_ms = M.state.time_ms + (M.tick_ms or 50)
        if M.update then M.update() end
    end
end

-- Tiny JSON encoder for the captured-output shape (arrays of flat tables
-- with string/number/bool/nil values). Sufficient for messages and
-- servo_writes; not a general encoder.
local function json_encode(v)
    local t = type(v)
    if t == "string" then
        return '"' .. v:gsub('\\', '\\\\'):gsub('"', '\\"'):gsub('\n', '\\n'):gsub('\r', '\\r'):gsub('\t', '\\t') .. '"'
    elseif t == "number" then
        if v ~= v then return "null" end -- NaN
        return tostring(v)
    elseif t == "boolean" then
        return tostring(v)
    elseif t == "nil" then
        return "null"
    elseif t == "table" then
        local n = #v
        local has_keys = false
        local key_count = 0
        for k, _ in pairs(v) do
            key_count = key_count + 1
            if type(k) ~= "number" or k < 1 or k > n or math.floor(k) ~= k then
                has_keys = true; break
            end
        end
        if has_keys then
            local parts = {}
            for k, val in pairs(v) do
                parts[#parts+1] = '"' .. tostring(k) .. '":' .. json_encode(val)
            end
            return "{" .. table.concat(parts, ",") .. "}"
        else
            -- Treat empty tables and integer-keyed tables as JSON arrays.
            -- Captured arrays (messages, servo_writes) are arrays even when empty.
            local parts = {}
            for i = 1, n do parts[i] = json_encode(v[i]) end
            return "[" .. table.concat(parts, ",") .. "]"
        end
    end
    return "null"
end

function M.dump()
    io.write(json_encode({
        messages      = M.messages,
        servo_writes  = M.servo_writes,
        rc_overrides  = M.rc_overrides,
        mode_attempts = M.mode_attempts,
        state_summary = {
            time_ms = M.state.time_ms,
            mode    = M.state.mode,
            rc_overrides = M.state.rc_overrides,
        },
    }))
end

return M
