-- Test harness for tools/tuning/lua/flight_tune.lua under plain Lua 5.3.
--
-- Mocks the ArduPilot scripting bindings the tuner uses and flies a simple
-- simulated fixed wing at 200 Hz:
--   * ArduPlane side: nav scripting as in ArduPlane 4.6.3 (enable only in the
--     current mode and only in LOITER/CRUISE/FBWB/..., 1 s timeout, a mode
--     change resets it), otherwise a mode that holds a bank and pitch angle;
--     a rate controller with FF/P/I/D, the speed scaler and the +-45 clip.
--   * Plant: roll rate K e^(-s tau) / ((1 + s/a)(1 + s/b)), pitch rate
--     K (1 + s/z) e^(-s tau) / ((1 + 2 zeta s/wn + s^2/wn^2)(1 + s/b)), both
--     from the controller output in degrees.
--   * The script is called when its requested delay has passed. Each call runs
--     under a count hook that raises "Exceeded CPU time" after VM_I_COUNT
--     instructions, as AP_Scripting does.

local H = {}
local S
local DT = 0.005
local VM_I_COUNT = 10000
local HOOK_STEP = 100

local function new_state()
    return {
        t = 0,                       -- s
        params = {
            AIRSPEED_MIN = 20, AIRSPEED_MAX = 33, SCALING_SPEED = 24, PTCH2SRV_RLL = 1, SCHED_LOOP_RATE = 200,
            RCMAP_ROLL = 1, RCMAP_PITCH = 2, RCMAP_THROTTLE = 3,
            RLL_RATE_FF = 0.15, RLL_RATE_P = 0.05, RLL_RATE_I = 0.05, RLL_RATE_D = 0.0,
            RLL_RATE_IMAX = 0.666, RLL_RATE_FLTT = 3.18, RLL_RATE_FLTE = 0, RLL_RATE_FLTD = 10,
            RLL_RATE_NTF = 0, RLL_RATE_NEF = 0, RLL_RATE_PDMX = 0,
            RLL_RATE_SMAX = 150, PTCH_RATE_SMAX = 150,
            RLL2SRV_TCONST = 0.5, RLL2SRV_RMAX = 75,
            PTCH_RATE_FF = 0.6, PTCH_RATE_P = 0.08, PTCH_RATE_I = 0.1, PTCH_RATE_D = 0.0,
            PTCH_RATE_IMAX = 0.666, PTCH_RATE_FLTT = 2.0, PTCH_RATE_FLTE = 0, PTCH_RATE_FLTD = 10,
            PTCH_RATE_NTF = 0, PTCH_RATE_NEF = 0, PTCH_RATE_PDMX = 0,
            PTCH2SRV_TCONST = 0.6, PTCH2SRV_RMAX_UP = 60, PTCH2SRV_RMAX_DN = 60,
            RC9_OPTION = 300,
        },
        saved = {},
        param_writes = {},
        tables = {},
        msgs = {},
        long_msgs = {},
        fail_set = {},
        logs = {},
        log_rows = {FTGN = {}, FTVR = {}, FTDC = {}, FTMD = {}, FTFR = {}, FTSP = {}, FTPR = {}},
        aux = {},
        pwm = {[1] = 1500, [2] = 1500, [3] = 1500, [9] = 1000},
        norm = {[1] = 0, [2] = 0, [3] = 0},
        rc_valid = true,
        armed = true,
        flying = true,
        mode = 12,
        ns = {enabled = false, ms = 0, roll = 0, pitch = 0, thr = 0},
        ns_enable_true = 0,
        ns_calls = 0,
        ns_active_ticks = 0,
        ns_max_rate = 0,
        asp = 24, e2t = 1, alt = 150,
        batt_fs = false, ekf_fs = false,
        -- plant
        plant = {roll = {K = 4.0, a = 6.0, b = 40.0, tau = 0.02},
                 pitch = {K = 1.0, z = 3.0, wn = 9.0, zeta = 0.5, b = 40.0, tau = 0.02, bias = -2.0}},
        mode_roll = 20, mode_pitch = 2,
        x = {phi = 20, p = 0, rl = 0, theta = 2, x1 = 0, x2 = 0, el = 0},
        buf = {roll = {}, pitch = {}},
        ctl = {roll = {i = 0, tf = 0, derr = 0, eprev = 0, out = 0},
               pitch = {i = 0, tf = 0, derr = 0, eprev = 0, out = 0}},
        thr = 55,
        next_call = 0,
        script_dead = false,
        script_error = nil,
        max_instr = 0,
        calls = 0,
        stall_until = nil,
        jitter_s = nil,
        rng = math.random,
        hooks = {},
    }
end

local function millis_obj(ms)
    return {toint = function() return math.floor(ms) end, tofloat = function() return ms + 0.0 end}
end

local function now_ms() return S.t * 1000 end

local function deg(x) return x * 180 / math.pi end
local function rad(x) return x * math.pi / 180 end

local NS_MODES = {[1] = true, [2] = true, [4] = true, [5] = true, [6] = true, [7] = true, [12] = true}

local function install_mocks()
    _G.millis = function() return millis_obj(now_ms()) end
    _G.param = {
        add_table = function(_, key, prefix, n)
            S.tables[key] = prefix
            return true
        end,
        add_param = function(_, key, idx, name, default)
            local full = S.tables[key] .. name
            if S.params[full] == nil then S.params[full] = default end
            return true
        end,
        get = function(_, name) return S.params[name] end,
        set = function(_, name, v)
            if S.params[name] == nil or S.fail_set[name] then return false end
            S.params[name] = v
            S.param_writes[#S.param_writes + 1] = name
            return true
        end,
        set_and_save = function(_, name, v)
            if S.params[name] == nil or S.fail_set[name] then return false end
            S.params[name] = v
            S.saved[name] = v
            S.param_writes[#S.param_writes + 1] = name
            return true
        end,
    }
    _G.gcs = {send_text = function(_, sev, msg)
        S.msgs[#S.msgs + 1] = msg
        -- MAVLink STATUSTEXT carries 50 characters; longer text is cut in logs and on old GCSs
        if #msg > 50 and not string.find(msg, "internal error", 1, true) then S.long_msgs[#S.long_msgs + 1] = msg end
    end}
    _G.logger = {write = function(_, name, labels, fmt, ...)
        S.logs[name] = (S.logs[name] or 0) + 1
        local rows = S.log_rows[name]
        if rows then
            local row = {}
            local i = 1
            for lab in string.gmatch(labels, "[^,]+") do
                row[lab] = select(i, ...)
                i = i + 1
            end
            rows[#rows + 1] = row
        end
        assert(#fmt == select("#", ...), "logger: format does not match arguments for " .. name)
    end}
    _G.vehicle = {
        get_mode = function() return S.mode end,
        get_likely_flying = function() return S.flying end,
        has_ekf_failsafed = function() return S.ekf_fs end,
        nav_scripting_enable = function(_, mode)
            if mode == S.mode and NS_MODES[S.mode] then
                S.ns.enabled = true
                S.ns.ms = now_ms()
                S.ns_enable_true = S.ns_enable_true + 1
            else
                S.ns.enabled = false
            end
            return S.ns.enabled
        end,
        set_target_throttle_rate_rpy = function(_, thr, r, p, y)
            S.ns.thr, S.ns.roll, S.ns.pitch = thr, math.max(-180, math.min(180, r)), math.max(-180, math.min(180, p))
            S.ns.ms = now_ms()
            S.ns_calls = S.ns_calls + 1
        end,
    }
    _G.arming = {is_armed = function() return S.armed end}
    _G.battery = {has_failsafed = function() return S.batt_fs end}
    local function vec(x, y, z)
        return {x = function() return x end, y = function() return y end, z = function() return z end}
    end
    _G.ahrs = {
        get_gyro = function() return vec(rad(S.x.p), rad(S.q or 0), 0) end,
        get_roll = function() return rad(S.x.phi) end,
        get_pitch = function() return rad(S.x.theta) end,
        airspeed_estimate = function() return S.asp end,
        get_EAS2TAS = function() return S.e2t end,
        get_relative_position_D_home = function() return -S.alt end,
        healthy = function() return true end,
    }
    _G.SRV_Channels = {get_output_scaled = function(_, fn)
        if fn == 4 then return S.ctl.roll.out * 100 end
        if fn == 19 then return S.ctl.pitch.out * 100 end
        if fn == 70 then return S.thr end
        -- elevon mixer as ArduPlane servos.cpp with MIXING_GAIN 0.5, plus an optional offset
        local ail, elev = S.ctl.roll.out, S.ctl.pitch.out
        if fn == 77 then return math.max(-45, math.min(45, (elev - ail) * 0.5 + (S.elevon_offset or 0))) * 100 end
        if fn == 78 then return math.max(-45, math.min(45, (elev + ail) * 0.5 + (S.elevon_offset or 0))) * 100 end
        return 0
    end}
    _G.rc = {
        get_aux_cached = function(_, fn) return S.aux[fn] end,
        has_valid_input = function() return S.rc_valid end,
        get_pwm = function(_, ch) return S.pwm[ch] end,
        get_channel = function(_, n)
            return {norm_input = function() return S.norm[n] or 0 end}
        end,
    }
end

-- ---------- simulated ArduPlane + aircraft ----------
local function scaler()
    local p = S.params
    local amin = math.max(p.AIRSPEED_MIN, 5)
    local smin = math.min(0.5, p.SCALING_SPEED / (2 * p.AIRSPEED_MAX))
    local smax = math.max(2.0, p.SCALING_SPEED / (0.7 * amin))
    return math.max(smin, math.min(smax, p.SCALING_SPEED / S.asp))
end

local function lp_alpha(hz)
    if hz <= 0 then return 1 end
    local rc = 1 / (2 * math.pi * hz)
    return DT / (DT + rc)
end

local function rate_ctl(c, pre, target, meas, sc)
    local p = S.params
    c.tf = c.tf + lp_alpha(p[pre .. "FLTT"]) * (target - c.tf)
    local err = c.tf - meas
    local derr = (err - c.eprev) / DT
    c.eprev = err
    c.derr = c.derr + lp_alpha(p[pre .. "FLTD"]) * (derr - c.derr)
    local sc2 = sc * sc
    local imax = p[pre .. "IMAX"] * 180 / math.pi
    if math.abs(c.out) < 45 then
        c.i = c.i + sc2 * p[pre .. "I"] * err * DT
    end
    c.i = math.max(-imax, math.min(imax, c.i))
    local out = p[pre .. "FF"] * sc / S.e2t * c.tf + sc2 * (p[pre .. "P"] * err + p[pre .. "D"] * c.derr) + c.i
    c.out = math.max(-45, math.min(45, out))
end

local function angle_rate(err, tc, rmax_up, rmax_dn, off)
    local r = err / math.max(tc, 0.05) + (off or 0)
    if rmax_dn > 0 and r < -rmax_dn then r = -rmax_dn end
    if rmax_up > 0 and r > rmax_up then r = rmax_up end
    return r
end

local function delayed(buf, value, tau)
    buf[#buf + 1] = value
    local n = math.floor(tau / DT + 0.5) + 1
    while #buf > n do table.remove(buf, 1) end
    return buf[1]
end

local function sim_step()
    local p = S.params
    local x = S.x
    -- ArduPlane nav scripting timeout (Plane::nav_scripting_active)
    if S.ns.enabled and now_ms() - S.ns.ms > 1000 then S.ns.enabled = false end
    local sc = scaler()
    local rr, pr
    if S.ns.enabled then
        rr, pr = S.ns.roll, S.ns.pitch
        S.ns_active_ticks = S.ns_active_ticks + 1
        S.ns_max_rate = math.max(S.ns_max_rate, math.abs(rr))
    else
        rr = angle_rate(S.mode_roll - x.phi, p.RLL2SRV_TCONST, p.RLL2SRV_RMAX, p.RLL2SRV_RMAX)
        pr = angle_rate(S.mode_pitch - x.theta, p.PTCH2SRV_TCONST, p.PTCH2SRV_RMAX_UP, p.PTCH2SRV_RMAX_DN)
    end
    rate_ctl(S.ctl.roll, "RLL_RATE_", rr, x.p, sc)
    rate_ctl(S.ctl.pitch, "PTCH_RATE_", pr, S.q or 0, sc)
    -- roll plant
    local pl = S.plant.roll
    local ud = delayed(S.buf.roll, S.ctl.roll.out, pl.tau)
    x.rl = x.rl + DT * pl.b * (ud - x.rl)
    local dist = S.dist and S.dist(S.t) or 0
    x.p = x.p + DT * (pl.a * (pl.K * x.rl - x.p) + dist)
    x.phi = x.phi + DT * x.p
    -- pitch plant
    local pp = S.plant.pitch
    local ed = delayed(S.buf.pitch, S.ctl.pitch.out, pp.tau)
    x.el = x.el + DT * pp.b * (ed - x.el)
    local x1dd = pp.wn * pp.wn * (x.el + pp.bias - x.x1) - 2 * pp.zeta * pp.wn * x.x2
    x.x2 = x.x2 + DT * x1dd
    x.x1 = x.x1 + DT * x.x2
    S.q = pp.K * (x.x1 + x.x2 / pp.z)
    x.theta = x.theta + DT * S.q
    S.t = S.t + DT
end

-- ---------- script driving ----------
local update_fn

local function call_script()
    local count = 0
    debug.sethook(function()
        count = count + 1
        if count * HOOK_STEP > VM_I_COUNT then
            local tb = debug.traceback("Exceeded CPU time", 2)
            S.first_overrun = S.first_overrun or tb
            error(tb, 0)
        end
    end, "", HOOK_STEP)
    local ok, fn, delay = pcall(update_fn)
    debug.sethook()
    S.calls = S.calls + 1
    if count * HOOK_STEP > S.max_instr then S.max_instr = count * HOOK_STEP end
    if not ok then
        S.script_dead = true
        S.script_error = tostring(fn)
        return
    end
    if fn == nil then
        S.script_dead = true
        return
    end
    update_fn = fn
    -- scheduling jitter of the scripting thread: S.jitter_s adds a random late start
    local late = 0
    if S.jitter_s then late = S.rng() * S.jitter_s end
    S.next_call = S.t + delay / 1000 + late
    S.max_late = math.max(S.max_late or 0, late)
end

function H.load_script(path, export)
    S = new_state()
    install_mocks()
    if export then _G.FTUN_TEST_EXPORT = {} end
    local chunk = assert(loadfile(path))
    local fn, delay = chunk()
    update_fn = fn
    S.next_call = S.t + delay / 1000
    return _G.FTUN_TEST_EXPORT
end

function H.state() return S end

-- advance the simulation; calls the script when due (unless stalled)
function H.run(seconds)
    local stop = S.t + seconds
    while S.t < stop - 1e-9 do
        for _, h in ipairs(S.hooks) do h(S) end
        if not S.script_dead and S.t >= S.next_call - 1e-9 then
            if not (S.stall_until and S.t < S.stall_until) then call_script() end
        end
        sim_step()
    end
end

-- run until a message containing text appears (or the time limit)
function H.run_until(text, seconds)
    local stop = S.t + seconds
    while S.t < stop do
        local n = #S.msgs
        H.run(0.1)
        for i = n + 1, #S.msgs do
            if string.find(S.msgs[i], text, 1, true) then return true end
        end
    end
    return false
end

function H.set_switch(pos) S.aux[300] = pos end

-- ---------- JSON out ----------
local function encode(v)
    local t = type(v)
    if t == "table" then
        if #v > 0 or next(v) == nil then
            local parts = {}
            for i = 1, #v do parts[i] = encode(v[i]) end
            return "[" .. table.concat(parts, ",") .. "]"
        end
        local parts = {}
        for k, val in pairs(v) do
            if type(val) ~= "function" then parts[#parts + 1] = string.format("%q", tostring(k)) .. ":" .. encode(val) end
        end
        return "{" .. table.concat(parts, ",") .. "}"
    elseif t == "string" then
        return string.format("%q", v)
    elseif t == "number" then
        if v ~= v or v == math.huge or v == -math.huge then return "null" end
        return string.format("%.10g", v)
    elseif t == "boolean" then
        return tostring(v)
    end
    return "null"
end

function H.dump(extra)
    local out = {
        msgs = S.msgs, long_msgs = S.long_msgs, saved = S.saved, logs = S.logs, rows = S.log_rows,
        max_instr = S.max_instr, calls = S.calls, script_dead = S.script_dead,
        script_error = S.script_error, first_overrun = S.first_overrun, ns_enable_true = S.ns_enable_true, ns_calls = S.ns_calls,
        ns_enabled = S.ns.enabled, ns_active_ticks = S.ns_active_ticks, param_writes = S.param_writes,
        t = S.t, ns_max_rate = S.ns_max_rate,
        params = {}, extra = extra,
    }
    for k, v in pairs(S.params) do
        if string.find(k, "RATE_") or string.find(k, "FTUN_") then out.params[k] = v end
    end
    print(encode(out))
end

H.encode = encode
return H
