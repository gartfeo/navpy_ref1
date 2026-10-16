--[[
flight_tune.lua - in-flight roll and pitch tuning for ArduPlane 4.6 fixed wings
(elevons, or ailerons and elevator; no rudder control).

Status: SITL only. Not cleared for flight.

What it does, one axis at a time (roll, then pitch):
  1. Test. Short left-right rolls (pitch up-down for pitch), flown by the
     script through ArduPilot's own rate controller. The script sends rate
     demands with vehicle:set_target_throttle_rate_rpy, the hook ArduPilot's
     aerobatics scripts use. It measures the frequency response from the
     controller output to the body rate, using its own test signal as the
     reference, so turbulence and feedback do not bias it.
  2. Model. A small model is fitted to that response (gain, poles, delay).
  3. Design. FF comes from the model's steady-state gain. P, I and D are
     searched for the fastest rate loop that keeps the phase and gain M.margins
     (FTUN_PM, FTUN_GM) at AIRSPEED_MIN, the test speed and AIRSPEED_MAX, with
     the measured response kept away from -1. TCONST and RMAX are not changed.
  4. Check. Attitude steps are flown with the old and the new gains
     alternately (old, new, old, new). The new gains are kept only if they
     track better in both directions without more overshoot or servo activity.
     Kept gains are saved when FTUN_SAVE is 1; otherwise they stay active until a
     reboot brings back the saved values.

Use: FTUN_ENABLE 1 and a switch with RCx_OPTION 300 (Scripting1). Fly LOITER,
CRUISE or FBWB at a safe height and move the switch high. Switch low, stick
input, a mode change, RC loss, a failsafe, the parachute switch, low height or
airspeed out of range stops it at once. Script control is released and the
gains of the axis being tuned go back to their old values. New gains are active
only while the script itself flies a check manoeuvre, until they are kept.

Known limits of a Lua tuner (review 2026-10-11): if the script dies during a check
with new gains (instruction overrun, out of memory), ArduPlane drops script control
after 1 s but the new gains stay until reboot; Lua 4.6 cannot read airspeed-sensor
health. Firmware ownership of temporary gains would close both.

Why not FBWA: chute_deploy.lua takes over by switching to FBWA, and only a real
mode change ends script control (Plane::set_mode ignores a change to the mode
already active), so tuning in FBWA could hold off the parachute pitch-up.
--]]

-- ---------- Parameters ----------
local PARAM_TABLE_KEY = 91
local PREFIX = "FTUN_"
local PARAM_DEFS = {
    {1,  "ENABLE",     0},    -- 1 lets the switch start a tune
    {2,  "AXES",       3},    -- bit 0 roll, bit 1 pitch
    {3,  "ALT_MIN",    80},   -- m above home: no test below this height
    {4,  "ALT_LOSS",   25},   -- m: a manoeuvre stops after this height loss
    {5,  "RLL_RATE",   60},   -- deg/s: roll test rate
    {6,  "PIT_RATE",   15},   -- deg/s: pitch test rate
    {7,  "SPD_MARGIN", 3},    -- m/s: tests start only this far inside AIRSPEED_MIN..MAX
    {8,  "PM",         45},   -- deg: minimum phase margin of new gains
    {9,  "GM",         6},    -- dB: minimum gain margin of new gains
    {10, "STICK",      0.1},  -- stick deflection (0..1) that stops the tune
    {11, "REPS",       4},    -- test manoeuvres per axis before the fit
    {12, "IMPROVE",    0.05}, -- new gains must cut tracking error by this fraction
    {13, "SAVE",       1},    -- 1: save kept gains
}
assert(param:add_table(PARAM_TABLE_KEY, PREFIX, 20), "flight_tune: cannot add FTUN_ table (key 91)")
for _, d in ipairs(PARAM_DEFS) do
    assert(param:add_param(PARAM_TABLE_KEY, d[1], d[2], d[3]), "flight_tune: cannot add FTUN_" .. d[2])
end
local function fparam(name) return param:get(PREFIX .. name) end

-- ---------- Constants ----------
-- one table: ArduPilot's Lua allows at most 100 active locals per function
local K = {}
K.SWITCH_AUX = 300   -- RCx_OPTION Scripting1: the tuning switch
K.AUTOTUNE_AUX = 107   -- RCx_OPTION AUTOTUNE: must be off
K.SWITCH_HIGH = 2     -- rc:get_aux_cached position
K.NO_MODE = 255   -- nav_scripting_enable(255) never matches the mode, so it releases control
K.MODE_FBWB, K.MODE_CRUISE, K.MODE_LOITER = 6, 7, 12
K.ALLOWED_MODES = {[K.MODE_FBWB] = "FBWB", [K.MODE_CRUISE] = "CRUISE", [K.MODE_LOITER] = "LOITER"}
K.FN_AILERON, K.FN_ELEVATOR, K.FN_THROTTLE = 4, 19, 70   -- SRV_Channel functions
K.FN_ELEVON_LEFT, K.FN_ELEVON_RIGHT = 77, 78            -- mixed outputs, clipped at +-45 (servos.cpp)
K.CHUTE_HIGH_PWM = 1700  -- chute_deploy.lua RC_HIGH_PWM
K.GRAVITY = 9.80665
K.MIN_AIRSPEED_MIN = 5    -- ArduPlane defines.h, used by the speed scaler

K.IDLE_TICK_MS = 200
K.WAIT_TICK_MS = 50
K.MAN_TICK_MS = 20    -- 50 Hz manoeuvre loop
K.COMPUTE_TICK_MS = 10
K.MAX_TICK_GAP_MS = 100   -- a longer gap means the demand was held too long: the record is dropped
K.GAP_BETWEEN_MS = 3000  -- released flight between manoeuvres
K.WAIT_TIMEOUT_MS = 60000 -- stop if conditions do not allow a manoeuvre for this long
K.START_ALT_MARGIN = 10   -- m above FTUN_ALT_MIN to start a manoeuvre
K.START_PITCH_MAX = 15    -- deg
K.LEVEL_RATE_MAX = 45    -- deg/s while levelling the wings before a test
K.LEVEL_DEG = 5
K.LEVEL_RATE_DPS = 10
K.LEVEL_TIMEOUT_S = 3
K.SETTLE_S = 0.6   -- level hold before the test signal; sets the trim offsets
K.ROLL_ENVELOPE = 60    -- deg: stop if exceeded
K.TEST_ANGLE_MAX = {35, 10}  -- deg, roll and pitch: largest planned excursion of a test doublet
K.PITCH_ENVELOPE_UP, K.PITCH_ENVELOPE_DN = 30, -30
K.OUT_SAT = 44    -- deg of controller output; the rate controllers clip at 45
K.SAT_TICKS = 3
K.SAT_AMP_FACTOR = 0.7   -- test rate reduction after a saturated record
K.MAX_SOFT_ABORTS = 4
K.MAX_REPS = 8
K.REPS_EXTRA = 2     -- extra records when the response scatter is high
K.SPREAD_MAX = 0.35  -- median record-to-record scatter of ln G at 3-20 rad/s
K.SIGMA_FLOOR = 0.05
K.EXC_MIN_FRAC = 1e-3  -- frequencies with less input than this fraction are not used
K.FIT_RMS_MAX = 0.3   -- RMS of the fit error in ln G (about 30 % or 17 deg)
K.LM_MAX_ITER = 30
K.LM_STARTS = 3
K.JAC_COLS_PER_TICK = 2
K.I_RATIO = 5     -- integrator corner = crossover / I_RATIO
-- margins must hold with this much more delay than measured (delay estimate error,
-- servo load); predictions use the measured delay
K.DELAY_MARGIN_S = 0.010
-- rate-loop crossover at most this fraction of the highest usable test frequency, so
-- the margins rest on measured response and not on the model beyond it
K.WC_BAND_FRAC = 0.5
-- phase margin may be this much lower at AIRSPEED_MIN and AIRSPEED_MAX than at the test
-- speed: the model's speed scaling is an assumption there, and ArduPilot's own tuning
-- holds no margin away from its test speed
K.EDGE_PM_RELIEF = 10
-- FF tried as these fractions of the model's steady-state value: a lower FF slows the
-- angle loop when TCONST asks for more than the airframe's roll or pitch lag allows
K.FF_GRID = {1.0, 0.85, 0.7, 0.55}
-- designs within this fraction of the lowest predicted step error count as equal; the
-- one with the faster rate loop (better gust rejection) is then chosen
K.MAE_TIE = 0.03
K.MS_MAX = 2.0   -- largest allowed sensitivity peak on measured points
K.HF_W = 50    -- rad/s: frequency where servo jitter is scaled between gain sets
-- predicted servo jitter slew must stay below this part of SMAX, so ArduPilot's slew
-- limiter does not start reducing P and D
K.SLEW_FRACTION = 0.25
K.SLEW_REF_DEFAULT = 150  -- deg/s, used when SMAX is 0 (slew limiter off); ArduPilot's default
K.FF_CHANGE_MAX = 3.0   -- model FF more than this factor from the current FF: data suspect
K.NO_CHANGE_FRAC = 0.05  -- all gains within this fraction: keep the current gains
K.OVERSHOOT_MAX = 0.10
K.OVERSHOOT_ABS_MAX = 0.20   -- new gains never kept above this, even if the old ones overshoot more
-- deg: peak controller output allowed on a check step in the model at the test speed
-- (of +-45), or the current gains' peak if that is higher
K.OUT_USE_MAX = 35
K.SIM_STEPS_PER_TICK = 40
K.ACTIVITY_GROWTH = 2.0   -- RMS output slew on holds (deg/s) may grow at most this much
K.ACTIVITY_FLOOR = 2.5   -- deg/s of controller output slew
K.ACT_WINDOW_S = 0.8   -- end of each hold used for servo activity
K.VERIFY_SUBSTEPS = 4
K.PARAM_TOL = 1e-5     -- relative difference that counts as a changed parameter
K.FROZEN_CHECK_MS = 200   -- parameter check interval while computing (name lookups cost CPU)

-- test signal: {seconds, rate as a fraction of the test rate}; doublets of
-- 0.5, 0.25, 0.12 and 0.06 s cover about 1.5 to 40 rad/s
K.ID_PROGRAM = {
    {0.5, 1}, {0.5, -1}, {0.4, 0},
    {0.25, -1}, {0.25, 1}, {0.3, 0},
    {0.12, 1}, {0.12, -1}, {0.3, 0},
    {0.06, -1}, {0.06, 1}, {0.06, -1}, {0.06, 1}, {1.0, 0},
}
-- check manoeuvre: attitude offsets in units of the axis step size; every step
-- has the same size, two in each direction
K.VERIFY_PROGRAM = {{1.5, 1}, {1.5, 0}, {1.5, -1}, {1.5, 0}}
-- design check: one step of the axis size, as long as one check hold
K.SEARCH_PROGRAM = {{1.5, 1}}

local M = {}         -- identification and design maths
local abs, sqrt, exp, log, sin, cos, atan, floor = math.abs, math.sqrt, math.exp, math.log, math.sin, math.cos, math.atan, math.floor
local DEG = 180 / math.pi
local LN10 = log(10)

local NW = 15
local ID_W = {}                 -- rad/s, frequencies of the measured response
for i = 1, NW do ID_W[i] = 1.5 * (38 / 1.5) ^ ((i - 1) / (NW - 1)) end
local NDW = 40
local DES_W, DES_LNW = {}, {}   -- rad/s, design frequencies
for k = 1, NDW do
    DES_W[k] = 0.5 * (150 / 0.5) ^ ((k - 1) / (NDW - 1))
    DES_LNW[k] = log(DES_W[k])
end
K.TD_GRID = {0, 0.0025, 0.005, 0.0075, 0.01, 0.015, 0.02, 0.03}   -- D/P in seconds
K.P_GRID_LO, K.P_GRID_HI = -12, 6                                     -- P = P_ref * 2^(k/4)

local AXES = {
    {id = 1, name = "roll", bit = 1, out_fn = K.FN_AILERON, rate_param = "RLL_RATE", step_deg = 20,
     gains = {FF = "RLL_RATE_FF", P = "RLL_RATE_P", I = "RLL_RATE_I", D = "RLL_RATE_D"},
     tc = "RLL2SRV_TCONST", rmax_up = "RLL2SRV_RMAX", rmax_dn = "RLL2SRV_RMAX", imax = "RLL_RATE_IMAX",
     fltt = "RLL_RATE_FLTT", flte = "RLL_RATE_FLTE", fltd = "RLL_RATE_FLTD", smax = "RLL_RATE_SMAX",
     unsupported = {"RLL_RATE_NTF", "RLL_RATE_NEF", "RLL_RATE_PDMX", "RLL_RATE_D_FF"}},
    {id = 2, name = "pitch", bit = 2, out_fn = K.FN_ELEVATOR, rate_param = "PIT_RATE", step_deg = 5,
     gains = {FF = "PTCH_RATE_FF", P = "PTCH_RATE_P", I = "PTCH_RATE_I", D = "PTCH_RATE_D"},
     tc = "PTCH2SRV_TCONST", rmax_up = "PTCH2SRV_RMAX_UP", rmax_dn = "PTCH2SRV_RMAX_DN", imax = "PTCH_RATE_IMAX",
     fltt = "PTCH_RATE_FLTT", flte = "PTCH_RATE_FLTE", fltd = "PTCH_RATE_FLTD", smax = "PTCH_RATE_SMAX",
     unsupported = {"PTCH_RATE_NTF", "PTCH_RATE_NEF", "PTCH_RATE_PDMX", "PTCH_RATE_D_FF"}},
}
local GAIN_KEYS = {"FF", "P", "I", "D"}

-- ---------- Small helpers ----------
local function clamp(x, lo, hi)
    if x < lo then return lo elseif x > hi then return hi end
    return x
end

local function say(sev, fmt, ...)
    gcs:send_text(sev, "FTUN: " .. string.format(fmt, ...))
end

local function copy(t)
    local c = {}
    for i = 1, #t do c[i] = t[i] end
    return c
end

-- Plane::calc_speed_scaler with an airspeed estimate
local function speed_scaler(asp, ac)
    local amin = math.max(ac.AIRSPEED_MIN, K.MIN_AIRSPEED_MIN)
    local smin = math.min(0.5, ac.SCALING_SPEED / (2 * ac.AIRSPEED_MAX))
    local smax = math.max(2.0, ac.SCALING_SPEED / (0.7 * amin))
    local s = smax
    if asp > 0.0001 then s = ac.SCALING_SPEED / asp end
    return clamp(s, smin, smax)
end

-- AP_PitchController::_get_coordination_rate_offset (upright), deg/s
local function coord_rate(roll_deg, pitch_deg, asp, e2t, ac)
    if abs(pitch_deg) > 70 then return 0 end
    local bank = clamp(roll_deg, -80, 80) / DEG
    local v = math.max(asp * e2t, math.max(ac.AIRSPEED_MIN, 1))
    return cos(pitch_deg / DEG) * abs(DEG * (K.GRAVITY / v) * math.tan(bank) * sin(bank)) * ac.PTCH2SRV_RLL
end

-- angle loop of get_servo_out: error / TCONST, limited to RMAX (0 = no limit)
local function angle_rate(err_deg, tc, rmax_up, rmax_dn, offset)
    local r = err_deg / math.max(tc, 0.05) + (offset or 0)
    if rmax_dn > 0 and r < -rmax_dn then r = -rmax_dn end
    if rmax_up > 0 and r > rmax_up then r = rmax_up end
    return r
end

-- ---------- Plant models ----------
-- roll:  x = {ln K, ln a, ln b, tau}
--        G(s) = K e^(-s tau) / ((1 + s/a)(1 + s/b)), a <= b; a scales with airspeed
-- pitch: x = {ln K, ln z, ln wn, zeta, ln b, tau}
--        G(s) = K (1 + s/z) e^(-s tau) / ((1 + 2 zeta s/wn + s^2/wn^2)(1 + s/b))
-- G is body rate (deg/s) per controller output (deg) at the test airspeed. At
-- speed ratio r the gain, a, z and wn scale with r (control power with r^2,
-- damping with r); the lag b and the delay do not.
-- Returns ln|G(jw)| and the phase, continuous in w.
function M.model_logresp(axis_id, x, w, r)
    if axis_id == 1 then
        local wa, wb = w / (exp(x[2]) * r), w / exp(x[3])
        return x[1] + log(r) - 0.5 * log(1 + wa * wa) - 0.5 * log(1 + wb * wb),
               -atan(wa) - atan(wb) - w * x[4]
    end
    local wz, ww, wb = w / (exp(x[2]) * r), w / (exp(x[3]) * r), w / exp(x[5])
    local re, im = 1 - ww * ww, 2 * x[4] * ww
    return x[1] + log(r) + 0.5 * log(1 + wz * wz) - 0.5 * log(re * re + im * im) - 0.5 * log(1 + wb * wb),
           atan(wz) - atan(im, re) - atan(wb) - w * x[6]
end

K.BOUNDS = {
    {lo = {log(0.05), log(0.3), log(3), 0}, hi = {log(500), log(80), log(400), 0.12}},
    {lo = {log(0.01), log(0.2), log(1), 0.1, log(3), 0}, hi = {log(500), log(50), log(80), 2.5, log(400), 0.12}},
}

-- ---------- Frequency response estimate ----------
-- One record: running Fourier sums of rate p, output u and test signal x.
function M.dft_new()
    local d = {pr = {}, pi = {}, ur = {}, ui = {}, xr = {}, xi = {}, ec = {}, es = {}}
    for i = 1, NW do
        d.pr[i], d.pi[i], d.ur[i], d.ui[i], d.xr[i], d.xi[i] = 0, 0, 0, 0, 0, 0
    end
    return d
end

-- Adds the interval from the previous sample to this one (time t, rate p, output u;
-- x is the test signal held over the interval). The integral of e^(-jwt) times the
-- straight line between the two samples is exact, so uneven script timing does not
-- bias the sums (a plain sum of samples would).
function M.dft_add(d, t, p, u, x)
    local ec, es = d.ec, d.es
    if d.t0 == nil then
        for i = 1, NW do
            local a = ID_W[i] * t
            ec[i], es[i] = cos(a), sin(a)
        end
        d.t0, d.p0, d.u0 = t, p, u
        return
    end
    local h = t - d.t0
    if h <= 0 then return end
    local pr, pi, ur, ui, xr, xi = d.pr, d.pi, d.ur, d.ui, d.xr, d.xi
    local p0, u0 = d.p0, d.u0
    local mp, mu = (p - p0) / h, (u - u0) / h
    for i = 1, NW do
        local w = ID_W[i]
        local c1, s1 = cos(w * t), sin(w * t)
        -- A = (E0 - E1) / (jw), B = integral of (t - t0) e^(-jwt), E = e^(-jwt)
        local dr, di = ec[i] - c1, s1 - es[i]
        local ar, ai = di / w, -dr / w
        local w2 = w * w
        local br, bi = h * s1 / w - dr / w2, h * c1 / w - di / w2
        pr[i] = pr[i] + p0 * ar + mp * br
        pi[i] = pi[i] + p0 * ai + mp * bi
        ur[i] = ur[i] + u0 * ar + mu * br
        ui[i] = ui[i] + u0 * ai + mu * bi
        xr[i] = xr[i] + x * ar
        xi[i] = xi[i] + x * ai
        ec[i], es[i] = c1, s1
    end
    d.t0, d.p0, d.u0 = t, p, u
end

-- add a finished record: cross spectra with the test signal, and the record's own P/U
function M.spectra_add(sp, d)
    local g = {r = {}, i = {}}
    for i = 1, NW do
        local xr, xi = d.xr[i], d.xi[i]
        sp.pxr[i] = sp.pxr[i] + d.pr[i] * xr + d.pi[i] * xi
        sp.pxi[i] = sp.pxi[i] + d.pi[i] * xr - d.pr[i] * xi
        sp.uxr[i] = sp.uxr[i] + d.ur[i] * xr + d.ui[i] * xi
        sp.uxi[i] = sp.uxi[i] + d.ui[i] * xr - d.ur[i] * xi
        local den = d.ur[i] * d.ur[i] + d.ui[i] * d.ui[i]
        if den > 0 then
            g.r[i] = (d.pr[i] * d.ur[i] + d.pi[i] * d.ui[i]) / den
            g.i[i] = (d.pi[i] * d.ur[i] - d.pr[i] * d.ui[i]) / den
        else
            g.r[i], g.i[i] = 0, 0
        end
    end
    sp.recs[#sp.recs + 1] = g
end

function M.spectra_new()
    local sp = {pxr = {}, pxi = {}, uxr = {}, uxi = {}, recs = {}}
    for i = 1, NW do sp.pxr[i], sp.pxi[i], sp.uxr[i], sp.uxi[i] = 0, 0, 0, 0 end
    return sp
end

-- G = sum(P X*) / sum(U X*), its ln magnitude and unwrapped phase, a weight per
-- frequency from the record-to-record scatter, and the median scatter at 3-20 rad/s.
function M.estimate(sp)
    local est = {gr = {}, gi = {}, lm = {}, ph = {}, w = {}, sd = {}, nvalid = 0}
    local umax = 0
    for i = 1, NW do umax = math.max(umax, sqrt(sp.uxr[i] ^ 2 + sp.uxi[i] ^ 2)) end
    local m = #sp.recs
    local prev
    local band = {}
    for i = 1, NW do
        local ur, ui = sp.uxr[i], sp.uxi[i]
        local den = ur * ur + ui * ui
        local gr, gi = 0, 0
        if den > 0 then
            gr = (sp.pxr[i] * ur + sp.pxi[i] * ui) / den
            gi = (sp.pxi[i] * ur - sp.pxr[i] * ui) / den
        end
        est.gr[i], est.gi[i] = gr, gi
        local mag2 = gr * gr + gi * gi
        est.w[i], est.sd[i], est.lm[i], est.ph[i] = 0, 0, 0, 0
        if sqrt(den) >= K.EXC_MIN_FRAC * umax and mag2 > 0 and m > 0 then
            est.lm[i] = 0.5 * log(mag2)
            local ph = atan(gi, gr)
            if prev then
                while ph - prev > math.pi do ph = ph - 2 * math.pi end
                while ph - prev < -math.pi do ph = ph + 2 * math.pi end
            end
            prev = ph
            est.ph[i] = ph
            local var = 0
            for k = 1, m do
                local rr, ri = sp.recs[k].r[i], sp.recs[k].i[i]
                -- ratio record / mean = g_k conj(G) / |G|^2
                local qr, qi = (rr * gr + ri * gi) / mag2, (ri * gr - rr * gi) / mag2
                local q2 = qr * qr + qi * qi
                if q2 > 0 then
                    local lr, la = 0.5 * log(q2), atan(qi, qr)
                    var = var + lr * lr + la * la
                else
                    var = var + 10
                end
            end
            var = var / m
            est.sd[i] = sqrt(var)
            est.w[i] = 1 / sqrt(var / m + K.SIGMA_FLOOR * K.SIGMA_FLOOR)
            est.nvalid = est.nvalid + 1
            if ID_W[i] >= 3 and ID_W[i] <= 20 then band[#band + 1] = est.sd[i] end
        end
    end
    table.sort(band)
    est.spread = #band > 0 and band[floor((#band + 1) / 2)] or 10
    return est
end

-- ---------- Model fit (Levenberg-Marquardt, a few steps per call) ----------
function M.residuals(axis_id, x, est, out)
    local n = 0
    for i = 1, NW do
        local w = est.w[i]
        if w > 0 then
            local lm, ph = M.model_logresp(axis_id, x, ID_W[i], 1)
            out[n + 1] = w * (lm - est.lm[i])
            out[n + 2] = w * (ph - est.ph[i])
            n = n + 2
        end
    end
    return n
end

function M.sumsq(r, n)
    local c = 0
    for i = 1, n do c = c + r[i] * r[i] end
    return c
end

-- solve the n x n system in M (augmented with column n+1); nil if singular
function M.gauss_solve(mat, n)
    for c = 1, n do
        local piv, best = c, abs(mat[c][c])
        for r = c + 1, n do
            if abs(mat[r][c]) > best then piv, best = r, abs(mat[r][c]) end
        end
        if best < 1e-300 then return nil end
        mat[c], mat[piv] = mat[piv], mat[c]
        for r = c + 1, n do
            local f = mat[r][c] / mat[c][c]
            if f ~= 0 then
                for k = c, n + 1 do mat[r][k] = mat[r][k] - f * mat[c][k] end
            end
        end
    end
    local x = {}
    for r = n, 1, -1 do
        local s = mat[r][n + 1]
        for k = r + 1, n do s = s - mat[r][k] * x[k] end
        x[r] = s / mat[r][r]
    end
    return x
end

function M.lm_new(axis_id, x0, est)
    return {axis = axis_id, x = copy(x0), n = #x0, est = est, b = K.BOUNDS[axis_id],
            r = {}, J = {}, lambda = 1e-2, iter = 0, col = 0, st = "init"}
end

-- one bounded piece of work; returns true when finished
function M.lm_step(lm)
    if lm.st == "init" then
        lm.m = M.residuals(lm.axis, lm.x, lm.est, lm.r)
        lm.cost = M.sumsq(lm.r, lm.m)
        lm.st, lm.col = "jac", 0
        return lm.m == 0
    elseif lm.st == "jac" then
        for _ = 1, K.JAC_COLS_PER_TICK do
            lm.col = lm.col + 1
            if lm.col > lm.n then
                lm.st = "normal"
                return false
            end
            local xh = copy(lm.x)
            local h = 1e-4
            xh[lm.col] = xh[lm.col] + h
            local rh = lm.J[lm.col] or {}
            M.residuals(lm.axis, xh, lm.est, rh)
            for i = 1, lm.m do rh[i] = (rh[i] - lm.r[i]) / h end
            lm.J[lm.col] = rh
        end
        return false
    elseif lm.st == "normal" then
        local n, m, J, r = lm.n, lm.m, lm.J, lm.r
        local A, g = {}, {}
        for a = 1, n do A[a] = {} end
        for a = 1, n do
            local Ja = J[a]
            local s = 0
            for i = 1, m do s = s + Ja[i] * r[i] end
            g[a] = -s
            for b = 1, a do
                local Jb = J[b]
                local t = 0
                for i = 1, m do t = t + Ja[i] * Jb[i] end
                A[a][b], A[b][a] = t, t
            end
        end
        lm.A, lm.g, lm.st = A, g, "trial"
        return false
    elseif lm.st == "trial" then
        local n = lm.n
        local mat = {}
        for a = 1, n do
            mat[a] = {}
            for b = 1, n do mat[a][b] = lm.A[a][b] end
            mat[a][a] = mat[a][a] * (1 + lm.lambda) + 1e-12
            mat[a][n + 1] = lm.g[a]
        end
        local dx = M.gauss_solve(mat, n)
        if dx then
            local xt = {}
            for a = 1, n do xt[a] = clamp(lm.x[a] + dx[a], lm.b.lo[a], lm.b.hi[a]) end
            local rt = {}
            M.residuals(lm.axis, xt, lm.est, rt)
            local ct = M.sumsq(rt, lm.m)
            if ct < lm.cost then
                local rel = (lm.cost - ct) / math.max(lm.cost, 1e-300)
                lm.x, lm.r, lm.cost = xt, rt, ct
                lm.lambda = math.max(lm.lambda / 3, 1e-7)
                lm.iter = lm.iter + 1
                if rel < 1e-6 or lm.iter >= K.LM_MAX_ITER then
                    lm.st = "done"
                    return true
                end
                lm.st, lm.col = "jac", 0
                return false
            end
        end
        lm.lambda = lm.lambda * 4
        if lm.lambda > 1e6 then
            lm.st = "done"
            return true
        end
        return false
    end
    return true
end

-- starting points for the fit; K is set from the lowest usable frequency
function M.fit_starts(axis_id, est)
    local i0
    for i = 1, NW do
        if est.w[i] > 0 then i0 = i break end
    end
    if not i0 then return {} end
    local w0, lm0 = ID_W[i0], est.lm[i0]
    local starts = {}
    if axis_id == 1 then
        for _, a in ipairs({2, 5, 12}) do
            for _, b in ipairs({25, 80}) do
                for _, tau in ipairs({0.01, 0.04}) do
                    local x = {0, log(a), log(b), tau}
                    x[1] = lm0 - (M.model_logresp(1, x, w0, 1) - 0)
                    starts[#starts + 1] = x
                end
            end
        end
    else
        for _, z in ipairs({1.5, 4}) do
            for _, wn in ipairs({4, 8, 16}) do
                local x = {0, log(z), log(wn), 0.6, log(60), 0.02}
                x[1] = lm0 - M.model_logresp(2, x, w0, 1)
                starts[#starts + 1] = x
            end
        end
    end
    return starts
end

-- unweighted RMS of the complex ln G error over the usable frequencies
function M.fit_rms(axis_id, x, est)
    local s, n = 0, 0
    for i = 1, NW do
        if est.w[i] > 0 then
            local lm, ph = M.model_logresp(axis_id, x, ID_W[i], 1)
            s = s + (lm - est.lm[i]) ^ 2 + (ph - est.ph[i]) ^ 2
            n = n + 1
        end
    end
    return n > 0 and sqrt(s / n) or 10
end

-- ---------- Loop analysis ----------
-- Phase and gain margin from ln|L| and continuous phase on DES_W.
-- PM is wrapped to [-180, 180); negative means unstable at that crossover.
function M.margins(lnmag, phase)
    local pm, gm, wc = 180, 60, nil
    for k = 1, NDW - 1 do
        local m1, m2 = lnmag[k], lnmag[k + 1]
        local p1, p2 = phase[k] * DEG, phase[k + 1] * DEG
        if m1 >= 0 and m2 < 0 then
            local f = m1 / (m1 - m2)
            local p = ((p1 + f * (p2 - p1) + 360) % 360) - 180
            if p < pm then pm = p end
            wc = exp(DES_LNW[k] + f * (DES_LNW[k + 1] - DES_LNW[k]))
        end
        local c1, c2 = floor((p1 + 180) / 360), floor((p2 + 180) / 360)
        if c1 ~= c2 then
            local target = 360 * math.max(c1, c2) - 180
            local f = (target - p1) / (p2 - p1)
            local g = -20 * (m1 + f * (m2 - m1)) / LN10
            if g < gm then gm = g end
        end
    end
    return pm, gm, wc
end

-- controller filter factors at one frequency: D path with FLTD, error filter FLTE,
-- and A = FT / (TCONST jw), the target filter then the angle loop's 1/(TCONST s)
function M.design_factors(ctl, w)
    local f = {}
    local wd = ctl.fltd > 0 and 2 * math.pi * ctl.fltd or nil
    local den = wd and (1 + (w / wd) ^ 2) or 1
    f.dre = wd and (w * w / wd) / den or 0
    f.dim = w / den
    local we = ctl.flte > 0 and 2 * math.pi * ctl.flte or nil
    f.felm = we and -0.5 * log(1 + (w / we) ^ 2) or 0
    f.feph = we and -atan(w / we) or 0
    local wt = ctl.fltt > 0 and 2 * math.pi * ctl.fltt or nil
    local tre, tim = 1, 0
    if wt then
        local d2 = 1 + (w / wt) ^ 2
        tre, tim = 1 / d2, -(w / wt) / d2
    end
    local tau = math.max(ctl.tc, 0.05)
    f.ar, f.ai = tim / (tau * w), -tre / (tau * w)
    return f
end

-- design tables; dz.s[1..3] are AIRSPEED_MIN, the test speed and AIRSPEED_MAX.
-- The measured response runs from the logged output to the gyro as read; the
-- controller acts on a gyro sample up to one main-loop period older, so the
-- step simulation adds one loop period of delay (dz.loop_dt) and the margin
-- analysis adds that plus K.DELAY_MARGIN_S (dz.margin_dt).
function M.design_new(axis_id, x, ac, v_test, e2t, ctl, est)
    local loop_dt = 1 / clamp(ac.SCHED_LOOP_RATE, 50, 400)
    local w_top = 0
    for i = 1, NW do
        if est.w[i] > 0 then w_top = ID_W[i] end
    end
    return {s = {}, e2t = e2t, ctl = ctl, axis = axis_id, x = x, est = est, ac = ac, v_test = v_test,
            speeds = {ac.AIRSPEED_MIN, v_test, ac.AIRSPEED_MAX}, part = 0,
            loop_dt = loop_dt, margin_dt = loop_dt + K.DELAY_MARGIN_S, wc_max = K.WC_BAND_FRAC * w_top}
end

-- builds the tables one piece per call; returns true when complete
function M.design_build(dz)
    dz.part = dz.part + 1
    if dz.part <= 3 then
        local v = dz.speeds[dz.part]
        local r = v / dz.v_test
        local sp = {v = v, sc = speed_scaler(v, dz.ac), glm = {}, gph = {}}
        for k = 1, NDW do
            sp.glm[k], sp.gph[k] = M.model_logresp(dz.axis, dz.x, DES_W[k], r)
            sp.gph[k] = sp.gph[k] - DES_W[k] * dz.margin_dt
        end
        dz.s[dz.part] = sp
        return false
    elseif dz.part == 4 then
        dz.f = {}
        for k = 1, NDW do dz.f[k] = M.design_factors(dz.ctl, DES_W[k]) end
        return false
    end
    dz.fm = {}
    for i = 1, NW do dz.fm[i] = M.design_factors(dz.ctl, ID_W[i]) end
    return true
end

-- controller (P, I, D with the D filter, no scaler, no error filter) at one frequency
function M.c_parts(f, w, P, I, D)
    return P + D * f.dre, -I / w + D * f.dim
end

-- rate-loop margins at speed index si
function M.rate_margins(dz, si, P, I, D, lnmag, phase)
    local sp = dz.s[si]
    local lsc = 2 * log(sp.sc)
    for k = 1, NDW do
        local f = dz.f[k]
        local cre, cim = M.c_parts(f, DES_W[k], P, I, D)
        lnmag[k] = sp.glm[k] + lsc + 0.5 * log(cre * cre + cim * cim) + f.felm
        phase[k] = sp.gph[k] + atan(cim, cre) + f.feph
    end
    return M.margins(lnmag, phase)
end

-- attitude-loop margins at the test speed:
-- L = G [C (1 + A) + Kff A], Kff = FF scaler / EAS2TAS, A = FT / (TCONST s)
function M.attitude_margins(dz, P, I, D, FF, lnmag, phase)
    local sp = dz.s[2]
    local sc2 = sp.sc * sp.sc
    local kff = FF * sp.sc / dz.e2t
    local prev
    for k = 1, NDW do
        local f = dz.f[k]
        local cre, cim = M.c_parts(f, DES_W[k], P, I, D)
        local fem = exp(f.felm)
        local fr, fi = fem * cos(f.feph), fem * sin(f.feph)
        cre, cim = sc2 * (cre * fr - cim * fi), sc2 * (cre * fi + cim * fr)
        -- C (1 + A) + Kff A
        local ar1, ai1 = 1 + f.ar, f.ai
        local mre = cre * ar1 - cim * ai1 + kff * f.ar
        local mim = cre * ai1 + cim * ar1 + kff * f.ai
        local ph = atan(mim, mre)
        if prev then
            while ph - prev > math.pi do ph = ph - 2 * math.pi end
            while ph - prev < -math.pi do ph = ph + 2 * math.pi end
        end
        prev = ph
        lnmag[k] = sp.glm[k] + 0.5 * log(mre * mre + mim * mim)
        phase[k] = sp.gph[k] + ph
    end
    return M.margins(lnmag, phase)
end

-- smallest |1 + L| over the measured points (test speed, measured response)
function M.measured_distance(dz, P, I, D)
    local est = dz.est
    local sc2 = dz.s[2].sc ^ 2
    local dmin = 1e9
    for i = 1, NW do
        if est.w[i] > 0 and est.sd[i] <= K.SPREAD_MAX then
            local f = dz.fm[i]
            local cre, cim = M.c_parts(f, ID_W[i], P, I, D)
            local fem = exp(f.felm)
            local fr, fi = fem * cos(f.feph), fem * sin(f.feph)
            cre, cim = sc2 * (cre * fr - cim * fi), sc2 * (cre * fi + cim * fr)
            -- measured G with the extra loop period of delay
            local dl = -ID_W[i] * dz.margin_dt
            local gr = est.gr[i] * cos(dl) - est.gi[i] * sin(dl)
            local gi = est.gr[i] * sin(dl) + est.gi[i] * cos(dl)
            local lr = gr * cre - gi * cim
            local li = gr * cim + gi * cre
            local d = sqrt((1 + lr) ^ 2 + li * li)
            if d < dmin then dmin = d end
        end
    end
    return dmin
end

-- controller gain magnitude at K.HF_W (scaler omitted: it is common to old and new gains)
function M.hf_gain(dz, P, I, D)
    local w = K.HF_W
    local wd = dz.ctl.fltd > 0 and 2 * math.pi * dz.ctl.fltd or nil
    local den = wd and (1 + (w / wd) ^ 2) or 1
    local cre = P + D * (wd and (w * w / wd) / den or 0)
    local cim = -I / w + D * w / den
    return sqrt(cre * cre + cim * cim)
end

-- Rate loop of gain set c.g, one speed per call (test speed, AIRSPEED_MIN, AIRSPEED_MAX).
-- Returns true when done; c.res.ok is false if a requirement failed (with req nil every
-- speed is evaluated). A loop with no crossover at a speed is stable there (wc 0).
K.EVAL_SPEED_ORDER = {2, 1, 3}
function M.eval_rate(dz, c, req, scratch)
    local g = c.g
    c.part = (c.part or 0) + 1
    local res = c.res
    if c.part == 1 then
        res = {ok = true, pm_min = 180, gm_min = 60, wc_min = 1e9, wc_max = 0}
        c.res = res
    end
    local si = K.EVAL_SPEED_ORDER[c.part]
    local pm, gm, wc = M.rate_margins(dz, si, g.P, g.I, g.D, scratch.m, scratch.p)
    res.pm_min = math.min(res.pm_min, pm)
    res.gm_min = math.min(res.gm_min, gm)
    if si == 2 then res.pm_t, res.gm_t, res.wc_t = pm, gm, wc end
    res.wc_min = math.min(res.wc_min, wc or 0)
    res.wc_max = math.max(res.wc_max, wc or 0)
    if req then
        local pm_req = si == 2 and req.pm or req.pm - K.EDGE_PM_RELIEF
        if pm < pm_req or gm < req.gm or (wc and wc > dz.wc_max) then
            res.ok = false
            return true
        end
    end
    return c.part >= #K.EVAL_SPEED_ORDER
end

-- attitude loop, measured points and servo-noise gain of gain set g; adds to res
function M.eval_outer(dz, g, req, scratch, res)
    res.pm_att, res.gm_att = M.attitude_margins(dz, g.P, g.I, g.D, g.FF, scratch.m, scratch.p)
    res.dist = M.measured_distance(dz, g.P, g.I, g.D)
    res.hf = M.hf_gain(dz, g.P, g.I, g.D)
    if req then
        if res.pm_att < req.pm or res.gm_att < req.gm then res.ok = false end
        if res.dist < 1 / K.MS_MAX then res.ok = false end
        if res.hf > req.hf_max then res.ok = false end
    end
end

-- whole check in one go (tests only: it can exceed the per-call instruction budget)
function M.evaluate(dz, g, req, scratch)
    local c = {g = g}
    while not M.eval_rate(dz, c, req, scratch) do end
    if c.res.ok or not req then M.eval_outer(dz, g, req, scratch, c.res) end
    return c.res
end

-- ---------- Step response in the model ----------
-- An attitude step of step_deg flown through ArduPilot's angle loop and rate
-- controller (target filter, error filter, P, I with IMAX and the +-45 clip,
-- filtered D, FF) on the model at speed index si, at the main loop rate.
-- Returns peak output, overshoot and the mean error against the ideal response.
function M.lp_alpha(dt, hz)
    if hz <= 0 then return 1 end
    return dt / (dt + 1 / (2 * math.pi * hz))
end

-- prog: {seconds, attitude in units of step_deg} segments, as K.VERIFY_PROGRAM
function M.stepsim_new(dz, g, si, step_deg, loop_rate, prog)
    local sp, x = dz.s[si], dz.x
    local r = sp.v / dz.v_test
    local ctl = dz.ctl
    local dt = 1 / clamp(loop_rate, 50, 400)
    local total = 0
    for _, seg in ipairs(prog) do total = total + seg[1] end
    local m = {g = g, sc = sp.sc, kff = g.FF * sp.sc / dz.e2t, axis = dz.axis, dt = dt, k = 0,
               n = floor(total / dt + 0.5), size = step_deg, prog = prog, seg = 0, seg_end = 0,
               step = 0, prev = 0, dir = 1, jump = 1, ctl = ctl,
               aT = M.lp_alpha(dt, ctl.fltt), aE = M.lp_alpha(dt, ctl.flte), aD = M.lp_alpha(dt, ctl.fltd),
               imax = ctl.imax * DEG, tf = 0, ef = 0, eprev = 0, df = 0, i = 0, out = 0,
               xb = 0, x1 = 0, x2 = 0, rate = 0, ang = 0, ideal = 0, peak = 0, os = 0, mae = 0,
               buf = {}, bi = 0}
    if dz.axis == 1 then
        m.K, m.a, m.b, m.tau = exp(x[1]) * r, exp(x[2]) * r, exp(x[3]), x[4]
    else
        m.K, m.z, m.wn, m.zeta, m.b, m.tau = exp(x[1]) * r, exp(x[2]) * r, exp(x[3]) * r, x[4], exp(x[5]), x[6]
    end
    m.nd = floor((m.tau + dz.loop_dt) / dt + 0.5)
    for i = 0, m.nd do m.buf[i] = 0 end
    -- exact discrete lag and roll pole (stable for any b and a)
    m.eb = 1 - exp(-m.b * dt)
    if m.a then m.ea = 1 - exp(-m.a * dt) end
    return m
end

-- runs up to K.SIM_STEPS_PER_TICK steps; returns true when the step is complete
-- (the loop body is written out in full to stay inside the per-call instruction budget)
function M.stepsim_run(m)
    local g, ctl, dt = m.g, m.ctl, m.dt
    local sc2 = m.sc * m.sc
    local P, I, D, kff = g.P, g.I, g.D, m.kff
    local tau = math.max(ctl.tc, 0.05)
    local up = ctl.rmax_up > 0 and ctl.rmax_up or math.huge
    local dn = ctl.rmax_dn > 0 and -ctl.rmax_dn or -math.huge
    local aT, aE, aD, imax, step = m.aT, m.aE, m.aD, m.imax, m.step
    local roll = m.axis == 1
    local last = math.min(m.k + K.SIM_STEPS_PER_TICK, m.n)
    for k = m.k + 1, last do
        if k > m.seg_end and m.seg < #m.prog then
            -- next segment: new target, step direction and size for the overshoot
            m.seg = m.seg + 1
            local seg = m.prog[m.seg]
            m.prev = step
            step = m.size * seg[2]
            m.step = step
            m.seg_end = m.seg_end + floor(seg[1] / dt + 0.5)
            m.dir = step >= m.prev and 1 or -1
            m.jump = abs(step - m.prev)
        end
        local rd = (step - m.ang) / tau
        if rd > up then rd = up elseif rd < dn then rd = dn end
        local tf = m.tf + aT * (rd - m.tf)
        m.tf = tf
        local ef = m.ef + aE * ((tf - m.rate) - m.ef)
        m.ef = ef
        m.df = m.df + aD * ((ef - m.eprev) / dt - m.df)
        m.eprev = ef
        -- AC_PID::update_i: with the output at its limit the integrator may only shrink
        local out = m.out
        local di = sc2 * I * ef * dt
        if I == 0 then
            m.i = 0                    -- AC_PID clears the integrator when I is 0
        elseif (out < 45 and out > -45) or di * m.i < 0 then
            local i = m.i + di
            if i > imax then i = imax elseif i < -imax then i = -imax end
            m.i = i
        end
        out = kff * tf + sc2 * (P * ef + D * m.df) + m.i
        if out > 45 then out = 45 elseif out < -45 then out = -45 end
        m.out = out
        if out > m.peak then m.peak = out elseif -out > m.peak then m.peak = -out end
        local ud = out
        if m.nd > 0 then
            -- ring of nd values: the slot read now was written nd steps ago
            local bi = (m.bi + 1) % m.nd
            m.bi = bi
            ud = m.buf[bi]
            m.buf[bi] = out
        end
        local xb = m.xb + m.eb * (ud - m.xb)
        m.xb = xb
        if roll then
            m.rate = m.rate + m.ea * (m.K * xb - m.rate)
        else
            local x1dd = m.wn * m.wn * (xb - m.x1) - 2 * m.zeta * m.wn * m.x2
            m.x2 = m.x2 + dt * x1dd
            m.x1 = m.x1 + dt * m.x2
            m.rate = m.K * (m.x1 + m.x2 / m.z)
        end
        local ang = m.ang + dt * m.rate
        m.ang = ang
        local ri = (step - m.ideal) / tau
        if ri > up then ri = up elseif ri < dn then ri = dn end
        m.ideal = m.ideal + dt * ri
        local e = ang - m.ideal
        m.mae = m.mae + (e > 0 and e or -e) * dt
        if m.jump > 0 then
            local os = (ang - step) * m.dir / m.jump
            if os > m.os then m.os = os end
        end
    end
    m.k = last
    if m.k >= m.n then
        m.mae = m.mae / (m.n * dt)
        return true
    end
    return false
end

-- ---------- Logging ----------
local function log_tick(ax, ph, dem, exc, rate, out, ang, ref, asp)
    logger:write("FTUN", "Ax,Ph,Dem,Exc,Rate,Out,Ang,Ref,As", "BBfffffff",
                 ax, ph, dem, exc, rate, out, ang, ref, asp)
end

-- ---------- State ----------
local state = "IDLE"      -- IDLE, WAIT, MAN, COMPUTE, STOPPED, DONE
local switch_seen_low = false
local chute_latched = false
local sess, man = nil, nil

local function read_ac()
    local names = {"AIRSPEED_MIN", "AIRSPEED_MAX", "SCALING_SPEED", "PTCH2SRV_RLL", "SCHED_LOOP_RATE",
                   "RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_THROTTLE"}
    local ac = {}
    for _, n in ipairs(names) do
        local v = param:get(n)
        if v == nil then return nil, n .. " missing" end
        ac[n] = v
    end
    return ac
end

local function read_ctl(ax)
    local c = {tc = param:get(ax.tc), rmax_up = param:get(ax.rmax_up), rmax_dn = param:get(ax.rmax_dn),
               fltt = param:get(ax.fltt), flte = param:get(ax.flte), fltd = param:get(ax.fltd),
               smax = param:get(ax.smax), imax = param:get(ax.imax)}
    for k, v in pairs(c) do
        if v == nil then return nil, k .. " missing" end
    end
    return c
end

local function read_gains(ax)
    local g = {}
    for _, k in ipairs(GAIN_KEYS) do
        g[k] = param:get(ax.gains[k])
        if g[k] == nil then return nil end
    end
    return g
end

local function same(a, b)
    return a ~= nil and abs(a - b) <= K.PARAM_TOL * math.max(abs(b), 1e-6)
end

-- writes the gains and reads them back; false if any write did not take
local function apply_gains(ax, g)
    local ok = true
    for _, k in ipairs(GAIN_KEYS) do
        if not param:set(ax.gains[k], g[k]) or not same(param:get(ax.gains[k]), g[k]) then ok = false end
    end
    return ok
end

local function save_gains(ax, g)
    local ok = true
    for _, k in ipairs(GAIN_KEYS) do
        if not param:set_and_save(ax.gains[k], g[k]) or not same(param:get(ax.gains[k]), g[k]) then ok = false end
    end
    return ok
end

local function release()
    vehicle:nav_scripting_enable(K.NO_MODE)
end

local function read_sensors()
    local gyro = ahrs:get_gyro()
    local asp = ahrs:airspeed_estimate()
    if not gyro or not asp then return nil end
    return {
        p = gyro:x() * DEG, q = gyro:y() * DEG,
        roll = ahrs:get_roll() * DEG, pitch = ahrs:get_pitch() * DEG,
        asp = asp, e2t = ahrs:get_EAS2TAS(),
        alt = -ahrs:get_relative_position_D_home(),
        ail = SRV_Channels:get_output_scaled(K.FN_AILERON) * 0.01,
        elev = SRV_Channels:get_output_scaled(K.FN_ELEVATOR) * 0.01,
        elv_l = SRV_Channels:get_output_scaled(K.FN_ELEVON_LEFT) * 0.01,
        elv_r = SRV_Channels:get_output_scaled(K.FN_ELEVON_RIGHT) * 0.01,
        thr = SRV_Channels:get_output_scaled(K.FN_THROTTLE),
        mode = vehicle:get_mode(),
    }
end

local function chute_switch_high()
    local ch = param:get("CHUTE_RC_CHAN")
    if ch == nil then return false end
    local pwm = rc:get_pwm(floor(ch))
    return pwm ~= nil and pwm > K.CHUTE_HIGH_PWM
end

local function switch_channel()
    for n = 1, 16 do
        if param:get(string.format("RC%d_OPTION", n)) == K.SWITCH_AUX then return n end
    end
    return nil
end

local function sticks_moved(limit)
    local r = sess.rc_roll:norm_input()
    local p = sess.rc_pitch:norm_input()
    local t = sess.rc_thr:norm_input()
    return abs(r) > limit or abs(p) > limit or abs(t - sess.thr_stick0) > 2 * limit
end

-- failsafe and envelope reasons that stop the session; nil when all is well
local function hard_reason(s)
    if rc:get_aux_cached(K.SWITCH_AUX) ~= K.SWITCH_HIGH then return "switch off" end
    if fparam("ENABLE") ~= 1 then return "FTUN_ENABLE 0" end
    if not rc:has_valid_input() then return "RC lost" end
    if chute_switch_high() then
        chute_latched = true
        return "parachute switch"
    end
    if not arming:is_armed() then return "disarmed" end
    if battery:has_failsafed() then return "battery failsafe" end
    -- Plane does not implement has_ekf_failsafed (always false); AHRS health is the check
    if not ahrs:healthy() then return "AHRS unhealthy" end
    if fence and fence:get_breaches() ~= 0 then return "fence breach" end
    return nil
end

local function frozen_changed()
    for n, v in pairs(sess.frozen) do
        if v ~= nil and not same(param:get(n), v) then return n end
    end
    return nil
end

-- takeover and envelope reasons that end the session in any state
local function session_reason(s)
    local reason = hard_reason(s)
    if reason then return reason end
    if s.mode ~= sess.mode then return "mode changed" end
    if sticks_moved(fparam("STICK")) then return "stick input" end
    return nil
end

local function axis_state() return sess.ax[sess.ai] end
local function cur_axis() return sess.axes[sess.ai] end

-- ---------- Session ----------
local function stop_session(reason)
    release()
    if sess then
        local ax = cur_axis()
        if ax and not axis_state().done then apply_gains(ax, sess.base[ax.id]) end
    end
    if reason then say(4, "stopped: %s", reason) end
    sess, man = nil, nil
    state = "STOPPED"
end

local begin_axis

local function next_axis()
    sess.ai = sess.ai + 1
    if sess.ai > #sess.axes then
        say(6, "done")
        sess, man = nil, nil
        state = "DONE"
        return
    end
    begin_axis()
end

begin_axis = function()
    local ax = cur_axis()
    -- the first doublet's half period at the test rate sets the attitude excursion
    local amp = math.min(fparam(ax.rate_param), K.TEST_ANGLE_MAX[ax.id] / K.ID_PROGRAM[1][1])
    sess.ax[sess.ai] = {phase = "ID", reps = 0, reps_target = clamp(floor(fparam("REPS")), 2, K.MAX_REPS),
                        sp = M.spectra_new(), amp = amp, soft = 0, v_sum = 0, e2t_sum = 0, n_sum = 0,
                        jit_sum = 0, jit_n = 0,
                        verify = {}, done = false}
    sess.last_release_ms = millis():toint() - K.GAP_BETWEEN_MS
    sess.wait_start_ms = millis():toint()
    state = "WAIT"
end

local function start_session(s)
    local ac, why = read_ac()
    if not ac then return say(4, "cannot start: %s", why) end
    if not K.ALLOWED_MODES[s.mode] then return say(4, "use LOITER, CRUISE or FBWB") end
    if param:get("MTUNE_BTMSK") ~= nil then return say(3, "remove tune_pitch_roll.lua first") end
    local sw = switch_channel()
    local chute = param:get("CHUTE_RC_CHAN")
    if sw and chute and floor(chute) == sw then return say(3, "switch is the parachute channel") end
    if rc:get_aux_cached(K.AUTOTUNE_AUX) == K.SWITCH_HIGH then return say(4, "AUTOTUNE switch is on") end
    if not vehicle:get_likely_flying() then return say(4, "not flying") end
    -- auto-trim moves SERVOn_TRIM, which changes the output-to-surface mapping mid-test
    if param:get("SERVO_AUTO_TRIM") == 1 then return say(4, "set SERVO_AUTO_TRIM 0") end
    local axes = {}
    local mask = floor(fparam("AXES") or 0)
    for _, ax in ipairs(AXES) do
        if mask & ax.bit ~= 0 then
            for _, n in ipairs(ax.unsupported) do
                local v = param:get(n)
                if v ~= nil and v ~= 0 then return say(4, "%s must be 0", n) end
            end
            axes[#axes + 1] = ax
        end
    end
    if #axes == 0 then return say(4, "FTUN_AXES selects no axis") end
    sess = {axes = axes, ai = 1, ax = {}, base = {}, ac = ac, ctl = {}}
    -- both axes' angle-loop settings: the hold loops fly both axes in every manoeuvre
    for _, ax in ipairs(AXES) do
        local c, cwhy = read_ctl(ax)
        if not c then
            sess = nil
            return say(4, "cannot read %s settings (%s)", ax.name, cwhy)
        end
        sess.ctl[ax.id] = c
    end
    for _, ax in ipairs(axes) do
        local g = read_gains(ax)
        if not g then
            sess = nil
            return say(4, "cannot read %s gains", ax.name)
        end
        sess.base[ax.id] = g
    end
    sess.rc_roll = rc:get_channel(floor(ac.RCMAP_ROLL))
    sess.rc_pitch = rc:get_channel(floor(ac.RCMAP_PITCH))
    sess.rc_thr = rc:get_channel(floor(ac.RCMAP_THROTTLE))
    if not (sess.rc_roll and sess.rc_pitch and sess.rc_thr) then
        sess = nil
        return say(4, "RC channels not available")
    end
    sess.thr_stick0 = sess.rc_thr:norm_input()
    sess.mode = s.mode
    -- everything the tests and the design depend on; a change stops the session
    sess.frozen = {}
    for _, ax in ipairs(AXES) do
        for _, n in ipairs({ax.tc, ax.rmax_up, ax.rmax_dn, ax.fltt, ax.flte, ax.fltd, ax.smax, ax.imax}) do
            sess.frozen[n] = param:get(n)
        end
        for _, k in ipairs(GAIN_KEYS) do sess.frozen[ax.gains[k]] = param:get(ax.gains[k]) end
    end
    for _, n in ipairs({"MIXING_GAIN", "SERVO1_TRIM", "SERVO2_TRIM", "SCALING_SPEED", "AIRSPEED_MIN", "AIRSPEED_MAX"}) do
        sess.frozen[n] = param:get(n)
    end
    say(6, "start %s", #axes == 2 and "roll and pitch" or axes[1].name)
    begin_axis()
end

-- ---------- Manoeuvres ----------
local function program_at(prog, t)
    local e = 0
    for i, seg in ipairs(prog) do
        e = e + seg[1]
        if t < e then return seg[2], i, t - (e - seg[1]) end
    end
    return nil
end

local function start_manoeuvre(s, now)
    local ax, st = cur_axis(), axis_state()
    local kind = st.phase == "ID" and "id" or "verify"
    man = {kind = kind, mode = s.mode, last_ms = now, t = 0, stage = "level", stage_t = 0,
           thr = s.thr, alt0 = s.alt, ref_roll = 0, ref_pitch = s.pitch, exc = 0, sat = 0}
    if kind == "id" then
        man.sign = (st.reps % 2 == 0) and 1 or -1
        man.dft = M.dft_new()
        man.p0, man.u0, man.n0 = 0, 0, 0
        man.v_sum, man.e2t_sum, man.n_sum = 0, 0, 0
        man.jit_sum, man.jit_n = 0, 0
    else
        local run = #st.verify + 1
        man.run = run
        man.set = (run % 2 == 1) and "A" or "B"
        man.sign = (run <= 2) and 1 or -1
        man.ideal, man.act_n, man.act_sum, man.u_prev = 0, 0, 0, nil
        man.mae = {[1] = 0, [-1] = 0}
        man.tsum = {[1] = 0, [-1] = 0}
        man.os = 0
        if man.set == "B" and not apply_gains(ax, st.cand) then
            apply_gains(ax, sess.base[ax.id])
            man = nil
            stop_session(ax.name .. " gain write failed")
            return
        end
    end
    -- demands are set before control is taken, so stale ones are never used
    vehicle:set_target_throttle_rate_rpy(s.thr, 0, 0, 0)
    if not vehicle:nav_scripting_enable(s.mode) then
        if kind == "verify" then apply_gains(ax, sess.base[ax.id]) end
        man = nil
        stop_session("control refused")
        return
    end
    state = "MAN"
    return true
end

local process_compute

local function finish_manoeuvre(ok, reason, hard)
    release()
    local ax, st = cur_axis(), axis_state()
    if man.kind == "verify" and man.set == "B" and not apply_gains(ax, sess.base[ax.id]) then
        man = nil
        return stop_session(ax.name .. " gain restore failed")
    end
    local m = man
    man = nil
    sess.last_release_ms = millis():toint()
    sess.wait_start_ms = sess.last_release_ms
    if hard then return stop_session(reason) end
    if not ok then
        st.soft = st.soft + 1
        if reason == "saturated" then st.amp = st.amp * K.SAT_AMP_FACTOR end
        say(4, "%s test dropped: %s", ax.name, reason)
        if st.soft >= K.MAX_SOFT_ABORTS then return stop_session("too many dropped tests") end
        state = "WAIT"
        return
    end
    if m.kind == "id" then
        M.spectra_add(st.sp, m.dft)
        st.reps = st.reps + 1
        st.v_sum, st.e2t_sum, st.n_sum = st.v_sum + m.v_sum, st.e2t_sum + m.e2t_sum, st.n_sum + m.n_sum
        st.jit_sum, st.jit_n = st.jit_sum + m.jit_sum, st.jit_n + m.jit_n
        if st.reps >= st.reps_target then
            state = "COMPUTE"
            st.job = {step = "estimate"}
        else
            say(6, "%s test %d/%d", ax.name, st.reps + 1, st.reps_target)
            state = "WAIT"
        end
        return
    end
    -- check manoeuvre
    local r = {set = m.set, run = m.run}
    local tt = m.tsum[1] + m.tsum[-1]
    r.mae = tt > 0 and (m.mae[1] + m.mae[-1]) / tt or 1e9
    r.mae_pos = m.tsum[1] > 0 and m.mae[1] / m.tsum[1] or 0
    r.mae_neg = m.tsum[-1] > 0 and m.mae[-1] / m.tsum[-1] or 0
    r.os = m.os
    r.act = m.act_n > 0 and sqrt(m.act_sum / m.act_n) or 0
    st.verify[#st.verify + 1] = r
    logger:write("FTVR", "Ax,Set,Run,MAE,MAEp,MAEn,OS,Act", "BBBfffff",
                 ax.id, m.set == "A" and 0 or 1, m.run, r.mae, r.mae_pos, r.mae_neg, r.os, r.act)
    if #st.verify >= 4 then
        state = "COMPUTE"
        st.job = {step = "decide"}
    else
        say(6, "%s check %d/4 (%s gains)", ax.name, #st.verify + 1, (#st.verify % 2 == 0) and "old" or "new")
        state = "WAIT"
    end
end

-- one 50 Hz step of a manoeuvre
local function manoeuvre_tick(s, now)
    local ax, st = cur_axis(), axis_state()
    local ac, ctl_r, ctl_p = sess.ac, sess.ctl[1], sess.ctl[2]
    local reason = hard_reason(s)
    if reason then return finish_manoeuvre(false, reason, true) end
    if s.mode ~= man.mode then return finish_manoeuvre(false, "mode changed", true) end
    if sticks_moved(fparam("STICK")) then return finish_manoeuvre(false, "stick input", true) end
    if s.alt < fparam("ALT_MIN") or s.alt < man.alt0 - fparam("ALT_LOSS") then
        return finish_manoeuvre(false, "height", true)
    end
    if s.asp < ac.AIRSPEED_MIN or s.asp > ac.AIRSPEED_MAX then return finish_manoeuvre(false, "airspeed", true) end
    if abs(s.roll) > K.ROLL_ENVELOPE or s.pitch > K.PITCH_ENVELOPE_UP or s.pitch < K.PITCH_ENVELOPE_DN then
        return finish_manoeuvre(false, "attitude limit", true)
    end
    if now - man.last_ms > K.MAX_TICK_GAP_MS then return finish_manoeuvre(false, "script delayed", false) end
    local dt = (now - man.last_ms) * 0.001
    man.last_ms = now

    -- controller output of the axis under test, and saturation
    local u = ax.id == 1 and s.ail or s.elev
    -- a saturated test record is not linear; check manoeuvres may saturate like normal flight
    if man.kind == "id" and (abs(s.ail) >= K.OUT_SAT or abs(s.elev) >= K.OUT_SAT
                             or abs(s.elv_l) >= K.OUT_SAT or abs(s.elv_r) >= K.OUT_SAT) then
        man.sat = man.sat + 1
        if man.sat >= K.SAT_TICKS then return finish_manoeuvre(false, "saturated", false) end
    else
        man.sat = 0
    end

    local rate_meas = ax.id == 1 and s.p or s.q
    local ang = ax.id == 1 and s.roll or s.pitch
    man.stage_t = man.stage_t + dt
    local exc, ph_code = 0, 1
    local rr, pr

    if man.stage == "level" then
        man.ref_roll, man.ref_pitch = 0, man.ref_pitch
        if abs(s.roll) < K.LEVEL_DEG and abs(s.p) < K.LEVEL_RATE_DPS then
            man.stage, man.stage_t = "settle", 0
            man.ref_pitch = s.pitch
        elseif man.stage_t > K.LEVEL_TIMEOUT_S then
            return finish_manoeuvre(false, "could not level", false)
        end
    elseif man.stage == "settle" then
        ph_code = 2
        if man.kind == "id" then
            man.p0, man.u0, man.n0 = man.p0 + rate_meas, man.u0 + u, man.n0 + 1
            -- servo jitter with the current gains: output change per tick in a steady hold
            if man.u_hold and dt > 0 then
                man.jit_sum, man.jit_n = man.jit_sum + ((u - man.u_hold) / dt) ^ 2, man.jit_n + 1
            end
            man.u_hold = u
        end
        if man.stage_t >= K.SETTLE_S then
            man.stage, man.stage_t = "run", 0
            man.base_ang = ax.id == 1 and 0 or man.ref_pitch
            man.ideal = ang
            if man.kind == "id" and man.n0 > 0 then man.p0, man.u0 = man.p0 / man.n0, man.u0 / man.n0 end
        end
    end

    if man.stage == "run" then
        local t = man.stage_t
        if man.kind == "id" then
            ph_code = 3
            -- the test signal held over the last interval moves the reference attitude
            if ax.id == 1 then man.ref_roll = man.ref_roll + man.exc * dt
            else man.ref_pitch = man.ref_pitch + man.exc * dt end
            M.dft_add(man.dft, t, rate_meas - man.p0, u - man.u0, man.exc)
            man.v_sum, man.e2t_sum, man.n_sum = man.v_sum + s.asp, man.e2t_sum + s.e2t, man.n_sum + 1
            local v = program_at(K.ID_PROGRAM, t)
            if v == nil then
                man.exc = 0
                return finish_manoeuvre(true)
            end
            exc = man.sign * st.amp * v
            man.exc = exc
        else
            ph_code = man.set == "A" and 4 or 5
            local v, seg, tin = program_at(K.VERIFY_PROGRAM, t)
            if v == nil then return finish_manoeuvre(true) end
            local step = ax.step_deg * man.sign
            local target = man.base_ang + step * v
            local prev_v = seg > 1 and K.VERIFY_PROGRAM[seg - 1][2] or 0
            local dir = (v - prev_v) * man.sign >= 0 and 1 or -1
            if ax.id == 1 then man.ref_roll = target else man.ref_pitch = target end
            -- ideal attitude response: the angle loop with a perfect rate loop
            local c = sess.ctl[ax.id]
            local h = dt / K.VERIFY_SUBSTEPS
            for _ = 1, K.VERIFY_SUBSTEPS do
                man.ideal = man.ideal + h * angle_rate(target - man.ideal, c.tc, c.rmax_up, c.rmax_dn)
            end
            local e = ang - man.ideal
            man.mae[dir] = man.mae[dir] + abs(e) * dt
            man.tsum[dir] = man.tsum[dir] + dt
            local size = abs(step * (v - prev_v))
            if size > 0 then
                local os = (ang - target) * dir / size
                if os > man.os then man.os = os end
            end
            if tin > K.VERIFY_PROGRAM[seg][1] - K.ACT_WINDOW_S and man.u_prev and dt > 0 then
                man.act_sum = man.act_sum + ((u - man.u_prev) / dt) ^ 2
                man.act_n = man.act_n + 1
            end
            man.u_prev = u
        end
    end

    -- demands: hold loops on both axes plus the test signal on the axis under test
    local level_limit = man.stage == "level" and K.LEVEL_RATE_MAX or nil
    rr = angle_rate(man.ref_roll - s.roll, ctl_r.tc, ctl_r.rmax_up, ctl_r.rmax_dn)
    if level_limit then rr = clamp(rr, -level_limit, level_limit) end
    pr = angle_rate(man.ref_pitch - s.pitch, ctl_p.tc, ctl_p.rmax_up, ctl_p.rmax_dn,
                    coord_rate(s.roll, s.pitch, s.asp, s.e2t, ac))
    if ax.id == 1 then rr = rr + exc else pr = pr + exc end
    vehicle:set_target_throttle_rate_rpy(man.thr, rr, pr, 0)
    log_tick(ax.id, ph_code, ax.id == 1 and rr or pr, exc, rate_meas, u, ang,
             ax.id == 1 and man.ref_roll or man.ref_pitch, s.asp)
end

-- ---------- Compute jobs (a bounded piece per call) ----------
local function fmt_g(g)
    return string.format("FF %.3f P %.3f I %.3f D %.4f", g.FF, g.P, g.I, g.D)
end

local JOBS = {}      -- compute steps, one bounded piece of work per call

function JOBS.estimate(ax, st, job)
    local est = M.estimate(st.sp)
    st.est = est
    logger:write("FTSP", "Ax,N,Spr,Valid", "BBfB", ax.id, st.reps, est.spread, est.nvalid)
    if est.spread > K.SPREAD_MAX and st.reps_target < K.MAX_REPS then
        st.reps_target = math.min(st.reps_target + K.REPS_EXTRA, K.MAX_REPS)
        say(6, "%s response scatter %.2f: %d more tests", ax.name, est.spread, st.reps_target - st.reps)
        state = "WAIT"
        return
    end
    if est.spread > K.SPREAD_MAX then return stop_session(string.format("%s: air too rough (%.2f)", ax.name, est.spread)) end
    if est.nvalid < 6 then return stop_session(ax.name .. ": too few usable frequencies") end
    job.starts = M.fit_starts(ax.id, est)
    job.costs, job.si = {}, 0
    job.step = "starts"
end

function JOBS.starts(ax, st, job)
    local r = {}
    for _ = 1, 3 do
        job.si = job.si + 1
        if job.si > #job.starts then
            local order = {}
            for i = 1, #job.starts do order[i] = i end
            table.sort(order, function(a, b) return job.costs[a] < job.costs[b] end)
            job.queue = {}
            for i = 1, math.min(K.LM_STARTS, #order) do job.queue[i] = job.starts[order[i]] end
            job.qi, job.best = 0, nil
            job.step = "lm"
            return
        end
        local n = M.residuals(ax.id, job.starts[job.si], st.est, r)
        job.costs[job.si] = M.sumsq(r, n)
    end
end

function JOBS.lm(ax, st, job)
    if not job.lm then
        job.qi = job.qi + 1
        if job.qi > #job.queue then
            job.step = "fitted"
            return
        end
        job.lm = M.lm_new(ax.id, job.queue[job.qi], st.est)
    end
    if M.lm_step(job.lm) then
        if job.lm.cost and (not job.best or job.lm.cost < job.best.cost) then job.best = job.lm end
        job.lm = nil
    end
end

function JOBS.fitted(ax, st, job)
    local x = job.best and job.best.x
    if not x then return stop_session(ax.name .. ": model fit failed") end
    if ax.id == 1 and x[2] > x[3] then x[2], x[3] = x[3], x[2] end     -- slower pole scales with speed
    local rms = M.fit_rms(ax.id, x, st.est)
    st.model, st.fit_rms = x, rms
    for i = 1, NW do
        local lm, ph = M.model_logresp(ax.id, x, ID_W[i], 1)
        logger:write("FTFR", "Ax,W,GR,GI,Sd,Wt,ML,MP", "Bfffffff", ax.id, ID_W[i], st.est.gr[i], st.est.gi[i],
                     st.est.sd[i], st.est.w[i], lm, ph)
    end
    local v = st.n_sum > 0 and st.v_sum / st.n_sum or sess.ac.AIRSPEED_MIN
    st.v_test = clamp(v, sess.ac.AIRSPEED_MIN, sess.ac.AIRSPEED_MAX)
    st.e2t = st.n_sum > 0 and st.e2t_sum / st.n_sum or 1
    if ax.id == 1 then
        logger:write("FTMD", "Ax,K,P1,P2,P3,P4,P5,Rms,V", "Bffffffff", ax.id, exp(x[1]), exp(x[2]), exp(x[3]), x[4], 0, 0, rms, st.v_test)
    else
        logger:write("FTMD", "Ax,K,P1,P2,P3,P4,P5,Rms,V", "Bffffffff", ax.id, exp(x[1]), exp(x[2]), exp(x[3]), x[4], exp(x[5]), x[6], rms, st.v_test)
    end
    say(6, "%s model fit %.2f, delay %.0f ms", ax.name, rms, x[ax.id == 1 and 4 or 6] * 1000)
    if rms > K.FIT_RMS_MAX then return stop_session(string.format("%s: model does not fit (%.2f)", ax.name, rms)) end
    job.step = "design_setup"
end

function JOBS.design_setup(ax, st, job)
    local base = sess.base[ax.id]
    -- FF from the steady-state gain: FF scaler / EAS2TAS = 1 / K
    local sc = speed_scaler(st.v_test, sess.ac)
    local ff = st.e2t / (sc * exp(st.model[1]))
    if ff > K.FF_CHANGE_MAX * base.FF or ff < base.FF / K.FF_CHANGE_MAX then
        return stop_session(string.format("%s FF %.2f vs %.2f: suspect", ax.name, ff, base.FF))
    end
    job.ff = ff
    job.scratch = {m = {}, p = {}}
    st.dz = M.design_new(ax.id, st.model, sess.ac, st.v_test, st.e2t, sess.ctl[ax.id], st.est)
    job.step = "design_build"
end

function JOBS.design_build(ax, st, job)
    if M.design_build(st.dz) then
        job.base_c = {g = sess.base[ax.id]}
        job.step = "baseline"
    end
end

function JOBS.baseline(ax, st, job)
    local c = job.base_c
    if not c.res or c.part < #K.EVAL_SPEED_ORDER then
        M.eval_rate(st.dz, c, nil, job.scratch)
        return
    end
    M.eval_outer(st.dz, c.g, nil, job.scratch, c.res)
    local br = c.res
    job.base_res = br
    say(6, "%s now PM %.0f GM %.1f wc %.1f", ax.name, br.pm_min, br.gm_min, br.wc_min)
    -- servo jitter scales with the controller gain at high frequency; keep its slew
    -- below K.SLEW_FRACTION of SMAX
    local ctl = sess.ctl[ax.id]
    local slew_ref = (ctl.smax and ctl.smax > 0) and ctl.smax or K.SLEW_REF_DEFAULT
    local jit_limit = K.SLEW_FRACTION * slew_ref
    local jit = st.jit_n > 0 and sqrt(st.jit_sum / st.jit_n) or 0
    st.jitter = jit
    local hf_max = math.huge
    if jit > 0 then hf_max = br.hf * jit_limit / jit end
    if jit > jit_limit then say(4, "%s servo jitter %.2f above %.2f", ax.name, jit, jit_limit) end
    job.req = {pm = fparam("PM"), gm = fparam("GM"), hf_max = hf_max}
    -- P that alone gives |L| = 1 at 10 rad/s at the test speed
    local glm = M.model_logresp(ax.id, st.model, 10, 1)
    job.p_ref = 1 / (st.dz.s[2].sc ^ 2 * exp(glm))
    job.pk, job.tk = K.P_GRID_LO, 1
    job.best, job.cand = nil, nil
    job.base_sim = M.stepsim_new(st.dz, sess.base[ax.id], 2, ax.step_deg, sess.ac.SCHED_LOOP_RATE, K.SEARCH_PROGRAM)
    job.step = "base_sim"
end

function JOBS.base_sim(ax, st, job)
    if not M.stepsim_run(job.base_sim) then return end
    job.peak_max = math.max(K.OUT_USE_MAX, job.base_sim.peak)
    job.step = "search"
end

-- ranking: lower predicted step error, then the faster rate loop within K.MAE_TIE
local function better(mae, wc, best)
    if not best then return true end
    if mae < best.mae * (1 - K.MAE_TIE) then return true end
    return mae < best.mae * (1 + K.MAE_TIE) and wc > best.res.wc_min
end

-- Search over P and D/P; for each pair that passes the rate-loop requirements, FF
-- fractions from K.FF_GRID are checked with the attitude loop and a simulated step.
function JOBS.search(ax, st, job)
    local dz, scratch = st.dz, job.scratch
    local c = job.cand
    if not c then
        if job.pk > K.P_GRID_HI then
            job.step = "designed"
            return
        end
        local P = job.p_ref * 2 ^ (job.pk / 4)
        local D = P * K.TD_GRID[job.tk]
        job.tk = job.tk + 1
        if job.tk > #K.TD_GRID then job.tk, job.pk = 1, job.pk + 1 end
        -- crossover with P and D only sets the integrator corner
        local _, _, wc = M.rate_margins(dz, 2, P, 0, D, scratch.m, scratch.p)
        if wc then job.cand = {g = {FF = job.ff, P = P, I = P * wc / K.I_RATIO, D = D}, fi = 0} end
        return
    end
    if not c.rate_done then
        if not M.eval_rate(dz, c, job.req, scratch) then return end
        c.rate_done = true
        if not c.res.ok then job.cand = nil end
        return
    end
    if c.sim then
        if not M.stepsim_run(c.sim) then return end
        local m = c.sim
        c.sim = nil
        if m.peak <= job.peak_max and m.os <= K.OVERSHOOT_MAX and better(m.mae, c.trial_res.wc_min, job.best) then
            job.best = {g = c.trial, res = c.trial_res, mae = m.mae}
        end
        return
    end
    c.fi = c.fi + 1
    if c.fi > #K.FF_GRID then
        job.cand = nil
        return
    end
    local g = {P = c.g.P, I = c.g.I, D = c.g.D, FF = job.ff * K.FF_GRID[c.fi]}
    local res = {}
    for k, v in pairs(c.res) do res[k] = v end
    M.eval_outer(dz, g, job.req, scratch, res)
    if res.ok then
        c.trial, c.trial_res = g, res
        c.sim = M.stepsim_new(dz, g, 2, ax.step_deg, sess.ac.SCHED_LOOP_RATE, K.SEARCH_PROGRAM)
    end
end

function JOBS.designed(ax, st, job)
    local base = sess.base[ax.id]
    local br = job.base_res
    if not job.base_logged then
        job.base_logged = true
        logger:write("FTGN", "Ax,Set,FF,P,I,D,PMr,GMr,Wc,PMa", "BBffffffff", ax.id, 0, base.FF, base.P, base.I, base.D,
                     br.pm_min, br.gm_min, br.wc_min, br.pm_att or 0)
    end
    if not job.best then
        st.done = true
        say(4, "%s: no gains meet PM %.0f GM %.0f, kept old", ax.name, job.req.pm, job.req.gm)
        return next_axis()
    end
    -- predicted check-step error at the test speed, old then new gains
    if not job.pred then
        job.pred = {M.stepsim_new(st.dz, base, 2, ax.step_deg, sess.ac.SCHED_LOOP_RATE, K.VERIFY_PROGRAM),
                    M.stepsim_new(st.dz, job.best.g, 2, ax.step_deg, sess.ac.SCHED_LOOP_RATE, K.VERIFY_PROGRAM)}
        job.pi = 1
        return
    end
    if job.pi <= 2 then
        if M.stepsim_run(job.pred[job.pi]) then job.pi = job.pi + 1 end
        return
    end
    local pa, pb = job.pred[1], job.pred[2]
    st.pred = {mae_a = pa.mae, mae_b = pb.mae, os_a = pa.os, os_b = pb.os, peak_a = pa.peak, peak_b = pb.peak}
    logger:write("FTPR", "Ax,MAEa,MAEb,OSa,OSb,PKa,PKb", "Bffffff", ax.id, pa.mae, pb.mae, pa.os, pb.os, pa.peak, pb.peak)
    local g, res = job.best.g, job.best.res
    logger:write("FTGN", "Ax,Set,FF,P,I,D,PMr,GMr,Wc,PMa", "BBffffffff", ax.id, 1, g.FF, g.P, g.I, g.D,
                 res.pm_min, res.gm_min, res.wc_min, res.pm_att)
    say(6, "%s new PM %.0f GM %.1f wc %.1f", ax.name, res.pm_min, res.gm_min, res.wc_min)
    say(6, "%s model step error %.2f -> %.2f deg", ax.name, pa.mae, pb.mae)
    say(6, "%s %s", ax.name, fmt_g(g))
    local same = true
    for _, k in ipairs(GAIN_KEYS) do
        local b = base[k]
        if abs(g[k] - b) > K.NO_CHANGE_FRAC * math.max(abs(b), 1e-6) then same = false end
    end
    if same then
        st.done = true
        say(6, "%s gains already right, kept", ax.name)
        return next_axis()
    end
    st.cand = g
    st.phase = "VERIFY"
    say(6, "%s check 1/4 (old gains)", ax.name)
    state = "WAIT"
end

local function mean2(a, b) return 0.5 * (a + b) end

function JOBS.decide(ax, st, job)
    local v = st.verify
    local A = {v[1], v[3]}
    local B = {v[2], v[4]}
    local mA, mB = mean2(A[1].mae, A[2].mae), mean2(B[1].mae, B[2].mae)
    local mAp, mBp = mean2(A[1].mae_pos, A[2].mae_pos), mean2(B[1].mae_pos, B[2].mae_pos)
    local mAn, mBn = mean2(A[1].mae_neg, A[2].mae_neg), mean2(B[1].mae_neg, B[2].mae_neg)
    local spread = math.max(abs(A[1].mae - A[2].mae), abs(B[1].mae - B[2].mae))
    local osA, osB = math.max(A[1].os, A[2].os), math.max(B[1].os, B[2].os)
    local actA, actB = mean2(A[1].act, A[2].act), mean2(B[1].act, B[2].act)
    local why
    if not (mB <= (1 - fparam("IMPROVE")) * mA) then why = "no gain"
    elseif not (mA - mB > spread) then why = "within scatter"
    elseif mBp > mAp or mBn > mAn then why = "one side worse"
    elseif osB > math.max(K.OVERSHOOT_MAX, osA) then why = "overshoot"
    elseif actB > K.ACTIVITY_GROWTH * actA + K.ACTIVITY_FLOOR then why = "servo activity"
    end
    local change = mA > 0 and 100 * (mB - mA) / mA or 0
    if osB > K.OVERSHOOT_ABS_MAX and not why then why = "overshoot" end
    logger:write("FTDC", "Ax,MAEa,MAEb,Spr,OSa,OSb,ACTa,ACTb,Keep", "BfffffffB", ax.id, mA, mB, spread, osA, osB, actA, actB,
                 why and 0 or 1)
    if why then
        if not apply_gains(ax, sess.base[ax.id]) then return stop_session(ax.name .. " gain write failed") end
        st.done = true
        say(5, "%s kept old: %s (%+.0f%%)", ax.name, why, change)
    else
        local ok
        if fparam("SAVE") == 1 then ok = save_gains(ax, st.cand) else ok = apply_gains(ax, st.cand) end
        if not ok then return stop_session(ax.name .. " gain write failed") end
        st.done = true
        for _, k in ipairs(GAIN_KEYS) do sess.frozen[ax.gains[k]] = st.cand[k] end
        if fparam("SAVE") == 1 then
            say(5, "%s new gains saved: error %+.0f%%", ax.name, change)
        else
            say(5, "%s new gains set, not saved: error %+.0f%%", ax.name, change)
        end
    end
    next_axis()
end

process_compute = function()
    local ax, st = cur_axis(), axis_state()
    local job = st.job
    JOBS[job.step](ax, st, job)
end

-- ---------- Main loop ----------
local function idle_tick()
    local sw = rc:get_aux_cached(K.SWITCH_AUX)
    if sw ~= K.SWITCH_HIGH then
        switch_seen_low = true
        return K.IDLE_TICK_MS
    end
    if not switch_seen_low or fparam("ENABLE") ~= 1 or chute_latched then return K.IDLE_TICK_MS end
    switch_seen_low = false      -- one start per switch movement
    local s = read_sensors()
    if not s then
        say(4, "cannot start: no airspeed or gyro")
        return K.IDLE_TICK_MS
    end
    if not arming:is_armed() then
        say(4, "cannot start: disarmed")
        return K.IDLE_TICK_MS
    end
    if chute_switch_high() then
        chute_latched = true
        say(4, "parachute switch high: disabled")
        return K.IDLE_TICK_MS
    end
    start_session(s)
    return K.WAIT_TICK_MS
end

local function wait_tick(now)
    local s = read_sensors()
    if not s then return stop_session("no airspeed or gyro") end
    local reason = session_reason(s)
    if reason then return stop_session(reason) end
    local changed = frozen_changed()
    if changed then return stop_session(changed .. " changed") end
    local ac = sess.ac
    local margin = fparam("SPD_MARGIN")
    local ready = s.alt >= fparam("ALT_MIN") + K.START_ALT_MARGIN
        and s.asp >= ac.AIRSPEED_MIN + margin and s.asp <= ac.AIRSPEED_MAX - margin
        and abs(s.pitch) < K.START_PITCH_MAX
        and now - sess.last_release_ms >= K.GAP_BETWEEN_MS
    if ready then
        if start_manoeuvre(s, now) then manoeuvre_tick(s, now) end
        return
    end
    if now - sess.wait_start_ms > K.WAIT_TIMEOUT_MS then
        stop_session("no safe conditions for 60 s")
    end
end

local function update()
    local now = millis():toint()
    if state == "IDLE" then
        return idle_tick()
    end
    if state == "STOPPED" or state == "DONE" then
        if rc:get_aux_cached(K.SWITCH_AUX) ~= K.SWITCH_HIGH then
            switch_seen_low = true
            state = "IDLE"
        end
        return K.IDLE_TICK_MS
    end
    if state == "WAIT" then
        wait_tick(now)
    elseif state == "MAN" then
        local s = read_sensors()
        if not s then
            finish_manoeuvre(false, "no airspeed or gyro", true)
        else
            manoeuvre_tick(s, now)
        end
    elseif state == "COMPUTE" then
        local s = read_sensors()
        local reason = "no airspeed or gyro"
        if s then reason = session_reason(s) end
        if not reason and now - (sess.frozen_ms or 0) >= K.FROZEN_CHECK_MS then
            sess.frozen_ms = now
            local changed = frozen_changed()
            if changed then reason = changed .. " changed" end
        end
        if reason then
            stop_session(reason)
        else
            process_compute()
        end
    end
    if state == "MAN" then return K.MAN_TICK_MS end
    if state == "COMPUTE" then return K.COMPUTE_TICK_MS end
    if state == "WAIT" then return K.WAIT_TICK_MS end
    return K.IDLE_TICK_MS
end

local function protected_update()
    local ok, result = pcall(update)
    if ok then return protected_update, result end
    -- internal error: give control back and restore the axis under test, then stop the script
    release()
    if sess and sess.axes and sess.ai and sess.axes[sess.ai] and sess.ax[sess.ai]
       and not sess.ax[sess.ai].done then
        apply_gains(sess.axes[sess.ai], sess.base[sess.axes[sess.ai].id])
    end
    gcs:send_text(0, "FTUN: internal error, script stopped: " .. tostring(result))
end

-- test harness access to the calculation functions; nil on the aircraft
if FTUN_TEST_EXPORT then
    FTUN_TEST_EXPORT.model_logresp = M.model_logresp
    FTUN_TEST_EXPORT.estimate = M.estimate
    FTUN_TEST_EXPORT.spectra_new = M.spectra_new
    FTUN_TEST_EXPORT.spectra_add = M.spectra_add
    FTUN_TEST_EXPORT.dft_new = M.dft_new
    FTUN_TEST_EXPORT.dft_add = M.dft_add
    FTUN_TEST_EXPORT.margins = M.margins
    FTUN_TEST_EXPORT.lm_new = M.lm_new
    FTUN_TEST_EXPORT.lm_step = M.lm_step
    FTUN_TEST_EXPORT.fit_starts = M.fit_starts
    FTUN_TEST_EXPORT.residuals = M.residuals
    FTUN_TEST_EXPORT.sumsq = M.sumsq
    FTUN_TEST_EXPORT.fit_rms = M.fit_rms
    FTUN_TEST_EXPORT.design_new = M.design_new
    FTUN_TEST_EXPORT.design_build = M.design_build
    FTUN_TEST_EXPORT.evaluate = M.evaluate
    FTUN_TEST_EXPORT.K = K
    FTUN_TEST_EXPORT.stepsim_new = M.stepsim_new
    FTUN_TEST_EXPORT.stepsim_run = M.stepsim_run
    FTUN_TEST_EXPORT.speed_scaler = speed_scaler
    FTUN_TEST_EXPORT.ID_W, FTUN_TEST_EXPORT.DES_W = ID_W, DES_W
    FTUN_TEST_EXPORT.state = function() return state end
end

say(6, "loaded (switch RCx_OPTION %d)", K.SWITCH_AUX)
return protected_update, K.IDLE_TICK_MS
