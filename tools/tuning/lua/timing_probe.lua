--[[
timing_probe.lua - measures how regularly ArduPilot runs a Lua script at 50 Hz on
this flight controller, with the same per-call work as flight_tune.lua's test loop.

It moves nothing and changes no parameter. Load it next to the aircraft's other
scripts, power the board (bench, propeller off), and read the messages: every 10 s
it sends "FTIM: n <calls> med <ms> p90 <ms> p99 <ms> max <ms> >40 <count>" and logs
an FTIM record. Remove it after the test.
--]]

local PERIOD_MS = 20          -- flight_tune.lua manoeuvre tick
local REPORT_MS = 10000
local NW = 15                 -- frequencies summed per call in flight_tune.lua
local LATE_MS = 40

local gaps = {}
local last_ms, report_ms = nil, nil
local acc = {}
for i = 1, NW do acc[i] = 0 end

-- about the arithmetic of one flight_tune.lua test tick
local function work(t)
    for i = 1, NW do
        local w = 1.5 * i
        local c, s = math.cos(w * t), math.sin(w * t)
        acc[i] = acc[i] + c * 0.3 + s * 0.2 - c * s * 0.1
    end
end

local function quantile(sorted, q)
    return sorted[math.max(1, math.floor(q * #sorted + 0.5))]
end

local function update()
    local now = millis():tofloat()
    if last_ms then gaps[#gaps + 1] = now - last_ms end
    last_ms = now
    report_ms = report_ms or now
    work(now * 0.001)
    if now - report_ms >= REPORT_MS and #gaps > 0 then
        table.sort(gaps)
        local late = 0
        for _, g in ipairs(gaps) do
            if g > LATE_MS then late = late + 1 end
        end
        local med, p90, p99, mx = quantile(gaps, 0.5), quantile(gaps, 0.9), quantile(gaps, 0.99), gaps[#gaps]
        gcs:send_text(6, string.format("FTIM: n %d med %.0f p90 %.0f p99 %.0f max %.0f >40 %d",
                                       #gaps, med, p90, p99, mx, late))
        logger:write("FTIM", "N,Med,P90,P99,Max,Late", "Hffffi", #gaps, med, p90, p99, mx, late)
        gaps = {}
        report_ms = now
    end
    return update, PERIOD_MS
end

gcs:send_text(6, "FTIM: timing probe running")
return update, PERIOD_MS
