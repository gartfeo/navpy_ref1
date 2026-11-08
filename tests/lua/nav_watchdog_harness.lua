-- Mocks the ArduPilot Lua bindings nav_watchdog.lua uses, so it runs under plain Lua 5.3. Every call runs under
-- a count hook that raises "Exceeded CPU time" past VM_I_COUNT instructions, as AP_Scripting does. The hook fires
-- every instruction: a coarser count hook here overcounts calls into C functions about tenfold.

local M = {}

local VM_I_COUNT = 10000            -- SCR_VM_I_COUNT default
local HOOK_STEP = 1

local function fresh_state()
    return {
        params = {
            SYSID_THISMAV = 1,
            AAS_NWD_ENABLE = 1, AAS_NWD_TMO = 1.5, AAS_NWD_RTL = 30, AAS_NWD_COMP = 191,
        },
        time_ms = 0,
        mode = 15,                  -- GUIDED
        armed = true,
        mode_refused = {},          -- { [mode] = true }: set_mode(mode) returns false
        get_mode_error = nil,       -- a string: vehicle:get_mode() raises it
        queue = {},
        init_args = nil,
        registered = {},
        table_added = nil,
        added_params = {},
    }
end

M.state = fresh_state()
M.out = { msgs = {}, floats = {}, modes = {}, max_instr = 0, calls = 0 }

local function unhex(s)
    return (s:gsub("..", function(c) return string.char(tonumber(c, 16)) end))
end

local function install()
    _G.param = {
        get = function(_, name) return M.state.params[name] end,
        add_table = function(_, key, prefix, n) M.state.table_added = { key, prefix, n }; return true end,
        add_param = function(_, key, index, name, default)
            M.state.added_params[#M.state.added_params + 1] = { key = key, index = index, name = name, default = default }
            return true
        end,
    }
    _G.vehicle = {
        get_mode = function()
            if M.state.get_mode_error then error(M.state.get_mode_error) end
            return M.state.mode
        end,
        set_mode = function(_, mode)
            local ok = not M.state.mode_refused[mode]
            M.out.modes[#M.out.modes + 1] = { mode = mode, ok = ok, t = M.state.time_ms }
            if ok then M.state.mode = mode end
            return ok
        end,
    }
    _G.arming = { is_armed = function() return M.state.armed end }
    _G.millis = function()
        local t = M.state.time_ms
        return { toint = function() return t end }
    end
    _G.gcs = {
        send_text = function(_, sev, msg)
            M.out.msgs[#M.out.msgs + 1] = { sev = sev, msg = msg, t = M.state.time_ms }
        end,
        send_named_float = function(_, name, value)
            M.out.floats[#M.out.floats + 1] = { name = name, value = value, t = M.state.time_ms }
        end,
    }
    _G.mavlink = {
        init = function(_, queue, ids) M.state.init_args = { queue, ids } end,
        register_rx_msgid = function(_, id) M.state.registered[#M.state.registered + 1] = id; return true end,
        receive_chan = function()
            local msg = table.remove(M.state.queue, 1)
            if msg == nil then return end
            return msg, 0, M.state.time_ms
        end,
    }
end

function M.load(path)
    install()
    local fn, delay = dofile(path)
    M.update, M.tick_ms = fn, delay
end

-- A mavlink_message_t (hex), as the test builds it.
-- Dropped when the queue is full, as ArduPilot's ObjectBuffer of mavlink:init's queue length drops it.
function M.push(hex)
    local limit = M.state.init_args and M.state.init_args[1] or 0
    if #M.state.queue < limit then
        M.state.queue[#M.state.queue + 1] = unhex(hex)
    end
end

local function call()
    local count = 0
    debug.sethook(function()
        count = count + 1
        if count * HOOK_STEP > VM_I_COUNT then error("Exceeded CPU time", 0) end
    end, "", HOOK_STEP)
    local ok, fn, delay = pcall(M.update)
    debug.sethook()
    M.out.calls = M.out.calls + 1
    if count * HOOK_STEP > M.out.max_instr then M.out.max_instr = count * HOOK_STEP end
    if not ok then
        M.out.script_error = tostring(fn)
        M.update = nil
        return
    end
    M.update, M.tick_ms = fn, delay
end

-- Advance ``ms``, one script call per tick; ``hb_hex`` (optional) is pushed every ``hb_every_ms``.
function M.run(ms, hb_hex, hb_every_ms)
    local stop = M.state.time_ms + ms
    while M.state.time_ms < stop and M.update do
        M.state.time_ms = M.state.time_ms + M.tick_ms
        if hb_hex and M.state.time_ms % hb_every_ms == 0 then M.push(hb_hex) end
        call()
    end
end

local function encode(v)
    local t = type(v)
    if t == "string" then
        return '"' .. v:gsub('\\', '\\\\'):gsub('"', '\\"'):gsub('\n', '\\n') .. '"'
    elseif t == "number" or t == "boolean" then
        return tostring(v)
    elseif t == "nil" then
        return "null"
    end
    local parts, array = {}, #v > 0 or next(v) == nil
    if array then
        for i = 1, #v do parts[i] = encode(v[i]) end
        return "[" .. table.concat(parts, ",") .. "]"
    end
    for k, val in pairs(v) do parts[#parts + 1] = '"' .. tostring(k) .. '":' .. encode(val) end
    return "{" .. table.concat(parts, ",") .. "}"
end

function M.dump()
    M.out.state = {
        time_ms = M.state.time_ms, mode = M.state.mode, init_args = M.state.init_args,
        registered = M.state.registered, table_added = M.state.table_added, added_params = M.state.added_params,
    }
    io.write(encode(M.out))
end

return M
