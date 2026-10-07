local PARAM_TABLE_KEY = 88

assert(param:add_table(PARAM_TABLE_KEY, "AAS_", 30), "Failed to add AAS parameter table")
assert(param:add_param(PARAM_TABLE_KEY, 1,  "DEL_PITCH", 0), "Failed to add DEL_PITCH parameter")
assert(param:add_param(PARAM_TABLE_KEY, 2,  "DEL_THR", -1), "Failed to add DEL_THR parameter")
assert(param:add_param(PARAM_TABLE_KEY, 3,  "DEL_DIR", 0), "Failed to add DEL_DIR parameter")
assert(param:add_param(PARAM_TABLE_KEY, 4,  "DEL_P_KP", 1.5), "Failed to add DEL_P_KP parameter")
assert(param:add_param(PARAM_TABLE_KEY, 5,  "DEL_PLD", -1), "Failed to add DEL_PLD parameter")
assert(param:add_param(PARAM_TABLE_KEY, 6,  "DEL_PLRD", -1), "Failed to add DEL_PLRD parameter")
assert(param:add_param(PARAM_TABLE_KEY, 7,  "USE_TRN", 1), "Failed to add USE_TRN parameter")

assert(param:add_param(PARAM_TABLE_KEY, 8,  "TARG_WPS", 16), "Failed to add TARG_WPS parameter")
assert(param:add_param(PARAM_TABLE_KEY, 9,  "TARG_ALT", 150), "Failed to add TARG_ALT parameter")

assert(param:add_param(PARAM_TABLE_KEY, 10, "NAV_LAST_WP", 3), "Failed to add NAV_LAST_WP parameter")
assert(param:add_param(PARAM_TABLE_KEY, 11, "NAV_MIN_ALT", 150), "Failed to add NAV_MIN_ALT parameter")
assert(param:add_param(PARAM_TABLE_KEY, 12, "NAV_CWT", 30), "Failed to add NAV_CWT parameter")
assert(param:add_param(PARAM_TABLE_KEY, 13, "NAV_AUTO_CM", 1), "Failed to add NAV_AUTO_CM parameter")
assert(param:add_param(PARAM_TABLE_KEY, 14, "NAV_CM_FL", 0), "Failed to add NAV_CM_FL parameter")  -- MISS-04: 0=reject unapproved POI on confirm-window expiry, 1=confirm
assert(param:add_param(PARAM_TABLE_KEY, 18, "NAV_ONESHOT", 0), "Failed to add NAV_ONESHOT parameter")  -- 0=repeat tasks, 1=stop after one completed navigation run; disarm only in simulation

assert(param:add_param(PARAM_TABLE_KEY, 15, "LOG_DEFER", 0), "Failed to add LOG_DEFER parameter")
assert(param:add_param(PARAM_TABLE_KEY, 16, "LOG_RATE", 2), "Failed to add LOG_RATE parameter")

assert(param:add_param(PARAM_TABLE_KEY, 17, "DEL_CTRL", 1), "Failed to add DEL_CTRL parameter")  -- 0=PID, 1=PN, 2=vision-nav-pn

assert(param:add_param(PARAM_TABLE_KEY, 19, "NAV_CGT", 15), "Failed to add NAV_CGT parameter")  -- operator gate timeout (sec)
