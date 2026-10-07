"""AAS parameter metadata.

Single source of truth for the mapping between the JSON field names exposed
by the GCS API (`/api/vehicles/{sys_id}/params`) and the MAVLink AAS_*
parameter names stored on the autopilot, plus the type-coercion sets used
when crossing that boundary.

AAS_TARG_* / targ_* identify simulator POIs; AAS_DEL_* /
del_* describe final-approach controls.

Vehicle-side AAS parameters are owned by the autopilot, not by GCS settings,
so this module deliberately lives outside `settings_model`.
"""
from __future__ import annotations


AAS_PARAM_MAP: dict[str, str] = {
    "del_pitch": "AAS_DEL_PITCH",
    "del_thr": "AAS_DEL_THR",
    "del_dir": "AAS_DEL_DIR",
    "del_p_kp": "AAS_DEL_P_KP",
    "del_pld": "AAS_DEL_PLD",
    "del_plrd": "AAS_DEL_PLRD",
    "del_ctrl": "AAS_DEL_CTRL",
    "use_trn": "AAS_USE_TRN",
    "targ_wps": "AAS_TARG_WPS",
    "targ_alt": "AAS_TARG_ALT",
    "nav_last_wp": "AAS_NAV_LAST_WP",
    "nav_min_alt": "AAS_NAV_MIN_ALT",
    "nav_cwt": "AAS_NAV_CWT",
    "nav_cgt": "AAS_NAV_CGT",
    "nav_auto_cm": "AAS_NAV_AUTO_CM",
    "nav_cm_fl": "AAS_NAV_CM_FL",
    "nav_oneshot": "AAS_NAV_ONESHOT",
    "log_defer": "AAS_LOG_DEFER",
    "log_rate": "AAS_LOG_RATE",
}

# Fields exposed as bool on the API but stored as 0.0/1.0 float in MAVLink.
AAS_BOOL_FIELDS: set[str] = {
    "del_dir",
    "use_trn",
    "nav_auto_cm",
    "nav_cm_fl",
    "nav_oneshot",
    "log_defer",
}

# Fields exposed as int on the API but stored as float in MAVLink.
AAS_INT_FIELDS: set[str] = {
    "del_ctrl",
    "targ_wps",
    "nav_last_wp",
    "nav_cgt",
}
