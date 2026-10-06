// ArduPilot parameter classification for the "Compare parameters" feature.
//
// WHEN PUSHING A SHARED .param FILE TO MULTIPLE AIRFRAMES, some parameters must
// NOT be copied: they are unique to each physical board (sensor calibration and
// auto-detected device identities). Copying UAV-A's compass offsets onto UAV-B
// corrupts UAV-B's heading solution. This module names those parameters so the
// compare/apply UI can exclude them by default.
//
// Two anchored-regex arrays, matched against UPPERCASE parameter names:
//   IDENTITY_CAL_SKIP  — per-board identity / calibration → SKIP by default.
//   BUS_PORT_KEEP      — wiring / port / protocol config → KEEP (push like any
//                        normal tune value). Also a regression guard: a unit test
//                        asserts no KEEP sample matches a SKIP pattern, which
//                        catches over-broad SKIP edits (see the named collisions
//                        below).
//
// EDITING THIS LIST:
//   - Patterns are anchored (^…$) on purpose. Unanchored patterns over-match
//     (e.g. a bare COMPASS_O* would wrongly swallow COMPASS_ORIENT).
//   - ArduPilot puts the instance digit in two places: the 2nd compass is
//     COMPASS2_OFS_X (digit on the prefix) while some fields also carry a
//     trailing digit (COMPASS_DEV_ID2). Patterns use \d? in BOTH positions so
//     instance 1 (no digit) and instances 2..N all match.
//   - If you add a SKIP pattern, run tests/gcs/test_param_skip_list_js.py; the
//     non-overlap guard will fail if it accidentally catches a KEEP param.
//
// Verified against ArduPlane-4.5 (AP_InertialSensor, AP_Compass, AP_AHRS,
// AP_GPS, AP_Baro, AP_SerialManager). Tested via tests/gcs/test_param_skip_list_js.py.

// ── Category A — identity / calibration (SKIP by default) ──────────────────
export const IDENTITY_CAL_SKIP = [
  // A1. Inertial sensor calibration (per-board).
  /^INS_ACC\d?OFFS_[XYZ]$/,            // accel offsets (INS_ACCOFFS_*, INS_ACC2OFFS_*…)
  /^INS_ACC\d?SCAL_[XYZ]$/,            // accel scale factors
  /^INS_GYR\d?OFFS_[XYZ]$/,            // gyro offsets (recomputed per boot, per board)
  /^INS_ACC\d?_CALTEMP$/,             // accel cal temperature
  /^INS_GYR\d?_CALTEMP$/,             // gyro cal temperature
  /^INS_TCAL\d?_(ENABLE|TMIN|TMAX|ACC\d_[XYZ]|GYR\d_[XYZ])$/, // thermal-cal tree

  // A2. Inertial sensor device IDs (auto-detected per board). Anchored to
  // _(ACC|GYR)\d?_ID$ so INS_USE*, INS_POS*, INS_*_FILTER survive (KEEP).
  /^INS_(ACC|GYR)\d?_ID$/,

  // A3. Compass calibration (per-board hard/soft iron).
  /^COMPASS\d?_OFS\d?_[XYZ]$/,         // hard-iron offsets
  /^COMPASS\d?_DIA\d?_[XYZ]$/,         // soft-iron diagonal
  /^COMPASS\d?_ODI\d?_[XYZ]$/,         // soft-iron off-diagonal
  /^COMPASS\d?_MOT\d?_[XYZ]$/,         // motor-interference compensation
  /^COMPASS\d?_SCALE\d?$/,             // per-sensor scale-error factor

  // A4. Compass device IDs (auto-detected per board).
  /^COMPASS_DEV_ID\d?$/,               // detected device IDs (DEV_ID..DEV_ID8)
  /^COMPASS_PRIO\d_ID$/,               // priority ordering by detected device ID

  // A5. Barometer device IDs + per-power-on ground calibration.
  /^BARO\d?_DEVID$/,                   // detected baro device IDs
  /^BARO\d?_GND_PRESS$/,               // calibrated ground pressure (per power-on)
  /^BARO_(GND_TEMP|ALT_OFFSET)$/,      // per-session ground temp / alt offset

  // A6. AHRS board-mount trim (per-airframe level cal). NOTE: only TRIM — every
  // other AHRS_* param is shared config and stays in KEEP.
  /^AHRS_TRIM_[XYZ]$/,

  // A7. GPS / CAN auto-detected node identity (read-only per device).
  /^GPS_CAN_NODEID\d$/,                // detected DroneCAN node id of the GPS
  /^CAN_D\d_UC_NODE$/,                 // this autopilot's own DroneCAN node id

  // A8. System statistics / vehicle identity. STAT_ is enumerated (not a bare
  // prefix) so a future STAT_* config param is not silently skipped.
  /^STAT_(BOOTCNT|FLTTIME|RUNTIME|RESET)$/, // cumulative per-board counters / reset
  /^SYSID_THISMAV$/,                   // this vehicle's MAVLink system id (network address)
  /^FORMAT_VERSION$/,                  // storage-format version, owned by firmware
  /^BRD_SERIAL_NUM$/,                  // per-board serial tag

  // A9. Airspeed sensor calibration + identity (per-sensor; ArduPlane). Copying
  // another aircraft's offset/ratio corrupts airspeed; ARSPD_TYPE/BUS/etc. are
  // wiring/role config and stay in KEEP.
  /^ARSPD\d?_OFFSET$/,                 // zero-airspeed pressure offset (per-sensor cal)
  /^ARSPD\d?_RATIO$/,                  // dynamic-pressure ratio (per-sensor cal)
  /^ARSPD\d?_DEVID$/,                  // detected airspeed device id

  // A10. Battery monitor calibration + identity (per power module). Narrow on
  // purpose — BATT_CAPACITY / BATT_MONITOR / failsafe thresholds are shared
  // config and must NOT be caught here.
  /^BATT\d?_VOLT_MULT$/,               // voltage-divider multiplier
  /^BATT\d?_AMP_PERVLT$/,              // current-sensor amps-per-volt
  /^BATT\d?_AMP_OFFSET$/,              // current-sensor zero offset
  /^BATT\d?_SERIAL_NUM$/,              // smart-battery serial identity
];

// ── Category B — bus / port / protocol config (KEEP — do NOT skip) ─────────
// These look hardware-specific but describe how the autopilot is WIRED and what
// each port/bus speaks; they are identical across identical airframes and are
// the whole point of a shared config file. Listed both for runtime keep-list
// precedence and as the regression guard against SKIP over-match.
export const BUS_PORT_KEEP = [
  /^SERIAL\d+_(PROTOCOL|BAUD|OPTIONS)$/,        // serial port role/baud/options
  /^SERIAL_PASS(1|2|TIMO)$/,                    // serial passthrough routing
  /^CAN_P\d_(DRIVER|BITRATE|FDBITRATE)$/,       // CAN physical bus driver/bitrate
  /^CAN_D\d_PROTOCOL$/,                         // CAN logical-driver protocol
  /^GPS\d?_?TYPE\d?$/,                          // GPS driver/role (TYPE, not device id)
  /^GPS_(GNSS_MODE2?|RATE_MS2?|AUTO_SWITCH|AUTO_CONFIG|PRIMARY|DRV_OPTIONS|SBAS_MODE|NAVFILTER|MIN_ELEV|MIN_DGPS|BLEND_MASK|INJECT_TO|SAVE_CFG|COM_PORT2?)$/,
  /^GPS\d?_CAN_OVRIDE$/,                        // operator-set node-id override (config, not detection)
  /^BARO_(EXT_BUS|PROBE_EXT|PRIMARY|OPTIONS|FLTR_RNG|ALTERR_MAX)$/,
  /^COMPASS\d?_(EXTERNAL|EXTERN\d)$/,           // is-external / which cable (wiring)
  /^COMPASS\d?_ORIENT\d?$/,                     // mounting orientation (build geometry)
  /^COMPASS\d?_USE\d?$/,                        // use this compass for heading (role)
  /^COMPASS_(DEC|AUTODEC|LEARN|MOTCT|ENABLE|CAL_FIT|OFFS_MAX|DISBLMSK|FLTR_RNG|AUTO_ROT|OPTIONS|TYPEMASK)$/,
  /^INS_POS\d?_[XYZ]$/,                         // IMU mount offset (geometry, not cal)
  /^GPS_POS\d_[XYZ]$/,                          // GPS antenna mount offset (geometry)
  /^AHRS_(EKF_TYPE|ORIENTATION|GPS_USE|GPS_GAIN|GPS_MINSATS|YAW_P|RP_P|WIND_MAX|COMP_BETA|OPTIONS)$/,
  /^ARSPD\d?_(TYPE|BUS|PIN|USE|PRIMARY|OPTIONS)$/, // airspeed role/wiring config (vs A9 cal)
];

/**
 * True when `name` matches an IDENTITY_CAL_SKIP pattern. Ignores keep-list
 * precedence — used by the non-overlap regression test to assert KEEP samples
 * never hit a SKIP pattern. Application code should use isUavSpecificParam.
 */
export function matchesIdentitySkip(name) {
  if (typeof name !== 'string') return false;
  const up = name.toUpperCase();
  return IDENTITY_CAL_SKIP.some((re) => re.test(up));
}

/**
 * True when `name` matches a BUS_PORT_KEEP pattern.
 */
export function matchesBusPortKeep(name) {
  if (typeof name !== 'string') return false;
  const up = name.toUpperCase();
  return BUS_PORT_KEEP.some((re) => re.test(up));
}

/**
 * Whether a parameter is per-board identity/calibration that should be SKIPPED
 * by default when pushing a shared file. Keep-list wins: a name that somehow
 * matches both lists is treated as config (kept), so future broadening of a
 * SKIP pattern can't silently start protecting a wiring/protocol param.
 */
export function isUavSpecificParam(name) {
  if (matchesBusPortKeep(name)) return false;
  return matchesIdentitySkip(name);
}
