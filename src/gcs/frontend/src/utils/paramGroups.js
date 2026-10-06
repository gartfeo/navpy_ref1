// Functional priority groups for the "Compare parameters" view. Instead of
// hiding non-actionable rows, the compare sidebar buckets diffs into a handful of
// operator-meaningful groups shown in review-priority order, so you can focus on
// the tuning that matters and leave per-board calibration for last (nothing is
// hidden — every group is one click away, with a live diff count).
//
// Taxonomy sourced from the ArduPilot-expert consult (ArduPlane 4.x), plus a
// NavPy-specific "Navigation (AAS_*)" group promoted to the top. The two lowest
// groups REUSE the existing skip-list so there is one source of truth for the
// per-board vs wiring boundary (and its non-overlap regression guard):
//   per-board-cal-identity  == matchesIdentitySkip  (IDENTITY_CAL_SKIP)
//   sensors-wiring-ports     == matchesBusPortKeep   (BUS_PORT_KEEP)
//
// Precedence (deterministic): per-board identity/cal is a HARD override checked
// FIRST so calibration can never be mis-sorted into tuning; then first-match down
// the priority order; wiring/ports sits low (after airframe); anything unmatched
// falls to 'other'. Tested via tests/gcs/test_param_groups_js.py.

import { matchesIdentitySkip, matchesBusPortKeep } from './paramSkipList';

// NavPy navigation params — mission-critical, reviewed first. AAS_* is the navigation
// core; MTUNE_* is the in-flight manual-tuning support for navigation.
const NAVIGATION = [/^AAS_/, /^MTUNE_/];

const CONTROL_TUNING = [
  /^RLL_RATE_(P|I|D|IMAX|FF|FLTT|FLTE|FLTD|SMAX)$/,
  /^PTCH_RATE_(P|I|D|IMAX|FF|FLTT|FLTE|FLTD|SMAX)$/,
  /^YAW_RATE_(P|I|D|IMAX|FF|FLTT|FLTE|FLTD|SMAX)$/,
  /^RLL2SRV_(P|I|D|IMAX|FF|TCONST|RMAX)$/,
  /^PTCH2SRV_(P|I|D|IMAX|FF|TCONST|RMAX_UP|RMAX_DN)$/,
  /^YAW2SRV_(SLIP|INT|DAMP|RLL|IMAX)$/,
  /^TECS_[A-Z0-9_]+$/,
  /^NAVL1_(PERIOD|DAMPING|XTRACK_I|LIM_BANK)$/,
  /^L1_[A-Z0-9_]+$/,
  /^ACRO_(ROLL_RATE|PITCH_RATE|YAW_RATE|LOCKING)$/,
  /^AUTOTUNE_LEVEL$/, /^AT_[A-Z0-9_]+$/, /^QUIK_[A-Z0-9_]+$/,
  /^GROUND_STEER_(ALT|DPS)$/,
  /^STEER2SRV_(P|I|D|IMAX|FF|TCONST|RMAX|MINSPD|DRTSPD|DRTFCT)$/,
  /^KFF_(RDDMIX|RDDRMIX|THR2PTCH|GNDSTEER)$/, /^RUDD_DT_GAIN$/,
  /^TRIM_(THROTTLE|ARSPD_CM)$/, /^ARSPD_FBW_(MIN|MAX)$/,
  /^SCALING_SPEED$/, /^TUNE_[A-Z0-9_]+$/,
  // Speed envelope (ArduPlane 4.4+ names) — tuning targets, not wiring.
  /^AIRSPEED_(MIN|MAX|CRUISE)$/,
  // Attitude limits are tune targets on this airframe (both old + 4.4 names).
  /^PTCH_LIM_(MAX|MIN)_DEG$/, /^ROLL_LIMIT_DEG$/,
  /^LIM_(ROLL_CD|PITCH_MAX|PITCH_MIN)$/,
  // Landing pitch, RC-override timeout, and SBUS servo rate tuned per airframe.
  /^LAND_PITCH_(DEG|CD)$/, /^RC_OVERRIDE_TIME$/, /^SERVO_SBUS_RATE$/,
];

const QUADPLANE = [
  /^Q_A_(RAT_(RLL|PIT|YAW)_(P|I|D|IMAX|FF|FLTT|FLTE|FLTD|SMAX)|ANG_(RLL|PIT|YAW)_P|ANGLE_BOOST|ACCEL_[RPY]_MAX)$/,
  /^Q_P_(POSXY_P|POSZ_P|VELXY_[A-Z_]+|VELZ_[A-Z_]+|ACCZ_(P|I|D|IMAX|FLTT|FLTE|FLTD|SMAX))$/,
  /^Q_M_[A-Z0-9_]+$/,
  /^Q_(WVANE|LOIT|WP|ANGLE_MAX|TRANS_[A-Z_]+|ASSIST_[A-Z_]+|TILT_[A-Z_]+|OPTIONS|TYPE|ENABLE|FRAME_[A-Z]+|VFWD_[A-Z]+|RTL_[A-Z]+|LAND_[A-Z]+|TKOFF_[A-Z]+|THR_[A-Z]+)[A-Z0-9_]*$/,
];

const NAV_MISSION = [
  /^WP_(RADIUS|LOITER_RAD|MAX_RADIUS)$/,
  /^MIS_(TOTAL|RESTART|OPTIONS)$/,
  /^RTL_(RADIUS|ALTITUDE|AUTOLAND|CLIMB_MIN|ALT_[A-Z]*)$/,
  /^ALT_HOLD_RTL$/, /^NAV_[A-Z0-9_]+$/, /^LOITER_RAD$/,
  /^CRUISE_(ALT_FLOOR|HEIGHT)$/,
  /^TKOFF_(ALT|LVL_ALT|LVL_PITCH|DIST|GND_PITCH|ROTATE_SPD|THR_MINACC|THR_DELAY|TDRAG_[A-Z]+|OPTIONS|FLAP_[A-Z]+)$/,
  /^LAND_(FLARE_ALT|FLARE_SEC|PF_[A-Z]+|SLOPE_[A-Z]+|DISARMDELAY|FLAP_[A-Z]+|THEN_NEUTRL|ABORT_[A-Z]+|OPTIONS|TYPE)$/,
  /^RNGFND_LANDING$/, /^RALLY_(TOTAL|LIMIT_KM|INCL_HOME)$/,
  /^FENCE_(ENABLE|TYPE|ACTION|ALT_MAX|ALT_MIN|RADIUS|MARGIN|TOTAL|RET_RALLY|RET_ALT|AUTOENABLE|OPTIONS)$/,
  /^STALL_PREVENTION$/, /^LEVEL_ROLL_LIMIT$/,
];

const FAILSAFE = [
  /^FS_[A-Z0-9_]+$/, /^THR_FAILSAFE$/, /^THR_FS_VALUE$/,
  /^BATT\d?_(LOW_VOLT|LOW_MAH|CRT_VOLT|CRT_MAH|FS_LOW_ACT|FS_CRT_ACT|FS_VOLTSRC|LOW_TIMER|MONITOR|CAPACITY|WATT_MAX)$/,
  /^ARMING_[A-Z0-9_]+$/, /^CRASH_(DETECT|ACC_THRESH)$/, /^AFS_[A-Z0-9_]+$/,
  /^RUDDER_ONLY$/,
];

const MODES_RC = [
  /^FLTMODE\d$/, /^FLTMODE_CH$/, /^INITIAL_MODE$/, /^MODE\d$/,
  /^RC\d+_(MIN|MAX|TRIM|REVERSED|DZ|OPTION)$/,
  /^RCMAP_(ROLL|PITCH|THROTTLE|YAW|FORWARD|LATERAL)$/, /^RC_OPTIONS$/,
  /^SERVO\d+_(FUNCTION|MIN|MAX|TRIM|REVERSED)$/,
  /^SERVO_(AUTO_TRIM|RATE|DSHOT_[A-Z]+|BLH_[A-Z0-9_]+|RC_FS_MSK|GPIO_MASK|32_[A-Z]+)$/,
  /^MIXING_(GAIN|OFFSET)$/, /^ELEVON_[A-Z]+$/, /^VTAIL_OUTPUT$/,
  /^FLAP_(1_PERCNT|1_SPEED|2_PERCNT|2_SPEED|IN_CHANNEL|SLEWRATE|AUTO_[A-Z]+)$/,
  /^THROTTLE_NUDGE$/, /^FLAPERON_OUTPUT$/,
];

const AIRFRAME = [
  // Battery ADC pin wiring (the cal multipliers stay per-board; the pins are config).
  /^BATT\d?_(VOLT_PIN|CURR_PIN)$/,
  /^BRD_(?!SERIAL_NUM$)[A-Z0-9_]+$/, /^LOG_[A-Z0-9_]+$/, /^NTF_[A-Z0-9_]+$/,
  /^SCR_[A-Z0-9_]+$/, /^SR\d+_[A-Z0-9_]+$/, /^MAV_[A-Z0-9_]+$/, /^FRAME_(CLASS|TYPE)$/,
  /^THR_(MIN|MAX|SLEWRATE|SUPP_MAN|PASS_STAB|WATT_MAX)$/,
  /^SCHED_(LOOP_RATE|OPTIONS|DEBUG)$/,
  /^INS_(GYRO_FILTER|ACCEL_FILTER|USE\d?|FAST_SAMPLE|GYRO_RATE|ENABLE_MASK|NOTCH[_A-Z0-9]*|HNTCH[_A-Z0-9]*|LOG_[A-Z]+)$/,
  /^EK3_[A-Z0-9_]+$/, /^EK2_[A-Z0-9_]+$/, /^GPS_(DELAY_MS\d?|POS_[A-Z]+)$/,
  /^RSSI_[A-Z0-9_]+$/, /^RPM\d?_[A-Z0-9_]+$/, /^RELAY\d?_[A-Z0-9_]+$/,
  /^CAM_[A-Z0-9_]+$/, /^MNT\d?_[A-Z0-9_]+$/, /^ADSB_[A-Z0-9_]+$/,
  /^AVD_[A-Z0-9_]+$/, /^OA_[A-Z0-9_]+$/, /^RNGFND\d*_(?!LANDING$)[A-Z0-9_]+$/,
  /^EFI_[A-Z0-9_]+$/, /^ICE_[A-Z0-9_]+$/,
];

// First-match, priority order (identity + wiring handled specially in classifyParam).
const ORDERED = [
  ['navigation', NAVIGATION],
  ['control-tuning', CONTROL_TUNING],
  ['quadplane', QUADPLANE],
  ['navigation-mission', NAV_MISSION],
  ['failsafe-limits', FAILSAFE],
  ['flight-modes-rc', MODES_RC],
  ['airframe-system', AIRFRAME],
];

// Display / review-priority order for the sidebar. 'file-only' is a status bucket
// (a param absent on the vehicle) appended last by the summarizer, not a functional
// group, so it's not in classifyParam's output.
export const PARAM_GROUP_ORDER = [
  'navigation', 'control-tuning', 'quadplane', 'navigation-mission', 'failsafe-limits',
  'flight-modes-rc', 'airframe-system', 'sensors-wiring-ports', 'per-board-cal-identity', 'other',
];

// Short labels — the compare sidebar is narrow, so keep these from truncating.
export const PARAM_GROUP_LABELS = {
  'navigation': 'Navigation (AAS)',
  'control-tuning': 'Control & Tuning',
  'quadplane': 'Quadplane / VTOL',
  'navigation-mission': 'Nav & Mission',
  'failsafe-limits': 'Failsafe & Limits',
  'flight-modes-rc': 'Flight Modes & RC',
  'airframe-system': 'Airframe & System',
  'sensors-wiring-ports': 'Sensors & Wiring',
  'per-board-cal-identity': 'Per-board Cal',
  'other': 'Other',
  'file-only': 'File-only',
  'read-only': 'Read-only',
};

// Low-priority groups the UI mutes / sinks to the bottom (per-board, file-only,
// read-only view-only bucket).
export const LOW_PRIORITY_GROUPS = new Set([
  'sensors-wiring-ports', 'per-board-cal-identity', 'file-only', 'read-only',
]);

/**
 * Functional group for a parameter name. Per-board identity/cal is a hard override
 * (checked before the priority walk) so calibration never lands in tuning; wiring/
 * ports is a low-priority ordinary group; unmatched names fall to 'other'.
 */
export function classifyParam(name) {
  if (typeof name !== 'string') return 'other';
  const up = name.toUpperCase();
  if (matchesIdentitySkip(up)) return 'per-board-cal-identity';   // hard override
  for (const [id, pats] of ORDERED) {
    if (pats.some((re) => re.test(up))) return id;
  }
  if (matchesBusPortKeep(up)) return 'sensors-wiring-ports';       // low-priority group
  return 'other';
}

/** The sidebar bucket for a compare row: read-only (view-only firmware params) and
 * file-only rows bucket by status, the rest by functional group. Read-only wins so
 * a firmware param never lands in a functional group or the "All" view. */
export function groupBucketForRow(row) {
  if (!row) return 'other';
  if (row.readOnly) return 'read-only';
  if (row.status === 'file-only') return 'file-only';
  return classifyParam(row.name);
}

/**
 * Summarise compare rows into the ordered, non-empty sidebar groups with counts.
 * `count` is the number of DIFFERING rows in the group (what the operator acts on);
 * file-only rows are counted under 'file-only'; read-only (view-only firmware)
 * rows under 'read-only'. Neither counts toward totalDiffer (the "All" total),
 * since neither is writable. Returns { order: [id...], counts: {id: n}, totalDiffer }
 * with the two status buckets appended last ('file-only', then 'read-only').
 */
export function summarizeCompareGroups(rows) {
  const counts = {};
  let totalDiffer = 0;
  for (const row of (Array.isArray(rows) ? rows : [])) {
    if (!row) continue;
    if (row.readOnly) {
      counts['read-only'] = (counts['read-only'] || 0) + 1;
    } else if (row.status === 'differ') {
      const id = classifyParam(row.name);
      counts[id] = (counts[id] || 0) + 1;
      totalDiffer += 1;
    } else if (row.status === 'file-only') {
      counts['file-only'] = (counts['file-only'] || 0) + 1;
    }
  }
  const order = PARAM_GROUP_ORDER.filter((id) => counts[id]);
  if (counts['file-only']) order.push('file-only');
  if (counts['read-only']) order.push('read-only');
  return { order, counts, totalDiffer };
}
