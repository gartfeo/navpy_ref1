// Curated failsafe-parameter config + pure helpers for the Failsafe tab.
// No React, no fetch — tested via tests/gcs/test_failsafe_params_js.py
// (Node subprocess), same style as fullParams.js.
//
// The Failsafe tab is a filtered/curated VIEW over the full per-vehicle
// parameter snapshot (see useFullParams.js). It never owns transport; it
// reads the cached snapshot, edits via setVehicleDraft, and writes via the
// shared full-param write path scoped to a group's param names.
//
// Enum integer -> action-name maps are extracted verbatim from ArduPilot's
// machine-readable parameter metadata (apm.pdef.xml). They are FIRMWARE-
// SPECIFIC where ArduPlane and ArduCopter diverge. The battery-action enums
// are the dangerous case: on Copter 1=Land / 2=RTL, but on Plane 1=RTL /
// 2=Land. A single shared table would mislabel the action. Throttle/RC and
// GCS failsafe use different PARAM NAMES per firmware, so presence-filtering
// already selects the right firmware's params for those.

// integer -> English label pairs. The i18n key for each option is derived
// as `settings.failsafe.actions.<ENUM>.<value>` so the component can show a
// translated label with the English text below as the fallback.
const ACTION_LABELS = {
  // FS_THR_ENABLE — Copter throttle/RC failsafe enable+action.
  FS_THR_ENABLE: [
    [0, 'Disabled'],
    [1, 'Always RTL'],
    [2, 'Continue mission in Auto (removed 4.0+)'],
    [3, 'Always Land'],
    [4, 'SmartRTL or RTL'],
    [5, 'SmartRTL or Land'],
    [6, 'Auto DO_LAND_START or RTL'],
    [7, 'Always Brake or Land'],
  ],
  // THR_FAILSAFE — Plane throttle+RC failsafe enable (no action selector;
  // the action is governed by FS_SHORT_ACTN / FS_LONG_ACTN).
  THR_FAILSAFE: [
    [0, 'Disabled'],
    [1, 'Enabled'],
    [2, 'Enabled, no failsafe action'],
  ],
  // FS_SHORT_ACTN — Plane short-failsafe action.
  FS_SHORT_ACTN: [
    [0, 'Circle / no change'],
    [1, 'Circle'],
    [2, 'FBWA, zero throttle'],
    [3, 'Disabled'],
    [4, 'FBWB'],
  ],
  // FS_LONG_ACTN — Plane long-failsafe action.
  FS_LONG_ACTN: [
    [0, 'Continue'],
    [1, 'Return to Launch (RTL)'],
    [2, 'Glide'],
    [3, 'Deploy parachute'],
    [4, 'Auto'],
    [5, 'AUTOLAND'],
  ],
  // BATT_FS_LOW_ACT / BATT_FS_CRT_ACT — Copter battery action.
  BATT_ACTION_COPTER: [
    [0, 'Warn only'],
    [1, 'Land'],
    [2, 'RTL'],
    [3, 'SmartRTL or RTL'],
    [4, 'SmartRTL or Land'],
    [5, 'Terminate'],
    [6, 'Auto DO_LAND_START or RTL'],
    [7, 'Brake or Land'],
  ],
  // BATT_FS_LOW_ACT / BATT_FS_CRT_ACT — Plane battery action. NOTE the
  // 1=RTL / 2=Land swap versus Copter.
  BATT_ACTION_PLANE: [
    [0, 'Warn only'],
    [1, 'RTL'],
    [2, 'Land'],
    [3, 'Terminate'],
    [4, 'QLand'],
    [5, 'Parachute'],
    [6, 'Loiter to QLand'],
    [7, 'AUTOLAND or RTL'],
  ],
  // FS_GCS_ENABLE — Copter GCS-loss failsafe action.
  FS_GCS_ENABLE: [
    [0, 'Disabled'],
    [1, 'RTL'],
    [2, 'RTL or Continue mission in Auto (removed 4.0+)'],
    [3, 'SmartRTL or RTL'],
    [4, 'SmartRTL or Land'],
    [5, 'Land'],
    [6, 'Auto DO_LAND_START or RTL'],
    [7, 'Brake or Land'],
  ],
  // FS_GCS_ENABL — Plane GCS failsafe TRIGGER condition (not an action; the
  // action is FS_LONG_ACTN). Spelled without the trailing E.
  FS_GCS_ENABL: [
    [0, 'Disabled'],
    [1, 'Heartbeat'],
    [2, 'Heartbeat + REM RSSI'],
    [3, 'Heartbeat + AUTO'],
  ],
};

// Firmware-exclusive parameter names used to tell Plane from Copter. We
// infer firmware from the snapshot rather than trusting a vehicle-type field
// because the snapshot is the authoritative thing we actually have.
const PLANE_MARKERS = [
  'THR_FAILSAFE', 'FS_LONG_ACTN', 'FS_SHORT_ACTN', 'FS_GCS_ENABL',
  'FS_LONG_TIMEOUT', 'THR_FS_VALUE',
];
const COPTER_MARKERS = ['FS_THR_ENABLE', 'FS_GCS_ENABLE', 'FS_THR_VALUE'];

/**
 * Curated failsafe parameter groups. A param is only shown if it is actually
 * present on the connected vehicle's snapshot (presence-filtered), so the
 * union of Plane + Copter names below renders only what the vehicle has.
 *
 * `firmwareEnum: true` means the action labels depend on detected firmware
 * (the battery-action swap). Plain `enum` names a fixed ACTION_LABELS table.
 */
export const FAILSAFE_GROUPS = [
  {
    id: 'rcThrottle',
    titleKey: 'settings.failsafe.groups.rcThrottle',
    title: 'RC / Throttle',
    params: [
      { name: 'FS_THR_ENABLE', type: 'select', enum: 'FS_THR_ENABLE', firmware: 'copter', labelKey: 'settings.failsafe.fields.fsThrEnable', label: 'Throttle / RC failsafe' },
      { name: 'THR_FAILSAFE', type: 'select', enum: 'THR_FAILSAFE', firmware: 'plane', labelKey: 'settings.failsafe.fields.thrFailsafe', label: 'Throttle / RC failsafe' },
      { name: 'FS_THR_VALUE', type: 'number', firmware: 'copter', labelKey: 'settings.failsafe.fields.fsThrValue', label: 'Throttle FS threshold (PWM)', min: 850, max: 2200, step: 1 },
      { name: 'THR_FS_VALUE', type: 'number', firmware: 'plane', labelKey: 'settings.failsafe.fields.thrFsValue', label: 'Throttle FS threshold (PWM)', min: 850, max: 2200, step: 1 },
      { name: 'FS_SHORT_ACTN', type: 'select', enum: 'FS_SHORT_ACTN', firmware: 'plane', labelKey: 'settings.failsafe.fields.fsShortActn', label: 'Short failsafe action' },
      { name: 'FS_LONG_ACTN', type: 'select', enum: 'FS_LONG_ACTN', firmware: 'plane', labelKey: 'settings.failsafe.fields.fsLongActn', label: 'Long failsafe action' },
      { name: 'FS_LONG_TIMEOUT', type: 'number', firmware: 'plane', labelKey: 'settings.failsafe.fields.fsLongTimeout', label: 'Long failsafe timeout (s)', min: 1, max: 300, step: 0.5 },
    ],
  },
  {
    id: 'battery',
    titleKey: 'settings.failsafe.groups.battery',
    title: 'Battery',
    params: [
      { name: 'BATT_FS_LOW_ACT', type: 'select', firmwareEnum: true, labelKey: 'settings.failsafe.fields.battFsLowAct', label: 'Low battery action' },
      { name: 'BATT_LOW_VOLT', type: 'number', labelKey: 'settings.failsafe.fields.battLowVolt', label: 'Low battery voltage (V)', min: 0, step: 0.1 },
      { name: 'BATT_LOW_MAH', type: 'number', labelKey: 'settings.failsafe.fields.battLowMah', label: 'Low battery capacity (mAh)', min: 0, step: 50 },
      { name: 'BATT_FS_CRT_ACT', type: 'select', firmwareEnum: true, labelKey: 'settings.failsafe.fields.battFsCrtAct', label: 'Critical battery action' },
      { name: 'BATT_CRT_VOLT', type: 'number', labelKey: 'settings.failsafe.fields.battCrtVolt', label: 'Critical battery voltage (V)', min: 0, step: 0.1 },
    ],
  },
  {
    id: 'gcs',
    titleKey: 'settings.failsafe.groups.gcs',
    title: 'GCS Link',
    params: [
      { name: 'FS_GCS_ENABLE', type: 'select', enum: 'FS_GCS_ENABLE', firmware: 'copter', labelKey: 'settings.failsafe.fields.fsGcsEnable', label: 'GCS failsafe action' },
      { name: 'FS_GCS_ENABL', type: 'select', enum: 'FS_GCS_ENABL', firmware: 'plane', labelKey: 'settings.failsafe.fields.fsGcsEnabl', label: 'GCS failsafe trigger', noteKey: 'settings.failsafe.gcsPlaneNote' },
    ],
  },
];

/** All curated failsafe param names, across both firmwares. */
export function allFailsafeParamNames() {
  const out = [];
  for (const g of FAILSAFE_GROUPS) for (const p of g.params) out.push(p.name);
  return out;
}

/**
 * Infer firmware from the set of parameter names present on the vehicle.
 * Returns 'plane', 'copter', or 'unknown'. Names are firmware-exclusive so
 * conflicts shouldn't happen; if they do (or nothing matches), we say
 * 'unknown' and the UI degrades safely (no possibly-wrong enum labels).
 */
export function detectFirmware(presentNames) {
  const set = presentNames instanceof Set ? presentNames : new Set(presentNames || []);
  const plane = PLANE_MARKERS.some((n) => set.has(n));
  const copter = COPTER_MARKERS.some((n) => set.has(n));
  if (plane && !copter) return 'plane';
  if (copter && !plane) return 'copter';
  return 'unknown';
}

/**
 * Resolve the ACTION_LABELS table name for a param def given firmware.
 * Returns null when no safe table applies (e.g. a firmware-specific battery
 * enum on an undetected firmware) — callers then fall back to a raw number.
 */
export function enumNameForParam(def, firmware) {
  if (!def || def.type !== 'select') return null;
  if (def.firmwareEnum) {
    if (firmware === 'plane') return 'BATT_ACTION_PLANE';
    if (firmware === 'copter') return 'BATT_ACTION_COPTER';
    return null;
  }
  return def.enum || null;
}

/**
 * Selectable options for a param def: `[{ value, label, key }]`. Empty when
 * the param is not a select or no firmware-safe enum table applies.
 */
export function optionsForParam(def, firmware) {
  const name = enumNameForParam(def, firmware);
  if (!name) return [];
  const pairs = ACTION_LABELS[name] || [];
  return pairs.map(([value, label]) => ({
    value,
    label,
    key: `settings.failsafe.actions.${name}.${value}`,
  }));
}

/**
 * English label for a value, or null if the value is outside the enum (the
 * caller should then show the raw number rather than invent a label).
 */
export function actionLabel(def, value, firmware) {
  const opts = optionsForParam(def, firmware);
  const match = opts.find((o) => o.value === Number(value));
  return match ? match.label : null;
}

/**
 * Param defs in `group` that are present on the vehicle and applicable to the
 * detected firmware. Firmware-tagged params for the other firmware are hidden
 * even on the off chance both names appear in a snapshot.
 */
export function visibleParamsForGroup(group, presentNames, firmware) {
  const set = presentNames instanceof Set ? presentNames : new Set(presentNames || []);
  return (group?.params || []).filter((d) => {
    if (!set.has(d.name)) return false;
    if (d.firmware && firmware !== 'unknown' && d.firmware !== firmware) return false;
    return true;
  });
}

/**
 * Pre-flight "is failsafe configured?" checks against effective values
 * (`valueByName`: name -> number/string). Each returned check is
 * `{ id, labelKey, label, ok, level }`. Only checks whose backing param is
 * present on the vehicle are emitted, so an absent param never reads as a
 * red warning the operator can't act on.
 *
 * A check is `ok` only when the failsafe is actually doing something:
 *   - RC/throttle failsafe enabled (> 0, i.e. not Disabled)
 *   - low battery action set (> 0, i.e. not "Warn only")
 *   - a low battery threshold set (voltage or mAh > 0)
 *   - GCS-link failsafe enabled (> 0)
 */
export function buildFailsafeChecks(valueByName, firmware) {
  const vals = valueByName || {};
  const has = (n) => Object.prototype.hasOwnProperty.call(vals, n)
    && vals[n] != null && vals[n] !== '';
  const num = (n) => {
    const v = Number(vals[n]);
    return Number.isFinite(v) ? v : null;
  };
  const pick = (planeName, copterName) => {
    if (firmware === 'plane') return has(planeName) ? planeName : null;
    if (firmware === 'copter') return has(copterName) ? copterName : null;
    if (has(planeName)) return planeName;
    if (has(copterName)) return copterName;
    return null;
  };
  const checks = [];

  const thr = pick('THR_FAILSAFE', 'FS_THR_ENABLE');
  if (thr) {
    const v = num(thr);
    checks.push({ id: 'throttle', labelKey: 'settings.failsafe.checks.throttle', label: 'RC / throttle failsafe', ok: v != null && v > 0 });
  }

  if (has('BATT_FS_LOW_ACT')) {
    const v = num('BATT_FS_LOW_ACT');
    checks.push({ id: 'batteryAction', labelKey: 'settings.failsafe.checks.batteryAction', label: 'Low battery action', ok: v != null && v > 0 });
  }

  if (has('BATT_LOW_VOLT') || has('BATT_LOW_MAH')) {
    const volt = num('BATT_LOW_VOLT');
    const mah = num('BATT_LOW_MAH');
    checks.push({
      id: 'batteryThreshold',
      labelKey: 'settings.failsafe.checks.batteryThreshold',
      label: 'Low battery threshold',
      ok: (volt != null && volt > 0) || (mah != null && mah > 0),
    });
  }

  const gcs = pick('FS_GCS_ENABL', 'FS_GCS_ENABLE');
  if (gcs) {
    const v = num(gcs);
    checks.push({ id: 'gcs', labelKey: 'settings.failsafe.checks.gcs', label: 'GCS link failsafe', ok: v != null && v > 0 });
  }

  return checks.map((c) => ({ ...c, level: c.ok ? 'ok' : 'warn' }));
}

/**
 * Roll checks up to a banner state: configured only when there is at least
 * one applicable check and none are warnings.
 */
export function summarizeChecks(checks) {
  const list = checks || [];
  const warnCount = list.filter((c) => !c.ok).length;
  return { total: list.length, warnCount, configured: list.length > 0 && warnCount === 0 };
}
