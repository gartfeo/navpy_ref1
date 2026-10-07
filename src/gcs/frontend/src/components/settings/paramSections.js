export const PARAM_SECTIONS = [
  {
    title: 'Final Approach',
    titleKey: 'settings.aas.sections.finalApproach',
    fields: [
      { key: 'del_pitch', label: 'Approach pitch (deg)', labelKey: 'settings.aas.fields.delPitch', step: 1, min: -90 },
      { key: 'del_thr', label: 'Approach throttle (%)', labelKey: 'settings.aas.fields.approachThrottle', step: 1, min: -1, placeholder: '-1 = TECS auto', placeholderKey: 'settings.aas.placeholders.tecsAuto' },
    ],
  },
  {
    title: 'Control',
    titleKey: 'settings.aas.sections.control',
    fields: [
      { key: 'del_ctrl', label: 'Algorithm', labelKey: 'settings.aas.fields.algorithm', type: 'select', options: [{ value: 2, label: 'Vision PN' }, { value: 1, label: 'PN (legacy geo)', labelKey: 'settings.aas.options.pnLegacyGeo' }, { value: 0, label: 'PID (legacy geo)', labelKey: 'settings.aas.options.pidLegacyGeo' }] },
      { key: 'del_p_kp', label: 'Pitch Kp', labelKey: 'settings.aas.fields.pitchKp', step: 0.1 },
      { key: 'del_pld', label: 'Pitch lock dist (m)', labelKey: 'settings.aas.fields.pitchLockDist', step: 10, min: -1 },
      { key: 'del_plrd', label: 'Pitch lock roll (deg)', labelKey: 'settings.aas.fields.pitchLockRoll', step: 1, min: -1 },
      { key: 'use_trn', label: 'Use terrain', labelKey: 'settings.aas.fields.useTerrain', type: 'checkbox' },
    ],
  },
  {
    title: 'Logging',
    titleKey: 'settings.aas.sections.logging',
    fields: [
      { key: 'log_defer', label: 'Defer logging', labelKey: 'settings.aas.fields.deferLogging', type: 'checkbox' },
      { key: 'log_rate', label: 'Log rate (Hz)', labelKey: 'settings.aas.fields.logRate', step: 1 },
    ],
  },
];

/**
 * Operationally-coupled confirmation params. Rendered as standard
 * grid rows alongside the other AAS sections. Manual-only rows
 * (nav_cwt, nav_cm_fl) collapse only when EVERY selected vehicle is
 * in automatic mode (`rowVisibleWhen`). Within a visible manual-only
 * row, individual cells are disabled for vehicles that are themselves
 * in automatic mode (`cellDisabledWhen`) -- the cell still occupies
 * its grid slot so columns stay aligned, but it is greyed and
 * pointer-disabled to signal "not editable here."
 */
export const CONFIRMATION_PARAM_SECTION = {
  title: 'Confirmation',
  titleKey: 'settings.aas.sections.confirmation',
  fields: [
    {
      key: 'nav_auto_cm', label: 'Mode', labelKey: 'settings.aas.fields.mode', type: 'select',
      options: [
        { value: true,  label: 'Automatic', labelKey: 'confirmSection.automatic' },
        { value: false, label: 'Manual', labelKey: 'confirmSection.manual' },
      ],
    },
    {
      key: 'nav_cwt', label: 'Wait time (s)', labelKey: 'settings.aas.fields.waitTime', type: 'number', step: 1, min: 0,
      rowVisibleWhen: (snapshotsBySysId) =>
        Object.values(snapshotsBySysId).some((vp) => vp?.nav_auto_cm === false),
      cellDisabledWhen: (vp) => vp?.nav_auto_cm !== false,
    },
    {
      key: 'nav_cm_fl', label: 'On timeout', labelKey: 'settings.aas.fields.onTimeout', type: 'select',
      options: [
        { value: true,  label: 'Approve', labelKey: 'confirmSection.approve' },
        { value: false, label: 'Deny', labelKey: 'confirmSection.deny' },
      ],
      rowVisibleWhen: (snapshotsBySysId) =>
        Object.values(snapshotsBySysId).some((vp) => vp?.nav_auto_cm === false),
      cellDisabledWhen: (vp) => vp?.nav_auto_cm !== false,
    },
    {
      key: 'nav_cgt', label: 'Quality image timeout (s)', labelKey: 'settings.aas.fields.qualityImageTimeout', type: 'number', step: 1, min: 0,
    },
  ],
};

export const SIM_PARAM_SECTIONS = [
  {
    title: 'Simulation POIs',
    titleKey: 'settings.aas.sections.simulationPois',
    fields: [
      { key: 'targ_alt', label: 'Dock altitude (m)', labelKey: 'settings.aas.fields.dockAlt', step: 10 },
      { key: 'nav_min_alt', label: 'Min alt (m)', labelKey: 'settings.aas.fields.minAlt', step: 10 },
      { key: 'nav_last_wp', label: 'Start detect WP', labelKey: 'settings.aas.fields.startDetectWp', step: 1 },
      { key: 'targ_wps', label: 'Simulation POI WPs', labelKey: 'settings.aas.fields.simPoiWps', type: 'bitmask' },
      { key: 'del_dir', label: 'Simulator reference position (legacy)', labelKey: 'settings.aas.fields.simulatorReferencePosition', type: 'checkbox' },
      { key: 'nav_oneshot', label: 'Disarm after attempt', labelKey: 'settings.aas.fields.disarmAfterAttempt', type: 'checkbox' },
    ],
  },
];
