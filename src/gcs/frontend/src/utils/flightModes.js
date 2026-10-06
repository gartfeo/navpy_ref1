/**
 * Flight mode constants for ArduPilot fixed-wing vehicles.
 * Names must match FlightMode enum values on the backend exactly.
 */

export const PRIMARY_MODES = ['MANUAL', 'FBWA', 'LOITER', 'RTL', 'AUTO'];

export const SECONDARY_MODES = ['GUIDED', 'STABILIZE', 'CIRCLE', 'CRUISE', 'FBWB'];

export const ALL_MODES = [...PRIMARY_MODES, ...SECONDARY_MODES];

/** Modes unsafe to leave without active control — need stick input or GCS commands. */
export const RADIO_MODES = ['MANUAL', 'FBWA', 'FBWB', 'STABILIZE', 'GUIDED', 'CRUISE'];

export const QUICK_MODES = ['AUTO', 'FBWA', 'RTL'];

export const OTHER_MODES = ['MANUAL', 'LOITER', 'GUIDED', 'STABILIZE', 'CIRCLE', 'CRUISE', 'FBWB'];

export function isPrimaryMode(mode) {
  return PRIMARY_MODES.includes(mode);
}

export function isSecondaryMode(mode) {
  return SECONDARY_MODES.includes(mode);
}
