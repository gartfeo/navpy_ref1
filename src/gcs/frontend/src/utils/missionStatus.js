import { colors } from '../styles.js';

/**
 * Status definitions keyed by status id.
 * Each entry holds the display label and CSS color.
 */
const STATUS_TABLE = {
  idle:        { labelKey: 'missionStatus.idle',        color: colors.textDim },
  en_route:    { labelKey: 'missionStatus.enRoute',     color: colors.textDim },
  searching:   { labelKey: 'missionStatus.searching',   color: colors.success },
  assigned:    { labelKey: 'missionStatus.assigned',    color: colors.accent },
  approaching: { labelKey: 'missionStatus.approaching', color: colors.warning },
  confirming:  { labelKey: 'missionStatus.confirming',  color: colors.warning },
  confirmed:   { labelKey: 'missionStatus.confirmed',   color: colors.error },
};

/**
 * Compute the mission lifecycle status for a UAV.
 *
 * @param {object} vehicle - Telemetry data: { mode, armed, lat, lon, mission_progress }
 * @param {object|null} assignment - Assignment state: { taskId, taskType, lat, lon, status } or null
 * @param {number} wpOffset - Count of pre-search waypoints (corridor approach, takeoff, etc.)
 * @param {boolean} [hasPendingConfirm=false] - Whether a confirm popup is active for this vehicle
 * @returns {{ status: string, label: string, color: string }}
 */
export function computeMissionStatus(vehicle, assignment, wpOffset, hasPendingConfirm) {
  const statusId = deriveStatusId(vehicle, assignment, wpOffset, hasPendingConfirm);
  const entry = STATUS_TABLE[statusId];
  return { status: statusId, labelKey: entry.labelKey, color: entry.color };
}

/**
 * Derive the raw status id string from vehicle and assignment state.
 * Checked in priority order — first match wins.
 */
function deriveStatusId(vehicle, assignment, wpOffset, hasPendingConfirm) {
  if (assignment) {
    if (assignment.status === 'confirmed') return 'confirmed';
    if (hasPendingConfirm) return 'confirming';
    if (vehicle.mode === 'GUIDED') return 'approaching';
    return 'assigned';
  }

  if (!vehicle.armed) return 'idle';

  if (vehicle.mode === 'AUTO' && vehicle.mission_progress > 0) {
    if ((vehicle.mission_progress - wpOffset) < 0) return 'en_route';
  }

  if (vehicle.mode === 'AUTO') return 'searching';

  return 'idle';
}
