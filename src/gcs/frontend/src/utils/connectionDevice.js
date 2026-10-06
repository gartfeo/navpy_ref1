export const DEFAULT_GCS_DEVICE = 'udp:0.0.0.0:15550';
export const LEGACY_MISSION_PLANNER_DEVICE = 'udp:0.0.0.0:14550';

export function resolvePanelDevice(currentDevice, defaultDevice, userEdited) {
  if (!defaultDevice || userEdited) return currentDevice || '';
  const trimmed = (currentDevice || '').trim();
  if (!trimmed || trimmed === LEGACY_MISSION_PLANNER_DEVICE) return defaultDevice;
  return currentDevice;
}
