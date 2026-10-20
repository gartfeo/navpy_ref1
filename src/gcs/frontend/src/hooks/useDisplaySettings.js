import { useCallback, useState } from 'react';

const NO_IDS = new Set();

/**
 * Display toggle state for map layers.
 *
 * Extracted from useMissionState to keep display concerns
 * separate from planning/mission data.
 *
 * Trail and camera-mark history (docs/design/gcs_trails_marks_control.md):
 * `showTrails` / `showCoverage` are the two layers, `hiddenSysIds` the UAVs
 * unchecked in the Trails panel (absent means shown; kept by sys_id for the
 * session, so a returning UAV keeps its choice), `historyPausedAt` the time
 * recording was paused (null = recording). Hiding never stops recording.
 */
export default function useDisplaySettings() {
  const [showZones, setShowZones] = useState(true);
  const [showTracks, setShowTracks] = useState(true);
  const [showLaunchZone, setShowLaunchZone] = useState(true);
  const [showCoverage, setShowCoverage] = useState(false);
  const [showVision, setShowVision] = useState(true);
  const [showTrails, setShowTrails] = useState(true);
  const [hiddenSysIds, setHiddenSysIds] = useState(NO_IDS);
  const [historyPausedAt, setHistoryPausedAt] = useState(null);

  const toggleSysIdHidden = useCallback((sysId) => {
    setHiddenSysIds((prev) => {
      const next = new Set(prev);
      if (next.has(sysId)) next.delete(sysId); else next.add(sysId);
      return next;
    });
  }, []);
  const toggleHistoryPaused = useCallback(() => {
    setHistoryPausedAt((prev) => (prev == null ? Date.now() : null));
  }, []);
  const resumeHistory = useCallback(() => setHistoryPausedAt(null), []);

  return {
    showZones, setShowZones,
    showTracks, setShowTracks,
    showLaunchZone, setShowLaunchZone,
    showCoverage, setShowCoverage,
    showVision, setShowVision,
    showTrails, setShowTrails,
    hiddenSysIds, toggleSysIdHidden,
    historyPausedAt, toggleHistoryPaused, resumeHistory,
  };
}
