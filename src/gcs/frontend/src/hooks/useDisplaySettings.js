import { useState } from 'react';

/**
 * Display toggle state for map layers.
 *
 * Extracted from useMissionState to keep display concerns
 * separate from planning/mission data.
 */
export default function useDisplaySettings() {
  const [showZones, setShowZones] = useState(true);
  const [showTracks, setShowTracks] = useState(true);
  const [showLaunchZone, setShowLaunchZone] = useState(true);
  const [showCoverage, setShowCoverage] = useState(false);
  const [showVision, setShowVision] = useState(true);
  const [showTrails, setShowTrails] = useState(true);

  return {
    showZones, setShowZones,
    showTracks, setShowTracks,
    showLaunchZone, setShowLaunchZone,
    showCoverage, setShowCoverage,
    showVision, setShowVision,
    showTrails, setShowTrails,
  };
}
