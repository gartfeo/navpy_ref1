import { useEffect, useRef } from 'react';
import { useTranslation } from 'react-i18next';
import { AVAILABLE_TASK_COLOR, makeAvailableTaskIcon } from '../constants/icons';
import { dockLabelStyle, dockMarkerPosition, dockPadScale, dockWordRange } from '../utils/dockLabel';

/**
 * Renders dashed orange circle markers for available (pre-assignment) tasks.
 * - Billboard: dashed orange circle with center dot
 * - Label: "#taskId taskType: detected" in orange
 */
export default function useAvailableTaskMarkers(cesiumRef, viewerRef, availableTasks, viewerReady) {
  const markersRef = useRef({});   // `${owner}:${task}` -> entity
  const { t, i18n } = useTranslation();

  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    const existing = markersRef.current;
    const seen = new Set();

    for (const [key, entry] of Object.entries(availableTasks || {})) {
      if (entry.lat == null || entry.lon == null) continue;
      seen.add(key);

      const labelText = `#${entry.taskId} ${entry.taskType}: ${t('mapMarkers.dockState.detected')}`;
      const style = dockLabelStyle(Cesium, AVAILABLE_TASK_COLOR, entry.labelRow);
      const position = dockMarkerPosition(Cesium, viewer, entry);

      if (existing[key]) {
        existing[key].position = position;
        existing[key].billboard.show = !entry.hidePad;
        Object.assign(existing[key].label, { text: labelText, ...style });
      } else {
        existing[key] = viewer.entities.add({
          position,
          billboard: {
            image: makeAvailableTaskIcon(32),
            show: !entry.hidePad,
            width: 32,
            height: 32,
            scaleByDistance: dockPadScale(Cesium),
            heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
          label: {
            text: labelText,
            font: 'bold 11px sans-serif',
            outlineWidth: 2,
            style: Cesium.LabelStyle.FILL_AND_OUTLINE,
            // Below the pad: hub and closest-approach labels sit above this same spot.
            verticalOrigin: Cesium.VerticalOrigin.TOP,
            scaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
            pixelOffsetScaleByDistance: dockPadScale(Cesium),
            distanceDisplayCondition: dockWordRange(Cesium),
            heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
            ...style,
          },
        });
      }
    }

    // Remove markers for tasks no longer available
    for (const key of Object.keys(existing)) {
      if (!seen.has(key)) {
        try { viewer.entities.remove(existing[key]); } catch {}
        delete existing[key];
      }
    }
    // i18n.language re-runs so labels follow a live locale switch.
  }, [availableTasks, viewerReady, i18n.language]);
}
