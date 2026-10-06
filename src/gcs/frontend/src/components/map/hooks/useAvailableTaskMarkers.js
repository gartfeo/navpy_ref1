import { useEffect, useRef } from 'react';
import { makeAvailableTaskIcon } from '../constants/icons';

const AVAILABLE_COLOR = '#ff9800';

/**
 * Renders dashed orange circle markers for available (pre-assignment) tasks.
 * - Billboard: dashed orange circle with center dot
 * - Label: "#taskId taskType" in orange
 */
export default function useAvailableTaskMarkers(cesiumRef, viewerRef, availableTasks, viewerReady) {
  const markersRef = useRef({});   // task_id -> entity

  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    const existing = markersRef.current;
    const seen = new Set();

    for (const [tidStr, entry] of Object.entries(availableTasks || {})) {
      const tid = Number(tidStr);
      if (entry.lat == null || entry.lon == null) continue;
      seen.add(tid);

      const labelText = `#${entry.taskId} ${entry.taskType}`;
      const position = Cesium.Cartesian3.fromDegrees(entry.lon, entry.lat, entry.alt || 0);

      if (existing[tid]) {
        existing[tid].position = position;
        existing[tid].label.text = labelText;
      } else {
        existing[tid] = viewer.entities.add({
          position,
          billboard: {
            image: makeAvailableTaskIcon(32),
            width: 32,
            height: 32,
            scaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
            heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
          label: {
            text: labelText,
            font: 'bold 11px sans-serif',
            fillColor: Cesium.Color.fromCssColorString(AVAILABLE_COLOR),
            outlineColor: Cesium.Color.BLACK,
            outlineWidth: 2,
            style: Cesium.LabelStyle.FILL_AND_OUTLINE,
            verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
            pixelOffset: new Cesium.Cartesian2(0, -18),
            scaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
            pixelOffsetScaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
            heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
        });
      }
    }

    // Remove markers for tasks no longer available
    for (const tidStr of Object.keys(existing)) {
      if (!seen.has(Number(tidStr))) {
        try { viewer.entities.remove(existing[tidStr]); } catch {}
        delete existing[tidStr];
      }
    }
  }, [availableTasks, viewerReady]);
}
