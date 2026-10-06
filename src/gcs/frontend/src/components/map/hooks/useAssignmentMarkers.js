import { useEffect, useRef } from 'react';
import { makeAssignmentIcon } from '../constants/icons';
import { zoneColorsSolid } from '../../../styles';
import { useTelemetryStore } from '../../../hooks/useTelemetryStore';

/**
 * Renders crosshair markers for peer-assigned POIs.
 * - Billboard: crosshair colored to match assigned UAV
 * - Label: "UAV {id} > {taskType}" with checkmark on accept
 */
export default function useAssignmentMarkers(cesiumRef, viewerRef, assignments, storeRef, viewerReady) {
  const markersRef = useRef({});   // task_id -> entity
  const sysIdKey = useTelemetryStore(s => s.getSysIdKey());

  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    const existing = markersRef.current;
    const seen = new Set();
    const vehicleList = storeRef.current.getVehicleList();
    const sysIdIndex = new Map();

    for (let i = 0; i < vehicleList.length; i++) {
      sysIdIndex.set(vehicleList[i].sys_id, i);
    }

    for (const [tidStr, entry] of Object.entries(assignments || {})) {
      const tid = Number(tidStr);
      if (entry.lat == null || entry.lon == null) continue;
      seen.add(tid);

      const vIdx = sysIdIndex.get(entry.receiverId) ?? -1;
      const uavColor = zoneColorsSolid[vIdx >= 0 ? vIdx % zoneColorsSolid.length : 0];

      const labelSuffix = entry.status === 'assigned' ? ' \u2713' : '';
      const labelText = `UAV ${entry.receiverId} > ${entry.taskType}${labelSuffix}`;
      const position = Cesium.Cartesian3.fromDegrees(entry.lon, entry.lat, entry.alt || 0);

      if (existing[tid]) {
        // Update existing marker
        existing[tid].position = position;
        existing[tid].billboard.image = makeAssignmentIcon(32, uavColor);
        existing[tid].label.text = labelText;
        existing[tid].label.fillColor = Cesium.Color.fromCssColorString(uavColor);
      } else {
        // Create new marker
        existing[tid] = viewer.entities.add({
          position,
          billboard: {
            image: makeAssignmentIcon(32, uavColor),
            width: 32,
            height: 32,
            scaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
            heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
          label: {
            text: labelText,
            font: 'bold 11px sans-serif',
            fillColor: Cesium.Color.fromCssColorString(uavColor),
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

    // Remove markers for gone assignments
    for (const tidStr of Object.keys(existing)) {
      if (!seen.has(Number(tidStr))) {
        try { viewer.entities.remove(existing[tidStr]); } catch {}
        delete existing[tidStr];
      }
    }
  }, [assignments, viewerReady, sysIdKey]);
}
