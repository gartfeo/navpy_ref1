import { useEffect, useRef } from 'react';
import { makeAssignmentIcon } from '../constants/icons';
import { zoneColorsSolid } from '../../../styles';
import { useTelemetryStore } from '../../../hooks/useTelemetryStore';
import { isAssignedOrLater } from '../../../utils/taskAssignmentState';

/**
 * Renders crosshair markers for peer-assigned POIs.
 * - Billboard: crosshair colored to match assigned UAV
 * - Label: "UAV {id} > {taskType}" with a checkmark once the owner applied it
 */
export default function useAssignmentMarkers(cesiumRef, viewerRef, assignments, storeRef, viewerReady) {
  const markersRef = useRef({});   // assignment entry key -> entity
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

    for (const [key, entry] of Object.entries(assignments || {})) {
      if (entry.lat == null || entry.lon == null) continue;
      seen.add(key);

      const vIdx = sysIdIndex.get(entry.receiverId) ?? -1;
      const uavColor = zoneColorsSolid[vIdx >= 0 ? vIdx % zoneColorsSolid.length : 0];

      const labelSuffix = isAssignedOrLater(entry) ? ' \u2713' : '';
      const labelText = `UAV ${entry.receiverId} > ${entry.taskType}${labelSuffix}`;
      const position = Cesium.Cartesian3.fromDegrees(entry.lon, entry.lat, entry.alt || 0);

      if (existing[key]) {
        // Update existing marker
        existing[key].position = position;
        existing[key].billboard.image = makeAssignmentIcon(32, uavColor);
        existing[key].label.text = labelText;
        existing[key].label.fillColor = Cesium.Color.fromCssColorString(uavColor);
      } else {
        // Create new marker
        existing[key] = viewer.entities.add({
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
    for (const key of Object.keys(existing)) {
      if (!seen.has(key)) {
        try { viewer.entities.remove(existing[key]); } catch {}
        delete existing[key];
      }
    }
  }, [assignments, viewerReady, sysIdKey]);
}
