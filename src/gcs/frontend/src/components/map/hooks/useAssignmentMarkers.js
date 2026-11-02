import { useEffect, useRef } from 'react';
import { useTranslation } from 'react-i18next';
import {
  AVAILABLE_TASK_COLOR,
  AWAITING_DOCK_COLOR,
  HANDLED_DOCK_COLOR,
  makeApprovedDockIcon,
  makeAssignmentIcon,
  makeAvailableTaskIcon,
  makeAwaitingDockIcon,
  makeHandledDockIcon,
} from '../constants/icons';
import { dockLabelStyle, dockMarkerPosition, dockPadScale, dockWordRange } from '../utils/dockLabel';
import { zoneColorsSolid } from '../../../styles';
import { useTelemetryStore } from '../../../hooks/useTelemetryStore';
import { uavOf } from '../../../utils/taskAssignmentState';

const HANDLED_WORDS = {
  docked: 'mapMarkers.dockState.docked',
  failed: 'mapMarkers.dockState.dockFailed',
  canceled: 'mapMarkers.dockState.canceled',
  unknown: 'mapMarkers.dockState.outcomeUnknown',
};

// One look per state; an entry's own label says which UAV it belongs to.
function markerLook(entry, uavColor, t) {
  const uav = uavOf(entry);
  const onUav = (key) => `UAV ${uav} > ${entry.taskType}: ${t(key)}`;
  switch (entry.status) {
    case 'waiting':
      return {
        icon: makeAvailableTaskIcon(32),
        color: AVAILABLE_TASK_COLOR,
        label: `#${entry.taskId} ${entry.taskType}: ${t('mapMarkers.dockState.offered', { uav })}`,
      };
    case 'confirming':
      return {
        icon: makeAwaitingDockIcon(32),
        color: AWAITING_DOCK_COLOR,
        label: onUav('mapMarkers.dockState.confirm'),
      };
    case 'confirmed':
      return {
        icon: makeApprovedDockIcon(32, uavColor),
        color: uavColor,
        label: onUav('mapMarkers.dockState.approved'),
      };
    case 'handled':
      return {
        icon: makeHandledDockIcon(32, entry.outcome),
        color: HANDLED_DOCK_COLOR,
        label: onUav(HANDLED_WORDS[entry.outcome] ?? HANDLED_WORDS.unknown),
      };
    default:
      return {
        icon: makeAssignmentIcon(32, uavColor),
        color: uavColor,
        label: onUav('mapMarkers.dockState.assigned'),
      };
  }
}

/**
 * Renders one marker per dock entry, drawn by its state: offered, awaiting
 * confirmation, assigned, approved or handled. An entry with no position draws
 * nothing, never a guessed point.
 */
export default function useAssignmentMarkers(cesiumRef, viewerRef, assignments, storeRef, viewerReady) {
  const markersRef = useRef({});   // assignment entry key -> entity
  const sysIdKey = useTelemetryStore(s => s.getSysIdKey());
  const { t, i18n } = useTranslation();

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

      const vIdx = sysIdIndex.get(uavOf(entry)) ?? -1;
      const uavColor = zoneColorsSolid[vIdx >= 0 ? vIdx % zoneColorsSolid.length : 0];

      const look = markerLook(entry, uavColor, t);
      const style = dockLabelStyle(Cesium, look.color, entry.labelRow);
      const position = dockMarkerPosition(Cesium, viewer, entry);

      if (existing[key]) {
        // Update existing marker
        existing[key].position = position;
        existing[key].billboard.image = look.icon;
        existing[key].billboard.show = !entry.hidePad;
        Object.assign(existing[key].label, { text: look.label, ...style });
      } else {
        // Create new marker
        existing[key] = viewer.entities.add({
          position,
          billboard: {
            image: look.icon,
            show: !entry.hidePad,
            width: 32,
            height: 32,
            scaleByDistance: dockPadScale(Cesium),
            heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
          label: {
            text: look.label,
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

    // Remove markers for gone assignments
    for (const key of Object.keys(existing)) {
      if (!seen.has(key)) {
        try { viewer.entities.remove(existing[key]); } catch {}
        delete existing[key];
      }
    }
    // i18n.language re-runs so labels follow a live locale switch.
  }, [assignments, viewerReady, sysIdKey, i18n.language]);
}
