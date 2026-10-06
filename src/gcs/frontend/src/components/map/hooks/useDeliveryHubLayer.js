import { useEffect, useRef } from 'react';
import { getDeliveryHubIcon } from '../constants/deliveryHubIcons.js';
import { zoneColorsSolid } from '../../../styles';

/**
 * Renders delivery hub markers on the Cesium map.
 * Shows type-specific icons with labels, and dashed assignment lines
 * from zone track endpoints (at altitude) down to their assigned delivery hubs (on ground).
 */
export default function useDeliveryHubLayer(cesiumRef, viewerRef, deliveryHubs, deliveryHubAssignments, plan, viewerReady, terrainReady, trackPosRef, terrainBaseRef, showTracks) {
  const entitiesRef = useRef([]);

  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    // Remove old markers
    for (const e of entitiesRef.current) {
      try { viewer.entities.remove(e); } catch {}
    }

    if (!terrainReady) {
      entitiesRef.current = [];
      return;
    }

    if (!deliveryHubs || deliveryHubs.length === 0) {
      entitiesRef.current = [];
      return;
    }

    const added = [];
    for (let i = 0; i < deliveryHubs.length; i++) {
      const location = deliveryHubs[i];
      const position = Cesium.Cartesian3.fromDegrees(location.lon, location.lat, 0);
      // Billboard with type icon — tagged for picking
      const props = new Cesium.PropertyBag();
      props.addProperty('deliveryHubIndex', i);
      const e = viewer.entities.add({
        position,
        properties: props,
        billboard: {
          image: getDeliveryHubIcon(location.type),
          width: 28,
          height: 28,
          heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
          scaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
        },
        label: {
          text: location.name || `Default delivery hub ${i + 1}`,
          font: 'bold 11px sans-serif',
          fillColor: Cesium.Color.WHITE,
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
      added.push(e);
    }

    // Assignment lines: dashed polyline from zone's last track point (at altitude) to delivery hub (on ground)
    const zones = plan?.zones || [];
    if (showTracks && deliveryHubAssignments) {
      for (let zi = 0; zi < deliveryHubAssignments.length; zi++) {
        const oi = deliveryHubAssignments[zi];
        if (oi == null || !deliveryHubs[oi]) continue;
        const zone = zones[zi];
        if (!zone?.track?.length) continue;
        const capturedZi = zi;
        const capturedOi = oi;
        const lineColor = Cesium.Color.fromCssColorString(
          zoneColorsSolid[zi % zoneColorsSolid.length]
        ).withAlpha(0.7);
        const line = viewer.entities.add({
          polyline: {
            positions: new Cesium.CallbackProperty(() => {
              const tp = trackPosRef?.current?.[capturedZi];
              if (!tp || tp.length < 1) return [];
              const groundH = terrainBaseRef?.current?.[capturedZi] || 0;
              return [
                tp[tp.length - 1],
                Cesium.Cartesian3.fromDegrees(deliveryHubs[capturedOi].lon, deliveryHubs[capturedOi].lat, groundH),
              ];
            }, false),
            width: 2,
            material: new Cesium.PolylineDashMaterialProperty({
              color: lineColor,
              dashLength: 12,
            }),
          },
        });
        added.push(line);
      }
    }

    entitiesRef.current = added;
    viewer.scene.requestRender();
  }, [deliveryHubs, deliveryHubAssignments, plan, showTracks, terrainReady]);
}
