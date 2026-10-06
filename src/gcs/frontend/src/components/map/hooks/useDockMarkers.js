import { useEffect } from 'react';
import { useTranslation } from 'react-i18next';
import { zoneColorsSolid } from '../../../styles';
import { DOCK_ICON } from '../constants/deliveryLocationIcons.js';

/** Render ground-clamped dock symbols at the configured simulation coordinates. */
export default function useDockMarkers(cesiumRef, viewerRef, entitiesRef, simDocks, viewerReady) {
  const { t, i18n } = useTranslation();
  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    const ents = entitiesRef.current;
    const existing = ents.simDocks || [];

    // Remove old markers
    for (const e of existing) {
      try { viewer.entities.remove(e); } catch {}
    }

    if (!simDocks || simDocks.length === 0) {
      ents.simDocks = [];
      return;
    }

    const added = [];
    for (let i = 0; i < simDocks.length; i++) {
      const target = simDocks[i];
      const position = Cesium.Cartesian3.fromDegrees(target.lon, target.lat, 0);
      const colorHex = zoneColorsSolid[target.zoneIndex % zoneColorsSolid.length];
      const labelColor = Cesium.Color.fromCssColorString(colorHex);

      const e = viewer.entities.add({
        position,
        billboard: {
          image: DOCK_ICON,
          width: 40,
          height: 40,
          heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
        label: {
          text: `${t('mapMarkers.dock')}${i + 1}`,
          font: 'bold 11px sans-serif',
          fillColor: labelColor,
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

    ents.simDocks = added;
    // viewerReady gates re-run so target markers/labels set before the async
    // Cesium viewer finished init aren't lost — same fresh-load race as
    // usePolygonLayer.
    // i18n.language re-runs so the Ц/T/Թ label follows a live locale switch.
  }, [simDocks, viewerReady, i18n.language]);
}
