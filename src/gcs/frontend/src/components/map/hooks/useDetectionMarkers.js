import { useEffect } from 'react';
import { useTranslation } from 'react-i18next';
import { zoneColorsSolid } from '../../../styles';

/** Build an observation-post icon+label data URI in the given hex color.
 * `prefix` is the localized abbreviation (e.g. OP / ДК / ԴԿ). */
function makeObservationPostIcon(hex, wpNumber, prefix) {
  const r = parseInt(hex.slice(1, 3), 16);
  const g = parseInt(hex.slice(3, 5), 16);
  const b = parseInt(hex.slice(5, 7), 16);
  const label = `${prefix} ${wpNumber}`;
  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="48" height="36" viewBox="0 0 48 36" fill="none">
    <g transform="translate(12,0)">
      <line x1="1" y1="16" x2="23" y2="16" stroke="${hex}" stroke-width="2.5"/>
      <path d="M1 16 A11 11 0 0 1 23 16" fill="rgba(${r},${g},${b},0.15)" stroke="${hex}" stroke-width="2.5"/>
      <circle cx="12" cy="11" r="2.5" fill="${hex}"/>
    </g>
    <text x="24" y="34" text-anchor="middle" font-family="sans-serif" font-weight="bold" font-size="11" fill="${hex}" stroke="#000" stroke-width="3" paint-order="stroke">${label}</text>
  </svg>`;
  return 'data:image/svg+xml,' + encodeURIComponent(svg);
}

/**
 * Renders observation post (НП) markers at the first detection waypoint for each zone.
 * Positioned at the plan altitude using Cesium's relative-to-ground height.
 * Icon + label are baked into a single SVG billboard for stable rendering.
 *
 * @param {object} cesiumRef - Ref to Cesium module
 * @param {object} viewerRef - Ref to Cesium Viewer
 * @param {object} entitiesRef - Shared entity tracking ref (uses .detectionStart key)
 * @param {Array<{lat,lon,alt,zoneIndex,wpNumber}>|null} detectionPoints - Resolved points or null
 */
export default function useDetectionMarkers(cesiumRef, viewerRef, entitiesRef, detectionPoints, viewerReady) {
  const { t, i18n } = useTranslation();
  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer) return;

    const opPrefix = t('mapMarkers.observationPost');

    const ents = entitiesRef.current;
    const existing = ents.detectionStart || [];

    // Remove old markers
    for (const e of existing) {
      try { viewer.entities.remove(e); } catch {}
    }

    if (!detectionPoints || detectionPoints.length === 0) {
      ents.detectionStart = [];
      return;
    }

    const added = [];
    for (let i = 0; i < detectionPoints.length; i++) {
      const pt = detectionPoints[i];
      const colorHex = zoneColorsSolid[pt.zoneIndex % zoneColorsSolid.length];
      const position = Cesium.Cartesian3.fromDegrees(pt.lon, pt.lat, pt.alt);

      const e = viewer.entities.add({
        position,
        billboard: {
          image: makeObservationPostIcon(colorHex, pt.wpNumber, opPrefix),
          width: 48,
          height: 36,
          verticalOrigin: Cesium.VerticalOrigin.CENTER,
          heightReference: Cesium.HeightReference.RELATIVE_TO_GROUND,
          scaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
      });
      added.push(e);
    }

    ents.detectionStart = added;
    // viewerReady gates re-run so detection markers/labels set before the
    // async Cesium viewer finished init aren't lost — same fresh-load race as
    // usePolygonLayer.
    // i18n.language re-runs so the НП/OP/ԴԿ label follows a live locale switch.
  }, [detectionPoints, viewerReady, i18n.language]);
}
