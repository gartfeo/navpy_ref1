import { nearestPointOnPolygonEdge } from '../../../utils/geo';
import { cartographicToMapPoint } from './planVertexPlacement.js';

/**
 * Ray-casting point-in-polygon test. pt and polygon items are {lat, lon}.
 */
export function pointInPolygon(pt, polygon) {
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const xi = polygon[i].lon, yi = polygon[i].lat;
    const xj = polygon[j].lon, yj = polygon[j].lat;
    const intersect = ((yi > pt.lat) !== (yj > pt.lat)) &&
      (pt.lon < (xj - xi) * (pt.lat - yi) / (yj - yi) + xi);
    if (intersect) inside = !inside;
  }
  return inside;
}

/**
 * Pick lat/lon on actual terrain surface (not ellipsoid).
 */
export function pickCartographic(Cesium, viewer, screenPos) {
  const ray = viewer.camera.getPickRay(screenPos);
  if (!ray) return null;
  const cartesian = viewer.scene.globe.pick(ray, viewer.scene);
  if (Cesium.defined(cartesian)) {
    const carto = Cesium.Ellipsoid.WGS84.cartesianToCartographic(cartesian);
    return cartographicToMapPoint(Cesium, carto);
  }
  // Fallback to ellipsoid pick (before terrain loads)
  const ellipsoidPick = viewer.camera.pickEllipsoid(screenPos, viewer.scene.globe.ellipsoid);
  if (Cesium.defined(ellipsoidPick)) {
    const carto = Cesium.Ellipsoid.WGS84.cartesianToCartographic(ellipsoidPick);
    return cartographicToMapPoint(Cesium, carto);
  }
  return null;
}

/**
 * Pick an entity at a screen position. Uses drillPick to find ALL entities
 * and prioritizes interactive elements (billboards) over polygon fill.
 * Returns { type, index, setIdx, corridorIndex } or null.
 */
export function pickEntity(Cesium, viewer, screenPos) {
  const picks = viewer.scene.drillPick(screenPos, 5);
  let polygonHit = false;
  for (const picked of picks) {
    if (!Cesium.defined(picked) || !picked.id) continue;
    const props = picked.id.properties;
    if (!props) continue;
    if (Cesium.defined(props.vertexIndex)) {
      return { type: 'vertex', index: props.vertexIndex.getValue() };
    }
    if (Cesium.defined(props.midpointIndex)) {
      return { type: 'midpoint', index: props.midpointIndex.getValue() };
    }
    if (Cesium.defined(props.fenceVertexIndex)) {
      return { type: 'fenceVertex', index: props.fenceVertexIndex.getValue() };
    }
    if (Cesium.defined(props.fenceMidpointIndex)) {
      return { type: 'fenceMidpoint', index: props.fenceMidpointIndex.getValue() };
    }
    if (Cesium.defined(props.exclusionVertexIndex)) {
      return {
        type: 'exclusionVertex',
        ring: props.exclusionVertexRing.getValue(),
        index: props.exclusionVertexIndex.getValue(),
      };
    }
    if (Cesium.defined(props.exclusionMidIndex)) {
      return {
        type: 'exclusionMidpoint',
        ring: props.exclusionMidRing.getValue(),
        index: props.exclusionMidIndex.getValue(),
      };
    }
    if (Cesium.defined(props.isCorridorMidpoint)) {
      return { type: 'corridorMidpoint', setIdx: props.setIdx.getValue(), index: props.corridorMidIndex.getValue() };
    }
    if (Cesium.defined(props.isSetCorridorPoint)) {
      return { type: 'setCorridorPoint', setIdx: props.setIdx.getValue(), corridorIndex: props.corridorIndex.getValue() };
    }
    if (Cesium.defined(props.isPartitionHandle)) {
      return { type: 'partitionHandle' };
    }
    if (Cesium.defined(props.isTrackWp)) {
      return { type: 'trackWp', zoneIndex: props.zoneIndex.getValue(), wpIndex: props.wpIndex.getValue() };
    }
    if (Cesium.defined(props.deliveryHubIndex)) {
      return { type: 'deliveryHub', index: props.deliveryHubIndex.getValue() };
    }
    if (Cesium.defined(props.isPolygonFill)) {
      polygonHit = true;
    }
  }
  return polygonHit ? { type: 'polygon' } : null;
}

/**
 * Clamp a point inside the rendered launch zone boundary (outer).
 */
export function clampToRing(pt, clampOuterRef) {
  const outer = clampOuterRef.current;
  let c = { lat: pt.lat, lon: pt.lon };
  if (outer && outer.length >= 3 && !pointInPolygon(c, outer)) {
    c = nearestPointOnPolygonEdge(c, outer);
  }
  return c;
}
