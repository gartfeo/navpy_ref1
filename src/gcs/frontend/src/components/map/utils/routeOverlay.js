import { zoneColorsSolid } from '../../../styles';

const LINE_WIDTH_PX = 3;
const DOT_PX = 8;
const NEXT_DOT_PX = 13;
const LABEL_FONT = 'bold 13px sans-serif';
const LABEL_OFFSET_PX = -14;

/**
 * The viewed UAV's route in a first-person view (utils/fpvRoute.js routeFor),
 * drawn with scene primitives, so it works on a CesiumWidget (no entities) and
 * on the main map alike. Waypoints are dots; the next one is larger and
 * outlined. Labels: 'all' numbers every waypoint, 'next' only the next one.
 * Dots and labels draw over the terrain (no depth test), like the main map's
 * track markers; the line is depth tested.
 *
 * @param {object} Cesium
 * @param {object} scene
 * @returns {{ update: (route: object|null, opts: {line: boolean, labels: 'all'|'next'}) => boolean,
 *   destroy: () => void }} update returns true when it changed what is drawn.
 */
export function createRouteOverlay(Cesium, scene) {
  const lines = scene.primitives.add(new Cesium.PolylineCollection());
  const dots = scene.primitives.add(new Cesium.PointPrimitiveCollection());
  const labels = scene.primitives.add(new Cesium.LabelCollection());
  let drawnKey = null;

  const clear = () => {
    lines.removeAll();
    dots.removeAll();
    labels.removeAll();
  };

  return {
    update(route, { line, labels: labelMode }) {
      const key = route
        ? [route.positions, route.zone, route.next, line, labelMode]
        : null;
      if (sameKey(key, drawnKey)) return false;
      drawnKey = key;
      clear();
      if (!route) return true;
      const color = Cesium.Color.fromCssColorString(zoneColorsSolid[route.zone % zoneColorsSolid.length]);
      if (line && route.positions.length >= 2) {
        lines.add({
          positions: route.positions,
          width: LINE_WIDTH_PX,
          material: Cesium.Material.fromType('Color', { color: color.withAlpha(0.9) }),
        });
      }
      route.positions.forEach((position, i) => {
        const isNext = i === route.next;
        dots.add({
          position,
          pixelSize: isNext ? NEXT_DOT_PX : DOT_PX,
          color: isNext ? Cesium.Color.WHITE : color,
          outlineColor: isNext ? color : Cesium.Color.BLACK,
          outlineWidth: isNext ? 3 : 1,
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        });
        if (labelMode === 'all' || (labelMode === 'next' && isNext)) {
          labels.add({
            position,
            text: String(i + 1),
            font: LABEL_FONT,
            fillColor: Cesium.Color.WHITE,
            outlineColor: Cesium.Color.BLACK,
            outlineWidth: 3,
            style: Cesium.LabelStyle.FILL_AND_OUTLINE,
            pixelOffset: new Cesium.Cartesian2(0, LABEL_OFFSET_PX),
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          });
        }
      });
      return true;
    },
    destroy() {
      for (const c of [lines, dots, labels]) {
        if (!scene.isDestroyed?.()) scene.primitives.remove(c);
      }
    },
  };
}

function sameKey(a, b) {
  if (a === b) return true;
  if (!a || !b) return false;
  return a.every((v, i) => v === b[i]);
}
