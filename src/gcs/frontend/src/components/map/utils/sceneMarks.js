import { colors, zoneColorsSolid } from '../../../styles';
import { DOCK_ICON, getDeliveryHubIcon } from '../constants/deliveryHubIcons';

const DOCK_ICON_PX = 34;
const HUB_ICON_PX = 26;
const FLEET_DOT_PX = 10;
const LABEL_FONT = 'bold 13px sans-serif';
const LABEL_OFFSET_PX = -22;

/**
 * The Synthetic view's marks beside the route (utils/syntheticScene.js): the
 * UAV's dock, highlighted with its distance (the scene's anchor, like a
 * cockpit synthetic vision display's active runway), its home hub,
 * and each other fleet UAV as a dot in its zone colour with its name
 * (cooperative vehicles, never a threat colour). Scene primitives, so it works
 * on a CesiumWidget. Ground marks clamp to the terrain and draw over it (no
 * depth test), like the main map's markers.
 *
 * @returns {{ update: (marks: object) => boolean, destroy: () => void }}
 *   update returns true when it changed what is drawn.
 */
export function createSceneMarks(Cesium, scene) {
  const billboards = scene.primitives.add(new Cesium.BillboardCollection({ scene }));
  const groundLabels = scene.primitives.add(new Cesium.LabelCollection({ scene }));
  const fleetDots = scene.primitives.add(new Cesium.PointPrimitiveCollection());
  const fleetLabels = scene.primitives.add(new Cesium.LabelCollection());
  let groundKey = null;
  let dockLabel = null;
  let fleetKey = null;

  const labelStyle = (text, extra = {}) => ({
    text,
    font: LABEL_FONT,
    fillColor: Cesium.Color.WHITE,
    outlineColor: Cesium.Color.BLACK,
    outlineWidth: 3,
    style: Cesium.LabelStyle.FILL_AND_OUTLINE,
    pixelOffset: new Cesium.Cartesian2(0, LABEL_OFFSET_PX),
    disableDepthTestDistance: Number.POSITIVE_INFINITY,
    ...extra,
  });
  const ground = (lat, lon) => ({
    position: Cesium.Cartesian3.fromDegrees(lon, lat, 0),
    heightReference: Cesium.HeightReference.CLAMP_TO_GROUND,
  });

  return {
    /**
     * @param {object} marks
     * @param {{lat: number, lon: number}|null} marks.dock
     * @param {string} [marks.dockText] - its label, with the distance.
     * @param {{lat: number, lon: number, type?: string}|null} marks.hub
     * @param {string} [marks.hubText]
     * @param {Array<{position: object, label: string, zone: number}>} marks.fleet
     */
    update({ dock, dockText = '', hub, hubText = '', fleet = [] }) {
      let changed = false;
      const key = [dock?.lat, dock?.lon, hub?.lat, hub?.lon, hub?.type, hubText].join(':');
      if (key !== groundKey) {
        groundKey = key;
        billboards.removeAll();
        groundLabels.removeAll();
        dockLabel = null;
        if (hub) {
          billboards.add({
            ...ground(hub.lat, hub.lon),
            image: getDeliveryHubIcon(hub.type),
            width: HUB_ICON_PX,
            height: HUB_ICON_PX,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          });
          groundLabels.add(labelStyle(hubText, ground(hub.lat, hub.lon)));
        }
        if (dock) {
          billboards.add({
            ...ground(dock.lat, dock.lon),
            image: DOCK_ICON,
            width: DOCK_ICON_PX,
            height: DOCK_ICON_PX,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          });
          dockLabel = groundLabels.add(labelStyle(dockText, {
            ...ground(dock.lat, dock.lon),
            fillColor: Cesium.Color.BLACK,
            outlineWidth: 0,
            style: Cesium.LabelStyle.FILL,
            showBackground: true,
            backgroundColor: Cesium.Color.fromCssColorString(colors.accent),
          }));
        }
        changed = true;
      }
      if (dockLabel && dockLabel.text !== dockText) {
        dockLabel.text = dockText;
        changed = true;
      }
      const nextFleetKey = fleet.map((f) => `${f.label}:${f.zone}:${f.position.x}:${f.position.y}:${f.position.z}`).join('|');
      if (nextFleetKey !== fleetKey) {
        fleetKey = nextFleetKey;
        fleetDots.removeAll();
        fleetLabels.removeAll();
        for (const f of fleet) {
          const color = Cesium.Color.fromCssColorString(zoneColorsSolid[f.zone % zoneColorsSolid.length]);
          fleetDots.add({
            position: f.position,
            pixelSize: FLEET_DOT_PX,
            color,
            outlineColor: Cesium.Color.BLACK,
            outlineWidth: 2,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          });
          fleetLabels.add(labelStyle(f.label, { position: f.position, fillColor: color }));
        }
        changed = true;
      }
      return changed;
    },
    destroy() {
      for (const c of [billboards, groundLabels, fleetDots, fleetLabels]) {
        if (!scene.isDestroyed?.()) scene.primitives.remove(c);
      }
    },
  };
}
