// How a dock marker is placed and drawn, shared by the dock marker hooks.
const LABEL_OFFSET_PX = 18;   // below the 32 px pad
const LABEL_ROW_PX = 14;      // one 11 px line and its outline
// Far out the words of near docks print over each other, so only the pad and badge
// show. On a full HD map, docks 150 m apart clear the longest word at about this range.
const DOCK_WORD_MAX_RANGE_M = 1500;
const MIN_OUTLINE_CONTRAST = 3;

function contrastOnBlack(hex) {
  const channel = (at) => {
    const value = parseInt(hex.slice(at, at + 2), 16) / 255;
    return value <= 0.03928 ? value / 12.92 : ((value + 0.055) / 1.055) ** 2.4;
  };
  const luminance = 0.2126 * channel(1) + 0.7152 * channel(3) + 0.0722 * channel(5);
  return (luminance + 0.05) / 0.05;
}

export function dockLabelStyle(Cesium, color, row = 0) {
  return {
    fillColor: Cesium.Color.fromCssColorString(color),
    outlineColor: contrastOnBlack(color) < MIN_OUTLINE_CONTRAST ? Cesium.Color.WHITE : Cesium.Color.BLACK,
    pixelOffset: new Cesium.Cartesian2(0, LABEL_OFFSET_PX + row * LABEL_ROW_PX),
  };
}

export function dockWordRange(Cesium) {
  return new Cesium.DistanceDisplayCondition(0, DOCK_WORD_MAX_RANGE_M);
}

// A pad's size by camera range: about 22 px at Reset View (7 to 9 km), where its badge
// must still read and docks 150 m apart still clear each other.
export function dockPadScale(Cesium) {
  return new Cesium.NearFarScalar(500, 1.0, 20000, 0.6);
}

// Cesium before 1.140 draws a clamped label's new letters at the raw position until a
// new terrain tile renders, so the raw height has to be the ground already.
export function dockMarkerPosition(Cesium, viewer, entry) {
  const ground = viewer.scene.globe.getHeight(Cesium.Cartographic.fromDegrees(entry.lon, entry.lat));
  return Cesium.Cartesian3.fromDegrees(entry.lon, entry.lat, ground ?? (entry.alt || 0));
}
