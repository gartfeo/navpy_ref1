import { useEffect, useRef } from 'react';
import { useTranslation } from 'react-i18next';
import { zoneColorsSolid } from '../../../styles';
import { uavModelOrientation } from '../utils/uavOrientation';

/** Marks kept per UAV in the browser; the oldest is dropped. */
export const SIM_CLOSEST_APPROACH_MARKS_PER_UAV = 20;
/** Camera range (m) within which the ghost and the leg labels show. */
export const SIM_CLOSEST_APPROACH_DETAIL_RANGE_M = 200;

// Aborted: NAV ended the approach early (a NAV_FAIL or a cancel).
const OUTCOME_COLORS = { docked: '#22c55e', missed: '#ef4444', aborted: '#f59e0b' };
const GHOST_ALPHA = 0.35;

const finite = (value) => typeof value === 'number' && Number.isFinite(value);

/** A record the map can place: both heights over home are known. */
function drawable(record) {
  return [
    record.lat, record.lon, record.alt_rel_m, record.dock_lat, record.dock_lon,
    record.dock_alt_rel_m, record.d3_m, record.h_m, record.v_m,
  ].every(finite);
}

function hasAttitude(record) {
  return finite(record.roll_deg) && finite(record.pitch_deg) && finite(record.yaw_deg);
}

const metres = (value) => `${value.toFixed(2)} m`;

export function closestApproachLabel(record, t) {
  const reason = record.outcome === 'aborted' && record.reason ? ` (${record.reason})` : '';
  const outcome = `${t(`mapMarkers.closestApproach.${record.outcome}`)}${reason}`;
  return `UAV ${record.sys_id} · ${metres(record.d3_m)} ${outcome}\n`
    + `h ${metres(record.h_m)} · v ${metres(record.v_m)}`;
}

/**
 * Sim only: each UAV's closest approach to its dock after a final approach
 * (docs/design/gcs_sim_closest_approach.md). `records` is the latest
 * `navpy_sim_status` list (null before the first one). Each record key is
 * consumed once, the first time a status shows it, and becomes a mark only if
 * sim mode is on, recording is not paused, the UAV is in the fleet and this is
 * not the first status after load. The marks follow the trail: `show` and
 * `hiddenSysIds` hide them, a change of `resetKey` clears them (keys stay
 * consumed), leaving the fleet or sim mode removes them.
 */
export default function useClosestApproachMarks(
  cesiumRef, viewerRef, storeRef, viewerReady, records,
  simMode = false, show = true, resetKey = 0, hiddenSysIds = null, paused = false,
) {
  const { t, i18n } = useTranslation();
  const marksRef = useRef(new Map()); // key -> { record, colorIdx, entities }
  const consumedRef = useRef(new Set());
  const firstStatusSeenRef = useRef(false);
  const stateRef = useRef({});
  stateRef.current = { simMode, show, hiddenSysIds, paused, t };

  const visibleFor = (sysId) => stateRef.current.show && !stateRef.current.hiddenSysIds?.has(sysId);

  function undraw(mark) {
    const viewer = viewerRef.current;
    for (const e of mark.entities || []) {
      try { viewer?.entities.remove(e); } catch {}
    }
    mark.entities = null;
  }

  function removeWhere(predicate) {
    for (const [key, mark] of marksRef.current) {
      if (!predicate(mark)) continue;
      undraw(mark);
      marksRef.current.delete(key);
    }
  }

  function draw(key, mark) {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer || mark.entities) return;
    const r = mark.record;
    const shown = visibleFor(r.sys_id);
    const outcome = Cesium.Color.fromCssColorString(OUTCOME_COLORS[r.outcome] || OUTCOME_COLORS.missed);
    const uavColor = Cesium.Color.fromCssColorString(zoneColorsSolid[mark.colorIdx % zoneColorsSolid.length]);
    const detail = new Cesium.DistanceDisplayCondition(0, SIM_CLOSEST_APPROACH_DETAIL_RANGE_M);
    const point = Cesium.Cartesian3.fromDegrees(r.lon, r.lat, r.alt_rel_m);
    const dock = Cesium.Cartesian3.fromDegrees(r.dock_lon, r.dock_lat, r.dock_alt_rel_m);
    const corner = Cesium.Cartesian3.fromDegrees(r.lon, r.lat, r.dock_alt_rel_m);
    const midpoint = (a, b) => Cesium.Cartesian3.midpoint(a, b, new Cesium.Cartesian3());
    const ghost = hasAttitude(r);
    const properties = {
      closestApproachKey: key,
      closestApproachView: { lon: r.lon, lat: r.lat, alt: r.alt_rel_m, headingDeg: ghost ? r.yaw_deg : null },
    };
    const legLabel = (text) => ({
      text,
      font: '11px sans-serif',
      fillColor: Cesium.Color.WHITE,
      outlineColor: Cesium.Color.BLACK,
      outlineWidth: 2,
      style: Cesium.LabelStyle.FILL_AND_OUTLINE,
      distanceDisplayCondition: detail,
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    });
    const dashed = new Cesium.PolylineDashMaterialProperty({ color: Cesium.Color.WHITE, dashLength: 8 });
    const add = (entity) => viewer.entities.add({ show: shown, ...entity });

    const entities = [
      add({
        position: point,
        properties,
        point: {
          pixelSize: 10,
          color: outcome,
          outlineColor: Cesium.Color.WHITE,
          outlineWidth: 2,
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
        label: {
          text: closestApproachLabel(r, stateRef.current.t),
          font: '12px sans-serif',
          fillColor: outcome,
          showBackground: true,
          backgroundColor: Cesium.Color.BLACK.withAlpha(0.7),
          verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
          pixelOffset: new Cesium.Cartesian2(0, -12),
          scaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
          pixelOffsetScaleByDistance: new Cesium.NearFarScalar(500, 1.0, 20000, 0.3),
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
      }),
      add({
        position: dock,
        point: {
          pixelSize: 6,
          color: Cesium.Color.WHITE,
          outlineColor: Cesium.Color.BLACK,
          outlineWidth: 1,
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
        },
      }),
      add({
        position: midpoint(dock, corner),
        polyline: { positions: [dock, corner], width: 2, material: dashed },
        label: legLabel(`h ${metres(r.h_m)}`),
      }),
      add({
        position: midpoint(corner, point),
        polyline: { positions: [corner, point], width: 2, material: dashed },
        label: legLabel(`v ${metres(r.v_m)}`),
      }),
      add({ polyline: { positions: [dock, point], width: 1.5, material: outcome } }),
    ];
    if (ghost) {
      entities.push(add({
        position: point,
        orientation: uavModelOrientation(Cesium, point, r.yaw_deg, r.roll_deg, r.pitch_deg),
        properties,
        model: {
          uri: '/Peron15.glb',
          scale: 1.0,
          color: uavColor.withAlpha(GHOST_ALPHA),
          colorBlendMode: Cesium.ColorBlendMode.MIX,
          colorBlendAmount: 0.7,
          distanceDisplayCondition: detail,
        },
      }));
    }
    mark.entities = entities;
  }

  function drawAll() {
    for (const [key, mark] of marksRef.current) draw(key, mark);
  }

  // Consume each new record key once.
  useEffect(() => {
    if (records == null) return;
    const consumed = consumedRef.current;
    const firstStatus = !firstStatusSeenRef.current;
    firstStatusSeenRef.current = true;
    const { simMode: simOn, paused: isPaused } = stateRef.current;
    const fleet = storeRef.current?.getVehicleList() || [];
    const listed = new Set();
    for (const record of records) {
      listed.add(record.key);
      if (consumed.has(record.key)) continue;
      consumed.add(record.key);
      if (firstStatus || !simOn || isPaused || !drawable(record)) continue;
      const colorIdx = fleet.findIndex(v => v.sys_id === record.sys_id);
      if (colorIdx < 0) continue;
      marksRef.current.set(record.key, { record, colorIdx, entities: null });
      const own = [...marksRef.current].filter(([, m]) => m.record.sys_id === record.sys_id);
      for (const [key, mark] of own.slice(0, Math.max(0, own.length - SIM_CLOSEST_APPROACH_MARKS_PER_UAV))) {
        undraw(mark);
        marksRef.current.delete(key);
      }
    }
    // The backend never shows a dropped key again: keep the set bounded.
    for (const key of consumed) if (!listed.has(key)) consumed.delete(key);
    drawAll();
  }, [records]);

  // Clear all / Restart mission.
  useEffect(() => {
    removeWhere(() => true);
  }, [resetKey]);

  useEffect(() => {
    if (!simMode) removeWhere(() => true);
  }, [simMode]);

  // Hide and show without losing marks.
  useEffect(() => {
    for (const mark of marksRef.current.values()) {
      const shown = visibleFor(mark.record.sys_id);
      for (const e of mark.entities || []) e.show = shown;
    }
  }, [show, hiddenSysIds]);

  useEffect(() => {
    for (const mark of marksRef.current.values()) {
      if (!mark.entities) continue;
      mark.entities[0].label.text = closestApproachLabel(mark.record, t);
    }
  }, [i18n.language]);

  useEffect(() => {
    if (viewerReady) drawAll();
  }, [viewerReady]);

  // A UAV that leaves the fleet loses its marks.
  useEffect(() => {
    const store = storeRef.current;
    if (!store) return undefined;
    const update = () => {
      const fleet = new Set((store.getVehicleList() || []).map(v => v.sys_id));
      removeWhere(mark => !fleet.has(mark.record.sys_id));
    };
    update();
    return store.subscribe(update);
  }, []);

  useEffect(() => () => removeWhere(() => true), []);
}
