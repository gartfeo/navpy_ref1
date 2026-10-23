import React, { useRef } from 'react';
import { act, fireEvent, render } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import useDisplaySettings from './useDisplaySettings';
import useVehicleCommands from './useVehicleCommands';
import useUavTrails from '../components/map/hooks/useUavTrails';
import useCoverageLayer from '../components/map/hooks/useCoverageLayer';
import useClosestApproachMarks from '../components/map/hooks/useClosestApproachMarks';
import TrailsControl from '../components/map/TrailsControl';

vi.mock('../utils/planner', () => ({ getDeviceConfigs: () => [{ primary: true }] }));
vi.mock('../components/map/utils/cameraFootprint', () => ({
  cameraHitsGround: () => true,
  computeLiveGimbalDetectionEnvelope: (lat, lon) => [
    { lat, lon }, { lat: lat + 0.001, lon }, { lat, lon: lon + 0.001 },
  ],
  COVERAGE_SAMPLE_DIST: 10,
  padFootprintLoop: (e) => e,
}));
vi.mock('../components/map/utils/gimbalTelemetry', () => ({
  selectGimbalTelemetry: () => ({ pitch: -90 }),
  resolveLiveGimbalCameraConfig: (d) => d,
  gimbalTelemetrySignature: (t) => JSON.stringify(t),
}));

class PolygonHierarchy { constructor(p) { this.positions = p; } }
class CallbackProperty { constructor(fn) { this.fn = fn; } }
class Cartesian3 {
  static fromDegrees(lon, lat) { return { lon, lat }; }
  static fromDegreesArrayHeights(flat) { return [...flat]; }
  static lerp(a) { return a; }
  static midpoint(a) { return a; }
}
const fakeColor = () => ({ withAlpha: (a) => ({ a }) });

function fakeStore() {
  const listeners = new Set();
  const store = {
    vehicles: new Map(),
    fresh: new Set(),
    subscribe(fn) { listeners.add(fn); return () => listeners.delete(fn); },
    getVehicleList() { return [...store.vehicles.values()]; },
    isTelemetryFresh(sid) { return store.fresh.has(sid); },
    send(sid, lon) {
      store.vehicles.set(sid, {
        sys_id: sid, lat: 40, lon, alt_rel: 100, heading: 0, pitch: 0, roll: 0, mode: 'AUTO',
      });
      store.fresh.add(sid);
      act(() => { for (const fn of listeners) fn(); });
    },
  };
  return store;
}

/** App's wiring in miniature: panel -> display state -> commands -> the history layers. */
function Harness({ store, viewer, cesium, plan, api, entitiesRef, records = null }) {
  const d = useDisplaySettings();
  const c = useVehicleCommands({
    api, vehicleList: [], taskConfirmReset: () => {}, taskAssignReset: () => {},
    setLaunchStates: () => {}, resumeHistory: d.resumeHistory,
  });
  const storeRef = useRef(store);
  const paused = d.historyPausedAt != null;
  useUavTrails(cesium, viewer, entitiesRef, storeRef, true, d.showTrails,
    `${c.trackResetKey}:${c.historyResetKey}`, d.hiddenSysIds, paused);
  useCoverageLayer(cesium, viewer, storeRef, plan, d.showCoverage, d.showVision,
    c.trackResetKey, c.historyResetKey, d.hiddenSysIds, paused, true, true);
  useClosestApproachMarks(cesium, viewer, storeRef, true, records, true, d.showTrails,
    `${c.trackResetKey}:${c.historyResetKey}`, d.hiddenSysIds, paused);
  return (
    <>
      <button type="button" onClick={() => c.handleRestart([1]).catch(() => {})}>restart</button>
      <TrailsControl
        vehicleList={[{ sys_id: 1 }]}
        showTrails={d.showTrails} onToggleTrails={() => d.setShowTrails((s) => !s)}
        showMarks={d.showCoverage} onToggleMarks={() => d.setShowCoverage((s) => !s)}
        hiddenSysIds={d.hiddenSysIds} onToggleUav={d.toggleSysIdHidden}
        pausedAt={d.historyPausedAt} onTogglePause={d.toggleHistoryPaused}
        onClearAll={c.handleClearHistory}
      />
    </>
  );
}

function setup(planRef = { current: { altitude_m: 100, zones: [] } }) {
  const store = fakeStore();
  const live = new Set();
  const entities = {
    add: vi.fn((e) => { live.add(e); return e; }),
    remove: vi.fn((e) => { live.delete(e); }),
  };
  const cesium = { current: {
    Cartesian3,
    PolygonHierarchy,
    CallbackProperty,
    Color: { fromCssColorString: fakeColor, WHITE: fakeColor(), BLACK: fakeColor() },
    Cartesian2: class {},
    NearFarScalar: class {},
    DistanceDisplayCondition: class {},
    PolylineDashMaterialProperty: class {},
    LabelStyle: {},
    VerticalOrigin: {},
    ClassificationType: { TERRAIN: 'terrain' },
  } };
  const viewer = { current: { entities } };
  const entitiesRef = { current: { trailMap: {} } };
  const api = { sendCommand: vi.fn(), restartMission: vi.fn(() => Promise.reject(new Error('down'))) };
  const props = { store, viewer, cesium, api, entitiesRef };
  const utils = render(<Harness {...props} plan={planRef.current} />);
  const marks = () => [...live].filter((e) => e.polygon?.hierarchy instanceof PolygonHierarchy).length;
  const trailPoints = () => {
    const pool = entitiesRef.current.trailMap[1];
    return pool ? pool.segs.flat().reduce((n, e) => n + e.polyline.positions.length / 3, 0) : 0;
  };
  const panel = (name) => {
    if (!utils.queryByTestId('trails-panel')) {
      fireEvent.click(utils.getByRole('button', { name: 'trailsPanel.title' }));
    }
    fireEvent.click(utils.getByRole('button', { name }));
  };
  const rerenderPlan = (plan) => utils.rerender(<Harness {...props} plan={plan} />);
  const showRecords = (records) => utils.rerender(<Harness {...props} plan={planRef.current} records={records} />);
  const approaches = () => [...live].filter((e) => e.properties?.closestApproachKey);
  return { store, utils, marks, trailPoints, panel, rerenderPlan, showRecords, approaches };
}

describe('Trails panel wiring into both history layers', () => {
  it('Pause in the panel stops trail and marks; Resume records again', () => {
    const t = setup();
    t.store.send(1, 44.00);
    t.store.send(1, 44.01);
    const [marks, points] = [t.marks(), t.trailPoints()];
    expect(marks).toBeGreaterThan(0);
    t.panel('trailsPanel.pause');
    t.store.send(1, 44.02);
    t.store.send(1, 44.03);
    expect(t.marks()).toBe(marks);
    expect(t.trailPoints()).toBe(points);
    t.panel('trailsPanel.resume');
    t.store.send(1, 44.04);
    t.store.send(1, 44.05);
    expect(t.marks()).toBeGreaterThan(marks);
    expect(t.trailPoints()).toBeGreaterThan(points);
  });

  it('Clear all while paused empties both layers and stays paused', () => {
    const t = setup();
    t.store.send(1, 44.00);
    t.store.send(1, 44.01);
    t.panel('trailsPanel.pause');
    t.panel('trailsPanel.clearAll');
    expect(t.marks()).toBe(0);
    expect(t.trailPoints()).toBe(0);
    t.store.send(1, 44.02);
    expect(t.marks()).toBe(0);
    expect(t.utils.getByTestId('trails-paused-badge')).toBeInTheDocument();
  });

  it('Restart clears and resumes even when the restart request fails', async () => {
    const t = setup();
    t.store.send(1, 44.00);
    t.store.send(1, 44.01);
    expect(t.trailPoints()).toBe(2);
    t.panel('trailsPanel.pause');
    await act(async () => { fireEvent.click(t.utils.getByRole('button', { name: 'restart' })); });
    expect(t.marks()).toBe(0);
    expect(t.trailPoints()).toBe(0);
    expect(t.utils.queryByTestId('trails-paused-badge')).toBeNull();
    t.store.send(1, 44.05);
    t.store.send(1, 44.06);
    expect(t.marks()).toBe(2);
    expect(t.trailPoints()).toBe(2);
  });

  it('the UAV toggle hides its closest-approach marks and Pause stops new ones', () => {
    const t = setup();
    t.store.send(1, 44.00);
    const record = (key) => ({
      key, sys_id: 1, outcome: 'missed', lat: 40, lon: 44, alt_rel_m: 41, d3_m: 2, h_m: 1, v_m: 1,
      dock_lat: 40, dock_lon: 44, dock_alt_rel_m: 40,
    });
    t.showRecords([]); // the first status after load is consumed unshown
    t.showRecords([record('b:1:1')]);
    expect(t.approaches().map((e) => e.show)).toEqual([true]);
    t.panel('UAV 1');
    expect(t.approaches().map((e) => e.show)).toEqual([false]);
    t.panel('UAV 1');
    t.panel('trailsPanel.pause');
    t.showRecords([record('b:1:1'), record('b:1:2')]);
    expect(t.approaches()).toHaveLength(1);
    t.panel('trailsPanel.resume');
    t.showRecords([record('b:1:1'), record('b:1:2'), record('b:1:3')]);
    expect(t.approaches()).toHaveLength(2);
  });

  it('a new plan object keeps the recorded marks', () => {
    const t = setup();
    t.store.send(1, 44.00);
    t.store.send(1, 44.01);
    const marks = t.marks();
    t.rerenderPlan({ altitude_m: 100, zones: [{ id: 'z1' }] });
    expect(t.marks()).toBe(marks);
  });
});
