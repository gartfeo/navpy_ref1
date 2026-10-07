import { afterEach, describe, expect, it, vi } from 'vitest';
import { act, renderHook } from '@testing-library/react';
import usePlanPersistence from './usePlanPersistence';

// Exercise the real file reader/writer with in-memory browser I/O.
// No settings are persisted and no vehicle/network calls are made.
afterEach(() => vi.unstubAllGlobals());

function fixture(overrides = {}) {
  const props = {
    polygon: [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }, { lat: 1, lon: 0 }],
    launchPoint: null, corridorPoints: [], searchPattern: 'distributed',
    setLaunchPoints: [], setCorridorPointsArr: [[]],
    settings: { default_delivery_hubs: [{ name: 'Fixture', type: 'other', lat: 0, lon: 0 }] },
    deliveryHubAssignments: [0], simDockWps: { 0: [2] }, detectAfterWps: { 0: 1 },
    drawing: { loadVertices: vi.fn() }, undoRef: { current: [] },
    suppressRegenRef: { current: false }, ...overrides,
  };
  for (const key of ['setPolygon','setSearchPattern',
    'setLaunchPoint','setCorridorPoints','setSetLaunchPoints','setSetCorridorPoints',
    'setActiveSetIndex','setAnalysis','setPlan','localAnalyze','setDeliveryHubAssignments',
    'setSimDockWps','setDetectAfterWps','handleSaveSettings',
    'setFenceCustomVertices','setExclusionPolygons','setFenceEnabled','setFenceTouched','setFenceOffsetM',
    'authorFenceIntent','clearFenceIntent','resetFenceObservations']) props[key] = vi.fn();
  let input;
  let written;
  let contents;
  const create = document.createElement.bind(document);
  vi.spyOn(document, 'createElement').mockImplementation((tag, ...args) => {
    const element = create(tag, ...args);
    if (tag === 'input') input = element;
    if (tag === 'input' || tag === 'a') element.click = vi.fn();
    return element;
  });
  vi.stubGlobal('Blob', class { constructor(parts) { written = parts.join(''); } });
  vi.stubGlobal('URL', { createObjectURL: () => 'blob:fixture', revokeObjectURL: vi.fn() });
  vi.stubGlobal('FileReader', class {
    readAsText() { this.onload({ target: { result: contents } }); }
  });
  const { result } = renderHook(() => usePlanPersistence(props));
  return {
    props,
    save() { act(() => result.current.handleSavePolygon()); return JSON.parse(written); },
    load(data) {
      this.loadText(JSON.stringify(data));
    },
    loadText(text) {
      contents = text;
      act(() => {
        result.current.handleLoadPolygon();
        input.onchange({ target: { files: [{}] } });
      });
    },
  };
}

describe('current development plan files', () => {
  it('neither requires nor reads dock class selections: every zone targets the dock', () => {
    const alert = vi.spyOn(window, 'alert').mockImplementation(() => {});
    const io = fixture();
    io.load({ polygon: io.props.polygon, dock_classes: ['small'], per_uav_dock_classes: { 0: ['large'] } });
    expect(alert).not.toHaveBeenCalled();
    expect(io.props.setPolygon).toHaveBeenCalledWith(io.props.polygon);
    expect(io.props.localAnalyze).toHaveBeenCalledWith(io.props.polygon);
  });

  it('rejects malformed JSON before changing state', () => {
    const alert = vi.spyOn(window, 'alert').mockImplementation(() => {});
    const io = fixture();
    io.loadText('{');
    for (const value of Object.values(io.props)) {
      if (vi.isMockFunction(value)) expect(value).not.toHaveBeenCalled();
    }
    expect(io.props.drawing.loadVertices).not.toHaveBeenCalled();
    expect(alert).toHaveBeenCalledWith('planFile.invalidData');
  });

  it('accepts corridor plans with an empty polygon', () => {
    const io = fixture();
    io.load({ polygon: [], search_pattern: 'corridor',
      set_launch_points: [{ lat: 1, lon: 2 }], set_corridors: [[]] });
    expect(io.props.setSetLaunchPoints).toHaveBeenCalledWith([{ lat: 1, lon: 2 }]);
    expect(io.props.localAnalyze).not.toHaveBeenCalled();
  });

  it.each(['distributed', 'corridor'])('round-trips the current %s search pattern through the actual hook', (searchPattern) => {
    const io = fixture({ searchPattern, launchPoint: { lat: 1, lon: 2 },
      setLaunchPoints: [{ lat: 1, lon: 2 }, null],
      setCorridorPointsArr: [[{ lat: 2, lon: 3 }], []] });
    const data = io.save();
    expect(data.search_pattern).toBe(searchPattern);
    expect(Object.hasOwn(data, 'tactic')).toBe(false);
    expect(Object.hasOwn(data, 'searchPattern')).toBe(false);
    expect(data.default_delivery_hubs).toEqual(io.props.settings.default_delivery_hubs);
    for (const key of ['dock_classes','per_uav_dock_classes','poi_classes','per_uav_poi_classes','objects_of_interest',
      'ooi_assignments','sim_poi_wps','plan_format_version','launch_point','corridor']) {
      expect(Object.hasOwn(data, key)).toBe(false);
    }
    io.load(data);
    expect(io.props.setSearchPattern).toHaveBeenCalledWith(data.search_pattern);
    expect(io.props.handleSaveSettings).toHaveBeenCalledWith({ default_delivery_hubs: data.default_delivery_hubs });
    expect(io.props.setDeliveryHubAssignments).toHaveBeenCalledWith(data.delivery_hub_assignments);
    expect(io.props.setSimDockWps).toHaveBeenCalledWith(data.sim_dock_wps);
    expect(io.props.setDetectAfterWps).toHaveBeenCalledWith(data.detect_after_wps);
    expect(io.props.setSetLaunchPoints).toHaveBeenCalledWith(data.set_launch_points);
    expect(io.props.setSetCorridorPoints).toHaveBeenCalledWith(data.set_corridors);
  });

  it.each([null, 7, [], { polygon: [], poi_classes: ['old'] }, {}, { polygon: [], delivery_docks: [] }, { polygon: [], delivery_dock_assignments: [] },
    { polygon: [], tactic: 'corridor' },
    { polygon: [], tactic: 'corridor', search_pattern: 'distributed' },
  ].map(data => [data]))(
    'rejects unsupported structure %j before any state changes', (data) => {
      const alert = vi.spyOn(window, 'alert').mockImplementation(() => {});
      const io = fixture();
      io.props.undoRef.current = ['existing-undo'];
      io.load(data);
      for (const value of Object.values(io.props)) {
        if (vi.isMockFunction(value)) expect(value).not.toHaveBeenCalled();
      }
      expect(io.props.drawing.loadVertices).not.toHaveBeenCalled();
      expect(io.props.undoRef.current).toEqual(['existing-undo']);
      expect(alert).toHaveBeenCalledWith('planFile.invalidData');
    },
  );

  it('preserves existing empty optional-array behavior', () => {
    const io = fixture();
    io.load({ polygon: io.props.polygon, default_delivery_hubs: [], delivery_hub_assignments: [] });
    expect(io.props.handleSaveSettings).not.toHaveBeenCalled();
    expect(io.props.setDeliveryHubAssignments).not.toHaveBeenCalled();
    expect(io.props.setSimDockWps).not.toHaveBeenCalled();
    expect(io.props.setDetectAfterWps).not.toHaveBeenCalled();
  });

  it.each([{}, null])('omits empty optional simulation fields %j when saving', (value) => {
    const data = fixture({ simDockWps: value, detectAfterWps: value }).save();
    expect(Object.hasOwn(data, 'sim_dock_wps')).toBe(false);
    expect(Object.hasOwn(data, 'detect_after_wps')).toBe(false);
  });

  it('saves no fence block when only the vehicles were observed', () => {
    // Observations live outside the plan file: an unknown/mixed download is not
    // an operator decision, so the saved plan must carry no fence intent.
    const data = fixture({ fenceEnabled: false, fenceTouched: false, fenceIntent: null }).save();
    expect(Object.hasOwn(data, 'fence')).toBe(false);
  });

  it('saves a pending fence request even after the toggle returned to off', () => {
    const data = fixture({
      fenceEnabled: false, fenceTouched: true, fenceIntent: { generation: 2, enabled: false },
    }).save();
    expect(data.fence).toMatchObject({ enabled: false });
  });

  it('loads a legacy saved fence as authored intent, not as observation', () => {
    const io = fixture();
    io.load({ polygon: io.props.polygon, fence: { enabled: false, offset_m: 40 } });
    expect(io.props.authorFenceIntent).toHaveBeenCalledWith(false);
    expect(io.props.setFenceOffsetM).toHaveBeenCalledWith(40);
  });

  it('invalidates earlier vehicle readback when a replacement plan loads', () => {
    const io = fixture();
    io.load({ polygon: io.props.polygon });
    expect(io.props.resetFenceObservations).toHaveBeenCalled();
  });

  it('drops the previous plan\'s pending fence request when the loaded plan has none', () => {
    // A plan file with no fence block carries no fence decision. The request
    // still pending from the plan being replaced belongs to THAT plan; leaving
    // it in place would send the discarded plan's fence on the next upload.
    const io = fixture({
      fenceEnabled: true, fenceTouched: true, fenceIntent: { generation: 5, enabled: false },
    });
    io.load({ polygon: io.props.polygon });
    expect(io.props.clearFenceIntent).toHaveBeenCalled();
    expect(io.props.authorFenceIntent).not.toHaveBeenCalled();
    expect(io.props.setFenceEnabled).toHaveBeenCalledWith(false);
    expect(io.props.setFenceTouched).toHaveBeenCalledWith(false);
  });

  it('keeps the loaded plan\'s own fence decision instead of clearing it', () => {
    const io = fixture({ fenceIntent: { generation: 5, enabled: false } });
    io.load({ polygon: io.props.polygon, fence: { enabled: true } });
    expect(io.props.authorFenceIntent).toHaveBeenCalledWith(true);
    expect(io.props.clearFenceIntent).not.toHaveBeenCalled();
  });

  it('loads a simulation waypoint selection without changing omitted detection selection', () => {
    const io = fixture();
    io.load({ polygon: io.props.polygon, sim_dock_wps: { 0: 7 } });
    expect(io.props.setSimDockWps).toHaveBeenCalledWith({ 0: 7 });
    expect(io.props.setDetectAfterWps).not.toHaveBeenCalled();
  });
});
