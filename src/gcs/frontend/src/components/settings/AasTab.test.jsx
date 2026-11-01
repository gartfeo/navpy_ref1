/**
 * AasTab's two mission download paths: the automatic one on open and the
 * waypoint editor's "Download missions". The backend answers a vehicle with no
 * mission with 200 and an empty waypoint list, and a failed download with 404.
 * An empty answer removes the vehicle's entry; a failed one keeps it.
 */
import { act, fireEvent, render, screen } from '@testing-library/react';
import { useState } from 'react';
import { afterEach, describe, expect, it, vi } from 'vitest';
import AasTab from './AasTab.jsx';

// The editor is only the place the download button lives; it is not under test.
vi.mock('./WaypointEditorOverlay.jsx', () => ({
  default: ({ onDownload }) => <button onClick={onDownload}>download missions</button>,
}));

const VEHICLES = [
  { sys_id: 1, name: 'UAV1' },
  { sys_id: 2, name: 'UAV2' },
  { sys_id: 3, name: 'UAV3' },
];
const track = (lat) => [{ lat, lon: 34.0, alt: 100 }];
const OLD = { 1: track(31.1), 2: track(31.2), 3: track(31.3) };

const EMPTY = { status: 200, body: { sys_id: 1, mission_count: 0, waypoints: [] } };
const FAILED = { status: 404, body: { detail: 'Mission download failed' } };
const mission = (lat) => ({ status: 200, body: { sys_id: 3, mission_count: 4, waypoints: track(lat) } });

afterEach(() => vi.unstubAllGlobals());

function answerMissions(bySysId) {
  vi.stubGlobal('fetch', vi.fn(async (url) => {
    const answer = bySysId[Number(String(url).match(/vehicles\/(\d+)\/mission/)[1])];
    return { ok: answer.status === 200, status: answer.status, json: async () => answer.body };
  }));
}

function renderTab(initialMissions) {
  const published = [];
  const aasParams = {
    confirmedByVehicle: { 1: {}, 2: {}, 3: {} },
    draftByVehicle: { 1: {}, 2: {}, 3: {} },
    setVehicleDraft: vi.fn(),
    refreshVehicles: vi.fn().mockResolvedValue(undefined),
    uploadDraft: vi.fn(),
    resetDraft: vi.fn(),
  };
  function Harness() {
    const [missions, setMissions] = useState(initialMissions);
    return (
      <AasTab
        vehicleList={VEHICLES}
        onOpenConnect={vi.fn()}
        onClose={vi.fn()}
        vehicleMissions={missions}
        setVehicleMissions={(next) => { published.push(next); setMissions(next); }}
        simMode
        aasParams={aasParams}
      />
    );
  }
  render(<Harness />);
  return published;
}

describe('AasTab mission download', () => {
  it('on open, removes an empty vehicle and keeps a failed one', async () => {
    answerMissions({ 1: EMPTY, 2: FAILED, 3: mission(32.3) });
    const published = renderTab(OLD);
    await act(async () => {});

    expect(published).toHaveLength(1);
    expect(published[0]).toEqual({ 2: OLD[2], 3: track(32.3) });
  });

  it('from the editor, removes an empty vehicle and keeps a failed one', async () => {
    // The open download fails everywhere, so every old entry is still there.
    answerMissions({ 1: FAILED, 2: FAILED, 3: FAILED });
    const published = renderTab({ ...OLD, 9: track(39.9) });
    await act(async () => {});
    expect(published[0]).toEqual({ ...OLD, 9: track(39.9) });

    answerMissions({ 1: EMPTY, 2: FAILED, 3: mission(32.3) });
    vi.spyOn(window, 'confirm').mockReturnValue(true);
    fireEvent.click(screen.getByText('vehicle.editPois'));
    await act(async () => { fireEvent.click(screen.getByText('download missions')); });

    // Only the listed vehicles: 9 is not one of them.
    expect(published.at(-1)).toEqual({ 2: OLD[2], 3: track(32.3) });
  });
});
