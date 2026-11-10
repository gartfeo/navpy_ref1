import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { useState } from 'react';
import { render, act, fireEvent, within, waitFor } from '@testing-library/react';
import MonitoringSidebar from './MonitoringSidebar';
import ConfirmProvider from '../ConfirmProvider';
import { LONG_PRESS_MS } from '../../utils/armAction';
import { usePlanCameraGate } from '../../hooks/usePlanCameraCheck';
import useVehicleCommands from '../../hooks/useVehicleCommands';

// Container launch through the real sidebar and the real useContainerLauncher
// (MonitoringSidebar.test.jsx stands the launcher in). Owner's rehearsal,
// 2026-11-09: HOLD TO LAUNCH, then LAUNCH, sent nothing, because opening the
// launch panel unmounted the control whose signal the launch carries.
vi.mock('./VehicleStatusCard', () => ({ default: ({ vehicle }) => <div>card {vehicle.sys_id}</div> }));
vi.mock('../../utils/prearmChecks', async (importOriginal) => ({
  ...(await importOriginal()),
  computeLaunchReadiness: () => ({ ready: true, issues: [], advisories: [] }),
}));

const FLEET = [{ sys_id: 7, name: 's1-u7', armed: false, mission_total: 3 },
  { sys_id: 8, name: 's1-u8', armed: false, mission_total: 3 }];
const PLAN = { zones: [{ track: [] }] };
const SETTINGS = { launch: { launch_type: 'container' } };

function Host(props) {
  const [slot, setSlot] = useState(null);
  return (
    <>
      <MonitoringSidebar
        vehicleList={FLEET}
        plan={PLAN}
        navpyStatus={{}}
        launchStates={{}}
        settings={SETTINGS}
        onOpenReadiness={null}
        footerEl={slot}
        {...props}
      />
      <div data-testid="slot" ref={setSlot} />
    </>
  );
}

function deferred() {
  let resolve;
  const promise = new Promise((r) => { resolve = r; });
  return { promise, resolve };
}

const holdButton = (view) => within(view.getByTestId('slot'))
  .queryByRole('button', { name: 'monitor.holdToLaunch' });

/** Once the launcher status is read, hold HOLD TO LAUNCH, then press LAUNCH. */
async function holdAndLaunch(view) {
  await waitFor(() => expect(holdButton(view)).not.toBeNull());
  const button = holdButton(view);
  fireEvent.mouseDown(button);
  act(() => { vi.advanceTimersByTime(LONG_PRESS_MS + 32); });
  fireEvent.mouseUp(button);
  act(() => { fireEvent.click(view.getByRole('button', { name: 'launch.containerLaunchGo' })); });
}

beforeEach(() => {
  globalThis.fetch = vi.fn(() => Promise.resolve({
    ok: true, json: () => Promise.resolve({ reachable: true, simulator_running: true }),
  }));
  vi.useFakeTimers({ toFake: ['performance', 'requestAnimationFrame', 'cancelAnimationFrame'] });
});

afterEach(() => {
  vi.useRealTimers();
  delete globalThis.fetch;
});

describe('MonitoringSidebar container launch', () => {
  it('LAUNCH keeps HOLD TO LAUNCH until the session is prepared, then shows the panel before the first trigger', async () => {
    const question = deferred();
    // As handleLaunch: await the plan camera check, send nothing once the signal aborted.
    const onLaunch = vi.fn(async (sysIds, opts) => {
      await question.promise;
      if (opts.signal?.aborted) return null;
      return { status: 'container_prepared', sys_ids: sysIds };
    });
    const holdShownAtTrigger = [];
    const view = render(<Host onLaunch={onLaunch} onTriggerVehicle={vi.fn(async () => {
      holdShownAtTrigger.push(holdButton(view) !== null);
      return {};
    })} />);

    await holdAndLaunch(view);
    expect(onLaunch).toHaveBeenCalledTimes(1);
    expect(holdButton(view)).not.toBeNull();

    await act(async () => { question.resolve(); });
    await waitFor(() => expect(holdShownAtTrigger).toHaveLength(2));
    expect(holdShownAtTrigger).toEqual([false, false]);
    expect(holdButton(view)).toBeNull();
  });

  it('a session prepared after the sidebar left the screen triggers nothing', async () => {
    const question = deferred();
    // The prepare request already went out; its reply comes after the operator left.
    const onLaunch = vi.fn(async (sysIds) => {
      await question.promise;
      return { status: 'container_prepared', sys_ids: sysIds };
    });
    const onTriggerVehicle = vi.fn(async () => ({}));
    const view = render(<Host onLaunch={onLaunch} onTriggerVehicle={onTriggerVehicle} />);

    await holdAndLaunch(view);
    view.unmount();
    await act(async () => { question.resolve(); await question.promise; });
    await act(async () => { await Promise.resolve(); });
    expect(onTriggerVehicle).not.toHaveBeenCalled();
  });
});

// The demo-rehearsal path (2026-11-10): the real handleLaunch behind a real
// plan camera check that has docks to check, so the launch waits on the check
// request. The launch has to reach the prepare request (POST
// /api/control/launch, api.launchSequence) after the confirm closes.
const TRACK = [{ lat: 0, lon: 0 }, { lat: 0.001, lon: 0.001 }];
const DOCK_PLAN = { altitude_m: 200, zones: [{ track: TRACK }, { track: TRACK }] };
// targ_wps bit 1 = waypoint 2: one dock per zone, so the gate sends a check.
const TARG_WPS = { 7: 0b10, 8: 0b10 };
const getAas = (_sysId, name) => ({ targ_alt: 0, use_trn: false })[name];
const noop = () => {};

function WiredHost({ fleet, api }) {
  const confirmBeforeStart = usePlanCameraGate({
    enabled: true, plan: DOCK_PLAN, searchPattern: 'corridor', vehicleList: fleet,
    vehicleTargWps: TARG_WPS, vehicleNavLastWp: {}, getAas, check: api.checkPlanCamera,
  });
  const commands = useVehicleCommands({
    api, vehicleList: fleet,
    taskConfirmReset: noop, taskAssignReset: noop, setLaunchStates: noop, resumeHistory: noop,
    confirmBeforeStart,
  });
  return (
    <Host
      vehicleList={fleet}
      plan={DOCK_PLAN}
      onLaunch={commands.handleLaunch}
      onTriggerVehicle={commands.handleTriggerVehicle}
    />
  );
}

function wiredApi() {
  const check = deferred();
  return {
    check,
    checkPlanCamera: vi.fn(() => check.promise),
    launchSequence: vi.fn(async (sysIds) => ({ status: 'container_prepared', sys_ids: sysIds })),
    triggerVehicle: vi.fn(async () => ({})),
    sendCommand: vi.fn(),
  };
}

describe('MonitoringSidebar container launch with the real camera check', () => {
  it('a confirmed launch keeps its control mounted and reaches the prepare request', async () => {
    const api = wiredApi();
    const view = render(<ConfirmProvider><WiredHost fleet={FLEET} api={api} /></ConfirmProvider>);

    await waitFor(() => expect(holdButton(view)).not.toBeNull());
    const control = holdButton(view);
    await holdAndLaunch(view);
    // The check request is out; the control the launch's signal belongs to
    // must be the same element, not a remount.
    expect(api.checkPlanCamera).toHaveBeenCalledTimes(1);
    expect(api.checkPlanCamera.mock.calls[0][0].every((z) => z.docks.length === 1)).toBe(true);
    expect(holdButton(view)).toBe(control);

    await act(async () => { api.check.resolve({ status: 'ok', zones: [] }); });
    await waitFor(() => expect(api.triggerVehicle).toHaveBeenCalledTimes(2));
    expect(api.launchSequence).toHaveBeenCalledTimes(1);
    expect(api.launchSequence.mock.calls[0][0]).toEqual([7, 8]);
    expect(api.triggerVehicle.mock.calls.map((c) => c[0])).toEqual([7, 8]);
  });

  it('a control that really leaves PRE_LAUNCH during the check withdraws the launch', async () => {
    const api = wiredApi();
    const view = render(<ConfirmProvider><WiredHost fleet={FLEET} api={api} /></ConfirmProvider>);

    await holdAndLaunch(view);
    expect(api.checkPlanCamera).toHaveBeenCalledTimes(1);
    // A UAV arms: the phase goes to IN_FLIGHT and HOLD TO LAUNCH unmounts.
    const armed = FLEET.map((v, i) => (i === 0 ? { ...v, armed: true } : v));
    view.rerender(<ConfirmProvider><WiredHost fleet={armed} api={api} /></ConfirmProvider>);
    expect(holdButton(view)).toBeNull();

    await act(async () => { api.check.resolve({ status: 'ok', zones: [] }); });
    await act(async () => { await Promise.resolve(); });
    expect(api.launchSequence).not.toHaveBeenCalled();
    expect(api.triggerVehicle).not.toHaveBeenCalled();
  });
});
