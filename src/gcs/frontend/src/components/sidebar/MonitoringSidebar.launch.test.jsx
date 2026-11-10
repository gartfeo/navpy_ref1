import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { useState } from 'react';
import { render, act, fireEvent, within, waitFor } from '@testing-library/react';
import MonitoringSidebar from './MonitoringSidebar';
import { LONG_PRESS_MS } from '../../utils/armAction';

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
