import { describe, it, expect, vi, afterEach } from 'vitest';
import { render, screen, act } from '@testing-library/react';
import useContainerLauncher from './useContainerLauncher';
import useMountSignal from './useMountSignal';

function stubStatus(reachable) {
  globalThis.fetch = vi.fn(() => Promise.resolve({
    ok: true, json: () => Promise.resolve({ reachable, simulator_running: reachable }),
  }));
}

afterEach(() => { delete globalThis.fetch; });

// Owner's rehearsal, 2026-11-09: HOLD TO LAUNCH, then LAUNCH, did nothing.
// doContainerLaunch opened the launch panel first; that unmounted the launch
// control, its unmount signal aborted, and handleLaunch, still awaiting the
// plan camera check, sent nothing. The harness keeps the real shape: the
// control renders only while no launch panel is open and passes its signal.
function LaunchHarness({ onLaunch, onTriggerVehicle, launchRef }) {
  const launcher = useContainerLauncher({
    isContainer: true, vehicleList: [{ sys_id: 1 }, { sys_id: 2 }], launchStates: {},
    onLaunch, onTriggerVehicle, settings: {},
  });
  launchRef.current = launcher.doContainerLaunch;
  return launcher.containerLaunching ? <div>launch panel</div> : <Control launchRef={launchRef} />;
}

function Control({ launchRef }) {
  const mountSignal = useMountSignal();
  launchRef.signal = mountSignal;
  return <button type="button">HOLD TO LAUNCH</button>;
}

function deferred() {
  let resolve;
  const promise = new Promise((r) => { resolve = r; });
  return { promise, resolve };
}

describe('useContainerLauncher launch', () => {
  it('opens the launch panel only after the session is prepared, so the control signal survives', async () => {
    stubStatus(true);
    const question = deferred();
    // As handleLaunch: await the camera question, send nothing once the signal aborted.
    const onLaunch = vi.fn(async (sysIds, opts) => {
      await question.promise;
      if (opts.signal?.aborted) return null;
      return { status: 'container_prepared', sys_ids: sysIds };
    });
    const onTriggerVehicle = vi.fn(async () => ({}));
    const launchRef = { current: null, signal: null };
    render(<LaunchHarness onLaunch={onLaunch} onTriggerVehicle={onTriggerVehicle} launchRef={launchRef} />);
    let done;
    act(() => { done = launchRef.current({ signal: launchRef.signal() }); });
    // The camera question is still open: the control is still on screen.
    expect(screen.getByText('HOLD TO LAUNCH')).toBeTruthy();
    await act(async () => { question.resolve(); await done; });
    expect(screen.getByText('launch panel')).toBeTruthy();
    expect(onTriggerVehicle.mock.calls.map((c) => c[0])).toEqual([1, 2]);
  });

  it('a refused launch leaves the control on screen and triggers nothing', async () => {
    stubStatus(true);
    const onLaunch = vi.fn(async () => null);
    const onTriggerVehicle = vi.fn();
    const launchRef = { current: null, signal: null };
    render(<LaunchHarness onLaunch={onLaunch} onTriggerVehicle={onTriggerVehicle} launchRef={launchRef} />);
    await act(async () => { await launchRef.current({ signal: launchRef.signal() }); });
    expect(screen.getByText('HOLD TO LAUNCH')).toBeTruthy();
    expect(onTriggerVehicle).not.toHaveBeenCalled();
  });

  it('a second launch while one is preparing is refused', async () => {
    stubStatus(true);
    const question = deferred();
    const onLaunch = vi.fn(async (sysIds) => {
      await question.promise;
      return { status: 'container_prepared', sys_ids: sysIds };
    });
    const launchRef = { current: null, signal: null };
    render(<LaunchHarness onLaunch={onLaunch} onTriggerVehicle={vi.fn(async () => ({}))} launchRef={launchRef} />);
    let first;
    act(() => { first = launchRef.current({ signal: launchRef.signal() }); });
    await act(async () => { await launchRef.current({ signal: launchRef.signal() }); });
    expect(onLaunch).toHaveBeenCalledTimes(1);
    await act(async () => { question.resolve(); await first; });
  });
});
