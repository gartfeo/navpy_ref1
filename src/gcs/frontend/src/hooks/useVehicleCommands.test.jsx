import { act, renderHook } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import useVehicleCommands from './useVehicleCommands';

function setup(confirmBeforeStart) {
  const api = {
    launchSequence: vi.fn().mockResolvedValue({ status: 'ok' }),
    restartMission: vi.fn().mockResolvedValue({ status: 'ok' }),
  };
  const deps = {
    api, vehicleList: [{ sys_id: 1 }],
    taskConfirmReset: vi.fn(), taskAssignReset: vi.fn(), setLaunchStates: vi.fn(),
    confirmBeforeStart,
  };
  const { result } = renderHook(() => useVehicleCommands(deps));
  return { result, api, deps };
}

describe('camera check before a mission starts', () => {
  it.each([
    ['launch', { force: true }],
    ['launch', undefined],
    ['restart', undefined],
  ])('a cancelled %s sends nothing and resets nothing', async (kind, opts) => {
    const { result, api, deps } = setup(vi.fn().mockResolvedValue(false));
    const handler = kind === 'launch' ? result.current.handleLaunch : result.current.handleRestart;
    let out;
    await act(async () => { out = await handler([1], opts); });
    expect(out).toBeNull();
    expect(deps.confirmBeforeStart).toHaveBeenCalledOnce();
    expect(api.launchSequence).not.toHaveBeenCalled();
    expect(api.restartMission).not.toHaveBeenCalled();
    expect(deps.setLaunchStates).not.toHaveBeenCalled();
    expect(deps.taskConfirmReset).not.toHaveBeenCalled();
    expect(deps.taskAssignReset).not.toHaveBeenCalled();
  });

  it('an accepted launch and restart go ahead', async () => {
    const { result, api, deps } = setup(vi.fn().mockResolvedValue(true));
    await act(async () => { await result.current.handleLaunch([1], { force: true }); });
    await act(async () => { await result.current.handleRestart([1]); });
    expect(api.launchSequence).toHaveBeenCalledWith([1], { force: true });
    expect(api.restartMission).toHaveBeenCalledWith([1], undefined);
    expect(deps.setLaunchStates).toHaveBeenCalledWith({});
    expect(deps.taskConfirmReset).toHaveBeenCalledOnce();
  });
});
