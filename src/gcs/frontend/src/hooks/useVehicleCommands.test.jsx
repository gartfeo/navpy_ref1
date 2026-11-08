import { act, renderHook } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import useVehicleCommands from './useVehicleCommands';
import { confirmStub } from '../test/confirmStub';

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

describe('set mode on all vehicles', () => {
  // Asked in-page (todo 233), never with the browser's blocking confirm.
  const run = async (answer) => {
    const native = vi.spyOn(window, 'confirm');
    const api = { sendCommand: vi.fn() };
    const { confirm, wrapper } = confirmStub(answer);
    const { result } = renderHook(() => useVehicleCommands({
      api, vehicleList: [{ sys_id: 1 }, { sys_id: 2 }],
      taskConfirmReset: vi.fn(), taskAssignReset: vi.fn(), setLaunchStates: vi.fn(),
    }), { wrapper });
    await act(async () => { await result.current.handleSetModeAll('RTL'); });
    expect(native).not.toHaveBeenCalled();
    return { api, confirm };
  };

  it('sends the mode to every vehicle after Confirm', async () => {
    const { api, confirm } = await run(true);
    expect(confirm).toHaveBeenCalledWith('Set ALL vehicles to RTL?');
    expect(api.sendCommand).toHaveBeenCalledWith('set_mode', [1, 2], { mode: 'RTL' });
  });

  it('sends nothing after Cancel', async () => {
    const { api } = await run(false);
    expect(api.sendCommand).not.toHaveBeenCalled();
  });
});

describe('a launch or start withdrawn with its control (opts.signal)', () => {
  // The camera question is asked by App for a Start or Launch button; when the
  // button leaves, its answer must not launch or start anything.
  it.each(['launch', 'restart'])('%s: an answer after the control is gone sends nothing', async (kind) => {
    const control = new AbortController();
    const gate = vi.fn(async () => { control.abort(); return true; });
    const { result, api, deps } = setup(gate);
    const handler = kind === 'launch' ? result.current.handleLaunch : result.current.handleRestart;
    let out;
    await act(async () => { out = await handler([1], { force: true, signal: control.signal }); });
    expect(gate).toHaveBeenCalledWith(control.signal);
    expect(out).toBeNull();
    expect(api.launchSequence).not.toHaveBeenCalled();
    expect(api.restartMission).not.toHaveBeenCalled();
    expect(deps.taskConfirmReset).not.toHaveBeenCalled();
  });

  it('the signal stays out of the request', async () => {
    const control = new AbortController();
    const { result, api } = setup(vi.fn().mockResolvedValue(true));
    await act(async () => { await result.current.handleLaunch([1], { force: true, signal: control.signal }); });
    expect(api.launchSequence).toHaveBeenCalledWith([1], { force: true });
  });
});
