import { act, renderHook } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import useDisplaySettings from './useDisplaySettings';
import useVehicleCommands from './useVehicleCommands';

const api = { sendCommand: vi.fn(), restartMission: vi.fn(() => Promise.resolve()) };

/** The Trails panel state as App wires it: display settings feed the commands. */
function useWired() {
  const display = useDisplaySettings();
  const commands = useVehicleCommands({
    api,
    vehicleList: [],
    taskConfirmReset: () => {},
    taskAssignReset: () => {},
    setLaunchStates: () => {},
    resumeHistory: display.resumeHistory,
  });
  return { display, commands };
}

describe('trail and camera-mark history state', () => {
  it('hides by sys_id and remembers the choice', () => {
    const { result } = renderHook(useWired);
    act(() => result.current.display.toggleSysIdHidden(2));
    expect([...result.current.display.hiddenSysIds]).toEqual([2]);
    act(() => result.current.display.toggleSysIdHidden(2));
    expect(result.current.display.hiddenSysIds.size).toBe(0);
  });

  it('Clear all bumps one history key and keeps pause; Restart clears and resumes', () => {
    const { result } = renderHook(useWired);
    act(() => result.current.display.toggleHistoryPaused());
    expect(result.current.display.historyPausedAt).not.toBeNull();
    const key = result.current.commands.historyResetKey;
    act(() => result.current.commands.handleClearHistory());
    expect(result.current.commands.historyResetKey).toBe(key + 1);
    expect(result.current.display.historyPausedAt).not.toBeNull();
    const track = result.current.commands.trackResetKey;
    act(() => { result.current.commands.handleRestart([1]); });
    expect(result.current.commands.trackResetKey).toBe(track + 1);
    expect(result.current.display.historyPausedAt).toBeNull();
  });
});
