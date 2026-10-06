import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, fireEvent, act } from '@testing-library/react';
import { ForceStartButton } from './LaunchControls';
import { LONG_PRESS_MS } from '../../utils/armAction';

// ForceStartButton drives the real useLongPress; fake the clock primitives it
// relies on so a completed hold is deterministic. The setup file restores real
// timers and spies after each test.
beforeEach(() => {
  vi.useFakeTimers({ toFake: ['performance', 'requestAnimationFrame', 'cancelAnimationFrame'] });
});

const advance = (ms) => act(() => { vi.advanceTimersByTime(ms); });
const renderBtn = (props) => render(<ForceStartButton label="START" disabled={false} {...props} />);

describe('ForceStartButton — bungee (armReady false)', () => {
  // #4 — the bungee START MISSION button keeps single-click behaviour.
  it('fires onStart on a single click when ready', () => {
    const onStart = vi.fn();
    const { getByRole } = renderBtn({ ready: true, onStart });
    fireEvent.click(getByRole('button'));
    expect(onStart).toHaveBeenCalledTimes(1);
  });
});

describe('ForceStartButton — container arm mode (armReady + ready)', () => {
  // #1 (button level) — a single click must NOT arm or launch; a container
  // launch can never start on one stray click.
  it('does nothing on a single click (no onArm, no onStart)', () => {
    const onStart = vi.fn();
    const onArm = vi.fn();
    const { getByRole } = renderBtn({ armReady: true, ready: true, onStart, onArm });
    fireEvent.click(getByRole('button'));
    expect(onArm).not.toHaveBeenCalled();
    expect(onStart).not.toHaveBeenCalled();
  });

  // #2 (button level) — a completed hold requests confirmation via onArm and
  // never launches directly.
  it('calls onArm exactly once after a completed hold (never onStart)', () => {
    const onStart = vi.fn();
    const onArm = vi.fn();
    const { getByRole } = renderBtn({ armReady: true, ready: true, onStart, onArm });
    fireEvent.mouseDown(getByRole('button'));
    advance(LONG_PRESS_MS + 32);
    expect(onArm).toHaveBeenCalledTimes(1);
    expect(onStart).not.toHaveBeenCalled();
  });
});

describe('ForceStartButton — not-ready force path', () => {
  // #3 — a single click is inert; no confirm, no launch.
  it('does not confirm or launch on a single click', () => {
    const onStart = vi.fn();
    const confirmSpy = vi.spyOn(window, 'confirm').mockReturnValue(true);
    const { getByRole } = renderBtn({ ready: false, onStart });
    fireEvent.click(getByRole('button'));
    expect(confirmSpy).not.toHaveBeenCalled();
    expect(onStart).not.toHaveBeenCalled();
  });

  // #3 — a completed hold prompts window.confirm; accepting force-launches once.
  it('force-launches once after a completed hold when confirm is accepted', () => {
    const onStart = vi.fn();
    const onArm = vi.fn();
    const confirmSpy = vi.spyOn(window, 'confirm').mockReturnValue(true);
    const { getByRole } = renderBtn({ ready: false, onStart, onArm });
    fireEvent.mouseDown(getByRole('button'));
    advance(LONG_PRESS_MS + 32);
    expect(confirmSpy).toHaveBeenCalledTimes(1);
    expect(onStart).toHaveBeenCalledTimes(1);
    expect(onStart).toHaveBeenCalledWith({ force: true });
    expect(onArm).not.toHaveBeenCalled();
  });

  // #3 — declining the confirm cancels the launch.
  it('does not launch when confirm is declined', () => {
    const onStart = vi.fn();
    const confirmSpy = vi.spyOn(window, 'confirm').mockReturnValue(false);
    const { getByRole } = renderBtn({ ready: false, onStart });
    fireEvent.mouseDown(getByRole('button'));
    advance(LONG_PRESS_MS + 32);
    expect(confirmSpy).toHaveBeenCalledTimes(1);
    expect(onStart).not.toHaveBeenCalled();
  });
});
