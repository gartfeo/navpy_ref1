import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, fireEvent, act } from '@testing-library/react';
import useLongPress from './useLongPress';
import { LONG_PRESS_MS } from '../utils/armAction';

// Minimal probe: wires the hook's handlers onto a real <button> so tests
// exercise the actual DOM event wiring (mouse / keyboard / blur), not just the
// handler functions in isolation.
function Probe({ onComplete, enabled = true, resetKey, duration = LONG_PRESS_MS }) {
  const { handlers } = useLongPress({ onComplete, enabled, resetKey, duration });
  return <button {...handlers}>press</button>;
}

// useLongPress schedules rAF recursively and measures elapsed with
// performance.now() (no setTimeout). Fake exactly those clock primitives;
// advancing the clock drives the ticks. Wrap advances in act() because each
// tick calls setProgress. The setup file restores real timers after each test.
beforeEach(() => {
  vi.useFakeTimers({ toFake: ['performance', 'requestAnimationFrame', 'cancelAnimationFrame'] });
});

const advance = (ms) => act(() => { vi.advanceTimersByTime(ms); });

describe('useLongPress', () => {
  // Sanity: the SAME driver the cancellation tests use completes an
  // uninterrupted hold. Without this, a broken driver would make every
  // "not called" assertion below pass vacuously.
  it('fires onComplete exactly once for an uninterrupted mouse hold', () => {
    const onComplete = vi.fn();
    const { getByRole } = render(<Probe onComplete={onComplete} />);
    fireEvent.mouseDown(getByRole('button'));
    advance(LONG_PRESS_MS + 32);
    expect(onComplete).toHaveBeenCalledTimes(1);
  });

  it('does not fire before the hold completes', () => {
    const onComplete = vi.fn();
    const { getByRole } = render(<Probe onComplete={onComplete} />);
    fireEvent.mouseDown(getByRole('button'));
    advance(LONG_PRESS_MS - 100);
    expect(onComplete).not.toHaveBeenCalled();
  });

  it('cancels the hold on mouseup', () => {
    const onComplete = vi.fn();
    const { getByRole } = render(<Probe onComplete={onComplete} />);
    const btn = getByRole('button');
    fireEvent.mouseDown(btn);
    advance(500);
    fireEvent.mouseUp(btn);
    advance(LONG_PRESS_MS);
    expect(onComplete).not.toHaveBeenCalled();
  });

  // #5a — enabled flips false mid-press aborts the running hold.
  it('cancels an in-flight hold when enabled flips false', () => {
    const onComplete = vi.fn();
    const { getByRole, rerender } = render(
      <Probe onComplete={onComplete} enabled resetKey="arm" />,
    );
    fireEvent.mouseDown(getByRole('button'));
    advance(500);
    rerender(<Probe onComplete={onComplete} enabled={false} resetKey="arm" />);
    advance(LONG_PRESS_MS);
    expect(onComplete).not.toHaveBeenCalled();
  });

  // #5b — resetKey (resolved hold mode) changing mid-press aborts the hold, so
  // an "arm" hold can never complete as a "force" launch (or vice versa).
  it('cancels an in-flight hold when resetKey changes', () => {
    const onComplete = vi.fn();
    const { getByRole, rerender } = render(
      <Probe onComplete={onComplete} enabled resetKey="arm" />,
    );
    fireEvent.mouseDown(getByRole('button'));
    advance(500);
    rerender(<Probe onComplete={onComplete} enabled resetKey="force" />);
    advance(LONG_PRESS_MS);
    expect(onComplete).not.toHaveBeenCalled();
  });

  // #6 — a held Enter/Space completes; preventDefault suppresses the native
  // button activation so a key press only ever fires via a completed hold.
  it.each(['Enter', ' '])('completes a held %j key and suppresses native activation', (key) => {
    const onComplete = vi.fn();
    const { getByRole } = render(<Probe onComplete={onComplete} />);
    const notPrevented = fireEvent.keyDown(getByRole('button'), { key });
    expect(notPrevented).toBe(false); // false => preventDefault was called
    advance(LONG_PRESS_MS + 32);
    expect(onComplete).toHaveBeenCalledTimes(1);
  });

  // #6 — a plain activation (keydown immediately released) must NOT fire.
  it.each(['Enter', ' '])('does not fire on a plain %j activation', (key) => {
    const onComplete = vi.fn();
    const { getByRole } = render(<Probe onComplete={onComplete} />);
    const btn = getByRole('button');
    fireEvent.keyDown(btn, { key });
    fireEvent.keyUp(btn, { key });
    advance(LONG_PRESS_MS + 32);
    expect(onComplete).not.toHaveBeenCalled();
  });

  it('ignores non-hold keys (no hold, no preventDefault)', () => {
    const onComplete = vi.fn();
    const { getByRole } = render(<Probe onComplete={onComplete} />);
    const notPrevented = fireEvent.keyDown(getByRole('button'), { key: 'a' });
    expect(notPrevented).toBe(true); // not a hold key => not prevented
    advance(LONG_PRESS_MS + 32);
    expect(onComplete).not.toHaveBeenCalled();
  });

  it('ignores auto-repeat keydown while a hold is running (fires once)', () => {
    const onComplete = vi.fn();
    const { getByRole } = render(<Probe onComplete={onComplete} />);
    const btn = getByRole('button');
    fireEvent.keyDown(btn, { key: 'Enter' });
    advance(500);
    fireEvent.keyDown(btn, { key: 'Enter' }); // auto-repeat must not restart the clock
    advance(600); // ~1100 ms after the first press
    expect(onComplete).toHaveBeenCalledTimes(1);
  });

  it('cancels the hold on blur', () => {
    const onComplete = vi.fn();
    const { getByRole } = render(<Probe onComplete={onComplete} />);
    const btn = getByRole('button');
    fireEvent.keyDown(btn, { key: 'Enter' });
    advance(500);
    fireEvent.blur(btn);
    advance(LONG_PRESS_MS);
    expect(onComplete).not.toHaveBeenCalled();
  });
});
