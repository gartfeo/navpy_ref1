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

  // A system gesture (a palm, a notification shade) must not complete a hold.
  it('cancels the hold on touchcancel', () => {
    const onComplete = vi.fn();
    const { getByRole } = render(<Probe onComplete={onComplete} />);
    const btn = getByRole('button');
    fireEvent.touchStart(btn);
    advance(500);
    fireEvent.touchCancel(btn);
    advance(LONG_PRESS_MS);
    expect(onComplete).not.toHaveBeenCalled();
  });

  // The handheld panel collapses with display: none and stays mounted, so a
  // second finger on the toggle must not let a launch hold complete unseen.
  it('cancels the hold when the pressed element loses its layout box', () => {
    const onComplete = vi.fn();
    const { getByRole } = render(<Probe onComplete={onComplete} />);
    const btn = getByRole('button');
    let boxes = [{ width: 100, height: 32 }];
    btn.getClientRects = () => boxes;
    fireEvent.touchStart(btn);
    advance(500);
    boxes = [];
    advance(LONG_PRESS_MS);
    expect(onComplete).not.toHaveBeenCalled();
    // And it does not come back when the panel reopens: a fresh hold is needed.
    boxes = [{ width: 100, height: 32 }];
    advance(LONG_PRESS_MS);
    expect(onComplete).not.toHaveBeenCalled();
  });

  it('cancels a key hold when the pressed element loses its layout box', () => {
    const onComplete = vi.fn();
    const { getByRole } = render(<Probe onComplete={onComplete} />);
    const btn = getByRole('button');
    let boxes = [{ width: 100, height: 32 }];
    btn.getClientRects = () => boxes;
    fireEvent.keyDown(btn, { key: 'Enter' });
    advance(500);
    boxes = [];
    advance(LONG_PRESS_MS);
    expect(onComplete).not.toHaveBeenCalled();
  });

  it('completes while the pressed element keeps its layout box', () => {
    const onComplete = vi.fn();
    const { getByRole } = render(<Probe onComplete={onComplete} />);
    const btn = getByRole('button');
    btn.getClientRects = () => [{ width: 100, height: 32 }];
    fireEvent.mouseDown(btn);
    advance(LONG_PRESS_MS + 32);
    expect(onComplete).toHaveBeenCalledTimes(1);
  });

  // A confirm round arriving above the button moves it away from a still finger.
  // A touch gets no event for that, and the hold must not finish on an element
  // the operator no longer has under the finger.
  describe('when the pressed element moves', () => {
    const place = (btn, top) => {
      btn.getClientRects = () => [{ width: 100, height: 32 }];
      btn.getBoundingClientRect = () => ({ left: 0, right: 100, top, bottom: top + 32 });
    };
    const FINGER = 1;
    const at = (clientY) => ({ identifier: FINGER, clientX: 50, clientY });
    const AT_316 = at(316);
    const presses = [
      ['touch', (btn) => fireEvent.touchStart(btn, { touches: [AT_316], changedTouches: [AT_316] })],
      ['mouse', (btn) => fireEvent.mouseDown(btn, AT_316)],
    ];
    const moves = [
      ['touch', (btn, clientY) => {
        const point = at(clientY);
        fireEvent.touchStart(btn, { touches: [at(301)], changedTouches: [at(301)] });
        advance(100);
        fireEvent.touchMove(btn, { touches: [point], changedTouches: [point] });
      }],
      ['mouse', (btn, clientY) => {
        fireEvent.mouseDown(btn, at(301));
        advance(100);
        fireEvent.mouseMove(btn, at(clientY));
      }],
    ];
    const holdWith = () => {
      const onComplete = vi.fn();
      const { getByRole } = render(<Probe onComplete={onComplete} />);
      const btn = getByRole('button');
      place(btn, 300);
      return { onComplete, btn };
    };

    // RC: a thumb stays on a stick while the other hand holds a button.
    it('another finger already on the screen is not the press point', () => {
      const { onComplete, btn } = holdWith();
      const stick = { identifier: 2, clientX: 700, clientY: 500 };
      fireEvent.touchStart(btn, { touches: [stick, AT_316], changedTouches: [AT_316] });
      advance(LONG_PRESS_MS + 32);
      expect(onComplete).toHaveBeenCalledTimes(1);
    });

    it('another finger moving is not the press moving', () => {
      const { onComplete, btn } = holdWith();
      const stick = { identifier: 2, clientX: 700, clientY: 500 };
      fireEvent.touchStart(btn, { touches: [stick, AT_316], changedTouches: [AT_316] });
      advance(100);
      fireEvent.touchMove(btn, { touches: [stick, AT_316], changedTouches: [stick] });
      advance(LONG_PRESS_MS + 32);
      expect(onComplete).toHaveBeenCalledTimes(1);
    });

    // A finger drifts while it holds, so the check uses where it is, not where it began.
    it.each(moves)('completes a %s hold when the button moves onto where the finger drifted', (_name, drift) => {
      const { onComplete, btn } = holdWith();
      drift(btn, 325);
      place(btn, 310); // 310 to 342: no longer over the press at 301
      advance(LONG_PRESS_MS + 32);
      expect(onComplete).toHaveBeenCalledTimes(1);
    });

    it.each(moves)('cancels a %s hold when the button leaves where the finger drifted', (_name, drift) => {
      const { onComplete, btn } = holdWith();
      drift(btn, 325);
      place(btn, 270); // 270 to 302: still over the press at 301
      advance(LONG_PRESS_MS + 32);
      expect(onComplete).not.toHaveBeenCalled();
    });

    it('cancels a touch hold when the finger drifts off the button, and needs a fresh hold', () => {
      const { onComplete, btn } = holdWith();
      moves[0][1](btn, 345);
      advance(LONG_PRESS_MS + 32);
      expect(onComplete).not.toHaveBeenCalled();
      const back = at(316);
      fireEvent.touchMove(btn, { touches: [back], changedTouches: [back] });
      advance(LONG_PRESS_MS + 32);
      expect(onComplete).not.toHaveBeenCalled();
    });

    it('a finger within 8 px of the button still holds it', () => {
      const { onComplete, btn } = holdWith();
      moves[0][1](btn, 340); // the button ends at 332
      advance(LONG_PRESS_MS + 32);
      expect(onComplete).toHaveBeenCalledTimes(1);
    });

    it('a finger more than 8 px from the button no longer holds it', () => {
      const { onComplete, btn } = holdWith();
      moves[0][1](btn, 341);
      advance(LONG_PRESS_MS + 32);
      expect(onComplete).not.toHaveBeenCalled();
    });

    it('a mouse moving during a touch hold is not the press', () => {
      const { onComplete, btn } = holdWith();
      fireEvent.touchStart(btn, { touches: [AT_316], changedTouches: [AT_316] });
      advance(100);
      fireEvent.mouseMove(btn, { clientX: 50, clientY: 900 });
      advance(LONG_PRESS_MS + 32);
      expect(onComplete).toHaveBeenCalledTimes(1);
    });

    it('a touch moving during a mouse hold is not the press', () => {
      const { onComplete, btn } = holdWith();
      fireEvent.mouseDown(btn, AT_316);
      advance(100);
      fireEvent.touchMove(btn, { touches: [at(900)], changedTouches: [at(900)] });
      advance(LONG_PRESS_MS + 32);
      expect(onComplete).toHaveBeenCalledTimes(1);
    });

    it.each(presses)('cancels a %s hold when it leaves the press point, and needs a fresh hold', (_name, press) => {
      const onComplete = vi.fn();
      const { getByRole } = render(<Probe onComplete={onComplete} />);
      const btn = getByRole('button');
      place(btn, 300);
      press(btn);
      advance(500);
      place(btn, 520);
      advance(LONG_PRESS_MS);
      expect(onComplete).not.toHaveBeenCalled();
      // Moving back does not revive it.
      place(btn, 300);
      advance(LONG_PRESS_MS);
      expect(onComplete).not.toHaveBeenCalled();
    });

    it.each(presses)('completes a %s hold when it moves and stays under the press point', (_name, press) => {
      const onComplete = vi.fn();
      const { getByRole } = render(<Probe onComplete={onComplete} />);
      const btn = getByRole('button');
      place(btn, 300);
      press(btn);
      advance(500);
      place(btn, 290);
      advance(LONG_PRESS_MS);
      expect(onComplete).toHaveBeenCalledTimes(1);
    });

    it('a key hold has no press point: moving does not cancel it', () => {
      const onComplete = vi.fn();
      const { getByRole } = render(<Probe onComplete={onComplete} />);
      const btn = getByRole('button');
      place(btn, 300);
      fireEvent.keyDown(btn, { key: 'Enter' });
      advance(500);
      place(btn, 520);
      advance(LONG_PRESS_MS);
      expect(onComplete).toHaveBeenCalledTimes(1);
    });
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
