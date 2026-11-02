import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { render, act, fireEvent, createEvent } from '@testing-library/react';
import TaskConfirmCard from './TaskConfirmCard';
import { PRESS_GUARD_MS, EXPIRY_OPEN_MS, comparePlace } from '../hooks/usePressGuard';
import { LONG_PRESS_MS } from '../utils/armAction';

// jsdom has no layout: the row's rectangle and the scroller are set by hand,
// and the frame loop runs when the test advances the faked clock.
const WAIT_SEC = 30;
const FRAME_MS = 16;
let rect;

beforeEach(() => {
  vi.useFakeTimers({
    toFake: ['Date', 'performance', 'requestAnimationFrame', 'cancelAnimationFrame', 'setInterval', 'clearInterval', 'setTimeout', 'clearTimeout'],
  });
  rect = { top: 400, left: 10, width: 300, height: 40 };
  vi.spyOn(Element.prototype, 'getBoundingClientRect').mockImplementation(() => rect);
});
afterEach(() => { vi.restoreAllMocks(); });

const advance = (ms) => act(() => { vi.advanceTimersByTime(ms); });

function entryFor(roundUid = 'a') {
  return {
    taskId: 7, roundUid, taskType: 'DOCK', lat: 32.5, lon: 34.8, alt: 100, imageB64: null,
    status: 'pending', action: null, receivedAt: Date.now(), decidedAt: null,
  };
}

function setup({ forced = false, autoApprove = false, scroller = null, entry = entryFor() } = {}) {
  const handlers = { onApprove: vi.fn(), onDeny: vi.fn(), onCancel: vi.fn() };
  const scrollRef = { current: scroller };
  const ui = (e = entry) => (
    <TaskConfirmCard
      sysId={1} entry={e} vehicleName="UAV 1" vehicleIndex={0}
      {...handlers} autoApprove={autoApprove} timeoutSec={WAIT_SEC} forced={forced} scrollRef={scrollRef}
    />
  );
  const view = render(ui());
  return { view, handlers, ui, approve: () => view.getByText(/task\.(approve|holdToApprove|pressAgain)/), deny: () => view.getAllByRole('button').at(-1) };
}

/** A real press: pointerdown then a click with detail 1. `at` overrides the input's time. */
function pointerDown(el, at = performance.now()) {
  const ev = createEvent.pointerDown(el, { bubbles: true });
  Object.defineProperty(ev, 'timeStamp', { value: at });
  fireEvent(el, ev);
}
const pointerClick = (el) => fireEvent.click(el, { detail: 1 });
function press(el, at) {
  pointerDown(el, at);
  pointerClick(el);
}

describe('a press after the buttons appear', () => {
  it('is refused inside the guard and works from its end, at the exact boundary', () => {
    const { handlers, approve } = setup();
    advance(PRESS_GUARD_MS - 1);
    press(approve());
    expect(handlers.onApprove).not.toHaveBeenCalled();
    advance(1);
    press(approve());
    expect(handlers.onApprove).toHaveBeenCalledTimes(1);
  });

  it('counts a new round on the same card as an appearance', () => {
    const { view, handlers, ui, approve } = setup();
    advance(PRESS_GUARD_MS * 2);
    view.rerender(ui(entryFor('b')));
    advance(FRAME_MS);
    press(approve());
    expect(handlers.onApprove).not.toHaveBeenCalled();
  });

  it('guards Deny the same way', () => {
    const { handlers, deny } = setup();
    press(deny());
    expect(handlers.onDeny).not.toHaveBeenCalled();
    advance(PRESS_GUARD_MS);
    press(deny());
    expect(handlers.onDeny).toHaveBeenCalledTimes(1);
  });
});

describe('a press after the row moves', () => {
  const settled = (opts) => { const s = setup(opts); advance(PRESS_GUARD_MS * 2); return s; };

  it('is refused inside the guard, then a deliberate press works', () => {
    const { handlers, approve } = settled();
    rect = { ...rect, top: rect.top - 36 };
    advance(FRAME_MS);
    press(approve());
    expect(handlers.onApprove).not.toHaveBeenCalled();
    advance(PRESS_GUARD_MS);
    press(approve());
    expect(handlers.onApprove).toHaveBeenCalledTimes(1);
  });

  it('sees a move at the press when no frame has run (frames suspended)', () => {
    const { handlers, approve } = settled();
    rect = { ...rect, top: rect.top - 57 };
    press(approve());
    expect(handlers.onApprove).not.toHaveBeenCalled();
  });

  it('uses the time of the input, so a late handler still counts the press as early', () => {
    const { handlers, approve } = settled();
    const before = performance.now();
    rect = { ...rect, top: rect.top - 57 };
    advance(PRESS_GUARD_MS * 2);
    press(approve(), before);
    expect(handlers.onApprove).not.toHaveBeenCalled();
  });

  it('refuses a press begun elsewhere at any age of the move', () => {
    const { view, handlers, approve } = settled();
    const elsewhere = view.getByText(/task\.title/);
    pointerDown(elsewhere);
    rect = { ...rect, top: rect.top - 57 };
    advance(PRESS_GUARD_MS * 4);
    pointerClick(approve());
    expect(handlers.onApprove).not.toHaveBeenCalled();
  });

  it('refuses a click whose pointerdown was on the other button', () => {
    const { handlers, approve, deny } = settled();
    pointerDown(deny());
    pointerClick(approve());
    expect(handlers.onApprove).not.toHaveBeenCalled();
  });

  it('refuses a press held across a move', () => {
    const { handlers, approve } = settled();
    pointerDown(approve());
    rect = { ...rect, top: rect.top - 57 };
    advance(PRESS_GUARD_MS * 2);
    pointerClick(approve());
    expect(handlers.onApprove).not.toHaveBeenCalled();
  });

  it('uses a press record once', () => {
    const { handlers, approve } = settled();
    press(approve());
    pointerClick(approve());
    expect(handlers.onApprove).toHaveBeenCalledTimes(1);
  });

  it('refuses after a cancelled press', () => {
    const { handlers, approve } = settled();
    pointerDown(approve());
    fireEvent.pointerCancel(approve());
    pointerClick(approve());
    expect(handlers.onApprove).not.toHaveBeenCalled();
  });

  it('dims the buttons while the cooldown runs and says press again after a refusal', () => {
    const { view, approve } = settled();
    const row = () => view.container.querySelector('[data-press-guard]');
    expect(row().dataset.pressGuard).toBe('off');
    expect(row().style.opacity).toBe('1');
    rect = { ...rect, top: rect.top - 57 };
    advance(FRAME_MS);
    expect(row().dataset.pressGuard).toBe('on');
    expect(row().style.opacity).toBe('0.6');
    expect(row().dataset.pressMoves).toBe('2');
    press(approve());
    expect(approve().textContent).toBe('task.pressAgain');
    advance(PRESS_GUARD_MS);
    expect(row().style.opacity).toBe('1');
    advance(2000);
    expect(approve().textContent).toBe('task.approve');
  });
});

describe('what is not a move', () => {
  const scroller = () => ({ scrollTop: 100, scrollHeight: 2000 });

  it('lets a press through right after the operator scrolls', () => {
    const list = scroller();
    const { handlers, approve } = setup({ scroller: list });
    advance(PRESS_GUARD_MS * 2);
    list.scrollTop += 200;
    rect = { ...rect, top: rect.top - 200 };
    advance(FRAME_MS);
    press(approve());
    expect(handlers.onApprove).toHaveBeenCalledTimes(1);
  });

  it('lets a press through when scroll anchoring holds the button still', () => {
    const list = scroller();
    const { handlers, approve } = setup({ scroller: list });
    advance(PRESS_GUARD_MS * 2);
    list.scrollTop -= 57;
    list.scrollHeight -= 57;
    advance(FRAME_MS);
    press(approve());
    expect(handlers.onApprove).toHaveBeenCalledTimes(1);
  });

  it('counts a clamped scroll as a move, when content below goes', () => {
    const list = { scrollTop: 300, scrollHeight: 2000 };
    const { handlers, approve } = setup({ scroller: list });
    advance(PRESS_GUARD_MS * 2);
    list.scrollTop = 200;
    list.scrollHeight = 1900;
    rect = { ...rect, top: rect.top + 100 };
    advance(FRAME_MS);
    press(approve());
    expect(handlers.onApprove).not.toHaveBeenCalled();
  });

  it('adds up slow drift in size against the baseline', () => {
    const base = { top: 100, left: 0, width: 10, height: 10, scrollTop: 0, scrollHeight: 0 };
    let state = base;
    let moved = false;
    for (let i = 1; i <= 3; i += 1) {
      const next = { ...base, height: 10 + 0.4 * i };
      ({ moved, base: state } = comparePlace(state, next));
    }
    expect(moved).toBe(true);
  });

  it('adds up slow vertical drift, through the card', () => {
    const list = scroller();
    const { handlers, approve } = setup({ scroller: list });
    advance(PRESS_GUARD_MS * 2);
    for (let i = 0; i < 3; i += 1) {
      rect = { ...rect, top: rect.top + 0.4 };
      advance(FRAME_MS);
    }
    press(approve());
    expect(handlers.onApprove).not.toHaveBeenCalled();
  });

  it('takes anchoring into the baseline, so a later scroll is still not a move', () => {
    const list = scroller();
    const { handlers, approve } = setup({ scroller: list });
    advance(PRESS_GUARD_MS * 2);
    list.scrollTop += 57;
    list.scrollHeight += 57;
    advance(FRAME_MS);
    list.scrollTop += 100;
    rect = { ...rect, top: rect.top - 100 };
    advance(FRAME_MS);
    press(approve());
    expect(handlers.onApprove).toHaveBeenCalledTimes(1);
  });
});

describe('what the guard never blocks', () => {
  it('lets a keyboard or assistive click through inside the guard', () => {
    const { handlers, approve, deny } = setup();
    fireEvent.click(approve(), { detail: 0 });
    expect(handlers.onApprove).toHaveBeenCalledTimes(1);
    fireEvent.click(deny(), { detail: 1, buttons: 1 });
    expect(handlers.onDeny).toHaveBeenCalledTimes(1);
  });

  it.each([[true, 'onApprove'], [false, 'onDeny']])('auto-fires on time, with the same action, inside the guard (auto-approve %s)', (autoApprove, fired) => {
    const { handlers } = setup({ autoApprove });
    advance((WAIT_SEC * 1000) - 1000 - FRAME_MS);
    rect = { ...rect, top: rect.top - 57 };
    advance(FRAME_MS);
    expect(handlers[fired]).not.toHaveBeenCalled();
    advance(1000);
    expect(handlers[fired]).toHaveBeenCalledTimes(1);
  });

  it.each([[true], [false]])('has no cooldown near expiry, for either button (auto-approve %s)', (autoApprove) => {
    const { handlers, approve, deny } = setup({ autoApprove });
    advance((WAIT_SEC * 1000) - EXPIRY_OPEN_MS);
    rect = { ...rect, top: rect.top - 57 };
    advance(FRAME_MS - 1);
    press(deny());
    expect(handlers.onDeny).toHaveBeenCalledTimes(1);
    press(approve());
    expect(handlers.onApprove).toHaveBeenCalledTimes(1);
  });

  it('keeps the cooldown just before the expiry window', () => {
    const { handlers, deny } = setup({ autoApprove: true });
    advance((WAIT_SEC * 1000) - EXPIRY_OPEN_MS - PRESS_GUARD_MS);
    rect = { ...rect, top: rect.top - 57 };
    advance(FRAME_MS);
    press(deny());
    expect(handlers.onDeny).not.toHaveBeenCalled();
  });

  it('still refuses a press begun elsewhere near expiry', () => {
    const { view, handlers, approve } = setup();
    advance((WAIT_SEC * 1000) - EXPIRY_OPEN_MS);
    pointerDown(view.getByText(/task\.title/));
    pointerClick(approve());
    expect(handlers.onApprove).not.toHaveBeenCalled();
  });

  it('replaces a stale press record: a drag-out then a press begun elsewhere near expiry', () => {
    const { view, handlers, approve } = setup();
    advance((WAIT_SEC * 1000) - EXPIRY_OPEN_MS);
    pointerDown(approve());
    pointerDown(view.getByText(/task\.title/));
    pointerClick(approve());
    expect(handlers.onApprove).not.toHaveBeenCalled();
  });

  it('does not dim a card near expiry', () => {
    const { view } = setup();
    advance((WAIT_SEC * 1000) - EXPIRY_OPEN_MS);
    rect = { ...rect, top: rect.top - 57 };
    advance(FRAME_MS);
    expect(view.container.querySelector('[data-press-guard]').dataset.pressGuard).toBe('off');
  });
});

describe('a forced card', () => {
  it('completes a hold started inside the guard and across a move', () => {
    const { handlers, approve } = setup({ forced: true });
    fireEvent.mouseDown(approve());
    rect = { ...rect, top: rect.top - 57 };
    advance(LONG_PRESS_MS + FRAME_MS);
    expect(handlers.onApprove).toHaveBeenCalledTimes(1);
  });

  it('completes a keyboard hold across a move', () => {
    const { handlers, approve } = setup({ forced: true });
    fireEvent.keyDown(approve(), { key: 'Enter' });
    rect = { ...rect, top: rect.top - 57 };
    advance(LONG_PRESS_MS + FRAME_MS);
    expect(handlers.onApprove).toHaveBeenCalledTimes(1);
  });

  it('ends a hold when the browser cancels the touch', () => {
    const { handlers, approve } = setup({ forced: true });
    fireEvent.touchStart(approve());
    advance(FRAME_MS);
    fireEvent.touchCancel(approve());
    advance(LONG_PRESS_MS + FRAME_MS);
    expect(handlers.onApprove).not.toHaveBeenCalled();
  });

  it('guards the Deny', () => {
    const { handlers, deny } = setup({ forced: true });
    press(deny());
    expect(handlers.onDeny).not.toHaveBeenCalled();
  });
});

describe('the frame loop', () => {
  const spyFrames = () => vi.spyOn(globalThis, 'requestAnimationFrame');

  it('stops when the card is decided and when it unmounts', () => {
    const spy = spyFrames();
    const { view, ui } = setup();
    advance(FRAME_MS * 3);
    expect(spy).toHaveBeenCalled();
    view.rerender(ui({ ...entryFor(), status: 'denied', decidedAt: Date.now() }));
    spy.mockClear();
    advance(FRAME_MS * 3);
    expect(spy).not.toHaveBeenCalled();
  });

  it('removes its press listeners on unmount', () => {
    const spy = vi.spyOn(document, 'removeEventListener');
    const { view } = setup();
    view.unmount();
    expect(spy.mock.calls.map(([type]) => type)).toEqual(expect.arrayContaining(['pointerdown', 'pointercancel']));
  });
});
