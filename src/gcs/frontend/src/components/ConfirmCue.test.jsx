import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, act, fireEvent } from '@testing-library/react';
import { useCallback } from 'react';
import ConfirmCardList from './ConfirmCardList';
import TaskConfirmCard from './TaskConfirmCard';
import useTaskConfirmation from '../hooks/useTaskConfirmation';
import { CONFIRM_PHASE } from '../hooks/taskConfirmationState';

// Todos 67 and 68 (gcs_confirm_cue.md): the pending-confirm chip, the cards
// ordered by time left, and the words for what the GCS sends at zero. The
// i18n stub returns keys, so times are read from data-time.

beforeEach(() => {
  vi.useFakeTimers({ toFake: ['Date', 'setInterval', 'clearInterval'] });
  vi.setSystemTime(1_000_000);
});

const advance = (ms) => act(() => { vi.advanceTimersByTime(ms); });

const FLEET = [
  { sys_id: 1, name: 'U1' },
  { sys_id: 2, name: 'U2' },
  { sys_id: 3, name: 'U3' },
];

/** Cached params like useAasParams: `params[sid][key]`, absent = not loaded. */
function aasFrom(params) {
  return {
    getConfirmedValue: (sid, key, fallback) => {
      const v = params[sid]?.[key];
      return v === undefined ? fallback : v;
    },
  };
}

/** App's wiring: the hook reads the raw wait, both hosts get its cue. */
function Harness({ aas, control }) {
  const waitSecFor = useCallback((sid) => aas.getConfirmedValue(sid, 'nav_cwt', undefined), [aas]);
  const tc = useTaskConfirmation({ waitSecFor });
  control.current = tc;
  return (
    <ConfirmCardList
      pendingConfirms={tc.pendingConfirms}
      vehicleList={FLEET}
      onApprove={tc.approve}
      onDeny={tc.deny}
      onCancel={tc.cancel}
      forcedConfirms={tc.forcedConfirms}
      aasParams={aas}
      cue={tc.cue}
      style={{ height: '100%' }}
    />
  );
}

function setup(params) {
  const control = { current: null };
  const view = render(<Harness aas={aasFrom(params)} control={control} />);
  return { view, control };
}

function ask(control, sysId, taskId = sysId * 10) {
  act(() => {
    control.current.handleConfirmRequest({ sys_id: sysId, task_id: taskId, round_uid: `r${sysId}`, lat: 1, lon: 2, alt: 3 });
  });
}

function respond(control, sysId, action) {
  act(() => {
    control.current.handleConfirmResponse({
      sys_id: sysId, task_id: sysId * 10, round_uid: `r${sysId}`, is_confirmed: action === 'approve', action,
    });
  });
}

const chip = (view) => view.queryByTestId('confirm-summary');
const chipTime = (view) => view.getByTestId('confirm-summary-time').textContent;
const cardTime = (view, sid) => view.getByTestId(`confirm-countdown-${sid}`).dataset.time;
const cardLabel = (view, sid) => view.getByTestId(`confirm-countdown-${sid}`).dataset.label;
const cardOrder = (view) => [...view.container.querySelectorAll('[data-testid^="confirm-countdown-"]')]
  .map((el) => Number(el.dataset.testid.replace('confirm-countdown-', '')));

const PARAMS = {
  1: { nav_cwt: 30, nav_cm_fl: false },
  2: { nav_cwt: 20, nav_cm_fl: true },
  3: { nav_cwt: 10, nav_cm_fl: false },
};

describe('pending-confirm chip and order (todo 67)', () => {
  it('orders the cards by time left in the host, not by arrival', () => {
    const { view, control } = setup(PARAMS);
    ask(control, 1);
    advance(5000);
    ask(control, 3); // NAV_CWT 10, 5 s later: deadline 15 s vs UAV 1's 30 s
    expect(cardOrder(view)).toEqual([3, 1]);
    ask(control, 2); // NAV_CWT 20 from 5 s: deadline 25 s
    expect(cardOrder(view)).toEqual([3, 2, 1]);
  });

  it('renders, ticks each second, and shows the same whole second as the card', () => {
    const { view, control } = setup(PARAMS);
    ask(control, 1);
    advance(300);
    ask(control, 3); // off the whole-second grid: rounding must agree
    const c = chip(view);
    expect(c).not.toBeNull();
    expect(c.dataset.shown).toBe('3');
    expect(view.getByTestId('confirm-summary-more').textContent).toBe('+1');
    // jsdom has no AudioContext
    expect(view.getByTestId('confirm-summary-words').textContent).toBe('task.noSound');
    vi.spyOn(globalThis, 'fetch').mockReturnValue(new Promise(() => {}));
    const shown = [];
    for (let i = 0; i < 14; i += 1) {
      shown.push(chip(view).dataset.shown);
      expect(chipTime(view)).toBe(cardTime(view, chip(view).dataset.shown));
      advance(1000);
    }
    expect(shown[0]).toBe('3');
    expect(shown.at(-1)).toBe('1'); // UAV 3 fired at its zero; the chip moved on
  });

  it('a new round never shows more than its wait (list clock up to a tick old)', () => {
    const { view, control } = setup(PARAMS);
    ask(control, 1);
    advance(700);
    ask(control, 3);
    expect(cardTime(view, 3)).toBe('0:10');
    expect(chipTime(view)).toBe('0:10');
  });

  it('a round after a quiet gap starts at its wait (list clock frozen while none pending)', () => {
    const { view, control } = setup(PARAMS);
    ask(control, 3);
    respond(control, 3, 'deny');
    advance(228_000);
    // every value the countdown and the chip take, including the first paint
    // before the list's resample effect runs
    const seen = new Set();
    const record = () => {
      for (const el of view.container.querySelectorAll('[data-testid="confirm-countdown-3"]')) seen.add(el.dataset.time);
      const t = view.queryByTestId('confirm-summary-time');
      if (t) seen.add(t.textContent);
    };
    const keep = (records) => {
      for (const m of records) {
        // a value overwritten before anyone read it is still in oldValue
        if (m.attributeName === 'data-time' && m.oldValue != null) seen.add(m.oldValue);
      }
      record();
    };
    const observer = new MutationObserver(keep);
    observer.observe(view.container, {
      subtree: true, childList: true, attributes: true, attributeOldValue: true, attributeFilter: ['data-time'],
    });
    ask(control, 3, 31);
    keep(observer.takeRecords());
    observer.disconnect();
    expect([...seen]).toEqual(['0:10']);
  });

  it('follows the clock after a backward step (no receipt floor)', () => {
    const { view, control } = setup(PARAMS);
    ask(control, 1); // 30 s
    advance(20_000);
    ask(control, 3); // 10 s, received 20 s later
    vi.setSystemTime(Date.now() - 19_000); // the wall clock steps back 19 s
    advance(1000);
    // UAV 1: deadline 30 s, the clock now reads 2 s after its receipt
    expect(cardTime(view, 1)).toBe('0:28');
    expect(chipTime(view)).toBe(cardTime(view, chip(view).dataset.shown));
  });

  it('counts down once per second', () => {
    const { view, control } = setup(PARAMS);
    ask(control, 3);
    expect(chipTime(view)).toBe('0:10');
    advance(1000);
    expect(chipTime(view)).toBe('0:09');
    advance(1000);
    expect(chipTime(view)).toBe('0:08');
  });

  it('a tap scrolls the shown card to the top of the scrolling part and never decides', () => {
    const fetchSpy = vi.spyOn(globalThis, 'fetch').mockResolvedValue({ ok: true, status: 200 });
    const { view, control } = setup(PARAMS);
    ask(control, 1);
    ask(control, 3);
    const scroll = view.getByTestId('confirm-scroll');
    const soonest = view.getByTestId('confirm-countdown-3');
    // the scrolling part's top is at 100 px; the soonest card's wrapper at 340
    scroll.getBoundingClientRect = () => ({ top: 100 });
    for (const el of scroll.querySelectorAll('div')) {
      el.getBoundingClientRect = () => ({ top: el.contains(soonest) ? 340 : 0 });
    }
    scroll.scrollTop = 0;
    fireEvent.click(chip(view));
    expect(scroll.scrollTop).toBe(240);
    expect(fetchSpy).not.toHaveBeenCalled();
  });

  it('disappears when no round is pending', () => {
    const { view, control } = setup(PARAMS);
    ask(control, 1);
    ask(control, 3);
    respond(control, 3, 'deny');
    expect(chip(view).dataset.shown).toBe('1');
    expect(view.queryByTestId('confirm-summary-more')).toBeNull();
    respond(control, 1, 'approve');
    expect(chip(view)).toBeNull();
  });

  it('a fired round stays "time up" while its answer is in flight; the chip moves on', () => {
    // the auto-deny POST never completes: the round stays pending
    vi.spyOn(globalThis, 'fetch').mockReturnValue(new Promise(() => {}));
    const { view, control } = setup(PARAMS);
    ask(control, 1);
    ask(control, 3);
    advance(11_000);
    expect(cardLabel(view, 3)).toBe('task.zero.fired');
    expect(cardOrder(view)).toEqual([3, 1]); // fired rounds first
    expect(chip(view).dataset.shown).toBe('1');
    expect(cardLabel(view, 1)).toBe('task.zero.autoDeny');
  });
});

describe('what the GCS sends at zero (todo 68)', () => {
  it('names the action from the cached policy', () => {
    const { view, control } = setup({ ...PARAMS, 2: {} });
    ask(control, 1);
    ask(control, 3);
    ask(control, 2);
    expect(cardLabel(view, 1)).toBe('task.zero.autoDeny');
    expect(cardTime(view, 1)).toBe('0:30');
    expect(cardLabel(view, 2)).toBe('task.zero.autoDenyUnknown');
    expect(cardTime(view, 2)).toBe('~0:30');
  });

  it('says auto-approve when NAV_CM_FL is on', () => {
    const { view, control } = setup(PARAMS);
    ask(control, 2);
    expect(cardLabel(view, 2)).toBe('task.zero.autoApprove');
  });

  it('says the sound is off until a tap unlocks it', () => {
    window.AudioContext = class { constructor() { this.state = 'suspended'; } addEventListener() {} };
    try {
      const { view, control } = setup(PARAMS);
      ask(control, 2);
      expect(view.getByTestId('confirm-summary-words').textContent).toBe('task.soundOff');
    } finally {
      delete window.AudioContext;
    }
  });

  it('uses the chip words once the sound is on', async () => {
    const { view, control } = setup(PARAMS);
    window.AudioContext = class {
      constructor() { this.state = 'suspended'; this.currentTime = 0; this.destination = {}; }
      addEventListener() {}
      resume() { this.state = 'running'; return Promise.resolve(); }
      createOscillator() { return { frequency: {}, connect() {}, start() {}, stop() {} }; }
      createGain() { return { gain: {}, connect() {} }; }
      close() {}
    };
    try {
      ask(control, 2);
      await act(async () => { fireEvent.click(chip(view)); });
      expect(view.getByTestId('confirm-summary-words').textContent).toBe('task.zeroChip.autoApprove');
    } finally {
      delete window.AudioContext;
    }
  });
});

describe('TaskConfirmCard labels', () => {
  const entry = () => ({
    taskId: 7, roundUid: 'a', taskType: 'DOCK', lat: 1, lon: 2, alt: 3, imageB64: null,
    status: 'pending', action: null, receivedAt: Date.now(), decidedAt: null,
  });
  const card = (props) => (
    <TaskConfirmCard
      sysId={1} entry={props.entry ?? entry()} vehicleName="UAV 1" vehicleIndex={0}
      onApprove={vi.fn()} onDeny={vi.fn()} onCancel={vi.fn()}
      autoApprove={false} timeoutSec={30} {...props}
    />
  );
  const label = (view) => view.getByTestId('confirm-countdown-1').dataset;

  it('defaults to its own auto-fire props: "Auto-deny in 0:30"', () => {
    const view = render(card({}));
    expect(label(view).label).toBe('task.zero.autoDeny');
    expect(label(view).time).toBe('0:30');
  });

  it('a zero wait says the GCS answers now, until it has fired', () => {
    const view = render(card({ timeoutSec: 0, labelPolicy: { cmFl: true, cwt: 0 } }));
    expect(label(view).label).toBe('task.zero.noWaitApprove');
    view.rerender(card({ timeoutSec: 0, labelPolicy: { cmFl: true, cwt: 0 }, fired: true }));
    expect(label(view).label).toBe('task.zero.fired');
  });

  it('at zero, before firing, says the answer is being sent', () => {
    const e = entry();
    const view = render(card({ entry: e, now: e.receivedAt + 30_000 }));
    expect(label(view).label).toBe('task.zero.due');
  });

  it('latches fired from its own auto-fire, once, with the round key', () => {
    const onAutoFire = vi.fn();
    const onDeny = vi.fn();
    const view = render(card({ onAutoFire, onDeny }));
    advance(31_000);
    expect(onDeny).toHaveBeenCalledTimes(1);
    expect(onAutoFire).toHaveBeenCalledTimes(1);
    expect(onAutoFire.mock.calls[0][0]).toBe(1);
    view.rerender(card({ onAutoFire, onDeny, fired: true }));
    expect(label(view).label).toBe('task.zero.fired');
    advance(5000);
    expect(onAutoFire).toHaveBeenCalledTimes(1);
  });

  it('a fired round shows 0:00 even if a longer wait or a clock step back gives it time', () => {
    const e = entry();
    const view = render(card({ entry: e, fired: true, now: e.receivedAt + 5000, labelPolicy: { cmFl: false, cwt: 60 } }));
    expect(label(view).label).toBe('task.zero.fired');
    expect(label(view).time).toBe('0:00');
  });

  it('leaving the list keeps the right time (no stale local state)', () => {
    const e = entry();
    const view = render(card({ entry: e, now: e.receivedAt }));
    advance(10_000);
    view.rerender(card({ entry: e, now: e.receivedAt + 10_000 }));
    expect(label(view).time).toBe('0:20');
    view.rerender(card({ entry: e }));
    expect(label(view).time).toBe('0:20');
  });

  it('the timer bar follows the list clock while hosted', () => {
    const e = entry();
    const view = render(card({ entry: e, now: e.receivedAt }));
    const bar = () => [...view.container.querySelectorAll('div')]
      .find((el) => el.style.transition.startsWith('width'));
    expect(bar().style.width).toBe('100%');
    advance(15_000);
    view.rerender(card({ entry: e, now: e.receivedAt + 15_000 }));
    expect(bar().style.width).toBe('50%');
  });

  it('exposes the phase values the list uses', () => {
    expect(CONFIRM_PHASE).toEqual({ LIVE: 'live', DUE: 'due', FIRED: 'fired' });
  });
});
