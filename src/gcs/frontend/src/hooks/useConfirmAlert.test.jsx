import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import useConfirmAlert, {
  alertPatternDue, ALERT_REPEAT_MS, URGENT_REPEAT_MS, URGENT_LEFT_SEC, ARRIVAL_GAP_MS,
} from './useConfirmAlert';
import { confirmRequest, confirmResponse, confirmRoundId } from './taskConfirmationState';

// Todo 67 (gcs_confirm_cue.md decision 2): the confirm sound's repeat and
// escalate policy, its stop conditions and the gesture unlock, on a fake
// AudioContext that records every oscillator start.

class FakeAudioContext {
  static instances = [];

  constructor() {
    this.state = 'suspended';
    this.currentTime = 0;
    this.destination = {};
    this.started = []; // { freq, at }
    this.stopped = 0;
    this.closed = false;
    this.listeners = {};
    FakeAudioContext.instances.push(this);
  }

  addEventListener(type, fn) { (this.listeners[type] ||= []).push(fn); }

  setState(state) {
    this.state = state;
    for (const fn of this.listeners.statechange || []) fn();
  }

  resume() { this.setState('running'); return Promise.resolve(); }

  close() { this.closed = true; this.setState('closed'); return Promise.resolve(); }

  createGain() { return { gain: { value: 0 }, connect() {} }; }

  createOscillator() {
    const ctx = this;
    const osc = {
      frequency: { value: 0 },
      connect() {},
      start(at) { ctx.started.push({ freq: osc.frequency.value, at }); },
      stop() { ctx.stopped += 1; },
    };
    return osc;
  }
}

const ctx = () => FakeAudioContext.instances.at(-1);
const freqs = () => (ctx() ? ctx().started.map((s) => s.freq) : []);
const WAIT = { 1: 30 };
const waitFor = (sid) => WAIT[sid];
const req = (sysId, taskId, t) => ({ sys_id: sysId, task_id: taskId, receivedAt: t, round_uid: `r${sysId}` });

function mount(state, fired = {}) {
  return renderHook(({ s, f }) => useConfirmAlert(s, waitFor, f), { initialProps: { s: state, f: fired } });
}

async function tap() {
  await act(async () => { window.dispatchEvent(new Event('pointerdown')); });
}

const advance = (ms) => act(() => { vi.advanceTimersByTime(ms); });

describe('alertPatternDue', () => {
  it('beeps every ALERT_REPEAT_MS while the soonest has more than URGENT_LEFT_SEC', () => {
    expect(alertPatternDue({ soonestLiveSec: 20, sinceLastMs: ALERT_REPEAT_MS })).toBe('beep');
    expect(alertPatternDue({ soonestLiveSec: 20, sinceLastMs: ALERT_REPEAT_MS - 1 })).toBeNull();
  });

  it('double-beeps every URGENT_REPEAT_MS at URGENT_LEFT_SEC or less', () => {
    expect(alertPatternDue({ soonestLiveSec: URGENT_LEFT_SEC, sinceLastMs: URGENT_REPEAT_MS })).toBe('urgent');
    expect(alertPatternDue({ soonestLiveSec: 1, sinceLastMs: URGENT_REPEAT_MS - 1 })).toBeNull();
  });

  it('beeps on arrival unless a sound played under ARRIVAL_GAP_MS ago; urgent wins', () => {
    expect(alertPatternDue({ soonestLiveSec: null, sinceLastMs: Infinity, arrival: true })).toBe('beep');
    expect(alertPatternDue({ soonestLiveSec: null, sinceLastMs: ARRIVAL_GAP_MS - 1, arrival: true })).toBeNull();
    expect(alertPatternDue({ soonestLiveSec: 5, sinceLastMs: URGENT_REPEAT_MS, arrival: true })).toBe('urgent');
  });

  it('is silent with no live round', () => {
    expect(alertPatternDue({ soonestLiveSec: null, sinceLastMs: Infinity })).toBeNull();
  });
});

describe('useConfirmAlert', () => {
  beforeEach(() => {
    FakeAudioContext.instances = [];
    vi.useFakeTimers({ toFake: ['Date', 'setInterval', 'clearInterval'] });
    vi.setSystemTime(0);
    window.AudioContext = FakeAudioContext;
  });

  afterEach(() => {
    vi.useRealTimers();
    delete window.AudioContext;
  });

  it('starts suspended and unlocks on the first pointerdown', async () => {
    const { result } = mount({});
    expect(result.current.soundState).toBe('suspended');
    await tap();
    expect(ctx().state).toBe('running');
    expect(result.current.soundState).toBe('running');
  });

  it('re-arms the gesture unlock when the context is suspended again', async () => {
    const { result } = mount({});
    await tap();
    act(() => { ctx().setState('suspended'); });
    expect(result.current.soundState).toBe('suspended');
    await tap();
    expect(result.current.soundState).toBe('running');
  });

  it('reports unavailable without an AudioContext and never throws', () => {
    delete window.AudioContext;
    const { result } = mount({});
    expect(result.current.soundState).toBe('unavailable');
    act(() => { result.current.onArrival(); result.current.unlockSound(); });
    expect(result.current.soundState).toBe('unavailable');
  });

  it('plays nothing while the browser refuses to resume, and queues nothing for later', async () => {
    let allowed = false;
    window.AudioContext = class extends FakeAudioContext {
      resume() { return allowed ? super.resume() : Promise.reject(new Error('no gesture')); }
    };
    const { result } = mount({});
    await act(async () => { result.current.onArrival(); });
    expect(result.current.soundState).toBe('suspended');
    expect(freqs()).toEqual([]);
    allowed = true;
    await tap();
    expect(result.current.soundState).toBe('running');
    expect(freqs()).toEqual([], 'the missed arrival beep is not replayed');
  });

  it('an arrival resumes a context the browser lets resume, then beeps', () => {
    const { result } = mount({});
    act(() => { result.current.onArrival(); });
    expect(freqs()).toEqual([880]);
  });

  it('beeps once per arrival burst', async () => {
    const { result } = mount({});
    await tap();
    act(() => { result.current.onArrival(); result.current.onArrival(); });
    expect(freqs()).toEqual([880]);
    advance(ARRIVAL_GAP_MS);
    act(() => { result.current.onArrival(); });
    expect(freqs()).toEqual([880, 880]);
  });

  it('repeats every 5 s while live, then double-beeps every 2 s at 10 s or less', async () => {
    const state = confirmRequest({}, req(1, 10, 0)); // NAV_CWT 30: deadline 30 s
    const { result, rerender } = mount({});
    await tap();
    rerender({ s: state, f: {} });
    act(() => { result.current.onArrival(); });
    expect(freqs()).toEqual([880]);
    advance(4000);
    expect(freqs()).toEqual([880]);
    advance(1000); // 5 s: 25 s left
    expect(freqs()).toEqual([880, 880]);
    advance(5000); // 10 s: 20 s left
    advance(5000); // 15 s: 15 s left
    expect(freqs()).toEqual([880, 880, 880, 880]);
    ctx().started.length = 0;
    advance(5000); // 20 s: 10 s left -> urgent
    expect(freqs()).toEqual([1320, 1320]);
    advance(1000);
    expect(freqs()).toEqual([1320, 1320]);
    advance(1000); // 2 s later
    expect(freqs()).toEqual([1320, 1320, 1320, 1320]);
  });

  it('stops when the round is decided', async () => {
    const state = confirmRequest({}, req(1, 10, 0));
    const { rerender } = mount(state);
    await tap();
    const decided = confirmResponse(state, { sys_id: 1, task_id: 10, is_confirmed: true, action: 'approve', round_uid: 'r1', at: 1 });
    rerender({ s: decided, f: {} });
    advance(30_000);
    expect(freqs()).toEqual([]);
  });

  it('a fired round is silent while its deadline arithmetic still has time', async () => {
    const state = confirmRequest({}, req(1, 10, 0)); // deadline 30 s
    const control = mount(state); // unfired twin: proves the window would sound
    const latched = mount(state, { [confirmRoundId(1, state[1])]: 1 });
    await tap();
    const [ctrlCtx, firedCtx] = FakeAudioContext.instances;
    advance(20_000);
    expect(ctrlCtx.started.length).toBeGreaterThan(0);
    expect(firedCtx.started).toEqual([]);
    control.unmount();
    latched.unmount();
  });

  it('a round that went due between renders stays silent', async () => {
    const { rerender } = mount(confirmRequest({}, req(1, 10, 0)));
    await tap();
    advance(30_000);
    ctx().started.length = 0;
    advance(10_000);
    expect(freqs()).toEqual([]);
    rerender({ s: {}, f: {} });
  });

  it('round, image and param updates do not push a due repeat back', async () => {
    let state = confirmRequest({}, req(1, 10, 0));
    const { result, rerender } = mount(state);
    await tap();
    act(() => { result.current.onArrival(1); });
    for (let t = 0; t < 5000; t += 500) {
      state = { ...state, 1: { ...state[1], imageB64: `img${t}` } }; // a new object every 0.5 s
      rerender({ s: state, f: {} });
      advance(500);
    }
    expect(freqs()).toEqual([880, 880]);
  });

  it('an arrival with 10 s or less plays the urgent pattern', async () => {
    WAIT[2] = 8;
    try {
      const { result } = mount({});
      await tap();
      act(() => { result.current.onArrival(2); });
      expect(freqs()).toEqual([1320, 1320]);
    } finally {
      delete WAIT[2];
    }
  });

  it("an arrival counts the other UAVs' live rounds: urgent when one is close", async () => {
    WAIT[3] = 30;
    try {
      const { result } = mount(confirmRequest({}, req(1, 10, 0))); // UAV 1: 30 s
      advance(22_000); // UAV 1 has 8 s left; sound still off, so nothing played
      await tap();
      act(() => { result.current.onArrival(3); }); // UAV 3 has 30 s
      expect(freqs()).toEqual([1320, 1320]);
    } finally {
      delete WAIT[3];
    }
  });

  it('stops when the rounds are cleared', async () => {
    const state = confirmRequest({}, req(1, 10, 0));
    const { rerender } = mount(state);
    await tap();
    rerender({ s: {}, f: {} });
    advance(30_000);
    expect(freqs()).toEqual([]);
  });

  it('stops the oscillators and closes the context on unmount', async () => {
    const { result, unmount } = mount({});
    await tap();
    act(() => { result.current.onArrival(); });
    unmount();
    expect(ctx().stopped).toBeGreaterThanOrEqual(2); // scheduled stop + unmount stop
    expect(ctx().closed).toBe(true);
  });
});
