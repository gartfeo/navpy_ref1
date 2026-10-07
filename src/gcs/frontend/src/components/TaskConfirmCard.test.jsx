import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, act, fireEvent } from '@testing-library/react';
import TaskConfirmCard from './TaskConfirmCard';
import { LONG_PRESS_MS } from '../utils/armAction';

// The card counts down with setInterval and measures elapsed with Date.now().
// Fake exactly those so advancing the clock drives the countdown; the setup
// file restores real timers after each test.
beforeEach(() => {
  vi.useFakeTimers({ toFake: ['Date', 'setInterval', 'clearInterval'] });
});

const advance = (ms) => act(() => { vi.advanceTimersByTime(ms); });

const TIMEOUT_SEC = 30;

function entryFor({ roundUid, taskId = 7 }) {
  return {
    taskId,
    roundUid,
    taskType: 'DOCK',
    lat: 32.5,
    lon: 34.8,
    alt: 100,
    imageB64: null,
    status: 'pending',
    action: null,
    receivedAt: Date.now(),
    decidedAt: null,
  };
}

function cardFor(entry, handlers, { forced = false } = {}) {
  return (
    <TaskConfirmCard
      sysId={1}
      entry={entry}
      vehicleName="UAV 1"
      vehicleIndex={0}
      onApprove={handlers.onApprove}
      onDeny={handlers.onDeny}
      onCancel={handlers.onCancel}
      autoApprove={false}
      timeoutSec={TIMEOUT_SEC}
      forced={forced}
    />
  );
}

function renderCard(entry, handlers, options) {
  return render(cardFor(entry, handlers, options));
}

describe('TaskConfirmCard auto-timeout', () => {
  // Sanity: the same driver the re-ask test uses does fire a single round's
  // timeout. Without this the assertions below could pass vacuously.
  it('fires the timeout action once when the countdown runs out', () => {
    const handlers = { onApprove: vi.fn(), onDeny: vi.fn(), onCancel: vi.fn() };
    renderCard(entryFor({ roundUid: '424242:11' }), handlers);

    advance((TIMEOUT_SEC + 1) * 1000);

    expect(handlers.onDeny).toHaveBeenCalledTimes(1);
    expect(handlers.onApprove).not.toHaveBeenCalled();
  });

  it('does not fire twice for the same round', () => {
    const handlers = { onApprove: vi.fn(), onDeny: vi.fn(), onCancel: vi.fn() };
    renderCard(entryFor({ roundUid: '424242:11' }), handlers);

    advance((TIMEOUT_SEC + 30) * 1000);

    expect(handlers.onDeny).toHaveBeenCalledTimes(1);
  });

  it('fires again for a D-14 re-ask of the same task without a remount', () => {
    // The re-ask carries the same (sysId, taskId) and only a new round uid,
    // so a card keyed by sysId-taskId alone is NOT remounted: the same
    // component instance must still arm a fresh timeout for the new round,
    // or the operator is left with a card that never resolves itself.
    const handlers = { onApprove: vi.fn(), onDeny: vi.fn(), onCancel: vi.fn() };
    const { rerender } = renderCard(entryFor({ roundUid: '424242:11' }), handlers);

    advance((TIMEOUT_SEC + 1) * 1000);
    expect(handlers.onDeny).toHaveBeenCalledTimes(1);

    // Round B for the same POI: fresh uid, fresh receivedAt, pending again.
    const roundB = entryFor({ roundUid: '424242:12' });
    rerender(cardFor(roundB, handlers));

    advance((TIMEOUT_SEC + 1) * 1000);

    expect(handlers.onDeny).toHaveBeenCalledTimes(2);
  });

  it('does not re-arm while the round uid is unchanged', () => {
    const handlers = { onApprove: vi.fn(), onDeny: vi.fn(), onCancel: vi.fn() };
    const entry = entryFor({ roundUid: '424242:11' });
    const { rerender } = renderCard(entry, handlers);

    advance((TIMEOUT_SEC + 1) * 1000);
    expect(handlers.onDeny).toHaveBeenCalledTimes(1);

    // A rerender that is NOT a new round (same uid) must not re-arm.
    rerender(cardFor({ ...entry }, handlers));
    advance((TIMEOUT_SEC + 1) * 1000);

    expect(handlers.onDeny).toHaveBeenCalledTimes(1);
  });
});

// CONF-03 (D-19): a forced (gate-overridden) popup approves only on a
// press-and-hold. The hold measures elapsed time with performance.now() and
// ticks on rAF, so those are the clock primitives to fake here.
describe('TaskConfirmCard forced press-and-hold', () => {
  beforeEach(() => {
    vi.useFakeTimers({ toFake: ['Date', 'performance', 'requestAnimationFrame', 'cancelAnimationFrame'] });
  });

  const holdButton = (view) => view.getByRole('button', { name: 'task.holdToApprove' });

  it('approves after an uninterrupted hold on one round', () => {
    const handlers = { onApprove: vi.fn(), onDeny: vi.fn(), onCancel: vi.fn() };
    const view = renderCard(entryFor({ roundUid: '424242:11' }), handlers, { forced: true });

    fireEvent.mouseDown(holdButton(view));
    advance(LONG_PRESS_MS + 32);

    expect(handlers.onApprove).toHaveBeenCalledTimes(1);
  });

  it('does not let a hold started on one round approve the next one', () => {
    // Without a per-round reset the operator's in-progress hold keeps running
    // across a re-ask and approves a POI they were never shown -- the exact
    // accidental approve the press-and-hold exists to prevent.
    const handlers = { onApprove: vi.fn(), onDeny: vi.fn(), onCancel: vi.fn() };
    const view = renderCard(entryFor({ roundUid: '424242:11' }), handlers, { forced: true });

    fireEvent.mouseDown(holdButton(view));
    advance(LONG_PRESS_MS - 200);
    expect(handlers.onApprove).not.toHaveBeenCalled();

    // Round B arrives mid-hold (same POI, new round).
    view.rerender(cardFor(entryFor({ roundUid: '424242:12' }), handlers, { forced: true }));
    advance(LONG_PRESS_MS);

    expect(handlers.onApprove).not.toHaveBeenCalled();
  });
});
