import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, act, fireEvent } from '@testing-library/react';
import { useState } from 'react';
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

  it('marks only a forced card gate-overridden', () => {
    const handlers = { onApprove: vi.fn(), onDeny: vi.fn(), onCancel: vi.fn() };
    const forced = renderCard(entryFor({ roundUid: 'f' }), handlers, { forced: true });
    expect(forced.getByText('task.gateOverridden')).toBeInTheDocument();
    forced.unmount();
    const normal = renderCard(entryFor({ roundUid: 'n' }), handlers);
    expect(normal.queryByText('task.gateOverridden')).toBeNull();
  });

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

describe('TaskConfirmCard View camera', () => {
  const handlers = () => ({ onApprove: vi.fn(), onDeny: vi.fn(), onCancel: vi.fn() });
  const card = (entry, h, videoAction) => (
    <TaskConfirmCard
      sysId={1} entry={entry} vehicleName="UAV 1" vehicleIndex={0}
      onApprove={h.onApprove} onDeny={h.onDeny} onCancel={h.onCancel}
      autoApprove={false} timeoutSec={TIMEOUT_SEC} videoAction={videoAction}
    />
  );

  it('is a neutral 44 px button on a pending card, with the countdown note', () => {
    const { getByText } = render(card(entryFor({ roundUid: 'a' }), handlers(), { viewing: false, onView: vi.fn() }));
    const button = getByText('task.viewCamera');
    expect(button.style.minHeight).toBe('44px');
    expect(button.style.background).not.toBe(getByText('task.approve').style.background);
    expect(getByText('task.viewCameraNote')).toBeInTheDocument();
    // Its row keeps a 12 px margin before the timer bar and Approve/Deny.
    expect(button.parentElement.style.marginBottom).toBe('12px');
  });

  it('shows "Viewing" when the panel already shows that UAV', () => {
    const { getByText } = render(card(entryFor({ roundUid: 'a' }), handlers(), { viewing: true, onView: vi.fn() }));
    expect(getByText('task.viewing')).toBeDisabled();
  });

  it('is absent on decided cards and when not offered (no stream, RC)', () => {
    const approved = { ...entryFor({ roundUid: 'a' }), status: 'approved' };
    const r1 = render(card(approved, handlers(), { viewing: false, onView: vi.fn() }));
    expect(r1.queryByText('task.viewCamera')).toBeNull();
    r1.unmount();
    const r2 = render(card(entryFor({ roundUid: 'a' }), handlers(), null));
    expect(r2.queryByText('task.viewCamera')).toBeNull();
  });

  it('tapped midway keeps the original deadline, calls neither approve nor deny, and expiry fires once', () => {
    const h = handlers();
    const onView = vi.fn();
    const { getByText } = render(card(entryFor({ roundUid: 'a' }), h, { viewing: false, onView }));
    advance(10_000);
    expect(getByText('20s')).toBeInTheDocument();
    fireEvent.click(getByText('task.viewCamera'));
    expect(onView).toHaveBeenCalledTimes(1);
    expect(h.onApprove).not.toHaveBeenCalled();
    expect(h.onDeny).not.toHaveBeenCalled();
    advance(5_000);
    expect(getByText('15s')).toBeInTheDocument();
    advance(16_000);
    expect(h.onDeny).toHaveBeenCalledTimes(1);
    expect(h.onApprove).not.toHaveBeenCalled();
  });

  it('the pending card survives leaving FPV through View camera', () => {
    const h = handlers();
    const entry = entryFor({ roundUid: 'a' });
    function Host() {
      const [fpvOn, setFpvOn] = useState(true);
      return (
        <div>
          <span>fpv:{String(fpvOn)}</span>
          {card(entry, h, { viewing: false, onView: () => setFpvOn(false) })}
        </div>
      );
    }
    const { getByText } = render(<Host />);
    advance(12_000);
    fireEvent.click(getByText('task.viewCamera'));
    expect(getByText('fpv:false')).toBeInTheDocument();
    expect(getByText('18s')).toBeInTheDocument();
    advance(19_000);
    expect(h.onDeny).toHaveBeenCalledTimes(1);
  });
});

describe('TaskConfirmCard position', () => {
  const handlers = () => ({ onApprove: vi.fn(), onDeny: vi.fn(), onCancel: vi.fn() });

  it('shows the coordinates and a map link when the companion sent a position', () => {
    const { getByText, queryByText } = renderCard(entryFor({ roundUid: '1:1' }), handlers());
    expect(getByText('task.map')).toBeInTheDocument();
    expect(queryByText('task.positionUnknown')).toBeNull();
  });

  it('says the position is unknown instead of a fake 0,0 when none was sent', () => {
    const entry = { ...entryFor({ roundUid: '1:1' }), lat: null, lon: null, alt: null };
    const { container, getByText, queryByText } = renderCard(entry, handlers());
    expect(getByText('task.positionUnknown')).toBeInTheDocument();
    expect(queryByText('task.map')).toBeNull();
    expect(container.textContent).not.toMatch(/0\.000000/);
    expect(container.textContent).not.toMatch(/— 0m/);
  });
});
