import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, act, fireEvent } from '@testing-library/react';
import { useCallback } from 'react';
import TaskConfirmCard from './TaskConfirmCard';
import ConfirmCardList from './ConfirmCardList';
import useTaskConfirmation from '../hooks/useTaskConfirmation';
import {
  isStandingConfirm, confirmZeroLabel, pendingSummary, CONFIRM_PHASE, HOME_DOCK_TASK_TYPE,
} from '../hooks/taskConfirmationState';

// Owner ruling 2026-11-09: a request to park at the home dock (the DDH,
// task_type "HOME_DOCK") is a standing round: Approve only, no countdown, no
// auto-fire, no Cancel task after approval. Delivery-dock rounds ("DOCK")
// keep the countdown, the auto-fire, Deny and Cancel task. The i18n stub
// returns keys.

beforeEach(() => {
  vi.useFakeTimers({ toFake: ['Date', 'setInterval', 'clearInterval'] });
  vi.setSystemTime(1_000_000);
});

const advance = (ms) => act(() => { vi.advanceTimersByTime(ms); });

const TIMEOUT_SEC = 30;
const PAST_TIMEOUT_MS = (TIMEOUT_SEC + 30) * 1000;

function entryFor(taskType, status = 'pending') {
  return {
    taskId: 7, roundUid: 'r7', taskType, lat: 32.5, lon: 34.8, alt: 100, imageB64: null,
    status, action: status === 'approved' ? 'approve' : null,
    receivedAt: Date.now(), decidedAt: status === 'pending' ? null : Date.now(),
  };
}

function renderCard(entry, { autoApprove = false } = {}) {
  const handlers = { onApprove: vi.fn(), onDeny: vi.fn(), onCancel: vi.fn() };
  const view = render(
    <TaskConfirmCard
      sysId={1} entry={entry} vehicleName="UAV 1" vehicleIndex={0}
      onApprove={handlers.onApprove} onDeny={handlers.onDeny} onCancel={handlers.onCancel}
      autoApprove={autoApprove} timeoutSec={TIMEOUT_SEC}
    />,
  );
  return { view, handlers };
}

describe('isStandingConfirm', () => {
  it('is true only for the home dock task type', () => {
    expect(isStandingConfirm({ taskType: HOME_DOCK_TASK_TYPE })).toBe(true);
    expect(isStandingConfirm({ taskType: 'DOCK' })).toBe(false);
    expect(isStandingConfirm({ taskType: 'UNKNOWN' })).toBe(false);
    expect(isStandingConfirm(null)).toBe(false);
  });
});

describe('home dock card (standing)', () => {
  it('shows Approve only, the standing label and note, and no countdown', () => {
    const { view } = renderCard(entryFor('HOME_DOCK'));
    expect(view.getByText('task.approve')).toBeTruthy();
    expect(view.queryByText('task.deny')).toBeNull();
    expect(view.queryByTestId('confirm-countdown-1')).toBeNull();
    expect(view.getByTestId('confirm-standing-1').textContent).toBe('task.standing.title');
    expect(view.getByTestId('confirm-standing-note-1').textContent).toBe('task.standing.note');
  });

  it.each([false, true])('never auto-fires past the delivery countdown (autoApprove=%s)', (autoApprove) => {
    const { view, handlers } = renderCard(entryFor('HOME_DOCK'), { autoApprove });
    advance(PAST_TIMEOUT_MS);
    expect(handlers.onDeny).not.toHaveBeenCalled();
    expect(handlers.onApprove).not.toHaveBeenCalled();
    expect(view.getByText('task.approve')).toBeTruthy();
  });

  it('approves on click', () => {
    const { view, handlers } = renderCard(entryFor('HOME_DOCK'));
    fireEvent.click(view.getByText('task.approve'));
    expect(handlers.onApprove).toHaveBeenCalledWith(1);
  });

  it('has no Cancel task once approved', () => {
    const { view } = renderCard(entryFor('HOME_DOCK', 'approved'));
    expect(view.getByText('task.approvalSent')).toBeTruthy();
    expect(view.queryByText('task.cancelTask')).toBeNull();
  });
});

describe('delivery dock card (unchanged)', () => {
  it('has Deny and a countdown, and auto-fires at zero', () => {
    const { view, handlers } = renderCard(entryFor('DOCK'));
    expect(view.getByText('task.deny')).toBeTruthy();
    expect(view.getByTestId('confirm-countdown-1')).toBeTruthy();
    expect(view.queryByTestId('confirm-standing-1')).toBeNull();
    advance(PAST_TIMEOUT_MS);
    expect(handlers.onDeny).toHaveBeenCalledTimes(1);
  });

  it('has Cancel task once approved', () => {
    const { view } = renderCard(entryFor('DOCK', 'approved'));
    expect(view.getByText('task.cancelTask')).toBeTruthy();
  });
});

const FLEET = [{ sys_id: 1, name: 'U1' }, { sys_id: 2, name: 'U2' }];
const AAS = {
  getConfirmedValue: (sid, key, fallback) => ({ nav_cwt: TIMEOUT_SEC, nav_cm_fl: false }[key] ?? fallback),
};

function Harness({ control }) {
  const waitSecFor = useCallback((sid) => AAS.getConfirmedValue(sid, 'nav_cwt', undefined), []);
  const tc = useTaskConfirmation({ waitSecFor });
  control.current = tc;
  return (
    <ConfirmCardList
      pendingConfirms={tc.pendingConfirms} vehicleList={FLEET}
      onApprove={tc.approve} onDeny={tc.deny} onCancel={tc.cancel}
      forcedConfirms={tc.forcedConfirms} aasParams={AAS} cue={tc.cue}
    />
  );
}

function setup() {
  const control = { current: null };
  const view = render(<Harness control={control} />);
  return { view, control };
}

function ask(control, sysId, taskType) {
  act(() => {
    control.current.handleConfirmRequest({
      sys_id: sysId, task_id: 7, round_uid: `r${sysId}`, task_type: taskType, lat: 1, lon: 2, alt: 3,
    });
  });
}

const posted = (fetchSpy) => fetchSpy.mock.calls
  .filter(([url]) => url === '/api/control/task_confirm')
  .map(([, init]) => JSON.parse(init.body));

describe('useTaskConfirmation with a home dock round', () => {
  it('refuses deny and timeout actions: no request is sent', async () => {
    const fetchSpy = vi.spyOn(globalThis, 'fetch').mockResolvedValue({ ok: true, status: 200 });
    const { control } = setup();
    ask(control, 1, 'HOME_DOCK');
    await act(async () => { await control.current.deny(1); });
    advance(PAST_TIMEOUT_MS);
    await act(async () => {});
    expect(posted(fetchSpy)).toEqual([]);
    expect(control.current.pendingConfirms[1].status).toBe('pending');
  });

  it('sends approve, then refuses cancel', async () => {
    const fetchSpy = vi.spyOn(globalThis, 'fetch').mockResolvedValue({ ok: true, status: 200 });
    const { view, control } = setup();
    ask(control, 1, 'HOME_DOCK');
    await act(async () => { fireEvent.click(view.getByText('task.approve')); });
    expect(posted(fetchSpy).map((b) => b.action)).toEqual(['approve']);
    expect(control.current.pendingConfirms[1].status).toBe('approved');
    expect(view.queryByText('task.cancelTask')).toBeNull();
    await act(async () => { await control.current.cancel(1); });
    expect(posted(fetchSpy).map((b) => b.action)).toEqual(['approve']);
    expect(control.current.pendingConfirms[1].status).toBe('approved');
  });

  it('still sends deny and cancel for a delivery dock round', async () => {
    const fetchSpy = vi.spyOn(globalThis, 'fetch').mockResolvedValue({ ok: true, status: 200 });
    const { control } = setup();
    ask(control, 1, 'DOCK');
    await act(async () => { await control.current.deny(1); });
    ask(control, 2, 'DOCK');
    await act(async () => { await control.current.approve(2); });
    await act(async () => { await control.current.cancel(2); });
    expect(posted(fetchSpy).map((b) => [b.sys_id, b.action])).toEqual([[1, 'deny'], [2, 'approve'], [2, 'cancel']]);
  });

  it('the chip shows no time and is not urgent; a delivery round is shown first', () => {
    vi.spyOn(globalThis, 'fetch').mockReturnValue(new Promise(() => {}));
    const { view, control } = setup();
    ask(control, 1, 'HOME_DOCK');
    const chip = () => view.getByTestId('confirm-summary');
    expect(chip().dataset.shown).toBe('1');
    expect(view.getByTestId('confirm-summary-time').textContent).toBe('');
    expect(chip().getAttribute('aria-label')).toContain('task.standing.title');
    expect(chip().style.border).not.toContain('244, 67, 54');
    ask(control, 2, 'DOCK');
    expect(chip().dataset.shown).toBe('2');
    expect(view.getByTestId('confirm-summary-time').textContent).toBe('0:30');
  });
});

describe('standing round in the pure selectors', () => {
  it('has a standing phase and label with no time', () => {
    const state = { 1: entryFor('HOME_DOCK') };
    const summary = pendingSummary(state, () => TIMEOUT_SEC, Date.now() + PAST_TIMEOUT_MS);
    expect(summary.phase).toBe(CONFIRM_PHASE.STANDING);
    expect(summary.secondsLeft).toBe(0);
    expect(summary.liveCount).toBe(0);
    expect(confirmZeroLabel({ phase: CONFIRM_PHASE.STANDING, cmFl: false, cwt: TIMEOUT_SEC, seconds: 0 }))
      .toEqual({ cardKey: 'task.standing.title', chipKey: 'task.standing.title', time: '' });
  });
});
