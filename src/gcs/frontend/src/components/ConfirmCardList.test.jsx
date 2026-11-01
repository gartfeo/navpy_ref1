import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, act, fireEvent, within } from '@testing-library/react';
import ConfirmCardList from './ConfirmCardList';
import useTaskConfirmation from '../hooks/useTaskConfirmation';

// Todo 69: every approved UAV's CANCEL TASK sits in the pinned part with its
// UAV badge and cancels that UAV only; pending cards and the opened still
// scroll below. The real useTaskConfirmation hook owns the rounds, as in App.

const FLEET = [
  { sys_id: 7, name: 's1-u7' },
  { sys_id: 8, name: 's1-u8' },
  { sys_id: 9, name: 's1-u9' },
];
const AAS = { getConfirmedValue: (sid, key, fallback) => fallback };
const IMAGE = 'aGVsbG8=';

function okFetch() {
  return vi.spyOn(globalThis, 'fetch').mockResolvedValue({ ok: true, status: 200 });
}
const postedBodies = (fetchSpy) => fetchSpy.mock.calls.map(([, init]) => JSON.parse(init.body));

/**
 * App's two hosts: the right panel (one ConfirmCardList over the planning or
 * monitor sidebar, View camera only in monitor) and the RC overlay (its own
 * list). `host` is 'monitor' | 'planning' | 'rc'.
 */
function Harness({ host, vehicleList = FLEET, control, videoActionFor = null }) {
  const tc = useTaskConfirmation();
  control.current = tc;
  const common = {
    pendingConfirms: tc.pendingConfirms,
    vehicleList,
    onApprove: tc.approve,
    onDeny: tc.deny,
    onCancel: tc.cancel,
    forcedConfirms: tc.forcedConfirms,
    aasParams: AAS,
  };
  if (host === 'rc') {
    return <ConfirmCardList key="rc" {...common} style={{ position: 'absolute', maxHeight: 207 }} />;
  }
  return (
    <ConfirmCardList key="panel" {...common} videoActionFor={host === 'monitor' ? videoActionFor : null} style={{ height: '100%' }}>
      <div>{host} sidebar</div>
    </ConfirmCardList>
  );
}

function setup(props = {}) {
  const control = { current: null };
  const view = render(<Harness host="monitor" control={control} {...props} />);
  const rerender = (next) => view.rerender(<Harness host="monitor" control={control} {...props} {...next} />);
  return { ...view, control, rerender };
}

function ask(control, sysId, taskId, { roundUid = `r${sysId}-${taskId}`, image = IMAGE } = {}) {
  act(() => {
    control.current.handleConfirmRequest({ sys_id: sysId, task_id: taskId, round_uid: roundUid, lat: 40.1824, lon: 44.5171, alt: 1012 });
  });
  if (image) {
    act(() => { control.current.handleConfirmImage({ sys_id: sysId, task_id: taskId, round_uid: roundUid, image_b64: image }); });
  }
}

function approve(control, sysId, taskId, roundUid = `r${sysId}-${taskId}`) {
  act(() => {
    control.current.handleConfirmResponse({ sys_id: sysId, task_id: taskId, round_uid: roundUid, is_confirmed: true, action: 'approve' });
  });
}

const row = (view, sysId) => view.getByTestId(`confirm-row-${sysId}`);
const cancelIn = (view, sysId) => within(row(view, sysId)).getByRole('button', { name: 'task.cancelTask' });
const stillIn = (view, sysId) => within(row(view, sysId)).getByRole('button', { name: /task\.(show|hide)Still/ });

async function click(element) {
  await act(async () => { fireEvent.click(element); });
}

describe('ConfirmCardList pinned rows', () => {
  it('puts each approved CANCEL TASK in the pinned part next to its UAV badge', () => {
    const view = setup();
    for (const sid of [7, 8, 9]) { ask(view.control, sid, 100 + sid); approve(view.control, sid, 100 + sid); }

    const pinned = view.getByTestId('confirm-pinned');
    for (const sid of [7, 8, 9]) {
      expect(pinned).toContainElement(row(view, sid));
      expect(within(row(view, sid)).getByText(`s1-u${sid}`)).toBeInTheDocument();
      expect(within(row(view, sid)).getByText('task.approvalSent')).toBeInTheDocument();
      expect(cancelIn(view, sid).style.height).toBe('32px');
    }
    expect(view.getByTestId('confirm-scroll')).not.toContainElement(row(view, 7));
  });

  it.each([7, 8, 9])('sends cancel for UAV %i only', async (sid) => {
    const fetchSpy = okFetch();
    const view = setup();
    for (const s of [7, 8, 9]) { ask(view.control, s, 100 + s); approve(view.control, s, 100 + s); }

    await click(cancelIn(view, sid));

    expect(postedBodies(fetchSpy)).toEqual([
      expect.objectContaining({ sys_id: sid, task_id: 100 + sid, action: 'cancel', round_uid: `r${sid}-${100 + sid}` }),
    ]);
    expect(within(row(view, sid)).getByText('task.cancellationRequested')).toBeInTheDocument();
    expect(within(row(view, sid)).queryByRole('button')).toBeNull();
    for (const other of [7, 8, 9].filter((s) => s !== sid)) {
      expect(cancelIn(view, other)).toBeInTheDocument();
    }
  });

  it('keeps each row bound to its UAV when the roster order changes', async () => {
    const fetchSpy = okFetch();
    const view = setup();
    for (const s of [7, 8, 9]) { ask(view.control, s, 100 + s); approve(view.control, s, 100 + s); }

    // Every UAV changes place, so no row can match its UAV by position.
    view.rerender({ vehicleList: [FLEET[2], FLEET[0], FLEET[1]] });

    for (const sid of [7, 8, 9]) {
      expect(within(row(view, sid)).getByText(`s1-u${sid}`)).toBeInTheDocument();
    }
    await click(cancelIn(view, 7));
    expect(postedBodies(fetchSpy).map((b) => b.sys_id)).toEqual([7]);
  });

  it('a new round for the same UAV replaces its row; the other rows keep their UAV', async () => {
    const fetchSpy = okFetch();
    const view = setup();
    for (const s of [8, 9]) { ask(view.control, s, 100 + s); approve(view.control, s, 100 + s); }

    ask(view.control, 8, 208);

    expect(view.queryByTestId('confirm-row-8')).toBeNull();
    expect(within(view.getByTestId('confirm-scroll')).getByText('task.approve')).toBeInTheDocument();
    await click(cancelIn(view, 9));
    expect(postedBodies(fetchSpy)).toEqual([expect.objectContaining({ sys_id: 9, task_id: 109, action: 'cancel' })]);
  });

  it.each([
    ['a rejected', () => vi.spyOn(globalThis, 'fetch').mockResolvedValue({ ok: false, status: 409 })],
    ['a failed', () => vi.spyOn(globalThis, 'fetch').mockRejectedValue(new Error('offline'))],
  ])('%s cancel POST keeps the row and its button', async (_label, mockFetch) => {
    mockFetch();
    vi.spyOn(console, 'error').mockImplementation(() => {});
    const view = setup();
    ask(view.control, 8, 108); approve(view.control, 8, 108);

    await click(cancelIn(view, 8));

    expect(within(row(view, 8)).getByText('task.approvalSent')).toBeInTheDocument();
    expect(cancelIn(view, 8)).toBeInTheDocument();
  });

  it('a UAV name too long for the row is cut off, never CANCEL TASK', () => {
    const name = 'operator-named-uav-with-a-very-long-name-that-fills-the-row';
    const view = setup({ vehicleList: [FLEET[0], { sys_id: 8, name }, FLEET[2]] });
    ask(view.control, 8, 108); approve(view.control, 8, 108);

    // jsdom has no layout, so this checks the flex rules (the live check
    // measures them): the badge's box may shrink to nothing and clips its
    // name on one line, while CANCEL TASK keeps its min-content width.
    const badgeBox = within(row(view, 8)).getByText(name).parentElement;
    expect(badgeBox.style.flexShrink).toBe('1');
    expect(badgeBox.style.minWidth).toBe('0px');
    expect(badgeBox.style.overflow).toBe('hidden');
    expect(badgeBox.style.whiteSpace).toBe('nowrap');
    expect(badgeBox.title).toBe(name);
    expect(cancelIn(view, 8)).toBeInTheDocument();
  });

  it('denied rows show the outcome with no button', () => {
    const view = setup();
    ask(view.control, 7, 107);
    act(() => { view.control.current.handleConfirmResponse({ sys_id: 7, task_id: 107, round_uid: 'r7-107', is_confirmed: false, action: 'deny' }); });

    expect(within(row(view, 7)).getByText('task.denied')).toBeInTheDocument();
    expect(within(row(view, 7)).queryByRole('button')).toBeNull();
  });

  it('pending cards scroll below the pinned rows, before the sidebar', () => {
    const view = setup();
    ask(view.control, 7, 107);
    ask(view.control, 8, 108); approve(view.control, 8, 108);

    const scroll = view.getByTestId('confirm-scroll');
    const approveButton = within(scroll).getByText('task.approve');
    expect(scroll.style.overflowY).toBe('auto');
    expect(approveButton.compareDocumentPosition(within(scroll).getByText('monitor sidebar')))
      .toBe(Node.DOCUMENT_POSITION_FOLLOWING);
  });
});

describe('ConfirmCardList hosts', () => {
  beforeEach(() => {
    vi.useFakeTimers({ toFake: ['Date', 'setInterval', 'clearInterval'] });
  });

  it('rounds survive monitor -> planning -> RC -> monitor, with an expiring round and a forced hold', async () => {
    const fetchSpy = okFetch();
    const control = { current: null };
    const view = render(<Harness host="monitor" control={control} />);
    const show = (host) => view.rerender(<Harness host={host} control={control} />);

    ask(control, 7, 107);                       // pending, 30 s to its timeout (deny)
    ask(control, 8, 108); approve(control, 8, 108);
    ask(control, 9, 109);
    await act(async () => { await control.current.forceConfirm(9, 109); });

    for (const host of ['planning', 'rc', 'monitor']) {
      act(() => { vi.advanceTimersByTime(8_000); });
      show(host);
      expect(cancelIn(view, 8)).toBeInTheDocument();
      expect(view.getByText('task.holdToApprove')).toBeInTheDocument();
    }
    // 24 s gone: UAV 7's and 9's rounds still ask, with their original deadline
    // ("~": NAV_CWT is not loaded here, so the 30 s default is an estimate).
    expect(['7', '9'].map((sid) => view.getByTestId(`confirm-countdown-${sid}`).dataset.time))
      .toEqual(['~0:06', '~0:06']);
    await act(async () => { vi.advanceTimersByTime(7_000); });

    const sevens = postedBodies(fetchSpy).filter((b) => b.sys_id === 7);
    expect(sevens).toEqual([expect.objectContaining({ task_id: 107, action: 'deny' })]);
    expect(within(row(view, 7)).getByText('task.denied')).toBeInTheDocument();
    expect(cancelIn(view, 8)).toBeInTheDocument();
  });

  it('offers View camera on a pending card in monitor only', () => {
    const videoActionFor = () => ({ viewing: false, onView: () => {} });
    const control = { current: null };
    const view = render(<Harness host="monitor" control={control} videoActionFor={videoActionFor} />);
    ask(control, 7, 107);
    expect(view.getByText('task.viewCamera')).toBeInTheDocument();

    view.rerender(<Harness host="planning" control={control} videoActionFor={videoActionFor} />);
    expect(view.getByText('task.approve')).toBeInTheDocument();
    expect(view.queryByText('task.viewCamera')).toBeNull();

    view.rerender(<Harness host="rc" control={control} videoActionFor={videoActionFor} />);
    expect(view.queryByText('task.viewCamera')).toBeNull();
  });

  it('the RC list keeps its rows pinned over a scrolling part, inside the host cap', () => {
    const control = { current: null };
    const view = render(<Harness host="rc" control={control} />);
    ask(control, 7, 107);
    ask(control, 8, 108); approve(control, 8, 108);

    const pinned = view.getByTestId('confirm-pinned');
    const scroll = view.getByTestId('confirm-scroll');
    const root = pinned.parentElement;
    expect(root.style.maxHeight).toBe('207px');
    expect(root.style.display).toBe('flex');
    expect(root.style.flexDirection).toBe('column');
    expect(pinned.style.flexShrink).toBe('0');
    expect(scroll.style.overflowY).toBe('auto');
    expect(scroll.style.minHeight).toBe('0px');
    expect(within(scroll).getByText('task.approve')).toBeInTheDocument();
  });
});

describe('ConfirmCardList still', () => {
  function threeApproved() {
    const view = setup();
    for (const s of [7, 8, 9]) { ask(view.control, s, 100 + s); approve(view.control, s, 100 + s); }
    return view;
  }

  it('opens in the scrolling part and never moves a row', () => {
    const view = threeApproved();
    const pinned = view.getByTestId('confirm-pinned');
    const before = [...pinned.children];

    fireEvent.click(stillIn(view, 8));

    const still = view.getByTestId('confirm-still');
    expect(view.getByTestId('confirm-scroll')).toContainElement(still);
    expect(pinned).not.toContainElement(still);
    expect(pinned.children).toHaveLength(before.length);
    before.forEach((el, i) => expect(pinned.children[i]).toBe(el));
    expect(within(still).getByText('s1-u8')).toBeInTheDocument();
    expect(within(still).getByRole('img').style.maxHeight).toBe('120px');
    expect(stillIn(view, 8)).toHaveAttribute('aria-pressed', 'true');
  });

  it('shows one still at a time, and its button closes it', () => {
    const view = threeApproved();
    fireEvent.click(stillIn(view, 8));
    fireEvent.click(stillIn(view, 9));

    expect(view.getAllByTestId('confirm-still')).toHaveLength(1);
    expect(within(view.getByTestId('confirm-still')).getByText('s1-u9')).toBeInTheDocument();
    expect(stillIn(view, 8)).toHaveAttribute('aria-pressed', 'false');

    fireEvent.click(stillIn(view, 9));
    expect(view.queryByTestId('confirm-still')).toBeNull();
  });

  it('closes when its round is replaced, and stays closed for the new round', () => {
    const view = threeApproved();
    fireEvent.click(stillIn(view, 8));

    ask(view.control, 8, 208);
    expect(view.queryByTestId('confirm-still')).toBeNull();
    approve(view.control, 8, 208);
    expect(view.queryByTestId('confirm-still')).toBeNull();
    expect(stillIn(view, 8)).toHaveAttribute('aria-pressed', 'false');
  });

  it('closes when the same task is asked again, and stays closed for that new round', () => {
    const view = threeApproved();
    fireEvent.click(stillIn(view, 8));

    ask(view.control, 8, 108, { roundUid: 'r8-108-again', image: 'YWdhaW4=' });
    expect(view.queryByTestId('confirm-still')).toBeNull();
    approve(view.control, 8, 108, 'r8-108-again');
    expect(view.queryByTestId('confirm-still')).toBeNull();
    expect(stillIn(view, 8)).toHaveAttribute('aria-pressed', 'false');
  });

  it('closes when its round is cancelled', async () => {
    okFetch();
    const view = threeApproved();
    fireEvent.click(stillIn(view, 8));
    await click(cancelIn(view, 8));
    expect(view.queryByTestId('confirm-still')).toBeNull();
  });

  it('opened while scrolled to the end, it scrolls into view', () => {
    const view = threeApproved();
    ask(view.control, 6, 106);
    const scroll = view.getByTestId('confirm-scroll');
    scroll.scrollTop = 400;
    expect(scroll.scrollTop).toBe(400);

    fireEvent.click(stillIn(view, 9));

    expect(scroll.scrollTop).toBe(0);
    expect(scroll.firstElementChild).toContainElement(view.getByTestId('confirm-still'));
  });

  it('has no Still button when the round sent no picture', () => {
    const view = setup();
    ask(view.control, 8, 108, { image: null }); approve(view.control, 8, 108);
    expect(within(row(view, 8)).queryByRole('button', { name: /Still/ })).toBeNull();
    expect(cancelIn(view, 8)).toBeInTheDocument();
  });
});
