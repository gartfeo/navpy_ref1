// Messages window: telemetry entries reach the per-UAV history, the backlog
// loads on open, filtering and UAV switching work.
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { render, screen, fireEvent, waitFor, act } from '@testing-library/react';
import MessagesWindow from './MessagesWindow';
import telemetryStore from '../stores/telemetryStore';
import messageLogStore from '../stores/messageLogStore';
import { resetBackendClock } from '../utils/backendClock';

class FakeSocket {
  static OPEN = 1;

  static last = null;

  constructor() {
    this.readyState = 0;
    FakeSocket.last = this;
  }

  open() {
    this.readyState = FakeSocket.OPEN;
    this.onopen();
  }

  telemetry(vehicles, ts = 2000) {
    this.onmessage({ data: JSON.stringify({ type: 'telemetry', vehicles, ts }) });
  }

  drop() {
    this.readyState = 3;
    this.onclose();
  }

  close() {}
}

const entry = (seq, text, severity = 6, kind = 'text', epoch = 'e1') => ({
  seq, epoch, kind, ts: 1990 + seq, source: 'autopilot', severity,
  label: severity === 4 ? 'WARN' : 'INFO', text,
});

let ws;
beforeEach(() => {
  vi.stubGlobal('WebSocket', FakeSocket);
  messageLogStore.reset();
  telemetryStore.connect();
  ws = FakeSocket.last;
  ws.open();
});

afterEach(() => {
  telemetryStore.disconnect();
  resetBackendClock();
  vi.unstubAllGlobals();
});

function backlog(bySysId) {
  vi.stubGlobal('fetch', vi.fn((url) => {
    const sysId = Number(url.match(/vehicles\/(\d+)\/messages/)[1]);
    return Promise.resolve({
      ok: true,
      json: () => Promise.resolve({ epoch: 'e1', entries: bySysId[sysId] || [] }),
    });
  }));
}

describe('telemetry feed', () => {
  it('moves `messages` into the history and out of the snapshot', () => {
    act(() => ws.telemetry([{ sys_id: 1, name: 'UAV 1', messages: [entry(1, 'Armed')] }]));
    expect(messageLogStore.getEntries(1).map((e) => e.text)).toEqual(['Armed']);
    expect(telemetryStore.getVehicles()[1].messages).toBeUndefined();
  });
});

describe('MessagesWindow', () => {
  it('loads the backlog, then shows live entries after it', async () => {
    backlog({ 1: [entry(1, 'PreArm: Throttle below failsafe', 4)] });
    act(() => ws.telemetry([{ sys_id: 1, name: 'UAV 1' }]));
    render(<MessagesWindow sysId={1} onSelect={() => {}} onClose={() => {}} />);
    expect(fetch).toHaveBeenCalledWith('/api/vehicles/1/messages');
    await screen.findByText('PreArm: Throttle below failsafe');
    act(() => ws.telemetry([{ sys_id: 1, messages: [entry(2, 'Mission: 1 WP')] }]));
    expect(screen.getByText('Mission: 1 WP')).toBeInTheDocument();
  });

  it('filters to warnings', async () => {
    backlog({ 1: [entry(1, 'info line'), entry(2, 'warn line', 4)] });
    act(() => ws.telemetry([{ sys_id: 1, name: 'UAV 1' }]));
    render(<MessagesWindow sysId={1} onSelect={() => {}} onClose={() => {}} />);
    await screen.findByText('info line');
    fireEvent.change(screen.getByLabelText('messages.filter'), { target: { value: 'warnings' } });
    expect(screen.queryByText('info line')).toBeNull();
    expect(screen.getByText('warn line')).toBeInTheDocument();
  });

  it('switches UAV from its badges and closes', async () => {
    backlog({});
    act(() => ws.telemetry([{ sys_id: 1, name: 'UAV 1' }, { sys_id: 2, name: 'UAV 2' }]));
    const onSelect = vi.fn();
    const onClose = vi.fn();
    render(<MessagesWindow sysId={1} onSelect={onSelect} onClose={onClose} />);
    await waitFor(() => expect(screen.getByText('messages.empty')).toBeInTheDocument());
    fireEvent.click(screen.getByText('UAV 2'));
    expect(onSelect).toHaveBeenCalledWith(2);
    fireEvent.click(screen.getByLabelText('settings.close'));
    expect(onClose).toHaveBeenCalled();
  });

  it('shows named values and marks one that stopped arriving', async () => {
    backlog({});
    act(() => ws.telemetry([{
      sys_id: 1,
      name: 'UAV 1',
      named_values: {
        NAVWD: { value: 1, source: 'autopilot', ts: 1990 },
        TASK: { value: 4, source: 'companion', ts: 1999.5 },
      },
    }], 2000));
    render(<MessagesWindow sysId={1} onSelect={() => {}} onClose={() => {}} />);
    const values = await screen.findByTestId('messages-named-values');
    expect(values.textContent).toContain('NAVWD 1');
    expect(values.textContent).toContain('messages.staleFor');
    expect(values.textContent).toContain('TASK 4');
  });

  // The backend keeps draining while the socket is down; those entries are
  // only in its backlog, so a reconnect loads it again.
  it('loads the backlog again after the socket reconnects', async () => {
    const server = [entry(1, 'before')];
    backlog({ 1: server });
    act(() => ws.telemetry([{ sys_id: 1, name: 'UAV 1' }]));
    render(<MessagesWindow sysId={1} onSelect={() => {}} onClose={() => {}} />);
    await screen.findByText('before');
    act(() => ws.drop());
    server.push(entry(2, 'while down'));  // broadcast to nobody
    act(() => telemetryStore.connect());  // what the store's reconnect timer does
    ws = FakeSocket.last;
    act(() => ws.open());
    act(() => ws.telemetry([{ sys_id: 1, messages: [entry(3, 'after')] }]));
    await screen.findByText('while down');
    expect(fetch).toHaveBeenCalledTimes(2);
    expect(messageLogStore.getEntries(1).map((e) => e.text)).toEqual(['before', 'while down', 'after']);
  });

  // A UAV switch mid-load: the first UAV's late failure must not show on the second.
  it('ignores a load answered after the UAV changed', async () => {
    let failFirst;
    vi.stubGlobal('fetch', vi.fn((url) => (url.includes('/1/')
      ? new Promise((_, reject) => { failFirst = reject; })
      : Promise.resolve({ ok: true, json: () => Promise.resolve({ epoch: 'e1', entries: [] }) }))));
    act(() => ws.telemetry([{ sys_id: 1, name: 'UAV 1' }, { sys_id: 2, name: 'UAV 2' }]));
    const view = render(<MessagesWindow sysId={1} onSelect={() => {}} onClose={() => {}} />);
    view.rerender(<MessagesWindow sysId={2} onSelect={() => {}} onClose={() => {}} />);
    await waitFor(() => expect(fetch).toHaveBeenCalledWith('/api/vehicles/2/messages'));
    await act(async () => { failFirst(new Error('late')); });
    expect(screen.queryByText('messages.loadFailed')).toBeNull();
    expect(screen.getByText('messages.empty')).toBeInTheDocument();
  });

  // At capacity a new row evicts the oldest, so the row count stays 500.
  it('keeps following the newest row once the history is full', async () => {
    const full = Array.from({ length: 500 }, (_, i) => entry(i + 1, `row ${i + 1}`));
    backlog({ 1: full });
    act(() => ws.telemetry([{ sys_id: 1, name: 'UAV 1' }]));
    let scrolls = 0;
    vi.spyOn(HTMLElement.prototype, 'scrollHeight', 'get').mockReturnValue(10000);
    vi.spyOn(HTMLElement.prototype, 'clientHeight', 'get').mockReturnValue(300);
    let top = 0;
    vi.spyOn(HTMLElement.prototype, 'scrollTop', 'get').mockImplementation(() => top);
    vi.spyOn(HTMLElement.prototype, 'scrollTop', 'set').mockImplementation((v) => { top = v; scrolls += 1; });
    render(<MessagesWindow sysId={1} onSelect={() => {}} onClose={() => {}} />);
    await screen.findByText('row 500');
    const before = scrolls;
    act(() => ws.telemetry([{ sys_id: 1, messages: [entry(501, 'row 501')] }]));
    expect(screen.getByText('row 501')).toBeInTheDocument();
    expect(screen.queryByText('row 1')).toBeNull();
    expect(scrolls).toBeGreaterThan(before);
  });

  it('marks a long text with a lost chunk', async () => {
    backlog({ 1: [{ ...entry(1, 'PreArm: part'), incomplete: true }] });
    act(() => ws.telemetry([{ sys_id: 1, name: 'UAV 1' }]));
    render(<MessagesWindow sysId={1} onSelect={() => {}} onClose={() => {}} />);
    await screen.findByText(/messages\.incomplete/);
  });

  // The UAV reconnects (a new log epoch) while the socket stays open: a
  // backlog reply from the old epoch must not replace the new log's rows.
  it('drops a backlog reply from a superseded epoch', async () => {
    let answerOld;
    vi.stubGlobal('fetch', vi.fn()
      .mockImplementationOnce(() => new Promise((resolve) => { answerOld = resolve; }))
      .mockImplementation(() => Promise.resolve({
        ok: true,
        json: () => Promise.resolve({ epoch: 'e2', entries: [entry(1, 'new log', 4, 'text', 'e2')] }),
      })));
    act(() => ws.telemetry([{ sys_id: 1, name: 'UAV 1', message_epoch: 'e1' }]));
    render(<MessagesWindow sysId={1} onSelect={() => {}} onClose={() => {}} />);
    act(() => ws.telemetry([{
      sys_id: 1, message_epoch: 'e2', messages: [entry(1, 'new log', 4, 'text', 'e2')],
    }]));
    await waitFor(() => expect(fetch).toHaveBeenCalledTimes(2));
    await act(async () => answerOld({ ok: true, json: () => Promise.resolve({ epoch: 'e1', entries: [] }) }));
    expect(screen.getByText('new log')).toBeInTheDocument();
    expect(messageLogStore.getEntries(1).map((e) => [e.epoch, e.text])).toEqual([['e2', 'new log']]);
  });

  // After a backend restart the socket can reopen before the UAV is
  // discovered again: that load gets a 404. The UAV's next telemetry retries.
  it("retries a failed load when the UAV's telemetry next arrives", async () => {
    vi.stubGlobal('fetch', vi.fn()
      .mockImplementationOnce(() => Promise.resolve({ ok: false, status: 404 }))
      .mockImplementation(() => Promise.resolve({
        ok: true, json: () => Promise.resolve({ epoch: 'e1', entries: [entry(1, 'found')] }),
      })));
    act(() => ws.telemetry([{ sys_id: 1, name: 'UAV 1', message_epoch: 'e1' }]));
    render(<MessagesWindow sysId={1} onSelect={() => {}} onClose={() => {}} />);
    await screen.findByText('messages.loadFailed');
    expect(fetch).toHaveBeenCalledTimes(1);  // no timer retry
    act(() => ws.telemetry([{ sys_id: 1, mode: 'FBWA' }]));
    await screen.findByText('found');
    expect(fetch).toHaveBeenCalledTimes(2);
    expect(screen.queryByText('messages.loadFailed')).toBeNull();
  });

  it('says when the backlog could not load', async () => {
    vi.stubGlobal('fetch', vi.fn(() => Promise.resolve({ ok: false, status: 404 })));
    act(() => ws.telemetry([{ sys_id: 1, name: 'UAV 1' }]));
    render(<MessagesWindow sysId={1} onSelect={() => {}} onClose={() => {}} />);
    await screen.findByText('messages.loadFailed');
  });
});
