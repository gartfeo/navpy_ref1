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

  close() {}
}

const entry = (seq, text, severity = 6, kind = 'text') => ({
  seq, kind, ts: 1990 + seq, source: 'autopilot', severity,
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
    return Promise.resolve({ ok: true, json: () => Promise.resolve({ entries: bySysId[sysId] || [] }) });
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

  it('says when the backlog could not load', async () => {
    vi.stubGlobal('fetch', vi.fn(() => Promise.resolve({ ok: false, status: 404 })));
    act(() => ws.telemetry([{ sys_id: 1, name: 'UAV 1' }]));
    render(<MessagesWindow sysId={1} onSelect={() => {}} onClose={() => {}} />);
    await screen.findByText('messages.loadFailed');
  });
});
