// Telemetry freshness: a cached snapshot is current only after new telemetry
// for that vehicle since the socket last opened.
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import telemetryStore from './telemetryStore';

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

  telemetry(vehicles) {
    this.onmessage({ data: JSON.stringify({ type: 'telemetry', vehicles }) });
  }

  drop() {
    this.readyState = 3;
    this.onclose();
  }

  close() {}
}

beforeEach(() => {
  vi.useFakeTimers();
  vi.stubGlobal('WebSocket', FakeSocket);
});

afterEach(() => {
  telemetryStore.disconnect();
  vi.unstubAllGlobals();
  vi.useRealTimers();
});

describe('isTelemetryFresh', () => {
  it('needs an open socket and a snapshot since it opened', () => {
    telemetryStore.connect();
    const ws = FakeSocket.last;
    expect(telemetryStore.isTelemetryFresh(1)).toBe(false);
    ws.open();
    ws.telemetry([{ sys_id: 1, link_ok: true }]);
    expect(telemetryStore.isTelemetryFresh(1)).toBe(true);
    expect(telemetryStore.isTelemetryFresh(2)).toBe(false);
  });

  it('a closed socket keeps the cache but not freshness; reopening needs new telemetry', () => {
    telemetryStore.connect();
    const ws = FakeSocket.last;
    ws.open();
    ws.telemetry([{ sys_id: 1, link_ok: true }]);
    ws.drop();
    expect(telemetryStore.getVehicles()[1]).toBeDefined();
    expect(telemetryStore.isTelemetryFresh(1)).toBe(false);

    vi.advanceTimersByTime(2000);
    const again = FakeSocket.last;
    expect(again).not.toBe(ws);
    again.open();
    expect(telemetryStore.isTelemetryFresh(1)).toBe(false);
    again.telemetry([{ sys_id: 1, link_ok: true }]);
    expect(telemetryStore.isTelemetryFresh(1)).toBe(true);
  });
});
