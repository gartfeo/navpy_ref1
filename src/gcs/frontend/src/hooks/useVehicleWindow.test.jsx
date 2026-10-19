// Vehicle window state: the UAV pick, the camera states, the source and the
// size (docs/design/gcs_video_streams.md section 5, "Vehicle window" and the
// transition table).
import { describe, it, expect, beforeEach } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import useVehicleWindow, { SOURCE, WINDOW_STORAGE_KEY } from './useVehicleWindow';

const stream = (camera, id) => ({
  key: `${camera}:${id}`, camera_component: camera, stream_id: id, name: 'tracking overlay',
  encoding: 'H264', whep_url: `http://192.168.144.1${camera % 10}:8889/s${id}/whep`,
});
const entry = (sysId, streams = []) => ({
  sysId, index: sysId - 1, label: `UAV ${sysId}`, streams, linkOk: true, holdReason: null,
});

const setup = (fleet, followSysId = null, active = true) =>
  renderHook((props) => useVehicleWindow(props), { initialProps: { fleet, followSysId, active } });

beforeEach(() => localStorage.clear());

describe('first pick', () => {
  it('no streams: the first UAV, on "No camera"', () => {
    const { result } = setup([entry(1), entry(2)]);
    expect(result.current.sysId).toBe(1);
    expect(result.current.shown.kind).toBe('noCamera');
    expect(result.current.source).toBe(SOURCE.CAMERA);
  });

  it('the followed UAV with video, else the first UAV with video', () => {
    expect(setup([entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])], 2).result.current.sysId).toBe(2);
    expect(setup([entry(1), entry(2, [stream(100, 1)]), entry(3, [stream(100, 1)])], 1).result.current.sysId).toBe(2);
  });

  it('empty fleet: nothing picked; the first UAV to join is picked', () => {
    const { result, rerender } = setup([]);
    expect(result.current.sysId).toBeNull();
    expect(result.current.shown.kind).toBe('idle');
    rerender({ fleet: [entry(4)], followSysId: null, active: true });
    expect(result.current.sysId).toBe(4);
  });

  it('D1: later follow changes never move the window', () => {
    const fleet = [entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])];
    const { result, rerender } = setup(fleet, 1);
    rerender({ fleet, followSysId: 2, active: true });
    rerender({ fleet, followSysId: null, active: true });
    expect(result.current.sysId).toBe(1);
  });

  it('automatic on "No camera": moves to the first UAV that starts advertising', () => {
    const { result, rerender } = setup([entry(1), entry(2)]);
    rerender({ fleet: [entry(1), entry(2, [stream(100, 1)])], followSysId: null, active: true });
    expect(result.current.sysId).toBe(2);
    expect(result.current.shown.kind).toBe('stream');
  });

  it('a badge tap ends the automatic pick: no move when another UAV starts advertising', () => {
    const { result, rerender } = setup([entry(1), entry(2)]);
    act(() => result.current.selectUav(1));
    rerender({ fleet: [entry(1), entry(2, [stream(100, 1)])], followSysId: null, active: true });
    expect(result.current.sysId).toBe(1);
    expect(result.current.shown.kind).toBe('noCamera');
  });

  it('the switch also ends the automatic pick', () => {
    const { result, rerender } = setup([entry(1), entry(2)]);
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    act(() => result.current.chooseSource(SOURCE.CAMERA));
    rerender({ fleet: [entry(1), entry(2, [stream(100, 1)])], followSysId: null, active: true });
    expect(result.current.sysId).toBe(1);
  });

  it('the window\'s own UAV starting to advertise starts its video', () => {
    const { result, rerender } = setup([entry(1)]);
    act(() => result.current.selectUav(1));
    rerender({ fleet: [entry(1, [stream(100, 1)])], followSysId: null, active: true });
    expect(result.current.shown.kind).toBe('stream');
    expect(result.current.shown.stream.key).toBe('100:1');
  });

  it('an empty fleet re-arms the first pick and resets source and swap', () => {
    const { result, rerender } = setup([entry(1), entry(2, [stream(100, 1)])]);
    act(() => result.current.selectUav(1));
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    act(() => result.current.toggleMaximized());
    rerender({ fleet: [], followSysId: null, active: true });
    expect(result.current.sysId).toBeNull();
    expect(result.current.source).toBe(SOURCE.CAMERA);
    expect(result.current.swapped).toBe(false);
    rerender({ fleet: [entry(1), entry(2, [stream(100, 1)])], followSysId: null, active: true });
    expect(result.current.sysId).toBe(2);
  });
});

describe('badges and View camera', () => {
  it('badges pick the UAV and keep the source; the selected badge is a no-op', () => {
    const fleet = [entry(1, [stream(100, 1)]), entry(2, [stream(100, 2), stream(101, 1)])];
    const { result } = setup(fleet);
    const before = result.current.session;
    act(() => result.current.selectUav(1));
    expect(result.current.session).toBe(before);
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    act(() => result.current.selectUav(2));
    expect(result.current.sysId).toBe(2);
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    // Default stream = the lowest stream id.
    expect(result.current.shown.stream.key).toBe('101:1');
  });

  it('View camera: that UAV, Camera, a fresh session and a minimized window restored; the swap stays', () => {
    const fleet = [entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])];
    const { result } = setup(fleet);
    act(() => result.current.toggleMaximized());
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    const before = result.current.session;
    act(() => result.current.viewCamera(1));
    expect(result.current.source).toBe(SOURCE.CAMERA);
    expect(result.current.session).not.toBe(before);
    expect(result.current.swapped).toBe(true);
    act(() => result.current.minimize());
    act(() => result.current.viewCamera(2));
    expect(result.current.minimized).toBe(false);
    expect(result.current.sysId).toBe(2);
  });

  it('"Viewing" needs Camera, a shown stream of that UAV and a window that is not minimized', () => {
    const { result, rerender } = setup([entry(1, [stream(100, 1)]), entry(2)]);
    expect(result.current.viewing(1)).toBe(true);
    expect(result.current.viewing(2)).toBe(false);
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    expect(result.current.viewing(1)).toBe(false);
    act(() => result.current.chooseSource(SOURCE.CAMERA));
    act(() => result.current.minimize());
    expect(result.current.viewing(1)).toBe(false);
    act(() => result.current.restore());
    rerender({ fleet: [entry(1, [stream(100, 1)]), entry(2)], followSysId: null, active: false });
    expect(result.current.viewing(1)).toBe(false);
  });
});

describe('camera states', () => {
  it('stream removal: the same UAV\'s other stream, kept when the original returns', () => {
    const two = entry(1, [stream(100, 1), stream(101, 2)]);
    const { result, rerender } = setup([two, entry(2, [stream(100, 1)])]);
    expect(result.current.shown.stream.key).toBe('100:1');
    rerender({ fleet: [entry(1, [stream(101, 2)]), entry(2, [stream(100, 1)])], followSysId: null, active: true });
    expect(result.current.shown.stream.key).toBe('101:2');
    rerender({ fleet: [two, entry(2, [stream(100, 1)])], followSysId: null, active: true });
    expect(result.current.shown.stream.key).toBe('101:2');
  });

  it('last stream gone: lost, never another UAV; it comes back on the same UAV', () => {
    const { result, rerender } = setup([entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])]);
    rerender({ fleet: [entry(1), entry(2, [stream(100, 1)])], followSysId: null, active: true });
    expect(result.current.shown.kind).toBe('lost');
    expect(result.current.sysId).toBe(1);
    rerender({ fleet: [entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])], followSysId: null, active: true });
    expect(result.current.shown.kind).toBe('stream');
    expect(result.current.sysId).toBe(1);
  });

  it('Dismiss goes to "No camera" for the same UAV, never another', () => {
    const { result, rerender } = setup([entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])]);
    rerender({ fleet: [entry(1), entry(2, [stream(100, 1)])], followSysId: null, active: true });
    act(() => result.current.dismiss());
    expect(result.current.shown.kind).toBe('noCamera');
    expect(result.current.sysId).toBe(1);
  });

  it('the selected UAV leaves the fleet: the selection stays', () => {
    const { result, rerender } = setup([entry(1, [stream(100, 1)]), entry(2)]);
    rerender({ fleet: [entry(2)], followSysId: null, active: true });
    expect(result.current.sysId).toBe(1);
    expect(result.current.shown.kind).toBe('lost');
  });

  it('Reconnect video opens a fresh session on the same stream', () => {
    const { result } = setup([entry(1, [stream(100, 1)])]);
    const before = result.current.session;
    act(() => result.current.reconnect());
    expect(result.current.session).not.toBe(before);
    expect(result.current.shown.stream.key).toBe('100:1');
  });
});

describe('source', () => {
  it('Synthetic only by the switch; back to Camera is a fresh connection', () => {
    const { result } = setup([entry(1, [stream(100, 1)])]);
    const before = result.current.session;
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    act(() => result.current.chooseSource(SOURCE.CAMERA));
    expect(result.current.session).not.toBe(before);
  });

  it('a stream appearing or going away on Synthetic never changes the source', () => {
    const { result, rerender } = setup([entry(1)]);
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    rerender({ fleet: [entry(1, [stream(100, 1)])], followSysId: null, active: true });
    rerender({ fleet: [entry(1)], followSysId: null, active: true });
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
  });

  it('synthetic failure: Camera with a notice, cleared by the next operator action', () => {
    const { result } = setup([entry(1)]);
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    act(() => result.current.syntheticFailed());
    expect(result.current.source).toBe(SOURCE.CAMERA);
    expect(result.current.notice).toBe('syntheticUnavailable');
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    expect(result.current.notice).toBeNull();
  });

  it('RC entry or leaving the monitor view: swap back and Camera; the window returns as it was', () => {
    const fleet = [entry(1)];
    const { result, rerender } = setup(fleet);
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    act(() => result.current.toggleMaximized());
    rerender({ fleet, followSysId: null, active: false });
    expect(result.current.source).toBe(SOURCE.CAMERA);
    expect(result.current.swapped).toBe(false);
    rerender({ fleet, followSysId: null, active: true });
    expect(result.current.swapped).toBe(false);
    expect(result.current.minimized).toBe(false);
  });
});

describe('size', () => {
  it('minimized is saved; maximized is not and ends on minimize', () => {
    const { result } = setup([entry(1)]);
    act(() => result.current.toggleMaximized());
    expect(result.current.swapped).toBe(true);
    act(() => result.current.minimize());
    expect(result.current.swapped).toBe(false);
    expect(result.current.minimized).toBe(true);
    expect(localStorage.getItem(WINDOW_STORAGE_KEY)).toBe('minimized');
    act(() => result.current.restore());
    expect(localStorage.getItem(WINDOW_STORAGE_KEY)).toBe('normal');
    // Restore gives the normal size, not the swap.
    expect(result.current.swapped).toBe(false);
  });

  it('a reload opens minimized as saved, unswapped, on Camera', () => {
    localStorage.setItem(WINDOW_STORAGE_KEY, 'minimized');
    const { result } = setup([entry(1)]);
    expect(result.current.minimized).toBe(true);
    expect(result.current.swapped).toBe(false);
    expect(result.current.source).toBe(SOURCE.CAMERA);
  });

  it('storage that throws: the window opens normal and still minimizes', () => {
    const original = Object.getOwnPropertyDescriptor(globalThis, 'localStorage');
    Object.defineProperty(globalThis, 'localStorage', { configurable: true, get() { throw new Error('blocked'); } });
    try {
      const { result } = setup([entry(1)]);
      expect(result.current.minimized).toBe(false);
      act(() => result.current.minimize());
      expect(result.current.minimized).toBe(true);
    } finally {
      Object.defineProperty(globalThis, 'localStorage', original);
    }
  });

  it('Restore or a tap on the small map swaps back; nothing reconnects', () => {
    const { result } = setup([entry(1, [stream(100, 1)])]);
    const before = result.current.session;
    act(() => result.current.toggleMaximized());
    act(() => result.current.unswap());
    expect(result.current.swapped).toBe(false);
    act(() => result.current.toggleMaximized());
    act(() => result.current.toggleMaximized());
    expect(result.current.swapped).toBe(false);
    expect(result.current.session).toBe(before);
  });
});
