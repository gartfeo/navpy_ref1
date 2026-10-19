// Vehicle window state: the picker, the UAV pick, the camera states, the
// source and the swap (docs/design/gcs_video_streams.md section 5, "Vehicle
// window" and the transition table).
import { describe, it, expect, vi, afterEach } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import useVehicleWindow, { SOURCE } from './useVehicleWindow';

const stream = (camera, id) => ({
  key: `${camera}:${id}`, camera_component: camera, stream_id: id, name: 'tracking overlay',
  encoding: 'H264', whep_url: `http://192.168.144.1${camera % 10}:8889/s${id}/whep`,
});
const entry = (sysId, streams = []) => ({
  sysId, index: sysId - 1, label: `UAV ${sysId}`, streams, linkOk: true, holdReason: null,
});

const setup = (fleet, active = true) =>
  renderHook((props) => useVehicleWindow(props), { initialProps: { fleet, active } });

afterEach(() => vi.restoreAllMocks());

describe('the picker', () => {
  it('starts on the picker, also when UAVs have streams: no automatic pick', () => {
    const { result, rerender } = setup([entry(1, [stream(100, 1)]), entry(2)]);
    expect(result.current.sysId).toBeNull();
    expect(result.current.shown.kind).toBe('idle');
    expect(result.current.session).toBeNull();
    expect(result.current.swapped).toBe(false);
    rerender({ fleet: [entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])], active: true });
    expect(result.current.sysId).toBeNull();
  });

  it('nothing is read from or written to browser storage', () => {
    const get = vi.spyOn(Storage.prototype, 'getItem');
    const set = vi.spyOn(Storage.prototype, 'setItem');
    const { result } = setup([entry(1, [stream(100, 1)])]);
    act(() => result.current.selectUav(1));
    act(() => result.current.toggleMaximized());
    act(() => result.current.close());
    expect(get).not.toHaveBeenCalled();
    expect(set).not.toHaveBeenCalled();
  });

  it('Maximize does nothing in the picker', () => {
    const { result } = setup([entry(1)]);
    act(() => result.current.toggleMaximized());
    expect(result.current.swapped).toBe(false);
  });

  it('an empty fleet clears the selection and the swap; the next UAV brings the picker', () => {
    const fleet = [entry(1), entry(2, [stream(100, 1)])];
    const { result, rerender } = setup(fleet);
    act(() => result.current.selectUav(2));
    act(() => result.current.toggleMaximized());
    rerender({ fleet: [], active: true });
    expect(result.current.sysId).toBeNull();
    expect(result.current.swapped).toBe(false);
    rerender({ fleet, active: true });
    expect(result.current.sysId).toBeNull();
    expect(result.current.swapped).toBe(false);
  });
});

describe('UAV buttons and View camera', () => {
  it('a camera-icon pick opens Camera on its default stream; a map-icon pick opens Synthetic', () => {
    const fleet = [entry(1, [stream(100, 2), stream(101, 1)]), entry(2)];
    const { result } = setup(fleet);
    act(() => result.current.selectUav(1));
    expect(result.current.sysId).toBe(1);
    expect(result.current.source).toBe(SOURCE.CAMERA);
    // Default stream = the lowest stream id.
    expect(result.current.shown.stream.key).toBe('101:1');
    act(() => result.current.selectUav(2));
    expect(result.current.sysId).toBe(2);
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    // From the header too: back to the camera-icon UAV opens Camera, not the last source.
    act(() => result.current.selectUav(1));
    expect(result.current.source).toBe(SOURCE.CAMERA);
  });

  it('picking the selected UAV again does nothing', () => {
    const { result } = setup([entry(1, [stream(100, 1)])]);
    act(() => result.current.selectUav(1));
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    const before = result.current.session;
    act(() => result.current.selectUav(1));
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    expect(result.current.session).toBe(before);
  });

  it('a pick while swapped keeps the swap', () => {
    const { result } = setup([entry(1, [stream(100, 1)]), entry(2)]);
    act(() => result.current.selectUav(1));
    act(() => result.current.toggleMaximized());
    act(() => result.current.selectUav(2));
    expect(result.current.swapped).toBe(true);
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
  });

  it('View camera: that UAV on Camera with a fresh session, also from the picker; the swap stays', () => {
    const fleet = [entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])];
    const { result } = setup(fleet);
    act(() => result.current.viewCamera(2));
    expect(result.current.sysId).toBe(2);
    expect(result.current.source).toBe(SOURCE.CAMERA);
    act(() => result.current.toggleMaximized());
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    const before = result.current.session;
    act(() => result.current.viewCamera(2));
    expect(result.current.source).toBe(SOURCE.CAMERA);
    expect(result.current.session).not.toBe(before);
    expect(result.current.swapped).toBe(true);
  });

  it('View camera always opens Camera, even for a UAV without streams', () => {
    const { result } = setup([entry(1)]);
    act(() => result.current.viewCamera(1));
    expect(result.current.source).toBe(SOURCE.CAMERA);
    expect(result.current.shown.kind).toBe('noCamera');
  });

  it('"Viewing" needs Camera, a shown stream of that UAV and the active window', () => {
    const fleet = [entry(1, [stream(100, 1)]), entry(2)];
    const { result, rerender } = setup(fleet);
    expect(result.current.viewing(1)).toBe(false);
    act(() => result.current.selectUav(1));
    expect(result.current.viewing(1)).toBe(true);
    expect(result.current.viewing(2)).toBe(false);
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    expect(result.current.viewing(1)).toBe(false);
    act(() => result.current.chooseSource(SOURCE.CAMERA));
    rerender({ fleet, active: false });
    expect(result.current.viewing(1)).toBe(false);
  });
});

describe('camera states', () => {
  it('stream removal: the same UAV\'s other stream, kept when the original returns', () => {
    const two = entry(1, [stream(100, 1), stream(101, 2)]);
    const { result, rerender } = setup([two, entry(2, [stream(100, 1)])]);
    act(() => result.current.selectUav(1));
    expect(result.current.shown.stream.key).toBe('100:1');
    rerender({ fleet: [entry(1, [stream(101, 2)]), entry(2, [stream(100, 1)])], active: true });
    expect(result.current.shown.stream.key).toBe('101:2');
    rerender({ fleet: [two, entry(2, [stream(100, 1)])], active: true });
    expect(result.current.shown.stream.key).toBe('101:2');
  });

  it('last stream gone on Camera: lost, never another UAV and never Synthetic by itself', () => {
    const { result, rerender } = setup([entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])]);
    act(() => result.current.selectUav(1));
    rerender({ fleet: [entry(1), entry(2, [stream(100, 1)])], active: true });
    expect(result.current.shown.kind).toBe('lost');
    expect(result.current.sysId).toBe(1);
    expect(result.current.source).toBe(SOURCE.CAMERA);
    rerender({ fleet: [entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])], active: true });
    expect(result.current.shown.kind).toBe('stream');
    expect(result.current.sysId).toBe(1);
  });

  it('Dismiss opens Synthetic for the same UAV, never another', () => {
    const { result, rerender } = setup([entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])]);
    act(() => result.current.selectUav(1));
    rerender({ fleet: [entry(1), entry(2, [stream(100, 1)])], active: true });
    act(() => result.current.dismiss());
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    expect(result.current.sysId).toBe(1);
  });

  it('the picked UAV leaves the fleet: the selection stays, lost; it resumes on return', () => {
    const fleet = [entry(1, [stream(100, 1)]), entry(2)];
    const { result, rerender } = setup(fleet);
    act(() => result.current.selectUav(1));
    rerender({ fleet: [entry(2)], active: true });
    expect(result.current.sysId).toBe(1);
    expect(result.current.shown.kind).toBe('lost');
    rerender({ fleet, active: true });
    expect(result.current.shown.kind).toBe('stream');
  });

  it('Reconnect video opens a fresh session on the same stream', () => {
    const { result } = setup([entry(1, [stream(100, 1)])]);
    act(() => result.current.selectUav(1));
    const before = result.current.session;
    act(() => result.current.reconnect());
    expect(result.current.session).not.toBe(before);
    expect(result.current.shown.stream.key).toBe('100:1');
  });
});

describe('source', () => {
  it('the switch: Synthetic, and back to Camera is a fresh connection', () => {
    const { result } = setup([entry(1, [stream(100, 1)])]);
    act(() => result.current.selectUav(1));
    const before = result.current.session;
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    act(() => result.current.chooseSource(SOURCE.CAMERA));
    expect(result.current.session).not.toBe(before);
  });

  it('a stream appearing or going away on Synthetic never changes the source', () => {
    const { result, rerender } = setup([entry(1)]);
    act(() => result.current.selectUav(1));
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    rerender({ fleet: [entry(1, [stream(100, 1)])], active: true });
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    rerender({ fleet: [entry(1)], active: true });
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
  });

  it('synthetic failure: Camera with a notice, cleared by the next operator action', () => {
    const { result } = setup([entry(1)]);
    act(() => result.current.selectUav(1));
    act(() => result.current.syntheticFailed());
    expect(result.current.source).toBe(SOURCE.CAMERA);
    expect(result.current.notice).toBe('syntheticUnavailable');
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    expect(result.current.notice).toBeNull();
  });

  it('RC entry or leaving the monitor view swaps back; the return re-picks the same UAV by its icon', () => {
    const { result, rerender } = setup([entry(1), entry(2)]);
    act(() => result.current.selectUav(1));
    act(() => result.current.chooseSource(SOURCE.CAMERA));
    act(() => result.current.toggleMaximized());
    rerender({ fleet: [entry(1), entry(2)], active: false });
    expect(result.current.swapped).toBe(false);
    rerender({ fleet: [entry(1), entry(2)], active: true });
    expect(result.current.sysId).toBe(1);
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    expect(result.current.swapped).toBe(false);
    // The icon at the return decides: a stream that appeared meanwhile gives Camera.
    rerender({ fleet: [entry(1), entry(2)], active: false });
    rerender({ fleet: [entry(1, [stream(100, 1)]), entry(2)], active: true });
    expect(result.current.source).toBe(SOURCE.CAMERA);
    expect(result.current.shown.kind).toBe('stream');
  });

  it('the picker stays the picker across RC', () => {
    const { result, rerender } = setup([entry(1)]);
    rerender({ fleet: [entry(1)], active: false });
    rerender({ fleet: [entry(1)], active: true });
    expect(result.current.sysId).toBeNull();
  });
});

describe('Close and the swap', () => {
  it('Close swaps back and returns to the picker', () => {
    const { result } = setup([entry(1, [stream(100, 1)])]);
    act(() => result.current.selectUav(1));
    act(() => result.current.toggleMaximized());
    act(() => result.current.close());
    expect(result.current.swapped).toBe(false);
    expect(result.current.sysId).toBeNull();
    expect(result.current.session).toBeNull();
    // The next maximize needs a new pick.
    act(() => result.current.selectUav(1));
    expect(result.current.swapped).toBe(false);
  });

  it('Restore or a tap on the small map swaps back; nothing reconnects', () => {
    const { result } = setup([entry(1, [stream(100, 1)])]);
    act(() => result.current.selectUav(1));
    const before = result.current.session;
    act(() => result.current.toggleMaximized());
    expect(result.current.swapped).toBe(true);
    act(() => result.current.unswap());
    expect(result.current.swapped).toBe(false);
    act(() => result.current.toggleMaximized());
    act(() => result.current.toggleMaximized());
    expect(result.current.swapped).toBe(false);
    expect(result.current.session).toBe(before);
  });
});
