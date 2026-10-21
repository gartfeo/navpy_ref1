// Vehicle window state: open or closed, the picker, the UAV pick, the camera states, the
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

const setupClosed = (fleet, active = true) =>
  renderHook((props) => useVehicleWindow(props), { initialProps: { fleet, active } });
/** The window opened with the video icon, as most cases need. */
function setup(fleet, active = true) {
  const hook = setupClosed(fleet, active);
  act(() => hook.result.current.toggleOpen());
  return hook;
}

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
    act(() => result.current.toggleOpen());
    expect(get).not.toHaveBeenCalled();
    expect(set).not.toHaveBeenCalled();
  });

  it('Maximize does nothing in the picker', () => {
    const { result } = setup([entry(1)]);
    act(() => result.current.toggleMaximized());
    expect(result.current.swapped).toBe(false);
  });

  it('an empty fleet closes the window and clears the selection and the swap; the next fleet starts closed', () => {
    const fleet = [entry(1), entry(2, [stream(100, 1)])];
    const { result, rerender } = setup(fleet);
    act(() => result.current.selectUav(2));
    act(() => result.current.toggleMaximized());
    rerender({ fleet: [], active: true });
    expect(result.current.open).toBe(false);
    expect(result.current.sysId).toBeNull();
    expect(result.current.swapped).toBe(false);
    rerender({ fleet, active: true });
    expect(result.current.open).toBe(false);
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

  it('View camera is refused for a UAV without a stream: nothing opens or changes', () => {
    const { result } = setupClosed([entry(1), entry(2, [stream(100, 1)])]);
    act(() => result.current.viewCamera(1));
    expect(result.current.open).toBe(false);
    expect(result.current.sysId).toBeNull();
    act(() => result.current.viewCamera(2));
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    const before = result.current.session;
    act(() => result.current.viewCamera(1));
    expect(result.current.sysId).toBe(2);
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    expect(result.current.session).toBe(before);
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

  it('the picked UAV on Synthetic leaves the fleet: it stays selected, Camera unavailable; nothing switches on return', () => {
    const fleet = [entry(1, [stream(100, 1)]), entry(2)];
    const { result, rerender } = setup(fleet);
    act(() => result.current.selectUav(2));
    rerender({ fleet: [entry(1, [stream(100, 1)])], active: true });
    expect(result.current.sysId).toBe(2);
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    expect(result.current.cameraAvailable).toBe(false);
    rerender({ fleet: [entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])], active: true });
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    expect(result.current.cameraAvailable).toBe(true);
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

  it('Camera without a stream: refused with the reason, no source change; cleared by the next source change or pick', () => {
    const { result, rerender } = setup([entry(1), entry(2)]);
    act(() => result.current.selectUav(1));
    expect(result.current.cameraAvailable).toBe(false);
    const before = result.current.session;
    act(() => result.current.chooseSource(SOURCE.CAMERA));
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    expect(result.current.session).toBe(before);
    expect(result.current.cameraRefused).toBe(true);
    // RC and closing keep it: only a source change or a pick clears it.
    rerender({ fleet: [entry(1), entry(2)], active: false });
    rerender({ fleet: [entry(1), entry(2)], active: true });
    act(() => result.current.toggleOpen());
    act(() => result.current.toggleOpen());
    expect(result.current.cameraRefused).toBe(true);
    act(() => result.current.selectUav(2));
    expect(result.current.cameraRefused).toBe(false);
  });

  it('Camera still pressed after its last stream went: activating it says why, nothing changes', () => {
    const { result, rerender } = setup([entry(1, [stream(100, 1)])]);
    act(() => result.current.selectUav(1));
    rerender({ fleet: [entry(1)], active: true });
    expect(result.current.shown.kind).toBe('lost');
    const before = result.current.session;
    act(() => result.current.chooseSource(SOURCE.CAMERA));
    expect(result.current.cameraRefused).toBe(true);
    expect(result.current.source).toBe(SOURCE.CAMERA);
    expect(result.current.session).toBe(before);
  });

  it("Camera availability is the selected UAV's own: another UAV's stream never counts", () => {
    const { result, rerender } = setup([entry(1), entry(2, [stream(100, 1)])]);
    act(() => result.current.selectUav(1));
    expect(result.current.cameraAvailable).toBe(false);
    rerender({ fleet: [entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])], active: true });
    // Available now, but nothing switches.
    expect(result.current.cameraAvailable).toBe(true);
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    act(() => result.current.chooseSource(SOURCE.CAMERA));
    expect(result.current.source).toBe(SOURCE.CAMERA);
    expect(result.current.shown.kind).toBe('stream');
    expect(result.current.shown.stream.key).toBe('100:1');
  });

  it('synthetic failure keeps Synthetic, latched across streams; Retry clears it', () => {
    const { result, rerender } = setup([entry(1, [stream(100, 1)])]);
    act(() => result.current.selectUav(1));
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    act(() => result.current.reportSyntheticFailure());
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    expect(result.current.syntheticFailed).toBe(true);
    rerender({ fleet: [entry(1, [stream(100, 1), stream(101, 2)])], active: true });
    expect(result.current.syntheticFailed).toBe(true);
    act(() => result.current.retrySynthetic());
    expect(result.current.syntheticFailed).toBe(false);
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
  });

  it('a synthetic failure is cleared by a pick or a source change', () => {
    const { result } = setup([entry(1, [stream(100, 1)]), entry(2)]);
    act(() => result.current.selectUav(2));
    act(() => result.current.reportSyntheticFailure());
    act(() => result.current.selectUav(1));
    expect(result.current.syntheticFailed).toBe(false);
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    act(() => result.current.reportSyntheticFailure());
    act(() => result.current.chooseSource(SOURCE.CAMERA));
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    expect(result.current.syntheticFailed).toBe(false);
  });

  it('Dismiss then a returning stream: Synthetic stays', () => {
    const { result, rerender } = setup([entry(1, [stream(100, 1)])]);
    act(() => result.current.selectUav(1));
    rerender({ fleet: [entry(1)], active: true });
    act(() => result.current.dismiss());
    rerender({ fleet: [entry(1, [stream(100, 1)])], active: true });
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    expect(result.current.cameraAvailable).toBe(true);
  });

  it('RC entry or leaving the monitor view swaps back; the return keeps the UAV and its source', () => {
    const { result, rerender } = setup([entry(1, [stream(100, 1)]), entry(2)]);
    act(() => result.current.selectUav(1));
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    act(() => result.current.toggleMaximized());
    rerender({ fleet: [entry(1, [stream(100, 1)]), entry(2)], active: false });
    expect(result.current.swapped).toBe(false);
    rerender({ fleet: [entry(1, [stream(100, 1)]), entry(2)], active: true });
    expect(result.current.sysId).toBe(1);
    // A camera UAV on Synthetic stays on Synthetic (v12 re-picked Camera from its icon).
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    expect(result.current.swapped).toBe(false);
    // On Camera, the return is a fresh connection.
    act(() => result.current.chooseSource(SOURCE.CAMERA));
    const before = result.current.session;
    rerender({ fleet: [entry(1, [stream(100, 1)]), entry(2)], active: false });
    rerender({ fleet: [entry(1, [stream(100, 1)]), entry(2)], active: true });
    expect(result.current.source).toBe(SOURCE.CAMERA);
    expect(result.current.session).not.toBe(before);
  });

  it('the picker stays the picker across RC', () => {
    const { result, rerender } = setup([entry(1)]);
    rerender({ fleet: [entry(1)], active: false });
    rerender({ fleet: [entry(1)], active: true });
    expect(result.current.sysId).toBeNull();
  });
});

describe('open, closed and the swap', () => {
  it('starts closed with no UAV picked; the video icon opens it on the picker', () => {
    const { result } = setupClosed([entry(1, [stream(100, 1)])]);
    expect(result.current.open).toBe(false);
    expect(result.current.sysId).toBeNull();
    act(() => result.current.toggleMaximized());
    expect(result.current.swapped).toBe(false);
    act(() => result.current.toggleOpen());
    expect(result.current.open).toBe(true);
    expect(result.current.sysId).toBeNull();
  });

  it('closing swaps back and keeps the UAV and source; opening again is a fresh session', () => {
    const { result } = setup([entry(1, [stream(100, 1)]), entry(2)]);
    act(() => result.current.selectUav(1));
    act(() => result.current.chooseSource(SOURCE.SYNTHETIC));
    act(() => result.current.toggleMaximized());
    const before = result.current.session;
    act(() => result.current.toggleOpen());
    expect(result.current.open).toBe(false);
    expect(result.current.swapped).toBe(false);
    expect(result.current.viewing(1)).toBe(false);
    act(() => result.current.toggleOpen());
    expect(result.current.open).toBe(true);
    expect(result.current.sysId).toBe(1);
    expect(result.current.source).toBe(SOURCE.SYNTHETIC);
    expect(result.current.swapped).toBe(false);
    expect(result.current.session).not.toBe(before);
  });

  it('View camera opens a closed window on the camera of that UAV', () => {
    const { result } = setupClosed([entry(1), entry(2, [stream(100, 1)])]);
    act(() => result.current.viewCamera(2));
    expect(result.current.open).toBe(true);
    expect(result.current.sysId).toBe(2);
    expect(result.current.source).toBe(SOURCE.CAMERA);
    expect(result.current.viewing(2)).toBe(true);
  });

  it('open or closed is kept across RC', () => {
    const { result, rerender } = setup([entry(1)]);
    rerender({ fleet: [entry(1)], active: false });
    rerender({ fleet: [entry(1)], active: true });
    expect(result.current.open).toBe(true);
    act(() => result.current.toggleOpen());
    rerender({ fleet: [entry(1)], active: false });
    rerender({ fleet: [entry(1)], active: true });
    expect(result.current.open).toBe(false);
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
