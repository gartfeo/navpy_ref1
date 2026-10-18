import { describe, it, expect } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import useVideoSelection from './useVideoSelection';

const stream = (camera, id) => ({
  key: `${camera}:${id}`, camera_component: camera, stream_id: id, name: 'tracking overlay',
  encoding: 'H264', whep_url: `http://192.168.144.1${camera % 10}:8889/s${id}/whep`,
});
const entry = (sysId, streams = []) => ({
  sysId, index: sysId - 1, label: `UAV ${sysId}`, streams, linkOk: true, fpvEligible: true, holdReason: null,
});

const setup = (fleet, followSysId = null) =>
  renderHook((props) => useVideoSelection(props), { initialProps: { fleet, followSysId } });

describe('useVideoSelection', () => {
  it('no streams: no panel and nothing selected', () => {
    const { result } = setup([entry(1), entry(2)]);
    expect(result.current.hasPanel).toBe(false);
    expect(result.current.selection).toBeNull();
  });

  it('the follow selector picks the first feed: the followed UAV with video', () => {
    const { result } = setup([entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])], 2);
    expect(result.current.selection).toMatchObject({ sysId: 2, key: '100:1' });
  });

  it('first feed falls back to the first UAV with video when the followed one has none', () => {
    const { result } = setup([entry(1), entry(2, [stream(100, 1)]), entry(3, [stream(100, 1)])], 1);
    expect(result.current.selection.sysId).toBe(2);
  });

  it('D1: later follow changes keep the video', () => {
    const fleet = [entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])];
    const { result, rerender } = setup(fleet, 1);
    expect(result.current.selection.sysId).toBe(1);
    rerender({ fleet, followSysId: 2 });
    rerender({ fleet, followSysId: null });
    expect(result.current.selection.sysId).toBe(1);
  });

  it('badges switch the video; the selected badge is a no-op', () => {
    const fleet = [entry(1, [stream(100, 1)]), entry(2, [stream(100, 2), stream(101, 1)])];
    const { result } = setup(fleet);
    const before = result.current.session;
    act(() => result.current.selectUav(1));
    expect(result.current.session).toBe(before);
    act(() => result.current.selectUav(2));
    // Default stream = the lowest stream id.
    expect(result.current.selection).toMatchObject({ sysId: 2, key: '101:1' });
  });

  it('View camera always opens a fresh session, even for the shown UAV', () => {
    const fleet = [entry(1, [stream(100, 1)])];
    const { result } = setup(fleet);
    const before = result.current.session;
    act(() => result.current.setCollapsed(true));
    act(() => result.current.viewCamera(1));
    expect(result.current.session).not.toBe(before);
    expect(result.current.collapsed).toBe(false);
  });

  it('stream removal: the same UAV\'s other stream, kept when the original returns', () => {
    const two = entry(1, [stream(100, 1), stream(101, 2)]);
    const { result, rerender } = setup([two, entry(2, [stream(100, 1)])]);
    expect(result.current.selection.key).toBe('100:1');

    rerender({ fleet: [entry(1, [stream(101, 2)]), entry(2, [stream(100, 1)])], followSysId: null });
    expect(result.current.shown.stream.key).toBe('101:2');
    expect(result.current.selection.key).toBe('101:2');

    rerender({ fleet: [two, entry(2, [stream(100, 1)])], followSysId: null });
    expect(result.current.selection.key).toBe('101:2');
  });

  it('last stream of the UAV gone: lost, never another UAV, until dismissed', () => {
    const { result, rerender } = setup([entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])]);
    rerender({ fleet: [entry(1), entry(2, [stream(100, 1)])], followSysId: null });
    expect(result.current.shown.kind).toBe('lost');
    expect(result.current.selection.sysId).toBe(1);

    // It comes back: the same UAV plays again.
    rerender({ fleet: [entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])], followSysId: null });
    expect(result.current.shown.kind).toBe('stream');
    expect(result.current.selection.sysId).toBe(1);
  });

  it('panel gone only after the last lost feed is dismissed and nothing is advertised', () => {
    const { result, rerender } = setup([entry(1, [stream(100, 1)])]);
    rerender({ fleet: [entry(1)], followSysId: null });
    expect(result.current.hasPanel).toBe(true);
    act(() => result.current.dismiss());
    expect(result.current.hasPanel).toBe(false);
  });

  it('after Dismiss with other streams the panel stays idle; no auto-switch', () => {
    const fleet = [entry(1, [stream(100, 1)]), entry(2, [stream(100, 1)])];
    const { result } = setup(fleet);
    act(() => result.current.dismiss());
    expect(result.current.hasPanel).toBe(true);
    expect(result.current.shown.kind).toBe('idle');
  });

  it('Reconnect video opens a fresh session on the same stream', () => {
    const { result } = setup([entry(1, [stream(100, 1)])]);
    const before = result.current.session;
    act(() => result.current.reconnect());
    expect(result.current.session).not.toBe(before);
    expect(result.current.selection.key).toBe('100:1');
  });
});
