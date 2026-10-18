import { describe, it, expect } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import useVideoFleet, { summarizeFleet } from './useVideoFleet';

function makeStore(list) {
  let version = 0;
  const listeners = new Set();
  return {
    list,
    subscribe: (fn) => { listeners.add(fn); return () => listeners.delete(fn); },
    getSnapshot: () => version,
    getVehicleList() { return this.list; },
    set(next) { this.list = next; version += 1; listeners.forEach((fn) => fn()); },
  };
}

const v = (extra) => ({ sys_id: 1, link_ok: true, lat: 1, lon: 2, alt_rel: 3, roll: 0, pitch: 0, heading: 0, ...extra });

describe('useVideoFleet', () => {
  it('summarizes streams, link and FPV eligibility per vehicle', () => {
    const [one, two] = summarizeFleet([v({ video_streams: [{ key: '100:1', whep_url: 'http://x/whep', stream_id: 1 }] }), v({ sys_id: 2, link_ok: false })]);
    expect(one).toMatchObject({ sysId: 1, index: 0, label: 'UAV 1', linkOk: true, fpvEligible: true });
    expect(one.streams).toHaveLength(1);
    expect(two).toMatchObject({ sysId: 2, index: 1, linkOk: false, fpvEligible: false, holdReason: 'linkLost', streams: [] });
  });

  it('keeps the same array across position-only updates, and changes on a stream change', () => {
    const store = makeStore([v()]);
    const { result } = renderHook(() => useVideoFleet({ current: store }));
    const first = result.current;
    act(() => store.set([v({ lat: 1.5 })]));
    expect(result.current).toBe(first);
    act(() => store.set([v({ video_streams: [{ key: '100:1', whep_url: 'http://x/whep', stream_id: 1 }] })]));
    expect(result.current).not.toBe(first);
    expect(result.current[0].streams).toHaveLength(1);
  });
});
