// FPV view mode: every row of the design's transition table, eligibility on
// each entry path, and map-camera ownership (docs/design/gcs_video_streams.md
// section 5 "FPV").
import { describe, it, expect, vi } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import { useState } from 'react';
import useFpvView from './useFpvView';
import { PHASES } from './useMissionState';

const entry = (sysId, fpvEligible = true) => ({
  sysId, index: sysId - 1, label: `UAV ${sysId}`, streams: [], linkOk: fpvEligible, fpvEligible,
  holdReason: fpvEligible ? null : 'linkLost',
});

function setup({ fleet = [entry(1), entry(2), entry(3)], panelSysId = null, follow = null } = {}) {
  const camera = { capture: vi.fn(() => ({ saved: 'map' })), restore: vi.fn() };
  const mapCameraRef = { current: camera };
  const centerView = vi.fn();
  const notify = vi.fn();
  const hook = renderHook((props) => {
    const [followSysId, setFollowSysId] = useState(follow);
    const [rc, setRc] = useState({ enabled: false, target: null });
    const fpv = useFpvView({
      phase: props.phase ?? PHASES.MONITOR,
      followSysId, setFollowSysId,
      fleet: props.fleet,
      panelSysId: props.panelSysId,
      rcEnabled: rc.enabled, rcTarget: rc.target,
      mapCameraRef, centerView, notify,
    });
    return { fpv, followSysId, setFollowSysId, setRc };
  }, { initialProps: { fleet, panelSysId } });
  return { ...hook, camera, centerView, notify };
}

describe('FPV toggle', () => {
  it('works for a UAV without video, from Free it takes the panel UAV, and the selector names it', () => {
    const { result, camera } = setup({ panelSysId: 2 });
    act(() => result.current.fpv.toggleFpv());
    expect(result.current.fpv.fpvOn).toBe(true);
    expect(result.current.followSysId).toBe(2);
    expect(result.current.fpv.fpvSysId).toBe(2);
    expect(result.current.fpv.cameraFpv).toBe(true);
    // The map camera is taken as the view leaves the free map.
    expect(camera.capture).toHaveBeenCalledTimes(1);
  });

  it('from Free with an ineligible panel UAV takes the first eligible UAV', () => {
    const { result } = setup({ fleet: [entry(1, false), entry(2), entry(3)], panelSysId: 1 });
    act(() => result.current.fpv.toggleFpv());
    expect(result.current.followSysId).toBe(2);
  });

  it('shows the followed UAV', () => {
    const { result } = setup({ follow: 3, panelSysId: 1 });
    act(() => result.current.fpv.toggleFpv());
    expect(result.current.fpv.fpvSysId).toBe(3);
  });

  it('no eligible UAV: stays off with a notice', () => {
    const { result, notify } = setup({ fleet: [entry(1, false)] });
    act(() => result.current.fpv.toggleFpv());
    expect(result.current.fpv.fpvOn).toBe(false);
    expect(result.current.followSysId).toBeNull();
    expect(notify).toHaveBeenCalledWith('fpvView.noEligible');
  });

  it('ineligible followed UAV on entry: the view stays as it is, with a notice', () => {
    const { result, notify, camera } = setup({ fleet: [entry(1, false), entry(2)], follow: 1 });
    act(() => result.current.fpv.toggleFpv());
    expect(result.current.fpv.fpvOn).toBe(false);
    expect(result.current.followSysId).toBe(1);
    expect(notify).toHaveBeenCalledWith('fpvView.ineligible');
    expect(camera.capture).not.toHaveBeenCalled();
  });

  it('ineligible target on a follow change in FPV: refused, the selector keeps naming the FPV UAV', () => {
    const { result, notify } = setup({ fleet: [entry(1), entry(2, false), entry(3)] });
    act(() => result.current.fpv.toggleFpv());
    expect(result.current.followSysId).toBe(1);
    act(() => result.current.fpv.changeFollow(2));
    expect(result.current.followSysId).toBe(1);
    expect(result.current.fpv.fpvSysId).toBe(1);
    expect(notify).toHaveBeenCalledWith('fpvView.ineligible');
    // An eligible one is taken, without overwriting the saved map camera.
    act(() => result.current.fpv.changeFollow(3));
    expect(result.current.fpv.fpvSysId).toBe(3);
  });

  it('switching the FPV UAV does not overwrite the saved map camera', () => {
    const { result, camera } = setup();
    act(() => result.current.fpv.toggleFpv());
    act(() => result.current.fpv.changeFollow(2));
    act(() => result.current.fpv.changeFollow(3));
    expect(camera.capture).toHaveBeenCalledTimes(1);
  });

  it('never enables RC', () => {
    const { result } = setup();
    act(() => result.current.fpv.toggleFpv());
    expect(result.current.fpv.fpvKind).toBe('view');
  });
});

describe('FPV transition table', () => {
  it('FPV off: chase of the same UAV; the saved camera is restored when follow goes to Free', () => {
    const { result, camera, centerView } = setup();
    act(() => result.current.fpv.toggleFpv());
    act(() => result.current.fpv.toggleFpv());
    expect(result.current.fpv.fpvOn).toBe(false);
    expect(result.current.fpv.cameraFpv).toBe(false);
    expect(result.current.followSysId).toBe(1);
    expect(camera.restore).not.toHaveBeenCalled();

    act(() => result.current.fpv.changeFollow(null));
    expect(camera.restore).toHaveBeenCalledWith({ saved: 'map' });
    expect(centerView).not.toHaveBeenCalled();
  });

  it('Follow set to Free (FPV on): FPV off, saved map camera restored', () => {
    const { result, camera } = setup();
    act(() => result.current.fpv.toggleFpv());
    act(() => result.current.fpv.changeFollow(null));
    expect(result.current.fpv.fpvOn).toBe(false);
    expect(camera.restore).toHaveBeenCalledTimes(1);
  });

  it('Followed UAV removed: as Free', () => {
    const { result, camera } = setup();
    act(() => result.current.fpv.toggleFpv());
    // App clears follow when the vehicle leaves the list.
    act(() => result.current.setFollowSysId(null));
    expect(result.current.fpv.fpvOn).toBe(false);
    expect(camera.restore).toHaveBeenCalledTimes(1);
  });

  it('View camera (FPV on): FPV off as above', () => {
    const { result } = setup();
    act(() => result.current.fpv.toggleFpv());
    act(() => result.current.fpv.leaveFpv());
    expect(result.current.fpv.fpvOn).toBe(false);
    expect(result.current.followSysId).toBe(1);
  });

  it('RC entry (FPV on): RC FPV of the RC target, FPV view off', () => {
    const { result } = setup();
    act(() => result.current.fpv.toggleFpv());
    act(() => result.current.setRc({ enabled: true, target: null }));
    expect(result.current.fpv.fpvOn).toBe(false);
    expect(result.current.fpv.cameraFpv).toBe(false);
    expect(result.current.fpv.cameraOwned).toBe(true);
    act(() => result.current.setRc({ enabled: true, target: 2 }));
    expect(result.current.fpv.fpvKind).toBe('rc');
    expect(result.current.fpv.fpvSysId).toBe(2);
  });

  it('RC entry (FPV off, from the map): the map camera is saved', () => {
    const { result, camera } = setup();
    act(() => result.current.setRc({ enabled: true, target: null }));
    expect(camera.capture).toHaveBeenCalledTimes(1);
  });

  it('RC exit: Free map with the saved map camera; FPV off', () => {
    const { result, camera, centerView } = setup();
    act(() => result.current.fpv.toggleFpv());
    act(() => result.current.setRc({ enabled: true, target: 1 }));
    act(() => {
      result.current.setRc({ enabled: false, target: null });
      result.current.setFollowSysId(null);
    });
    expect(result.current.fpv.fpvOn).toBe(false);
    expect(result.current.fpv.cameraFpv).toBe(false);
    expect(camera.restore).toHaveBeenCalledTimes(1);
    expect(centerView).not.toHaveBeenCalled();
  });

  it('chase follow without FPV or RC keeps today\'s auto-center on Free', () => {
    const { result, camera, centerView } = setup();
    act(() => result.current.fpv.changeFollow(2));
    act(() => result.current.fpv.changeFollow(null));
    // Saved on leaving the map (the chase may become FPV), but not restored.
    expect(camera.restore).not.toHaveBeenCalled();
    expect(centerView).toHaveBeenCalledTimes(1);
  });

  it('Free → chase → FPV → Free restores the map camera saved before the chase', () => {
    const { result, camera, centerView } = setup();
    act(() => result.current.fpv.changeFollow(2));
    act(() => result.current.fpv.toggleFpv());
    act(() => result.current.fpv.changeFollow(null));
    expect(camera.capture).toHaveBeenCalledTimes(1);
    expect(camera.restore).toHaveBeenCalledWith({ saved: 'map' });
    expect(centerView).not.toHaveBeenCalled();
  });

  it('Free → chase → RC → exit restores the map camera saved before the chase', () => {
    const { result, camera, centerView } = setup();
    act(() => result.current.fpv.changeFollow(2));
    act(() => result.current.setRc({ enabled: true, target: 2 }));
    act(() => {
      result.current.setRc({ enabled: false, target: null });
      result.current.setFollowSysId(null);
    });
    expect(camera.restore).toHaveBeenCalledWith({ saved: 'map' });
    expect(centerView).not.toHaveBeenCalled();
  });

  it('camera persistence is suspended exactly while FPV or RC owns the camera', () => {
    const { result } = setup();
    expect(result.current.fpv.cameraOwned).toBe(false);
    act(() => result.current.fpv.toggleFpv());
    expect(result.current.fpv.cameraOwned).toBe(true);
    act(() => result.current.fpv.changeFollow(null));
    expect(result.current.fpv.cameraOwned).toBe(false);
  });

  it('leaving the monitor view ends FPV', () => {
    const { result, rerender } = setup();
    act(() => result.current.fpv.toggleFpv());
    rerender({ fleet: [entry(1), entry(2), entry(3)], panelSysId: null, phase: PHASES.PLANNING });
    expect(result.current.fpv.fpvOn).toBe(false);
  });
});
