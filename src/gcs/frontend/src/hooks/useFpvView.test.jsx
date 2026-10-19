// RC FPV and map-camera ownership (docs/design/gcs_video_streams.md section 5,
// "Synthetic view" RC and "Map camera ownership": only RC takes the map camera).
import { describe, it, expect, vi } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import { useState } from 'react';
import useFpvView from './useFpvView';
import { PHASES } from './useMissionState';

function setup({ follow = null } = {}) {
  const camera = { capture: vi.fn(() => ({ saved: 'map' })), restore: vi.fn() };
  const mapCameraRef = { current: camera };
  const centerView = vi.fn();
  const hook = renderHook((props) => {
    const [followSysId, setFollowSysId] = useState(follow);
    const [rc, setRc] = useState({ enabled: false, target: null });
    const fpv = useFpvView({
      phase: props.phase ?? PHASES.MONITOR,
      followSysId,
      rcEnabled: rc.enabled, rcTarget: rc.target,
      mapCameraRef, centerView,
    });
    return { fpv, followSysId, setFollowSysId, setRc };
  }, { initialProps: {} });
  return { ...hook, camera, centerView };
}

describe('RC FPV', () => {
  it('no FPV outside RC; following a UAV is a chase, not first-person', () => {
    const { result } = setup();
    act(() => result.current.setFollowSysId(2));
    expect(result.current.fpv.cameraFpv).toBe(false);
    expect(result.current.fpv.fpvSysId).toBeNull();
    expect(result.current.fpv.cameraOwned).toBe(false);
  });

  it('RC entry: RC FPV of the RC target, once it has one', () => {
    const { result } = setup();
    act(() => result.current.setRc({ enabled: true, target: null }));
    expect(result.current.fpv.cameraFpv).toBe(false);
    expect(result.current.fpv.cameraOwned).toBe(true);
    act(() => result.current.setRc({ enabled: true, target: 2 }));
    expect(result.current.fpv.cameraFpv).toBe(true);
    expect(result.current.fpv.fpvSysId).toBe(2);
  });

  it('RC entry from the free map: the map camera is saved', () => {
    const { result, camera } = setup();
    act(() => result.current.setRc({ enabled: true, target: null }));
    expect(camera.capture).toHaveBeenCalledTimes(1);
  });

  it('RC exit: Free map with the saved map camera, no auto-center', () => {
    const { result, camera, centerView } = setup();
    act(() => result.current.setRc({ enabled: true, target: 1 }));
    act(() => result.current.setRc({ enabled: false, target: null }));
    expect(result.current.fpv.cameraFpv).toBe(false);
    expect(camera.restore).toHaveBeenCalledWith({ saved: 'map' });
    expect(centerView).not.toHaveBeenCalled();
  });

  it('chase back to Free keeps today\'s auto-center and restores nothing', () => {
    const { result, camera, centerView } = setup();
    act(() => result.current.setFollowSysId(2));
    act(() => result.current.setFollowSysId(null));
    // Saved on leaving the map (the chase may become RC), but not restored.
    expect(camera.capture).toHaveBeenCalledTimes(1);
    expect(camera.restore).not.toHaveBeenCalled();
    expect(centerView).toHaveBeenCalledTimes(1);
  });

  it('Free → chase → RC → exit restores the map camera saved before the chase', () => {
    const { result, camera, centerView } = setup();
    act(() => result.current.setFollowSysId(2));
    act(() => result.current.setRc({ enabled: true, target: 2 }));
    act(() => {
      result.current.setRc({ enabled: false, target: null });
      result.current.setFollowSysId(null);
    });
    expect(camera.capture).toHaveBeenCalledTimes(1);
    expect(camera.restore).toHaveBeenCalledWith({ saved: 'map' });
    expect(centerView).not.toHaveBeenCalled();
  });

  it('camera persistence is suspended exactly while RC owns the camera', () => {
    const { result } = setup();
    act(() => result.current.setFollowSysId(1));
    expect(result.current.fpv.cameraOwned).toBe(false);
    act(() => result.current.setRc({ enabled: true, target: 1 }));
    expect(result.current.fpv.cameraOwned).toBe(true);
    act(() => result.current.setRc({ enabled: false, target: null }));
    expect(result.current.fpv.cameraOwned).toBe(false);
  });

  it('a chase ending outside the monitor view does not auto-center', () => {
    const { result, rerender, centerView } = setup();
    act(() => result.current.setFollowSysId(2));
    rerender({ phase: PHASES.PLANNING });
    act(() => result.current.setFollowSysId(null));
    expect(centerView).not.toHaveBeenCalled();
  });
});
