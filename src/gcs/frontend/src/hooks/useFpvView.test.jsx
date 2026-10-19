// RC FPV and map-camera ownership (docs/design/gcs_video_streams.md section 5,
// "Synthetic view" RC and "Map camera ownership": only RC takes the map camera).
import { describe, it, expect, vi } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import { useState } from 'react';
import useFpvView from './useFpvView';

function setup() {
  const camera = { capture: vi.fn(() => ({ saved: 'map' })), restore: vi.fn() };
  const mapCameraRef = { current: camera };
  const hook = renderHook(() => {
    const [rc, setRc] = useState({ enabled: false, target: null });
    const fpv = useFpvView({ rcEnabled: rc.enabled, rcTarget: rc.target, mapCameraRef });
    return { fpv, setRc };
  });
  return { ...hook, camera };
}

describe('RC FPV', () => {
  it('outside RC: no FPV, the camera is not owned, nothing saved', () => {
    const { result, camera } = setup();
    expect(result.current.fpv.fpvSysId).toBeNull();
    expect(result.current.fpv.cameraOwned).toBe(false);
    expect(camera.capture).not.toHaveBeenCalled();
  });

  it('RC entry: the map camera is saved once; RC FPV of the RC target, once it has one', () => {
    const { result, camera } = setup();
    act(() => result.current.setRc({ enabled: true, target: null }));
    expect(result.current.fpv.fpvSysId).toBeNull();
    expect(result.current.fpv.cameraOwned).toBe(true);
    act(() => result.current.setRc({ enabled: true, target: 2 }));
    expect(result.current.fpv.fpvSysId).toBe(2);
    act(() => result.current.setRc({ enabled: true, target: 3 }));
    expect(result.current.fpv.fpvSysId).toBe(3);
    expect(camera.capture).toHaveBeenCalledTimes(1);
  });

  it('RC exit: the saved map camera is restored and the camera released', () => {
    const { result, camera } = setup();
    act(() => result.current.setRc({ enabled: true, target: 1 }));
    act(() => result.current.setRc({ enabled: false, target: null }));
    expect(result.current.fpv.fpvSysId).toBeNull();
    expect(result.current.fpv.cameraOwned).toBe(false);
    expect(camera.restore).toHaveBeenCalledTimes(1);
    expect(camera.restore).toHaveBeenCalledWith({ saved: 'map' });
  });

  it('a second RC session saves and restores its own camera', () => {
    const { result, camera } = setup();
    act(() => result.current.setRc({ enabled: true, target: 1 }));
    act(() => result.current.setRc({ enabled: false, target: null }));
    camera.capture.mockReturnValueOnce({ saved: 'second' });
    act(() => result.current.setRc({ enabled: true, target: 2 }));
    act(() => result.current.setRc({ enabled: false, target: null }));
    expect(camera.restore).toHaveBeenLastCalledWith({ saved: 'second' });
  });
});
