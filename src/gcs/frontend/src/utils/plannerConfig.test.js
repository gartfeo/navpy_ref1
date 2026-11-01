import { describe, it, expect } from 'vitest';
import { configurePlanner, getConfig } from './plannerConfig';

const dims = { 0: { width_m: 4, height_m: 4, size_m: 4 } };
const profile = (pitch, altitude) => ({
  devices: [{
    name: 'cam', image_width: 1280, image_height: 720, pitch_deg: pitch,
    zooms: { 1: { fx: 1400, fy: 1400, detect_range_m: 800 } },
  }],
  dock_presets: { dock: { altitude_m: altitude } },
});
const catalog = { def: profile(-14, 200), other: profile(-30, 150) };

describe('configurePlanner camera profile', () => {
  it('uses the catalog default profile when the settings select none', () => {
    configurePlanner({ camera: { vision_profile: null } }, catalog, dims, 'def');
    const c = getConfig();
    expect(c.DOCK_PRESETS.dock.altitude_m).toBe(200);
    expect(c.CAMERA_PITCH_DEG).toBe(-14);
  });

  it('uses the selected profile over the default', () => {
    configurePlanner({ camera: { vision_profile: 'other' } }, catalog, dims, 'def');
    expect(getConfig().DOCK_PRESETS.dock.altitude_m).toBe(150);
  });

  it('falls back to the default when the selection is not in the catalog', () => {
    configurePlanner({ camera: { vision_profile: 'gone' } }, catalog, dims, 'def');
    expect(getConfig().DOCK_PRESETS.dock.altitude_m).toBe(200);
  });
});
