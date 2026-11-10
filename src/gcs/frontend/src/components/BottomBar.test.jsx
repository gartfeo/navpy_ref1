import { describe, it, expect, vi } from 'vitest';
import { render } from '@testing-library/react';
import BottomBar from './BottomBar';
import { PHASES } from '../hooks/useMissionState';

// The shared stub prints keys only; the progress test needs the percentage.
vi.mock('react-i18next', () => ({
  useTranslation: () => ({ t: (key, p) => (p?.percentage != null ? `${key}=${p.percentage}` : key) }),
}));

// On a 375 px pane the stats wrapped and clipped, and Upload lay on top of them
// (docs/design/gcs_handheld_layout.md). jsdom has no layout: this checks the rules
// that keep the bar on one line; the live check measures them.
const ANALYSIS = {
  area_km2: 12, strip_count: 8, total_distance_km: 96, estimated_time_min: 80, required_uavs: 3, launch_zone_buffer_m: 2000,
};
const POLYGON = [{ lat: 0, lon: 0 }, { lat: 0, lon: 1 }, { lat: 1, lon: 1 }];

const planning = () => render(
  <BottomBar phase={PHASES.PLANNING} analysis={ANALYSIS} polygon={POLYGON} vehicleList={[{ sys_id: 7 }]} />,
);

describe('BottomBar in planning', () => {
  it('the stats keep one line and scroll sideways', () => {
    const view = planning();
    const stat = view.getByText('bottomBar.area:').parentElement;
    expect(stat.style.flexShrink).toBe('0');
    const strip = stat.parentElement;
    expect(strip.style.overflowX).toBe('auto');
    expect(strip.style.whiteSpace).toBe('nowrap');
    expect(strip.style.minWidth).toBe('0px');
  });

  // Three items shrunk into a phone's strip printed their bars and labels over each other.
  it('during an upload each UAV keeps its width and the strip scrolls sideways', () => {
    const view = render(
      <BottomBar
        phase={PHASES.PLANNING}
        analysis={ANALYSIS}
        polygon={POLYGON}
        vehicleList={[{ sys_id: 7, name: 's1-u7' }, { sys_id: 8, name: 's1-u8' }]}
        uploading
        uploadProgress={{ 7: { stage: 'clearing' }, 8: { stage: 'clearing' } }}
      />,
    );
    const item = view.getByText('s1-u7').parentElement.parentElement.parentElement;
    expect(item.style.flexShrink).toBe('0');
    expect(item.parentElement.style.overflowX).toBe('auto');
    expect(view.getByText('s1-u8').parentElement.parentElement.parentElement.parentElement).toBe(item.parentElement);
  });

  it('the buttons never shrink or wrap', () => {
    const view = planning();
    const upload = view.getByRole('button', { name: 'bottomBar.upload' });
    expect(upload.style.whiteSpace).toBe('nowrap');
    expect(view.getByRole('button', { name: 'bottomBar.exitPlan' }).style.whiteSpace).toBe('nowrap');
    expect(upload.parentElement.style.flexShrink).toBe('0');
  });
});

// The monitor's "plan not on the UAVs" fix opens plan mode with Upload ringed.
describe('BottomBar Upload as the fix', () => {
  const ringOf = (view) => view.getByRole('button', { name: 'bottomBar.upload' }).closest('[data-fix-highlight]');

  it('is ringed only when plan mode was opened to upload', () => {
    const plain = planning();
    expect(ringOf(plain)).toBeNull();
    plain.unmount();
    const view = render(
      <BottomBar phase={PHASES.PLANNING} analysis={ANALYSIS} polygon={POLYGON} vehicleList={[{ sys_id: 7 }]} highlightUpload />,
    );
    expect(ringOf(view)).not.toBeNull();
  });

  it('is not ringed while it cannot upload', () => {
    const view = render(
      <BottomBar phase={PHASES.PLANNING} analysis={null} polygon={[]} vehicleList={[{ sys_id: 7 }]} highlightUpload />,
    );
    expect(ringOf(view)).toBeNull();
  });
});

// Seen on the UniRC (2026-11-08): a mission made outside the GCS, downloaded,
// UAV in AUTO at item 3 of 5, read 0% complete (utils/missionProgress.test.js).
describe('BottomBar in monitoring', () => {
  it('counts progress on a downloaded mission by the vehicle\'s own item numbers', () => {
    const track = [
      { lat: 40.300, lon: 44.450, seq: 2 }, { lat: 40.310, lon: 44.450, seq: 3 }, { lat: 40.310, lon: 44.460, seq: 4 },
    ];
    const hull = [{ lat: 40.300, lon: 44.450 }, { lat: 40.310, lon: 44.450 }, { lat: 40.310, lon: 44.460 }];
    const view = render(
      <BottomBar
        phase={PHASES.MONITOR}
        polygon={hull}
        plan={{ zones: [{ zone_index: 0, track }] }}
        vehicleList={[{ sys_id: 1, link_ok: true, mode: 'AUTO', mission_progress: 4, lat: 40.310, lon: 44.455 }]}
      />,
    );
    // At item 4, the last track point, half way along the second leg: 78 %
    // of the track's length (the old offset rule gave 0 %).
    expect(view.getByText(/^bottomBar\.percentComplete=/).textContent).toBe('bottomBar.percentComplete=78');
  });
});
