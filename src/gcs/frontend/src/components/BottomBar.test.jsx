import { describe, it, expect } from 'vitest';
import { render } from '@testing-library/react';
import BottomBar from './BottomBar';
import { PHASES } from '../hooks/useMissionState';

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
