// The first-person HUD shared by RC control and the Synthetic picture
// (docs/design/gcs_video_streams.md section 5, "Synthetic view").
import { describe, it, expect, vi } from 'vitest';
import { render } from '@testing-library/react';
import TelemetryHud from './TelemetryHud';
import { pictureHudLayout, rcHudLayout } from '../utils/hudLayout';

const { vehicles } = vi.hoisted(() => ({ vehicles: {} }));
vi.mock('../hooks/useTelemetryStore', () => ({
  useTelemetryStore: (selector) => selector({ getVehicles: () => vehicles }),
}));
// Canvas tapes: record their size props only.
const { tape } = vi.hoisted(() => ({
  tape: (name) => ({ default: (p) => <div data-testid={name} data-width={p.width} data-height={p.height} /> }),
}));
vi.mock('./hud/HeadingTape', () => tape('heading'));
vi.mock('./hud/SpeedTape', () => tape('speed'));
vi.mock('./hud/AltitudeTape', () => tape('altitude'));
vi.mock('./hud/AttitudeIndicator', () => ({ default: () => <div data-testid="attitude" /> }));

vehicles[1] = {
  air_speed: 20, ground_speed: 21, alt_rel: 100, heading: 90, pitch: 1, roll: 2,
  battery: 80, voltage: 12.1, current: 3, status_texts: [], gps_fix: 3, gps_hacc: 1, link_ok: true, link_quality: 90,
};

const rc = rcHudLayout({ tapeBox: { top: 100, height: 200 }, speedBox: { left: 12, top: 120, height: 180 } });
const picture = pictureHudLayout({
  width: 318, height: 179, tagHeight: 24, status: { width: 100, height: 26 }, creditsPx: 16, smallMap: null,
}).hud;

describe('TelemetryHud', () => {
  it.each([['RC control', rc], ['Synthetic picture', picture]])(
    'in %s: display only, no control of any kind, every part where the layout puts it',
    (_host, layout) => {
      const { container, getByTestId } = render(<TelemetryHud sysId={1} layout={layout} />);
      expect(container.querySelectorAll('button, input, select, a, [role="slider"], [tabindex]')).toHaveLength(0);
      for (const el of container.children) expect(el.style.pointerEvents).toBe('none');
      expect(getByTestId('heading').dataset).toMatchObject({
        width: String(layout.heading.width), height: String(layout.heading.height),
      });
      expect(getByTestId('altitude').dataset.height).toBe(String(layout.altitude.height));
      expect(getByTestId('speed').dataset.width).toBe(String(layout.speed.width));
      expect(getByTestId('hud-status').style.right).toBe(`${layout.status.right}px`);
      expect(getByTestId('hud-attitude').style.right).toBe(`${layout.attitude.right}px`);
    },
  );

  it('RC control: the heading tape centred, as before', () => {
    const { getByTestId } = render(<TelemetryHud sysId={1} layout={rc} />);
    expect(getByTestId('heading').parentElement.style.transform).toBe('translateX(-50%)');
  });

  it('the attitude indicator between its side insets (maximized Synthetic: clear of the small map)', () => {
    const { getByTestId } = render(<TelemetryHud sysId={1} layout={{ ...picture, attitude: { left: 344, right: 344 } }} />);
    expect(getByTestId('hud-attitude').style).toMatchObject({ left: '344px', right: '344px', top: '0px', bottom: '0px' });
  });

  it('a null tape is not drawn', () => {
    const { queryByTestId } = render(<TelemetryHud sysId={1} layout={{ ...picture, speed: null, altitude: null }} />);
    expect(queryByTestId('speed')).toBeNull();
    expect(queryByTestId('altitude')).toBeNull();
  });

  it('nothing for a UAV without telemetry', () => {
    const { container } = render(<TelemetryHud sysId={9} layout={rc} />);
    expect(container.innerHTML).toBe('');
  });
});
