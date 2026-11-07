// The first-person HUD shared by RC control and the Synthetic picture
// (docs/design/gcs_video_streams.md section 5, "Synthetic view").
import { describe, it, expect, vi } from 'vitest';
import { render } from '@testing-library/react';
import TelemetryHud from './TelemetryHud';
import { pictureHudLayout, rcHudLayout } from '../utils/hudLayout';

vi.mock('react-i18next', () => ({
  useTranslation: () => ({
    t: (key, o) => (o ? `${key}(${Object.entries(o).map(([k, v]) => `${k}=${v}`).join(',')})` : key),
  }),
}));
const { vehicles } = vi.hoisted(() => ({ vehicles: {} }));
vi.mock('../hooks/useTelemetryStore', () => ({
  useTelemetryStore: (selector) => selector({ getVehicles: () => vehicles }),
}));
// Canvas parts: record their props only.
const { tape } = vi.hoisted(() => ({
  tape: (name) => ({ default: (p) => <div data-testid={name} data-width={p.width} data-height={p.height} /> }),
}));
vi.mock('./hud/HeadingTape', () => tape('heading'));
vi.mock('./hud/SpeedTape', () => tape('speed'));
vi.mock('./hud/AltitudeTape', () => tape('altitude'));
vi.mock('./hud/AttitudeIndicator', () => ({
  default: (p) => <div data-testid="attitude" data-scale={String(p.scale)} />,
}));
vi.mock('./hud/StatusStrip', () => ({ default: (p) => <div data-testid="strip" data-max={p.maxMessages} /> }));
vi.mock('./hud/BatteryIndicator', () => ({ default: () => <div data-testid="battery" /> }));
vi.mock('./hud/StatusBarIndicator', () => ({ default: () => <div data-testid="link" /> }));

vehicles[1] = {
  air_speed: 20, ground_speed: 21, alt_rel: 100, heading: 90, pitch: 1, roll: 2, mode: 'AUTO', lat: 0, lon: 0,
  battery: 80, voltage: 12.1, current: 3, status_texts: [], gps_fix: 3, gps_hacc: 1, link_ok: true, link_quality: 90,
};
// A waypoint 0.001 degrees north: about 111 m, 5 s at 21 m/s.
const routeSourceRef = {
  current: { route: () => ({ zone: 0, positions: [{}, {}], track: [{ lat: 0, lon: 0 }, { lat: 0.001, lon: 0 }], next: 1 }) },
};

const NO_CORNER = { width: 0, height: 0 };
const rc = () => rcHudLayout({
  tapeBox: { top: 100, height: 200 }, speedBox: { left: 12, top: 120, height: 180 }, corner: NO_CORNER,
});
const full = (panelShown) => pictureHudLayout({
  width: 1200, height: 700, tagHeight: 24, corner: NO_CORNER, creditsPx: 16, smallMap: { width: 320, height: 180 }, panelShown,
}).hud;
const compact = (over = {}) => pictureHudLayout({
  width: 318, height: 179, tagHeight: 24, corner: NO_CORNER, creditsPx: 16, smallMap: null, panelShown: false, ...over,
}).hud;

const hud = (layout) => render(<TelemetryHud sysId={1} layout={layout} routeSourceRef={routeSourceRef} />);

describe('TelemetryHud', () => {
  it.each([['RC control', rc()], ['maximized Synthetic', full(false)], ['normal-size Synthetic', compact()]])(
    'in %s: display only, no control of any kind',
    (_host, layout) => {
      const { container } = hud(layout);
      expect(container.querySelectorAll('button, input, select, a, [role="slider"], [tabindex]')).toHaveLength(0);
      for (const el of container.children) expect(el.style.pointerEvents).toBe('none');
    },
  );

  it('maximized: tapes, heading tape, attitude scale, and the next waypoint with distance and time', () => {
    const { getByTestId, queryByTestId } = hud(full(true));
    expect(getByTestId('speed')).toBeTruthy();
    expect(getByTestId('altitude')).toBeTruthy();
    expect(getByTestId('heading')).toBeTruthy();
    expect(getByTestId('attitude').dataset.scale).toBe('true');
    expect(getByTestId('hud-waypoint').textContent).toBe('fpvView.hud.nextWaypointEte(n=2,dist=111 m,ete=00:05)');
    expect(queryByTestId('hud-speed-number')).toBeNull();
  });

  it('right panel open: no mode, link, battery or status texts; hidden: all of them', () => {
    const open = hud(full(true));
    for (const id of ['hud-mode', 'link', 'battery', 'strip']) expect(open.queryByTestId(id)).toBeNull();
    open.unmount();
    const hidden = hud(full(false));
    expect(hidden.getByTestId('hud-mode').textContent).toBe('AUTO');
    for (const id of ['link', 'battery', 'strip']) expect(hidden.getByTestId(id)).toBeTruthy();
  });

  it('RC control never repeats the mode (its column shows it); link and battery always (no right panel in RC)', () => {
    const { queryByTestId, getByTestId } = hud(rc());
    expect(queryByTestId('hud-mode')).toBeNull();
    expect(getByTestId('battery')).toBeTruthy();
    expect(getByTestId('link')).toBeTruthy();
    expect(getByTestId('hud-waypoint')).toBeTruthy();
  });

  it('normal size: speed and altitude as plain numbers, the short waypoint readout, no tapes, no link or battery', () => {
    const { getByTestId, queryByTestId } = hud(compact());
    expect(getByTestId('hud-speed-number').textContent).toBe('fpvView.hud.speed(as=20,gs=21)');
    expect(getByTestId('hud-altitude-number').textContent).toBe('fpvView.hud.altitude(alt=100)');
    expect(getByTestId('hud-waypoint').textContent).toBe('fpvView.hud.waypoint(n=2,dist=111 m)');
    for (const id of ['speed', 'altitude', 'battery', 'link', 'hud-mode']) expect(queryByTestId(id)).toBeNull();
    expect(getByTestId('attitude').dataset.scale).toBe('false');
    expect(getByTestId('strip').dataset.max).toBe('1');
  });

  it('no room for the numbers row: they take the top row instead of the waypoint', () => {
    const layout = compact({ height: 82, creditsPx: 38 });
    expect(layout.corner.numbers).toBe(true);
    const { getByTestId, queryByTestId } = hud(layout);
    expect(getByTestId('hud-numbers').textContent).toBe('fpvView.hud.speed(as=20,gs=21) · fpvView.hud.altitude(alt=100)');
    expect(queryByTestId('hud-waypoint')).toBeNull();
    expect(queryByTestId('heading')).toBeNull();
  });

  it('no next waypoint: no readout; too slow for a time: distance only', () => {
    const none = render(<TelemetryHud sysId={1} layout={full(true)} routeSourceRef={{ current: { route: () => null } }} />);
    expect(none.queryByTestId('hud-waypoint')).toBeNull();
    none.unmount();
    vehicles[2] = { ...vehicles[1], ground_speed: 0.5 };
    const slow = render(<TelemetryHud sysId={2} layout={full(true)} routeSourceRef={routeSourceRef} />);
    expect(slow.getByTestId('hud-waypoint').textContent).toBe('fpvView.hud.nextWaypoint(n=2,dist=111 m)');
  });

  it('the attitude indicator in its box (maximized Synthetic: clear of the small map)', () => {
    const layout = { ...full(true), attitude: { left: 344, right: 344, top: 0, bottom: 0, scale: true } };
    const { getByTestId } = hud(layout);
    expect(getByTestId('hud-attitude').style).toMatchObject({ left: '344px', right: '344px', top: '0px', bottom: '0px' });
  });

  it('no attitude box from the host (the gimbal view off the nose axis): no indicator, the rest stays', () => {
    const { queryByTestId, getByTestId } = hud({ ...full(true), attitude: null });
    expect(queryByTestId('hud-attitude')).toBeNull();
    expect(getByTestId('speed')).toBeTruthy();
    expect(getByTestId('hud-waypoint')).toBeTruthy();
  });

  it('nothing for a UAV without telemetry', () => {
    const { container } = render(<TelemetryHud sysId={9} layout={rc()} />);
    expect(container.innerHTML).toBe('');
  });
});
