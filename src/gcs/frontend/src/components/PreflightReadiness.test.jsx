// Preflight readiness grid: the companion row's "Connected" wording and
// tooltip, and the detector camera / video stream rows
// (docs/design/gcs_preflight_camera_row.md, todos 70 and 72a).
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen, fireEvent } from '@testing-library/react';
import PreflightReadiness from './PreflightReadiness';

// Interpolating t, so a cell can be checked for the UAVs it names.
vi.mock('react-i18next', () => ({
  useTranslation: () => ({
    t: (key, opts) => (opts ? `${key} ${Object.values(opts).join(' ')}` : key),
    i18n: { language: 'en' },
  }),
}));

const fleet = vi.hoisted(() => ({ vehicles: [] }));
vi.mock('../hooks/useTelemetryStore', () => ({
  useTelemetryStore: () => null,
  useVehicleList: () => fleet.vehicles,
}));

const URL = 'http://192.168.144.10:8889/uav1/whep';
const camera = (extra = {}) => ({
  component: 100, lost: false, vendor: 'navpy', status: 'active', discovery: 'done',
  has_video: true, rejected: [], ...extra,
});
const vehicle = (sysId, extra = {}) => ({
  sys_id: sysId, name: `UAV ${sysId}`, link_ok: true, armed: false,
  prearm_check_state: 'ok', gps_fix: 3, gps_sats: 14, gps_hacc: 0.5, battery: 80, voltage: 16.2,
  companion_ok: true, companion_status: 'ok', mission_uploaded: true,
  ekf: { flags: 831, age_s: 0.1, velocity_variance: 0.1, pos_horiz_variance: 0.1,
    pos_vert_variance: 0.1, compass_variance: 0.1, terrain_alt_variance: 0 },
  sensors: { gyro: true, accel: true, mag: true, abs_pressure: true, rc: true },
  cameras: [camera()], video_streams: [{ key: '100:1', whep_url: URL }],
  camera_wait_over: false, video_stream_expected: false, video_stream_duplicates: [],
  ...extra,
});

function renderGrid(vehicles) {
  fleet.vehicles = vehicles;
  render(<PreflightReadiness settings={{}} onClose={() => {}} onReboot={() => {}} />);
}

// The row label cell and its per-UAV cells, in grid order.
function row(labelKey) {
  const label = screen.getByText(labelKey);
  const cells = [];
  for (let el = label.nextElementSibling; cells.length < fleet.vehicles.length;
    el = el.nextElementSibling) cells.push(el);
  return { label, cells };
}

describe('PreflightReadiness companion row', () => {
  beforeEach(() => { fleet.vehicles = []; });

  it('reads Connected, Checking and Disconnected for go, warn and nogo', () => {
    renderGrid([
      vehicle(1),
      vehicle(2, { companion_status: 'checking' }),
      vehicle(3, { companion_status: 'down', companion_ok: false }),
    ]);
    const { cells } = row('preflight.companion');
    expect(cells.map((c) => c.textContent)).toEqual([
      'preflight.ccConnected', 'preflight.ccChecking', 'preflight.ccDisconnected',
    ]);
  });

  it('explains Connected on the row label and the GO cell only', () => {
    renderGrid([vehicle(1), vehicle(2, { companion_status: 'checking' })]);
    const { label, cells } = row('preflight.companion');
    expect(label).toHaveAttribute('title', 'preflight.ccTooltip');
    expect(cells[0]).toHaveAttribute('title', 'preflight.ccTooltip');
    expect(cells[1]).not.toHaveAttribute('title');
  });
});

describe('PreflightReadiness camera rows', () => {
  beforeEach(() => { fleet.vehicles = []; });

  it('sits right under the companion row', () => {
    renderGrid([vehicle(1)]);
    const companion = row('preflight.companion');
    expect(companion.cells[0].nextElementSibling).toHaveTextContent('preflight.detectorCamera');
    expect(row('preflight.detectorCamera').cells[0].nextElementSibling)
      .toHaveTextContent('preflight.videoStream');
  });

  it('shows each UAV its own camera and stream state', () => {
    renderGrid([
      vehicle(1),
      vehicle(2, { cameras: [camera({ vendor: 'navpy_sim', status: 'critical' }), camera({ component: 101 })] }),
      vehicle(3, { companion_status: 'checking' }),
    ]);
    expect(row('preflight.detectorCamera').cells.map((c) => c.textContent)).toEqual([
      'preflight.cameraState.ready',
      'preflight.cameraState.noFrames preflight.cameraCount 1 2',
      '—',
    ]);
    expect(row('preflight.videoStream').cells.map((c) => c.textContent)).toEqual([
      'preflight.streamCount 1', 'preflight.streamCount 1', '—',
    ]);
  });

  it('names the other UAVs that announce the same stream', () => {
    renderGrid([
      vehicle(1, { video_stream_duplicates: [{ key: '100:1', sys_ids: [2, 9] }] }),
      vehicle(2, { video_stream_duplicates: [{ key: '100:1', sys_ids: [1] }] }),
    ]);
    expect(row('preflight.videoStream').cells.map((c) => c.textContent)).toEqual([
      'preflight.streamState.duplicate UAV 2, #9',
      'preflight.streamState.duplicate UAV 1',
    ]);
  });

  it('shows the rejected reason and hints the fix on hover', () => {
    renderGrid([vehicle(1, {
      cameras: [camera({ rejected: [{ stream_id: 1, reason: 'port' }] })], video_streams: [],
    })]);
    const [cell] = row('preflight.videoStream').cells;
    expect(cell).toHaveTextContent('preflight.streamState.uriRejected port');
    // The action strip names the hint once; hovering the cell adds its tooltip.
    expect(screen.getAllByText('preflight.fix.streamUriRejected')).toHaveLength(1);
    fireEvent.mouseEnter(cell);
    expect(screen.getAllByText('preflight.fix.streamUriRejected')).toHaveLength(2);
  });

  it('a camera caution reads CAUTION, not NO-GO', () => {
    renderGrid([vehicle(1, { cameras: [camera({ status: 'critical' })] })]);
    expect(screen.getByText('preflight.caution')).toBeInTheDocument();
    expect(screen.queryByText('preflight.noGo')).not.toBeInTheDocument();
  });
});
