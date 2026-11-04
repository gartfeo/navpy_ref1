import { describe, it, expect, vi } from 'vitest';
import { render, fireEvent, waitFor } from '@testing-library/react';
import ConnectionPanel from './ConnectionPanel';

// The panel reads the live fleet and the heard sys_ids from the telemetry store.
const fakeStore = { getVehicleList: () => [], getSeenIds: () => [3] };
vi.mock('../hooks/useTelemetryStore', () => ({
  useTelemetryStore: (selector) => selector(fakeStore),
}));

// Owner request: the panel shows which address each vehicle is connected on.

function renderPanel(props = {}) {
  return render(
    <ConnectionPanel
      vehicleList={[]}
      seenIds={null}
      onScan={vi.fn()}
      onConnect={vi.fn()}
      onDisconnect={vi.fn()}
      onDownloadPlan={vi.fn()}
      downloadingSysIds={new Set()}
      defaultDevice="udp:0.0.0.0:14550"
      {...props}
    />,
  );
}

describe('ConnectionPanel connection address', () => {
  it('shows the connection string of each connected vehicle', () => {
    const view = renderPanel({
      vehicleList: [
        { sys_id: 1, name: 'u1', device: 'udp:0.0.0.0:14551', link_ok: true },
        { sys_id: 2, name: 'u2', device: 'tcp:127.0.0.1:5762', link_ok: true },
      ],
    });
    const shown = view.getAllByTestId('connection-device').map((el) => el.textContent);
    expect(shown).toEqual(['udp:0.0.0.0:14551', 'tcp:127.0.0.1:5762']);
    expect(view.getAllByTestId('connection-device')[0].getAttribute('title')).toBe('udp:0.0.0.0:14551');
  });

  it('shows no address when the backend gives none', () => {
    const view = renderPanel({ vehicleList: [{ sys_id: 1, name: 'u1', link_ok: true }] });
    expect(view.getByText('u1')).toBeInTheDocument();
    expect(view.queryByTestId('connection-device')).toBeNull();
  });

  it('shows the scanned address while a vehicle is connecting', async () => {
    const onScan = vi.fn().mockResolvedValue({ found: 1, vehicles: [{ sys_id: 3, name: 'u3' }] });
    const onConnect = vi.fn(() => new Promise(() => {}));
    const view = renderPanel({ onScan, onConnect, defaultDevice: 'COM7' });
    fireEvent.click(view.getByRole('button', { name: 'connection.connect' }));
    const connectRow = await view.findByText('u3');
    fireEvent.click(connectRow.parentElement.parentElement.querySelector('button'));
    await waitFor(() => expect(view.getByTestId('connection-device').textContent).toBe('COM7'));
  });
});
