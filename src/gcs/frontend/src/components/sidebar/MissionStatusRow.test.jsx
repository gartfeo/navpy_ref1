import { render, screen } from '@testing-library/react';
import { describe, expect, it } from 'vitest';
import MissionStatusRow from './MissionStatusRow';
import { computeMissionStatus } from '../../utils/missionStatus';

// NavPy's phase on the card (docs/design/gcs_mission_phase.md).
const armed = { mode: 'GUIDED', armed: true, lat: 40, lon: 44, mission_progress: 5 };
const slot = {
  taskId: 3, taskType: 'dock', lat: 40.01, lon: 44.01, status: 'assigned',
  doingUid: { boot_id: 7, msg_seq: 4 }, assignedUid: { boot_id: 7, msg_seq: 4 },
};

function renderRow(vehicle, assignment = null) {
  const missionStatus = computeMissionStatus(vehicle, assignment, 0, false);
  return render(
    <MissionStatusRow
      vehicle={vehicle}
      missionStatus={missionStatus}
      assignment={missionStatus.assignment}
    />,
  );
}

describe('MissionStatusRow with a NavPy phase', () => {
  it('shows DOCK FAILED (SIM) with its reason row', () => {
    const { container } = renderRow({
      ...armed, armed: false,
      swarm: { phase: 'HOLD', outcome: 'failed', fail_reason: 'no_detection', boot: 7, stale: false },
    });
    expect(container).toHaveTextContent('missionStatus.dockFailed');
    expect(container).toHaveTextContent('missionStatus.reason');
  });

  it('shows DOCKED (SIM CONTACT) with no reason row', () => {
    const { container } = renderRow({
      ...armed, armed: false,
      swarm: { phase: 'HOLD', outcome: 'docked', fail_reason: null, boot: 7, stale: false },
    });
    expect(container).toHaveTextContent('missionStatus.docked');
    expect(container).not.toHaveTextContent('missionStatus.reason');
  });

  it('shows a held outcome as a Last line while searching', () => {
    const { container } = renderRow({
      ...armed, mode: 'AUTO',
      swarm: { phase: 'SEARCH', outcome: 'docked', fail_reason: null, boot: 7, stale: false },
    });
    expect(container).toHaveTextContent('missionStatus.searching');
    expect(container).toHaveTextContent('missionStatus.last');
  });

  it('marks a stale end state as last heard', () => {
    const { container } = renderRow({
      ...armed, armed: false, mode: 'MANUAL',
      swarm: { phase: 'HOLD', outcome: 'docked', fail_reason: null, boot: 7, stale: true },
    });
    expect(container).toHaveTextContent('missionStatus.docked');
    expect(container).toHaveTextContent('missionStatus.lastHeard');
  });

  it('labels TO_TASK from a current-boot ASSIGNED slot', () => {
    renderRow({ ...armed, swarm: { phase: 'TO_TASK', boot: 7, stale: false } }, slot);
    expect(screen.getByText('missionStatus.approaching')).toBeInTheDocument();
    expect(screen.getByText('#3 dock')).toBeInTheDocument();
  });

  it('shows no label or coordinates from an earlier boot', () => {
    const { container } = renderRow(
      { ...armed, swarm: { phase: 'TO_TASK', boot: 8, stale: false } }, slot,
    );
    expect(container).toHaveTextContent('missionStatus.approaching');
    expect(container).not.toHaveTextContent('#3 dock');
    expect(container).not.toHaveTextContent('40.01000');
  });
});
