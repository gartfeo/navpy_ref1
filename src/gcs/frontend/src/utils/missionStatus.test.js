import { describe, expect, it } from 'vitest';
import { computeMissionStatus } from './missionStatus';

const failedWith = (failReason) => computeMissionStatus(
  {
    mode: 'GUIDED', armed: true, mission_progress: null,
    swarm: { phase: 'HOLD', outcome: 'failed', fail_reason: failReason, boot: 1, stale: false },
  },
  null,
  0,
);

describe('computeMissionStatus fail reasons', () => {
  it('labels a NavPy internal error abort with its own reason', () => {
    expect(failedWith('internal_error').outcome.reasonKey)
      .toBe('missionStatus.reasons.internal_error');
  });

  it('still labels a stale-command abort', () => {
    expect(failedWith('source_command_stale').outcome.reasonKey)
      .toBe('missionStatus.reasons.source_command_stale');
  });

  it('maps an unlisted reason to unknown', () => {
    expect(failedWith('not_a_reason').outcome.reasonKey)
      .toBe('missionStatus.reasons.unknown');
  });
});
