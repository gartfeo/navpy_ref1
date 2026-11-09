import { describe, expect, it } from 'vitest';
import { rallyFailureLines, rallyUploadFailures } from './rallyOutcome';

describe('rallyUploadFailures', () => {
  it('lists only vehicles whose rally update was not applied', () => {
    const result = {
      results: [
        { sys_id: 1, rally: { applied: true, action: 'set' } },
        { sys_id: 2, rally: { applied: false, action: 'set', error: 'parameter write failed: RALLY_LIMIT_KM', failed_params: ['RALLY_LIMIT_KM'] } },
        { sys_id: 3, rally: null },
        { sys_id: 4, rally: { applied: false, action: 'clear', error: null } },
      ],
    };
    expect(rallyUploadFailures(result)).toEqual([
      { sysId: 2, action: 'set', error: 'parameter write failed: RALLY_LIMIT_KM', failedParams: ['RALLY_LIMIT_KM'] },
      { sysId: 4, action: 'clear', error: null, failedParams: [] },
    ]);
  });

  it('is empty for a missing result', () => {
    expect(rallyUploadFailures(null)).toEqual([]);
    expect(rallyUploadFailures({})).toEqual([]);
  });
});

describe('rallyFailureLines', () => {
  it('says where an RTL would go instead', () => {
    const lines = rallyFailureLines([
      { sysId: 2, action: 'set', error: 'rally point not acknowledged after 3 attempts', failedParams: [] },
      { sysId: 4, action: 'clear', error: null, failedParams: [] },
    ]);
    expect(lines[0]).toContain('Vehicle 2: rally point not acknowledged');
    expect(lines[0]).toContain('launch point');
    expect(lines[1]).toContain('Vehicle 4: clear failed');
    expect(lines[1]).toContain('older hub');
  });
});
