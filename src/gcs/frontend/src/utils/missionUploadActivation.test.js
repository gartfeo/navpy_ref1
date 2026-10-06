import { describe, expect, it, vi } from 'vitest';

import { activateUploadedSimPlan } from './missionUploadActivation';


const TARG_CALLS = [
  { sys_id: 1, targ_wps: 5 },
  { sys_id: 2, targ_wps: 6 },
  { sys_id: 3, targ_wps: 7 },
];
const NAV_CALLS = [
  { sys_id: 1, nav_last_wp: 2 },
  { sys_id: 2, nav_last_wp: 2 },
  { sys_id: 3, nav_last_wp: 2 },
];


function readyResult() {
  return {
    status: 'ready',
    instances: TARG_CALLS.map((target, index) => ({
      sys_id: target.sys_id,
      status: 'ready',
      old_pid: 100 + index,
      pid: 200 + index,
      runtime: {
        mission_items: 10,
        targ_wps: target.targ_wps,
        nav_last_wp: NAV_CALLS[index].nav_last_wp,
      },
    })),
  };
}


describe('activateUploadedSimPlan', () => {
  it('awaits every confirmed parameter write before one exact restart', async () => {
    const resolvers = [];
    const writeAasParams = vi.fn((sysId, params) => new Promise((resolve) => {
      resolvers.push(() => resolve({
        sys_id: sysId,
        results: { targ_wps: true, nav_last_wp: true },
        params,
      }));
    }));
    const restartReady = vi.fn().mockResolvedValue(readyResult());

    const pending = activateUploadedSimPlan({
      targWpsCalls: TARG_CALLS,
      navLastWpCalls: NAV_CALLS,
      writeAasParams,
      restartReady,
    });

    expect(writeAasParams).toHaveBeenCalledTimes(3);
    expect(writeAasParams).toHaveBeenNthCalledWith(
      1, 1, { targ_wps: 5, nav_last_wp: 2 },
    );
    expect(restartReady).not.toHaveBeenCalled();
    resolvers[0]();
    resolvers[1]();
    await Promise.resolve();
    expect(restartReady).not.toHaveBeenCalled();
    resolvers[2]();

    const result = await pending;

    expect(restartReady).toHaveBeenCalledTimes(1);
    expect(restartReady).toHaveBeenCalledWith([
      { sys_id: 1, expected_params: { targ_wps: 5, nav_last_wp: 2 } },
      { sys_id: 2, expected_params: { targ_wps: 6, nav_last_wp: 2 } },
      { sys_id: 3, expected_params: { targ_wps: 7, nav_last_wp: 2 } },
    ]);
    expect(result.targWpsUpdates).toEqual({ 1: 5, 2: 6, 3: 7 });
    expect(result.navLastWpUpdates).toEqual({ 1: 2, 2: 2, 3: 2 });
  });

  it.each([
    ['a rejected write', (_sysId, _params, index) => (
      index === 1 ? Promise.reject(new Error('link down')) : Promise.resolve({
        sys_id: TARG_CALLS[index].sys_id,
        results: { targ_wps: true, nav_last_wp: true },
      })
    )],
    ['a false parameter echo', (sysId) => Promise.resolve({
      sys_id: sysId,
      results: { targ_wps: true, nav_last_wp: sysId !== 2 },
    })],
    ['a missing parameter echo', (sysId) => Promise.resolve({
      sys_id: sysId,
      results: { targ_wps: true },
    })],
  ])('rejects %s only after all writes settle', async (_label, responseFor) => {
    let completed = 0;
    const writeAasParams = vi.fn((sysId, params) => {
      const index = TARG_CALLS.findIndex((call) => call.sys_id === sysId);
      return Promise.resolve(responseFor(sysId, params, index)).then(
        (value) => { completed += 1; return value; },
        (error) => { completed += 1; throw error; },
      );
    });
    const restartReady = vi.fn();

    await expect(activateUploadedSimPlan({
      targWpsCalls: TARG_CALLS,
      navLastWpCalls: NAV_CALLS,
      writeAasParams,
      restartReady,
    })).rejects.toThrow(/parameter/i);

    expect(completed).toBe(3);
    expect(restartReady).not.toHaveBeenCalled();
  });

  it.each([
    ['missing vehicle', () => {
      const result = readyResult();
      result.instances.pop();
      return result;
    }],
    ['extra vehicle', () => {
      const result = readyResult();
      result.instances.push({ sys_id: 4, status: 'ready', runtime: {} });
      return result;
    }],
    ['non-ready vehicle', () => {
      const result = readyResult();
      result.instances[1].status = 'starting';
      return result;
    }],
    ['stale runtime params', () => {
      const result = readyResult();
      result.instances[2].runtime.targ_wps = 999;
      return result;
    }],
    ['unchanged pid', () => {
      const result = readyResult();
      result.instances[0].pid = result.instances[0].old_pid;
      return result;
    }],
  ])('rejects restart readiness with %s', async (_label, buildResult) => {
    const writeAasParams = vi.fn((sysId) => Promise.resolve({
      sys_id: sysId,
      results: { targ_wps: true, nav_last_wp: true },
    }));

    await expect(activateUploadedSimPlan({
      targWpsCalls: TARG_CALLS,
      navLastWpCalls: NAV_CALLS,
      writeAasParams,
      restartReady: vi.fn().mockResolvedValue(buildResult()),
    })).rejects.toThrow(/ready|runtime|pid/i);
  });

  it('rejects mismatched or duplicate parameter sysids before writing', async () => {
    const writeAasParams = vi.fn();

    await expect(activateUploadedSimPlan({
      targWpsCalls: TARG_CALLS,
      navLastWpCalls: NAV_CALLS.slice(0, 2),
      writeAasParams,
      restartReady: vi.fn(),
    })).rejects.toThrow(/sys_id/i);

    expect(writeAasParams).not.toHaveBeenCalled();
  });
});
