/**
 * Regression coverage for progressive mission-download reconciliation.
 *
 * Run via: node tests/gcs/frontend/test_mission_reconciliation_logic.js
 */
const assert = require('assert');
const path = require('path');
const { readFileSync } = require('fs');

function loadEsm(relPath) {
  const abs = path.resolve(__dirname, '..', '..', '..', 'src', 'gcs', 'frontend', 'src', relPath);
  let src = readFileSync(abs, 'utf8');
  src = src.replace(/export\s+(async\s+)?(const|function|class)\s+/g, '$1$2 ');
  src = src.replace(/export\s*\{[^}]*\}/g, '');
  src = src.replace(/export\s+default\s+/g, '');
  const names = [];
  src.replace(/^(?:async\s+)?(?:const|function|class)\s+(\w+)/gm, (_, n) => { names.push(n); return _; });
  src += '\nmodule.exports = { ' + names.join(', ') + ' };\n';
  const m = { exports: {} };
  new Function('module', 'exports', 'require', src)(m, m.exports, require);
  return m.exports;
}

function deferred() {
  let resolve;
  const promise = new Promise((r) => { resolve = r; });
  return { promise, resolve };
}

async function main() {
  const {
    backendMissionDownloadInFlight,
    reconcileMissionDownloads,
    waitForBackendMissionSettlement,
  } = loadEsm('utils/missionReconciliation.js');
  assert.strictEqual(backendMissionDownloadInFlight({ is_probing: true }), true);
  assert.strictEqual(backendMissionDownloadInFlight({ mission_download_progress: [4, 9] }), true);
  assert.strictEqual(backendMissionDownloadInFlight({ is_probing: false, mission_download_progress: null }), false);
  const initial = new Map([[161, deferred()], [162, deferred()], [163, deferred()]]);
  const late163 = deferred();
  const telemetryListeners = new Set();
  let backendVehicle163 = { sys_id: 163, is_probing: true, mission_download_progress: [7, 9] };
  const telemetryStore = {
    getVehicles: () => ({ 163: backendVehicle163 }),
    subscribe: (listener) => {
      telemetryListeners.add(listener);
      return () => telemetryListeners.delete(listener);
    },
  };
  const attempts = new Map();
  const snapshots = [];
  const awaitingLate = [];
  const settled = [];

  const run = reconcileMissionDownloads({
    vehicles: [{ sys_id: 161 }, { sys_id: 162 }, { sys_id: 163 }],
    downloadMission: (sysId) => {
      const attempt = (attempts.get(sysId) || 0) + 1;
      attempts.set(sysId, attempt);
      return attempt === 1 ? initial.get(sysId).promise : late163.promise;
    },
    waitForBackendSettlement: (sysId) => {
      assert.strictEqual(sysId, 163);
      return waitForBackendMissionSettlement(telemetryStore, sysId);
    },
    onResultsChanged: (results) => snapshots.push(results.filter((r) => !r.error).map((r) => r.sys_id)),
    onAwaitingLate: (sysId) => awaitingLate.push(sysId),
    onVehicleSettled: (sysId, result) => settled.push([sysId, result.error || 'success']),
  });

  // Healthy paths arrive before the client deadline and must be published
  // immediately, in fleet order even if completion order differs.
  initial.get(162).resolve({ waypoints: [{ lat: 2, lon: 2 }] });
  await Promise.resolve();
  initial.get(161).resolve({ waypoints: [{ lat: 1, lon: 1 }] });
  await new Promise((resolve) => setTimeout(resolve, 0));
  assert.deepStrictEqual(snapshots, [[162], [161, 162]]);

  // UAV 163 crosses the 45 s client deadline. It remains pending while the
  // already-running backend probe completes; its healthy siblings stay drawn.
  initial.get(163).resolve({ error: 'timeout' });
  await new Promise((resolve) => setTimeout(resolve, 0));
  assert.deepStrictEqual(awaitingLate, [163]);
  assert.deepStrictEqual(settled.map(([sysId]) => sysId).sort(), [161, 162]);
  assert.deepStrictEqual(snapshots.at(-1), [161, 162]);

  // Once the backend signals settlement, the retry reads its cached success and
  // automatically reconciles the third path without losing the first two.
  assert.strictEqual(telemetryListeners.size, 1);
  backendVehicle163 = { sys_id: 163, is_probing: false, mission_download_progress: null };
  for (const listener of telemetryListeners) listener();
  await Promise.resolve();
  assert.strictEqual(telemetryListeners.size, 0);
  late163.resolve({ waypoints: [{ lat: 3, lon: 3 }] });
  await run;
  assert.deepStrictEqual(snapshots.at(-1), [161, 162, 163]);
  assert.deepStrictEqual(settled, [
    [162, 'success'],
    [161, 'success'],
    [163, 'success'],
  ]);
  assert.strictEqual(attempts.get(163), 2);

  // A retry that also times out settles as a visible failure instead of
  // leaving the vehicle pinned in pending state forever.
  const finalFailures = [];
  await reconcileMissionDownloads({
    vehicles: [{ sys_id: 163 }],
    downloadMission: async () => ({ error: 'timeout' }),
    waitForBackendSettlement: async () => {},
    onVehicleSettled: (sysId, result) => finalFailures.push([sysId, result.error]),
  });
  assert.deepStrictEqual(finalFailures, [[163, 'timeout']]);

  console.log('Mission reconciliation regression passed');
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
