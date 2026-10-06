/**
 * Node.js truth-table tests for computePlanEntry — the monitor map's plan
 * entry ("Edit Plan" / "Start Planning") visibility + label logic.
 *
 * Imports the REAL source (constants/monitorPhases.js) via dynamic import so the
 * test cannot drift from the implementation.
 *
 * Run via: node tests/gcs/frontend/test_plan_entry_logic.js
 */

const assert = require('assert');
const path = require('path');
const { pathToFileURL } = require('url');

const SRC = path.join(
  __dirname, '..', '..', '..',
  'src', 'gcs', 'frontend', 'src', 'constants', 'monitorPhases.js',
);

(async () => {
  const { computePlanEntry } = await import(pathToFileURL(SRC).href);

  // A plan exists → always shown, always the "Edit Plan" label, regardless of
  // arm / mission / connecting state. This is the primary case (and the one that
  // used to collide with E-STOP in the sidebar header).
  for (const allDisarmed of [true, false]) {
    for (const vehiclesHaveMission of [true, false]) {
      const r = computePlanEntry({ hasPlan: true, vehiclesHaveMission, isBusyConnecting: false, allDisarmed });
      assert.strictEqual(r.show, true, 'hasPlan → show');
      assert.strictEqual(r.isEdit, true, 'hasPlan → Edit Plan label');
    }
  }

  // Idle pre-launch, no plan / mission / connecting → hidden: the sidebar already
  // renders the primary full-width "Start Planning" button, so showing it on the
  // map too would be a duplicate CTA.
  {
    const r = computePlanEntry({ hasPlan: false, vehiclesHaveMission: false, isBusyConnecting: false, allDisarmed: true });
    assert.strictEqual(r.show, false, 'idle pre-launch, no plan → hidden (avoid duplicate)');
    assert.strictEqual(r.isEdit, false, 'no plan → Start Planning label');
  }

  // In-flight (some armed → not allDisarmed), no GCS plan → keep planning
  // reachable with the "Start Planning" label (ubiquity the old header link had).
  {
    const r = computePlanEntry({ hasPlan: false, vehiclesHaveMission: true, isBusyConnecting: false, allDisarmed: false });
    assert.strictEqual(r.show, true, 'in-flight, no plan → show Start Planning');
    assert.strictEqual(r.isEdit, false, 'no plan → Start Planning label');
  }

  // Vehicles carry a mission but no GCS plan yet, idle (download-plan state) →
  // shown. This is ALSO the reachable container-launch shape: a real container
  // launch fires vehicles that carry a mission, so `vehiclesHaveMission` is true
  // and the entry stays shown even though App's `allDisarmed` proxy can't see the
  // container-launch flag. Proves the proxy never drops the entry mid-launch.
  {
    const r = computePlanEntry({ hasPlan: false, vehiclesHaveMission: true, isBusyConnecting: false, allDisarmed: true });
    assert.strictEqual(r.show, true, 'mission present, no GCS plan → show (download / container-launch)');
    assert.strictEqual(r.isEdit, false, 'no plan → Start Planning label');
  }

  // Connecting / downloading a plan, no plan yet → shown (planning stays reachable).
  {
    const r = computePlanEntry({ hasPlan: false, vehiclesHaveMission: false, isBusyConnecting: true, allDisarmed: true });
    assert.strictEqual(r.show, true, 'busy connecting, no plan → show');
    assert.strictEqual(r.isEdit, false, 'no plan → Start Planning label');
  }

  console.log('PASS');
})().catch((e) => { console.error(e); process.exit(1); });
