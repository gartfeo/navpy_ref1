function indexedCalls(calls, valueKey, label) {
  if (!Array.isArray(calls) || calls.length === 0) {
    throw new Error(`${label} calls must contain at least one vehicle`);
  }
  const bySysId = new Map();
  for (const call of calls) {
    const sysId = Number(call?.sys_id);
    const value = Number(call?.[valueKey]);
    if (!Number.isInteger(sysId) || sysId <= 0 || !Number.isInteger(value)) {
      throw new Error(`${label} contains an invalid sys_id or value`);
    }
    if (bySysId.has(sysId)) {
      throw new Error(`${label} contains duplicate sys_id ${sysId}`);
    }
    bySysId.set(sysId, value);
  }
  return bySysId;
}


function configuredInstances(targWpsCalls, navLastWpCalls) {
  const pois = indexedCalls(targWpsCalls, 'targ_wps', 'targ_wps');
  const nav = indexedCalls(navLastWpCalls, 'nav_last_wp', 'nav_last_wp');
  const poiIds = [...pois.keys()].sort((a, b) => a - b);
  const navIds = [...nav.keys()].sort((a, b) => a - b);
  if (poiIds.length !== navIds.length
      || poiIds.some((sysId, index) => sysId !== navIds[index])) {
    throw new Error('targ_wps and nav_last_wp sys_id sets do not match');
  }
  return poiIds.map((sysId) => ({
    sys_id: sysId,
    params: {
      targ_wps: pois.get(sysId),
      nav_last_wp: nav.get(sysId),
    },
  }));
}


function parameterWriteError(instance, settled) {
  if (settled.status === 'rejected') {
    return `vehicle ${instance.sys_id}: ${settled.reason?.message || settled.reason}`;
  }
  const response = settled.value;
  if (Number(response?.sys_id) !== instance.sys_id) {
    return `vehicle ${instance.sys_id}: response sys_id did not match`;
  }
  const results = response?.results;
  const keys = results && typeof results === 'object'
    ? Object.keys(results).sort()
    : [];
  if (keys.length !== 2 || keys[0] !== 'nav_last_wp' || keys[1] !== 'targ_wps') {
    return `vehicle ${instance.sys_id}: parameter result keys were incomplete`;
  }
  if (results.targ_wps !== true || results.nav_last_wp !== true) {
    return `vehicle ${instance.sys_id}: parameter echo was not confirmed`;
  }
  return null;
}


function validateReadyResponse(response, configured) {
  if (response?.status !== 'ready' || !Array.isArray(response.instances)) {
    throw new Error('NavPy restart-ready response was not ready');
  }
  const expected = new Map(configured.map((item) => [item.sys_id, item.params]));
  const actual = new Map();
  for (const instance of response.instances) {
    const sysId = Number(instance?.sys_id);
    if (!Number.isInteger(sysId) || actual.has(sysId)) {
      throw new Error('NavPy restart-ready response had invalid vehicle coverage');
    }
    actual.set(sysId, instance);
  }
  if (actual.size !== expected.size
      || [...expected.keys()].some((sysId) => !actual.has(sysId))) {
    throw new Error('NavPy restart-ready response did not cover the exact sys_id set');
  }
  for (const [sysId, params] of expected) {
    const instance = actual.get(sysId);
    if (instance.status !== 'ready') {
      throw new Error(`NavPy restart-ready vehicle ${sysId} was not ready`);
    }
    if (!Number.isInteger(instance.old_pid)
        || !Number.isInteger(instance.pid)
        || instance.old_pid === instance.pid) {
      throw new Error(`NavPy restart-ready vehicle ${sysId} did not get a new pid`);
    }
    const runtime = instance.runtime;
    if (!Number.isInteger(runtime?.mission_items) || runtime.mission_items <= 0) {
      throw new Error(`NavPy restart-ready vehicle ${sysId} loaded no mission`);
    }
    if (Number(runtime.targ_wps) !== params.targ_wps
        || Number(runtime.nav_last_wp) !== params.nav_last_wp) {
      throw new Error(`NavPy runtime parameters did not match for vehicle ${sysId}`);
    }
  }
}


/**
 * Make an uploaded simulator plan active in all companions.
 *
 * Every ArduPilot parameter write must settle and echo success before any
 * companion is restarted. The restart call is one server-side operation so
 * the browser cannot accidentally stop unrelated processes or race START.
 */
export async function activateUploadedSimPlan({
  targWpsCalls,
  navLastWpCalls,
  writeAasParams,
  restartReady,
}) {
  const configured = configuredInstances(targWpsCalls, navLastWpCalls);
  const writes = await Promise.allSettled(configured.map((instance) => (
    writeAasParams(instance.sys_id, instance.params)
  )));
  const errors = writes
    .map((settled, index) => parameterWriteError(configured[index], settled))
    .filter(Boolean);
  if (errors.length > 0) {
    throw new Error(`Mission parameter activation failed: ${errors.join('; ')}`);
  }

  const request = configured.map((instance) => ({
    sys_id: instance.sys_id,
    expected_params: { ...instance.params },
  }));
  let response;
  try {
    response = await restartReady(request);
  } catch (error) {
    throw new Error(`NavPy restart-ready failed: ${error?.message || error}`);
  }
  validateReadyResponse(response, configured);
  return {
    targWpsUpdates: Object.fromEntries(
      configured.map((instance) => [instance.sys_id, instance.params.targ_wps]),
    ),
    navLastWpUpdates: Object.fromEntries(
      configured.map((instance) => [instance.sys_id, instance.params.nav_last_wp]),
    ),
  };
}


export { configuredInstances, validateReadyResponse };
