import { useCallback, useRef } from 'react';
import fetchWithRetry from '../utils/fetchRetry';
import { diagnosticRequest, emitDiagnostic } from '../utils/diagnostics';

// Hard caps so a wedged backend / dropped link can't leave a request — and the
// UI "connecting" / "downloading" indicator that keys off it — pending forever.
// The mission download can legitimately take tens of seconds (blocking MAVLink
// download, possibly queued behind the connect-time probe on the backend's
// per-vehicle mission lock), so its cap is generous.
const CONNECT_TIMEOUT_MS = 20000;
const MISSION_DOWNLOAD_TIMEOUT_MS = 45000;

// A caller-owned AbortController that fires after `ms`. Gives requests a TOTAL
// deadline (across fetchWithRetry's retries, which pass the signal through to
// fetch and bail on AbortError instead of retrying into the same wall).
function abortAfter(ms) {
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), ms);
  return { signal: ctrl.signal, clear: () => clearTimeout(timer) };
}

/**
 * Hook for calling backend planning API endpoints.
 * Debounces analyze calls to avoid hammering the server during polygon edits.
 */
export default function usePlanningApi({ setAnalysis, setPlan }) {
  const debounceRef = useRef(null);

  const analyze = useCallback(
    (polygon, dockClasses) => {
      // Debounce: wait 300ms after last change
      if (debounceRef.current) clearTimeout(debounceRef.current);

      if (!polygon || polygon.length < 3) {
        setAnalysis(null);
        return;
      }

      debounceRef.current = setTimeout(async () => {
        try {
          const res = await fetch('/api/plan/analyze', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
              polygon: polygon.map((p) => ({ lat: p.lat, lon: p.lon })),
              dock_classes: dockClasses,
            }),
          });
          if (res.ok) {
            setAnalysis(await res.json());
          }
        } catch (e) {
          console.error('Analyze error:', e);
        }
      }, 300);
    },
    [setAnalysis]
  );

  const generate = useCallback(
    async (polygon, dockClasses, searchPattern, uavCount, launchPoint, corridorWaypoints) => {
      // Corridor search pattern doesn't need a polygon — use corridor waypoints as dummy polygon if needed
      if (searchPattern === 'corridor') {
        if (!corridorWaypoints || corridorWaypoints.length < 2) return null;
        if (!polygon || polygon.length < 3) polygon = corridorWaypoints;
      } else {
        if (!polygon || polygon.length < 3) return null;
      }
      try {
        const body = {
          polygon: polygon.map((p) => ({ lat: p.lat, lon: p.lon })),
          dock_classes: dockClasses,
          search_pattern: searchPattern,
          uav_count: uavCount,
        };
        if (launchPoint) {
          body.launch_point = { lat: launchPoint.lat, lon: launchPoint.lon };
        }
        if (corridorWaypoints?.length > 0) {
          body.corridor_waypoints = corridorWaypoints.map((p) => ({ lat: p.lat, lon: p.lon }));
        }
        const res = await fetch('/api/plan/generate', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(body),
        });
        if (res.ok) {
          const data = await res.json();
          setPlan(data);
          return data;
        }
      } catch (e) {
        console.error('Generate error:', e);
      }
      return null;
    },
    [setPlan]
  );

  const uploadMissions = useCallback(async (assignments, fence) => {
    try {
      const res = await fetchWithRetry('/api/vehicles/upload', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ assignments, ...(fence ? { fence } : {}) }),
      }, { maxRetries: 2, baseDelay: 1000 });
      if (res.ok) {
        return await res.json();
      }
      const err = await res.json().catch(() => null);
      return { status: 'error', results: [], error: err?.detail || `HTTP ${res.status}` };
    } catch (e) {
      console.error('Upload error:', e);
      return { status: 'error', results: [], error: e.message };
    }
  }, []);

  const writeAasParams = useCallback(async (sysId, params) => {
    const res = await fetch(`/api/vehicles/${sysId}/params`, {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(params),
    });
    const body = await res.json().catch(() => null);
    if (!res.ok) {
      throw new Error(body?.detail || `Vehicle ${sysId} parameter write failed (HTTP ${res.status})`);
    }
    return body;
  }, []);

  const restartCompanionsReady = useCallback(async (instances) => {
    const res = await fetch('/api/control/navpy-sim/restart-ready', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ instances }),
    });
    const body = await res.json().catch(() => null);
    if (!res.ok) {
      throw new Error(body?.detail || `NavPy restart failed (HTTP ${res.status})`);
    }
    return body;
  }, []);

  const sendCommand = useCallback(async (command, sysIds, params) => {
    try {
      const res = await fetch('/api/control/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command, sys_ids: sysIds, params }),
      });
      if (res.ok) return await res.json();
    } catch (e) {
      console.error('Command error:', e);
    }
    return null;
  }, []);

  const launchSequence = useCallback(async (sysIds, opts) => {
    try {
      const res = await fetch('/api/control/launch', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          sys_ids: sysIds,
          ...(opts?.force ? { force: true } : {}),
          ...(opts?.pitotCovered ? { pitot_covered: true } : {}),
        }),
      });
      if (res.ok) return await res.json();
      const err = await res.json().catch(() => null);
      console.error('Launch failed:', res.status, err?.detail || err);
    } catch (e) {
      console.error('Launch error:', e);
    }
    return null;
  }, []);

  const triggerVehicle = useCallback(async (sysId) => {
    try {
      const res = await fetch('/api/control/launch/trigger', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ sys_id: sysId }),
      });
      if (res.ok) return await res.json();
    } catch (e) {
      console.error('Trigger error:', e);
    }
    return null;
  }, []);

  const connectVehicle = useCallback(async (device, sysId, name) => {
    const t = abortAfter(CONNECT_TIMEOUT_MS);
    const diagnostic = diagnosticRequest();
    emitDiagnostic('client_request_started', { request_id: diagnostic.requestId, sys_id: sysId, phase: 'connect' });
    try {
      const params = new URLSearchParams({ device, sys_id: sysId });
      if (name) params.set('name', name);
      const res = await fetch(`/api/vehicles/connect?${params}`, { method: 'POST', signal: t.signal, headers: diagnostic.headers });
      if (res.ok) {
        emitDiagnostic('vehicle_operation_finished', { request_id: diagnostic.requestId, sys_id: sysId, phase: 'connect', outcome: 'success' });
        return await res.json();
      }
      const err = await res.json().catch(() => null);
      return { error: err?.detail || `HTTP ${res.status}` };
    } catch (e) {
      if (e.name === 'AbortError') {
        emitDiagnostic('client_timeout', { request_id: diagnostic.requestId, sys_id: sysId, phase: 'connect', outcome: 'timeout' });
        return { error: 'timeout' };
      }
      console.error('Connect error:', e);
      return { error: e.message };
    } finally {
      t.clear();
    }
  }, []);

  const scanVehicles = useCallback(async (device) => {
    try {
      const params = new URLSearchParams({ device, timeout: '5' });
      const res = await fetch(`/api/vehicles/scan?${params}`, { method: 'POST' });
      if (res.ok) return await res.json();
      const err = await res.json().catch(() => null);
      return { error: err?.detail || `HTTP ${res.status}` };
    } catch (e) {
      console.error('Scan error:', e);
      return { error: e.message };
    }
  }, []);

  const discoverVehicles = useCallback(async (device) => {
    const diagnostic = diagnosticRequest();
    try {
      const params = new URLSearchParams({ device, timeout: '5' });
      const res = await fetch(`/api/vehicles/discover?${params}`, { method: 'POST', headers: diagnostic.headers });
      if (res.ok) return await res.json();
      const err = await res.json().catch(() => null);
      return { error: err?.detail || `HTTP ${res.status}` };
    } catch (e) {
      console.error('Discover error:', e);
      return { error: e.message };
    }
  }, []);

  const disconnectVehicle = useCallback(async (sysId) => {
    const diagnostic = diagnosticRequest();
    try {
      const res = await fetch(`/api/vehicles/disconnect?sys_id=${sysId}`, { method: 'POST', headers: diagnostic.headers });
      if (res.ok) return await res.json();
    } catch (e) {
      console.error('Disconnect error:', e);
    }
    return null;
  }, []);

  const downloadMission = useCallback(async (sysId) => {
    const t = abortAfter(MISSION_DOWNLOAD_TIMEOUT_MS);
    const diagnostic = diagnosticRequest();
    emitDiagnostic('client_request_started', { request_id: diagnostic.requestId, sys_id: sysId, phase: 'mission' });
    try {
      const res = await fetchWithRetry(`/api/vehicles/${sysId}/mission`, { signal: t.signal, headers: diagnostic.headers });
      if (res.ok) {
        const data = await res.json();
        emitDiagnostic('client_request_finished', { request_id: diagnostic.requestId, sys_id: sysId, phase: 'mission', outcome: 'success', total: data?.waypoints?.length || 0 });
        return { ...data, _diagnostic_request_id: diagnostic.requestId };
      }
      const err = await res.json().catch(() => null);
      emitDiagnostic('client_request_finished', { request_id: diagnostic.requestId, sys_id: sysId, phase: 'mission', outcome: 'failure', status_code: res.status });
      return { error: err?.detail || `HTTP ${res.status}`, _diagnostic_request_id: diagnostic.requestId };
    } catch (e) {
      if (e.name === 'AbortError') {
        emitDiagnostic('client_timeout', { request_id: diagnostic.requestId, sys_id: sysId, phase: 'mission', outcome: 'timeout' });
        return { error: 'timeout', _diagnostic_request_id: diagnostic.requestId };
      }
      console.error('Download mission error:', e);
      emitDiagnostic('client_request_finished', { request_id: diagnostic.requestId, sys_id: sysId, phase: 'mission', outcome: 'failure', error_code: e.name || 'Error' });
      return { error: e.message, _diagnostic_request_id: diagnostic.requestId };
    } finally {
      t.clear();
    }
  }, []);

  // Download the vehicle's stored geofence (inclusion ring + keep-out rings) —
  // the fence's own "waypoints", restored alongside the mission so the plan
  // loads and cleans the same way the zone does.
  const downloadFence = useCallback(async (sysId) => {
    try {
      const res = await fetch(`/api/vehicles/${sysId}/fence`);
      if (res.ok) return await res.json();
    } catch (e) {
      console.error('Download fence error:', e);
    }
    return null;
  }, []);

  const restartMission = useCallback(async (sysIds, opts) => {
    try {
      const res = await fetch('/api/control/restart', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          sys_ids: sysIds,
          ...(opts?.force ? { force: true } : {}),
          ...(opts?.pitotCovered ? { pitot_covered: true } : {}),
        }),
      });
      if (res.ok) return await res.json();
    } catch (e) {
      console.error('Restart error:', e);
    }
    return null;
  }, []);

  return {
    analyze,
    generate,
    uploadMissions,
    writeAasParams,
    restartCompanionsReady,
    sendCommand,
    launchSequence,
    triggerVehicle,
    restartMission,
    connectVehicle,
    scanVehicles,
    discoverVehicles,
    disconnectVehicle,
    downloadMission,
    downloadFence,
  };
}
