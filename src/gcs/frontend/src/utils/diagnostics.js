let fallbackId = 0;

function newId() {
  if (globalThis.crypto?.randomUUID) return globalThis.crypto.randomUUID();
  fallbackId += 1;
  return `fallback-${Date.now()}-${fallbackId}`;
}

function getClientId() {
  try {
    const key = 'gcs-diagnostic-client-id';
    let value = globalThis.sessionStorage?.getItem(key);
    if (!value) {
      value = newId();
      globalThis.sessionStorage?.setItem(key, value);
    }
    return value;
  } catch {
    return newId();
  }
}

const clientId = getClientId();
let clientSequence = 0;
let deliveryQueue = Promise.resolve();

export function diagnosticRequest() {
  const requestId = newId();
  return {
    requestId,
    headers: {
      'X-GCS-Client-ID': clientId,
      'X-GCS-Request-ID': requestId,
    },
  };
}

export function emitDiagnostic(event, fields = {}) {
  clientSequence += 1;
  const body = JSON.stringify({
    event,
    fields: { ...fields, client_id: clientId, client_seq: clientSequence },
  });
  try {
    deliveryQueue = deliveryQueue.catch(() => {}).then(() => fetch('/api/diagnostics/events', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body,
        keepalive: true,
      }).catch(() => {}));
  } catch {
    // Diagnostics must never change operator workflow or request outcomes.
  }
  return deliveryQueue;
}

export function flushDiagnosticEvents() {
  return deliveryQueue.catch(() => {});
}

export function planDiagnosticFields(zones = []) {
  return {
    zone_count: zones.length,
    zone_sys_ids: zones.map((zone) => zone.sys_id).filter((sid) => sid != null),
    waypoint_counts: zones.map((zone) => zone.track?.length || 0),
  };
}
