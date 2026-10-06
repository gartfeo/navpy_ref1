/**
 * Plain JS pub/sub store for vehicle telemetry data.
 * Holds state outside React so 5 Hz WebSocket updates don't trigger
 * full-tree re-renders. Components subscribe via useTelemetryStore hooks.
 */

let _vehicles = {};
let _connected = false;
let _seenIds = [];
let _version = 0;

const _listeners = new Set();
const _messageHandlers = {};

let _ws = null;
let _reconnectTimer = null;

// Cached derived values — recomputed lazily when version changes
let _vehicleList = [];
let _vehicleListVersion = -1;
let _sysIdKey = '';
let _sysIdKeyVersion = -1;

function _notify() {
  _version++;
  for (const fn of _listeners) fn();
}

function subscribe(listener) {
  _listeners.add(listener);
  return () => _listeners.delete(listener);
}

function getSnapshot() {
  return _version;
}

function getVehicles() {
  return _vehicles;
}

function getVehicleList() {
  if (_vehicleListVersion !== _version) {
    _vehicleList = Object.values(_vehicles);
    _vehicleListVersion = _version;
  }
  return _vehicleList;
}

function getSysIdKey() {
  if (_sysIdKeyVersion !== _version) {
    const keys = Object.keys(_vehicles);
    keys.sort((a, b) => a - b);
    _sysIdKey = keys.join(',');
    _sysIdKeyVersion = _version;
  }
  return _sysIdKey;
}

function isConnected() {
  return _connected;
}

function getSeenIds() {
  return _seenIds;
}

function getMessageHandlers() {
  return _messageHandlers;
}

function removeVehicle(sysId) {
  const next = { ..._vehicles };
  delete next[sysId];
  _vehicles = next;
  _notify();
}

function sendWsMessage(msg) {
  if (_ws?.readyState === WebSocket.OPEN) {
    _ws.send(JSON.stringify(msg));
  }
}

function connect() {
  if (_ws?.readyState === WebSocket.OPEN) return;

  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const ws = new WebSocket(`${protocol}//${window.location.host}/ws/telemetry`);
  _ws = ws;
  ws.binaryType = 'arraybuffer';

  ws.onopen = () => {
    _connected = true;
    _notify();
    if (_reconnectTimer) {
      clearTimeout(_reconnectTimer);
      _reconnectTimer = null;
    }
  };

  ws.onmessage = (evt) => {
    try {
      const data = typeof evt.data === 'string'
        ? JSON.parse(evt.data)
        : JSON.parse(new TextDecoder().decode(evt.data));

      if (data.type === 'telemetry' && data.vehicles) {
        const next = { ..._vehicles };
        for (const v of data.vehicles) {
          const existing = next[v.sys_id];
          if (v.status_texts && existing?.status_texts) {
            v.status_texts = [...existing.status_texts, ...v.status_texts].slice(-10);
          }
          next[v.sys_id] = { ...existing, ...v };
        }
        if (data.active_ids) {
          const active = new Set(data.active_ids);
          for (const id of Object.keys(next)) {
            if (!active.has(Number(id))) delete next[id];
          }
        }
        _vehicles = next;
        if (data.seen_ids) _seenIds = data.seen_ids;
        _notify();
      } else {
        const handler = _messageHandlers[data.type];
        if (handler) handler(data);
      }
    } catch (e) {
      console.error('WS parse error:', e);
    }
  };

  ws.onclose = () => {
    _connected = false;
    _notify();
    _reconnectTimer = setTimeout(connect, 2000);
  };

  ws.onerror = () => {
    ws.close();
  };
}

function disconnect() {
  if (_reconnectTimer) {
    clearTimeout(_reconnectTimer);
    _reconnectTimer = null;
  }
  if (_ws) {
    _ws.onclose = null;
    _ws.close();
    _ws = null;
  }
}

const telemetryStore = {
  subscribe,
  getSnapshot,
  getVehicles,
  getVehicleList,
  getSysIdKey,
  isConnected,
  getSeenIds,
  getMessageHandlers,
  removeVehicle,
  sendWsMessage,
  connect,
  disconnect,
};

export default telemetryStore;
