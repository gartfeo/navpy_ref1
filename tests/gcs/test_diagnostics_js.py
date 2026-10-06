"""Node-backed tests for frontend diagnostic IDs and privacy-safe plan summaries."""
from __future__ import annotations

import json
import re
import subprocess
from pathlib import Path


SOURCE_PATH = Path(__file__).parents[2] / "src" / "gcs" / "frontend" / "src" / "utils" / "diagnostics.js"
VEHICLE_CONNECTION_PATH = SOURCE_PATH.parents[1] / "hooks" / "useVehicleConnection.js"


def _run(script: str) -> dict:
    source = SOURCE_PATH.read_text(encoding="utf-8")
    source = re.sub(r"^export function ", "function ", source, flags=re.MULTILINE)
    program = f"""
globalThis.crypto = {{ randomUUID: (() => {{ let n = 0; return () => `id-${{++n}}`; }})() }};
globalThis.sessionStorage = {{ getItem: () => null, setItem: () => {{}} }};
const sent = [];
globalThis.fetch = async (url, options) => {{ sent.push({{ url, options }}); return {{ ok: true }}; }};
{source}
{script}
"""
    result = subprocess.run(["node", "-e", program], capture_output=True, text=True, check=True)
    return json.loads(result.stdout)


def test_request_ids_are_unique_and_headers_match():
    result = _run("""
const a = diagnosticRequest(); const b = diagnosticRequest();
console.log(JSON.stringify({ a, b }));
""")
    assert result["a"]["requestId"] != result["b"]["requestId"]
    assert result["a"]["headers"]["X-GCS-Request-ID"] == result["a"]["requestId"]
    assert result["a"]["headers"]["X-GCS-Client-ID"]


def test_plan_summary_contains_only_ids_and_counts():
    result = _run("""
const summary = planDiagnosticFields([
  { sys_id: 161, track: [{ lat: 40, lon: 44 }, { lat: 41, lon: 45 }] },
  { sys_id: 162, track: [{ lat: 42, lon: 46 }] },
]);
console.log(JSON.stringify(summary));
""")
    assert result == {"zone_count": 2, "zone_sys_ids": [161, 162], "waypoint_counts": [2, 1]}
    assert "lat" not in json.dumps(result)


def test_emit_is_fire_and_forget_and_adds_client_id():
    result = _run("""
emitDiagnostic('client_timeout', { request_id: 'req-1', sys_id: 163 });
setTimeout(() => console.log(JSON.stringify(JSON.parse(sent[0].options.body))), 0);
""")
    assert result["event"] == "client_timeout"
    assert result["fields"]["request_id"] == "req-1"
    assert result["fields"]["client_id"]
    assert result["fields"]["client_seq"] == 1


def test_delivery_is_serialized_and_flush_waits_for_pending_events():
    result = _run("""
(async () => {
  const pending = [];
  globalThis.fetch = (url, options) => new Promise((resolve) => {
    sent.push({ url, options }); pending.push(resolve);
  });
  emitDiagnostic('client_timeout', { request_id: 'req-1', sys_id: 161, phase: 'mission', outcome: 'timeout' });
  emitDiagnostic('client_timeout', { request_id: 'req-2', sys_id: 162, phase: 'mission', outcome: 'timeout' });
  await new Promise((resolve) => setImmediate(resolve));
  const beforeFirstSettlement = sent.length;
  pending[0]({ ok: true });
  await new Promise((resolve) => setImmediate(resolve));
  const afterFirstSettlement = sent.length;
  pending[1]({ ok: true });
  await flushDiagnosticEvents();
  console.log(JSON.stringify({
    beforeFirstSettlement,
    afterFirstSettlement,
    sequences: sent.map((item) => JSON.parse(item.options.body).fields.client_seq),
  }));
})();
""")
    assert result == {
        "beforeFirstSettlement": 1,
        "afterFirstSettlement": 2,
        "sequences": [1, 2],
    }


def test_mission_settlement_uses_callback_sys_id_in_diagnostic_event():
    source = VEHICLE_CONNECTION_PATH.read_text(encoding="utf-8")
    settlement = source.split("onVehicleSettled: (sysId, result) => {", 1)[1].split("if (result.error)", 1)[0]
    assert "sys_id: sysId" in settlement
    assert "sys_id," not in settlement
