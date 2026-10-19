/**
 * Pure helpers for the per-vehicle camera streams the backend advertises in
 * each vehicle's telemetry snapshot (`video_streams`, docs/design/
 * gcs_video_streams.md section 2): `{ key, camera_component, stream_id, name,
 * encoding, whep_url }`, ordered by camera component then stream id.
 *
 * Video is operator display only; nothing here feeds navigation.
 */

/** Operator-facing vehicle label, the same "UAV n" fallback the rest of the GCS uses. */
export function vehicleLabel(vehicle, sysId) {
  return vehicle?.name || `UAV ${sysId ?? vehicle?.sys_id}`;
}

const isPlayable = (s) => s && typeof s.key === 'string' && typeof s.whep_url === 'string' && s.whep_url !== '';

/** The vehicle's advertised streams (an absent field means none). */
export function streamsOf(vehicle) {
  const list = vehicle?.video_streams;
  return Array.isArray(list) ? list.filter(isPlayable) : [];
}

/**
 * The stream v1 plays for a UAV: the lowest stream id (the companion config
 * gives the display stream id 1). On a tie between two cameras the earlier one
 * in the backend's order (lower camera component) wins.
 */
export function defaultStream(streams) {
  let best = null;
  for (const s of streams || []) {
    if (best === null || s.stream_id < best.stream_id) best = s;
  }
  return best;
}

/**
 * The UAV with video the vehicle window picks by itself (owner decision D1):
 * the followed UAV if it has video, else the first UAV with video, else null.
 * Later follow changes do not call this again.
 */
export function pickFirstFeed(followSysId, fleet) {
  const withVideo = (fleet || []).filter((e) => e.streams.length > 0);
  const followed = withVideo.find((e) => e.sysId === followSysId);
  return (followed ?? withVideo[0])?.sysId ?? null;
}

/** The window's first pick: a UAV with video as above, else the first UAV in the fleet. */
export function pickFirstUav(followSysId, fleet) {
  return pickFirstFeed(followSysId, fleet) ?? fleet?.[0]?.sysId ?? null;
}

/**
 * What the window's camera shows for the selection `{ sysId, key }`; `key`
 * null means no stream is expected (the UAV never had one, or the operator
 * dismissed a lost feed):
 * - `stream`: that stream, or, when it disappeared or none was expected, the
 *   same UAV's default stream (`switched` true);
 * - `lost`: the shown stream went away and the UAV has none left. Never
 *   another UAV.
 * - `noCamera`: no stream expected and the UAV advertises none.
 * - `idle`: nothing selected (empty fleet).
 */
export function resolveShownStream(selection, fleet) {
  if (!selection) return { kind: 'idle' };
  const entry = (fleet || []).find((e) => e.sysId === selection.sysId) ?? null;
  const streams = entry?.streams ?? [];
  const exact = streams.find((s) => s.key === selection.key);
  if (exact) return { kind: 'stream', entry, stream: exact, switched: false };
  const fallback = defaultStream(streams);
  if (fallback) return { kind: 'stream', entry, stream: fallback, switched: true };
  return { kind: selection.key == null ? 'noCamera' : 'lost', entry };
}

const CODEC_MIME = { H264: 'video/h264', H265: 'video/h265' };

/**
 * Can this browser decode the announced encoding over WebRTC? Checked against
 * RTCRtpReceiver.getCapabilities('video') before connecting. An unknown or
 * missing encoding, or a browser without the capability API, is tried.
 */
export function codecSupported(encoding, receiver = globalThis.RTCRtpReceiver) {
  const mime = encoding ? CODEC_MIME[String(encoding).toUpperCase()] : undefined;
  if (!mime) return true;
  const caps = receiver?.getCapabilities?.('video');
  if (!caps?.codecs) return true;
  return caps.codecs.some((c) => String(c.mimeType).toLowerCase() === mime);
}

/** The vehicle window is on screen in the monitor view, outside manual control, whenever the fleet has a UAV. */
export function vehicleWindowVisible({ monitor, rcEnabled, fleetSize }) {
  return monitor && !rcEnabled && fleetSize > 0;
}
