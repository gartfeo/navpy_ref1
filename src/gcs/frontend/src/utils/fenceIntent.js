/**
 * Geofence observation and intent — pure helpers.
 *
 * Two things that used to be one. They must stay apart:
 *
 * - An OBSERVATION is what a vehicle reports about its own fence (its stored
 *   ring plus its raw FENCE_ENABLE/AUTOENABLE/TYPE/ACTION). It is display
 *   truth only. It never becomes upload geometry, and it never stands in for
 *   an operator decision.
 * - An INTENT is an explicit operator request (enable / disable) that has not
 *   yet been acknowledged by an upload. It carries the generation it was
 *   authored at, so an acknowledgement can only consume the exact request it
 *   answers: a failed upload keeps it pending, and an edit made while the
 *   upload was in flight is never silently thrown away.
 */

/** Minimum vertices ArduPilot will enforce as an inclusion polygon. */
export const MIN_FENCE_RING = 3;

/** Latitude/longitude equality tolerance. Rings round-trip through int 1e7. */
const RING_EPS = 1e-7;

export const FENCE_MODE = {
  /** Nothing observed yet (no download in this plan's lifecycle). */
  NONE: 'none',
  /** Observed, but the vehicle's fence parameters could not be read. */
  UNKNOWN: 'unknown',
  /** FENCE_ENABLE=0 and FENCE_AUTOENABLE=0. */
  OFF: 'off',
  /** FENCE_ENABLE=0 with FENCE_AUTOENABLE>0 — the vehicle arms its own
   *  fence, NOT off. Which behaviour depends on the value; see
   *  `autoenableModes` below. */
  AUTO: 'auto',
  /** FENCE_ENABLE>0 — enforced now. */
  ON: 'on',
  /** The observed vehicles do not agree. Never collapse this to one mode. */
  MIXED: 'mixed',
};

const copyRing = (points) => (points || [])
  .filter((p) => p && p.lat != null && p.lon != null)
  .map((p) => ({ lat: p.lat, lon: p.lon }));

function modeFromParams(params) {
  if (!params) return FENCE_MODE.UNKNOWN;
  if (params.enable > 0) return FENCE_MODE.ON;
  if (params.autoenable > 0) return FENCE_MODE.AUTO;
  return FENCE_MODE.OFF;
}

/**
 * Normalize one `GET /api/vehicles/{id}/fence` response into an observation.
 *
 * A missing response is a FAILED readback, not an empty fence: "we could not
 * ask" and "the vehicle has no fence" are different facts, and only the second
 * one may ever be displayed as a fence that is off.
 */
export function fenceObservationFromDownload(fence) {
  if (!fence) {
    return { status: 'failed', mode: FENCE_MODE.UNKNOWN, ring: [], exclusions: [], params: null };
  }
  // The backend sends the raw four values, or null when any of them could not
  // be read. It never invents a 0, and neither do we.
  const params = fence.params_readback === 'ok' && fence.params ? {
    enable: fence.params.enable,
    autoenable: fence.params.autoenable,
    type: fence.params.type,
    action: fence.params.action,
  } : null;
  return {
    status: 'known',
    mode: modeFromParams(params),
    ring: copyRing(fence.vertices),
    exclusions: (fence.exclusions || [])
      .filter((r) => Array.isArray(r) && r.length >= MIN_FENCE_RING)
      .map(copyRing),
    params,
  };
}

/** A vehicle we have asked about but not yet heard from. */
export function pendingFenceObservation(startedSeq = null) {
  return {
    status: 'pending', mode: FENCE_MODE.UNKNOWN, ring: [], exclusions: [], params: null,
    // Newest request STARTED for this vehicle. Carried through every state so a
    // reply older than it can be recognized as stale even while we are pending.
    startedSeq,
  };
}

/** Newest request started (or landed) for a vehicle. */
function watermarkOf(entry) {
  if (!entry) return null;
  const seen = [entry.startedSeq, entry.seq].filter((s) => s != null);
  return seen.length ? Math.max(...seen) : null;
}

/** Initial (nothing observed, nothing asked) state. */
export function emptyFenceObservations() {
  return { epoch: 0, edit: 0, bySysId: {} };
}

/**
 * Start a new observation lifecycle — a new/replacement plan, a cleared plan,
 * or a roster change. Results still in flight from the previous epoch are
 * dropped by {@link recordFenceObservation}.
 */
export function resetFenceObservations(state) {
  return { epoch: (state?.epoch ?? 0) + 1, edit: state?.edit ?? 0, bySysId: {} };
}

/**
 * Invalidate reads that are in flight because the operator authored a change.
 * Unlike a reset this KEEPS what we already know: the fleet's recorded state is
 * still the fleet's state; only answers to questions asked before the edit are
 * no longer trustworthy as a response to the current one.
 */
export function invalidateFenceObservations(state) {
  const base = state ?? emptyFenceObservations();
  return { epoch: base.epoch, edit: base.edit + 1, bySysId: base.bySysId };
}

/**
 * Mark a roster as asked-but-unanswered, BEFORE the requests are issued.
 *
 * This has to happen before the plan is published: otherwise there is a window
 * where a plan exists and no observation does, and anything keyed on "we know
 * nothing about the fleet's fence" — the demo auto-enable default — fires into
 * it and authors a fence the operator never asked for.
 *
 * `seq` is the request this reservation belongs to, when it has one. Returning
 * a vehicle to "pending" must never lower its ordering watermark: dropping it
 * let a reply from an older read land after a newer read had already answered.
 */
export function reserveFenceObservations(state, sysIds, seq = null) {
  const base = state ?? emptyFenceObservations();
  const bySysId = { ...base.bySysId };
  for (const sysId of sysIds || []) {
    if (sysId == null) continue;
    const previous = watermarkOf(bySysId[sysId]);
    const startedSeq = previous == null ? seq
      : (seq == null ? previous : Math.max(previous, seq));
    bySysId[sysId] = pendingFenceObservation(startedSeq);
  }
  return { epoch: base.epoch, edit: base.edit, bySysId };
}

/** Token identifying one in-flight read: its lifecycle and its order. */
export function fenceObservationToken(state, seq) {
  const base = state ?? emptyFenceObservations();
  return { epoch: base.epoch, edit: base.edit, seq };
}

/**
 * Store one vehicle's observation, unless the answer is stale: it belongs to a
 * superseded plan/roster lifecycle, it answers a question the operator has
 * since overridden, or a NEWER read of the same vehicle already landed (slow
 * responses can overtake each other). Returns the unchanged state object when
 * the result is rejected, so callers can skip a re-render.
 */
export function recordFenceObservation(state, sysId, observation, token) {
  const base = state ?? emptyFenceObservations();
  if (!token || token.epoch !== base.epoch || token.edit !== base.edit) return base;
  const previous = base.bySysId[sysId];
  // Compared against the newest request STARTED for this vehicle, not merely
  // the last one that landed: a reservation between the two puts the entry back
  // to pending, and the older reply must still lose.
  const watermark = watermarkOf(previous);
  if (watermark != null && token.seq < watermark) return base;
  return {
    epoch: base.epoch,
    edit: base.edit,
    bySysId: {
      ...base.bySysId,
      [sysId]: { ...observation, seq: token.seq, startedSeq: token.seq },
    },
  };
}

function ringsEqual(a, b) {
  if (!a || !b || a.length !== b.length) return false;
  return a.every((p, i) => (
    Math.abs(p.lat - b[i].lat) <= RING_EPS && Math.abs(p.lon - b[i].lon) <= RING_EPS
  ));
}

/**
 * Fleet view of the observed fences, restricted to the current roster.
 *
 * `ring` is non-null only when every rostered vehicle reported the SAME ring —
 * a single ring is the only thing that can be drawn honestly for the fleet.
 * It is display geometry; nothing here may be uploaded.
 */
export function summarizeFenceObservations(state, sysIds) {
  const bySysId = state?.bySysId || {};
  const roster = sysIds || [];
  // A rostered vehicle we have no entry for is NOT absent from the fleet
  // answer — it is a vehicle we do not know about. Dropping it here is how a
  // single "enabled" reply used to speak for a whole fleet.
  const observations = roster.map((id) => bySysId[id] ?? pendingFenceObservation());
  const modes = {};
  // The raw FENCE_AUTOENABLE value per vehicle. The collapsed mode says only
  // THAT a fence arms itself; the value identifies which behaviour (AC_Fence:
  // 3 = ONLY_WHEN_ARMED, 2 = ENABLE_DISABLE_FLOOR_ONLY), and the operator
  // needs it to decide whether to leave a fleet alone. Reported as the raw
  // value, never as a trigger we did not read.
  const autoenableModes = {};
  roster.forEach((id, i) => {
    modes[id] = observations[i].mode;
    autoenableModes[id] = observations[i].params?.autoenable ?? null;
  });
  const asked = roster.some((id) => bySysId[id] !== undefined);
  const autoenableValues = roster.map((id) => autoenableModes[id]);
  const autoenableMode = autoenableValues.length > 0
    && autoenableValues.every((v) => v != null && v === autoenableValues[0])
    ? autoenableValues[0] : null;

  if (observations.length === 0 || !asked) {
    return {
      hasObservations: false, count: 0, knownCount: 0, failedCount: 0,
      allFailed: false, mode: FENCE_MODE.NONE, ring: null, ringMixed: false,
      exclusions: [], modes, autoenableModes, autoenableMode: null,
    };
  }

  // "Not known" covers both a failed readback and a vehicle still pending.
  const failed = observations.filter((o) => o.status !== 'known');
  const known = observations.filter((o) => o.status === 'known');
  const distinct = new Set(observations.map((o) => o.mode));
  const mode = distinct.size === 1 ? [...distinct][0] : FENCE_MODE.MIXED;

  const rings = known.map((o) => o.ring).filter((r) => r.length >= MIN_FENCE_RING);
  const homogeneous = rings.length > 0 && rings.every((r) => ringsEqual(r, rings[0]));
  const completeRoster = failed.length === 0 && rings.length === observations.length;

  // Keep-outs follow the same rule as the ring: displayable only when the whole
  // roster agrees, and never uploadable.
  const exclusionSets = known.map((o) => o.exclusions || []);
  const exclusionsAgree = completeRoster && exclusionSets.length > 0
    && exclusionSets.every((set) => (
      set.length === exclusionSets[0].length
      && set.every((r, i) => ringsEqual(r, exclusionSets[0][i]))
    ));

  return {
    exclusions: exclusionsAgree ? exclusionSets[0].map((r) => r.map((p) => ({ ...p }))) : [],
    hasObservations: true,
    count: observations.length,
    knownCount: known.length,
    failedCount: failed.length,
    allFailed: known.length === 0,
    mode,
    ring: homogeneous && completeRoster ? rings[0].map((p) => ({ ...p })) : null,
    ringMixed: rings.length > 1 && !homogeneous,
    modes,
    autoenableModes,
    autoenableMode,
  };
}

/**
 * Stable identity of a fence request: what would actually be written.
 *
 * Coordinates are rounded to the precision the vehicle stores them at (int
 * 1e7), so a re-render that recomputes the same ring is the same request,
 * while an operator edit of a vertex or a keep-out is a different one.
 */
export function fenceSignature({ enabled, vertices, exclusions }) {
  const round = (p) => [Math.round(p.lat * 1e7), Math.round(p.lon * 1e7)];
  const ringOf = (r) => copyRing(r).map(round);
  return JSON.stringify({
    enabled: !!enabled,
    vertices: ringOf(vertices),
    exclusions: (exclusions || [])
      .filter((r) => Array.isArray(r) && r.length >= MIN_FENCE_RING)
      .map(ringOf),
  });
}

/**
 * Should the demo default switch the fence on?
 *
 * Only for a plan we have authored nothing about AND asked no vehicle about.
 * Once a fence read is outstanding — pending, failed, or answered — the fleet's
 * own configuration is the subject, and a default that silently overwrites it
 * is not a default, it is an edit.
 */
export function shouldAutoEnableDemoFence({
  simMode, polygonLength, fenceTouched, fenceEnabled, observations,
}) {
  if (!simMode || polygonLength < MIN_FENCE_RING) return false;
  if (fenceTouched || fenceEnabled) return false;
  return !observations?.hasObservations;
}

/**
 * Build the upload request's fence block.
 *
 * Absent (null) means "do not touch the vehicle's fence" — the default, and
 * what an untouched download/upload round trip must produce. An explicit
 * disable is only sent while that request is still pending. An enable carries
 * the operator's OWN geometry; invalid geometry is reported as `invalid` so
 * the caller can refuse, because quietly downgrading an enable request to a
 * disable switches a fence off that the operator asked to switch on.
 */
export function fenceUploadPayload({ enabled, intent, vertices, exclusions, acknowledged }) {
  if (enabled) {
    const ring = copyRing(vertices);
    if (ring.length < MIN_FENCE_RING) return { payload: null, invalid: true, signature: null };
    const rings = (exclusions || [])
      .filter((r) => Array.isArray(r) && r.length >= MIN_FENCE_RING)
      .map(copyRing);
    // Only a request that is still outstanding is sent. An enable that a
    // previous upload already applied is NOT re-sent: re-writing the fence on
    // every later upload would overwrite whatever the vehicles hold by then,
    // for no operator reason. A toggle raises new intent; a geometry edit
    // changes the signature, which is itself a new request.
    const signature = fenceSignature({ enabled: true, vertices: ring, exclusions: rings });
    const pending = intent != null
      || acknowledged == null
      || acknowledged.signature !== signature;
    if (!pending) return { payload: null, invalid: false, signature };
    return {
      invalid: false,
      signature,
      payload: {
        enabled: true,
        action: 1, // FENCE_ACTION: RTL
        type: 4,   // FENCE_TYPE: polygon (inclusion + exclusion)
        vertices: ring,
        exclusions: rings,
      },
    };
  }
  if (intent && intent.enabled === false) {
    // Disable touches FENCE_ENABLE/FENCE_AUTOENABLE only; the backend keeps the
    // vehicle's stored ring, FENCE_TYPE and FENCE_ACTION.
    const payload = { enabled: false, action: 1, type: 4, vertices: [], exclusions: [] };
    return { invalid: false, payload, signature: fenceSignature(payload) };
  }
  return { payload: null, invalid: false, signature: null };
}

/**
 * Resolve pending intent against one upload result. Intent survives a failure
 * and survives an edit made after the upload started; only the exact
 * acknowledged generation is consumed.
 */
export function consumeFenceIntent(intent, ackGeneration, ok) {
  if (!intent || !ok) return intent;
  if (intent.generation !== ackGeneration) return intent;
  return null;
}

/**
 * Did the fence work the operator asked for actually land? `fence_ok` is the
 * backend's fleet answer; the per-vehicle outcomes name what failed.
 */
export function fenceUploadFailures(result) {
  if (!result) return [];
  return (result.results || [])
    .filter((r) => r.fence && r.fence.applied === false)
    .map((r) => ({
      sysId: r.sys_id,
      action: r.fence.action,
      error: r.fence.error || null,
      failedParams: r.fence.failed_params || [],
    }));
}
