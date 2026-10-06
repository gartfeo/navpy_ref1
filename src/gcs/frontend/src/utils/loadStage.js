/**
 * Per-vehicle connect/load stage — the "traceable loading" state for a UAV
 * between first contact and being fully ready.
 *
 * Derived from BACKEND telemetry (link_ok + is_probing) so it works in
 * auto-connect mode too, where the frontend never runs the /mission download and
 * so has no local download state to key off. A frontend-driven download
 * (downloadingSysIds, set by the manual connect / Download Plan paths) also
 * counts as "downloading".
 *
 *   connecting  — no fresh heartbeat yet (link not up)
 *   downloading — backend mission probe in flight (v.is_probing) OR a
 *                 frontend-driven mission download is in flight
 *   ready       — link up and the mission probe/download has settled
 *
 * Note: a vehicle with no mission on board still reaches "ready" — readiness is
 * "probe finished", not "a mission was found" (mission_uploaded stays false when
 * there's nothing to download).
 */
export function computeLoadStage(v, downloadingSysIds) {
  if (!v) return 'ready';
  if (v.pending) return downloadingSysIds?.has?.(v.sys_id) ? 'downloading' : 'connecting';
  if (!v.link_ok) return 'connecting';
  if (v.is_probing || downloadingSysIds?.has?.(v.sys_id)) return 'downloading';
  return 'ready';
}

/**
 * Is a vehicle still initializing (coming online) as opposed to ready?
 *
 * Drives which card sections to hide while a UAV loads: during connecting /
 * downloading the card shows only identity + connection progress + the stage
 * badge, hiding premature/contradictory bits (a "mission not uploaded" warning
 * while the mission is downloading, a force-launch button before the UAV is
 * ready, stale on-ground mode/speed/alt). An unknown/missing stage is treated as
 * NOT initializing (undefined → false) so the full card shows by default — the
 * hiding only kicks in when we positively know the UAV is still loading.
 */
export function isInitializing(loadStage) {
  return !!loadStage && loadStage !== 'ready';
}

/**
 * Derive per-vehicle display stages for a whole list, LATCHING 'ready'.
 *
 * Pure (no mutation of inputs): given the current vehicles and the prior
 * "ever-ready" set, returns `{ stages, ready }` where `stages` is the displayed
 * stage per sys_id and `ready` is the NEXT ever-ready set. The latch pins a UAV
 * at 'ready' once it has finished its initial connect+probe, so a later transient
 * link drop surfaces via the link/CC indicators rather than reverting the status
 * row to a misleading "Connecting". The latch is narrow: it suppresses only a
 * post-ready `connecting` (a link blip); a genuine later `downloading` (e.g. a
 * manual "Download Plan" re-download) is NOT suppressed, so re-download progress
 * still surfaces. `ready` only ever contains vehicles present in `vehicleList`,
 * so a vehicle that leaves (disconnect) is auto-pruned and a genuine reconnect
 * replays the stages.
 *
 * Kept pure + separate from the hook so it is unit-testable and so the hook can
 * read it during render but only *commit* the new set in an effect (no
 * render-time ref mutation).
 */
export function deriveLoadStages(vehicleList, downloadingSysIds, prevReady) {
  const ready = new Set();
  const stages = new Map();
  for (const v of vehicleList) {
    const raw = computeLoadStage(v, downloadingSysIds);
    // Latch only a post-ready link blip (`connecting`), never a real `downloading`:
    // a manual re-download must still surface progress instead of flapping to 'ready'.
    const latched = raw === 'ready' || (prevReady.has(v.sys_id) && raw === 'connecting');
    if (latched) ready.add(v.sys_id);
    stages.set(v.sys_id, latched ? 'ready' : raw);
  }
  return { stages, ready };
}

/**
 * Download completion percentage from a `[current, total]` progress pair (the
 * shape of the backend's `mission_download_progress` telemetry field), or null
 * when there's nothing to show yet (no pair, or total not yet known).
 */
export function progressPct(progress) {
  if (!Array.isArray(progress)) return null;
  const [current, total] = progress;
  if (!(total > 0)) return null;
  return Math.max(0, Math.min(100, Math.round((current / total) * 100)));
}
