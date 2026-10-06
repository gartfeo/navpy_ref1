import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import {
  selectChangedFields,
  mergeAckedFieldsIntoBaseline,
  aggregateFanoutResults,
  getConsensus,
} from '../utils/aasParams';

/**
 * Per-vehicle AAS-parameter store.
 *
 * Single owner of:
 *   - confirmedByVehicle: last-known autopilot truth (downloaded or acked).
 *   - draftByVehicle: staged edits per vehicle, baseline = confirmed.
 *   - sessionDraft: pre-connect edits; never auto-pushed on connect.
 *   - pendingByVehicle: { [sysId]: 'refresh' | 'upload' | null }.
 *   - lastStatus: structured outcome of the most recent action.
 *
 * The hook intentionally exposes both raw state and stable getters. State
 * is the natural way to drive renders; getters are convenience accessors
 * for call sites that only read one cell at a time.
 */
export default function useAasParams(vehicleList) {
  const [confirmedByVehicle, setConfirmedByVehicle] = useState({});
  const [draftByVehicle, setDraftByVehicle] = useState({});
  const [sessionDraft, setSessionDraftState] = useState({});
  const [pendingByVehicle, setPendingByVehicle] = useState({});
  const [lastStatus, setLastStatus] = useState({ kind: 'idle' });

  // Live ref to current vehicleList sys_ids -- async callbacks below need
  // to know whether a sys_id is still connected without becoming stale.
  const liveSysIdsRef = useRef(new Set());
  useEffect(() => {
    liveSysIdsRef.current = new Set((vehicleList || []).map((v) => v.sys_id));
  }, [vehicleList]);

  // Mirror of pendingByVehicle so claimPending can compute "skipped" ids
  // synchronously without abusing a setState updater for return data.
  const pendingByVehicleRef = useRef({});
  useEffect(() => { pendingByVehicleRef.current = pendingByVehicle; }, [pendingByVehicle]);

  // Prune confirmed/draft/pending entries for vehicles that left the list.
  // Late acks for removed vehicles are ignored at merge-time via the
  // liveSysIdsRef check, but state hygiene still matters when the same
  // sys_id reconnects.
  useEffect(() => {
    const live = new Set((vehicleList || []).map((v) => v.sys_id));
    setConfirmedByVehicle((prev) => pruneToSet(prev, live));
    setDraftByVehicle((prev) => pruneToSet(prev, live));
    setPendingByVehicle((prev) => pruneToSet(prev, live));
  }, [vehicleList]);

  const setVehicleDraft = useCallback((sysId, key, value) => {
    setDraftByVehicle((prev) => ({
      ...prev,
      [sysId]: { ...(prev[sysId] || {}), [key]: value },
    }));
  }, []);

  const setSessionDraft = useCallback((key, value) => {
    setSessionDraftState((prev) => ({ ...prev, [key]: value }));
  }, []);

  const resetDraft = useCallback((ids) => {
    setDraftByVehicle((prev) => {
      const next = { ...prev };
      for (const sid of ids) {
        const conf = confirmedByVehicleRef.current[sid];
        if (conf) next[sid] = { ...conf };
      }
      return next;
    });
  }, []);

  // Confirmed-by-vehicle ref so async callbacks observe the latest baseline
  // when merging acks. Avoids stale closures without forcing re-renders.
  const confirmedByVehicleRef = useRef(confirmedByVehicle);
  useEffect(() => { confirmedByVehicleRef.current = confirmedByVehicle; }, [confirmedByVehicle]);

  const draftByVehicleRef = useRef(draftByVehicle);
  useEffect(() => { draftByVehicleRef.current = draftByVehicle; }, [draftByVehicle]);

  /**
   * Mark vehicles as busy with a given op kind. Returns the subset of
   * requested ids that were already busy with another op (so the caller
   * can surface a "skipped" list to the user).
   *
   * Uses pendingByVehicleRef for the synchronous "skipped" decision so
   * the return value does not depend on a setState updater running.
   */
  const claimPending = useCallback((ids, kind) => {
    const current = pendingByVehicleRef.current;
    const skipped = ids.filter((sid) => !!current[sid]);
    const claimed = ids.filter((sid) => !current[sid]);
    if (claimed.length > 0) {
      const merged = { ...current };
      for (const sid of claimed) merged[sid] = kind;
      pendingByVehicleRef.current = merged;
      setPendingByVehicle(merged);
    }
    return skipped;
  }, []);

  const releasePending = useCallback((ids) => {
    const current = pendingByVehicleRef.current;
    let any = false;
    const next = { ...current };
    for (const sid of ids) {
      if (Object.prototype.hasOwnProperty.call(next, sid)) {
        delete next[sid];
        any = true;
      }
    }
    if (any) {
      pendingByVehicleRef.current = next;
      setPendingByVehicle(next);
    }
  }, []);

  /**
   * Download AAS params for each id. On success: confirmed AND draft are
   * replaced with the downloaded payload (Download wipes edits). Returns
   * `{ paramsBySysId, skippedIds }` so callers can sync adjacent state
   * (e.g. AasTab also fetches missions in parallel for WaypointEditorOverlay).
   */
  const refreshVehicles = useCallback(async (ids) => {
    if (!ids || ids.length === 0) {
      return { paramsBySysId: {}, skippedIds: [] };
    }
    const skipped = claimPending(ids, 'refresh');
    const active = ids.filter((sid) => !skipped.includes(sid));
    setLastStatus({ kind: 'downloading', counts: { uavTotal: active.length } });

    const responses = await Promise.all(active.map((sid) =>
      fetch(`/api/vehicles/${sid}/params`)
        .then((r) => r.ok ? r.json() : null)
        .catch(() => null),
    ));

    // Compute paramsBySysId synchronously from responses BEFORE any
    // setState call -- React state updaters are not a return-data
    // mechanism. Drop late results for vehicles that disconnected
    // during the round-trip.
    const paramsBySysId = {};
    for (let i = 0; i < active.length; i++) {
      const sid = active[i];
      if (!liveSysIdsRef.current.has(sid)) continue;
      const data = responses[i];
      if (data && data.params) paramsBySysId[sid] = { ...data.params };
    }
    if (Object.keys(paramsBySysId).length > 0) {
      setConfirmedByVehicle((prev) => ({ ...prev, ...paramsBySysId }));
      setDraftByVehicle((prev) => {
        const next = { ...prev };
        for (const sid of Object.keys(paramsBySysId)) {
          next[sid] = { ...paramsBySysId[sid] };
        }
        return next;
      });
    }

    releasePending(active);
    const okCount = Object.keys(paramsBySysId).length;
    setLastStatus({
      kind: okCount === active.length ? 'ok' : 'partial',
      op: 'refresh',
      counts: { uavOk: okCount, uavTotal: active.length },
      skippedIds: skipped,
    });
    return { paramsBySysId, skippedIds: skipped };
  }, [claimPending, releasePending]);

  /**
   * Upload only the staged-and-changed fields per vehicle. Optional
   * `keys` whitelist further narrows the comparison (e.g. ConfirmSection
   * uploads only nav_auto_cm even when other drafts have drifted).
   *
   * Snapshot semantics: the per-vehicle `submitted` payload is computed
   * BEFORE the network round-trip, so an in-flight re-edit cannot be
   * silently baselined by the autopilot's ack for the older value.
   */
  const uploadDraft = useCallback(async (ids, keys) => {
    if (!ids || ids.length === 0) {
      return { submittedByVehicle: {}, perVehicleResults: {}, aggregated: emptyAgg(), skippedIds: [] };
    }
    const skipped = claimPending(ids, 'upload');
    const active = ids.filter((sid) => !skipped.includes(sid));

    const submittedByVehicle = {};
    for (const sid of active) {
      const draft = draftByVehicleRef.current[sid];
      const conf = confirmedByVehicleRef.current[sid];
      submittedByVehicle[sid] = selectChangedFields(draft, conf, keys);
    }

    setLastStatus({ kind: 'uploading', counts: { uavTotal: active.length } });

    const perVehicleResults = await fanoutPut(submittedByVehicle);

    // Only merge for vehicles that (a) actually had something submitted
    // and (b) are still connected. Skipping empty submissions avoids
    // creating empty {} confirmed entries for vehicles whose draft had
    // no diff against baseline.
    setConfirmedByVehicle((prev) => {
      const next = { ...prev };
      for (const sid of active) {
        if (!liveSysIdsRef.current.has(sid)) continue;
        const submitted = submittedByVehicle[sid];
        if (!submitted || Object.keys(submitted).length === 0) continue;
        next[sid] = mergeAckedFieldsIntoBaseline(
          prev[sid],
          submitted,
          perVehicleResults[sid] || null,
        );
      }
      return next;
    });

    releasePending(active);
    const aggregated = aggregateFanoutResults(perVehicleResults, submittedByVehicle);
    setLastStatus({
      kind: aggregated.uavFail === 0 ? 'ok' : (aggregated.uavOk === 0 ? 'error' : 'partial'),
      op: 'upload',
      counts: aggregated,
      skippedIds: skipped,
    });
    return { submittedByVehicle, perVehicleResults, aggregated, skippedIds: skipped };
  }, [claimPending, releasePending]);

  /**
   * Upload an explicit patch to every requested vehicle. The patch is
   * the truth -- draft state is not consulted. Used by ConfirmSection
   * (Step 5) which commits a single field on select-change or blur.
   * Updates confirmed via mergeAckedFieldsIntoBaseline; does not modify
   * draft for unrelated keys.
   */
  const uploadPatch = useCallback(async (ids, patch) => {
    if (!ids || ids.length === 0 || !patch || Object.keys(patch).length === 0) {
      return { submittedByVehicle: {}, perVehicleResults: {}, aggregated: emptyAgg(), skippedIds: [] };
    }
    const skipped = claimPending(ids, 'upload');
    const active = ids.filter((sid) => !skipped.includes(sid));

    const submittedByVehicle = {};
    for (const sid of active) submittedByVehicle[sid] = { ...patch };

    setLastStatus({ kind: 'uploading', counts: { uavTotal: active.length } });

    const perVehicleResults = await fanoutPut(submittedByVehicle);

    setConfirmedByVehicle((prev) => {
      const next = { ...prev };
      for (const sid of active) {
        if (!liveSysIdsRef.current.has(sid)) continue;
        next[sid] = mergeAckedFieldsIntoBaseline(
          prev[sid],
          submittedByVehicle[sid],
          perVehicleResults[sid] || null,
        );
      }
      return next;
    });
    // Acked fields in draft also advance to the patch value -- avoids
    // showing the user a "pending change" indicator immediately after
    // their explicit commit landed. Same liveSysIdsRef guard as confirmed
    // state so a late ack after disconnect cannot resurrect draft
    // entries for a removed vehicle.
    setDraftByVehicle((prev) => {
      const next = { ...prev };
      for (const sid of active) {
        if (!liveSysIdsRef.current.has(sid)) continue;
        const results = perVehicleResults[sid];
        if (!results) continue;
        const ackedSlice = {};
        for (const k of Object.keys(patch)) {
          if (results[k] === true) ackedSlice[k] = patch[k];
        }
        if (Object.keys(ackedSlice).length > 0) {
          next[sid] = { ...(prev[sid] || {}), ...ackedSlice };
        }
      }
      return next;
    });

    releasePending(active);
    const aggregated = aggregateFanoutResults(perVehicleResults, submittedByVehicle);
    setLastStatus({
      kind: aggregated.uavFail === 0 ? 'ok' : (aggregated.uavOk === 0 ? 'error' : 'partial'),
      op: 'upload',
      counts: aggregated,
      skippedIds: skipped,
    });
    return { submittedByVehicle, perVehicleResults, aggregated, skippedIds: skipped };
  }, [claimPending, releasePending]);

  // ---- Read helpers (stable refs via useCallback) ----

  const getDraftValue = useCallback((sysId, key) => {
    const d = draftByVehicle[sysId];
    if (d && Object.prototype.hasOwnProperty.call(d, key)) return d[key];
    const c = confirmedByVehicle[sysId];
    return c ? c[key] : undefined;
  }, [draftByVehicle, confirmedByVehicle]);

  const getConfirmedValue = useCallback((sysId, key, fallback) => {
    const c = confirmedByVehicle[sysId];
    if (c && c[key] !== undefined) return c[key];
    return fallback;
  }, [confirmedByVehicle]);

  const getConfirmedConsensus = useCallback((key) => {
    const sysIds = (vehicleList || []).map((v) => v.sys_id);
    return getConsensus(confirmedByVehicle, sysIds, key);
  }, [confirmedByVehicle, vehicleList]);

  // useMemo so identity stable across renders that don't change inputs.
  return useMemo(() => ({
    confirmedByVehicle,
    draftByVehicle,
    sessionDraft,
    pendingByVehicle,
    lastStatus,
    setVehicleDraft,
    setSessionDraft,
    resetDraft,
    refreshVehicles,
    uploadDraft,
    uploadPatch,
    getDraftValue,
    getConfirmedValue,
    getConfirmedConsensus,
  }), [
    confirmedByVehicle, draftByVehicle, sessionDraft, pendingByVehicle, lastStatus,
    setVehicleDraft, setSessionDraft, resetDraft,
    refreshVehicles, uploadDraft, uploadPatch,
    getDraftValue, getConfirmedValue, getConfirmedConsensus,
  ]);
}


// ---- Module-private helpers ----

function pruneToSet(map, live) {
  let changed = false;
  const next = {};
  for (const k of Object.keys(map)) {
    // sys_ids may be number or string depending on source; compare both.
    if (live.has(Number(k)) || live.has(k)) {
      next[k] = map[k];
    } else {
      changed = true;
    }
  }
  return changed ? next : map;
}

async function fanoutPut(submittedByVehicle) {
  const sysIds = Object.keys(submittedByVehicle).filter(
    (sid) => Object.keys(submittedByVehicle[sid]).length > 0,
  );
  const responses = await Promise.all(sysIds.map((sid) =>
    fetch(`/api/vehicles/${sid}/params`, {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ params: submittedByVehicle[sid] }),
    })
      .then((r) => r.ok ? r.json() : null)
      .catch(() => null),
  ));

  const out = {};
  for (let i = 0; i < sysIds.length; i++) {
    const sid = sysIds[i];
    const data = responses[i];
    out[sid] = data && data.results ? data.results : null;
  }
  return out;
}

function emptyAgg() {
  return { uavOk: 0, uavFail: 0, fieldOk: 0, fieldFail: 0, perKeyOk: {}, perKeyFail: {} };
}
