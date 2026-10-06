import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { diagnosticRequest, emitDiagnostic } from '../utils/diagnostics';
import {
  aggregateBatchResults,
  consensusByName,
  draftValueMatchesSubmitted,
  fetchFullParamFleet,
  filterChangesByNames,
  isArmedTokenLive,
  normaliseSnapshot,
  ownsFullParamRefresh,
  pruneToSet,
  resolveSubmissions,
} from '../utils/fullParams';

/**
 * Per-vehicle full-parameter store. Independent of `useAasParams`.
 *
 * Owner of:
 *   - `snapshotsByVehicle: { [sysId]: NormalisedSnapshot }`
 *   - `draftsByVehicle:    { [sysId]: { [name]: value } }`
 *   - `pendingByVehicle:   { [sysId]: 'refresh'|'write' }`
 *   - `armedTokensByVehicle: { [sysId]: { nonce, expires_at_unix_s } }`
 *   - `lastStatus`
 *
 * State is pruned when vehicles leave `vehicleList`. Late acks for
 * disconnected vehicles are dropped at merge-time.
 */
export default function useFullParams(vehicleList) {
  const [snapshotsByVehicle, setSnapshotsByVehicle] = useState({});
  const [draftsByVehicle, setDraftsByVehicle] = useState({});
  const [pendingByVehicle, setPendingByVehicle] = useState({});
  const [armedTokensByVehicle, setArmedTokensByVehicle] = useState({});
  const [writeProgressByVehicle, setWriteProgressByVehicle] = useState({});
  const [refreshProgressByVehicle, setRefreshProgressByVehicle] = useState({});
  const [lastStatus, setLastStatus] = useState({ kind: 'idle' });

  const liveSysIdsRef = useRef(new Set());
  const refreshGenerationRef = useRef({});
  const refreshBatchGenerationRef = useRef(0);
  useEffect(() => {
    liveSysIdsRef.current = new Set((vehicleList || []).map((v) => v.sys_id));
  }, [vehicleList]);

  // Prune state for disconnected vehicles.
  useEffect(() => {
    const live = liveSysIdsRef.current;
    for (const sid of Object.keys(refreshGenerationRef.current)) {
      if (!live.has(Number(sid))) {
        refreshGenerationRef.current[sid] += 1;
      }
    }
    setSnapshotsByVehicle((prev) => pruneToSet(prev, live));
    setDraftsByVehicle((prev) => pruneToSet(prev, live));
    setPendingByVehicle((prev) => {
      const next = pruneToSet(prev, live);
      pendingRef.current = next;
      return next;
    });
    setArmedTokensByVehicle((prev) => pruneToSet(prev, live));
    setWriteProgressByVehicle((prev) => pruneToSet(prev, live));
    setRefreshProgressByVehicle((prev) => pruneToSet(prev, live));
  }, [vehicleList]);

  // Live mirrors so async callbacks observe latest state without stale closures.
  const snapshotsRef = useRef(snapshotsByVehicle);
  const draftsRef = useRef(draftsByVehicle);
  const pendingRef = useRef(pendingByVehicle);
  const tokensRef = useRef(armedTokensByVehicle);
  useEffect(() => { snapshotsRef.current = snapshotsByVehicle; }, [snapshotsByVehicle]);
  useEffect(() => { draftsRef.current = draftsByVehicle; }, [draftsByVehicle]);
  useEffect(() => { pendingRef.current = pendingByVehicle; }, [pendingByVehicle]);
  useEffect(() => { tokensRef.current = armedTokensByVehicle; }, [armedTokensByVehicle]);

  // ---- Pending bookkeeping ------------------------------------------
  const claimPending = useCallback((ids, kind) => {
    const current = pendingRef.current;
    const skipped = ids.filter((sid) => !!current[sid]);
    const claimed = ids.filter((sid) => !current[sid]);
    if (claimed.length > 0) {
      const merged = { ...current };
      for (const sid of claimed) merged[sid] = kind;
      pendingRef.current = merged;
      setPendingByVehicle(merged);
    }
    return skipped;
  }, []);

  const releasePending = useCallback((ids) => {
    const current = pendingRef.current;
    let any = false;
    const next = { ...current };
    for (const sid of ids) {
      if (Object.prototype.hasOwnProperty.call(next, sid)) {
        delete next[sid];
        any = true;
      }
    }
    if (any) {
      pendingRef.current = next;
      setPendingByVehicle(next);
    }
  }, []);

  const clearVehicleDrafts = useCallback((ids) => {
    if (!ids || ids.length === 0) return;
    const current = draftsRef.current;
    let next = current;
    for (const sid of ids) {
      if (!Object.prototype.hasOwnProperty.call(current, sid)) continue;
      if (next === current) next = { ...current };
      delete next[sid];
    }
    if (next !== current) {
      draftsRef.current = next;
      setDraftsByVehicle(next);
    }
  }, []);

  // ---- Refresh ------------------------------------------------------
  const refreshVehicles = useCallback(async (ids, { force = false } = {}) => {
    if (!ids || ids.length === 0) return { snapshotsBySysId: {}, skippedIds: [] };
    const skipped = claimPending(ids, 'refresh');
    const active = ids.filter((sid) => !skipped.includes(sid));
    // Everything requested is already in flight (e.g. a double-clicked retry) —
    // don't publish a fresh, empty 'downloading' status over the real one.
    if (active.length === 0) return { snapshotsBySysId: {}, skippedIds: skipped };
    const batchGeneration = refreshBatchGenerationRef.current + 1;
    refreshBatchGenerationRef.current = batchGeneration;
    const requestGenerationBySid = {};
    for (const sid of active) {
      const generation = (refreshGenerationRef.current[sid] || 0) + 1;
      refreshGenerationRef.current[sid] = generation;
      requestGenerationBySid[sid] = generation;
    }
    const ownsRefresh = (sid) => ownsFullParamRefresh(
      refreshGenerationRef.current,
      liveSysIdsRef.current,
      sid,
      requestGenerationBySid[sid],
    );
    const byVehicle = {};
    for (const sid of active) {
      byVehicle[sid] = {
        bytesRead: 0,
        sizeEstimate: 0,
        totalBytes: 0,
        done: false,
        error: null,
      };
    }
    setRefreshProgressByVehicle((prev) => ({ ...prev, ...byVehicle }));
    const progress = { uavDone: 0, uavOk: 0, uavTotal: active.length, byVehicle };
    const diagnosticsBySid = {};
    setLastStatus({ kind: 'downloading', op: 'refresh', counts: { ...progress } });

    const outcomes = await fetchFullParamFleet(active, async (sid) => {
      const url = `/api/vehicles/${sid}/parameters${force ? '?refresh=true' : ''}`;
      const diagnostic = diagnosticRequest();
      diagnosticsBySid[sid] = diagnostic;
      emitDiagnostic('client_request_started', {
        request_id: diagnostic.requestId, sys_id: sid,
        phase: force ? 'parameter_refresh' : 'parameter_cache_allowed',
      });
      const r = await fetch(url, { headers: diagnostic.headers });
      diagnostic.statusCode = r.status;
      if (!r.ok) {
        let detail = '';
        try {
          const body = await r.json();
          detail = body?.detail ? `: ${body.detail}` : '';
        } catch {
          // The HTTP status remains useful if the body is not JSON.
        }
        throw new Error(`HTTP ${r.status}${detail}`);
      }
      return r.json();
    }, (sid, data) => {
      if (!ownsRefresh(sid)) return;
      const diagnostic = diagnosticsBySid[sid];
      emitDiagnostic('parameter_snapshot_accepted', {
        request_id: diagnostic.requestId, sys_id: sid, outcome: 'accepted',
        status_code: diagnostic.statusCode, record_count: data?.num_params || 0,
        stale: !!data?.stale,
      });
      const snapshot = normaliseSnapshot(data);
      setSnapshotsByVehicle((prev) => {
        const next = { ...prev, [sid]: snapshot };
        snapshotsRef.current = next;
        return next;
      });
      clearVehicleDrafts([sid]);
    }, (sid, outcome) => {
      if (!ownsRefresh(sid)) return;
      if (!outcome.ok) {
        const diagnostic = diagnosticsBySid[sid];
        emitDiagnostic('parameter_snapshot_rejected', {
          request_id: diagnostic.requestId, sys_id: sid, outcome: 'failure',
          ...(diagnostic.statusCode ? { status_code: diagnostic.statusCode } : {}),
        });
      }
      progress.uavDone += 1;
      if (outcome.ok) progress.uavOk += 1;
      setRefreshProgressByVehicle((prev) => {
        const current = prev[sid] || {};
        const bytesRead = Math.max(0, Number(current.bytesRead) || 0);
        const totalBytes = Math.max(
          0,
          Number(current.totalBytes || (outcome.ok ? bytesRead : 0)) || 0,
        );
        return {
          ...prev,
          [sid]: { ...current, bytesRead, totalBytes, done: true, error: outcome.error },
        };
      });
      setLastStatus((prev) => {
        if (refreshBatchGenerationRef.current !== batchGeneration) return prev;
        if (prev?.kind !== 'downloading') return prev;
        const counts = prev.counts || {};
        const prevByVehicle = counts.byVehicle || {};
        const current = prevByVehicle[sid] || {};
        const bytesRead = Math.max(0, Number(current.bytesRead) || 0);
        const totalBytes = Math.max(
          0,
          Number(current.totalBytes || (outcome.ok ? bytesRead : 0)) || 0,
        );
        return {
          kind: 'downloading',
          op: 'refresh',
          counts: {
            ...counts,
            uavDone: progress.uavDone,
            uavOk: progress.uavOk,
            uavTotal: active.length,
            byVehicle: {
              ...prevByVehicle,
              [sid]: {
                ...current,
                bytesRead,
                totalBytes,
                done: true,
                error: outcome.error,
              },
            },
          },
        };
      });
    });

    const snapshotsBySysId = {};
    for (const outcome of outcomes) {
      if (outcome.ok && ownsRefresh(outcome.sysId)) {
        snapshotsBySysId[outcome.sysId] = normaliseSnapshot(outcome.data);
      }
    }

    releasePending(active.filter(ownsRefresh));
    const okCount = Object.keys(snapshotsBySysId).length;
    setLastStatus((prev) => {
      if (refreshBatchGenerationRef.current !== batchGeneration) return prev;
      return {
        kind: okCount === active.length ? 'ok' : 'partial',
        op: 'refresh',
        counts: {
          ...(prev?.counts || {}),
          uavDone: active.length,
          uavOk: okCount,
          uavTotal: active.length,
        },
        skippedIds: skipped,
      };
    });
    return { snapshotsBySysId, skippedIds: skipped };
  }, [claimPending, releasePending, clearVehicleDrafts]);

  const handleDownloadProgress = useCallback((event) => {
    const sid = Number(event?.sys_id);
    if (!Number.isFinite(sid)) return;
    setRefreshProgressByVehicle((prev) => {
      const current = prev[sid];
      if (!current || current.done) return prev;
      return {
        ...prev,
        [sid]: {
          ...current,
          bytesRead: Math.max(0, Number(event.bytes_read ?? current.bytesRead ?? 0) || 0),
          sizeEstimate: Math.max(0, Number(event.size_estimate ?? current.sizeEstimate ?? 0) || 0),
          totalBytes: Math.max(0, Number(event.total_bytes ?? current.totalBytes ?? 0) || 0),
          done: !!event.done,
        },
      };
    });
    setLastStatus((prev) => {
      if (prev?.kind !== 'downloading') return prev;
      const counts = prev.counts || {};
      const prevByVehicle = counts.byVehicle || {};
      const current = prevByVehicle[sid] || {};
      const bytesRead = Math.max(
        0,
        Number(event.bytes_read ?? current.bytesRead ?? 0) || 0,
      );
      const sizeEstimate = Math.max(
        0,
        Number(event.size_estimate ?? current.sizeEstimate ?? 0) || 0,
      );
      const totalBytes = Math.max(
        0,
        Number(event.total_bytes ?? current.totalBytes ?? 0) || 0,
      );
      return {
        ...prev,
        counts: {
          ...counts,
          byVehicle: {
            ...prevByVehicle,
            [sid]: {
              bytesRead,
              sizeEstimate,
              totalBytes,
              done: event.done ?? current.done ?? false,
              error: event.error ?? current.error ?? null,
            },
          },
        },
      };
    });
  }, []);

  // Per-vehicle write progress fed by `full_param_write_progress` WS events
  // (one per parameter written). Cleared when the batch PUT resolves.
  const handleWriteProgress = useCallback((event) => {
    const sid = Number(event?.sys_id);
    if (!Number.isFinite(sid)) return;
    // Ignore stragglers that arrive after the batch PUT resolved (the write
    // slot is released just before the ring is cleared) — otherwise a late
    // non-final frame would re-create a ring that never clears.
    if (!pendingRef.current[sid]) return;
    const total = Math.max(0, Math.floor(Number(event.total) || 0));
    const written = Math.max(0, Math.min(total, Math.floor(Number(event.written) || 0)));
    setWriteProgressByVehicle((prev) => ({
      ...prev,
      [sid]: { written, total, done: !!event.done },
    }));
  }, []);

  // ---- Edits --------------------------------------------------------
  const setVehicleDraft = useCallback((sysId, name, value) => {
    const prev = draftsRef.current;
    const snap = snapshotsRef.current[sysId];
    const rec = snap?.paramsByName?.[name];
    let next = prev;
    // No-op prune: when the new draft value matches the snapshot value
    // by parameter semantics, drop the draft entry entirely so the cell is
    // no longer marked modified.
    if (rec && draftValueMatchesSubmitted(rec, value, rec.value)) {
      const draft = prev[sysId];
      if (draft && Object.prototype.hasOwnProperty.call(draft, name)) {
        const nextDraft = { ...draft };
        delete nextDraft[name];
        next = { ...prev };
        if (Object.keys(nextDraft).length === 0) {
          delete next[sysId];
        } else {
          next[sysId] = nextDraft;
        }
      }
    } else if ((prev[sysId] || {})[name] !== value) {
      next = {
        ...prev,
        [sysId]: { ...(prev[sysId] || {}), [name]: value },
      };
    }
    if (next !== prev) {
      draftsRef.current = next;
      setDraftsByVehicle(next);
    }
  }, []);

  const resetDraft = useCallback((ids) => {
    clearVehicleDrafts(ids);
  }, [clearVehicleDrafts]);

  // ---- Armed token --------------------------------------------------
  const requestArmedToken = useCallback(async (sysId) => {
    try {
      const r = await fetch(`/api/vehicles/${sysId}/parameters/arm-token`, {
        method: 'POST',
      });
      if (!r.ok) return null;
      const data = await r.json();
      const token = {
        nonce: data.nonce,
        expires_at_unix_s: data.expires_at_unix_s,
      };
      // Update the ref synchronously so an awaited acquire-then-write flow
      // sees the new token (per Codex post-step finding 3 for Step 4).
      tokensRef.current = { ...tokensRef.current, [sysId]: token };
      setArmedTokensByVehicle(tokensRef.current);
      return token;
    } catch {
      return null;
    }
  }, []);

  // ---- Write changed (per-vehicle diff) -----------------------------
  // `opts.changesByVehicle` scopes the write to an explicit per-vehicle change
  // set (e.g. compare-apply) instead of diffing drafts. `opts.onlyNames`
  // (array/Set) further restricts the write to those param names — the Failsafe
  // tab uses it so a per-group "Save" doesn't flush unrelated drafts from the
  // shared store. Either way the submissions are re-validated against the live
  // snapshot (resolveSubmissions). Omitting both writes the full draft diff, so
  // existing callers are unaffected.
  const writeChangedToVehicles = useCallback(async (ids, { changesByVehicle, onlyNames } = {}) => {
    if (!ids || ids.length === 0) {
      return {
        submittedByVehicle: {},
        perVehicleResults: {},
        aggregated: emptyAgg(),
        skippedIds: [],
      };
    }
    const skipped = claimPending(ids, 'write');
    const active = ids.filter((sid) => !skipped.includes(sid));

    // Snapshot the diff BEFORE the round trip — preserves draft semantics
    // if the user keeps editing while the PUT is in flight.
    const submittedByVehicle = resolveSubmissions(active, {
      changesByVehicle,
      draftsByVehicle: draftsRef.current,
      snapshotsByVehicle: snapshotsRef.current,
    });
    // Optional name-scoping (Failsafe per-group Save) — applied after the
    // snapshot gate so it composes with either the draft or explicit-changes path.
    if (onlyNames != null) {
      for (const sid of active) {
        submittedByVehicle[sid] = filterChangesByNames(submittedByVehicle[sid], onlyNames);
      }
    }

    setLastStatus({ kind: 'uploading', counts: { uavTotal: active.length } });

    // Seed per-vehicle write progress at 0% so the header ring shows
    // immediately, before the first backend `full_param_write_progress`
    // event lands. Vehicles with nothing to submit get no ring.
    const seededWrite = {};
    for (const sid of active) {
      const n = (submittedByVehicle[sid] || []).length;
      if (n > 0) seededWrite[sid] = { written: 0, total: n, done: false };
    }
    if (Object.keys(seededWrite).length > 0) {
      setWriteProgressByVehicle((prev) => ({ ...prev, ...seededWrite }));
    }

    const now = Date.now() / 1000;
    const tokens = tokensRef.current;
    const perVehicleResults = {};
    const responses = await Promise.all(active.map(async (sid) => {
      const changes = submittedByVehicle[sid];
      if (!changes || changes.length === 0) return null;
      const body = { changes };
      const diagnostic = diagnosticRequest();
      const tok = tokens[sid];
      if (tok && isArmedTokenLive(tok, now)) body.armed_token = tok.nonce;
      try {
        const r = await fetch(`/api/vehicles/${sid}/parameters`, {
          method: 'PUT',
          headers: { 'Content-Type': 'application/json', ...diagnostic.headers },
          body: JSON.stringify(body),
        });
        if (!r.ok) {
          emitDiagnostic('parameter_write_finished', {
            request_id: diagnostic.requestId, sys_id: sid, outcome: 'rejected',
            status_code: r.status, rejected_count: changes.length,
          });
          return { sid, ok: false };
        }
        const data = await r.json();
        const accepted = Object.values(data.results || {}).filter((item) => item?.ok).length;
        emitDiagnostic('parameter_write_finished', {
          request_id: diagnostic.requestId, sys_id: sid,
          outcome: accepted === changes.length ? 'success' : 'partial',
          accepted_count: accepted, rejected_count: changes.length - accepted,
          stale: !!data.snapshot_stale,
        });
        return { sid, ok: true, data };
      } catch {
        emitDiagnostic('parameter_write_finished', {
          request_id: diagnostic.requestId, sys_id: sid, outcome: 'network_failure',
          rejected_count: changes.length,
        });
        return { sid, ok: false };
      }
    }));

    const staleBySysId = {};
    for (const resp of responses) {
      if (!resp) continue;
      if (resp.ok && resp.data) {
        perVehicleResults[resp.sid] = resp.data.results || {};
        // Stale flag is per-vehicle so one vehicle's stale write doesn't
        // badge unrelated vehicles (per Codex post-step finding 5 for Step 4).
        if (resp.data.snapshot_stale) staleBySysId[resp.sid] = true;
      } else {
        perVehicleResults[resp.sid] = null;
      }
    }

    // Drop the armed tokens — backend invalidated them on first use.
    setArmedTokensByVehicle((prev) => {
      const next = { ...prev };
      let any = false;
      for (const sid of active) {
        if (Object.prototype.hasOwnProperty.call(next, sid)) {
          delete next[sid];
          any = true;
        }
      }
      return any ? next : prev;
    });

    // Merge ack'd values into the snapshot WITHOUT discarding draft edits
    // made during the round-trip. We only baseline the names we submitted.
    setSnapshotsByVehicle((prev) => {
      const next = { ...prev };
      for (const sid of active) {
        if (!liveSysIdsRef.current.has(sid)) continue;
        const submitted = submittedByVehicle[sid];
        if (!submitted || submitted.length === 0) continue;
        const results = perVehicleResults[sid];
        if (!results) continue;
        const snap = prev[sid];
        if (!snap) continue;
        let mutated = false;
        const updatedByName = { ...snap.paramsByName };
        for (const change of submitted) {
          const cell = results[change.name];
          if (!cell || !cell.ok) continue;
          const rec = updatedByName[change.name];
          if (!rec) continue;
          updatedByName[change.name] = { ...rec, value: change.value };
          mutated = true;
        }
        const sidStale = !!staleBySysId[sid];
        if (mutated) {
          next[sid] = {
            ...snap,
            paramsByName: updatedByName,
            records: snap.nameOrder.map((n) => updatedByName[n]),
            stale: sidStale,
          };
        } else if (sidStale) {
          next[sid] = { ...snap, stale: true };
        }
      }
      return next;
    });

    // Drop ack'd entries from draft so the UI no longer shows "pending"
    // for cells that just landed — but only if the user hasn't typed a
    // newer value over the same cell while the PUT was in flight (per
    // Codex post-step finding 2 for Step 4). Unack'd cells stay in draft.
    const currentDrafts = draftsRef.current;
    let nextDrafts = currentDrafts;
    for (const sid of active) {
      if (!liveSysIdsRef.current.has(sid)) continue;
      const submitted = submittedByVehicle[sid];
      if (!submitted || submitted.length === 0) continue;
      const results = perVehicleResults[sid];
      if (!results) continue;
      const draft = { ...(nextDrafts[sid] || {}) };
      let any = false;
      const snap = snapshotsRef.current[sid];
      for (const change of submitted) {
        const cell = results[change.name];
        if (!cell || !cell.ok) continue;
        if (!Object.prototype.hasOwnProperty.call(draft, change.name)) continue;
        // Only delete if the current draft still equals the value the
        // user just submitted; preserve any newer typed value.
        const rec = snap?.paramsByName?.[change.name];
        const stillSubmitted = rec
          ? draftValueMatchesSubmitted(rec, draft[change.name], change.value)
          : Object.is(draft[change.name], change.value);
        if (stillSubmitted) {
          delete draft[change.name];
          any = true;
        }
      }
      if (any) {
        if (nextDrafts === currentDrafts) nextDrafts = { ...currentDrafts };
        if (Object.keys(draft).length === 0) {
          delete nextDrafts[sid];
        } else {
          nextDrafts[sid] = draft;
        }
      }
    }
    if (nextDrafts !== currentDrafts) {
      draftsRef.current = nextDrafts;
      setDraftsByVehicle(nextDrafts);
    }

    releasePending(active);

    // Write finished (success or failure) — drop the transient write rings so
    // each header reverts to its loaded / unsaved state.
    setWriteProgressByVehicle((prev) => {
      const next = { ...prev };
      let any = false;
      for (const sid of active) {
        if (Object.prototype.hasOwnProperty.call(next, sid)) {
          delete next[sid];
          any = true;
        }
      }
      return any ? next : prev;
    });

    const aggregated = aggregateBatchResults(perVehicleResults, submittedByVehicle);
    setLastStatus({
      kind: aggregated.uavFail === 0 && aggregated.uavOk > 0 ? 'ok'
        : aggregated.uavOk === 0 && aggregated.uavFail > 0 ? 'error'
        : 'partial',
      op: 'write',
      counts: aggregated,
      skippedIds: skipped,
    });
    return { submittedByVehicle, perVehicleResults, aggregated, skippedIds: skipped };
  }, [claimPending, releasePending]);

  // ---- Patch (multi-vehicle, e.g. .param load in Step 7) ------------
  const writePatch = useCallback(async (ids, patch) => {
    if (!ids || ids.length === 0 || !patch || Object.keys(patch).length === 0) {
      return {
        submittedByVehicle: {},
        perVehicleResults: {},
        aggregated: emptyAgg(),
        skippedIds: [],
      };
    }
    // Update the ref synchronously so writeChangedToVehicles, which
    // reads draftsRef.current, sees the freshly staged patch — React
    // state updates haven't landed yet at this point. (Per Codex
    // post-step finding 1 for Step 4.)
    const stagedDrafts = { ...draftsRef.current };
    for (const sid of ids) {
      stagedDrafts[sid] = { ...(stagedDrafts[sid] || {}), ...patch };
    }
    draftsRef.current = stagedDrafts;
    setDraftsByVehicle(stagedDrafts);
    return writeChangedToVehicles(ids);
  }, [writeChangedToVehicles]);

  // ---- Read helpers --------------------------------------------------
  const getDraftValue = useCallback((sysId, name) => {
    const d = draftsByVehicle[sysId];
    if (d && Object.prototype.hasOwnProperty.call(d, name)) return d[name];
    const s = snapshotsByVehicle[sysId];
    return s?.paramsByName?.[name]?.value;
  }, [draftsByVehicle, snapshotsByVehicle]);

  const getSnapshotRecord = useCallback((sysId, name) => {
    const s = snapshotsByVehicle[sysId];
    return s?.paramsByName?.[name];
  }, [snapshotsByVehicle]);

  const getConsensus = useCallback((name) => {
    const sysIds = (vehicleList || []).map((v) => v.sys_id);
    return consensusByName(snapshotsByVehicle, sysIds, name);
  }, [snapshotsByVehicle, vehicleList]);

  return useMemo(() => ({
    snapshotsByVehicle,
    draftsByVehicle,
    pendingByVehicle,
    armedTokensByVehicle,
    writeProgressByVehicle,
    refreshProgressByVehicle,
    lastStatus,
    setVehicleDraft,
    resetDraft,
    requestArmedToken,
    refreshVehicles,
    handleDownloadProgress,
    handleWriteProgress,
    writeChangedToVehicles,
    writePatch,
    getDraftValue,
    getSnapshotRecord,
    getConsensus,
  }), [
    snapshotsByVehicle, draftsByVehicle, pendingByVehicle,
    armedTokensByVehicle, writeProgressByVehicle, refreshProgressByVehicle, lastStatus,
    setVehicleDraft, resetDraft, requestArmedToken,
    refreshVehicles, handleDownloadProgress, handleWriteProgress,
    writeChangedToVehicles, writePatch,
    getDraftValue, getSnapshotRecord, getConsensus,
  ]);
}

function emptyAgg() {
  return { uavOk: 0, uavFail: 0, cellOk: 0, cellFail: 0, errors: [] };
}
