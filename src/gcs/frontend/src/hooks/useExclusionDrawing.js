import { useCallback, useEffect, useRef, useState } from 'react';
import { nextExclusionClickState, activeRingAfterRemove, revokeInsertedVertex } from '../utils/exclusionDraw';

// Max gap between the two clicks of a double-click. Cesium delivers both
// LEFT_CLICKs before LEFT_DOUBLE_CLICK, so the finish handler revokes an
// active-ring insert made this recently — it was the double-click's first
// half, not a deliberate reshape.
const DOUBLE_CLICK_REVOKE_MS = 600;

/**
 * Keep-out (exclusion) polygon drawing — same logic as the search zone:
 * the ring CLOSES automatically at 3 vertices (no double-click needed) and
 * later clicks insert into its nearest edge. Double-click just ends editing of
 * the current ring (the next click starts a new one); toggling the mode off
 * ends editing and discards any incomplete (<3 pts) draft.
 *
 * Owns the in-progress draft and which committed ring is still click-editable;
 * committed rings live in the mission's `exclusionPolygons`. The placement-mode
 * flag itself lives in usePlacementModes (mutually exclusive with the other
 * draw/place modes); this hook just reacts to it.
 */
export default function useExclusionDrawing({
  placingExclusion, exclusionPolygons, setExclusionPolygons,
}) {
  const [draftExclusion, setDraftExclusion] = useState([]);
  const draftRef = useRef([]);
  const activeRingRef = useRef(null);
  // Last active-ring insertion — revoked by handleFinishExclusion when it was
  // the first click of the finishing double-click. {ring, index, latlon, at}.
  const lastInsertRef = useRef(null);
  // Ring state lives in the mission; keep a ref fresh for synchronous reads
  // inside click handlers (state updates land next render).
  const ringsRef = useRef(exclusionPolygons || []);
  ringsRef.current = exclusionPolygons || [];

  const updateDraft = useCallback((next) => {
    draftRef.current = next;
    setDraftExclusion(next);
  }, []);

  // Leaving the mode ends ring editing and discards any incomplete draft
  // (a draft is always < 3 points now — a 3rd point commits it as a ring).
  useEffect(() => {
    if (!placingExclusion) {
      activeRingRef.current = null;
      lastInsertRef.current = null;
      if (draftRef.current.length > 0) updateDraft([]);
    }
  }, [placingExclusion, updateDraft]);

  const handlePlaceExclusionVertex = useCallback((latlon) => {
    const next = nextExclusionClickState({
      draft: draftRef.current,
      rings: ringsRef.current,
      activeRing: activeRingRef.current,
    }, latlon);
    activeRingRef.current = next.activeRing;
    const noop = next.draft === draftRef.current && next.rings === ringsRef.current;
    if (next.inserted) {
      lastInsertRef.current = { ...next.inserted, latlon: { lat: latlon.lat, lon: latlon.lon }, at: Date.now() };
    } else if (!noop) {
      lastInsertRef.current = null; // a new deliberate action supersedes the record
    }
    // no-op click (coincident-dropped): PRESERVE the record — a real
    // double-click delivers click#1 (insert, recorded), click#2 (coincident,
    // this branch), then LEFT_DOUBLE_CLICK, whose finish handler must still be
    // able to revoke click#1's insert. Clearing here broke revocation for
    // genuine double-clicks (caught in live browser testing).
    if (next.draft !== draftRef.current) updateDraft(next.draft);
    if (next.rings !== ringsRef.current) {
      ringsRef.current = next.rings;
      setExclusionPolygons(next.rings);
    }
  }, [setExclusionPolygons, updateDraft]);

  // Double-click: stop shaping the current ring; the next click starts a new
  // one. Cesium fired both LEFT_CLICKs first, so an active-ring vertex the
  // first click just inserted is a stray — revoke it.
  const handleFinishExclusion = useCallback(() => {
    const ins = lastInsertRef.current;
    if (ins && Date.now() - ins.at < DOUBLE_CLICK_REVOKE_MS) {
      const next = revokeInsertedVertex(ringsRef.current, ins);
      if (next !== ringsRef.current) {
        ringsRef.current = next;
        setExclusionPolygons(next);
      }
    }
    lastInsertRef.current = null;
    activeRingRef.current = null;
    updateDraft([]);
  }, [setExclusionPolygons, updateDraft]);

  const handleRemoveExclusion = useCallback((index) => {
    activeRingRef.current = activeRingAfterRemove(activeRingRef.current, index);
    lastInsertRef.current = null;
    setExclusionPolygons((prev) => (prev || []).filter((_, i) => i !== index));
  }, [setExclusionPolygons]);

  const handleClearExclusions = useCallback(() => {
    activeRingRef.current = null;
    lastInsertRef.current = null;
    setExclusionPolygons([]);
    updateDraft([]);
  }, [setExclusionPolygons, updateDraft]);

  // ---- Zone-style ring editing (drag vertex / midpoint insert / delete) ----
  // Same gestures as the search zone and the fence; operate on any committed
  // ring by index. ringsRef is kept in sync so rapid drag moves read fresh.
  const _setRings = useCallback((next) => {
    ringsRef.current = next;
    setExclusionPolygons(next);
  }, [setExclusionPolygons]);

  const handleExclusionVertexDrag = useCallback((ringIdx, index, latlon) => {
    const rings = ringsRef.current;
    const ring = rings[ringIdx];
    if (!ring || index < 0 || index >= ring.length || !latlon) return;
    _setRings(rings.map((r, i) => (
      i === ringIdx ? r.map((v, j) => (j === index ? { lat: latlon.lat, lon: latlon.lon } : v)) : r
    )));
  }, [_setRings]);

  const handleExclusionMidpointInsert = useCallback((ringIdx, afterIndex, latlon) => {
    const rings = ringsRef.current;
    const ring = rings[ringIdx];
    if (!ring || afterIndex < 0 || afterIndex >= ring.length || !latlon) return;
    _setRings(rings.map((r, i) => (
      i === ringIdx
        ? [...r.slice(0, afterIndex + 1), { lat: latlon.lat, lon: latlon.lon }, ...r.slice(afterIndex + 1)]
        : r
    )));
  }, [_setRings]);

  const handleExclusionVertexDelete = useCallback((ringIdx, index) => {
    const rings = ringsRef.current;
    const ring = rings[ringIdx];
    // Keep at least 3 vertices — same floor as the zone polygon.
    if (!ring || ring.length <= 3 || index < 0 || index >= ring.length) return;
    _setRings(rings.map((r, i) => (
      i === ringIdx ? r.filter((_, j) => j !== index) : r
    )));
  }, [_setRings]);

  return {
    draftExclusion,
    handlePlaceExclusionVertex,
    handleFinishExclusion,
    handleRemoveExclusion,
    handleClearExclusions,
    handleExclusionVertexDrag,
    handleExclusionMidpointInsert,
    handleExclusionVertexDelete,
  };
}
