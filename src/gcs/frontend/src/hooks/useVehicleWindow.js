import { useCallback, useEffect, useRef, useState } from 'react';
import { defaultStream, hasVideo, resolveShownStream } from '../utils/videoStreams';

export const SOURCE = { CAMERA: 'camera', SYNTHETIC: 'synthetic' };

const entryOf = (fleet, sysId) => fleet.find((e) => e.sysId === sysId);

/**
 * A pick of `sysId`: the source its icon gives (Camera when it advertises a
 * stream, else Synthetic), its default stream and a fresh session.
 */
function pickOf(fleet, sysId, previous) {
  const entry = entryOf(fleet, sysId);
  return {
    sysId,
    key: defaultStream(entry?.streams)?.key ?? null,
    source: hasVideo(entry) ? SOURCE.CAMERA : SOURCE.SYNTHETIC,
    nonce: (previous?.nonce ?? 0) + 1,
  };
}

/**
 * State of the monitor view's vehicle window (docs/design/gcs_video_streams.md,
 * section 5 "Vehicle window"): which UAV, which source, and the swap.
 *
 * - No selection is the picker. Only the operator picks: the UAV buttons and
 *   View camera. A pick opens what the UAV's icon shows (Camera with a stream,
 *   else Synthetic); View camera always opens Camera; picking the selected UAV
 *   again does nothing. Close goes back to the picker; an empty fleet too.
 * - Once picked, the source changes only by the switch, Dismiss (to
 *   Synthetic) or a synthetic failure (to Camera, with a notice). A stream
 *   appearing or going away only changes what Camera shows: the same UAV's
 *   other stream, else "lost". Never another UAV.
 * - Not `active` (RC, leaving the monitor view): swap back; on return the
 *   window comes back on the same UAV with the source its icon gives.
 * - Nothing is saved: a reload opens the picker, unswapped.
 * - `session` changes whenever a fresh camera connection is wanted (a pick,
 *   View camera, Reconnect video, back to Camera); the window keys its reader on it.
 *
 * @param {object} p
 * @param {Array} p.fleet - useVideoFleet entries.
 * @param {boolean} p.active - monitor view and not RC.
 */
export default function useVehicleWindow({ fleet, active }) {
  const [selection, setSelection] = useState(null);
  const [maximized, setMaximized] = useState(false);
  const [notice, setNotice] = useState(null);

  const empty = fleet.length === 0;
  const shown = resolveShownStream(selection, fleet);

  // An empty fleet: the window goes, and the next UAV brings the picker.
  useEffect(() => {
    if (!empty) return;
    setSelection(null);
    setMaximized(false);
    setNotice(null);
  }, [empty]);

  // Keep the same-UAV fallback (or the stream that appeared on "No camera"),
  // so the original stream coming back later does not switch the picture again.
  const fallbackKey = shown.kind === 'stream' && shown.switched ? shown.stream.key : null;
  useEffect(() => {
    if (fallbackKey) setSelection((s) => (s ? { ...s, key: fallbackKey } : s));
  }, [fallbackKey]);

  // RC entry or leaving the monitor view: swap back. Back again: the same UAV
  // with the source its icon gives now.
  const fleetRef = useRef(fleet);
  fleetRef.current = fleet;
  const wasActiveRef = useRef(active);
  useEffect(() => {
    const wasActive = wasActiveRef.current;
    wasActiveRef.current = active;
    if (!active) {
      setMaximized(false);
      return;
    }
    if (wasActive) return;
    setNotice(null);
    setSelection((s) => (s ? pickOf(fleetRef.current, s.sysId, s) : s));
  }, [active]);

  const selectedSysId = selection?.sysId ?? null;

  /** A UAV button: that UAV with the source its icon gives; the selected one is a no-op. */
  const selectUav = useCallback((sysId) => {
    if (sysId === selectedSysId) return;
    setNotice(null);
    setSelection((s) => pickOf(fleet, sysId, s));
  }, [fleet, selectedSysId]);

  /** View camera on a confirm card: that UAV, Camera, a fresh connection, also from the picker. */
  const viewCamera = useCallback((sysId) => {
    setNotice(null);
    setSelection((s) => ({ ...pickOf(fleet, sysId, s), source: SOURCE.CAMERA }));
  }, [fleet]);

  /** The Camera / Synthetic switch. Back to Camera is a fresh connection. */
  const chooseSource = useCallback((next) => {
    if (!selection || next === selection.source) return;
    setNotice(null);
    setSelection((s) => (s ? {
      ...s, source: next, nonce: next === SOURCE.CAMERA ? s.nonce + 1 : s.nonce,
    } : s));
  }, [selection]);

  const reconnect = useCallback(() => {
    setSelection((s) => (s ? { ...s, nonce: s.nonce + 1 } : s));
  }, []);

  /** Dismiss a lost feed: Synthetic for the same UAV, as a pick of a map-icon UAV would. */
  const dismiss = useCallback(() => {
    setNotice(null);
    setSelection((s) => (s ? { ...s, key: null, source: SOURCE.SYNTHETIC } : s));
  }, []);

  /** Close: swap back, stop the picture and show the picker. */
  const close = useCallback(() => {
    setMaximized(false);
    setNotice(null);
    setSelection(null);
  }, []);

  /** Maximize is offered only while viewing, not in the picker. */
  const toggleMaximized = useCallback(() => {
    if (selection) setMaximized((m) => !m);
  }, [selection]);
  const unswap = useCallback(() => setMaximized(false), []);

  /** The synthetic view failed: back to Camera with a notice. Retry is a new tap. */
  const syntheticFailed = useCallback(() => {
    setNotice('syntheticUnavailable');
    setSelection((s) => (s ? { ...s, source: SOURCE.CAMERA, nonce: s.nonce + 1 } : s));
  }, []);

  const source = selection?.source ?? SOURCE.CAMERA;
  return {
    /** The window's UAV; null is the picker. */
    sysId: selectedSysId,
    shown,
    source,
    /** The map and window are swapped. */
    swapped: active && !empty && maximized && selection != null,
    notice,
    session: selection ? `${selection.sysId}:${shown.stream?.key ?? '-'}:${selection.nonce}` : null,
    /** "Viewing" on a confirm card: this UAV's camera is on screen. */
    viewing: (id) => active && source === SOURCE.CAMERA && selectedSysId === id && shown.kind === 'stream',
    selectUav,
    viewCamera,
    chooseSource,
    reconnect,
    dismiss,
    close,
    toggleMaximized,
    unswap,
    syntheticFailed,
  };
}
