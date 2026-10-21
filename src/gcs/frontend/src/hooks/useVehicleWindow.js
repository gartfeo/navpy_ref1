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
 * section 5 "Vehicle window", v12 and v12.1): open or closed, which UAV, which
 * source, and the swap.
 *
 * - The video icon in the map's action bar opens and closes the window; it
 *   starts closed. Closing swaps back and stops the picture (the window
 *   unmounts); the UAV and source are kept, and opening again plays them with
 *   a fresh connection. View camera opens it. An empty fleet closes it and
 *   clears the UAV, so a returning fleet starts closed with no UAV picked.
 * - No selection is the picker. Only the operator picks: the picker, the UAV
 *   list and View camera. A pick opens what the UAV's icon shows (Camera with
 *   a stream, else Synthetic); View camera opens Camera and is refused for a
 *   UAV without a stream; picking the selected UAV again does nothing.
 * - Only the operator changes the source: the switch or Dismiss (to
 *   Synthetic). Camera is available only while the selected UAV advertises a
 *   stream; choosing it otherwise changes nothing and says why. A
 *   synthetic failure keeps Synthetic and latches until Retry, a source change
 *   or a pick. A stream appearing or going away only changes what Camera
 *   shows: the same UAV's other stream, else "lost". Never another UAV.
 * - Not `active` (RC, leaving the monitor view): swap back; on return the
 *   window comes back on the same UAV and source, with a fresh connection.
 * - Open or closed is kept across RC and view changes in the session, never
 *   saved: a reload starts closed, with no UAV picked.
 * - `session` changes whenever a fresh camera connection is wanted (a pick,
 *   View camera, opening, Reconnect video, back to Camera); the window keys its
 *   reader on it.
 *
 * @param {object} p
 * @param {Array} p.fleet - useVideoFleet entries.
 * @param {boolean} p.active - monitor view and not RC.
 */
export default function useVehicleWindow({ fleet, active }) {
  const [open, setOpen] = useState(false);
  const [selection, setSelection] = useState(null);
  const [maximized, setMaximized] = useState(false);
  const [cameraRefused, setCameraRefused] = useState(false);

  const empty = fleet.length === 0;
  const shown = resolveShownStream(selection, fleet);

  // An empty fleet: the window closes, and the next fleet starts closed on the picker.
  useEffect(() => {
    if (!empty) return;
    setOpen(false);
    setSelection(null);
    setMaximized(false);
    setCameraRefused(false);
  }, [empty]);

  // Keep the same-UAV fallback (or the stream that appeared on "No camera"),
  // so the original stream coming back later does not switch the picture again.
  const fallbackKey = shown.kind === 'stream' && shown.switched ? shown.stream.key : null;
  useEffect(() => {
    if (fallbackKey) setSelection((s) => (s ? { ...s, key: fallbackKey } : s));
  }, [fallbackKey]);

  // RC entry or leaving the monitor view: swap back. Back again: the same UAV
  // and source, with a fresh connection.
  const wasActiveRef = useRef(active);
  useEffect(() => {
    const wasActive = wasActiveRef.current;
    wasActiveRef.current = active;
    if (!active) {
      setMaximized(false);
      return;
    }
    if (wasActive) return;
    setSelection((s) => (s ? { ...s, nonce: s.nonce + 1 } : s));
  }, [active]);

  const selectedSysId = selection?.sysId ?? null;
  const cameraAvailable = selection != null && hasVideo(entryOf(fleet, selectedSysId));

  /** A UAV button: that UAV with the source its icon gives; the selected one is a no-op. */
  const selectUav = useCallback((sysId) => {
    if (sysId === selectedSysId) return;
    setCameraRefused(false);
    setSelection((s) => pickOf(fleet, sysId, s));
  }, [fleet, selectedSysId]);

  /** View camera on a confirm card: opens the window on that UAV, Camera, a fresh connection. */
  const viewCamera = useCallback((sysId) => {
    if (!hasVideo(entryOf(fleet, sysId))) return;
    setOpen(true);
    setCameraRefused(false);
    setSelection((s) => ({ ...pickOf(fleet, sysId, s), source: SOURCE.CAMERA }));
  }, [fleet]);

  /**
   * The Camera / Synthetic switch. Back to Camera is a fresh connection.
   * Camera without a stream changes nothing and says why.
   */
  const chooseSource = useCallback((next) => {
    if (!selection) return;
    // Before the same-source check: Camera still pressed after its last stream went says why too.
    if (next === SOURCE.CAMERA && !cameraAvailable) {
      setCameraRefused(true);
      return;
    }
    if (next === selection.source) return;
    setCameraRefused(false);
    setSelection((s) => (s ? {
      ...s, source: next, syntheticFailed: false, nonce: next === SOURCE.CAMERA ? s.nonce + 1 : s.nonce,
    } : s));
  }, [selection, cameraAvailable]);

  const reconnect = useCallback(() => {
    setSelection((s) => (s ? { ...s, nonce: s.nonce + 1 } : s));
  }, []);

  /** Dismiss a lost feed: Synthetic for the same UAV, as a pick of a map-icon UAV would. */
  const dismiss = useCallback(() => {
    setCameraRefused(false);
    setSelection((s) => (s ? { ...s, key: null, source: SOURCE.SYNTHETIC } : s));
  }, []);

  /**
   * The video icon. Closing swaps back first; the window then unmounts, which
   * stops the reader and the synthetic widget. Opening is a fresh connection.
   */
  const toggleOpen = useCallback(() => {
    if (open) {
      setMaximized(false);
      setOpen(false);
      return;
    }
    setSelection((s) => (s ? { ...s, nonce: s.nonce + 1 } : s));
    setOpen(true);
  }, [open]);

  /** Maximize is offered only while viewing, not in the picker. */
  const toggleMaximized = useCallback(() => {
    if (open && selection) setMaximized((m) => !m);
  }, [open, selection]);
  const unswap = useCallback(() => setMaximized(false), []);

  /** The synthetic view failed: Synthetic stays selected, the failure latched until Retry. */
  const reportSyntheticFailure = useCallback(() => {
    setSelection((s) => (s ? { ...s, syntheticFailed: true } : s));
  }, []);
  const retrySynthetic = useCallback(() => {
    setSelection((s) => (s ? { ...s, syntheticFailed: false } : s));
  }, []);

  const source = selection?.source ?? SOURCE.CAMERA;
  return {
    /** The window is open (the video icon pressed). */
    open,
    /** The window's UAV; null is the picker. */
    sysId: selectedSysId,
    shown,
    source,
    /** The selected UAV advertises a stream, so Camera can be chosen. */
    cameraAvailable,
    /** The synthetic view failed and waits for Retry. */
    syntheticFailed: selection?.syntheticFailed === true,
    /** The map and window are swapped. */
    swapped: active && !empty && open && maximized && selection != null,
    /** Camera was chosen without a stream: the window says why. Cleared by the next source change or pick. */
    cameraRefused,
    session: selection ? `${selection.sysId}:${shown.stream?.key ?? '-'}:${selection.nonce}` : null,
    /** "Viewing" on a confirm card: this UAV's camera is on screen (App adds the picture room). */
    viewing: (id) => active && open && source === SOURCE.CAMERA && selectedSysId === id && shown.kind === 'stream',
    selectUav,
    viewCamera,
    chooseSource,
    reconnect,
    dismiss,
    toggleOpen,
    toggleMaximized,
    unswap,
    reportSyntheticFailure,
    retrySynthetic,
  };
}
