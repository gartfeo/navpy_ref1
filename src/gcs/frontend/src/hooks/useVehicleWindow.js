import { useCallback, useEffect, useState } from 'react';
import { defaultStream, pickFirstFeed, pickFirstUav, resolveShownStream } from '../utils/videoStreams';

/** Browser storage key: only "minimized" or "normal" is saved, never the source or the swap. */
export const WINDOW_STORAGE_KEY = 'gcs-vehicle-window';

export const SOURCE = { CAMERA: 'camera', SYNTHETIC: 'synthetic' };

const streamKeyOf = (fleet, sysId) =>
  defaultStream(fleet.find((e) => e.sysId === sysId)?.streams)?.key ?? null;

function readMinimized() {
  try {
    return globalThis.localStorage?.getItem(WINDOW_STORAGE_KEY) === 'minimized';
  } catch {
    return false;
  }
}

function saveMinimized(minimized) {
  try {
    globalThis.localStorage?.setItem(WINDOW_STORAGE_KEY, minimized ? 'minimized' : 'normal');
  } catch {
    // Storage unavailable: the window still works, it just opens normal next time.
  }
}

/**
 * State of the monitor view's vehicle window (docs/design/gcs_video_streams.md,
 * section 5 "Vehicle window"): which UAV, which source, and which size.
 *
 * - Badges and View camera pick the UAV for both sources; the follow selector
 *   only feeds the first pick (D1). The first pick stays automatic until the
 *   operator taps a badge, the switch, View camera or Dismiss; while automatic and on
 *   "No camera", it moves to the first UAV that starts advertising. An empty
 *   fleet re-arms it.
 * - The shown stream disappears: the same UAV's other stream, else "lost"
 *   until it returns or the operator dismisses it to "No camera". Never
 *   another UAV.
 * - Synthetic only by a tap on the switch; never saved. A synthetic failure
 *   goes back to Camera with a notice.
 * - Minimized is saved; maximized (the map swap) is not, and ends when the
 *   window is not `active` (RC, leaving the monitor view) or the fleet empties.
 * - `session` changes whenever a fresh camera connection is wanted (View
 *   camera, Reconnect video, back to Camera); the window keys its reader on it.
 *
 * @param {object} p
 * @param {Array} p.fleet - useVideoFleet entries.
 * @param {number|null} p.followSysId
 * @param {boolean} p.active - monitor view and not RC.
 */
export default function useVehicleWindow({ fleet, followSysId, active }) {
  const [selection, setSelection] = useState(null);
  const [auto, setAuto] = useState(true);
  const [source, setSource] = useState(SOURCE.CAMERA);
  const [minimized, setMinimizedState] = useState(readMinimized);
  const [maximized, setMaximized] = useState(false);
  const [notice, setNotice] = useState(null);

  const empty = fleet.length === 0;
  const shown = resolveShownStream(selection, fleet);

  // First pick, and the re-arm on an empty fleet.
  useEffect(() => {
    if (empty) {
      setSelection(null);
      setAuto(true);
      setSource(SOURCE.CAMERA);
      setMaximized(false);
      setNotice(null);
      return;
    }
    if (selection) return;
    const sysId = pickFirstUav(followSysId, fleet);
    setSelection({ sysId, key: streamKeyOf(fleet, sysId), nonce: 0 });
  }, [empty, selection, followSysId, fleet]);

  // Automatic and on "No camera": follow the first UAV that starts advertising.
  const autoMoveTo = auto && shown.kind === 'noCamera' && source === SOURCE.CAMERA
    ? pickFirstFeed(followSysId, fleet) : null;
  useEffect(() => {
    if (autoMoveTo == null) return;
    setSelection((s) => ({ sysId: autoMoveTo, key: streamKeyOf(fleet, autoMoveTo), nonce: (s?.nonce ?? 0) + 1 }));
  }, [autoMoveTo, fleet]);

  // Keep the same-UAV fallback (or the stream that appeared on "No camera"),
  // so the original stream coming back later does not switch the picture again.
  const fallbackKey = shown.kind === 'stream' && shown.switched ? shown.stream.key : null;
  useEffect(() => {
    if (fallbackKey) setSelection((s) => (s ? { ...s, key: fallbackKey } : s));
  }, [fallbackKey]);

  // RC entry or leaving the monitor view: swap back, Camera.
  useEffect(() => {
    if (active) return;
    setMaximized(false);
    setSource(SOURCE.CAMERA);
  }, [active]);

  const setMinimized = useCallback((value) => {
    setMinimizedState(value);
    saveMinimized(value);
  }, []);

  const operatorPick = useCallback(() => {
    setAuto(false);
    setNotice(null);
  }, []);

  /** Badge: that UAV in the current source; the selected badge is a no-op. */
  const selectUav = useCallback((sysId) => {
    operatorPick();
    setSelection((s) => (s?.sysId === sysId ? s
      : { sysId, key: streamKeyOf(fleet, sysId), nonce: (s?.nonce ?? 0) + 1 }));
  }, [fleet, operatorPick]);

  /** View camera on a confirm card: that UAV, Camera, a fresh connection, a minimized window restored. */
  const viewCamera = useCallback((sysId) => {
    operatorPick();
    setSource(SOURCE.CAMERA);
    setMinimized(false);
    setSelection((s) => ({ sysId, key: streamKeyOf(fleet, sysId), nonce: (s?.nonce ?? 0) + 1 }));
  }, [fleet, operatorPick, setMinimized]);

  /** The Camera / Synthetic switch. Back to Camera is a fresh connection. */
  const chooseSource = useCallback((next) => {
    operatorPick();
    if (next === source) return;
    setSource(next);
    if (next === SOURCE.CAMERA) setSelection((s) => (s ? { ...s, nonce: s.nonce + 1 } : s));
  }, [source, operatorPick]);

  const reconnect = useCallback(() => {
    setSelection((s) => (s ? { ...s, nonce: s.nonce + 1 } : s));
  }, []);

  /** Dismiss a lost feed: "No camera" for the same UAV. An operator tap, so
   * it also ends the automatic pick: never a move to another UAV after it. */
  const dismiss = useCallback(() => {
    setAuto(false);
    setSelection((s) => (s ? { ...s, key: null } : s));
  }, []);

  const minimize = useCallback(() => {
    setMaximized(false);
    setMinimized(true);
  }, [setMinimized]);
  const restore = useCallback(() => setMinimized(false), [setMinimized]);
  const toggleMaximized = useCallback(() => setMaximized((m) => !m), []);
  const unswap = useCallback(() => setMaximized(false), []);

  /** The synthetic view failed: back to Camera with a notice. Retry is a new tap. */
  const syntheticFailed = useCallback(() => {
    setSource(SOURCE.CAMERA);
    setNotice('syntheticUnavailable');
    setSelection((s) => (s ? { ...s, nonce: s.nonce + 1 } : s));
  }, []);

  const sysId = selection?.sysId ?? null;
  return {
    sysId,
    shown,
    source,
    minimized,
    /** The map and window are swapped. */
    swapped: active && !empty && maximized && !minimized,
    notice,
    session: selection ? `${selection.sysId}:${shown.stream?.key ?? '-'}:${selection.nonce}` : null,
    /** "Viewing" on a confirm card: this UAV's camera is on screen. */
    viewing: (id) => active && !minimized && source === SOURCE.CAMERA && sysId === id && shown.kind === 'stream',
    selectUav,
    viewCamera,
    chooseSource,
    reconnect,
    dismiss,
    minimize,
    restore,
    toggleMaximized,
    unswap,
    syntheticFailed,
  };
}
