import { useCallback, useEffect, useState } from 'react';
import { defaultStream, pickFirstFeed, resolveShownStream } from '../utils/videoStreams';

const streamKeyOf = (fleet, sysId) =>
  defaultStream(fleet.find((e) => e.sysId === sysId)?.streams)?.key ?? null;

/**
 * Which camera feed the panel shows (docs/design/gcs_video_streams.md,
 * section 5 "Camera panel", owner decision D1).
 *
 * - Badges and View camera move the video; the follow selector picks only the
 *   first feed (the followed UAV if it has video, else the first UAV with
 *   video). Later follow changes leave the video alone.
 * - The shown stream disappears: the same UAV's other stream, else "lost"
 *   until it returns or the operator dismisses it. Never another UAV.
 * - `session` changes whenever a fresh connection is wanted (View camera,
 *   Reconnect video), even for the same stream; the panel keys its reader on it.
 *
 * Lives above the panel so the choice survives the panel being hidden (FPV,
 * manual control).
 */
export default function useVideoSelection({ fleet, followSysId }) {
  const [selection, setSelection] = useState(null);
  const [autoPicked, setAutoPicked] = useState(false);
  const [collapsed, setCollapsed] = useState(false);

  const anyStreams = fleet.some((e) => e.streams.length > 0);
  const shown = resolveShownStream(selection, fleet);

  // First feed only (D1).
  useEffect(() => {
    if (selection || autoPicked || !anyStreams) return;
    const sysId = pickFirstFeed(followSysId, fleet);
    setSelection({ sysId, key: streamKeyOf(fleet, sysId), nonce: 0 });
    setAutoPicked(true);
  }, [selection, autoPicked, anyStreams, followSysId, fleet]);

  // The panel went away (nothing advertised, nothing pending): the next feed
  // to appear is a first feed again.
  useEffect(() => {
    if (!anyStreams && !selection && autoPicked) setAutoPicked(false);
  }, [anyStreams, selection, autoPicked]);

  // Keep the same-UAV fallback, so the original stream coming back later does
  // not switch the picture a second time.
  const fallbackKey = shown.kind === 'stream' && shown.switched ? shown.stream.key : null;
  useEffect(() => {
    if (fallbackKey) setSelection((s) => (s ? { ...s, key: fallbackKey } : s));
  }, [fallbackKey]);

  const open = useCallback((sysId, fresh) => {
    setAutoPicked(true);
    setCollapsed(false);
    setSelection((s) => {
      if (!fresh && s?.sysId === sysId) return s;
      return { sysId, key: streamKeyOf(fleet, sysId), nonce: (s?.nonce ?? 0) + 1 };
    });
  }, [fleet]);

  /** Badge: show that UAV's default stream; the selected badge is a no-op. */
  const selectUav = useCallback((sysId) => open(sysId, false), [open]);
  /** View camera on a confirm card: always a fresh connection to that UAV. */
  const viewCamera = useCallback((sysId) => open(sysId, true), [open]);
  const reconnect = useCallback(() => {
    setSelection((s) => (s ? { ...s, nonce: s.nonce + 1 } : s));
  }, []);
  const dismiss = useCallback(() => setSelection(null), []);

  return {
    selection,
    shown,
    session: selection ? `${selection.sysId}:${shown.stream?.key ?? '-'}:${selection.nonce}` : null,
    // Shown while any UAV advertises a stream or a lost feed is still on screen.
    hasPanel: anyStreams || shown.kind === 'lost',
    collapsed,
    setCollapsed,
    selectUav,
    viewCamera,
    reconnect,
    dismiss,
  };
}
