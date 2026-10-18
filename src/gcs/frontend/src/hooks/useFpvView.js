import { useCallback, useEffect, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { PHASES } from './useMissionState';
import { pickFpvTarget } from '../utils/fpvView';

/**
 * FPV view mode and map-camera ownership (docs/design/gcs_video_streams.md,
 * section 5 "FPV").
 *
 * View mode is separate from control mode: the FPV toggle never enables RC,
 * and RC keeps its own body FPV of the RC target. The follow selector moves the
 * map and FPV; it always names the UAV FPV shows.
 *
 * Map camera: one saved camera, taken when the view first leaves the free map
 * for FPV or RC, restored when the view comes back to the free map. Switching
 * the FPV UAV does not overwrite it. Without a saved camera, coming back to
 * Free from a chase view centers the map, as before.
 *
 * | Event                         | Result                                        |
 * |-------------------------------|-----------------------------------------------|
 * | FPV off                       | chase of the same UAV; panel back             |
 * | Follow set to Free (FPV on)   | FPV off, saved map camera restored            |
 * | Followed UAV removed          | as Free                                       |
 * | View camera (FPV on)          | FPV off as above (caller then picks the feed) |
 * | RC entry (FPV on or off)      | FPV off; RC's own FPV of the RC target        |
 * | RC exit                       | Free map with the saved map camera            |
 *
 * @param {object} p
 * @param {string} p.phase
 * @param {number|null} p.followSysId
 * @param {(sysId: number|null) => void} p.setFollowSysId
 * @param {Array} p.fleet - useVideoFleet entries.
 * @param {number|null} p.panelSysId - the camera panel's UAV.
 * @param {boolean} p.rcEnabled
 * @param {number|null} p.rcTarget
 * @param {{ current: { capture: () => object, restore: (cam: object) => void } | null }} p.mapCameraRef
 * @param {() => void} p.centerView
 * @param {(text: string) => void} p.notify
 */
export default function useFpvView({
  phase, followSysId, setFollowSysId, fleet, panelSysId,
  rcEnabled, rcTarget, mapCameraRef, centerView, notify,
}) {
  const { t } = useTranslation();
  const [fpvOn, setFpvOn] = useState(false);

  const labelOf = useCallback(
    (sysId) => fleet.find((e) => e.sysId === sysId)?.label ?? `UAV ${sysId}`,
    [fleet],
  );
  const notifyIneligible = useCallback(
    (sysId) => notify(t('fpvView.ineligible', { name: labelOf(sysId) })),
    [notify, t, labelOf],
  );

  const toggleFpv = useCallback(() => {
    if (fpvOn) {
      setFpvOn(false);
      return;
    }
    const target = pickFpvTarget({ followSysId, panelSysId, fleet });
    if (target.none) {
      notify(t('fpvView.noEligible'));
    } else if (target.ineligible != null) {
      // The current view stays as it is.
      notifyIneligible(target.ineligible);
    } else {
      setFollowSysId(target.sysId);
      setFpvOn(true);
    }
  }, [fpvOn, followSysId, panelSysId, fleet, notify, notifyIneligible, setFollowSysId, t]);

  /** The follow selector. In FPV an ineligible UAV is refused, so the selector keeps naming the FPV UAV. */
  const changeFollow = useCallback((sysId) => {
    if (fpvOn && sysId != null && !fleet.find((e) => e.sysId === sysId)?.fpvEligible) {
      notifyIneligible(sysId);
      return;
    }
    setFollowSysId(sysId);
  }, [fpvOn, fleet, notifyIneligible, setFollowSysId]);

  const leaveFpv = useCallback(() => setFpvOn(false), []);

  // Free (selected, or the followed UAV removed), RC entry, or leaving the
  // monitor view all end FPV.
  const mustEnd = fpvOn && (followSysId == null || rcEnabled || phase !== PHASES.MONITOR);
  useEffect(() => {
    if (mustEnd) setFpvOn(false);
  }, [mustEnd]);

  const viewFpv = fpvOn && !mustEnd;

  // Map camera ownership. Runs after CesiumMap's follow hook (child effects run
  // first), so a restore or fly-to never fights a still-locked chase camera.
  // The free-map camera is saved whenever the view leaves the free map (chase
  // too, since a chase may become FPV); it is restored on return only if FPV
  // or RC took the camera meanwhile. A plain chase keeps today's auto-center.
  const savedCameraRef = useRef(null);
  const cameraTakenRef = useRef(false);
  const prevRef = useRef({ follow: followSysId, fpv: viewFpv, rc: rcEnabled });
  useEffect(() => {
    const prev = prevRef.current;
    prevRef.current = { follow: followSysId, fpv: viewFpv, rc: rcEnabled };
    const wasFreeMap = prev.follow == null && !prev.fpv && !prev.rc;
    const isFreeMap = followSysId == null && !viewFpv && !rcEnabled;

    if (wasFreeMap && !isFreeMap) {
      savedCameraRef.current = mapCameraRef.current?.capture() ?? null;
      cameraTakenRef.current = false;
    }
    if (viewFpv || rcEnabled) cameraTakenRef.current = true;
    if (isFreeMap && !wasFreeMap) {
      const saved = cameraTakenRef.current ? savedCameraRef.current : null;
      savedCameraRef.current = null;
      cameraTakenRef.current = false;
      if (saved) mapCameraRef.current?.restore(saved);
      else if (phase === PHASES.MONITOR && prev.follow != null) centerView();
    }
  }, [followSysId, viewFpv, rcEnabled, phase, mapCameraRef, centerView]);

  const rcFpv = rcEnabled && rcTarget != null;
  let fpvSysId = null;
  if (rcFpv) fpvSysId = rcTarget;
  else if (viewFpv && !rcEnabled) fpvSysId = followSysId;

  return {
    fpvOn: viewFpv,
    toggleFpv,
    changeFollow,
    leaveFpv,
    /** The map camera is first-person (view FPV or RC FPV). */
    cameraFpv: fpvSysId != null,
    /** The UAV the first-person camera shows, and whose FPV it is. */
    fpvSysId,
    fpvKind: rcFpv ? 'rc' : (fpvSysId != null ? 'view' : null),
    /** FPV or RC owns the camera: camera moves must not be persisted as the startup view. */
    cameraOwned: viewFpv || rcEnabled,
  };
}
