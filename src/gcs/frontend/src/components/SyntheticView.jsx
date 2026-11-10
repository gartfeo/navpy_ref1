import React, { useEffect, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../styles';
import { fpvHoldReason } from '../utils/fpvView';
import { WINDOW_FONT_PX } from '../utils/vehicleWindowLayout';
import { pictureHudLayout } from '../utils/hudLayout';
import useElementSize from '../hooks/useElementSize';
import { pictureTagStyle } from './windowStyles';
import TelemetryHud from './TelemetryHud';
import { ION_TERRAIN_ASSET_ID, MAP_TARGET_FRAME_RATE } from './map/constants/icons';
import { applyBaseLayer } from './map/utils/baseLayers';
import { createRouteOverlay } from './map/utils/routeOverlay';
import { createPoseMath, noseOrientation } from './map/utils/fpvPose';
import { GIMBAL_ON_AXIS_RAD, cesiumFov, gimbalOrientation, liveGimbalView } from './map/utils/gimbalView';
import { getDeviceConfigs } from '../utils/plannerConfig';
import { observeHeight } from './map/utils/observeHeight';
import { createSceneMarks } from './map/utils/sceneMarks';
import { flightPathMarkerPoint } from './map/utils/flightPathMarker';
import { smoothVector, velocityEnu } from '../utils/syntheticScene';
import { flatDist } from '../utils/geo';
import { formatDistance } from '../utils/fpvRoute';

// At normal size the credit line (Cesium ion logo and the map data credits,
// two lines at the handheld's picture width) took a fifth of the picture
// (owner, 2026-11-08). There a © button opens it instead, on its own row
// above the bottom edge so it never covers the status line; maximized the
// credits always show. The main map shows them in full the whole time.
export const CREDITS_BUTTON_PX = 24;
const CREDITS_BUTTON_GAP_PX = 4;
const CREDITS_BUTTON_ROW_PX = CREDITS_BUTTON_PX + 2 * CREDITS_BUTTON_GAP_PX;

/** The flight path marker: a ring with wings, sized to read on the small picture. */
const FPM_RING_PX = 7;
const FPM_WING_PX = 9;

/** The globe's colour under the imagery, while its tiles load or if they fail. */
const GROUND_COLOR = '#7d8a68';
/** The tag's accent border, which its measured (client) height leaves out. */
const TAG_BORDER_PX = 1;
const NO_SIZE = { width: 0, height: 0 };

/**
 * The vehicle window's Synthetic source (docs/design/gcs_video_streams.md,
 * section 5 "Synthetic view"): RC control's first-person view without the
 * controls. A second public CesiumWidget with the main map's base layer on its
 * terrain, unlit like the map, with the UAV's own route, its dock (highlighted,
 * with the distance), its home hub and the rest of the fleet
 * (map/utils/sceneMarks.js), and a flight path marker where its measured ground
 * velocity points (display only; utils/syntheticScene.js), looking
 * through the window UAV's gimbal camera where it has live gimbal telemetry
 * (its attitude and field of view, map/utils/gimbalView.js), else through its
 * nose, under the HUD RC control shows (TelemetryHud). The tag says which.
 * Operator display only; it never confirms anything.
 *
 * - The HUD stays the aircraft's data; its attitude indicator is left out
 *   while the camera looks more than GIMBAL_ON_AXIS_RAD off the nose, where
 *   the aircraft's horizon would not match the picture's.
 *
 * - Pose from the main map's pose source (`poseSourceRef`), applied only when
 *   the shared hold check passes and the copied numbers are finite; otherwise
 *   the last good pose holds. One render request per telemetry update.
 * - "Waiting for pose · UAV n" until a frame was drawn with the window UAV's
 *   pose, so a previous UAV's view never shows under a new label. At normal
 *   size the picture stays collapsed to that text row until the UAV has a
 *   valid pose; then an opaque cover holds until the first such frame.
 *   Maximized, the cover fills the large area.
 * - Failure (create, render error, WebGL context lost, a throw in the pose
 *   callback) calls `onFail` once; the parent then switches to Camera, which
 *   unmounts this and destroys the widget.
 *
 * @param {object} props
 * @param {number} props.sysId
 * @param {string} props.label
 * @param {{ current: { pose: (sysId: number) => object|null } | null }} props.poseSourceRef
 * @param {{ current: object }} props.storeRef - telemetry store.
 * @param {boolean} props.ionTerrain - the main map upgraded to Ion terrain.
 * @param {string} props.baseLayer - the main map's base layer id (map/utils/baseLayers.js), available.
 * @param {(id: string, err: unknown) => void} [props.onBaseLayerFailed]
 * @param {() => void} props.onFail
 * @param {(px: number) => void} [props.onCreditsHeight]
 * @param {object} [props.style] - box of the picture.
 * @param {boolean} [props.expanded] - maximized: the waiting state keeps the
 *   large area; at normal size it collapses to one text row.
 * @param {{width: number, height: number}|null} [props.smallMap] - maximized: the small
 *   map in the picture's top-right corner; the tag and the HUD stay clear of it.
 * @param {{ current: { route: (sysId: number) => object|null } | null }} [props.routeSourceRef] - the
 *   UAV's own route (utils/fpvRoute.js), drawn over the terrain: numbered when
 *   maximized, only the next waypoint's number at normal size.
 * @param {boolean} [props.panelShown] - the right panel is open: the HUD leaves out what it shows.
 * @param {{dock: object|null, hub: object|null}} [props.marks] - the UAV's dock and default
 *   delivery hub (utils/syntheticScene.js), marked in the scene with the rest of the fleet.
 * @param {boolean} [props.held] - the view holds its last pose (the window's "View held"
 *   row says why): the picture greys out and the flight path marker hides, so a held
 *   picture never passes for a live one.
 */
export default function SyntheticView({
  sysId, label, poseSourceRef, storeRef, ionTerrain, baseLayer, onBaseLayerFailed, onFail, onCreditsHeight, style,
  expanded = false, smallMap = null, routeSourceRef = null, panelShown = true, marks = null, held = false,
}) {
  const { t } = useTranslation();
  const containerRef = useRef(null);
  const widgetRef = useRef(null);
  const cesiumRef = useRef(null);
  const checkReadyRef = useRef(null);
  const [widgetReady, setWidgetReady] = useState(false);
  // The UAV with a valid pose (the picture may expand), and the UAV whose
  // pose the last drawn frame used (the cover may go).
  const [readyFor, setReadyFor] = useState(null);
  const [drawnFor, setDrawnFor] = useState(null);
  // Through the gimbal camera (else the nose), and the HUD's horizon matches the picture.
  const [gimbalShown, setGimbalShown] = useState(false);
  const [creditsOpen, setCreditsOpen] = useState(false);
  const creditsHidden = !expanded && !creditsOpen;
  const [horizonShown, setHorizonShown] = useState(true);
  // Where the UAV is going, in the picture (CSS px), or null.
  const [fpm, setFpm] = useState(null);
  const marksRef = useRef(marks);
  marksRef.current = marks;
  const heldRef = useRef(held);
  heldRef.current = held;
  const tRef = useRef(t);
  tRef.current = t;

  const sysIdRef = useRef(sysId);
  sysIdRef.current = sysId;
  const onFailRef = useRef(onFail);
  onFailRef.current = onFail;
  const onCreditsRef = useRef(onCreditsHeight);
  onCreditsRef.current = onCreditsHeight;
  const onBaseLayerFailedRef = useRef(onBaseLayerFailed);
  onBaseLayerFailedRef.current = onBaseLayerFailed;
  const routeSourceRefRef = useRef(routeSourceRef);
  routeSourceRefRef.current = routeSourceRef;
  const expandedRef = useRef(expanded);
  expandedRef.current = expanded;
  const [creditsPx, setCreditsPx] = useState(0);
  const [cornerSize, setCornerSize] = useState(NO_SIZE);
  const pictureRef = useRef(null);
  const picture = useElementSize(pictureRef, NO_SIZE);
  const tagRef = useRef(null);
  const tag = useElementSize(tagRef, NO_SIZE);

  useEffect(() => {
    let cancelled = false;
    let failed = false;
    const cleanups = [];
    const fail = (e) => {
      if (failed || cancelled) return;
      failed = true;
      console.warn('Synthetic view unavailable:', e);
      onFailRef.current();
    };

    (async () => {
      const Cesium = await import('cesium');
      if (cancelled) return;
      let widget;
      try {
        widget = new Cesium.CesiumWidget(containerRef.current, {
          // Never undefined (that adds Ion world imagery), never imageryProvider:
          // the effect below adds the main map's base layer.
          baseLayer: false,
          // No sky box, so no sun and moon either; sky atmosphere stays (horizon cue).
          skyBox: false,
          requestRenderMode: true,
          maximumRenderTimeChange: Infinity,
          targetFrameRate: MAP_TARGET_FRAME_RATE,
          msaaSamples: 1,
          showRenderLoopErrors: false,
          useBrowserRecommendedResolution: true,
        });
      } catch (e) {
        fail(e);
        return;
      }
      // destroy() leaves the WebGL context to the garbage collector, and the
      // browser keeps only about 16 live contexts, dropping the oldest (the main
      // map's) first: release it now. Cleanups run in reverse, so the
      // context-lost listener below is already removed by then.
      const gl = widget.canvas.getContext('webgl2') ?? widget.canvas.getContext('webgl');
      cleanups.push(() => {
        widget.destroy();
        gl?.getExtension('WEBGL_lose_context')?.loseContext();
      });
      const { scene } = widget;
      scene.globe.baseColor = Cesium.Color.fromCssColorString(GROUND_COLOR);
      // Unlit, with no ground haze: the imagery shows as on the main map.
      scene.globe.enableLighting = false;
      scene.globe.showGroundAtmosphere = false;
      scene.screenSpaceCameraController.enableInputs = false;

      const onRenderError = (_scene, e) => fail(e);
      scene.renderError.addEventListener(onRenderError);
      cleanups.push(() => scene.renderError.removeEventListener(onRenderError));
      const onContextLost = (e) => fail(e);
      widget.canvas.addEventListener('webglcontextlost', onContextLost);
      cleanups.push(() => widget.canvas.removeEventListener('webglcontextlost', onContextLost));

      // The UAV's own route, redrawn only when it, its next waypoint or the size changes.
      const route = createRouteOverlay(Cesium, scene);
      cleanups.push(() => route.destroy());
      const syncRoute = () => {
        const r = routeSourceRefRef.current?.current?.route(sysIdRef.current) ?? null;
        return route.update(r, { line: true, labels: expandedRef.current ? 'all' : 'next' });
      };

      // The dock, the delivery hub and the rest of the fleet, per telemetry update.
      const sceneMarks = createSceneMarks(Cesium, scene);
      cleanups.push(() => sceneMarks.destroy());
      const syncMarks = () => {
        const store = storeRef.current;
        const id = sysIdRef.current;
        const me = store.getVehicles()[id];
        const { dock = null, hub = null } = marksRef.current || {};
        const tr = tRef.current;
        const dockText = !dock ? ''
          : Number.isFinite(me?.lat) && Number.isFinite(me?.lon)
            ? tr('vehicleWindow.sceneDock', { dist: formatDistance(flatDist({ lat: me.lat, lon: me.lon }, dock)) })
            : tr('vehicleWindow.sceneDockNoDistance');
        const fleet = [];
        (store.getVehicleList() || []).forEach((v, zone) => {
          if (!v || v.sys_id === id || fpvHoldReason(v, store.isTelemetryFresh(v.sys_id))) return;
          const pose = poseSourceRef.current?.pose(v.sys_id);
          if (pose?.position) fleet.push({ position: pose.position, label: v.name ?? `UAV ${v.sys_id}`, zone });
        });
        sceneMarks.update({ dock, dockText, hub, hubText: tr('vehicleWindow.sceneHub'), fleet });
      };
      // The flight path marker's smoothed ground velocity (east, north, up), for the window UAV.
      let fpmVel = null;
      let fpmAt = 0;
      let fpmFor = null;
      const syncFpm = () => {
        const id = sysIdRef.current;
        const store = storeRef.current;
        const now = performance.now();
        const target = heldRef.current ? null : velocityEnu(store.getVehicles()[id]?.velocity_ned);
        fpmVel = smoothVector(fpmFor === id ? fpmVel : null, target, (now - fpmAt) / 1000);
        fpmAt = now;
        fpmFor = id;
      };

      const poseMath = createPoseMath(Cesium);
      const noseFov = widget.camera.frustum.fov;
      // The window UAV's pose when the shared hold check passes and the copied
      // numbers are finite; else null (the last good pose holds). Through the
      // gimbal camera where it has live telemetry, else the nose.
      const validPose = (id) => {
        const store = storeRef.current;
        const vehicle = store.getVehicles()[id];
        if (fpvHoldReason(vehicle, store.isTelemetryFresh(id))) return null;
        const pose = poseSourceRef.current?.pose(id);
        if (!pose) return null;
        const nose = noseOrientation(poseMath.vehicleHpr(pose.position, pose.orientation));
        const gimbal = liveGimbalView(vehicle, getDeviceConfigs());
        const through = gimbal && gimbalOrientation(nose, gimbal.cameraToVehicle);
        const orientation = through
          ? { heading: through.heading, pitch: through.pitch, roll: through.roll }
          : { heading: nose.heading, pitch: nose.pitch, roll: nose.roll };
        const fov = through ? cesiumFov(gimbal.fovH, widget.camera.frustum.aspectRatio) : noseFov;
        const values = [pose.position.x, pose.position.y, pose.position.z,
          orientation.heading, orientation.pitch, orientation.roll, fov];
        if (!values.every(Number.isFinite)) return null;
        return {
          position: pose.position, orientation, fov, gimbal: !!through, horizon: !through || through.offNose <= GIMBAL_ON_AXIS_RAD,
        };
      };
      // Readiness is checked per telemetry update, not in preRender: while
      // waiting the picture is collapsed, and a zero-size canvas never renders.
      const checkReady = () => {
        try {
          const id = sysIdRef.current;
          if (validPose(id)) setReadyFor(id);
          syncRoute();
          syncMarks();
          syncFpm();
          scene.requestRender();
        } catch (e) {
          fail(e);
        }
      };
      // Cesium raises preRender outside its error wrapper, and a throw there
      // stops the widget's render loop with no event: catch it here.
      // The UAV whose pose the camera holds; a held pose keeps the previous one.
      let appliedFor = null;
      const onPreRender = () => {
        try {
          const id = sysIdRef.current;
          const pose = validPose(id);
          if (!pose) return;
          widget.camera.setView({ destination: pose.position, orientation: pose.orientation });
          if (widget.camera.frustum.fov !== pose.fov) widget.camera.frustum.fov = pose.fov;
          // React keeps equal state without a render.
          setGimbalShown(pose.gimbal);
          setHorizonShown(pose.horizon);
          appliedFor = id;
        } catch (e) {
          fail(e);
        }
      };
      let fpmShown = null;
      const onPostRender = () => {
        const id = sysIdRef.current;
        if (appliedFor === id) setDrawnFor(appliedFor);
        let point = null;
        try {
          if (fpmVel && fpmFor === id && appliedFor === id && !heldRef.current) {
            const canvas = widget.canvas;
            point = flightPathMarkerPoint(Cesium, scene, widget.camera.positionWC, fpmVel,
              { width: canvas.clientWidth, height: canvas.clientHeight });
          }
        } catch (e) {
          fail(e);
          return;
        }
        if (point?.x !== fpmShown?.x || point?.y !== fpmShown?.y) {
          fpmShown = point;
          setFpm(point);
        }
      };
      scene.preRender.addEventListener(onPreRender);
      cleanups.push(() => scene.preRender.removeEventListener(onPreRender));
      scene.postRender.addEventListener(onPostRender);
      cleanups.push(() => scene.postRender.removeEventListener(onPostRender));

      cleanups.push(storeRef.current.subscribe(checkReady));
      // The credit line itself (CesiumWidget's inner credits element), not the
      // credit container, which is the whole widget by default.
      cleanups.push(observeHeight(widget.creditDisplay.container, (h) => {
        setCreditsPx(h);
        onCreditsRef.current?.(h);
      }));

      cesiumRef.current = Cesium;
      widgetRef.current = widget;
      checkReadyRef.current = checkReady;
      setWidgetReady(true);
      checkReady();
    })().catch(fail);

    return () => {
      cancelled = true;
      widgetRef.current = null;
      checkReadyRef.current = null;
      for (const c of cleanups.reverse()) c();
      onCreditsRef.current?.(0);
    };
  }, [poseSourceRef, storeRef]);

  // The credit line shows maximized, or at normal size once the © button opens it.
  useEffect(() => {
    const credits = widgetRef.current?.creditDisplay?.container;
    if (credits) credits.style.display = creditsHidden ? 'none' : '';
  }, [widgetReady, creditsHidden]);

  // A badge tap moves the same widget to the new UAV; the picture expands
  // once that UAV has a valid pose.
  useEffect(() => {
    checkReadyRef.current?.();
  }, [sysId, widgetReady, expanded, marks, held]);
  useEffect(() => {
    widgetRef.current?.scene.requestRender();
  }, [readyFor]);

  // Its own instance of the main map's base layer, following the operator's pick.
  useEffect(() => {
    const Cesium = cesiumRef.current;
    const widget = widgetRef.current;
    if (!widgetReady || !Cesium || !widget || widget.isDestroyed()) return;
    applyBaseLayer(Cesium, widget, baseLayer, (e) => onBaseLayerFailedRef.current?.(baseLayer, e));
  }, [baseLayer, widgetReady]);

  // Its own terrain instance of the main map's source, following the Ion upgrade.
  useEffect(() => {
    const Cesium = cesiumRef.current;
    if (!ionTerrain || !widgetReady || !Cesium) return undefined;
    let cancelled = false;
    Cesium.CesiumTerrainProvider.fromIonAssetId(ION_TERRAIN_ASSET_ID)
      .then((terrain) => {
        const widget = widgetRef.current;
        if (cancelled || !widget || widget.isDestroyed()) return;
        widget.terrainProvider = terrain;
        widget.scene.requestRender();
      })
      .catch((e) => console.warn('Synthetic view: Ion terrain unavailable, flat ground:', e));
    return () => { cancelled = true; };
  }, [ionTerrain, widgetReady]);

  const waiting = drawnFor !== sysId;
  // Waiting at normal size: no picture, one text row (never a black box). The
  // widget stays mounted, at zero height, so it keeps its state.
  const collapsed = readyFor !== sysId && !expanded;
  const waitingText = t('vehicleWindow.waitingForPose', { name: label });
  const { hud: layoutHud, tagMaxWidth } = pictureHudLayout({
    width: picture.width,
    height: picture.height,
    tagHeight: tag.height + 2 * TAG_BORDER_PX,
    corner: cornerSize,
    creditsPx: expanded ? creditsPx : (creditsHidden ? 0 : creditsPx) + CREDITS_BUTTON_ROW_PX,
    smallMap,
    panelShown,
  });
  const hud = horizonShown ? layoutHud : { ...layoutHud, attitude: null };
  return (
    <>
      {collapsed && (
        <div
          role="status"
          style={{
            position: 'relative',
            padding: '6px 8px',
            fontSize: WINDOW_FONT_PX,
            fontWeight: 700,
            color: colors.textDim,
            borderTop: `1px solid ${colors.border}`,
          }}
        >
          {waitingText}
        </div>
      )}
      <div
        ref={pictureRef}
        style={collapsed
          ? { position: 'relative', height: 0, overflow: 'hidden', flex: '0 0 auto' }
          : { position: 'relative', background: colors.bg, ...style }}
      >
        <div
          ref={containerRef}
          data-testid="synthetic-widget"
          style={{ position: 'absolute', inset: 0, ...(held && { filter: 'grayscale(1) brightness(0.7)' }) }}
        />
        {waiting && !collapsed && (
          <div
            role="status"
            style={{
              position: 'absolute',
              inset: 0,
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              fontSize: WINDOW_FONT_PX,
              fontWeight: 700,
              color: colors.textDim,
              background: colors.bgLight,
            }}
          >
            {waitingText}
          </div>
        )}
        {/* RC control's HUD without its controls; z-index 0 and before the tag, so the tag stays on top. */}
        {!waiting && !collapsed && (
          <div
            data-testid="synthetic-hud"
            style={{ position: 'absolute', inset: 0, zIndex: 0, isolation: 'isolate', pointerEvents: 'none' }}
          >
            <TelemetryHud
              key={sysId}
              sysId={sysId}
              layout={hud}
              routeSourceRef={routeSourceRef}
              onCornerSize={setCornerSize}
            />
          </div>
        )}
        {fpm && !waiting && !collapsed && !held && (
          <svg
            data-testid="synthetic-fpm"
            aria-hidden="true"
            width={2 * (FPM_RING_PX + FPM_WING_PX) + 2}
            height={2 * FPM_RING_PX + FPM_WING_PX}
            style={{
              position: 'absolute',
              left: fpm.x - (FPM_RING_PX + FPM_WING_PX + 1),
              top: fpm.y - FPM_RING_PX - FPM_WING_PX,
              pointerEvents: 'none',
              zIndex: 1,
              overflow: 'visible',
            }}
          >
            <g
              transform={`translate(${FPM_RING_PX + FPM_WING_PX + 1} ${FPM_RING_PX + FPM_WING_PX})`}
              fill="none"
              stroke={colors.success}
              strokeWidth={2.5}
              style={{ filter: 'drop-shadow(0 0 1px #000) drop-shadow(0 0 1px #000)' }}
            >
              <circle r={FPM_RING_PX} />
              <line x1={-FPM_RING_PX - FPM_WING_PX} y1={0} x2={-FPM_RING_PX} y2={0} />
              <line x1={FPM_RING_PX} y1={0} x2={FPM_RING_PX + FPM_WING_PX} y2={0} />
              <line x1={0} y1={-FPM_RING_PX} x2={0} y2={-FPM_RING_PX - FPM_WING_PX * 0.6} />
            </g>
          </svg>
        )}
        <div
          data-testid="synthetic-frame"
          style={{ position: 'absolute', inset: 0, pointerEvents: 'none', boxShadow: `inset 0 0 0 3px ${colors.accent}` }}
        />
        {!expanded && !collapsed && (
          <button
            type="button"
            aria-label={t('vehicleWindow.mapCredits')}
            aria-expanded={creditsOpen}
            title={t('vehicleWindow.mapCredits')}
            onClick={() => setCreditsOpen((open) => !open)}
            style={{
              position: 'absolute',
              left: CREDITS_BUTTON_GAP_PX + 3,
              bottom: (creditsHidden ? 0 : creditsPx) + CREDITS_BUTTON_GAP_PX,
              width: CREDITS_BUTTON_PX,
              height: CREDITS_BUTTON_PX,
              padding: 0,
              borderRadius: '50%',
              border: `1px solid ${colors.border}`,
              background: 'rgba(0,0,0,0.55)',
              color: colors.text,
              fontSize: 13,
              lineHeight: `${CREDITS_BUTTON_PX - 2}px`,
              cursor: 'pointer',
              zIndex: 1,
            }}
          >
            ©
          </button>
        )}
        <div
          ref={tagRef}
          role="note"
          style={{ ...pictureTagStyle(colors.accent, tagMaxWidth), border: `${TAG_BORDER_PX}px solid ${colors.accent}` }}
        >
          {t(gimbalShown ? 'vehicleWindow.syntheticTagGimbal' : 'vehicleWindow.syntheticTag')}
        </div>
      </div>
    </>
  );
}
