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
import { createPoseMath, noseOrientation } from './map/utils/fpvPose';
import { observeHeight } from './map/utils/observeHeight';

/** The globe's colour under the imagery, while its tiles load or if they fail. */
const GROUND_COLOR = '#7d8a68';
/** The tag's accent border, which its measured (client) height leaves out. */
const TAG_BORDER_PX = 1;
const NO_SIZE = { width: 0, height: 0 };

/**
 * The vehicle window's Synthetic source (docs/design/gcs_video_streams.md,
 * section 5 "Synthetic view"): RC control's first-person view without the
 * controls. A second public CesiumWidget with the main map's base layer on its
 * terrain, unlit like the map, and no entities, looking through the window
 * UAV's nose, under the HUD RC control shows (TelemetryHud). Operator display
 * only; it never confirms anything.
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
 */
export default function SyntheticView({
  sysId, label, poseSourceRef, storeRef, ionTerrain, baseLayer, onBaseLayerFailed, onFail, onCreditsHeight, style,
  expanded = false, smallMap = null,
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

  const sysIdRef = useRef(sysId);
  sysIdRef.current = sysId;
  const onFailRef = useRef(onFail);
  onFailRef.current = onFail;
  const onCreditsRef = useRef(onCreditsHeight);
  onCreditsRef.current = onCreditsHeight;
  const onBaseLayerFailedRef = useRef(onBaseLayerFailed);
  onBaseLayerFailedRef.current = onBaseLayerFailed;
  const [creditsPx, setCreditsPx] = useState(0);
  const [statusSize, setStatusSize] = useState(NO_SIZE);
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

      const poseMath = createPoseMath(Cesium);
      // The window UAV's pose when the shared hold check passes and the copied
      // numbers are finite; else null (the last good pose holds).
      const validPose = (id) => {
        const store = storeRef.current;
        if (fpvHoldReason(store.getVehicles()[id], store.isTelemetryFresh(id))) return null;
        const pose = poseSourceRef.current?.pose(id);
        if (!pose) return null;
        const orientation = noseOrientation(poseMath.vehicleHpr(pose.position, pose.orientation));
        const values = [pose.position.x, pose.position.y, pose.position.z,
          orientation.heading, orientation.pitch, orientation.roll];
        return values.every(Number.isFinite) ? { position: pose.position, orientation } : null;
      };
      // Readiness is checked per telemetry update, not in preRender: while
      // waiting the picture is collapsed, and a zero-size canvas never renders.
      const checkReady = () => {
        try {
          const id = sysIdRef.current;
          if (validPose(id)) setReadyFor(id);
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
          appliedFor = id;
        } catch (e) {
          fail(e);
        }
      };
      const onPostRender = () => {
        if (appliedFor === sysIdRef.current) setDrawnFor(appliedFor);
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

  // A badge tap moves the same widget to the new UAV; the picture expands
  // once that UAV has a valid pose.
  useEffect(() => {
    checkReadyRef.current?.();
  }, [sysId, widgetReady]);
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
  const { hud, tagMaxWidth } = pictureHudLayout({
    width: picture.width,
    height: picture.height,
    tagHeight: tag.height + 2 * TAG_BORDER_PX,
    status: statusSize,
    creditsPx,
    smallMap,
  });
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
        <div ref={containerRef} data-testid="synthetic-widget" style={{ position: 'absolute', inset: 0 }} />
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
            <TelemetryHud key={sysId} sysId={sysId} layout={hud} onStatusSize={setStatusSize} />
          </div>
        )}
        <div
          data-testid="synthetic-frame"
          style={{ position: 'absolute', inset: 0, pointerEvents: 'none', boxShadow: `inset 0 0 0 3px ${colors.accent}` }}
        />
        <div
          ref={tagRef}
          role="note"
          style={{ ...pictureTagStyle(colors.accent, tagMaxWidth), border: `${TAG_BORDER_PX}px solid ${colors.accent}` }}
        >
          {t('vehicleWindow.syntheticTag')}
        </div>
      </div>
    </>
  );
}
