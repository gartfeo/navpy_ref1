import React, { useEffect, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../styles';
import { fpvHoldReason } from '../utils/fpvView';
import { HEADER_FONT_PX } from '../utils/vehicleWindowLayout';
import { ION_TERRAIN_ASSET_ID, MAP_TARGET_FRAME_RATE } from './map/constants/icons';
import { createPoseMath, noseOrientation } from './map/utils/fpvPose';
import { observeHeight } from './map/utils/observeHeight';

/** One ground colour for the shaded terrain: plainly unlike camera video. */
const GROUND_COLOR = '#7d8a68';
/**
 * Fixed light, not the sun, so shading does not change with the time of day:
 * travelling east-south and down in the UAV's local east-north-up frame, i.e.
 * from the north-west, above.
 */
const LIGHT_ENU = [0.5, -0.5, -0.7];

/**
 * The vehicle window's Synthetic source (docs/design/gcs_video_streams.md,
 * section 5 "Synthetic view"): a second public CesiumWidget with shaded
 * terrain, no imagery and no entities, looking through the window UAV's nose.
 * Operator display only; it never confirms anything.
 *
 * - Pose from the main map's pose source (`poseSourceRef`), applied only when
 *   the shared hold check passes and the copied numbers are finite; otherwise
 *   the last good pose holds. One render request per telemetry update.
 * - "Waiting for pose · UAV n" covers the picture until the window's UAV has
 *   had a valid pose, so a previous UAV's view never shows under a new label.
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
 * @param {() => void} props.onFail
 * @param {(px: number) => void} [props.onCreditsHeight]
 * @param {object} [props.style] - box of the picture.
 */
export default function SyntheticView({
  sysId, label, poseSourceRef, storeRef, ionTerrain, onFail, onCreditsHeight, style,
}) {
  const { t } = useTranslation();
  const containerRef = useRef(null);
  const widgetRef = useRef(null);
  const cesiumRef = useRef(null);
  const [widgetReady, setWidgetReady] = useState(false);
  const [poseFor, setPoseFor] = useState(null);

  const sysIdRef = useRef(sysId);
  sysIdRef.current = sysId;
  const onFailRef = useRef(onFail);
  onFailRef.current = onFail;
  const onCreditsRef = useRef(onCreditsHeight);
  onCreditsRef.current = onCreditsHeight;

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
          // Never undefined (that adds Ion world imagery), never imageryProvider.
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
      cleanups.push(() => widget.destroy());
      const { scene } = widget;
      scene.globe.baseColor = Cesium.Color.fromCssColorString(GROUND_COLOR);
      scene.globe.enableLighting = true;
      const light = new Cesium.DirectionalLight({ direction: new Cesium.Cartesian3(0, 0, -1) });
      scene.light = light;
      scene.screenSpaceCameraController.enableInputs = false;

      const onRenderError = (_scene, e) => fail(e);
      scene.renderError.addEventListener(onRenderError);
      cleanups.push(() => scene.renderError.removeEventListener(onRenderError));
      const onContextLost = (e) => fail(e);
      widget.canvas.addEventListener('webglcontextlost', onContextLost);
      cleanups.push(() => widget.canvas.removeEventListener('webglcontextlost', onContextLost));

      const poseMath = createPoseMath(Cesium);
      const lightEnu = new Cesium.Cartesian3(...LIGHT_ENU);
      const lightDir = new Cesium.Cartesian3();
      let shownFor = null;
      // Cesium raises preRender outside its error wrapper, and a throw there
      // stops the widget's render loop with no event: catch it here.
      const onPreRender = () => {
        try {
          const id = sysIdRef.current;
          const store = storeRef.current;
          if (fpvHoldReason(store.getVehicles()[id], store.isTelemetryFresh(id))) return;
          const pose = poseSourceRef.current?.pose(id);
          if (!pose) return;
          const orientation = noseOrientation(poseMath.vehicleHpr(pose.position, pose.orientation));
          const values = [pose.position.x, pose.position.y, pose.position.z,
            orientation.heading, orientation.pitch, orientation.roll];
          if (!values.every(Number.isFinite)) return;
          widget.camera.setView({ destination: pose.position, orientation });
          Cesium.Matrix4.multiplyByPointAsVector(poseMath.transform, lightEnu, lightDir);
          light.direction = Cesium.Cartesian3.normalize(lightDir, new Cesium.Cartesian3());
          if (shownFor !== id) {
            shownFor = id;
            setPoseFor(id);
          }
        } catch (e) {
          fail(e);
        }
      };
      scene.preRender.addEventListener(onPreRender);
      cleanups.push(() => scene.preRender.removeEventListener(onPreRender));

      cleanups.push(storeRef.current.subscribe(() => scene.requestRender()));
      cleanups.push(observeHeight(widget.creditContainer, (h) => onCreditsRef.current?.(h)));

      cesiumRef.current = Cesium;
      widgetRef.current = widget;
      setWidgetReady(true);
      scene.requestRender();
    })().catch(fail);

    return () => {
      cancelled = true;
      widgetRef.current = null;
      for (const c of cleanups.reverse()) c();
      onCreditsRef.current?.(0);
    };
  }, [poseSourceRef, storeRef]);

  // A badge tap moves the same widget to the new UAV.
  useEffect(() => {
    widgetRef.current?.scene.requestRender();
  }, [sysId, widgetReady]);

  // Its own terrain instance of the main map's source, following the Ion upgrade.
  useEffect(() => {
    const Cesium = cesiumRef.current;
    if (!ionTerrain || !widgetReady || !Cesium) return undefined;
    let cancelled = false;
    Cesium.CesiumTerrainProvider.fromIonAssetId(ION_TERRAIN_ASSET_ID, { requestVertexNormals: true })
      .then((terrain) => {
        const widget = widgetRef.current;
        if (cancelled || !widget || widget.isDestroyed()) return;
        widget.terrainProvider = terrain;
        widget.scene.requestRender();
      })
      .catch((e) => console.warn('Synthetic view: Ion terrain unavailable, flat ground:', e));
    return () => { cancelled = true; };
  }, [ionTerrain, widgetReady]);

  return (
    <div style={{ position: 'relative', background: colors.bg, ...style }}>
      <div ref={containerRef} data-testid="synthetic-widget" style={{ position: 'absolute', inset: 0 }} />
      {poseFor !== sysId && (
        <div
          role="status"
          style={{
            position: 'absolute',
            inset: 0,
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            fontSize: HEADER_FONT_PX,
            fontWeight: 700,
            color: colors.textDim,
            background: colors.bgLight,
          }}
        >
          {t('vehicleWindow.waitingForPose', { name: label })}
        </div>
      )}
      <div
        data-testid="synthetic-frame"
        style={{ position: 'absolute', inset: 0, pointerEvents: 'none', boxShadow: `inset 0 0 0 3px ${colors.accent}` }}
      />
      <div
        role="note"
        style={{
          position: 'absolute',
          top: 6,
          left: 6,
          padding: '4px 8px',
          fontSize: HEADER_FONT_PX,
          fontWeight: 700,
          color: colors.textBright,
          background: colors.bgLight,
          border: `1px solid ${colors.accent}`,
          borderRadius: 4,
          pointerEvents: 'none',
        }}
      >
        {t('vehicleWindow.syntheticTag')}
      </div>
    </div>
  );
}
