import { useRef, useEffect, useState } from 'react';
import { CAMERA_KEY } from '../constants/icons';
import { flyToOptions } from '../../../utils/cameraFlyTo';
import { cesiumIonToken } from '../../../utils/cesiumConfig';

/** Convert a map tile zoom level to approximate camera height in meters. */
function zoomToHeight(zoom) {
  return 35200000 / Math.pow(2, zoom);
}

/**
 * A vehicle contributes its live position to the framed view only when it
 * reports a usable GPS fix (3D or better — the same threshold the launch gate
 * and preflight checks use). Without a fix the fused EKF position can be (0, 0)
 * or stale, which would otherwise blow the framed rectangle out to span the
 * globe and defeat the whole point of centering.
 */
export function hasGpsFix(v) {
  return v?.gps_fix != null && v.gps_fix >= 3;
}

/**
 * Collect the lat/lon points that "center view" should frame: planning-polygon
 * vertices, planned track waypoints, the launch point, corridor points, and the
 * positions of GPS-fixed vehicles. Pure and exported for unit testing.
 */
export function collectFitPoints({ polygon, vehicleList, plan, launchPoint, corridorPoints } = {}) {
  const pts = [];
  if (polygon?.length > 0) {
    for (const p of polygon) pts.push({ lat: p.lat, lon: p.lon ?? p.lng });
  }
  if (plan?.zones?.length > 0) {
    for (const zone of plan.zones) {
      if (zone.track) {
        for (const wp of zone.track) pts.push({ lat: wp.lat, lon: wp.lon });
      }
    }
  }
  if (launchPoint) pts.push({ lat: launchPoint.lat, lon: launchPoint.lon ?? launchPoint.lng });
  if (corridorPoints?.length > 0) {
    for (const cp of corridorPoints) pts.push({ lat: cp.lat, lon: cp.lon ?? cp.lng });
  }
  if (vehicleList?.length > 0) {
    for (const v of vehicleList) {
      if (hasGpsFix(v) && v.lat != null && v.lon != null) {
        pts.push({ lat: v.lat, lon: v.lon });
      }
    }
  }
  return pts;
}

/**
 * Viewer lifecycle hook — initializes Cesium, manages camera persistence,
 * and exposes the reset-view and fly-to-location imperative refs. SRP: init only.
 */
export default function useCesiumViewer(resetViewRef, flyToRef, mapDefaults) {
  const containerRef = useRef(null);
  const viewerRef = useRef(null);
  const cesiumRef = useRef(null);
  const readyRef = useRef(false);
  const mapDefaultsRef = useRef(mapDefaults);
  mapDefaultsRef.current = mapDefaults;
  const [viewerReady, setViewerReady] = useState(false);
  const [terrainReady, setTerrainReady] = useState(false);
  // Base map has painted its first tiles — the signal that dismisses the loading
  // overlay (perceived speed), distinct from the slower Ion terrain upgrade.
  const [tilesReady, setTilesReady] = useState(false);

  useEffect(() => {
    let cancelled = false;

    (async () => {
      const Cesium = await import('cesium');
      if (cancelled) return;
      cesiumRef.current = Cesium;

      // Load CSS
      if (!document.getElementById('cesium-css')) {
        const link = document.createElement('link');
        link.id = 'cesium-css';
        link.rel = 'stylesheet';
        link.href = '/Cesium/Widgets/widgets.css';
        document.head.appendChild(link);
      }

      self.CESIUM_BASE_URL = '/Cesium/';
      const ionToken = cesiumIonToken();
      if (ionToken) Cesium.Ion.defaultAccessToken = ionToken;

      const viewDiv = document.createElement('div');
      viewDiv.style.width = '100%';
      viewDiv.style.height = '100%';
      viewDiv.style.position = 'relative';
      containerRef.current.innerHTML = '';
      containerRef.current.appendChild(viewDiv);

      // OSM imagery as initial base layer
      const osmImagery = new Cesium.OpenStreetMapImageryProvider({
        url: 'https://tile.openstreetmap.org/',
      });

      const viewer = new Cesium.Viewer(viewDiv, {
        imageryProvider: osmImagery,
        terrainProvider: undefined,
        infoBox: false,
        selectionIndicator: false,
        baseLayerPicker: false,
        animation: false,
        timeline: false,
        geocoder: false,
        homeButton: false,
        sceneModePicker: false,
        navigationHelpButton: false,
        fullscreenButton: false,
        orderIndependentTranslucency: false,
        useBrowserRecommendedResolution: true,
        requestRenderMode: true,
        maximumRenderTimeChange: Infinity,
      });

      // Continuous clock ticking so CallbackProperty animations render
      viewer.clock.shouldAnimate = true;
      // Cap at 30 fps — sufficient for 5 Hz telemetry with lerp interpolation
      viewer.targetFrameRate = 30;

      // Enable depth test so tracks above terrain render correctly
      viewer.scene.globe.depthTestAgainstTerrain = true;

      // Disable cosmetic effects — saves GPU every frame
      viewer.scene.skyAtmosphere.show = false;
      viewer.scene.fog.enabled = false;
      viewer.scene.globe.showGroundAtmosphere = false;
      viewer.scene.globe.enableLighting = false;
      viewer.scene.sun.show = false;
      viewer.scene.moon.show = false;
      viewer.scene.skyBox.show = false;

      // Coarser terrain mesh — height queries still accurate, visual detail reduced
      viewer.scene.globe.maximumScreenSpaceError = 2;

      // Clamp zoom so Earth is always visible
      viewer.scene.screenSpaceCameraController.maximumZoomDistance = 1000000;
      viewer.scene.screenSpaceCameraController.minimumZoomDistance = 50;

      // Disable default double-click zoom — we handle double-click ourselves
      viewer.cesiumWidget.screenSpaceEventHandler.removeInputAction(
        Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK
      );

      // Dismiss path: flip tilesReady once the base map's first tiles have loaded
      // and the load queue drains (OSM initially, or Ion after the swap below) —
      // i.e. the imagery is renderable, not merely requested.
      // tileLoadProgressEvent reports tiles still queued; we wait for it to drain
      // to 0 *after* seeing a positive count (so we don't fire on the idle 0
      // before any tile is requested). This is the ONLY signal that flips
      // tilesReady — the Ion upgrade below deliberately does NOT, since a resolved
      // provider proves only metadata, not a loaded/renderable tile. Harmless if
      // it never fires: CesiumMap has a bounded fallback (LOADING_FALLBACK_MS).
      let sawTiles = false;
      const onTileProgress = (remaining) => {
        if (remaining > 0) { sawTiles = true; return; }
        if (!sawTiles || cancelled) return;
        viewer.scene.globe.tileLoadProgressEvent.removeEventListener(onTileProgress);
        setTilesReady(true);
      };
      viewer.scene.globe.tileLoadProgressEvent.addEventListener(onTileProgress);

      // Upgrade to Cesium Ion imagery + terrain, kicked immediately (dropping the
      // old fixed 1.2 s delay). This runs UNCONDITIONALLY — the previous version
      // gated the whole load behind the (slower, network-bound) Ion terrain fetch,
      // so the "AAS LOADING" screen stayed up for the entire upgrade. Now the
      // overlay is dismissed as soon as the IMAGERY tiles have loaded/drained
      // (via the tileLoadProgressEvent drain watcher above) — NOT on provider-add,
      // which only proves metadata resolved and would blank the map if Ion tiles
      // are slow or failing. The terrain geometry keeps loading behind the
      // already-visible map; terrain-dependent features gate on terrainReady,
      // unchanged. Without a configured Ion token the map stays on OSM.
      if (ionToken) (async () => {
        try {
          const ionImagery = await Cesium.IonImageryProvider.fromAssetId(3);
          if (cancelled || viewer.isDestroyed()) return;
          viewer.imageryLayers.removeAll();
          viewer.imageryLayers.addImageryProvider(ionImagery);
          viewer.scene.requestRender();
          const ionTerrain = await Cesium.CesiumTerrainProvider.fromIonAssetId(1);
          await ionTerrain.requestTileGeometry(0, 0, 0);
          if (cancelled || viewer.isDestroyed()) return;
          viewer.terrainProvider = ionTerrain;
          setTerrainReady(true);
          viewer.scene.requestRender();
        } catch (e) {
          console.warn('Cesium Ion upgrade failed, keeping OSM tiles:', e);
        }
      })();

      // Camera persistence: restore from localStorage, fall back to settings defaults
      try {
        const saved = JSON.parse(localStorage.getItem(CAMERA_KEY));
        if (saved) {
          viewer.camera.setView({
            destination: Cesium.Cartesian3.fromDegrees(saved.lon, saved.lat, saved.height),
            orientation: {
              heading: Cesium.Math.toRadians(saved.heading || 0),
              pitch: Cesium.Math.toRadians(saved.pitch || -90),
              roll: Cesium.Math.toRadians(saved.roll || 0),
            },
          });
        } else {
          const md = mapDefaultsRef.current;
          if (md?.default_lat != null && md?.default_lon != null) {
            viewer.camera.setView({
              destination: Cesium.Cartesian3.fromDegrees(
                md.default_lon, md.default_lat, zoomToHeight(md.default_zoom ?? 12),
              ),
              orientation: { heading: 0, pitch: Cesium.Math.toRadians(-90), roll: 0 },
            });
          }
        }
      } catch {}

      // Camera persistence: save on moveEnd
      viewer.camera.moveEnd.addEventListener(() => {
        try {
          const carto = Cesium.Ellipsoid.WGS84.cartesianToCartographic(viewer.camera.positionWC);
          const data = {
            lat: Cesium.Math.toDegrees(carto.latitude),
            lon: Cesium.Math.toDegrees(carto.longitude),
            height: carto.height,
            heading: Cesium.Math.toDegrees(viewer.camera.heading),
            pitch: Cesium.Math.toDegrees(viewer.camera.pitch),
            roll: Cesium.Math.toDegrees(viewer.camera.roll),
          };
          localStorage.setItem(CAMERA_KEY, JSON.stringify(data));
        } catch {}
      });

      viewerRef.current = viewer;

      // Expose reset-view function via ref
      if (resetViewRef) {
        resetViewRef.current = (polygon, vehicleList, plan, launchPoint, corridorPoints) => {
          const pts = collectFitPoints({ polygon, vehicleList, plan, launchPoint, corridorPoints });
          if (pts.length === 0) {
            const md = mapDefaultsRef.current;
            const lon = md?.default_lon ?? 34.8;
            const lat = md?.default_lat ?? 31.5;
            const height = md?.default_zoom != null ? zoomToHeight(md.default_zoom) : 800000;
            viewer.camera.flyTo({
              destination: Cesium.Cartesian3.fromDegrees(lon, lat, height),
              orientation: { heading: 0, pitch: Cesium.Math.toRadians(-90), roll: 0 },
              duration: 1.0,
            });
            return;
          }
          const rect = Cesium.Rectangle.fromCartographicArray(
            pts.map((p) => Cesium.Cartographic.fromDegrees(p.lon, p.lat))
          );
          // Pad the rectangle by 30% so the whole plan is comfortably visible
          const dLon = Math.max((rect.east - rect.west) * 0.30, Cesium.Math.toRadians(0.002));
          const dLat = Math.max((rect.north - rect.south) * 0.30, Cesium.Math.toRadians(0.002));
          const padded = new Cesium.Rectangle(
            rect.west - dLon, rect.south - dLat,
            rect.east + dLon, rect.north + dLat,
          );
          viewer.camera.flyTo({
            destination: padded,
            orientation: { heading: 0, pitch: Cesium.Math.toRadians(-90), roll: 0 },
            duration: 1.0,
          });
        };
      }

      // Expose fly-to-location via ref (mirrors resetViewRef; closes over viewer + Cesium)
      if (flyToRef) {
        flyToRef.current = (lat, lon) => {
          viewer.camera.flyTo(flyToOptions(Cesium, lat, lon));
        };
      }

      readyRef.current = true;
      setViewerReady(true);
    })();

    return () => {
      cancelled = true;
    };
  }, []);

  return { containerRef, viewerRef, cesiumRef, readyRef, viewerReady, terrainReady, tilesReady };
}
