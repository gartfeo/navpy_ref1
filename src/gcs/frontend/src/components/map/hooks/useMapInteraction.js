import { useEffect, useRef } from 'react';
import { handleClick } from './handlers/handleClick';
import { handleDoubleClick } from './handlers/handleDoubleClick';
import { handleDragStart } from './handlers/handleDragStart';
import { handleDragMove } from './handlers/handleDragMove';
import { handleDragEnd } from './handlers/handleDragEnd';

/**
 * Event handler orchestrator — registers Cesium event handlers that delegate
 * to focused handler functions. No business logic, just wiring.
 */
export default function useMapInteraction(cesiumRef, viewerRef, readyRef, callbacksRef, refs) {
  const handlerRef = useRef(null);

  useEffect(() => {
    const Cesium = cesiumRef.current;
    const viewer = viewerRef.current;
    if (!Cesium || !viewer || !readyRef.current) return;

    const handler = new Cesium.ScreenSpaceEventHandler(viewer.scene.canvas);
    handlerRef.current = handler;

    let dragState = null;
    let skipNextClick = false;
    const dragRefs = {
      ...refs,
      lastCorridorDown: { index: -1, setIdx: -1, time: 0 },
    };

    // LEFT_CLICK
    handler.setInputAction((click) => {
      if (skipNextClick) {
        skipNextClick = false;
        return;
      }
      handleClick(Cesium, viewer, click, callbacksRef);
    }, Cesium.ScreenSpaceEventType.LEFT_CLICK);

    // LEFT_DOUBLE_CLICK
    handler.setInputAction((click) => {
      handleDoubleClick(Cesium, viewer, click, callbacksRef, refs.activeSetIndex);
    }, Cesium.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);

    // LEFT_DOWN
    handler.setInputAction((click) => {
      const result = handleDragStart(Cesium, viewer, click, callbacksRef, dragRefs);
      if (result) {
        dragState = result.dragState;
        if (result.skipNextClick) skipNextClick = true;
      }
    }, Cesium.ScreenSpaceEventType.LEFT_DOWN);

    // MOUSE_MOVE
    handler.setInputAction((movement) => {
      handleDragMove(Cesium, viewer, movement, dragState, callbacksRef, refs);
    }, Cesium.ScreenSpaceEventType.MOUSE_MOVE);

    // LEFT_UP
    handler.setInputAction(() => {
      if (handleDragEnd(viewer, dragState, callbacksRef)) {
        dragState = null;
      }
    }, Cesium.ScreenSpaceEventType.LEFT_UP);

    return () => {
      handler.destroy();
      handlerRef.current = null;
    };
  }, [readyRef.current]);
}
