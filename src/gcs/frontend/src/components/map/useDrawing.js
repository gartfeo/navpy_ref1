import { useState, useCallback, useRef } from 'react';
import { closestEdgeIndex } from '../../utils/geo';
import { rectangleCorners, sideLength, MIN_RECT_SIDE_M } from '../../utils/rectangleDraw';

/** Search-area shapes the operator can draw. Both end as a plain polygon. */
export const DRAW_SHAPES = ['polygon', 'rectangle'];

/**
 * Polygon drawing/editing state management.
 * - Click to add vertices, double-click to finish
 * - Drag vertices/midpoints to reshape
 * - Double-click vertex to delete
 * - Drag inside polygon to move all vertices
 * - Undo removes last vertex, Clear resets everything
 * - Rectangle shape (while the zone has < 3 vertices): click two corners for
 *   one side, a third click sets the width; drawing then stops. Undo of the
 *   third click goes back to the two-corner draft.
 *
 * Uses refs for vertices/undoStack so callbacks can read current
 * values synchronously without nesting state setters inside updaters.
 */
export default function useDrawing({ setPolygon, analyze }) {
  const [isDrawing, setIsDrawing] = useState(false);
  const [vertices, setVertices] = useState([]);
  const [drawShape, setDrawShapeState] = useState('polygon');
  const verticesRef = useRef([]);
  const undoStackRef = useRef([]);

  // Helpers to update both ref and state in sync
  function updateVertices(next) {
    verticesRef.current = next;
    setVertices(next);
  }
  function pushUndo(entry) {
    undoStackRef.current = [...undoStackRef.current, entry];
  }

  const startDraw = useCallback(() => {
    setIsDrawing(true);
    if (verticesRef.current.length === 0) {
      undoStackRef.current = [];
      updateVertices([]);
      setPolygon([]);
    }
  }, [setPolygon, drawShape]);

  // Returns true when the click changed the zone (an ignored click must not
  // leave an undo entry behind).
  const onMapClick = useCallback(
    (latlon) => {
      if (!latlon) return false;

      const prev = verticesRef.current;
      let next, insertAt;

      if (drawShape === 'rectangle' && prev.length < 3) {
        if (prev.length === 1 && sideLength(prev[0], latlon) < MIN_RECT_SIDE_M) return false;
        if (prev.length === 2) {
          const corners = rectangleCorners(prev[0], prev[1], latlon);
          if (!corners) return false;
          pushUndo({ restore: prev });
          updateVertices(corners);
          setPolygon(corners);
          analyze(corners);
          setIsDrawing(false);
          return true;
        }
      }

      if (prev.length >= 3) {
        const idx = closestEdgeIndex(prev, latlon);
        insertAt = idx + 1;
        next = [...prev];
        next.splice(insertAt, 0, latlon);
      } else {
        insertAt = prev.length;
        next = [...prev, latlon];
      }

      pushUndo({ index: insertAt });
      updateVertices(next);
      setPolygon(next);
      if (next.length >= 3) {
        analyze(next);
      }
      return true;
    },
    [setPolygon, analyze, drawShape]
  );

  // Pick the shape for the next zone. A started draft (under 3 vertices)
  // belongs to the old shape's gesture, so it is dropped.
  const setDrawShape = useCallback((shape) => {
    if (!DRAW_SHAPES.includes(shape) || shape === drawShape) return;
    setDrawShapeState(shape);
    if (verticesRef.current.length > 0 && verticesRef.current.length < 3) {
      undoStackRef.current = [];
      updateVertices([]);
      setPolygon([]);
    }
  }, [setPolygon, drawShape]);

  const undo = useCallback(() => {
    const stack = undoStackRef.current;
    if (stack.length === 0) return;
    const last = stack[stack.length - 1];

    const prev = verticesRef.current;
    if (prev.length === 0) return;

    undoStackRef.current = stack.slice(0, -1);
    if (last.restore) {
      // Back to the two-corner rectangle draft: keep drawing its width.
      updateVertices(last.restore);
      setPolygon(last.restore);
      setIsDrawing(true);
      return;
    }

    const next = [...prev];
    next.splice(last.index, 1);

    updateVertices(next);
    setPolygon(next);
    if (next.length >= 3) {
      analyze(next);
    }
  }, [setPolygon, analyze]);

  const clear = useCallback(() => {
    undoStackRef.current = [];
    updateVertices([]);
    setPolygon([]);
    setIsDrawing(false);
  }, [setPolygon]);

  const stopDraw = useCallback(() => {
    setIsDrawing(false);
  }, []);

  const onVertexDrag = useCallback(
    (index, newLatLon) => {
      const prev = verticesRef.current;
      const next = [...prev];
      next[index] = newLatLon;
      updateVertices(next);
      setPolygon(next);
      if (next.length >= 3) {
        analyze(next);
      }
    },
    [setPolygon, analyze]
  );

  const onVertexDelete = useCallback(
    (index) => {
      const prev = verticesRef.current;
      if (prev.length <= 3) return; // keep at least 3 vertices
      const next = [...prev];
      next.splice(index, 1);
      updateVertices(next);
      setPolygon(next);
      if (next.length >= 3) {
        analyze(next);
      }
    },
    [setPolygon, analyze]
  );

  const onMidpointInsert = useCallback(
    (afterIndex, latlon) => {
      const prev = verticesRef.current;
      const next = [...prev];
      next.splice(afterIndex + 1, 0, latlon);
      updateVertices(next);
      setPolygon(next);
      if (next.length >= 3) {
        analyze(next);
      }
    },
    [setPolygon, analyze]
  );

  const onPolygonMove = useCallback(
    (dlat, dlon) => {
      const prev = verticesRef.current;
      const next = prev.map((v) => ({
        lat: v.lat + dlat,
        lon: v.lon + dlon,
      }));
      updateVertices(next);
      setPolygon(next);
      if (next.length >= 3) {
        analyze(next);
      }
    },
    [setPolygon, analyze]
  );

  // Sync internal vertices when polygon is loaded externally (file load)
  const loadVertices = useCallback((poly) => {
    undoStackRef.current = [];
    updateVertices(poly);
  }, []);

  // Restore vertices from a snapshot (drag undo) — preserves drawing undo stack
  const restoreVertices = useCallback((poly) => {
    updateVertices(poly);
    setPolygon(poly);
    if (poly.length >= 3) {
      analyze(poly);
    }
  }, [setPolygon, analyze]);

  return {
    isDrawing,
    vertices,
    drawShape,
    setDrawShape,
    canUndo: undoStackRef.current.length > 0,
    startDraw,
    stopDraw,
    onMapClick,
    undo,
    clear,
    loadVertices,
    restoreVertices,
    onVertexDrag,
    onVertexDelete,
    onMidpointInsert,
    onPolygonMove,
  };
}
