import { useCallback, useRef } from 'react';
import { fenceVertexDrag, fenceMidpointInsert, fenceVertexDelete } from '../utils/fenceEdit';

/**
 * Operator editing of the inclusion-fence polygon — same gestures as the
 * search zone (drag a vertex, drag a midpoint to insert, double-click a vertex
 * to delete). The first gesture snapshots the auto-derived fence into
 * `fenceCustomVertices`; from then on the fence is operator-owned until
 * "Reset to auto".
 *
 * `fencePolygon` is whatever is currently rendered (custom ?? auto); a ref is
 * kept fresh each render so drag gestures read it synchronously.
 */
export default function useFenceEditing({
  fencePolygon, setFenceCustomVertices,
}) {
  const fencePolygonRef = useRef(fencePolygon || null);
  fencePolygonRef.current = fencePolygon || null;

  const handleFenceVertexDrag = useCallback((index, latlon) => {
    setFenceCustomVertices((prev) => fenceVertexDrag(prev, fencePolygonRef.current, index, latlon));
  }, [setFenceCustomVertices]);

  const handleFenceMidpointInsert = useCallback((afterIndex, latlon) => {
    setFenceCustomVertices((prev) => fenceMidpointInsert(prev, fencePolygonRef.current, afterIndex, latlon));
  }, [setFenceCustomVertices]);

  const handleFenceVertexDelete = useCallback((index) => {
    setFenceCustomVertices((prev) => fenceVertexDelete(prev, fencePolygonRef.current, index));
  }, [setFenceCustomVertices]);

  const handleFenceReset = useCallback(() => {
    setFenceCustomVertices(null);
  }, [setFenceCustomVertices]);

  return {
    handleFenceVertexDrag,
    handleFenceMidpointInsert,
    handleFenceVertexDelete,
    handleFenceReset,
  };
}
