import { useEffect, useRef } from 'react';
import { deriveLoadStages } from '../utils/loadStage';

/**
 * Per-vehicle load stage ('connecting' | 'downloading' | 'ready') for a vehicle
 * list, latching 'ready' (see deriveLoadStages).
 *
 * The latch's "ever-ready" set lives in a ref. Render only READS it (via
 * deriveLoadStages) and the new set is committed in an effect — matching the
 * codebase convention (useFullParams / useAasParams mutate their Set ref
 * post-commit, not during render), so there is no render-time side effect for
 * React 18 StrictMode / concurrent rendering to trip on.
 *
 * Returns a Map<sys_id, stage>.
 */
export default function useLoadStages(vehicleList, downloadingSysIds) {
  const readyRef = useRef(new Set());
  const { stages, ready } = deriveLoadStages(vehicleList, downloadingSysIds, readyRef.current);
  useEffect(() => {
    readyRef.current = ready;
  });
  return stages;
}
