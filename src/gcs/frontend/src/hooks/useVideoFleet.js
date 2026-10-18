import { useRef, useSyncExternalStore } from 'react';
import { streamsOf, vehicleLabel } from '../utils/videoStreams';
import { fpvHoldReason } from '../utils/fpvView';

/**
 * Per-vehicle facts the camera panel and the FPV toggle need, in the fleet's
 * own order (the index is the UAV's zone colour, as elsewhere in the GCS).
 * Pure, exported for tests.
 */
export function summarizeFleet(vehicleList) {
  return (vehicleList || []).map((v, index) => {
    const holdReason = fpvHoldReason(v);
    return {
      sysId: v.sys_id,
      index,
      label: vehicleLabel(v, v.sys_id),
      streams: streamsOf(v),
      linkOk: v.link_ok === true,
      fpvEligible: holdReason === null,
      holdReason,
    };
  });
}

/**
 * Subscribes to the telemetry store, but re-renders only when one of the facts
 * above changes, not on every 5 Hz position update.
 *
 * @param {{ current: object }} storeRef - the telemetry store (subscribe, getSnapshot, getVehicleList).
 */
export default function useVideoFleet(storeRef) {
  const cacheRef = useRef({ version: null, signature: null, fleet: [] });
  const store = storeRef.current;
  return useSyncExternalStore(store.subscribe, () => {
    const cache = cacheRef.current;
    const version = store.getSnapshot();
    if (cache.version === version) return cache.fleet;
    cache.version = version;
    const fleet = summarizeFleet(store.getVehicleList());
    const signature = JSON.stringify(fleet);
    if (signature !== cache.signature) {
      cache.signature = signature;
      cache.fleet = fleet;
    }
    return cache.fleet;
  });
}
