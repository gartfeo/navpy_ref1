import { useSyncExternalStore, useRef } from 'react';
import store from '../stores/telemetryStore';

/**
 * Subscribe to a selected slice of the telemetry store.
 * Re-renders only when the selected value changes (via Object.is).
 */
export function useTelemetryStore(selector) {
  return useSyncExternalStore(
    store.subscribe,
    () => selector(store),
  );
}

/**
 * Returns a stable ref to the telemetry store singleton.
 * Never triggers re-renders — use in CesiumMap hooks that
 * read vehicle data inside effects/callbacks.
 */
export function useTelemetryRef() {
  const ref = useRef(store);
  return ref;
}

/**
 * Returns the vehicle list array. Re-renders only when the set of
 * connected sys_ids changes (vehicle added/removed), not on every
 * position/attitude update.
 */
export function useVehicleList() {
  useSyncExternalStore(store.subscribe, () => store.getSysIdKey());
  return store.getVehicleList();
}
