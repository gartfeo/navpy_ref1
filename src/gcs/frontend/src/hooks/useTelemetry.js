import { useEffect, useRef } from 'react';
import store from '../stores/telemetryStore';
import { useVehicleList } from './useTelemetryStore';

/**
 * Thin wrapper that initializes the telemetry WebSocket connection
 * and returns store accessors. Vehicle state lives in the external
 * store — vehicleList only triggers re-renders on structural changes
 * (vehicle added/removed).
 */
export default function useTelemetry() {
  const storeRef = useRef(store);

  useEffect(() => {
    store.connect();
    return () => store.disconnect();
  }, []);

  const vehicleList = useVehicleList();

  return {
    vehicles: store.getVehicles(),
    connected: store.isConnected(),
    vehicleList,
    seenIds: store.getSeenIds(),
    removeVehicle: store.removeVehicle,
    messageHandlersRef: { current: store.getMessageHandlers() },
    sendWsMessage: store.sendWsMessage,
    storeRef,
  };
}
