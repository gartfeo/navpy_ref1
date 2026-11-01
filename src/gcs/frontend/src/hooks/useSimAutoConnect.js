import { useEffect, useRef } from 'react';

export const AUTO_CONNECT_ATTEMPTS = 20;
export const AUTO_CONNECT_RETRY_MS = 3000;

// Auto-connect on startup (sim mode): after settings load, discover + connect
// on the default device without waiting for a manual Connect click. SITL may
// not be up yet, so retry quietly until vehicles appear.
//
// The backend auto-connects at startup too, and discover skips vehicles that
// are already connected, so a scan finds 0 once the fleet is up. Stop as soon
// as the fleet is connected (vehicleCount > 0), not only when a scan finds
// vehicles; otherwise every open page scans for the whole retry budget.
//
// Depend only on stable primitives (not scan's identity, which churns while
// drawing) so the retry loop isn't cancelled mid-flight; read scan and the
// vehicle count via refs. Latches only AFTER the fleet is connected (or retries
// run out), not synchronously at the top: under React StrictMode (dev) the
// effect runs mount → cleanup → mount, and an up-front latch made the second
// mount bail so the retry loop never fired.
export default function useSimAutoConnect({ enabled, simMode, device, scan, vehicleCount }) {
  const doneRef = useRef(false);
  const scanRef = useRef(scan);
  scanRef.current = scan;
  const vehicleCountRef = useRef(vehicleCount);
  vehicleCountRef.current = vehicleCount;

  useEffect(() => {
    if (doneRef.current || !enabled || !simMode || !device) return undefined;
    let cancelled = false;
    let timer = null;
    let attempts = 0;
    const tryConnect = async () => {
      if (cancelled) return;
      if (vehicleCountRef.current > 0) {
        doneRef.current = true;
        return;
      }
      attempts += 1;
      let found = 0;
      try {
        const res = await scanRef.current(device);
        found = res?.found || 0;
      } catch { /* SITL not up yet */ }
      if (cancelled) return;
      if (found > 0 || attempts >= AUTO_CONNECT_ATTEMPTS) {
        doneRef.current = true;
        return;
      }
      timer = setTimeout(tryConnect, AUTO_CONNECT_RETRY_MS);
    };
    tryConnect();
    return () => { cancelled = true; if (timer) clearTimeout(timer); };
  }, [enabled, simMode, device]);
}
