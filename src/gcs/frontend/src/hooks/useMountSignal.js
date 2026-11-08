import { useCallback, useEffect, useRef } from 'react';

/**
 * `() => AbortSignal` that aborts when the calling component unmounts. A
 * control hands it to whatever it starts (a confirm, a launch), so that work
 * is withdrawn with the control that began it.
 */
export default function useMountSignal() {
  const controller = useRef(null);
  useEffect(() => {
    const c = new AbortController();
    controller.current = c;
    return () => c.abort();
  }, []);
  return useCallback(() => controller.current?.signal, []);
}

