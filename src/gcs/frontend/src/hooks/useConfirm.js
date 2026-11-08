import { useCallback, useContext, useEffect, useRef } from 'react';
import { ConfirmContext } from '../components/ConfirmProvider';

/**
 * `confirm(message) => Promise<boolean>` from the nearest ConfirmProvider
 * (never window.confirm). A question the calling component asked is
 * withdrawn, and resolves false, when that component unmounts: the action it
 * was about no longer has a place on screen (a force start whose button left
 * with the phase change), so Confirm must not run it.
 */
export default function useConfirm() {
  const confirm = useContext(ConfirmContext);
  const mounted = useRef(null);
  useEffect(() => {
    const controller = new AbortController();
    mounted.current = controller;
    return () => controller.abort();
  }, []);
  return useCallback((message) => confirm(message, { signal: mounted.current?.signal }), [confirm]);
}
