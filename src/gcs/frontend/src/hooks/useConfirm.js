import { useCallback, useContext } from 'react';
import { ConfirmContext } from '../components/ConfirmProvider';
import useMountSignal from './useMountSignal';

/**
 * `confirm(message, { signal, dialog }) => Promise<boolean | 'alt'>` from the nearest
 * ConfirmProvider (never window.confirm). The question is withdrawn, and
 * resolves false, when the calling component unmounts or when `signal`
 * aborts. Pass `signal` when the question belongs to a control other than the
 * hook's owner: the plan camera check asked by App for a Start or Launch
 * button goes with that button (opts.signal, useVehicleCommands). `dialog`
 * words the question and may add a third answer (ConfirmProvider).
 */
export default function useConfirm() {
  const confirm = useContext(ConfirmContext);
  const mountSignal = useMountSignal();
  return useCallback(
    (message, { signal, dialog } = {}) => confirm(message, { signals: [mountSignal(), signal], dialog }),
    [confirm, mountSignal],
  );
}
