import { useContext } from 'react';
import { ConfirmContext } from '../components/ConfirmProvider';

/** `confirm(message) => Promise<boolean>` from the nearest ConfirmProvider (never window.confirm). */
export default function useConfirm() {
  return useContext(ConfirmContext);
}
