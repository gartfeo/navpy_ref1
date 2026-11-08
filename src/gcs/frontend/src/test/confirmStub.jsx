// Test helper: an in-page confirm (components/ConfirmProvider) that answers
// `answer` without a dialog. Render with `wrapper` (render or renderHook);
// `confirm` records the messages asked.
import React from 'react';
import { vi } from 'vitest';
import { ConfirmContext } from '../components/ConfirmProvider';

export function confirmStub(answer) {
  // Records only the message: useConfirm also passes its unmount signal.
  const confirm = vi.fn();
  const ask = (message) => { confirm(message); return Promise.resolve(answer); };
  const wrapper = ({ children }) => <ConfirmContext.Provider value={ask}>{children}</ConfirmContext.Provider>;
  return { confirm, wrapper };
}
