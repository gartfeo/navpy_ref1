// Test helper: an in-page confirm (components/ConfirmProvider) that answers
// `answer` without a dialog. Render with `wrapper` (render or renderHook);
// `confirm` records the messages asked.
import React from 'react';
import { vi } from 'vitest';
import { ConfirmContext } from '../components/ConfirmProvider';

export function confirmStub(answer) {
  const confirm = vi.fn(() => Promise.resolve(answer));
  const wrapper = ({ children }) => <ConfirmContext.Provider value={confirm}>{children}</ConfirmContext.Provider>;
  return { confirm, wrapper };
}
