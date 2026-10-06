// Vitest setup for the React component tests (loaded via vite.config.js
// `test.setupFiles`). Keeps every test deterministic and isolated.
import '@testing-library/jest-dom/vitest';
import { afterEach, vi } from 'vitest';
import { cleanup } from '@testing-library/react';

// Unmount React trees and restore any spies/fake timers between tests so a
// leaked fake clock or window.confirm spy can't bleed into the next test.
afterEach(() => {
  cleanup();
  vi.restoreAllMocks();
  vi.useRealTimers();
});

// react-i18next stub: components under test call useTranslation() only for
// labels. Returning the key keeps queries deterministic without pulling in
// language detection / localStorage / resource loading. Translation
// correctness stays covered by the existing locale tests.
vi.mock('react-i18next', () => ({
  useTranslation: () => ({
    t: (key) => key,
    i18n: { language: 'en', changeLanguage: () => Promise.resolve() },
  }),
  initReactI18next: { type: '3rdParty', init: () => {} },
}));
