import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { render, fireEvent } from '@testing-library/react';
import { useRef } from 'react';
import useFullscreen from './useFullscreen';

// jsdom has no Fullscreen API and no matchMedia: stub an installed app
// (display-mode standalone) that is not in fullscreen.
let requestFullscreen;
function stubInstalledApp({ standalone = true } = {}) {
  requestFullscreen = vi.fn(() => Promise.resolve());
  Object.defineProperty(document, 'fullscreenEnabled', { value: true, configurable: true });
  Object.defineProperty(document, 'fullscreenElement', { value: null, configurable: true });
  document.documentElement.requestFullscreen = requestFullscreen;
  window.matchMedia = vi.fn((q) => ({ matches: standalone && q === '(display-mode: standalone)' }));
}

afterEach(() => {
  delete document.fullscreenEnabled;
  delete document.fullscreenElement;
  delete document.documentElement.requestFullscreen;
  delete window.matchMedia;
});

// Import CSV in Settings -> Targets: a button that clicks a mounted file input.
function Probe() {
  useFullscreen();
  const fileRef = useRef(null);
  return (
    <div>
      <button onClick={() => fileRef.current.click()}>Import CSV</button>
      <input ref={fileRef} type="file" data-testid="file" style={{ display: 'none' }} />
      <span>map</span>
    </div>
  );
}

describe('useFullscreen re-entry in the installed app', () => {
  beforeEach(() => stubInstalledApp());

  it('re-enters fullscreen on a plain tap', () => {
    const { getByText } = render(<Probe />);
    fireEvent.click(getByText('map'));
    expect(requestFullscreen).toHaveBeenCalledTimes(1);
  });

  it('leaves a file input click alone', () => {
    // Chrome opens the file chooser only with the tap's user activation;
    // a fullscreen request during this click would use it up first.
    const { getByTestId } = render(<Probe />);
    fireEvent.click(getByTestId('file'));
    expect(requestFullscreen).not.toHaveBeenCalled();
  });

  it('re-enters only after the file chooser click of the same tap', () => {
    const { getByText } = render(<Probe />);
    // Registered after the hook's listener, so it runs right after it.
    let requestsDuringFileClick = null;
    const afterHook = (e) => {
      if (e.target.type === 'file') requestsDuringFileClick = requestFullscreen.mock.calls.length;
    };
    document.addEventListener('click', afterHook);
    try {
      fireEvent.click(getByText('Import CSV'));
    } finally {
      document.removeEventListener('click', afterHook);
    }
    expect(requestsDuringFileClick).toBe(0);
    expect(requestFullscreen).toHaveBeenCalledTimes(1);
  });
});

describe('useFullscreen in a browser tab', () => {
  beforeEach(() => stubInstalledApp({ standalone: false }));

  it('never re-enters on its own', () => {
    const { getByText } = render(<Probe />);
    fireEvent.click(getByText('map'));
    expect(requestFullscreen).not.toHaveBeenCalled();
  });
});
