import { useCallback, useEffect, useRef, useState } from 'react';
import {
  isFullscreenActive,
  isFullscreenSupported,
  isInstalledApp,
  toggleFullscreen,
} from '../utils/fullscreen';

const warnFailed = (err) => console.warn('Fullscreen toggle failed:', err);

// Tracks document fullscreen, including exits by the Android back gesture.
// In the installed app, the operator's next tap re-enters fullscreen until
// they leave it with the button.
export default function useFullscreen() {
  const supported = isFullscreenSupported(document);
  const [active, setActive] = useState(() => isFullscreenActive(document));
  const installedRef = useRef(supported && isInstalledApp(window));
  const autoEnterRef = useRef(installedRef.current);
  // One request at a time: a tap on the button reaches both the button's
  // handler and the document listener while the first request is pending.
  const pendingRef = useRef(false);

  const run = useCallback(() => {
    if (pendingRef.current) return;
    pendingRef.current = true;
    toggleFullscreen(document)
      .catch(warnFailed)
      .finally(() => { pendingRef.current = false; });
  }, []);

  useEffect(() => {
    const onChange = () => setActive(isFullscreenActive(document));
    document.addEventListener('fullscreenchange', onChange);
    return () => document.removeEventListener('fullscreenchange', onChange);
  }, []);

  useEffect(() => {
    if (!supported) return undefined;
    // A file chooser opens only with the tap's user activation, which
    // requestFullscreen consumes. So the listener runs in the bubble phase,
    // after the tapped control's own handler, and skips the click on the file
    // input itself (Import CSV clicks a mounted input from its handler, and
    // that nested click reaches the document before the chooser opens); the
    // outer tap re-enters afterwards. click, not pointerdown: the tap's
    // target is already resolved, so hiding the bars cannot move it.
    const onClick = (event) => {
      if (event.target?.matches?.('input[type="file"]')) return;
      if (autoEnterRef.current && !isFullscreenActive(document)) run();
    };
    document.addEventListener('click', onClick);
    return () => document.removeEventListener('click', onClick);
  }, [supported, run]);

  const toggle = useCallback(() => {
    // Leaving with the button is deliberate: stop re-entering on taps.
    autoEnterRef.current = !isFullscreenActive(document) && installedRef.current;
    run();
  }, [run]);

  return { supported, active, toggle };
}
