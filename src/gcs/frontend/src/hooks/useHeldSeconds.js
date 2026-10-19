import { useEffect, useState } from 'react';
import { VIDEO_CLOCK_TICK_MS } from '../constants/video';

/**
 * Whole seconds a first-person view has held its last pose for `holdReason`
 * (null = not held). Restarts at 0 when the reason changes; ticks once per
 * second like the camera's "N s" counters.
 */
export default function useHeldSeconds(holdReason) {
  const [heldSince, setHeldSince] = useState(null);
  const [now, setNow] = useState(() => Date.now());

  useEffect(() => {
    if (!holdReason) {
      setHeldSince(null);
      return undefined;
    }
    setHeldSince(Date.now());
    setNow(Date.now());
    const id = setInterval(() => setNow(Date.now()), VIDEO_CLOCK_TICK_MS);
    return () => clearInterval(id);
  }, [holdReason]);

  return heldSince == null ? 0 : Math.max(0, Math.floor((now - heldSince) / 1000));
}
