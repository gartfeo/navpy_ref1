import { useCallback, useState } from 'react';
import { DOCKED_MIN_VIEWPORT_PX, panelOpenAtStart, sidePanelMode } from '../utils/appLayout';

/**
 * The right panel's one flag (docs/design/gcs_handheld_layout.md). Two
 * transitions only: the toggle flips it, and crossing up to the docking width
 * opens it. Crossing down keeps it, so a resize or rotation never hides the
 * panel: only the operator's own tap does.
 *
 * @param {number} viewportWidth
 * @returns {{open: boolean, mode: 'docked'|'overlay'|'collapsed', toggle: () => void}}
 */
export default function useSidePanel(viewportWidth) {
  const wide = viewportWidth >= DOCKED_MIN_VIEWPORT_PX;
  const [open, setOpen] = useState(() => panelOpenAtStart(viewportWidth));
  const [prevWide, setPrevWide] = useState(wide);
  if (wide !== prevWide) {
    setPrevWide(wide);
    if (wide) setOpen(true);
  }
  const toggle = useCallback(() => setOpen((v) => !v), []);
  return { open, mode: sidePanelMode(open, viewportWidth), toggle };
}
