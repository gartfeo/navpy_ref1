import React, { useEffect, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import { VIDEO_CLOCK_TICK_MS } from '../../constants/video';
import { HEADER_FONT_PX, PANEL_MARGIN_PX } from '../../utils/videoPanelLayout';

const HELD_KEY = {
  linkLost: 'fpvView.heldLinkLost',
  poseUnavailable: 'fpvView.heldPose',
};

/**
 * Always-on label of the synthetic first-person view (view FPV and RC FPV):
 * a frame in the accent colour (not a warning colour), a fixed top-centre tag
 * "SYNTHETIC · UAV n · nose view, not camera", and while the camera holds its
 * last pose a full-width "View held" banner with the time held. Never takes
 * taps: everything here is pointer-events none.
 *
 * @param {{ label: string, holdReason: string|null, top: number }} props
 */
export default function FpvViewOverlay({ label, holdReason, top }) {
  const { t } = useTranslation();
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

  const heldSeconds = heldSince == null ? 0 : Math.max(0, Math.floor((now - heldSince) / 1000));
  const text = {
    fontSize: HEADER_FONT_PX,
    fontWeight: 700,
    color: colors.textBright,
    background: colors.bgLight,
    padding: '6px 12px',
    borderRadius: 6,
  };

  return (
    <div aria-hidden={false} style={{ position: 'absolute', inset: 0, zIndex: 8, pointerEvents: 'none' }}>
      <div data-testid="fpv-frame" style={{ position: 'absolute', inset: 0, boxShadow: `inset 0 0 0 3px ${colors.accent}` }} />
      <div
        role="note"
        style={{
          ...text,
          position: 'absolute',
          top,
          left: '50%',
          transform: 'translateX(-50%)',
          border: `1px solid ${colors.accent}`,
          whiteSpace: 'nowrap',
          maxWidth: `calc(100% - ${2 * PANEL_MARGIN_PX}px)`,
          overflow: 'hidden',
          textOverflow: 'ellipsis',
        }}
      >
        {t('fpvView.tag', { name: label })}
      </div>
      {holdReason && (
        <div
          role="status"
          style={{
            ...text,
            position: 'absolute',
            top: top + 44,
            left: PANEL_MARGIN_PX,
            right: PANEL_MARGIN_PX,
            textAlign: 'center',
            border: `1px solid ${colors.warning}`,
            color: colors.warning,
          }}
        >
          {t(HELD_KEY[holdReason], { seconds: heldSeconds })}
        </div>
      )}
    </div>
  );
}
