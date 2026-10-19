import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import useHeldSeconds from '../../hooks/useHeldSeconds';
import { HEADER_FONT_PX, PANEL_MARGIN_PX } from '../../utils/vehicleWindowLayout';

/**
 * Always-on label of RC FPV, the synthetic first-person view on the main map:
 * a frame in the accent colour (not a warning colour), a fixed top-centre tag
 * "SYNTHETIC · UAV n · nose view, not camera", and while the camera holds its
 * last pose a full-width "View held" banner with the time held. Never takes
 * taps: everything here is pointer-events none.
 *
 * @param {{ label: string, holdReason: string|null, top: number }} props
 */
export default function FpvViewOverlay({ label, holdReason, top }) {
  const { t } = useTranslation();
  const heldSeconds = useHeldSeconds(holdReason);
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
          {t(`fpvView.held.${holdReason}`, { seconds: heldSeconds })}
        </div>
      )}
    </div>
  );
}
