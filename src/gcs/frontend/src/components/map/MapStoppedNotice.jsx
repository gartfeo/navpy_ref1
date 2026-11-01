import React, { useLayoutEffect, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';

export function reloadPage() {
  window.location.reload();
}

/** The whole card is visible inside its own box: nothing overflows it. */
function cardFits(card) {
  return card.scrollHeight <= card.clientHeight && card.scrollWidth <= card.clientWidth;
}

function WarningIcon() {
  return (
    <svg width="16" height="16" viewBox="0 0 16 16" fill="none" style={{ flexShrink: 0 }}>
      <path d="M8 1.8L14.8 13.4a1 1 0 01-.86 1.5H2.06a1 1 0 01-.86-1.5L8 1.8z" stroke={colors.warning} strokeWidth="1.3" strokeLinejoin="round"/>
      <path d="M8 6v3.2" stroke={colors.warning} strokeWidth="1.3" strokeLinecap="round"/>
      <circle cx="8" cy="11.6" r="0.7" fill={colors.warning}/>
    </svg>
  );
}

function ReloadIcon() {
  return (
    <svg width="14" height="14" viewBox="0 0 16 16" fill="none" style={{ flexShrink: 0 }}>
      <path d="M13.5 8a5.5 5.5 0 11-1.6-3.9" stroke={colors.accent} strokeWidth="1.5" strokeLinecap="round"/>
      <path d="M12.2 1.6v2.8H9.4" stroke={colors.accent} strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
    </svg>
  );
}

const titleStyle = {
  display: 'flex', alignItems: 'center', gap: 6,
  color: colors.warning, fontSize: 14, fontWeight: 700, whiteSpace: 'nowrap',
};

/**
 * Over the map box once the map has stopped for good (useCesiumViewer
 * `mapStopped`: WebGL context lost or a render-loop error). The card says what
 * to do and offers Reload. It shows only when all of it fits the box; it is
 * always laid out and measured, and hidden with `visibility` (keeps its size,
 * takes no focus or clicks) when it does not fit. Then only the title shows,
 * so Reload never comes without the warning that unsaved plan edits are lost.
 * The layer lets clicks through to the map box (the small map's
 * click-to-enlarge); only Reload takes them.
 * Design: docs/design/gcs_map_stopped_notice.md.
 *
 * @param {{ onReload?: () => void }} props
 */
export default function MapStoppedNotice({ onReload = reloadPage }) {
  const { t } = useTranslation();
  const layerRef = useRef(null);
  const cardRef = useRef(null);
  const contentRef = useRef(null);
  const [fits, setFits] = useState(true);

  // Re-measure when the box resizes or the content changes size (text,
  // language, fonts).
  useLayoutEffect(() => {
    const measure = () => setFits(cardFits(cardRef.current));
    measure();
    if (typeof ResizeObserver === 'undefined') return undefined;
    const observer = new ResizeObserver(measure);
    observer.observe(layerRef.current);
    observer.observe(contentRef.current);
    return () => observer.disconnect();
  }, []);

  return (
    <div
      ref={layerRef}
      role="alert"
      style={{
        position: 'absolute', inset: 0, zIndex: 5, display: 'flex',
        alignItems: 'center', justifyContent: 'center', padding: 8, boxSizing: 'border-box',
        background: `${colors.bg}b8`, pointerEvents: 'none',
      }}
    >
      <div
        ref={cardRef}
        data-testid="map-stopped-card"
        aria-hidden={!fits}
        style={{
          visibility: fits ? 'visible' : 'hidden',
          maxWidth: 360, maxHeight: '100%', boxSizing: 'border-box', overflow: 'hidden',
          padding: '12px 16px', background: colors.bgLight,
          border: `1px solid ${colors.warning}`, borderRadius: 6,
        }}
      >
        <div
          ref={contentRef}
          data-testid="map-stopped-content"
          style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 8, textAlign: 'center' }}
        >
          <div style={titleStyle}><WarningIcon /><span>{t('mapStopped.title')}</span></div>
          <div style={{ fontSize: 12, lineHeight: 1.4, color: colors.textDim }}>{t('mapStopped.line')}</div>
          <button
            type="button"
            onClick={onReload}
            style={{
              display: 'flex', alignItems: 'center', gap: 6, flexShrink: 0,
              padding: '6px 14px', borderRadius: 4, border: `1px solid ${colors.accent}`,
              background: colors.surface, color: colors.accent, fontSize: 13, fontWeight: 700,
              whiteSpace: 'nowrap', cursor: 'pointer', pointerEvents: 'auto',
            }}
          >
            <ReloadIcon />{t('mapStopped.reload')}
          </button>
        </div>
      </div>
      {!fits && (
        <div
          style={{
            position: 'absolute', maxWidth: 'calc(100% - 16px)', boxSizing: 'border-box',
            padding: '6px 8px', background: colors.bgLight,
            border: `1px solid ${colors.warning}`, borderRadius: 6, overflow: 'hidden',
          }}
        >
          <div style={{ ...titleStyle, maxWidth: '100%' }}>
            <WarningIcon />
            <span style={{ overflow: 'hidden', textOverflow: 'ellipsis' }}>{t('mapStopped.title')}</span>
          </div>
        </div>
      )}
    </div>
  );
}
