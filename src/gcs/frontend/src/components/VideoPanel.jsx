import React, { useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../styles';
import useWhepStream, { WHEP_STATUS } from '../hooks/useWhepStream';
import { whepUrl } from '../utils/videoConfig';

// Docked size, 16:9. Expanded stays anchored to the same bottom-left corner and
// is capped so it can never reach the map overlay buttons along the top, the
// reset-view button in the bottom-right corner, or the sidebar.
const FIT_WIDTH = 240;
const FIT_HEIGHT = 135;

// Both widths are clamped to what the map pane actually offers: the panel sits
// at left 12px, so `calc(100% - 24px)` leaves the same margin on the right.
// Without it the docked 240px overflows a narrow pane.
const AVAILABLE_WIDTH = 'calc(100% - 24px)';
// Expanded floors at the docked width before the pane clamp, so enlarging can
// never make the panel narrower than it already was — 52% of a narrow pane is
// far below 240px. Both sides then take the same clamp, so at panes too narrow
// for 240px the two sizes meet instead of crossing over.
// Exported for the width-contract test: jsdom's CSS parser drops min()/max()
// outright, so a rendered element's style.width reads back empty there.
export const FIT_WIDTH_CSS = `min(${FIT_WIDTH}px, ${AVAILABLE_WIDTH})`;
export const EXPANDED_WIDTH = `min(max(52%, ${FIT_WIDTH}px), 640px, ${AVAILABLE_WIDTH})`;
const EXPANDED_MAX_HEIGHT = 'calc(100% - 120px)';

// Below the map overlay controls (z-index 10) and the confirmation cards
// (z-index 20): operator controls always stay on top of the video.
const Z_INDEX = 9;

const statusKey = {
  [WHEP_STATUS.CONNECTING]: 'videoPanel.connecting',
  [WHEP_STATUS.STALLED]: 'videoPanel.stalled',
  [WHEP_STATUS.ERROR]: 'videoPanel.error',
};

/**
 * The single Jetson tracking-overlay video feed.
 *
 * Renders nothing when no WHEP URL was configured at build time, so an
 * unconfigured operator build shows no dead panel. `url` is injectable for
 * tests; production reads the build-time value.
 */
export default function VideoPanel({ url = whepUrl() }) {
  const { t } = useTranslation();
  const [expanded, setExpanded] = useState(false);
  const { status, error, videoRef, retry } = useWhepStream(url);

  if (!url) return null;

  const title = t('videoPanel.title');
  const overlayKey = statusKey[status];

  return (
    <section
      aria-label={title}
      style={{
        position: 'absolute',
        bottom: 12,
        left: 12,
        zIndex: Z_INDEX,
        width: expanded ? EXPANDED_WIDTH : FIT_WIDTH_CSS,
        maxHeight: expanded ? EXPANDED_MAX_HEIGHT : undefined,
        display: 'flex',
        flexDirection: 'column',
        borderRadius: 6,
        overflow: 'hidden',
        border: `1px solid ${colors.border}`,
        background: 'rgba(22, 33, 62, 0.85)',
        backdropFilter: 'blur(4px)',
        pointerEvents: 'auto',
      }}
    >
      <header
        style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          gap: 6,
          padding: '4px 6px 4px 8px',
          fontSize: 11,
          fontWeight: 600,
          color: colors.textDim,
          letterSpacing: 0.4,
          textTransform: 'uppercase',
        }}
      >
        <span style={{ overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
          {title}
        </span>
        <button
          type="button"
          onClick={() => setExpanded((v) => !v)}
          title={expanded ? t('videoPanel.collapse') : t('videoPanel.expand')}
          aria-label={expanded ? t('videoPanel.collapse') : t('videoPanel.expand')}
          aria-pressed={expanded}
          style={{
            flexShrink: 0,
            width: 22,
            height: 22,
            display: 'inline-flex',
            alignItems: 'center',
            justifyContent: 'center',
            padding: 0,
            borderRadius: 4,
            cursor: 'pointer',
            border: `1px solid ${colors.border}`,
            background: 'transparent',
            color: colors.text,
          }}
        >
          <svg width="12" height="12" viewBox="0 0 12 12" fill="none" aria-hidden="true">
            {expanded ? (
              <path
                d="M5 1.5v3.5H1.5M7 10.5V7h3.5"
                stroke="currentColor"
                strokeWidth="1.3"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            ) : (
              <path
                d="M7.5 1.5h3v3M4.5 10.5h-3v-3"
                stroke="currentColor"
                strokeWidth="1.3"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            )}
          </svg>
        </button>
      </header>

      <div style={{ position: 'relative', background: '#000', minHeight: 0 }}>
        {/* Muted + playsInline so the WebView autoplays it; contain, because
            cropping a tracking overlay would hide part of what it marks. */}
        <video
          ref={videoRef}
          muted
          autoPlay
          playsInline
          aria-label={title}
          style={{
            display: 'block',
            width: '100%',
            height: expanded ? 'auto' : FIT_HEIGHT,
            maxHeight: expanded ? '100%' : FIT_HEIGHT,
            aspectRatio: expanded ? '16 / 9' : undefined,
            objectFit: 'contain',
            background: '#000',
          }}
        />
        {overlayKey && (
          <div
            role="status"
            style={{
              position: 'absolute',
              inset: 0,
              display: 'flex',
              flexDirection: 'column',
              alignItems: 'center',
              justifyContent: 'center',
              gap: 6,
              padding: 8,
              textAlign: 'center',
              fontSize: 11,
              color: status === WHEP_STATUS.CONNECTING ? colors.textDim : colors.warning,
              background: 'rgba(15, 20, 35, 0.72)',
            }}
          >
            <span>{t(overlayKey)}</span>
            {status === WHEP_STATUS.ERROR && (
              <button
                type="button"
                onClick={retry}
                title={error || undefined}
                style={{
                  padding: '3px 10px',
                  borderRadius: 4,
                  fontSize: 11,
                  fontWeight: 600,
                  cursor: 'pointer',
                  border: `1px solid ${colors.accent}`,
                  background: 'transparent',
                  color: colors.accent,
                }}
              >
                {t('videoPanel.retry')}
              </button>
            )}
          </div>
        )}
      </div>
    </section>
  );
}
