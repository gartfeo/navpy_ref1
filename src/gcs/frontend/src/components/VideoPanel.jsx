import React, { useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors, zoneColorsSolid } from '../styles';
import useVideoFeed from '../hooks/useVideoFeed';
import useElementSize from '../hooks/useElementSize';
import { FEED_FAILURE, FEED_STATUS } from '../utils/videoFeed';
import { VIDEO_FIRST_FRAME_DEADLINE_S, VIDEO_STALL_DEADLINE_S } from '../constants/video';
import {
  DEFAULT_TOP_CONTROLS_BOTTOM_PX, HEADER_FONT_PX, HEADER_GAP_PX, HEADER_PADDING_PX,
  PANEL_MARGIN_PX, STATE_PX, TITLE_MIN_PX, TOUCH_TARGET_PX, videoPanelLayout,
} from '../utils/videoPanelLayout';

// Below the map overlay controls (z-index 10) and the confirmation cards
// (z-index 20): operator controls always stay on top of the video.
export const Z_INDEX = 9;

/** Desktop map pane, used until the real pane is measured. */
const FALLBACK_PANE = { width: 1280, height: 800 };

const STATE_TONE = {
  [FEED_STATUS.CONNECTING]: colors.textDim,
  [FEED_STATUS.LIVE]: colors.success,
  [FEED_STATUS.STALE]: colors.warning,
  [FEED_STATUS.FAILED]: colors.error,
  hidden: colors.textDim,
  lost: colors.error,
};

const REASON = {
  [FEED_FAILURE.NO_FIRST_FRAME]: (t) => t('videoPanel.reason.noFirstFrame', { seconds: VIDEO_FIRST_FRAME_DEADLINE_S }),
  [FEED_FAILURE.STALLED]: (t) => t('videoPanel.reason.stalled', { seconds: VIDEO_STALL_DEADLINE_S }),
  [FEED_FAILURE.CODEC]: (t, stream) => t('videoPanel.reason.codec', { codec: stream?.encoding ?? '' }),
  [FEED_FAILURE.CONNECTION]: (t) => t('videoPanel.reason.connection'),
  notAdvertised: (t) => t('videoPanel.reason.notAdvertised'),
};

const buttonStyle = {
  minHeight: TOUCH_TARGET_PX,
  minWidth: TOUCH_TARGET_PX,
  padding: '0 12px',
  borderRadius: 6,
  fontSize: HEADER_FONT_PX,
  fontWeight: 600,
  cursor: 'pointer',
  border: `1px solid ${colors.border}`,
  background: colors.surface,
  color: colors.textBright,
  touchAction: 'manipulation',
};

function Badge({ entry, selected, width, label, onSelect }) {
  const fill = zoneColorsSolid[entry.index % zoneColorsSolid.length];
  return (
    <button
      type="button"
      aria-pressed={selected}
      aria-label={entry.label}
      onClick={() => onSelect(entry.sysId)}
      style={{
        ...buttonStyle,
        width,
        minWidth: width,
        padding: '0 6px',
        flexShrink: 0,
        overflow: 'hidden',
        textOverflow: 'ellipsis',
        whiteSpace: 'nowrap',
        // Selected: fill + outline + label (never colour alone).
        background: selected ? fill : 'transparent',
        outline: selected ? `2px solid ${colors.textBright}` : 'none',
        outlineOffset: -2,
        fontWeight: selected ? 800 : 600,
      }}
    >
      {label === 'number' ? entry.sysId : entry.label}
    </button>
  );
}

function Header({ fleetWithVideo, selectedSysId, layout, title, stateKey, onSelectUav }) {
  const { t } = useTranslation();
  const [menuOpen, setMenuOpen] = useState(false);
  const overflow = layout.overflow.map((i) => fleetWithVideo[i]);
  return (
    <header
      style={{
        position: 'relative',
        display: 'flex',
        alignItems: 'center',
        gap: HEADER_GAP_PX,
        padding: HEADER_PADDING_PX,
        fontSize: HEADER_FONT_PX,
        fontWeight: 600,
        color: colors.textBright,
        background: colors.bgLight,
        whiteSpace: 'nowrap',
      }}
    >
      {layout.visible.map((i) => (
        <Badge
          key={fleetWithVideo[i].sysId}
          entry={fleetWithVideo[i]}
          selected={fleetWithVideo[i].sysId === selectedSysId}
          width={layout.badgeWidth}
          label={layout.badgeLabel}
          onSelect={onSelectUav}
        />
      ))}
      {overflow.length > 0 && (
        <button
          type="button"
          aria-haspopup="true"
          aria-expanded={menuOpen}
          aria-label={t('videoPanel.moreUavsLabel')}
          onClick={() => setMenuOpen((v) => !v)}
          style={{ ...buttonStyle, padding: '0 6px', flexShrink: 0 }}
        >
          {t('videoPanel.moreUavs', { count: overflow.length })}
        </button>
      )}
      {menuOpen && overflow.length > 0 && (
        <div
          role="menu"
          style={{
            position: 'absolute',
            bottom: '100%',
            left: 0,
            display: 'flex',
            flexDirection: 'column',
            gap: HEADER_GAP_PX,
            padding: HEADER_PADDING_PX,
            background: colors.bgLight,
            border: `1px solid ${colors.border}`,
            borderRadius: 6,
          }}
        >
          {overflow.map((entry) => (
            <Badge
              key={entry.sysId}
              entry={entry}
              selected={entry.sysId === selectedSysId}
              width="auto"
              label="full"
              onSelect={(sysId) => { setMenuOpen(false); onSelectUav(sysId); }}
            />
          ))}
        </div>
      )}
      <span
        data-testid="video-title"
        style={{ flex: 1, minWidth: TITLE_MIN_PX, overflow: 'hidden', textOverflow: 'ellipsis' }}
      >
        {title}
      </span>
      {stateKey && (
        <span
          role="status"
          style={{
            width: STATE_PX,
            flexShrink: 0,
            textAlign: 'right',
            overflow: 'hidden',
            textOverflow: 'ellipsis',
            color: STATE_TONE[stateKey],
            fontWeight: 700,
          }}
        >
          {stateKey === FEED_STATUS.LIVE && <span aria-hidden="true">● </span>}
          {t(`videoPanel.state.${stateKey}`)}
        </span>
      )}
    </header>
  );
}

function Row({ children, tone }) {
  return (
    <div
      style={{
        display: 'flex',
        flexWrap: 'wrap',
        alignItems: 'center',
        gap: HEADER_GAP_PX * 2,
        padding: HEADER_PADDING_PX,
        fontSize: HEADER_FONT_PX,
        color: tone ?? colors.text,
        background: colors.bgLight,
        borderTop: `1px solid ${colors.border}`,
      }}
    >
      {children}
    </div>
  );
}

/** "Vehicle link lost" first, then "Camera video lost" with its reason and actions. */
function LossRows({ label, linkLost, videoLostReason, onReconnect, reconnectDisabled, onDismiss }) {
  const { t } = useTranslation();
  return (
    <>
      {linkLost && (
        <Row tone={colors.error}>
          <span role="alert">{t('videoPanel.linkLost', { name: label })}</span>
        </Row>
      )}
      {videoLostReason && (
        <Row tone={colors.warning}>
          <span role="alert" style={{ flexBasis: '100%' }}>
            {t('videoPanel.videoLost', { name: label })} — {videoLostReason}
          </span>
          <button type="button" style={buttonStyle} onClick={onReconnect} disabled={reconnectDisabled}>
            {t('videoPanel.reconnect')}
          </button>
          <button type="button" style={buttonStyle} onClick={onDismiss}>
            {t('videoPanel.dismiss')}
          </button>
        </Row>
      )}
    </>
  );
}

/** One live session; remounted (new key) for every fresh connection. */
function LiveFeed({ header, stream, label, linkLost, expanded, maxHeight, onToggleExpanded, onCollapse, onReconnect, onDismiss }) {
  const { t } = useTranslation();
  const feed = useVideoFeed(stream);
  const failed = feed.status === FEED_STATUS.FAILED;
  return (
    <>
      {header(feed.status)}
      <LossRows
        label={label}
        linkLost={linkLost}
        videoLostReason={failed ? REASON[feed.reason](t, stream) : null}
        onReconnect={onReconnect}
        onDismiss={onDismiss}
      />
      {!failed && (
        <>
          <div style={{ position: 'relative', background: '#000', minHeight: 0 }}>
            {/* Muted + playsInline so the WebView autoplays it; contain, because
                cropping a tracking overlay would hide part of what it marks. */}
            <video
              ref={feed.videoRef}
              muted
              autoPlay
              playsInline
              aria-label={label}
              style={{
                display: 'block',
                width: '100%',
                aspectRatio: '16 / 9',
                maxHeight: expanded ? Math.max(0, maxHeight - 3 * (TOUCH_TARGET_PX + 2 * HEADER_PADDING_PX)) : undefined,
                objectFit: 'contain',
                background: '#000',
              }}
            />
            {feed.status === FEED_STATUS.CONNECTING && (
              <div style={pictureText(colors.textDim, true)}>{t('videoPanel.connectingUav', { name: label })}</div>
            )}
            {feed.status === FEED_STATUS.STALE && (
              <div style={pictureText(colors.warning, false)}>{t('videoPanel.noFrames', { seconds: feed.seconds })}</div>
            )}
          </div>
          <Row>
            <button type="button" style={buttonStyle} aria-pressed={expanded} onClick={onToggleExpanded}>
              {expanded ? t('videoPanel.restore') : t('videoPanel.expand')}
            </button>
            <button type="button" style={buttonStyle} onClick={onCollapse}>
              {t('videoPanel.collapse')}
            </button>
          </Row>
        </>
      )}
    </>
  );
}

const pictureText = (color, centered) => ({
  position: 'absolute',
  ...(centered ? { inset: 0, display: 'flex', alignItems: 'center', justifyContent: 'center' } : { top: 6, left: 6 }),
  padding: centered ? 8 : '4px 8px',
  fontSize: HEADER_FONT_PX,
  fontWeight: 700,
  color,
  background: centered ? 'rgba(15, 20, 35, 0.72)' : colors.bgLight,
  borderRadius: centered ? 0 : 4,
});

/**
 * Monitor-view camera panel, bottom-left of the map: one feed at a time
 * (docs/design/gcs_video_streams.md, section 5). Selection lives in
 * useVideoSelection; this component only shows it. The parent mounts it only
 * in the monitor view, outside FPV and manual control, while there is
 * something to show.
 */
export default function VideoPanel({
  fleet, selection, shown, session, collapsed, onCollapse,
  onSelectUav, onReconnect, onDismiss,
  topControlsBottom = DEFAULT_TOP_CONTROLS_BOTTOM_PX,
}) {
  const { t } = useTranslation();
  const anchorRef = useRef(null);
  const pane = useElementSize(anchorRef, FALLBACK_PANE, (el) => el.parentElement);
  const [expanded, setExpanded] = useState(false);

  const fleetWithVideo = fleet.filter((e) => e.streams.length > 0);
  const selectedSysId = selection?.sysId ?? null;
  const layout = videoPanelLayout({
    paneWidth: pane.width,
    paneHeight: pane.height,
    topControlsBottom,
    badgeCount: fleetWithVideo.length,
    selectedIndex: fleetWithVideo.findIndex((e) => e.sysId === selectedSysId),
    expanded: expanded && shown.kind === 'stream' && !collapsed,
  });

  const entry = shown.entry ?? null;
  const label = entry?.label ?? (selectedSysId != null ? `UAV ${selectedSysId}` : '');
  const linkLost = selectedSysId != null && !entry?.linkOk;
  const streamName = shown.stream?.name;
  const title = selectedSysId == null
    ? t('videoPanel.title')
    : (streamName ? `${label} · ${streamName}` : label);
  const header = (stateKey) => (
    <Header
      fleetWithVideo={fleetWithVideo}
      selectedSysId={selectedSysId}
      layout={layout}
      title={title}
      stateKey={stateKey}
      onSelectUav={onSelectUav}
    />
  );

  let body;
  if (shown.kind === 'stream' && !collapsed) {
    body = (
      <LiveFeed
        key={session}
        header={header}
        stream={shown.stream}
        label={label}
        linkLost={linkLost}
        expanded={expanded}
        maxHeight={layout.maxHeight}
        onToggleExpanded={() => setExpanded((v) => !v)}
        onCollapse={() => onCollapse(true)}
        onReconnect={onReconnect}
        onDismiss={onDismiss}
      />
    );
  } else if (shown.kind === 'stream') {
    body = (
      <>
        {header('hidden')}
        <LossRows label={label} linkLost={linkLost} />
        <Row>
          <button type="button" style={buttonStyle} onClick={() => onCollapse(false)}>
            {t('videoPanel.show')}
          </button>
        </Row>
      </>
    );
  } else if (shown.kind === 'lost') {
    body = (
      <>
        {header('lost')}
        <LossRows
          label={label}
          linkLost={linkLost}
          videoLostReason={REASON.notAdvertised(t)}
          onReconnect={onReconnect}
          reconnectDisabled
          onDismiss={onDismiss}
        />
      </>
    );
  } else {
    body = (
      <>
        {header(null)}
        <Row tone={colors.textDim}>{t('videoPanel.pickUav')}</Row>
      </>
    );
  }

  return (
    <section
      ref={anchorRef}
      aria-label={t('videoPanel.region')}
      style={{
        position: 'absolute',
        bottom: PANEL_MARGIN_PX,
        left: PANEL_MARGIN_PX,
        zIndex: Z_INDEX,
        width: layout.width,
        maxHeight: layout.maxHeight,
        display: 'flex',
        flexDirection: 'column',
        borderRadius: 6,
        overflow: 'visible',
        border: `1px solid ${colors.border}`,
        background: colors.bgLight,
        pointerEvents: 'auto',
      }}
    >
      {body}
    </section>
  );
}
