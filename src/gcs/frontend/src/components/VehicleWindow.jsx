import React, { useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors, zoneColorsSolid } from '../styles';
import useVideoFeed from '../hooks/useVideoFeed';
import useHeldSeconds from '../hooks/useHeldSeconds';
import { SOURCE } from '../hooks/useVehicleWindow';
import { FEED_FAILURE, FEED_STATUS } from '../utils/videoFeed';
import { fleetEntryHoldReason } from '../utils/fpvView';
import { VIDEO_FIRST_FRAME_DEADLINE_S, VIDEO_STALL_DEADLINE_S } from '../constants/video';
import {
  HEADER_FONT_PX, HEADER_GAP_PX, HEADER_PADDING_PX, PANEL_MARGIN_PX, PICTURE_ASPECT,
  STATE_PX, TITLE_MIN_PX, TOUCH_TARGET_PX,
} from '../utils/vehicleWindowLayout';
import SyntheticView from './SyntheticView';

// Below the map overlay controls (z-index 10) and the confirmation cards
// (z-index 20): operator controls always stay on top of the window. While
// swapped the small map is drawn above the window (MAP_SWAPPED_Z_INDEX).
export const Z_INDEX = 9;
export const MAP_SWAPPED_Z_INDEX = Z_INDEX;
const SWAPPED_Z_INDEX = Z_INDEX - 1;

const STATE_TONE = {
  [FEED_STATUS.CONNECTING]: colors.textDim,
  [FEED_STATUS.LIVE]: colors.success,
  [FEED_STATUS.STALE]: colors.warning,
  [FEED_STATUS.FAILED]: colors.error,
  lost: colors.error,
  noCamera: colors.textDim,
  synthetic: colors.accent,
};

const REASON = {
  [FEED_FAILURE.NO_FIRST_FRAME]: (t) => t('vehicleWindow.reason.noFirstFrame', { seconds: VIDEO_FIRST_FRAME_DEADLINE_S }),
  [FEED_FAILURE.STALLED]: (t) => t('vehicleWindow.reason.stalled', { seconds: VIDEO_STALL_DEADLINE_S }),
  [FEED_FAILURE.CODEC]: (t, stream) => t('vehicleWindow.reason.codec', { codec: stream?.encoding ?? '' }),
  [FEED_FAILURE.CONNECTION]: (t) => t('vehicleWindow.reason.connection'),
  notAdvertised: (t) => t('vehicleWindow.reason.notAdvertised'),
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

const pressedStyle = (pressed) => ({
  ...buttonStyle,
  background: pressed ? colors.accent : colors.surface,
  color: pressed ? colors.bg : colors.textBright,
  fontWeight: pressed ? 800 : 600,
});

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

function Header({ fleet, selectedSysId, layout, title, stateKey, onSelectUav }) {
  const { t } = useTranslation();
  const [menuOpen, setMenuOpen] = useState(false);
  const overflow = layout.overflow.map((i) => fleet[i]);
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
        flexShrink: 0,
      }}
    >
      {layout.visible.map((i) => (
        <Badge
          key={fleet[i].sysId}
          entry={fleet[i]}
          selected={fleet[i].sysId === selectedSysId}
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
          aria-label={t('vehicleWindow.moreUavsLabel')}
          onClick={() => setMenuOpen((v) => !v)}
          style={{ ...buttonStyle, padding: '0 6px', flexShrink: 0 }}
        >
          {t('vehicleWindow.moreUavs', { count: overflow.length })}
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
          {t(`vehicleWindow.state.${stateKey}`)}
        </span>
      )}
    </header>
  );
}

/** The switch (left) and Maximize and Minimize (right); glyphs with names on a narrow pane. */
function ControlRow({ source, onSource, swapped, narrow, onToggleMaximized, onMinimize }) {
  const { t } = useTranslation();
  const sizeLabel = swapped ? t('vehicleWindow.restore') : t('vehicleWindow.maximize');
  return (
    <div
      style={{
        display: 'flex',
        alignItems: 'center',
        gap: HEADER_GAP_PX,
        padding: HEADER_PADDING_PX,
        background: colors.bgLight,
        borderTop: `1px solid ${colors.border}`,
        flexShrink: 0,
      }}
    >
      <div role="group" aria-label={t('vehicleWindow.source.group')} style={{ display: 'flex', gap: HEADER_GAP_PX }}>
        {[SOURCE.CAMERA, SOURCE.SYNTHETIC].map((s) => (
          <button key={s} type="button" aria-pressed={source === s} style={pressedStyle(source === s)} onClick={() => onSource(s)}>
            {t(`vehicleWindow.source.${s}`)}
          </button>
        ))}
      </div>
      <span style={{ flex: 1 }} />
      <button
        type="button"
        aria-pressed={swapped}
        aria-label={narrow ? sizeLabel : undefined}
        title={narrow ? sizeLabel : undefined}
        style={buttonStyle}
        onClick={onToggleMaximized}
      >
        {narrow ? (swapped ? '❐' : '□') : sizeLabel}
      </button>
      <button
        type="button"
        aria-label={narrow ? t('vehicleWindow.minimize') : undefined}
        title={narrow ? t('vehicleWindow.minimize') : undefined}
        style={buttonStyle}
        onClick={onMinimize}
      >
        {narrow ? '–' : t('vehicleWindow.minimize')}
      </button>
    </div>
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
        flexShrink: 0,
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
          <span role="alert">{t('vehicleWindow.linkLost', { name: label })}</span>
        </Row>
      )}
      {videoLostReason && (
        <Row tone={colors.warning}>
          <span role="alert" style={{ flexBasis: '100%' }}>
            {t('vehicleWindow.videoLost', { name: label })} — {videoLostReason}
          </span>
          <button type="button" style={buttonStyle} onClick={onReconnect} disabled={reconnectDisabled}>
            {t('vehicleWindow.reconnect')}
          </button>
          <button type="button" style={buttonStyle} onClick={onDismiss}>
            {t('vehicleWindow.dismiss')}
          </button>
        </Row>
      )}
    </>
  );
}

function HeldRow({ holdReason }) {
  const { t } = useTranslation();
  const seconds = useHeldSeconds(holdReason);
  if (!holdReason) return null;
  return (
    <Row tone={colors.warning}>
      <span role="status">{t(`fpvView.held.${holdReason}`, { seconds })}</span>
    </Row>
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

/** The picture box: 16:9 shrinking to the height limit; swapped, it fills the window. */
const pictureBox = (swapped) => (swapped
  ? { flex: '1 1 auto', minHeight: 0 }
  : { flex: '0 1 auto', minHeight: 0, width: '100%', aspectRatio: `${PICTURE_ASPECT}` });

/** Swapped and nothing to draw: the large area states it in text, never a black box. */
function StateArea({ text }) {
  return (
    <div style={{ ...pictureBox(true), position: 'relative', background: colors.bg }}>
      <div style={pictureText(colors.textDim, true)}>{text}</div>
    </div>
  );
}

/** One live camera session; remounted (new key) for every fresh connection. */
function CameraFeed({ chrome, stream, label, linkLost, swapped, onReconnect, onDismiss }) {
  const { t } = useTranslation();
  const feed = useVideoFeed(stream);
  const failed = feed.status === FEED_STATUS.FAILED;
  const reason = failed ? REASON[feed.reason](t, stream) : null;
  return (
    <>
      {chrome(feed.status)}
      <LossRows label={label} linkLost={linkLost} videoLostReason={reason} onReconnect={onReconnect} onDismiss={onDismiss} />
      {failed && swapped && <StateArea text={t('vehicleWindow.videoLost', { name: label })} />}
      {!failed && (
        <div style={{ ...pictureBox(swapped), position: 'relative', background: '#000' }}>
          {/* Muted + playsInline so the WebView autoplays it; contain, because
              cropping a tracking overlay would hide part of what it marks. */}
          <video
            ref={feed.videoRef}
            muted
            autoPlay
            playsInline
            aria-label={label}
            style={{ display: 'block', width: '100%', height: '100%', objectFit: 'contain', background: '#000' }}
          />
          {feed.status === FEED_STATUS.CONNECTING && (
            <div style={pictureText(colors.textDim, true)}>{t('vehicleWindow.connectingUav', { name: label })}</div>
          )}
          {feed.status === FEED_STATUS.STALE && (
            <div style={pictureText(colors.warning, false)}>{t('vehicleWindow.noFrames', { seconds: feed.seconds })}</div>
          )}
        </div>
      )}
    </>
  );
}

function handleState(source, shown, linkLost) {
  if (linkLost) return 'linkLost';
  if (source === SOURCE.SYNTHETIC) return 'synthetic';
  return shown.kind === 'stream' ? 'camera' : 'noCamera';
}

/**
 * The monitor view's vehicle window, bottom-left of the map pane
 * (docs/design/gcs_video_streams.md, section 5 "Vehicle window"): one UAV,
 * one source (Camera or Synthetic), normal, minimized to a handle, or
 * maximized (swapped with the map). State lives in useVehicleWindow; sizes
 * come from vehicleWindowLayout. The parent mounts it in the monitor view,
 * outside RC, whenever the fleet has a UAV.
 */
export default function VehicleWindow({
  fleet, win, layout, poseSourceRef, storeRef, ionTerrain, onSyntheticCreditsHeight,
}) {
  const { t } = useTranslation();
  const { sysId, shown, source, minimized, swapped } = win;

  const entry = fleet.find((e) => e.sysId === sysId) ?? null;
  const label = entry?.label ?? (sysId != null ? `UAV ${sysId}` : '');
  const linkLost = sysId != null && !entry?.linkOk;
  const box = layout.window;

  const section = (children, extra) => (
    <section
      aria-label={t('vehicleWindow.region')}
      style={{
        position: 'absolute',
        bottom: box.bottom,
        left: PANEL_MARGIN_PX,
        zIndex: swapped ? SWAPPED_Z_INDEX : Z_INDEX,
        width: box.width,
        height: box.height ?? undefined,
        maxHeight: box.maxHeight,
        display: 'flex',
        flexDirection: 'column',
        borderRadius: 6,
        overflow: 'visible',
        border: `1px solid ${colors.border}`,
        background: colors.bgLight,
        pointerEvents: 'auto',
        ...extra,
      }}
    >
      {children}
    </section>
  );

  if (minimized) {
    const state = t(`vehicleWindow.handle.${handleState(source, shown, linkLost)}`);
    return section(
      <button
        type="button"
        aria-label={t('vehicleWindow.handle.restoreLabel', { name: label, state })}
        onClick={win.restore}
        style={{ ...buttonStyle, border: 'none', background: colors.bgLight, textAlign: 'left', whiteSpace: 'nowrap' }}
      >
        {t('vehicleWindow.handle.label', { name: label, state })}
      </button>,
      { width: 'auto', maxWidth: box.width },
    );
  }

  const synthetic = source === SOURCE.SYNTHETIC;
  const streamName = shown.stream?.name;
  let title = streamName ? `${label} · ${streamName}` : label;
  if (synthetic) title = t('vehicleWindow.syntheticTitle', { name: label });

  const chrome = (stateKey) => (
    <>
      <Header
        fleet={fleet}
        selectedSysId={sysId}
        layout={layout}
        title={title}
        stateKey={stateKey}
        onSelectUav={win.selectUav}
      />
      <ControlRow
        source={source}
        onSource={win.chooseSource}
        swapped={swapped}
        narrow={layout.narrow}
        onToggleMaximized={win.toggleMaximized}
        onMinimize={win.minimize}
      />
      {win.notice && (
        <Row tone={colors.warning}>
          <span role="alert">{t(`vehicleWindow.${win.notice}`)}</span>
        </Row>
      )}
    </>
  );

  let body;
  if (synthetic) {
    body = (
      <>
        {chrome('synthetic')}
        <HeldRow holdReason={fleetEntryHoldReason(entry)} />
        <SyntheticView
          sysId={sysId}
          label={label}
          poseSourceRef={poseSourceRef}
          storeRef={storeRef}
          ionTerrain={ionTerrain}
          onFail={win.syntheticFailed}
          onCreditsHeight={onSyntheticCreditsHeight}
          style={pictureBox(swapped)}
        />
      </>
    );
  } else if (shown.kind === 'stream') {
    body = (
      <CameraFeed
        key={win.session}
        chrome={chrome}
        stream={shown.stream}
        label={label}
        linkLost={linkLost}
        swapped={swapped}
        onReconnect={win.reconnect}
        onDismiss={win.dismiss}
      />
    );
  } else if (shown.kind === 'lost') {
    body = (
      <>
        {chrome('lost')}
        <LossRows
          label={label}
          linkLost={linkLost}
          videoLostReason={REASON.notAdvertised(t)}
          onReconnect={win.reconnect}
          reconnectDisabled
          onDismiss={win.dismiss}
        />
        {swapped && <StateArea text={t('vehicleWindow.videoLost', { name: label })} />}
      </>
    );
  } else {
    body = (
      <>
        {chrome('noCamera')}
        <LossRows label={label} linkLost={linkLost} />
        <Row tone={colors.textDim}>{t('vehicleWindow.noCamera', { name: label })}</Row>
        {swapped && <StateArea text={t('vehicleWindow.noCamera', { name: label })} />}
      </>
    );
  }

  return section(body);
}
