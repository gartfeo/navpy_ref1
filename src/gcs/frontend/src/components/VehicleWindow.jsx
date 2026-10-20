import React, { useCallback, useEffect, useLayoutEffect, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors, zoneColorsSolid } from '../styles';
import useVideoFeed from '../hooks/useVideoFeed';
import useHeldSeconds from '../hooks/useHeldSeconds';
import { SOURCE } from '../hooks/useVehicleWindow';
import { FEED_FAILURE, FEED_STATUS } from '../utils/videoFeed';
import { fleetEntryHoldReason } from '../utils/fpvView';
import { hasVideo } from '../utils/videoStreams';
import { VIDEO_FIRST_FRAME_DEADLINE_S, VIDEO_STALL_DEADLINE_S } from '../constants/video';
import {
  HEADER_FONT_PX, HEADER_GAP_PX, HEADER_PADDING_PX, PICTURE_ASPECT, PICTURE_TAG_INSET_PX,
  STATE_PX, TITLE_MIN_PX, TOUCH_TARGET_PX, WINDOW_BORDER_PX,
} from '../utils/vehicleWindowLayout';
import { observeHeight } from './map/utils/observeHeight';
import SyntheticView from './SyntheticView';

// Below the map overlay controls (z-index 10) and the confirmation cards
// (z-index 20): operator controls always stay on top of the window. While
// swapped the small map is drawn above the window's picture
// (MAP_SWAPPED_Z_INDEX); the swapped window then has no z-index of its own,
// so its overflow menu (MENU_Z_INDEX) can still draw above the small map.
export const Z_INDEX = 9;
export const MAP_SWAPPED_Z_INDEX = Z_INDEX;
export const MENU_Z_INDEX = MAP_SWAPPED_Z_INDEX + 1;

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

/** Camera icon: the UAV advertises a stream. */
function CameraIcon() {
  return (
    <svg width="14" height="14" viewBox="0 0 14 14" fill="none" aria-hidden="true" data-icon="camera" style={{ flexShrink: 0 }}>
      <rect x="1" y="4" width="9" height="7" rx="1.2" stroke="currentColor" strokeWidth="1.3" />
      <path d="M10 6.5l3-1.8v5.6l-3-1.8z" stroke="currentColor" strokeWidth="1.2" strokeLinejoin="round" />
    </svg>
  );
}

/** Map icon: no stream; a pick opens the synthetic view. */
function MapIcon() {
  return (
    <svg width="14" height="14" viewBox="0 0 14 14" fill="none" aria-hidden="true" data-icon="map" style={{ flexShrink: 0 }}>
      <path d="M1 3l4-1.5 4 1.5 4-1.5v9.5l-4 1.5-4-1.5-4 1.5z" stroke="currentColor" strokeWidth="1.2" strokeLinejoin="round" />
      <path d="M5 1.5v9.5M9 3v9.5" stroke="currentColor" strokeWidth="1.1" />
    </svg>
  );
}

/** "UAV 2, camera" / "UAV 3, no camera, synthetic view", plus the lost link. */
function uavButtonName(t, entry) {
  const base = t(hasVideo(entry) ? 'vehicleWindow.uav.camera' : 'vehicleWindow.uav.map', { name: entry.label });
  return entry.linkOk ? base : t('vehicleWindow.uav.withLinkLost', { name: base });
}

/**
 * One UAV button: its camera or map icon, its name (the number only on a
 * narrow header), and "link lost" in text when its link is lost. Selected:
 * fill + outline + label, never colour alone. `pressed` undefined in the picker.
 */
function UavButton({ entry, pressed, width, label, onSelect }) {
  const { t } = useTranslation();
  const selected = pressed === true;
  const fill = zoneColorsSolid[entry.index % zoneColorsSolid.length];
  return (
    <button
      type="button"
      data-sysid={entry.sysId}
      aria-pressed={pressed}
      aria-label={uavButtonName(t, entry)}
      onClick={() => onSelect(entry.sysId)}
      style={{
        ...buttonStyle,
        width,
        minWidth: width === 'auto' ? TOUCH_TARGET_PX : width,
        padding: '0 6px',
        flexShrink: 0,
        overflow: 'hidden',
        display: 'inline-flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        background: selected ? fill : 'transparent',
        outline: selected ? `2px solid ${colors.textBright}` : 'none',
        outlineOffset: -2,
        fontWeight: selected ? 800 : 600,
      }}
    >
      <span style={{ display: 'inline-flex', alignItems: 'center', gap: HEADER_GAP_PX, maxWidth: '100%' }}>
        {hasVideo(entry) ? <CameraIcon /> : <MapIcon />}
        <span style={{ overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
          {label === 'number' ? entry.sysId : entry.label}
        </span>
      </span>
      {!entry.linkOk && (
        <span style={{ fontSize: 10, lineHeight: '11px', fontWeight: 700, color: colors.error, whiteSpace: 'normal' }}>
          {t('vehicleWindow.uav.linkLost')}
        </span>
      )}
    </button>
  );
}

/** No UAV picked: the prompt, then one button per UAV, wrapping; past the height limit it scrolls. */
function Picker({ fleet, onSelect }) {
  const { t } = useTranslation();
  const prompt = t('vehicleWindow.pickerPrompt');
  return (
    <>
      <div
        style={{
          padding: HEADER_PADDING_PX,
          fontSize: HEADER_FONT_PX,
          fontWeight: 600,
          color: colors.textBright,
          background: colors.bgLight,
          flexShrink: 0,
        }}
      >
        {prompt}
      </div>
      <div
        role="group"
        aria-label={prompt}
        data-testid="uav-picker"
        style={{
          display: 'flex',
          flexWrap: 'wrap',
          gap: HEADER_GAP_PX,
          padding: HEADER_PADDING_PX,
          paddingTop: 0,
          minHeight: 0,
          overflowY: 'auto',
          background: colors.bgLight,
        }}
      >
        {fleet.map((entry) => (
          <UavButton key={entry.sysId} entry={entry} width="auto" label="full" onSelect={onSelect} />
        ))}
      </div>
    </>
  );
}

/**
 * The menu button for the UAV buttons that do not fit, and its menu: it opens
 * downward under the button, stays inside the window (then scrolls; measured
 * again whenever the window changes size while open) and draws above the
 * small swapped map. A pick, Escape or a tap outside closes it; Escape
 * returns focus to the button.
 */
function OverflowMenu({ entries, selectedSysId, onSelectUav }) {
  const { t } = useTranslation();
  const [open, setOpen] = useState(false);
  const [maxHeight, setMaxHeight] = useState(null);
  const wrapRef = useRef(null);
  const buttonRef = useRef(null);

  useEffect(() => {
    if (!open) return undefined;
    const onPointerDown = (e) => {
      if (!wrapRef.current?.contains(e.target)) setOpen(false);
    };
    const onKeyDown = (e) => {
      if (e.key !== 'Escape') return;
      setOpen(false);
      buttonRef.current?.focus();
    };
    document.addEventListener('pointerdown', onPointerDown);
    document.addEventListener('keydown', onKeyDown);
    return () => {
      document.removeEventListener('pointerdown', onPointerDown);
      document.removeEventListener('keydown', onKeyDown);
    };
  }, [open]);

  // Room down to the window's bottom edge, before paint and on every window
  // resize while open (a failed feed collapses the picture); unmeasured (no
  // layout): no limit.
  useLayoutEffect(() => {
    const section = wrapRef.current?.closest('section');
    if (!open || !section) return undefined;
    const measure = () => {
      const room = section.getBoundingClientRect().bottom - WINDOW_BORDER_PX
        - buttonRef.current.getBoundingClientRect().bottom;
      setMaxHeight(room > 0 ? room : null);
    };
    measure();
    if (typeof ResizeObserver === 'undefined') return undefined;
    const observer = new ResizeObserver(measure);
    observer.observe(section);
    return () => observer.disconnect();
  }, [open]);

  return (
    <div ref={wrapRef} style={{ position: 'relative', flexShrink: 0 }}>
      <button
        ref={buttonRef}
        type="button"
        aria-haspopup="true"
        aria-expanded={open}
        aria-label={t('vehicleWindow.moreUavsLabel')}
        onClick={() => setOpen((v) => !v)}
        style={{ ...buttonStyle, padding: '0 6px' }}
      >
        {t('vehicleWindow.moreUavs', { count: entries.length })}
      </button>
      {open && (
        <div
          role="menu"
          style={{
            position: 'absolute',
            top: '100%',
            left: 0,
            zIndex: MENU_Z_INDEX,
            display: 'flex',
            flexDirection: 'column',
            gap: HEADER_GAP_PX,
            padding: HEADER_PADDING_PX,
            maxHeight: maxHeight ?? undefined,
            overflowY: 'auto',
            background: colors.bgLight,
            border: `1px solid ${colors.border}`,
            borderRadius: 6,
          }}
        >
          {entries.map((entry) => (
            <UavButton
              key={entry.sysId}
              entry={entry}
              pressed={entry.sysId === selectedSysId}
              width="auto"
              label="full"
              onSelect={(sysId) => { setOpen(false); onSelectUav(sysId); }}
            />
          ))}
        </div>
      )}
    </div>
  );
}

function Header({ fleet, selectedSysId, layout, title, stateKey, onSelectUav }) {
  const { t } = useTranslation();
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
        <UavButton
          key={fleet[i].sysId}
          entry={fleet[i]}
          pressed={fleet[i].sysId === selectedSysId}
          width={layout.badgeWidth}
          label={layout.badgeLabel}
          onSelect={onSelectUav}
        />
      ))}
      {overflow.length > 0 && (
        <OverflowMenu entries={overflow} selectedSysId={selectedSysId} onSelectUav={onSelectUav} />
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

/** The switch (left) and Maximize and Close (right); glyphs with names on a narrow pane. */
function ControlRow({ source, onSource, swapped, canSwap, narrow, onToggleMaximized, onClose }) {
  const { t } = useTranslation();
  const sizeLabel = swapped ? t('vehicleWindow.restore') : t('vehicleWindow.maximize');
  const closeLabel = t('vehicleWindow.close');
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
      {(swapped || canSwap) && (
        <button
          type="button"
          aria-pressed={swapped}
          aria-label={narrow ? sizeLabel : undefined}
          title={narrow ? sizeLabel : undefined}
          data-control="size"
          style={buttonStyle}
          onClick={onToggleMaximized}
        >
          {narrow ? (swapped ? '❐' : '□') : sizeLabel}
        </button>
      )}
      <button
        type="button"
        data-control="close"
        aria-label={narrow ? closeLabel : undefined}
        title={narrow ? closeLabel : undefined}
        style={buttonStyle}
        onClick={onClose}
      >
        {narrow ? '×' : closeLabel}
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

/**
 * "Vehicle link lost" first, then "Camera video lost" with its reason and
 * actions. Dismiss only for a withdrawn stream; a failed feed that is still
 * advertised offers Reconnect video alone.
 */
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
          {onDismiss && (
            <button type="button" style={buttonStyle} onClick={onDismiss}>
              {t('vehicleWindow.dismiss')}
            </button>
          )}
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

/** Text over the picture: centred, or a top-left tag that wraps at `maxWidth` (left of the small map). */
const pictureText = (color, centered, maxWidth = null) => ({
  position: 'absolute',
  ...(centered
    ? { inset: 0, display: 'flex', alignItems: 'center', justifyContent: 'center' }
    : { top: PICTURE_TAG_INSET_PX, left: PICTURE_TAG_INSET_PX, maxWidth: maxWidth ?? undefined, boxSizing: 'border-box' }),
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
function CameraFeed({ rows, chrome, stream, label, linkLost, swapped, tagMaxWidth, onReconnect }) {
  const { t } = useTranslation();
  const feed = useVideoFeed(stream);
  const failed = feed.status === FEED_STATUS.FAILED;
  const reason = failed ? REASON[feed.reason](t, stream) : null;
  return (
    <>
      {rows(
        <>
          {chrome(feed.status)}
          <LossRows label={label} linkLost={linkLost} videoLostReason={reason} onReconnect={onReconnect} />
        </>,
      )}
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
            <div style={pictureText(colors.warning, false, tagMaxWidth)}>{t('vehicleWindow.noFrames', { seconds: feed.seconds })}</div>
          )}
        </div>
      )}
    </>
  );
}

/**
 * The monitor view's vehicle window, top-right of the map pane under the
 * action bar (docs/design/gcs_video_streams.md, section 5 "Vehicle window"):
 * the UAV picker until the operator picks a UAV, then one UAV and one source
 * (Camera or Synthetic), normal or maximized (swapped with the map). State
 * lives in useVehicleWindow; sizes come from vehicleWindowLayout. The parent
 * mounts it in the monitor view, outside RC, whenever the fleet has a UAV.
 * `onChromeHeight` reports the height of the rows above the picture, which
 * the swapped small map sits under.
 */
export default function VehicleWindow({
  fleet, win, layout, poseSourceRef, storeRef, ionTerrain, onSyntheticCreditsHeight, onChromeHeight,
}) {
  const { t } = useTranslation();
  const { sysId, shown, source } = win;
  // Swapped only while the pane has room for a whole small map (App ends the swap otherwise).
  const swapped = win.swapped && layout.canSwap;
  const box = layout.window;

  const stopChromeRef = useRef(null);
  const chromeRef = useCallback((el) => {
    stopChromeRef.current?.();
    stopChromeRef.current = el && onChromeHeight ? observeHeight(el, onChromeHeight) : null;
  }, [onChromeHeight]);

  // Close removes the focused control: focus the closed UAV's picker button,
  // or the first one if that UAV has left the fleet.
  const sectionRef = useRef(null);
  const focusAfterCloseRef = useRef(null);
  const close = () => {
    focusAfterCloseRef.current = sysId;
    win.close();
  };
  useEffect(() => {
    const target = focusAfterCloseRef.current;
    if (sysId != null || target == null) return;
    focusAfterCloseRef.current = null;
    const section = sectionRef.current;
    (section?.querySelector(`[data-sysid="${target}"]`) ?? section?.querySelector('[data-sysid]'))?.focus();
  }, [sysId]);

  // A swap ended by a pane too short for the small map removes a focused
  // Restore (browsers drop focus to the body): focus Close instead.
  const sizeShown = swapped || layout.canSwap;
  const lastFocusRef = useRef(null);
  useLayoutEffect(() => {
    if (sizeShown || lastFocusRef.current !== 'size') return;
    lastFocusRef.current = null;
    const active = document.activeElement;
    if (active == null || active === document.body) sectionRef.current?.querySelector('[data-control="close"]')?.focus();
  }, [sizeShown]);

  const section = (children) => (
    <section
      ref={sectionRef}
      aria-label={t('vehicleWindow.region')}
      onFocus={(e) => { lastFocusRef.current = e.target.dataset?.control ?? null; }}
      style={{
        position: 'absolute',
        top: box.top,
        right: box.right,
        zIndex: swapped ? undefined : Z_INDEX,
        width: box.width,
        height: box.height ?? undefined,
        maxHeight: box.maxHeight,
        display: 'flex',
        flexDirection: 'column',
        borderRadius: 6,
        overflow: 'visible',
        border: `${WINDOW_BORDER_PX}px solid ${colors.border}`,
        background: colors.bgLight,
        pointerEvents: 'auto',
      }}
    >
      {children}
    </section>
  );

  if (sysId == null) return section(<Picker fleet={fleet} onSelect={win.selectUav} />);

  const entry = fleet.find((e) => e.sysId === sysId) ?? null;
  const label = entry?.label ?? `UAV ${sysId}`;
  const linkLost = !entry?.linkOk;
  const synthetic = source === SOURCE.SYNTHETIC;
  const streamName = shown.stream?.name;
  let title = streamName ? `${label} · ${streamName}` : label;
  if (synthetic) title = t('vehicleWindow.syntheticTitle', { name: label });

  // The rows above the picture, measured as one block.
  const rows = (children) => (
    <div ref={chromeRef} style={{ display: 'flex', flexDirection: 'column', flexShrink: 0 }}>
      {children}
    </div>
  );
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
        canSwap={layout.canSwap}
        narrow={layout.narrow}
        onToggleMaximized={win.toggleMaximized}
        onClose={close}
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
        {/* Keyed by UAV: the held time is that UAV's, not the previous one's. */}
        {rows(<>{chrome('synthetic')}<HeldRow key={sysId} holdReason={fleetEntryHoldReason(entry)} /></>)}
        <SyntheticView
          sysId={sysId}
          label={label}
          poseSourceRef={poseSourceRef}
          storeRef={storeRef}
          ionTerrain={ionTerrain}
          onFail={win.syntheticFailed}
          onCreditsHeight={onSyntheticCreditsHeight}
          style={pictureBox(swapped)}
          expanded={swapped}
          tagMaxWidth={layout.tagMaxWidth}
        />
      </>
    );
  } else if (shown.kind === 'stream') {
    body = (
      <CameraFeed
        key={win.session}
        rows={rows}
        chrome={chrome}
        stream={shown.stream}
        label={label}
        linkLost={linkLost}
        swapped={swapped}
        tagMaxWidth={layout.tagMaxWidth}
        onReconnect={win.reconnect}
      />
    );
  } else if (shown.kind === 'lost') {
    body = (
      <>
        {rows(
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
          </>,
        )}
        {swapped && <StateArea text={t('vehicleWindow.videoLost', { name: label })} />}
      </>
    );
  } else {
    body = (
      <>
        {rows(
          <>
            {chrome('noCamera')}
            <LossRows label={label} linkLost={linkLost} />
            <Row tone={colors.textDim}>{t('vehicleWindow.noCamera', { name: label })}</Row>
          </>,
        )}
        {swapped && <StateArea text={t('vehicleWindow.noCamera', { name: label })} />}
      </>
    );
  }

  return section(body);
}
