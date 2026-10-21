import React, { useCallback, useEffect, useId, useLayoutEffect, useRef, useState } from 'react';
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
  BAR_GAP_PX, BAR_PADDING_PX, CHIP_MAX_PX, MAP_CONTROL_PX, PICTURE_ASPECT, WINDOW_BORDER_PX, WINDOW_FONT_PX,
} from '../utils/vehicleWindowLayout';
import { observeHeight } from './map/utils/observeHeight';
import MapIconBtn from './MapIconBtn';
import SyntheticView from './SyntheticView';
import { OVERLAY_BG, pictureTagStyle, textButtonStyle } from './windowStyles';

// Below the map overlay controls (z-index 10) and the confirmation cards
// (z-index 20): operator controls always stay on top of the window. While
// swapped the small map is drawn above the window's picture
// (MAP_SWAPPED_Z_INDEX); the swapped window then has no z-index of its own,
// so its UAV list (MENU_Z_INDEX) can still draw above the small map.
export const Z_INDEX = 9;
export const MAP_SWAPPED_Z_INDEX = Z_INDEX;
export const MENU_Z_INDEX = MAP_SWAPPED_Z_INDEX + 1;
/** The UAV list's least height, one row: with less room below the chip it still shows a scrollable row. */
const LIST_MIN_PX = MAP_CONTROL_PX + 2 * BAR_PADDING_PX + 2 * WINDOW_BORDER_PX;

const REASON = {
  [FEED_FAILURE.NO_FIRST_FRAME]: (t) => t('vehicleWindow.reason.noFirstFrame', { seconds: VIDEO_FIRST_FRAME_DEADLINE_S }),
  [FEED_FAILURE.STALLED]: (t) => t('vehicleWindow.reason.stalled', { seconds: VIDEO_STALL_DEADLINE_S }),
  [FEED_FAILURE.CODEC]: (t, stream) => t('vehicleWindow.reason.codec', { codec: stream?.encoding ?? '' }),
  [FEED_FAILURE.CONNECTION]: (t) => t('vehicleWindow.reason.connection'),
  notAdvertised: (t) => t('vehicleWindow.reason.notAdvertised'),
};

const zoneColor = (entry) => zoneColorsSolid[entry.index % zoneColorsSolid.length];

/** Camera icon: the UAV advertises a stream. Slashed: Camera is unavailable (a cue beyond colour). */
function CameraIcon({ slashed = false }) {
  return (
    <svg width="14" height="14" viewBox="0 0 14 14" fill="none" aria-hidden="true" data-icon={slashed ? 'camera-off' : 'camera'} style={{ flexShrink: 0 }}>
      <rect x="1" y="4" width="9" height="7" rx="1.2" stroke="currentColor" strokeWidth="1.3" />
      <path d="M10 6.5l3-1.8v5.6l-3-1.8z" stroke="currentColor" strokeWidth="1.2" strokeLinejoin="round" />
      {slashed && <path d="M1 1.5l12 11" stroke="currentColor" strokeWidth="1.3" strokeLinecap="round" />}
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

/** Maximize (arrows out) or Restore (arrows in). */
function SizeIcon({ swapped }) {
  const d = swapped
    ? 'M12.5 5.5h-4v-4M1.5 8.5h4v4M8.5 5.5L13 1M5.5 8.5L1 13'
    : 'M8.5 1.5h4v4M5.5 12.5h-4v-4M12.5 1.5L8 6M1.5 12.5L6 8';
  return (
    <svg width="14" height="14" viewBox="0 0 14 14" fill="none" aria-hidden="true">
      <path d={d} stroke="currentColor" strokeWidth="1.4" strokeLinecap="round" strokeLinejoin="round" />
    </svg>
  );
}

function Chevron() {
  return (
    <svg width="10" height="10" viewBox="0 0 12 12" fill="none" aria-hidden="true" style={{ flexShrink: 0 }}>
      <path d="M2.5 4.5L6 8l3.5-3.5" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" />
    </svg>
  );
}

function Check() {
  return (
    <svg width="12" height="12" viewBox="0 0 12 12" fill="none" aria-hidden="true" data-icon="check" style={{ flexShrink: 0, marginLeft: 'auto' }}>
      <path d="M2 6.5l2.5 2.5L10 3.5" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" />
    </svg>
  );
}

/** The UAV's zone colour as a 10 px swatch; text never sits on it. */
function Swatch({ entry }) {
  return (
    <span
      aria-hidden="true"
      data-swatch=""
      style={{ width: 10, height: 10, borderRadius: 3, flexShrink: 0, background: entry ? zoneColor(entry) : colors.border }}
    />
  );
}

/** "UAV 2, camera" / "UAV 3, no camera, synthetic view", plus the lost link. */
function uavButtonName(t, entry) {
  const base = t(hasVideo(entry) ? 'vehicleWindow.uav.camera' : 'vehicleWindow.uav.map', { name: entry.label });
  return entry.linkOk ? base : t('vehicleWindow.uav.withLinkLost', { name: base });
}

const LINK_LOST_STYLE = { color: colors.error, fontWeight: 700, flexShrink: 0 };

/** The window's text controls: the map button's 32 px, fill and border. */
const controlStyle = {
  height: MAP_CONTROL_PX,
  display: 'inline-flex',
  alignItems: 'center',
  borderRadius: 6,
  border: `1px solid ${colors.border}`,
  background: OVERLAY_BG,
  color: colors.textBright,
  fontSize: WINDOW_FONT_PX,
  fontWeight: 700,
  whiteSpace: 'nowrap',
  cursor: 'pointer',
  touchAction: 'manipulation',
};

/**
 * One UAV button in the picker or the UAV list: its camera or map glyph, its
 * zone swatch, its name and "link lost" in text. In the list it also says
 * what a pick opens, and the selected row is outlined, checked and pressed.
 */
function UavButton({ entry, pressed, row = false, onSelect }) {
  const { t } = useTranslation();
  const selected = pressed === true;
  return (
    <button
      type="button"
      data-sysid={entry.sysId}
      aria-pressed={pressed}
      aria-label={uavButtonName(t, entry)}
      onClick={() => onSelect(entry.sysId)}
      style={{
        ...controlStyle,
        gap: 6,
        padding: '0 10px',
        flexShrink: 0,
        ...(row ? { width: '100%', border: 'none', background: 'transparent' } : null),
        outline: selected ? `2px solid ${colors.textBright}` : 'none',
        outlineOffset: -2,
      }}
    >
      {hasVideo(entry) ? <CameraIcon /> : <MapIcon />}
      <Swatch entry={entry} />
      <span>{entry.label}</span>
      {row && (
        <span style={{ color: colors.textDim, fontWeight: 400 }}>
          {t(hasVideo(entry) ? 'vehicleWindow.uav.opensCamera' : 'vehicleWindow.uav.opensSynthetic')}
        </span>
      )}
      {!entry.linkOk && <span style={LINK_LOST_STYLE}>{t('vehicleWindow.uav.linkLost')}</span>}
      {row && selected && <Check />}
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
          position: 'relative',
          padding: '6px 6px 0',
          fontSize: WINDOW_FONT_PX,
          fontWeight: 600,
          color: colors.textBright,
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
          position: 'relative',
          display: 'flex',
          flexWrap: 'wrap',
          gap: BAR_GAP_PX,
          padding: 6,
          minHeight: 0,
          overflowY: 'auto',
        }}
      >
        {fleet.map((entry) => <UavButton key={entry.sysId} entry={entry} onSelect={onSelect} />)}
      </div>
    </>
  );
}

/**
 * The UAV chip and its list (disclosure: aria-expanded on the chip, plain
 * buttons in the list). The list opens downward, draws above the small swapped
 * map, and scrolls past the room from the chip down to the window's bottom
 * limit (`limitHeight` below the window's top: the margin above reset view at
 * normal size, the window's bottom when swapped). A pick, Escape or a tap
 * outside closes it; Escape returns focus to the chip.
 */
function UavChip({ fleet, sysId, label, entry, limitHeight, chipRef, onPick }) {
  const { t } = useTranslation();
  const [open, setOpen] = useState(false);
  const [maxHeight, setMaxHeight] = useState(null);
  const wrapRef = useRef(null);
  const listId = useId();
  const linkLost = !entry?.linkOk;

  useEffect(() => {
    if (!open) return undefined;
    const onPointerDown = (e) => {
      if (!wrapRef.current?.contains(e.target)) setOpen(false);
    };
    const onKeyDown = (e) => {
      if (e.key !== 'Escape') return;
      setOpen(false);
      chipRef.current?.focus();
    };
    document.addEventListener('pointerdown', onPointerDown);
    document.addEventListener('keydown', onKeyDown);
    return () => {
      document.removeEventListener('pointerdown', onPointerDown);
      document.removeEventListener('keydown', onKeyDown);
    };
  }, [open, chipRef]);

  // The room from the chip down to the bottom limit, before paint. The limit
  // comes from the layout, never from the window's rendered bottom (a window
  // without a picture is short, and the list would get no room).
  useLayoutEffect(() => {
    const section = wrapRef.current?.closest('section');
    if (!open || !section) return;
    const room = section.getBoundingClientRect().top + limitHeight - WINDOW_BORDER_PX
      - chipRef.current.getBoundingClientRect().bottom - BAR_GAP_PX;
    setMaxHeight(Math.max(room, LIST_MIN_PX));
  }, [open, limitHeight, chipRef]);

  const name = linkLost ? t('vehicleWindow.uav.withLinkLost', { name: label }) : label;
  return (
    <div ref={wrapRef} style={{ position: 'relative', flex: '0 1 auto', minWidth: 0, maxWidth: CHIP_MAX_PX }}>
      <button
        ref={chipRef}
        type="button"
        data-control="chip"
        aria-label={t('vehicleWindow.chip', { name })}
        aria-expanded={open}
        aria-controls={listId}
        onClick={() => setOpen((v) => !v)}
        style={{
          ...controlStyle, gap: 5, padding: '0 8px', maxWidth: '100%', border: 'none', borderRadius: '6px 0 0 6px',
        }}
      >
        <Swatch entry={entry} />
        <span data-testid="chip-name" style={{ overflow: 'hidden', textOverflow: 'ellipsis', minWidth: 0 }}>{label}</span>
        {linkLost && <span style={LINK_LOST_STYLE}>{t('vehicleWindow.uav.linkLost')}</span>}
        <Chevron />
      </button>
      {open && (
        <div
          id={listId}
          role="group"
          aria-label={t('vehicleWindow.uavList')}
          style={{
            position: 'absolute',
            top: `calc(100% + ${BAR_GAP_PX}px)`,
            left: 0,
            zIndex: MENU_Z_INDEX,
            display: 'flex',
            flexDirection: 'column',
            gap: BAR_GAP_PX,
            padding: BAR_PADDING_PX,
            minWidth: 200,
            maxHeight,
            overflowY: 'auto',
            background: colors.bgLight,
            border: `1px solid ${colors.border}`,
            borderRadius: 6,
          }}
        >
          {fleet.map((e) => (
            <UavButton
              key={e.sysId}
              entry={e}
              row
              pressed={e.sysId === sysId}
              onSelect={(id) => { setOpen(false); chipRef.current.focus(); onPick(id); }}
            />
          ))}
        </div>
      )}
    </div>
  );
}

/**
 * The one bar: the UAV group [chip | Camera | Synthetic], then Maximize/Restore
 * apart at the right end, all the app's 32 px map button. The group never
 * splits: when it and Maximize do not fit on one line, Maximize wraps; when
 * the group alone does not fit, the chip's name ellipsizes. Camera without a
 * stream stays in place, slashed and dimmed, and focusable.
 */
function Bar({
  barRef, children, label, source, onSource, cameraAvailable, cameraDescribedBy, synthRef,
  swapped, canSwap, onToggleMaximized,
}) {
  const { t } = useTranslation();
  const sizeLabel = swapped ? t('vehicleWindow.restore') : t('vehicleWindow.maximize');
  const sourceTitle = (s) => (s === SOURCE.CAMERA && !cameraAvailable
    ? t('vehicleWindow.source.noCameraFor', { name: label })
    : t(`vehicleWindow.source.${s}`));
  return (
    <div
      ref={barRef}
      data-testid="window-bar"
      style={{
        position: 'relative',
        display: 'flex',
        flexWrap: 'wrap',
        alignItems: 'center',
        gap: BAR_GAP_PX,
        padding: BAR_PADDING_PX,
        flexShrink: 0,
      }}
    >
      <div
        data-testid="uav-group"
        style={{
          display: 'flex',
          flex: '0 1 auto',
          minWidth: 0,
          maxWidth: '100%',
          borderRadius: 6,
          // An inset outline, painted over the children's edges: the group
          // stays 32 px high and takes no width beyond its controls.
          outline: `${WINDOW_BORDER_PX}px solid ${colors.border}`,
          outlineOffset: -WINDOW_BORDER_PX,
        }}
      >
        {children}
        <div
          role="group"
          aria-label={t('vehicleWindow.source.groupFor', { name: label })}
          style={{ display: 'flex', flexShrink: 0, overflow: 'hidden', borderRadius: '0 6px 6px 0' }}
        >
          {[SOURCE.CAMERA, SOURCE.SYNTHETIC].map((s) => {
            const unavailable = s === SOURCE.CAMERA && !cameraAvailable;
            return (
              <MapIconBtn
                key={s}
                ref={s === SOURCE.SYNTHETIC ? synthRef : undefined}
                joined
                unavailable={unavailable}
                title={sourceTitle(s)}
                aria-label={t(`vehicleWindow.source.${s}`)}
                aria-pressed={source === s}
                aria-disabled={unavailable || undefined}
                aria-describedby={unavailable ? cameraDescribedBy : undefined}
                active={source === s}
                onClick={() => onSource(s)}
              >
                {s === SOURCE.CAMERA ? <CameraIcon slashed={unavailable} /> : <MapIcon />}
              </MapIconBtn>
            );
          })}
        </div>
      </div>
      {(swapped || canSwap) && (
        <div style={{ marginLeft: 'auto', flexShrink: 0 }}>
          <MapIconBtn
            title={sizeLabel}
            aria-label={sizeLabel}
            aria-pressed={swapped}
            data-control="size"
            onClick={onToggleMaximized}
          >
            <SizeIcon swapped={swapped} />
          </MapIconBtn>
        </div>
      )}
    </div>
  );
}

function Row({ children, tone }) {
  return (
    <div
      style={{
        position: 'relative',
        display: 'flex',
        flexWrap: 'wrap',
        alignItems: 'center',
        gap: 6,
        padding: '6px 8px',
        fontSize: WINDOW_FONT_PX,
        color: tone ?? colors.text,
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
function LossRows({ label, linkLost, videoLostReason, onReconnect, reconnectDisabled = false, onDismiss }) {
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
          <button type="button" style={textButtonStyle(reconnectDisabled)} onClick={onReconnect} disabled={reconnectDisabled}>
            {t('vehicleWindow.reconnect')}
          </button>
          {onDismiss && (
            <button type="button" style={textButtonStyle()} onClick={onDismiss}>
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

/** Too little room for the picture: the reader or widget is not mounted. Never part of the measured rows. */
function NoRoomRow() {
  const { t } = useTranslation();
  return (
    <Row tone={colors.textDim}>
      <span role="status">{t('vehicleWindow.noRoom')}</span>
    </Row>
  );
}

/** Centred text over the picture (connecting, or the swapped area's state). */
const centredText = (color) => ({
  position: 'absolute',
  inset: 0,
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'center',
  padding: 8,
  fontSize: WINDOW_FONT_PX,
  fontWeight: 700,
  color,
  background: 'rgba(15, 20, 35, 0.72)',
});

/** The picture box: 16:9 shrinking to the height limit; swapped, it fills the window. */
const pictureBox = (swapped) => ({
  overflow: 'hidden',
  borderRadius: `0 0 ${6 - WINDOW_BORDER_PX}px ${6 - WINDOW_BORDER_PX}px`,
  ...(swapped
    ? { flex: '1 1 auto', minHeight: 0 }
    : { flex: '0 1 auto', minHeight: 0, width: '100%', aspectRatio: `${PICTURE_ASPECT}` }),
});

/** Swapped and nothing to draw: the large area states it in text, never a black box. */
function StateArea({ text }) {
  return (
    <div style={{ ...pictureBox(true), position: 'relative', background: colors.bg }}>
      <div style={centredText(colors.textDim)}>{text}</div>
    </div>
  );
}

/** The camera picture's top-left tag: the live state and the stream name; it wraps, never cut. */
function CameraTag({ feed, streamName, maxWidth }) {
  const { t } = useTranslation();
  const stale = feed.status === FEED_STATUS.STALE;
  const live = feed.status === FEED_STATUS.LIVE;
  return (
    <div data-testid="picture-tag" style={pictureTagStyle(stale ? colors.warning : colors.textBright, maxWidth)}>
      <span role="status">
        {live && <span aria-hidden="true" style={{ color: colors.success }}>● </span>}
        {stale ? t('vehicleWindow.noFrames', { seconds: feed.seconds }) : t(`vehicleWindow.state.${feed.status}`)}
      </span>
      {streamName && <span> · {streamName}</span>}
    </div>
  );
}

/**
 * One camera session; remounted (new key) for every fresh connection. A
 * failure latches here, outside the picture: its row makes the window's
 * chrome taller, and a picture removed for lack of room must not take the
 * failure with it and reconnect on its own.
 */
function CameraFeed({ rows, extraRows, stream, label, linkLost, swapped, pictureFits, tagMaxWidth, onReconnect }) {
  const { t } = useTranslation();
  const [failure, setFailure] = useState(null);
  const reason = failure ? REASON[failure](t, stream) : null;
  let picture = null;
  if (failure) {
    if (swapped) picture = <StateArea text={t('vehicleWindow.videoLost', { name: label })} />;
  } else if (pictureFits) {
    picture = <LivePicture stream={stream} label={label} swapped={swapped} tagMaxWidth={tagMaxWidth} onFailed={setFailure} />;
  } else {
    picture = <NoRoomRow />;
  }
  return (
    <>
      {rows(
        <>
          {extraRows}
          <LossRows label={label} linkLost={linkLost} videoLostReason={reason} onReconnect={onReconnect} />
        </>,
      )}
      {picture}
    </>
  );
}

/** The camera picture and its reader; mounted only while the picture has room. */
function LivePicture({ stream, label, swapped, tagMaxWidth, onFailed }) {
  const { t } = useTranslation();
  const feed = useVideoFeed(stream);
  const failedReason = feed.status === FEED_STATUS.FAILED ? feed.reason : null;
  useLayoutEffect(() => {
    if (failedReason) onFailed(failedReason);
  }, [failedReason, onFailed]);
  if (failedReason) return null;
  return (
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
        <div style={centredText(colors.textDim)}>{t('vehicleWindow.connectingUav', { name: label })}</div>
      )}
      <CameraTag feed={feed} streamName={stream?.name} maxWidth={tagMaxWidth} />
    </div>
  );
}

/** A callback ref that reports the element's height while it is mounted. */
function useObservedHeight(onHeight) {
  const stopRef = useRef(null);
  return useCallback((el) => {
    stopRef.current?.();
    stopRef.current = el ? observeHeight(el, onHeight) : null;
  }, [onHeight]);
}

/**
 * The monitor view's vehicle window, top-right of the map pane under the
 * action bar (docs/design/gcs_video_streams.md, section 5 "Vehicle window",
 * v12 and v12.1): the UAV picker until the operator picks a UAV, then one slim
 * bar (the UAV group [chip | Camera | Synthetic], Maximize) over that UAV's picture, normal or
 * maximized (swapped with the map). The video icon in the action bar opens and
 * closes it; the parent mounts it only while open, in the monitor view outside
 * RC, with a UAV in the fleet. State lives in useVehicleWindow; sizes come from
 * vehicleWindowLayout. `onChromeHeight` reports the height of the bar and the
 * warning rows above the picture: the swapped small map sits under them, and
 * the picture's room is computed from them.
 */
export default function VehicleWindow({
  fleet, win, layout, poseSourceRef, storeRef, ionTerrain, onSyntheticCreditsHeight, onChromeHeight,
}) {
  const { t } = useTranslation();
  const { sysId, shown, source } = win;
  // Swapped only while the pane has room for a whole small map (App ends the swap otherwise).
  const swapped = win.swapped && layout.canSwap;
  const box = layout.window;
  const { pictureFits } = layout;

  const refusedId = useId();
  const synthRef = useRef(null);
  const [barHeight, setBarHeight] = useState(0);
  const [rowsHeight, setRowsHeight] = useState(0);
  const barRef = useObservedHeight(setBarHeight);
  const rowsRef = useObservedHeight(setRowsHeight);
  const picked = sysId != null;
  // Reported once the bar is measured (never 0 then): a 0 before that would
  // let a picture mount that the real height has no room for.
  useEffect(() => {
    if (picked && barHeight > 0) onChromeHeight?.(barHeight + rowsHeight);
  }, [picked, barHeight, rowsHeight, onChromeHeight]);
  // Closed, the chrome is unknown again: the next opening measures before a picture mounts.
  useEffect(() => () => onChromeHeight?.(null), [onChromeHeight]);

  // A pick in the picker or the UAV list moves focus to the chip, after commit.
  // The bar is outside the reader keyed by session, so a source switch keeps
  // focus on the pressed source button.
  const chipRef = useRef(null);
  const focusChipRef = useRef(false);
  // Only a pick that changes the UAV re-renders; the same UAV queues nothing
  // (the list focuses the chip itself), or a later source switch would lose focus.
  const pick = (id) => {
    if (id !== sysId) focusChipRef.current = true;
    win.selectUav(id);
  };
  useLayoutEffect(() => {
    if (!focusChipRef.current || !chipRef.current) return;
    focusChipRef.current = false;
    chipRef.current.focus();
  });

  // A swap ended by a pane too short for the small map removes a focused
  // Restore (browsers drop focus to the body): focus the chip instead.
  const sizeShown = swapped || layout.canSwap;
  const lastFocusRef = useRef(null);
  useLayoutEffect(() => {
    if (sizeShown || lastFocusRef.current !== 'size') return;
    lastFocusRef.current = null;
    const active = document.activeElement;
    if (active == null || active === document.body) chipRef.current?.focus();
  }, [sizeShown]);

  const section = (children) => (
    <section
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
        color: colors.textBright,
        pointerEvents: 'auto',
      }}
    >
      {/* The map overlay fill and blur on a layer behind the content: a
          backdrop filter on the section would make a stacking context, and
          the swapped window must not have one, so its UAV list can draw above
          the small map. Every child is positioned, so it paints above this layer. */}
      <div
        aria-hidden="true"
        data-testid="window-backdrop"
        style={{
          position: 'absolute',
          inset: 0,
          borderRadius: 6 - WINDOW_BORDER_PX,
          background: OVERLAY_BG,
          backdropFilter: 'blur(4px)',
          pointerEvents: 'none',
        }}
      />
      {children}
    </section>
  );

  if (!picked) return section(<Picker fleet={fleet} onSelect={pick} />);

  const entry = fleet.find((e) => e.sysId === sysId) ?? null;
  const label = entry?.label ?? `UAV ${sysId}`;
  const linkLost = !entry?.linkOk;
  const synthetic = source === SOURCE.SYNTHETIC;

  // The warning rows above the picture; measured with the bar.
  const rows = (children) => (
    <div ref={rowsRef} style={{ position: 'relative', display: 'flex', flexDirection: 'column', flexShrink: 0 }}>
      {children}
    </div>
  );
  // Camera chosen without a stream: the reason, also Camera's description (touch has no tooltip).
  const notice = win.cameraRefused && (
    <Row tone={colors.warning}>
      <span id={refusedId} role="status">{t('vehicleWindow.source.noCameraFor', { name: label })}</span>
    </Row>
  );

  let body;
  if (synthetic && win.syntheticFailed) {
    // Latched outside the widget: no room, a resize or a new stream never retries it.
    const retry = () => {
      win.retrySynthetic();
      synthRef.current?.focus();
    };
    body = (
      <>
        {rows(
          <>
            {notice}
            <Row tone={colors.warning}>
              <span role="alert" style={{ flexBasis: '100%' }}>{t('vehicleWindow.syntheticUnavailable')}</span>
              <button type="button" style={textButtonStyle()} onClick={retry}>{t('vehicleWindow.retry')}</button>
            </Row>
            <HeldRow key={sysId} holdReason={fleetEntryHoldReason(entry)} />
          </>,
        )}
        {swapped && <StateArea text={t('vehicleWindow.syntheticUnavailable')} />}
      </>
    );
  } else if (synthetic) {
    body = (
      <>
        {/* Keyed by UAV: the held time is that UAV's, not the previous one's. */}
        {rows(<>{notice}<HeldRow key={sysId} holdReason={fleetEntryHoldReason(entry)} /></>)}
        {pictureFits ? (
          <SyntheticView
            sysId={sysId}
            label={label}
            poseSourceRef={poseSourceRef}
            storeRef={storeRef}
            ionTerrain={ionTerrain}
            onFail={win.reportSyntheticFailure}
            onCreditsHeight={onSyntheticCreditsHeight}
            style={pictureBox(swapped)}
            expanded={swapped}
            tagMaxWidth={layout.tagMaxWidth}
          />
        ) : <NoRoomRow />}
      </>
    );
  } else if (shown.kind === 'stream') {
    body = (
      <CameraFeed
        key={win.session}
        rows={rows}
        extraRows={notice}
        stream={shown.stream}
        label={label}
        linkLost={linkLost}
        swapped={swapped}
        pictureFits={pictureFits}
        tagMaxWidth={layout.tagMaxWidth}
        onReconnect={win.reconnect}
      />
    );
  } else {
    // Camera with no stream left: lost, with Dismiss to Synthetic.
    body = (
      <>
        {rows(
          <>
            {notice}
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
  }

  return section(
    <>
      <Bar
        barRef={barRef}
        label={label}
        source={source}
        onSource={win.chooseSource}
        cameraAvailable={win.cameraAvailable}
        cameraDescribedBy={win.cameraRefused ? refusedId : undefined}
        synthRef={synthRef}
        swapped={swapped}
        canSwap={layout.canSwap}
        onToggleMaximized={win.toggleMaximized}
      >
        <UavChip
          fleet={fleet}
          sysId={sysId}
          label={label}
          entry={entry}
          limitHeight={box.height ?? box.maxHeight}
          chipRef={chipRef}
          onPick={pick}
        />
      </Bar>
      {body}
    </>,
  );
}
