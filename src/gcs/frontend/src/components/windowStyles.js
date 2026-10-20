import { colors } from '../styles';
import { MAP_CONTROL_PX, PICTURE_TAG_INSET_PX, WINDOW_FONT_PX } from '../utils/vehicleWindowLayout';

/** The map overlay fill of MapIconBtn and the plan button; the vehicle window uses it too. */
export const OVERLAY_BG = 'rgba(22, 33, 62, 0.85)';

/** A top-left tag over the picture (LIVE, NO FRAMES, SYNTHETIC · not camera); it wraps at `maxWidth`. */
export const pictureTagStyle = (color, maxWidth = null) => ({
  position: 'absolute',
  top: PICTURE_TAG_INSET_PX,
  left: PICTURE_TAG_INSET_PX,
  maxWidth: maxWidth ?? undefined,
  boxSizing: 'border-box',
  padding: '3px 8px',
  fontSize: WINDOW_FONT_PX,
  fontWeight: 700,
  color,
  background: OVERLAY_BG,
  borderRadius: 4,
  pointerEvents: 'none',
});

/** A text button in the plan button's style (Reconnect video, Dismiss). */
export const textButtonStyle = (disabled = false) => ({
  height: MAP_CONTROL_PX,
  display: 'inline-flex',
  alignItems: 'center',
  padding: '0 12px',
  borderRadius: 6,
  fontSize: WINDOW_FONT_PX,
  fontWeight: 600,
  cursor: disabled ? 'not-allowed' : 'pointer',
  border: `1px solid ${disabled ? colors.border : colors.accent}`,
  background: OVERLAY_BG,
  color: disabled ? colors.textDim : colors.accent,
  touchAction: 'manipulation',
  whiteSpace: 'nowrap',
});
