import React, { useEffect, useRef } from 'react';
import { useTranslation } from 'react-i18next';
import HeadingTape from './hud/HeadingTape';
import AttitudeIndicator from './hud/AttitudeIndicator';
import SpeedTape from './hud/SpeedTape';
import AltitudeTape from './hud/AltitudeTape';
import StatusStrip from './hud/StatusStrip';
import BatteryIndicator from './hud/BatteryIndicator';
import StatusBarIndicator from './hud/StatusBarIndicator';
import { useTelemetryStore } from '../hooks/useTelemetryStore';
import useElementSize from '../hooks/useElementSize';
import { colors } from '../styles';
import { READOUT_LINE_PX } from '../utils/hudLayout';
import { formatDistance, formatEte, nextWaypointInfo } from '../utils/fpvRoute';

// The tapes take no taps: below the RC confirm list (9) and the first-person
// tag and held banner (8), so those win where they meet; above the attitude (5).
const TAPE_Z_INDEX = 7;
const NO_SIZE = { width: 0, height: 0 };
/** The plain-text readouts' font; READOUT_LINE_PX is its line with the outline. */
const READOUT_FONT_PX = 12;
const READOUT_STYLE = {
  fontFamily: '"Consolas", "Monaco", "Courier New", monospace',
  fontSize: READOUT_FONT_PX,
  lineHeight: `${READOUT_LINE_PX}px`,
  fontWeight: 700,
  color: colors.textBright,
  textShadow: '0 0 3px #000, 0 0 3px #000',
  whiteSpace: 'nowrap',
};

const num = (v, digits = 0) => (Number.isFinite(v) ? v.toFixed(digits) : '--');

/**
 * The first-person HUD, shared by RC control (over the main map) and the
 * vehicle window's Synthetic picture (docs/design/gcs_video_streams.md,
 * section 5 "Synthetic view"). Display only: it takes no taps and holds no
 * control; the sticks, mode buttons and arming stay with RC control.
 *
 * @param {object} p
 * @param {number} p.sysId
 * @param {object} p.layout - where each part goes, from utils/hudLayout.js; a null part is not drawn.
 * @param {{ current: { route: (sysId: number) => object|null } | null }} [p.routeSourceRef] - the
 *   UAV's route (utils/fpvRoute.js), for the next waypoint readout.
 * @param {(size: {width: number, height: number}) => void} [p.onCornerSize] - the
 *   top-right column's measured size, for a host that lays out around it.
 */
export default function TelemetryHud({ sysId, layout, routeSourceRef, onCornerSize }) {
  const vehicle = useTelemetryStore(s => s.getVehicles()[sysId]);
  if (!vehicle) return null;
  const route = routeSourceRef?.current?.route(sysId) ?? null;
  return <Hud vehicle={vehicle} layout={layout} waypoint={nextWaypointInfo(route, vehicle)} onCornerSize={onCornerSize} />;
}

/** Mounted with a vehicle, so the corner column exists when it is measured. */
function Hud({ vehicle, layout, waypoint, onCornerSize }) {
  const { t } = useTranslation();
  const cornerRef = useRef(null);
  const cornerSize = useElementSize(cornerRef, NO_SIZE);
  const onCornerSizeRef = useRef(onCornerSize);
  onCornerSizeRef.current = onCornerSize;
  useEffect(() => {
    onCornerSizeRef.current?.(cornerSize);
  }, [cornerSize]);

  const {
    air_speed, ground_speed, alt_rel, heading, mode,
    pitch, roll, battery, voltage, current, status_texts,
    gps_fix, gps_hacc, link_ok, link_quality,
  } = vehicle;
  const { attitude, heading: headingBox, corner, speed, altitude, numbers, strip } = layout;
  const speedText = t('fpvView.hud.speed', { as: num(air_speed), gs: num(ground_speed) });
  const altText = t('fpvView.hud.altitude', { alt: num(alt_rel) });
  let waypointText = null;
  if (waypoint && corner.waypoint === 'compact') {
    waypointText = t('fpvView.hud.waypoint', { n: waypoint.number, dist: formatDistance(waypoint.distanceM) });
  } else if (waypoint && corner.waypoint === 'full') {
    waypointText = waypoint.eteS == null
      ? t('fpvView.hud.nextWaypoint', { n: waypoint.number, dist: formatDistance(waypoint.distanceM) })
      : t('fpvView.hud.nextWaypointEte', {
        n: waypoint.number, dist: formatDistance(waypoint.distanceM), ete: formatEte(waypoint.eteS),
      });
  }

  return (
    <>
      {/* Attitude indicator — centred in its box; the scale only where the box can show it */}
      <div data-testid="hud-attitude" style={{
        position: 'absolute',
        top: attitude.top,
        bottom: attitude.bottom,
        left: attitude.left,
        right: attitude.right,
        pointerEvents: 'none',
        zIndex: 5,
      }}>
        <AttitudeIndicator pitch={pitch} roll={roll} scale={attitude.scale} />
      </div>

      {/* Heading tape — top, centred unless the layout gives its left edge */}
      {headingBox && (
        <div style={{
          position: 'absolute',
          top: headingBox.top,
          ...(headingBox.left == null ? { left: '50%', transform: 'translateX(-50%)' } : { left: headingBox.left }),
          pointerEvents: 'none',
          zIndex: 10,
        }}>
          <HeadingTape heading={heading} width={headingBox.width} height={headingBox.height} />
        </div>
      )}

      {/* Top-right column: mode, link and battery (panel hidden), the next waypoint, or the numbers */}
      <div ref={cornerRef} data-testid="hud-corner" style={{
        position: 'absolute',
        top: corner.top,
        right: corner.right,
        pointerEvents: 'none',
        zIndex: 10,
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'flex-end',
        gap: 2,
      }}>
        {(corner.status || corner.mode) && (
          <div data-testid="hud-status" style={{ display: 'flex', alignItems: 'flex-start', gap: 4 }}>
            {corner.mode && <span data-testid="hud-mode" style={READOUT_STYLE}>{mode ?? '--'}</span>}
            {corner.status && (
              <>
                <StatusBarIndicator gpsFix={gps_fix} gpsHacc={gps_hacc} linkOk={link_ok} linkQuality={link_quality} />
                <BatteryIndicator battery={battery} voltage={voltage} current={current} />
              </>
            )}
          </div>
        )}
        {waypointText && <span data-testid="hud-waypoint" style={READOUT_STYLE}>{waypointText}</span>}
        {corner.numbers && (
          <span data-testid="hud-numbers" style={READOUT_STYLE}>{`${speedText} · ${altText}`}</span>
        )}
      </div>

      {/* Compact: speed and altitude as plain numbers, one row */}
      {numbers && (
        <>
          <span data-testid="hud-speed-number" style={{
            ...READOUT_STYLE, position: 'absolute', top: numbers.top, left: numbers.left, pointerEvents: 'none', zIndex: 10,
          }}>{speedText}</span>
          <span data-testid="hud-altitude-number" style={{
            ...READOUT_STYLE, position: 'absolute', top: numbers.top, right: numbers.right, pointerEvents: 'none', zIndex: 10,
          }}>{altText}</span>
        </>
      )}

      {/* Speed tape — left, below the mode column or beside it */}
      {speed && (
        <div style={{
          position: 'absolute',
          top: speed.top,
          left: speed.left,
          pointerEvents: 'none',
          zIndex: TAPE_Z_INDEX,
        }}>
          <SpeedTape airSpeed={air_speed} groundSpeed={ground_speed} width={speed.width} height={speed.height} />
        </div>
      )}

      {/* Altitude tape — right, above the stick */}
      {altitude && (
        <div style={{
          position: 'absolute',
          top: altitude.top,
          right: altitude.right,
          pointerEvents: 'none',
          zIndex: TAPE_Z_INDEX,
        }}>
          <AltitudeTape altitude={alt_rel} width={altitude.width} height={altitude.height} />
        </div>
      )}

      {/* STATUSTEXT warnings — bottom center, panel hidden only */}
      {strip && (
        <div data-testid="hud-strip" style={{
          position: 'absolute',
          bottom: strip.bottom,
          left: '50%',
          transform: 'translateX(-50%)',
          pointerEvents: 'none',
          zIndex: 10,
        }}>
          <StatusStrip statusTexts={status_texts} maxMessages={strip.maxMessages} />
        </div>
      )}
    </>
  );
}
