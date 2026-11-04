import React, { useEffect, useRef } from 'react';
import HeadingTape from './hud/HeadingTape';
import AttitudeIndicator from './hud/AttitudeIndicator';
import SpeedTape from './hud/SpeedTape';
import AltitudeTape from './hud/AltitudeTape';
import StatusStrip from './hud/StatusStrip';
import BatteryIndicator from './hud/BatteryIndicator';
import StatusBarIndicator from './hud/StatusBarIndicator';
import { useTelemetryStore } from '../hooks/useTelemetryStore';
import useElementSize from '../hooks/useElementSize';

// The tapes take no taps: below the RC confirm list (9) and the first-person
// tag and held banner (8), so those win where they meet; above the attitude (5).
const TAPE_Z_INDEX = 7;
const NO_SIZE = { width: 0, height: 0 };

/**
 * The first-person HUD, shared by RC control (over the main map) and the
 * vehicle window's Synthetic picture (docs/design/gcs_video_streams.md,
 * section 5 "Synthetic view"). Display only: it takes no taps and holds no
 * control; the sticks, mode buttons and arming stay with RC control.
 *
 * @param {object} p
 * @param {number} p.sysId
 * @param {object} p.layout - where each part goes, from utils/hudLayout.js
 *   (rcHudLayout or pictureHudLayout); a null tape is not drawn.
 * @param {(size: {width: number, height: number}) => void} [p.onStatusSize] - the
 *   link and battery row's measured size, for a host that lays out around it.
 */
export default function TelemetryHud({ sysId, layout, onStatusSize }) {
  const vehicle = useTelemetryStore(s => s.getVehicles()[sysId]);
  if (!vehicle) return null;
  return <Hud vehicle={vehicle} layout={layout} onStatusSize={onStatusSize} />;
}

/** Mounted with a vehicle, so the status row exists when it is measured. */
function Hud({ vehicle, layout, onStatusSize }) {
  const statusRef = useRef(null);
  const statusSize = useElementSize(statusRef, NO_SIZE);
  const onStatusSizeRef = useRef(onStatusSize);
  onStatusSizeRef.current = onStatusSize;
  useEffect(() => {
    onStatusSizeRef.current?.(statusSize);
  }, [statusSize]);

  const {
    air_speed, ground_speed, alt_rel, heading,
    pitch, roll, battery, voltage, current, status_texts,
    gps_fix, gps_hacc, link_ok, link_quality,
  } = vehicle;
  const { attitude, heading: headingBox, status, speed, altitude, strip } = layout;

  return (
    <>
      {/* Attitude indicator — full height, centred, between the layout's side insets */}
      <div data-testid="hud-attitude" style={{
        position: 'absolute',
        top: 0,
        bottom: 0,
        left: attitude.left,
        right: attitude.right,
        pointerEvents: 'none',
        zIndex: 5,
      }}>
        <AttitudeIndicator pitch={pitch} roll={roll} />
      </div>

      {/* Heading tape — top, centred unless the layout gives its left edge */}
      <div style={{
        position: 'absolute',
        top: headingBox.top,
        ...(headingBox.left == null ? { left: '50%', transform: 'translateX(-50%)' } : { left: headingBox.left }),
        pointerEvents: 'none',
        zIndex: 10,
      }}>
        <HeadingTape heading={heading} width={headingBox.width} height={headingBox.height} />
      </div>

      {/* Status + Battery — top right, phone-style row */}
      <div ref={statusRef} data-testid="hud-status" style={{
        position: 'absolute',
        top: status.top,
        right: status.right,
        pointerEvents: 'none',
        zIndex: 10,
        display: 'flex',
        alignItems: 'flex-start',
        gap: 4,
      }}>
        <StatusBarIndicator gpsFix={gps_fix} gpsHacc={gps_hacc} linkOk={link_ok} linkQuality={link_quality} />
        <BatteryIndicator battery={battery} voltage={voltage} current={current} />
      </div>

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

      {/* STATUSTEXT warnings — bottom center */}
      <div style={{
        position: 'absolute',
        bottom: strip.bottom,
        left: '50%',
        transform: 'translateX(-50%)',
        pointerEvents: 'none',
        zIndex: 10,
      }}>
        <StatusStrip statusTexts={status_texts} />
      </div>
    </>
  );
}
