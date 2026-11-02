import React from 'react';
import HeadingTape from './hud/HeadingTape';
import AttitudeIndicator from './hud/AttitudeIndicator';
import SpeedTape from './hud/SpeedTape';
import AltitudeTape from './hud/AltitudeTape';
import StatusStrip from './hud/StatusStrip';
import BatteryIndicator from './hud/BatteryIndicator';
import StatusBarIndicator from './hud/StatusBarIndicator';
import { useTelemetryStore } from '../hooks/useTelemetryStore';

// The tapes take no taps: below the RC confirm list (9) and the first-person
// tag and held banner (8), so those win where they meet; above the attitude (5).
const TAPE_Z_INDEX = 7;

/**
 * @param {object} p
 * @param {number} p.sysId
 * @param {{top: number, height: number}} p.tapeBox - the altitude tape's, from utils/appLayout.js hudTapeBox.
 * @param {{left: number, top: number, height: number}} p.speedBox - from utils/appLayout.js speedTapeBox.
 */
export default function TelemetryHud({ sysId, tapeBox, speedBox }) {
  const vehicle = useTelemetryStore(s => s.getVehicles()[sysId]);
  if (!vehicle) return null;

  const {
    air_speed, ground_speed, alt_rel, heading,
    pitch, roll, battery, voltage, current, status_texts,
    gps_fix, gps_hacc, link_ok, link_quality,
  } = vehicle;

  return (
    <>
      {/* Attitude indicator — full screen background */}
      <div style={{
        position: 'absolute',
        inset: 0,
        pointerEvents: 'none',
        zIndex: 5,
      }}>
        <AttitudeIndicator pitch={pitch} roll={roll} />
      </div>

      {/* Heading tape — top center */}
      <div style={{
        position: 'absolute',
        top: 12,
        left: '50%',
        transform: 'translateX(-50%)',
        pointerEvents: 'none',
        zIndex: 10,
      }}>
        <HeadingTape heading={heading} />
      </div>

      {/* Status + Battery — top right, phone-style row */}
      <div style={{
        position: 'absolute',
        top: 12,
        right: 12,
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
      <div style={{
        position: 'absolute',
        top: speedBox.top,
        left: speedBox.left,
        pointerEvents: 'none',
        zIndex: TAPE_Z_INDEX,
      }}>
        <SpeedTape airSpeed={air_speed} groundSpeed={ground_speed} height={speedBox.height} />
      </div>

      {/* Altitude tape — right, above the stick */}
      <div style={{
        position: 'absolute',
        top: tapeBox.top,
        right: 12,
        pointerEvents: 'none',
        zIndex: TAPE_Z_INDEX,
      }}>
        <AltitudeTape altitude={alt_rel} height={tapeBox.height} />
      </div>

      {/* STATUSTEXT warnings — bottom center */}
      <div style={{
        position: 'absolute',
        bottom: 60,
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
