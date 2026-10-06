import React from 'react';
import HeadingTape from './hud/HeadingTape';
import AttitudeIndicator from './hud/AttitudeIndicator';
import SpeedTape from './hud/SpeedTape';
import AltitudeTape from './hud/AltitudeTape';
import StatusStrip from './hud/StatusStrip';
import BatteryIndicator from './hud/BatteryIndicator';
import StatusBarIndicator from './hud/StatusBarIndicator';
import { useTelemetryStore } from '../hooks/useTelemetryStore';

export default function TelemetryHud({ sysId }) {
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

      {/* Speed tape — left center */}
      <div style={{
        position: 'absolute',
        top: '50%',
        left: 12,
        transform: 'translateY(-50%)',
        pointerEvents: 'none',
        zIndex: 10,
      }}>
        <SpeedTape airSpeed={air_speed} groundSpeed={ground_speed} />
      </div>

      {/* Altitude tape — right center */}
      <div style={{
        position: 'absolute',
        top: '50%',
        right: 12,
        transform: 'translateY(-50%)',
        pointerEvents: 'none',
        zIndex: 10,
      }}>
        <AltitudeTape altitude={alt_rel} />
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
