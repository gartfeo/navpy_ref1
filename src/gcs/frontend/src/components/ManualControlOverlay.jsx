import React from 'react';
import { useTranslation } from 'react-i18next';
import VirtualJoystick from './VirtualJoystick';

/**
 * Overlay that positions left + right virtual joysticks at the bottom
 * corners of the map. Rendered inside the map container div.
 */
export default function ManualControlOverlay({ onLeftMove, onRightMove, size = 150, initialThrottleY }) {
  const { t } = useTranslation();
  return (
    <>
      <div style={{
        position: 'absolute',
        bottom: 24,
        left: 24,
        zIndex: 10,
        pointerEvents: 'auto',
      }}>
        <VirtualJoystick size={size} onMove={onLeftMove} label={t('manualControl.thrYaw')} stickyY initialY={initialThrottleY} />
      </div>
      <div style={{
        position: 'absolute',
        bottom: 24,
        right: 24,
        zIndex: 10,
        pointerEvents: 'auto',
      }}>
        <VirtualJoystick size={size} onMove={onRightMove} label={t('manualControl.pitchRoll')} />
      </div>
    </>
  );
}
