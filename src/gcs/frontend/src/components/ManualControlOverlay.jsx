import React from 'react';
import { useTranslation } from 'react-i18next';
import VirtualJoystick from './VirtualJoystick';

/** Stick size and its inset from the map's bottom corners; App places the RC confirm list beside the right stick. */
export const STICK_SIZE_PX = 150;
export const STICK_INSET_PX = 24;

/**
 * Overlay that positions left + right virtual joysticks at the bottom
 * corners of the map. Rendered inside the map container div.
 */
export default function ManualControlOverlay({ onLeftMove, onRightMove, size = STICK_SIZE_PX, initialThrottleY }) {
  const { t } = useTranslation();
  return (
    <>
      <div style={{
        position: 'absolute',
        bottom: STICK_INSET_PX,
        left: STICK_INSET_PX,
        zIndex: 10,
        pointerEvents: 'auto',
      }}>
        <VirtualJoystick size={size} onMove={onLeftMove} label={t('manualControl.thrYaw')} stickyY initialY={initialThrottleY} />
      </div>
      <div style={{
        position: 'absolute',
        bottom: STICK_INSET_PX,
        right: STICK_INSET_PX,
        zIndex: 10,
        pointerEvents: 'auto',
      }}>
        <VirtualJoystick size={size} onMove={onRightMove} label={t('manualControl.pitchRoll')} />
      </div>
    </>
  );
}
