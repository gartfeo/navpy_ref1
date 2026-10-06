import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import { SettingsField, SettingsSection } from './SettingsField.jsx';

/**
 * Behavior-preserving defaults for the DEV-tunable launch knobs. Mirrors
 * LaunchSettings (settings_model.py). Used to decide whether any tuning is
 * non-default (for the non-DEV "tuning active" note).
 */
export const LAUNCH_TUNING_DEFAULTS = Object.freeze({
  container_settle_s: 0.3,
  bungee_settle_s: 0.0,
  arm_timeout_s: 10.0,
  bungee_arm_timeout_s: 5.0,
  container_stagger_s: 0.0,
  container_gap_s: 0.0,
  bungee_stagger_s: 0.0,
  airborne_require_armed: false,
  airborne_require_throttle: false,
  airborne_min_throttle_pct: 20.0,
  min_climb_rate_ms: 0.0,
  climb_confirm_s: 0.0,
  check_gps_enabled: true,
  check_gps_acc_enabled: false,
  max_gps_hacc_m: 1.0,
  check_throttle_enabled: true,
  max_throttle_rc3: 1050,
  check_battery_enabled: true,
  min_battery_pct: 15.0,
  check_prearm_enabled: true,
  block_on_unknown_battery: false,
});

/** True if any DEV-tunable launch value differs from its default. */
export function hasNonDefaultTuning(launch) {
  const l = launch || {};
  return Object.keys(LAUNCH_TUNING_DEFAULTS).some(
    (k) => l[k] !== undefined && l[k] !== LAUNCH_TUNING_DEFAULTS[k],
  );
}

/**
 * DEV-only advanced launch tuning: per-path timing, container airborne/climb
 * confirmation, and per-check launch-readiness gates. Backend always applies
 * the saved values; this section only controls editing.
 */
export default function LaunchTuningSection({ l, set, isContainer }) {
  const { t } = useTranslation();
  const num = (k) => (l[k] ?? LAUNCH_TUNING_DEFAULTS[k]);
  const bool = (k) => (l[k] ?? LAUNCH_TUNING_DEFAULTS[k]);

  return (
    <>
      {/* Timing — only the fields for the selected launch type */}
      <SettingsSection title={t('launchTab.tuningTiming')}>
        {isContainer ? (
          <>
            <SettingsField label={t('launchTab.containerSettle')} info={t('launchTab.containerSettleInfo')} value={num('container_settle_s')} onChange={(v) => set('container_settle_s', v)} step={0.1} min={0} />
            <SettingsField label={t('launchTab.containerStagger')} info={t('launchTab.containerStaggerInfo')} value={num('container_stagger_s')} onChange={(v) => set('container_stagger_s', v)} step={0.5} min={0} />
            <SettingsField label={t('launchTab.containerGap')} info={t('launchTab.containerGapInfo')} value={num('container_gap_s')} onChange={(v) => set('container_gap_s', v)} step={0.5} min={0} />
          </>
        ) : (
          <>
            <SettingsField label={t('launchTab.bungeeSettle')} info={t('launchTab.bungeeSettleInfo')} value={num('bungee_settle_s')} onChange={(v) => set('bungee_settle_s', v)} step={0.1} min={0} />
            <SettingsField label={t('launchTab.bungeeArmTimeout')} info={t('launchTab.bungeeArmTimeoutInfo')} value={num('bungee_arm_timeout_s')} onChange={(v) => set('bungee_arm_timeout_s', v)} step={1} min={0.1} />
            <SettingsField label={t('launchTab.bungeeStagger')} info={t('launchTab.bungeeStaggerInfo')} value={num('bungee_stagger_s')} onChange={(v) => set('bungee_stagger_s', v)} step={0.5} min={0} />
          </>
        )}
      </SettingsSection>

      {/* Airborne / climb confirmation is booster (container) only */}
      {isContainer && (
        <SettingsSection title={t('launchTab.tuningAirborne')}>
          <div style={{ fontSize: 11, color: colors.textDim, marginBottom: 4 }}>
            {t('launchTab.tuningAirborneHelp')}
          </div>
          <SettingsField type="checkbox" label={t('launchTab.requireArmed')} info={t('launchTab.requireArmedInfo')} value={bool('airborne_require_armed')} onChange={(v) => set('airborne_require_armed', v)} />
          <SettingsField type="checkbox" label={t('launchTab.requireThrottle')} info={t('launchTab.requireThrottleInfo')} value={bool('airborne_require_throttle')} onChange={(v) => set('airborne_require_throttle', v)} />
          <SettingsField label={t('launchTab.minThrottlePct')} info={t('launchTab.minThrottlePctInfo')} value={num('airborne_min_throttle_pct')} onChange={(v) => set('airborne_min_throttle_pct', v)} step={5} min={0} max={100} />
          <SettingsField label={t('launchTab.minClimbRate')} info={t('launchTab.minClimbRateInfo')} value={num('min_climb_rate_ms')} onChange={(v) => set('min_climb_rate_ms', v)} step={0.5} min={0} description={t('launchTab.climbDisabledHint')} />
          <SettingsField label={t('launchTab.climbConfirm')} info={t('launchTab.climbConfirmInfo')} value={num('climb_confirm_s')} onChange={(v) => set('climb_confirm_s', v)} step={0.5} min={0} />
        </SettingsSection>
      )}

      <SettingsSection title={t('launchTab.tuningReadiness')}>
        <div style={{ fontSize: 11, color: colors.textDim, marginBottom: 4 }}>
          {t('launchTab.tuningReadinessHelp')}
        </div>
        <SettingsField type="checkbox" label={t('launchTab.checkPrearm')} info={t('launchTab.checkPrearmInfo')} value={bool('check_prearm_enabled')} onChange={(v) => set('check_prearm_enabled', v)} />
        <SettingsField type="checkbox" label={t('launchTab.checkGps')} info={t('launchTab.checkGpsInfo')} value={bool('check_gps_enabled')} onChange={(v) => set('check_gps_enabled', v)} />
        <SettingsField type="checkbox" label={t('launchTab.checkGpsAcc')} info={t('launchTab.checkGpsAccInfo')} value={bool('check_gps_acc_enabled')} onChange={(v) => set('check_gps_acc_enabled', v)} />
        <SettingsField label={t('launchTab.maxGpsHacc')} info={t('launchTab.maxGpsHaccInfo')} value={num('max_gps_hacc_m')} onChange={(v) => set('max_gps_hacc_m', v)} step={0.1} min={0} />
        <SettingsField type="checkbox" label={t('launchTab.checkThrottle')} info={t('launchTab.checkThrottleInfo')} value={bool('check_throttle_enabled')} onChange={(v) => set('check_throttle_enabled', v)} />
        <SettingsField label={t('launchTab.maxThrottleRc3')} info={t('launchTab.maxThrottleRc3Info')} value={num('max_throttle_rc3')} onChange={(v) => set('max_throttle_rc3', Math.round(v))} step={10} min={900} max={2000} />
        <SettingsField type="checkbox" label={t('launchTab.checkBattery')} info={t('launchTab.checkBatteryInfo')} value={bool('check_battery_enabled')} onChange={(v) => set('check_battery_enabled', v)} />
        <SettingsField label={t('launchTab.minBatteryPct')} info={t('launchTab.minBatteryPctInfo')} value={num('min_battery_pct')} onChange={(v) => set('min_battery_pct', v)} step={1} min={0} max={100} />
        <SettingsField type="checkbox" label={t('launchTab.blockUnknownBattery')} info={t('launchTab.blockUnknownBatteryInfo')} value={bool('block_on_unknown_battery')} onChange={(v) => set('block_on_unknown_battery', v)} description={t('launchTab.blockUnknownBatteryHint')} />
      </SettingsSection>
    </>
  );
}
