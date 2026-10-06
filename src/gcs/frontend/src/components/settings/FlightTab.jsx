import React from 'react';
import { useTranslation } from 'react-i18next';
import { SettingsField, SettingsSection } from './SettingsField.jsx';

export default function FlightTab({ draft, setDraft }) {
  const { t } = useTranslation();
  const f = draft.flight || {};
  const set = (k, v) => setDraft({ ...draft, flight: { ...f, [k]: v } });
  return (
    <SettingsSection title={t('flightTab.title')}>
      <SettingsField label={t('flightTab.cruiseSpeed')} value={f.cruise_speed_ms} onChange={(v) => set('cruise_speed_ms', v)} step={0.5} />
      <SettingsField label={t('flightTab.flightBudget')} value={f.flight_budget_km} onChange={(v) => set('flight_budget_km', v)} step={1} />
      <SettingsField label={t('flightTab.safetyReserve')} value={f.safety_reserve_km} onChange={(v) => set('safety_reserve_km', v)} step={1} />
      <SettingsField label={t('flightTab.minLaunchBuffer')} value={f.min_launch_zone_buffer_km} onChange={(v) => set('min_launch_zone_buffer_km', v)} step={0.5} />
      <SettingsField label={t('flightTab.takeoffAltitude')} value={f.takeoff_altitude_m} onChange={(v) => set('takeoff_altitude_m', v)} step={5} min={1} info={t('flightTab.takeoffAltitudeInfo')} />
    </SettingsSection>
  );
}
