import React from 'react';
import { useTranslation } from 'react-i18next';
import { SettingsField, SettingsSection } from './SettingsField.jsx';

export default function MapTab({ draft, setDraft }) {
  const { t } = useTranslation();
  const m = draft.map_display || {};
  const set = (k, v) => setDraft({ ...draft, map_display: { ...m, [k]: v } });
  return (
    <SettingsSection title={t('mapTab.title')}>
      <SettingsField label={t('mapTab.defaultLat')} value={m.default_lat} onChange={(v) => set('default_lat', v)} step={0.001} />
      <SettingsField label={t('mapTab.defaultLon')} value={m.default_lon} onChange={(v) => set('default_lon', v)} step={0.001} />
      <SettingsField label={t('mapTab.defaultZoom')} value={m.default_zoom} onChange={(v) => set('default_zoom', v)} step={0.5} />
    </SettingsSection>
  );
}
