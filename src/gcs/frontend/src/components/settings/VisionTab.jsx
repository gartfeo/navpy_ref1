import React, { useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import { SettingsField, SettingsSection, NumericInput, inputStyle } from './SettingsField.jsx';
import ProfileSelector from './ProfileSelector.jsx';
import { applyPresetEdit, mergeDisplayPresets } from './visionTabUtils.js';

const presetGridCell = { ...inputStyle, maxWidth: 72, fontSize: 12, padding: '3px 6px' };

export default function VisionTab({ draft, setDraft, preSaveRef, preResetRef }) {
  const { t } = useTranslation();
  const c = draft.camera || {};
  const f = draft.flight || {};
  const setF = (k, v) => setDraft({ ...draft, flight: { ...f, [k]: v } });

  // Dock presets are owned by the active vision profile (vision_profiles.json),
  // NOT settings. `catalogPresets` is the catalog's resolved base for the active
  // profile (reported up by ProfileSelector); `presetOverrides` is the live edit
  // buffer flushed to PUT /api/vision-profiles/{profile} on Apply. Merging the two
  // gives the displayed values without writing presets into the settings draft.
  const [catalogPresets, setCatalogPresets] = useState({});
  const [presetOverrides, setPresetOverrides] = useState({});

  const setPreset = (key, field, value) => {
    setPresetOverrides((prev) => applyPresetEdit(catalogPresets, prev, key, field, value));
  };

  const presets = mergeDisplayPresets(catalogPresets, presetOverrides);
  const presetKeys = Object.keys(presets);

  // Write only the active SELECTION (vision_profile / vision_device / vision_zoom)
  // into the settings draft. Pitch + presets persist to the profile JSON, not here.
  const applySelection = (vals) => {
    setDraft({ ...draft, camera: { ...c, ...vals } });
  };

  return (
    <>
      <ProfileSelector
        draft={draft}
        onApply={applySelection}
        preSaveRef={preSaveRef}
        preResetRef={preResetRef}
        presetOverrides={presetOverrides}
        setPresetOverrides={setPresetOverrides}
        onCatalogPresets={setCatalogPresets}
      />
      <SettingsSection title={t('visionTab.title')}>
        {presetKeys.length > 0 && (
          <>
            <div style={{ display: 'flex', gap: 12, alignItems: 'flex-end', marginBottom: 4 }}>
              <span style={{ fontSize: 12, color: colors.text, minWidth: 200, paddingBottom: 4 }}>{t('visionTab.searchAltitude')}</span>
              {presetKeys.map((key) => (
                <div key={key} style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 2 }}>
                  <span style={{ fontSize: 11, color: colors.textDim }}>{t('planningSidebar.dockPreset', { defaultValue: presets[key].label || key })}</span>
                  <NumericInput
                    value={presets[key].altitude_m}
                    onChange={(v) => setPreset(key, 'altitude_m', v)}
                    step={10} min={10} max={10000}
                    style={presetGridCell}
                  />
                </div>
              ))}
            </div>
            {/* Per-class confirmation pixel threshold. Persists to the profile's
                detector.dock_presets[*].min_pixel_size via the same PUT flush;
                a low value lets final approach confirm on fixed/low-zoom
                cameras that can't reach recognition-size pixels. */}
            <div style={{ display: 'flex', gap: 12, alignItems: 'flex-end', marginBottom: 4 }}>
              <span style={{ fontSize: 12, color: colors.text, minWidth: 200, paddingBottom: 4 }}>{t('visionTab.minPixelSize')}</span>
              {presetKeys.map((key) => (
                <div key={key} style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 2 }}>
                  <NumericInput
                    value={presets[key].min_pixel_size}
                    onChange={(v) => setPreset(key, 'min_pixel_size', v)}
                    step={1} min={1} max={1000}
                    style={presetGridCell}
                  />
                </div>
              ))}
            </div>
          </>
        )}
        <SettingsField label={t('visionTab.overlapFraction')} value={f.overlap_fraction} onChange={(v) => setF('overlap_fraction', v)} step={0.05} min={0} max={0.9} />
        <SettingsField label={t('visionTab.altitudeSeparation')} value={f.altitude_separation_m} onChange={(v) => setF('altitude_separation_m', v)} step={5} />
        <SettingsField label={t('visionTab.uavsPerSet')} value={f.uavs_per_set} onChange={(v) => setF('uavs_per_set', Math.max(1, Math.round(v)))} step={1} />
      </SettingsSection>
    </>
  );
}
