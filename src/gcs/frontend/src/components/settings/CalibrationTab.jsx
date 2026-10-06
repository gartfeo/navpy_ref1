import React, { useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import RadioCalSection from './calibration/RadioCalSection.jsx';
import CompassCalSection from './calibration/CompassCalSection.jsx';
import AccelCalSection from './calibration/AccelCalSection.jsx';

/**
 * Unified Calibration tab (INTEG-04, D-04): a mini-SettingsModal shell
 * hosting the Radio / Compass / Accel sections. Each section is ported from
 * its own WIP branch (radio -> compass -> accel, newest base first) and
 * plugged into this static array + component map — the same TABS /
 * TAB_COMPONENTS idiom SettingsModal itself uses, scoped down one level.
 *
 * Compass and Accel are wired in by later plan tasks; until then they render
 * a stable "not yet available" placeholder rather than being absent from the
 * section list, so the sub-tab layout itself does not change again later.
 */
const SECTIONS = ['Radio', 'Compass', 'Accel'];

const SECTION_COMPONENTS = {
  Radio: RadioCalSection,
  Compass: CompassCalSection,
  Accel: AccelCalSection,
};

export default function CalibrationTab(props) {
  const { t } = useTranslation();
  const [activeSection, setActiveSection] = useState(SECTIONS[0]);

  const SECTION_LABELS = {
    Radio: t('settings.calibration.radio'),
    Compass: t('settings.calibration.compass'),
    Accel: t('settings.calibration.accel'),
  };

  const SectionComponent = SECTION_COMPONENTS[activeSection];

  return (
    <div style={{ display: 'flex', flexDirection: 'column', minHeight: 0, flex: 1 }}>
      {/* Sub-tabs */}
      <div style={{
        display: 'flex',
        flexWrap: 'wrap',
        gap: 0,
        borderBottom: `1px solid ${colors.border}`,
        marginBottom: 12,
      }}>
        {SECTIONS.map((section) => (
          <button
            key={section}
            onClick={() => setActiveSection(section)}
            style={{
              flexShrink: 0,
              whiteSpace: 'nowrap',
              padding: '6px 12px',
              fontSize: 12,
              fontWeight: activeSection === section ? 600 : 400,
              color: activeSection === section ? colors.accent : colors.textDim,
              background: 'none',
              border: 'none',
              borderBottom: activeSection === section ? `2px solid ${colors.accent}` : '2px solid transparent',
              cursor: 'pointer',
            }}
          >
            {SECTION_LABELS[section] || section}
          </button>
        ))}
      </div>

      {/* Active section */}
      <div style={{ flex: 1, overflow: 'auto', minHeight: 0 }}>
        {SectionComponent ? (
          <SectionComponent {...props} />
        ) : (
          <div style={{ fontSize: 13, color: colors.textDim, padding: '8px 0' }}>
            {t('settings.calibration.notYetAvailable')}
          </div>
        )}
      </div>
    </div>
  );
}
