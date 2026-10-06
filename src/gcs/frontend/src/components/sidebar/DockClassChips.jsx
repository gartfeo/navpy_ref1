import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import DockClassIcon, { DOCK_CLASS_IDS } from './dockClassIcons.jsx';

/**
 * Single-row toggle-chip selector for dock classes (Small / Medium / Large).
 * Replaces the old stacked checkbox list: compact, icon-labelled, wraps only when
 * the row can't fit. Selected chips fill with the accent color; the rest are
 * outlined. Reused by both the mission-wide selector and the per-UAV override.
 *
 * @param {{
 *   selected: string[],
 *   onToggle: (id: string) => void,
 *   compact?: boolean,      // compact class labels for tight per-UAV rows
 *   iconSize?: number,
 * }} props
 */
export default function DockClassChips({ selected, onToggle, compact = false, iconSize = 16 }) {
  const { t } = useTranslation();
  const sel = selected || [];

  return (
    <div style={{ display: 'flex', gap: 6, flexWrap: 'wrap' }}>
      {DOCK_CLASS_IDS.map((id) => {
        const isSel = sel.includes(id);
        const label = compact
          ? t('planningSidebar.dockClassesShort.' + id, t('planningSidebar.dockClasses.' + id))
          : t('planningSidebar.dockClasses.' + id);
        return (
          <button
            key={id}
            type="button"
            onClick={() => onToggle(id)}
            aria-pressed={isSel}
            title={t('planningSidebar.dockClasses.' + id)}
            style={{
              display: 'flex',
              alignItems: 'center',
              gap: 5,
              padding: compact ? '3px 8px' : '5px 11px',
              borderRadius: 14,
              cursor: 'pointer',
              fontSize: compact ? 11 : 12,
              fontWeight: 500,
              lineHeight: 1,
              whiteSpace: 'nowrap',
              background: isSel ? colors.accent : 'transparent',
              color: isSel ? '#04222b' : colors.textDim,
              border: `1px solid ${isSel ? colors.accent : colors.border}`,
            }}
          >
            <DockClassIcon id={id} size={compact ? iconSize - 2 : iconSize} />
            {label}
          </button>
        );
      })}
    </div>
  );
}
