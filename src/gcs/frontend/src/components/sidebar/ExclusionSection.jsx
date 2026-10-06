import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import { Divider, Label } from './SidebarPrimitives';

const KEEPOUT_COLOR = '#ff3b30';

// conflict.kind -> i18n key for the "what" phrase (corridor leg / scan track /
// confirmation orbit). Dispatch map keeps the wording data-driven.
const KIND_KEY = {
  corridor: 'exclusionSection.kind.corridor',
  track: 'exclusionSection.kind.track',
  orbit: 'exclusionSection.kind.orbit',
};

/**
 * Keep-out (exclusion) management + non-blocking conflict warnings.
 *
 * Renders only when there is at least one keep-out or an active conflict, so it
 * stays out of the way until the operator draws a keep-out. Deleting a keep-out
 * or clearing all is done here; drawing happens via the map toolbar.
 */
export default function ExclusionSection({
  exclusionPolygons,
  exclusionConflicts,
  selfIntersectingExclusions,
  onRemoveExclusion,
  onClearExclusions,
}) {
  const { t } = useTranslation();
  const polys = exclusionPolygons || [];
  const conflicts = exclusionConflicts || [];
  if (polys.length === 0 && conflicts.length === 0) return null;

  // Which keep-outs are in conflict (for per-row emphasis).
  const conflictIdx = new Set(conflicts.map((c) => c.exclusionIndex));
  // Self-intersecting rings: ArduPilot's even-odd test shrinks the enforced
  // area, so the drawn outline overstates the real keep-out.
  const selfIntersectIdx = new Set(selfIntersectingExclusions || []);

  return (
    <>
      <Divider />
      <div style={{ display: 'flex', alignItems: 'center', gap: 6, marginBottom: 8 }}>
        <svg width="14" height="14" viewBox="0 0 14 14" fill="none" style={{ flexShrink: 0 }}>
          <circle cx="7" cy="7" r="5.5" stroke={KEEPOUT_COLOR} strokeWidth="1.4" />
          <line x1="3.1" y1="3.1" x2="10.9" y2="10.9" stroke={KEEPOUT_COLOR} strokeWidth="1.5" strokeLinecap="round" />
        </svg>
        <span style={{ color: colors.textBright, fontSize: 13, fontWeight: 700 }}>
          {t('exclusionSection.title')}
        </span>
        <span style={{ flex: 1 }} />
        {polys.length > 0 && (
          <button
            onClick={onClearExclusions}
            style={{
              background: 'transparent',
              border: 'none',
              color: colors.textDim,
              cursor: 'pointer',
              fontSize: 11,
              padding: '0 2px',
              textDecoration: 'underline',
            }}
          >
            {t('exclusionSection.clearAll')}
          </button>
        )}
      </div>

      {/* Keep-out list — each row deletable */}
      {polys.length > 0 ? (
        <div style={{ display: 'flex', flexDirection: 'column', gap: 4, marginBottom: conflicts.length > 0 ? 10 : 0 }}>
          {polys.map((ring, i) => {
            const inConflict = conflictIdx.has(i);
            return (
              <div
                key={i}
                style={{
                  display: 'flex', alignItems: 'center', gap: 6,
                  padding: '4px 8px',
                  borderRadius: 4,
                  background: inConflict ? 'rgba(255, 23, 68, 0.10)' : colors.surface,
                  border: `1px solid ${inConflict ? KEEPOUT_COLOR : colors.border}`,
                }}
              >
                <span style={{ width: 8, height: 8, borderRadius: 2, background: KEEPOUT_COLOR, flexShrink: 0 }} />
                <span style={{ color: colors.textBright, fontSize: 12 }}>
                  {t('exclusionSection.item', { index: i + 1 })}
                </span>
                <span style={{ color: colors.textDim, fontSize: 11 }}>
                  {t('exclusionSection.vertices', { count: (ring || []).length })}
                </span>
                {selfIntersectIdx.has(i) && (
                  <span
                    title={t('exclusionSection.selfIntersectHint')}
                    style={{
                      background: 'rgba(255, 23, 68, 0.16)', color: '#ff5252',
                      border: `1px solid ${KEEPOUT_COLOR}`, borderRadius: 4,
                      padding: '0 5px', fontSize: 10,
                    }}
                  >
                    {t('exclusionSection.selfIntersect')}
                  </span>
                )}
                <span style={{ flex: 1 }} />
                <button
                  onClick={() => onRemoveExclusion(i)}
                  title={t('exclusionSection.remove')}
                  aria-label={t('exclusionSection.remove')}
                  style={{
                    background: 'transparent', border: 'none', color: colors.textDim,
                    cursor: 'pointer', fontSize: 14, lineHeight: 1, padding: '0 2px',
                  }}
                >
                  {'×'}
                </button>
              </div>
            );
          })}
        </div>
      ) : (
        <div style={{ color: colors.textDim, fontSize: 11, marginBottom: 10 }}>
          {t('exclusionSection.drawHint')}
        </div>
      )}

      {/* Conflict warnings — non-blocking; upload still allowed */}
      {conflicts.length > 0 && (
        <div style={{
          border: `1px solid ${KEEPOUT_COLOR}`,
          borderRadius: 6,
          padding: 8,
          background: 'rgba(255, 59, 48, 0.08)',
        }}>
          <div style={{ color: KEEPOUT_COLOR, fontSize: 12, fontWeight: 700, marginBottom: 4 }}>
            {'⚠ '}{t('exclusionSection.conflictTitle')}
          </div>
          <div style={{ display: 'flex', flexDirection: 'column', gap: 3 }}>
            {conflicts.map((c, i) => (
              <div key={i} style={{ color: colors.text, fontSize: 11, lineHeight: 1.4 }}>
                {t('exclusionSection.conflict', {
                  label: c.label,
                  what: t(KIND_KEY[c.kind] || 'exclusionSection.kind.corridor'),
                  index: c.exclusionIndex + 1,
                })}
              </div>
            ))}
          </div>
          <div style={{ color: colors.textDim, fontSize: 11, marginTop: 6 }}>
            {t('exclusionSection.conflictHint')}
          </div>
        </div>
      )}
    </>
  );
}
