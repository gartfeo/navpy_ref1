import React, { useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import { pickDisplayValue } from '../../utils/aasParams';

/**
 * Planning sidebar confirmation editor.
 *
 * Reads and writes the per-vehicle AAS confirmation params (nav_auto_cm,
 * nav_cwt, nav_cm_fl) directly through the useAasParams hook -- the
 * vehicle is the source of truth. Edits fan out to all currently
 * connected vehicles via uploadPatch (which sends MAVLink PARAM_SET).
 *
 * When no vehicle is connected, edits are held in sessionDraft (RAM
 * only, lost on reload) and are NOT auto-pushed to vehicles on connect.
 *
 * Cross-vehicle "Mixed" displays as a placeholder; the operator's first
 * explicit selection commits to all connected vehicles.
 *
 * The numeric wait-time input keeps a local pending string so the field
 * does not flicker between user typing and the consensus value mid-edit;
 * it commits on blur or Enter (parsed via parseFloat, NaN ignored).
 */
export default function ConfirmSection({ vehicleList, aasParams }) {
  const { t } = useTranslation();

  const sysIds = (vehicleList || []).map((v) => v.sys_id);
  const sessionDraft = aasParams.sessionDraft;

  const writeKey = (key, value) => {
    if (sysIds.length === 0) {
      aasParams.setSessionDraft(key, value);
    } else {
      aasParams.uploadPatch(sysIds, { [key]: value });
    }
  };

  // ---- Mode (nav_auto_cm) ----
  const modeConsensus = aasParams.getConfirmedConsensus('nav_auto_cm');
  const modeValue = pickDisplayValue(modeConsensus, sessionDraft, 'nav_auto_cm');
  const modeMixed = modeConsensus.state === 'mixed';
  const isManual = modeValue === false;

  // ---- On-timeout (nav_cm_fl) ----
  const failConsensus = aasParams.getConfirmedConsensus('nav_cm_fl');
  const failValue = pickDisplayValue(failConsensus, sessionDraft, 'nav_cm_fl');
  const failMixed = failConsensus.state === 'mixed';

  // ---- Wait time (nav_cwt) ----
  const cwtConsensus = aasParams.getConfirmedConsensus('nav_cwt');
  const cwtValue = pickDisplayValue(cwtConsensus, sessionDraft, 'nav_cwt');
  const cwtMixed = cwtConsensus.state === 'mixed';

  // Local pending string for the numeric input. Cleared after commit so
  // the displayed value falls back to the consensus / session value.
  const [cwtPending, setCwtPending] = useState('');

  const commitCwt = (raw) => {
    const next = parseFloat(raw);
    if (!Number.isFinite(next)) { setCwtPending(''); return; }
    writeKey('nav_cwt', next);
    setCwtPending('');
  };

  const fieldStyle = {
    width: 60,
    background: colors.surface,
    border: `1px solid ${colors.border}`,
    borderRadius: 4,
    padding: '3px 6px',
    color: colors.textBright,
    fontSize: 12,
    textAlign: 'center',
  };
  const selectStyle = {
    background: colors.surface,
    color: colors.textBright,
    border: `1px solid ${colors.border}`,
    borderRadius: 4,
    padding: '3px 6px',
    fontSize: 12,
    cursor: 'pointer',
  };

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 6, marginBottom: 8 }}>
      <select
        value={modeMixed || modeValue === null ? '' : (modeValue ? 'automatic' : 'manual')}
        onChange={(e) => writeKey('nav_auto_cm', e.target.value === 'automatic')}
        title="AAS_NAV_AUTO_CM -- Automatically approve available tasks without operator input."
        style={selectStyle}
      >
        {(modeMixed || modeValue === null) && (
          <option value="" disabled style={{ background: colors.surface, color: colors.textDim }}>
            {modeMixed ? 'Mixed' : '—'}
          </option>
        )}
        <option value="automatic" style={{ background: colors.surface, color: colors.textBright }}>{t('confirmSection.automatic')}</option>
        <option value="manual" style={{ background: colors.surface, color: colors.textBright }}>{t('confirmSection.manual')}</option>
      </select>
      {isManual && (
        <div
          style={{ display: 'flex', alignItems: 'center', gap: 5, fontSize: 12, color: colors.text, flexWrap: 'wrap' }}
          title="AAS_NAV_CWT / AAS_NAV_CM_FL -- Action taken if no operator response within wait time."
        >
          <span style={{ color: colors.textDim }}>{t('confirmSection.noResponseAfter')}</span>
          <input
            type="number"
            value={cwtPending !== '' ? cwtPending : (cwtValue ?? '')}
            step={1}
            placeholder={cwtMixed ? 'Mixed' : ''}
            onChange={(e) => setCwtPending(e.target.value)}
            onBlur={() => { if (cwtPending !== '') commitCwt(cwtPending); else setCwtPending(''); }}
            onKeyDown={(e) => { if (e.key === 'Enter') e.target.blur(); }}
            style={fieldStyle}
          />
          <span style={{ color: colors.textDim }}>{t('confirmSection.secondsArrow')}</span>
          <select
            value={failMixed || failValue === null ? '' : (failValue ? 'approve' : 'deny')}
            onChange={(e) => writeKey('nav_cm_fl', e.target.value === 'approve')}
            style={selectStyle}
          >
            {(failMixed || failValue === null) && (
              <option value="" disabled style={{ background: colors.surface, color: colors.textDim }}>
                {failMixed ? 'Mixed' : '—'}
              </option>
            )}
            <option value="approve" style={{ background: colors.surface, color: colors.textBright }}>{t('confirmSection.approve')}</option>
            <option value="deny" style={{ background: colors.surface, color: colors.textBright }}>{t('confirmSection.deny')}</option>
          </select>
        </div>
      )}
    </div>
  );
}

