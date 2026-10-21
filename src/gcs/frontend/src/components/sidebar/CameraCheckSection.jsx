import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import { describeCameraCheck } from '../../utils/planCameraCheck';
import { zoneLabel } from './DeliveryHubAssignmentSection';

/**
 * Planning sidebar: why the active camera cannot detect the plan's sim docks,
 * and what to change. Renders nothing for a compatible plan.
 */
export default function CameraCheckSection({ result, zones }) {
  const { t } = useTranslation();
  if (!result) return null;
  if (result.status === 'error') {
    return (
      <div data-testid="camera-check" style={{ ...boxStyle(colors.warning), marginBottom: 12 }}>
        {t('cameraCheck.couldNotRunShort', { error: result.error })}
      </div>
    );
  }
  const { problems, notes } = describeCameraCheck(result, t, (i) => zoneLabel(zones || [], i));
  if (!problems.length && !notes.length) return null;
  const hasError = problems.some((p) => p.severity === 'error');
  return (
    <div data-testid="camera-check" style={{ marginBottom: 12 }}>
      {problems.length > 0 && (
        <div style={boxStyle(hasError ? colors.error : colors.warning)}>
          <div style={{ fontWeight: 700, marginBottom: 6 }}>{t('cameraCheck.panelTitle')}</div>
          {problems.map((p, i) => (
            <div key={i} style={{ marginBottom: 8 }}>
              <div>{p.cause}</div>
              <div style={{ color: colors.textDim, marginTop: 2 }}>{t('cameraCheck.whatToDo')}:</div>
              <ul style={{ margin: '2px 0 0 16px', padding: 0 }}>
                {p.fixes.map((f) => <li key={f}>{f}</li>)}
              </ul>
            </div>
          ))}
        </div>
      )}
      {notes.map((n) => (
        <div key={n} style={{ color: colors.textDim, fontSize: 11, marginTop: 4 }}>{n}</div>
      ))}
    </div>
  );
}

function boxStyle(color) {
  return {
    border: `2px solid ${color}`,
    borderRadius: 6,
    padding: '8px 10px',
    color: colors.textBright,
    background: `${color}22`,
    fontSize: 12,
    lineHeight: 1.4,
  };
}
