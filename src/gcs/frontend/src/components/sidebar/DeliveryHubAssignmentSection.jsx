import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors, zoneColorsLabel } from '../../styles';
import { autoAssignDeliveryHubs, zoneToDeliveryHubDistance } from '../../utils/deliveryHubAssignment';
import { DELIVERY_HUB_TYPES } from '../map/constants/deliveryHubIcons.js';
import { Label } from './SidebarPrimitives';

/** Compute S#U# label for a zone based on its set_index within the zones array. */
export function zoneLabel(zones, zi) {
  const si = zones[zi]?.set_index ?? 0;
  let uavNum = 0;
  for (let i = 0; i <= zi; i++) {
    if ((zones[i]?.set_index ?? 0) === si) uavNum++;
  }
  return `S${si + 1}U${uavNum}`;
}

export default function DeliveryHubAssignmentSection({ plan, settings, deliveryHubAssignments, setDeliveryHubAssignments, setManualDeliveryHubEdit }) {
  const { t } = useTranslation();
  const deliveryHubs = settings?.default_delivery_hubs || [];
  const zones = plan?.zones || [];
  if (deliveryHubs.length === 0) return (
    <>
      <Label>{t('deliveryHub.missionDeliveryHubs')}</Label>
      <div style={{
        color: '#ff6b6b',
        fontSize: 12,
        marginBottom: 8,
        padding: '6px 8px',
        background: 'rgba(255, 107, 107, 0.1)',
        border: '1px solid rgba(255, 107, 107, 0.3)',
        borderRadius: 4,
      }}>
        {t('deliveryHub.noHubsDefined')}
      </div>
    </>
  );

  const handleAutoAssign = () => {
    const result = autoAssignDeliveryHubs(zones, deliveryHubs);
    setDeliveryHubAssignments(result);
    if (setManualDeliveryHubEdit) setManualDeliveryHubEdit(false);
  };

  const handleChange = (zoneIdx, value) => {
    const next = [...(deliveryHubAssignments || [])];
    while (next.length <= zoneIdx) next.push(null);
    next[zoneIdx] = value === '' ? null : parseInt(value, 10);
    setDeliveryHubAssignments(next);
    if (setManualDeliveryHubEdit) setManualDeliveryHubEdit(true);
  };

  return (
    <>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 6 }}>
        <Label style={{ marginBottom: 0 }}>{t('deliveryHub.missionDeliveryHubs')}</Label>
        {zones.length > 0 && (
          <button
            onClick={handleAutoAssign}
            style={{
              background: 'transparent',
              color: colors.accent,
              border: `1px solid ${colors.accent}`,
              borderRadius: 4,
              padding: '2px 8px',
              fontSize: 11,
              cursor: 'pointer',
            }}
          >
            {t('deliveryHub.autoAssign')}
          </button>
        )}
      </div>
      {zones.length === 0 ? (
        <div style={{ color: colors.textDim, fontSize: 12, marginBottom: 8 }}>
          {t('deliveryHub.generatePlanFirst')}
        </div>
      ) : (
        <div style={{ display: 'flex', flexDirection: 'column', gap: 4, marginBottom: 8 }}>
          {zones.map((zone, zi) => {
            const assignedDeliveryHub = deliveryHubAssignments?.[zi];
            const zoneColor = zoneColorsLabel[zi % zoneColorsLabel.length];
            const label = zoneLabel(zones, zi);
            const dist = assignedDeliveryHub != null && deliveryHubs[assignedDeliveryHub]
              ? zoneToDeliveryHubDistance(zone, deliveryHubs[assignedDeliveryHub])
              : null;
            return (
              <div key={zi} style={{ display: 'flex', alignItems: 'center', gap: 6, fontSize: 12 }}>
                <span style={{
                  width: 10, height: 10, borderRadius: '50%',
                  background: zoneColor, flexShrink: 0,
                }} />
                <span style={{ color: colors.textDim, minWidth: 30 }}>{label}</span>
                {zone.altitude_m != null && (
                  <span style={{ color: colors.textDim, fontSize: 11, minWidth: 38, textAlign: 'right' }}>
                    {Math.round(zone.altitude_m)}m
                  </span>
                )}
                <select
                  value={assignedDeliveryHub ?? ''}
                  onChange={(e) => handleChange(zi, e.target.value)}
                  style={{
                    flex: 1,
                    minWidth: 0, // allow shrinking below the widest option text — keeps the row inside the sidebar
                    background: colors.surface,
                    color: colors.textBright,
                    border: `1px solid ${colors.border}`,
                    borderRadius: 4,
                    padding: '3px 20px 3px 6px', // right padding reserves the native arrow area
                    fontSize: 12,
                    cursor: 'pointer',
                    textOverflow: 'ellipsis',
                    whiteSpace: 'nowrap',
                    overflow: 'hidden',
                  }}
                >
                  <option value="">{t('deliveryHub.none')}</option>
                  {deliveryHubs.map((location, oi) => (
                    <option key={oi} value={oi}>
                      {location.name || `Default delivery hub ${oi + 1}`} ({t('deliveryHub.types.' + location.type) || location.type})
                    </option>
                  ))}
                </select>
                {dist != null && (
                  <span style={{ color: colors.textDim, fontSize: 11, minWidth: 50, textAlign: 'right' }}>
                    {dist < 1000 ? `${Math.round(dist)}m` : `${(dist / 1000).toFixed(1)}km`}
                  </span>
                )}
              </div>
            );
          })}
        </div>
      )}
    </>
  );
}
