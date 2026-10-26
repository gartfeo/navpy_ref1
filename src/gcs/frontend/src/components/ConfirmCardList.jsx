import React, { useRef, useState } from 'react';
import { colors } from '../styles';
import TaskConfirmCard, { ConfirmStill } from './TaskConfirmCard';
import { isPending, canCancel, confirmRoundKey } from '../hooks/taskConfirmationState';
import { AAS_DEFAULTS } from '../utils/aasParams';

/**
 * The task-confirm rounds of one host: the right panel (planning and
 * monitor) or the RC overlay. Each approved UAV's Cancel task must be
 * reachable with no scroll in every state (todo 69), so the list has:
 * - a pinned part, never scrolled: one 32 px row per decided round, in
 *   sys_id order (todo 67's summary line goes first, while a round is
 *   pending);
 * - a scrolling part: the still a row opened (here, never under its row, so
 *   no row moves; one at a time), then the full pending cards, then
 *   `children` (the right panel's planning or monitor sidebar).
 *
 * `style` places the root (the host's size, position and frame).
 * `videoActionFor` gives a pending card its View camera, or is null where
 * the vehicle window cannot show (planning, RC).
 */
export default function ConfirmCardList({
  pendingConfirms,
  vehicleList,
  onApprove,
  onDeny,
  onCancel,
  forcedConfirms,
  aasParams,
  videoActionFor = null,
  style,
  children,
}) {
  // The open still: { sysId, roundKey }. It shows only while that round is
  // still approved, so it closes when its round is cancelled or replaced.
  const [still, setStill] = useState(null);
  const scrollRef = useRef(null);

  const entries = Object.entries(pendingConfirms || {}).map(([sysId, entry]) => [Number(sysId), entry]);
  const decided = entries.filter(([, entry]) => !isPending(entry));
  const pending = entries.filter(([, entry]) => isPending(entry));

  const vehicleOf = (sid) => {
    const vi = vehicleList.findIndex((v) => v.sys_id === sid);
    return { name: (vi >= 0 && vehicleList[vi].name) || `UAV ${sid}`, index: vi >= 0 ? vi : 0 };
  };
  const stillEntry = still && pendingConfirms?.[still.sysId];
  const stillShown = !!stillEntry && canCancel(stillEntry) && !!stillEntry.imageB64
    && confirmRoundKey(stillEntry) === still.roundKey;

  const toggleStill = (sid, entry) => {
    const key = confirmRoundKey(entry);
    if (stillShown && still.sysId === sid && still.roundKey === key) {
      setStill(null);
      return;
    }
    setStill({ sysId: sid, roundKey: key });
    // The still opens at the top of the scrolling part: bring it into view.
    if (scrollRef.current) scrollRef.current.scrollTop = 0;
  };

  const card = (sid, entry, extra) => {
    const vehicle = vehicleOf(sid);
    return (
      <TaskConfirmCard
        key={`${sid}-${entry.taskId}-${entry.roundUid ?? ''}`}
        sysId={sid}
        entry={entry}
        vehicleName={vehicle.name}
        vehicleIndex={vehicle.index}
        onApprove={onApprove}
        onDeny={onDeny}
        onCancel={onCancel}
        autoApprove={aasParams.getConfirmedValue(sid, 'nav_cm_fl', AAS_DEFAULTS.nav_cm_fl)}
        timeoutSec={aasParams.getConfirmedValue(sid, 'nav_cwt', AAS_DEFAULTS.nav_cwt)}
        forced={forcedConfirms?.[sid] === entry.taskId}
        {...extra}
      />
    );
  };

  return (
    <div style={{ display: 'flex', flexDirection: 'column', minHeight: 0, ...style }}>
      {decided.length > 0 && (
        <div
          data-testid="confirm-pinned"
          style={{
            flexShrink: 0,
            padding: '10px 16px 4px',
            background: colors.bgLight,
            borderBottom: `1px solid ${colors.border}`,
          }}
        >
          {decided.map(([sid, entry]) => card(sid, entry, {
            stillOpen: stillShown && still.sysId === sid,
            onToggleStill: canCancel(entry) && entry.imageB64 ? () => toggleStill(sid, entry) : null,
          }))}
        </div>
      )}
      {/* The panel must never scroll horizontally: rows shrink to fit. */}
      <div
        ref={scrollRef}
        data-testid="confirm-scroll"
        style={{ flex: '1 1 auto', minHeight: 0, overflowY: 'auto', overflowX: 'hidden' }}
      >
        {(stillShown || pending.length > 0) && (
          <div style={{ padding: '16px 16px 0' }}>
            {stillShown && (
              <ConfirmStill
                entry={stillEntry}
                vehicleName={vehicleOf(still.sysId).name}
                vehicleIndex={vehicleOf(still.sysId).index}
              />
            )}
            {pending.map(([sid, entry]) => card(sid, entry, {
              videoAction: videoActionFor?.(sid) ?? null,
            }))}
          </div>
        )}
        {children}
      </div>
    </div>
  );
}
