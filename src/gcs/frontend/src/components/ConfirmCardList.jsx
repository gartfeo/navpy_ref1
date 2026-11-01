import React, { useEffect, useRef, useState } from 'react';
import { colors } from '../styles';
import TaskConfirmCard, { ConfirmStill } from './TaskConfirmCard';
import ConfirmSummary from './ConfirmSummary';
import {
  isPending, canCancel, confirmRoundKey, confirmRoundId, orderedConfirmEntries, pendingSummary,
  confirmZeroLabel,
} from '../hooks/taskConfirmationState';
import { confirmNow } from '../utils/confirmClock';
import { AAS_DEFAULTS } from '../utils/aasParams';

/** The list's shared clock sample period: the card labels and the chip read one `now`. */
const LIST_TICK_MS = 1000;
const NO_CUE = Object.freeze({ fired: {}, onAutoFire: null, soundState: 'running', unlockSound: null });

/**
 * The task-confirm rounds of one host: the right panel (planning and
 * monitor) or the RC overlay. Each approved UAV's Cancel task must be
 * reachable with no scroll in every state (todo 69), so the list has:
 * - a pinned part, never scrolled: the summary chip while a round is
 *   pending (todo 67), then one 32 px row per decided round, approved before
 *   denied or canceled, each by sys_id;
 * - a scrolling part: the still a row opened (here, never under its row, so
 *   no row moves; one at a time), then the full pending cards, soonest
 *   deadline first (orderedConfirmEntries), then `children` (the right
 *   panel's planning or monitor sidebar).
 *
 * `style` places the root (the host's size, position and frame).
 * `videoActionFor` gives a pending card its View camera, or is null where
 * the vehicle window cannot show (planning, RC).
 * `cue` (useTaskConfirmation's) carries the fired latch and the sound state.
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
  cue = NO_CUE,
  style,
  children,
}) {
  // The open still: { sysId, roundKey }. It shows only while that round is
  // still approved, so it closes when its round is cancelled or replaced.
  const [still, setStill] = useState(null);
  const scrollRef = useRef(null);
  // Pending card elements by confirmRoundId: the chip scrolls to the shown one.
  const cardRefs = useRef(new Map());

  // Raw cached values (undefined = not loaded) for the labels and the order;
  // the cards' auto-fire keeps today's AAS_DEFAULTS fallbacks below.
  const rawParam = (sid, key) => aasParams.getConfirmedValue(sid, key, undefined);
  const waitSecFor = (sid) => rawParam(sid, 'nav_cwt');
  const fired = cue.fired || {};

  const entries = orderedConfirmEntries(pendingConfirms, waitSecFor, fired);
  const decided = entries.filter(([, entry]) => !isPending(entry));
  const pending = entries.filter(([, entry]) => isPending(entry));

  // One clock reading per render for every label and the chip, so a card and
  // the chip never show different seconds. Read at render, not kept in state:
  // a stored sample is up to a tick old when a round arrives (a new card would
  // show more than its wait) and frozen while none is pending. The tick only
  // re-renders the list each second while a round is pending.
  const [, setTick] = useState(0);
  const anyPending = pending.length > 0;
  useEffect(() => {
    if (!anyPending) return undefined;
    const id = setInterval(() => setTick((n) => n + 1), LIST_TICK_MS);
    return () => clearInterval(id);
  }, [anyPending]);
  const now = confirmNow();
  const summary = anyPending ? pendingSummary(pendingConfirms, waitSecFor, now, fired) : null;

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

  const labelFor = (sid, entry, phase, seconds) => confirmZeroLabel({
    phase, cmFl: rawParam(sid, 'nav_cm_fl'), cwt: rawParam(sid, 'nav_cwt'), seconds,
  });

  // Bring the shown UAV's card to the top of the scrolling part (an open still
  // sits above the pending cards), and unlock the sound. Nothing else.
  const showSoonest = () => {
    cue.unlockSound?.();
    const container = scrollRef.current;
    const el = summary && cardRefs.current.get(confirmRoundId(summary.shownSysId, summary.shownEntry));
    if (!container || !el) return;
    container.scrollTop += el.getBoundingClientRect().top - container.getBoundingClientRect().top;
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
      {(decided.length > 0 || summary) && (
        <div
          data-testid="confirm-pinned"
          style={{
            flexShrink: 0,
            padding: '10px 16px 4px',
            background: colors.bgLight,
            borderBottom: `1px solid ${colors.border}`,
          }}
        >
          {summary && (
            <ConfirmSummary
              summary={summary}
              label={labelFor(summary.shownSysId, summary.shownEntry, summary.phase, summary.secondsLeft)}
              vehicle={vehicleOf(summary.shownSysId)}
              names={summary.sysIds.map((sid) => vehicleOf(sid).name)}
              soundState={cue.soundState}
              onActivate={showSoonest}
            />
          )}
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
            {pending.map(([sid, entry]) => {
              const id = confirmRoundId(sid, entry);
              return (
                <div
                  key={id}
                  ref={(el) => { if (el) cardRefs.current.set(id, el); else cardRefs.current.delete(id); }}
                >
                  {card(sid, entry, {
                    videoAction: videoActionFor?.(sid) ?? null,
                    labelPolicy: { cmFl: rawParam(sid, 'nav_cm_fl'), cwt: rawParam(sid, 'nav_cwt') },
                    now,
                    fired: fired[id] != null,
                    onAutoFire: cue.onAutoFire,
                  })}
                </div>
              );
            })}
          </div>
        )}
        {children}
      </div>
    </div>
  );
}
