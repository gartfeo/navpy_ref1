import React, { useEffect, useMemo, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { colors, zoneColorsSolid } from '../../styles';
import { selectChangedNames } from '../../utils/fullParams';
import {
  FAILSAFE_GROUPS,
  actionLabel,
  buildFailsafeChecks,
  detectFirmware,
  enumNameForParam,
  optionsForParam,
  summarizeChecks,
  visibleParamsForGroup,
} from '../../utils/failsafeParams';

/**
 * Failsafe configuration & check tab. A curated, per-vehicle view over the
 * full parameter snapshot (useFullParams): it reads the cached failsafe
 * params, groups them (RC/throttle, battery, GCS link) with human-readable
 * action dropdowns, lets the operator edit and write each group, and shows a
 * quick pre-flight "is failsafe configured?" summary. It reuses the shared
 * full-param transport — edits go to the shared draft via setVehicleDraft and
 * writes go through writeChangedToVehicles scoped to a group's param names.
 */
export default function FailsafeTab({ vehicleList, fullParams, onOpenConnect, onClose }) {
  const { t } = useTranslation();
  const vehicles = vehicleList || [];
  const sysIds = useMemo(() => vehicles.map((v) => v.sys_id), [vehicles]);

  const [selectedSid, setSelectedSid] = useState(sysIds[0] ?? null);
  const [savingGroup, setSavingGroup] = useState(null);
  const [statusByGroup, setStatusByGroup] = useState({});

  const snapshotsByVehicle = fullParams?.snapshotsByVehicle || {};
  const draftsByVehicle = fullParams?.draftsByVehicle || {};

  // Keep the selection valid as vehicles connect/disconnect.
  useEffect(() => {
    if (selectedSid != null && sysIds.includes(selectedSid)) return;
    setSelectedSid(sysIds[0] ?? null);
  }, [sysIds, selectedSid]);

  // Auto-fetch the snapshot for any vehicle we haven't loaded yet. Ref guard
  // against refetch loops on failure, mirroring ParametersTab.
  const fetchedRef = useRef(new Set());
  useEffect(() => {
    if (!fullParams) return;
    const missing = sysIds.filter(
      (sid) => !snapshotsByVehicle[sid] && !fetchedRef.current.has(sid),
    );
    if (missing.length === 0) return;
    for (const sid of missing) fetchedRef.current.add(sid);
    fullParams.refreshVehicles(missing);
  }, [sysIds, snapshotsByVehicle, fullParams]);

  const sid = selectedSid;
  const snapshot = sid != null ? snapshotsByVehicle[sid] : null;
  const paramsByName = snapshot?.paramsByName || {};
  const draft = (sid != null && draftsByVehicle[sid]) || {};

  const presentNames = useMemo(() => Object.keys(paramsByName), [paramsByName]);
  const firmware = useMemo(() => detectFirmware(presentNames), [presentNames]);

  // Effective value: draft override falls back to the snapshot value.
  const effective = (name) => {
    if (Object.prototype.hasOwnProperty.call(draft, name)) return draft[name];
    return paramsByName[name]?.value;
  };

  // Names whose draft value actually differs from the snapshot (same coercion
  // semantics as the write path).
  const changedNames = useMemo(() => {
    const set = new Set();
    if (snapshot) {
      for (const c of selectChangedNames(draft, snapshot)) set.add(c.name);
    }
    return set;
  }, [draft, snapshot]);

  // Pre-flight checks against effective values across present failsafe params.
  const checks = useMemo(() => {
    if (!snapshot) return [];
    const valueByName = {};
    for (const g of FAILSAFE_GROUPS) {
      for (const d of g.params) {
        if (paramsByName[d.name]) valueByName[d.name] = effective(d.name);
      }
    }
    return buildFailsafeChecks(valueByName, firmware);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [snapshot, paramsByName, draft, firmware]);
  const summary = useMemo(() => summarizeChecks(checks), [checks]);

  const visibleGroups = useMemo(
    () => FAILSAFE_GROUPS
      .map((g) => ({ group: g, params: visibleParamsForGroup(g, presentNames, firmware) }))
      .filter((g) => g.params.length > 0),
    [presentNames, firmware],
  );

  const setVal = (name, value) => fullParams?.setVehicleDraft(sid, name, value);

  const saveGroup = async (group, params) => {
    if (sid == null || savingGroup) return;
    const names = params.map((d) => d.name);
    const changed = names.filter((n) => changedNames.has(n));
    if (changed.length === 0) return;
    const vehicle = vehicles.find((v) => v.sys_id === sid);
    if (vehicle?.armed) {
      const ok = window.confirm(
        t('settings.failsafe.armedConfirm', {
          defaultValue: 'Vehicle {{sysId}} is ARMED. Write failsafe parameters now?',
          sysId: sid,
        }),
      );
      if (!ok) return;
      await fullParams.requestArmedToken(sid);
    }
    setSavingGroup(group.id);
    try {
      const { aggregated } = await fullParams.writeChangedToVehicles([sid], {
        onlyNames: new Set(names),
      });
      // Only report a status when cells were actually attempted. If the diff
      // was emptied between render and click (e.g. a concurrent .param load
      // pruned the draft), submitting nothing is a no-op, not a failure — so
      // don't flash a misleading "Save failed (0)".
      const attempted = aggregated.cellOk + aggregated.cellFail;
      if (attempted > 0) {
        setStatusByGroup((prev) => ({
          ...prev,
          [group.id]: {
            ok: aggregated.cellFail === 0,
            cellOk: aggregated.cellOk,
            cellFail: aggregated.cellFail,
          },
        }));
      }
    } catch {
      setStatusByGroup((prev) => ({ ...prev, [group.id]: { ok: false, cellOk: 0, cellFail: changed.length } }));
    } finally {
      setSavingGroup(null);
    }
  };

  if (vehicles.length === 0) {
    return (
      <div style={{
        padding: '24px 16px', textAlign: 'center', color: colors.textDim, fontSize: 13,
        display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 12,
      }}>
        <span>{t('settings.failsafe.noVehicle', 'No connected vehicle. Connect to review failsafe.')}</span>
        {onOpenConnect && (
          <button
            onClick={() => { onClose?.(); onOpenConnect(); }}
            style={{
              padding: '4px 12px', fontSize: 12, fontWeight: 600,
              background: colors.accent, color: '#000', border: 'none',
              borderRadius: 4, cursor: 'pointer',
            }}
          >
            {t('connection.connect', 'Connect')}
          </button>
        )}
      </div>
    );
  }

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 12, color: colors.text }}>
      {/* Vehicle selector */}
      <div style={{
        display: 'flex', alignItems: 'center', gap: 8, flexWrap: 'wrap',
        paddingBottom: 8, borderBottom: `1px solid ${colors.border}`,
      }}>
        {vehicles.map((v, i) => {
          const active = v.sys_id === sid;
          const clr = zoneColorsSolid[i % zoneColorsSolid.length];
          return (
            <button
              key={v.sys_id}
              onClick={() => setSelectedSid(v.sys_id)}
              style={{
                padding: '3px 10px', fontSize: 12, fontWeight: 600, borderRadius: 4,
                cursor: 'pointer', border: `1px solid ${clr}`,
                background: active ? clr : 'transparent', color: active ? '#fff' : clr,
              }}
            >
              {v.name || `UAV ${v.sys_id}`}
              {v.armed ? ` · ${t('flightMode.armed', 'ARMED')}` : ''}
            </button>
          );
        })}
      </div>

      {!snapshot && (
        <div style={{ padding: '16px 4px', color: colors.textDim, fontSize: 13 }}>
          {t('settings.failsafe.loading', 'Reading failsafe parameters…')}
        </div>
      )}

      {snapshot && visibleGroups.length === 0 && (
        <div style={{ padding: '16px 4px', color: colors.textDim, fontSize: 13 }}>
          {t('settings.failsafe.summary.none', 'No failsafe parameters found on this vehicle.')}
        </div>
      )}

      {snapshot && visibleGroups.length > 0 && (
        <>
          <FailsafeSummary summary={summary} checks={checks} firmware={firmware} t={t} />
          {visibleGroups.map(({ group, params }) => (
            <FailsafeGroup
              key={group.id}
              group={group}
              params={params}
              firmware={firmware}
              paramsByName={paramsByName}
              effective={effective}
              changedNames={changedNames}
              setVal={setVal}
              onSave={() => saveGroup(group, params)}
              saving={savingGroup === group.id}
              status={statusByGroup[group.id]}
              t={t}
            />
          ))}
        </>
      )}
    </div>
  );
}

function FailsafeSummary({ summary, checks, firmware, t }) {
  const configured = summary.configured;
  const bannerColor = configured ? colors.success : colors.warning;
  const fwLabel = t(`settings.failsafe.firmware.${firmware}`, {
    defaultValue: firmware === 'plane' ? 'ArduPlane' : firmware === 'copter' ? 'ArduCopter' : 'Unknown firmware',
  });
  return (
    <div style={{
      border: `1px solid ${bannerColor}`, borderRadius: 6, padding: '8px 12px',
      background: 'transparent',
    }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: 8, marginBottom: 8 }}>
        <span style={{
          fontSize: 12, fontWeight: 700, color: colors.textBright,
          textTransform: 'uppercase', letterSpacing: 1,
        }}>
          {t('settings.failsafe.summary.title', 'Pre-flight failsafe check')}
        </span>
        <span style={{ display: 'inline-flex', alignItems: 'center', gap: 6, color: bannerColor, fontSize: 12, fontWeight: 700 }}>
          <span style={{ width: 8, height: 8, borderRadius: '50%', background: bannerColor }} />
          {configured
            ? t('settings.failsafe.summary.configured', 'Failsafe configured')
            : t('settings.failsafe.summary.needsAttention', 'Needs attention — {{count}} unset', { count: summary.warnCount })}
        </span>
      </div>
      <div style={{ display: 'flex', flexWrap: 'wrap', gap: 6 }}>
        {checks.map((c) => {
          const clr = c.ok ? colors.success : colors.warning;
          return (
            <span key={c.id} style={{
              display: 'inline-flex', alignItems: 'center', gap: 5,
              padding: '2px 8px', border: `1px solid ${clr}`, borderRadius: 3,
              color: clr, fontSize: 11, whiteSpace: 'nowrap',
            }}>
              <span style={{ width: 6, height: 6, borderRadius: '50%', background: clr }} />
              {t(c.labelKey, c.label)}
              {' · '}
              {c.ok
                ? t('settings.failsafe.summary.ok', 'OK')
                : t('settings.failsafe.summary.warn', 'not set')}
            </span>
          );
        })}
        <span style={{ marginLeft: 'auto', color: colors.textDim, fontSize: 11, alignSelf: 'center' }}>
          {fwLabel}
        </span>
      </div>
    </div>
  );
}

function FailsafeGroup({
  group, params, firmware, paramsByName, effective, changedNames, setVal, onSave, saving, status, t,
}) {
  const changedCount = params.filter((d) => changedNames.has(d.name)).length;
  const canSave = changedCount > 0 && !saving;
  return (
    <div style={{ border: `1px solid ${colors.border}`, borderRadius: 6, overflow: 'hidden' }}>
      <div style={{
        display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: 8,
        padding: '6px 12px', background: colors.bg, borderBottom: `1px solid ${colors.border}`,
      }}>
        <span style={{ fontSize: 12, fontWeight: 700, color: colors.accent, textTransform: 'uppercase', letterSpacing: 1 }}>
          {t(group.titleKey, group.title)}
        </span>
        <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
          {status && (
            <span style={{ fontSize: 11, color: status.ok ? colors.success : colors.error }}>
              {status.ok
                ? t('settings.failsafe.saved', 'Saved')
                : t('settings.failsafe.saveFailed', 'Save failed ({{fail}})', { fail: status.cellFail })}
            </span>
          )}
          <button
            onClick={onSave}
            disabled={!canSave}
            style={{
              padding: '3px 12px', fontSize: 12, fontWeight: 600, borderRadius: 4,
              background: canSave ? colors.success : colors.surfaceLight,
              color: canSave ? colors.textBright : colors.textDim,
              border: `1px solid ${canSave ? 'transparent' : colors.border}`,
              cursor: canSave ? 'pointer' : 'default',
            }}
          >
            {saving
              ? t('settings.failsafe.saving', 'Saving…')
              : changedCount > 0
                ? `${t('settings.failsafe.save', 'Save')} (${changedCount})`
                : t('settings.failsafe.noChanges', 'No changes')}
          </button>
        </div>
      </div>

      <div style={{
        display: 'grid', gridTemplateColumns: 'minmax(150px, 1.3fr) 1fr 1.2fr',
        gap: '6px 10px', alignItems: 'center', padding: '8px 12px', fontSize: 12,
      }}>
        <span style={{ color: colors.textDim, fontSize: 10, textTransform: 'uppercase', letterSpacing: 1 }}>
          {t('settings.failsafe.param', 'Parameter')}
        </span>
        <span style={{ color: colors.textDim, fontSize: 10, textTransform: 'uppercase', letterSpacing: 1 }}>
          {t('settings.failsafe.current', 'Current')}
        </span>
        <span style={{ color: colors.textDim, fontSize: 10, textTransform: 'uppercase', letterSpacing: 1 }}>
          {t('settings.failsafe.edited', 'New value')}
        </span>

        {params.map((def) => (
          <FailsafeRow
            key={def.name}
            def={def}
            firmware={firmware}
            record={paramsByName[def.name]}
            value={effective(def.name)}
            changed={changedNames.has(def.name)}
            setVal={setVal}
            t={t}
          />
        ))}
      </div>
    </div>
  );
}

const cellInput = {
  background: colors.bg,
  border: `1px solid ${colors.border}`,
  borderRadius: 4,
  padding: '3px 6px',
  color: colors.textBright,
  fontSize: 12,
  width: '100%',
  boxSizing: 'border-box',
};

function FailsafeRow({ def, firmware, record, value, changed, setVal, t }) {
  const options = def.type === 'select' ? optionsForParam(def, firmware) : [];
  const isSelectable = def.type === 'select' && options.length > 0;
  const numValue = Number(value);
  const current = record?.value;
  const currentText = formatCurrent(def, current, firmware, t);
  const changedStyle = changed
    ? { borderLeft: `3px solid ${colors.accent}`, paddingLeft: 4 }
    : {};

  return (
    <>
      <span style={{ display: 'flex', flexDirection: 'column', ...changedStyle }}>
        <span style={{ color: colors.text, fontWeight: 600 }}>{t(def.labelKey, def.label)}</span>
        <span style={{ color: colors.textDim, fontFamily: 'monospace', fontSize: 10 }}>{def.name}</span>
        {def.noteKey && (
          <span style={{ color: colors.textDim, fontSize: 10, marginTop: 2 }}>
            {t(def.noteKey, '')}
          </span>
        )}
      </span>

      <span style={{ color: colors.textDim }}>{currentText}</span>

      <span>
        {isSelectable ? (
          <select
            value={Number.isFinite(numValue) ? String(numValue) : ''}
            onChange={(e) => setVal(def.name, Number(e.target.value))}
            style={{ ...cellInput, cursor: 'pointer' }}
          >
            {/* Surface an out-of-enum current value so it isn't silently lost.
                Guard on isFinite so a stray non-numeric draft (e.g. staged by a
                .param import into the shared store) can't render a bogus
                "NaN — unknown" option. */}
            {Number.isFinite(numValue) && options.every((o) => o.value !== numValue) && (
              <option value={String(numValue)}>
                {`${numValue} — ${t('settings.failsafe.unknownValue', 'unknown')}`}
              </option>
            )}
            {options.map((o) => (
              <option key={o.value} value={String(o.value)}>
                {t(o.key, o.label)}
              </option>
            ))}
          </select>
        ) : (
          <input
            type="number"
            value={value ?? ''}
            step={def.step}
            min={def.min}
            max={def.max}
            onChange={(e) => {
              const next = parseFloat(e.target.value);
              if (Number.isFinite(next)) setVal(def.name, next);
            }}
            style={cellInput}
          />
        )}
      </span>
    </>
  );
}

function formatCurrent(def, current, firmware, t) {
  if (current === undefined || current === null) return '—';
  if (def.type === 'select') {
    const label = actionLabel(def, current, firmware);
    const enumName = enumNameForParam(def, firmware);
    if (label && enumName) {
      return `${Number(current)} — ${t(`settings.failsafe.actions.${enumName}.${Number(current)}`, label)}`;
    }
    return String(Number(current));
  }
  // Trim float noise on display only.
  const n = Number(current);
  return Number.isInteger(n) ? String(n) : String(Math.round(n * 1000) / 1000);
}
