import React, { useState, useEffect, useRef } from 'react';
import { useTranslation } from 'react-i18next';
import { colors, zoneColorsSolid } from '../../styles';
import { inputStyle } from './SettingsField.jsx';
import { PARAM_SECTIONS, SIM_PARAM_SECTIONS, CONFIRMATION_PARAM_SECTION } from './paramSections.js';
import WaypointEditorOverlay from './WaypointEditorOverlay.jsx';
import BitmaskInput from './BitmaskInput.jsx';

export default function AasTab({ vehicleList, onOpenConnect, onClose, vehicleMissions, setVehicleMissions, simMode, onVehicleTargWpsChange, onVehicleNavLastWpChange, preSaveRef, preResetRef, aasParams }) {
  const { t } = useTranslation();
  const vehicles = vehicleList || [];
  const [selectedVehicles, setSelectedVehicles] = useState(() => new Set(vehicles.map((v) => v.sys_id)));
  const [paramStatus, setParamStatus] = useState(null);
  const [showEditor, setShowEditor] = useState(false);
  const [dlMissionStatus, setDlMissionStatus] = useState(false);
  const autoDownloadedRef = useRef(false);
  const editorSnapshotRef = useRef(null);
  const [isUploading, setIsUploading] = useState(false);

  const {
    confirmedByVehicle, draftByVehicle,
    setVehicleDraft, refreshVehicles, uploadDraft, resetDraft,
  } = aasParams;

  const disabled = vehicles.length === 0;
  const selIds = vehicles.filter((v) => selectedVehicles.has(v.sys_id)).map((v) => v.sys_id);

  const cols = selIds.map((sid) => {
    const vi = vehicles.findIndex((v) => v.sys_id === sid);
    return {
      sid,
      clr: zoneColorsSolid[(vi >= 0 ? vi : 0) % zoneColorsSolid.length],
      name: vehicles.find((v) => v.sys_id === sid)?.name || `UAV ${sid}`,
    };
  });

  const isLoaded = (sid) => !!confirmedByVehicle[sid];
  const getVal = (sid, key) => {
    if (!isLoaded(sid)) return undefined;
    return draftByVehicle[sid]?.[key];
  };
  const setVal = (sid, key, val) => {
    setVehicleDraft(sid, key, val);
  };
  const isChanged = (sid, key) => {
    const current = draftByVehicle[sid]?.[key];
    if (current === undefined) return false;
    const original = confirmedByVehicle[sid]?.[key];
    if (original === undefined) return true;  // user set a value not in download
    // eslint-disable-next-line eqeqeq
    return current != original;
  };

  const tr = (key, fallback) => (key ? t(key, { defaultValue: fallback }) : fallback);
  const sectionTitle = (section) => tr(section.titleKey, section.title);
  const fieldLabel = (field) => tr(field.labelKey, field.label);
  const fieldPlaceholder = (field) => tr(field.placeholderKey, field.placeholder);
  const optionLabel = (option) => tr(option.labelKey, option.label);
  const renderParamStatus = () => {
    if (!paramStatus) return null;
    if (typeof paramStatus === 'string') return paramStatus;
    return t(paramStatus.key, {
      defaultValue: paramStatus.fallback,
      ...(paramStatus.options || {}),
    });
  };

  const toggleVehicle = (sysId) => {
    setSelectedVehicles((prev) => {
      const next = new Set(prev);
      if (next.has(sysId)) next.delete(sysId);
      else next.add(sysId);
      return next;
    });
  };

  // Download params via the hook AND missions in parallel. The mission
  // fetch lives here (not in the hook) because missions are not AAS state;
  // they belong to the WaypointEditorOverlay flow.
  const doDownload = async (ids) => {
    setParamStatus({ key: 'connection.downloading', fallback: 'Downloading...' });
    try {
      const [, missionResults] = await Promise.all([
        refreshVehicles(ids),
        Promise.all(ids.map((sid) => fetch(`/api/vehicles/${sid}/mission`).then((r) => r.ok ? r.json() : null).catch(() => null))),
      ]);
      const newMissions = { ...vehicleMissions };
      for (let i = 0; i < ids.length; i++) {
        const md = missionResults[i];
        if (md?.waypoints) newMissions[ids[i]] = md.waypoints;
      }
      setVehicleMissions(newMissions);
      setParamStatus({ key: 'vehicle.downloadedFrom', fallback: 'Downloaded from {{count}} UAV(s)', options: { count: ids.length } });
    } catch { setParamStatus({ key: 'vehicle.downloadError', fallback: 'Download error' }); }
  };

  useEffect(() => {
    if (autoDownloadedRef.current || vehicles.length === 0) return;
    const allIds = vehicles.map((v) => v.sys_id);
    setSelectedVehicles(new Set(allIds));
    autoDownloadedRef.current = true;
    doDownload(allIds);
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [vehicles.length]);

  const handleUpload = async () => {
    if (selIds.length === 0 || isUploading) return;
    setIsUploading(true);
    setParamStatus({ key: 'bottomBar.uploading', fallback: 'Uploading...' });
    try {
      const { submittedByVehicle, aggregated } = await uploadDraft(selIds);
      let uploadedCount = 0;
      let totalAll = 0;
      for (const sid of selIds) {
        const submittedKeys = Object.keys(submittedByVehicle[sid] || {});
        if (submittedKeys.length === 0) continue;
        uploadedCount += 1;
        totalAll += submittedKeys.length;
      }
      if (uploadedCount === 0) {
        setParamStatus({ key: 'vehicle.noChanges', fallback: 'No changes to upload' });
      } else if (aggregated.fieldFail > 0) {
        setParamStatus({
          key: 'vehicle.uploadedParamsPartial',
          fallback: 'Uploaded {{ok}}/{{total}} params to {{count}} UAV(s) -- {{failed}} not confirmed',
          options: {
            ok: aggregated.fieldOk,
            total: totalAll,
            count: uploadedCount,
            failed: aggregated.fieldFail,
          },
        });
      } else {
        setParamStatus({
          key: 'vehicle.uploadedParams',
          fallback: 'Uploaded {{ok}}/{{total}} params to {{count}} UAV(s)',
          options: { ok: aggregated.fieldOk, total: totalAll, count: uploadedCount },
        });
      }
    } catch { setParamStatus({ key: 'vehicle.uploadError', fallback: 'Upload error' }); }
    setIsUploading(false);
  };

  const modifiedCount = (() => {
    let count = 0;
    for (const sid of selIds) {
      const vp = draftByVehicle[sid];
      if (!vp) continue;
      for (const k of Object.keys(vp)) {
        if (isChanged(sid, k)) count++;
      }
    }
    return count;
  })();

  const handleReset = () => {
    resetDraft(selIds);
    setParamStatus(null);
  };

  // Wire Apply to upload, Reset to revert vehicle params
  if (preSaveRef) preSaveRef.current = handleUpload;
  if (preResetRef) preResetRef.current = handleReset;
  useEffect(() => () => {
    if (preSaveRef) preSaveRef.current = null;
    if (preResetRef) preResetRef.current = null;
  }, [preSaveRef, preResetRef]);

  const poisMap = {};
  const navLastWpMap = {};
  for (const sid of selIds) {
    poisMap[sid] = parseInt(getVal(sid, 'targ_wps')) || 0;
    navLastWpMap[sid] = parseInt(getVal(sid, 'nav_last_wp')) || 0;
  }

  // Bubble per-vehicle targ_wps and nav_last_wp up to App for map markers
  useEffect(() => {
    if (!onVehicleTargWpsChange) return;
    const map = {};
    for (const v of vehicles) {
      const val = parseInt(draftByVehicle[v.sys_id]?.targ_wps) || 0;
      if (val > 0) map[v.sys_id] = val;
    }
    onVehicleTargWpsChange(map);
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [vehicles.map((v) => v.sys_id).join(','), ...vehicles.map((v) => draftByVehicle[v.sys_id]?.targ_wps)]);

  useEffect(() => {
    if (!onVehicleNavLastWpChange) return;
    const map = {};
    for (const v of vehicles) {
      const val = parseInt(draftByVehicle[v.sys_id]?.nav_last_wp) || 0;
      if (val > 0) map[v.sys_id] = val;
    }
    onVehicleNavLastWpChange(map);
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [vehicles.map((v) => v.sys_id).join(','), ...vehicles.map((v) => draftByVehicle[v.sys_id]?.nav_last_wp)]);

  const handleToggleWp = (sysId, idx) => {
    if (idx > 23) return;
    const curMask = poisMap[sysId] || 0;
    setVal(sysId, 'targ_wps', curMask ^ (1 << idx));
  };

  const handlePerVehicleNavLastWpChange = (sysId, val) => {
    setVal(sysId, 'nav_last_wp', val);
  };

  const handleDownloadMissions = async () => {
    if (Object.keys(vehicleMissions).length > 0) {
      if (!window.confirm(t('vehicle.replaceWaypoints'))) return;
    }
    setDlMissionStatus(true);
    try {
      const results = await Promise.all(
        vehicles.map((v) => fetch(`/api/vehicles/${v.sys_id}/mission`).then((r) => r.ok ? r.json() : null).catch(() => null)),
      );
      const newMissions = {};
      results.forEach((data, i) => {
        if (data?.waypoints) newMissions[vehicles[i].sys_id] = data.waypoints;
      });
      setVehicleMissions(newMissions);
    } catch { /* ignore */ }
    setDlMissionStatus(false);
  };

  const openEditor = () => {
    const snapshot = {};
    for (const sid of selIds) {
      snapshot[sid] = {
        targ_wps: draftByVehicle[sid]?.targ_wps,
        nav_last_wp: draftByVehicle[sid]?.nav_last_wp,
      };
    }
    editorSnapshotRef.current = snapshot;
    setShowEditor(true);
  };

  const handleEditorDiscard = () => {
    if (!editorSnapshotRef.current) return;
    for (const [sid, snap] of Object.entries(editorSnapshotRef.current)) {
      setVehicleDraft(sid, 'targ_wps', snap.targ_wps);
      setVehicleDraft(sid, 'nav_last_wp', snap.nav_last_wp);
    }
  };

  const chipStyle = (active, idx) => {
    const zoneColor = zoneColorsSolid[idx % zoneColorsSolid.length];
    return {
      padding: '3px 10px', fontSize: 12, fontWeight: 600,
      borderRadius: 4, cursor: 'pointer',
      border: `1px solid ${zoneColor}`,
      background: active ? zoneColor : 'transparent',
      color: active ? '#fff' : zoneColor,
    };
  };

  const cellInput = {
    background: colors.bg,
    borderTop: `1px solid ${colors.border}`,
    borderRight: `1px solid ${colors.border}`,
    borderBottom: `1px solid ${colors.border}`,
    borderLeft: `1px solid ${colors.border}`,
    borderRadius: 4,
    paddingTop: 2, paddingBottom: 2, paddingLeft: 6, paddingRight: 6,
    color: colors.textBright,
    fontSize: 12,
    width: '100%',
    boxSizing: 'border-box',
  };

  const renderGrid = (sections) => {
    if (cols.length === 0) return null;
    return (
      <div style={{
        display: 'grid',
        gridTemplateColumns: `150px repeat(${cols.length}, 90px)`,
        gap: '4px 8px',
        alignItems: 'center',
        fontSize: 12,
      }}>
        {sections.map((section, si) => (
          <React.Fragment key={section.title}>
            {si === 0 ? (
              <>
                <div style={{
                  fontSize: 11, fontWeight: 600, color: colors.accent,
                  textTransform: 'uppercase', letterSpacing: 1,
                  marginTop: 8, marginBottom: 2,
                }}>
                  {sectionTitle(section)}
                </div>
                {cols.map((c) => (
                  <span key={c.sid} style={{ display: 'flex', alignItems: 'center', gap: 3, justifyContent: 'center', marginTop: 8, marginBottom: 2 }}>
                    <span style={{ width: 7, height: 7, borderRadius: '50%', background: c.clr, flexShrink: 0 }} />
                    <span style={{ color: c.clr, fontWeight: 600, fontSize: 11 }}>{c.name}</span>
                  </span>
                ))}
              </>
            ) : (
              <div style={{
                gridColumn: '1 / -1',
                fontSize: 11, fontWeight: 600, color: colors.accent,
                textTransform: 'uppercase', letterSpacing: 1,
                marginTop: 8, marginBottom: 2,
              }}>
                {sectionTitle(section)}
              </div>
            )}
            {section.fields.map((field) => {
              if (field.rowVisibleWhen) {
                const snaps = {};
                for (const c of cols) snaps[c.sid] = draftByVehicle[c.sid];
                if (!field.rowVisibleWhen(snaps)) return null;
              }
              return (
              <React.Fragment key={field.key}>
                <span style={{ color: colors.textDim, fontWeight: 600, fontSize: 11 }}>
                  {fieldLabel(field)}
                </span>
                {cols.map((c) => {
                  const loaded = isLoaded(c.sid);
                  const val = getVal(c.sid, field.key);
                  const ch = isChanged(c.sid, field.key);
                  const chStyle = ch ? { borderLeft: `3px solid ${colors.accent}`, paddingLeft: 4 } : {};
                  // A field can mark cells as inapplicable (e.g. nav_cwt
                  // is meaningful only when this vehicle is in manual
                  // mode). The cell still occupies its grid slot so
                  // columns stay aligned, but the input is greyed and
                  // pointer-disabled to signal "not editable here."
                  const inapplicable = field.cellDisabledWhen && field.cellDisabledWhen(draftByVehicle[c.sid]);
                  const cellDisabled = !loaded || inapplicable;
                  const disabledStyle = cellDisabled ? { opacity: 0.3, pointerEvents: 'none' } : {};
                  if (field.type === 'checkbox') {
                    return (
                      <div key={c.sid} style={{ display: 'flex', justifyContent: 'center', ...chStyle, ...disabledStyle }}>
                        <input type="checkbox" checked={!!val} disabled={cellDisabled} onChange={(e) => setVal(c.sid, field.key, e.target.checked)} style={{ accentColor: colors.accent, cursor: cellDisabled ? 'default' : 'pointer' }} />
                      </div>
                    );
                  }
                  if (field.type === 'select') {
                    return (
                      <select key={c.sid} disabled={cellDisabled} value={val === undefined ? '' : String(val)} onChange={(e) => setVal(c.sid, field.key, field.options.find((o) => String(o.value) === e.target.value)?.value ?? val)} style={{ ...cellInput, ...chStyle, ...disabledStyle, cursor: cellDisabled ? 'default' : 'pointer' }}>
                        {field.options.map((o) => <option key={String(o.value)} value={String(o.value)}>{optionLabel(o)}</option>)}
                      </select>
                    );
                  }
                  if (field.type === 'bitmask') {
                    return (
                      <BitmaskInput key={c.sid} value={val} onChange={(mask) => setVal(c.sid, field.key, mask)} placeholder={loaded ? '3,6' : ''} style={{ ...cellInput, ...chStyle, ...disabledStyle }} />
                    );
                  }
                  const placeholder = fieldPlaceholder(field);
                  return (
                    <input key={c.sid} type="number" disabled={cellDisabled} value={val ?? ''} step={field.step} min={field.min} placeholder={placeholder} title={placeholder} onChange={(e) => {
                      // Drop NaN (empty input or non-numeric typing) so the
                      // draft never holds a value that JSON-serializes to null
                      // and breaks the backend float(value) path.
                      const next = parseFloat(e.target.value);
                      if (Number.isFinite(next)) setVal(c.sid, field.key, next);
                    }} style={{ ...cellInput, ...chStyle, ...disabledStyle }} />
                  );
                })}
              </React.Fragment>
              );
            })}
          </React.Fragment>
        ))}
      </div>
    );
  };

  return (
    <>
      {disabled && (
        <div style={{
          padding: '12px 16px', marginBottom: 12,
          background: 'rgba(255,255,255,0.05)', borderRadius: 4,
          color: colors.textDim, fontSize: 13, textAlign: 'center',
          display: 'flex', alignItems: 'center', justifyContent: 'center', gap: 10,
        }}>
          <span>{t('vehicle.noUavsConnected')}</span>
          <button
            onClick={() => { onClose(); onOpenConnect(); }}
            style={{
              padding: '4px 12px', fontSize: 12, fontWeight: 600,
              background: colors.accent, color: '#000',
              border: 'none', borderRadius: 4, cursor: 'pointer',
            }}
          >
            {t('connection.connect')}
          </button>
        </div>
      )}

      <div style={{
        display: 'flex', alignItems: 'center', gap: 8, flexWrap: 'wrap',
        padding: '8px 0', marginBottom: 12,
        borderBottom: `1px solid ${colors.border}`,
        opacity: disabled ? 0.4 : 1,
        pointerEvents: disabled ? 'none' : 'auto',
      }}>
        {vehicles.map((v, i) => (
          <button key={v.sys_id} onClick={() => toggleVehicle(v.sys_id)} style={chipStyle(selectedVehicles.has(v.sys_id), i)}>
            {v.name || `UAV ${v.sys_id}`}
          </button>
        ))}
        {paramStatus && (
          <span style={{ fontSize: 11, color: colors.textDim }}>{renderParamStatus()}</span>
        )}
        {modifiedCount > 0 && (
          <span style={{ fontSize: 11, color: colors.accent, fontWeight: 600 }}>
            {t('vehicle.changesPending', { count: modifiedCount })}
          </span>
        )}
      </div>

      <div style={{ opacity: disabled ? 0.4 : 1, pointerEvents: disabled ? 'none' : 'auto' }}>
        {renderGrid([...PARAM_SECTIONS, CONFIRMATION_PARAM_SECTION])}
      </div>

      {simMode && (
        <div style={{
          marginTop: 16,
          padding: '12px 0',
          borderTop: `1px solid ${colors.border}`,
          opacity: disabled ? 0.4 : 1,
          pointerEvents: disabled ? 'none' : 'auto',
        }}>
          <div style={{
            fontSize: 13, fontWeight: 600, color: colors.warning,
            marginBottom: 8, textTransform: 'uppercase', letterSpacing: 1,
          }}>
            {t('vehicle.simulation')}
          </div>
          {renderGrid(SIM_PARAM_SECTIONS)}
          {cols.length > 0 && (
            <div style={{ marginTop: 8 }}>
              <button
                onClick={openEditor}
                disabled={!selIds.some((sid) => isLoaded(sid))}
                style={{
                  padding: '4px 12px', fontSize: 12, fontWeight: 600,
                  background: 'transparent', border: `1px solid ${colors.border}`,
                  borderRadius: 4,
                  color: selIds.some((sid) => isLoaded(sid)) ? colors.accent : colors.textDim,
                  cursor: selIds.some((sid) => isLoaded(sid)) ? 'pointer' : 'not-allowed',
                }}
              >
                {t('vehicle.editPois')}
              </button>
            </div>
          )}
          {showEditor && (
            <WaypointEditorOverlay
              vehicleMissions={vehicleMissions}
              vehicleList={vehicles}
              selectedVehicles={selectedVehicles}
              poisMap={poisMap}
              navLastWpMap={navLastWpMap}
              onPerVehicleNavLastWpChange={handlePerVehicleNavLastWpChange}
              vehicleParams={draftByVehicle}
              onToggle={handleToggleWp}
              onDownload={handleDownloadMissions}
              downloading={dlMissionStatus}
              onPerVehicleTargChange={(sid, mask) => setVal(sid, 'targ_wps', mask)}
              onDiscard={handleEditorDiscard}
              onApply={() => setShowEditor(false)}
              onClose={() => setShowEditor(false)}
            />
          )}
        </div>
      )}
    </>
  );
}
