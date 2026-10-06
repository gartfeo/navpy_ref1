import React, { useEffect, useMemo, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { computeVisibleRange, formatValueForDisplay, recordsAllSame } from './paramTree';
import { paramHeaderStatus, coerceValueForRecord, describeParamType } from '../../utils/fullParams';
import { computeGridLayout } from '../../utils/paramGridLayout';
import { harmonizeCellStates, effectiveWinner, effectiveBaseSysId } from '../../utils/paramHarmonize';
import { CompareNameTags, CompareValueCell, FileValueCell } from './ParamCompareCells.jsx';
import { HarmonizeValueCell } from './ParamHarmonizeCells.jsx';
import CircularProgress from './CircularProgress.jsx';
import { colors } from '../../styles';

// Reserved width for the per-UAV status indicator so the column header doesn't
// re-truncate the vehicle name when the setup spinner swaps to ring + percent
// (16px ring + 4px gap + up to "100%"). Covers both active layouts.
const STATUS_MIN_WIDTH = 50;
/**
 * Multi-vehicle parameter grid. Editable in normal mode; in compare mode it
 * adds a read-only File column + per-row classification tags and shows current
 * values with a per-cell differ highlight (no editing).
 *
 * Custom windowed renderer: only rows in (visibleStart .. visibleEnd) are
 * rendered. Sticky header row + sticky first column (param name) so
 * scrolling either axis keeps context visible. Column widths come from
 * computeGridLayout so the sticky header and the body never diverge.
 */
export default function VirtualParamGrid({
  rowNames,
  vehicles,
  snapshotsByVehicle,
  staleByVehicle,
  draftsByVehicle,
  setVehicleDraft,
  cellErrorsByVehicle,
  progressByVehicle,
  writeProgressByVehicle,
  changedByVehicle,
  onRetryVehicle,
  editable = true,
  mode = 'normal',
  compareRowsByName = null,
  selectable = false,
  selectedNames = null,
  onToggleRow = null,
  onToggleAll = null,
  masterChecked = false,
  masterIndeterminate = false,
  compareCellErrorsByVehicle = null,
  // Harmonize mode
  harmonizeRowsByName = null,
  winnersByName = null,
  onPickWinner = null,
}) {
  const { t } = useTranslation();
  const scrollRef = useRef(null);
  const [scrollTop, setScrollTop] = useState(0);
  const [viewportHeight, setViewportHeight] = useState(400);
  const masterRef = useRef(null);

  const compareMode = mode === 'compare';
  const harmonizeMode = mode === 'harmonize';

  const layout = useMemo(
    () => computeGridLayout({ vehicleCount: vehicles.length, mode, selectable }),
    [vehicles.length, mode, selectable],
  );
  const {
    rowHeight: ROW_HEIGHT,
    headerHeight: HEADER_HEIGHT,
    nameWidth: NAME_COL_WIDTH,
    valWidth: VAL_COL_WIDTH,
    fileWidth: FILE_COL_WIDTH,
    checkboxWidth: CHECKBOX_COL_WIDTH,
    totalWidth,
  } = layout;

  // The master checkbox's indeterminate (partial) state is a DOM property,
  // not a React attribute, so it has to be set imperatively.
  useEffect(() => {
    if (masterRef.current) masterRef.current.indeterminate = masterIndeterminate;
  }, [masterIndeterminate]);

  useEffect(() => {
    const el = scrollRef.current;
    if (!el) return;
    const measure = () => setViewportHeight(el.clientHeight);
    measure();
    const ro = new ResizeObserver(measure);
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  const onScroll = (e) => setScrollTop(e.currentPoi.scrollTop);

  const range = useMemo(
    () => computeVisibleRange(scrollTop, viewportHeight, ROW_HEIGHT, rowNames.length, 6),
    [scrollTop, viewportHeight, rowNames.length],
  );

  const totalHeight = rowNames.length * ROW_HEIGHT;

  return (
    <div
      ref={scrollRef}
      onScroll={onScroll}
      style={{
        flex: 1,
        overflow: 'auto',
        position: 'relative',
        background: colors.bgLight,
        border: `1px solid ${colors.border}`,
        minHeight: 200,
      }}
    >
      {/* Header row — sticky top */}
      <div
        style={{
          position: 'sticky',
          top: 0,
          zIndex: 3,
          display: 'flex',
          height: HEADER_HEIGHT,
          width: totalWidth,
          background: colors.bgLighter,
          borderBottom: `1px solid ${colors.border}`,
          color: colors.text,
        }}
      >
        {selectable && (
          <div
            style={{
              width: CHECKBOX_COL_WIDTH,
              flex: '0 0 auto',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              position: 'sticky',
              left: 0,
              background: colors.bgLighter,
              zIndex: 5,
              borderRight: `1px solid ${colors.border}`,
            }}
            title={t('settings.parameters.select_all', 'Select all / clear')}
          >
            <input
              ref={masterRef}
              type="checkbox"
              checked={masterChecked}
              onChange={() => onToggleAll && onToggleAll()}
              style={{ accentColor: colors.accent }}
            />
          </div>
        )}
        <div
          style={{
            width: NAME_COL_WIDTH,
            flex: '0 0 auto',
            boxSizing: 'border-box',
            padding: '0 8px',
            display: 'flex',
            alignItems: 'center',
            fontWeight: 600,
            position: 'sticky',
            left: CHECKBOX_COL_WIDTH,
            background: colors.bgLighter,
            zIndex: 4,
            borderRight: `1px solid ${colors.border}`,
          }}
        >
          {t('settings.parameters.name', 'Name')}
        </div>
        {compareMode && (
          <div
            style={{
              width: FILE_COL_WIDTH,
              flex: '0 0 auto',
              padding: '0 6px',
              display: 'flex',
              alignItems: 'center',
              fontWeight: 600,
              color: colors.accent,
              borderRight: `1px solid ${colors.border}`,
            }}
          >
            {t('settings.parameters.file', 'File')}
          </div>
        )}
        {vehicles.map((v) => (
          <div
            key={v.sys_id}
            style={{
              width: VAL_COL_WIDTH,
              flex: '0 0 auto',
              boxSizing: 'border-box',
              padding: '0 8px',
              display: 'flex',
              alignItems: 'center',
              gap: 6,
              fontWeight: 600,
            }}
            title={v.name}
          >
            <span style={{ flex: 1, minWidth: 0, overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
              {v.name || `s1-u${v.sys_id}`}
            </span>
            {staleByVehicle?.[v.sys_id] && (
              <span
                title={t('settings.parameters.stale', 'Snapshot stale — refresh')}
                style={{
                  display: 'inline-block', flex: '0 0 auto',
                  width: 8, height: 8, borderRadius: '50%',
                  background: colors.warning,
                }}
              />
            )}
            <VehicleHeaderStatus
              progress={progressByVehicle?.[v.sys_id]}
              writeProgress={writeProgressByVehicle?.[v.sys_id]}
              loaded={!!snapshotsByVehicle?.[v.sys_id]}
              changed={!!changedByVehicle?.[v.sys_id]}
              sysId={v.sys_id}
              name={v.name || `s1-u${v.sys_id}`}
              onRetry={onRetryVehicle}
              t={t}
            />
          </div>
        ))}
      </div>

      {rowNames.length === 0 && (() => {
        const anyActive = (vehicles || []).some((v) => {
          const p = progressByVehicle?.[v.sys_id];
          return p && !p.done && !p.error;
        });
        const anyLoaded = (vehicles || []).some((v) => snapshotsByVehicle?.[v.sys_id]);
        // "No match" only when nothing is loading and something is loaded (i.e.
        // a filter excluded everything). Otherwise we're mid-download.
        const msg = (!anyActive && anyLoaded)
          ? t('settings.parameters.no_match', 'No parameters match the current filter.')
          : t('settings.parameters.loading_params', 'Loading parameters');
        return (
          <div style={{ padding: 24, color: colors.textDim, textAlign: 'center' }}>{msg}</div>
        );
      })()}

      {/* Spacer for total height so the scrollbar is correct.
          width = totalWidth so horizontal scrolling matches the header. */}
      <div style={{ height: totalHeight, width: totalWidth, position: 'relative' }}>
        {rowNames.slice(range.start, range.end).map((name, i) => {
          const rowIdx = range.start + i;
          // Rows live BELOW the sticky header inside the same scroll
          // container, so add HEADER_HEIGHT to the top offset (per Codex
          // post-step finding 1 for Step 5).
          const top = rowIdx * ROW_HEIGHT;
          const valuesByVehicle = vehicles.map((v) => {
            const snap = snapshotsByVehicle?.[v.sys_id];
            return snap?.paramsByName?.[name];
          });
          // Tolerant equality + partial-missing → mixed (matches Step 4
          // consensus semantics; per Codex post-step finding 3 for Step 5).
          const allSame = recordsAllSame(valuesByVehicle);
          const compareRow = compareMode
            ? (compareRowsByName ? compareRowsByName[name] : null)
            : null;
          const harmonizeRow = harmonizeMode
            ? (harmonizeRowsByName ? harmonizeRowsByName[name] : null)
            : null;
          const harmonizeWinner = harmonizeRow
            ? effectiveWinner(harmonizeRow, winnersByName)
            : undefined;
          const harmonizeStates = harmonizeRow
            ? harmonizeCellStates(harmonizeRow, harmonizeWinner) : null;
          const harmonizeBaseSid = harmonizeRow ? effectiveBaseSysId(harmonizeRow, winnersByName) : null;
          // Sync (winner/outlier) colors only render for a row the operator has
          // actually selected — an unselected row shows plain values, since
          // nothing will be written for it. Selection state lives in the shared
          // selectedNames set (ParametersTab aliases it to harmonizeSelected).
          const harmonizeRowSelected = harmonizeMode
            && !!(selectedNames && selectedNames.has(name));
          const tagRow = compareRow || harmonizeRow;

          return (
            <div
              key={name}
              style={{
                position: 'absolute',
                top,
                left: 0,
                right: 0,
                height: ROW_HEIGHT,
                display: 'flex',
                borderBottom: `1px solid ${colors.border}`,
                background: rowIdx % 2 === 0 ? colors.bgLight : colors.bgLighter,
              }}
            >
              {selectable && (
                <div
                  style={{
                    width: CHECKBOX_COL_WIDTH,
                    flex: '0 0 auto',
                    position: 'sticky',
                    left: 0,
                    background: 'inherit',
                    zIndex: 2,
                    borderRight: `1px solid ${colors.border}`,
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                  }}
                >
                  <input
                    type="checkbox"
                    checked={!!(selectedNames && selectedNames.has(name))}
                    disabled={harmonizeMode
                      ? (!harmonizeRow || harmonizeRow.readOnly
                        || harmonizeWinner === null || harmonizeWinner === undefined)
                      : (!compareRow || !compareRow.anyDiffers)}
                    onChange={() => onToggleRow && onToggleRow(name)}
                    style={{ accentColor: colors.accent }}
                  />
                </div>
              )}
              <div
                title={name}
                style={{
                  width: NAME_COL_WIDTH,
                  flex: '0 0 auto',
                  boxSizing: 'border-box',
                  padding: '0 8px',
                  position: 'sticky',
                  left: CHECKBOX_COL_WIDTH,
                  background: 'inherit',
                  zIndex: 1,
                  borderRight: `1px solid ${colors.border}`,
                  display: 'flex',
                  alignItems: 'center',
                  overflow: 'hidden',
                  textOverflow: 'ellipsis',
                  whiteSpace: 'nowrap',
                  fontFamily: 'monospace',
                  fontSize: 12,
                  color: !allSame ? colors.warning : colors.text,
                  gap: 6,
                }}
              >
                {/* flex: 1 1 auto + minWidth: 0 so the name (not the fixed-width
                    tag icons) is what truncates when space is tight — mirrors the
                    header's vehicle-name span. */}
                <span style={{
                  flex: '1 1 auto',
                  minWidth: 0,
                  overflow: 'hidden',
                  textOverflow: 'ellipsis',
                  whiteSpace: 'nowrap',
                }}>
                  {name}
                </span>
                {(compareMode || harmonizeMode) && (
                  <CompareNameTags row={tagRow} harmonizeMode={harmonizeMode} />
                )}
              </div>
              {compareMode && <FileValueCell row={compareRow} width={FILE_COL_WIDTH} />}
              {valuesByVehicle.map((rec, j) => {
                const v = vehicles[j];
                if (harmonizeMode) {
                  return (
                    <HarmonizeValueCell
                      key={v.sys_id}
                      rec={rec}
                      cell={harmonizeRow ? harmonizeRow.perVehicle?.[v.sys_id] : null}
                      state={harmonizeRowSelected && harmonizeStates ? harmonizeStates[v.sys_id] : null}
                      error={compareCellErrorsByVehicle ? compareCellErrorsByVehicle[v.sys_id]?.[name] : null}
                      width={VAL_COL_WIDTH}
                      isBase={harmonizeRowSelected && harmonizeBaseSid === v.sys_id}
                      onPick={onPickWinner && !harmonizeRow?.readOnly ? (value) => {
                        // Clicking the cell that's already this row's shown winner
                        // (only meaningful once the row is selected, since that's
                        // the only time it's shown as green) toggles the row OFF
                        // instead of re-picking the same value — "click again to
                        // unselect". Any other click picks the value and, if the
                        // row wasn't selected yet, selects it so the sync colors
                        // appear.
                        const isCurrentWinnerCell = harmonizeBaseSid === v.sys_id;
                        if (isCurrentWinnerCell && harmonizeRowSelected) {
                          if (onToggleRow) onToggleRow(name);
                          return;
                        }
                        onPickWinner(name, value, v.sys_id);
                        if (!harmonizeRowSelected && onToggleRow) onToggleRow(name);
                      } : null}
                    />
                  );
                }
                if (compareMode) {
                  return (
                    <CompareValueCell
                      key={v.sys_id}
                      rec={rec}
                      cell={compareRow ? compareRow.perVehicle?.[v.sys_id] : null}
                      error={compareCellErrorsByVehicle ? compareCellErrorsByVehicle[v.sys_id]?.[name] : null}
                      width={VAL_COL_WIDTH}
                    />
                  );
                }
                const draftValue = draftsByVehicle?.[v.sys_id]?.[name];
                const hasDraft = draftValue !== undefined;
                const cellErr = cellErrorsByVehicle?.[v.sys_id]?.[name];
                return (
                  <EditableValueCell
                    key={v.sys_id}
                    name={name}
                    rec={rec}
                    sysId={v.sys_id}
                    draftValue={draftValue}
                    hasDraft={hasDraft}
                    cellError={cellErr}
                    allSame={allSame}
                    editable={editable}
                    onChange={setVehicleDraft}
                    width={VAL_COL_WIDTH}
                  />
                );
              })}
            </div>
          );
        })}
      </div>
    </div>
  );
}


/**
 * Inline-editable cell. Click to edit, Enter or blur to commit. Shows:
 *   - draft value (italic) when modified vs snapshot
 *   - red border + tooltip on per-cell write error
 *   - discard-edit (↺) button while a pending edit exists, reverting the
 *     cell to the vehicle's current value
 */
function EditableValueCell({
  name, rec, sysId, draftValue, hasDraft, cellError, allSame, editable, onChange, width,
}) {
  const [editing, setEditing] = useState(false);
  const [text, setText] = useState('');
  const inputRef = useRef(null);
  // Suppress the blur-fired commit when the user cancels via Escape
  // (per Codex post-step finding 4 for Step 6).
  const suppressBlurRef = useRef(false);
  const present = rec != null;
  const readOnly = present && !!rec.read_only;
  const baseDisplay = present ? formatValueForDisplay(rec) : '—';
  const displayValue = hasDraft ? String(draftValue) : baseDisplay;

  const startEdit = () => {
    if (!editable || !present || readOnly) return;
    setText(displayValue);
    suppressBlurRef.current = false;
    setEditing(true);
    setTimeout(() => inputRef.current?.select?.(), 0);
  };

  const commit = () => {
    if (suppressBlurRef.current) {
      suppressBlurRef.current = false;
      setEditing(false);
      return;
    }
    setEditing(false);
    if (!editable || !present) return;
    // Empty input is treated as cancel/no-op (a numeric param can't hold
    // "nothing"); the cell reverts to its prior display.
    if (onChange && text !== '') {
      // Stage the value the autopilot would actually store (honest-permissive):
      // int params truncate + clamp, float passes through. Non-empty non-numeric
      // text is kept as typed so the operator sees their invalid entry, never a
      // silent revert.
      const coerced = coerceValueForRecord(rec, text);
      onChange(sysId, name, coerced !== null ? coerced : text);
    }
  };

  const cancel = () => {
    suppressBlurRef.current = true;
    setEditing(false);
  };

  const onDiscardDraft = (e) => {
    e.stopPropagation();
    if (!present || onChange == null) return;
    // Re-stage the vehicle's current value; setVehicleDraft prunes it as a
    // no-op, which discards the pending edit and clears "Write Changed".
    onChange(sysId, name, rec.value);
  };

  const tooltip = (() => {
    const parts = [];
    if (present) {
      if (readOnly) parts.push('read-only (firmware-maintained)');
      const ti = describeParamType(rec.ap_type);
      const typeStr = ti.isInt ? `${ti.label} (${ti.min}..${ti.max})` : ti.label;
      parts.push(`value: ${rec.value}`);
      parts.push(`type: ${typeStr}`);
      parts.push(`default: ${rec.default_known ? rec.default : '—'}`);
    } else {
      parts.push('not present on this vehicle');
    }
    if (hasDraft) parts.push(`draft: ${draftValue}`);
    if (cellError) parts.push(`error: ${cellError}`);
    return parts.join('\n');
  })();

  const borderColor = cellError
    ? colors.error
    : hasDraft ? colors.accent : 'transparent';
  const color = !present
    ? colors.textDim
    : readOnly ? colors.textDim
      : hasDraft ? colors.textBright
        : !allSame ? colors.warning
          : colors.text;
  const fontStyle = hasDraft ? 'italic' : 'normal';
  // The ↺ button discards a pending edit (reverts to the vehicle's current
  // value); it only appears when there is a draft to discard. Read-only cells
  // can't be edited, so they never have a draft and never show it.
  const showDiscardBtn = present && hasDraft;

  return (
    <div
      title={tooltip}
      onClick={editing ? undefined : startEdit}
      style={{
        width,
        flex: '0 0 auto',
        boxSizing: 'border-box',
        padding: '0 4px',
        display: 'flex',
        alignItems: 'center',
        gap: 4,
        overflow: 'hidden',
        whiteSpace: 'nowrap',
        fontFamily: 'monospace',
        fontSize: 12,
        color,
        fontStyle,
        border: `1px solid ${borderColor}`,
        cursor: editable && present && !readOnly && !editing ? 'text' : 'default',
        background: cellError ? 'rgba(244, 67, 54, 0.14)' : 'transparent',
      }}
    >
      {editing ? (
        <input
          ref={inputRef}
          value={text}
          onChange={(e) => setText(e.target.value)}
          onBlur={commit}
          onKeyDown={(e) => {
            if (e.key === 'Enter') commit();
            else if (e.key === 'Escape') cancel();
          }}
          style={{
            flex: 1,
            background: colors.bg,
            color: colors.textBright,
            border: `1px solid ${colors.accent}`,
            outline: 'none',
            fontFamily: 'monospace',
            fontSize: 12,
            padding: '0 4px',
            width: '100%',
          }}
        />
      ) : (
        <span style={{
          flex: 1,
          minWidth: 0,
          display: 'flex',
          alignItems: 'center',
          gap: 4,
        }}>
          {readOnly && (
            <svg width="10" height="10" viewBox="0 0 24 24" aria-hidden="true"
              fill="none" stroke={colors.textDim} strokeWidth="2.2"
              strokeLinecap="round" strokeLinejoin="round" style={{ flex: '0 0 auto' }}>
              <rect x="4" y="11" width="16" height="9" rx="2" />
              <path d="M8 11V7a4 4 0 0 1 8 0v4" />
            </svg>
          )}
          <span style={{ overflow: 'hidden', textOverflow: 'ellipsis' }}>{displayValue}</span>
        </span>
      )}
      {showDiscardBtn && !editing && (
        <button
          onClick={onDiscardDraft}
          title={`discard edit — revert to ${rec.value}`}
          style={{
            background: 'none',
            border: 'none',
            color: colors.textDim,
            cursor: 'pointer',
            padding: 0,
            fontSize: 11,
            lineHeight: 1,
          }}
        >
          ↺
        </button>
      )}
    </div>
  );
}


/**
 * Per-vehicle indicator shown in the grid column header:
 *   - downloading      -> circular ring + live percent
 *   - download failed  -> red ✕
 *   - loaded, clean    -> green check
 *   - loaded, edited   -> orange check (unsaved drafts)
 */
function VehicleHeaderStatus({ progress, writeProgress, loaded, changed, sysId, name, onRetry, t }) {
  // A write in progress takes precedence over download/loaded state — show a
  // green write ring + percent (mirrors the blue download ring).
  if (writeProgress && writeProgress.total > 0 && !writeProgress.done) {
    const wpct = Math.max(0, Math.min(100,
      Math.round((writeProgress.written / writeProgress.total) * 100)));
    const writing = t('settings.parameters.writing', 'Writing…');
    return (
      <span
        title={writing}
        aria-label={`${writing} ${wpct}%`}
        style={{ display: 'inline-flex', alignItems: 'center', gap: 4, flex: '0 0 auto', minWidth: STATUS_MIN_WIDTH }}
      >
        <CircularProgress percent={wpct} color={colors.success} size={16} />
        <span style={{ fontSize: 11, color: colors.success, fontWeight: 600, fontVariantNumeric: 'tabular-nums' }}>
          {wpct}%
        </span>
      </span>
    );
  }
  const kind = paramHeaderStatus(progress, loaded);
  if (kind === 'active') {
    // Setup phase — contacting the UAV / opening the param file: no bytes have
    // flowed and the file size is unknown, so the percent would be a frozen 0.
    // Show an indeterminate spinner so the header always reads as "working".
    if (!progress.hasBytes) {
      const contacting = t('settings.parameters.contacting', 'Contacting UAV…');
      return (
        <span
          title={contacting}
          aria-label={contacting}
          style={{ display: 'inline-flex', alignItems: 'center', flex: '0 0 auto', minWidth: STATUS_MIN_WIDTH }}
        >
          <CircularProgress indeterminate color={colors.accent} size={16} />
        </span>
      );
    }
    const pct = Math.max(0, Math.min(100, Math.round(progress.percent || 0)));
    const downloading = t('settings.parameters.downloading', 'Downloading parameters');
    return (
      <span
        title={downloading}
        aria-label={`${downloading} ${pct}%`}
        style={{ display: 'inline-flex', alignItems: 'center', gap: 4, flex: '0 0 auto', minWidth: STATUS_MIN_WIDTH }}
      >
        <CircularProgress percent={pct} color={colors.accent} size={16} />
        <span style={{ fontSize: 11, color: colors.accent, fontWeight: 600, fontVariantNumeric: 'tabular-nums' }}>
          {pct}%
        </span>
      </span>
    );
  }
  if (kind === 'error') {
    const failed = t('settings.parameters.download_failed', 'Download failed');
    const detail = progress?.error ? ` — ${String(progress.error)}` : '';
    return (
      <button
        type="button"
        onClick={(e) => { e.stopPropagation(); onRetry?.(sysId); }}
        title={`${failed}${detail}. ${t('settings.parameters.click_retry', 'Click to retry.')}`}
        aria-label={`${t('settings.parameters.retry', 'Retry')} ${name || sysId} — ${failed}`}
        style={{
          display: 'inline-flex', alignItems: 'center', gap: 3, flex: '0 0 auto',
          padding: '1px 6px', borderRadius: 3,
          border: `1px solid ${colors.error}`, background: 'transparent',
          color: colors.error, cursor: 'pointer', fontSize: 11, fontWeight: 600,
        }}
      >
        <svg width="11" height="11" viewBox="0 0 24 24" aria-hidden="true" fill="none" stroke={colors.error} strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round">
          <path d="M21 12a9 9 0 1 1-2.64-6.36" />
          <path d="M21 3v6h-6" />
        </svg>
        {t('settings.parameters.retry', 'Retry')}
      </button>
    );
  }
  if (kind === 'done') {
    const color = changed ? colors.warning : colors.success;
    return (
      <span
        title={changed
          ? t('settings.parameters.unsaved', 'Unsaved changes')
          : t('settings.parameters.loaded_ok', 'Loaded')}
        style={{ flex: '0 0 auto', display: 'inline-flex' }}
      >
        <svg width="14" height="14" viewBox="0 0 16 16" aria-hidden="true">
          <path d="M3.5 8.5 l3 3 l6 -7" fill="none" stroke={color} strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" />
        </svg>
      </span>
    );
  }
  return null;
}
