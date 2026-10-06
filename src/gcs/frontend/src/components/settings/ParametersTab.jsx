import React, { useEffect, useMemo, useRef, useState } from 'react';
import { useTranslation } from 'react-i18next';
import {
  buildPrefixSidebar,
  canonicalNameOrder,
  differingParamNames,
  filterCanonicalNames,
} from './paramTree';
import {
  fullParamLoadProgress,
  selectChangedNames,
  selectNonDefaultNames,
} from '../../utils/fullParams';
import {
  parseParamFile,
  snapshotToParamFile,
  validateRowsForVehicle,
} from '../../utils/paramFile';
import {
  computeParamCompare,
  collectCompareChanges,
  defaultCompareSelection,
  reconcileCompareSelection,
} from '../../utils/paramCompare';
import {
  computeFleetHarmonize,
  collectHarmonizeChanges,
  defaultHarmonizeSelection,
  reconcileHarmonizeSelection,
  droppedHarmonizeWinnerNames,
} from '../../utils/paramHarmonize';
import {
  groupBucketForRow,
  summarizeCompareGroups,
  PARAM_GROUP_LABELS,
  LOW_PRIORITY_GROUPS,
} from '../../utils/paramGroups';
import VirtualParamGrid from './VirtualParamGrid.jsx';
import ParamCompareBar from './ParamCompareBar.jsx';
import ParamHarmonizeBar from './ParamHarmonizeBar.jsx';
import { colors } from '../../styles';

const ARMED_LONG_PRESS_MS = 1000;

/**
 * Mission Planner-style Parameters tab. Read-only base from Step 5;
 * Step 6 adds editing, write-changed batching, modified-only filter,
 * per-cell discard-edit, per-cell partial-failure surfacing, and a
 * long-press armed-write gate.
 */
export default function ParametersTab({ vehicleList, fullParams }) {
  const { t } = useTranslation();
  const sysIds = useMemo(
    () => (vehicleList || []).map((v) => v.sys_id),
    [vehicleList],
  );

  const [selectedPrefix, setSelectedPrefix] = useState('(all)');
  // Compare-mode sidebar: functional priority group in focus ('all' = every group).
  const [selectedGroup, setSelectedGroup] = useState('all');
  const [searchText, setSearchText] = useState('');
  const [modifiedOnly, setModifiedOnly] = useState(false);
  const [nonDefaultOnly, setNonDefaultOnly] = useState(false);
  const [differingOnly, setDifferingOnly] = useState(false);
  const [refreshingFlag, setRefreshingFlag] = useState(false);
  const [writingFlag, setWritingFlag] = useState(false);
  const [longPressActive, setLongPressActive] = useState(false);
  const [longPressMs, setLongPressMs] = useState(0);
  const [paramFileReport, setParamFileReport] = useState(null);
  const fileInputRef = useRef(null);

  // Compare mode: a loaded .param file diffed against current values. Kept
  // entirely separate from drafts — loading a compare file never stages a draft.
  const [compareFile, setCompareFile] = useState(null); // { name, rows, parseErrors }
  const [skipUavSpecific, setSkipUavSpecific] = useState(true);
  const [compareDiffersOnly, setCompareDiffersOnly] = useState(true);
  const [selectedNames, setSelectedNames] = useState(() => new Set());
  const [applyResult, setApplyResult] = useState(null); // aggregated result of last Apply
  const compareFileInputRef = useRef(null);

  // Sync UAVs (harmonize) mode: reconcile params where the connected UAVs differ.
  // Mutually exclusive with compare mode. Uses the shared skipUavSpecific toggle.
  const [harmonizeMode, setHarmonizeMode] = useState(false);
  const [harmonizeSelected, setHarmonizeSelected] = useState(() => new Set());
  const [harmonizeWinners, setHarmonizeWinners] = useState({}); // { name: { value, sysId } }
  const [harmonizeApplyResult, setHarmonizeApplyResult] = useState(null);

  const snapshotsByVehicle = fullParams?.snapshotsByVehicle || {};
  const draftsByVehicle = fullParams?.draftsByVehicle || {};
  const lastStatus = fullParams?.lastStatus || { kind: 'idle' };

  // First-visit + auto-fetch newly-connected vehicles. Ref guard against
  // refetch loops on failure.
  const fetchedRef = useRef(new Set());
  useEffect(() => {
    if (!fullParams) return;
    const live = new Set(sysIds);
    for (const sid of fetchedRef.current) {
      if (!live.has(sid)) fetchedRef.current.delete(sid);
    }
    const missing = sysIds.filter(
      (sid) => !snapshotsByVehicle[sid] && !fetchedRef.current.has(sid),
    );
    if (missing.length === 0) return;
    for (const sid of missing) fetchedRef.current.add(sid);
    fullParams.refreshVehicles(missing);
  }, [sysIds, snapshotsByVehicle, fullParams]);

  const canonical = useMemo(
    () => canonicalNameOrder(snapshotsByVehicle, sysIds),
    [snapshotsByVehicle, sysIds],
  );

  const compareMode = !!compareFile;
  const compareModel = useMemo(() => {
    if (!compareFile) return null;
    // No skipUavSpecific here — compare's default selection no longer depends on
    // it (Codex gate 15: that flag is Sync UAVs' own toggle; sharing it let that
    // unrelated mode's state leak into compare's selection).
    return computeParamCompare({
      fileRows: compareFile.rows,
      snapshotsByVehicle,
      sysIds,
    });
  }, [compareFile, snapshotsByVehicle, sysIds]);
  const compareRowsByName = useMemo(() => {
    if (!compareModel) return null;
    const out = {};
    for (const r of compareModel.rows) out[r.name] = r;
    return out;
  }, [compareModel]);

  const harmonizeModel = useMemo(() => {
    if (!harmonizeMode) return null;
    return computeFleetHarmonize({ snapshotsByVehicle, sysIds, skipUavSpecific });
  }, [harmonizeMode, snapshotsByVehicle, sysIds, skipUavSpecific]);
  const harmonizeRowsByName = useMemo(() => {
    if (!harmonizeModel) return null;
    const out = {};
    for (const r of harmonizeModel.rows) out[r.name] = r;
    return out;
  }, [harmonizeModel]);

  // Selection lifecycle: nothing is pre-selected — a fresh file, a skip-toggle
  // change, or a late vehicle finishing load all clear the selection so the
  // operator explicitly opts every row in (individually or via "Select all",
  // which still targets the safe/recommended subset — see scopedDefaultNames).
  // On a snapshot-driven recompute (refresh/post-write) with no reset trigger,
  // prune to still-writable rows while preserving manual (e.g. identity) picks
  // — per Codex gate-09.
  const prevCompareFileRef = useRef(null);
  const prevSkipRef = useRef(skipUavSpecific);
  const prevLoadedKeyRef = useRef('');
  useEffect(() => {
    const loadedKey = sysIds.filter((sid) => !!snapshotsByVehicle[sid]).join(',');
    if (!compareModel) {
      prevCompareFileRef.current = compareFile;
      prevSkipRef.current = skipUavSpecific;
      prevLoadedKeyRef.current = loadedKey;
      setSelectedNames(new Set());
      setApplyResult(null);
      return;
    }
    const isReset = prevCompareFileRef.current !== compareFile
      || prevSkipRef.current !== skipUavSpecific
      || prevLoadedKeyRef.current !== loadedKey;
    prevCompareFileRef.current = compareFile;
    prevSkipRef.current = skipUavSpecific;
    prevLoadedKeyRef.current = loadedKey;
    if (isReset) {
      setSelectedNames(new Set());
      setApplyResult(null);
    } else {
      setSelectedNames((prev) => new Set(reconcileCompareSelection(prev, compareModel.rows)));
    }
  }, [compareModel, compareFile, skipUavSpecific, sysIds, snapshotsByVehicle]);

  const toggleCompareRow = (name) => {
    setApplyResult(null); // a fresh selection -> last apply's status/errors are stale
    setSelectedNames((prev) => {
      const next = new Set(prev);
      if (next.has(name)) next.delete(name); else next.add(name);
      return next;
    });
  };
  const defaultSelectionNames = useMemo(
    () => (compareModel ? defaultCompareSelection(compareModel.rows) : []),
    [compareModel],
  );
  // Master "select all" is scoped to the focused group: when a functional group is
  // selected in the sidebar it only touches that group's default-selectable rows,
  // leaving other groups' picks alone ('all' = every group, as before).
  const scopedDefaultNames = useMemo(() => {
    if (!compareMode || selectedGroup === 'all') return defaultSelectionNames;
    return defaultSelectionNames.filter((n) => groupBucketForRow(compareRowsByName?.[n]) === selectedGroup);
  }, [compareMode, selectedGroup, defaultSelectionNames, compareRowsByName]);
  const masterChecked = scopedDefaultNames.length > 0
    && scopedDefaultNames.every((n) => selectedNames.has(n));
  const masterIndeterminate = scopedDefaultNames.some((n) => selectedNames.has(n)) && !masterChecked;
  const onToggleAllCompare = () => {
    setApplyResult(null);
    setSelectedNames((prev) => {
      if (scopedDefaultNames.length > 0 && scopedDefaultNames.every((n) => prev.has(n))) {
        // checked -> clear only THIS group's defaults (keep other groups' picks)
        const next = new Set(prev);
        for (const n of scopedDefaultNames) next.delete(n);
        return next;
      }
      // mixed/unchecked -> add this group's defaults, preserving manual picks
      return new Set([...prev, ...scopedDefaultNames]);
    });
  };
  // Selected rows that actually have something to write.
  const applySelectedCount = useMemo(() => {
    if (!compareRowsByName) return 0;
    let n = 0;
    for (const name of selectedNames) if (compareRowsByName[name]?.anyDiffers) n += 1;
    return n;
  }, [selectedNames, compareRowsByName]);
  // Per-cell apply errors, scoped to the last Apply (not a prior normal write).
  const compareCellErrorsByVehicle = useMemo(() => {
    if (!applyResult) return null;
    const out = {};
    for (const e of (applyResult.errors || [])) {
      if (!out[e.sysId]) out[e.sysId] = {};
      out[e.sysId][e.name] = e.error;
    }
    return out;
  }, [applyResult]);

  // ---- Harmonize selection / winner lifecycle (mirrors compare) ----------
  // Nothing is pre-selected — a reset (entering sync, a skip-toggle change, or a
  // late vehicle finishing load) clears the selection so the operator explicitly
  // opts every row in (individually or via "Select all", which still targets the
  // safe/recommended subset — see hzDefaultSelection).
  const prevHzOnRef = useRef(false);
  const prevHzSkipRef = useRef(skipUavSpecific);
  const prevHzLoadedRef = useRef('');
  useEffect(() => {
    const loadedKey = sysIds.filter((sid) => !!snapshotsByVehicle[sid]).join(',');
    if (!harmonizeModel) {
      prevHzOnRef.current = harmonizeMode;
      prevHzSkipRef.current = skipUavSpecific;
      prevHzLoadedRef.current = loadedKey;
      setHarmonizeSelected(new Set());
      setHarmonizeWinners({});
      setHarmonizeApplyResult(null);
      return;
    }
    const isReset = prevHzOnRef.current !== harmonizeMode
      || prevHzSkipRef.current !== skipUavSpecific
      || prevHzLoadedRef.current !== loadedKey;
    prevHzOnRef.current = harmonizeMode;
    prevHzSkipRef.current = skipUavSpecific;
    prevHzLoadedRef.current = loadedKey;
    if (isReset) {
      setHarmonizeSelected(new Set());
      setHarmonizeWinners({});
      setHarmonizeApplyResult(null);
    } else {
      // Uses the current-render harmonizeWinners (intentionally NOT a dep — adding
      // it would loop, since the reset branch sets a fresh winners object). The
      // effectiveWinner self-heal guards correctness at write time regardless.
      const reconciled = reconcileHarmonizeSelection(
        harmonizeSelected, harmonizeModel.rows, harmonizeWinners,
      );
      setHarmonizeSelected(new Set(reconciled));
      // A plain recompute (e.g. post-write/refresh) can also drop a row out of
      // the selection — no longer divergent, or its winner became invalid. Same
      // staleness rule as the two operator-driven deselect paths below: forget
      // that row's manual pick too, so a LATER divergence of the same param
      // can't silently resurface a pick the operator can no longer see was made
      // (Codex gate 15 follow-up on PR #90 — deferred, non-blocking there).
      const dropped = droppedHarmonizeWinnerNames(harmonizeSelected, reconciled);
      if (dropped.length > 0) {
        setHarmonizeWinners((prev) => {
          let changed = false;
          const next = { ...prev };
          for (const n of dropped) {
            if (n in next) { delete next[n]; changed = true; }
          }
          return changed ? next : prev;
        });
      }
    }
  }, [harmonizeModel, harmonizeMode, skipUavSpecific, sysIds, snapshotsByVehicle]);

  const onPickWinner = (name, value, sysId) => {
    setHarmonizeApplyResult(null);
    // Store the chosen value AND the clicked UAV (the "base"), so the grid can
    // badge exactly which cell the operator picked — not every cell that happens
    // to share that value. effectiveWinner/effectiveBaseSysId read this shape.
    setHarmonizeWinners((prev) => ({ ...prev, [name]: { value, sysId } }));
  };
  const toggleHarmonizeRow = (name) => {
    setHarmonizeApplyResult(null);
    const wasSelected = harmonizeSelected.has(name);
    setHarmonizeSelected((prev) => {
      const next = new Set(prev);
      if (next.has(name)) next.delete(name); else next.add(name);
      return next;
    });
    if (wasSelected) {
      // Deselecting (via the checkbox or "click the winner cell again")
      // forgets any manual winner pick for this row, so a later reselect
      // starts fresh at the modal/default winner instead of resurrecting a
      // stale pick the operator can no longer see was chosen (Codex gate 15
      // — blocking: a picked winner used to survive deselection and could
      // silently apply again via master select-all or a checkbox reselect).
      setHarmonizeWinners((prev) => {
        if (!(name in prev)) return prev;
        const { [name]: _dropped, ...rest } = prev;
        return rest;
      });
    }
  };
  const hzDefaultSelection = useMemo(
    () => (harmonizeModel ? defaultHarmonizeSelection(harmonizeModel.rows) : []),
    [harmonizeModel],
  );
  const hzMasterChecked = hzDefaultSelection.length > 0
    && hzDefaultSelection.every((n) => harmonizeSelected.has(n));
  const hzMasterIndeterminate = harmonizeSelected.size > 0 && !hzMasterChecked;
  const onToggleAllHarmonize = () => {
    setHarmonizeApplyResult(null);
    const allSelected = hzDefaultSelection.length > 0
      && hzDefaultSelection.every((n) => harmonizeSelected.has(n));
    if (allSelected) {
      // Clearing the whole selection also forgets every manual winner pick
      // that was selected — same reason as toggleHarmonizeRow above; a mass
      // deselect via "select all" must not leave stale picks that resurface
      // on a later reselect (Codex gate 15 — blocking).
      const cleared = harmonizeSelected;
      setHarmonizeSelected(new Set());
      setHarmonizeWinners((prev) => {
        let changed = false;
        const next = { ...prev };
        for (const n of cleared) {
          if (n in next) { delete next[n]; changed = true; }
        }
        return changed ? next : prev;
      });
    } else {
      setHarmonizeSelected((prev) => new Set([...prev, ...hzDefaultSelection]));
    }
  };
  const harmonizeChanges = useMemo(
    () => (harmonizeModel
      ? collectHarmonizeChanges(harmonizeModel.rows, harmonizeSelected, harmonizeWinners)
      : {}),
    [harmonizeModel, harmonizeSelected, harmonizeWinners],
  );
  const harmonizeRowCount = useMemo(() => {
    const names = new Set();
    for (const sid of Object.keys(harmonizeChanges)) {
      for (const ch of harmonizeChanges[sid]) names.add(ch.name);
    }
    return names.size;
  }, [harmonizeChanges]);
  const harmonizeCellErrorsByVehicle = useMemo(() => {
    if (!harmonizeApplyResult) return null;
    const out = {};
    for (const e of (harmonizeApplyResult.errors || [])) {
      if (!out[e.sysId]) out[e.sysId] = {};
      out[e.sysId][e.name] = e.error;
    }
    return out;
  }, [harmonizeApplyResult]);

  // Per-vehicle valid-change diff. Uses the same coercion semantics as
  // the write path so an invalid string draft like "abc" or a no-op
  // "1.0" vs 1 doesn't get counted (per Codex post-step finding 3 for
  // Step 6).
  const changesByVehicle = useMemo(() => {
    const out = {};
    for (const sid of sysIds) {
      const draft = draftsByVehicle[sid];
      const snap = snapshotsByVehicle[sid];
      if (!draft || !snap) { out[sid] = []; continue; }
      out[sid] = selectChangedNames(draft, snap);
    }
    return out;
  }, [sysIds, draftsByVehicle, snapshotsByVehicle]);

  // Modified-only filter: union of valid-change names across vehicles.
  const modifiedNames = useMemo(() => {
    if (!modifiedOnly) return null;
    const out = new Set();
    for (const sid of sysIds) {
      for (const c of (changesByVehicle[sid] || [])) out.add(c.name);
    }
    return out;
  }, [modifiedOnly, sysIds, changesByVehicle]);

  const nonDefaultNames = useMemo(() => {
    if (!nonDefaultOnly) return null;
    return new Set(selectNonDefaultNames(
      snapshotsByVehicle,
      sysIds,
      canonical,
      draftsByVehicle,
    ));
  }, [nonDefaultOnly, snapshotsByVehicle, sysIds, canonical, draftsByVehicle]);

  // Row universe: harmonize -> the divergent params; compare -> the file's params;
  // otherwise the union of vehicle params (canonical). Read-only (view-only
  // firmware) rows are excluded from the harmonize prefix tree so it stays clean —
  // they live only in the dedicated "Read-only" sidebar item (see readOnlyNames).
  const sidebarSource = useMemo(() => {
    if (harmonizeMode) return (harmonizeModel?.rows || []).filter((r) => !r.readOnly).map((r) => r.name);
    if (compareMode) return compareModel?.order || [];
    return canonical;
  }, [harmonizeMode, harmonizeModel, compareMode, compareModel, canonical]);
  // View-only read-only params for the current mode's model — the "Read-only"
  // sidebar bucket's contents (harmonize prefix tree / compare group both draw on
  // this). Excluded from every writable/functional view.
  const readOnlyNames = useMemo(() => {
    const model = harmonizeMode ? harmonizeModel : (compareMode ? compareModel : null);
    return model ? model.rows.filter((r) => r.readOnly).map((r) => r.name) : [];
  }, [harmonizeMode, harmonizeModel, compareMode, compareModel]);
  const readOnlyCount = readOnlyNames.length;

  const differingNames = useMemo(() => {
    if (!differingOnly) return null;
    // Filter stays inactive (shows everything) until at least two UAVs have
    // actually loaded, so it can't hide every row when the checkbox is hidden
    // or while a UAV is still downloading.
    const loadedCount = sysIds.filter((sid) => snapshotsByVehicle[sid]?.paramsByName).length;
    if (loadedCount < 2) return null;
    return differingParamNames(snapshotsByVehicle, sysIds, canonical);
  }, [differingOnly, snapshotsByVehicle, sysIds, canonical]);
  const sidebarPrefixes = useMemo(
    () => buildPrefixSidebar(sidebarSource),
    [sidebarSource],
  );
  // Compare-mode sidebar groups: ordered, non-empty functional groups with diff
  // counts (Option A). Replaces the prefix list while comparing a file.
  const compareGroups = useMemo(
    () => (compareMode && compareModel ? summarizeCompareGroups(compareModel.rows) : null),
    [compareMode, compareModel],
  );
  const visibleNames = useMemo(() => {
    // Read-only view (harmonize sidebar's "Read-only" item): only the view-only
    // firmware params, search-filtered. They're absent from sidebarSource, so this
    // is the one place they surface.
    if (harmonizeMode && selectedPrefix === '(readonly)') {
      return filterCanonicalNames(readOnlyNames, { selectedPrefix: '(all)', searchText });
    }
    let names = filterCanonicalNames(sidebarSource, { selectedPrefix, searchText });
    if (harmonizeMode) return names; // divergent rows; read-only already excluded
    if (compareMode) {
      // "Differs only" hides identical rows; differ / file-only / pending stay.
      if (compareDiffersOnly && compareRowsByName) {
        names = names.filter((n) => compareRowsByName[n]?.status !== 'same');
      }
      // Focus the grid on the selected functional group ('all' shows every group,
      // but never read-only — those live only in their own bucket). Priority
      // grouping replaces the old blunt "hide non-actionable" toggle: the per-board
      // / wiring / file-only / read-only groups are still one click away, not gone.
      if (compareRowsByName) {
        if (selectedGroup === 'all') {
          names = names.filter((n) => !compareRowsByName[n]?.readOnly);
        } else {
          names = names.filter((n) => {
            const r = compareRowsByName[n];
            if (!r) return false;
            return groupBucketForRow(r) === selectedGroup;
          });
        }
      }
      return names;
    }
    if (modifiedNames) names = names.filter((n) => modifiedNames.has(n));
    if (nonDefaultNames) names = names.filter((n) => nonDefaultNames.has(n));
    if (differingNames) names = names.filter((n) => differingNames.has(n));
    return names;
  }, [sidebarSource, readOnlyNames, selectedPrefix, searchText, harmonizeMode, compareMode,
    compareDiffersOnly, selectedGroup, compareRowsByName, modifiedNames, nonDefaultNames,
    differingNames]);

  const staleByVehicle = useMemo(() => {
    const out = {};
    for (const sid of sysIds) out[sid] = !!snapshotsByVehicle[sid]?.stale;
    return out;
  }, [snapshotsByVehicle, sysIds]);

  // Per-cell errors from the last write batch.
  const cellErrorsByVehicle = useMemo(() => {
    if (lastStatus?.op !== 'write' || !lastStatus.counts) return {};
    const errs = {};
    for (const e of (lastStatus.counts.errors || [])) {
      if (!errs[e.sysId]) errs[e.sysId] = {};
      errs[e.sysId][e.name] = e.error;
    }
    return errs;
  }, [lastStatus]);

  // ---- .param file save/load --------------------------------------
  const onSaveParamFile = () => {
    // Save the FIRST loaded vehicle (most common case is single-vehicle).
    // Multi-vehicle save: open the file dialog per vehicle would be noisy,
    // so v1 saves a single file based on whichever vehicle is "primary".
    const sid = sysIds.find((s) => !!snapshotsByVehicle[s]);
    if (sid == null) return;
    const snap = snapshotsByVehicle[sid];
    const vehicle = (vehicleList || []).find((v) => v.sys_id === sid);
    const vehicleName = vehicle?.name || `s1-u${sid}`;
    const text = snapshotToParamFile(snap, {
      vehicleName,
      exportedAtIso: new Date().toISOString(),
    });
    const blob = new Blob([text], { type: 'text/plain;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `${vehicleName}.param`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  };

  const onLoadParamFileClick = () => {
    fileInputRef.current?.click();
  };

  const onLoadParamFileSelected = async (e) => {
    const file = e.target.files?.[0];
    e.target.value = '';
    if (!file) return;
    const text = await file.text();
    const parsed = parseParamFile(text);
    // Validate against each connected vehicle's snapshot. Stage accepted
    // rows into each vehicle's draft. Show the operator a summary
    // BEFORE staging (writePatch is what actually triggers PUT later).
    const perVehicle = {};
    let totalAccepted = 0;
    let totalSkipped = 0;
    let totalRejected = 0;
    let totalCoerced = 0;
    for (const sid of sysIds) {
      const snap = snapshotsByVehicle[sid];
      const v = validateRowsForVehicle(parsed.rows, snap);
      perVehicle[sid] = v;
      totalAccepted += v.accepted.length;
      totalSkipped += v.skipped.length;
      totalRejected += v.rejected.length;
      // Surface int rows the autopilot would store differently (truncate/clamp)
      // rather than hiding the adjustment in the accepted count.
      totalCoerced += v.accepted.filter((a) => a.coerced).length;
    }
    // Stage accepted rows per vehicle directly via setVehicleDraft so the
    // grid shows them as drafts; user clicks Write to commit.
    for (const sid of sysIds) {
      for (const a of (perVehicle[sid].accepted || [])) {
        fullParams.setVehicleDraft(sid, a.name, a.value);
      }
    }
    setParamFileReport({
      filename: file.name,
      parseErrors: parsed.errors.length,
      accepted: totalAccepted,
      skipped: totalSkipped,
      rejected: totalRejected,
      coerced: totalCoerced,
      perVehicle,
    });
  };

  // ---- Compare .param (separate from Load — never stages drafts) --------
  const onCompareFileClick = () => compareFileInputRef.current?.click();
  const onCompareFileSelected = async (e) => {
    const file = e.target.files?.[0];
    e.target.value = '';
    if (!file) return;
    const text = await file.text();
    const parsed = parseParamFile(text);
    setHarmonizeMode(false); // mutually exclusive with sync mode
    setCompareFile({ name: file.name, rows: parsed.rows, parseErrors: parsed.errors.length });
    // Reset the prefix so a stale selection from the vehicle tree doesn't hide
    // the freshly loaded file's rows.
    setSelectedPrefix('(all)');
    setSelectedGroup('all');
  };
  const onExitCompare = () => {
    setCompareFile(null);
    setSelectedPrefix('(all)');
    setSelectedGroup('all');
  };

  // ---- Sync UAVs (harmonize) mode toggle (mutually exclusive with compare) ----
  const onEnterHarmonize = () => {
    setCompareFile(null);
    setSelectedPrefix('(all)');
    setSelectedGroup('all');
    setHarmonizeMode(true);
  };
  const onExitHarmonize = () => {
    setHarmonizeMode(false);
    setSelectedPrefix('(all)');
    setSelectedGroup('all');
  };

  const onRefresh = async () => {
    if (refreshingFlag) return;
    setApplyResult(null); // a manual refresh supersedes the last apply's status
    setHarmonizeApplyResult(null);
    setRefreshingFlag(true);
    try {
      await fullParams.refreshVehicles(sysIds, { force: true });
    } finally {
      setRefreshingFlag(false);
    }
  };

  // Re-download a single UAV (e.g. after a failed download) without touching
  // the others. Triggered by clicking that UAV's failed indicator.
  const onRetryVehicle = (sid) => {
    if (sid == null) return;
    fullParams.refreshVehicles([sid], { force: true });
  };

  const anyArmed = useMemo(
    () => (vehicleList || []).some((v) => v.armed),
    [vehicleList],
  );

  const draftCount = useMemo(() => {
    let n = 0;
    for (const sid of sysIds) n += (changesByVehicle[sid] || []).length;
    return n;
  }, [sysIds, changesByVehicle]);

  const performWrite = async () => {
    if (writingFlag || draftCount === 0) return;
    setWritingFlag(true);
    try {
      // For armed vehicles, request a token first so the backend accepts the PUT.
      const armedIds = (vehicleList || []).filter((v) => v.armed).map((v) => v.sys_id);
      for (const sid of armedIds) {
        await fullParams.requestArmedToken(sid);
      }
      // Do NOT auto-refresh after write — refreshVehicles drops drafts
      // wholesale, which would lose unsuccessful cells the operator may
      // want to retry (per Codex post-step finding 1 for Step 6). The
      // per-vehicle stale badge tells the user when to manually refresh.
      await fullParams.writeChangedToVehicles(sysIds);
    } finally {
      setWritingFlag(false);
    }
  };

  // Compare-apply: write only the selected differing file values via the scoped
  // writer. Passes NUMERIC sysIds from vehicle state (never Object.keys of the
  // string-keyed changes map) so arm-token + result attribution line up.
  const performApply = async () => {
    if (writingFlag || !compareModel) return;
    const changesByVehicle = collectCompareChanges(compareModel.rows, selectedNames);
    if (Object.keys(changesByVehicle).length === 0) return;
    setWritingFlag(true);
    try {
      // Only request arm tokens for armed vehicles that actually have a write in
      // this apply, so an unrelated armed UAV doesn't get a needless token.
      const armedIds = (vehicleList || [])
        .filter((v) => v.armed && (changesByVehicle[v.sys_id] || []).length > 0)
        .map((v) => v.sys_id);
      for (const sid of armedIds) {
        await fullParams.requestArmedToken(sid);
      }
      const res = await fullParams.writeChangedToVehicles(sysIds, { changesByVehicle });
      setApplyResult(res?.aggregated || null);
    } finally {
      setWritingFlag(false);
    }
  };

  // Harmonize-apply: push each selected row's winner to its outlier UAVs. Only
  // runs when the whole fleet is loaded (harmonizeModel.allLoaded).
  const performHarmonizeApply = async () => {
    if (writingFlag || !harmonizeModel || !harmonizeModel.allLoaded) return;
    const changesByVehicle = harmonizeChanges;
    if (Object.keys(changesByVehicle).length === 0) return;
    setWritingFlag(true);
    try {
      const armedIds = (vehicleList || [])
        .filter((v) => v.armed && (changesByVehicle[v.sys_id] || []).length > 0)
        .map((v) => v.sys_id);
      for (const sid of armedIds) {
        await fullParams.requestArmedToken(sid);
      }
      const res = await fullParams.writeChangedToVehicles(sysIds, { changesByVehicle });
      setHarmonizeApplyResult(res?.aggregated || null);
    } finally {
      setWritingFlag(false);
    }
  };

  // Long-press gate for armed writes. When any vehicle is armed, the operator
  // must hold the Write Changed / Apply button for ARMED_LONG_PRESS_MS before
  // the backend's arm-token check is requested. Disarmed vehicles skip it.
  const longPressTimer = useRef(null);
  const longPressInterval = useRef(null);
  const beginArmedPress = (action) => {
    if (!anyArmed) {
      action();
      return;
    }
    setLongPressActive(true);
    setLongPressMs(0);
    const startedAt = Date.now();
    longPressInterval.current = setInterval(() => {
      setLongPressMs(Math.min(ARMED_LONG_PRESS_MS, Date.now() - startedAt));
    }, 50);
    longPressTimer.current = setTimeout(() => {
      cancelLongPress();
      action();
    }, ARMED_LONG_PRESS_MS);
  };
  const onWriteMouseDown = () => beginArmedPress(performWrite);
  const cancelLongPress = () => {
    if (longPressTimer.current) clearTimeout(longPressTimer.current);
    if (longPressInterval.current) clearInterval(longPressInterval.current);
    longPressTimer.current = null;
    longPressInterval.current = null;
    setLongPressActive(false);
    setLongPressMs(0);
  };
  useEffect(() => () => cancelLongPress(), []);
  // Cancel any in-progress long-press when compare mode toggles or when the
  // selection/model changes mid-hold, so an armed Apply timer can't fire with a
  // stale selected set (or after the compare UI has been exited).
  useEffect(() => { cancelLongPress(); }, [
    compareMode, selectedNames, compareModel,
    harmonizeMode, harmonizeSelected, harmonizeWinners, harmonizeModel,
  ]);

  const totalLoaded = sysIds.filter((sid) => !!snapshotsByVehicle[sid]).length;
  const totalRows = sidebarSource.length;
  const anyRefreshPending = Object.values(fullParams?.pendingByVehicle || {})
    .some((kind) => kind === 'refresh');
  const paramsRefreshing = refreshingFlag
    || anyRefreshPending;

  // Per-vehicle download progress + edited flag, surfaced as a ring/check in
  // each grid column header.
  const progressByVehicle = useMemo(() => {
    const durable = fullParams?.refreshProgressByVehicle;
    if (durable) {
      const out = {};
      for (const [sid, p] of Object.entries(durable)) {
        const bytesRead = Math.max(0, Number(p.bytesRead) || 0);
        const denominator = Math.max(0, Number(p.totalBytes || p.sizeEstimate) || 0);
        out[sid] = {
          ...p,
          sysId: Number(sid),
          bytesRead,
          hasBytes: bytesRead > 0 || denominator > 0,
          percent: denominator > 0
            ? Math.min(100, Math.round((bytesRead / denominator) * 100))
            : (p.done ? 100 : 0),
        };
      }
      return out;
    }
    const lp = fullParamLoadProgress(lastStatus);
    const out = {};
    for (const p of lp?.vehicleProgress || []) out[p.sysId] = p;
    return out;
  }, [fullParams?.refreshProgressByVehicle, lastStatus]);
  const changedByVehicle = useMemo(() => {
    const out = {};
    for (const sid of sysIds) out[sid] = (changesByVehicle[sid] || []).length > 0;
    return out;
  }, [sysIds, changesByVehicle]);

  if (sysIds.length === 0) {
    return (
      <div style={{ padding: 24, color: colors.textDim }}>
        {t('settings.parameters.no_vehicle', 'No connected vehicle. Connect to fetch parameters.')}
      </div>
    );
  }

  const writeBtnLabel = writingFlag
    ? t('settings.parameters.writing', 'Writing…')
    : draftCount === 0
      ? t('settings.parameters.write_disabled', 'No changes')
      : anyArmed
        ? `${t('settings.parameters.write_armed', 'Hold to Write')} (${draftCount})`
        : `${t('settings.parameters.write_changed', 'Write Changed')} (${draftCount})`;

  return (
    <div style={{
      display: 'flex',
      flexDirection: 'column',
      height: '100%',
      minHeight: 480,
      background: colors.bgLight,
      color: colors.text,
    }}
    aria-busy={paramsRefreshing ? 'true' : 'false'}>
      <div style={{
        display: 'flex',
        gap: 8,
        alignItems: 'center',
        flexWrap: 'wrap',
        padding: '8px 4px',
        borderBottom: `1px solid ${colors.border}`,
      }}>
        <button
          onClick={onRefresh}
          disabled={paramsRefreshing}
          style={{
            padding: '4px 12px',
            background: paramsRefreshing ? colors.surfaceLight : colors.accent,
            color: paramsRefreshing ? colors.textDim : colors.bg,
            border: `1px solid ${paramsRefreshing ? colors.border : colors.accent}`,
            borderRadius: 4,
            cursor: paramsRefreshing ? 'wait' : 'pointer',
            fontWeight: 600,
          }}
        >
          {paramsRefreshing
            ? t('settings.parameters.refreshing', 'Refreshing…')
            : t('settings.parameters.refresh', 'Refresh')}
        </button>
        {!compareMode && !harmonizeMode && (
        <button
          onMouseDown={onWriteMouseDown}
          onMouseUp={anyArmed ? cancelLongPress : undefined}
          onMouseLeave={anyArmed ? cancelLongPress : undefined}
          disabled={paramsRefreshing || writingFlag || draftCount === 0}
          style={{
            padding: '4px 14px',
            position: 'relative',
            background: paramsRefreshing || writingFlag || draftCount === 0
              ? colors.surfaceLight
              : anyArmed ? colors.error : colors.success,
            color: paramsRefreshing || writingFlag || draftCount === 0 ? colors.textDim : colors.textBright,
            border: `1px solid ${paramsRefreshing || writingFlag || draftCount === 0 ? colors.border : 'transparent'}`,
            borderRadius: 4,
            cursor: paramsRefreshing || writingFlag || draftCount === 0 ? 'default' : 'pointer',
            overflow: 'hidden',
            fontWeight: 600,
          }}
        >
          {writeBtnLabel}
          {anyArmed && longPressActive && (
            <span
              style={{
                position: 'absolute',
                left: 0, bottom: 0,
                height: 3, width: `${(longPressMs / ARMED_LONG_PRESS_MS) * 100}%`,
                background: colors.textBright,
              }}
            />
          )}
        </button>
        )}
        <button
          onClick={onSaveParamFile}
          disabled={paramsRefreshing || totalLoaded === 0}
          style={{
            padding: '4px 10px',
            background: paramsRefreshing || totalLoaded === 0 ? colors.surfaceLight : colors.surface,
            color: paramsRefreshing || totalLoaded === 0 ? colors.textDim : colors.text,
            border: `1px solid ${colors.border}`,
            borderRadius: 4,
            cursor: paramsRefreshing || totalLoaded === 0 ? 'default' : 'pointer',
          }}
        >
          {t('settings.parameters.save_file', 'Save .param')}
        </button>
        <button
          onClick={onLoadParamFileClick}
          disabled={paramsRefreshing}
          style={{
            padding: '4px 10px',
            background: paramsRefreshing ? colors.surfaceLight : colors.surface,
            color: paramsRefreshing ? colors.textDim : colors.text,
            border: `1px solid ${colors.border}`,
            borderRadius: 4,
            cursor: paramsRefreshing ? 'default' : 'pointer',
          }}
        >
          {t('settings.parameters.load_file', 'Load .param')}
        </button>
        <input
          ref={fileInputRef}
          type="file"
          accept=".param,.txt,text/*"
          style={{ display: 'none' }}
          onChange={onLoadParamFileSelected}
        />
        <button
          onClick={onCompareFileClick}
          disabled={paramsRefreshing}
          style={{
            padding: '4px 10px',
            background: compareMode ? colors.surfaceLight : colors.surface,
            color: paramsRefreshing ? colors.textDim : colors.accent,
            border: `1px solid ${colors.accent}`,
            borderRadius: 4,
            cursor: paramsRefreshing ? 'default' : 'pointer',
            fontWeight: 600,
          }}
        >
          {t('settings.parameters.compare_file', 'Compare .param')}
        </button>
        <input
          ref={compareFileInputRef}
          type="file"
          accept=".param,.txt,text/*"
          style={{ display: 'none' }}
          onChange={onCompareFileSelected}
        />
        <button
          onClick={onEnterHarmonize}
          disabled={paramsRefreshing || sysIds.length < 2}
          title={sysIds.length < 2
            ? t('settings.parameters.hz_need_two', 'Connect at least two UAVs to sync')
            : t('settings.parameters.hz_title', 'Sync UAVs')}
          style={{
            padding: '4px 10px',
            background: harmonizeMode ? colors.surfaceLight : colors.surface,
            color: (paramsRefreshing || sysIds.length < 2) ? colors.textDim : colors.accent,
            border: `1px solid ${colors.accent}`,
            borderRadius: 4,
            cursor: (paramsRefreshing || sysIds.length < 2) ? 'default' : 'pointer',
            fontWeight: 600,
          }}
        >
          {t('settings.parameters.hz_title', 'Sync UAVs')}
        </button>
        <input
          type="text"
          value={searchText}
          onChange={(e) => setSearchText(e.target.value)}
          placeholder={t('settings.parameters.search', 'Search…')}
          style={{
            flex: 1,
            minWidth: 180,
            padding: '4px 8px',
            background: colors.bg,
            color: colors.text,
            border: `1px solid ${colors.border}`,
            borderRadius: 4,
            fontFamily: 'monospace',
          }}
        />
        {!compareMode && !harmonizeMode && (
        <label style={{ display: 'flex', alignItems: 'center', gap: 4, color: colors.textDim, fontSize: 12 }}>
          <input
            type="checkbox"
            checked={modifiedOnly}
            onChange={(e) => setModifiedOnly(e.target.checked)}
            style={{ accentColor: colors.accent }}
          />
          {t('settings.parameters.modified_only', 'Modified only')}
        </label>
        )}
        {!compareMode && !harmonizeMode && (
        <label style={{ display: 'flex', alignItems: 'center', gap: 4, color: colors.textDim, fontSize: 12 }}>
          <input
            type="checkbox"
            checked={nonDefaultOnly}
            onChange={(e) => setNonDefaultOnly(e.target.checked)}
            style={{ accentColor: colors.accent }}
          />
          {t('settings.parameters.non_default_only', 'Non-default only')}
        </label>
        )}
        {!compareMode && !harmonizeMode && sysIds.length >= 2 && (
          <label style={{ display: 'flex', alignItems: 'center', gap: 4, color: colors.textDim, fontSize: 12 }}>
            <input
              type="checkbox"
              checked={differingOnly}
              onChange={(e) => setDifferingOnly(e.target.checked)}
              style={{ accentColor: colors.accent }}
            />
            {t('settings.parameters.differs_across', 'Differs across UAVs')}
          </label>
        )}
        <span style={{ color: colors.textDim, fontSize: 12 }}>
          {totalLoaded}/{sysIds.length} {t('settings.parameters.loaded', 'loaded')} · {totalRows} {t('settings.parameters.params', 'params')}
        </span>
      </div>

      {compareMode && (
        <ParamCompareBar
          filename={compareFile.name}
          counts={compareModel?.counts}
          parseErrors={compareFile.parseErrors}
          differsOnly={compareDiffersOnly}
          onToggleDiffers={setCompareDiffersOnly}
          applyStatus={applyResult ? {
            ok: applyResult.cellOk || 0,
            fail: applyResult.cellFail || 0,
          } : null}
          onExit={onExitCompare}
        >
          <button
            onMouseDown={() => { if (!(paramsRefreshing || writingFlag || applySelectedCount === 0)) beginArmedPress(performApply); }}
            onMouseUp={anyArmed ? cancelLongPress : undefined}
            onMouseLeave={anyArmed ? cancelLongPress : undefined}
            disabled={paramsRefreshing || writingFlag || applySelectedCount === 0}
            style={{
              padding: '5px 12px',
              position: 'relative',
              overflow: 'hidden',
              background: (paramsRefreshing || writingFlag || applySelectedCount === 0)
                ? colors.surfaceLight
                : anyArmed ? colors.error : colors.success,
              color: (paramsRefreshing || writingFlag || applySelectedCount === 0)
                ? colors.textDim : colors.textBright,
              border: `1px solid ${(paramsRefreshing || writingFlag || applySelectedCount === 0) ? colors.border : 'transparent'}`,
              borderRadius: 4,
              cursor: (paramsRefreshing || writingFlag || applySelectedCount === 0) ? 'default' : 'pointer',
              fontWeight: 600,
            }}
          >
            {writingFlag
              ? t('settings.parameters.applying', 'Applying…')
              : anyArmed
                ? `${t('settings.parameters.apply_armed', 'Hold to Apply')} (${applySelectedCount})`
                : `${t('settings.parameters.apply_selected', 'Apply selected')} (${applySelectedCount})`}
            {anyArmed && longPressActive && (
              <span style={{
                position: 'absolute', left: 0, bottom: 0,
                height: 3, width: `${(longPressMs / ARMED_LONG_PRESS_MS) * 100}%`,
                background: colors.textBright,
              }} />
            )}
          </button>
        </ParamCompareBar>
      )}

      {harmonizeMode && (
        <ParamHarmonizeBar
          counts={harmonizeModel?.counts}
          allLoaded={!!harmonizeModel?.allLoaded}
          loadedCount={totalLoaded}
          totalCount={sysIds.length}
          skipUavSpecific={skipUavSpecific}
          onToggleSkip={setSkipUavSpecific}
          applyStatus={harmonizeApplyResult ? {
            ok: harmonizeApplyResult.cellOk || 0,
            fail: harmonizeApplyResult.cellFail || 0,
          } : null}
          onExit={onExitHarmonize}
        >
          {(() => {
            const hzDisabled = paramsRefreshing || writingFlag
              || harmonizeRowCount === 0 || !harmonizeModel?.allLoaded;
            return (
              <button
                onMouseDown={() => { if (!hzDisabled) beginArmedPress(performHarmonizeApply); }}
                onMouseUp={anyArmed ? cancelLongPress : undefined}
                onMouseLeave={anyArmed ? cancelLongPress : undefined}
                disabled={hzDisabled}
                style={{
                  padding: '5px 12px',
                  position: 'relative',
                  overflow: 'hidden',
                  background: hzDisabled ? colors.surfaceLight : anyArmed ? colors.error : colors.success,
                  color: hzDisabled ? colors.textDim : colors.textBright,
                  border: `1px solid ${hzDisabled ? colors.border : 'transparent'}`,
                  borderRadius: 4,
                  cursor: hzDisabled ? 'default' : 'pointer',
                  fontWeight: 600,
                }}
              >
                {writingFlag
                  ? t('settings.parameters.applying', 'Applying…')
                  : anyArmed
                    ? `${t('settings.parameters.hz_apply_armed', 'Hold to Harmonize')} (${harmonizeRowCount})`
                    : `${t('settings.parameters.hz_apply', 'Harmonize selected')} (${harmonizeRowCount})`}
                {anyArmed && longPressActive && (
                  <span style={{
                    position: 'absolute', left: 0, bottom: 0,
                    height: 3, width: `${(longPressMs / ARMED_LONG_PRESS_MS) * 100}%`,
                    background: colors.textBright,
                  }} />
                )}
              </button>
            );
          })()}
        </ParamHarmonizeBar>
      )}

      {paramFileReport && (
        <div
          style={{
            padding: '6px 12px',
            background: paramFileReport.rejected > 0
              ? 'rgba(231, 76, 60, 0.15)'
              : 'rgba(39, 174, 96, 0.15)',
            borderBottom: `1px solid ${colors.border}`,
            color: colors.text,
            fontSize: 12,
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center',
            gap: 8,
          }}
        >
          <span>
            <strong>{paramFileReport.filename}</strong>
            {' '}— {paramFileReport.accepted} {t('settings.parameters.accepted', 'staged')}
            {paramFileReport.coerced > 0
              ? `, ${paramFileReport.coerced} ${t('settings.parameters.coerced', 'adjusted to fit')}`
              : ''}
            , {paramFileReport.skipped} {t('settings.parameters.skipped', 'skipped (unknown)')}
            , {paramFileReport.rejected} {t('settings.parameters.rejected', 'rejected (type/range)')}
            {paramFileReport.parseErrors > 0
              ? `, ${paramFileReport.parseErrors} ${t('settings.parameters.parse_errors', 'malformed lines')}`
              : ''}
          </span>
          <button
            onClick={() => setParamFileReport(null)}
            style={{ background: 'none', border: 'none', color: colors.textDim, cursor: 'pointer' }}
          >
            ×
          </button>
        </div>
      )}

      <div style={{ display: 'flex', flex: 1, gap: 8, minHeight: 0 }}>
        <div style={{
          width: 180,
          flex: '0 0 auto',
          overflowY: 'auto',
          borderRight: `1px solid ${colors.border}`,
          padding: '4px 0',
        }}>
          {compareMode && compareGroups ? (
            <React.Fragment>
              {/* Compare mode: functional priority groups (Option A) instead of the
                  prefix tree — click one to focus the grid, "All" shows every group.
                  Per-board / wiring / file-only sink to the bottom, muted. */}
              <PrefixItem
                label={t('settings.parameters.all', 'All')}
                count={compareGroups.totalDiffer}
                active={selectedGroup === 'all'}
                onClick={() => setSelectedGroup('all')}
              />
              {compareGroups.order.map((id) => (
                <PrefixItem
                  key={id}
                  label={t(`settings.parameters.grp_${id.replace(/-/g, '_')}`, PARAM_GROUP_LABELS[id])}
                  count={compareGroups.counts[id]}
                  active={selectedGroup === id}
                  muted={LOW_PRIORITY_GROUPS.has(id)}
                  onClick={() => setSelectedGroup(selectedGroup === id ? 'all' : id)}
                />
              ))}
            </React.Fragment>
          ) : (
            <React.Fragment>
              <PrefixItem
                label={t('settings.parameters.all', 'All')}
                count={totalRows}
                active={selectedPrefix === '(all)'}
                onClick={() => setSelectedPrefix('(all)')}
              />
              {sidebarPrefixes.map(({ prefix, count, children }) => {
                const showChildren = selectedPrefix === prefix
                  || selectedPrefix.startsWith(`${prefix}_`);
                return (
                  <React.Fragment key={prefix}>
                    <PrefixItem
                      label={prefix}
                      count={count}
                      active={selectedPrefix === prefix}
                      hasChildren={(children || []).length > 0}
                      expanded={showChildren}
                      onClick={() => setSelectedPrefix(showChildren ? '(all)' : prefix)}
                    />
                    {showChildren && (children || []).map((child) => (
                      <PrefixItem
                        key={child.prefix}
                        label={child.prefix}
                        count={child.count}
                        active={selectedPrefix === child.prefix}
                        depth={1}
                        onClick={() => setSelectedPrefix(child.prefix)}
                      />
                    ))}
                  </React.Fragment>
                );
              })}
              {/* Read-only (view-only firmware params) get their own muted bucket at
                  the bottom of the Sync-UAVs sidebar — entering it is the explicit
                  choice to inspect them; they're excluded from every other view. */}
              {harmonizeMode && readOnlyCount > 0 && (
                <PrefixItem
                  label={t('settings.parameters.grp_read_only', 'Read-only')}
                  count={readOnlyCount}
                  active={selectedPrefix === '(readonly)'}
                  muted
                  onClick={() => setSelectedPrefix(selectedPrefix === '(readonly)' ? '(all)' : '(readonly)')}
                />
              )}
            </React.Fragment>
          )}
        </div>
        <VirtualParamGrid
          rowNames={visibleNames}
          vehicles={vehicleList || []}
          snapshotsByVehicle={snapshotsByVehicle}
          staleByVehicle={staleByVehicle}
          draftsByVehicle={draftsByVehicle}
          setVehicleDraft={fullParams?.setVehicleDraft}
          cellErrorsByVehicle={cellErrorsByVehicle}
          editable={!paramsRefreshing && !compareMode && !harmonizeMode}
          mode={harmonizeMode ? 'harmonize' : compareMode ? 'compare' : 'normal'}
          compareRowsByName={compareRowsByName}
          selectable={compareMode || harmonizeMode}
          selectedNames={harmonizeMode ? harmonizeSelected : selectedNames}
          onToggleRow={harmonizeMode ? toggleHarmonizeRow : toggleCompareRow}
          onToggleAll={harmonizeMode ? onToggleAllHarmonize : onToggleAllCompare}
          masterChecked={harmonizeMode ? hzMasterChecked : masterChecked}
          masterIndeterminate={harmonizeMode ? hzMasterIndeterminate : masterIndeterminate}
          compareCellErrorsByVehicle={harmonizeMode ? harmonizeCellErrorsByVehicle : compareCellErrorsByVehicle}
          harmonizeRowsByName={harmonizeRowsByName}
          winnersByName={harmonizeWinners}
          onPickWinner={onPickWinner}
          progressByVehicle={progressByVehicle}
          writeProgressByVehicle={fullParams?.writeProgressByVehicle || {}}
          changedByVehicle={changedByVehicle}
          onRetryVehicle={onRetryVehicle}
        />
      </div>
    </div>
  );
}

function PrefixItem({
  label,
  count,
  active,
  onClick,
  depth = 0,
  hasChildren = false,
  expanded = false,
  muted = false,
}) {
  return (
    <button
      onClick={onClick}
      style={{
        display: 'flex',
        justifyContent: 'space-between',
        width: '100%',
        padding: `4px 12px 4px ${12 + depth * 16}px`,
        background: active ? colors.accent : 'transparent',
        color: active ? colors.bg : (muted ? colors.textDim : colors.text),
        border: 'none',
        cursor: 'pointer',
        fontFamily: 'monospace',
        fontSize: 12,
        textAlign: 'left',
      }}
    >
      <span style={{
        display: 'flex',
        alignItems: 'center',
        gap: 4,
        overflow: 'hidden',
        textOverflow: 'ellipsis',
        whiteSpace: 'nowrap',
      }}>
        {hasChildren && (
          <span style={{
            color: active ? colors.bg : colors.textDim,
            width: 10,
            flex: '0 0 auto',
          }}>
            {expanded ? 'v' : '>'}
          </span>
        )}
        {label}
      </span>
      <span style={{ color: active ? colors.bg : colors.textDim }}>{count}</span>
    </button>
  );
}
