import { useCallback, useEffect, useRef, useState } from 'react';
import { configurePlanner } from '../utils/planner';
import { resolvePlannerState } from './settingsPlannerState';

/**
 * Manages settings state: fetch on mount, save, reset, and auto-reopen after vehicle connect.
 */
export default function useSettings(vehicleList) {
  const [showSettings, setShowSettings] = useState(false);
  const [settingsTab, setSettingsTab] = useState(null);
  const reopenSettingsRef = useRef(false);
  const [settings, setSettings] = useState(null);
  const catalogRef = useRef(null);
  const [settingsVersion, setSettingsVersion] = useState(0);
  const [plannerReady, setPlannerReady] = useState(false);
  const [plannerLoadError, setPlannerLoadError] = useState(null);

  const applyPlannerState = useCallback((nextSettings, nextCatalog) => {
    const resolved = resolvePlannerState(nextSettings, nextCatalog);
    setPlannerReady(resolved.plannerReady);
    setPlannerLoadError(resolved.plannerLoadError);
    if (!resolved.plannerReady) return;

    configurePlanner(nextSettings, resolved.plannerProfiles, resolved.plannerDetectorClassDimensions);
    setSettingsVersion((v) => v + 1);
  }, []);

  useEffect(() => {
    let cancelled = false;
    setPlannerReady(false);
    setPlannerLoadError(null);

    Promise.all([
      fetch('/api/settings').then((r) => r.ok ? r.json() : null),
      fetch('/api/vision-profiles').then((r) => r.ok ? r.json() : null).catch(() => null),
    ]).then(([data, catalog]) => {
      if (cancelled) return;
      if (catalog) catalogRef.current = catalog;
      if (data) setSettings(data);
      applyPlannerState(data, catalog);
    }).catch(() => {
      if (cancelled) return;
      setPlannerReady(false);
      setPlannerLoadError('Failed to load planner configuration.');
    });

    return () => {
      cancelled = true;
    };
  }, [applyPlannerState]);

  // Reopen settings modal after connecting from the UAV tab
  useEffect(() => {
    if (reopenSettingsRef.current && vehicleList.length > 0) {
      reopenSettingsRef.current = false;
      setSettingsTab('UAV');
      setShowSettings(true);
    }
  }, [vehicleList]);

  const handleSaveSettings = useCallback(async (draft) => {
    // Optimistic: merge immediately so UI reflects changes before round-trip
    setSettings((prev) => ({ ...prev, ...draft }));
    try {
      const res = await fetch('/api/settings', {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(draft),
      });
      if (res.ok) {
        const updated = await res.json();
        setSettings(updated);
        let latestCatalog = null;
        // Re-fetch catalog in case profile edits were saved just before this call.
        try {
          const catRes = await fetch('/api/vision-profiles');
          if (catRes.ok) latestCatalog = await catRes.json();
        } catch { /* ignore */ }
        catalogRef.current = latestCatalog;
        applyPlannerState(updated, latestCatalog);
      }
    } catch { /* ignore */ }
  }, [applyPlannerState]);

  const handleResetSettings = useCallback(async () => {
    try {
      const res = await fetch('/api/settings');
      if (res.ok) {
        const saved = await res.json();
        setSettings(saved);
        applyPlannerState(saved, catalogRef.current);
      }
    } catch { /* ignore */ }
  }, [applyPlannerState]);

  return {
    settings,
    showSettings, setShowSettings,
    settingsTab, setSettingsTab,
    reopenSettingsRef,
    plannerReady,
    plannerLoadError,
    settingsVersion,
    handleSaveSettings,
    handleResetSettings,
  };
}
