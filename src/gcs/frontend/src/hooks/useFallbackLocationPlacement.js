import { useCallback, useEffect, useState } from 'react';
import { useTranslation } from 'react-i18next';

/**
 * Manages fallback location placement, removal, and drag on the map.
 *
 * @param {object} params
 * @param {object} params.settings - Current settings (contains fallback_delivery_locations)
 * @param {Function} params.handleSaveSettings - Save settings callback
 * @param {Function} params.setFallbackLocationAssignments - Update fallback location assignments
 * @param {string} params.phase - Current app phase
 */
export default function useFallbackLocationPlacement({ settings, handleSaveSettings, setFallbackLocationAssignments, phase }) {
  const { t } = useTranslation();
  const [placingFallbackLocation, setPlacingFallbackLocation] = useState(false);
  const [placingFallbackLocationType, setPlacingFallbackLocationType] = useState('other');

  // Cancel placement when leaving planning phase
  useEffect(() => {
    if (phase !== 'PLANNING') setPlacingFallbackLocation(false);
  }, [phase]);

  const handlePlaceFallbackLocation = useCallback((latlon) => {
    const fallbackLocations = settings?.fallback_delivery_locations || [];
    const label = t('fallbackLocation.types.' + placingFallbackLocationType);
    const sameTypeCount = fallbackLocations.filter((o) => o.type === placingFallbackLocationType).length;
    const newFallbackLocation = {
      name: `${label}${sameTypeCount + 1}`,
      type: placingFallbackLocationType,
      lat: latlon.lat,
      lon: latlon.lon,
    };
    handleSaveSettings({ fallback_delivery_locations: [...fallbackLocations, newFallbackLocation] });
  }, [settings, placingFallbackLocationType, handleSaveSettings, t]);

  const handleRemoveFallbackLocation = useCallback((fallbackLocationIndex) => {
    const fallbackLocations = settings?.fallback_delivery_locations || [];
    if (fallbackLocationIndex < 0 || fallbackLocationIndex >= fallbackLocations.length) return;
    const newFallbackLocations = fallbackLocations.filter((_, i) => i !== fallbackLocationIndex);
    handleSaveSettings({ fallback_delivery_locations: newFallbackLocations });
    setFallbackLocationAssignments((prev) => {
      if (!prev) return prev;
      return prev.map((oi) => {
        if (oi === null) return null;
        if (oi === fallbackLocationIndex) return null;
        return oi > fallbackLocationIndex ? oi - 1 : oi;
      });
    });
  }, [settings, handleSaveSettings, setFallbackLocationAssignments]);

  const handleMoveFallbackLocation = useCallback((fallbackLocationIndex, latlon) => {
    const fallbackLocations = settings?.fallback_delivery_locations || [];
    if (fallbackLocationIndex < 0 || fallbackLocationIndex >= fallbackLocations.length) return;
    const newFallbackLocations = fallbackLocations.map((o, i) =>
      i === fallbackLocationIndex ? { ...o, lat: latlon.lat, lon: latlon.lon } : o
    );
    handleSaveSettings({ fallback_delivery_locations: newFallbackLocations });
  }, [settings, handleSaveSettings]);

  const startPlacing = useCallback(() => {
    setPlacingFallbackLocation(true);
  }, []);

  const stopPlacing = useCallback(() => {
    setPlacingFallbackLocation(false);
  }, []);

  return {
    placingFallbackLocation,
    setPlacingFallbackLocation,
    placingFallbackLocationType,
    setPlacingFallbackLocationType,
    handlePlaceFallbackLocation,
    handleRemoveFallbackLocation,
    handleMoveFallbackLocation,
    startPlacing,
    stopPlacing,
  };
}
