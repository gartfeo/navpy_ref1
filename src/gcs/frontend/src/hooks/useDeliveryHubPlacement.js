import { useCallback, useEffect, useState } from 'react';
import { useTranslation } from 'react-i18next';

/**
 * Manages delivery hub placement, removal, and drag on the map.
 *
 * @param {object} params
 * @param {object} params.settings - Current settings (contains default_delivery_hubs)
 * @param {Function} params.handleSaveSettings - Save settings callback
 * @param {Function} params.setDeliveryHubAssignments - Update delivery hub assignments
 * @param {string} params.phase - Current app phase
 */
export default function useDeliveryHubPlacement({ settings, handleSaveSettings, setDeliveryHubAssignments, phase }) {
  const { t } = useTranslation();
  const [placingDeliveryHub, setPlacingDeliveryHub] = useState(false);
  const [placingDeliveryHubType, setPlacingDeliveryHubType] = useState('other');

  // Cancel placement when leaving planning phase
  useEffect(() => {
    if (phase !== 'PLANNING') setPlacingDeliveryHub(false);
  }, [phase]);

  const handlePlaceDeliveryHub = useCallback((latlon) => {
    const deliveryHubs = settings?.default_delivery_hubs || [];
    const label = t('deliveryHub.types.' + placingDeliveryHubType);
    const sameTypeCount = deliveryHubs.filter((o) => o.type === placingDeliveryHubType).length;
    const newDeliveryHub = {
      name: `${label}${sameTypeCount + 1}`,
      type: placingDeliveryHubType,
      lat: latlon.lat,
      lon: latlon.lon,
    };
    handleSaveSettings({ default_delivery_hubs: [...deliveryHubs, newDeliveryHub] });
  }, [settings, placingDeliveryHubType, handleSaveSettings, t]);

  const handleRemoveDeliveryHub = useCallback((deliveryHubIndex) => {
    const deliveryHubs = settings?.default_delivery_hubs || [];
    if (deliveryHubIndex < 0 || deliveryHubIndex >= deliveryHubs.length) return;
    const newDeliveryHubs = deliveryHubs.filter((_, i) => i !== deliveryHubIndex);
    handleSaveSettings({ default_delivery_hubs: newDeliveryHubs });
    setDeliveryHubAssignments((prev) => {
      if (!prev) return prev;
      return prev.map((oi) => {
        if (oi === null) return null;
        if (oi === deliveryHubIndex) return null;
        return oi > deliveryHubIndex ? oi - 1 : oi;
      });
    });
  }, [settings, handleSaveSettings, setDeliveryHubAssignments]);

  const handleMoveDeliveryHub = useCallback((deliveryHubIndex, latlon) => {
    const deliveryHubs = settings?.default_delivery_hubs || [];
    if (deliveryHubIndex < 0 || deliveryHubIndex >= deliveryHubs.length) return;
    const newDeliveryHubs = deliveryHubs.map((o, i) =>
      i === deliveryHubIndex ? { ...o, lat: latlon.lat, lon: latlon.lon } : o
    );
    handleSaveSettings({ default_delivery_hubs: newDeliveryHubs });
  }, [settings, handleSaveSettings]);

  const startPlacing = useCallback(() => {
    setPlacingDeliveryHub(true);
  }, []);

  const stopPlacing = useCallback(() => {
    setPlacingDeliveryHub(false);
  }, []);

  return {
    placingDeliveryHub,
    setPlacingDeliveryHub,
    placingDeliveryHubType,
    setPlacingDeliveryHubType,
    handlePlaceDeliveryHub,
    handleRemoveDeliveryHub,
    handleMoveDeliveryHub,
    startPlacing,
    stopPlacing,
  };
}
