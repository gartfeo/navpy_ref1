import { useCallback, useEffect, useState } from 'react';
import { loadCesiumIonToken } from '../../../utils/cesiumConfig';
import {
  DEFAULT_BASE_LAYER, isBaseLayerAvailable, layerAfterFailure, loadBaseLayer, saveBaseLayer,
} from '../utils/baseLayers';

/**
 * The operator's base map layer: restored per browser, saved on change, and
 * checked against the Ion token once the backend reports it.
 */
export default function useBaseLayerChoice(loadIonToken = loadCesiumIonToken) {
  const [hasIon, setHasIon] = useState(false);
  // The saved choice; an Ion layer is dropped below if no token turns up.
  const [baseLayer, setBaseLayer] = useState(() => loadBaseLayer(true));

  useEffect(() => {
    let live = true;
    loadIonToken().then((token) => {
      if (!live) return;
      const ion = token != null;
      setHasIon(ion);
      setBaseLayer((cur) => (isBaseLayerAvailable(cur, ion) ? cur : DEFAULT_BASE_LAYER));
    });
    return () => { live = false; };
  }, [loadIonToken]);

  const onChange = useCallback((id) => { setBaseLayer(id); saveBaseLayer(id); }, []);
  // A layer that fails to load (e.g. Ion refused) falls back to the default for
  // this session; the saved choice stays, so it is retried on the next load.
  const onFailed = useCallback((id, err) => {
    console.warn(`Base map layer "${id}" failed to load:`, err);
    setBaseLayer((cur) => layerAfterFailure(cur, id));
  }, []);

  return { baseLayer, hasIon, onChange, onFailed };
}
