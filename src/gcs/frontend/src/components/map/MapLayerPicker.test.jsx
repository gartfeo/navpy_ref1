/**
 * Base map layer switch: the layers button opens one choice per layer, the
 * active one is marked, and satellite is disabled without an Ion token.
 */
import { render, fireEvent } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';
import MapLayerPicker from './MapLayerPicker';

const open = (props) => {
  const utils = render(<MapLayerPicker value="terrain" onChange={vi.fn()} hasIon {...props} />);
  fireEvent.click(utils.getByTitle('mapLayers.title'));
  return utils;
};

describe('MapLayerPicker', () => {
  it('starts closed and opens on the layers button', () => {
    const { queryByRole, getByTitle } = render(
      <MapLayerPicker value="terrain" onChange={vi.fn()} hasIon />,
    );
    expect(queryByRole('group')).toBeNull();
    fireEvent.click(getByTitle('mapLayers.title'));
    expect(queryByRole('group')).not.toBeNull();
  });

  it('marks the active layer', () => {
    const { getByText } = open({ value: 'street' });
    expect(getByText('mapLayers.street')).toHaveAttribute('aria-pressed', 'true');
    expect(getByText('mapLayers.terrain')).toHaveAttribute('aria-pressed', 'false');
  });

  it('reports the picked layer and closes', () => {
    const onChange = vi.fn();
    const { getByText, queryByRole } = open({ onChange });
    fireEvent.click(getByText('mapLayers.satellite'));
    expect(onChange).toHaveBeenCalledWith('satellite');
    expect(queryByRole('group')).toBeNull();
  });

  it('disables satellite without an Ion token', () => {
    const onChange = vi.fn();
    const { getByText } = open({ onChange, hasIon: false });
    const sat = getByText('mapLayers.satellite');
    expect(sat).toBeDisabled();
    expect(sat).toHaveAttribute('title', 'mapLayers.needsIon');
    fireEvent.click(sat);
    expect(onChange).not.toHaveBeenCalled();
    expect(getByText('mapLayers.terrain')).toBeEnabled();
    expect(getByText('mapLayers.street')).toBeEnabled();
  });
});
