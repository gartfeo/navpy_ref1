import { describe, it, expect, vi } from 'vitest';
import { render, fireEvent } from '@testing-library/react';
import MonitorMapOverlay from './MonitorMapOverlay';

const props = (extra = {}) => ({
  onToggleManualControl: vi.fn(),
  onToggleVision: vi.fn(),
  onToggleCoverage: vi.fn(),
  onClearCoverage: vi.fn(),
  onToggleTrails: vi.fn(),
  onClearTrails: vi.fn(),
  onEditPlan: vi.fn(),
  planEntry: { show: true, isEdit: false },
  ...extra,
});

describe('MonitorMapOverlay top-right bar', () => {
  it('no follow selector and no FPV button: the map never chases a UAV', () => {
    const p = props();
    const { queryByRole, getByTestId } = render(<MonitorMapOverlay {...p} />);
    expect(queryByRole('combobox')).toBeNull();
    expect(getByTestId('monitor-top-right').querySelector('select')).toBeNull();
    expect(queryByRole('button', { name: /FPV/ })).toBeNull();
    expect(p.onToggleManualControl).not.toHaveBeenCalled();
  });

  it('the bar buttons still reach their handlers', () => {
    const p = props();
    const { getByRole } = render(<MonitorMapOverlay {...p} />);
    fireEvent.click(getByRole('button', { name: 'monitorOverlay.toggleVision' }));
    expect(p.onToggleVision).toHaveBeenCalledTimes(1);
  });

  it('wraps the top-right controls instead of hiding the plan button', () => {
    const { getByTestId, getByRole } = render(<MonitorMapOverlay {...props()} />);
    const bar = getByTestId('monitor-top-right');
    expect(bar.style.flexWrap).toBe('wrap');
    expect(bar.style.maxWidth).toBe('calc(100% - 68px)');
    expect(getByRole('button', { name: 'monitor.startPlanning' })).toBeInTheDocument();
  });

  it('reports the bottom edge of the top controls', () => {
    const onTopControlsBottom = vi.fn();
    render(<MonitorMapOverlay {...props({ onTopControlsBottom })} />);
    expect(onTopControlsBottom).toHaveBeenCalledWith(44);
  });
});
