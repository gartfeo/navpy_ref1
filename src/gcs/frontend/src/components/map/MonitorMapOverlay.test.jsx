import { describe, it, expect, vi } from 'vitest';
import { render, fireEvent } from '@testing-library/react';
import MonitorMapOverlay from './MonitorMapOverlay';

const props = (extra = {}) => ({
  vehicleList: [{ sys_id: 1 }, { sys_id: 2 }],
  followSysId: null,
  onFollowChange: vi.fn(),
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
  it('the follow selector is as tall as the map buttons and moves only the map; no FPV button', () => {
    const p = props();
    const { getByRole, queryByRole } = render(<MonitorMapOverlay {...p} />);
    const select = getByRole('combobox', { name: 'monitorOverlay.follow' });
    expect(select.style.height).toBe('32px');
    fireEvent.change(select, { target: { value: '2' } });
    expect(p.onFollowChange).toHaveBeenCalledWith(2);
    fireEvent.change(select, { target: { value: '' } });
    expect(p.onFollowChange).toHaveBeenLastCalledWith(null);
    expect(queryByRole('button', { name: /FPV/ })).toBeNull();
    expect(p.onToggleManualControl).not.toHaveBeenCalled();
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
