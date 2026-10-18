import { describe, it, expect, vi } from 'vitest';
import { render, fireEvent } from '@testing-library/react';
import MonitorMapOverlay, { FPV_LABEL } from './MonitorMapOverlay';

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
  fpvOn: false,
  onToggleFpv: vi.fn(),
  ...extra,
});

describe('MonitorMapOverlay FPV toggle', () => {
  it('is a text-labelled 44 px toggle beside the follow selector', () => {
    const p = props();
    const { getByRole } = render(<MonitorMapOverlay {...p} />);
    const toggle = getByRole('button', { name: FPV_LABEL });
    const select = getByRole('combobox', { name: 'monitorOverlay.follow' });
    expect(toggle).toHaveAttribute('aria-pressed', 'false');
    expect(toggle.style.height).toBe('44px');
    expect(select.style.height).toBe('44px');
    expect(select.nextElementSibling).toBe(toggle);
    fireEvent.click(toggle);
    expect(p.onToggleFpv).toHaveBeenCalledTimes(1);
    // FPV is a view toggle: it never touches RC.
    expect(p.onToggleManualControl).not.toHaveBeenCalled();
  });

  it('shows its on state plainly: filled and labelled "FPV · ON"', () => {
    const { getByRole } = render(<MonitorMapOverlay {...props({ fpvOn: true })} />);
    const toggle = getByRole('button', { pressed: true });
    expect(toggle).toHaveTextContent(`${FPV_LABEL} · fpvView.on`);
    expect(toggle.style.background).not.toContain('rgba');
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
    expect(onTopControlsBottom).toHaveBeenCalledWith(56);
  });
});
