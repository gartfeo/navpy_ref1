import { describe, it, expect, vi, afterEach } from 'vitest';
import { render, fireEvent, act } from '@testing-library/react';
import MonitorMapOverlay from './MonitorMapOverlay';

const props = (extra = {}) => ({
  onToggleManualControl: vi.fn(),
  onToggleVision: vi.fn(),
  trails: { vehicleList: [{ sys_id: 1 }], hiddenSysIds: new Set(), pausedAt: null },
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

  it('one Trails button replaces the four trail and coverage show/clear buttons', () => {
    const { getByTestId, getByRole } = render(<MonitorMapOverlay {...props()} />);
    const bar = getByTestId('monitor-top-right');
    expect(getByRole('button', { name: 'trailsPanel.title' })).toBeInTheDocument();
    for (const old of ['toggleCoverage', 'clearCoverage', 'toggleTrails', 'clearTrails']) {
      expect(bar.querySelector(`[title="monitorOverlay.${old}"]`)).toBeNull();
    }
  });

  it('the video icon: last in the bar, a stable name, the tooltip and pressed state follow the window', () => {
    const p = props({ videoOpen: false, onToggleVideo: vi.fn() });
    const { getByRole, getByTestId, rerender } = render(<MonitorMapOverlay {...p} />);
    const video = getByRole('button', { name: 'monitorOverlay.video' });
    const buttons = getByTestId('monitor-top-right').querySelectorAll('button');
    expect(buttons[buttons.length - 1]).toBe(video);
    expect(video).toHaveAttribute('aria-pressed', 'false');
    expect(video).toHaveAttribute('title', 'monitorOverlay.showVideo');
    fireEvent.click(video);
    expect(p.onToggleVideo).toHaveBeenCalledTimes(1);
    rerender(<MonitorMapOverlay {...p} videoOpen />);
    expect(getByRole('button', { name: 'monitorOverlay.video' })).toHaveAttribute('aria-pressed', 'true');
    expect(video).toHaveAttribute('title', 'monitorOverlay.hideVideo');
    expect(video.style.width).toBe('32px');
  });

  describe('a wrapped bar', () => {
    afterEach(() => {
      vi.restoreAllMocks();
      delete globalThis.ResizeObserver;
    });

    it('reports the new bottom when the bar wraps to a second row, so the window moves down', () => {
      let resize;
      globalThis.ResizeObserver = class { constructor(cb) { resize = cb; } observe() {} disconnect() {} };
      let height = 32;
      vi.spyOn(HTMLElement.prototype, 'clientWidth', 'get').mockReturnValue(300);
      vi.spyOn(HTMLElement.prototype, 'clientHeight', 'get').mockImplementation(() => height);
      const onTopControlsBottom = vi.fn();
      render(<MonitorMapOverlay {...props({ onTopControlsBottom })} />);
      expect(onTopControlsBottom).toHaveBeenLastCalledWith(44);
      height = 68;
      act(() => resize());
      expect(onTopControlsBottom).toHaveBeenLastCalledWith(80);
    });
  });
});
