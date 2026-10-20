import { describe, it, expect, vi } from 'vitest';
import { render, fireEvent } from '@testing-library/react';
import TrailsControl, { panelBounds } from './TrailsControl';
import en from '../../locales/en.json';
import hy from '../../locales/hy.json';

const props = (extra = {}) => ({
  vehicleList: [{ sys_id: 1 }, { sys_id: 2, name: 'Bravo' }],
  showTrails: true, onToggleTrails: vi.fn(),
  showMarks: false, onToggleMarks: vi.fn(),
  hiddenSysIds: new Set([2]),
  onToggleUav: vi.fn(), onShowAllUavs: vi.fn(), onHideAllUavs: vi.fn(),
  pausedAt: null, onTogglePause: vi.fn(),
  onClearAll: vi.fn(),
  ...extra,
});

const open = (utils) => fireEvent.click(utils.getByRole('button', { name: 'trailsPanel.title' }));

describe('TrailsControl', () => {
  it('opens a panel with layers, one checkbox per UAV and the history actions', () => {
    const p = props();
    const utils = render(<TrailsControl {...p} />);
    expect(utils.queryByTestId('trails-panel')).toBeNull();
    open(utils);
    expect(utils.getByRole('button', { name: 'trailsPanel.title' }).getAttribute('aria-expanded')).toBe('true');
    expect(utils.getByLabelText('trailsPanel.flightTrail').checked).toBe(true);
    expect(utils.getByLabelText('trailsPanel.cameraMarks').checked).toBe(false);
    expect(utils.getByLabelText('UAV 1').checked).toBe(true);
    expect(utils.getByLabelText('Bravo').checked).toBe(false);

    fireEvent.click(utils.getByLabelText('trailsPanel.flightTrail'));
    fireEvent.click(utils.getByLabelText('trailsPanel.cameraMarks'));
    fireEvent.click(utils.getByLabelText('Bravo'));
    fireEvent.click(utils.getByRole('button', { name: 'trailsPanel.all' }));
    fireEvent.click(utils.getByRole('button', { name: 'trailsPanel.none' }));
    fireEvent.click(utils.getByRole('button', { name: 'trailsPanel.pause' }));
    fireEvent.click(utils.getByRole('button', { name: 'trailsPanel.clearAll' }));
    expect(p.onToggleTrails).toHaveBeenCalledTimes(1);
    expect(p.onToggleMarks).toHaveBeenCalledTimes(1);
    expect(p.onToggleUav).toHaveBeenCalledWith(2);
    expect(p.onShowAllUavs).toHaveBeenCalledTimes(1);
    expect(p.onHideAllUavs).toHaveBeenCalledTimes(1);
    expect(p.onTogglePause).toHaveBeenCalledTimes(1);
    expect(p.onClearAll).toHaveBeenCalledTimes(1);
  });

  it('shows the Paused badge on the button and Resume plus the time in the panel', () => {
    const utils = render(<TrailsControl {...props({ pausedAt: Date.now() })} />);
    expect(utils.getByTestId('trails-paused-badge').textContent).toBe('trailsPanel.paused');
    open(utils);
    expect(utils.getByRole('button', { name: 'trailsPanel.resume' })).toBeInTheDocument();
    expect(utils.getByText('trailsPanel.notRecordingSince')).toBeInTheDocument();
  });

  it('closes on Escape with focus back on the button, and on a tap outside', () => {
    const utils = render(<div><span data-testid="outside" /><TrailsControl {...props()} /></div>);
    open(utils);
    fireEvent.keyDown(document, { key: 'Escape' });
    expect(utils.queryByTestId('trails-panel')).toBeNull();
    expect(document.activeElement).toBe(utils.getByRole('button', { name: 'trailsPanel.title' }));
    open(utils);
    fireEvent.pointerDown(utils.getByTestId('outside'));
    expect(utils.queryByTestId('trails-panel')).toBeNull();
  });

  it('panel controls are 44 px touch targets; the bar button keeps 32 px', () => {
    const utils = render(<TrailsControl {...props()} />);
    expect(utils.getByRole('button', { name: 'trailsPanel.title' }).style.height).toBe('32px');
    open(utils);
    expect(utils.getByRole('button', { name: 'trailsPanel.clearAll' }).style.minHeight).toBe('44px');
    expect(utils.getByLabelText('UAV 1').closest('label').style.minHeight).toBe('44px');
  });

  it('every panel string exists in English and Armenian', () => {
    const keys = ['button', 'title', 'flightTrail', 'cameraMarks', 'uavs', 'all', 'none',
      'pause', 'resume', 'clearAll', 'paused', 'notRecordingSince'];
    for (const k of keys) {
      expect(en.trailsPanel[k], `en ${k}`).toBeTruthy();
      expect(hy.trailsPanel[k], `hy ${k}`).toBeTruthy();
    }
    expect(hy.trailsPanel.notRecordingSince).toContain('{{time}}');
  });

  it('keeps the panel inside the map pane, not the browser window', () => {
    // Narrow pane: 188 px from the pane margin to the button's right edge.
    expect(panelBounds({ right: 300, bottom: 44 }, { left: 100, bottom: 200 }))
      .toEqual({ width: 188, maxHeight: 138 });
    // Wide pane: the panel keeps its 320 px width.
    expect(panelBounds({ right: 1500, bottom: 44 }, { left: 0, bottom: 900 }).width).toBe(320);
  });
});
