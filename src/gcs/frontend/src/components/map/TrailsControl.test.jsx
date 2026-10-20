import { describe, it, expect, vi } from 'vitest';
import { act, render, fireEvent } from '@testing-library/react';
import TrailsControl, { panelBounds } from './TrailsControl';
import en from '../../locales/en.json';
import hy from '../../locales/hy.json';

const props = (extra = {}) => ({
  vehicleList: [{ sys_id: 1 }, { sys_id: 2, name: 'Bravo' }],
  showTrails: true, onToggleTrails: vi.fn(),
  showMarks: false, onToggleMarks: vi.fn(),
  hiddenSysIds: new Set([2]),
  onToggleUav: vi.fn(),
  pausedAt: null, onTogglePause: vi.fn(),
  onClearAll: vi.fn(),
  ...extra,
});

const open = (utils) => fireEvent.click(utils.getByRole('button', { name: 'trailsPanel.title' }));

describe('TrailsControl', () => {
  it('opens a panel of map-bar toggles: layers, one per UAV, pause and clear', () => {
    const p = props();
    const utils = render(<TrailsControl {...p} />);
    expect(utils.queryByTestId('trails-panel')).toBeNull();
    open(utils);
    expect(utils.getByRole('button', { name: 'trailsPanel.title' }).getAttribute('aria-expanded')).toBe('true');
    const pressed = (name) => utils.getByRole('button', { name }).getAttribute('aria-pressed');
    expect(pressed('trailsPanel.flightTrail')).toBe('true');
    expect(pressed('trailsPanel.cameraMarks')).toBe('false');
    expect(pressed('UAV 1')).toBe('true');
    expect(pressed('Bravo')).toBe('false');
    expect(utils.queryByRole('checkbox')).toBeNull();
    expect(utils.getByText('trailsPanel.recording')).toBeInTheDocument();

    fireEvent.click(utils.getByRole('button', { name: 'trailsPanel.flightTrail' }));
    fireEvent.click(utils.getByRole('button', { name: 'trailsPanel.cameraMarks' }));
    fireEvent.click(utils.getByRole('button', { name: 'Bravo' }));
    fireEvent.click(utils.getByRole('button', { name: 'trailsPanel.pause' }));
    fireEvent.click(utils.getByRole('button', { name: 'trailsPanel.clearAll' }));
    expect(p.onToggleTrails).toHaveBeenCalledTimes(1);
    expect(p.onToggleMarks).toHaveBeenCalledTimes(1);
    expect(p.onToggleUav).toHaveBeenCalledWith(2);
    expect(p.onTogglePause).toHaveBeenCalledTimes(1);
    expect(p.onClearAll).toHaveBeenCalledTimes(1);
  });

  it('shows the Paused badge on the button and Resume plus the time in the panel', () => {
    const utils = render(<TrailsControl {...props({ pausedAt: Date.now() })} />);
    expect(utils.getByTestId('trails-paused-badge').textContent).toBe('trailsPanel.paused');
    open(utils);
    expect(utils.getByRole('button', { name: 'trailsPanel.resume' })).toBeInTheDocument();
    expect(utils.getByText('trailsPanel.pausedSince')).toBeInTheDocument();
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

  it('panel controls are compact 28 px map-bar toggles; the bar button keeps 32 px', () => {
    const utils = render(<TrailsControl {...props()} />);
    expect(utils.getByRole('button', { name: 'trailsPanel.title' }).style.height).toBe('32px');
    open(utils);
    expect(utils.getByRole('button', { name: 'trailsPanel.clearAll' }).style.height).toBe('28px');
    expect(utils.getByRole('button', { name: 'UAV 1' }).style.height).toBe('28px');
  });

  it('every panel string exists in English and Armenian', () => {
    const keys = ['button', 'title', 'flightTrail', 'cameraMarks',
      'pause', 'resume', 'clearAll', 'paused', 'recording', 'pausedSince'];
    for (const k of keys) {
      expect(en.trailsPanel[k], `en ${k}`).toBeTruthy();
      expect(hy.trailsPanel[k], `hy ${k}`).toBeTruthy();
    }
    expect(hy.trailsPanel.pausedSince).toContain('{{time}}');
  });

  it('keeps the panel inside the map pane, not the browser window', () => {
    // Narrow pane: 188 px from the pane margin to the button's right edge.
    expect(panelBounds({ right: 300, bottom: 44 }, { left: 100, bottom: 200 }))
      .toEqual({ width: 188, maxHeight: 138 });
    // Wide pane: the panel keeps its 196 px width.
    expect(panelBounds({ right: 1500, bottom: 44 }, { left: 0, bottom: 900 }).width).toBe(196);
  });

  it('re-measures when the map pane resizes without a window resize', () => {
    let onResize;
    const Orig = globalThis.ResizeObserver;
    globalThis.ResizeObserver = class { constructor(cb) { onResize = cb; } observe() {} disconnect() {} };
    try {
      const utils = render(
        <div data-testid="pane" style={{ position: 'relative' }}><div><TrailsControl {...props()} /></div></div>,
      );
      const pane = utils.getByTestId('pane');
      Object.defineProperty(pane, 'offsetParent', { value: null });
      const bar = utils.getByRole('button', { name: 'trailsPanel.title' }).parentElement.parentElement;
      Object.defineProperty(bar, 'offsetParent', { value: pane });
      pane.getBoundingClientRect = () => ({ left: 0, bottom: 900 });
      utils.getByRole('button', { name: 'trailsPanel.title' }).getBoundingClientRect = () => ({ right: 1500, bottom: 44 });
      open(utils);
      expect(utils.getByTestId('trails-panel').style.width).toBe('196px');
      pane.getBoundingClientRect = () => ({ left: 1340, bottom: 900 });
      act(() => onResize());
      expect(utils.getByTestId('trails-panel').style.width).toBe('148px');
    } finally {
      globalThis.ResizeObserver = Orig;
    }
  });
});
