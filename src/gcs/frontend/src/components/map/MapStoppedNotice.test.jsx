// The map-stopped notice and its fit rule (docs/design/gcs_map_stopped_notice.md).
// jsdom has no layout, so the card's measured sizes and the ResizeObserver are stubbed.
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { render, screen, fireEvent, act } from '@testing-library/react';
import MapStoppedNotice from './MapStoppedNotice';

const FITS = { scrollWidth: 300, clientWidth: 300, scrollHeight: 120, clientHeight: 120 };
let box;
let observers;

class FakeResizeObserver {
  constructor(callback) {
    this.callback = callback;
    this.targets = [];
    this.disconnected = false;
    observers.push(this);
  }

  observe(el) { this.targets.push(el); }

  disconnect() { this.disconnected = true; }
}

// The card is the only element measured; every other element reads 0.
function stubSize(name) {
  vi.spyOn(Element.prototype, name, 'get').mockImplementation(function size() {
    return this.dataset?.testid === 'map-stopped-card' ? box[name] : 0;
  });
}

function resize(next) {
  box = { ...box, ...next };
  act(() => { for (const o of observers) o.callback([]); });
}

const reloadButton = () => screen.queryByRole('button', { name: /mapStopped\.reload/ });

describe('MapStoppedNotice', () => {
  beforeEach(() => {
    box = { ...FITS };
    observers = [];
    vi.stubGlobal('ResizeObserver', FakeResizeObserver);
    for (const name of ['scrollWidth', 'clientWidth', 'scrollHeight', 'clientHeight']) stubSize(name);
  });

  afterEach(() => {
    vi.unstubAllGlobals();
  });

  it('shows the whole card as an alert when it fits, and Reload reloads', () => {
    const onReload = vi.fn();
    render(<MapStoppedNotice onReload={onReload} />);
    const alert = screen.getByRole('alert');
    expect(alert).toHaveTextContent('mapStopped.title');
    expect(alert).toHaveTextContent('mapStopped.line');
    fireEvent.click(reloadButton());
    expect(onReload).toHaveBeenCalledTimes(1);
  });

  it('lets clicks through the layer; only Reload takes them', () => {
    render(<MapStoppedNotice onReload={() => {}} />);
    expect(screen.getByRole('alert')).toHaveStyle({ pointerEvents: 'none' });
    expect(reloadButton()).toHaveStyle({ pointerEvents: 'auto' });
  });

  it('shows the title only, with no Reload, when the card is too tall', () => {
    box = { ...FITS, scrollHeight: 121 };
    render(<MapStoppedNotice onReload={() => {}} />);
    expect(screen.getByRole('alert')).toHaveTextContent('mapStopped.title');
    expect(reloadButton()).toBeNull();
    expect(screen.getByTestId('map-stopped-card')).toHaveStyle({ visibility: 'hidden' });
  });

  it('shows the title only, with no Reload, when the card is too wide', () => {
    box = { ...FITS, scrollWidth: 301 };
    render(<MapStoppedNotice onReload={() => {}} />);
    expect(reloadButton()).toBeNull();
  });

  it('shows the card again when the box grows back, and hides it when it shrinks', () => {
    box = { ...FITS, scrollHeight: 200 };
    render(<MapStoppedNotice onReload={() => {}} />);
    expect(reloadButton()).toBeNull();
    resize({ clientHeight: 200 });
    expect(reloadButton()).not.toBeNull();
    expect(screen.getByTestId('map-stopped-card')).toHaveStyle({ visibility: 'visible' });
    resize({ clientHeight: 150 });
    expect(reloadButton()).toBeNull();
  });

  it('watches the box and the card content, and stops watching on unmount', () => {
    const { unmount } = render(<MapStoppedNotice onReload={() => {}} />);
    expect(observers).toHaveLength(1);
    const [observer] = observers;
    expect(observer.targets).toEqual([
      screen.getByRole('alert'),
      screen.getByTestId('map-stopped-content'),
    ]);
    unmount();
    expect(observer.disconnected).toBe(true);
  });
});
