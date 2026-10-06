import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, fireEvent, act } from '@testing-library/react';
import VideoPanel, { FIT_WIDTH_CSS, EXPANDED_WIDTH } from './VideoPanel';

const { readers, FakeReader } = vi.hoisted(() => {
  const readers = [];
  class FakeReader {
    constructor(conf) {
      this.conf = conf;
      this.closeCount = 0;
      readers.push(this);
    }

    close() {
      this.closeCount += 1;
    }
  }
  return { readers, FakeReader };
});

vi.mock('../vendor/mediamtx/reader', () => ({ default: FakeReader }));

const URL_CONFIGURED = 'http://192.168.144.10:8889/tracking/whep';
const latest = () => readers[readers.length - 1];
const goLive = (container) => {
  act(() => latest().conf.onTrack({ streams: [{ getTracks: () => [] }] }));
  fireEvent.playing(container.querySelector('video'));
};

beforeEach(() => {
  readers.length = 0;
  vi.spyOn(HTMLMediaElement.prototype, 'play').mockResolvedValue(undefined);
});

describe('VideoPanel', () => {
  it('renders nothing and opens no session when the feed is not configured', () => {
    const { container } = render(<VideoPanel url={null} />);
    expect(container).toBeEmptyDOMElement();
    expect(readers).toHaveLength(0);
  });

  it('labels the feed and shows the connecting state first', () => {
    const { getByRole, getByText } = render(<VideoPanel url={URL_CONFIGURED} />);
    expect(getByRole('region', { name: 'videoPanel.title' })).toBeInTheDocument();
    expect(getByText('videoPanel.connecting')).toBeInTheDocument();
  });

  it('plays muted and inline so the WebView can autoplay it', () => {
    const { container } = render(<VideoPanel url={URL_CONFIGURED} />);
    const video = container.querySelector('video');
    expect(video).toHaveAttribute('autoplay');
    expect(video).toHaveAttribute('playsinline');
    expect(video.muted).toBe(true);
  });

  it('drops the status overlay once the feed is live', () => {
    const { container, queryByText } = render(<VideoPanel url={URL_CONFIGURED} />);
    goLive(container);
    expect(queryByText('videoPanel.connecting')).not.toBeInTheDocument();
    expect(queryByText('videoPanel.error')).not.toBeInTheDocument();
  });

  it('reports a stalled feed without a retry action', () => {
    const { container, getByText, queryByText } = render(<VideoPanel url={URL_CONFIGURED} />);
    goLive(container);
    fireEvent.waiting(container.querySelector('video'));
    expect(getByText('videoPanel.stalled')).toBeInTheDocument();
    // The reader reconnects a dropped stream itself; nothing to retry by hand.
    expect(queryByText('videoPanel.retry')).not.toBeInTheDocument();
  });

  it('offers retry on an error, and retry starts a fresh session', () => {
    const { getByText } = render(<VideoPanel url={URL_CONFIGURED} />);
    act(() => latest().conf.onError('peer connection closed'));

    expect(getByText('videoPanel.error')).toBeInTheDocument();
    const first = latest();
    fireEvent.click(getByText('videoPanel.retry'));

    expect(first.closeCount).toBe(1);
    expect(readers).toHaveLength(2);
    expect(getByText('videoPanel.connecting')).toBeInTheDocument();
  });

  it('closes the session when it is unmounted', () => {
    const { unmount } = render(<VideoPanel url={URL_CONFIGURED} />);
    const reader = latest();
    unmount();
    expect(reader.closeCount).toBe(1);
  });
});

describe('VideoPanel placement', () => {
  const panelOf = (url = URL_CONFIGURED) => {
    const { getByRole, getByLabelText } = render(<VideoPanel url={url} />);
    return { panel: getByRole('region', { name: 'videoPanel.title' }), getByLabelText };
  };

  it('sits in the bottom-left corner, under the map controls', () => {
    const { panel } = panelOf();
    // Map overlay buttons are z-index 10 and the confirmation cards 20: the
    // video must never be the topmost layer, or it would swallow their taps.
    expect(Number(panel.style.zIndex)).toBeLessThan(10);
    expect(panel.style.bottom).toBe('12px');
    expect(panel.style.left).toBe('12px');
    // Reset-view lives in the opposite corner, so it cannot be covered.
    expect(panel.style.right).toBe('');
    expect(panel.style.top).toBe('');
  });

  it('stays anchored and height-capped when expanded', () => {
    const { panel, getByLabelText } = panelOf();
    const zIndex = panel.style.zIndex;

    fireEvent.click(getByLabelText('videoPanel.expand'));

    const expanded = getByLabelText('videoPanel.collapse');
    expect(expanded).toHaveAttribute('aria-pressed', 'true');
    // Still the same corner, still below the control layers, and bounded so it
    // cannot grow up into the top overlay row.
    expect(panel.style.bottom).toBe('12px');
    expect(panel.style.left).toBe('12px');
    expect(panel.style.zIndex).toBe(zIndex);
    expect(panel.style.maxHeight).not.toBe('');
    expect(panel.style.width).not.toBe('100%');

    fireEvent.click(expanded);
    expect(getByLabelText('videoPanel.expand')).toHaveAttribute('aria-pressed', 'false');
  });

  // jsdom does no layout: it cannot resolve min()/max()/percentages, so these
  // assert the width *expressions* instead of measured pixels. They guard the
  // two terms the live bug was missing — the pane clamp and the expanded floor
  // — and cover the arithmetic those terms imply at narrow and wide panes. Real
  // measured widths still have to be checked in a browser.
  it('keeps both widths inside the map pane, and expanded never below docked', () => {
    const docked = FIT_WIDTH_CSS;
    const enlarged = EXPANDED_WIDTH;

    // Same clamp on both sides: the docked 240px used to overflow a narrow
    // pane, and left:12px means the right margin has to be subtracted too.
    expect(docked).toContain('calc(100% - 24px)');
    expect(enlarged).toContain('calc(100% - 24px)');
    // The floor: 52% of a narrow pane is far under 240px, which is what made
    // Enlarge shrink the panel to ~124px in the browser.
    expect(enlarged).toContain('max(52%, 240px)');
    expect(enlarged).toContain('640px');

    const widthAt = (expr, pane) =>
      Function(
        'pane',
        `const min = Math.min, max = Math.max;
         const pct = (p) => (p / 100) * pane;
         return ${expr
           .replace(/calc\(100% - 24px\)/g, '(pane - 24)')
           .replace(/(\d+(?:\.\d+)?)%/g, 'pct($1)')
           .replace(/(\d+(?:\.\d+)?)px/g, '$1')};`,
      )(pane);

    for (const pane of [240, 320, 462, 800, 1600]) {
      const fit = widthAt(docked, pane);
      const big = widthAt(enlarged, pane);
      expect(fit).toBeLessThanOrEqual(pane - 24);
      expect(big).toBeLessThanOrEqual(pane - 24);
      expect(big).toBeGreaterThanOrEqual(fit);
    }
  });
});
