import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, act } from '@testing-library/react';
import FpvViewOverlay from './FpvViewOverlay';

vi.mock('react-i18next', () => ({
  useTranslation: () => ({ t: (key, opts) => (opts ? `${key} ${Object.values(opts).join(' ')}` : key) }),
}));

beforeEach(() => { vi.useFakeTimers(); });
const advance = (ms) => act(() => { vi.advanceTimersByTime(ms); });

describe('FpvViewOverlay', () => {
  it('always shows the SYNTHETIC tag naming the UAV and a frame, without taking taps', () => {
    const { getByRole, getByTestId, container } = render(<FpvViewOverlay label="UAV 2" holdReason={null} top={64} />);
    expect(getByRole('note')).toHaveTextContent('fpvView.tag UAV 2');
    expect(getByRole('note').style.top).toBe('64px');
    expect(getByTestId('fpv-frame').style.boxShadow).toContain('inset');
    expect(container.firstChild.style.pointerEvents).toBe('none');
  });

  it('link lost: "View held" banner with the time held; link back: banner gone', () => {
    const { getByRole, queryByRole, rerender } = render(<FpvViewOverlay label="UAV 1" holdReason="linkLost" top={64} />);
    expect(getByRole('status')).toHaveTextContent('fpvView.held.linkLost 0');
    advance(3000);
    expect(getByRole('status')).toHaveTextContent('fpvView.held.linkLost 3');
    rerender(<FpvViewOverlay label="UAV 1" holdReason={null} top={64} />);
    expect(queryByRole('status')).toBeNull();
  });

  it('linked but pose not finite: "pose unavailable", and the time restarts per hold', () => {
    const { getByRole, rerender } = render(<FpvViewOverlay label="UAV 1" holdReason="poseUnavailable" top={54} />);
    advance(2000);
    expect(getByRole('status')).toHaveTextContent('fpvView.held.poseUnavailable 2');
    rerender(<FpvViewOverlay label="UAV 1" holdReason={null} top={54} />);
    rerender(<FpvViewOverlay label="UAV 1" holdReason="poseUnavailable" top={54} />);
    expect(getByRole('status')).toHaveTextContent('fpvView.held.poseUnavailable 0');
  });

  it('telemetry disconnected has its own held reason', () => {
    const { getByRole } = render(<FpvViewOverlay label="UAV 1" holdReason="telemetryDisconnected" top={54} />);
    expect(getByRole('status')).toHaveTextContent('fpvView.held.telemetryDisconnected 0');
  });
});
