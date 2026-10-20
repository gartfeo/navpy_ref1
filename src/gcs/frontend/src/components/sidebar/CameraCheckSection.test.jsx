import { render, screen } from '@testing-library/react';
import { describe, expect, it } from 'vitest';
import CameraCheckSection from './CameraCheckSection';

const problemResult = {
  status: 'problems',
  zones: [{
    zone_index: 0, checked: true, estimate: false, height_m: 403, max_height_m: 389.58, windows: [],
    problems: [{ kind: 'no_search_window', severity: 'error', near_m: 757, range_m: 829, max_height_m: 389.58 }],
  }],
};

describe('CameraCheckSection', () => {
  it('shows the cause and what to do for an impossible plan', () => {
    render(<CameraCheckSection result={problemResult} zones={[{}]} />);
    const box = screen.getByTestId('camera-check');
    expect(box).toHaveTextContent('cameraCheck.panelTitle');
    expect(box).toHaveTextContent('cameraCheck.noWindow');
    expect(box).toHaveTextContent('cameraCheck.fixLowerAltitude');
  });

  it('renders nothing for a compatible plan or no result', () => {
    const { container, rerender } = render(<CameraCheckSection result={{ status: 'ok', zones: [] }} zones={[]} />);
    expect(container).toBeEmptyDOMElement();
    rerender(<CameraCheckSection result={null} zones={[]} />);
    expect(container).toBeEmptyDOMElement();
  });

  it('says when the check could not run', () => {
    render(<CameraCheckSection result={{ status: 'error', error: 'down', zones: [] }} zones={[]} />);
    expect(screen.getByTestId('camera-check')).toHaveTextContent('cameraCheck.couldNotRunShort');
  });
});
