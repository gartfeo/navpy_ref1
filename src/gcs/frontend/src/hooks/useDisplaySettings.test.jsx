import { describe, expect, it } from 'vitest';
import { fireEvent, render } from '@testing-library/react';
import useDisplaySettings from './useDisplaySettings';

function Probe() {
  const { showVision, setShowVision } = useDisplaySettings();
  return (
    <button onClick={() => setShowVision(!showVision)}>
      {showVision ? 'vision-on' : 'vision-off'}
    </button>
  );
}

describe('useDisplaySettings', () => {
  it('shows forward vision cones by default and still allows hiding them', () => {
    const { getByRole } = render(<Probe />);
    const toggle = getByRole('button');

    expect(toggle).toHaveTextContent('vision-on');
    fireEvent.click(toggle);
    expect(toggle).toHaveTextContent('vision-off');
  });
});
