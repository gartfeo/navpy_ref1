// The NavPy Start/Stop row on a UAV card: the connection URL keeps a gap from
// the button and truncates instead of pushing into it.
import { describe, it, expect } from 'vitest';
import { render } from '@testing-library/react';
import NavpySimButton from './NavpySimButton';

const LONG_URL = 'udpout:192.168.100.200:14550?source_system=255&very_long_suffix=1';

function renderRow(props = {}) {
  const { container, getByTestId } = render(
    <NavpySimButton sysId={1} instance={{ running: false }} busy={false} connection={LONG_URL} onToggle={() => {}} {...props} />,
  );
  return { row: container.querySelector('button').parentElement, button: container.querySelector('button'), url: getByTestId('navpy-connection') };
}

describe('NavpySimButton connection URL', () => {
  it('sits a 6 px gap away from the Start/Stop button', () => {
    const { row } = renderRow();
    expect(row.style.display).toBe('flex');
    expect(row.style.gap).toBe('6px');
  });

  it('truncates with an ellipsis and keeps the full URL as a tooltip', () => {
    const { url } = renderRow();
    expect(url.style.minWidth).toBe('0px');
    expect(url.style.overflow).toBe('hidden');
    expect(url.style.textOverflow).toBe('ellipsis');
    expect(url.style.whiteSpace).toBe('nowrap');
    expect(url.title).toBe(LONG_URL);
    expect(url.textContent).toBe(LONG_URL);
  });

  it('the button never shrinks under a long URL', () => {
    const { button } = renderRow();
    expect(button.style.flexShrink).toBe('0');
  });

  it('no URL: no URL span', () => {
    const { container } = render(<NavpySimButton sysId={1} instance={{ running: true }} busy={false} onToggle={() => {}} />);
    expect(container.querySelector('[data-testid="navpy-connection"]')).toBeNull();
  });
});
