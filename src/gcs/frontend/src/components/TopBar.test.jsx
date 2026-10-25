import { describe, it, expect, vi, afterEach } from 'vitest';
import { render, fireEvent } from '@testing-library/react';
import TopBar from './TopBar';

// Todo 69: E-STOP's one host is the top bar, in every phase, never inside a
// scroll container, and absent with no vehicles to stop.

const FLEET = [{ sys_id: 7, name: 's1-u7' }, { sys_id: 8, name: 's1-u8' }];

afterEach(() => { vi.restoreAllMocks(); });

function renderTopBar({ phase = 'MONITOR', vehicleList = FLEET } = {}) {
  return render(
    <TopBar
      connected
      phase={phase}
      vehicleList={vehicleList}
      seenIds={new Set()}
      showConn={false}
      setShowConn={() => {}}
      onOpenSettings={() => {}}
      onEstop={vi.fn()}
    />,
  );
}

describe('TopBar E-STOP', () => {
  it.each(['PLANNING', 'MONITOR'])('shows exactly one 32 px E-STOP in %s', (phase) => {
    const { getAllByRole } = renderTopBar({ phase });
    const estops = getAllByRole('button', { name: /topBar\.estop/ });
    expect(estops).toHaveLength(1);
    expect(estops[0].style.height).toBe('32px');
  });

  it('is outside any scroll container', () => {
    const { getByRole } = renderTopBar();
    for (let el = getByRole('button', { name: /topBar\.estop/ }); el; el = el.parentElement) {
      expect(['auto', 'scroll']).not.toContain(el.style.overflowY);
      expect(['auto', 'scroll']).not.toContain(el.style.overflow);
    }
  });

  it('its scope menu ends on screen and scrolls when the fleet is long', () => {
    // 448 px screen, the button's box ending at y 40 in the 48 px top bar.
    vi.spyOn(window, 'innerHeight', 'get').mockReturnValue(448);
    vi.spyOn(HTMLElement.prototype, 'getBoundingClientRect').mockReturnValue({
      top: 8, bottom: 40, left: 0, right: 96, width: 96, height: 32, x: 0, y: 8,
    });
    const fleet = Array.from({ length: 12 }, (_, i) => ({ sys_id: i + 1, name: `s1-u${i + 1}` }));
    const { getByRole } = renderTopBar({ vehicleList: fleet });

    fireEvent.click(getByRole('button', { name: /topBar\.estop/ }));

    const menu = getByRole('menu');
    expect(menu.style.maxHeight).toBe(`${448 - 40 - 4 - 8}px`);
    expect(menu.style.overflowY).toBe('auto');
    expect(menu.querySelectorAll('button')).toHaveLength(13);
  });

  it('its open scope menu re-measures its cap when the screen size changes', () => {
    let height = 648;
    vi.spyOn(window, 'innerHeight', 'get').mockImplementation(() => height);
    vi.spyOn(HTMLElement.prototype, 'getBoundingClientRect').mockReturnValue({
      top: 8, bottom: 40, left: 0, right: 96, width: 96, height: 32, x: 0, y: 8,
    });
    const fleet = Array.from({ length: 12 }, (_, i) => ({ sys_id: i + 1, name: `s1-u${i + 1}` }));
    const { getByRole } = renderTopBar({ vehicleList: fleet });

    fireEvent.click(getByRole('button', { name: /topBar\.estop/ }));
    expect(getByRole('menu').style.maxHeight).toBe(`${648 - 40 - 4 - 8}px`);

    height = 448;
    fireEvent(window, new Event('resize'));
    expect(getByRole('menu').style.maxHeight).toBe(`${448 - 40 - 4 - 8}px`);
  });

  it('is absent with no vehicles', () => {
    const { queryByRole } = renderTopBar({ vehicleList: [] });
    expect(queryByRole('button', { name: /topBar\.estop/ })).toBeNull();
  });
});
