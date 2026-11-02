import { describe, it, expect, vi, afterEach } from 'vitest';
import { render, fireEvent, within } from '@testing-library/react';
import TopBar from './TopBar';
import { topBarHeight } from '../styles';

// Todo 69: E-STOP's one host is the top bar, in every phase, never inside a
// scroll container, and absent with no vehicles to stop.

const FLEET = [{ sys_id: 7, name: 's1-u7' }, { sys_id: 8, name: 's1-u8' }];

afterEach(() => { vi.restoreAllMocks(); });

function renderTopBar({ phase = 'MONITOR', vehicleList = FLEET, ...props } = {}) {
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
      {...props}
    />,
  );
}

describe('TopBar panel toggle (handheld layout)', () => {
  const toggleOf = (view) => view.getByRole('button', { name: /topBar\.(show|hide)Panel/ });

  it('shows E-STOP and the toggle while the panel is collapsed', () => {
    const view = renderTopBar({ compact: true, panelOpen: false, onTogglePanel: () => {} });
    expect(view.getByRole('button', { name: /topBar\.estop/ })).toBeInTheDocument();
    const toggle = toggleOf(view);
    expect(toggle.getAttribute('aria-expanded')).toBe('false');
    expect(toggle.getAttribute('aria-label')).toBe('topBar.showPanel');
  });

  it('says it is open and flips on a tap', () => {
    const onTogglePanel = vi.fn();
    const view = renderTopBar({ panelOpen: true, onTogglePanel });
    const toggle = toggleOf(view);
    expect(toggle.getAttribute('aria-expanded')).toBe('true');
    expect(toggle.getAttribute('aria-label')).toBe('topBar.hidePanel');
    fireEvent.click(toggle);
    expect(onTogglePanel).toHaveBeenCalledTimes(1);
  });

  it('sits at the right end of the bar', () => {
    const view = renderTopBar({ panelOpen: true, onTogglePanel: () => {} });
    const right = view.getByRole('button', { name: /topBar\.estop/ }).parentElement.parentElement;
    expect(right.lastElementChild).toBe(toggleOf(view));
  });

  it('carries the count of pending rounds, and no badge at zero', () => {
    const counted = renderTopBar({ panelOpen: false, onTogglePanel: () => {}, pendingCount: 2 });
    expect(within(toggleOf(counted)).getByTestId('panel-pending-count').textContent).toBe('2');
    counted.unmount();
    const none = renderTopBar({ panelOpen: false, onTogglePanel: () => {}, pendingCount: 0 });
    expect(within(toggleOf(none)).queryByTestId('panel-pending-count')).toBeNull();
  });

  it('is absent where the panel is already gone (RC)', () => {
    const view = renderTopBar({ panelOpen: true });
    expect(view.queryByRole('button', { name: /topBar\.(show|hide)Panel/ })).toBeNull();
  });

  // RC can end with no tap (target lost, phase change): E-STOP must not move then.
  it.each([true, false])('at compact=%s RC keeps its place, so E-STOP does not move', (compact) => {
    const rightOf = (view) => view.getByRole('button', { name: /topBar\.estop/ }).parentElement.parentElement;
    const shown = renderTopBar({ compact, panelOpen: true, onTogglePanel: () => {} });
    const withToggle = { count: rightOf(shown).children.length, width: toggleOf(shown).style.width };
    shown.unmount();

    const rc = renderTopBar({ compact, panelOpen: true });
    const right = rightOf(rc);
    expect(right.children.length).toBe(withToggle.count);
    expect(right.lastElementChild).toBe(rc.getByTestId('panel-toggle-place'));
    expect(right.lastElementChild.style.width).toBe(withToggle.width);
    expect(withToggle.width).toBe('32px');
  });
});

describe('TopBar shrink rule (handheld layout)', () => {
  const groups = (view) => {
    const right = view.getByRole('button', { name: /topBar\.estop/ }).parentElement.parentElement;
    return { right, left: right.previousElementSibling };
  };

  it.each([true, false])('at compact=%s the left group clips and the right group never shrinks', (compact) => {
    const { left, right } = groups(renderTopBar({ compact }));
    expect(left.style.minWidth).toBe('0px');
    expect(left.style.overflow).toBe('hidden');
    expect(right.style.flexShrink).toBe('0');
  });

  it('wide: the label, the logo and the count text stay', () => {
    const view = renderTopBar();
    expect(view.getByText('app.gcs')).toBeInTheDocument();
    expect(view.getByAltText('AAS')).toBeInTheDocument();
    expect(view.getByRole('button', { name: /topBar\.uavCount/ })).toBeInTheDocument();
  });

  it('compact: no label; the connection button shows the dot and the count, the full text in its title', () => {
    const view = renderTopBar({ compact: true });
    expect(view.queryByText('app.gcs')).toBeNull();
    const conn = view.getByTitle(/topBar\.uavCount/);
    expect(conn.textContent).toBe('2');
  });

  it('compact and no UAVs: the dot and a 0, the full text in its title', () => {
    const view = renderTopBar({ compact: true, vehicleList: [] });
    const conn = view.getByTitle('topBar.noUavs');
    expect(conn.textContent).toBe('0');
  });

  it('compact in planning: the PLANNING chip replaces the logo', () => {
    const view = renderTopBar({ compact: true, phase: 'PLANNING' });
    expect(view.getByText('app.planning')).toBeInTheDocument();
    expect(view.queryByAltText('AAS')).toBeNull();
  });

  it('compact: the connection panel opens fixed, inside the screen', () => {
    const view = renderTopBar({ compact: true, showConn: true });
    const panel = view.getByTestId('connection-dropdown');
    expect(panel.style.position).toBe('fixed');
    expect(panel.style.top).toBe(`${topBarHeight}px`);
    expect(panel.style.right).toBe('8px');
    expect(panel.style.maxWidth).toBe('calc(100vw - 16px)');
  });

  it('wide: the connection panel keeps dropping from its button', () => {
    const view = renderTopBar({ showConn: true });
    expect(view.getByTestId('connection-dropdown').style.position).toBe('absolute');
  });
});

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
