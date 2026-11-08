// The RC mode picker (docs/design/gcs_rc_physical_sticks.md): one button with the
// current mode; a tap opens every mode; the list never outlives the UAV and
// mode it was opened over.
import { describe, it, expect, vi } from 'vitest';
import { act, fireEvent, render, screen } from '@testing-library/react';
import FlightModeColumn, { MODE_BUTTON_WIDTH_PX } from './FlightModeColumn';
import { PICKER_MODES } from '../utils/flightModes';
import { LONG_PRESS_MS } from '../utils/armAction';
import { confirmStub } from '../test/confirmStub';

const props = (over = {}) => ({
  currentMode: 'FBWA',
  armed: false,
  sysId: 7,
  onSetMode: vi.fn(),
  onArmDisarm: () => {},
  onToggleManualControl: () => {},
  paneHeight: 437,
  ...over,
});

const picker = () => document.querySelector('[aria-haspopup="listbox"]');
const openPicker = () => fireEvent.click(picker());
const options = () => screen.queryAllByRole('option');

describe('RC mode picker', () => {
  it('shows the current mode, and opens the ten modes in a fixed order', () => {
    render(<FlightModeColumn {...props()} />);
    expect(picker()).toHaveTextContent('FBWA');
    expect(options()).toHaveLength(0);
    openPicker();
    expect(options().map((o) => o.textContent)).toEqual([
      'AUTO', 'FBWA', 'RTL', 'MANUAL', 'LOITER', 'GUIDED', 'STABILIZE', 'CIRCLE', 'CRUISE', 'FBWB',
    ]);
    expect(PICKER_MODES).toHaveLength(10);
    expect(screen.getByRole('option', { name: 'FBWA' })).toHaveAttribute('aria-selected', 'true');
  });

  it('shows an unknown mode by its name and no mode as "Mode"', () => {
    const { rerender } = render(<FlightModeColumn {...props({ currentMode: 'QSTABILIZE' })} />);
    expect(picker()).toHaveTextContent('QSTABILIZE');
    rerender(<FlightModeColumn {...props({ currentMode: undefined })} />);
    expect(picker()).toHaveTextContent('flightMode.mode');
  });

  it('a pick sends that mode to the selected UAV and closes the list', () => {
    const p = props();
    render(<FlightModeColumn {...p} />);
    openPicker();
    fireEvent.click(screen.getByRole('option', { name: 'RTL' }));
    expect(p.onSetMode).toHaveBeenCalledWith(7, 'RTL');
    expect(options()).toHaveLength(0);
  });

  it('the current mode and a second tap on the picker close it with nothing sent', () => {
    const p = props();
    render(<FlightModeColumn {...p} />);
    openPicker();
    fireEvent.click(screen.getByRole('option', { name: 'FBWA' }));
    expect(options()).toHaveLength(0);
    openPicker();
    openPicker();
    expect(options()).toHaveLength(0);
    expect(p.onSetMode).not.toHaveBeenCalled();
  });

  it('is disabled with no UAV selected', () => {
    render(<FlightModeColumn {...props({ sysId: null })} />);
    expect(picker()).toBeDisabled();
    openPicker();
    expect(options()).toHaveLength(0);
  });

  it('closes when the selected UAV or its reported mode changes', () => {
    const p = props();
    const { rerender } = render(<FlightModeColumn {...p} />);
    openPicker();
    rerender(<FlightModeColumn {...p} sysId={8} />);
    expect(options()).toHaveLength(0);
    openPicker();
    rerender(<FlightModeColumn {...p} sysId={8} currentMode="RTL" />);
    expect(options()).toHaveLength(0);
    expect(p.onSetMode).not.toHaveBeenCalled();
  });

  it('closes on Escape and on a press outside the column, without consuming that press', () => {
    render(
      <>
        <FlightModeColumn {...props()} />
        <div data-testid="stick" />
      </>,
    );
    openPicker();
    fireEvent.keyDown(document, { key: 'Escape' });
    expect(options()).toHaveLength(0);
    openPicker();
    const stick = screen.getByTestId('stick');
    const seen = vi.fn();
    // A stick that stops the press: the column must still hear it (capture phase).
    stick.addEventListener('pointerdown', (e) => { e.stopPropagation(); seen(); });
    fireEvent.pointerDown(stick);
    expect(options()).toHaveLength(0);
    expect(seen).toHaveBeenCalledTimes(1);
  });

  it('a press inside the list does not close it before the pick', () => {
    render(<FlightModeColumn {...props()} />);
    openPicker();
    fireEvent.pointerDown(screen.getByRole('option', { name: 'CRUISE' }));
    expect(options()).toHaveLength(10);
  });

  it('caps the list at the pane and scrolls, with rows that do not shrink', () => {
    render(<FlightModeColumn {...props({ paneHeight: 352 })} />);
    openPicker();
    const list = screen.getByRole('listbox').style;
    // jsdom does no layout: the picker's offsetTop reads 0, so the cap is 352 - 12 - 12.
    expect(list.maxHeight).toBe('328px');
    expect(list.overflowY).toBe('auto');
    expect(list.boxSizing).toBe('border-box');
    for (const o of options()) {
      expect(o.style.flexShrink).toBe('0');
      expect(o.style.minHeight).toBe('32px');
    }
  });

  it('keeps the column 80 px wide, so long warnings and the hint wrap', () => {
    const { container } = render(
      <FlightModeColumn {...props({ prearmSeverity: 'warn', prearmWarnings: ['A very long pre-arm warning that wraps'] })} />,
    );
    const column = container.firstChild;
    expect(column.style.width).toBe(`${MODE_BUTTON_WIDTH_PX}px`);
    for (const child of column.children) {
      if (child.style.maxWidth) expect(child.style.maxWidth).toBe(`${MODE_BUTTON_WIDTH_PX}px`);
    }
  });

  it('reports its height on mount, when warnings change it, and null on unmount', () => {
    const observers = [];
    vi.stubGlobal('ResizeObserver', class {
      constructor(cb) { this.cb = cb; observers.push(this); }
      observe() {}
      disconnect() { this.gone = true; }
    });
    // jsdom does no layout: stand in for two warning lines making the column taller.
    vi.spyOn(HTMLElement.prototype, 'offsetHeight', 'get').mockImplementation(function height() {
      return 100 + (this.textContent.includes('Compass') ? 28 : 0);
    });
    const onHeight = vi.fn();
    const p = props({ onHeight, prearmSeverity: 'warn' });
    const { rerender, unmount } = render(<FlightModeColumn {...p} />);
    expect(onHeight).toHaveBeenLastCalledWith(100);
    rerender(<FlightModeColumn {...p} prearmWarnings={['GPS', 'Compass']} />);
    observers[0].cb();
    expect(onHeight).toHaveBeenLastCalledWith(128);
    unmount();
    expect(observers[0].gone).toBe(true);
    expect(onHeight).toHaveBeenLastCalledWith(null);
    render(<FlightModeColumn {...p} />);
    expect(onHeight).toHaveBeenLastCalledWith(100);
    vi.unstubAllGlobals();
  });

  // The list is tied in render to the UAV and mode it opened over, so no pick
  // can land between a change and an effect closing it.
  it('a pick made after the UAV changed, before the list closed, sends nothing', () => {
    const p = props();
    const { rerender } = render(<FlightModeColumn {...p} />);
    openPicker();
    const rtl = screen.getByRole('option', { name: 'RTL' });
    rerender(<FlightModeColumn {...p} sysId={8} />);
    fireEvent.click(rtl);
    expect(p.onSetMode).not.toHaveBeenCalled();
    // Back on the first UAV the old list does not come back.
    rerender(<FlightModeColumn {...p} />);
    expect(options()).toHaveLength(0);
  });
});

describe('ARM hold confirm (todo 233)', () => {
  // A completed hold asks the in-page confirm, never the browser's blocking
  // window.confirm, and only Confirm sends the arm command.
  const holdArm = async (answer) => {
    vi.useFakeTimers({ toFake: ['performance', 'requestAnimationFrame', 'cancelAnimationFrame'] });
    const native = vi.spyOn(window, 'confirm');
    const onArmDisarm = vi.fn();
    const { confirm, wrapper } = confirmStub(answer);
    render(<FlightModeColumn {...props({ prearmSeverity: 'warn', onArmDisarm })} />, { wrapper });
    fireEvent.mouseDown(screen.getByText('flightMode.arm'));
    act(() => { vi.advanceTimersByTime(LONG_PRESS_MS + 32); });
    await act(async () => {});
    return { confirm, native, onArmDisarm };
  };

  it('arms after Confirm', async () => {
    const { confirm, native, onArmDisarm } = await holdArm(true);
    expect(confirm).toHaveBeenCalledWith('flightMode.forceArm');
    expect(native).not.toHaveBeenCalled();
    expect(onArmDisarm).toHaveBeenCalledWith(7, true, { force: true });
  });

  it('sends nothing after Cancel', async () => {
    const { confirm, onArmDisarm } = await holdArm(false);
    expect(confirm).toHaveBeenCalledTimes(1);
    expect(onArmDisarm).not.toHaveBeenCalled();
  });
});
