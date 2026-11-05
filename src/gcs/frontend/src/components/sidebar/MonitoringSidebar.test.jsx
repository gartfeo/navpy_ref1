import { describe, it, expect, vi, beforeEach } from 'vitest';
import { useState } from 'react';
import { render, act, fireEvent, within } from '@testing-library/react';
import MonitoringSidebar from './MonitoringSidebar';
import { LONG_PRESS_MS } from '../../utils/armAction';

// The handheld footer (docs/design/gcs_handheld_layout.md): the phase's action
// renders into the slot App places in the list's scrolling part, so it stays in
// view on a short screen. A timed round outranks the start action, never the
// abort. Everything else stays in the flow. The launcher and the UAV cards are
// stood in for: they are not what these tests are about.

const launcher = {
  containerLaunching: false, launcherConnected: true, simulatorRunning: true, checkingLauncher: false,
  checkLauncher: () => {}, doContainerLaunch: vi.fn(async () => {}), esp32Host: 'h', esp32Port: 1,
};
vi.mock('../../hooks/useContainerLauncher', () => ({ default: () => launcher }));
vi.mock('./VehicleStatusCard', () => ({ default: ({ vehicle }) => <div>card {vehicle.sys_id}</div> }));
vi.mock('../../utils/prearmChecks', async (importOriginal) => ({
  ...(await importOriginal()),
  computeLaunchReadiness: () => readiness.current,
}));
const READY = { ready: true, issues: [], advisories: [] };
const readiness = { current: READY };

const UAV = { sys_id: 7, name: 's1-u7', armed: false, mission_total: 3 };
const PLAN = { zones: [{ track: [] }] };
const SETTINGS = (launchType) => ({ launch: { launch_type: launchType } });

/** App's slot: a callback ref in state, so the sidebar gets the element. */
function Host({ withSlot = true, ...props }) {
  const [slot, setSlot] = useState(null);
  return (
    <>
      <MonitoringSidebar
        vehicleList={[UAV]}
        plan={PLAN}
        navpyStatus={{}}
        launchStates={{}}
        settings={SETTINGS('bungee')}
        onOpenReadiness={null}
        footerEl={withSlot ? slot : null}
        {...props}
      />
      {withSlot && <div data-testid="slot" ref={setSlot} />}
    </>
  );
}

const inSlot = (view) => within(view.getByTestId('slot'));

beforeEach(() => {
  launcher.containerLaunching = false;
  readiness.current = READY;
  vi.useFakeTimers({ toFake: ['performance', 'requestAnimationFrame', 'cancelAnimationFrame'] });
});

describe('MonitoringSidebar footer', () => {
  it('bungee with a plan: HOLD TO START is in the slot', () => {
    const view = render(<Host />);
    expect(inSlot(view).getByRole('button', { name: 'monitor.holdToStart' })).toBeInTheDocument();
  });

  it('with no GCS plan: the download button is in the slot, for each launch type', () => {
    for (const type of ['bungee', 'container']) {
      const view = render(<Host plan={null} settings={SETTINGS(type)} />);
      expect(inSlot(view).getByRole('button', { name: 'monitor.downloadPlan' })).toBeInTheDocument();
      view.unmount();
    }
  });

  it('container with a plan: HOLD TO LAUNCH is in the slot, the launch list stays in the flow', () => {
    const view = render(<Host settings={SETTINGS('container')} vehicleList={[{ ...UAV, armed: false }]} />);
    const slot = inSlot(view);
    expect(slot.getByRole('button', { name: 'monitor.holdToLaunch' })).toBeInTheDocument();
    expect(slot.queryByText('monitor.launchSequence')).toBeNull();
    expect(view.getByText('monitor.launchSequence')).toBeInTheDocument();
  });

  it('container launching: abort is in the slot, also with an airborne UAV in a staggered launch', () => {
    launcher.containerLaunching = true;
    const view = render(
      <Host
        settings={SETTINGS('container')}
        vehicleList={[UAV, { ...UAV, sys_id: 8, armed: true }]}
        launchStates={{ 7: { state: 'queued' }, 8: { state: 'airborne' } }}
        onAbortLaunch={() => {}}
      />,
    );
    expect(inSlot(view).getByRole('button', { name: 'launch.abortLaunch' })).toBeInTheDocument();
    expect(inSlot(view).queryByText('monitor.launchSequence')).toBeNull();
  });

  it('in flight: restart stays in the flow, not one stray tap from the map', () => {
    const view = render(<Host vehicleList={[{ ...UAV, armed: true }]} />);
    expect(inSlot(view).queryByRole('button')).toBeNull();
    expect(view.getByRole('button', { name: 'monitor.restartMission' })).toBeInTheDocument();
  });

  it('with no slot the action renders in the flow', () => {
    const view = render(<Host withSlot={false} />);
    expect(view.getByRole('button', { name: 'monitor.holdToStart' })).toBeInTheDocument();
  });

  it('takes back the sidebar padding: same width and gap as in the flow', () => {
    const view = render(<Host />);
    const wrapper = view.getByTestId('slot').firstChild;
    expect(wrapper.style.padding).toBe('0px 16px 8px');
    expect(wrapper.style.marginTop).toBe('-16px');
    expect(wrapper.style.background).not.toBe('');
  });
});

describe('MonitoringSidebar footer pinning', () => {
  const pinning = (view) => {
    const { position, bottom } = view.getByTestId('slot').firstChild.style;
    return { position, bottom };
  };
  const PINNED = { position: 'sticky', bottom: '0px' };
  const IN_FLOW = { position: '', bottom: '' };
  const START_ACTIONS = [
    ['bungee HOLD TO START', { settings: SETTINGS('bungee') }],
    ['container HOLD TO LAUNCH', { settings: SETTINGS('container') }],
    ['download with no plan, bungee', { settings: SETTINGS('bungee'), plan: null }],
    ['download with no plan, container', { settings: SETTINGS('container'), plan: null }],
  ];

  it.each(START_ACTIONS)('%s is pinned with no round, in the flow while a round exists', (_name, props) => {
    const view = render(<Host {...props} />);
    expect(pinning(view)).toEqual(PINNED);
    view.rerender(<Host {...props} startPinned={false} />);
    expect(pinning(view)).toEqual(IN_FLOW);
    view.rerender(<Host {...props} startPinned />);
    expect(pinning(view)).toEqual(PINNED);
  });

  it('a round never moves the start action to another tree position: the pitot tick survives', () => {
    const settings = { launch: { launch_type: 'bungee', auto_preflight_cal: true } };
    const fleet = [{ ...UAV, airspeed_present: true }];
    const view = render(<Host settings={settings} vehicleList={fleet} />);
    const box = inSlot(view).getByRole('checkbox');
    const start = inSlot(view).getByRole('button', { name: 'monitor.holdToStart' });
    fireEvent.click(box);
    view.rerender(<Host settings={settings} vehicleList={fleet} startPinned={false} />);
    // The same elements: a remount would lose a running hold, which lives in the button.
    expect(inSlot(view).getByRole('checkbox')).toBe(box);
    expect(inSlot(view).getByRole('button', { name: 'monitor.holdToStart' })).toBe(start);
    expect(box.checked).toBe(true);
  });

  it('a round moving HOLD TO LAUNCH from under the finger cancels the hold: no launch confirmation', () => {
    const settings = SETTINGS('container');
    const view = render(<Host settings={settings} />);
    const hold = inSlot(view).getByRole('button', { name: 'monitor.holdToLaunch' });
    const place = (top) => {
      hold.getClientRects = () => [{ width: 287, height: 45 }];
      hold.getBoundingClientRect = () => ({ left: 649, right: 936, top, bottom: top + 45 });
    };
    place(335);
    const finger = { clientX: 790, clientY: 355 };
    fireEvent.touchStart(hold, { touches: [finger], changedTouches: [finger] });
    act(() => { vi.advanceTimersByTime(LONG_PRESS_MS / 2); });

    view.rerender(<Host settings={settings} startPinned={false} />);
    place(600);
    act(() => { vi.advanceTimersByTime(LONG_PRESS_MS + 32); });
    expect(view.queryByText('launch.containerLaunchGo')).toBeNull();
  });

  it('the abort stays pinned while a round exists', () => {
    launcher.containerLaunching = true;
    const view = render(
      <Host
        settings={SETTINGS('container')}
        vehicleList={[UAV, { ...UAV, sys_id: 8, armed: true }]}
        launchStates={{ 7: { state: 'queued' }, 8: { state: 'airborne' } }}
        onAbortLaunch={() => {}}
        startPinned={false}
      />,
    );
    expect(inSlot(view).getByRole('button', { name: 'launch.abortLaunch' })).toBeInTheDocument();
    expect(pinning(view)).toEqual(PINNED);
  });
});

describe('MonitoringSidebar bungee START MISSION: hold, then confirm', () => {
  const hold = (button) => {
    fireEvent.mouseDown(button);
    act(() => { vi.advanceTimersByTime(LONG_PRESS_MS + 32); });
    fireEvent.mouseUp(button);
  };
  const goButton = (view) => view.queryByRole('button', { name: 'monitor.startMissionGo' });

  it('a single click starts nothing and opens no confirmation', () => {
    const onRestart = vi.fn(async () => {});
    const view = render(<Host onRestart={onRestart} />);
    fireEvent.click(inSlot(view).getByRole('button', { name: 'monitor.holdToStart' }));
    expect(goButton(view)).toBeNull();
    expect(onRestart).not.toHaveBeenCalled();
  });

  it('a hold shorter than the arm time opens no confirmation', () => {
    const onRestart = vi.fn(async () => {});
    const view = render(<Host onRestart={onRestart} />);
    const button = inSlot(view).getByRole('button', { name: 'monitor.holdToStart' });
    fireEvent.mouseDown(button);
    act(() => { vi.advanceTimersByTime(LONG_PRESS_MS / 2); });
    fireEvent.mouseUp(button);
    act(() => { vi.advanceTimersByTime(LONG_PRESS_MS); });
    expect(goButton(view)).toBeNull();
    expect(onRestart).not.toHaveBeenCalled();
  });

  it('a full hold opens the confirmation; Cancel starts nothing; Confirm starts once', async () => {
    const onRestart = vi.fn(async () => {});
    const view = render(<Host onRestart={onRestart} />);
    const button = inSlot(view).getByRole('button', { name: 'monitor.holdToStart' });

    hold(button);
    expect(goButton(view)).not.toBeNull();
    expect(onRestart).not.toHaveBeenCalled();
    fireEvent.click(view.getByRole('button', { name: 'confirm.cancel' }));
    expect(goButton(view)).toBeNull();
    expect(onRestart).not.toHaveBeenCalled();

    hold(button);
    await act(async () => { fireEvent.click(goButton(view)); });
    expect(goButton(view)).toBeNull();
    expect(onRestart).toHaveBeenCalledTimes(1);
    expect(onRestart).toHaveBeenCalledWith([7], { pitotCovered: false });
  });

  it('collapsing closes an open start confirmation', () => {
    const onRestart = vi.fn(async () => {});
    const view = render(<Host onRestart={onRestart} />);
    hold(inSlot(view).getByRole('button', { name: 'monitor.holdToStart' }));
    expect(goButton(view)).not.toBeNull();
    view.rerender(<Host onRestart={onRestart} hidden />);
    view.rerender(<Host onRestart={onRestart} hidden={false} />);
    expect(goButton(view)).toBeNull();
    expect(onRestart).not.toHaveBeenCalled();
  });

  it('not ready: the force override is unchanged (hold, browser confirm, force start)', async () => {
    readiness.current = { ready: false, issues: ['low battery'], advisories: [] };
    const onRestart = vi.fn(async () => {});
    const confirm = vi.spyOn(window, 'confirm').mockReturnValue(true);
    try {
      const view = render(<Host onRestart={onRestart} />);
      const button = inSlot(view).getByRole('button', { name: 'monitor.startMission' });
      fireEvent.click(button);
      expect(onRestart).not.toHaveBeenCalled();
      await act(async () => { hold(button); });
      expect(confirm).toHaveBeenCalledWith('monitor.forceStartConfirm');
      expect(goButton(view)).toBeNull();
      expect(onRestart).toHaveBeenCalledWith([7], { force: true, pitotCovered: false });
    } finally {
      confirm.mockRestore();
    }
  });
});

describe('MonitoringSidebar hidden', () => {
  it('collapsing closes an open launch confirmation; reopening does not bring it back', () => {
    const view = render(<Host settings={SETTINGS('container')} />);
    const armed = inSlot(view).getByRole('button', { name: 'monitor.holdToLaunch' });
    fireEvent.mouseDown(armed);
    act(() => { vi.advanceTimersByTime(LONG_PRESS_MS + 32); });
    fireEvent.mouseUp(armed);
    expect(view.getByText('launch.containerLaunchGo')).toBeInTheDocument();

    view.rerender(<Host settings={SETTINGS('container')} hidden />);
    expect(view.queryByText('launch.containerLaunchGo')).toBeNull();
    view.rerender(<Host settings={SETTINGS('container')} hidden={false} />);
    expect(view.queryByText('launch.containerLaunchGo')).toBeNull();
    expect(launcher.doContainerLaunch).not.toHaveBeenCalled();
  });

  it('keeps pending form data: the pitot acknowledgement survives a collapse', () => {
    const settings = { launch: { launch_type: 'bungee', auto_preflight_cal: true } };
    const fleet = [{ ...UAV, airspeed_present: true }];
    const view = render(<Host settings={settings} vehicleList={fleet} />);
    const box = inSlot(view).getByRole('checkbox');
    fireEvent.click(box);
    expect(box.checked).toBe(true);
    view.rerender(<Host settings={settings} vehicleList={fleet} hidden />);
    view.rerender(<Host settings={settings} vehicleList={fleet} hidden={false} />);
    expect(inSlot(view).getByRole('checkbox').checked).toBe(true);
  });
});
