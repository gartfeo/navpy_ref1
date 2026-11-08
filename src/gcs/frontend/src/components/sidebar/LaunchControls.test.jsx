import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, fireEvent, act } from '@testing-library/react';
import { ForceLaunchButton, ForceStartButton } from './LaunchControls';
import { LONG_PRESS_MS } from '../../utils/armAction';
import { confirmStub } from '../../test/confirmStub';
import ConfirmProvider from '../ConfirmProvider';

// ForceStartButton drives the real useLongPress; fake the clock primitives it
// relies on so a completed hold is deterministic. The setup file restores real
// timers and spies after each test.
beforeEach(() => {
  vi.useFakeTimers({ toFake: ['performance', 'requestAnimationFrame', 'cancelAnimationFrame'] });
});

const advance = (ms) => act(() => { vi.advanceTimersByTime(ms); });
const renderBtn = (props, wrapper) => render(<ForceStartButton label="START" disabled={false} {...props} />, { wrapper });
// The in-page confirm answers asynchronously: let its promise settle.
const settle = () => act(async () => {});

describe('ForceStartButton — armReady false', () => {
  // #4 — without armReady the button is a plain single-click start.
  it('fires onStart on a single click when ready', () => {
    const onStart = vi.fn();
    const { getByRole } = renderBtn({ ready: true, onStart });
    fireEvent.click(getByRole('button'));
    expect(onStart).toHaveBeenCalledTimes(1);
  });
});

describe('ForceStartButton — container arm mode (armReady + ready)', () => {
  // #1 (button level) — a single click must NOT arm or launch; a container
  // launch can never start on one stray click.
  it('does nothing on a single click (no onArm, no onStart)', () => {
    const onStart = vi.fn();
    const onArm = vi.fn();
    const { getByRole } = renderBtn({ armReady: true, ready: true, onStart, onArm });
    fireEvent.click(getByRole('button'));
    expect(onArm).not.toHaveBeenCalled();
    expect(onStart).not.toHaveBeenCalled();
  });

  // #2 (button level) — a completed hold requests confirmation via onArm and
  // never launches directly.
  it('calls onArm exactly once after a completed hold (never onStart)', () => {
    const onStart = vi.fn();
    const onArm = vi.fn();
    const { getByRole } = renderBtn({ armReady: true, ready: true, onStart, onArm });
    fireEvent.mouseDown(getByRole('button'));
    advance(LONG_PRESS_MS + 32);
    expect(onArm).toHaveBeenCalledTimes(1);
    expect(onStart).not.toHaveBeenCalled();
  });
});

describe('ForceStartButton — not-ready force path', () => {
  // #3 — a single click is inert; no confirm, no launch.
  it('does not confirm or launch on a single click', () => {
    const onStart = vi.fn();
    const { confirm: confirmSpy, wrapper } = confirmStub(true);
    const { getByRole } = renderBtn({ ready: false, onStart }, wrapper);
    fireEvent.click(getByRole('button'));
    expect(confirmSpy).not.toHaveBeenCalled();
    expect(onStart).not.toHaveBeenCalled();
  });

  // #3 — a completed hold asks the in-page confirm; accepting force-launches once.
  it('force-launches once after a completed hold when confirm is accepted', async () => {
    const onStart = vi.fn();
    const onArm = vi.fn();
    const { confirm: confirmSpy, wrapper } = confirmStub(true);
    const { getByRole } = renderBtn({ ready: false, onStart, onArm }, wrapper);
    fireEvent.mouseDown(getByRole('button'));
    advance(LONG_PRESS_MS + 32);
    await settle();
    expect(confirmSpy).toHaveBeenCalledTimes(1);
    expect(onStart).toHaveBeenCalledTimes(1);
    expect(onStart).toHaveBeenCalledWith({ force: true, signal: expect.any(AbortSignal) });
    expect(onArm).not.toHaveBeenCalled();
  });

  // #3 — declining the confirm cancels the launch.
  it('does not launch when confirm is declined', async () => {
    const onStart = vi.fn();
    const { confirm: confirmSpy, wrapper } = confirmStub(false);
    const { getByRole } = renderBtn({ ready: false, onStart }, wrapper);
    fireEvent.mouseDown(getByRole('button'));
    advance(LONG_PRESS_MS + 32);
    await settle();
    expect(confirmSpy).toHaveBeenCalledTimes(1);
    expect(onStart).not.toHaveBeenCalled();
  });
});

describe('ForceStartButton — force confirm outlives the button', () => {
  // Telemetry can leave PRE_LAUNCH while the force-start question is open, and
  // the button unmounts: the question goes with it and nothing starts.
  it('a pending force start is withdrawn when the button unmounts', async () => {
    const onStart = vi.fn();
    const view = render(
      <ConfirmProvider><ForceStartButton label="START" disabled={false} ready={false} onStart={onStart} /></ConfirmProvider>,
    );
    fireEvent.mouseDown(view.getByRole('button', { name: 'START' }));
    advance(LONG_PRESS_MS + 32);
    expect(view.getByRole('alertdialog')).toHaveTextContent('monitor.forceStartConfirm');
    view.rerender(<ConfirmProvider>{null}</ConfirmProvider>);
    await settle();
    expect(view.queryByRole('alertdialog')).toBeNull();
    expect(onStart).not.toHaveBeenCalled();
  });
});

describe('launch and start controls hand their unmount signal on (opts.signal)', () => {
  // Later questions of the launch (App's plan camera check) close with the
  // control that began it, not with App.
  it('ForceStartButton: the force start, the click start and the arm carry a signal that aborts on unmount', async () => {
    const onStart = vi.fn();
    const onArm = vi.fn();
    const { wrapper } = confirmStub(true);
    const forced = renderBtn({ ready: false, onStart }, wrapper);
    fireEvent.mouseDown(forced.getByRole('button'));
    advance(LONG_PRESS_MS + 32);
    await settle();
    const { signal } = onStart.mock.calls[0][0];
    expect(signal.aborted).toBe(false);
    forced.unmount();
    expect(signal.aborted).toBe(true);

    const clicked = renderBtn({ ready: true, onStart }, wrapper);
    fireEvent.click(clicked.getByRole('button'));
    expect(onStart.mock.calls[1][0]).toEqual({ signal: expect.any(AbortSignal) });
    clicked.unmount();

    const armed = renderBtn({ ready: true, armReady: true, onStart, onArm }, wrapper);
    fireEvent.mouseDown(armed.getByRole('button'));
    advance(LONG_PRESS_MS + 32);
    const armSignal = onArm.mock.calls[0][0];
    expect(armSignal).toBeInstanceOf(AbortSignal);
    armed.unmount();
    expect(armSignal.aborted).toBe(true);
  });

  it('ForceLaunchButton: ready click and force launch carry the signal', async () => {
    const onLaunch = vi.fn();
    const { wrapper } = confirmStub(true);
    const ready = render(<ForceLaunchButton sysId={4} ready onLaunch={onLaunch} />, { wrapper });
    fireEvent.click(ready.getByRole('button'));
    const { signal } = onLaunch.mock.calls[0][1];
    ready.unmount();
    expect(signal.aborted).toBe(true);

    const forced = render(<ForceLaunchButton sysId={4} ready={false} onLaunch={onLaunch} />, { wrapper });
    fireEvent.mouseDown(forced.getByRole('button'));
    advance(LONG_PRESS_MS + 32);
    await settle();
    expect(onLaunch.mock.calls[1]).toEqual([4, { force: true, signal: expect.any(AbortSignal) }]);
  });
});
