import { describe, it, expect, vi, beforeEach } from 'vitest';
import { useState } from 'react';
import { render, fireEvent, act, screen } from '@testing-library/react';
import { ForceStartButton } from './LaunchControls';
import ConfirmModal from '../ConfirmModal';
import { LONG_PRESS_MS } from '../../utils/armAction';

// Focused wiring harness for the container-launch guard. It renders the REAL
// ForceStartButton + REAL ConfirmModal + REAL useLongPress and reproduces ONLY
// the container-launch glue from MonitoringSidebar.jsx:
//
//   <ForceStartButton armReady ready={allLaunchReady}
//     onArm={() => setShowLaunchConfirm(true)}
//     onStart={async (opts) => { await launcher.doContainerLaunch(...); }} />   (~L566)
//   {showLaunchConfirm && (
//     <ConfirmModal onConfirm={async () => { setShowLaunchConfirm(false);
//                                            await launcher.doContainerLaunch(...); }}
//                   onCancel={() => setShowLaunchConfirm(false)} /> )}          (~L674)
//
// RESIDUAL GAP (accepted, per plan): this harness verifies the copied
// interaction CONTRACT (click-inert / hold→arm / confirm→launch-once), NOT the
// real parent wiring or the gates around it. Because it copies the glue instead
// of rendering the (heavily-entangled) MonitoringSidebar, it stays green even if
// MonitoringSidebar later:
//   - drops `armReady`, or wires `onArm` to the wrong state;
//   - initializes `showLaunchConfirm` wrong, or stops conditionally rendering
//     the modal, or rewires the modal's Cancel/Confirm;
//   - launches from `onStart` on the ready path, or calls a different launcher
//     method than doContainerLaunch;
//   - renders this button in the wrong phase/mode, or mis-gates `ready`/
//     `disabled`;
//   - drops the `pitotCovered` payload or the post-launch `setPitotCovered`
//     reset (payload/reset are out of this test's scope entirely).
// Keep this harness in lock-step with the MonitoringSidebar source lines above;
// those parent-wiring paths remain unguarded until a full-sidebar test exists.

const ARM_LABEL = 'HOLD TO LAUNCH';   // shown while armed (arm mode) — the button's accessible name
const CONFIRM_LABEL = 'LAUNCH ALL';   // modal confirm button
const CANCEL_NAME = 'confirm.cancel'; // ConfirmModal cancel uses t('confirm.cancel'); i18n stub => key

function ContainerLaunchGuardHarness({ doContainerLaunch, ready = true }) {
  const [showLaunchConfirm, setShowLaunchConfirm] = useState(false);
  return (
    <>
      <ForceStartButton
        label="START"
        armLabel={ARM_LABEL}
        armReady
        ready={ready}
        disabled={false}
        onArm={() => setShowLaunchConfirm(true)}
        onStart={async (opts) => { await doContainerLaunch({ ...opts }); }}
        large
      />
      {showLaunchConfirm && (
        <ConfirmModal
          title="Launch all UAVs?"
          message="Confirm container launch"
          confirmLabel={CONFIRM_LABEL}
          tone="caution"
          onConfirm={async () => {
            setShowLaunchConfirm(false);
            await doContainerLaunch();
          }}
          onCancel={() => setShowLaunchConfirm(false)}
        />
      )}
    </>
  );
}

beforeEach(() => {
  vi.useFakeTimers({ toFake: ['performance', 'requestAnimationFrame', 'cancelAnimationFrame'] });
});

const advance = (ms) => act(() => { vi.advanceTimersByTime(ms); });
const startButton = () => screen.getByRole('button', { name: ARM_LABEL });
const modalConfirm = () => screen.queryByRole('button', { name: CONFIRM_LABEL });

describe('container-launch guard (wiring harness)', () => {
  // #1 — a single click must neither launch nor open the confirm modal.
  it('a single click does not launch and does not open the modal', () => {
    const doContainerLaunch = vi.fn().mockResolvedValue(undefined);
    render(<ContainerLaunchGuardHarness doContainerLaunch={doContainerLaunch} />);

    fireEvent.click(startButton());

    expect(doContainerLaunch).not.toHaveBeenCalled();
    expect(modalConfirm()).toBeNull();
  });

  // #2 — completed hold opens the modal; Cancel is a no-op; Confirm launches once.
  it('hold opens the modal; Cancel does nothing; Confirm launches exactly once', () => {
    const doContainerLaunch = vi.fn().mockResolvedValue(undefined);
    render(<ContainerLaunchGuardHarness doContainerLaunch={doContainerLaunch} />);

    // No modal until a hold completes.
    expect(modalConfirm()).toBeNull();

    // Completed hold opens the modal but does NOT launch yet.
    fireEvent.mouseDown(startButton());
    advance(LONG_PRESS_MS + 32);
    expect(modalConfirm()).not.toBeNull();
    expect(doContainerLaunch).not.toHaveBeenCalled();

    // Cancel closes the modal and launches nothing.
    fireEvent.click(screen.getByRole('button', { name: CANCEL_NAME }));
    expect(modalConfirm()).toBeNull();
    expect(doContainerLaunch).not.toHaveBeenCalled();

    // A fresh hold re-opens the modal.
    fireEvent.mouseDown(startButton());
    advance(LONG_PRESS_MS + 32);
    expect(modalConfirm()).not.toBeNull();

    // Confirm launches exactly once and closes the modal.
    fireEvent.click(modalConfirm());
    expect(doContainerLaunch).toHaveBeenCalledTimes(1);
    expect(modalConfirm()).toBeNull();
  });
});
