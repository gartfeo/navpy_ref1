// The in-page confirm that replaces window.confirm (todo 233): it answers
// through a promise, never calls the native dialog, shows one question at a
// time, never answers a question twice, takes no yes from a double tap, and
// never leaves a question without an answer.
import React from 'react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { act, fireEvent, render, renderHook, screen } from '@testing-library/react';
import ConfirmProvider, { CONFIRM_ARM_MS } from './ConfirmProvider';
import useConfirm from '../hooks/useConfirm';

beforeEach(() => {
  vi.useFakeTimers({ toFake: ['setTimeout', 'clearTimeout'] });
});

function setup() {
  const native = vi.spyOn(window, 'confirm');
  const hook = renderHook(() => useConfirm(), { wrapper: ConfirmProvider });
  return { confirm: hook.result.current, native, hook };
}

const dialog = () => screen.queryByRole('alertdialog');
const okButton = () => screen.getByText('confirm.ok');
const arm = () => act(() => { vi.advanceTimersByTime(CONFIRM_ARM_MS); });
const ask = (confirm, message) => {
  let answer;
  act(() => { answer = confirm(message); });
  return answer;
};
// Records whether a promise has settled, without awaiting it.
const track = (p) => {
  const state = { settled: false, value: undefined };
  p.then((v) => { state.settled = true; state.value = v; });
  return state;
};
const flush = () => act(async () => {});

describe('ConfirmProvider', () => {
  it('resolves true on Confirm and never opens the native dialog', async () => {
    const { confirm, native } = setup();
    const answer = ask(confirm, 'Arm vehicle 7?');
    expect(dialog()).toHaveTextContent('Arm vehicle 7?');
    arm();
    fireEvent.click(okButton());
    await expect(answer).resolves.toBe(true);
    expect(dialog()).toBeNull();
    expect(native).not.toHaveBeenCalled();
  });

  it('resolves false on Cancel and on a tap outside the dialog', async () => {
    const { confirm } = setup();
    const cancelled = ask(confirm, 'first?');
    fireEvent.click(screen.getByText('confirm.cancel'));
    await expect(cancelled).resolves.toBe(false);

    const outside = ask(confirm, 'second?');
    fireEvent.click(dialog().parentElement);
    await expect(outside).resolves.toBe(false);
    expect(dialog()).toBeNull();
  });

  it('resolves false on Escape', async () => {
    const { confirm } = setup();
    const answer = ask(confirm, 'Arm vehicle 7?');
    fireEvent.keyDown(window, { key: 'Escape' });
    await expect(answer).resolves.toBe(false);
    expect(dialog()).toBeNull();
  });

  it('shows queued questions one at a time, in order', async () => {
    const { confirm } = setup();
    const first = ask(confirm, 'first?');
    const second = ask(confirm, 'second?');
    expect(screen.getAllByRole('alertdialog')).toHaveLength(1);
    expect(dialog()).toHaveTextContent('first?');
    arm();
    fireEvent.click(okButton());
    await expect(first).resolves.toBe(true);
    expect(dialog()).toHaveTextContent('second?');
    fireEvent.click(screen.getByText('confirm.cancel'));
    await expect(second).resolves.toBe(false);
  });

  it('a double tap on the shown Confirm accepts only the first question', async () => {
    const { confirm } = setup();
    const first = track(ask(confirm, 'first?'));
    const second = track(ask(confirm, 'second?'));
    arm();
    fireEvent.click(okButton());
    fireEvent.click(okButton()); // the second tap lands on the next question's Confirm
    await flush();
    expect(first).toEqual({ settled: true, value: true });
    expect(second.settled).toBe(false);
    expect(dialog()).toHaveTextContent('second?');
    // A deliberate tap after the double-tap window still answers it.
    arm();
    fireEvent.click(okButton());
    await flush();
    expect(second).toEqual({ settled: true, value: true });
  });

  it('a new question takes no Confirm within the double-tap window', async () => {
    const { confirm } = setup();
    const answer = track(ask(confirm, 'Clear all hubs?'));
    act(() => { vi.advanceTimersByTime(CONFIRM_ARM_MS - 1); });
    fireEvent.click(okButton());
    await flush();
    expect(answer.settled).toBe(false);
    expect(okButton()).toBeDisabled();
    act(() => { vi.advanceTimersByTime(1); });
    expect(okButton()).not.toBeDisabled();
  });

  it('the same question asked again while pending resolves false at once', async () => {
    const { confirm } = setup();
    const first = ask(confirm, 'Exit RC?');
    const again = ask(confirm, 'Exit RC?');
    await expect(again).resolves.toBe(false);
    expect(screen.getAllByRole('alertdialog')).toHaveLength(1);
    arm();
    fireEvent.click(okButton());
    await expect(first).resolves.toBe(true);
    // Answered, the question can be asked again.
    const later = ask(confirm, 'Exit RC?');
    expect(dialog()).toHaveTextContent('Exit RC?');
    arm();
    fireEvent.click(okButton());
    await expect(later).resolves.toBe(true);
  });

  it('a question resolves false and closes when the asking component unmounts', async () => {
    function Asker({ onAnswer }) {
      const confirm = useConfirm();
      return <button onClick={async () => onAnswer(await confirm('Force start?'))}>ask</button>;
    }
    const onAnswer = vi.fn();
    const view = render(
      <ConfirmProvider>
        <Asker onAnswer={onAnswer} />
      </ConfirmProvider>,
    );
    fireEvent.click(screen.getByText('ask'));
    expect(dialog()).toHaveTextContent('Force start?');
    view.rerender(<ConfirmProvider>{null}</ConfirmProvider>);
    await flush();
    expect(onAnswer).toHaveBeenCalledWith(false);
    expect(dialog()).toBeNull();
  });

  it('unmounting the provider resolves every pending question false', async () => {
    const { confirm, hook } = setup();
    const first = ask(confirm, 'first?');
    const second = ask(confirm, 'second?');
    hook.unmount();
    await expect(first).resolves.toBe(false);
    await expect(second).resolves.toBe(false);
  });

  it('keeps line breaks of long messages', () => {
    const { confirm } = setup();
    ask(confirm, 'line one\nline two');
    expect(screen.getByText(/line one/).style.whiteSpace).toBe('pre-line');
  });

  it('outside a provider confirm() rejects instead of using the native dialog', async () => {
    const native = vi.spyOn(window, 'confirm');
    const { result } = renderHook(() => useConfirm());
    await expect(result.current('Arm?')).rejects.toThrow('ConfirmProvider');
    expect(native).not.toHaveBeenCalled();
  });
});
