// The in-page confirm that replaces window.confirm (todo 233): it answers
// through a promise, never calls the native dialog, shows one question at a
// time and never answers a question twice.
import React from 'react';
import { describe, it, expect, vi } from 'vitest';
import { act, fireEvent, render, renderHook, screen } from '@testing-library/react';
import ConfirmProvider from './ConfirmProvider';
import useConfirm from '../hooks/useConfirm';

function setup() {
  const native = vi.spyOn(window, 'confirm');
  const { result } = renderHook(() => useConfirm(), { wrapper: ConfirmProvider });
  return { confirm: result.current, native };
}

const dialog = () => screen.queryByRole('alertdialog');
const ask = (confirm, message) => {
  let answer;
  act(() => { answer = confirm(message); });
  return answer;
};

describe('ConfirmProvider', () => {
  it('resolves true on Confirm and never opens the native dialog', async () => {
    const { confirm, native } = setup();
    const answer = ask(confirm, 'Arm vehicle 7?');
    expect(dialog()).toHaveTextContent('Arm vehicle 7?');
    fireEvent.click(screen.getByText('confirm.ok'));
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

  it('shows queued questions one at a time, in order', async () => {
    const { confirm } = setup();
    const first = ask(confirm, 'first?');
    const second = ask(confirm, 'second?');
    expect(screen.getAllByRole('alertdialog')).toHaveLength(1);
    expect(dialog()).toHaveTextContent('first?');
    fireEvent.click(screen.getByText('confirm.ok'));
    await expect(first).resolves.toBe(true);
    expect(dialog()).toHaveTextContent('second?');
    fireEvent.click(screen.getByText('confirm.cancel'));
    await expect(second).resolves.toBe(false);
  });

  it('a double tap answers only the question it was shown for', async () => {
    const { confirm } = setup();
    const first = ask(confirm, 'first?');
    const second = ask(confirm, 'second?');
    const ok = screen.getByText('confirm.ok');
    fireEvent.click(ok);
    fireEvent.click(ok); // the same, now stale, button
    await expect(first).resolves.toBe(true);
    expect(dialog()).toHaveTextContent('second?');
    fireEvent.click(screen.getByText('confirm.cancel'));
    await expect(second).resolves.toBe(false);
  });

  it('the same question asked again while pending resolves false at once', async () => {
    const { confirm } = setup();
    const first = ask(confirm, 'Exit RC?');
    const again = ask(confirm, 'Exit RC?');
    await expect(again).resolves.toBe(false);
    expect(screen.getAllByRole('alertdialog')).toHaveLength(1);
    fireEvent.click(screen.getByText('confirm.ok'));
    await expect(first).resolves.toBe(true);
    // Answered, the question can be asked again.
    const later = ask(confirm, 'Exit RC?');
    expect(dialog()).toHaveTextContent('Exit RC?');
    fireEvent.click(screen.getByText('confirm.ok'));
    await expect(later).resolves.toBe(true);
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
