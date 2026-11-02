import { describe, it, expect } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import useSidePanel from './useSidePanel';

// docs/design/gcs_handheld_layout.md: only the operator's own tap hides the
// panel; a resize or rotation never does.

const setup = (width) => renderHook(({ w }) => useSidePanel(w), { initialProps: { w: width } });

describe('useSidePanel', () => {
  it('starts docked at 695 px and collapsed at 694', () => {
    expect(setup(695).result.current).toMatchObject({ open: true, mode: 'docked' });
    expect(setup(694).result.current).toMatchObject({ open: false, mode: 'collapsed' });
  });

  it('the toggle flips it at every width', () => {
    const { result } = setup(1920);
    act(() => result.current.toggle());
    expect(result.current).toMatchObject({ open: false, mode: 'collapsed' });
    act(() => result.current.toggle());
    expect(result.current).toMatchObject({ open: true, mode: 'docked' });

    const phone = setup(375);
    act(() => phone.result.current.toggle());
    expect(phone.result.current).toMatchObject({ open: true, mode: 'overlay' });
  });

  it('crossing down keeps an open panel open: a rotation never hides it', () => {
    const { result, rerender } = setup(812);
    rerender({ w: 375 });
    expect(result.current).toMatchObject({ open: true, mode: 'overlay' });
  });

  it('crossing up opens it: a phone rotated from portrait docks', () => {
    const { result, rerender } = setup(375);
    expect(result.current.open).toBe(false);
    rerender({ w: 812 });
    expect(result.current).toMatchObject({ open: true, mode: 'docked' });
  });

  it('a resize that crosses nothing keeps a hand-collapsed panel collapsed', () => {
    const { result, rerender } = setup(1920);
    act(() => result.current.toggle());
    rerender({ w: 960 });
    expect(result.current.mode).toBe('collapsed');

    const phone = setup(375);
    act(() => phone.result.current.toggle());
    act(() => phone.result.current.toggle());
    phone.rerender({ w: 412 });
    expect(phone.result.current.mode).toBe('collapsed');
  });

  it('a hand-collapsed panel stays collapsed when crossing down, and opens on the way up', () => {
    const { result, rerender } = setup(960);
    act(() => result.current.toggle());
    rerender({ w: 600 });
    expect(result.current.mode).toBe('collapsed');
    rerender({ w: 960 });
    expect(result.current.mode).toBe('docked');
  });
});
