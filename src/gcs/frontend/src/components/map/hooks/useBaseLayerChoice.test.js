import { renderHook, act, waitFor } from '@testing-library/react';
import { beforeEach, describe, expect, it, vi } from 'vitest';
import useBaseLayerChoice from './useBaseLayerChoice';
import { BASE_LAYER_KEY } from '../utils/baseLayers';

const tokenLoader = (token) => vi.fn(() => Promise.resolve(token));

describe('useBaseLayerChoice', () => {
  beforeEach(() => localStorage.clear());

  it('starts on terrain and enables satellite once the backend has a token', async () => {
    const { result } = renderHook(() => useBaseLayerChoice(tokenLoader('abc')));
    expect(result.current.baseLayer).toBe('terrain');
    await waitFor(() => expect(result.current.hasIon).toBe(true));
  });

  it('keeps a saved satellite choice when a token is served', async () => {
    localStorage.setItem(BASE_LAYER_KEY, 'satellite');
    const { result } = renderHook(() => useBaseLayerChoice(tokenLoader('abc')));
    await waitFor(() => expect(result.current.hasIon).toBe(true));
    expect(result.current.baseLayer).toBe('satellite');
  });

  it('drops a saved satellite choice when no token is served, without overwriting it', async () => {
    localStorage.setItem(BASE_LAYER_KEY, 'satellite');
    const loader = tokenLoader(null);
    const { result } = renderHook(() => useBaseLayerChoice(loader));
    await waitFor(() => expect(loader).toHaveBeenCalled());
    await waitFor(() => expect(result.current.baseLayer).toBe('terrain'));
    expect(result.current.hasIon).toBe(false);
    expect(localStorage.getItem(BASE_LAYER_KEY)).toBe('satellite');
  });

  it('saves a pick and falls back to terrain when that layer fails', async () => {
    vi.spyOn(console, 'warn').mockImplementation(() => {});
    const { result } = renderHook(() => useBaseLayerChoice(tokenLoader('abc')));
    await waitFor(() => expect(result.current.hasIon).toBe(true));
    act(() => result.current.onChange('satellite'));
    expect(localStorage.getItem(BASE_LAYER_KEY)).toBe('satellite');
    act(() => result.current.onFailed('satellite', new Error('401')));
    expect(result.current.baseLayer).toBe('terrain');
    expect(localStorage.getItem(BASE_LAYER_KEY)).toBe('satellite');
  });
});
