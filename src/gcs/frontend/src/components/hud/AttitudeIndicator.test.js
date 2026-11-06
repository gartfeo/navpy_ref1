// One attitude frame (AttitudeIndicator draw) on a recording 2D context.
import { describe, it, expect } from 'vitest';
import { draw } from './AttitudeIndicator';

function recordingCanvas() {
  const calls = [];
  const ctx = new Proxy({}, {
    get: (target, key) => (key in target ? target[key] : (...args) => calls.push([key, ...args])),
    set: (target, key, value) => { target[key] = value; return true; },
  });
  return { calls, canvas: { getContext: () => ctx } };
}

describe('AttitudeIndicator draw', () => {
  it.each([true, false])('draws a frame without throwing (scale %s)', (scale) => {
    const { canvas, calls } = recordingCanvas();
    expect(() => draw(canvas, 5, 10, 300, 200, scale)).not.toThrow();
    expect(calls.some(([k]) => k === 'stroke')).toBe(true);
  });

  it('the roll arc and its pointer only with the scale', () => {
    const arcs = (scale) => {
      const { canvas, calls } = recordingCanvas();
      draw(canvas, 0, 0, 300, 200, scale);
      return calls.filter(([k]) => k === 'arc').length;
    };
    // Without the scale: the centre dot only.
    expect(arcs(false)).toBe(1);
    expect(arcs(true)).toBeGreaterThan(1);
  });
});
