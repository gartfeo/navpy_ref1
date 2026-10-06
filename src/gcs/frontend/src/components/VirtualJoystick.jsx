import React, { useRef, useEffect, useCallback } from 'react';
import { colors } from '../styles';
import { clampToSquare } from '../utils/joystickMath';

/**
 * Canvas-based virtual joystick.
 *
 * Props:
 *   size      - diameter in px (default 150)
 *   onMove    - called with {x, y} each in [-1, 1]
 *   label     - optional text drawn below center
 *   stickyY   - if true, Y axis stays in place on release (for throttle)
 */
export default function VirtualJoystick({ size = 150, onMove, label, stickyY = false, initialY }) {
  const canvasRef = useRef(null);
  const posRef = useRef({ x: 0, y: initialY ?? (stickyY ? -1 : 0) });   // normalised -1..1
  const activeRef = useRef(false);
  const rafRef = useRef(null);
  const mountedRef = useRef(false);

  const half = size / 2;
  const thumbRadius = size * 0.14;
  const baseRadius = half - 2;

  // Send initial position on mount
  useEffect(() => {
    if (!mountedRef.current) {
      mountedRef.current = true;
      onMove?.(posRef.current);
    }
  }, []);

  // ---------- drawing ----------
  const draw = useCallback(() => {
    const ctx = canvasRef.current?.getContext('2d');
    if (!ctx) return;
    const dpr = window.devicePixelRatio || 1;
    ctx.clearRect(0, 0, size * dpr, size * dpr);
    ctx.save();
    ctx.scale(dpr, dpr);

    // Base rounded rectangle
    const corner = 10;
    const bx = half - baseRadius;
    const by = half - baseRadius;
    const bw = baseRadius * 2;
    const bh = baseRadius * 2;
    ctx.beginPath();
    ctx.roundRect(bx, by, bw, bh, corner);
    ctx.fillStyle = 'rgba(22, 33, 62, 0.55)';
    ctx.fill();
    ctx.strokeStyle = colors.border;
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Center reference tick marks at midpoints of each edge
    const tickLen = 8;
    ctx.strokeStyle = 'rgba(255,255,255,0.25)';
    ctx.lineWidth = 1;
    // Top
    ctx.beginPath();
    ctx.moveTo(half, by);
    ctx.lineTo(half, by + tickLen);
    ctx.stroke();
    // Bottom
    ctx.beginPath();
    ctx.moveTo(half, by + bh);
    ctx.lineTo(half, by + bh - tickLen);
    ctx.stroke();
    // Left
    ctx.beginPath();
    ctx.moveTo(bx, half);
    ctx.lineTo(bx + tickLen, half);
    ctx.stroke();
    // Right
    ctx.beginPath();
    ctx.moveTo(bx + bw, half);
    ctx.lineTo(bx + bw - tickLen, half);
    ctx.stroke();

    // Thumb position in pixels
    const tx = half + posRef.current.x * (baseRadius - thumbRadius);
    const ty = half - posRef.current.y * (baseRadius - thumbRadius);

    ctx.beginPath();
    ctx.arc(tx, ty, thumbRadius, 0, Math.PI * 2);
    ctx.fillStyle = activeRef.current ? colors.accent : 'rgba(0, 210, 255, 0.6)';
    ctx.fill();
    ctx.strokeStyle = colors.accent;
    ctx.lineWidth = 1.2;
    ctx.stroke();

    // Label
    if (label) {
      ctx.fillStyle = colors.textDim;
      ctx.font = '10px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(label, half, size - 4);
    }

    ctx.restore();
  }, [size, half, baseRadius, thumbRadius, label]);

  // RAF loop
  useEffect(() => {
    let running = true;
    const loop = () => {
      if (!running) return;
      draw();
      rafRef.current = requestAnimationFrame(loop);
    };
    loop();
    return () => { running = false; cancelAnimationFrame(rafRef.current); };
  }, [draw]);

  // Resize canvas for HiDPI
  useEffect(() => {
    const c = canvasRef.current;
    if (!c) return;
    const dpr = window.devicePixelRatio || 1;
    c.width = size * dpr;
    c.height = size * dpr;
  }, [size]);

  // ---------- pointer events ----------
  const toNorm = useCallback((e) => {
    const rect = canvasRef.current.getBoundingClientRect();
    const px = e.clientX - rect.left;
    const py = e.clientY - rect.top;
    const nx = (px - half) / (baseRadius - thumbRadius);
    const ny = -(py - half) / (baseRadius - thumbRadius);
    return clampToSquare(nx, ny);
  }, [half, baseRadius, thumbRadius]);

  const onPointerDown = useCallback((e) => {
    e.preventDefault();
    canvasRef.current.setPointerCapture(e.pointerId);
    activeRef.current = true;
    const pos = toNorm(e);
    posRef.current = pos;
    onMove?.(pos);
  }, [toNorm, onMove]);

  const onPointerMove = useCallback((e) => {
    if (!activeRef.current) return;
    const pos = toNorm(e);
    posRef.current = pos;
    onMove?.(pos);
  }, [toNorm, onMove]);

  const onPointerUp = useCallback((e) => {
    if (!activeRef.current) return;
    activeRef.current = false;
    try { canvasRef.current?.releasePointerCapture(e.pointerId); } catch (_) {}
    const releasePos = stickyY
      ? { x: 0, y: posRef.current.y }
      : { x: 0, y: 0 };
    posRef.current = releasePos;
    onMove?.(releasePos);
  }, [onMove]);

  return (
    <canvas
      ref={canvasRef}
      style={{
        width: size,
        height: size,
        touchAction: 'none',
        cursor: 'pointer',
      }}
      onPointerDown={onPointerDown}
      onPointerMove={onPointerMove}
      onPointerUp={onPointerUp}
      onPointerCancel={onPointerUp}
    />
  );
}
