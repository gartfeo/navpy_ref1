import React, { useRef, useEffect } from 'react';
import { scheduleRaf, cancelRaf } from '../../utils/rafScheduler';
import { normalizeHeading, headingLabel } from '../../utils/hudDraw';

const POINTER_COLOR = '#00d2ff';
const LERP = 0.15;

/** Shortest-path lerp for heading (handles 359→1 wrap). */
function lerpHeading(current, target) {
  let diff = target - current;
  if (diff > 180) diff -= 360;
  else if (diff < -180) diff += 360;
  return current + diff * LERP;
}

export default React.memo(function HeadingTape({ heading = 0, width = 300, height = 36 }) {
  const canvasRef = useRef(null);
  const poiRef = useRef(0);
  const displayRef = useRef(0);
  poiRef.current = heading || 0;

  useEffect(() => {
    const id = 'heading';
    scheduleRaf(id, () => {
      displayRef.current = lerpHeading(displayRef.current, poiRef.current);
      draw(canvasRef.current, displayRef.current, width, height);
    });
    return () => cancelRaf(id);
  }, [width, height]);

  useEffect(() => {
    const c = canvasRef.current;
    if (!c) return;
    const dpr = window.devicePixelRatio || 1;
    c.width = width * dpr;
    c.height = height * dpr;
  }, [width, height]);

  return (
    <canvas
      ref={canvasRef}
      style={{ width, height }}
    />
  );
});

function draw(canvas, hdgRaw, width, height) {
  const ctx = canvas?.getContext('2d');
  if (!ctx) return;
  const dpr = window.devicePixelRatio || 1;
  ctx.clearRect(0, 0, width * dpr, height * dpr);
  ctx.save();
  ctx.scale(dpr, dpr);

  const hdg = normalizeHeading(hdgRaw);
  const pxPerDeg = width / 60; // 60° visible range
  const cx = width / 2;

  // Draw ticks and labels
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  const tickTop = 14;
  const minorH = 6;
  const majorH = 10;

  for (let d = -40; d <= 40; d++) {
    const deg = normalizeHeading(Math.round(hdg) + d);
    const x = cx + (d - (hdg - Math.round(hdg))) * pxPerDeg;
    if (x < -10 || x > width + 10) continue;

    if (deg % 10 === 0) {
      // Major tick
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(x, tickTop);
      ctx.lineTo(x, tickTop + majorH);
      ctx.stroke();

      // Label with dark outline for legibility
      const lbl = headingLabel(deg);
      ctx.font = 'bold 10px sans-serif';
      ctx.strokeStyle = 'rgba(0,0,0,0.7)';
      ctx.lineWidth = 2;
      ctx.lineJoin = 'round';
      ctx.strokeText(lbl, x, tickTop + majorH + 1);
      ctx.fillStyle = deg % 90 === 0 ? POINTER_COLOR : '#ffffff';
      ctx.fillText(lbl, x, tickTop + majorH + 1);
    } else if (deg % 5 === 0) {
      // Minor tick
      ctx.strokeStyle = 'rgba(255,255,255,0.5)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(x, tickTop);
      ctx.lineTo(x, tickTop + minorH);
      ctx.stroke();
    }
  }

  // Center pointer triangle (pointing down from top)
  ctx.fillStyle = POINTER_COLOR;
  ctx.beginPath();
  ctx.moveTo(cx, 12);
  ctx.lineTo(cx - 5, 2);
  ctx.lineTo(cx + 5, 2);
  ctx.closePath();
  ctx.fill();

  // Current heading readout — outlined text, no background
  const txt = `${String(Math.round(normalizeHeading(hdg))).padStart(3, '0')}\u00B0`;
  ctx.font = 'bold 11px sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  ctx.strokeStyle = 'rgba(0,0,0,0.7)';
  ctx.lineWidth = 3;
  ctx.lineJoin = 'round';
  ctx.strokeText(txt, cx, 1);
  ctx.fillStyle = '#ffffff';
  ctx.fillText(txt, cx, 1);

  ctx.restore();
}
