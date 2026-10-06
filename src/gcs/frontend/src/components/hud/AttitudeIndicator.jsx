import React, { useRef, useEffect, useState } from 'react';
import { scheduleRaf, cancelRaf } from '../../utils/rafScheduler';

const WING_COLOR = '#00d2ff';
const LERP = 0.15;

export default React.memo(function AttitudeIndicator({ pitch = 0, roll = 0 }) {
  const canvasRef = useRef(null);
  const [dims, setDims] = useState(null);

  // Smooth display values
  const poiRef = useRef({ p: 0, r: 0 });
  const displayRef = useRef({ p: 0, r: 0 });
  poiRef.current = { p: pitch || 0, r: roll || 0 };

  // Track parent size
  useEffect(() => {
    const c = canvasRef.current;
    if (!c) return;
    const parent = c.parentElement;
    if (!parent) return;
    const obs = new ResizeObserver((entries) => {
      const { width, height } = entries[0].contentRect;
      if (width > 0 && height > 0) setDims({ w: Math.round(width), h: Math.round(height) });
    });
    obs.observe(parent);
    return () => obs.disconnect();
  }, []);

  // Single RAF loop — lerps + draws
  useEffect(() => {
    if (!dims) return;
    const id = 'attitude';
    scheduleRaf(id, () => {
      const d = displayRef.current;
      const t = poiRef.current;
      d.p += (t.p - d.p) * LERP;
      d.r += (t.r - d.r) * LERP;
      draw(canvasRef.current, d.p, d.r, dims.w, dims.h);
    });
    return () => cancelRaf(id);
  }, [dims]);

  useEffect(() => {
    if (!dims) return;
    const c = canvasRef.current;
    if (!c) return;
    const dpr = window.devicePixelRatio || 1;
    c.width = dims.w * dpr;
    c.height = dims.h * dpr;
  }, [dims]);

  return (
    <canvas
      ref={canvasRef}
      style={{ width: '100%', height: '100%' }}
    />
  );
});

function draw(canvas, pitch, roll, w, h) {
  const ctx = canvas?.getContext('2d');
  if (!ctx) return;
  const dpr = window.devicePixelRatio || 1;
  ctx.clearRect(0, 0, w * dpr, h * dpr);
  ctx.save();
  ctx.scale(dpr, dpr);

  const cx = w / 2, cy = h / 2;
  const diag = Math.sqrt(w * w + h * h);
  const pxPerDeg = h / 60;
  const pitchPx = pitch * pxPerDeg;
  const rollRad = (roll * Math.PI) / 180;

  // Rotate around center by -roll
  ctx.translate(cx, cy);
  ctx.rotate(-rollRad);

  // Horizon line — full-width, white
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(-diag, pitchPx);
  ctx.lineTo(diag, pitchPx);
  ctx.stroke();

  // Pitch ladder — extends to the roll arc radius
  const ladderW = Math.min(w, h) * 0.35;
  const fontSize = Math.round(Math.min(w, h) / 25);
  ctx.font = `bold ${fontSize}px sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 1;

  for (const deg of [-30, -20, -10, 10, 20, 30]) {
    const y = pitchPx - deg * pxPerDeg;
    const hw = deg < 0 ? ladderW * 0.7 : ladderW;

    ctx.beginPath();
    ctx.moveTo(-hw, y);
    ctx.lineTo(-hw * 0.3, y);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(hw * 0.3, y);
    ctx.lineTo(hw, y);
    ctx.stroke();

    const labelOff = Math.min(w, h) / 30;
    const lbl = `${deg}`;
    // Dark outline for legibility, then white fill
    ctx.strokeStyle = 'rgba(0,0,0,0.7)';
    ctx.lineWidth = 3;
    ctx.lineJoin = 'round';
    ctx.strokeText(lbl, -hw - labelOff, y);
    ctx.strokeText(lbl, hw + labelOff, y);
    ctx.fillStyle = '#ffffff';
    ctx.fillText(lbl, -hw - labelOff, y);
    ctx.fillText(lbl, hw + labelOff, y);
    // Restore line style for next ladder rung
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1;
  }

  // Roll arc + ticks + labels (rotate with horizon)
  const arcR = Math.min(w, h) * 0.35;
  ctx.strokeStyle = 'rgba(255,255,255,0.5)';
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  ctx.arc(0, 0, arcR, Math.PI + Math.PI / 6, -Math.PI / 6);
  ctx.stroke();

  const rollTicks = [0, 10, 20, 30, 45, 60];
  const rollFontSize = Math.round(Math.min(w, h) / 30);
  ctx.font = `bold ${rollFontSize}px sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  for (const t of rollTicks) {
    for (const sign of [1, -1]) {
      const angle = -Math.PI / 2 + (sign * t * Math.PI) / 180;
      const inner = arcR - (t % 30 === 0 ? 8 : 5);
      ctx.strokeStyle = 'rgba(255,255,255,0.5)';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(Math.cos(angle) * inner, Math.sin(angle) * inner);
      ctx.lineTo(Math.cos(angle) * arcR, Math.sin(angle) * arcR);
      ctx.stroke();

      if (t > 0) {
        const labelR = arcR + rollFontSize * 0.8;
        const lx = Math.cos(angle) * labelR;
        const ly = Math.sin(angle) * labelR;
        ctx.strokeStyle = 'rgba(0,0,0,0.7)';
        ctx.lineWidth = 2;
        ctx.lineJoin = 'round';
        ctx.strokeText(`${t}`, lx, ly);
        ctx.fillStyle = '#ffffff';
        ctx.fillText(`${t}`, lx, ly);
      }
    }
  }

  // Reset rotation — everything below is fixed on screen
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

  // Aircraft wings symbol (fixed center)
  const wingScale = Math.min(w, h) / 200;
  const wingLen = 35 * wingScale;
  const wingGap = 8 * wingScale;

  ctx.strokeStyle = WING_COLOR;
  ctx.fillStyle = WING_COLOR;
  ctx.lineWidth = 2.5;

  // Center dot
  ctx.beginPath();
  ctx.arc(cx, cy, 3, 0, Math.PI * 2);
  ctx.fill();

  // Left wing
  ctx.beginPath();
  ctx.moveTo(cx - wingGap, cy);
  ctx.lineTo(cx - wingGap - wingLen, cy);
  ctx.lineTo(cx - wingGap - wingLen, cy + 6);
  ctx.stroke();

  // Right wing
  ctx.beginPath();
  ctx.moveTo(cx + wingGap, cy);
  ctx.lineTo(cx + wingGap + wingLen, cy);
  ctx.lineTo(cx + wingGap + wingLen, cy + 6);
  ctx.stroke();

  // Roll pointer — fixed triangle at top center, pointing inward
  ctx.fillStyle = WING_COLOR;
  ctx.beginPath();
  ctx.moveTo(cx, cy - arcR + 10);
  ctx.lineTo(cx - 5, cy - arcR);
  ctx.lineTo(cx + 5, cy - arcR);
  ctx.closePath();
  ctx.fill();

  ctx.restore();
}
