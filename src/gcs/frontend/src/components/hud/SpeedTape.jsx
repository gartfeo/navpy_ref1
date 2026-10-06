import React, { useRef, useEffect } from 'react';
import { scheduleRaf, cancelRaf } from '../../utils/rafScheduler';
import { colors } from '../../styles';
import { fmtVal } from '../../utils/hudDraw';

const POINTER_COLOR = '#00d2ff';
const LERP = 0.15;

export default React.memo(function SpeedTape({ airSpeed, groundSpeed, width = 70, height = 200 }) {
  const canvasRef = useRef(null);
  const targetRef = useRef({ as: 0, gs: 0 });
  const displayRef = useRef({ as: 0, gs: 0 });
  targetRef.current = { as: airSpeed ?? 0, gs: groundSpeed ?? 0 };

  useEffect(() => {
    const id = 'speed';
    scheduleRaf(id, () => {
      const d = displayRef.current;
      const t = targetRef.current;
      d.as += (t.as - d.as) * LERP;
      d.gs += (t.gs - d.gs) * LERP;
      draw(canvasRef.current, d.as, d.gs, width, height);
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

function draw(canvas, speed, gs, width, height) {
  const ctx = canvas?.getContext('2d');
  if (!ctx) return;
  const dpr = window.devicePixelRatio || 1;
  ctx.clearRect(0, 0, width * dpr, height * dpr);
  ctx.save();
  ctx.scale(dpr, dpr);

  const pxPerUnit = height / 40; // 40 m/s visible range
  const cy = height / 2;
  const tickX = width - 4; // ticks on right edge

  // Draw ticks
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';

  const minSpeed = speed - 20;
  const maxSpeed = speed + 20;
  const startTick = Math.floor(minSpeed / 5) * 5;
  const endTick = Math.ceil(maxSpeed / 5) * 5;

  for (let v = startTick; v <= endTick; v += 5) {
    const y = cy - (v - speed) * pxPerUnit;
    if (y < -5 || y > height + 5) continue;

    if (v % 10 === 0) {
      // Major tick + label
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(tickX, y);
      ctx.lineTo(tickX - 10, y);
      ctx.stroke();

      ctx.font = '10px sans-serif';
      ctx.strokeStyle = 'rgba(0,0,0,0.7)';
      ctx.lineWidth = 2;
      ctx.lineJoin = 'round';
      ctx.strokeText(String(v), tickX - 13, y);
      ctx.fillStyle = '#ffffff';
      ctx.fillText(String(v), tickX - 13, y);
    } else {
      // Minor tick
      ctx.strokeStyle = 'rgba(255,255,255,0.4)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(tickX, y);
      ctx.lineTo(tickX - 6, y);
      ctx.stroke();
    }
  }

  // Center pointer — right-pointing triangle on right edge
  ctx.fillStyle = POINTER_COLOR;
  ctx.beginPath();
  ctx.moveTo(width, cy);
  ctx.lineTo(width - 8, cy - 5);
  ctx.lineTo(width - 8, cy + 5);
  ctx.closePath();
  ctx.fill();

  // Current speed readout — outlined text with unit
  const txt = `${fmtVal(speed, 0)} m/s`;
  ctx.font = 'bold 13px sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.strokeStyle = 'rgba(0,0,0,0.7)';
  ctx.lineWidth = 3;
  ctx.lineJoin = 'round';
  ctx.strokeText(txt, width / 2, cy);
  ctx.fillStyle = '#ffffff';
  ctx.fillText(txt, width / 2, cy);

  // GS readout at bottom — outlined text
  ctx.font = '10px sans-serif';
  ctx.textAlign = 'center';
  ctx.strokeStyle = 'rgba(0,0,0,0.7)';
  ctx.lineWidth = 2;
  ctx.lineJoin = 'round';
  ctx.strokeText(`GS ${fmtVal(gs, 0)} m/s`, width / 2, height - 8);
  ctx.fillStyle = colors.textDim;
  ctx.fillText(`GS ${fmtVal(gs, 0)} m/s`, width / 2, height - 8);

  ctx.restore();
}
