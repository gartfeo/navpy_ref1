import React, { useRef, useEffect } from 'react';
import { scheduleRaf, cancelRaf } from '../../utils/rafScheduler';
import { colors } from '../../styles';
import { fmtVal } from '../../utils/hudDraw';

const POINTER_COLOR = '#00d2ff';
const LERP = 0.15;

export default React.memo(function AltitudeTape({ altitude, width = 70, height = 200 }) {
  const canvasRef = useRef(null);
  const poiRef = useRef(0);
  const displayRef = useRef(0);
  poiRef.current = altitude ?? 0;

  useEffect(() => {
    const id = 'altitude';
    scheduleRaf(id, () => {
      displayRef.current += (poiRef.current - displayRef.current) * LERP;
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

function draw(canvas, alt, width, height) {
  const ctx = canvas?.getContext('2d');
  if (!ctx) return;
  const dpr = window.devicePixelRatio || 1;
  ctx.clearRect(0, 0, width * dpr, height * dpr);
  ctx.save();
  ctx.scale(dpr, dpr);

  const pxPerUnit = height / 100; // 100m visible range
  const cy = height / 2;
  const tickX = 4; // ticks on left edge

  // Draw ticks
  ctx.textAlign = 'left';
  ctx.textBaseline = 'middle';

  const minAlt = alt - 50;
  const maxAlt = alt + 50;
  const startTick = Math.floor(minAlt / 10) * 10;
  const endTick = Math.ceil(maxAlt / 10) * 10;

  for (let v = startTick; v <= endTick; v += 10) {
    const y = cy - (v - alt) * pxPerUnit;
    if (y < -5 || y > height + 5) continue;

    if (v % 20 === 0) {
      // Major tick + label
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(tickX, y);
      ctx.lineTo(tickX + 10, y);
      ctx.stroke();

      ctx.font = '10px sans-serif';
      ctx.strokeStyle = 'rgba(0,0,0,0.7)';
      ctx.lineWidth = 2;
      ctx.lineJoin = 'round';
      ctx.strokeText(String(v), tickX + 13, y);
      ctx.fillStyle = '#ffffff';
      ctx.fillText(String(v), tickX + 13, y);
    } else {
      // Minor tick
      ctx.strokeStyle = 'rgba(255,255,255,0.4)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(tickX, y);
      ctx.lineTo(tickX + 6, y);
      ctx.stroke();
    }
  }

  // Center pointer — left-pointing triangle on left edge
  ctx.fillStyle = POINTER_COLOR;
  ctx.beginPath();
  ctx.moveTo(0, cy);
  ctx.lineTo(8, cy - 5);
  ctx.lineTo(8, cy + 5);
  ctx.closePath();
  ctx.fill();

  // Current altitude readout — outlined text with unit
  const txt = `${fmtVal(alt, 0)} m`;
  ctx.font = 'bold 13px sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.strokeStyle = 'rgba(0,0,0,0.7)';
  ctx.lineWidth = 3;
  ctx.lineJoin = 'round';
  ctx.strokeText(txt, width / 2, cy);
  ctx.fillStyle = '#ffffff';
  ctx.fillText(txt, width / 2, cy);

  ctx.restore();
}
