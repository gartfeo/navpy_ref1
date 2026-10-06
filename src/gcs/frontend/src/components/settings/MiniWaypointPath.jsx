import React, { useState, useRef, useEffect } from 'react';
import { useTranslation } from 'react-i18next';
import { colors, zoneColorsSolid } from '../../styles';

export default function MiniWaypointPath({ vehicleMissions, vehicleList, selectedVehicles, poisMap, navLastWpMap, onToggle, onDownload, downloading, tall }) {
  const { t } = useTranslation();
  const selected = selectedVehicles || new Set();
  const visibleMissions = Object.entries(vehicleMissions || {}).filter(([sid]) => selected.has(Number(sid)));
  const hasMissions = visibleMissions.some(([, wps]) => wps.length > 0);

  if (!hasMissions) {
    return (
      <div style={{ display: 'flex', alignItems: 'center', gap: 10, padding: '8px 0' }}>
        <span style={{ fontSize: 11, color: colors.textDim, fontStyle: 'italic' }}>{t('vehicle.noMissionLoaded')}</span>
        {vehicleList.length > 0 && (
          <button
            onClick={onDownload}
            disabled={downloading}
            style={{
              padding: '3px 10px', fontSize: 11, fontWeight: 600,
              background: 'transparent', border: `1px solid ${colors.border}`,
              borderRadius: 4, color: colors.accent,
              cursor: downloading ? 'wait' : 'pointer',
            }}
          >
            {downloading ? t('connection.downloading') : t('vehicle.downloadMissions')}
          </button>
        )}
      </div>
    );
  }

  const SVG_W = 600;
  const SVG_H = tall ? 350 : 100;
  const PAD = 24;

  const allWps = visibleMissions.flatMap(([, wps]) => wps);
  const lats = allWps.map((w) => w.lat);
  const lons = allWps.map((w) => w.lon);
  const minLat = Math.min(...lats);
  const maxLat = Math.max(...lats);
  const minLon = Math.min(...lons);
  const maxLon = Math.max(...lons);
  const latRange = maxLat - minLat || 0.001;
  const lonRange = maxLon - minLon || 0.001;

  const project = (lat, lon) => ({
    x: PAD + ((lon - minLon) / lonRange) * (SVG_W - 2 * PAD),
    y: PAD + ((maxLat - lat) / latRange) * (SVG_H - 2 * PAD),
  });

  const vIdx = {};
  vehicleList.forEach((v, i) => { vIdx[v.sys_id] = i; });

  const svgRef = useRef(null);
  const [vb, setVb] = useState({ x: 0, y: 0, w: SVG_W, h: SVG_H });
  const dragRef = useRef(null);
  const wasDragRef = useRef(false);

  useEffect(() => { setVb({ x: 0, y: 0, w: SVG_W, h: SVG_H }); }, [visibleMissions.length, SVG_H]);

  // Register non-passive wheel listener so preventDefault works
  useEffect(() => {
    if (!tall) return;
    const svg = svgRef.current;
    if (!svg) return;
    const handler = (e) => {
      e.preventDefault();
      const rect = svg.getBoundingClientRect();
      const fx = (e.clientX - rect.left) / rect.width;
      const fy = (e.clientY - rect.top) / rect.height;
      const factor = e.deltaY > 0 ? 1.15 : 1 / 1.15;
      setVb((prev) => {
        const nw = Math.max(20, Math.min(SVG_W, prev.w * factor));
        const nh = Math.max(12, Math.min(SVG_H, prev.h * factor));
        return { x: prev.x + (prev.w - nw) * fx, y: prev.y + (prev.h - nh) * fy, w: nw, h: nh };
      });
    };
    svg.addEventListener('wheel', handler, { passive: false });
    return () => svg.removeEventListener('wheel', handler);
  }, [tall, SVG_W, SVG_H]);

  const handleMouseDown = tall ? (e) => {
    if (e.button !== 0) return;
    wasDragRef.current = false;
    dragRef.current = { px: e.clientX, py: e.clientY };
  } : undefined;

  const handleMouseMove = tall ? (e) => {
    if (!dragRef.current) return;
    const dx = e.clientX - dragRef.current.px;
    const dy = e.clientY - dragRef.current.py;
    if (Math.abs(dx) > 2 || Math.abs(dy) > 2) wasDragRef.current = true;
    if (!wasDragRef.current) return;
    const svg = svgRef.current;
    if (!svg) return;
    const rect = svg.getBoundingClientRect();
    dragRef.current.px = e.clientX;
    dragRef.current.py = e.clientY;
    setVb((prev) => ({
      ...prev,
      x: prev.x - (dx / rect.width) * prev.w,
      y: prev.y - (dy / rect.height) * prev.h,
    }));
  } : undefined;

  const handleMouseUp = tall ? () => { dragRef.current = null; } : undefined;

  return (
    <>
    <svg
      ref={svgRef}
      width="100%"
      viewBox={tall ? `${vb.x} ${vb.y} ${vb.w} ${vb.h}` : `0 0 ${SVG_W} ${SVG_H}`}
      style={{ background: 'rgba(0,0,0,0.2)', borderRadius: 4, display: 'block', cursor: tall ? 'grab' : 'default' }}
      onMouseDown={handleMouseDown}
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
      onMouseLeave={handleMouseUp}
    >
      {(() => {
        const s = tall ? vb.w / SVG_W : 1;
        const sw = 1.5 * s;
        return visibleMissions.map(([sysId, wps]) => {
          const idx = vIdx[sysId] ?? 0;
          const clr = zoneColorsSolid[idx % zoneColorsSolid.length];
          const pts = wps.map((w) => project(w.lat, w.lon));
          const str = pts.map((p) => `${p.x},${p.y}`).join(' ');
          const mask = parseInt(poisMap?.[sysId] ?? poisMap?.[Object.keys(poisMap || {})[0]]) || 0;
          const lastWp = parseInt(navLastWpMap?.[sysId] ?? navLastWpMap?.[Object.keys(navLastWpMap || {})[0]]) || 0;
          return (
            <g key={sysId}>
              <polyline points={str} fill="none" stroke={clr} strokeWidth={sw} strokeOpacity={0.6} />
              {pts.map((p, i) => {
                const selectable = i <= 23;
                const isPoi = selectable && (mask & (1 << i)) !== 0;
                const isActive = i >= lastWp && selectable;
                const r = (tall ? (isPoi ? 10 : 7) : (isPoi ? 7 : 5)) * s;
                const fs = (tall ? 11 : 9) * s;
                const tOff = (tall ? 14 : 10) * s;
                return (
                  <g key={i} style={{ cursor: selectable ? 'pointer' : 'default' }} onClick={() => selectable && !wasDragRef.current && onToggle(Number(sysId), i)}>
                    <circle cx={p.x} cy={p.y} r={r + 6 * s} fill="transparent" />
                    <circle
                      cx={p.x} cy={p.y}
                      r={r}
                      fill={isPoi ? colors.warning : clr}
                      fillOpacity={isPoi ? 0.4 : (!selectable ? 0.04 : (isActive ? 0.12 : 0.06))}
                      stroke={isActive ? clr : 'rgba(255,255,255,0.1)'}
                      strokeWidth={sw}
                    />
                    <text
                      x={p.x} y={p.y - tOff}
                      textAnchor="middle"
                      fontSize={fs}
                      fill={isPoi ? colors.warning : (isActive ? clr : 'rgba(255,255,255,0.2)')}
                      style={{ pointerEvents: 'none', userSelect: 'none' }}
                    >
                      {i + 1}
                    </text>
                  </g>
                );
              })}
            </g>
          );
        });
      })()}
    </svg>
  </>
  );
}
