import React from 'react';
import { useTranslation } from 'react-i18next';
import { colors } from '../../styles';
import { cameraHitsGround } from '../map/utils/cameraFootprint';

const SIDE_W = 380;
const TOP_W = 260;
const H = 170;
const PAD_L = 40;
const PAD_R = 10;
const PAD_T = 28;
const PAD_B = 24;
const SIDE_DW = SIDE_W - PAD_L - PAD_R;
const SIDE_DH = H - PAD_T - PAD_B;
const TOP_PL = 10;
const TOP_PR = 10;
const TOP_DW = TOP_W - TOP_PL - TOP_PR;
const TOP_DH = H - PAD_T - PAD_B;
const UAV_R = 5;
export const DEVICE_COLORS = [colors.accent, colors.warning, '#AA44FF', '#44AA44'];

/**
 * Side-view + top-view SVG diagram showing camera FOV cones and ground footprints.
 * Both views use uniform scaling so angles are geometrically correct.
 */
export default function DetectionRangeDiagram({ devices, altitude }) {
  const { t } = useTranslation();

  if (!devices || devices.length === 0 || altitude <= 0) return null;

  // === Side view: ray endpoint in world coords (forward, height) ===
  const sideRay = (elevation, mdd) => {
    let t = mdd;
    if (elevation < -1e-6) t = Math.min(mdd, altitude / Math.sin(-elevation));
    return [t * Math.cos(elevation), altitude + t * Math.sin(elevation)];
  };

  // === Top view: corner ray ground projection (forward, lateral) ===
  // Camera at (0,0,altitude), boresight at pitch θ from horizontal.
  // Corner (u,v): u=±1 horizontal, v=±1 vertical (-1=upper/far, +1=lower/near).
  const topCorner = (u, v, pitch, halfH, halfV, mdd) => {
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    const dx = cp + v * Math.tan(halfV) * sp;
    const dy = u * Math.tan(halfH);
    const dz = sp - v * Math.tan(halfV) * cp;
    const len = Math.sqrt(dx * dx + dy * dy + dz * dz);
    const nx = dx / len, ny = dy / len, nz = dz / len;
    let t = mdd;
    if (nz < -1e-6) t = Math.min(mdd, altitude / (-nz));
    return [t * nx, t * ny];
  };

  // === Per-device data for both views ===
  const devData = devices.map((dev) => {
    const p = dev.pitchDeg * Math.PI / 180;
    const hv = dev.fovV / 2;
    const hh = (dev.fovH || dev.fovV * 1.5) / 2;
    const mdd = dev.maxDetectDist;

    const upper = sideRay(p + hv, mdd);
    const lower = sideRay(p - hv, mdd);

    // Top view quadrilateral: far-left, far-right, near-right, near-left
    const corners = [
      topCorner(-1, -1, p, hh, hv, mdd),
      topCorner(+1, -1, p, hh, hv, mdd),
      topCorner(+1, +1, p, hh, hv, mdd),
      topCorner(-1, +1, p, hh, hv, mdd),
    ];

    // Shared with the live map footprint so both agree on whether the camera
    // reaches the ground (a near-horizontal forward camera fails this in both).
    const hitsGround = cameraHitsGround(dev.pitchDeg, dev.fovV, altitude, mdd);
    return { dev, upper, lower, mdd, corners, hitsGround };
  });

  // === Side view scale ===
  const sideEnds = devData.flatMap((d) => [d.upper, d.lower]);
  const sideXMax = Math.max(...sideEnds.map((e) => e[0]), altitude * 0.3);
  const sideYMax = Math.max(altitude, ...sideEnds.map((e) => e[1]));
  const sideScale = Math.min(SIDE_DW / sideXMax, SIDE_DH / sideYMax);

  const sSX = (wx) => PAD_L + wx * sideScale;
  const sSY = (wy) => PAD_T + (sideYMax - wy) * sideScale;
  const uavSX = sSX(0), uavSY = sSY(altitude), gndSY = sSY(0);

  // === Top view scale (only devices that see the ground) ===
  const allC = devData.filter((d) => d.hitsGround).flatMap((d) => d.corners);
  const topXMax = Math.max(...allC.map((c) => c[0]), 1);
  const topYMax = Math.max(...allC.map((c) => Math.abs(c[1])), 1);
  const topScale = Math.min(TOP_DW / topXMax, TOP_DH / 2 / topYMax);

  const tSX = (wx) => TOP_PL + wx * topScale;
  const tSY = (wy) => PAD_T + TOP_DH / 2 - wy * topScale;
  const uavTX = tSX(0), uavTY = tSY(0);

  // === Build elements (draw primary last = on top) ===
  const sorted = [...devData].sort((a, b) => (a.dev.primary ? 1 : 0) - (b.dev.primary ? 1 : 0));

  const sideFov = [];
  const sideRange = [];
  const sideFovLabels = [];
  const topFov = [];
  const topRange = [];
  const topFovLabels = [];

  sorted.forEach(({ dev, upper, lower, mdd, corners, hitsGround }) => {
    const ci = devices.indexOf(dev);
    const color = DEVICE_COLORS[ci % DEVICE_COLORS.length];

    // --- Side view FOV polygon ---
    // Gate the far ground vertex (and, transitively, the range markers below,
    // which key off pt[1] < 0.5) on the SAME hitsGround predicate as the top
    // view and the live map, so all three flip at one ground-reach threshold.
    const ptsW = [[0, altitude], upper];
    if (hitsGround && upper[1] > 0.5 && altitude < mdd) {
      const gx = Math.sqrt(mdd * mdd - altitude * altitude);
      if (gx > lower[0]) ptsW.push([gx, 0]);
    }
    ptsW.push(lower);

    sideFov.push(
      <path key={`sf-${dev.name}`}
        d={ptsW.map((pt, j) => `${j ? 'L' : 'M'}${sSX(pt[0]).toFixed(1)},${sSY(pt[1]).toFixed(1)}`).join(' ') + ' Z'}
        fill={color} fillOpacity={0.18} stroke={color} strokeWidth={1.2} strokeOpacity={0.6} />
    );

    // --- Side view: vertical FOV label on upper ray ---
    const fovVDeg = dev.fovV * 180 / Math.PI;
    const sideMidX = sSX(upper[0] * 0.55);
    const sideMidY = sSY(upper[1] * 0.55 + altitude * 0.45);
    sideFovLabels.push(
      <text key={`flv-${dev.name}`} x={sideMidX} y={sideMidY}
        fill={color} fontSize={10} textAnchor="start" opacity={0.85}>
        {`${fovVDeg.toFixed(1)}\u00B0`}
      </text>
    );

    // --- Side view: boresight line drawn to its GROUND intersection (the camera's
    // aim point), labeled with the slant range to that point — not a constant. ---
    const p = dev.pitchDeg * Math.PI / 180;
    const boreSlant = p < -1e-6 ? altitude / Math.sin(-p) : mdd;   // slant range to where the boresight meets the ground
    const boreEnd = p < -1e-6 ? [boreSlant * Math.cos(p), 0] : sideRay(p, mdd);
    const boreMidX = boreEnd[0] / 2;
    const boreMidY = (altitude + boreEnd[1]) / 2;
    const boreLabel = boreSlant >= 1000 ? `${(boreSlant / 1000).toFixed(1)}km` : `${Math.round(boreSlant)}m`;
    sideFov.push(
      <line key={`mddl-${dev.name}`}
        x1={sSX(0)} y1={sSY(altitude)} x2={sSX(boreEnd[0])} y2={sSY(boreEnd[1])}
        stroke={color} strokeWidth={1} strokeDasharray="3,3" opacity={0.7} />
    );
    sideFovLabels.push(
      <text key={`mdd-${dev.name}`} x={sSX(boreMidX)} y={sSY(boreMidY) - 4}
        fill={color} fontSize={9} fontWeight={600} textAnchor="middle" opacity={0.95}>
        {boreLabel}
      </text>
    );

    // --- Side view range markers (near + far ground intersection) ---
    let nearGX = Infinity, farGX = 0;
    ptsW.forEach((pt) => {
      if (pt[1] < 0.5) {
        farGX = Math.max(farGX, pt[0]);
        nearGX = Math.min(nearGX, pt[0]);
      }
    });
    const fmtDist = (d) => d >= 1000 ? `${(d / 1000).toFixed(1)}km` : `${d}m`;
    if (farGX > 0) {
      const mx = sSX(farGX);
      if (mx < PAD_L + SIDE_DW + 5) {
        sideRange.push(
          <g key={`sr-${dev.name}`}>
            <line x1={mx} y1={uavSY - 4} x2={mx} y2={gndSY + 4}
              stroke={color} strokeWidth={1} strokeDasharray="4,3" opacity={0.5} />
            <text x={mx} y={gndSY + 14} fill={color} fontSize={9} textAnchor="middle">
              {fmtDist(Math.round(farGX))}
            </text>
          </g>
        );
      }
    }
    if (nearGX < Infinity && nearGX > 1 && Math.abs(nearGX - farGX) > farGX * 0.1) {
      const nx = sSX(nearGX);
      if (nx > PAD_L + 10) {
        const mx = sSX(farGX);
        const len = Math.round(farGX - nearGX);
        sideRange.push(
          <g key={`sn-${dev.name}`}>
            <line x1={nx} y1={uavSY - 4} x2={nx} y2={gndSY + 4}
              stroke={color} strokeWidth={1} strokeDasharray="2,3" opacity={0.35} />
            <text x={nx} y={gndSY + 14} fill={color} fontSize={9} textAnchor="middle" opacity={0.7}>
              {fmtDist(Math.round(nearGX))}
            </text>
            {/* Ground footprint length between near and far */}
            <line x1={nx} y1={gndSY - 3} x2={mx} y2={gndSY - 3}
              stroke={color} strokeWidth={0.7} opacity={0.5} />
            <text x={(nx + mx) / 2} y={gndSY - 5} fill={color} fontSize={8} textAnchor="middle" opacity={0.7}>
              {fmtDist(len)}
            </text>
          </g>
        );
      }
    }

    // --- Top view footprint polygon (only if device sees the ground) ---
    if (hitsGround) {
      topFov.push(
        <path key={`tf-${dev.name}`}
          d={corners.map((c, j) => `${j ? 'L' : 'M'}${tSX(c[0]).toFixed(1)},${tSY(c[1]).toFixed(1)}`).join(' ') + ' Z'}
          fill={color} fillOpacity={0.18} stroke={color} strokeWidth={1.2} strokeOpacity={0.6} />
      );

      // --- Top view: horizontal FOV label at footprint centroid ---
      const fovHDeg = (dev.fovH || dev.fovV * 1.5) * 180 / Math.PI;
      const cx = corners.reduce((s, c) => s + c[0], 0) / 4;
      const cy = corners.reduce((s, c) => s + c[1], 0) / 4;
      topFovLabels.push(
        <text key={`flh-${dev.name}`} x={tSX(cx)} y={tSY(cy) + 3}
          fill={color} fontSize={10} textAnchor="middle" opacity={0.85}>
          {`${fovHDeg.toFixed(1)}\u00B0`}
        </text>
      );
    }

    // --- Top view lateral width marks (Y axis, only if device sees the ground) ---
    if (hitsGround) {
      const topMaxY = Math.max(...corners.map((c) => Math.abs(c[1])));
      if (topMaxY > 1) {
        const fmtD = (d) => d >= 1000 ? `${(d / 1000).toFixed(1)}km` : `${Math.round(d)}m`;
        const tyPos = tSY(topMaxY);
        const tyNeg = tSY(-topMaxY);
        const width = Math.round(topMaxY * 2);
        topRange.push(
          <g key={`try-${dev.name}`}>
            <line x1={uavTX - 4} y1={tyPos} x2={uavTX + 4} y2={tyPos}
              stroke={color} strokeWidth={1} strokeDasharray="4,3" opacity={0.5} />
            <line x1={uavTX - 4} y1={tyNeg} x2={uavTX + 4} y2={tyNeg}
              stroke={color} strokeWidth={1} strokeDasharray="4,3" opacity={0.5} />
            <text x={uavTX + 8} y={tyPos + 3} fill={color} fontSize={9}>
              {fmtD(width)}
            </text>
          </g>
        );
      }
    }
  });

  return (
    <div style={{ marginTop: 4, display: 'flex', gap: 8, alignItems: 'start' }}>
      {/* Side view */}
      <svg width={SIDE_W} height={H} style={{ display: 'block' }}>
        <text x={PAD_L + SIDE_DW / 2} y={11} fill={colors.textDim} fontSize={10} textAnchor="middle">{t('visionProfile.sideView')}</text>

        <line x1={PAD_L - 8} y1={gndSY} x2={SIDE_W - PAD_R} y2={gndSY}
          stroke={colors.textDim} strokeWidth={1} />
        <text x={SIDE_W - PAD_R + 2} y={gndSY + 4} fill={colors.textDim} fontSize={9} textAnchor="end">{t('visionProfile.ground')}</text>

        <line x1={uavSX} y1={uavSY + UAV_R + 2} x2={uavSX} y2={gndSY - 2}
          stroke={colors.textDim} strokeWidth={0.7} />
        <text x={uavSX - 4} y={(uavSY + gndSY) / 2 + 3}
          fill={colors.textDim} fontSize={9} textAnchor="end">{altitude}m</text>

        {sideFov}
        {sideRange}
        {sideFovLabels}

        <circle cx={uavSX} cy={uavSY} r={UAV_R}
          fill={colors.textBright} stroke={colors.border} strokeWidth={1.5} />

      </svg>

      {/* Top view */}
      <svg width={TOP_W} height={H} style={{ display: 'block' }}>
        <text x={TOP_PL + TOP_DW / 2} y={11} fill={colors.textDim} fontSize={10} textAnchor="middle">{t('visionProfile.topView')}</text>

        {/* Forward axis (X) */}
        <line x1={uavTX} y1={uavTY} x2={TOP_W - TOP_PR} y2={uavTY}
          stroke={colors.textDim} strokeWidth={0.7} />
        <path d={`M${TOP_W - TOP_PR - 4},${uavTY - 3} L${TOP_W - TOP_PR},${uavTY} L${TOP_W - TOP_PR - 4},${uavTY + 3}`}
          fill="none" stroke={colors.textDim} strokeWidth={0.7} />
        <text x={TOP_W - TOP_PR} y={uavTY - 5} fill={colors.textDim} fontSize={8} textAnchor="end">{t('visionProfile.forward')}</text>

        {/* Lateral axis (Y) */}
        <line x1={uavTX} y1={PAD_T} x2={uavTX} y2={H - PAD_B}
          stroke={colors.textDim} strokeWidth={0.7} />

        {topFov}
        {topRange}
        {topFovLabels}

        {/* UAV dot */}
        <circle cx={uavTX} cy={uavTY} r={UAV_R}
          fill={colors.textBright} stroke={colors.border} strokeWidth={1.5} />
      </svg>
    </div>
  );
}
