/**
 * Joint pitch + altitude optimizer for multi-camera surveillance & navigation.
 *
 * Balances two objectives:
 * - Surveillance: maximize union ground-coverage area (determines track spacing)
 * - Navigation: ensure continuous angular coverage across the UAV dive pitch
 *   envelope (min_pitch..max_pitch) so a detected POI stays visible
 *
 * Pure JS, zero imports — testable via Node subprocess.
 */

// ── Polygon math ────────────────────────────────────────────────────────

/**
 * Compute ground footprint corners for a camera at given pitch/altitude.
 * Reuses the topCorner ray-projection from DetectionRangeDiagram.
 * Returns 4 corners as [[x,y], ...] in metres (forward, lateral) or null
 * if the lower FOV edge doesn't reach the ground.
 */
function computeFootprintCorners(pitchDeg, fovH, fovV, mdd, altitude) {
  const pitch = pitchDeg * Math.PI / 180;
  const halfH = fovH / 2;
  const halfV = fovV / 2;

  const corner = (u, v) => {
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    const dx = cp + v * Math.tan(halfV) * sp;
    const dy = u * Math.tan(halfH);
    const dz = sp - v * Math.tan(halfV) * cp;
    const len = Math.sqrt(dx * dx + dy * dy + dz * dz);
    const nx = dx / len, ny = dy / len, nz = dz / len;
    if (nz > -1e-6) return null; // ray doesn't hit ground
    const t = Math.min(mdd, altitude / (-nz));
    return [t * nx, t * ny];
  };

  // far-left, far-right, near-right, near-left
  const pts = [
    corner(-1, -1),
    corner(+1, -1),
    corner(+1, +1),
    corner(-1, +1),
  ];
  if (pts.some((p) => p === null)) return null;
  return pts;
}

/** Signed area of a simple polygon via the shoelace formula. */
function shoelaceArea(pts) {
  let area = 0;
  const n = pts.length;
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    area += pts[i][0] * pts[j][1] - pts[j][0] * pts[i][1];
  }
  return Math.abs(area) / 2;
}

/**
 * Sutherland-Hodgman polygon clipping — clip `subject` against convex `clip`.
 * Both are arrays of [x,y]. Returns the clipped polygon (may be empty).
 */
function clipConvexPolygons(subject, clip) {
  if (subject.length === 0 || clip.length === 0) return [];
  let output = subject.slice();
  const n = clip.length;

  for (let i = 0; i < n; i++) {
    if (output.length === 0) return [];
    const input = output;
    output = [];
    const a = clip[i];
    const b = clip[(i + 1) % n];

    for (let j = 0; j < input.length; j++) {
      const p = input[j];
      const q = input[(j + 1) % input.length];
      const pInside = cross(a, b, p) >= 0;
      const qInside = cross(a, b, q) >= 0;

      if (pInside) {
        output.push(p);
        if (!qInside) output.push(intersect(a, b, p, q));
      } else if (qInside) {
        output.push(intersect(a, b, p, q));
      }
    }
  }
  return output;
}

function cross(a, b, p) {
  return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
}

function intersect(a, b, p, q) {
  const a1 = b[1] - a[1], b1 = a[0] - b[0], c1 = a1 * a[0] + b1 * a[1];
  const a2 = q[1] - p[1], b2 = p[0] - q[0], c2 = a2 * p[0] + b2 * p[1];
  const det = a1 * b2 - a2 * b1;
  if (Math.abs(det) < 1e-12) return [(p[0] + q[0]) / 2, (p[1] + q[1]) / 2];
  return [(c1 * b2 - c2 * b1) / det, (a1 * c2 - a2 * c1) / det];
}

/**
 * Union area of an array of convex polygons via inclusion-exclusion.
 * For 2-3 cameras this is exact and fast.
 */
function unionArea(polygons) {
  const valid = polygons.filter((p) => p && p.length >= 3);
  const n = valid.length;
  if (n === 0) return 0;
  if (n === 1) return shoelaceArea(valid[0]);

  // Sum individual areas
  let total = 0;
  for (let i = 0; i < n; i++) total += shoelaceArea(valid[i]);

  // Subtract pairwise intersections
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      const inter = clipConvexPolygons(valid[i], valid[j]);
      if (inter.length >= 3) total -= shoelaceArea(inter);
    }
  }

  // Add back triple intersections (for 3+ polygons)
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      for (let k = j + 1; k < n; k++) {
        const ij = clipConvexPolygons(valid[i], valid[j]);
        if (ij.length < 3) continue;
        const ijk = clipConvexPolygons(ij, valid[k]);
        if (ijk.length >= 3) total += shoelaceArea(ijk);
      }
    }
  }
  return total;
}

// ── Detection weighting ─────────────────────────────────────────────────

const MIN_DETECT_PX = 2;
const RELIABLE_DETECT_PX = 8;

/**
 * Detection probability weight for a footprint based on slant range.
 * At mdd the POI is MIN_DETECT_PX tall (barely visible) → weight ≈ 0.25.
 * At mdd/4 the POI is RELIABLE_DETECT_PX tall → weight = 1.0.
 */
function detectWeight(footprint, mdd, altitude) {
  if (!footprint || footprint.length < 3) return 0;
  const cx = footprint.reduce((s, p) => s + p[0], 0) / footprint.length;
  const slantRange = Math.sqrt(altitude * altitude + cx * cx);
  return Math.min(1, MIN_DETECT_PX * mdd / (slantRange * RELIABLE_DETECT_PX));
}

/**
 * Weighted union area via inclusion-exclusion.
 * Each polygon's contribution is scaled by its detection weight.
 * Intersections are weighted by the best (max) weight of the overlapping cameras.
 */
function weightedUnionArea(polygons, weights) {
  const valid = [];
  const w = [];
  for (let i = 0; i < polygons.length; i++) {
    if (polygons[i] && polygons[i].length >= 3) {
      valid.push(polygons[i]);
      w.push(weights[i]);
    }
  }
  const n = valid.length;
  if (n === 0) return 0;
  if (n === 1) return shoelaceArea(valid[0]) * w[0];

  // Sum individual areas × weight
  let total = 0;
  for (let i = 0; i < n; i++) total += shoelaceArea(valid[i]) * w[i];

  // Subtract pairwise intersections × max weight
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      const inter = clipConvexPolygons(valid[i], valid[j]);
      if (inter.length >= 3) {
        total -= shoelaceArea(inter) * Math.max(w[i], w[j]);
      }
    }
  }

  // Add back triple intersections × max weight (for 3+ polygons)
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      for (let k = j + 1; k < n; k++) {
        const ij = clipConvexPolygons(valid[i], valid[j]);
        if (ij.length < 3) continue;
        const ijk = clipConvexPolygons(ij, valid[k]);
        if (ijk.length >= 3) {
          total += shoelaceArea(ijk) * Math.max(w[i], w[j], w[k]);
        }
      }
    }
  }
  return total;
}

// ── Angular continuity ──────────────────────────────────────────────────

/**
 * Compute total angular gap between camera FOVs sorted by pitch.
 * Each device covers [pitchDeg - halfFovV, pitchDeg + halfFovV] in degrees.
 * Also checks coverage of the required UAV pitch envelope [minPitchReq, maxPitchReq].
 *
 * Returns total gap in degrees (0 = no blind spots in the required range).
 */
function angularGap(devices, minPitchReq, maxPitchReq) {
  if (devices.length === 0) return 180;

  const envSize = maxPitchReq - minPitchReq;
  if (envSize <= 0) return 0;

  // Build sorted intervals [lo, hi] in degrees
  const intervals = devices.map((d) => {
    const halfV = (d.fovV * 180 / Math.PI) / 2;
    return [d.pitchDeg - halfV, d.pitchDeg + halfV];
  }).sort((a, b) => a[0] - b[0]);

  // Merge overlapping intervals
  const merged = [intervals[0].slice()];
  for (let i = 1; i < intervals.length; i++) {
    const last = merged[merged.length - 1];
    if (intervals[i][0] <= last[1]) {
      last[1] = Math.max(last[1], intervals[i][1]);
    } else {
      merged.push(intervals[i].slice());
    }
  }

  // Weighted length integral: w(a) = 1 + (a - minPitchReq) / envSize
  // Integrates to (b-a) + (offB^2 - offA^2) / (2*envSize)
  const weightedLen = (a, b) => {
    if (b <= a) return 0;
    const offA = a - minPitchReq, offB = b - minPitchReq;
    return (b - a) + (offB * offB - offA * offA) / (2 * envSize);
  };

  // Weighted gap within the required range
  let wgap = 0;
  let cursor = minPitchReq;
  for (const [lo, hi] of merged) {
    const gapStart = cursor;
    const gapEnd = Math.min(lo, maxPitchReq);
    if (gapEnd > gapStart) wgap += weightedLen(gapStart, gapEnd);
    cursor = Math.max(cursor, hi);
    if (cursor >= maxPitchReq) break;
  }
  if (cursor < maxPitchReq) wgap += weightedLen(cursor, maxPitchReq);

  // Minimum achievable weighted gap: total FOV placed at the shallow end
  const totalFov = intervals.reduce((s, iv) => s + (iv[1] - iv[0]), 0);
  const unrawGap = Math.max(0, envSize - totalFov);
  const minWgap = unrawGap > 0 ? weightedLen(minPitchReq, minPitchReq + unrawGap) : 0;

  return Math.max(0, wgap - minWgap);
}

// ── Optimizer ───────────────────────────────────────────────────────────

/**
 * Joint optimisation of device pitches + altitude.
 *
 * @param {Array} devices — per device: { fovH, fovV, mdd, currentPitch }
 *   fovH/fovV in radians, mdd in metres.
 * @param {{ min: number, max: number }} altitudeRange
 * @param {{ min: number, max: number }} pitchEnvelope — UAV dive pitch range (degrees)
 * @returns {{ pitches: number[], altitude: number, score: number }}
 */
function optimizePitches(devices, altitudeRange, pitchEnvelope) {
  const minAlt = altitudeRange?.min ?? 30;
  const maxAlt = altitudeRange?.max ?? 500;
  const minPitchReq = pitchEnvelope?.min ?? -60;
  const maxPitchReq = pitchEnvelope?.max ?? 20;

  const nd = devices.length;

  const evaluate = (pitches, alt) => {
    const footprints = devices.map((d, i) =>
      computeFootprintCorners(pitches[i], d.fovH, d.fovV, d.mdd, alt)
    );
    const allNull = footprints.every((fp) => fp === null);
    if (allNull) return -Infinity;
    const weights = footprints.map((fp, i) =>
      detectWeight(fp, devices[i].mdd, alt)
    );
    const area = weightedUnionArea(footprints, weights);
    const devInfos = devices.map((d, i) => ({
      pitchDeg: pitches[i], fovV: d.fovV,
    }));
    const gap = angularGap(devInfos, minPitchReq, maxPitchReq);
    // Hard constraint: any gap-free solution beats any solution with gaps
    if (gap > 0.5) return -gap;
    return area;
  };

  // --- Coarse grid search: 1° pitch steps, 10m altitude steps ---
  let bestScore = -Infinity;
  let bestPitches = devices.map((d) => d.currentPitch);
  let bestAlt = (minAlt + maxAlt) / 2;

  // Build pitch candidates per device (full -90..90 range for gimbal freedom)
  const coarsePitchStep = 1;
  const pitchCandidates = [];
  for (let p = -90; p <= 90; p += coarsePitchStep) pitchCandidates.push(p);

  if (nd === 1) {
    // Single device: simple grid
    for (let alt = minAlt; alt <= maxAlt; alt += 10) {
      for (const p of pitchCandidates) {
        const s = evaluate([p], alt);
        if (s > bestScore) { bestScore = s; bestPitches = [p]; bestAlt = alt; }
      }
    }
  } else if (nd === 2) {
    // Two devices: nested grid
    for (let alt = minAlt; alt <= maxAlt; alt += 10) {
      for (const p0 of pitchCandidates) {
        for (const p1 of pitchCandidates) {
          if (p1 >= p0) continue; // enforce ordering: device 0 higher pitch
          const s = evaluate([p0, p1], alt);
          if (s > bestScore) { bestScore = s; bestPitches = [p0, p1]; bestAlt = alt; }
        }
      }
    }
  } else {
    // 3+ devices: use current pitches sorted, sweep with fixed spacing heuristic
    const sorted = devices.map((d, i) => ({ i, p: d.currentPitch })).sort((a, b) => b.p - a.p);
    for (let alt = minAlt; alt <= maxAlt; alt += 10) {
      for (const baseP of pitchCandidates) {
        // Spread devices evenly across FOV range from baseP
        const totalFovV = devices.reduce((s, d) => s + d.fovV * 180 / Math.PI, 0);
        const spacing = totalFovV / nd;
        const pitches = new Array(nd);
        sorted.forEach(({ i }, idx) => {
          pitches[i] = baseP - idx * spacing;
        });
        const s = evaluate(pitches, alt);
        if (s > bestScore) { bestScore = s; bestPitches = pitches.slice(); bestAlt = alt; }
      }
    }
  }

  // --- Fine grid search: ±2° pitch, ±20m altitude around coarse optimum ---
  const coarsePitches = bestPitches.slice();
  const coarseAlt = bestAlt;

  const fineRange = (center, halfW, step, lo, hi) => {
    const vals = [];
    for (let v = Math.max(lo, center - halfW); v <= Math.min(hi, center + halfW); v += step) {
      vals.push(Math.round(v * 10) / 10);
    }
    return vals;
  };

  const fineAlts = fineRange(coarseAlt, 20, 5, minAlt, maxAlt);

  if (nd === 1) {
    const fp0 = fineRange(coarsePitches[0], 2, 0.1, -90, 90);
    for (const alt of fineAlts) {
      for (const p of fp0) {
        const s = evaluate([p], alt);
        if (s > bestScore) { bestScore = s; bestPitches = [p]; bestAlt = alt; }
      }
    }
  } else if (nd === 2) {
    const fp0 = fineRange(coarsePitches[0], 2, 0.1, -90, 90);
    const fp1 = fineRange(coarsePitches[1], 2, 0.1, -90, 90);
    for (const alt of fineAlts) {
      for (const p0 of fp0) {
        for (const p1 of fp1) {
          const s = evaluate([p0, p1], alt);
          if (s > bestScore) { bestScore = s; bestPitches = [p0, p1]; bestAlt = alt; }
        }
      }
    }
  } else {
    // Fine-tune each device pitch independently
    for (let di = 0; di < nd; di++) {
      const fp = fineRange(coarsePitches[di], 2, 0.1, -90, 90);
      for (const alt of fineAlts) {
        for (const p of fp) {
          const trial = bestPitches.slice();
          trial[di] = p;
          const s = evaluate(trial, alt);
          if (s > bestScore) { bestScore = s; bestPitches = trial; bestAlt = alt; }
        }
      }
    }
  }

  return {
    pitches: bestPitches.map((p) => Math.round(p * 10) / 10),
    altitude: Math.round(bestAlt),
    score: bestScore,
  };
}

// ── Exports (stripped by test harness for Node) ─────────────────────────

export {
  computeFootprintCorners,
  shoelaceArea,
  clipConvexPolygons,
  unionArea,
  detectWeight,
  weightedUnionArea,
  angularGap,
  optimizePitches,
};
