import { describe, expect, it } from 'vitest';
import {
  FENCE_MODE,
  consumeFenceIntent,
  emptyFenceObservations,
  fenceObservationFromDownload,
  fenceObservationToken,
  fenceUploadPayload,
  recordFenceObservation,
  reserveFenceObservations,
  resetFenceObservations,
  summarizeFenceObservations,
  fenceMapDisplay,
  vehicleFenceInEffect,
} from './fenceIntent';

const ring = (d = 0) => [
  { lat: 32.0 + d, lon: 34.0 },
  { lat: 32.01 + d, lon: 34.0 },
  { lat: 32.0 + d, lon: 34.01 },
];

const download = (over = {}) => ({
  vertices: ring(), exclusions: [], total_items: 3,
  params: { enable: 1, autoenable: 1, type: 4, action: 1 },
  params_readback: 'ok',
  ...over,
});

describe('fenceObservationFromDownload', () => {
  it('records an unreachable readback as failed, never as an empty fence', () => {
    const obs = fenceObservationFromDownload(null);
    expect(obs.status).toBe('failed');
    expect(obs.params).toBeNull();
  });

  it('keeps the raw autoenable value instead of collapsing it to a boolean', () => {
    const obs = fenceObservationFromDownload(download({
      params: { enable: 0, autoenable: 2, type: 4, action: 1 },
    }));
    expect(obs.params.autoenable).toBe(2);
    expect(obs.mode).toBe(FENCE_MODE.AUTO);
  });

  it('reads ENABLE=0 with AUTOENABLE>0 as auto-enable, not off', () => {
    const obs = fenceObservationFromDownload(download({
      params: { enable: 0, autoenable: 1, type: 4, action: 1 },
    }));
    expect(obs.mode).toBe(FENCE_MODE.AUTO);
  });

  it('treats a failed param readback as unknown even when the ring downloaded', () => {
    const obs = fenceObservationFromDownload(download({
      params: null, params_readback: 'failed',
    }));
    expect(obs.status).toBe('known');
    expect(obs.mode).toBe(FENCE_MODE.UNKNOWN);
    expect(obs.ring).toHaveLength(3);
  });

  it('distinguishes a known-empty fence from an unknown one', () => {
    const obs = fenceObservationFromDownload(download({
      vertices: [], total_items: 0,
      params: { enable: 0, autoenable: 0, type: 4, action: 1 },
    }));
    expect(obs.mode).toBe(FENCE_MODE.OFF);
    expect(obs.ring).toEqual([]);
  });
});

describe('summarizeFenceObservations', () => {
  const observe = (entries) => {
    let state = emptyFenceObservations();
    let seq = 0;
    for (const [sysId, fence] of entries) {
      seq += 1;
      state = recordFenceObservation(
        state, sysId, fenceObservationFromDownload(fence),
        fenceObservationToken(state, seq),
      );
    }
    return state;
  };

  it('reports no observations before any download', () => {
    const s = summarizeFenceObservations(emptyFenceObservations(), [1, 2]);
    expect(s.hasObservations).toBe(false);
    expect(s.mode).toBe(FENCE_MODE.NONE);
  });

  it('reports mixed when vehicle modes differ, never a shared collapsed mode', () => {
    const state = observe([
      [1, download()],
      [2, download({ params: { enable: 0, autoenable: 0, type: 4, action: 1 } })],
    ]);
    const s = summarizeFenceObservations(state, [1, 2]);
    expect(s.mode).toBe(FENCE_MODE.MIXED);
    expect(s.modes).toEqual({ 1: FENCE_MODE.ON, 2: FENCE_MODE.OFF });
    // The ring is genuinely the same on both, so it stays displayable — but it
    // is still only an observation and cannot reach an upload.
    expect(s.ring).toHaveLength(3);
    expect(fenceUploadPayload({ enabled: false, intent: null, vertices: s.ring }).payload)
      .toBeNull();
  });

  it('reports unknown when every readback failed', () => {
    const state = observe([[1, null], [2, null]]);
    const s = summarizeFenceObservations(state, [1, 2]);
    expect(s.mode).toBe(FENCE_MODE.UNKNOWN);
    expect(s.allFailed).toBe(true);
    expect(s.hasObservations).toBe(true);
  });

  it('offers a homogeneous ring for display but flags differing rings as mixed', () => {
    const same = summarizeFenceObservations(
      observe([[1, download()], [2, download()]]), [1, 2],
    );
    expect(same.ring).toHaveLength(3);
    const differing = summarizeFenceObservations(
      observe([[1, download()], [2, download({ vertices: ring(0.5) })]]), [1, 2],
    );
    expect(differing.ring).toBeNull();
    expect(differing.ringMixed).toBe(true);
  });

  it('offers agreed keep-out rings for display only', () => {
    const keepOut = [ring(0.3)];
    const agreed = summarizeFenceObservations(
      observe([[1, download({ exclusions: keepOut })], [2, download({ exclusions: keepOut })]]),
      [1, 2],
    );
    expect(agreed.exclusions).toHaveLength(1);
    // Disagreeing keep-outs are not presented as a shared set.
    const disagreeing = summarizeFenceObservations(
      observe([[1, download({ exclusions: keepOut })], [2, download({ exclusions: [] })]]),
      [1, 2],
    );
    expect(disagreeing.exclusions).toEqual([]);
  });

  it('ignores observations for vehicles no longer on the roster', () => {
    const state = observe([[1, download()], [2, download()]]);
    const s = summarizeFenceObservations(state, [1]);
    expect(s.count).toBe(1);
  });
});

describe('observation lifecycle', () => {
  it('drops a result that arrives after the epoch advanced', () => {
    const start = emptyFenceObservations();
    const token = fenceObservationToken(start, 1);
    const reset = resetFenceObservations(start);
    const late = recordFenceObservation(
      reset, 1, fenceObservationFromDownload(download()), token,
    );
    expect(late).toBe(reset);
    expect(Object.keys(late.bySysId)).toHaveLength(0);
  });

  it('clears earlier observations when a replacement plan resets them', () => {
    const initial = emptyFenceObservations();
    const state = recordFenceObservation(
      initial, 1, fenceObservationFromDownload(download()),
      fenceObservationToken(initial, 1),
    );
    expect(Object.keys(state.bySysId)).toHaveLength(1);
    const reset = resetFenceObservations(state);
    expect(Object.keys(reset.bySysId)).toHaveLength(0);
    expect(reset.epoch).not.toBe(state.epoch);
  });

  it("reads every vehicle of a fleet-read lifecycle as asked, also one not yet reserved", () => {
    const reset = resetFenceObservations(emptyFenceObservations(), { readFleet: true });
    const asked = summarizeFenceObservations(reset, [1, 2]);
    expect(asked.hasObservations).toBe(true);
    expect(asked.knownCount).toBe(0);
    // With no vehicle connected nobody was asked.
    expect(summarizeFenceObservations(reset, []).hasObservations).toBe(false);
    // A vehicle that joins after the first reservation is still asked.
    const reserved = reserveFenceObservations(reset, [1]);
    const joined = summarizeFenceObservations(reserved, [2]);
    expect(joined.hasObservations).toBe(true);
    expect(joined.knownCount).toBe(0);
    expect(reserved.epoch).toBe(reset.epoch);
    // The next lifecycle starts without it.
    expect(summarizeFenceObservations(resetFenceObservations(reserved), [2]).hasObservations)
      .toBe(false);
  });

  it("starts a plain reset with nobody asked", () => {
    const reset = resetFenceObservations(emptyFenceObservations());
    expect(summarizeFenceObservations(reset, [1]).hasObservations).toBe(false);
  });
});

describe('fenceUploadPayload', () => {
  it('omits the fence entirely when nothing was authored', () => {
    expect(fenceUploadPayload({ enabled: false, intent: null, vertices: [] }).payload)
      .toBeNull();
  });

  it('omits the fence after an acknowledged disable, so a re-upload preserves it', () => {
    // Intent consumed by a matching-generation ack -> nothing pending.
    expect(fenceUploadPayload({ enabled: false, intent: null, vertices: ring() }).payload)
      .toBeNull();
  });

  it('sends an explicit disable while the operator intent is still pending', () => {
    const { payload } = fenceUploadPayload({
      enabled: false, intent: { generation: 3, enabled: false }, vertices: [],
    });
    expect(payload).toMatchObject({ enabled: false, vertices: [], exclusions: [] });
  });

  it('sends authored geometry when enabled', () => {
    const { payload, invalid } = fenceUploadPayload({
      enabled: true, intent: { generation: 1, enabled: true },
      vertices: ring(), exclusions: [ring(0.2)],
    });
    expect(invalid).toBe(false);
    expect(payload).toMatchObject({ enabled: true, action: 1, type: 4 });
    expect(payload.vertices).toHaveLength(3);
    expect(payload.exclusions).toHaveLength(1);
  });

  it('rejects an enabled fence with invalid geometry instead of disabling it', () => {
    const res = fenceUploadPayload({
      enabled: true, intent: { generation: 1, enabled: true },
      vertices: [{ lat: 1, lon: 1 }],
    });
    expect(res.invalid).toBe(true);
    expect(res.payload).toBeNull();
  });
});

describe('consumeFenceIntent', () => {
  const intent = { generation: 7, enabled: false };

  it('consumes intent acknowledged for its own generation', () => {
    expect(consumeFenceIntent(intent, 7, true)).toBeNull();
  });

  it('retains intent when the upload failed', () => {
    expect(consumeFenceIntent(intent, 7, false)).toBe(intent);
  });

  it('retains a newer edit against an older upload result', () => {
    const newer = { generation: 9, enabled: true };
    expect(consumeFenceIntent(newer, 7, true)).toBe(newer);
  });
});

describe('summarizeFenceObservations — exact autoenable mode', () => {
  const token = (state, seq) => ({ epoch: state.epoch, edit: state.edit, seq });
  const observed = (autoenable) => fenceObservationFromDownload(download({
    params: { enable: 0, autoenable, type: 4, action: 1 },
  }));

  it('carries each vehicle\'s raw FENCE_AUTOENABLE value, not just "auto"', () => {
    let s = emptyFenceObservations();
    s = recordFenceObservation(s, 1, observed(2), token(s, 1));
    s = recordFenceObservation(s, 2, observed(2), token(s, 2));
    const sum = summarizeFenceObservations(s, [1, 2]);
    // "auto" alone does not tell the operator WHEN the fence arms: 2 is
    // "on arming", 1 is "on takeoff". The configured value has to survive.
    expect(sum.autoenableModes).toEqual({ 1: 2, 2: 2 });
    expect(sum.autoenableMode).toBe(2);
  });

  it('reports no fleet autoenable value when the vehicles are configured differently', () => {
    let s = emptyFenceObservations();
    s = recordFenceObservation(s, 1, observed(1), token(s, 1));
    s = recordFenceObservation(s, 2, observed(2), token(s, 2));
    const sum = summarizeFenceObservations(s, [1, 2]);
    expect(sum.autoenableMode).toBeNull();
    expect(sum.autoenableModes).toEqual({ 1: 1, 2: 2 });
  });

  it('reports no autoenable value for a vehicle whose params could not be read', () => {
    let s = emptyFenceObservations();
    s = recordFenceObservation(s, 1, fenceObservationFromDownload(null), token(s, 1));
    const sum = summarizeFenceObservations(s, [1]);
    expect(sum.autoenableModes).toEqual({ 1: null });
    expect(sum.autoenableMode).toBeNull();
  });
});

describe('vehicleFenceInEffect', () => {
  const observed = (over = {}) => ({
    hasObservations: true, failedCount: 0, mode: FENCE_MODE.ON, ring: ring(), exclusions: [],
    ...over,
  });
  const undecided = { touched: false, intent: null, enabled: false };

  it('shows an agreed active fleet fence on an undecided plan', () => {
    expect(vehicleFenceInEffect({ ...undecided, observed: observed() }))
      .toEqual({ ring: ring(), exclusions: [], exclusionsMixed: false });
    expect(vehicleFenceInEffect({ ...undecided, observed: observed({ mode: FENCE_MODE.AUTO }) }))
      .not.toBeNull();
  });

  it('shows nothing for a fence that is off, unknown, partial or disagreeing', () => {
    for (const over of [
      { mode: FENCE_MODE.OFF }, { mode: FENCE_MODE.UNKNOWN }, { mode: FENCE_MODE.MIXED },
      { failedCount: 1 }, { ring: null }, { hasObservations: false },
    ]) {
      expect(vehicleFenceInEffect({ ...undecided, observed: observed(over) })).toBeNull();
    }
  });

  it('gives way to any fence decision of the plan', () => {
    expect(vehicleFenceInEffect({ ...undecided, touched: true, observed: observed() })).toBeNull();
    expect(vehicleFenceInEffect({ ...undecided, enabled: true, observed: observed() })).toBeNull();
    expect(vehicleFenceInEffect({
      ...undecided, intent: { generation: 1, enabled: false }, observed: observed(),
    })).toBeNull();
  });
});

describe('fenceMapDisplay', () => {
  const vehicleFence = { ring: ring(1), exclusions: [], exclusionsMixed: false };

  it("draws the plan's fence while it is on", () => {
    expect(fenceMapDisplay({ enabled: true, polygon: ring(), vehicleFence: null }))
      .toEqual({ show: true, polygon: ring(), source: 'planner' });
  });

  it("draws the fleet's fence while the switch shows it", () => {
    expect(fenceMapDisplay({ enabled: false, polygon: null, vehicleFence }))
      .toEqual({ show: true, polygon: ring(1), source: 'vehicles' });
  });

  it('draws nothing when off', () => {
    expect(fenceMapDisplay({ enabled: false, polygon: null, vehicleFence: null }))
      .toEqual({ show: false, polygon: null, source: 'none' });
  });
});

describe('summarizeFenceObservations — keep-out agreement', () => {
  it('flags keep-outs the fleet disagrees on, so an empty list is not read as none', () => {
    let state = emptyFenceObservations();
    const tok = (seq) => fenceObservationToken(state, seq);
    state = recordFenceObservation(state, 1, fenceObservationFromDownload(download({ exclusions: [ring(0.1)] })), tok(1));
    state = recordFenceObservation(state, 2, fenceObservationFromDownload(download({ exclusions: [] })), tok(2));
    const summary = summarizeFenceObservations(state, [1, 2]);
    expect(summary.exclusions).toEqual([]);
    expect(summary.exclusionsMixed).toBe(true);
  });
});
