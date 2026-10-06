import unittest
from dataclasses import replace

import numpy as np

from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.track_identity import (
    TrackIdentityResolver,
    build_track_identity_resolver,
)
from navpy.modules.vision.track_identity_appearance import (
    IdentityGallery,
    LateIdentityMerger,
)
from navpy.modules.vision.track_identity_commit import (
    IdentityObservationCommitter,
)
from navpy.modules.vision.track_identity_registry import IdentityRegistry
from navpy.modules.vision.track_identity_types import (
    IdentityCounters,
    IdentityKinematics,
    IdentityState,
    TrackIdentityPolicies,
)


def _track(obj_id, cx=100.0, cy=80.0, w=40.0, h=20.0, class_id=2, vx=0.0, vy=0.0):
    return TrackedObject(
        id=obj_id, cx=float(cx), cy=float(cy), w=float(w), h=float(h),
        confidence=0.8, class_id=class_id, age=1, hits=1, missed=0,
        is_confirmed=True, timestamp=100.0, vx=float(vx), vy=float(vy),
    )


EMB_A = np.array([1.0, 0.0], dtype=np.float32)
EMB_B = np.array([0.0, 1.0], dtype=np.float32)


def _resolver(**overrides: float | int) -> TrackIdentityResolver:
    """Compose non-default policies explicitly for focused algorithm tests."""
    policies = TrackIdentityPolicies()
    retention_names = {
        "max_lost_seconds",
        "grace_seconds",
        "merge_window",
        "max_identities",
    }
    geometry_names = {
        "max_center_distance",
        "motion_gate_growth",
        "motion_gate_cap",
        "predict_horizon",
        "max_size_error",
        "ambiguity_margin",
    }
    appearance_names = {
        "appearance_threshold": "threshold",
        "appearance_margin": "margin",
        "swap_threshold": "swap_threshold",
        "swap_patience": "swap_patience",
        "max_prototypes": "max_prototypes",
    }
    retention = {
        name: value
        for name, value in overrides.items()
        if name in retention_names
    }
    geometry = {
        name: value
        for name, value in overrides.items()
        if name in geometry_names
    }
    appearance = {
        appearance_names[name]: value
        for name, value in overrides.items()
        if name in appearance_names
    }
    recognized = retention_names | geometry_names | set(appearance_names)
    unknown = set(overrides) - recognized
    if unknown:
        raise ValueError(f"unknown identity policy overrides: {sorted(unknown)}")
    return build_track_identity_resolver(
        replace(
            policies,
            retention=replace(policies.retention, **retention),
            geometry=replace(policies.geometry, **geometry),
            appearance=replace(policies.appearance, **appearance),
        )
    )


class TestStableMapping(unittest.TestCase):
    def test_keeps_same_id_for_same_raw_track(self):
        r = _resolver()
        first = r.update([_track(7)], 640, 480, now=10.0)[0]
        second = r.update([_track(7, cx=102.0)], 640, 480, now=10.1)[0]
        self.assertEqual(second.id, first.id)

    def test_reuses_scene_id_for_lone_unambiguous_reappearance(self):
        r = _resolver(max_lost_seconds=5.0)
        first = r.update([_track(7, cx=60, cy=420)], 640, 480, now=10.0)[0]
        self.assertEqual(r.update([], 640, 480, now=11.0), [])
        reacquired = r.update([_track(99, cx=65, cy=416)], 640, 480, now=12.0)[0]
        self.assertEqual(reacquired.id, first.id)

    def test_does_not_reuse_scene_id_for_far_new_object(self):
        r = _resolver(max_lost_seconds=5.0, max_center_distance=0.05)
        first = r.update([_track(7, cx=60, cy=420)], 640, 480, now=10.0)[0]
        r.update([], 640, 480, now=11.0)
        far = r.update([_track(99, cx=500, cy=80)], 640, 480, now=12.0)[0]
        self.assertNotEqual(far.id, first.id)

    def test_does_not_reuse_scene_id_for_different_class(self):
        r = _resolver(max_lost_seconds=5.0)
        first = r.update([_track(7, class_id=2)], 640, 480, now=10.0)[0]
        r.update([], 640, 480, now=11.0)
        person = r.update([_track(99, class_id=0)], 640, 480, now=12.0)[0]
        self.assertNotEqual(person.id, first.id)

    def test_expires_lost_identity_after_timeout(self):
        r = _resolver(max_lost_seconds=1.0)
        first = r.update([_track(7)], 640, 480, now=10.0)[0]
        r.update([], 640, 480, now=10.5)
        late = r.update([_track(99)], 640, 480, now=12.0)[0]
        self.assertNotEqual(late.id, first.id)


class TestGlobalAssignmentNoSwap(unittest.TestCase):
    def test_two_far_objects_keep_their_own_ids_regardless_of_order(self):
        # Global assignment: even listed in crossed order, each well-separated
        # object re-binds to its own identity (the greedy bug would swap).
        r = _resolver(max_lost_seconds=5.0, max_center_distance=0.3,
                                  grace_seconds=0.0)
        out = r.update([_track(1, cx=100, cy=100), _track(2, cx=400, cy=100)], 640, 480, now=0.0)
        sid_p = next(t.id for t in out if round(t.cx) == 100)
        sid_q = next(t.id for t in out if round(t.cx) == 400)
        r.update([], 640, 480, now=0.1)
        # reappear, listed Q-first, each near its own original spot
        out2 = r.update([_track(7, cx=405, cy=100), _track(8, cx=105, cy=100)], 640, 480, now=0.2)
        got = {round(t.cx): t.id for t in out2}
        self.assertEqual(got[105], sid_p)
        self.assertEqual(got[405], sid_q)


class TestFailClosed(unittest.TestCase):
    def test_ambiguous_same_class_reappearance_gets_new_ids(self):
        # Two similar objects lost close together, reappearing ambiguously with
        # no appearance evidence -> refuse to reacquire (new ids), never swap.
        r = _resolver(max_lost_seconds=5.0, max_center_distance=0.3,
                                  ambiguity_margin=0.05)
        out = r.update([_track(1, cx=100, cy=100), _track(2, cx=130, cy=100)], 640, 480, now=0.0)
        old_ids = {t.id for t in out}
        r.update([], 640, 480, now=0.1)
        out2 = r.update([_track(7, cx=112, cy=100), _track(8, cx=118, cy=100)], 640, 480, now=0.2)
        new_ids = {t.id for t in out2}
        self.assertTrue(new_ids.isdisjoint(old_ids))

    def test_appearance_disambiguates_crossed_objects(self):
        r = _resolver(max_lost_seconds=5.0, max_center_distance=0.3,
                                  ambiguity_margin=0.05, grace_seconds=0.0)
        out = r.update(
            [_track(1, cx=100, cy=100), _track(2, cx=140, cy=100)], 640, 480, now=0.0,
            embeddings={1: EMB_A, 2: EMB_B},
        )
        sid_a = next(t.id for t in out if round(t.cx) == 100)
        sid_b = next(t.id for t in out if round(t.cx) == 140)
        r.update([], 640, 480, now=0.1)
        # crossed positions but appearance follows the object
        out2 = r.update(
            [_track(7, cx=138, cy=100), _track(8, cx=102, cy=100)], 640, 480, now=0.2,
            embeddings={7: EMB_A, 8: EMB_B},
        )
        by_emb = {7: None, 8: None}
        for t in out2:
            by_emb[7 if round(t.cx) == 138 else 8] = t.id
        self.assertEqual(by_emb[7], sid_a)  # EMB_A object keeps A's id
        self.assertEqual(by_emb[8], sid_b)

    def test_appearance_reacquires_poi_that_moved_far(self):
        # The core goal-2 case: a POI lost then re-detected far from where it
        # was lost must still be reacquired when appearance matches, even past
        # the geometric gate.
        r = _resolver(max_lost_seconds=5.0, max_center_distance=0.05,
                                  grace_seconds=0.0)
        first = r.update([_track(1, cx=60, cy=420)], 640, 480, now=0.0,
                         embeddings={1: EMB_A})[0]
        r.update([], 640, 480, now=0.1)
        out = r.update([_track(9, cx=560, cy=70)], 640, 480, now=0.2,
                       embeddings={9: EMB_A})[0]
        self.assertEqual(out.id, first.id)

    def test_live_id_swap_is_corrected_after_persistent_divergence(self):
        # A genuine swap persists across frames; appearance divergence, once it
        # passes the debounce window, drops the stale mapping and re-binds.
        r = _resolver(max_lost_seconds=5.0, max_center_distance=0.3,
                                  swap_patience=3, swap_threshold=0.5, grace_seconds=0.0)
        out = r.update([_track(1, cx=100), _track(2, cx=140)], 640, 480, now=0.0,
                       embeddings={1: EMB_A, 2: EMB_B})
        sid_a = next(t.id for t in out if round(t.cx) == 100)   # raw1 -> appearance A
        sid_b = next(t.id for t in out if round(t.cx) == 140)   # raw2 -> appearance B
        # raw1 now carries appearance B for several consecutive frames (a real swap)
        last = None
        for k in range(3):
            last = r.update([_track(1, cx=140)], 640, 480, now=0.1 * (k + 1),
                            embeddings={1: EMB_B})[0]
        self.assertNotEqual(last.id, sid_a)   # not poisoned with the wrong id
        self.assertEqual(last.id, sid_b)      # re-bound to the B identity

    def test_single_divergent_frame_does_not_churn_locked_id(self):
        # F4: one noisy/blurred/partially-occluded crop must NOT drop a healthy
        # locked mapping; an agreeing frame resets the mismatch counter.
        r = _resolver(max_lost_seconds=5.0, max_center_distance=0.3,
                                  swap_patience=3)
        first = r.update([_track(1, cx=100)], 640, 480, now=0.0, embeddings={1: EMB_A})[0]
        held = r.update([_track(1, cx=100)], 640, 480, now=0.1, embeddings={1: EMB_B})[0]
        self.assertEqual(held.id, first.id)   # debounced — id held
        back = r.update([_track(1, cx=100)], 640, 480, now=0.2, embeddings={1: EMB_A})[0]
        self.assertEqual(back.id, first.id)   # mismatch count reset, still the same id

    def test_appearance_mismatch_blocks_reacquire(self):
        r = _resolver(max_lost_seconds=5.0, max_center_distance=0.3)
        first = r.update([_track(1, cx=100, cy=100)], 640, 480, now=0.0,
                         embeddings={1: EMB_A})[0]
        r.update([], 640, 480, now=0.1)
        # lone reappearance at the same spot but a DIFFERENT appearance
        out = r.update([_track(9, cx=102, cy=100)], 640, 480, now=0.2,
                       embeddings={9: EMB_B})[0]
        self.assertNotEqual(out.id, first.id)

    def test_embedded_track_is_not_geometry_bound_to_galleryless_identity(self):
        # Codex case: lost A (has gallery, far left) and lost B (no gallery, far
        # right). A returning track carrying A's appearance near B's old position
        # must bind to A (appearance), never to B by geometry.
        r = _resolver(max_lost_seconds=5.0, max_center_distance=0.3,
                                  grace_seconds=0.0)
        out = r.update(
            [_track(1, cx=10, cy=100), _track(2, cx=630, cy=100)], 640, 480, now=0.0,
            embeddings={1: EMB_A},  # only track 1 is embedded -> B has no gallery
        )
        sid_a = next(t.id for t in out if round(t.cx) == 10)
        sid_b = next(t.id for t in out if round(t.cx) == 630)
        r.update([], 640, 480, now=0.1)
        out2 = r.update([_track(9, cx=630, cy=100)], 640, 480, now=0.2,
                        embeddings={9: EMB_A})
        self.assertEqual(out2[0].id, sid_a)       # appearance wins
        self.assertNotEqual(out2[0].id, sid_b)    # NOT geometry-bound to B

    def test_lone_weak_appearance_candidate_is_fail_closed(self):
        # A single candidate whose appearance is only "somewhat similar" (just
        # under threshold, not confident) must NOT reacquire (fail-closed).
        r = _resolver(max_lost_seconds=5.0, max_center_distance=0.3,
                                  appearance_threshold=0.5)
        first = r.update([_track(1, cx=100, cy=100)], 640, 480, now=0.0,
                         embeddings={1: np.array([1.0, 0.0], np.float32)})[0]
        r.update([], 640, 480, now=0.1)
        # cos distance ~0.45 (< threshold 0.5 but > strong 0.3) -> weak
        weak = np.array([np.cos(0.96), np.sin(0.96)], np.float32)  # ~0.45 distance
        out = r.update([_track(9, cx=105, cy=100)], 640, 480, now=0.2,
                       embeddings={9: weak})[0]
        self.assertNotEqual(out.id, first.id)


class TestBounded(unittest.TestCase):
    def test_identity_state_is_bounded(self):
        r = _resolver(max_lost_seconds=1000.0, max_identities=5)
        for i in range(40):
            r.update([_track(1000 + i, cx=10 * (i % 50), cy=300)], 640, 480, now=float(i))
        # active (1) + at most max_identities lost retained
        self.assertLessEqual(r.diagnostics().identity_count, 6)


class TestMotionAndFlicker(unittest.TestCase):
    def test_brief_flicker_keeps_stable_id(self):
        # A one/two-frame detection drop (backend revives the SAME raw id) must
        # not churn the stable id — the grace window holds the mapping.
        r = _resolver(grace_seconds=0.6)
        a = r.update([_track(1, cx=100, cy=100)], 640, 480, now=0.0)[0]
        r.update([], 640, 480, now=0.033)               # flicker (within grace)
        r.update([], 640, 480, now=0.066)
        b = r.update([_track(1, cx=120, cy=100)], 640, 480, now=0.10)[0]
        self.assertEqual(b.id, a.id)

    def test_moving_object_rebinds_via_motion_prediction(self):
        # A moving object whose raw id changes after a brief loss re-binds to its
        # id: the resolver predicts its motion forward, so the displaced
        # reappearance falls inside the (time-grown) gate even though a static
        # gate would miss it.
        r = _resolver(max_center_distance=0.05, grace_seconds=0.0)
        a = r.update([_track(1, cx=200, cy=300, vx=600)], 1280, 720, now=0.0)[0]
        # raw 1 gone; 0.2 s later returns as raw 9 displaced 600*0.2=120 px (cx=320)
        out = r.update([_track(9, cx=320, cy=300, vx=600)], 1280, 720, now=0.2)[0]
        self.assertEqual(out.id, a.id)

    def test_static_object_far_jump_still_gets_new_id(self):
        # A STATIONARY identity (no velocity) must not be re-bound to a far jump
        # (fail-closed still holds; motion prediction only helps real motion).
        r = _resolver(max_center_distance=0.05, motion_gate_growth=0.0,
                                  grace_seconds=0.0)
        a = r.update([_track(1, cx=200, cy=300, vx=0)], 1280, 720, now=0.0)[0]
        out = r.update([_track(9, cx=900, cy=300, vx=0)], 1280, 720, now=0.2)[0]
        self.assertNotEqual(out.id, a.id)


class TestLateMerge(unittest.TestCase):
    def test_young_fork_merges_back_when_evidence_arrives(self):
        # Async embeddings lag: a reappearing object is first (fail-closed)
        # given a fresh id; when its embedding arrives and strongly matches an
        # older lost identity, the young id merges back into the old one.
        r = _resolver(max_lost_seconds=10.0, max_center_distance=0.05,
                                  grace_seconds=0.0)
        old = r.update([_track(1, cx=100, cy=100)], 640, 480, now=0.0,
                       embeddings={1: EMB_A})[0]
        r.update([], 640, 480, now=0.1)                       # lost
        # reappears far away, NO embedding yet (lag) -> new id
        young = r.update([_track(9, cx=500, cy=300)], 640, 480, now=1.0)[0]
        self.assertNotEqual(young.id, old.id)
        # next frame the embedding arrives and matches the old gallery
        merged = r.update([_track(9, cx=505, cy=300)], 640, 480, now=1.03,
                          embeddings={9: EMB_A})[0]
        self.assertEqual(merged.id, old.id)                   # fork healed

    def test_no_merge_on_weak_or_old_evidence(self):
        r = _resolver(max_lost_seconds=10.0, max_center_distance=0.05,
                                  grace_seconds=0.0, merge_window=3.0)
        old = r.update([_track(1, cx=100, cy=100)], 640, 480, now=0.0,
                       embeddings={1: EMB_A})[0]
        r.update([], 640, 480, now=0.1)
        young = r.update([_track(9, cx=500, cy=300)], 640, 480, now=1.0)[0]
        # weak appearance (orthogonal) -> no merge
        kept = r.update([_track(9, cx=505, cy=300)], 640, 480, now=1.03,
                        embeddings={9: EMB_B})[0]
        self.assertEqual(kept.id, young.id)
        self.assertNotEqual(kept.id, old.id)


def _view(deg):
    """Unit embedding for a viewpoint at `deg` degrees (rotating object)."""
    r = np.radians(deg)
    return np.array([np.cos(r), np.sin(r)], dtype=np.float32)


class TestRotationRobustness(unittest.TestCase):
    def test_continuous_rotation_keeps_id_and_enrolls_views(self):
        # Object rotates 0->90 deg in 10-deg steps while continuously tracked:
        # appearance drifts far beyond the swap threshold, but smoothly -> the
        # id must never churn and the new viewpoints get enrolled.
        r = _resolver()
        first = r.update([_track(1)], 640, 480, now=0.0, embeddings={1: _view(0)})[0]
        for k in range(1, 10):
            out = r.update([_track(1)], 640, 480, now=0.033 * k,
                           embeddings={1: _view(10 * k)})[0]
            self.assertEqual(out.id, first.id)

    def test_reacquires_with_view_enrolled_before_loss(self):
        # Rotates 0->90 while tracked, lost for 2 s, reappears FAR AWAY showing
        # the 90-deg side view -> the enrolled prototype matches and the object
        # gets its old id back (appearance path, no center gate).
        r = _resolver(max_lost_seconds=10.0, max_center_distance=0.05,
                                  grace_seconds=0.0)
        first = r.update([_track(1, cx=100, cy=100)], 640, 480, now=0.0,
                         embeddings={1: _view(0)})[0]
        for k in range(1, 10):
            r.update([_track(1, cx=100 + 2 * k, cy=100)], 640, 480, now=0.033 * k,
                     embeddings={1: _view(10 * k)})
        r.update([], 640, 480, now=0.5)                       # lost
        out = r.update([_track(9, cx=560, cy=300)], 640, 480, now=2.5,
                       embeddings={9: _view(90)})[0]
        self.assertEqual(out.id, first.id)

    def test_unseen_view_near_position_rebinds_by_geometry(self):
        # Object rotated WHILE LOST: it reappears near its predicted position
        # with a viewpoint never enrolled (ambiguous appearance). Appearance
        # must not permanently fork the id -> geometry+motion re-binds.
        r = _resolver(max_lost_seconds=10.0, grace_seconds=0.0)
        first = r.update([_track(1, cx=300, cy=300)], 1280, 720, now=0.0,
                         embeddings={1: _view(0)})[0]
        r.update([], 1280, 720, now=0.1)
        # 75 deg view: cos-dist ~0.74 -> between threshold (0.5) and contradict (0.9)
        out = r.update([_track(9, cx=310, cy=300)], 1280, 720, now=0.4,
                       embeddings={9: _view(75)})[0]
        self.assertEqual(out.id, first.id)

    def test_contradicting_appearance_still_forks(self):
        # A reappearance whose appearance CONTRADICTS the gallery (opposite
        # vector, beyond the contradiction bar) must NOT take the old id even
        # at the same position (fail-closed).
        r = _resolver(max_lost_seconds=10.0, grace_seconds=0.0)
        first = r.update([_track(1, cx=300, cy=300)], 1280, 720, now=0.0,
                         embeddings={1: _view(0)})[0]
        r.update([], 1280, 720, now=0.1)
        out = r.update([_track(9, cx=305, cy=300)], 1280, 720, now=0.4,
                       embeddings={9: _view(180)})[0]
        self.assertNotEqual(out.id, first.id)

    def test_pixel_jitter_keeps_id(self):
        # Small per-frame embedding noise (pixel-level changes) never churns.
        rng = np.random.default_rng(5)
        r = _resolver()
        first = r.update([_track(1)], 640, 480, now=0.0, embeddings={1: _view(0)})[0]
        for k in range(1, 30):
            noisy = _view(0) + rng.normal(0, 0.05, 2).astype(np.float32)
            noisy /= np.linalg.norm(noisy)
            out = r.update([_track(1)], 640, 480, now=0.033 * k,
                           embeddings={1: noisy})[0]
            self.assertEqual(out.id, first.id)


class TestPinAndHint(unittest.TestCase):
    def test_pinned_identity_never_expires(self):
        r = _resolver(max_lost_seconds=1.0)
        sid = r.update([_track(1)], 640, 480, now=0.0)[0].id
        r.pin(sid)
        r.update([], 640, 480, now=5.0)            # 5s > max_lost_seconds
        # the SAME object returns far later -> still its id (state survived)
        r.update([], 640, 480, now=10.0)
        out = r.update([_track(9, cx=110, cy=80)], 640, 480, now=10.1)[0]
        self.assertEqual(out.id, sid)

    def test_unpinned_identity_expires(self):
        r = _resolver(max_lost_seconds=1.0)
        sid = r.update([_track(1)], 640, 480, now=0.0)[0].id
        r.update([], 640, 480, now=5.0)            # not pinned -> expires
        out = r.update([_track(9, cx=110, cy=80)], 640, 480, now=5.1)[0]
        self.assertNotEqual(out.id, sid)

    def test_position_hint_moves_rebind_anchor(self):
        # Without a hint, an object that doubled back is beyond the gate -> new
        # id. A hint placing the lost anchor at the true spot re-binds it.
        r = _resolver(max_lost_seconds=10.0, max_center_distance=0.05,
                                  motion_gate_growth=0.0, grace_seconds=0.0)
        sid = r.update([_track(1, cx=900, cy=300, vx=600)], 1280, 720, now=0.0)[0].id
        r.update([], 1280, 720, now=0.1)
        # bridge followed the object to x=300 (far from the frozen 900)
        r.hint_position(sid, 300 / 1280, 300 / 720, now=0.3)
        out = r.update([_track(9, cx=305, cy=300)], 1280, 720, now=0.35)[0]
        self.assertEqual(out.id, sid)

    def test_stable_of_maps_raw_to_stable(self):
        r = _resolver()
        sid = r.update([_track(7)], 640, 480, now=0.0)[0].id
        self.assertEqual(r.stable_of(7), sid)
        self.assertIsNone(r.stable_of(999))


class TestGraceWindowRebind(unittest.TestCase):
    """A backend raw-id churn WITHIN the grace window must not fork: the
    in-grace identity is still reachable for re-binding by a new raw id."""

    def test_raw_churn_inside_grace_keeps_id(self):
        r = _resolver(grace_seconds=0.6)
        sid = r.update([_track(1, cx=300, cy=200)], 640, 480, now=0.0)[0].id
        r.update([_track(1, cx=302, cy=200)], 640, 480, now=0.1)
        # backend churns: raw 1 gone, raw 9 appears at the same spot 0.1s later
        out = r.update([_track(9, cx=304, cy=200)], 640, 480, now=0.2)[0]
        self.assertEqual(out.id, sid)
        # the superseded raw mapping is gone: raw 1 must not resolve anymore
        self.assertIsNone(r.stable_of(1))
        self.assertEqual(r.stable_of(9), sid)

    def test_same_raw_flicker_still_revives_in_grace(self):
        r = _resolver(grace_seconds=0.6)
        sid = r.update([_track(1, cx=300, cy=200)], 640, 480, now=0.0)[0].id
        r.update([], 640, 480, now=0.1)                  # one-frame drop
        out = r.update([_track(1, cx=303, cy=200)], 640, 480, now=0.2)[0]
        self.assertEqual(out.id, sid)


class TestZoomSizeGate(unittest.TestCase):
    def test_appearance_match_bypasses_size_gate(self):
        # Across a 3x zoom the stored size error is ln(3)*2 = 2.2 (> the 1.6
        # log-sum gate); a confident appearance match must still re-bind.
        r = _resolver(grace_seconds=0.0)
        emb = np.array([1.0, 0.0], dtype=np.float32)
        sid = r.update([_track(1, cx=300, cy=200, w=40, h=30)], 640, 480,
                       now=0.0, embeddings={1: emb})[0].id
        r.update([], 640, 480, now=0.2)
        out = r.update([_track(9, cx=320, cy=210, w=120, h=90)], 640, 480,
                       now=0.4, embeddings={9: emb})[0]
        self.assertEqual(out.id, sid)

    def test_geometry_path_still_size_gated(self):
        # Without appearance, a 3x size jump (log-sum 2.2 > 1.6) must NOT
        # re-bind (fail-closed).
        r = _resolver(grace_seconds=0.0)
        sid = r.update([_track(1, cx=300, cy=200, w=40, h=30)], 640, 480,
                       now=0.0)[0].id
        r.update([], 640, 480, now=0.2)
        out = r.update([_track(9, cx=302, cy=200, w=120, h=90)], 640, 480,
                       now=0.4)[0]
        self.assertNotEqual(out.id, sid)


class TestGraceRebindBookkeeping(unittest.TestCase):
    def test_rebind_does_not_leak_raw_last_seen(self):
        # A raw-id churn within the grace window rebinds the same identity to a
        # NEW raw id each tick; the released old raw id must be dropped from
        # _raw_last_seen too, or it leaks one dict entry per churn.
        r = _resolver(grace_seconds=0.6)
        r.update([_track(1, cx=300, cy=200)], 640, 480, now=0.0)
        for k in range(1, 25):
            # new raw id each tick at ~the same spot -> rebinds the lost identity
            r.update([_track(100 + k, cx=300 + k, cy=200)], 640, 480, now=0.1 * k)
        # bounded: only the live raw id (+ at most a couple in-grace), NOT 25.
        diagnostics = r.diagnostics()
        self.assertLessEqual(diagnostics.raw_seen_count, 4)
        self.assertLessEqual(diagnostics.raw_binding_count, 4)


class TestResolverDiagnostics(unittest.TestCase):
    def test_reset_clears_state_but_preserves_decision_counters(self):
        resolver = _resolver()
        resolver.update([_track(1)], 640, 480, now=0.0)
        counters_before = dict(resolver.counters)
        self.assertEqual(counters_before["forks"], 1)

        resolver.reset()

        diagnostics = resolver.diagnostics()
        self.assertEqual(diagnostics.identity_count, 0)
        self.assertEqual(diagnostics.raw_binding_count, 0)
        self.assertEqual(diagnostics.raw_seen_count, 0)
        self.assertEqual(diagnostics.counters, counters_before)

    def test_diagnostics_are_a_snapshot(self):
        resolver = _resolver()
        resolver.update([_track(1)], 640, 480, now=0.0)
        diagnostics = resolver.diagnostics()

        resolver.counters["forks"] += 1

        self.assertEqual(diagnostics.counters["forks"], 1)


class TestIdentityObservationCommitter(unittest.TestCase):
    def test_reads_history_by_stable_id_when_raw_mapping_has_changed(self):
        policies = TrackIdentityPolicies()
        counters = IdentityCounters()
        registry = IdentityRegistry(policies.retention)
        gallery = IdentityGallery(policies.appearance)
        committer = IdentityObservationCommitter(
            registry,
            gallery,
            LateIdentityMerger(
                registry,
                policies.appearance,
                policies.retention,
                counters,
            ),
        )
        registry.store(
            IdentityState(
                stable_id=1,
                raw_id=7,
                class_id=2,
                kinematics=IdentityKinematics(
                    nx=0.1,
                    ny=0.1,
                    nw=0.1,
                    nh=0.1,
                    nvx=0.0,
                    nvy=0.0,
                    last_seen=3.0,
                ),
                created_at=3.0,
            )
        )
        registry.bind(7, 2, release_previous=False)

        committer.commit(
            [(_track(7), 1)],
            640,
            480,
            10.0,
            {},
        )

        state = registry.state_of(1)
        self.assertIsNotNone(state)
        self.assertEqual(state.created_at, 3.0)


if __name__ == "__main__":
    unittest.main()
