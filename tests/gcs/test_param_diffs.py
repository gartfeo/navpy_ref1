"""Tests for paramDiffs.js via Node.js subprocess."""
import json
import os
from tests.gcs.js_runner import run_node

import pytest

_JS_PATH = os.path.normpath(
    os.path.join(
        os.path.dirname(__file__),
        "..", "..", "src", "gcs", "frontend", "src", "utils", "paramDiffs.js",
    )
)
_JS_SRC = open(_JS_PATH, encoding="utf-8").read().replace(
    "export function ", "function "
)


def _run(script):
    full = _JS_SRC + "\n" + script
    result = run_node(full, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error: {result.stderr}")
    return json.loads(result.stdout.strip())


class TestComputeParamDiffs:
    """Test computeParamDiffs pure function."""

    def test_single_vehicle_returns_undefined(self):
        """Fewer than 2 selected vehicles → undefined."""
        result = _run(
            'console.log(JSON.stringify('
            'computeParamDiffs("del_pitch", 45, {1: {del_pitch: 45}}, [1])'
            ' ?? null));'
        )
        assert result is None

    def test_empty_selection_returns_undefined(self):
        result = _run(
            'console.log(JSON.stringify('
            'computeParamDiffs("del_pitch", 45, {}, [])'
            ' ?? null));'
        )
        assert result is None

    def test_no_diff_returns_undefined(self):
        """All vehicles match the edit value → undefined."""
        result = _run(
            'console.log(JSON.stringify('
            'computeParamDiffs("del_pitch", 45, '
            '{1: {del_pitch: 45}, 3: {del_pitch: 45}}, [1, 3])'
            ' ?? null));'
        )
        assert result is None

    def test_one_vehicle_differs(self):
        """One vehicle has a different value → shows that vehicle."""
        result = _run(
            'console.log(JSON.stringify('
            'computeParamDiffs("del_pitch", 45, '
            '{1: {del_pitch: 45}, 3: {del_pitch: 50}}, [1, 3])));'
        )
        assert len(result) == 1
        assert result[0]["label"] == "UAV 3"
        assert result[0]["value"] == 50

    def test_all_vehicles_differ(self):
        """All vehicles differ from edit value."""
        result = _run(
            'console.log(JSON.stringify('
            'computeParamDiffs("del_pitch", 99, '
            '{1: {del_pitch: 45}, 3: {del_pitch: 50}}, [1, 3])));'
        )
        assert len(result) == 2
        labels = {d["label"] for d in result}
        assert labels == {"UAV 1", "UAV 3"}

    def test_missing_param_on_vehicle_skipped(self):
        """Vehicle that doesn't have the param is skipped."""
        result = _run(
            'console.log(JSON.stringify('
            'computeParamDiffs("del_pitch", 45, '
            '{1: {del_pitch: 45}, 3: {}}, [1, 3])'
            ' ?? null));'
        )
        assert result is None

    def test_missing_vehicle_entry_skipped(self):
        """Vehicle with no entry in vehicleParams is skipped."""
        result = _run(
            'console.log(JSON.stringify('
            'computeParamDiffs("del_pitch", 45, '
            '{1: {del_pitch: 45}}, [1, 3])'
            ' ?? null));'
        )
        assert result is None

    def test_loose_equality_number_string(self):
        """Loose equality: string '45' should match number 45."""
        result = _run(
            'console.log(JSON.stringify('
            'computeParamDiffs("del_pitch", "45", '
            '{1: {del_pitch: 45}, 3: {del_pitch: 50}}, [1, 3])));'
        )
        # UAV 1 matches via loose equality (45 == "45"), only UAV 3 differs
        assert len(result) == 1
        assert result[0]["label"] == "UAV 3"

    def test_boolean_diff(self):
        """Boolean values show correctly in diffs."""
        result = _run(
            'console.log(JSON.stringify('
            'computeParamDiffs("use_trn", true, '
            '{1: {use_trn: true}, 3: {use_trn: false}}, [1, 3])));'
        )
        assert len(result) == 1
        assert result[0]["label"] == "UAV 3"
        assert result[0]["value"] is False

    def test_three_vehicles_mixed(self):
        """Three vehicles, two differ."""
        result = _run(
            'console.log(JSON.stringify('
            'computeParamDiffs("del_thr", -1, '
            '{1: {del_thr: -1}, 3: {del_thr: 50}, 5: {del_thr: 75}}, '
            '[1, 3, 5])));'
        )
        assert len(result) == 2
        labels = {d["label"] for d in result}
        assert labels == {"UAV 3", "UAV 5"}

    def test_color_idx_default_zero(self):
        """Without idToIndex, colorIdx defaults to 0."""
        result = _run(
            'console.log(JSON.stringify('
            'computeParamDiffs("del_pitch", 45, '
            '{1: {del_pitch: 45}, 3: {del_pitch: 50}}, [1, 3])));'
        )
        assert result[0]["colorIdx"] == 0

    def test_color_idx_from_map(self):
        """With idToIndex, colorIdx matches the vehicle's index."""
        result = _run(
            'console.log(JSON.stringify('
            'computeParamDiffs("del_pitch", 99, '
            '{1: {del_pitch: 45}, 3: {del_pitch: 50}}, '
            '[1, 3], {1: 0, 3: 2})));'
        )
        assert len(result) == 2
        by_label = {d["label"]: d for d in result}
        assert by_label["UAV 1"]["colorIdx"] == 0
        assert by_label["UAV 3"]["colorIdx"] == 2
