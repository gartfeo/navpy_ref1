"""Tests for upload_mission_with_retry() in waypoint_builder."""
from __future__ import annotations

from unittest.mock import MagicMock, call

import pytest

from gcs.backend.planner.waypoint_builder import upload_mission_with_retry, UploadResult


@pytest.fixture
def mock_vehicle():
    v = MagicMock()
    v.target_system = 1
    v.link_ok = True
    v.clear_mission = MagicMock()
    v.load_mission_items = MagicMock()
    v.upload_mission = MagicMock(return_value=True)
    v.download_mission = MagicMock(return_value=0)
    return v


class TestUploadMissionWithRetry:
    def test_success_first_attempt(self, mock_vehicle):
        """Upload succeeds and verification matches on first try."""
        # build_mission for 2 waypoints produces home + takeoff + 2 nav = 4 items
        mock_vehicle.download_mission.return_value = 4

        result = upload_mission_with_retry(
            mock_vehicle,
            [{"lat": 32.0, "lon": 34.0}, {"lat": 32.001, "lon": 34.001}],
            altitude_m=150.0,
        )
        assert result.success is True
        assert result.attempts == 1
        assert result.uploaded_count == 4
        assert result.expected_count == 4
        assert result.error is None
        assert result.link_lost is False

    def test_upload_fails_then_succeeds(self, mock_vehicle):
        """Upload returns False on attempt 1, succeeds on attempt 2."""
        mock_vehicle.upload_mission.side_effect = [False, True]
        mock_vehicle.download_mission.return_value = 4

        result = upload_mission_with_retry(
            mock_vehicle,
            [{"lat": 32.0, "lon": 34.0}, {"lat": 32.001, "lon": 34.001}],
            altitude_m=150.0,
        )
        assert result.success is True
        assert result.attempts == 2

    def test_verification_mismatch_then_succeeds(self, mock_vehicle):
        """Verification count mismatch on first attempt, matches on second."""
        mock_vehicle.download_mission.side_effect = [2, 4]

        result = upload_mission_with_retry(
            mock_vehicle,
            [{"lat": 32.0, "lon": 34.0}, {"lat": 32.001, "lon": 34.001}],
            altitude_m=150.0,
        )
        assert result.success is True
        assert result.attempts == 2

    def test_all_retries_exhausted(self, mock_vehicle):
        """All retries fail — returns failure."""
        mock_vehicle.upload_mission.return_value = False

        result = upload_mission_with_retry(
            mock_vehicle,
            [{"lat": 32.0, "lon": 34.0}, {"lat": 32.001, "lon": 34.001}],
            altitude_m=150.0,
            max_retries=3,
        )
        assert result.success is False
        assert result.attempts == 3
        assert "3 attempts" in result.error
        assert result.link_lost is False

    def test_link_lost_stops_immediately(self, mock_vehicle):
        """Link lost during upload stops immediately with link_lost=True."""
        mock_vehicle.upload_mission.return_value = False
        mock_vehicle.link_ok = False

        result = upload_mission_with_retry(
            mock_vehicle,
            [{"lat": 32.0, "lon": 34.0}, {"lat": 32.001, "lon": 34.001}],
            altitude_m=150.0,
            max_retries=3,
        )
        assert result.success is False
        assert result.link_lost is True
        assert result.attempts == 1
        # Should not retry after link loss
        assert mock_vehicle.upload_mission.call_count == 1

    def test_empty_track_returns_immediate_success(self, mock_vehicle):
        """Empty track list returns immediate success without calling vehicle."""
        result = upload_mission_with_retry(
            mock_vehicle,
            [],
            altitude_m=150.0,
        )
        assert result.success is True
        assert result.uploaded_count == 0
        assert result.expected_count == 0
        assert result.attempts == 0
        mock_vehicle.clear_mission.assert_not_called()

    def test_progress_callback_invoked(self, mock_vehicle):
        """on_progress callback is called for each stage."""
        mock_vehicle.download_mission.return_value = 4
        stages = []

        def on_progress(stage, attempt):
            stages.append((stage, attempt))

        upload_mission_with_retry(
            mock_vehicle,
            [{"lat": 32.0, "lon": 34.0}, {"lat": 32.001, "lon": 34.001}],
            altitude_m=150.0,
            on_progress=on_progress,
        )
        assert ("clearing", 1) in stages
        assert ("uploading", 1) in stages
        assert ("verifying", 1) in stages

    def test_clear_mission_called_each_attempt(self, mock_vehicle):
        """clear_mission is called on every attempt."""
        mock_vehicle.upload_mission.side_effect = [False, True]
        mock_vehicle.download_mission.return_value = 4

        upload_mission_with_retry(
            mock_vehicle,
            [{"lat": 32.0, "lon": 34.0}, {"lat": 32.001, "lon": 34.001}],
            altitude_m=150.0,
        )
        assert mock_vehicle.clear_mission.call_count == 2
