"""Unit tests for SettingsStore — persistence and merge logic."""
import json
import tempfile
from pathlib import Path

import pytest
from pydantic import ValidationError

from gcs.backend.settings_model import GcsSettings
from gcs.backend.settings_store import SETTINGS_PATH_ENV, SettingsStore


@pytest.fixture
def tmp_path_file(tmp_path):
    return tmp_path / "test_settings.json"


class TestSettingsStore:
    def test_environment_path_is_used_when_constructor_path_is_omitted(self, tmp_path, monkeypatch):
        isolated_path = tmp_path / "isolated" / "gcs_settings.json"
        isolated_path.parent.mkdir()
        isolated_path.write_text(
            json.dumps({"camera": {"vision_profile": "siyi_zr10"}}),
            encoding="utf-8",
        )
        monkeypatch.setenv(SETTINGS_PATH_ENV, str(isolated_path))

        store = SettingsStore()

        assert store.get().camera.vision_profile == "siyi_zr10"
        store.update({"simulation": {"simulated_vehicle_count": 3}})
        assert json.loads(isolated_path.read_text(encoding="utf-8"))["simulation"][
            "simulated_vehicle_count"
        ] == 3

    def test_defaults_when_no_file(self, tmp_path_file):
        store = SettingsStore(path=tmp_path_file)
        settings = store.get()
        assert isinstance(settings, GcsSettings)
        assert settings.flight.cruise_speed_ms == 22.0
        # Demo (sim) + dev are enabled by default now.
        assert settings.simulation.sim_mode is True

    def test_update_persists(self, tmp_path_file):
        store = SettingsStore(path=tmp_path_file)
        updated = store.update({"flight": {"cruise_speed_ms": 30.0}})
        assert updated.flight.cruise_speed_ms == 30.0
        # Verify file was written
        assert tmp_path_file.exists()
        data = json.loads(tmp_path_file.read_text(encoding="utf-8"))
        assert data["flight"]["cruise_speed_ms"] == 30.0

    def test_reload_from_file(self, tmp_path_file):
        store1 = SettingsStore(path=tmp_path_file)
        store1.update({"flight": {"cruise_speed_ms": 25.0}})
        # New store instance loads persisted data
        store2 = SettingsStore(path=tmp_path_file)
        assert store2.get().flight.cruise_speed_ms == 25.0

    def test_deep_merge_preserves_other_fields(self, tmp_path_file):
        store = SettingsStore(path=tmp_path_file)
        store.update({"flight": {"cruise_speed_ms": 30.0}})
        store.update({"flight": {"flight_budget_km": 100.0}})
        settings = store.get()
        # First update preserved
        assert settings.flight.cruise_speed_ms == 30.0
        # Second update applied
        assert settings.flight.flight_budget_km == 100.0
        # Unmodified fields intact
        assert settings.flight.safety_reserve_km == 20.0

    def test_reset_clears_to_defaults(self, tmp_path_file):
        store = SettingsStore(path=tmp_path_file)
        store.update({"flight": {"cruise_speed_ms": 99.0}})
        restored = store.reset()
        assert restored.flight.cruise_speed_ms == 22.0
        # File should be defaults now
        store2 = SettingsStore(path=tmp_path_file)
        assert store2.get().flight.cruise_speed_ms == 22.0

    def test_update_simulation_sim_mode(self, tmp_path_file):
        store = SettingsStore(path=tmp_path_file)
        assert store.get().simulation.sim_mode is True
        store.update({"simulation": {"sim_mode": False}})
        assert store.get().simulation.sim_mode is False

    def test_dev_mode_defaults_true(self, tmp_path_file):
        # Demo + dev are enabled by default now.
        store = SettingsStore(path=tmp_path_file)
        assert store.get().simulation.dev_mode is True

    def test_update_dev_mode(self, tmp_path_file):
        store = SettingsStore(path=tmp_path_file)
        store.update({"simulation": {"dev_mode": True}})
        assert store.get().simulation.dev_mode is True

    def test_dev_mode_independent_of_sim_mode(self, tmp_path_file):
        store = SettingsStore(path=tmp_path_file)
        store.update({"simulation": {"sim_mode": True, "dev_mode": True}})
        assert store.get().simulation.sim_mode is True
        assert store.get().simulation.dev_mode is True
        store.update({"simulation": {"sim_mode": False}})
        assert store.get().simulation.sim_mode is False
        assert store.get().simulation.dev_mode is True

    def test_corrupt_file_falls_back_to_defaults(self, tmp_path_file):
        tmp_path_file.write_text("not valid json{{{", encoding="utf-8")
        store = SettingsStore(path=tmp_path_file)
        settings = store.get()
        assert settings.flight.cruise_speed_ms == 22.0

    def test_bom_prefixed_file_is_loaded_not_replaced_by_defaults(self, tmp_path_file):
        """Windows PowerShell 5.1 and some editors write UTF-8 with a BOM.
        Such a file must load as written: falling back to defaults would
        silently turn sim_mode and dev_mode back on for a hardware setup."""
        hardware = {"simulation": {"sim_mode": False, "dev_mode": False}}
        tmp_path_file.write_bytes(b"\xef\xbb\xbf" + json.dumps(hardware).encode("utf-8"))
        settings = SettingsStore(path=tmp_path_file).get()
        assert settings.simulation.sim_mode is False
        assert settings.simulation.dev_mode is False

    def test_legacy_aas_pruned_on_load(self, tmp_path_file):
        """A pre-Step-6 settings file with an `aas` key must be loaded
        successfully AND have the `aas` key physically removed from the
        file on disk -- not merely ignored by Pydantic. This makes the
        migration deterministic across upgrades and reloads."""
        legacy = {
            "flight": {"cruise_speed_ms": 25.0},
            "aas": {"del_pitch": 10.0, "nav_auto_cm": False},
        }
        tmp_path_file.write_text(json.dumps(legacy), encoding="utf-8")
        # Loading the store should: (a) succeed, (b) preserve the
        # non-aas data, (c) rewrite the file without the aas key.
        store = SettingsStore(path=tmp_path_file)
        assert store.get().flight.cruise_speed_ms == 25.0
        on_disk = json.loads(tmp_path_file.read_text(encoding="utf-8"))
        assert "aas" not in on_disk
        assert on_disk["flight"]["cruise_speed_ms"] == 25.0

    def test_legacy_aas_null_also_pruned(self, tmp_path_file):
        """An explicit `"aas": null` (some legacy save paths emit this
        instead of a populated dict) must also be pruned -- detect by
        key presence, not non-null value."""
        legacy = {
            "flight": {"cruise_speed_ms": 22.0},
            "aas": None,
        }
        tmp_path_file.write_text(json.dumps(legacy), encoding="utf-8")
        store = SettingsStore(path=tmp_path_file)
        assert store.get().flight.cruise_speed_ms == 22.0
        on_disk = json.loads(tmp_path_file.read_text(encoding="utf-8"))
        assert "aas" not in on_disk

    def test_no_rewrite_when_aas_absent(self, tmp_path_file):
        """A clean settings file (no aas key) must NOT be rewritten on
        load -- the file content is byte-identical after store init."""
        clean = {"flight": {"cruise_speed_ms": 22.0}}
        tmp_path_file.write_text(json.dumps(clean), encoding="utf-8")
        original_text = tmp_path_file.read_text(encoding="utf-8")
        SettingsStore(path=tmp_path_file)
        assert tmp_path_file.read_text(encoding="utf-8") == original_text

    def test_non_dict_json_falls_back_to_defaults(self, tmp_path_file):
        """Valid JSON with the wrong shape (e.g. a list) must not crash
        the prune step; it should fall through to validation, fail, and
        return factory defaults."""
        tmp_path_file.write_text("[]", encoding="utf-8")
        store = SettingsStore(path=tmp_path_file)
        assert store.get().flight.cruise_speed_ms == 22.0

    def test_legacy_tcp_companion_presets_migrate_to_udp(self, tmp_path_file):
        """Persisted TCP-era companion presets (tcp:<host>:<5760+10k>) are
        rewritten to the UDP server form in memory AND on disk. Non-companion
        entries are genuine user edits and must survive untouched."""
        legacy = {
            "simulation": {
                "sitl_presets": [
                    "tcp:127.0.0.1:5760",
                    "tcp:172.23.113.35:5800",
                    "udp:0.0.0.0:14560",          # legacy router-era preset: not companion-band
                    "tcp:127.0.0.1:14550",        # not a companion port: user edit
                ],
            },
        }
        tmp_path_file.write_text(json.dumps(legacy), encoding="utf-8")
        store = SettingsStore(path=tmp_path_file)
        assert store.get().simulation.sitl_presets == [
            "udp:0.0.0.0:5760",
            "udp:0.0.0.0:5800",
            "udp:0.0.0.0:14560",
            "tcp:127.0.0.1:14550",
        ]
        on_disk = json.loads(tmp_path_file.read_text(encoding="utf-8"))
        assert on_disk["simulation"]["sitl_presets"][0] == "udp:0.0.0.0:5760"

    def test_default_presets_are_companion_udp_binds(self, tmp_path_file):
        """Fresh installs (no settings file) must default to the UDP companion
        binds — a tcp regression would pass the migration/injection tests yet
        leave non-launcher runs dialing a port nothing listens on."""
        store = SettingsStore(path=tmp_path_file)
        assert store.get().simulation.sitl_presets == [
            "udp:0.0.0.0:5760",
            "udp:0.0.0.0:5770",
            "udp:0.0.0.0:5780",
        ]

    def test_udp_presets_not_rewritten_on_load(self, tmp_path_file):
        """A post-migration file must not be rewritten again on load."""
        clean = {"simulation": {"sitl_presets": ["udp:0.0.0.0:5760"]}}
        tmp_path_file.write_text(json.dumps(clean), encoding="utf-8")
        original_text = tmp_path_file.read_text(encoding="utf-8")
        SettingsStore(path=tmp_path_file)
        assert tmp_path_file.read_text(encoding="utf-8") == original_text

    def test_takeoff_altitude_default_positive(self, tmp_path_file):
        store = SettingsStore(path=tmp_path_file)
        assert store.get().flight.takeoff_altitude_m == 40.0

    @pytest.mark.parametrize("bad", [0, -5.0])
    def test_takeoff_altitude_rejects_non_positive(self, tmp_path_file, bad):
        """A zero/negative safe-takeoff height must be rejected (it would
        upload an unsafe NAV_TAKEOFF altitude); stored settings stay intact."""
        store = SettingsStore(path=tmp_path_file)
        with pytest.raises(ValidationError):
            store.update({"flight": {"takeoff_altitude_m": bad}})
        # Update rejected → stored value unchanged (default), file not written.
        assert store.get().flight.takeoff_altitude_m == 40.0
        assert not tmp_path_file.exists()

    def test_takeoff_altitude_accepts_positive(self, tmp_path_file):
        store = SettingsStore(path=tmp_path_file)
        updated = store.update({"flight": {"takeoff_altitude_m": 25.0}})
        assert updated.flight.takeoff_altitude_m == 25.0
