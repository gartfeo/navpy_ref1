"""The simulator POI catalog transaction (``lockstep_slice.md``, "NavPy participant").

A mutation (``refresh``, ``set_sim_poi``) holds the mutation lock across
capture, which may park on round trips, pure resolution and publication; a
reader takes only the catalog lock, so it never waits on a round trip.

- Wall mode (real threads): a ``set_sim_poi`` and a second ``refresh`` wait
  for the refresh in progress, the captures never interleave, and readers of
  the provider and of ``SimPoiCatalog`` get the published tuple meanwhile.
- Under the baton: a reader is not blocked while a refresh is parked on a
  parameter read; the terrain report wait (``TERRAIN_ENABLE 0``: no report)
  is 1 s of sim time, then the DEM places the POI.
- Published records are frozen copies.
"""

from __future__ import annotations

import threading
from argparse import Namespace
from unittest.mock import Mock, patch

import pytest

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.location import Location
from navpy.sim.detection.sim_poi_catalog import SimPoiCatalog
from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime
from navpy.sim.world.poi_provider import CatalogInputs, PoiInput, PoiProvider, resolve_catalog
from navpy.sim.world.simulation_object import FrozenLocation

HOME = Location(40.0, 44.0, 1000.0)
TICK_US = 20_000
#: Host supervision of the wall tests' joins (a hang fails, never a timing).
JOIN_S = 10.0


class _Vehicle:
    home_location = HOME
    mission_items_count = 6

    def __init__(self, log: list) -> None:
        self._log = log

    def get_mission_item_location(self, index: int) -> Location:
        self._log.append(("capture", threading.current_thread().name, index))
        return Location(40.0 + index / 1000, 44.0, 0.0)

    def get_param_or_default(self, _name, default):
        return default


class _Args:
    """``NavigationPoiArgs``' stand-in: ``refresh`` is the parameter round trip."""

    def __init__(self, park) -> None:
        self.poi_wp_indices = {1: 1, 2: 2}
        self.poi_alt = 100.0
        self._park = park

    def refresh(self) -> None:
        self._park()


class _SpyLock:
    """The mutation lock, announcing each waiter before it blocks."""

    def __init__(self, lock) -> None:
        self._lock = lock
        self.waiting = threading.Event()

    def __enter__(self):
        if threading.current_thread().name != "first":
            self.waiting.set()
        return self._lock.__enter__()

    def __exit__(self, *exc):
        return self._lock.__exit__(*exc)


def _provider(args, log, zc_util=None) -> PoiProvider:
    with patch("navpy.sim.world.poi_provider.NavigationPoiArgs", return_value=args):
        return PoiProvider(Mock(), _Vehicle(log), zc_util, Mock())


def _wall_parked(log):
    """A provider whose next refresh parks until ``release``; ``entered`` once it has."""
    entered, release = threading.Event(), threading.Event()

    def park() -> None:
        entered.set()
        release.wait(JOIN_S)

    provider = _provider(_Args(park), log)
    provider._mutation_lock = _SpyLock(provider._mutation_lock)
    return provider, entered, release


def _start(name: str, target) -> threading.Thread:
    thread = threading.Thread(target=target, name=name)
    thread.start()
    return thread


@pytest.mark.parametrize("second", ["set_sim_poi", "refresh"])
def test_a_mutation_waits_for_the_refresh_in_progress_and_readers_do_not(second):
    log: list = []
    provider, entered, release = _wall_parked(log)
    catalog = SimPoiCatalog(provider)
    catalog._mutation_lock = _SpyLock(catalog._mutation_lock)
    before = provider.pois
    log.clear()
    first = _start("first", catalog.refresh)
    assert entered.wait(JOIN_S)
    mutate = (lambda: catalog.set_sim_poi(4, Location(41.0, 45.0, 5.0))) if second == "set_sim_poi" \
        else catalog.refresh
    other = _start("second", mutate)
    # The second mutation waits on the catalog's mutation lock; readers are
    # answered at once with the published tuple.
    assert catalog._mutation_lock.waiting.wait(JOIN_S)
    assert provider.pois == before and catalog.snapshot() == before
    release.set()
    first.join(JOIN_S)
    other.join(JOIN_S)
    assert not first.is_alive() and not other.is_alive()
    captures = [who for kind, who, _index in log if kind == "capture"]
    assert captures == sorted(captures, key=["first", "second"].index)  # never interleaved
    if second == "set_sim_poi":
        assert captures == ["first", "first", "second"]
        assert [p.uid for p in provider.pois] == [0, 1, 2]
        assert (provider.pois[2].g_loc.lat, provider.pois[2].g_loc.alt) == (41.0, 1005.0)
    else:
        assert captures == ["first", "first", "second", "second"]
        assert [p.uid for p in provider.pois] == [0, 1]
    assert catalog.snapshot() == provider.pois


def test_a_direct_set_sim_poi_waits_on_the_providers_own_mutation_lock():
    log: list = []
    provider, entered, release = _wall_parked(log)
    log.clear()
    first = _start("first", provider.refresh)
    assert entered.wait(JOIN_S)
    other = _start("second", lambda: provider.set_sim_poi(4, Location(41.0, 45.0, 5.0)))
    assert provider._mutation_lock.waiting.wait(JOIN_S)
    assert [who for _kind, who, _index in log] == []  # the second captured nothing yet
    assert len(provider.pois) == 2
    release.set()
    first.join(JOIN_S)
    other.join(JOIN_S)
    assert [who for _kind, who, _index in log] == ["first", "first", "second"]
    assert len(provider.pois) == 3


def test_an_override_lives_until_the_next_refresh_rebuilds_the_list():
    log: list = []
    provider = _provider(_Args(lambda: None), log)
    provider.set_sim_poi(4, Location(41.0, 45.0, 5.0), location_type="dock")
    assert [p.location_type for p in provider.pois] == [None, None, "dock"]
    provider.refresh()
    assert [p.location_type for p in provider.pois] == [None, None]


def test_published_records_are_frozen_copies():
    provider = _provider(_Args(lambda: None), [])
    poi = provider.pois[0]
    assert isinstance(poi.g_loc, FrozenLocation) and poi.g_loc == Location(40.001, 44.0, 1100.0, 0.0, True)
    with pytest.raises(AttributeError, match="frozen"):
        poi.g_loc.alt = 0.0
    with pytest.raises(AttributeError, match="frozen"):
        poi.uid = 9
    override = Location(41.0, 45.0, 5.0)
    provider.set_sim_poi(4, override)
    override.alt = 99.0  # the caller's location is not the published one
    assert provider.pois[2].height == 5.0 and provider.pois[2].g_loc.alt == 1005.0


def test_resolution_is_a_pure_function_of_the_captured_inputs():
    inputs = CatalogInputs((PoiInput(3, 40.0, 44.0, 50.0, 62.5, 1000.0, "dock"),))
    first, second = resolve_catalog(inputs), resolve_catalog(inputs)
    assert first[0].g_loc == second[0].g_loc == Location(40.0, 44.0, 1062.5, 0.0, True)
    assert (first[0].uid, first[0].height, first[0].location_type) == (0, 50.0, "dock")


# -- under the baton ----------------------------------------------------------------------


@pytest.fixture
def baton(monkeypatch) -> BatonRuntime:
    runtime = BatonRuntime()
    binding = rt.RuntimeBinding()
    binding.bind(LockstepRuntime.create(runtime, epoch_us=0, seed=1, ordinal=1))
    monkeypatch.setattr(rt, "_binding", binding)
    return runtime


def _ticks(baton: BatonRuntime, count: int) -> None:
    for k in range(1, count + 1):
        baton.tick(k * TICK_US)


def test_a_reader_is_not_blocked_while_a_refresh_parks_on_a_parameter_read(baton):
    built: dict = {}
    reads: list = []

    def parameter_read() -> None:  # a round trip: answered five ticks later
        rt.Event().wait(5 * TICK_US / 1e6)

    def build_then_refresh() -> None:
        built["provider"] = provider = _provider(_Args(parameter_read), [])
        built["catalog"] = catalog = SimPoiCatalog(provider)
        built["before"] = catalog.snapshot()
        catalog.refresh()
        built["refreshed at"] = rt.monotonic()

    def reader() -> None:
        while "refreshed at" not in built:
            reads.append((rt.monotonic(), built["catalog"].snapshot() is built["before"]))
            rt.sleep(TICK_US / 1e6)

    baton.spawn(build_then_refresh, name="refresh")
    baton.run_until_quiescent()
    baton.spawn(reader, name="reader")
    _ticks(baton, 8)
    assert built["refreshed at"] == pytest.approx(0.1)
    # The reader ran every tick while the refresh was parked, never waiting on
    # it, and read the tuple published before the refresh.
    parked = [(round(t, 6), unchanged) for t, unchanged in reads if t < built["refreshed at"]]
    assert parked == [(0.02, True), (0.04, True), (0.06, True), (0.08, True)]
    assert baton.threads.alive() == [] and not baton.threads.errors


def test_the_terrain_wait_is_one_second_of_sim_time_then_the_dem(baton):
    """``TERRAIN_ENABLE 0``: the autopilot never reports, so the 1 s report wait
    passes on ticks and the POI is placed from the DEM."""
    built: dict = {}

    class _NoTerrainReport(_Vehicle):
        def terrain_height_at(self, _lat: float, _lng: float):
            rt.Event().wait(1.0)  # the report wait, in sim time
            return None

    dem = Mock()
    dem.get_elevation.side_effect = lambda coords: 1000.0 if coords == [HOME.lat, HOME.lng] else 1012.0

    def build() -> None:
        with patch("navpy.sim.world.poi_provider.NavigationPoiArgs",
                   return_value=_Args(lambda: None)):
            args = Namespace(use_terrain=True)
            provider = PoiProvider(args, _NoTerrainReport([]), dem, Mock())
        built["at"] = rt.monotonic()
        built["altitudes"] = [p.g_loc.alt for p in provider.pois]

    baton.spawn(build, name="build")
    baton.run_until_quiescent()
    _ticks(baton, 120)
    assert built["at"] == pytest.approx(2.0)  # two POIs, one report wait each
    assert built["altitudes"] == pytest.approx([1112.0, 1112.0])
