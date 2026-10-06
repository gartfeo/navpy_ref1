"""SCRATCH: how many aircraft configurations can one worktree fly at once?

Launches N aircraft from ONE `run_swarm.sh` and gives each one its own PROCESS,
its own connection, its own configuration and its own result file. The parent
launches, waits, and tabulates; it never touches a vehicle itself.

WHY PROCESSES. The previous version ran one thread per aircraft inside this
process. At N=8 that process reached 0.973 CPU, starved its own readers, and
three aircraft returned a single ATTITUDE sample for an entire hold -- which the
harness reported as those aircraft failing to track a commanded pitch. A thread
cannot escape the GIL; the limit was the probe, not the machine.

ONE CHANNEL PER AIRCRAFT IS NOT THE ROUTER FLAG. Each child connects on
`companion_device(sysid)`, a dedicated per-vehicle UDP bind fed by that SITL's
serial0. That is true at every setting of `--router`: `router_win_ports`
publishes only the two GCS-facing ports and states companions are "not
router-exported" (instance_ports.py:186-193), and `launch_command` sets
COMPANION_UDP=1 unconditionally (swarm_run_wsl.py:126-127).

So `--no-router` does NOT change what a companion port carries, and an earlier
version of this docstring claiming it did was wrong. What it changes is that
`mavlink-routerd` is not running at all -- one fewer process competing for the
same cores. The A/B that motivated the default measured per-aircraft ATTITUDE at
21.0-27.9 Hz with the router up and 41.0-41.3 Hz with it down; that gap is real
and reproducible, but it is a CPU effect, not a channel-content effect.

Cross-talk on companion ports is also real and separately observed: before the
source filter went in, a single link counted 40 Hz at N=1 rising to 201 Hz at
N=5. Whatever relays neighbours onto a dedicated port, the code above says it is
not the router. Every reader therefore filters on srcSystem regardless.

A PASSING LADDER FINDS NO CEILING. Every level up to the largest one tested has
passed, so the supported statement is "at least N", never "N is the limit".

`SWARM_OFFSET` is a BASE sysid, not a chat: `swarm_run_runner.py:134` sets it to
`VEHICLES_PER_CHAT * chat` and the swarm numbers vehicles `offset+1 .. offset+N`.
N is therefore not capped at VEHICLES_PER_CHAT -- three-per-chat is a GCS
convention, not a SITL limit. Running past it reaches the NEXT chats' sysids, and
teardown kills by sysid pattern, so every level CLAIMS its whole span through
`instance_registry.claim` and holds it until after teardown, all-or-nothing.
Other sessions share this machine and the blast radius is their work, not this
probe's policy to opt out of.

A starved SITL still integrates its physics correctly in SIM time. It reports
perfectly plausible attitudes while its seconds quietly stop being seconds, so
the clock is measured directly rather than inferred from message cadence.

Delete once the scalability question is closed out.
"""

from __future__ import annotations

import argparse
import inspect
import json
import os
import statistics
import subprocess
import sys
import time
from pathlib import Path

WORKTREE = Path(__file__).resolve().parent.parent
SCRIPTS = Path(__file__).resolve().parent
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE), str(SCRIPTS)]

import swarm_run_wsl as wsl  # noqa: E402
from gcs.backend import instance_ports as ip  # noqa: E402
from gcs.backend import instance_registry as reg  # noqa: E402

# Below this the simulator is not keeping up and every duration it reports is
# stretched. Not a tuning knob: 0.95 means 5 % of real time already went
# missing, which is enough to move a final-approach error.
CLOCK_FIDELITY_FLOOR = 0.95

# Above this the number is not a fast clock, it is a MEASUREMENT FAULT. The one
# way to outrun the expected rate is to time queued telemetry as if it had just
# arrived. An earlier version did exactly that and reported 18.3x as healthy.
CLOCK_FIDELITY_CEILING = 1.5


def _winds(raw: str | None) -> list[tuple[float, float]]:
    """Parse `speed:dir,speed:dir` into per-aircraft wind configurations."""
    pairs: list[tuple[float, float]] = []
    for piece in (raw or "").split(","):
        piece = piece.strip()
        if not piece:
            continue
        speed, _, direction = piece.partition(":")
        pairs.append((float(speed), float(direction or 0.0)))
    return pairs


def _pitches(raw: str | None) -> list[float]:
    return [float(x) for x in (raw or "").split(",") if x.strip()]


def _machine_load() -> dict[str, object]:
    """What is ALREADY running on this box, measured before we launch.

    This machine is shared. A level that fails while somebody else's SITL is
    burning the cores has not found this harness's ceiling -- it has found the
    machine's current spare capacity, which is a different number and not a
    reproducible one. Recorded per level so a FAIL can be attributed rather than
    assumed, and so a ladder run under load can be thrown out instead of
    believed.

    WSL is where the aircraft live and where the sim clock is made, so its load
    average is the one that matters; the Windows core count bounds the children.

    `arduplane_running` is the decisive field, NOT the load average. `loadavg` is
    a trailing one-minute mean, so immediately after a level's teardown it still
    reports the load that level made -- measured at 12.36 on 10 CPUs while the
    aircraft count was already 0 and the box genuinely idle. Reading the average
    as current occupancy would throw out clean levels.
    """
    # LABELLED output, parsed by key, never by line number. The previous version
    # emitted bare values and read them positionally, with `pgrep -fc X || echo 0`
    # as a fallback -- but `pgrep -c` PRINTS "0" before it fails, so the fallback
    # added a second line and shifted every field after it. Measured on an idle
    # box: six lines where four were expected, and the router count was read from
    # the arduplane fallback. A running router would have been recorded as zero.
    #
    # `[a]rduplane`, not `arduplane`: `pgrep -f` matches full command lines
    # INCLUDING its own, and this probe's command line contains the pattern, so
    # the plain spelling can never return 0.
    probe = subprocess.run(
        wsl.wsl_argv(
            "echo load1=$(cut -d' ' -f1 /proc/loadavg); "
            "echo cpus=$(nproc); "
            "echo aircraft=$(pgrep -fc '[a]rduplane'); "
            "echo routers=$(pgrep -fc '[m]avlink-routerd')"
        ),
        capture_output=True,
    )
    fields: dict[str, str] = {}
    for line in probe.stdout.decode("utf-8", "replace").splitlines():
        key, _, value = line.strip().partition("=")
        if value:
            fields[key] = value

    def _number(key: str, cast: type[int] | type[float]) -> int | float | None:
        # Missing stays None, never 0: "nothing else is running" is exactly the
        # claim this function exists to stop anyone assuming.
        try:
            return cast(fields[key])
        except (KeyError, ValueError):
            return None

    return {
        "windows_logical_cpus": os.cpu_count(),
        "wsl_loadavg_1m": _number("load1", float),
        "wsl_cpus": _number("cpus", int),
        "arduplane_running": _number("aircraft", int),
        "routers_running": _number("routers", int),
    }


_CLK_TCK: list[int] = []


def _clk_tck() -> int:
    """Guest jiffies per second, asked for once rather than assumed.

    /proc/stat is denominated in USER_HZ. It is 100 nearly everywhere, but the
    delivery figure below divides by it, so a wrong constant would silently
    scale the one number that decides whether a saturated VM was actually
    getting the host.
    """
    if not _CLK_TCK:
        value = 100
        try:
            probe = subprocess.run(
                wsl.wsl_argv("getconf CLK_TCK"), capture_output=True
            )
            parsed = int(probe.stdout.decode("utf-8", "replace").strip())
            if parsed > 0:
                value = parsed
        except (ValueError, OSError):
            pass
        _CLK_TCK.append(value)
    return _CLK_TCK[0]


def _cpu_counters() -> dict[str, object]:
    """Cumulative CPU-time counters on both sides of the WSL boundary.

    Counters, not instantaneous percentages: two readings a window apart give
    the TRUE average over that window, where a point sample would just catch
    whatever the box happened to be doing at one instant.

    WSL is read per core, because "the machine is 50 % busy" and "one core is
    pegged and nine are idle" are the same aggregate number and completely
    different diagnoses. Windows matters because the children run there, on the
    same physical silicon the WSL VM is scheduled on.
    """
    # Wall time the counters were read at. A guest's jiffies alone cannot say
    # whether the host actually ran the VM; jiffies PER WALL SECOND can.
    counters: dict[str, object] = {"wsl": {}, "windows": None, "epoch": time.time()}
    probe = subprocess.run(
        wsl.wsl_argv("grep '^cpu' /proc/stat"), capture_output=True
    )
    for line in probe.stdout.decode("utf-8", "replace").splitlines():
        fields = line.split()
        if len(fields) < 5:
            continue
        try:
            values = [int(v) for v in fields[1:]]
        except ValueError:
            continue
        # idle + iowait: both are time the core had nothing to run.
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        counters["wsl"][fields[0]] = (idle, sum(values))
    try:
        import ctypes
        from ctypes import wintypes

        idle_ft, kernel_ft, user_ft = (
            wintypes.FILETIME(), wintypes.FILETIME(), wintypes.FILETIME()
        )
        if ctypes.windll.kernel32.GetSystemTimes(
            ctypes.byref(idle_ft), ctypes.byref(kernel_ft), ctypes.byref(user_ft)
        ):
            def _ticks(value: wintypes.FILETIME) -> int:
                return (value.dwHighDateTime << 32) | value.dwLowDateTime
            # Windows' "kernel" time INCLUDES idle, so total is kernel + user.
            counters["windows"] = (
                _ticks(idle_ft), _ticks(kernel_ft) + _ticks(user_ft)
            )
    except (AttributeError, OSError, ImportError):
        # Not Windows, or the call is unavailable: reported as unknown rather
        # than as zero.
        pass
    return counters


def _cpu_utilisation(before: dict, after: dict) -> dict[str, object]:
    """Average CPU utilisation between two counter readings.

    The point of measuring this at all: a level whose clock collapsed while the
    cores sat half idle did NOT run out of CPU, and calling it a capacity limit
    would be wrong. Utilisation is what separates "the machine is full" from
    "something is serialising and the machine is waiting".
    """
    def _percent(
        pair_before: tuple[int, int] | None, pair_after: tuple[int, int] | None,
    ) -> float | None:
        if not pair_before or not pair_after:
            return None
        idle = pair_after[0] - pair_before[0]
        total = pair_after[1] - pair_before[1]
        if total <= 0:
            return None
        return round(100.0 * (1.0 - idle / total), 1)

    cores = {
        name: _percent(before["wsl"].get(name), after["wsl"].get(name))
        for name in after.get("wsl", {})
        if name != "cpu"
    }
    measured = [value for value in cores.values() if value is not None]

    # CPU DELIVERED per wall-second, which the percentages above cannot express.
    # `wsl_total_pct` is idle/total INSIDE the guest: when the host does not
    # schedule the VM, those jiffies are never accrued at all, so a starved VM
    # whose processes stayed runnable reports near-100% busy while receiving
    # almost no CPU. Reading that as a capacity ceiling is the original false
    # failure this whole check exists to prevent.
    #
    # A VM that is fully scheduled accrues vcpus x elapsed seconds of jiffies,
    # busy and idle alike. The shortfall against that is the host's doing.
    delivery = None
    pair_before, pair_after = before["wsl"].get("cpu"), after["wsl"].get("cpu")
    elapsed_s = (after.get("epoch") or 0.0) - (before.get("epoch") or 0.0)
    if pair_before and pair_after and elapsed_s > 0 and cores:
        accrued_s = (pair_after[1] - pair_before[1]) / _clk_tck()
        expected_s = len(cores) * elapsed_s
        if expected_s > 0:
            delivery = round(min(1.0, accrued_s / expected_s), 3)
    return {
        "wsl_total_pct": _percent(before["wsl"].get("cpu"), after["wsl"].get("cpu")),
        "wsl_core_max_pct": max(measured) if measured else None,
        "wsl_core_min_pct": min(measured) if measured else None,
        "wsl_cores": cores,
        "wsl_delivery_frac": delivery,
        "windows_total_pct": _percent(before.get("windows"), after.get("windows")),
    }


class CpuSampler:
    """Utilisation sampled REPEATEDLY through the window, not once across it.

    A single before/after pair yields only the mean, and the mean cannot tell
    these apart:

    * every core pegged for the whole window -- a real capacity ceiling
    * a burst of saturation followed by waiting -- something serialising
    * one saturated thread MIGRATING across cores, which a window-averaged
      per-core reading smears into partial utilisation everywhere and can hide
      completely

    So the peak of short samples is reported next to the mean. They agree when
    the load is steady and diverge exactly when the mean is misleading.
    """

    def __init__(self, period_s: float = 2.0) -> None:
        self._period_s = period_s
        self._first = _cpu_counters()
        self._last = self._first
        self._next_s = time.monotonic() + period_s
        # Wall-clock, not monotonic: these are compared against timestamps taken
        # in the CHILD processes, and monotonic's reference point is undefined
        # across processes. Absolute time is the only clock both sides share.
        self._last_epoch = time.time()
        self.samples: list[dict[str, object]] = []

    def maybe_sample(self) -> None:
        now_s = time.monotonic()
        if now_s < self._next_s:
            return
        current = _cpu_counters()
        now_epoch = time.time()
        sample = _cpu_utilisation(self._last, current)
        # The interval this sample COVERS, so it can be matched against the
        # phase a shortfall was measured in.
        sample["from_epoch"] = self._last_epoch
        sample["to_epoch"] = now_epoch
        self.samples.append(sample)
        self._last = current
        self._last_epoch = now_epoch
        self._next_s = now_s + self._period_s

    def summary(self) -> dict[str, object]:
        overall = _cpu_utilisation(self._first, _cpu_counters())
        def _peak(key: str) -> float | None:
            values = [
                s[key] for s in self.samples if s.get(key) is not None
            ]
            return max(values) if values else None
        overall["wsl_total_peak_pct"] = _peak("wsl_total_pct")
        overall["wsl_core_max_peak_pct"] = _peak("wsl_core_max_pct")
        overall["windows_total_peak_pct"] = _peak("windows_total_pct")
        overall["samples"] = len(self.samples)
        # Every sample, not just its extremes. A max says a burst happened
        # somewhere; the series says whether the box was busy THROUGHOUT, which
        # is the difference between a capacity ceiling and one bad moment.
        overall["wsl_total_series_pct"] = [
            s.get("wsl_total_pct") for s in self.samples
        ]
        overall["series"] = self.samples
        return overall

    @staticmethod
    def mean_over(
        samples: list[dict[str, object]],
        intervals: list[tuple[float, float]],
        key: str = "wsl_total_pct",
    ) -> tuple[float | None, int]:
        """Mean WSL utilisation across samples overlapping ANY given interval.

        The reason this exists: clock fidelity is measured over the hold alone
        (the child clears its samples before it, scratch_sitl_uav.py), while
        this sampler runs from launch to teardown. A whole-window mean therefore
        answers a different question than the shortfall it was being used to
        explain -- sustained load during the climb could certify a hold-only
        collapse. Overlap, not containment: a 1s sample boundary must not
        discard a 3s hold.

        A LIST of intervals, not one span. Aircraft do not hold in lockstep, and
        an earlier version took min(starts) to max(ends) -- a bounding box with
        the gaps between holds inside it, so a burst while NOTHING was being
        measured counted as evidence about the measurement.
        """
        spans = [
            (a, b) for a, b in intervals
            if isinstance(a, (int, float)) and isinstance(b, (int, float))
        ]
        hit = [
            s[key] for s in samples
            if s.get(key) is not None
            and s.get("from_epoch") is not None and s.get("to_epoch") is not None
            and any(s["from_epoch"] < b and s["to_epoch"] > a for a, b in spans)
        ]
        if not hit:
            return None, 0
        return round(statistics.fmean(hit), 3), len(hit)


class AircraftWatch:
    """Counts arduplane processes DURING a level, not just before it.

    The one way foreign work can genuinely saturate WSL's own cores is by
    running inside WSL -- another session launching SITL into the same utility
    VM. Host-side foreign work cannot do it: WSL's counters measure the VM's
    vCPU threads, so host contention shows up as the VM getting less wall-time,
    not as its cores reading busier. In-VM aircraft are different, and they
    would hand this probe exactly the saturation it treats as proof of its own
    ceiling.

    The pre-launch count catches none of that, because it stops before the work
    starts. Counted at a coarse cadence: each count spawns wsl.exe, so sampling
    it per CPU-sample would perturb the very measurement it protects.
    """

    def __init__(self, expected: int, period_s: float = 5.0) -> None:
        self._expected = expected
        self._period_s = period_s
        self._next_s = time.monotonic() + period_s
        self.peak: int | None = None

    def maybe_count(self) -> None:
        now_s = time.monotonic()
        if now_s < self._next_s:
            return
        self._next_s = now_s + self._period_s
        running = _machine_load().get("arduplane_running")
        if isinstance(running, int):
            self.peak = running if self.peak is None else max(self.peak, running)

    def strangers(self) -> int | None:
        """How many aircraft appeared beyond the ones this level launched."""
        if self.peak is None:
            return None
        return max(0, self.peak - self._expected)


def _verdict(row: dict) -> str:
    """PASS, FAIL or VOID -- and VOID is not a polite FAIL.

    A level whose clock fell short while another session was taking cores has
    not found a ceiling; it has found nothing. Printing that as FAIL is what
    produced a capacity number that had to be withdrawn.
    """
    if row.get("void"):
        return "VOID (no answer): " + "; ".join(row.get("voids") or [])
    if row.get("passed"):
        return "PASS"
    return "FAIL: " + "; ".join(row.get("faults") or [])


def _foreign_windows_estimate(cpu: dict, load: dict) -> float | None:
    """ESTIMATE host work that is not this probe's, DURING a level.

    The pre-launch gate samples one window before anything is running, so it
    cannot see a neighbour that starts work mid-level -- and that is enough to
    turn a healthy level into a capacity failure.

    Windows' total counts the WSL vCPU threads too, so subtracting WSL's share
    of the host leaves roughly what else the box was doing.

    KNOWN BLIND SPOT, measured, not theorised. Windows' total saturates at
    100%, so as WSL fills the host the remainder floors out -- and this reads
    LOWEST exactly when the box is busiest. From one ladder (artefacts
    sitl-scale-20260814-215809), by level: wsl 41.9% / win 73.4% gave 31.5,
    while wsl 98.2% / win 100.0% gave 1.8. The four passing levels estimated
    24.8-32.1 and the failing ones 1.8-21.2, i.e. the ranking came out
    BACKWARDS against the verdicts.

    So: a high reading means company was present. A low one means nothing at
    all once the host is pinned. Never read it as an amount, and never treat a
    low value as evidence the box was quiet.
    """
    windows = cpu.get("windows_total_pct")
    wsl = cpu.get("wsl_total_pct")
    wsl_cpus = load.get("wsl_cpus")
    host_cpus = load.get("windows_logical_cpus")
    if None in (windows, wsl, wsl_cpus, host_cpus) or not host_cpus:
        return None
    return round(max(0.0, windows - wsl * (wsl_cpus / host_cpus)), 1)


def _wait_for_idle(
    idle_load: float, timeout_s: float, aircraft_allowed: int = 0,
    idle_windows_pct: float | None = None,
) -> dict[str, object]:
    """Block until the box has actually settled, then report what it looks like.

    Three conditions, because none alone is enough:

    * no arduplane process. Exact and immediate, but a box can be free of
      aircraft while still working through the wreckage of the last level.
    * one-minute load average under `idle_load`. Trailing and slow, which is
      precisely why it catches what the process count misses -- but it is a
      DECAYING average, so it reads high for up to a minute after a clean
      teardown and must never be treated as current occupancy on its own.
    * WINDOWS-side CPU under `idle_windows_pct`. Both readings above are taken
      inside WSL and are blind to the host. WSL2's vCPUs are host threads, so
      other work on Windows takes cores straight off the simulator. This box
      runs several sessions at once, and skipping this check let foreign load
      decide a capacity result: two N=48 levels with WSL CPU 63.6% and 64.6%
      -- the same box, by the only measure taken -- returned clock ratios of
      1.0104 and 0.8889. With no aircraft running, whatever Windows is doing
      here is by definition not ours.

    Returns the final reading with `waited_s` and whether it gave up, so a level
    measured on a box that never settled is labelled instead of believed.
    """
    started_s = time.monotonic()
    while True:
        load = _machine_load()
        aircraft = load["arduplane_running"]
        average = load["wsl_loadavg_1m"]
        foreign_pct = None
        if idle_windows_pct is not None:
            before = _cpu_counters()
            time.sleep(2.0)
            foreign_pct = _cpu_utilisation(before, _cpu_counters())["windows_total_pct"]
        load["windows_foreign_pct"] = foreign_pct
        # THREE states, not two. `GetSystemTimes` is absent off Windows and can
        # fail on it (_cpu_counters:202 leaves the counter out), which yields
        # None here -- and accepting None as "quiet" let the check turn itself
        # off and still report the level as settled. A gate that silently
        # disables itself is worse than no gate: no gate leaves the level
        # visibly unverified, this one stamped it verified. Unmeasurable does
        # not block (on a box where the counter never works that would stall
        # every level to its timeout and fail the lot), but it is recorded, and
        # the caller says so on the level.
        load["windows_foreign_measured"] = not (
            idle_windows_pct is not None and foreign_pct is None
        )
        # `aircraft_allowed`, not a hard 0. The count was hard-coded here while
        # --foreign-aircraft-max claimed to permit some, so raising that flag
        # made the wait run to its full timeout and the level fail as "never
        # settled" -- the option promised tolerance and delivered a stall.
        settled = (
            aircraft is not None
            and aircraft <= aircraft_allowed
            and (average is None or average <= idle_load)
            and (idle_windows_pct is None or foreign_pct is None
                 or foreign_pct <= idle_windows_pct)
        )
        waited_s = time.monotonic() - started_s
        if settled or waited_s >= timeout_s:
            load["waited_s"] = round(waited_s, 1)
            load["settled"] = bool(settled)
            return load
        time.sleep(5.0)


def _chats_spanned(sysids: list[int]) -> list[int]:
    """Every chat whose nominal sysids this run would touch."""
    return sorted({(sysid - 1) // ip.VEHICLES_PER_CHAT for sysid in sysids})


def _claim_span(sysids: list[int]) -> list[int]:
    """CLAIM every chat in the blast radius, or take none of them.

    Reading `reg.live()` and then pkilling is check-then-act: another session can
    claim and launch inside that window, and `pkill -f -- --sysid N` does not ask
    who started the process. `reg.claim` allocates under the registry's own lock,
    which is what makes the later teardown safe. All-or-nothing: a partial span
    is released rather than run, because the levels that need the extra chats are
    exactly the ones whose teardown would reach into them.
    """
    owner = reg.owner_for(str(WORKTREE))
    wanted = _chats_spanned(sysids)
    held: list[int] = []
    for chat in wanted:
        try:
            entry = reg.claim(
                label="sitl-eval", clone=str(WORKTREE), branch="",
                owner=owner, prefer=chat, lo=chat, hi=chat,
                launcher_pid=os.getpid(),
            )
        except Exception as error:  # noqa: BLE001 - any refusal means: do not run
            _release_span(held)
            raise SystemExit(
                f"refusing to run: cannot claim chat {chat} of span {wanted} "
                f"({type(error).__name__}: {error}). Another session most "
                "likely holds it; teardown here kills by sysid and would take "
                "their aircraft down."
            ) from error
        # `chat_index`, not `chat` -- the persisted shape is built by
        # `instance_registry_claims.new_entry:47-53`. An earlier guard read
        # `entry["chat"]`, matched nothing, and was silently inert.
        granted = entry.get("chat_index") if isinstance(entry, dict) else None
        if granted != chat:
            _release_span(held + ([granted] if granted is not None else []))
            raise SystemExit(
                f"refusing to run: asked for chat {chat}, registry granted "
                f"{granted!r}. Span {wanted} is not exclusively ours."
            )
        # A bare claim does NOT survive this run. `entries.is_alive` consults the
        # 45 s startup grace, the backend port, the backend pid, then
        # `sitl_alive` -- never `launcher_pid`. This probe runs no backend, so
        # once the grace expires the entry becomes prunable and a competing claim
        # can take the chat out from under a run still in progress. Registering
        # as the chat's SITL supervisor is the one liveness signal that tracks
        # THIS process for as long as it lives.
        try:
            reg.begin_sitl_launch(chat, supervisor_pid=os.getpid())
        except Exception as error:  # noqa: BLE001 - cannot hold it, do not run
            _release_span(held + [chat])
            raise SystemExit(
                f"refusing to run: claimed chat {chat} but could not register as "
                f"its SITL supervisor ({type(error).__name__}: {error}), so the "
                "claim would expire mid-run."
            ) from error
        held.append(chat)
    return held


def _release_span(chats: list[int]) -> None:
    """Give back only the chats THIS process still holds.

    `entries.release` is an unconditional pop. If our entry was pruned and
    replaced while we ran, releasing by chat number alone would delete the
    replacing session's entry -- turning our cleanup into their outage.
    """
    mine = reg.owner_for(str(WORKTREE))
    for chat in chats:
        try:
            entry = reg.get(chat)
            if entry is None:
                continue
            if entry.get("owner") != mine or entry.get("sitl_pid") != os.getpid():
                print(
                    f"  chat {chat}: entry is no longer ours "
                    f"(owner={entry.get('owner')!r}, sitl_pid="
                    f"{entry.get('sitl_pid')!r}) -- leaving it alone",
                    flush=True,
                )
                continue
            reg.release(chat)
        except Exception:  # noqa: BLE001 - teardown must not mask the result
            pass


def _cleanup_sysids(chat: int, sysids: list[int]) -> None:
    """Terminate exactly the aircraft this probe launched, plus its router.

    NOT `wsl.cleanup(chat)`: that builds its pattern list from
    `ip.sysids_for_chat(chat)`, always exactly VEHICLES_PER_CHAT entries
    (instance_ports.py:143-146). This probe deliberately launches past that, so
    at N>3 the extra aircraft would survive teardown and contend with the next
    level -- making later levels look slower for a reason that has nothing to do
    with scale.
    """
    patterns = [wsl.sysid_process_pattern(sysid) for sysid in sysids]
    patterns.append(wsl.router_process_pattern(chat))
    terminate = "; ".join(f"pkill -f -- '{p}' || true" for p in patterns)
    kill = "; ".join(f"pkill -KILL -f -- '{p}' || true" for p in patterns)
    wsl.run_wsl(f"{terminate}; sleep 2; {kill}")


def _spawn(
    python: Path, directory: Path, sysid: int, pitch: float | None,
    wind: tuple[float, float] | None, args: argparse.Namespace,
    throttle: float | None = None, roll: float | None = None,
) -> subprocess.Popen[bytes]:
    """One child process for one aircraft, on its own connection."""
    directory.mkdir(parents=True, exist_ok=True)
    command = [
        str(python), str(SCRIPTS / "scratch_sitl_uav.py"),
        "--connection", ip.companion_device(sysid),
        "--sysid", str(sysid),
        "--result", str(directory / "result.json"),
        "--roll-deg", repr(args.roll if roll is None else roll),
        "--throttle", repr(args.throttle if throttle is None else throttle),
        "--hold-s", repr(args.hold_s),
        "--settle-s", repr(args.settle_s),
        "--climb-to-m", repr(args.climb_to_m),
        "--floor-m", repr(args.floor_m),
        "--connect-timeout", repr(args.connect_timeout),
        "--speedup", repr(args.speedup),
    ]
    if pitch is not None:
        command += ["--pitch-deg", repr(pitch)]
    if wind is not None:
        command += ["--wind-speed", repr(wind[0]), "--wind-dir", repr(wind[1])]
    (directory / "child.cmd.json").write_text(
        json.dumps(command, indent=2), encoding="utf-8"
    )
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(WORKTREE / "src")
    environment["PYTHONUTF8"] = "1"
    return subprocess.Popen(
        command,
        cwd=str(WORKTREE),
        env=environment,
        stdout=(directory / "child.out.log").open("wb"),
        stderr=(directory / "child.err.log").open("wb"),
    )


def _await_children(
    children: dict[int, subprocess.Popen[bytes]], timeout_s: float,
    sampler: "CpuSampler | None" = None, watch: "AircraftWatch | None" = None,
) -> list[int]:
    """Wait for every child, returning the sysids that had to be killed."""
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        if sampler is not None:
            sampler.maybe_sample()
        if watch is not None:
            watch.maybe_count()
        if all(child.poll() is not None for child in children.values()):
            return []
        time.sleep(0.2)
    late: list[int] = []
    for sysid, child in children.items():
        if child.poll() is None:
            late.append(sysid)
            child.kill()
    # REAP them before anyone reads their output. `kill()` only sends the
    # signal; returning here left the parent racing a child that could still be
    # part-way through writing result.json, and the reader's `json.loads` would
    # then raise on a truncated file and abort the whole level -- losing every
    # other aircraft's result to one hung child.
    for sysid in late:
        try:
            children[sysid].wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass
    return late


def _run_level(level: int, root: Path, args: argparse.Namespace,
               repeat: int = 1) -> dict[str, object]:
    """Launch, fly and measure *level* aircraft, then tear the whole thing down."""
    pitches = _pitches(args.pitches)
    winds = _winds(args.winds)
    throttles = _pitches(args.throttles)
    rolls = _pitches(args.rolls)
    # Throttle and roll only reach the aircraft inside SET_ATTITUDE_TARGET,
    # which is sent only when a pitch is commanded. Sweeping either with
    # nothing commanded produces a grid no aircraft ever received -- a null
    # experiment that looks like a completed one.
    for name, values in (("--throttles", throttles), ("--rolls", rolls)):
        if values and not pitches:
            raise SystemExit(
                f"{name} needs --pitches: the value is carried in "
                "SET_ATTITUDE_TARGET, and with no commanded pitch that message "
                "is never sent, so every cell of the sweep would fly "
                "identically."
            )
    chat = args.chat
    offset = ip.VEHICLES_PER_CHAT * chat
    sysids = [offset + i for i in range(1, level + 1)]
    # A MAVLink system id is a uint8 (instance_ports.py:53). Past 255 the launch
    # would produce ids no vehicle can hold and chats past MAX_CHAT_INDEX, and
    # the failure would surface much later as aircraft that never answer --
    # looking exactly like the capacity ceiling this harness exists to find.
    if sysids[-1] > ip.MAX_SYSID:
        raise SystemExit(
            f"N={level} from chat {chat} needs sysids up to {sysids[-1]}, above "
            f"MAX_SYSID {ip.MAX_SYSID}. The most this chat can address is "
            f"{ip.MAX_SYSID - offset}."
        )
    if _chats_spanned(sysids)[-1] > ip.MAX_CHAT_INDEX:
        raise SystemExit(
            f"N={level} from chat {chat} spans up to chat "
            f"{_chats_spanned(sysids)[-1]}, above MAX_CHAT_INDEX "
            f"{ip.MAX_CHAT_INDEX}."
        )
    # Repeats of one level are the whole point of a confidence run, and they used
    # to share a directory: the second N=48 overwrote the first's result.json, so
    # a failure's per-aircraft evidence was destroyed by the pass that followed
    # it. Only the summary survived.
    # The speed is in the name too. Sweeping speedups reruns the same N, and
    # keyed on level alone the 1x artefacts would be overwritten by the 10x
    # ones -- the same collision repeats already caused, one dimension out.
    level_root = root / (
        f"n{level:02d}-x{args.speedup:g}"
        + ("" if repeat == 1 else f"-r{repeat}")
    )
    win_ports = ",".join(str(p) for p in ip.router_win_ports(chat))
    # `home_coords` is passed ONLY when asked for. It is not in HEAD's
    # `launch_command` (swarm_run_wsl.py) -- the support currently exists as an
    # uncommitted working-tree edit -- so passing it unconditionally made this
    # harness die with a TypeError on a clean checkout, before ever launching.
    # A probe must not require a change it does not ship.
    launch_kwargs: dict[str, object] = {}
    if args.home:
        if "home_coords" not in inspect.signature(wsl.launch_command).parameters:
            raise SystemExit(
                "--home needs `home_coords` support in "
                "swarm_run_wsl.launch_command, which this checkout does not "
                "have. Drop --home to use the launcher's default start point."
            )
        launch_kwargs["home_coords"] = args.home
    command = wsl.launch_command(
        args.speedup, offset, win_ports, level, args.distance, **launch_kwargs
    )
    if not args.router:
        # `run_swarm.sh:318` gates the router on RUN_ROUTER, and :106 defaults it
        # to 1. Prefixing works because launch_command emits `VAR=v ... cmd`.
        # Clearing ROUTER_WIN_PORTS instead would NOT disable it: :138 falls back
        # to a generated default when the value is empty.
        command = "RUN_ROUTER=0 " + command
    print(f"=== N={level} : sysids {sysids[0]}..{sysids[-1]} "
          f"router={'on' if args.router else 'off'} ===", flush=True)

    # BEFORE the claim and before any launch, so it describes the box we are
    # about to measure on rather than the one we just loaded.
    #
    # WAITING, not just reporting. Levels run back to back, and a level launched
    # while the previous one's teardown is still settling does not measure the
    # same machine: N=40 passed at ratio 0.995 after N=36, then FAILED twice --
    # once losing two aircraft, once at ratio 0.596 -- when launched at
    # loadavg 11.8 and 34.9. Without this wait the ladder measures the gradient
    # it created rather than the capacity of the box.
    load = _wait_for_idle(
        args.idle_load, args.idle_timeout, args.foreign_aircraft_max,
        idle_windows_pct=args.idle_windows_pct,
    )
    print(
        f"  before launch: wsl load1={load['wsl_loadavg_1m']}"
        f" over {load['wsl_cpus']} wsl cpus"
        f" ({load['windows_logical_cpus']} windows logical),"
        f" arduplane already running={load['arduplane_running']},"
        f" routers={load['routers_running']},"
        f" foreign windows cpu={load.get('windows_foreign_pct')}%,"
        f" waited {load.get('waited_s')}s"
        f" {'(settled)' if load.get('settled') else '(NOT SETTLED)'}",
        flush=True,
    )
    busy = (
        isinstance(load["arduplane_running"], int)
        and load["arduplane_running"] > args.foreign_aircraft_max
    )

    # Ownership BEFORE any pkill, and held for the whole level -- not a snapshot
    # another session can invalidate between the check and the kill.
    held = _claim_span(sysids)
    swarm: subprocess.Popen[bytes] | None = None
    children: dict[int, subprocess.Popen[bytes]] = {}
    try:
        # Inside the try so a failure here still releases the claim.
        _cleanup_sysids(chat, sysids)
        swarm = subprocess.Popen(
            wsl.wsl_argv(command),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        started_s = time.monotonic()
        # Cycled position by position, so `--pitches a,b --winds x,y` pairs into
        # (a,x) (b,y) (a,x) ... -- the two lists are NOT crossed.
        configs = {
            sysid: (
                pitches[index % len(pitches)] if pitches else None,
                winds[index % len(winds)] if winds else None,
                throttles[index % len(throttles)] if throttles else None,
                rolls[index % len(rolls)] if rolls else None,
            )
            for index, sysid in enumerate(sysids)
        }
        children = {
            sysid: _spawn(
                args.python.resolve(), level_root / f"uav-{sysid}", sysid,
                configs[sysid][0], configs[sysid][1], args, configs[sysid][2],
                configs[sysid][3],
            )
            for sysid in sysids
        }
        # Spans the whole child window -- connect, settle, climb and hold -- not
        # the hold alone, which the parent cannot see the boundaries of.
        sampler = CpuSampler(args.cpu_sample_s)
        # Ours plus whatever the pre-launch check already tolerated; anything
        # past that arrived after we started looking.
        watch = AircraftWatch(level + max(0, args.foreign_aircraft_max))
        late = _await_children(children, args.child_timeout, sampler, watch)
        cpu = sampler.summary()
        wall_s = time.monotonic() - started_s

        per_uav: dict[int, dict[str, object]] = {}
        for sysid in sysids:
            path = level_root / f"uav-{sysid}" / "result.json"
            try:
                per_uav[sysid] = json.loads(path.read_text(encoding="utf-8"))
            except FileNotFoundError:
                per_uav[sysid] = {
                    "sysid": sysid, "passed": False,
                    "errors": [f"no result file (exit {children[sysid].returncode})"],
                }
            except (OSError, ValueError) as error:
                # A child killed part-way through its write leaves truncated
                # JSON. That is one aircraft's failure, not the level's: letting
                # the decode error propagate would discard every other
                # aircraft's result too.
                per_uav[sysid] = {
                    "sysid": sysid, "passed": False,
                    "errors": [f"unreadable result: {type(error).__name__}: {error}"],
                }

        ratios = [
            row["clock_ratio"] for row in per_uav.values()
            if isinstance(row.get("clock_ratio"), (int, float))
        ]
        rates = [
            row["attitude_hz"] for row in per_uav.values()
            if isinstance(row.get("attitude_hz"), (int, float))
        ]
        flew = [
            sysid for sysid, row in per_uav.items()
            if float(row.get("max_rel_alt_m") or 0.0) >= args.airborne_m
        ]
        # A level passes only if the machine did the WHOLE job: every aircraft
        # produced a result, got airborne, held what it was told, and kept an
        # honest clock. Judging on the clock alone would score a level "ok" while
        # half its aircraft never left the ground -- the surviving few keep good
        # time precisely BECAUSE the others are missing.
        faults: list[str] = []
        # Facts worth reading next to the numbers, but not reasons to fail.
        notes: list[str] = []
        # Reasons this level cannot answer the capacity question either way.
        voids: list[str] = []
        # Every way the box was known to be someone else's during this level.
        contaminated: list[str] = []
        # A ceiling measured on a box that was already loaded is somebody else's
        # spare capacity, not this harness's limit. Said out loud rather than
        # left for a reader to infer from a number that looks like a result.
        if busy:
            notes.append(
                f"{load['arduplane_running']} foreign arduplane process(es) were "
                f"already running before launch (wsl load1="
                f"{load['wsl_loadavg_1m']}) -- treat any ceiling from this level "
                "as spare capacity, not as a limit"
            )
        if not load.get("windows_foreign_measured", True):
            notes.append(
                "foreign Windows load could not be read before launch, so the "
                "box was NOT confirmed quiet -- the pre-launch check passed "
                "only because it had nothing to test"
            )
        # ---- Facts first, classification after. Everything below reads these,
        # and an earlier arrangement computed them a hundred lines further down
        # than the contamination check that used them: `hold_delivery` was read
        # before it was assigned, so every completed level raised
        # UnboundLocalError. Keep derivations above the block that consumes them.
        def _hold_span(row: dict[str, object]) -> tuple[float, float] | None:
            a, b = row.get("hold_from_epoch"), row.get("hold_to_epoch")
            return (a, b) if isinstance(a, (int, float)) and isinstance(
                b, (int, float)) else None
        all_holds = [s for s in map(_hold_span, per_uav.values()) if s]
        # `clock slow` is min(ratios) -- ONE aircraft's shortfall. The evidence
        # that it was a capacity ceiling has to be the load while THAT aircraft
        # was being measured, not while the fleet collectively was.
        slowest = min(
            (row for row in per_uav.values()
             if isinstance(row.get("clock_ratio"), (int, float))),
            key=lambda row: row["clock_ratio"], default=None,
        )
        slowest_hold = _hold_span(slowest) if slowest else None
        evidence_holds = [slowest_hold] if slowest_hold else all_holds
        hold_wsl_pct, hold_samples = CpuSampler.mean_over(
            cpu.get("series") or [], evidence_holds,
        )
        hold_delivery, _ = CpuSampler.mean_over(
            cpu.get("series") or [], evidence_holds, key="wsl_delivery_frac",
        )
        # The pre-launch gate cannot see a neighbour that starts work after the
        # level does, and that alone can sink a clock. Reported whatever the
        # verdict: naming it only on failures would quietly excuse the bad
        # levels while leaving the passes unqualified.
        foreign_during = _foreign_windows_estimate(cpu, load)
        if foreign_during is not None and foreign_during > args.idle_windows_pct:
            contaminated.append(
                f"about {foreign_during:.0f}% of the host went to work that is "
                f"not this probe DURING the level (over the "
                f"{args.idle_windows_pct:.0f}% bar the box had to clear to "
                "start), estimated by subtracting WSL's share of the host -- "
                "read it as company present, not as an amount"
            )
        # Aircraft that are not ours, running INSIDE WSL, saturate the very
        # counters this level reads as its own ceiling. This is the one foreign
        # path that can manufacture saturation rather than hide it.
        # The host-side path, which no process count and no Windows-percentage
        # estimate can see: the VM was scheduled for less wall-time than it
        # asked for. Its internal busy/idle ratio stays high while it is being
        # starved, so this is the ONLY reading that separates "our simulators
        # filled the machine" from "the machine was taken away from them".
        if hold_delivery is not None and hold_delivery < args.vm_delivery_min:
            contaminated.append(
                f"the WSL VM received only {hold_delivery:.0%} of the CPU-time "
                f"its {load.get('wsl_cpus')} vCPUs should accrue during the hold "
                f"(bar {args.vm_delivery_min:.0%}) -- the host did not run it, so "
                "however busy it looked inside, that is not this level's ceiling"
            )
        strangers = watch.strangers()
        if strangers:
            contaminated.append(
                f"{strangers} arduplane process(es) beyond the {level} this "
                f"level launched appeared DURING it (peak {watch.peak}) -- "
                "foreign aircraft in the same WSL VM saturate the same cores, so "
                "saturation here is not evidence of THIS level's ceiling"
            )
        # A level the box never settled for is not a measurement of the box.
        if not load.get("settled"):
            contaminated.append(
                f"box never settled before launch (waited {load.get('waited_s')}s, "
                f"load1={load['wsl_loadavg_1m']}, aircraft="
                f"{load['arduplane_running']}, foreign windows cpu="
                f"{load.get('windows_foreign_pct')}%)"
            )
        broken = {
            sysid: row.get("errors")
            for sysid, row in per_uav.items() if row.get("errors")
        }
        if broken:
            faults.append(
                f"{len(broken)}/{level} failed: "
                + "; ".join(f"{s}={e}" for s, e in sorted(broken.items()))
            )
        if late:
            faults.append(
                f"{len(late)}/{level} killed at the {args.child_timeout:g}s "
                f"child timeout: {','.join(str(s) for s in late)}"
            )
        if len(flew) < level:
            faults.append(f"{level - len(flew)}/{level} never got airborne")
        if pitches:
            # A commanded attitude the aircraft never took is not a pass. An
            # early run reported passed=true with errors of 2.5-39.9 deg -- every
            # aircraft sitting at trim while the verdict called the run good.
            untracked = [
                (sysid, row["pitch_error_deg"])
                for sysid, row in per_uav.items()
                if isinstance(row.get("pitch_error_deg"), (int, float))
                and abs(row["pitch_error_deg"]) > args.pitch_tol_deg
            ]
            if untracked:
                faults.append(
                    f"{len(untracked)}/{level} never tracked commanded pitch "
                    f"(worst {max(abs(e) for _, e in untracked):.1f} deg)"
                )
        floored = [s for s, r in per_uav.items() if r.get("hold_end") == "condition"]
        if floored:
            notes.append(
                f"{len(floored)}/{level} hit the {args.floor_m:g} m floor "
                "before the hold ended"
            )
        short = [s for s, r in per_uav.items() if r.get("climb_end") == "limit"]
        if short:
            notes.append(
                f"{len(short)}/{level} never reached {args.climb_to_m:g} m "
                "before the climb cap"
            )
        stalled = [s for s, r in per_uav.items() if "stalled" in
                   (r.get("climb_end"), r.get("hold_end"))]
        if stalled:
            faults.append(
                f"{len(stalled)}/{level} stalled -- the aircraft clock stopped "
                f"advancing: {','.join(str(s) for s in stalled)}"
            )
        # What the utilisation MEANS for a level whose clock fell short. Written
        # out rather than left in the artefact, because "the machine ran out of
        # capacity" and "the machine sat waiting" produce the same failing clock
        # ratio and point at opposite fixes.
        clock_short = bool(ratios) and min(ratios) < CLOCK_FIDELITY_FLOOR
        overall = cpu.get("wsl_total_pct")
        windows = cpu.get("windows_total_pct")
        saturated = args.cpu_saturated_pct
        # PEAK for the per-core test, mean for the aggregate. A core pegged for
        # two seconds of a thirty-second window averages ~7 % and vanishes from
        # the mean entirely, so testing the window-averaged per-core figure --
        # which is what this did -- can only ever find load that was saturated
        # nearly the whole time. The peaks were computed and then used for
        # nothing but printing.
        peak = cpu.get("wsl_core_max_peak_pct")
        busiest = peak if peak is not None else cpu.get("wsl_core_max_pct")
        overall_peak = cpu.get("wsl_total_peak_pct")
        # How much of the run was actually saturated, for the reader. Decides
        # nothing on its own -- a fraction needs a threshold to become a verdict
        # and this file already has one too many.
        series = [v for v in (cpu.get("wsl_total_series_pct") or []) if v is not None]
        saturated_frac = (
            round(sum(1 for v in series if v >= args.cpu_saturated_pct) / len(series), 2)
            if series else None
        )
        if clock_short and overall is not None:
            # Every case gets a verdict, including the saturated one. An earlier
            # version only spoke when WSL was BELOW the threshold, so a genuine
            # capacity ceiling -- the one outcome the ladder exists to find --
            # was the single case that produced no explanation at all.
            #
            # These name what was OBSERVED. None of them names a remedy: CPU
            # utilisation cannot tell you whether work is distributable. A
            # thread migrating across cores, a per-instance serialisation and a
            # genuinely parallel load that happens to fit all read the same
            # here. Deciding that needs per-process or affinity-pinned
            # measurement, or a split-workload A/B across two machines -- none
            # of which this probe does.
            if overall >= saturated:
                notes.append(
                    f"clock fell short with WSL at {overall:.0f}% across "
                    f"{load['wsl_cpus']} cores -- SATURATED for the whole "
                    "window, consistent with a real capacity ceiling"
                )
            elif overall_peak is not None and overall_peak >= saturated:
                notes.append(
                    f"clock fell short with WSL averaging {overall:.0f}% but "
                    f"PEAKING at {overall_peak:.0f}% -- saturated in bursts, "
                    "not throughout, so the mean understates the load"
                )
            elif busiest is not None and busiest >= saturated:
                notes.append(
                    f"clock fell short while WSL averaged only {overall:.0f}% "
                    f"across {load['wsl_cpus']} cores yet some core peaked at "
                    f"{busiest:.0f}% -- consistent with ONE saturated thread, "
                    "but utilisation alone cannot separate that from a thread "
                    "migrating between cores; confirm with per-process or "
                    "affinity-pinned measurement before acting on it"
                )
            elif windows is not None and windows >= saturated:
                # The WSL VM is scheduled on the same physical cores as the
                # children. WSL can look idle precisely BECAUSE it is being
                # starved from the Windows side, so reporting "not CPU-bound"
                # off the WSL number alone would name the wrong bottleneck.
                notes.append(
                    f"clock fell short with WSL at only {overall:.0f}% but "
                    f"WINDOWS at {windows:.0f}% -- the host side is "
                    "saturated while the VM is not, which is what VM starvation "
                    "looks like. WHICH host work is responsible is not "
                    "measured here; this probe's children are only one "
                    "candidate"
                )
            else:
                notes.append(
                    f"clock fell short while NOTHING was saturated (wsl "
                    f"{overall:.0f}%, busiest core {busiest}%, windows "
                    f"{windows}%) -- this level was not CPU-bound at all, so it "
                    "is not evidence of a capacity ceiling. The cores were "
                    "waiting on something; what, this probe does not measure"
                )
        # The REQUESTED rate against what the aircraft actually run at. Without
        # this the launch argument is decorative: `launch_command` asked for 1.0
        # for this harness's whole life while every aircraft ran at the eeprom's
        # 10.0, and a "5x versus 10x" sweep would have compared two runs at the
        # same rate and drawn a conclusion from the difference.
        wrong_rate = sorted(
            sysid for sysid, row in per_uav.items()
            if isinstance(row.get("sim_speedup"), (int, float))
            and abs(row["sim_speedup"] - args.speedup) > 1e-6
        )
        if wrong_rate:
            got = per_uav[wrong_rate[0]]["sim_speedup"]
            faults.append(
                f"{len(wrong_rate)}/{level} ran at SIM_SPEEDUP {got:g}, not the "
                f"requested {args.speedup:g} -- the launcher did not apply the "
                "request, so this level does not measure the rate it claims"
            )
        # WSL owns only part of the machine, and that split is invisible in any
        # Windows-side reading. Surfaced when saturated because it is the
        # cheapest untested headroom there is.
        if (
            isinstance(load.get("wsl_cpus"), int)
            and isinstance(load.get("windows_logical_cpus"), int)
            and load["wsl_cpus"] < load["windows_logical_cpus"]
            and overall is not None and overall >= saturated
        ):
            notes.append(
                f"WSL is capped at {load['wsl_cpus']} of "
                f"{load['windows_logical_cpus']} logical CPUs and saturated "
                f"them. That cap is a .wslconfig `processors` setting, not a "
                "hardware limit -- and it is global to the single WSL2 utility "
                "VM, so a second distro shares it and would not add capacity"
            )
        if not ratios:
            faults.append("no clock samples")
        else:
            clock_faults = []
            if min(ratios) < CLOCK_FIDELITY_FLOOR:
                clock_faults.append(f"clock slow ({min(ratios):.3f})")
            if max(ratios) > CLOCK_FIDELITY_CEILING:
                clock_faults.append(
                    f"clock reading implausible ({max(ratios):.3f}) -- "
                    "measurement fault, not a result"
                )
            # A THIRD verdict, because two cannot say this. Foreign host work
            # steals exactly what the clock measures, so a shortfall on a
            # contaminated box is not evidence of a ceiling -- but it used to be
            # recorded as `clock slow`, which reads identically to one, with the
            # contamination demoted to a note beside it. That is the error this
            # whole ladder exists to avoid, and it is the reason a capacity
            # number was claimed and withdrawn earlier today. Void is not a
            # pass and not a failure: it is "run it again on a quiet box".
            #
            # Contamination is only the case where foreign load was CAUGHT.
            # The catcher is blind under saturation (_foreign_windows_estimate),
            # so keying the void solely on it leaves the original hole open:
            # foreign work that starts mid-level, pins the host, and starves the
            # VM produces a slow clock, an estimate near zero, and a FAIL that
            # reads as a ceiling.
            #
            # The mechanism closes it without measuring foreign load at all. A
            # capacity ceiling means the simulator GOT the cores and still could
            # not keep time -- so WSL itself must have saturated. A clock that
            # fell short while WSL never saturated is a VM that was starved or
            # something that serialised; either way this ladder cannot call it
            # capacity. The notes below already said exactly this and the level
            # still came out FAIL.
            # SUSTAINED, WHOLE-VM saturation. Both exclusions are deliberate.
            #
            # `busiest` (one core) is out: a single core at the bar while the
            # aggregate sits below it means the other cores were FREE -- the VM
            # did not run out of anything. The note at :870 already says
            # utilisation cannot tell that from one thread migrating across
            # cores and must be confirmed before being acted on.
            #
            # `overall_peak` is out: it is max() over the 2-second samples
            # (CpuSampler.summary), and the sampler spans the whole child
            # lifetime -- launch, connect, settle, climb, hold -- while the
            # clock shortfall is measured over the HOLD alone. One burst while
            # 48 simulators start would otherwise certify a shortfall that
            # happened in a different phase entirely.
            #
            # And the mean must be the mean OVER THE HOLD. The whole-window one
            # spans launch, connect, settle and climb; the shortfall is measured
            # over the hold alone, so sustained load in another phase could
            # certify a collapse it had nothing to do with. `hold_wsl_pct` is
            # built from the children's own hold timestamps.
            #
            # No overlapping samples means no evidence, which is NOT the same as
            # evidence of a quiet box: it voids rather than certifies.
            # BOTH readings required, and each must exist. Guest utilisation says
            # the VM was busy; delivery says the host actually ran it. A missing
            # delivery figure is an unanswered question, not a pass -- claiming
            # otherwise last round while the code still certified on utilisation
            # alone is exactly the gap being closed here.
            wsl_saturated = (
                hold_wsl_pct is not None and hold_wsl_pct >= saturated
                and hold_delivery is not None
                and hold_delivery >= args.vm_delivery_min
            )
            unattributable = bool(clock_faults) and not wsl_saturated
            if clock_faults and (contaminated or unattributable):
                if contaminated:
                    reason = ", ".join(contaminated)
                elif hold_wsl_pct is None:
                    reason = (
                        "no CPU sample overlapped the hold the clock was "
                        f"measured over, so nothing attributable was recorded "
                        f"(whole-run mean {overall}% covers other phases and "
                        "cannot stand in for it)"
                    )
                elif hold_delivery is None:
                    reason = (
                        f"the hold looked busy ({hold_wsl_pct}%) but how much "
                        "CPU the VM actually RECEIVED could not be measured, and "
                        "guest busy/idle alone cannot tell a full machine from a "
                        "starved one"
                    )
                else:
                    reason = (
                        f"WSL was not saturated DURING THE HOLD the clock was "
                        f"measured over (hold mean {hold_wsl_pct}% from "
                        f"{hold_samples} sample(s), bar {saturated:.0f}%; vm "
                        f"delivery {hold_delivery}). Not counted: whole-run mean "
                        f"{overall}%, aggregate peak {overall_peak}%, busiest "
                        f"core {busiest}% -- those cover launch, settle and "
                        "climb too"
                    )
                voids.extend(
                    f"{fault} -- NOT a capacity result: {reason}"
                    for fault in clock_faults
                )
            else:
                faults.extend(clock_faults)
        # Contamination that did NOT void anything still gets said. A level that
        # kept its clock while the box was busy is a real pass, and a stronger
        # one than a pass on a quiet box -- but the reader has to be told which
        # of the two they are looking at.
        if contaminated and not voids:
            notes.extend(contaminated)
        return {
            "level": level,
            "repeat": repeat,
            "artifact": level_root.name,
            "requested": level,
            "airborne": len(flew),
            "wall_s": round(wall_s, 1),
            "machine_load_before": load,
            "cpu_during": cpu,
            "foreign_windows_est_pct": foreign_during,
            "wsl_saturated_sample_frac": saturated_frac,
            "hold_wsl_pct": hold_wsl_pct,
            "hold_cpu_samples": hold_samples,
            "hold_vm_delivery_frac": hold_delivery,
            "aircraft_peak_during": watch.peak,
            "foreign_aircraft_during": strangers,
            "router": bool(args.router),
            # Requested next to measured. A speed sweep is only interpretable if
            # each row says which speed it ASKED for, not just what the
            # aircraft happened to report back.
            "speedup_requested": args.speedup,
            "sim_speedup": next(
                (r.get("sim_speedup") for r in per_uav.values() if r.get("sim_speedup")),
                None,
            ),
            "clock_ratio_min": round(min(ratios), 4) if ratios else None,
            "clock_ratio_mean": round(statistics.fmean(ratios), 4) if ratios else None,
            "attitude_hz_mean": round(statistics.fmean(rates), 1) if rates else None,
            # A void level is not a pass. It is also not a capacity failure --
            # `void` is what tells the two apart downstream.
            "passed": not faults and not voids,
            "void": bool(voids),
            "faults": faults,
            "voids": voids,
            "notes": notes,
            "per_uav": {str(k): v for k, v in per_uav.items()},
        }
    finally:
        for child in children.values():
            if child.poll() is None:
                child.kill()
        if swarm is not None:
            swarm.terminate()
            try:
                swarm.wait(timeout=20)
            except subprocess.TimeoutExpired:
                swarm.kill()
        _cleanup_sysids(chat, sysids)
        # Released only AFTER the kill: holding the claim across teardown is what
        # stops another session launching into a chat this pkill is about to
        # sweep.
        _release_span(held)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python", type=Path, default=Path(sys.executable))
    parser.add_argument("--levels", default="4")
    parser.add_argument("--chat", type=int, default=None,
                        help="chat slot to borrow; defaults to the eval band floor")
    parser.add_argument("--hold-s", type=float, default=30.0,
                        help="aircraft seconds to hold the commanded attitude")
    parser.add_argument("--settle-s", type=float, default=20.0,
                        help="aircraft seconds to climb out under TAKEOFF")
    parser.add_argument("--connect-timeout", type=float, default=180.0)
    parser.add_argument("--child-timeout", type=float, default=900.0,
                        help="wall seconds before a child is killed as hung")
    parser.add_argument("--distance", type=int, default=100)
    parser.add_argument("--home", default=None)
    parser.add_argument("--airborne-m", type=float, default=15.0)
    # Off by default because it is one fewer process on the same cores, NOT
    # because it changes the channel: companion ports are per-vehicle either
    # way. See the module docstring.
    parser.add_argument("--router", dest="router", action="store_true", default=False,
                        help="run mavlink-routerd. It publishes the two "
                             "GCS-facing ports; companion ports are unaffected. "
                             "For comparison only.")
    parser.add_argument("--no-router", dest="router", action="store_false",
                        help="RUN_ROUTER=0 (default) -- no router process "
                             "competing for cores")
    parser.add_argument(
        "--pitches", default=None,
        help="per-aircraft commanded pitch in degrees, comma separated. MUST be "
             "passed in equals form when the first value is negative: "
             "--pitches=-5,-10,-15 works, --pitches -5,-10,-15 fails with "
             "'expected one argument' because argparse reads a comma-joined "
             "negative list as another option. Cycled if fewer values than "
             "aircraft, and cycled INDEPENDENTLY of --winds rather than crossed "
             "with it, so a grid must be pre-crossed (repeat each pitch once per "
             "wind and let the shorter wind list cycle inside it). Omitted, each "
             "child only watches telemetry and commands nothing.",
    )
    parser.add_argument(
        "--winds", default=None,
        help="per-aircraft wind as speed_mps:dir_deg, comma separated "
             "(e.g. 0:0,5:90). Cycled alongside --pitches, so the two lists pair "
             "off position by position -- they are NOT crossed.",
    )
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument(
        "--rolls", default=None,
        help="per-aircraft commanded roll in degrees, comma separated, cycled "
             "like --pitches and --winds and INDEPENDENTLY of them. Use the "
             "equals form for negative values (--rolls=-20,-10,0). Overrides "
             "--roll. Without this the whole sweep flies wings-level, which "
             "leaves the entire lateral channel -- where crosswind correction "
             "and turn authority live -- uncharacterised.",
    )
    parser.add_argument("--climb-to-m", type=float, default=400.0,
                        help="climb to this relative altitude before the hold. "
                             "TAKEOFF stops near 50 m, which is not enough air "
                             "to hold a dive long enough to measure it.")
    parser.add_argument("--floor-m", type=float, default=80.0,
                        help="end the hold at this relative altitude, so a steep "
                             "command cannot fly the aircraft into the ground "
                             "and report the ground's pitch as the pitch held.")
    parser.add_argument(
        "--speedups", default=None,
        help="sim speeds to sweep, comma separated (e.g. 1,2,5,10). Each one "
             "relaunches the whole swarm, because SIM_SPEEDUP is set at launch "
             "and shared by every aircraft in it -- so this multiplies wall "
             "time by the number of speeds. Overrides --speedup. Its purpose is "
             "to check the plant is speed-invariant: if held pitch or settling "
             "differs between 1x and 10x, every fast sweep is measuring a "
             "different aircraft than the slow one.",
    )
    parser.add_argument(
        "--speedup", type=float, default=10.0,
        help="simulator speed to REQUEST at launch. Previously hard-coded to "
             "1.0 while every artefact reported 10.0, because this harness runs "
             "without the supervisor that verifies and persists SIM_SPEEDUP and "
             "so inherited whatever the instance template's eeprom carried. The "
             "request is now explicit AND checked against what each aircraft "
             "reports, so a speedup sweep cannot silently measure one rate while "
             "claiming another.",
    )
    parser.add_argument(
        "--vm-delivery-min", type=float, default=0.9,
        help="fraction of its vCPUs' CPU-time the WSL VM must actually receive "
             "during the hold for a saturated reading to count as this level's "
             "ceiling. Guest utilisation is idle/total INSIDE the VM, so a "
             "host-starved VM reports near-100%% busy while getting almost no "
             "CPU; without this, foreign Windows load manufactures the very "
             "saturation the verdict treats as proof.",
    )
    parser.add_argument(
        "--cpu-sample-s", type=float, default=1.0,
        help="CPU sampling period. The capacity verdict now uses only samples "
             "overlapping the HOLD, and at 10x speedup a 30 aircraft-second "
             "hold is ~3 wall seconds, so the old 2s period could yield one "
             "sample or none. Shorter samples are what make the evidence "
             "attributable to the phase the clock was measured over.",
    )
    parser.add_argument(
        "--cpu-saturated-pct", type=float, default=85.0,
        help="utilisation at or above which a core counts as saturated. Used "
             "only to interpret a failing clock: below this the level was not "
             "CPU-bound, so its failure is not evidence of a capacity ceiling.",
    )
    parser.add_argument(
        "--idle-load", type=float, default=2.0,
        help="one-minute WSL load average the box must fall under before a "
             "level launches. Levels run back to back and teardown does not "
             "settle instantly; without the wait a ladder measures the load "
             "gradient it created rather than the machine.",
    )
    parser.add_argument(
        "--idle-windows-pct", type=float, default=25.0,
        help="Windows-side CPU the box must fall under before a level launches. "
             "WSL2 vCPUs are host threads, so another session's work is taken "
             "straight off the simulator -- and both other idle checks are "
             "taken inside WSL and cannot see it. Pass a large value to "
             "disable the check.",
    )
    parser.add_argument(
        "--idle-timeout", type=float, default=180.0,
        help="how long to wait for that idle state before running anyway and "
             "marking the level as measured on an unsettled box.",
    )
    parser.add_argument(
        "--foreign-aircraft-max", type=int, default=0,
        help="how many arduplane processes may already be running before a "
             "level's result is flagged as measured under foreign load. This "
             "box is shared, and a ceiling found while someone else's SITL is "
             "burning cores is their spare capacity, not this harness's limit.",
    )
    parser.add_argument("--pitch-tol-deg", type=float, default=3.0,
                        help="how far the held pitch may sit from the commanded "
                             "pitch before the run fails.")
    parser.add_argument("--throttle", type=float, default=0.55,
                        help="throttle for every aircraft. Use --throttles to "
                             "sweep it instead.")
    parser.add_argument(
        "--throttles", default=None,
        help="per-aircraft throttle 0-1, comma separated, cycled like "
             "--pitches and --winds and INDEPENDENTLY of them. Throttle is the "
             "input; airspeed is the OUTPUT and is measured, because under "
             "SET_ATTITUDE_TARGET the aircraft is given a throttle, never a "
             "speed. Overrides --throttle.",
    )
    return parser


def main() -> int:
    args = _parser().parse_args()
    if args.chat is None:
        args.chat = ip.eval_band()[0]
    levels = [int(x) for x in args.levels.split(",") if x.strip()]
    root = WORKTREE / ".sitl-runs" / f"sitl-scale-{time.strftime('%Y%m%d-%H%M%S')}"
    root.mkdir(parents=True)

    results: list[dict[str, object]] = []
    seen: dict[tuple[int, float], int] = {}
    # Sim speed is an OUTER loop, not a per-aircraft dimension: it is handed to
    # `wsl.launch_command` at launch, so every aircraft in one swarm shares it
    # and sweeping it means relaunching. Worth sweeping anyway -- if the same
    # commanded pitch settles differently at 1x and 10x, the fast runs are not
    # measuring the same aircraft, and every sweep taken at 10x is suspect.
    speedups = _pitches(args.speedups) or [args.speedup]
    for speedup in speedups:
        args.speedup = speedup
        for level in levels:
            key = (level, speedup)
            seen[key] = seen.get(key, 0) + 1
            _run_one(level, speedup, seen[key], root, args, results)
    return _report(results, root)


def _run_one(
    level: int, speedup: float, repeat: int, root: Path,
    args: argparse.Namespace, results: list[dict[str, object]],
) -> None:
    row = _run_level(level, root, args, repeat=repeat)
    results.append(row)
    (root / "summary.json").write_text(
        json.dumps(results, indent=2), encoding="utf-8"
    )
    print(
        f"  N={row['level']:>2} x{speedup:g} "
        f"airborne={row.get('airborne')}/{row['requested']}"
        f" wall={row.get('wall_s')}s ratio={row.get('clock_ratio_min')}"
        f" att_hz={row.get('attitude_hz_mean')}"
        f" {_verdict(row)}",
        flush=True,
    )
    cpu = row.get("cpu_during") or {}
    print(
        f"       cpu during: wsl {cpu.get('wsl_total_pct')}% avg"
        f" (busiest core {cpu.get('wsl_core_max_pct')}%,"
        f" quietest {cpu.get('wsl_core_min_pct')}%),"
        f" windows {cpu.get('windows_total_pct')}%"
        f" | peaks: wsl {cpu.get('wsl_total_peak_pct')}%"
        f" core {cpu.get('wsl_core_max_peak_pct')}%"
        f" windows {cpu.get('windows_total_peak_pct')}%"
        f" over {cpu.get('samples')} samples"
        f" | DURING THE HOLD (what the verdict uses):"
        f" wsl {row.get('hold_wsl_pct')}%"
        f" from {row.get('hold_cpu_samples')} sample(s),"
        f" vm got {row.get('hold_vm_delivery_frac')} of its vCPU-time",
        flush=True,
    )
    for note in row.get("notes") or []:
        print(f"       note: {note}", flush=True)
    for sysid, uav in sorted((row.get("per_uav") or {}).items()):
        print(
            f"       {sysid} pitch={uav.get('cmd_pitch_deg')}"
            f"->{uav.get('held_pitch_deg')}"
            f" err={uav.get('pitch_error_deg')}"
            f" wind={uav.get('wind_speed_mps')}@{uav.get('wind_dir_deg')}"
            f" alt={uav.get('max_rel_alt_m')}->{uav.get('end_rel_alt_m')}"
            f" hz={uav.get('attitude_hz')} ratio={uav.get('clock_ratio')}"
            f" hold_end={uav.get('hold_end')}",
            flush=True,
        )


def _report(results: list[dict[str, object]], root: Path) -> int:
    print(f"\n{'N':>3} {'air':>4} {'wall_s':>7} {'speedup':>8} {'ratio':>8}"
          f" {'att_hz':>7}  verdict")
    for row in results:
        print(
            f"{row['requested']:>3} {row.get('airborne', 0):>4}"
            f" {str(row.get('wall_s')):>7} {str(row.get('sim_speedup')):>8}"
            f" {str(row.get('clock_ratio_min')):>8}"
            f" {str(row.get('attitude_hz_mean')):>7}"
            f"  {_verdict(row)}"
        )
    voided = [row for row in results if row.get("void")]
    if voided:
        print(
            f"\n{len(voided)} level(s) VOID -- the box was not ours during them, "
            "so they are neither a pass nor a ceiling. Re-run those on a quiet "
            "machine before reading any capacity number off this table."
        )
    print(f"\nartefacts: {root}")
    # A void level must not report success, and must not be counted as a
    # capacity failure either; the caller gets a distinct code for "no answer".
    if any(row.get("void") for row in results):
        return 2
    return 0 if all(row.get("passed") for row in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
