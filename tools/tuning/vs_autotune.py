"""The vs-autotune comparison after the takeoff (sitl_driver vs-autotune sets up the vehicle and takes off):
ArduPlane AUTOTUNE, then the flight tuner starting from AUTOTUNE's gains, then the same FBWA card flown
A-B-A-B with both gain sets, scored afterwards by card_eval.py (--marker FTUN_REPS).

Round 2 (HITL chain, Peron15_hitl2) showed what made the comparison meaningless, and what this does instead:
- AUTOTUNE flew on the pilot's 65 % throttle above AIRSPEED_MAX and lost tens of metres in roll doublets
  until a height guard ended it before pitch. FBWA, AUTOTUNE and the cards now fly the throttle LOITER holds at
  AIRSPEED_CRUISE in this run (flight_steps.pilot_throttle). AUTOTUNE is left for a LOITER climb back to
  TKOFF_ALT whenever it comes within AUTOTUNE_RECOVERY_M of the run's floor, then entered again.
- LOITER holds the height it is entered at (commands_logic.cpp:979), so LOITER between steps never climbed:
  the tuner needs FTUN_ALT_MIN + 10 m and never started, and card 13 hit the ground. The climbs are now
  commanded (flight_steps.climb_to): after AUTOTUNE and before every card.
- The run exited 0. It now fails below the floor (flight_floor.HeightFloor, set up by sitl_driver) and when the
  tuner does not finish, since the cards would then fly AUTOTUNE's gains twice.
- FBWA's stick centre flew its own level attitude, not the cruise's, so FBWA and AUTOTUNE flew faster than the
  cruise at the cruise throttle (port-check: several m/s too fast). Everything after the cruise reading now flies
  with port-check's cruise trim (cruise_trim.CruisePitchTrim: PTCH_TRIM_DEG for this run only, set back at the
  end), so AUTOTUNE, the tuner and the cards of both gain sets fly the same trim.
AUTOTUNE_AXES and FTUN_AXES both come from --axes (the same bits: Plane.h:1307-1311, AP_FlightTune.cpp:113-117),
so both tune the same axes, and the cards fly those axes.
"""
from __future__ import annotations

from tools.tuning.cruise_trim import CruisePitchTrim
from tools.tuning.flight_steps import (FTUNE_SWITCH_CH, PILOT_THROTTLE_PWM, THROTTLE_CH, TUNER_END_TEXTS,
                                       FlightCheckFailed, climb_to, pilot_throttle, read_gains, stick_sequence,
                                       wait_text)
from tools.tuning.vehicle import Vehicle

CARD_MARKER = "FTUN_REPS"     # 11..14 while each card flies so the log marks it; its own value between cards
CARD_ROLL = [(1800, 3.0), (1500, 3.0), (1200, 3.0), (1500, 3.0)]
CARD_PITCH = [(1650, 3.0), (1500, 3.0), (1350, 3.0), (1500, 3.0)]
CARD_STICKS = ((1, 1, CARD_ROLL), (2, 2, CARD_PITCH))      # axis bit, stick channel, card
RATE_GAINS = [f"{a}_RATE_{k}" for a in ("RLL", "PTCH") for k in ("FF", "P", "I", "D")]
# AP_AutoTune's axis name in its messages, the axis bit, the stick channel and the full-stick doublet
AUTOTUNE_DOUBLETS = (("Roll", 1, 1, 2000, 1000), ("Pitch", 2, 2, 1900, 1100))
DOUBLET_S = 1.5
# AUTOTUNE is left for a climb this far above the run's floor. The height is read after each doublet pair
# (3 s, a few metres at the sink AUTOTUNE showed in round 2), and LOITER then pulls up from wherever the
# last doublet left the aircraft.
AUTOTUNE_RECOVERY_M = 15.0
TUNER_SETTLE_S = 20.0         # LOITER before the tuning switch goes high
TUNER_DONE = "FTUN: done"


def run_autotune(sitl: Vehicle, seconds: float, axes: int, pause_m: float, climb_m: float) -> dict[str, dict]:
    """ArduPlane AUTOTUNE with full-stick doublets on each axis in `axes`, until the axis reports "Finished"
    (AP_AutoTune.cpp:414) or `seconds` of its doublets have flown, on the pilot's throttle as set.

    AUTOTUNE flies like FBWA (mode_autotune.cpp:12-23) and sinks in roll doublets. Below pause_m it is left
    for a LOITER climb to climb_m and entered again. Leaving (mode.cpp:19-22) keeps the gains once AUTOTUNE
    has found both its P and D limits and otherwise gives back those of that entry (AP_AutoTune.cpp:142-152).
    Returns per axis: finished, seconds of doublets, climbs."""
    flown_axes: dict[str, dict] = {}
    sitl.set_mode("AUTOTUNE")
    for name, bit, channel, high, low in AUTOTUNE_DOUBLETS:
        if not axes & bit:
            continue
        seen, flown, climbs, finished = len(sitl.messages), 0.0, 0, False
        while flown < seconds and not finished:
            height = sitl.state()["alt"]
            if height is not None and height < pause_m:
                print(f"[driver] AUTOTUNE left at {height:.0f} m to climb to {climb_m:g} m", flush=True)
                climb_to(sitl, climb_m)
                climbs += 1
                sitl.set_mode("AUTOTUNE")
            stick_sequence(sitl, channel, [(high, DOUBLET_S), (low, DOUBLET_S)])
            flown += 2 * DOUBLET_S
            finished = any(f"{name}: Finished" in m for m in sitl.messages[seen:])
        flown_axes[name] = {"finished": finished, "seconds": flown, "climbs": climbs}
    sitl.set_mode("FBWA")
    sitl.fly_for(5)
    return flown_axes


def fly_card(sitl: Vehicle, marker: int, axes: int) -> None:
    sitl.set_param(CARD_MARKER, marker)
    sitl.set_mode("FBWA")
    sitl.fly_for(3)
    for bit, channel, card in CARD_STICKS:
        if axes & bit:
            stick_sequence(sitl, channel, card)


def fly_comparison(sitl: Vehicle, result: dict, height_m: float, axes: int, autotune_s: float,
                   timeout_s: float, min_height_m: float) -> None:
    """Everything after the takeoff to height_m (TKOFF_ALT); fills result. pilot_throttle reads the cruise and
    leaves the aircraft in FBWA, where CruisePitchTrim trims its stick centre to the cruise pitch for the rest of
    the run."""
    pilot = pilot_throttle(sitl)
    result["pilot_throttle"] = pilot
    with CruisePitchTrim(sitl, pilot, result):
        _fly_trimmed(sitl, result, pilot, height_m, axes, autotune_s, timeout_s, min_height_m)


def _fly_trimmed(sitl: Vehicle, result: dict, pilot: dict, height_m: float, axes: int, autotune_s: float,
                 timeout_s: float, min_height_m: float) -> None:
    """AUTOTUNE, the tuner and the A-B-A-B cards, all at the run's cruise trim."""
    result["initial"] = read_gains(sitl)
    flown = run_autotune(sitl, autotune_s, axes, min_height_m + AUTOTUNE_RECOVERY_M, height_m)
    result["autotune_finished"] = {name: axis["finished"] for name, axis in flown.items()}
    result["autotune_flight"] = flown
    result["autotune"] = read_gains(sitl)
    climb_to(sitl, height_m)
    sitl.tx.set(THROTTLE_CH, PILOT_THROTTLE_PWM)       # the tuner's LOITER as in inflight-tune
    sitl.fly_for(TUNER_SETTLE_S)
    sitl.tx.set(FTUNE_SWITCH_CH, 2000)
    result["ftune_end"] = end = wait_text(sitl, TUNER_END_TEXTS, timeout_s)
    sitl.tx.set(FTUNE_SWITCH_CH, 1000)
    sitl.fly_for(3)
    result["flight_tune"] = read_gains(sitl)
    result["gain_sets_differ"] = any(result["autotune"][n] != result["flight_tune"][n] for n in RATE_GAINS)
    if end is None or TUNER_DONE not in end:
        raise FlightCheckFailed(f"the tuner did not finish ({end or 'no end message'}): the cards would fly "
                                "AUTOTUNE's gains twice")
    idle = sitl.get_param(CARD_MARKER)
    sets = {"A": result["autotune"], "B": result["flight_tune"]}
    for i, label in enumerate("ABAB"):
        for name in RATE_GAINS:
            sitl.set_param(name, sets[label][name])
        climb_to(sitl, height_m)
        sitl.tx.set(THROTTLE_CH, pilot["stick_pwm"])
        fly_card(sitl, 11 + i, axes)
        sitl.set_param(CARD_MARKER, idle)
