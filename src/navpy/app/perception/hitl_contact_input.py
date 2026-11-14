"""``--hitl-contact``: the HITL runner's contact link for one navpy run (hitl_guidance.md step 26).

Only a pc-sitl run gets a contact sensor, and only through three gates, each refusing at startup:

- the run record (``--integration-timing``) is present, so the detector is real and the mode is the record's;
- the vision profile sets ``hitl_contact: true`` (only ``siyi_zr10_hitl_encoded``; no flight profile has it);
- the link file (``tools/hitl/contact_service.py link``) is that run's: same run id, same mode, a mode in
  ``HITL_CONTACT_MODES``.

Without ``--hitl-contact`` nothing changes: a real build wires no contact sensor.
"""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace
from typing import Any, Optional

from navpy.app.attitude_boot_time import fresh_attitude_boot_s
from navpy.app.perception.integration_timing_input import IntegrationTimingInput
from navpy.core.perception.camera.integration_timing import IntegrationTimingConfig
from navpy.core.ports.dock_contact import DockContactCallback, DockContactIdentity, DockEpisodeEnd
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.plugins.contact.hitl_contact_link import UdpContactLink, load_link
from navpy.plugins.contact.hitl_contact_sensor import CONTACT_MAX_AGE_S, BoardClock, HitlDockContactSensor

HITL_CONTACT_PROFILE_KEY = "hitl_contact"
# Stage 1 only; flight and Qualification never get a software contact (a real aircraft is caught by the net).
HITL_CONTACT_MODES = frozenset({"pc-sitl"})


class HitlContact:
    """The adapter navpy's lifecycle sees: the sensor's three calls, and ``close`` for the link's thread."""

    def __init__(self, sensor: HitlDockContactSensor, link: UdpContactLink) -> None:
        self.sensor = sensor
        self.link = link

    def arm(self, identity: DockContactIdentity, on_docked: DockContactCallback) -> None:
        self.sensor.arm(identity, on_docked)

    def disarm(self, end: DockEpisodeEnd = DockEpisodeEnd()) -> None:
        self.sensor.disarm(end)

    def end_evidence(self, identity: DockContactIdentity) -> None:
        self.sensor.end_evidence(identity)

    def close(self) -> bool:
        """True once the link's thread has ended; False while its receiver is still busy (a later close joins
        again). Never raises into vision's stop."""
        return self.link.close()


def profile_allows_contact(profile: Mapping[str, Any]) -> bool:
    """Only an explicit ``true``; a malformed value is refused, never read as on."""
    value = profile.get(HITL_CONTACT_PROFILE_KEY, False)
    if not isinstance(value, bool):
        raise ValueError(f"{HITL_CONTACT_PROFILE_KEY} must be true or false, got {value!r}")
    return value


def open_hitl_contact(link_path: str, timing: Optional[IntegrationTimingConfig], profile: Mapping[str, Any],
                      board_time_s: BoardClock) -> Optional[HitlContact]:
    if not link_path:
        return None
    if timing is None:
        raise ValueError("--hitl-contact needs --integration-timing and --detector-type=real (a pc-sitl run)")
    if not profile_allows_contact(profile):
        raise ValueError(f"--hitl-contact needs a vision profile with {HITL_CONTACT_PROFILE_KEY}: true")
    config = load_link(link_path)
    if config.run_id != timing.run_id or config.mode != timing.mode:
        raise ValueError(f"{link_path}: the contact link is another run's")
    if config.mode not in HITL_CONTACT_MODES:
        raise ValueError(f"{link_path}: mode {config.mode!r} may not use a contact service")
    link: list[UdpContactLink] = []
    sensor = HitlDockContactSensor(config.run_id, config.key, lambda datagram: link[0].send(datagram), board_time_s)
    link.append(UdpContactLink(config, sensor.receive))
    return HitlContact(sensor, link[0])


def with_hitl_contact(integration: Optional[IntegrationTimingInput], link_path: str, profile: Mapping[str, Any],
                      vehicle: IVehicle) -> Optional[IntegrationTimingInput]:
    """The run's contact on its integration input. The board clock is navpy's own ATTITUDE boot time, and only
    while that ATTITUDE is fresh: a frozen telemetry stream gives no board time, so no contact is accepted."""
    contact = open_hitl_contact(link_path, None if integration is None else integration.config, profile,
                                lambda: fresh_attitude_boot_s(vehicle, CONTACT_MAX_AGE_S))
    if contact is None or integration is None:
        return integration
    return replace(integration, contact=contact)


__all__ = ["HITL_CONTACT_MODES", "HITL_CONTACT_PROFILE_KEY", "HitlContact", "open_hitl_contact",
           "profile_allows_contact", "with_hitl_contact"]
