"""Admission rules for mission-upload protocol replies."""

from __future__ import annotations

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_MISSION_ACCEPTED,
    MAV_MISSION_TYPE_MISSION,
    MAVLink_message,
)


class MissionUploadMatcher:
    def __init__(self, target_system: int) -> None:
        self._target = target_system

    def is_request(
        self,
        message: MAVLink_message,
        sequence: int | None = None,
        mission_type: int = MAV_MISSION_TYPE_MISSION,
    ) -> bool:
        if message.get_type() not in (
            "MISSION_REQUEST_INT",
            "MISSION_REQUEST",
        ):
            return False
        return (
            self._from_target_table(message, mission_type)
            and (
                sequence is None
                or getattr(message, "seq", None) == sequence
            )
        )

    def is_accepted_ack(
        self,
        message: MAVLink_message,
        mission_type: int,
    ) -> bool:
        return (
            message.get_type() == "MISSION_ACK"
            and self._from_target_table(message, mission_type)
            and getattr(message, "type", None) == MAV_MISSION_ACCEPTED
        )

    def _from_target_table(
        self,
        message: MAVLink_message,
        mission_type: int,
    ) -> bool:
        try:
            if message.get_srcSystem() != self._target:
                return False
        except Exception:
            return False
        return (
            getattr(
                message,
                "mission_type",
                MAV_MISSION_TYPE_MISSION,
            )
            == mission_type
        )


__all__ = ["MissionUploadMatcher"]
