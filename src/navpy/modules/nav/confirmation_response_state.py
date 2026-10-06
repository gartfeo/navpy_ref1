"""Confirmation response correlation and retained recall ownership."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from navpy.modules.nav.confirmation_round_transaction import (
    ConfirmationRequestRef,
    ConfirmationResponseKind,
    ConfirmationRound,
)
from navpy.modules.nav.confirmation_registry_state import ConfirmationRegistryState, ConfirmationStatus


@dataclass(frozen=True)
class ResolvedConfirmation:
    request_ref: Optional[ConfirmationRequestRef]
    accepts_legacy_response: bool


def matches_round(
    confirmation: ConfirmationRound,
    response_ref: Optional[ConfirmationRequestRef],
) -> bool:
    if response_ref is None:
        return confirmation.accepts_legacy_response
    return confirmation.request_ref == response_ref


def response_for_current_round(
    registry: ConfirmationRegistryState,
    target_id: int,
    is_confirmed: bool,
) -> ConfirmationResponseKind:
    current = registry.status_by_id(target_id)
    if current is ConfirmationStatus.CONFIRMING:
        registry.set_status(
            target_id,
            ConfirmationStatus.CONFIRMED if is_confirmed else ConfirmationStatus.REJECTED,
        )
        return (
            ConfirmationResponseKind.CONFIRMED
            if is_confirmed
            else ConfirmationResponseKind.REJECTED
        )
    if current is ConfirmationStatus.CONFIRMED and not is_confirmed:
        registry.set_status(target_id, ConfirmationStatus.REJECTED)
        return ConfirmationResponseKind.CANCELLATION_REQUESTED
    if current is ConfirmationStatus.REJECTED and is_confirmed:
        return ConfirmationResponseKind.ALREADY_REJECTED
    return ConfirmationResponseKind.WAKE_ONLY


class ConfirmationResponseLedger:
    """Retain exact response tokens only as long as recall can use them."""

    def __init__(self, registry: ConfirmationRegistryState) -> None:
        self._registry = registry
        self._resolved: dict[int, ResolvedConfirmation] = {}
        self._seen_target_ids: set[int] = set()

    def legacy_allowed(self, target_id: int) -> bool:
        return target_id not in self._seen_target_ids

    def start_review(self, target_id: int) -> None:
        """Invalidate prior recall authority without reopening legacy input."""
        self._resolved.pop(target_id, None)

    def install(self, target_id: int) -> None:
        self._resolved.pop(target_id, None)
        self._seen_target_ids.add(target_id)

    def remember_completion(
        self,
        confirmation: ConfirmationRound,
        status: ConfirmationStatus,
    ) -> None:
        if status is ConfirmationStatus.CONFIRMED:
            self._remember(confirmation)
        else:
            self._resolved.pop(confirmation.target_id, None)

    def record_result(
        self,
        confirmation: ConfirmationRound,
        result: ConfirmationResponseKind,
    ) -> None:
        if result is ConfirmationResponseKind.CONFIRMED:
            self._remember(confirmation)
        elif result is not ConfirmationResponseKind.WAKE_ONLY:
            self._resolved.pop(confirmation.target_id, None)

    def resolve_without_round(
        self,
        target_id: int,
        is_confirmed: bool,
        response_ref: Optional[ConfirmationRequestRef],
    ) -> ConfirmationResponseKind:
        resolved = self._resolved.get(target_id)
        if (
            self._registry.status_by_id(target_id) is ConfirmationStatus.CONFIRMED
            and not is_confirmed
            and resolved is not None
            and self._matches(resolved, response_ref)
        ):
            self._registry.set_status(target_id, ConfirmationStatus.REJECTED)
            self._resolved.pop(target_id, None)
            return ConfirmationResponseKind.CANCELLATION_REQUESTED
        return ConfirmationResponseKind.LATE_OR_DUPLICATE

    def clear(self) -> None:
        self._resolved.clear()

    def _remember(self, confirmation: ConfirmationRound) -> None:
        self._resolved[confirmation.target_id] = ResolvedConfirmation(
            confirmation.request_ref,
            confirmation.accepts_legacy_response,
        )

    @staticmethod
    def _matches(
        resolved: ResolvedConfirmation,
        response_ref: Optional[ConfirmationRequestRef],
    ) -> bool:
        if response_ref is None:
            return resolved.accepts_legacy_response
        return resolved.request_ref == response_ref


__all__ = [
    "ConfirmationResponseLedger",
    "matches_round",
    "response_for_current_round",
]
