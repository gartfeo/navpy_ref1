# Swarm messaging security backlog

Status: **deferred**. During the development phase, swarm task messages are
not authenticated. The physical link is expected to provide encryption, which
makes the scenarios below less likely. They are recorded here so they can be
addressed when authentication work starts.

Source: security review of PR #4 (assign-request resend and release,
2026-10-07).

## Background

Swarm task messages (AVAILABLE_TASK_REQUEST/RESPONSE, TASK_ASSIGN_REQUEST/
RESPONSE) identify the sender only by the MAVLink `srcSystem`. Nothing
authenticates that ID. The owner's `boot_id`, task ids and sequence numbers
are sent in plaintext.

## Known risks

1. **Spoofed release of a busy vehicle.** A peer drops its held task when the
   owner advertises it again (`task_capability.py`,
   `_release_if_re_advertised`). A forged AVAILABLE message carrying the
   owner's ID and a newer `msg_seq` makes the peer drop the task, while the
   owner still treats it as CONFIRMED. A forged assign request can then
   re-task the vehicle.
2. **Spoofed assign request into an idle slot.** This predates PR #4. An idle
   peer accepts any assign request from an admitted roster ID.
3. **High-`msg_seq` pin.** A forged request for the held task with a very
   high `msg_seq` raises the peer's accepted order
   (`SelectedTaskSlot.try_accept`). Every genuine release that follows is
   then ignored, and the peer and task stay pinned.
4. **Forged best bid.** A forged low-ETA bid from an offline roster peer wins
   the auction. The owner releases the task after its confirmation timeout
   (about 7 s), but this can repeat indefinitely.

## To do (when authentication is in scope)

- [ ] Decide the trust model. Either rely on the encrypted physical link
      alone, or add authentication at the message level. Record the
      decision here.
- [ ] If message level: enable MAVLink2 message signing on swarm links, or
      add an HMAC over task messages, with key provisioning per fleet.
- [ ] Bound how far a re-accept may raise the accepted order (a seq window
      around the stored order) to limit the high-seq pin.
- [ ] Replace the inferred release (re-advertisement) with an explicit
      owner→peer cancel message tied to the reservation.
- [ ] Add a final give-up for repeated releases to the same peer: an
      assignment-failed outcome, an operator alert, or excluding that peer
      for the task.
- [ ] Log revocations so operators can see unexpected un-assignments.
- [ ] Add security regression tests for the scenarios above.
