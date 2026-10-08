# Swarm messaging security backlog

Status: **deferred**. During the development phase, swarm task messages are
not authenticated. The physical link is expected to provide encryption, which
makes the scenarios below less likely. They are recorded here so they can be
addressed when authentication work starts.

Source: security review of PR #4 (assign-request resend and release,
2026-10-07); updated 2026-10-08 for the assignment acks
([swarm-task-assignment-ack.md](swarm-task-assignment-ack.md)).

## Background

Swarm task messages (AVAILABLE_TASK_REQUEST/RESPONSE, TASK_ASSIGN_REQUEST/
RESPONSE, SWARM_ACK) and the SWARM_HEARTBEAT node state identify the sender
only by the MAVLink `srcSystem`. Nothing authenticates that ID. Boot ids,
task ids and sequence numbers are sent in plaintext.

## Known risks

1. **Spoofed release of a waiting vehicle.** A WAITING helper drops its task
   when its owner advertises it again, later (`SelectedTaskSlot.on_advert`).
   A forged advert with the owner's ID and boot and a newer `msg_seq` drops
   it; if the owner already CONFIRMED that helper, the task is stranded. An
   ASSIGNED (flying) task is never dropped by adverts.
2. **Spoofed assign request into an idle slot.** This predates PR #4. An idle
   peer turns WAITING on any assign request from an admitted roster ID, and
   flies once an APPLIED names one of its step-4 copies (see 5).
3. **High-`msg_seq` pin.** A forged copy of the WAITING request with a very
   high `msg_seq` raises the held request (`SelectedTaskSlot.on_request`).
   Genuine release adverts are then ignored until the 18 s WAITING expiry.
4. **Forged best bid.** A forged low-ETA bid, with forged heartbeats so an
   offline roster peer does not fall silent, wins the auction. The owner
   releases the task 9 s after its first assign request, but this can repeat
   indefinitely.
5. **Forged acks and node state.** A forged APPLIED naming a helper's step-4
   copy (its UID is plaintext) makes a WAITING helper fly although its owner
   never confirmed it, so two UAVs can fly one task; a forged RECEIVED drops
   a WAITING task. A forged BUSY heartbeat keeps a free peer out of auctions;
   a forged FREE one makes a busy peer look free.

## To do (when authentication is in scope)

- [ ] Decide the trust model. Either rely on the encrypted physical link
      alone, or add authentication at the message level. Record the
      decision here.
- [ ] If message level: enable MAVLink2 message signing on swarm links, or
      add an HMAC over task messages, acks and heartbeats, with key
      provisioning per fleet.
- [ ] Bound how far a repeat may raise the held request (a seq window
      around it) to limit the high-seq pin.
- [ ] Replace the inferred release (re-advertisement) with an explicit
      owner→peer cancel message tied to the reservation.
- [ ] Add a final give-up for repeated releases to the same peer: an
      assignment-failed outcome, an operator alert, or excluding that peer
      for the task.
- [ ] Log revocations so operators can see unexpected un-assignments.
- [ ] Add security regression tests for the scenarios above.
