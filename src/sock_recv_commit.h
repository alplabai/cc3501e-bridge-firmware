/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: SOCK_RECV ring's lazy-commit/replay arithmetic.
 *
 * ===================== WHY THIS EXISTS =====================
 * hal/ti/cc3501e_hw_ti_sock.c's SOCK_RECV prefetch ring committed its tail
 * EAGERLY -- the instant it served bytes into a reply, before that reply had
 * even been clocked to the host.  If the host's CRC rejected the reply
 * (ALP_ERR_IO) and poll_by_repeat() re-sent the IDENTICAL frame (same 5-bit
 * header seq), nothing let the firmware tell "the host never saw the last
 * reply, re-serve it" from "this is a genuinely new recv, serve the next
 * chunk": alp_cc3501e_sock_recv_t carries no seq of its own (v9 protocol),
 * and the #88/#107 SOCK_SEND reply cache (protocol_sockets.c) is send-only.
 * A silently-lost reply therefore silently lost that reply's BYTES -- the
 * ring had already moved past them.  Bench-measured, e1m-aen-evk-01 run10: a
 * 262144 B HTTP body over 4071 B cc3501e_sock_recv() calls came back 258075
 * B, missing exactly one 4069 B block, sitting exactly on a reply boundary.
 *
 * FIX: lazy-commit.  A served chunk's bytes are NOT retired from the ring
 * until the NEXT call proves the host actually moved on.  protocol_sockets.c's
 * handle_sock_recv() decides that from the request's own 5-bit header seq
 * PLUS the handle (alp_cc3501e_sock_recv_t itself carries neither) and passes
 * the verdict in as @p replay.  A same-seq-same-handle re-issue re-serves the
 * SAME starting position, so it can only return the same bytes or MORE (if
 * the ring gained data meanwhile, since the PRODUCER's head keeps advancing
 * independently of the held-back tail) -- correct stream-prefix semantics,
 * never fewer.
 *
 * RESIDUAL 1 -- seq aliasing (NOT fixed here).  protocol_sockets.c's
 * handle_sock_recv() identifies a replay by the generic per-dispatch 5-bit
 * header seq (protocol.c's s_current_req_seq) plus handle, recorded on
 * EVERY SOCK_RECV dispatch for that handle.  That seq is not SOCK_RECV's
 * own counter -- it is shared by every opcode the host issues through
 * cc3501e_core.c's poll_by_repeat().  Two consecutive ring recvs on the
 * SAME handle therefore collide (the second is wrongly read as a replay of
 * the first, duplicating a block, still reported OK) when exactly 30 mod 31
 * OTHER seq-allocating poll_by_repeat() calls -- ANY opcode, not just
 * SOCK_RECV -- separate them: the 5-bit field has 31 non-zero values
 * (ALP_CC3501E_REQ_SEQ_NONE = 0 never claims a replay), so the counter
 * returns to the same value once every 31 allocations on that shared
 * counter.  The fix is a dedicated SOCK_RECV-only seq counter on the host
 * side, which removes this by construction (nothing else could ever
 * advance it between two ring recvs); this firmware-side change cannot
 * remove it alone, since the firmware only ever sees the seq the host chose
 * to send.
 *
 * RESIDUAL 2 -- a dropped reply with no retry (NOT fixed here).  Lazy-
 * commit retires the previous call's bytes when a DIFFERENT request
 * arrives, not when the previous reply is actually known to have reached
 * the host.  Those are usually the same event (a lost reply provokes
 * poll_by_repeat()'s same-seq retry, which this fix catches), but not
 * always: if the host drops a lost reply WITHOUT issuing that same-seq
 * retry -- poll_by_repeat()'s own deadline expiring immediately after the
 * CRC-failed attempt, or a corrupted status byte decoding as one of the
 * terminal codes (0x06/0x07/0xFF) instead of the CRC error it should have
 * been -- the next call this handle makes is a genuinely new (non-replay)
 * recv, which commits the still-unacknowledged block and loses it exactly
 * as before this fix.  The host side will need a same-seq grace re-poll for
 * recv (retry once more before giving up, rather than surfacing the error
 * immediately) to close this; this firmware change cannot close it alone,
 * since by the time a non-replay call arrives there is no way left to tell
 * "host never saw the last reply" from "host saw it and moved on".
 *
 * PURE ARITHMETIC, SILICON-FREE.  This file owns ONLY the tail/uncommitted
 * bookkeeping -- head/tail as plain integers, no ring buffer, no memcpy, no
 * lwIP -- so it links and runs identically on the host.
 * hal/ti/cc3501e_hw_ti_sock.c's cc3501e_hw_sock_recv_ring() is the sole
 * caller: it holds the real 64 KB ring (TCM-placed, TI-SDK-only, not
 * buildable on the host) and does the actual byte copy at the offset/length
 * this function returns.  Splitting the DECISION out this way -- the part a
 * CRC-rejected-reply bug actually lives in -- is what makes it unit-testable
 * on the host stub build, where the ring itself cannot be linked at all
 * (CC3501E_HAL_BACKEND=ti only, needs the TI SimpleLink SDK).
 * ============================================================
 */

#ifndef CC3501E_BRIDGE_SOCK_RECV_COMMIT_H
#define CC3501E_BRIDGE_SOCK_RECV_COMMIT_H

#include <stdbool.h>
#include <stdint.h>

/*
 * sock_recv_commit -- decide how many bytes THIS call serves, and whether to
 * retire the PREVIOUS call's bytes from the ring first.
 *
 *   @p tail        [in/out] the ring's committed read position.  On entry,
 *                  still the position as of the LAST commit (i.e. NOT yet
 *                  advanced for the previous call's serve).  Advanced by
 *                  *uncommitted (the previous call's byte count) UNLESS
 *                  @p replay is true, then left at the offset this call
 *                  should serve from.
 *   @p uncommitted [in/out] the byte count the PREVIOUS call served but that
 *                  has not yet been folded into *tail.  Reset to 0 then to
 *                  THIS call's own served count before returning.
 *   @p head        the ring's write position (bytes produced so far);
 *                  head - *tail (post-commit) is what is available to serve.
 *   @p replay      true when THIS call is a byte-identical re-issue of the
 *                  IMMEDIATELY PRECEDING call (protocol_sockets.c: same
 *                  request header seq AND same handle) -- i.e. the host
 *                  never collected the previous reply and is asking again,
 *                  not moving on to new bytes.
 *   @p cap         destination buffer capacity for this call.
 *
 * Returns the byte count in [0, cap] to serve, starting at the (possibly
 * just-committed) *tail on return.  The caller does the actual copy out of
 * its ring buffer at that offset/length -- this function touches no buffer,
 * only the two cursors, and is safe to call with head/tail read from a
 * volatile ring as long as the CALLER does not publish *tail back to that
 * ring until AFTER it has finished copying out the bytes this call reports
 * (see hal/ti/cc3501e_hw_ti_sock.c's own ordering comment on why: publishing
 * the tail advance before the copy completes would let the producer's
 * headroom check treat the not-yet-copied bytes as free and overwrite them).
 *
 * A replay leaves *tail UNCHANGED from before the call it replays (the
 * commit is skipped), so it re-serves the SAME starting byte that call did;
 * it can return MORE bytes than that call did (more data may have arrived in
 * the ring meanwhile) but never fewer, since head only ever grows and *tail
 * is identical to what it was for the call being replayed. */
uint32_t
sock_recv_commit(uint32_t *tail, uint32_t *uncommitted, uint32_t head, bool replay, uint32_t cap);

/*
 * sock_recv_commit_reset -- the "forget any served-but-not-retired count"
 * half of arming (or disarming) the prefetch ring for a handle
 * (hal/ti/cc3501e_hw_ti_sock.c's cc3501e_hw_sock_prefetch(), both branches).
 * A fresh arm must never inherit a stale count from whatever handle used
 * the ring last, or that handle's very first serve would wrongly fold
 * someone else's leftover bytes into its tail.
 *
 * Split out (rather than the caller just writing `uncommitted = 0u;`
 * inline, as an earlier version of this fix did) so the host test suite
 * can exercise the SAME reset production code runs, instead of a
 * hand-simulated stand-in for it -- rx_ring.tail/head themselves cannot
 * follow this same seam (they are `volatile`, ring-buffer-owned fields;
 * this function only ever owns the plain uint32_t `uncommitted`). */
void sock_recv_commit_reset(uint32_t *uncommitted);

#endif /* CC3501E_BRIDGE_SOCK_RECV_COMMIT_H */
