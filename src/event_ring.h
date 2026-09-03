/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge firmware: async-event ring buffer (the EVT_* queue).
 *
 * ===================== WHY THIS EXISTS (task #17) =====================
 * The firmware learns of asynchronous events (Wi-Fi connect/disconnect, BLE
 * connection, a GPIO edge) on its radio/host-driver threads, but the CC3501E is
 * the SPI slave and can never initiate a transfer, so it cannot PUSH an event
 * frame to the host.  The host POLLS instead: it issues
 * CMD_GET_PENDING_EVENTS, and handle_get_pending_events() DRAINS this ring
 * into the reply.
 *
 * There IS a slave->master ATTENTION path, and it is multiplexed onto the READY
 * wire (CC35 GPIO17 -> Alif P2_6, the rev-1 bodge line that the stock EVK does
 * not route): event_ring_push() below calls cc3501e_bridge_attn_pulse(), and a
 * host that armed the edge drains at once instead of waiting for its timer
 * (#130, alp-sdk#1721).  It does not replace the poll -- the firmware half is a
 * BUILD-TIME OPT-IN (CC3501E_ATTN_PULSE, default OFF; without it the pulse links
 * to a no-op stub and the host falls back to its timer).  A dedicated HOST_IRQ
 * pad is still a future board rev.  See DESIGN.md for both limits.
 *
 * The ring decouples the event PRODUCERS (thread context: the Wi-Fi event
 * callback / connect body, BLE, GPIO) from the CONSUMER (the SPI-ISR dispatch
 * that runs handle_get_pending_events).  Both push and drain are IRQ-safe --
 * short critical sections (the same worker_critical_enter/exit the worker uses,
 * a no-op on native / __disable_irq on the ti backend), so the ISR can drain
 * inline while a producer thread pushes.  The drain is a bounded memcpy, safe
 * to run in the ISR (unlike the seconds-long radio ops, which the worker
 * routes off-ISR).
 *
 * SILICON-FREE: this TU pulls in NO TI SDK; it is compiled into every backend
 * (stub + ti) and the native_sim transport test.
 * =====================================================================
 */

#ifndef CC3501E_BRIDGE_EVENT_RING_H
#define CC3501E_BRIDGE_EVENT_RING_H

#include <stddef.h>
#include <stdint.h>

/* The canonical wire contract (single-sourced from alp-sdk, see CMakeLists):
 * ALP_CC3501E_EVENT_PAYLOAD_MAX is the per-entry payload cap this ring
 * enforces. */
#include "alp/protocol/cc3501e.h"

/* Ring depth (number of queued events) and the per-entry payload cap.  16 slots
 * absorb a burst of connect/disconnect/GPIO events between two host polls; 16
 * payload bytes cover the largest EVT_* payload the firmware enqueues (the
 * 8-byte alp_cc3501e_gpio_event_t, with headroom).  Overflow drops the NEWEST
 * event (the ring never blocks a producer). */
#define CC3501E_EVENT_RING_SLOTS 16u
/* The per-entry payload cap is part of the WIRE CONTRACT, not a private
 * firmware size: a host that defines an EVT_* payload larger than this gets it
 * CLAMPED by event_ring_push() below and silently truncated.  So it is defined
 * once, in the canonical <alp/protocol/cc3501e.h> both repos compile, and
 * consumed here -- keeping the two numbers independent is how one of them
 * would eventually drift past the other unnoticed. */
#define CC3501E_EVENT_PAYLOAD_MAX ALP_CC3501E_EVENT_PAYLOAD_MAX

/* Initialise the ring to empty.  Called once from main(), right after
 * worker_init() and before the transport is started -- worker_init() itself does
 * NOT touch the ring. */
void event_ring_init(void);

/*
 * event_ring_push -- enqueue one async event.  IRQ-safe (short critical
 * section); callable from a producer thread OR an ISR.
 *
 *   evt_opcode -- the async opcode (one of ALP_CC3501E_EVT_*).
 *   payload    -- event payload bytes (may be NULL when len == 0).
 *   len        -- payload length, clamped to CC3501E_EVENT_PAYLOAD_MAX.
 *
 * Returns 1 if the event was queued, 0 if the ring was full (the event is
 * dropped -- the host will still learn the latest state on its next poll, e.g.
 * via CMD_WIFI_STATUS).
 */
int event_ring_push(uint8_t evt_opcode, const uint8_t *payload, uint8_t len);

/*
 * event_ring_drain -- pop queued events into @p out in the wire format
 * { evt_opcode(1) | len(1) | payload[len] } per entry, packed with no padding.
 * IRQ-safe; run inline from the SPI-ISR dispatch (handle_get_pending_events).
 *
 * Packs WHOLE entries only: an entry that would not fit in @p cap is LEFT in
 * the ring for the next drain (never split).  Returns the number of bytes
 * written to @p out (0 when the ring is empty or the first entry does not fit).
 *
 * The entry list is NOT self-delimiting, and protocol_build_reply() pads the
 * reply payload with zero bytes that the declared payload_len counts -- which is
 * exactly how an empty drain once decoded as three "opcode 0x00, len 0" events
 * (alp-sdk#1740).  The host walk stops at a zero opcode; keep opcode 0 out of
 * the wire format here, and read protocol.h's padding note before changing this
 * layout.
 */
size_t event_ring_drain(uint8_t *out, size_t cap);

#endif /* CC3501E_BRIDGE_EVENT_RING_H */
