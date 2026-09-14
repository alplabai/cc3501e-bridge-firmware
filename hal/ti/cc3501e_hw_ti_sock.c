/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * cc3501e-bridge HAL: TI backend -- TCP/UDP sockets (v0.5, lwIP BSD socket
 * path).
 *
 * Split by hardware subsystem out of cc3501e_hw_ti.c (issue #703, #461
 * Phase B).  cc3501e_hw_ti.c keeps platform lifecycle + the deferred-reboot
 * latch; see cc3501e_hw_ti_internal.h for the cross-TU seam.
 *
 * Built ONLY for CC3501E_HAL_BACKEND=ti (the bench build), against TI's
 * SimpleLink CC35xx SDK.  CI builds the stub backend instead, so this file
 * is never on the SDK-free path.
 */

#include <stdint.h>
#include <string.h>

#ifdef CC3501E_WIFI
/* lwIP BSD socket API for the TCP/UDP data path (CMD_SOCK_* 0x20..0x24): the
 * osi lwipopts enable LWIP_SOCKET + LWIP_COMPAT_SOCKETS + LWIP_TCP/UDP and the
 * prebuilt lwip.a carries sockets.c.  lwip_socket / lwip_connect / lwip_send /
 * lwip_recvfrom / lwip_close + struct sockaddr_in / SO_RCVTIMEO live here. */
#include <lwip/sockets.h>
/* Wi-Fi console UART logger (network_terminal demo adaptation/uart_term.c, linked
 * in the --wifi build): Report() surfaces the real reason a socket op failed on the
 * bench console -- the only diagnostic channel this headless bridge has. */
#include <uart_term.h>
/* FreeRTOS heap accounting (resolves at link time; declared here so the socket
 * failure path can report free heap without pulling the kernel headers). */
extern size_t xPortGetFreeHeapSize(void);
#endif

#include "alp/protocol/cc3501e.h"

/* Async-event ring (src/event_ring.h, on the firmware CMake include path):
 * cc3501e_hw_sock_accept_pump() publishes each accepted connection as an
 * EVT_SOCK_ACCEPTED entry the host drains with CMD_GET_PENDING_EVENTS -- the
 * same producer role hal/ti/cc3501e_hw_ti_wifi.c has for the Wi-Fi events. */
#include "event_ring.h"
/* Pure, silicon-free tail/uncommitted arithmetic for the lazy-commit fix
 * below (cc3501e-bridge-firmware, host review) -- see its own header for
 * why this is split out rather than inlined here. */
#include "sock_prefetch_arm.h"
#include "sock_recv_commit.h"
#include "sock_recv_ring_status.h"
#include "sock_worker_recv_eof.h"

#include "../cc3501e_hw.h"

/* --------------------------------------------------------------- */
/* TCP/UDP sockets (v0.5) -- lwIP BSD socket path.                   */
/*                                                                   */
/* CMD_SOCK_* (0x20..0x24) route here through the async worker: every */
/* lwip_* body below BLOCKS (a tcpip_apimsg round-trip to the lwIP   */
/* core thread; connect/recv also wait on the network), so -- like   */
/* the Wlan_* ops -- they MUST run in worker_run_pending, never the  */
/* SPI ISR.  The handle handed to the host is the lwIP fd + 1 so the */
/* protocol's "0 = invalid handle" contract holds (lwIP fds start at */
/* 0).  IPv4 only this rev (the osi lwipopts bring up an IPv4 stack). */
/* Under !CC3501E_WIFI (no lwIP) every body is NOTIMPL -> NOT_READY.  */
/* --------------------------------------------------------------- */
#ifdef CC3501E_WIFI
/* Bounded receive timeout so a worker RECV job can never wedge the drain on a
 * silent/half-open peer: after this window lwip_recv returns EWOULDBLOCK, which
 * the recv body maps to "0 bytes available" (OK) per the non-blocking wire
 * contract.  The host re-issues CMD_SOCK_RECV to poll for more. */
/* WAS 4000.  A blocking receive stalls the WHOLE WORKER for its duration, and
 * worker_run_pending() holds READY LOW across the whole job -- so no bridge
 * frame of ANY opcode is served while it waits.  A 4 s empty read therefore
 * blacked the bridge out for 4 s per poll, capping socket streaming at a
 * fraction of a frame per second and inverting against any host timeout
 * shorter than 4 s (the host gave up before the firmware could answer "0 bytes
 * available").
 *
 * cc3501e_hw_sock_recv() now passes MSG_DONTWAIT and does not rely on this at
 * all; it is kept as the socket's default so any OTHER blocking operation on
 * the handle is bounded to something short rather than to lwIP's default. */
/* Blocking timeout on the prefetch socket.  cc3501e_hw_sock_pump() runs once
 * per task tick, so EVERY momentarily-empty poll stalls the whole task for
 * this long while the bridge keeps draining the ring -- which is why replies
 * came back short.  Was 4000, then 50; 2 ms keeps the stall an order of
 * magnitude below a bridge transaction.  Do NOT go to 0/MSG_DONTWAIT: that
 * returned 0 bytes for 81 s on a connection the server had already fed
 * 256 KiB (bench-measured, reverted). */
#define CC3501E_SOCK_RCVTIMEO_MS 2

/* True iff fd is a SOCK_STREAM (TCP) socket.  Shared by sock_connect() below
 * (deciding whether to arm the prefetch ring) and, further down, the
 * worker-path sticky-EOF fix (deciding whether an lwip_recvfrom() n==0 means
 * "orderly close" rather than "empty UDP datagram"). */
static bool sock_is_stream(int fd)
{
	int       so_type    = 0;
	socklen_t so_type_sz = sizeof(so_type);
	return lwip_getsockopt(fd, SOL_SOCKET, SO_TYPE, &so_type, &so_type_sz) == 0 &&
	       so_type == SOCK_STREAM;
}

int cc3501e_hw_sock_open(uint8_t family, uint8_t type, uint8_t protocol, uint16_t *handle_out)
{
	if (handle_out == 0) {
		return CC3501E_HW_ERR_INVAL;
	}
	*handle_out = 0u;
	if (family != (uint8_t)ALP_CC3501E_SOCK_FAMILY_IPV4) {
		return CC3501E_HW_ERR_INVAL; /* v1 IP stack is IPv4-only */
	}
	const int st = (type == (uint8_t)ALP_CC3501E_SOCK_TYPE_DGRAM) ? SOCK_DGRAM : SOCK_STREAM;
	const int fd = lwip_socket(AF_INET, st, (int)protocol);
	if (fd < 0) {
		/* netconn allocation failed -- typically FreeRTOS-heap exhaustion for the
		 * recvmbox/sem, or MEMP_NUM_NETCONN starvation.  UNMASK the real reason on the
		 * bench console (errno + free heap), then FAIL FAST: return NOTIMPL, which the
		 * protocol layer maps to RESP_ERR_NOT_READY -- a NON-retryable host error.  (IO
		 * would map to RESP_ERR_RADIO -> host ALP_ERR_IO, which poll_by_repeat retries
		 * for the whole budget and masks as a -4 timeout.)  NOT_READY == "the IP stack
		 * cannot serve a socket right now", which is exactly this condition. */
		Report("\n\rcc3501e sock_open: lwip_socket failed errno=%d freeHeap=%u\n\r",
		       errno,
		       (unsigned)xPortGetFreeHeapSize());
		return CC3501E_HW_ERR_NOTIMPL;
	}
	/* lwIP fds are small non-negative ints; +1 keeps host handle 0 = invalid.  A
	 * full u16 table is unnecessary -- lwIP validates the fd (EBADF) on each op. */
	if (fd >= 0xFFFF) {
		(void)lwip_close(fd);
		return CC3501E_HW_ERR_IO;
	}
	struct timeval tv = { .tv_sec  = CC3501E_SOCK_RCVTIMEO_MS / 1000,
		                  .tv_usec = (CC3501E_SOCK_RCVTIMEO_MS % 1000) * 1000 };
	(void)lwip_setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	*handle_out = (uint16_t)(fd + 1);
	return CC3501E_HW_OK;
}

int cc3501e_hw_sock_connect(uint16_t handle, uint8_t family, uint16_t port, const uint8_t addr[4])
{
	if (handle == 0u || addr == 0) {
		return CC3501E_HW_ERR_INVAL;
	}
	if (family != (uint8_t)ALP_CC3501E_SOCK_FAMILY_IPV4) {
		return CC3501E_HW_ERR_INVAL;
	}
	const int          fd = (int)handle - 1;
	struct sockaddr_in sa;
	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port   = lwip_htons(port); /* host-order port -> network order */
	/* addr[0..3] are already big-endian (network order); s_addr is a network-order
	 * u32, so a straight copy lands the octets in the right byte positions. */
	memcpy(&sa.sin_addr.s_addr, addr, 4);
	if (lwip_connect(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0) {
		return CC3501E_HW_ERR_IO;
	}
	/* A connected STREAM socket is the bulk-receive case -- start prefetching so
	 * CMD_SOCK_RECV can be answered synchronously from the dispatch.
	 *
	 * STREAM ONLY (MINOR, host review of c354208): connect() is also legal on
	 * a DGRAM (UDP) socket -- it just latches a default peer, no handshake --
	 * and an earlier version of this call armed the ring for THAT too.  Under
	 * sock_prefetch_should_arm()'s first-connect-wins rule (MAJOR 4, 1118c99),
	 * a connected UDP socket then held the ring for its own lifetime and
	 * locked out any STREAM socket's prefetch -- the case this fast path
	 * actually exists for.  Check the real socket type with SO_TYPE rather
	 * than trust a naming convention. */
	if (sock_is_stream(fd)) {
		cc3501e_hw_sock_prefetch(handle, true);
	}
	return CC3501E_HW_OK;
}

/* ==================== LISTENING SOCKETS (protocol v9) ====================
 *
 * The host binds + listens, and every inbound connection is delivered as an
 * EVT_SOCK_ACCEPTED event carrying a ready-to-use handle.  There is no accept
 * opcode on the wire, because accept() blocks and a worker-routed blocking body
 * holds READY LOW for its whole duration -- an accept opcode would black the
 * whole bridge out for as long as no client happened to connect.
 *
 * So the accept runs on the TASK, NON-BLOCKING, once per housekeeping tick,
 * exactly like the RX prefetch pump above and for the same reason.
 *
 * The table below is what the pump iterates.  Four slots because a listening
 * socket costs 2 bytes here and a product may plausibly serve more than one
 * port (an HTTP console plus a provisioning port); this is NOT the
 * single-socket restriction the prefetch ring has, which exists for a different
 * reason (one producer / one consumer on that ring). */
#define CC3501E_SOCK_LISTEN_MAX 4u
/* lwIP backlog when the host passes 0.  Small on purpose: each queued
 * connection holds a netconn, and MEMP_NUM_NETCONN is the scarce resource that
 * makes lwip_socket() fail (see the sock_open failure path above). */
#define CC3501E_SOCK_LISTEN_BACKLOG_DEFAULT 4

/* Listening handles (fd + 1; 0 = free slot).  Touched only from the TASK:
 * bind/listen/close run in the worker drain and the pump runs in the tick, and
 * main.c runs both on the same bring-up task -- so no critical section is
 * needed here, unlike the ISR-vs-task ring above. */
static uint16_t listen_handles[CC3501E_SOCK_LISTEN_MAX];

static void listen_table_remove(uint16_t handle)
{
	for (unsigned i = 0u; i < CC3501E_SOCK_LISTEN_MAX; ++i) {
		if (listen_handles[i] == handle) {
			listen_handles[i] = 0u;
		}
	}
}

int cc3501e_hw_sock_bind(uint16_t handle, uint8_t family, uint16_t port, const uint8_t addr[4])
{
	if (handle == 0u || addr == 0) {
		return CC3501E_HW_ERR_INVAL;
	}
	if (family != (uint8_t)ALP_CC3501E_SOCK_FAMILY_IPV4) {
		return CC3501E_HW_ERR_INVAL;
	}
	const int fd = (int)handle - 1;

	/* SO_REUSEADDR so a server that closes and re-binds the same port does not
	 * have to wait out TIME_WAIT.  Restarting the serving app is the ordinary
	 * case for an embedded console, and without this the re-bind fails with
	 * EADDRINUSE for minutes, which reads as "the firmware broke". */
	int on = 1;
	(void)lwip_setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

	struct sockaddr_in sa;
	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port   = lwip_htons(port);
	/* addr[0..3] are already big-endian (network order), same convention as
	 * connect above; all-zero is INADDR_ANY, which is the normal choice for a
	 * server on the soft-AP (the AP address does not exist until the role is
	 * up, so binding it explicitly would race the role-up). */
	memcpy(&sa.sin_addr.s_addr, addr, 4);
	if (lwip_bind(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0) {
		Report(
		    "\n\rcc3501e sock_bind: lwip_bind port=%u failed errno=%d\n\r", (unsigned)port, errno);
		return CC3501E_HW_ERR_IO;
	}
	return CC3501E_HW_OK;
}

int cc3501e_hw_sock_listen(uint16_t handle, uint8_t backlog)
{
	if (handle == 0u) {
		return CC3501E_HW_ERR_INVAL;
	}
	const int fd = (int)handle - 1;

	/* NON-BLOCKING BEFORE PASSIVE, and the order matters: the pump calls
	 * lwip_accept() on this fd from the housekeeping tick, and a BLOCKING accept
	 * there stalls the whole task -- and with it the RX pump, the OTA pump and
	 * the SPI self-heals -- until a client connects.  That is a dead bridge,
	 * recoverable only by a power cycle.
	 *
	 * lwip_fcntl(O_NONBLOCK) is the primitive for it.  If it is unavailable on
	 * this build it returns -1, and rather than proceed with a socket whose
	 * accept can block forever, fall back to a 1 ms SO_RCVTIMEO -- netconn_accept
	 * honours the same receive timeout, so the accept stays BOUNDED.  Never
	 * leave this socket in the default wait-forever mode. */
	if (lwip_fcntl(fd, F_SETFL, O_NONBLOCK) < 0) {
		struct timeval tv = { .tv_sec = 0, .tv_usec = 1000 };

		if (lwip_setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0) {
			Report("\n\rcc3501e sock_listen: cannot bound accept (fcntl+rcvtimeo failed)\n\r");
			return CC3501E_HW_ERR_IO;
		}
	}

	/* RESERVE THE PUMP SLOT BEFORE GOING PASSIVE.  If lwip_listen() ran first and
	 * the table turned out to be full, the socket would already be accepting
	 * handshakes into a backlog nothing ever drains: a client would see a
	 * connected socket and then silence, which is worse than a refused connect
	 * and is invisible from the host.  Claiming the slot first means the failure
	 * happens while the socket is still inert.
	 *
	 * Re-listening on an already-registered handle refreshes its slot rather
	 * than consuming a second one. */
	listen_table_remove(handle);
	unsigned slot = CC3501E_SOCK_LISTEN_MAX;
	for (unsigned i = 0u; i < CC3501E_SOCK_LISTEN_MAX; ++i) {
		if (listen_handles[i] == 0u) {
			slot = i;
			break;
		}
	}
	if (slot == CC3501E_SOCK_LISTEN_MAX) {
		/* Table full, and the socket is NOT listening -- nothing to undo.  Fail
		 * loudly.  ERR_STATE, not ERR_IO: this is a deterministic, terminal
		 * refusal (the host asked for more listening sockets than this backend
		 * tracks), and ERR_IO would map to RESP_ERR_RADIO, which poll_by_repeat
		 * retries for its whole budget. */
		Report("\n\rcc3501e sock_listen: no free listen slot (max %u)\n\r",
		       (unsigned)CC3501E_SOCK_LISTEN_MAX);
		return CC3501E_HW_ERR_STATE;
	}
	listen_handles[slot] = handle;

	const int bl = (backlog == 0u) ? CC3501E_SOCK_LISTEN_BACKLOG_DEFAULT : (int)backlog;
	if (lwip_listen(fd, bl) != 0) {
		listen_handles[slot] = 0u; /* never went passive -- give the slot back */
		Report("\n\rcc3501e sock_listen: lwip_listen failed errno=%d\n\r", errno);
		return CC3501E_HW_ERR_IO;
	}
	return CC3501E_HW_OK;
}

void cc3501e_hw_sock_accept_pump(void)
{
	for (unsigned i = 0u; i < CC3501E_SOCK_LISTEN_MAX; ++i) {
		const uint16_t lh = listen_handles[i];

		if (lh == 0u) {
			continue;
		}
		struct sockaddr_in peer;
		socklen_t          peerlen = sizeof(peer);
		memset(&peer, 0, sizeof(peer));
		const int nfd = lwip_accept((int)lh - 1, (struct sockaddr *)&peer, &peerlen);
		if (nfd < 0) {
			continue; /* EWOULDBLOCK -- nobody connecting on this one right now */
		}
		if (nfd >= 0xFFFF) {
			(void)lwip_close(nfd); /* cannot be expressed as a u16 handle */
			continue;
		}
		/* The ACCEPTED socket does NOT inherit the listener's options, so give it
		 * the same bounded receive timeout cc3501e_hw_sock_open() sets.  Without
		 * it a worker-routed CMD_SOCK_RECV on an idle connection blocks in lwIP
		 * with no timeout at all, holding READY LOW and wedging the bridge.
		 *
		 * The SEND direction needs no option here: this SDK's lwIP build does not
		 * enable LWIP_SO_SNDTIMEO, so there is no SO_SNDTIMEO to set, and
		 * cc3501e_hw_sock_send() passes MSG_DONTWAIT instead (#107).  A peer that
		 * connects and stops reading now gets "0 bytes queued" replies rather
		 * than parking the worker inside lwip_send with READY LOW. */
		struct timeval tv = { .tv_sec  = CC3501E_SOCK_RCVTIMEO_MS / 1000,
			                  .tv_usec = (CC3501E_SOCK_RCVTIMEO_MS % 1000) * 1000 };
		(void)lwip_setsockopt(nfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

		/* alp_cc3501e_sock_accepted_evt_t, packed: listen_handle(LE16) |
		 * handle(LE16) | peer_port(LE16, host order) | peer_family | reserved |
		 * peer_addr[4] (network order, MSB first -- the same convention
		 * cc3501e_sock_connect's addr[] uses). */
		const uint16_t nh    = (uint16_t)(nfd + 1);
		const uint16_t pport = lwip_ntohs(peer.sin_port);
		uint8_t        ev[12];

		ev[0] = (uint8_t)(lh & 0xFFu);
		ev[1] = (uint8_t)((lh >> 8) & 0xFFu);
		ev[2] = (uint8_t)(nh & 0xFFu);
		ev[3] = (uint8_t)((nh >> 8) & 0xFFu);
		ev[4] = (uint8_t)(pport & 0xFFu);
		ev[5] = (uint8_t)((pport >> 8) & 0xFFu);
		ev[6] = (uint8_t)ALP_CC3501E_SOCK_FAMILY_IPV4;
		ev[7] = 0u;
		memcpy(&ev[8], &peer.sin_addr.s_addr, 4);

		if (event_ring_push((uint8_t)ALP_CC3501E_EVT_SOCK_ACCEPTED, ev, sizeof(ev)) == 0) {
			/* Ring full -- the host would never learn this handle exists, so it
			 * would leak a firmware socket for the life of the boot.  Close it and
			 * let the client retry; a full ring means the host is not draining. */
			(void)lwip_close(nfd);
			Report("\n\rcc3501e accept: event ring full, dropped a connection\n\r");
		}
	}
}

int cc3501e_hw_sock_send(uint16_t       handle,
                         uint8_t        flags,
                         const uint8_t *data,
                         uint16_t       data_len,
                         uint16_t      *sent_out)
{
	(void)flags; /* MORE hint is advisory; lwip_send has no matching flag here */
	if (sent_out != 0) *sent_out = 0u;
	if (handle == 0u || (data == 0 && data_len > 0u)) {
		return CC3501E_HW_ERR_INVAL;
	}
	const int fd = (int)handle - 1;

	/* NON-BLOCKING, deliberately.  The previous blocking lwip_send(..., 0) parked
	 * the worker for as long as the peer declined to read (#107): a peer that
	 * stops reading fills its window, the send buffer fills behind it, and
	 * lwip_send waits until lwIP abandons the connection.  SOCK_SEND is
	 * worker-routed and the worker holds READY low across the call, so the bridge
	 * served NO opcode for that whole period -- not just sockets.  It read as a
	 * dead link.  With the listening socket, any client associated to the
	 * soft-AP could trigger that on purpose.
	 *
	 * SO_SNDTIMEO would be the obvious bound and is NOT available: lwIP ships
	 * prebuilt in the vendor library without LWIP_SO_SNDTIMEO, so the option does
	 * not exist on this build.
	 *
	 * MSG_DONTWAIT copies whatever fits into the send buffer and returns at once.
	 * That is already what the wire contract describes -- SOCK_SEND replies with
	 * a QUEUED byte count and a short count is legal -- so a full buffer becomes
	 * "0 bytes queued", reported as success, and the host decides whether and
	 * when to retry.  Retry policy belongs to the host; the bridge's job is to
	 * stay responsive while the peer sulks.
	 *
	 * DO NOT assume this is equivalent to the receive side's MSG_DONTWAIT
	 * failure, and DO NOT assume it is safe either.  On receive, MSG_DONTWAIT
	 * returned 0 bytes for 81 s on a live connection because the data had not yet
	 * been moved from the pcb into the socket's receive box.  Send has no such
	 * intermediate hop -- it writes into the TCP send buffer directly -- so the
	 * same pathology is not expected, but this stack's non-blocking paths have
	 * surprised this tree before.  It must be measured with
	 * aen-cc3501e-socket-throughput before merge: if send throughput collapses
	 * toward zero, the non-blocking send is not accepting data and this change
	 * must not ship. */
	const ssize_t n = lwip_send(fd, data, data_len, MSG_DONTWAIT);
	if (n < 0) {
		if (errno == EWOULDBLOCK || errno == EAGAIN) {
			/* Send buffer full: the peer is not reading.  Nothing queued, and not
			 * an error -- report 0 so the host can back off and retry. */
			if (sent_out != 0) *sent_out = 0u;
			return CC3501E_HW_OK;
		}
		return CC3501E_HW_ERR_IO;
	}
	if (sent_out != 0) *sent_out = (uint16_t)n;
	return CC3501E_HW_OK;
}

/* ================= SOCKET RX PREFETCH RING =================
 *
 * WHY: protocol_dispatch() runs in the SPI transfer-complete callback (SWI/HWI
 * context) and cannot call lwIP, so CMD_SOCK_RECV had to be worker-routed --
 * handle_worker_routed_payload_reply always answers BUSY to the submit and the
 * host must come back for the answer.  That is TWO bridge transactions plus a
 * wait for the worker loop per frame, and it is the dominant per-frame cost of a
 * socket stream.
 *
 * The OTA write path already solved this shape: cc3501e_hw_ota_write is
 * SYNCHRONOUS because it only memcpy's into a staged window.  Do the same here.
 * The TASK side (cc3501e_hw_tick -> cc3501e_hw_sock_pump) does the lwIP work and
 * fills this ring; the DISPATCH side only memcpy's out of it, which is ISR-safe.
 * CMD_SOCK_RECV then costs ONE transaction and no worker round trip.
 *
 * Single stream socket by design: this serves the bulk-receive case, and one
 * ring keeps the ISR-vs-task handshake to a single producer and a single
 * consumer (head written by the task only, tail by the dispatch only). */
/* Ring size is the LARGEST measured throughput lever on this bridge.  The pump can
 * only refill from the TASK (lwip_recv cannot run in the SPI dispatch ISR) and the
 * task ticks every 10 ms, so a small ring runs dry between passes -- and every miss
 * costs the host a 1 ms poll_by_repeat backoff.  Measured end-to-end HTTP over the
 * bridge: 8 KB = ~660 kB/s, 16 KB = ~730 kB/s, 64 KB = ~742 kB/s.
 *
 * Past 16 KB it does not fit in DRAM (that bank has 431 bytes spare, and a 32 KB
 * ring overflows GROUP_8 by ~16 KB), so the ring is linked into TCM instead -- see
 * the .bss.sock_ring placement in ti/build_ti.ps1.  TCM is safe here BECAUSE the
 * ring is CPU-only memory: the pump memcpy's in, the dispatch memcpy's out into
 * reply_buf, and no DMA engine ever addresses it. */
#define CC3501E_SOCK_RING_BYTES 65536u
/* Max lwip_recv() calls per pump tick; see cc3501e_hw_sock_pump(). */
#define CC3501E_SOCK_PUMP_PASSES 8u

static struct {
	uint8_t           buf[CC3501E_SOCK_RING_BYTES];
	volatile uint32_t head;     /* task writes   */
	volatile uint32_t tail;     /* dispatch reads */
	volatile uint16_t fd_plus1; /* socket being prefetched, 0 = none */
	volatile bool     peer_closed;
	/* Bug 2 fix (ring path never reports a reset): set by cc3501e_hw_sock_pump()
	 * on a REAL lwip_recv() failure (RST etc, not EAGAIN/EWOULDBLOCK).  Before
	 * this field existed the pump silently ignored n < 0 and the ring just
	 * kept answering BUSY forever, so the host spun to timeout_ms and reported
	 * ALP_ERR_TIMEOUT instead of an error -- see cc3501e_hw_sock_recv_ring()'s
	 * drained-and-errored arm and cc3501e_hw_sock_pump() below. */
	volatile bool peer_error;
} rx_ring __attribute__((section(".bss.sock_ring")));

static uint32_t ring_used(void)
{
	return rx_ring.head - rx_ring.tail; /* free-running; unsigned wrap is correct */
}

/* LAZY-COMMIT byte count: the most recent cc3501e_hw_sock_recv_ring() call's
 * served-but-not-yet-retired byte count (see sock_recv_commit.h).  NOT
 * dispatch-context-only, unlike an earlier draft of this comment claimed:
 * cc3501e_hw_sock_prefetch() (below) also writes it, on the WORKER TASK,
 * from the socket open/close paths (sock_connect / sock_close) -- see the
 * CONCURRENCY comment below for why that makes this a genuine cross-context
 * field and how the publish order there keeps it safe without `volatile`. */
static uint32_t uncommitted;

/* CONCURRENCY: SPSC for the ring's DATA (head/tail/buf), but lazy-commit
 * gives `uncommitted` a SECOND writer beyond the dispatch/consumer:
 * rx_ring.head is written ONLY by the task (cc3501e_hw_sock_pump, the
 * producer) and read only by the dispatch/consumer.  rx_ring.tail is
 * written ONLY by the dispatch/consumer (cc3501e_hw_sock_recv_ring) and
 * read only by the task (ring_used(), for the pump's headroom guard).
 * `uncommitted` is written by the dispatch/consumer on every serve, AND by
 * the WORKER TASK's cc3501e_hw_sock_prefetch() on arm/disarm -- so, unlike
 * head/tail, it is not single-writer, and an earlier version of this
 * comment was wrong to say it "adds NO new cross-context field".
 *
 * That second writer is why cc3501e_hw_sock_prefetch()'s arm branch resets
 * rx_ring.head/tail/uncommitted BEFORE publishing rx_ring.fd_plus1, not
 * after: cc3501e_hw_sock_recv_ring() (dispatch context, can run at any SPI
 * callback) only ever touches uncommitted/head/tail for a handle that
 * currently matches fd_plus1.  Published last, fd_plus1 acts as the
 * release: dispatch cannot observe the new handle until head, tail, and
 * uncommitted are ALL already reset for it.  Publishing fd_plus1 first (an
 * earlier version of this function did) opens a window where a dispatch
 * call sees the new handle's fd_plus1 but a stale, not-yet-reset
 * uncommitted from whatever handle used the ring last: sock_recv_commit()
 * then folds that stale count into a tail that has ALREADY been zeroed,
 * producing tail = stale_uncommitted while head = 0, which underflows
 * ring_used()'s unsigned head - tail to a huge value -- stale ring memory
 * gets served as a normal OK reply, and the pump never gets a chance to
 * refill anything since the "used" it now sees is (falsely) enormous.
 *
 * The other new invariant lazy-commit relies on: cc3501e_hw_sock_recv_ring()
 * computes the post-commit tail into a LOCAL variable, does the full memcpy
 * out of the ring using that local value, and publishes it to the shared
 * rx_ring.tail LAST -- exactly the order the pre-existing code already used
 * (copy, then `tail += n`).  Publishing the advance before the copy would
 * let the pump's headroom check (ring_used(), which reads rx_ring.tail)
 * treat the not-yet-copied bytes as free room and overwrite them with
 * newly-pumped data while this function was still reading them out -- the
 * SAME hazard an eager, pre-copy tail advance would have had even without
 * lazy-commit; this fix does not introduce it, it only has to keep not
 * introducing it while holding the tail back for longer. */

/* TASK CONTEXT ONLY -- called from cc3501e_hw_tick().  Does the lwIP read. */
#ifdef CC3501E_RADIO_SPEEDTEST
/* BENCH: radio-only throughput.  Drains the prefetch socket and DISCARDS the
 * data, so the rate measured is what the RADIO delivers with the bridge
 * completely out of the path.  Every other number in this bring-up measures the
 * radio THROUGH the bridge and therefore cannot tell a radio limit from a bridge
 * limit -- this one can.  Result is published in the GET_DIAG_INFO free_heap
 * field as bytes/second. */
volatile uint32_t g_radio_bps;

static void radio_speedtest_pump(int fd)
{
	static uint8_t  sink[2048];
	static uint32_t t0_ms, total;

	for (uint32_t pass = 0u; pass < 16u; ++pass) {
		const ssize_t n = lwip_recv(fd, sink, sizeof(sink), 0);

		if (n <= 0) {
			break;
		}
		/* Start the clock on the FIRST BYTE, not on socket-open: the gap between
		 * arming the socket and the server's first segment is idle time that
		 * would otherwise be averaged into the rate. */
		if (t0_ms == 0u) {
			t0_ms = cc3501e_hw_uptime_ms() | 1u;
		}
		total += (uint32_t)n;
		if ((uint32_t)n < sizeof(sink)) {
			break; /* socket drained -- don't spend a timeout on the next pass */
		}
	}
	/* Publish a WINDOWED rate: once a window's worth of real data has landed,
	 * report it and restart.  A cumulative average from socket-open decays toward
	 * zero as soon as the transfer finishes and the socket goes idle, which is
	 * what made the first attempt read 12 kB/s on a link doing far more. */
	if (total >= 131072u && t0_ms != 0u) {
		const uint32_t dt = cc3501e_hw_uptime_ms() - t0_ms;

		if (dt > 0u) {
			g_radio_bps = (uint32_t)(((uint64_t)total * 1000u) / dt);
		}
		total = 0u;
		t0_ms = 0u;
	}
}
#endif

#ifdef CC3501E_RADIO_SPEEDTEST
/* UDP arm of the radio-only test: bind a socket and drain it, so the rate is the
 * radio's UDP capability -- the figure the datasheet's "20Mbps (UDP)" refers to,
 * and the one that decides whether a >1 MB/s target is reachable at all.  Needs
 * no host request: a PC just blasts datagrams at the board's IP on this port. */
#define CC3501E_RADIO_UDP_PORT 5001

static void radio_speedtest_udp(void)
{
	static int      ufd = -1;
	static uint8_t  usink[2048];
	static uint32_t ut0, utotal;

	if (ufd < 0) {
		struct sockaddr_in a;

		ufd = lwip_socket(AF_INET, SOCK_DGRAM, 0);
		if (ufd < 0) {
			return;
		}
		memset(&a, 0, sizeof(a));
		a.sin_family      = AF_INET;
		a.sin_port        = lwip_htons(CC3501E_RADIO_UDP_PORT);
		a.sin_addr.s_addr = 0; /* INADDR_ANY */
		if (lwip_bind(ufd, (struct sockaddr *)&a, sizeof(a)) != 0) {
			lwip_close(ufd);
			ufd = -1;
			return;
		}
		{
			struct timeval tv = { .tv_sec = 0, .tv_usec = 2000 };

			(void)lwip_setsockopt(ufd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
		}
	}
	for (uint32_t pass = 0u; pass < 32u; ++pass) {
		const ssize_t n = lwip_recv(ufd, usink, sizeof(usink), 0);

		if (n <= 0) {
			break;
		}
		if (ut0 == 0u) {
			ut0 = cc3501e_hw_uptime_ms() | 1u;
		}
		utotal += (uint32_t)n;
	}
	if (utotal >= 131072u && ut0 != 0u) {
		const uint32_t dt = cc3501e_hw_uptime_ms() - ut0;

		if (dt > 0u) {
			g_radio_bps = (uint32_t)(((uint64_t)utotal * 1000u) / dt);
		}
		utotal = 0u;
		ut0    = 0u;
	}
}
#endif

void cc3501e_hw_sock_pump(void)
{
#ifdef CC3501E_RADIO_SPEEDTEST
	radio_speedtest_udp(); /* runs whether or not the host armed a TCP socket */
#endif
	const uint16_t h = rx_ring.fd_plus1;
	if (h == 0u || rx_ring.peer_closed || rx_ring.peer_error) {
		return;
	}
#ifdef CC3501E_RADIO_SPEEDTEST
	(void)radio_speedtest_pump;
	return; /* discard mode: never fill the ring */
#endif
	/* Drain what lwIP already has, not one segment per tick.  The bridge takes up
	 * to ALP_CC3501E_MAX_PAYLOAD per transaction while a single lwip_recv()
	 * returns about one TCP segment, so a one-shot pump left the ring
	 * under-filled and every reply came back short.  Safe only because
	 * CC3501E_SOCK_RCVTIMEO_MS is small -- at the old 50 ms an extra pass cost
	 * more than a transaction. */
	for (uint32_t pass = 0u; pass < CC3501E_SOCK_PUMP_PASSES; ++pass) {
		/* ring_used() = head - tail already treats LAZY-COMMIT's uncommitted
		 * bytes as still "used": tail is not advanced past a served chunk
		 * until the NEXT dispatch call proves the host moved on
		 * (sock_recv_commit.h), so this headroom check sees exactly the
		 * same (or a MORE conservative, never smaller) `used` it would have
		 * without lazy-commit -- no change needed here for those bytes to
		 * stay protected from being overwritten before a possible replay
		 * re-serves them. */
		uint32_t used = ring_used();
		if (used > CC3501E_SOCK_RING_BYTES - (uint32_t)ALP_CC3501E_MAX_PAYLOAD) {
			return; /* keep at least one max frame of headroom */
		}
		const uint32_t idx   = rx_ring.head % CC3501E_SOCK_RING_BYTES;
		uint32_t       chunk = CC3501E_SOCK_RING_BYTES - idx; /* to the wrap only */
		const uint32_t space = CC3501E_SOCK_RING_BYTES - used;
		if (chunk > space) chunk = space;
		if (chunk == 0u) {
			return;
		}
		const ssize_t n = lwip_recv((int)h - 1, &rx_ring.buf[idx], (size_t)chunk, 0);
		if (n > 0) {
			rx_ring.head += (uint32_t)n;
			if ((uint32_t)n < chunk) {
				return; /* socket drained -- don't spend a timeout on the next pass */
			}
			continue;
		}
		if (n == 0) {
			rx_ring.peer_closed = true; /* orderly close */
			return;
		}
		/* n < 0.  EAGAIN/EWOULDBLOCK is just "nothing yet" -- next tick, ring
		 * stays armed.  Anything else (ECONNRESET etc) is a REAL failure: the
		 * top-of-function guard above stops this task from calling lwip_recv()
		 * on this fd again, so record it sticky for
		 * cc3501e_hw_sock_recv_ring() to report once the already-buffered
		 * bytes are drained (bug 2 fix -- see the peer_error field comment
		 * above rx_ring). */
		if (errno != EAGAIN
#ifdef EWOULDBLOCK
		    && errno != EWOULDBLOCK
#endif
		) {
			rx_ring.peer_error = true;
		}
		return;
	}
}

/* Arm/disarm prefetch for a handle.  Called on the WORKER TASK, from the
 * socket open/close paths (sock_connect / sock_close), which can run
 * concurrently with a dispatch-context (SPI callback) call into
 * cc3501e_hw_sock_recv_ring() below.
 *
 * Arm-on publishes rx_ring.fd_plus1 LAST, after head/tail/uncommitted are
 * ALL already reset to 0 -- not first, as an earlier version of this
 * function did.  fd_plus1 is the only field cc3501e_hw_sock_recv_ring()
 * checks before touching the rest of the ring, so publishing it last makes
 * it the release: dispatch cannot observe the new handle until everything
 * else is already consistent for it.  Publishing it first left a window
 * where a dispatch call could see the NEW handle's fd_plus1 while
 * `uncommitted` still held the OLD handle's stale served-but-not-retired
 * count -- sock_recv_commit() would then fold that stale count into a tail
 * already zeroed for the new handle, underflowing ring_used()'s unsigned
 * head - tail and serving stale ring memory as a normal OK reply forever
 * (see the CONCURRENCY comment above rx_ring for the full trace).
 *
 * Resets `uncommitted` on BOTH arms, not just the arm-on head/tail reset: a
 * fresh arm must never inherit a stale count from whatever handle used the
 * ring last (arm-on, ordering above), and a disarm must not leave one
 * behind to confuse a future arm that, for whatever reason, reads it before
 * its own first serve sets it (arm-off) -- cheap insurance for a single
 * uint32_t, not a case this needs to actually reach in practice; arm-off's
 * ordering has no equivalent race since fd_plus1 = 0 immediately stops
 * cc3501e_hw_sock_recv_ring() from touching the ring for this handle at
 * all, regardless of what order the rest of this branch runs in.
 *
 * ARM-ON REFUSES TO STEAL THE RING FROM A DIFFERENT, ALREADY-ARMED HANDLE
 * (MAJOR, host review, 1118c99).  sock_prefetch_should_arm()
 * (sock_prefetch_arm.h) is the whole decision: an earlier version of this
 * branch ran the head/tail/uncommitted reset UNCONDITIONALLY on every
 * arm-on call, so a SECOND concurrently-open STREAM socket's connect()
 * silently discarded the FIRST handle's already-pumped, not-yet-served
 * bytes the instant it landed -- the first handle then fell to the
 * worker-routed path and read PAST that hole with lwip_recvfrom(),
 * reporting OK on a stream with a silent gap in it.  Refusing to re-arm here
 * leaves the FIRST handle's ring untouched; the handle asking to arm just
 * falls back to the worker-routed path for its own recvs instead (covered by
 * protocol_sockets.c's own worker-fallback replay cache), same as any other
 * never-armed handle.  DISARM (the `on == false` arm below, called from
 * cc3501e_hw_sock_close()) is what frees the ring for a LATER connect to
 * arm: closing the CURRENTLY-armed handle always reaches the
 * `rx_ring.fd_plus1 == handle` branch and clears fd_plus1 to 0, so a
 * connect() on a NEW handle after that CAN arm -- confirmed against
 * cc3501e_hw_sock_close()'s own unconditional cc3501e_hw_sock_prefetch(handle,
 * false) call.  A handle NUMBER the host reuses after closing therefore
 * always arrives here as armed == 0 (disarmed by the close above), not
 * armed == requested -- that is the ordinary cold-arm path above, not the
 * same-handle no-op below.  NOT also cleared here: a peer that closes ITS
 * end and drains to EOF (rx_ring.peer_closed, cc3501e_hw_sock_pump()) does
 * NOT itself disarm the ring -- that is unrelated, pre-existing behaviour
 * this fix does not change; the ring stays "armed" for a half-closed handle
 * until the host explicitly issues SOCK_CLOSE on it.
 *
 * ARM-ON FOR THE SAME ALREADY-ARMED HANDLE IS A TRUE NO-OP (MINOR, host
 * review of c354208): an earlier version ran the head/tail/uncommitted reset
 * unconditionally whenever sock_prefetch_should_arm() allowed the arm at
 * all, including armed == requested, so re-arming the SAME still-open
 * handle dropped that handle's OWN already-pumped, not-yet-served bytes.
 * Only a cold arm (fd_plus1 == 0) or taking the ring from a DIFFERENT
 * handle resets it now; armed == requested returns immediately, below,
 * leaving the ring exactly as it was. */
void cc3501e_hw_sock_prefetch(uint16_t handle, bool on)
{
	if (on) {
		if (!sock_prefetch_should_arm(rx_ring.fd_plus1, handle)) {
			return;
		}
		if (rx_ring.fd_plus1 == handle) {
			/* Already armed for THIS handle (MINOR, host review of c354208):
			 * a true no-op, not a fresh arm -- the reset below is for taking
			 * the ring from cold (fd_plus1 == 0) or from a DIFFERENT prior
			 * handle, and running it here would drop whatever THIS SAME
			 * handle's socket has already pumped into the ring but not yet
			 * served.  Nothing to publish either: fd_plus1 already reads
			 * `handle`. */
			return;
		}
		rx_ring.head = rx_ring.tail = 0u;
		rx_ring.peer_closed         = false;
		rx_ring.peer_error          = false; /* bug 2 fix -- reset on arm, same as peer_closed */
		sock_recv_commit_reset(&uncommitted);
		rx_ring.fd_plus1 = handle; /* publish LAST -- see comment above */
	} else if (rx_ring.fd_plus1 == handle) {
		rx_ring.fd_plus1 = 0u;
		sock_recv_commit_reset(&uncommitted);
	}
}

/* DISPATCH CONTEXT (SWI/HWI) -- memcpy only, never lwIP.  Returns bytes taken,
 * or -1 when this handle is not the prefetched one so the caller can fall back
 * to the worker path.
 *
 * @p replay -- see sock_recv_commit.h and hal/cc3501e_hw.h's doc comment on
 * this function.  Threaded straight through to sock_recv_commit(), which
 * owns the tail/uncommitted decision; this function still owns the ring
 * buffer itself (the actual copy) and the publish-order guarantee (copy
 * fully out BEFORE advancing the shared rx_ring.tail the pump's headroom
 * check reads -- see the CONCURRENCY comment above rx_ring). */
int cc3501e_hw_sock_recv_ring(uint16_t  handle,
                              uint8_t  *buf,
                              uint16_t  cap,
                              bool      replay,
                              uint16_t *out_len)
{
	if (out_len != 0) *out_len = 0u;
	if (rx_ring.fd_plus1 != handle || handle == 0u || buf == 0) {
		return -1;
	}

	/* LAZY-COMMIT (issue: silent SOCK_RECV data loss on a CRC-rejected reply,
	 * host review): fold in the PREVIOUS call's serve now, unless @p replay
	 * says this call is poll_by_repeat() re-issuing that same request under
	 * the identical seq -- see sock_recv_commit.h for the full contract.
	 * This MUST run before the empty/closed checks below: they need the
	 * POST-commit position, not the raw rx_ring.tail, or a genuinely NEW
	 * (non-replay) request landing exactly on a ring with nothing past the
	 * just-committed bytes would wrongly re-serve them as if they were new
	 * instead of correctly reporting empty/closed.
	 *
	 * Computed into a LOCAL `tail`, NOT YET published to the shared,
	 * pump-visible rx_ring.tail -- see the CONCURRENCY comment above rx_ring
	 * for why that publish has to wait until after any copy below
	 * completes (or happen immediately when there is no copy -- the empty/
	 * closed paths below, where publishing right away is safe: nothing is
	 * being read out of the ring for the pump to race). */
	uint32_t       tail = rx_ring.tail;
	const uint32_t n = sock_recv_commit(&tail, &uncommitted, rx_ring.head, replay, (uint32_t)cap);

	if (n == 0u) {
		rx_ring.tail = tail; /* nothing to copy -- safe to publish now */
		/* Nothing left to serve.  Which of the three answers this is (EOF /
		 * terminal error / empty-for-now) is a pure decision over
		 * peer_closed and peer_error -- see sock_recv_ring_status.h for the
		 * full argument (in particular why EOF is checked first, why -2 vs
		 * -1 matters (#7: the pump is the ONLY reader of this fd, so the
		 * caller must NOT fall through to the worker path on either -2 or
		 * -3), and why -3 must not be answered as BUSY: nothing further is
		 * ever coming once peer_error is set, so BUSY would just spin the
		 * host to its poll_by_repeat timeout instead of reporting the
		 * failure (bug 2 fix)). */
		return sock_recv_ring_drained_status(rx_ring.peer_closed, rx_ring.peer_error);
	}

	const uint32_t idx   = tail % CC3501E_SOCK_RING_BYTES;
	uint32_t       first = CC3501E_SOCK_RING_BYTES - idx;
	if (first > n) first = n;
	memcpy(buf, &rx_ring.buf[idx], first);
	if (n > first) {
		memcpy(&buf[first], &rx_ring.buf[0], n - first);
	}
	/* Publish LAST, after the copy above has fully read out [idx, idx+n) --
	 * see the CONCURRENCY comment above rx_ring.  Until this line, the
	 * pump's headroom check (ring_used(), which reads rx_ring.tail) still
	 * sees the OLD tail and so still treats these bytes as used. */
	rx_ring.tail = tail;
	if (out_len != 0) *out_len = (uint16_t)n;
	return (int)n;
}

/* WORKER-PATH STICKY EOF (bug: worker-path recv after EOF answers
 * RESP_ERR_RADIO, bench-measured run12 P2b, 3/3 boots).  Mirrors the RING
 * path's rx_ring.peer_closed above: that ring already survives an orderly
 * close because cc3501e_hw_sock_pump() sets peer_closed exactly once (on
 * lwip_recv() n == 0) and cc3501e_hw_sock_recv_ring() keeps answering OK/0
 * for that handle forever after, without calling lwIP again.  The WORKER
 * path -- serving UDP sockets, and STREAM sockets accepted but never armed
 * for prefetch -- had no equivalent, and TI's lwIP (LWIP_NETCONN_FULLDUPLEX=0)
 * punishes a second call after FIN:
 *
 *   1st post-FIN recv: netconn_recv_data_tcp()'s handle_fin arm calls
 *      netconn_close_shutdown(conn, NETCONN_SHUT_RD) (lwip-stack/src/api/
 *      api_lib.c:763), which frees conn->recvmbox and returns ERR_CLSD.
 *      sockets.c's lwip_recv_tcp maps that to a 0-byte return with
 *      errno = ENOTCONN (lwip-stack/src/api/sockets.c:958-962; err.c's
 *      err_to_errno() table maps ERR_CLSD -> ENOTCONN).  This function
 *      already handles that fine: n == 0, OK, 0 bytes (below).
 *   2nd+ post-FIN recv: netconn_recv_data_tcp() now sees
 *      !NETCONN_RECVMBOX_WAITABLE(conn) (the mbox is already freed) and
 *      returns ERR_CONN before ever touching the pcb (api_lib.c:712-714).
 *      sockets.c maps THAT to a genuine -1/error return, errno ENOTCONN
 *      (err_to_errno(): ERR_CONN -> ENOTCONN too, but via the error path,
 *      not the 0-byte one).  This function's EAGAIN/EWOULDBLOCK guard below
 *      only excuses EAGAIN, so ENOTCONN falls to CC3501E_HW_ERR_IO ->
 *      src/protocol_sockets.c's sock_worker_hw_err_to_resp() ->
 *      RESP_ERR_RADIO -> the host's ALP_ERR_IO, even though the stream just
 *      ended in an orderly way.
 *
 * FIX (chosen over "treat ENOTCONN like EAGAIN"): record the EOF on the
 * FIRST 0-byte STREAM recv and short-circuit every LATER recv on that fd to
 * OK/0 WITHOUT calling lwIP at all, below -- so the ENOTCONN branch above is
 * simply never reached a second time.  An errno-based alternative (map
 * ENOTCONN to "0 bytes, OK" the same as EAGAIN) is smaller but was rejected:
 * it would ALSO quietly turn a genuinely stray recv on a LISTEN handle, or
 * the 3rd+ recv after a completed RST teardown, into a false "0 bytes, OK"
 * instead of an IO error -- ENOTCONN is not unique to "I already saw this
 * fd's FIN".  The per-fd table costs MEMP_NUM_NETCONN bytes of .bss and
 * removes that ambiguity entirely.
 *
 * BLOCKER (host review of bfb5f08): "the FIRST 0-byte STREAM recv" above is
 * not the same thing as "the first recv that saw the peer close" -- a recv
 * with want == min(max_len, cap) == 0 is a LEGAL call (alp-sdk's
 * cc3501e_sock_recv() only rejects a NULL buffer when cap is nonzero, so
 * cap == 0 is always accepted) and ALSO returns n == 0 from
 * lwip_recvfrom(), but TI's
 * lwip_recv_tcp() given a requested length of 0 copies zero bytes and
 * returns 0 with errno 0 -- it never reaches the ERR_CLSD path below at
 * all, so it says NOTHING about whether the peer actually closed.  Latching
 * on n == 0 alone (gated only on SOCK_STREAM, as an earlier version of this
 * fix did) let a single want == 0 poll on a perfectly LIVE socket
 * permanently latch a FALSE EOF -- every later recv then answered OK/0
 * without ever touching lwIP again, and the host's own 3-zero completion
 * heuristic reported a truncated stream as finished: silent data loss,
 * worse than the RESP_ERR_RADIO bug this fix exists to close.  The latch
 * decision therefore also requires want > 0 -- see
 * sock_worker_recv_eof.h's sock_worker_recv_eof_should_latch(), which owns
 * this decision (and the table's bounded get/set) as a pure, host-testable
 * function; this file only supplies the real n/want/is_stream/table.
 *
 * ECONNRESET/ECONNABORTED are NOT masked by this: lwIP delivers a reset
 * through lwip_netconn_err_to_msg() posting to conn->recvmbox BEFORE the
 * mbox is torn down the FIN way (a DIFFERENT lwIP file, api_msg.c:445-469 --
 * not api_lib.c/sockets.c, cited above, which is why it is not itself
 * "above" in this comment), so a reset always surfaces as its own errno on
 * an actual lwip_recvfrom() call at least once -- this table is only ever
 * set from the n == 0, want > 0 (orderly close) arm below, never from an
 * error return.
 *
 * Indexed by the lwIP fd directly (0 .. MEMP_NUM_NETCONN-1 -- lwipopts.h,
 * visible here via lwip/sockets.h -> lwip/opt.h; LWIP_SOCKET_OFFSET is 0 on
 * this SDK so alloc_socket()'s index IS the fd), not by the host handle
 * (fd+1): fd is what lwip_close() and every lwip_* call in this file already
 * key on, and lwIP will not reuse an fd number until it is freed by
 * lwip_close() -- so clearing the bit in cc3501e_hw_sock_close() (below)
 * makes a reused fd start clean regardless of what handle number the host
 * sees it as next.  Confirmed against every OTHER lwip_close() site in this
 * file (sock_open()'s failure path, accept_pump()'s two failure paths, and
 * radio_speedtest_udp()'s bind-failure path under CC3501E_RADIO_SPEEDTEST):
 * each of those closes an fd that was allocated moments earlier by
 * lwip_socket()/lwip_accept() and never used for a recv, so it can never be
 * marked here -- cc3501e_hw_sock_close() is the only path a marked fd can
 * ever reach lwip_close() through, so clearing there is sufficient.
 *
 * CC3501E_SOCK_EOF_MAX_FD == MEMP_NUM_NETCONN is a SINGLE source of truth
 * checked on both sides of the fd space, not two independently-verified
 * numbers that could drift: this array's own bound comes from lwipopts.h
 * (confirmed 16 in this SDK's default, non-SUPPORT_{4,8}_STREAMS_CONCURRENTLY
 * branch, ti_config/lwip-port/osi/include/lwipopts.h:205-211), and lwIP's
 * OWN fd-space bound -- the prebuilt lwip.a's sockets[] array
 * (sockets_priv.h: `#define NUM_SOCKETS MEMP_NUM_NETCONN`) -- resolves
 * against that identical macro at ITS build time, since the prebuilt
 * library ships built from this same config header.  A real fd this table
 * ever sees is therefore always < MEMP_NUM_NETCONN by construction, never
 * merely by luck. */
#define CC3501E_SOCK_EOF_MAX_FD MEMP_NUM_NETCONN
static bool sock_eof[CC3501E_SOCK_EOF_MAX_FD];

int cc3501e_hw_sock_recv(uint16_t  handle,
                         uint16_t  max_len,
                         uint8_t  *buf,
                         uint16_t  cap,
                         uint16_t *recv_len_out,
                         uint8_t   from_addr[4],
                         uint16_t *from_port_out)
{
	if (recv_len_out != 0) *recv_len_out = 0u;
	if (from_addr != 0) {
		memset(from_addr, 0, 4);
	}
	if (from_port_out != 0) *from_port_out = 0u;
	if (handle == 0u || buf == 0) {
		return CC3501E_HW_ERR_INVAL;
	}
	const int fd   = (int)handle - 1; /* handle != 0 here, so fd >= 0 always */
	uint16_t  want = (max_len < cap) ? max_len : cap;

	if (sock_worker_recv_eof_check(sock_eof, CC3501E_SOCK_EOF_MAX_FD, fd)) {
		/* Sticky EOF already recorded for this fd -- see the block comment
		 * above.  Answer OK/0 without touching lwIP at all. */
		return CC3501E_HW_OK;
	}
	if (want == 0u) {
		/* BLOCKER fix (host review of bfb5f08): nothing was actually asked
		 * for, so lwIP must not be called at all -- a want == 0
		 * lwip_recvfrom() proves nothing about the peer (see the block
		 * comment above sock_eof) and must never be allowed to reach the
		 * latch check below. */
		return CC3501E_HW_OK;
	}

	struct sockaddr_in from;
	socklen_t          fromlen = sizeof(from);
	memset(&from, 0, sizeof(from));
	/* BLOCKING, bounded by the socket's SO_RCVTIMEO (CC3501E_SOCK_RCVTIMEO_MS).
	 *
	 * MSG_DONTWAIT was tried here and is WRONG on this stack -- silicon-measured
	 * 2026-08-24: with it, a 256 KiB HTTP body that the server demonstrably
	 * delivered (two `GET /speed.bin HTTP/1.0` 200 hits logged from the device's
	 * own IP) produced `NET recv -> 0 (0 B)` on EVERY call for 81 s, i.e. 0 B/s.
	 * The non-blocking path returns EWOULDBLOCK before lwIP has moved anything
	 * into the socket, so the host never drains the connection at all.  A short
	 * blocking read does return data.
	 *
	 * The reason the timeout must stay SHORT is unchanged: this runs on the
	 * worker, and worker_run_pending() holds READY LOW across the whole job, so
	 * no bridge frame of ANY opcode is served while it waits.  That is why 4000
	 * became 50 -- not why it should become zero. */
	const ssize_t n = lwip_recvfrom(fd, buf, want, 0, (struct sockaddr *)&from, &fromlen);
	if (n < 0) {
		/* SO_RCVTIMEO expiry (EAGAIN / EWOULDBLOCK) is NOT an error at the wire: it
		 * means "no data yet" -- report OK with 0 bytes so the host re-polls.  Any
		 * other errno is a real socket failure (bad fd / reset) -> IO.  The ticlang
		 * C <errno.h> defines EAGAIN but not always EWOULDBLOCK, so guard the latter
		 * (lwIP treats the two as equal on this platform). */
		if (errno == EAGAIN
#ifdef EWOULDBLOCK
		    || errno == EWOULDBLOCK
#endif
		) {
			return CC3501E_HW_OK;
		}
		return CC3501E_HW_ERR_IO;
	}
	/* n == 0 on a STREAM socket, for a real want > 0 (guaranteed here -- the
	 * want == 0 short-circuit above already returned), means the peer
	 * closed -- still OK, 0 bytes.  Latch it (STREAM only -- a 0-byte UDP
	 * recv is a legitimate empty datagram, not EOF) so the NEXT recv on
	 * this fd takes the short-circuit above instead of reaching lwIP's
	 * post-FIN ENOTCONN path.  The full n/want/is_stream decision is
	 * sock_worker_recv_eof_should_latch() (sock_worker_recv_eof.h) -- see
	 * its own header for the want == 0 BLOCKER this specific check closes,
	 * and the block comment above sock_eof for the ENOTCONN bug it was
	 * originally written to close. */
	if (sock_worker_recv_eof_should_latch((int)n, want, sock_is_stream(fd))) {
		sock_worker_recv_eof_set(sock_eof, CC3501E_SOCK_EOF_MAX_FD, fd, true);
	}
	if (recv_len_out != 0) *recv_len_out = (uint16_t)n;
	if (from.sin_family == AF_INET) {
		if (from_addr != 0) memcpy(from_addr, &from.sin_addr.s_addr, 4);
		if (from_port_out != 0) *from_port_out = lwip_ntohs(from.sin_port);
	}
	return CC3501E_HW_OK;
}

int cc3501e_hw_sock_close(uint16_t handle)
{
	cc3501e_hw_sock_prefetch(handle, false);
	/* Drop it from the accept table too, or the pump keeps calling lwip_accept()
	 * on a closed fd -- and worse, on the fd number lwIP hands to whatever
	 * socket is opened next. */
	listen_table_remove(handle);
	if (handle == 0u) {
		return CC3501E_HW_ERR_INVAL;
	}
	{
		const int fd = (int)handle - 1;
		/* Free the fd for the sticky-EOF table too (see the block comment
		 * above sock_eof) -- unconditionally, whether or not lwip_close()
		 * below reports success: a failed lwip_close() means this fd is
		 * NOT freed for reuse, so leaving the bit cleared costs nothing
		 * (any recv would just observe lwIP directly again), while leaving
		 * it SET would risk short-circuiting a future close-then-reopen of
		 * the same fd number. */
		sock_worker_recv_eof_set(sock_eof, CC3501E_SOCK_EOF_MAX_FD, fd, false);
	}
	if (lwip_close((int)handle - 1) != 0) {
		return CC3501E_HW_ERR_IO;
	}
	return CC3501E_HW_OK;
}
#else  /* !CC3501E_WIFI -- no lwIP: report NOTIMPL (-> RESP_ERR_NOT_READY) */
int cc3501e_hw_sock_open(uint8_t family, uint8_t type, uint8_t protocol, uint16_t *handle_out)
{
	(void)family;
	(void)type;
	(void)protocol;
	if (handle_out != 0) *handle_out = 0u;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_sock_connect(uint16_t handle, uint8_t family, uint16_t port, const uint8_t addr[4])
{
	(void)handle;
	(void)family;
	(void)port;
	(void)addr;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_sock_send(uint16_t       handle,
                         uint8_t        flags,
                         const uint8_t *data,
                         uint16_t       data_len,
                         uint16_t      *sent_out)
{
	(void)handle;
	(void)flags;
	(void)data;
	(void)data_len;
	if (sent_out != 0) *sent_out = 0u;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_sock_recv(uint16_t  handle,
                         uint16_t  max_len,
                         uint8_t  *buf,
                         uint16_t  cap,
                         uint16_t *recv_len_out,
                         uint8_t   from_addr[4],
                         uint16_t *from_port_out)
{
	(void)handle;
	(void)max_len;
	(void)buf;
	(void)cap;
	if (recv_len_out != 0) *recv_len_out = 0u;
	if (from_addr != 0) memset(from_addr, 0, 4);
	if (from_port_out != 0) *from_port_out = 0u;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_sock_close(uint16_t handle)
{
	(void)handle;
	return CC3501E_HW_ERR_NOTIMPL;
}

/* The three prefetch-seam functions are called UNCONDITIONALLY --
 * hal/ti/cc3501e_hw_ti.c's tick runs cc3501e_hw_sock_pump() and
 * src/protocol_sockets.c's dispatch runs cc3501e_hw_sock_recv_ring() -- so
 * without these a ti build without -WifiHostDriver / -Ble does not link at all
 * (undefined symbol cc3501e_hw_sock_pump / _prefetch / _recv_ring), defeating
 * the whole point of this #else arm.  Bodies match hal/cc3501e_hw_stub.c. */
int cc3501e_hw_sock_bind(uint16_t handle, uint8_t family, uint16_t port, const uint8_t addr[4])
{
	(void)handle;
	(void)family;
	(void)port;
	(void)addr;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_sock_listen(uint16_t handle, uint8_t backlog)
{
	(void)handle;
	(void)backlog;
	return CC3501E_HW_ERR_NOTIMPL;
}

void cc3501e_hw_sock_pump(void)
{
}

void cc3501e_hw_sock_accept_pump(void)
{
}

void cc3501e_hw_sock_prefetch(uint16_t handle, bool on)
{
	(void)handle;
	(void)on;
}

int cc3501e_hw_sock_recv_ring(uint16_t  handle,
                              uint8_t  *buf,
                              uint16_t  cap,
                              bool      replay,
                              uint16_t *out_len)
{
	(void)handle;
	(void)buf;
	(void)cap;
	(void)replay;
	if (out_len != 0) *out_len = 0u;
	return -1; /* never the prefetched handle -> caller uses the worker path */
}
#endif /* CC3501E_WIFI */
