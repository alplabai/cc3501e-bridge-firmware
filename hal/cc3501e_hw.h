/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware-abstraction shim consumed by this firmware's src/.
 * Each function maps a firmware-level operation onto the CC3501E
 * silicon via TI's SimpleLink CC33xx SDK / driverlib.
 *
 * The default implementations in hal/cc3501e_hw_stub.c keep the
 * protocol path exercisable on the host with no TI SDK (every
 * HW-touching op returns CC3501E_HW_ERR_NOTIMPL).  The real
 * implementations live under hal/ti/ (CC3501E_HAL_BACKEND=ti, linked
 * against TI's SimpleLink SDK on the bench build).
 *
 * The surface below is grouped by command family and now covers the
 * whole bridge: chip init, the idle housekeeping tick, the factory MAC
 * read and a deferred self-reset, plus the GPIO proxy, camera enables, the
 * SPI1 host passthrough, Wi-Fi, sockets, BLE, OTA, power policy and
 * diagnostics.  Each group
 * sits beside the protocol handlers that call it.
 */

#ifndef CC3501E_BRIDGE_HAL_CC3501E_HW_H
#define CC3501E_BRIDGE_HAL_CC3501E_HW_H

#include <stddef.h> /* size_t (cc3501e_hw_wifi_scan) */
#include <stdbool.h>
#include <stdint.h>

/* Return codes.  0 = success; negatives are operation-independent
 * failure classes (mirrors the gd32-bridge BRIDGE_HW_* convention). */
#define CC3501E_HW_OK          0
#define CC3501E_HW_ERR_NOTIMPL -1 /* op not wired on this backend (stub)   */
#define CC3501E_HW_ERR_IO      -2 /* peripheral / radio access failed       */
#define CC3501E_HW_ERR_INVAL   -3 /* bad argument                           */
/* Op rejected because of the subsystem's CURRENT state (e.g. NimBLE's
 * ble_gatts_mutable() ordering guard on a GATT_REGISTER attempted while
 * advertising/scanning/connected -- BLE_HS_EBUSY, host/ble_hs.h).  Distinct
 * from CC3501E_HW_ERR_IO: this is a deterministic, terminal reject, not a
 * transport/radio glitch worth retrying. */
#define CC3501E_HW_ERR_STATE -4
/* NOT produced by any HAL body -- worker.c's own worker_poll() sets this
 * (never a HAL implementation) when a completed job's result is LARGER than
 * the caller's reply capacity, so a truncating memcpy is reported as a real
 * error instead of a silent short reply (host review: this is the general
 * form of the SOCK_RECV data-loss class -- a worker body reading more than
 * the reply can ever carry, then the collect silently dropping the
 * overrun).  Every worker-routed opcode's own cap is bounded by
 * CC3501E_REPLY_DATA_MAX (protocol.h) at the source (see worker_execute()'s
 * SOCK_RECV / WIFI_SCAN_START / BLE_SCAN_START / BLE_GATT_READ cases), so
 * this should never actually fire for a well-behaved opcode -- it exists as
 * a LOUD backstop for any future one that gets its own cap wrong, mapped by
 * the three generic worker-routed helpers (protocol.c) to
 * ALP_CC3501E_RESP_ERR_NO_MEM, the same wire code already used when a
 * reply's known-minimum size does not fit. */
#define CC3501E_HW_ERR_NO_MEM -5
#define CC3501E_HW_BUSY       1 /* op accepted, runs off-ISR; caller must re-poll */

/* --------------------------------------------------------------- */
/* Lifecycle                                                         */
/* --------------------------------------------------------------- */

/* One-time chip bring-up: clocks, power, and any board-level pin setup
 * the firmware owns before a transport is started.  No-op on the stub. */
void cc3501e_hw_init(void);

/* Periodic housekeeping, run on each WFI wakeup from main().  No-op on
 * this firmware rev; reserved for watchdog kick / deferred work. */
void cc3501e_hw_tick(void);

/* #142: the SPI link self-heals (dead-handle reopen, resync-burst, arm-fail,
 * reply-stall reinit, and the quiet-armed detector) as ONE function.  Called
 * from cc3501e_hw_tick() with @p in_connect_wait = false (the idle-tick
 * path: runs the first four, evidence-based heals; never evaluates the
 * quiet-armed detector, so an ordinarily-idle host/console session can never
 * trip it), and from INSIDE cc3501e_hw_wifi_connect_sta()'s own wait points
 * with @p in_connect_wait = true (the only path that evaluates the
 * quiet-armed detector too) -- see that function's own call sites (hal/ti/
 * cc3501e_hw_ti_wifi.c) for exactly which windows and why each is safe.
 *
 * A background task calling this independently of both those call sites was
 * tried and REJECTED (host review of b3dc1e2) -- see src/link_quiet_rearm.h's
 * top comment for the full writeup of what that broke.  No-op on the stub
 * backend (no real SPI slave to heal). */
void cc3501e_hw_link_heal(bool in_connect_wait);

/* #142 item 2 (host review of dfd5280): call ONCE per cc3501e_hw_wifi_
 * connect_sta() attempt, right after that body's own role-up reinit (or at
 * the same point if no reinit ran) -- resets the quiet-armed detector's
 * per-call fire cap and its "has a real transfer landed since this attempt
 * began" arm gate.  See hal/ti/cc3501e_hw_ti.c's quiet_arm_after_xfer_count/
 * quiet_fired_this_call for what this actually resets.  No-op on the stub
 * backend and on an SDIO build (the detector itself is compiled out there,
 * item 3). */
void cc3501e_hw_link_heal_begin_connect(void);

/* Diagnostic counter: quiet-rearm heals fired by cc3501e_hw_link_heal() since
 * boot (#142).  0 on the stub backend.  See that counter's own comment in
 * hal/ti/cc3501e_hw_ti.c for why it is not (yet) on the wire. */
uint32_t cc3501e_hw_link_quiet_rearm_count(void);

/* Bring the radio up ONCE at boot (radio<->SPI coexistence fix).
 *
 * The inter-chip bridge SPI slave cannot be serviced while the CC35 runs a
 * radio op (there is no host-IRQ to defer to), so the bridge is DOWN for the
 * duration of Wlan_Start / Wlan_RoleUp -- seconds.  Doing that radio init
 * LAZILY on the first GET_MAC tears the link down mid-traffic (ping climbs a
 * few, then reqhdr_rx -> 0x00000000 + desync).  Instead the bring-up task
 * calls this ONCE, early -- after the FreeRTOS scheduler + the SPI slave poll
 * task are up but BEFORE any host command is serviced -- so the long radio
 * init happens while there is no host traffic to disrupt (the bridge-down
 * window is then harmless).  After it returns the bridge is up AND the radio
 * is up, so a later GET_MAC only does the SHORT Wlan_Get.
 *
 * Idempotent (one-time guard inside): a no-op on the second+ call and on the
 * stub / silicon-free build (no radio -> nothing to start). */
void cc3501e_hw_wifi_boot_start(void);

/* Bring up the lwIP TCP/IP core (tcpip_init) ONCE at boot.  MUST be called EARLY --
 * from main()'s bring-up task BEFORE transport_spi_init() spawns the busy-poll bridge
 * slave task and before the radio is lazy-started -- because tcpip_init waits for the
 * lwIP thread to start, which the busy-poll task would otherwise starve (and the radio
 * would otherwise have eaten the heap the tcpip stack needs).  Prerequisite for the
 * STA netif (network_stack_add_if_sta) the Wi-Fi connect path needs.  No-op on the
 * stub / silicon-free build and on the ti build without CC3501E_WIFI (no lwIP). */
void cc3501e_hw_net_init(void);

/* --------------------------------------------------------------- */
/* Meta operations                                                   */
/* --------------------------------------------------------------- */

/* Read the CC3501E's 6-byte factory MAC into @p mac (TI wire order).
 * Returns CC3501E_HW_OK on success, CC3501E_HW_ERR_NOTIMPL on the stub
 * backend (the protocol layer maps that to RESP_ERR_NOT_READY). */
int cc3501e_hw_get_mac(uint8_t mac[6]);

/* Request a soft self-reset.  The implementation DEFERS the actual
 * reboot until the in-flight reply has been clocked back to the host
 * (so the host sees the CMD_RESET ack), then resets the chip.  No-op on
 * the stub backend. */
void cc3501e_hw_request_reset(void);

/* Transport -> HAL: the in-flight reply frame has been FULLY clocked back
 * to the host.  The slave-side SPI/SDIO transport calls this from its
 * reply-complete path.  The deferred-reset latch (cc3501e_hw_request_reset)
 * only fires the reboot once this has been observed for the CMD_RESET ack,
 * so the host always sees the ack before the link goes quiet.  No-op on
 * the stub backend. */
void cc3501e_hw_notify_reply_sent(void);

/* --------------------------------------------------------------- */
/* GPIO proxy (v0.4) + camera enables                                */
/* --------------------------------------------------------------- */

/* The CC3501E fronts E1M pads IO11 / IO13 / IO15..IO21 (plus mux/wake
 * lines) and the two camera-enable LDOs, per alp-sdk's
 * metadata/e1m_modules/aen/from-cc3501e.tsv.  These shims drive the
 * proxied CC3501E GPIOs on the Alif's behalf.  @p pad is the CC3501E
 * GPIO index; @p dir / @p pull / @p edge use the alp_cc3501e_gpio_*
 * enums in <alp/protocol/cc3501e.h>.  Return CC3501E_HW_* (NOTIMPL maps
 * to RESP_ERR_NOT_READY at the protocol layer). */
int cc3501e_hw_gpio_configure(uint8_t pad, uint8_t dir, uint8_t pull);
int cc3501e_hw_gpio_write(uint8_t pad, uint8_t level);
int cc3501e_hw_gpio_read(uint8_t pad, uint8_t *level_out);
int cc3501e_hw_gpio_set_interrupt(uint8_t pad, uint8_t edge, uint8_t enabled);

/* Camera-enable LDOs (per the E1M-AEN BDE-BW35N U4 netlist): @p which 0 ->
 * CAM_EN_LDO0 = GPIO_1 (pin54), 1 -> CAM_EN_LDO1 = GPIO_0 (pin55); @p on != 0
 * asserts the enable.  Default OFF at boot. */
int cc3501e_hw_cam_enable(uint8_t which, uint8_t on);

/* --------------------------------------------------------------- */
/* SPI1 host passthrough (v0.6)                                      */
/* --------------------------------------------------------------- */

/* The E1M connector's SPI1 lands on the CC3501E, NOT on the Alif
 * (E1M-AEN-2626-R2 netlist: AG10 SPI1_SCLK -> CC35 GPIO_32, AG9
 * SPI1_MOSI -> GPIO_33, AG8 SPI1_MISO -> GPIO_34, AH9 SPI1_CS0 ->
 * GPIO_31, AH8 SPI1_CS1 -> GPIO_15).  The host therefore cannot reach a
 * device on that bus directly; it reaches it by RELAY -- the CC3501E is
 * the SPI CONTROLLER and the host supplies the bytes, over
 * CMD_SPI1_CONFIGURE / _TRANSFER / _RELEASE (0x55..0x57).
 *
 * NOT the inter-chip bridge.  That is CC35 SPI0 (SCK GPIO_27, MISO
 * GPIO_28, MOSI GPIO_29, CSN GPIO_16), configured as a SLAVE, and it is
 * the link these very commands arrive over -- re-muxing one of those
 * pads bricks it.  Pads 15/31/32/33/34 belong to SPI1 for the same
 * reason 16/27/28/29 belong to SPI0: a GPIO-proxy write to one of them
 * re-muxes SCK out from under an in-flight controller transfer, so they
 * belong in the ti backend's gpio_pad_reserved() list.
 *
 * ALL THREE ARE WORKER-ROUTED at the protocol layer, and that is not a
 * style preference: a polled 4088-byte controller transfer is ~800 us of
 * bus time at 10 MHz.  Run from the SPI0 dispatch callback it would stall
 * the slave's re-arm for that whole window -- the desync/wedge signature
 * this firmware spent months chasing.  SPI_open() can block on a power
 * domain, so CONFIGURE goes off-ISR too.  Do not call any of these from
 * dispatch context. */

/* Acquire the SPI1 controller and pin the bus parameters until the next
 * configure or release.  Idempotent: re-issuing it re-opens with the new
 * parameters.  @p mode is (CPOL << 1) | CPHA (0..3); @p bits_per_word is 8
 * (the only width this rev accepts); @p cs picks the SOFTWARE-driven select,
 * 0 = GPIO_31 (E1 AH9), 1 = GPIO_15 (E1 AH8).  The wire layer range-checks
 * all three before calling, so a backend need not re-validate them.
 *
 * Both selects are software-driven because the SPIWFF3DMA driver carries
 * exactly ONE hardware csnSel per SPI_Config entry, so one instance cannot
 * hardware-frame two selects.  Consequence for callers: CS edges are
 * scheduler-timed, not clock-edge-exact.
 *
 * @p actual_freq_hz_out (may be NULL) receives the rate the divider actually
 * produced -- the reply carries it precisely because a real clock divides and
 * the host must not assume it got what it asked for.
 *
 * CC3501E_HW_ERR_IO here means SPI_open() failed.  The wire layer maps it to
 * RESP_ERR_RADIO, which on this family means BUS-level open failure, NOT an RF
 * problem, and the host RETRIES it -- correct, because a handle not yet closed
 * can free up on its own. */
int cc3501e_hw_spi1_configure(uint32_t  freq_hz,
                              uint8_t   mode,
                              uint8_t   bits_per_word,
                              uint8_t   cs,
                              uint32_t *actual_freq_hz_out);

/* Clock one full-duplex chunk of @p len bytes.  The NULL-buffer convention is
 * TI's own SPI_Transaction convention, so the wire flags collapse into the
 * pointers and no direction enum is needed:
 *
 *   @p tx == NULL  -> NO_TX: clock @p len copies of @p tx_fill instead.
 *   @p rx == NULL  -> NO_RX: clock the transfer and discard MISO.
 *
 * Both NULL is legal (clock fill, discard the answer).  @p len == 0 with
 * @p cs_hold false is a pure CS DEASSERT, which is why this family needs no
 * separate chip-select opcode.  @p cs_hold leaves CS asserted so the next call
 * continues the SAME device transaction (command + response, page program +
 * status poll); clear it on the last chunk.
 *
 * @p len is authoritative for both buffers -- the caller owns them and has
 * already bounded @p len by ALP_CC3501E_SPI1_MAX_XFER, so there is no cap
 * argument.
 *
 * A refused or SHORT transfer returns CC3501E_HW_ERR_STATE, NOT
 * CC3501E_HW_ERR_IO, and the difference is load-bearing: ERR_IO becomes
 * RESP_ERR_RADIO -> ALP_ERR_IO, which the host's poll_by_repeat RETRIES.  A
 * local controller refusing a transfer is deterministic, so retrying just
 * re-burns the poll budget to reach the same answer and surfaces as a
 * misleading ALP_ERR_TIMEOUT.  ERR_STATE is the terminal-reject code the worker
 * path already maps to RESP_ERR_STATE.
 *
 * Returns CC3501E_HW_ERR_NOTIMPL when no instance is open (or the backend has
 * no SPI1 at all); the wire layer maps that to RESP_ERR_NOT_READY, which is the
 * contract's answer for a TRANSFER issued before a successful CONFIGURE. */
int cc3501e_hw_spi1_transfer(const uint8_t *tx,
                             uint8_t       *rx,
                             uint16_t       len,
                             uint8_t        tx_fill,
                             bool           cs_hold);

/* Deassert CS unconditionally, close the instance and free the bus.  This is
 * the escape hatch, so it MUST NOT fail on state: calling it with nothing open
 * is a no-op that still returns CC3501E_HW_OK, which is what gives a host that
 * lost track of a CS_HOLD chain a guaranteed way back to a clean bus. */
int cc3501e_hw_spi1_release(void);

/* --------------------------------------------------------------- */
/* Wi-Fi (v0.2)                                                      */
/* --------------------------------------------------------------- */

/* Route to TI's SimpleLink Wi-Fi host (sl_Wlan* / sl_NetApp*) in the ti
 * backend; the stub + the silicon-free host build report NOTIMPL (->
 * RESP_ERR_NOT_READY).  @p security: 0 = open, 1 = WPA2-PSK, 2 = WPA3-SAE
 * (matches alp_cc3501e_wifi_connect_t.security).  ssid/psk are not NUL-
 * terminated; the lengths are authoritative. */
int cc3501e_hw_wifi_scan_start(void);
int cc3501e_hw_wifi_scan_stop(void);

/* Run a Wi-Fi scan and PACK the resulting AP list into @p buf in the host's
 * wire format -- per record: bssid[6] | rssi(1) | channel(1) | security(LE16) |
 * ssid_len(1) then ssid_len SSID bytes (the cc3501e_wifi_scan parser's
 * CC3501E_SCAN_REC_HDR=11 layout; security is the raw 16-bit TI SecurityInfo so
 * the host can decode open / WPA2 / WPA3).  Records are packed until @p cap
 * would be exceeded; *out_len receives the total bytes written.  Worker-routed
 * (the scan blocks for seconds), so this runs off the SPI ISR.  Returns
 * CC3501E_HW_OK on success; the stub / silicon-free build reports
 * CC3501E_HW_ERR_NOTIMPL with *out_len = 0. */
int cc3501e_hw_wifi_scan(uint8_t *buf, size_t cap, size_t *out_len);
int cc3501e_hw_wifi_connect_sta(const uint8_t *ssid,
                                uint8_t        ssid_len,
                                const uint8_t *psk,
                                uint8_t        psk_len,
                                uint8_t        security);
int cc3501e_hw_wifi_disconnect(void);
int cc3501e_hw_wifi_ap_start(const uint8_t *ssid,
                             uint8_t        ssid_len,
                             const uint8_t *psk,
                             uint8_t        psk_len,
                             uint8_t        security);
int cc3501e_hw_wifi_ap_stop(void);
int cc3501e_hw_wifi_get_rssi(int8_t *rssi_dbm_out);
/* Report one interface's IPv4 address.  @p iface is an
 * alp_cc3501e_wifi_iface_t: STA = the DHCP lease from the joined AP, AP = the
 * module's own address on the soft-AP it runs.  The 4 octets are written in the
 * same order for both, unchanged from the pre-v9 STA-only body. */
int cc3501e_hw_wifi_get_ip(uint8_t iface, uint8_t ip_out[4]);

/* ---- drain reinit handoff (issue #106, connect flavour) -------------------- *
 * cc3501e_hw_wifi_connect_sta's SUCCESS path re-arms the SPI slave (busy() +
 * bridge_transport_spi_hw_reinit()) ITSELF, right before it publishes
 * CONNECTED, instead of waiting for src/worker.c's drain to do it after the
 * body returns -- the drain's reinit landed too late, after the host's
 * WIFI_STATUS poll had already seen CONNECTED and started clocking
 * WIFI_GET_RSSI at its dense post-connect cadence into a slave the connect's
 * own Wlan_Connect + association wait had left torn down.  The body does NOT
 * call cc3501e_bridge_ready() itself; it hands the armed outcome here
 * instead, so the DRAIN can raise READY without paying a SECOND
 * SPI_close/SPI_open for the same event.  That is what this handoff buys --
 * NOT a READY-vs-worker_reset() ordering guarantee: bridge_transport_spi_hw_
 * reinit() already raises READY itself, as a side effect, when the arm
 * succeeds (transport_hw_ti_spi.c's arm_transfer()), independently of
 * whether the body or the drain also calls cc3501e_bridge_ready() -- so
 * READY's actual state is set before either of them gets a chance to touch
 * it, not after.  See the body for the full citation trail.
 *
 * take_reinit() is a ONE-SHOT read-and-clear, not a state query: it reports
 * true (and clears the latch) only for the run whose SUCCESS exit just took
 * that reinit, so the drain can skip paying it a second time and instead
 * trust @p armed_out for whether the slave actually came up armed.  A run
 * that took a FAILURE exit (bad args, role-up fail, Wlan_Connect reject,
 * association timeout, no DHCP lease) never reaches that reinit, so THIS
 * (SUCCESS-flavour) handoff reports false for it, unchanged from before
 * #106.  That does NOT mean the drain's own reinit runs unconditionally for
 * a FAILURE exit any more, though -- see the connect-FAILURE-flavour handoff
 * below, a SEPARATE per-run signal each FAILURE exit sets on its own.  The
 * stub / silicon-free build always reports false (no body ever takes the
 * reinit there). */
bool cc3501e_hw_wifi_connect_sta_take_reinit(bool *armed_out);

/* ---- drain reinit handoff (issue #106, RSSI flavour) ----------------------- *
 * cc3501e_hw_wifi_get_rssi() has TWO possible shapes depending on whether
 * Wi-Fi was already running: if it was, this call's only radio op is the
 * short Wlan_Get itself, with no reinit anywhere in the body -- if it was
 * NOT, cc3501e_hw_wifi_lazy_start() inside this call runs Wlan_Start() and
 * ITS OWN reinit first, and THROWS AWAY that reinit's armed/not-armed result.
 * The drain may only skip its own post-op reinit in the FIRST shape, where
 * nothing else has already tried (and had its result discarded) to recover
 * the slave -- skipping in the second shape would raise READY unconditionally
 * over a genuinely unknown state (the #1133 condition).
 *
 * take_reinit_skip() is a ONE-SHOT read-and-clear: it reports true (and
 * clears the latch) for every run of cc3501e_hw_wifi_get_rssi() -- success or
 * its CC3501E_HW_ERR_IO exit alike -- and hands back @p skip_ok_out for
 * whether Wi-Fi was ALREADY started when that run began (the drain may skip)
 * or not (the drain must reinit as normal).  The stub / silicon-free build
 * always reports false (no body ever runs there), which is safe: false means
 * "not this run's call to make", so the drain falls through to its normal
 * measured reinit + arm-check. */
bool cc3501e_hw_wifi_get_rssi_take_reinit_skip(bool *skip_ok_out);

/* ---- drain reinit handoff (connect-FAILURE flavour, advisor analysis) ------ *
 * cc3501e_hw_wifi_connect_sta()'s FAILURE exits (bad args aside -- role-up
 * fail, Wlan_Connect reject, association timeout, no DHCP lease) have always
 * paid src/worker.c's drain reinit unconditionally, unlike the SUCCESS exit
 * above.  Advisor analysis (moderate confidence, NOT bench-proven -- see
 * src/wifi_connect_fail_skip.h for the full argument and its citation trail)
 * argues that is the SAME destructive-reinit-on-a-live-slave hazard #106
 * measured for RSSI and WIFI_DISCONNECT.  What the trace actually covers:
 * Wlan_Connect() is synchronous but only queues a message for the CME task
 * (it does not itself touch the bridge's DMA); the failure path's trailing
 * Wlan_Disconnect() cleanup (wifi_clear_stale_assoc()) is safe on the SAME
 * evidence as the unconditional WIFI_DISCONNECT skip (src/worker.c's
 * wifi_disconnect group, sourced against worker.c ~1192-1212) -- a
 * synchronous message-queue post, no DMA/SPI/interrupt-mask touched.  What
 * the trace does NOT cover: the ASYNCHRONOUS CME association work that
 * actually performs the 802.11 handshake after Wlan_Connect()'s message is
 * drained -- that traffic is UNTRACED, which is exactly why a slave that was
 * armed before Wlan_Connect cannot simply be assumed to still be armed after
 * a failed attempt.
 *
 * So instead of trusting an early snapshot, each failure exit calls
 * wifi_connect_fail_mark_skip() (hal/ti/cc3501e_hw_ti_wifi.c) to POLL, right
 * there, for a host frame to complete within a short window -- see
 * wifi_wait_host_frame() (src/wifi_connect_fail_skip.h) for the pure wait
 * this wraps, including why an EARLIER baseline undercounts a slave that
 * died mid-attempt.  take_fail_skip() is a ONE-SHOT read-and-clear, set by
 * that poll immediately before its own exit's wifi_conn_set(FAILED) --
 * "before wifi_clear_stale_assoc() can run" is true only for THAT exit's own
 * trailing call, not across the whole function: on a RETRIED pass, an
 * EARLIER wifi_clear_stale_assoc() (the retry branch's own mid-loop cleanup)
 * has already run by the time this poll starts, and its effects (if any) are
 * exactly what the poll is measuring forward from.  @p skip_ok_out reports
 * whether a host frame landed in that window.  A run that took the SUCCESS
 * exit instead never reaches this handoff, so this reports false for it and
 * the drain's own reinit still runs as normal.  The stub / silicon-free
 * build always reports false (no body ever takes this exit there). */
bool cc3501e_hw_wifi_connect_sta_take_fail_skip(bool *skip_ok_out);

/* ---- async-connect status latch (CMD_WIFI_STATUS) -------------------------- *
 * The connect body (cc3501e_hw_wifi_connect_sta) BLOCKS for seconds on the
 * association event, so it is worker-routed off the SPI ISR.  The host no longer
 * blocks polling it; instead the firmware mirrors the outcome into a small latch
 * that the NON-blocking CMD_WIFI_STATUS reads (no radio op, ISR-safe).
 *
 * mark_connecting() is called SYNCHRONOUSLY when a connect is submitted (from the
 * protocol handler, before the drain runs the body) so the latch reads CONNECTING
 * from submit onward -- a host status poll never sees a stale CONNECTED from a
 * previous attempt during the brief queued window.  conn_status() copies the latch
 * out: @p state = alp_cc3501e_wifi_conn_state_t, @p fail_reason =
 * alp_cc3501e_wifi_fail_t (valid on FAILED), @p rssi_dbm = STA RSSI (valid on
 * CONNECTED).  Any out pointer may be NULL.  The stub / silicon-free build keeps
 * the latch DISCONNECTED. */
void cc3501e_hw_wifi_mark_connecting(void);
int  cc3501e_hw_wifi_conn_status(uint8_t *state, uint8_t *fail_reason, int8_t *rssi_dbm);

/* The reason/status code for THIS connect attempt, for CMD_WIFI_STATUS's
 * last_reason byte -- see @ref alp_cc3501e_wifi_status_t::last_reason
 * (formerly `reserved`) in <alp/protocol/cc3501e.h> (that header's own
 * rename + byte-meaning update is a separate host-side alp-sdk PR --
 * feat/cc3501e-wifi-status-reason, not yet merged; this is the firmware
 * half only).
 *
 * Exactly: the low byte of the IEEE 802.11 reason code from a DISCONNECT, or
 * the status code from an ASSOCIATION_REJECTED / AUTHENTICATION_REJECTED,
 * recorded ONLY while THIS attempt is OPEN -- between
 * cc3501e_hw_wifi_mark_connecting() (which also clears this to 0 for the new
 * attempt) and the terminal wifi_conn_set() that freezes it.  A DISCONNECT
 * carrying WLAN_DISCONNECT_USER_INITIATED (200) is never recorded, in any
 * state -- it is a vendor placeholder, not a real 802.11 reason.  0 = none
 * recorded: the boot default, the value on the stub / silicon-free build
 * (which never sees a real WLAN event), and what a fresh attempt reads until
 * it records one of its own.
 *
 * "Recorded" here means the underlying live tag (hal/ti/cc3501e_hw_ti_wifi.c),
 * NOT this accessor's own return value.  cc3501e_hw_wifi_last_reason() returns
 * g_wifi_conn.reason, the FROZEN copy wifi_conn_set() writes ONLY at the
 * terminal transition -- a host polling CMD_WIFI_STATUS WHILE this attempt is
 * still OPEN (CONNECTING) sees whatever this byte held before THIS attempt
 * started (typically 0), not a live view of what is being recorded underneath
 * it right now.  Some non-terminal cases in wifi_event_cb() (e.g.
 * ASSOCIATION_REJECTED's comeback-IE handling) record into the live tag
 * specifically so a LATER read within the SAME attempt (the retry-eligibility
 * check) sees it -- that is an internal handoff, not a host-visible one.
 *
 * FIRST REAL CODE WINS for a DISCONNECT specifically: it is recorded only
 * while this value is still 0 for the attempt.  ASSOCIATION_REJECTED status
 * 30 (WITH the AP's comeback-time IE) is non-terminal -- the vendor driver
 * retries the association itself -- and a retry sequence that ultimately
 * fails ends with a DISCONNECT carrying a generic, self-inflicted reason
 * (802.11 reason 3, WLAN_REASON_DEAUTH_LEAVING, from the supplicant's own
 * give-up path) that would otherwise overwrite the earlier, more specific
 * rejection status with a less informative one.  ASSOCIATION_REJECTED /
 * AUTHENTICATION_REJECTED are NOT given this same guard: each one is itself
 * a real, specific status worth recording, even a later one differing from
 * an earlier one in the same attempt, unlike a terminal DISCONNECT's generic
 * closing reason.
 *
 * This is an OBSERVABILITY byte: it is scoped to "was an attempt open when
 * this arrived", not to "did WE cause it".  A disconnect this firmware itself
 * issues while actually connected or mid-association (a host WIFI_DISCONNECT
 * while connected, or the #1437 stale-association cleanup after a failed
 * connect) is excluded not because either is specially flagged, but because
 * the #1437 cleanup genuinely cannot run while an attempt is open -- it
 * always runs AFTER its caller's own wifi_conn_set(FAILED, ...), which is
 * itself the terminal transition that closes the gate.  A host WIFI_DISCONNECT
 * is NOT similarly guaranteed to arrive only from CONNECTED -- nothing in
 * protocol_wifi.c:87-104 / worker.c:307-314 enforces that a host sends one
 * only then -- but cc3501e_hw_wifi_disconnect() does not depend on the
 * assumption either: it passes g_wifi_conn.reason (the frozen value from
 * whichever state actually preceded it) as its own `reason`, not a live
 * re-read, so it republishes correctly regardless of which state it is
 * actually called from (see that function's own comment).  The converse gap
 * also exists and is NOT recorded: a reject event
 * that arrives after the attempt has already been declared a TIMEOUT is
 * lost -- the attempt is already terminal (state is no longer CONNECTING)
 * by the time that late event shows up, so the gate is already closed
 * against it too.
 *
 * RESIDUAL (read this before trusting an exact match): the underlying
 * wifi_last_reason_tag (hal/ti/cc3501e_hw_ti_wifi.c) is cleared twice for a
 * new attempt -- once by cc3501e_hw_wifi_mark_connecting() at submit, and
 * again by cc3501e_hw_wifi_connect_sta() immediately before its own
 * Wlan_Connect -- but neither reset is the exact instant the vendor begins
 * processing that new connect.  A THIRD such reset happens for the SAME
 * attempt if cc3501e_hw_wifi_connect_sta()'s RUN9 bounded retry fires
 * (reason 30, see that function's own comment): the retry re-issues
 * Wlan_Connect once, and clears this value again immediately before doing
 * so, so a caller observing this byte mid-attempt cannot tell a first try
 * from a retried one -- only the FINAL value, once the attempt reaches a
 * terminal state or CONNECTED, is meaningful.  That FINAL value is NOT
 * simply whatever the retry pass's own events happen to produce: if the
 * retry pass itself ends in reason 3 (WLAN_REASON_DEAUTH_LEAVING -- USUALLY
 * either our own pre-retry cleanup's Wlan_Disconnect() echoing back, or
 * hostap's SME give-up timers reaching sme_deauth(), sme.c:2325-2345) or
 * reason 0 (nothing recorded on the retry pass at all),
 * cc3501e_hw_wifi_connect_sta() restores the FIRST pass's real, AP-issued
 * reason (the 30 that made this eligible for a retry in the first place)
 * instead of publishing that retry-pass outcome -- see
 * wifi_retry_should_restore_first_pass() (src/wifi_retry.h) for the
 * mechanism.  Any OTHER retry-pass reason (a real, different AP reject code)
 * still publishes as the retry's own outcome.
 *
 * THAT RESTORE ITSELF IS A HEURISTIC, NOT A PROOF, and can misfire: an AP's
 * OWN deauth or disassoc frame can legitimately carry ReasonCode 3 too
 * (drv_ti_mlme.c:1476/1521 copy the frame's own reason_code verbatim), and
 * a retry pass that ends in a genuine reason-3 AP deauth -- or one whose
 * own passphrase is wrong, which a first-pass AUTH-level 30 never actually
 * checked, and which happens to surface via a reason-3 DISCONNECT rather
 * than a fresh AUTHENTICATION_REJECTED -- would be wrongly relabeled as the
 * first pass's stale 30.  See wifi_retry.h's own RESIDUAL note for the full
 * discussion; this is accepted as the better default, not eliminated.
 *
 * A late event from the PREVIOUS attempt (still in flight on the
 * host-driver thread) that lands in the tiny window between that second
 * reset and the vendor actually processing the new connect can still be
 * recorded against the new one.  Not limited to a disconnect-then-connect's
 * reason 3 (WLAN_REASON_DEAUTH_LEAVING) -- any late event the prior attempt
 * produces, including a supplicant DISCONNECT with a different real reason
 * after an AUTHENTICATION_REJECTED, or a late ASSOCIATION_REJECTED /
 * AUTHENTICATION_REJECTED after a TIMEOUT, can land there too. A reader
 * should treat an unexpected value as POSSIBLY belonging to the prior
 * attempt, not necessarily the current one.
 *
 * SCOPE: covers the CONNECT ATTEMPT only -- the reason or status that ENDED
 * or REJECTED that attempt.  CONNECTED ALWAYS publishes 0, unconditionally --
 * even if a since-succeeded retry left a transient rejection status (e.g. 30)
 * recorded during the attempt: a CONNECTED attempt was neither ended nor
 * rejected, so this byte must not carry a stale reject alongside it.
 * Reaching CONNECTED also clears the underlying live tag, not only the
 * published g_wifi_conn.reason -- both read 0 from that point on.  A LATER
 * publish (a host-requested WIFI_DISCONNECT ending a clean, connected
 * session) passes g_wifi_conn.reason itself as its own reason, not the live
 * tag (see cc3501e_hw_wifi_disconnect()'s own comment, hal/ti/
 * cc3501e_hw_ti_wifi.c) -- it reads 0 because CONNECTED already froze that
 * field to 0, not because the live tag happens to still be clean at the
 * moment WIFI_DISCONNECT runs.  Either way it must read 0, not resurface an
 * old rejection from earlier in the same attempt.  Once an attempt
 * reaches CONNECTED this value is frozen at 0; a deauth that arrives AFTER a
 * successful CONNECTED does not update it (there is no post-connect tracking
 * here by design -- see the fuller note on g_wifi_conn's `reason`
 * field in hal/ti/cc3501e_hw_ti_wifi.c). */
int16_t cc3501e_hw_wifi_last_reason(void);

/* --------------------------------------------------------------- */
/* TCP/UDP sockets (v0.5)                                            */
/* --------------------------------------------------------------- */

/* Route to the firmware IP stack's BSD socket API (lwIP sockets: lwip_socket /
 * lwip_connect / lwip_send / lwip_recvfrom / lwip_close in the ti backend); the
 * stub + the silicon-free host build report NOTIMPL (-> RESP_ERR_NOT_READY).
 *
 * These bodies BLOCK (a socket op is a tcpip_apimsg round-trip to the lwIP core
 * thread; connect/recv can wait for the network), so the protocol handlers
 * WORKER-ROUTE all five off the SPI ISR -- exactly like the blocking Wlan_* ops.
 *
 * v1 IP-stack surface is IPv4-only: @p family is an alp_cc3501e_sock_family_t
 * (only IPV4 accepted), @p type an alp_cc3501e_sock_type_t (STREAM=TCP,
 * DGRAM=UDP), @p protocol the IP protocol number (0 = default for the type).
 * Addresses are 4 raw IPv4 octets in network (big-endian) order; @p port is
 * host byte order (the backend converts to network order on the wire). */

/* Allocate a socket; returns the firmware-side handle (non-zero; 0 is the
 * invalid handle) in @p handle_out. */
int cc3501e_hw_sock_open(uint8_t family, uint8_t type, uint8_t protocol, uint16_t *handle_out);

/* STREAM: start + complete the TCP handshake to @p addr:@p port.  DGRAM: set the
 * default peer for later sends.  Blocks until the handshake resolves. */
int cc3501e_hw_sock_connect(uint16_t handle, uint8_t family, uint16_t port, const uint8_t addr[4]);

/* Assign the socket's LOCAL endpoint (the serving side of connect).  An all-zero
 * @p addr is INADDR_ANY -- bind every interface, which is what a server on the
 * soft-AP wants since the AP address only exists once the role is up. */
int cc3501e_hw_sock_bind(uint16_t handle, uint8_t family, uint16_t port, const uint8_t addr[4]);

/* Make a bound STREAM socket passive, queueing at most @p backlog pending
 * connections (0 = the backend's default).  Does NOT wait for a connection:
 * cc3501e_hw_sock_accept_pump() picks each one up on the housekeeping tick. */
int cc3501e_hw_sock_listen(uint16_t handle, uint8_t backlog);

/* TASK CONTEXT ONLY -- called from cc3501e_hw_tick(), alongside
 * cc3501e_hw_sock_pump().  NON-BLOCKING accept on every listening socket: each
 * connection it takes is pushed onto the event ring as EVT_SOCK_ACCEPTED with
 * an alp_cc3501e_sock_accepted_evt_t payload, and the host owns the new handle
 * from that point (nothing here ever closes it).
 *
 * WHY A PUMP AND NOT AN OPCODE: accept() blocks, and a worker-routed blocking
 * body holds READY LOW for its whole duration (worker_run_pending brackets the
 * drain with cc3501e_bridge_busy/ready) -- so an accept opcode would black the
 * entire bridge out for as long as no client connects.  Same task-side shape as
 * cc3501e_hw_sock_pump(), for the same reason. */
void cc3501e_hw_sock_accept_pump(void);

/* Queue @p data_len bytes on the socket; @p sent_out receives the byte count the
 * stack accepted.  @p flags mirrors alp_cc3501e_sock_send_t::flags (bit 0 = MORE). */
int cc3501e_hw_sock_send(uint16_t       handle,
                         uint8_t        flags,
                         const uint8_t *data,
                         uint16_t       data_len,
                         uint16_t      *sent_out);

/* Receive up to min(@p max_len, @p cap) bytes into @p buf; @p recv_len_out gets
 * the byte count (0 = nothing available within the socket's receive timeout, or
 * peer closed -- still CC3501E_HW_OK, non-blocking semantics at the wire).  For
 * DGRAM sockets @p from_addr / @p from_port_out receive the datagram source
 * (zeroed for STREAM).  Any out pointer other than @p buf may be NULL. */
int cc3501e_hw_sock_recv(uint16_t  handle,
                         uint16_t  max_len,
                         uint8_t  *buf,
                         uint16_t  cap,
                         uint16_t *recv_len_out,
                         uint8_t   from_addr[4],
                         uint16_t *from_port_out);

/* Release the socket (STREAM: issue the TCP teardown).  The handle is invalid
 * afterwards and the firmware may reuse its value. */
int cc3501e_hw_sock_close(uint16_t handle);

/* ---- socket RX prefetch (bulk receive) ----------------------------------
 * CMD_SOCK_RECV is served from a ring the TASK fills, so the dispatch (which
 * runs in the SPI callback and cannot call lwIP) can answer synchronously --
 * one bridge transaction per frame instead of a submit/collect pair. */
void cc3501e_hw_sock_pump(void); /* task ctx: does the lwIP read */
void cc3501e_hw_sock_prefetch(uint16_t handle, bool on);
/* dispatch ctx: memcpy only, never lwIP.  Three distinct answers:
 *
 *   >= 0  bytes taken from the ring.
 *   -1    NOT the prefetched handle.  The worker path is the only reader of that
 *         fd, so the caller SHOULD fall through to it.
 *   -2    armed for this handle but the ring is momentarily empty.  The caller
 *         MUST NOT fall through: answer BUSY and let the host re-poll.
 *
 * EXCLUSIVITY RULE (#7): exactly one code path may call lwip_* on a prefetched
 * fd, and for an armed handle that path is cc3501e_hw_sock_pump().  Falling
 * through to the worker's cc3501e_hw_sock_recv() on an armed-but-empty ring made
 * two readers race the same socket and silently dropped a chunk of the stream.
 *
 * @p replay -- LAZY-COMMIT (a CRC-rejected reply's bytes must survive a
 * retry, see src/sock_recv_commit.h): true when the caller has determined
 * THIS request is a byte-identical re-issue of the immediately preceding
 * one (same header seq, same handle -- protocol_sockets.c's
 * handle_sock_recv()), i.e. the host never collected the last reply and is
 * asking again rather than moving on.  On a replay this re-serves the SAME
 * bytes (or more, if new data arrived) instead of advancing past bytes the
 * host may never have received; on a non-replay it first retires the
 * previous call's served bytes, then serves the next unconsumed chunk. */
int cc3501e_hw_sock_recv_ring(uint16_t  handle,
                              uint8_t  *buf,
                              uint16_t  cap,
                              bool      replay,
                              uint16_t *out_len);

/* --------------------------------------------------------------- */
/* BLE 5.4 (v0.3)                                                    */
/* --------------------------------------------------------------- */

/* Route to TI's BLE host (NimBLE, source/ti/net/ble_interface) in the ti
 * backend; the stub + the silicon-free host build report NOTIMPL.  adv/scan
 * intervals are in ms; addr is a 6-byte BLE device address; GATT ops carry
 * a 16-bit attribute handle. */
int cc3501e_hw_ble_enable(void);
int cc3501e_hw_ble_disable(void);
int cc3501e_hw_ble_adv_start(uint8_t        connectable,
                             uint16_t       interval_min_ms,
                             uint16_t       interval_max_ms,
                             const uint8_t *adv_data,
                             uint8_t        adv_data_len);
int cc3501e_hw_ble_adv_stop(void);
int cc3501e_hw_ble_scan_start(void);
int cc3501e_hw_ble_scan_stop(void);
/* Worker-routed, record-returning BLE scan (the BLE mirror of
 * cc3501e_hw_wifi_scan): PACK discovered advertisers into @p buf -- per record
 * addr[6] | addr_type(1) | rssi(1) | name_len(1) then name_len name bytes.
 * Requires the NimBLE host up (else NOTIMPL -> NOT_READY). */
int cc3501e_hw_ble_scan(uint8_t *buf, size_t cap, size_t *out_len);
int cc3501e_hw_ble_connect(uint8_t addr_type, const uint8_t addr[6]);
int cc3501e_hw_ble_disconnect(void);
/* Register a dynamic GATT service from a wire-format descriptor (see
 * ALP_CC3501E_CMD_BLE_GATT_REGISTER in <alp/protocol/cc3501e.h> for the exact
 * byte layout).  @p handles_out receives one attribute VALUE handle per
 * characteristic, in descriptor order, capped at @p handles_cap; @p
 * num_handles_out is always set (0 on any error). */
int cc3501e_hw_ble_gatt_register(const uint8_t *desc,
                                 uint16_t       desc_len,
                                 uint16_t      *handles_out,
                                 uint16_t       handles_cap,
                                 uint16_t      *num_handles_out);
int cc3501e_hw_ble_gatt_notify(uint16_t handle, const uint8_t *data, uint16_t data_len);
int cc3501e_hw_ble_gatt_read(uint16_t handle, uint8_t *out, uint16_t cap, uint16_t *out_len);
int cc3501e_hw_ble_gatt_write(uint16_t handle, const uint8_t *data, uint16_t data_len);

/* --------------------------------------------------------------- */
/* OTA firmware update (over-the-bridge PSA-FWU streaming)            */
/* --------------------------------------------------------------- */

/* Stream a new signed GPE vendor image into the CC3501E's NON-primary
 * vendor slot via PSA-FWU, then install + reboot so the cold BL2/MCUboot
 * swaps it to primary.  The Alif host drives the 0x40..0x44 OTA opcodes;
 * these shims carry the session.  ONE session at a time; bytes arrive
 * SEQUENTIALLY (offset == the running cursor).  Return CC3501E_HW_*
 * (NOTIMPL -> the stub / silicon-free build maps it to RESP_ERR_NOT_READY).
 * The real bodies live in the ti backend (psa_fwu_*). */

/* Open a session: pick the non-primary vendor slot, bring it READY, latch
 * @p total_len (full image size, must exceed the manifest). */
int cc3501e_hw_ota_begin(uint32_t total_len);

/* Accept a sequential image chunk at absolute @p offset (must equal the
 * running write cursor).  The first TI_FWU_MANIFEST_SIZE bytes are buffered
 * for psa_fwu_start; the remainder is psa_fwu_write()n into the slot. */
int cc3501e_hw_ota_write(uint32_t offset, const uint8_t *data, uint32_t len);

/* Finalize: psa_fwu_finish + psa_fwu_install (CANDIDATE -> STAGED), then arm
 * a DEFERRED reboot (cc3501e_hw_tick performs it once the FINISH ack has
 * clocked back, like CMD_RESET) so BL2 swaps the slot on the next boot.
 * Errors if the stream is incomplete. */
int cc3501e_hw_ota_finish(void);

/* Cancel an in-flight session (psa_fwu_cancel) and return to IDLE. */
int cc3501e_hw_ota_abort(void);

/* Promote an ALREADY-committed pending image: arm the same deferred swap-reboot
 * that FINISH uses, without needing a fresh session.  A STAGED image survives a
 * bare nRESET (which carries no swap request) while the RAM session state resets
 * to IDLE -- so once a slot is occupied, FINISH short-circuits and the swap can
 * never be requested.  This is the unjam/promote path: cc3501e_hw_tick performs
 * the reboot once the reply has clocked back.  Not gated on ota.state (that is
 * IDLE after the reset that jammed the slot). */
int cc3501e_hw_ota_promote(void);

/* Result of the last psa_fwu_request_reboot() (0 if none requested).  Since
 * request_reboot only RETURNS on refusal (success reboots), a non-zero value
 * means the swap was REFUSED (e.g. BL2 anti-rollback on a downgrade) -- lets the
 * host distinguish "refused" from "never fired".  Surfaced in OTA_STATUS. */
int8_t cc3501e_hw_ota_reboot_rc(void);

/* Flash-derived pending-image state of the slot an update targets, as an
 * alp_cc3501e_ota_pending_t.  Read from the image store (psa_fwu_query on the
 * NON-PRIMARY vendor slot), NOT from the RAM session -- so it stays correct
 * across a reset that cleared the session, which is exactly the case where the
 * RAM state reads IDLE while a fully staged image is still sitting in flash.
 *
 * This is what makes PROMOTE confirmable and an abandoned image visible, now
 * that FINISH no longer arms a swap on its own (#1123).  Returns
 * ALP_CC3501E_OTA_PENDING_UNKNOWN if the store cannot be queried or the primary
 * slot is ambiguous -- never NONE, because "cannot tell" must not read as
 * "nothing pending". */
uint8_t cc3501e_hw_ota_pending(void);

/* True while an OTA window flush is queued or running (#1610).  Published in
 * OTA_STATUS.reserved[1] so the host can HOLD OFF payload-bearing WRITE frames
 * and poll header-only across the flash blackout, instead of inferring a stall
 * from BUSY (which cannot distinguish a flush from any other in-flight op).
 * Backends that never flash mid-stream return false. */
bool cc3501e_hw_ota_flush_pending(void);

/* Bench triage (#1610): which psa_fwu_* call failed the last OTA flush and its
 * psa_status_t low byte.  Exists because the CC3501E has no UART on the debug
 * probe, so a failed flush can only report itself over the bridge.  Both zero
 * when nothing has failed since the last BEGIN/ABORT. */
void cc3501e_hw_ota_fault(uint8_t *stage, uint8_t *psa_lo);

/* Report session progress: @p state = alp_cc3501e_ota_state_t, @p
 * bytes_written = bytes accepted so far, @p total_len = the BEGIN value.
 * Any out pointer may be NULL. */
int cc3501e_hw_ota_status(uint8_t *state, uint32_t *bytes_written, uint32_t *total_len);

/* --------------------------------------------------------------- */
/* Power policy + diagnostics (configurability)                      */
/* --------------------------------------------------------------- */

/* Apply the host's power-policy hint (alp_cc3501e_power_policy_t fields:
 * coarse policy preset, wake-event bitmap, idle-before-sleep ms).  These
 * are firmware-side config knobs (no radio needed) -- the backend applies
 * what it can and returns OK. */
int cc3501e_hw_set_power_policy(uint8_t policy, uint8_t wake_events, uint32_t idle_ms_before_sleep);

/* Whether the LAST realised radio power-save apply succeeded AND was honoured
 * verbatim.
 *
 * cc3501e_hw_set_power_policy() runs in SPI-DISPATCH (ISR) context, where the
 * vendor radio call it needs is illegal, so the radio half is deferred to the
 * task.  Its RESP_OK therefore means QUEUED, not APPLIED -- the same semantic
 * OTA_BEGIN has.  This reports the outcome of the previous apply, so a host can
 * tell "the policy was accepted" from "the radio actually took it".
 *
 * ALSO false while an AP role is up and the requested policy was BALANCED /
 * LOW_POWER / DEEP_SLEEP: device-wide power management cannot be put to sleep
 * while a soft-AP is beaconing (#1562), so the HAL forces ALWAYS_ACTIVE
 * instead and reports that substitution here, even though every underlying
 * radio call succeeded.  A host polling this after such a policy sees false
 * and should read it as "not what you asked for", not as a wire failure --
 * the POWER_POLICY call itself still returned RESP_OK.
 *
 * Backends with no radio return true. */
bool cc3501e_hw_power_radio_ok(void);

/* Set firmware log verbosity (0 = off).  OK means ACCEPTED AND RECORDED, not
 * APPLIED: the ti backend stores the level but no log sink consumes it yet
 * (the UARTs are idle and diagnostics ride the bridge).  Same semantic as
 * cc3501e_hw_power_policy()'s deferred radio half. */
int cc3501e_hw_set_log_level(uint8_t level);

/* Diagnostics sources for GET_DIAG_INFO (best-effort; 0 / UNKNOWN when the
 * backend has no source yet).  reset_cause is an alp_cc3501e_reset_cause_t. */
uint8_t  cc3501e_hw_reset_cause(void);
uint32_t cc3501e_hw_uptime_ms(void);

/* Take the boot baseline cc3501e_hw_uptime_ms() subtracts (#111).  Called once
 * from cc3501e_hw_init().  Needed because the DPL clock behind the uptime read
 * SURVIVES a warm NVIC_SystemReset() on this silicon, so without a per-boot
 * baseline `uptime` counts from power-on and a host cannot tell the companion
 * rebooted underneath it. */
void     cc3501e_hw_uptime_mark_boot(void);
uint32_t cc3501e_hw_free_heap_bytes(void);

/* WI-FI role currently up, as an alp_cc3501e_role_t: ROLE_OFF, ROLE_WIFI_STA or
 * ROLE_WIFI_AP.  GET_DIAG_INFO used to hardcode ROLE_OFF, which made the one
 * field that could answer "is the soft-AP role still up?" useless -- the exact
 * question alp-sdk#1562 is stuck on, where the AP starts, advertises and then
 * stops with ap_start long since returned.  Reading it is non-disturbing: it
 * reports the backend's own bookkeeping and issues no radio call.
 *
 * SCOPE: Wi-Fi only.  BLE state is NOT folded in, so ROLE_BLE_PERIPHERAL /
 * ROLE_BLE_CENTRAL / ROLE_DUAL_WIFI_BLE are never returned even with BLE up --
 * the NimBLE accessor sits behind CC3501E_BLE in another TU and the Wi-Fi-only
 * build must keep compiling.  A host MUST NOT read ROLE_OFF here as "BLE is
 * down". Widening this means giving the backends a shared role latch. */
uint8_t cc3501e_hw_radio_role(void);

/* ID of the last Wi-Fi event the backend's event callback saw, of any type;
 * 0 means no WLAN event has fired since reset.  Surfaced via GET_DIAG_INFO's
 * reserved[0].  Bench value (#1562): an ap_start that leaves this at 0 never
 * got a WLAN event at all -- a far sharper signal than "the SSID did not appear
 * on another radio", and it needs no second radio.  Declared here rather than
 * hand-externed at its call site, which is how it spent a release impersonating
 * the free_heap_bytes field. */
uint32_t cc3501e_hw_wifi_last_event_id(void);

/* lwIP's own view of the STA DHCP client, for GET_DIAG_INFO's two APPENDED
 * bytes (16 and 17).  Nothing the host could previously read distinguishes
 * "dhcp_start() never ran" from "DISCOVERs are going out and nothing answers",
 * and on this SoM there is no console to watch it from -- the CC35 UART2 pins
 * and the radio tracer pin are both unrouted, measured 2026-08-29.
 *
 * @param state_out  lwIP dhcp->state PLUS ONE, so that 0 can mean "not
 *                   reported" -- either this firmware has no lwIP linked, or the
 *                   netif has no dhcp struct because DHCP was never started.
 *                   1 therefore means DHCP_STATE_OFF, 7 means SELECTING and 11
 *                   means BOUND.
 * @param flags_out  bit0 netif UP, bit1 netif LINK_UP, bits 2..7 dhcp->tries
 *                   saturated at 63.  tries is what separates "one DISCOVER went
 *                   out" from "six went out and none were answered".
 *
 * Both are best-effort diagnostics: a torn read costs one misleading diagnostic
 * byte and nothing else, which is why this does NOT take LOCK_TCPIP_CORE -- the
 * diag opcode must stay a light, non-blocking read. */
void cc3501e_hw_wifi_dhcp_diag(uint8_t *state_out, uint8_t *flags_out);

#endif /* CC3501E_BRIDGE_HAL_CC3501E_HW_H */
