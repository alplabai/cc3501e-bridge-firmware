/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Default HAL implementation: hardware-free.  Lifecycle hooks are
 * no-ops and HW-touching ops return CC3501E_HW_ERR_NOTIMPL, so the
 * protocol round-trip (PING / GET_VERSION) is exercisable on the host
 * with no TI SimpleLink SDK on the workspace.
 *
 * The real implementation against TI's SimpleLink CC33xx SDK lives
 * under hal/ti/; the build picks one or the other via
 * CC3501E_HAL_BACKEND in CMakeLists.txt.
 */

#include <stdint.h>
#include <string.h> /* memcpy/memset: the SPI1 loopback moves up to 4088 bytes */

#include "alp/protocol/cc3501e.h"
#include "cc3501e_hw.h"

void cc3501e_hw_init(void)
{
	/* no-op on the stub backend */
}

void cc3501e_hw_tick(void)
{
	/* no-op on the stub backend */
}

void cc3501e_hw_wifi_boot_start(void)
{
	/* No radio on the host stub -- nothing to bring up at boot. */
}

void cc3501e_hw_net_init(void)
{
	/* No lwIP on the host stub -- nothing to bring up. */
}

int cc3501e_hw_get_mac(uint8_t mac[6])
{
	/* No radio on the host stub -- zero the buffer and report NOTIMPL so
     * the protocol layer answers RESP_ERR_NOT_READY rather than handing
     * back a fabricated MAC. */
	if (mac != 0) {
		for (unsigned i = 0u; i < 6u; ++i)
			mac[i] = 0u;
	}
	return CC3501E_HW_ERR_NOTIMPL;
}

void cc3501e_hw_request_reset(void)
{
	/* no-op on the stub backend */
}

void cc3501e_hw_notify_reply_sent(void)
{
	/* no-op on the stub backend */
}

/* --------------------------------------------------------------- */
/* GPIO proxy (v0.4) -- in-memory simulation so the host protocol    */
/* path (configure/write/read/IRQ-arm + camera enables) is fully     */
/* exercisable on the host with no silicon.  Real pad I/O is in       */
/* hal/ti/.                                                          */
/* --------------------------------------------------------------- */
/* The CC3501E exposes GPIO0..GPIO37, so this 32 is NOT the device's pad count --
 * it is a stub-side array bound.  A host proxying a pad >= 32 against the stub
 * gets ERR_INVAL where real silicon would accept it.  Harmless for the
 * silicon-free build's purpose (wire-contract coverage), but do not read this
 * number as a hardware fact.  Issue #20. */
#define STUB_GPIO_MAX 32u
static uint8_t stub_gpio_level[STUB_GPIO_MAX];
static uint8_t stub_cam[2];

int cc3501e_hw_gpio_configure(uint8_t pad, uint8_t dir, uint8_t pull)
{
	(void)dir;
	(void)pull;
	if (pad >= STUB_GPIO_MAX) return CC3501E_HW_ERR_INVAL;
	return CC3501E_HW_OK;
}

int cc3501e_hw_gpio_write(uint8_t pad, uint8_t level)
{
	if (pad >= STUB_GPIO_MAX) return CC3501E_HW_ERR_INVAL;
	stub_gpio_level[pad] = level ? 1u : 0u;
	return CC3501E_HW_OK;
}

int cc3501e_hw_gpio_read(uint8_t pad, uint8_t *level_out)
{
	if (pad >= STUB_GPIO_MAX || level_out == 0) return CC3501E_HW_ERR_INVAL;
	*level_out = stub_gpio_level[pad];
	return CC3501E_HW_OK;
}

int cc3501e_hw_gpio_set_interrupt(uint8_t pad, uint8_t edge, uint8_t enabled)
{
	(void)edge;
	(void)enabled;
	if (pad >= STUB_GPIO_MAX) return CC3501E_HW_ERR_INVAL;
	return CC3501E_HW_OK;
}

int cc3501e_hw_cam_enable(uint8_t which, uint8_t on)
{
	if (which > 1u) return CC3501E_HW_ERR_INVAL;
	stub_cam[which] = on ? 1u : 0u;
	return CC3501E_HW_OK;
}

/* --------------------------------------------------------------- */
/* SPI1 host passthrough (v0.6) -- LOOPBACK fake.                    */
/*                                                                   */
/* Unlike Wi-Fi / BLE / OTA above, this family does NOT report        */
/* NOTIMPL here, and the reason is CI coverage rather than ambition:  */
/* CI builds this backend and never runs SysConfig, so the stub is    */
/* the ONLY SPI1 body CI ever links.  A NOTIMPL stub would make every */
/* test assert RESP_ERR_NOT_READY and prove nothing about the one     */
/* half of this family CI can actually check -- the inline-TX /       */
/* self-delimiting-RX framing.  So the stub behaves like a trivial    */
/* real device: a wire loop with MOSI tied to MISO.  A test that      */
/* clocks bytes out gets those same bytes back, and dropping the      */
/* payload on the floor fails instead of passing vacuously.           */
/*                                                                   */
/* What it does NOT model, so nobody reads a green CI run as bench    */
/* evidence: no clock divider, no CS timing, no bus errors.           */
/* --------------------------------------------------------------- */
static bool stub_spi1_open;

int cc3501e_hw_spi1_configure(uint32_t  freq_hz,
                              uint8_t   mode,
                              uint8_t   bits_per_word,
                              uint8_t   cs,
                              uint32_t *actual_freq_hz_out)
{
	(void)mode;
	(void)bits_per_word;
	(void)cs;
	/* No clock tree here, so the request IS the actual rate.  Do NOT read a
	 * matching value back as proof the host copes with a divided rate -- only
	 * silicon exercises that path, and inventing a plausible-looking divider
	 * ratio here would be a fabricated hardware fact. */
	if (actual_freq_hz_out != 0) *actual_freq_hz_out = freq_hz;
	stub_spi1_open = true;
	return CC3501E_HW_OK;
}

int cc3501e_hw_spi1_transfer(const uint8_t *tx,
                             uint8_t       *rx,
                             uint16_t       len,
                             uint8_t        tx_fill,
                             bool           cs_hold)
{
	(void)cs_hold; /* the loop has no chip select to hold */

	/* Enforced HERE and not only at the wire layer: this is the state the
	 * BACKEND owns (there is no open instance), so a TRANSFER that skipped
	 * CONFIGURE answers RESP_ERR_NOT_READY even if a handler forgets to
	 * check.  NOTIMPL is the only HAL code hw_to_resp() maps to NOT_READY. */
	if (!stub_spi1_open) return CC3501E_HW_ERR_NOTIMPL;

	if (rx != 0 && len != 0u) {
		if (tx != 0)
			memcpy(rx, tx, len); /* MOSI looped to MISO */
		else
			memset(rx, tx_fill, len); /* NO_TX: the fill byte is what went out */
	}
	return CC3501E_HW_OK;
}

int cc3501e_hw_spi1_release(void)
{
	/* Never fails on state -- it is the host's escape hatch out of a lost
	 * CS_HOLD chain, so releasing nothing is a successful release. */
	stub_spi1_open = false;
	return CC3501E_HW_OK;
}

/* --------------------------------------------------------------- */
/* Wi-Fi (v0.2) -- no radio on the host stub: report NOTIMPL so the  */
/* protocol path stays exercisable (the handlers parse + validate,    */
/* then map NOTIMPL -> RESP_ERR_NOT_READY).                           */
/* --------------------------------------------------------------- */
int cc3501e_hw_wifi_scan_start(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_wifi_scan_stop(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_wifi_scan(uint8_t *buf, size_t cap, size_t *out_len)
{
	(void)buf;
	(void)cap;
	if (out_len != 0) *out_len = 0u;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_wifi_connect_sta(const uint8_t *ssid,
                                uint8_t        ssid_len,
                                const uint8_t *psk,
                                uint8_t        psk_len,
                                uint8_t        security)
{
	(void)ssid;
	(void)ssid_len;
	(void)psk;
	(void)psk_len;
	(void)security;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_wifi_disconnect(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_wifi_ap_start(const uint8_t *ssid,
                             uint8_t        ssid_len,
                             const uint8_t *psk,
                             uint8_t        psk_len,
                             uint8_t        security)
{
	(void)ssid;
	(void)ssid_len;
	(void)psk;
	(void)psk_len;
	(void)security;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_wifi_ap_stop(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_wifi_get_rssi(int8_t *rssi_dbm_out)
{
	if (rssi_dbm_out != 0) *rssi_dbm_out = 0;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_wifi_get_ip(uint8_t iface, uint8_t ip_out[4])
{
	(void)iface;
	(void)ip_out;
	return CC3501E_HW_ERR_NOTIMPL;
}

void cc3501e_hw_wifi_mark_connecting(void)
{
	/* No radio on the host stub -- the connect-status latch stays DISCONNECTED. */
}

int cc3501e_hw_wifi_conn_status(uint8_t *state, uint8_t *fail_reason, int8_t *rssi_dbm)
{
	if (state != 0) *state = (uint8_t)ALP_CC3501E_WIFI_DISCONNECTED;
	if (fail_reason != 0) *fail_reason = (uint8_t)ALP_CC3501E_WIFI_FAIL_NONE;
	if (rssi_dbm != 0) *rssi_dbm = 0;
	return CC3501E_HW_OK;
}

/* --------------------------------------------------------------- */
/* TCP/UDP sockets (v0.5) -- no IP stack on the host stub: report     */
/* NOTIMPL so the protocol path stays exercisable (handlers parse +   */
/* validate, then map NOTIMPL -> RESP_ERR_NOT_READY).                 */
/* --------------------------------------------------------------- */
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
	if (from_addr != 0) {
		for (unsigned i = 0u; i < 4u; ++i)
			from_addr[i] = 0u;
	}
	if (from_port_out != 0) *from_port_out = 0u;
	return CC3501E_HW_ERR_NOTIMPL;
}

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
	/* No IP stack on the stub -- nothing ever connects, so nothing is pushed. */
}

void cc3501e_hw_sock_prefetch(uint16_t handle, bool on)
{
	(void)handle;
	(void)on;
}

int cc3501e_hw_sock_recv_ring(uint16_t handle, uint8_t *buf, uint16_t cap, uint16_t *out_len)
{
	(void)handle;
	(void)buf;
	(void)cap;
	if (out_len != 0) *out_len = 0u;
	return -1; /* no prefetch on the stub -- caller uses the worker path */
}

int cc3501e_hw_sock_close(uint16_t handle)
{
	(void)handle;
	return CC3501E_HW_ERR_NOTIMPL;
}

/* --------------------------------------------------------------- */
/* BLE 5.4 (v0.3) -- no BLE host on the stub: report NOTIMPL so the   */
/* protocol path stays exercisable (handlers parse + validate, then   */
/* map NOTIMPL -> RESP_ERR_NOT_READY).                                */
/* --------------------------------------------------------------- */
int cc3501e_hw_ble_enable(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_ble_disable(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_ble_adv_start(uint8_t        connectable,
                             uint16_t       interval_min_ms,
                             uint16_t       interval_max_ms,
                             const uint8_t *adv_data,
                             uint8_t        adv_data_len)
{
	(void)connectable;
	(void)interval_min_ms;
	(void)interval_max_ms;
	(void)adv_data;
	(void)adv_data_len;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_ble_adv_stop(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_ble_scan_start(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_ble_scan_stop(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_ble_scan(uint8_t *buf, size_t cap, size_t *out_len)
{
	(void)buf;
	(void)cap;
	if (out_len != 0) {
		*out_len = 0u;
	}
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_ble_connect(uint8_t addr_type, const uint8_t addr[6])
{
	(void)addr_type;
	(void)addr;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_ble_disconnect(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_ble_gatt_register(const uint8_t *desc,
                                 uint16_t       desc_len,
                                 uint16_t      *handles_out,
                                 uint16_t       handles_cap,
                                 uint16_t      *num_handles_out)
{
	(void)desc;
	(void)desc_len;
	(void)handles_out;
	(void)handles_cap;
	if (num_handles_out != 0) *num_handles_out = 0u;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_ble_gatt_notify(uint16_t handle, const uint8_t *data, uint16_t data_len)
{
	(void)handle;
	(void)data;
	(void)data_len;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_ble_gatt_read(uint16_t handle, uint8_t *out, uint16_t cap, uint16_t *out_len)
{
	(void)handle;
	(void)out;
	(void)cap;
	if (out_len != 0) *out_len = 0u;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_ble_gatt_write(uint16_t handle, const uint8_t *data, uint16_t data_len)
{
	(void)handle;
	(void)data;
	(void)data_len;
	return CC3501E_HW_ERR_NOTIMPL;
}

/* --------------------------------------------------------------- */
/* OTA -- no PSA-FWU flash on the host stub: report NOTIMPL so the    */
/* protocol layer maps it to RESP_ERR_NOT_READY (the ztests assert    */
/* the handler framing/parse, not a real flash install).             */
/* --------------------------------------------------------------- */
int cc3501e_hw_ota_begin(uint32_t total_len)
{
	(void)total_len;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_ota_write(uint32_t offset, const uint8_t *data, uint32_t len)
{
	(void)offset;
	(void)data;
	(void)len;
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_ota_finish(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_ota_abort(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int cc3501e_hw_ota_promote(void)
{
	return CC3501E_HW_ERR_NOTIMPL;
}

int8_t cc3501e_hw_ota_reboot_rc(void)
{
	return 0;
}

uint8_t cc3501e_hw_ota_pending(void)
{
	/* The stub has no image store to query.  NONE, not UNKNOWN: there genuinely
	 * is no slot here, so "nothing pending" is the truthful answer rather than a
	 * failure to look -- and it keeps the stub's OTA_STATUS reply deterministic
	 * for the wire-vector tests. */
	return (uint8_t)ALP_CC3501E_OTA_PENDING_NONE;
}

bool cc3501e_hw_ota_flush_pending(void)
{
	/* The stub stages nothing to flash, so no flush window ever exists and the
	 * host is never asked to hold off. */
	return false;
}

void cc3501e_hw_ota_fault(uint8_t *stage, uint8_t *psa_lo)
{
	/* The stub never reaches flash, so nothing can fault. */
	if (stage != 0) *stage = 0u;
	if (psa_lo != 0) *psa_lo = 0u;
}

int cc3501e_hw_ota_status(uint8_t *state, uint32_t *bytes_written, uint32_t *total_len)
{
	if (state != 0) *state = 0u;
	if (bytes_written != 0) *bytes_written = 0u;
	if (total_len != 0) *total_len = 0u;
	return CC3501E_HW_ERR_NOTIMPL;
}

/* --------------------------------------------------------------- */
/* Power policy + diagnostics -- firmware-side config (no radio), so */
/* the stub accepts config and reports a cold-start diag.            */
/* --------------------------------------------------------------- */
int cc3501e_hw_set_power_policy(uint8_t policy, uint8_t wake_events, uint32_t idle_ms_before_sleep)
{
	(void)policy;
	(void)wake_events;
	(void)idle_ms_before_sleep;
	return CC3501E_HW_OK;
}

/* No radio in the stub build, so nothing can fail to take the policy. */
bool cc3501e_hw_power_radio_ok(void)
{
	return true;
}

int cc3501e_hw_set_log_level(uint8_t level)
{
	(void)level;
	return CC3501E_HW_OK;
}

uint8_t cc3501e_hw_reset_cause(void)
{
	return (uint8_t)ALP_CC3501E_RESET_POWER_ON;
}

uint32_t cc3501e_hw_uptime_ms(void)
{
	return 0u;
}

uint32_t cc3501e_hw_free_heap_bytes(void)
{
	return 0u;
}

uint32_t cc3501e_hw_wifi_last_event_id(void)
{
	return 0u;
}

uint8_t cc3501e_hw_radio_role(void)
{
	/* The stub brings no radio up -- every wifi_ap_start / connect here returns
	 * NOTIMPL -- so no role is ever up and ROLE_OFF is the honest answer, not a
	 * placeholder. */
	return (uint8_t)ALP_CC3501E_ROLE_OFF;
}
