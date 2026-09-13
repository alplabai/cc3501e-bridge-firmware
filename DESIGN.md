# cc3501e-bridge firmware — design notes

Scope, the wire framing, and the bench bring-up plan.  The authoritative
wire contract is
[`include/alp/protocol/cc3501e.h`](https://github.com/alplabai/alp-sdk/blob/main/include/alp/protocol/cc3501e.h);
this file records the firmware-side decisions and the host/firmware
framing they share.

## Command scope

`protocol_dispatch()` (`src/protocol.c`) routes **49 opcodes** -- every
command family in the wire header: META, Wi-Fi station/AP/scan/status,
BLE (enable, advertise, scan, connect, GATT), sockets, the GPIO proxy,
camera enables, power policy, diagnostics + `GET_PENDING_EVENTS`, and OTA
including `OTA_UPDATE_MODE`.  All of them route to TI's CC35xx Wi-Fi /
NimBLE / lwIP / `psa_fwu` APIs through the `hal/ti/` backend.

The META group is still the floor the link is debugged against -- nothing
else is worth reading until these four answer:

| Opcode | Behaviour |
|--------|-----------|
| `PING` (0x00) | empty reply data → `RESP_OK`; the liveness signal |
| `GET_VERSION` (0x01) | reply data = `ALP_CC3501E_PROTOCOL_VERSION` (u16 LE) |
| `GET_MAC` (0x03) | reply data = 6-byte factory MAC (HAL); stub → `RESP_ERR_NOT_READY` |
| `RESET` (0x02) | ack `RESP_OK`, then HAL reboots after the ack is drained |

Routed is not the same as proven: `BRINGUP_STATUS.md` records what is
silicon-validated per pillar, and **sockets do not connect**
(alp-sdk#1746).

What survives unchanged from the bring-up contract is the rejection rule:
an opcode this firmware does not implement returns
`ALP_CC3501E_RESP_ERR_INVALID`, per the protocol header's contract that
firmware rejects opcodes it does not implement.

## Wire framing -- hardware-SS0 phases + READY (current HW rev)

Both sides build the same frame: a 4-byte LE header
`[cmd | flags | payload_len(LE16)]` + payload.  The reply echoes the
request `cmd`, uses `flags = 0`, and its payload is `[status][data...]`
— the response status (`ALP_CC3501E_RESP_*`) is the first payload byte,
per the header.  Framing + dispatch is centralised in
`protocol_build_reply()` so SPI and SDIO are byte-identical.

**Wire MAJOR 4 (#2035) adds a mandatory 2-byte CRC-16/CCITT-FALSE trailer
to every frame, both directions**, INSIDE the declared `payload_len` --
covering the 4 header bytes plus every payload byte except the trailing 2
CRC bytes themselves.  This firmware is the **strict** side of that
migration: it requires the CRC on every request unconditionally and emits
the CRC-bearing reply shape unconditionally, with no dual-mode fallback --
a 3.1-shaped (no-CRC) request is rejected with `RESP_ERR_PROTOCOL`, never
executed.

**ONE normative exception: `CMD_GET_VERSION` (`0x01`) is accepted with or
without the CRC trailer.** A host cannot know a peer's wire major -- and
therefore cannot know whether to append a request CRC at all -- before
`GET_VERSION` has told it, and the host only appends one once that major is
already negotiated.  So the very first `GET_VERSION` any host ever sends a
fresh peer is, by construction, the zero-payload, no-trailer 3.1 shape.
Requiring a CRC on it unconditionally, like every other opcode, would mean
no host could ever discover a MAJOR-4 peer at all -- and OTA, the only path
from 3.1 to 4.0 firmware, rides this identical gated request path, so that
failure would be permanent, not a one-time hiccup.  See
`src/protocol.c`'s `protocol_build_reply()` for exactly where this
carve-out lives, and
`tests/unit/transport_spi/src/test_transport_spi.c`'s
`test_get_version_accepts_crc_less_bootstrap_request` for the regression
test.

The host driver (`chips/cc3501e/cc3501e_core.c` in alp-sdk) is
the **bilingual** side: it decodes both the legacy (no-CRC, `0x00`-is-OK)
shape and the MAJOR-4 (CRC-required, `0x5A`-is-OK) shape, keyed off the
firmware major it negotiates at `GET_VERSION`.  `ALP_CC3501E_RESP_OK` also
moves off `0x00` to `0x5A` in the same bump -- `0x00` was indistinguishable
from a dead SPI phase clocking back nothing (#1378); the CRC trailer is the
structural fix for that, and the status-byte move is what makes the two
wire shapes tell themselves apart.

**Migration order (see the MAJOR-4 paragraph above
`ALP_CC3501E_PROTOCOL_MAJOR` in `<alp/protocol/cc3501e.h>` for the full
text):** the Alif host image rolls first (MCUboot + the ATOC, independent
of the coprocessor); coprocessors are then OTA'd to 4.0 firmware *through*
that already-bilingual host; only a release *after* the whole fleet is
confirmed on 4.0 may drop the host's legacy decode branch.  Never ship a
host that refuses wire MAJOR 3 before that OTA sweep completes -- OTA is
the only path from 3.1 to 4.0 firmware, and it needs a host that can still
talk to a 3.1 board.

`ALP_CC3501E_FLAG_ASYNC_EVENT` is defined by the wire header but **no
code in this firmware ever sets it**: every frame the CC3501E emits is a
solicited reply with `flags = 0`.  Async events do not travel as their
own frames at all — they are queued in the event ring and handed back
inside an ordinary `GET_PENDING_EVENTS` reply (the attention edge below
only tells the host *when* to ask).

The **current E1M-AEN rev wires SCLK/MOSI/MISO plus a real hardware
chip-select**: the Alif side muxes `P14_7` as `SPI1_SS0_C`, and the
dwc-ssi master asserts/deasserts SS0 around each SPI transfer.  READY
(`P2_6` on the Alif side) gates each reply phase so the host does not
clock a slave that has not re-armed yet.  A request/reply is therefore
clocked as **four SS0-framed protocol phases**, each side deriving the
next length from a header it already exchanged:

| # | master clocks | direction | length |
|---|---------------|-----------|--------|
| 1 | request header | MOSI | 4 |
| 2 | request payload | MOSI | `payload_len` (from #1) = data + 2-byte CRC |
| 3 | reply header | MISO | 4 |
| 4 | reply payload | MISO | reply `payload_len` (from #3) = status + data + zero pad + 2-byte CRC |

**Phase 4 is NOT `1 + data_len`.**  `protocol_build_reply()` rounds the
reply payload up to a multiple of `CC3501E_REPLY_PAD` (8) with zero bytes,
so the declared `payload_len` is `status + data + pad + crc` and the host
clocks the padded length.  The CRC trailer always occupies the LAST 2 bytes
of that padded span, so a bare-status reply (which already padded from 1 to
8 bytes before wire MAJOR 4) costs zero extra wire bytes: the CRC simply
consumes 2 of what used to be 7 pad bytes.  The padding buys the host's DW
SSI a burst-aligned DMA transfer; the cost is that **a variable-length
reply payload must be self-delimiting**, because `payload_len` no longer
delimits the data.  It was not, once: an empty `GET_PENDING_EVENTS` drain
came back as 7 zero pad bytes and the host walked them as three
`opcode 0x00, len 0` events, ~5.8 phantom events per second forever
(alp-sdk#1740, see `BRINGUP_STATUS.md`).  Any NEW variable-length reply
payload must carry its own count or its own terminator, or it walks into
the same trap.

**Sizing consequence of the request CRC (#2035):** a request's own
`payload_len` now also counts its 2-byte CRC trailer, so a maxed-out
`SPI1_TRANSFER` or `OTA_WRITE` chunk needs 2 fewer data bytes than
`ALP_CC3501E_SPI1_MAX_XFER` / `ALP_CC3501E_OTA_MAX_CHUNK` (the canonical
header's PRE-CRC constants) structurally allow, or the request's declared
`payload_len` exceeds `ALP_CC3501E_MAX_PAYLOAD` and the transport clamps it,
truncating exactly the CRC trailer at exactly the largest transfer size.
`src/protocol.h`'s `CC3501E_SPI1_MAX_XFER_V4` / `CC3501E_OTA_MAX_CHUNK_V4`
(each the header constant minus `ALP_CC3501E_CRC_BYTES`) are what this
firmware reports (`CMD_SPI1_CONFIGURE`'s `max_xfer`) and enforces.

**Both CRC directions run inside the SPI-slave ISR (#2035).**
`protocol_build_reply()` -- which now verifies the request CRC and appends
the reply CRC -- runs from `hal/ti/transport_hw_ti_spi.c`'s `on_transfer()`
callback via `dispatch_frame()`, so a maximum-size (~4096-byte) frame's CRC
work has to fit inside `CC3501E_PHASE_SETTLE_US` (250 us, `cc3501e_core.c`).
`<alp/protocol/crc16.h>`'s bitwise, per-bit form -- the canonical
single-source algorithm -- costs on the order of 50-65 cycles/byte (its own
doc comment: ~640 cycles for an 8..16-byte reply), so 4096 bytes is
~225,000 cycles, 4-11x OVER budget at any plausible CC3501E M33 clock
(80-200 MHz).  `src/protocol.c` therefore builds a 256-entry, LE16
table-driven equivalent (`crc16_table_update()`) -- GENERATED from the same
canonical bitwise routine (`alp_crc16_ccitt_false_update()`) rather than a
second, independently-typed implementation, so there is exactly one
algorithm and the table is provably a lookup-accelerated view of it -- at
3-4 cycles/byte, ~12,000-16,000 cycles for the same 4096-byte case, i.e.
60-200 us: inside budget with margin across that whole clock range, where
the bitwise form was not.  The 512-byte table (256 * `sizeof(uint16_t)`) is
kept in DRAM (a plain non-const array, lazily filled once) rather than
flash, so a lookup never waits on a flash access from inside the ISR --
negligible against the firmware's 512 KB DRAM budget (`BRINGUP_STATUS.md`).

The host waits for READY before the reply header and reply payload
phases.  Firmware side: `hal/ti/transport_hw_ti_spi.c` (a `SPI_PERIPHERAL`
+ `SPI_MODE_CALLBACK` state machine that replays the captured frame through
the silicon-free byte seams and advances on transfer completion;
`SPI_PERIPHERAL` is the CC35xx TI Drivers term for SPI slave).  Host
side: `chips/cc3501e/cc3501e_core.c` `cc3501e_request()` (matching four-phase
sequence + `resp_to_status()` on `payload[0]`).  Host + firmware are
reconciled to each other and to the header, and this hardware-SS0 bridge
has been bench-validated on E1M-AEN801.

GET_VERSION returns the *protocol* version (the host's compat gate),
not the firmware *release* version — the diag-struct comment that
implied otherwise was a header doc slip (now fixed); the release version
is surfaced via `GET_DIAG_INFO.fw_version` in v2.

## Three version numbers (keep them straight)

The bridge carries **three** independent version numbers; each gates a
different thing and they must not be conflated:

| Version | Source of truth | Surfaced by | Gates |
|---|---|---|---|
| **App SemVer** | `firmware-version.txt` (e.g. `0.2.0`) | `GET_DIAG_INFO.fw_version` (u16) | firmware release identity — human-facing "what's running" |
| **Wire protocol version** | `ALP_CC3501E_PROTOCOL_VERSION` in `<alp/protocol/cc3501e.h>` — MAJOR.MINOR since ADR 0033; this tree targets `4.0` (`CC3501E_FW_IMPLEMENTS_PROTOCOL_MAJOR`/`_MINOR`, `src/protocol_meta.c` — see `protocol-version.txt`).  v4.0 (#2035) moves `ALP_CC3501E_RESP_OK` off `0x00` to `0x5A` and adds a mandatory 2-byte CRC-16/CCITT-FALSE trailer to every frame, both directions; this firmware is the strict (CRC-required, no dual-mode) side, the host driver the bilingual side — see "Wire framing" above. | `GET_VERSION` (0x01) | host↔firmware wire compatibility (host refuses a **MAJOR** mismatch — enforced by `cc3501e_reset()` in `chips/cc3501e/cc3501e_core.c`, which reads `GET_VERSION` once the cold boot completes and returns `ALP_ERR_VERSION` on a MAJOR disagreement; a MINOR difference is additive and does not refuse the link; #1371, ADR 0033) |
| **GPE flash/image version** | `--version`, supplied EXPLICITLY to `ti/deploy_validate.sh` / `ti/regen_flashset.sh` / `ti/validate_gpio_bench.ps1` (no default) | — (programmer only) | CC35 vendor-RoT anti-rollback (unit rejects `<=` the programmed value) |

**App SemVer → `fw_version` marker.** The runtime u16 is *derived* from
`firmware-version.txt`, never hand-typed, so it cannot drift. Both build
paths parse the SemVer and pass the packed value in:
`CMakeLists.txt` (`target_compile_definitions`) and `ti/build_ti.sh`
(`-DCC3501E_BRIDGE_FW_VERSION_U16`). Pre-1.0 packing is
`(MINOR << 8) | PATCH`, so `0.2.0 → 0x0200`. `src/protocol.c` keeps an
`#ifndef` fallback equal to the current release for standalone compiles.

**GPE flash version** is *not* the app version. It is a monotonic
anti-rollback counter, and it is enforced by the SBL against the
last-seen version on the part **even when every rollback-protection fuse
reads 0** -- a warm programming run burns no fuses, so an all-zero fuse
report is not permission to go backwards.

Two hard rules, both bench-proven, both easy to get wrong:

- **`major` MUST be `0`.** A vendor image with `major >= 1` fails
  SES/BL2 secure-boot authentication (boot report `@0x28000104` sets
  `AUTH_ERROR 0x80`) and the app core never launches -- the host then
  reads `get_version = -5`. Byte-identical firmware authenticated at
  `0.0.1.0` and `AUTH_ERROR`'d at `1.0.0.0`.
- **Each field must be `<= 255`.** A human date like
  `1.<yy>.<mmdd>.<hhmm>` is INVALID: `mmdd = 0705` and `hhmm = 1531`
  overflow a byte and silently corrupt the version.

`deploy_validate.sh` USED to derive it as major `0` plus the low three bytes
of the epoch, `0.$(((_e>>16)&255)).$(((_e>>8)&255)).$((_e&255))`.  **That default
is gone** (#37, and #56 for the PowerShell script that kept one): an epoch-derived
stamp is a rollback stamp on any unit whose last flash was higher, which streams
clean and then refuses to boot, so all three scripts now REQUIRE the stamp and
refuse to run without it.  Pass a value strictly greater than anything ever
flashed on that unit, with major `0` and every field `<= 255`.  This paragraph
described the removed default as current behaviour in a normative recipe (#66).

> An earlier revision of this section prescribed exactly the two schemes
> above as REQUIREMENTS -- `major >= 1` and `major.<yy>.<mmdd>.<hhmm>`.
> Both produce an unbootable part. It cited `deploy_validate.sh`, which
> had already been corrected in the same PR that added this paragraph.
> `ti/package_cc3501e_prod.ps1` and `ti/validate_gpio_bench.ps1` carried
> the same wrong default (`1.0.0.0` / `1.0.0.1`) until 2026-08-28.

## Selectable host-control transport

SPI is the default + always-available baseline; SDIO is opt-in and
mutually exclusive with a micro-SD card (the Alif has one SDIO
controller).  Both transports compile; `CC3501E_CONTROL_TRANSPORT`
selects which `main()` starts.  See README.md.

## Power policy: both halves run on the TASK, never in the dispatch ISR

`handle_power_policy` (0x62) is reached from the SPI-dispatch ISR, and **neither**
half of the policy may run there:

- the radio half calls `Wlan_Set()`, a blocking vendor call -- the same reason
  `handle_sock_recv()` cannot call `lwip_recv()` and the worker seam exists;
- the core half calls `Power_enablePolicy()`, which arms the function pointer the
  idle loop runs, racing an idle loop that may be executing it.

  This bullet used to name `Power_setPolicy()` as well.  The core half
  deliberately stopped calling it -- see `hal/ti/cc3501e_hw_ti_power.c`, which
  explains at length that swapping the policy pointer at runtime is what made
  #1683 intermittent, and that holding the DISALLOW constraints expresses every
  preset without a policy swap.  Issue #20.

Running either inline did not merely fail: on silicon every preset returned `-4`
and the bridge itself went to `PING -> -5`. Both are therefore latched by the ISR
handler (`pp_core_dirty` / `pp_radio_dirty`) and drained by
`cc3501e_hw_power_service()` from `cc3501e_hw_tick()`, core first then radio.

**The wire consequence: a `RESP_OK` to `POWER_POLICY` means QUEUED, not APPLIED** --
the same semantic `OTA_BEGIN` turned out to have. The reply carries one data byte
reporting whether the *previous* radio apply was realised, surfaced to the host as
`cc3501e_power_policy()`'s `radio_ok_out`. Without it a host cannot distinguish an
accepted-and-applied policy from an accepted-and-silently-rejected one, which is
exactly how a broken radio apply went unnoticed on the bench.

`Wlan_Set()` is also rejected while the radio is down, so the latched policy is
re-applied once `Wlan_RoleUp(STA)` succeeds; otherwise a policy set before Wi-Fi
came up is dropped.  The RADIO half of that re-apply is no longer only a
tick-drained latch, though: `cc3501e_hw_wifi_ensure_sta_role()` calls
`cc3501e_hw_power_apply_radio_now()` SYNCHRONOUSLY, on the same task, right after
`Wlan_RoleUp(STA)` succeeds and before returning -- so an unconfigured STA's
effective policy (PERFORMANCE/ACTIVE) is in place before `Wlan_Connect` starts
association and the DHCP lease poll, not after the next `cc3501e_hw_tick()`.
`cc3501e_hw_power_service()`'s drain still exists for an explicit host
`POWER_POLICY` (which always runs from the ISR and must defer both halves,
same as above) and, with no STA role up yet, `pp_apply_radio_effective()`
skips calling `Wlan_Set()` at all rather than risk marking a policy applied
that the vendor SDK's own `WLAN_SET_POWER_SAVE` path silently drops with no
STA interface.  That skip does NOT leave anything latched for a later drain to
retry -- `cc3501e_hw_power_service()` clears its dirty flag regardless of
whether the apply underneath it actually ran.  The policy is not lost anyway:
`cc3501e_hw_wifi_ensure_sta_role()` calls the same effective-policy apply
UNCONDITIONALLY on every STA role-up, re-reading the latched policy VALUES
fresh each time rather than depending on the flag, so the first role-up after
a policy was set with no role up always applies it for real.

> **[#1691](https://github.com/alplabai/alp-sdk/issues/1691):** repeated BLE
> advertise/stop cycles can wedge the bridge. It is NOT power-related — it
> reproduces with no power policy applied. Diagnostics across the fault show the
> slave armed and idle in `PH_REQ_HEADER`, READY HIGH, `g_resync_count` and
> `g_arm_fail_count` both zero, and SPI transfers still completing, so no firmware
> self-heal can see it: `bridge_transport_spi_is_dead()` only reports a failed
> `SPI_open`, and the stall watchdog deliberately ignores `PH_REQ_HEADER`. Recovery
> is host-side via `cc3501e_recover()` (warm reset), which cleared every observed
> wedge. The `CC3501E_WEDGE_PROBE` build flag captures the state into `.TI.noinit`
> for further investigation.

## Backends

- `stub` (`hal/cc3501e_hw_stub.c`): hardware-free; HW ops return
  `CC3501E_HW_ERR_NOTIMPL`.  The host test + CI compile smoke build this.
- `ti` (`hal/ti/`): the real bench backend, built with `ticlang` against
  TI's SimpleLink CC35xx SDK (FreeRTOS + LwIP + TI Drivers).  Implemented:
  - `cc3501e_hw_ti.c`: `Board_init()` (Power + GPIO + DMA) then
    `SPI_init()`, and the deferred reset via CMSIS `NVIC_SystemReset()`
    after the ack is sent.
  - `cc3501e_hw_ti_wifi.c`: the lazy `Wlan_Start` bring-up
    (`cc3501e_hw_wifi_lazy_start`) and `cc3501e_hw_get_mac` via
    `Wlan_Get(WLAN_GET_MACADDRESS)` on a `WlanMacAddress_t`.  There is no
    `sl_*` SimpleLink call anywhere in this tree -- the CC35xx host API is
    `Wlan_*`.
  - `transport_hw_ti_spi.c`: the four-phase hardware-SS0 SPI-slave
    transport above (`SPI_open(..., SPI_PERIPHERAL, SPI_MODE_CALLBACK)`)
    with READY gating between phases.
  - `transport_hw_ti_sdio.c`: frame glue complete; the SDIO-device
    register bring-up remains to be written — off the critical path,
    since SPI is the default.

    The blocker here USED to be recorded as "needs SWRU626 §21 (no
    public SDK SDIO-device driver)".  Half of that is wrong and the
    correction matters, because it was being read as "the part may not
    do device mode at all".  It does, and §21 is precisely where TI
    documents it — §21.1: *"The SDIO module in the CC35xx acts as a SDIO
    card peripheral to an external SDIO host."*  The device-mode blocks
    are `SDIO_CARD_FN1` at `0x41B00000` (function 1 data path, §21.5)
    and `SDIO_CORE` at `0x41B05000` (function 0 / CIA, §21.4), with
    interrupt `SDIO CARD_IRQ` and uDMA peripheral indices 14 (RX) and
    15 (TX).  That is a distinct peripheral from the SDMMC *host*
    controller at `0x41912000`, which uses uDMA 12/13.  What is
    genuinely absent is a public SDK SDIO-**device** driver, so the
    register bring-up is ours to write; the documentation is not
    missing.

    Two requirements for whoever writes it, both from §21 and neither
    obvious from the SPI transport:

    - **The reply path needs an acknowledgement gate, not a
      block-sent event.**  §21.3.7.2: *"packet N+1 is handled (i.e.
      copied from internal memory) only after packet N is ACKED ...
      CC35xx SDIO interface is a card with necessary read
      acknowledgement."*  `IRQSTA` bit 6 is `HCIACK`, bit 7 `HCINACK`,
      bit 8 `HCIWRRET`.  This is not cosmetic: the deferred `CMD_RESET`
      reboot and the OTA `FINISH` swap-reboot are both armed by
      `cc3501e_hw_notify_reply_sent()`, so wiring that to "block sent"
      rather than to `HCIACK` lets either reboot fire on a packet the
      host NACKed.
    - **SDIO carries its own attention channel**, which the SPI link
      has to fake.  §21.3.1 gives in-band or out-of-band host
      interrupt, the OOB pin being `sdio_oob_irq` (`IOSEL 11h` on
      `GPIO17PCFG` — the pad the bridge already uses for READY — and on
      `GPIO_2`).  Better, `C2HMSG` (offset `30h`) carries bit 16
      `C2HIRQ` plus a 16-bit event bitmap in `C2HSTS[15:0]`, cleared by
      the host via `CLINTERD`, with `H2CMSG` (offset `34h`) for the
      reverse.  An async event would then carry its own identity
      instead of a bare edge, removing the 50 ms empty-drain backoff and
      the attention-suppressed-while-busy limitation.

    Note that on the E1M-AEN801 this transport is not merely optional,
    it is unavailable: the Alif has a single SDIO controller and this
    SoM commits it to the micro-SD card (see the transport table above).
    SDIO therefore needs a board that makes the opposite choice.

## SimpleLink command structs: a zero-init is NOT a set of defaults

TI's `Wlan_*` command structs are plain C structs the caller fills and the NWP
obeys literally.  Several of their fields have a meaningful zero rather than an
"unset" zero, so `Cmd_t c = { 0 };` plus the two or three fields you care about
silently configures the radio with values nobody chose.

This cost weeks on #1562.  `cc3501e_hw_wifi_ap_start()` filled `ssid`, `channel`
and `secParams` on a zeroed `RoleUpApCmd_t` and left `sta_limit` at 0 -- a field
TI documents as "limits the number of stations that the AP's has".  Zero
permitted zero clients.  The AP beaconed flawlessly (275 s at 33-95% on a second
radio), so every single-radio observation looked healthy, and the failure only
appears when something actually tries to associate.  TI's own reference filler
`ParseRoleUpApCmd()` (`demos/network_terminal/cmd_parser.c`) defaults it to 4 and
clamps anything outside `[1, 8]` back to 4 -- 0 is not merely a poor default, it
is outside the legal range.

Three sibling fields in the same struct had the same problem: `countryDomain`
`{0,0,0}` where TI sets the `"00"` world domain, `sae_anticlogging_threshold` 0
which *is* `SAE_ANTI_CLOGGING_ALWAYS` rather than unset, and `sae_pwe` 0 where TI
uses 2.

**The rule for this HAL: when filling a SimpleLink command struct, diff your
field list against TI's reference filler for that command, and set everything it
sets.**  Fields you deliberately leave at the zero value get a comment saying so
(`hidden = FALSE` and `tx_pow = 0` are genuinely TI's defaults, and are left
implicit on purpose).  A zero you did not think about is a configuration you did
not choose.

## Async events: the attention edge (shipped), and a dedicated HOST_IRQ (future)

The Alif is SPI master, so the CC3501E can never initiate a transfer, yet the
protocol defines async events (`EVT_WIFI_*`, `EVT_BLE_*`,
`EVT_GPIO_INTERRUPT`) with 5-10 ms latency budgets
([`docs/cc3501e-bridge.md`](https://github.com/alplabai/alp-sdk/blob/main/docs/cc3501e-bridge.md)).
Polling cannot meet those budgets without
hammering the bus.

### What shipped (#130, alp-sdk#1721 -- 2026-08-27)

Rather than wait for a board rev, the slave->master attention path was
**multiplexed onto the existing READY wire** (CC35 `GPIO17` -> Alif `P2_6`,
the rev-1 bodge line).  The firmware pulses that line when it queues an
event (`cc3501e_bridge_attn_pulse()`, called from `event_ring_push()`); the
host arms an edge interrupt with `cc3501e_attn_arm()` and drains on the
edge.  Validated on silicon: the host application saw 135 of 135 firmware
pushes across a 40 s armed window with the console timer poll set to 60 s,
so no timer delivery was possible.

Three properties of the shared wire are load-bearing, not incidental:

- **Sticky arm intent.**  The ISR masks the line and the request lock
  re-arms it, but ONLY inside an explicit `cc3501e_attn_arm(ctx, true)`
  window.  Unconditional re-arming in `lock_release()` hangs the device
  after `WIFI_SCAN`.
- **Empty-drain backoff (50 ms).**  The wire also carries READY flow
  control, so a transaction end is indistinguishable from an attention
  pulse and a drain's own trailing READY rise re-triggers the ISR.  A drain
  that delivered zero events backs off; one that delivered events re-arms
  immediately.  Without it: 11888 ISRs for 86 events.  With it: 220, with
  nothing lost.
- **A full ring must still pulse.**  `event_ring_push()` raises attention on
  a DROPPED push too.  Pulsing only on success is unrecoverable on an idle
  host: 16 boot-time Wi-Fi events fill `CC3501E_EVENT_RING_SLOTS`, after
  which no pulse means no drain means still full.

### Two limits to know before relying on it

- **The pulse is a build-time opt-in.**  `build_ti.ps1 -AttnPulse` sets
  `-DCC3501E_ATTN_PULSE=1`; it is **default OFF** because the wire is a
  rev-1 bodge absent on the stock EVK, and the pulse is an extra transition
  on the line every host uses for flow control.  `package_cc3501e_prod.ps1`
  does not pass it.  A build without the flag links a no-op stub, so a host
  that arms attention never gets an edge and falls back to its timer poll.
- **Only Wi-Fi events have a producer.**  `EVT_WIFI_CONNECTED` and
  `EVT_WIFI_DISCONNECTED` are pushed into the ring; `EVT_GPIO_INTERRUPT`
  and the BLE events are not pushed by anything today.  The transport is
  not the blocker for those -- an `event_ring_push()` call at the producer
  is.

### Still future: a dedicated HOST_IRQ line

Sharing the wire with flow control is what forces the empty-drain backoff
above.  A future board rev can give attention its own pad.  SPI and SDIO
are mutually-exclusive control transports, so a future SPI mode can reuse
an SDIO-capable CC3501E pad when SDIO is not active:

| CC3501E pin | SDIO mode | SPI mode (future) |
|-------------|-----------|-------------------|
| `GPIO3`     | SDIO.CLK  | **HOST_IRQ** -> Alif `P7_0` (E1M `IO0`) |

That pad consumes E1M `IO0` **only in SPI mode**; in SDIO mode the pin is
the SDIO clock and `IO0` is unaffected (a per-transport pinmux, configured
at build time alongside `CC3501E_CONTROL_TRANSPORT`).  Boot-safe:
active-high with an Alif pull-down; firmware drives it low early and the
Alif arms the interrupt only after the boot budget.

## Bench bring-up open items (AEN801)

The firmware is identical across all AEN SKUs (the CC3501E + its
inter-chip wiring are AEN-family-wide); these are board/SDK anchors, not
firmware redesigns:

1. **SysConfig anchor — DONE, kept here so it stays findable.** Both
   board files are committed: `ti/cc3501e_aen.syscfg` and
   `ti/cc3501e_aen_wifi.syscfg` (plus `ti/cc3501e_mem.syscfg`) define
   `CONFIG_SPI_0` — the inter-chip SPI on CC3501E GPIO_27/28/29 plus the
   CC35-side CSN resource paired with Alif hardware SS0.
2. **Async-event producers.** The attention transport shipped (#130,
   alp-sdk#1721) on the shared READY wire; what is missing is producers.
   Only `EVT_WIFI_CONNECTED` / `EVT_WIFI_DISCONNECTED` reach
   `event_ring_push()`.  `EVT_GPIO_INTERRUPT` and the BLE events need a
   push at the source, not new hardware.  A dedicated HOST_IRQ pad is a
   separate, later board-rev item.
3. **SDIO-device.** Implement the §21 register bring-up in
   `transport_hw_ti_sdio.c` if/when SDIO is needed (SPI is the default).
4. **Flashing.** The CC3501E is Alp-OTA-updated (`update_channel:
   alp_ota_spi_otp`, `flash_policy: recovery_only`); a customer flash
   is permitted only to recover a bricked device, with Alp Lab-supplied
   binaries.  Bench units are warm-programmed via SWD/J-Link (see alp-sdk
   [`docs/cc3501e-production.md`](https://github.com/alplabai/alp-sdk/blob/main/docs/cc3501e-production.md)).
   The retired public `cc3501e_usb_bootloader` backend and
   `flash.py` release/bench helper now live in `alp-sdk-internal`
   as Alp-internal OTA-build tooling.
