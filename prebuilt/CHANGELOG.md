# CC3501E prebuilt firmware — release notes

Each entry corresponds to a tagged release of this repository's source.
The signed binary, its detached signature, and a SHA-256 manifest are
dropped into this directory and named `cc3501e-vX.Y.Z.bin` (matching
`firmware-version.txt` at the repo root).

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Fixed

**Correction to v0.8.0's "An unconfigured station now defaults to ACTIVE" claim
below.** That fix (`cc3501e_hw_power_service()` applying the effective policy
after `Wlan_RoleUp(STA)`) only actually landed once `cc3501e_hw_tick()` next
drained it, which requires a return to `main()`'s bringup loop between the
role-up and the connect. A `wifi connect` issued as the FIRST radio op of a
boot runs `Wlan_RoleUp(STA)` and `Wlan_Connect` back-to-back inside the same
worker job, with no such return in between -- so the ACTIVE default did not
land before association and DHCP for that specific ordering, and v0.8.0's own
bench numbers below it (12 of 16 clean inside the connect call, 14 of 16 by any
route) do not generalise to a connect-first attempt. Source fix pending
release; see `hal/ti/cc3501e_hw_ti_wifi.c`'s `cc3501e_hw_wifi_ensure_sta_role()`
and `hal/ti/cc3501e_hw_ti_power.c`'s `cc3501e_hw_power_apply_radio_now()`,
which now apply the policy synchronously instead.

- **Worker-path `SOCK_RECV` after EOF answered `RESP_ERR_RADIO`.** A socket
  not owned by the prefetch ring (UDP, or an accepted STREAM socket never
  armed for it) got its full body, one `OK`/0-byte EOF recv, then
  `RESP_ERR_RADIO` on the NEXT recv. TI's lwIP frees the socket's `recvmbox`
  on the first post-FIN recv; every later recv then returns `ENOTCONN` as a
  genuine error, not another 0-byte `OK`. A per-fd sticky EOF latch now
  short-circuits every later recv on that fd to `OK`/0 without touching lwIP
  again.
- **A reset on the prefetch-ring socket spun to `ALP_ERR_TIMEOUT` instead of
  erroring.** The ring's pump silently ignored a real `lwip_recv()` failure
  (RST etc, not `EAGAIN`), so a drained, dead ring kept answering `BUSY`
  forever and the host spun to its own `timeout_ms`. The pump now latches the
  failure, and the ring answers `RESP_ERR_RADIO` once drained -- the same
  status a worker-path socket failure gets.
- **A zero-length recv could latch a false EOF.** The sticky-EOF latch above
  originally fired on any 0-byte result from a STREAM socket; a recv whose
  `max_len`/`cap` clamp to 0 also returns 0 bytes from TI's lwIP, with no
  bearing on whether the peer actually closed, so it could freeze a
  perfectly live socket as "EOF" and truncate the stream. The latch now also
  requires the request to have asked for more than 0 bytes.
- **A ring-served `SOCK_RECV` with `max_len == 0` silently dropped bytes.**
  The ring's room computation read wire `max_len == 0` as "no cap" rather
  than "the host's own capacity is 0", so it served up to a full frame from
  the ring and retired those bytes from the ring's lazy-commit tail -- the
  host then copied `min(data_len, cap == 0)` of them, i.e. none, and the
  rest were gone. `max_len == 0` now clamps the ring's room to 0 and
  answers `OK` immediately, matching the worker path.

## v0.8.0

GPE: `0.254.5.0`. Wire protocol: `4.0`. sha256
`c67ad58a8bbf8be493f025571643740fe8f39921d25e932e2d8530c18d1ba2a7`.

**RE-CUT 2026-09-12, from source, not merely re-wrapped.** v0.8.0 was never
distributed, so the version number was still free -- the same move this file
records for the 0.6.0 pre-release that became v0.5.1 before distribution. The
previous v0.8.0 blob (`c63fcbad...`, GPE `0.149.222.0`) is superseded and must
not be flashed: bench unit `2026W36-0003` has since climbed to `0.254.4.0`, and
the CC35 secure boot loader enforces GPE monotonicity per part, so that older
blob would stream clean onto it, exit 0, and then permanently refuse to boot.

### What this re-cut adds — the station now gets a DHCP lease

The station associated reliably and then failed to obtain an address about three
times in four. Two mechanisms were found and fixed, each verified on silicon by
the failure signature changing, not merely by a better number:

1. **The station was in power save across DHCP.** `Wlan_Start()` defaults to
   `WLAN_STATION_AUTO_PS_MODE`, and the bridge additionally applied its latched
   `BALANCED` policy right after `Wlan_RoleUp(STA)` -- which also maps to
   AUTO_PS. An AP buffers broadcast until a DTIM beacon, and a sleeping station
   on a marginal link misses beacons and therefore misses DTIMs. An unconfigured
   station now defaults to ACTIVE; a host `POWER_POLICY` still wins once sent.
   *Evidence:* the failing DHCP state moved from `SELECTING` (no OFFER arriving)
   to `REQUESTING` (OFFER arrives, ACK does not).

2. **lwIP's retransmit backoff turned retries into dead air.** `dhcp_select()`
   arms `1 << tries` seconds just as `dhcp_discover()` does, so by `tries = 5`
   the next REQUEST is 32 s away and the client spends the budget idle. A
   stalled client is now restarted -- gated on `tries >= 3` so an exchange about
   to be answered is never interrupted, capped at 2 restarts so an absent server
   still terminates. *Evidence:* `tries` on the failing attempt fell from 5 to 3
   and late leases moved from 12 s to 1 s.

Measured on `e1m-aen-evk-01` at -80/-81 dBm on the on-board antenna, cold-cycled
between every attempt:

| | before | after |
|---|---|---|
| address inside the connect call | ~1 in 4 | **12 of 16** |
| address by any route | ~1 in 4 | **14 of 16** |

**Residual, stated plainly:** 1 of 16 still failed at `REQUESTING`/`tries = 3`,
and 1 further attempt hit the separate first-radio-op scan wedge already recorded
for that place. Both are counted against the fractions above rather than
discarded. At this RF level a four-frame DHCP exchange failing occasionally is
link loss rather than a logic defect; a host that retries once reaches roughly
98%, and the driver now reports enough to decide -- a distinct no-address status,
the DHCP state and the retry count.

### The first-radio-op wedge is a KNOWN condition, handled host-side

Roughly 2 in 16 cold boots, the first worker-routed radio opcode of a boot times
out (`-4`) and the link then reads `-5` until a cold cycle. **The rule: if the
FIRST radio op of a boot fails that way, issue one hard reset and retry.** That
takes 2 in 16 to about 1 in 128 and costs no new mechanism -- it is the same
primitive already used for the ~20% Puya cold-boot miss.

**Do not "fix" this by calling `cc3501e_hw_wifi_boot_start()`.** It is the
obvious move and it is wrong, for reasons that are not the ones you will guess:

- The MCUboot trial accept is gated on `g_host_txn_count > 0` -- one fully
  drained host reply -- deliberately, so an image that boots but wedges the
  bridge never becomes permanent. Bringing the radio up before
  `transport_spi_init()` puts a possible hang in front of an accept that cannot
  fire yet, which on a freshly flashed TRIAL image is the 2026-06-18 no-launch
  state. At a 2-in-16 hang rate that is roughly a 12% chance per fresh flash of
  needing an SWD recovery, to win 2 in 16.
- `boot_start()` reaches `bridge_transport_spi_hw_reinit()` while `spi == NULL`,
  which opens and arms the slave; `transport_spi_init()` then opens it again,
  and `SPI_open` on an already-open index returns NULL. Nothing closes between.
- The host's blind post-reset settle is 3500 ms, and `Wlan_Start` plus `RoleUp`
  measures around 4.5 s here, so boot-time radio needs a host change too.

Note the suspend hazard is NOT the blocker at that call site: with `spi == NULL`
`bridge_transport_spi_hw_suspend()` degenerates to two flag writes and never
reaches `SPI_transferCancel`.

**The cheapest next evidence, if this is ever worth reopening,** is not a fix but
a discriminator: flash the existing `CC3501E_WEDGE_PROBE` build and read
`probe_ticks` after a wedge. Frozen means the worker task itself is stuck -- most
likely an unbounded `Wlan_Start`, whose `HIFInit` kills the bridge DMA before it
returns, so the recovering reinit never runs. Advancing means the task is alive
and only the slave is dead, which is the OTA pump's quiesce/retire/release/reinit
shape rather than anything in this list. The two signatures recorded so far are
consistent with two different causes, and one run separates them.

Also in this cut: the lease budget covers lwIP's fourth DISCOVER rather than
stopping four seconds short of it; `GET_DIAG_INFO` grew from 16 to 18 bytes to
report the STA DHCP state and retry count (an older host reads 16 and is
unaffected); the vendor console is initialised at boot, because `InitTerm` was
dead-stripped and `Report()` -- which the vendor's own `link_callback` calls
immediately before `dhcp_start()` -- was walking a NULL UART handle on every
connect.

**The wire carries a CRC-16/CCITT-FALSE trailer on every frame, both
directions** (MAJOR 4). It exists because a dead SPI phase was observed on real
silicon returning `RESP_OK` with a corrupted payload: the old `RESP_OK` of
`0x00` is exactly what a dead phase clocks back for every byte it touches, so a
total transport failure was indistinguishable from success. `RESP_OK` is now
`0x5A` and the CRC makes corruption detectable rather than silently accepted.

**`CC3501E_WIRE_CRC` makes that a build option, and it is a wire-major
selector rather than a cosmetic toggle.** ON (default) is MAJOR 4. OFF reports
MAJOR 3 and is byte-identical to the 3.1 wire — proven against a real pre-4.0
build, 31 of 31 shared vectors matching. A `_Static_assert` enforces the
pairing: an OFF build that still reported 4 would tell a MAJOR-4 host to append
CRCs this firmware ignores and demand CRCs it never sends, which is a silent,
total link failure that looks like a hardware fault. Turning it off removes the
dead-phase guard, which is why it is not the default.

**The fix that made 4.0 work at all.** The 256-entry CRC table was built lazily
on first use, and first use is `protocol_build_reply()` servicing request one —
inside the SPI interrupt handler. That is roughly 100 us at this core's 160 MHz,
against a host reply-header gate that is a blind fixed 200 us already mostly
spent on normal dispatch. Frame one overshot it: the host clocked into a slave
with nothing armed, those bytes sat in the RX FIFO, and every later transfer
returned the previous phase's bytes — a permanent one-transfer lag that no
retry clears. The table is now built on the boot path, before the slave is
armed. Without this, every opcode returned `-5` and the part looked dead while
it was answering the whole time.

Verified on an E1M-AEN803 (serial `2026W36-0003`) across 48 cold-cycled runs:
`GET_VERSION` reports `protocol v4.0 ... match`, `fw_version=0x0800` read back
over `GET_DIAG_INFO`, and `PING`, `GET_MAC`, `GET_CAPABILITIES`,
`WIFI_SCAN_START` and `BLE_ENABLE` all return `0`.

**`WIFI_CONNECT_STA` (0x12) IS NOT VERIFIED IN THIS RELEASE. Do not assume it
works.** The verification list above is exhaustive — `PING`, `GET_VERSION`,
`GET_MAC`, `GET_CAPABILITIES`, `WIFI_SCAN_START`, `BLE_ENABLE` — and station
association is deliberately absent from it.

Seven cold-booted attempts against a WPA3 AP at -79 dBm on 2026-09-11 never
completed an association, and no bridge exchange succeeded for 95 seconds
afterwards, while `PING` had answered on the first attempt moments earlier.
The radio's own verdict was never readable, so whether the association itself
would have succeeded is still unknown.

Two structural findings from that investigation, neither fixed here:

- This image does not call `cc3501e_hw_wifi_boot_start()` (see `src/main.c`),
  so the STA role is NOT brought up at boot. A `WIFI_CONNECT_STA` issued as
  the first radio operation therefore carries `Wlan_Start`, a `Wlan_Set` and a
  10 s `Wlan_RoleUp` *inside* the connect body, on top of the 30 s association
  wait and 10 s DHCP. Bounded pieces alone exceed 50 s, and four of those
  calls have no bound in our source at all.
- `cc3501e_hw_wifi_connect_sta()` skips the `bridge_transport_spi_hw_suspend()`
  bracket that the boot path uses around `Wlan_RoleUp`, on the stated premise
  that the role is "pre-cached at boot" — which the point above makes false on
  this image.

  **Adding that bracket has since been tried on silicon and it WEDGES THE
  LINK. Do not add it.** The obvious reading of the finding above is that
  `ensure_sta_role()` should quiesce the bridge around the role-up the way the
  boot path does. That change was written, built, signed and flashed to
  `2026W36-0003` on 2026-09-11, and it made things strictly worse: with it,
  `WIFI_SCAN_START` never completes and the bridge stays dead; without it, the
  same scan returns records.

  Measured as an A/B on one board, same host app, same host build, same core,
  same steps, only the CC3501E image differing:

  | | with the bracket | published v0.8.0 |
  |---|---|---|
  | `PING` after a 25 s silent window | never answered | ok, first attempt |
  | `cc3501e_wifi_scan()` | no records | `ALP_OK`, 4 records |

  The mechanism is in this repository already. `bridge_transport_spi_hw_suspend()`
  calls `SPI_transferCancel()`, which is the firmware's only call site for it,
  and on this image that call site is unreachable because `src/main.c` does not
  call `cc3501e_hw_wifi_boot_start()`. Bracketing `ensure_sta_role()` makes the
  cancel live for the first time in a shipped image. `hal/ti/cc3501e_hw_ti_ota.c`
  and `hal/ti/transport_hw_ti_spi.c` both already record that cancelling an
  armed callback-mode transfer from the bring-up task does not return, and the
  OTA pump replaced `suspend()` with a quiesce-then-release pattern for exactly
  that reason. A silent-bus experiment ruled out the obvious escape: even with
  the host issuing nothing at all for 25 s, the bracketed build stayed dead
  through ~70 s, so this is not a race against host traffic that quieting the
  host can avoid.

  If the role-up genuinely needs quiescing, the OTA pump's pattern is the shape
  to copy, not `suspend()`.

  **And the premise itself does not survive measurement.** The finding above
  reads as "the unguarded `Wlan_RoleUp` in the connect body is what wedges the
  link". It is not. `cc3501e_hw_wifi_scan_run()` performs the SAME
  `ensure_sta_role()` role-up, equally unguarded, and on this image a scan
  issued as the first radio operation of a boot completes and returns records,
  5 of 5 cold-booted runs. A `WIFI_CONNECT_STA` issued as the first radio
  operation still wedges, at WPA2 and WPA3 alike, and the `WIFI_STATUS` read
  after it times out too, so the radio's own verdict has never been readable.

  Scoped precisely, on `2026W36-0003`:

  | | `WIFI_SCAN_START` first | `WIFI_CONNECT_STA` first |
  |---|---|---|
  | published v0.8.0 | returns records | wedges |
  | v0.8.0 + the suspend bracket | wedges | wedges |

  So the bracket is a regression that additionally breaks the scan, and the
  connect wedge predates it and is untouched by it. Whatever wedges the connect
  is in the connect body AFTER the role-up, not the role-up.

Also note `Wlan_Disconnect()` now runs on every connect failure exit. It was
added after the last time station association was bench-proven, and it takes
no timeout parameter.

If you need station mode, call `GET_MAC` and `WIFI_SCAN_START` first — both
are verified — which moves `Wlan_Start` and the role transition out of the
connect body, and budget well beyond 40 s for the connect itself.

**Scan before you connect, and read the security kind off the scan rather
than assuming it.** On `2026W36-0003` a scan returns five networks, stable
across four cold-booted runs, at -74 to -92 dBm. Bluetooth advertisements on
the same board read -97 to -99 dBm, while a host Wi-Fi interface metres away
sits at -36 dBm.

**That gap is the ANTENNA DESIGN, not a fault.** This unit uses an ON-BOARD
antenna; earlier bring-up on this family used an external u.FL at J1. An
on-board antenna against a host laptop's is expected to read tens of dB down,
so roughly -75 dBm for a nearby AP is the normal operating point here, not
evidence of a broken RF path. An earlier version of this note read the gap as
pointing at "the antenna path on that unit", which overstated it.

What the levels DO mean for the connect path: the link budget is genuinely
tighter than an external-antenna bring-up, and WPA3-SAE costs extra round
trips over WPA2 (the SAE commit/confirm exchange plus PMF), so a marginal
signal bites a WPA3 association harder than a WPA2 one. Budget accordingly and
prefer the strongest AP available when testing association; do not read a
failed association at -83 dBm as proof of a firmware defect.

One caution learned the hard way there: a SINGLE scan is not enough to conclude
an AP is absent. One run on this board returned four records and omitted a
fifth that three later runs, plus a fourth from a different app, all reported at
a stable -83 dBm. Repeat a scan before concluding anything from what is missing
from it.

The scan also reports each record's security kind, and it is worth trusting
over an assumption: the AP this release's failed attempts targeted decodes as
WPA3, while every connect but one requested WPA2-PSK. Note the two encodings
differ -- the connect field is `0` open, `1` WPA2-PSK, `2` WPA3-SAE, while the
scan-result enum is `0` open, `1` WEP, `2` WPA, `3` WPA2, `4` WPA3 -- so read
the decoded name, never the raw number.

Treat the channel field of a scan record as the least trustworthy part of it:
across four runs here it moved between cold-identical runs while the matching
RSSI moved 2 dB, and it reported channel 1 for an AP whose name indicates 5 GHz.

**Known limitation, not a regression from v0.7.0.** The link can wedge during a
session: a transport desync after which every opcode fails until a cold cycle,
observed at roughly 7 runs in 24 with default host settings. It is host-side and
predates this release. alp-sdk provides two mitigations,
`CONFIG_ALP_SDK_CC3501E_POLL_GAP_MIN_MS` and
`CONFIG_ALP_SDK_CC3501E_POST_SUCCESS_GUARD_MS`; with them at 50 and 20 the
failure did not recur in 24 runs (Fisher one-sided p = 0.0047, 95% upper bound
on the residual rate 11.7%). Both default to a no-op, so a host that wants the
mitigation must opt in. A large `STREAM_WRITE` also wedges the link — 64 B and
256 B pass, 1024 B and 4092 B do not — and that is still open.

## v0.7.0

Three landings since v0.6.0, none of which were in a shipped artifact until now.

**Listening sockets** (#104, PR #108). `SOCK_BIND` (0x64), `SOCK_LISTEN` (0x65)
and the `EVT_SOCK_ACCEPTED` event, so a host can serve over the soft-AP instead
of only dialling out. The accept slot is reserved before the socket goes
passive, so an inbound connection cannot arrive with nowhere to land.

**The wire is versioned MAJOR.MINOR** (PR #109, alp-sdk ADR 0033). This replaces
the flat counter that went 5 -> 9 in a week and told a customer nothing about
whether their host still worked. MAJOR gates the link; MINOR is additive and
never refuses. `protocol-version.txt` reads `3.1`, and `GET_VERSION` answers
`0x0301`. `CMD_GET_CAPABILITIES` (0x06) reports which opcode families this build
actually implements as a 12-bit map, so a host discovers features instead of
inferring them from a version number.

**Two defects fixed** (PR #112):

- `ble enable` on an already-enabled NimBLE host tore the bridge SPI slave down
  and re-opened it *twice* — `worker_run_pending()` adds a second reinit after
  the body — for a call that does no work at all. Not a rare path:
  `cc3501e_nimble_host_disable()` never stops the host stack, so after the first
  enable in a boot **every** later one took it. Measured on the v0.6.0 image:
  over 120 s, and repeatedly a wedged link. Now the idempotent branch returns
  without touching the HIF. (`alplabai/alp-sdk#82`.)
- `GET_DIAG_INFO` could not tell a host the companion had rebooted. `uptime_ms`
  came from `ClockP_getSystemTicks()`, which survives a warm `NVIC_SystemReset()`
  on this silicon, so it counted from power-on and ran straight through a reboot;
  `reset_cause` said `power-on` after a software reset, because
  `PowerWFF3_getResetReason()` answers `PowerWFF3_RESET_PIN_POR` for a
  `SYSRESETREQ`. `uptime_ms` now subtracts a boot baseline taken in
  `cc3501e_hw_init()`, and the reset path records an address-guarded
  magic/inverse marker in `.TI.noinit` that `cc3501e_hw_reset_cause()` reads and
  clears once per boot. No wire change. (`alplabai/alp-sdk#111`.)

**Requires a wire-3.x host.** `cc3501e_reset()` reads `GET_VERSION` on every cold
boot and refuses a MAJOR mismatch, so flashing this blob under a host older than
the ADR 0033 change takes the companion link down by design. The paired host
work is on alp-sdk `dev`.

- `cc3501e-v0.7.0.bin`        -- wrapped TI `flash-images-builder` vendor image,
  signed in-band with the Alp Lab VALIDATION key. 1103236 bytes.
- `cc3501e-v0.7.0.bin.sha256` -- `2d7cdb515a7cc7fff2089eb90ade9dc91ec00b9d3fec8f15649865e61f07a27e`
- `cc3501e-v0.7.0.bin.sig`    -- detached **ECDSA-P256/SHA-256** signature,
  verify with `openssl dgst -sha256 -verify keys/alp_cc3501e_vendor_VALIDATION_public.pem
  -signature prebuilt/cc3501e-v0.7.0.bin.sig prebuilt/cc3501e-v0.7.0.bin`
- GPE stamp: **`0.149.92.0`** (major 0, every field <= 255).  Bench work for
  `#82`/`#111` flashed the E1M-AEN801 at `0.149.90.0` and then `0.149.91.0`, which
  left the v0.6.0 artifact (stamped `0.149.90.0`) unflashable on our own hardware
  — the same situation v0.6.0 itself hit and records below. `gpe-floor:` in
  `prebuilt/BUILT_FROM` moved `0.149.90.0` -> `0.149.92.0` in the same commit,
  because raising the floor without re-wrapping reddens
  `check_prebuilt_kind.py` on the newest blob.
- stage-1 raw image sha256: `baa8a7b723435328fdc855267fa128e3e87d66f901c6fa233d61e5fc311dda90`, 1097572 bytes
- built from `e5aac7ff992d52802f63be576e8aeada82f8f95a` (`main`)

Three distinct version numbers, still not interchangeable: app SemVer **0.7.0**,
wire protocol **3.1**, GPE image stamp **0.149.92.0**. `GET_DIAG_INFO` reports
`fw_version=0x0700`; `GET_VERSION` answers `0x0301`.

> **NOT YET FLASH-VERIFIED AS AN ARTIFACT.** The source at
> `e5aac7ff992d52802f63be576e8aeada82f8f95a` is bench-proven on an E1M-AEN801 — the
> `#82` and `#111` evidence in PR #112 was taken from that exact tree, wrapped at
> `0.149.91.0`. This *artifact*, wrapped at `0.149.92.0` with
> `firmware-version.txt` bumped to 0.7.0, has **not** been written to a part: the
> XDS110 was disconnected from the bench before the programming run started (no
> `programming_report.txt` was produced, and the unit was confirmed still running
> the previous image at `uptime: 2576174 ms`). The outstanding check is one flash
> plus a cold cycle, confirming `fw 0.7.0` / `fw: 0x0700 (v7.0)` and `wire 3.1`.

## v0.6.0

Wire **protocol 8**: request identity for every worker-routed opcode
(issue #102, PR #103). Flags bits 3..7 of the request header carry a 5-bit
retry seq, so a reply lost in transit can no longer make the host's
poll-by-repeat look like a new request and re-execute the operation. Zero wire
bytes added. `SOCK_SEND` keeps its own 8-bit struct seq and is exempt, as is
`SOCK_RECV` (both consume stream state); header seq 0 is reserved to mean "no
identity", so a pre-v8 host can never be answered from the latch.

`DIAG_GET_STATS` (0x70) grew additively 8 -> 16 bytes, adding `worker_execs`
and `retry_latch_hits` (both LE32). An old host reading only the first 8 bytes
is unaffected.

**Requires a protocol-8 host.** A protocol-8 firmware and a protocol-7 host
refuse each other at `GET_VERSION` by design; the paired alp-sdk change is
alplabai/alp-sdk#1891 and must land after this ships.

- `cc3501e-v0.6.0.bin`        -- wrapped TI `flash-images-builder` vendor image,
  signed in-band with the Alp Lab VALIDATION key. 1099784 bytes.
- `cc3501e-v0.6.0.bin.sha256` -- `51e78dc4f9a3285c4a046b2f866188dfd790902879330fc0a4c2091c4de0272d`
- `cc3501e-v0.6.0.bin.sig`    -- detached **ECDSA-P256/SHA-256** signature,
  verify with `openssl dgst -sha256 -verify keys/alp_cc3501e_vendor_VALIDATION_public.pem
  -signature prebuilt/cc3501e-v0.6.0.bin.sig prebuilt/cc3501e-v0.6.0.bin`
- GPE stamp: **`0.149.90.0`** (strictly above `0.149.80.0`, the highest stamp this
  bench unit has been flashed at; major 0, every field <= 255).  The blob was
  first cut at `0.149.76.0`; bench work for issue #18 then iterated the unit to
  `0.149.78.0`, which would have left the shipped artifact unflashable on our own
  hardware, so it was re-wrapped from the identical stage-1 `.out` (raw-sha256
  unchanged) at a stamp that clears the new floor.
- stage-1 raw image sha256: `973879585648343b13f726069c553661679d316a479f4a4d00ee7e53f339fa42`, 1094120 bytes

**Bench-verified on E1M-AEN801** (`AE822FA0E5597LS0` Rev A0, module rev r1,
serial `2617-0001`), against a protocol-8 host:

```
[CTRL ] no drop:  execs +1  latch +0
[DROP] discarding OK reply for cmd 0x03, re-asking seq 11
[LATCH] mac_rc=0  execs 10->11 (+1)  latch 0->1 (+1)
```

One deliberately dropped reply -> the operation executed **once** and the retry
was served from the latch, versus `execs +2` without the mechanism. Link
stability across the same run: `frames_ok` 161 -> 541 with `frames_err` frozen
at 31, i.e. 380 consecutive OK frames with the seq riding in the flags byte and
no framing regression.


## [0.5.1] — 2026-08-31

> **Re-cut from 0.6.0, before distribution.**  This content shipped briefly as
> **0.6.0**, which burned a minor version it did not need to.  0.5.0 had never
> been tagged or released either — `gh release list` and the tag list are both
> empty for this repository — so the correct step was a **patch** bump, not a
> minor.  That is the same rule 0.5.0's own re-cut applied a day earlier; it
> simply was not applied here.  The three `cc3501e-v0.6.0.*` files are deleted
> and nothing outside this repository ever referred to them.
>
> **0.5.1, not a second 0.5.0.**  The `fw_version` marker is stamped from
> `firmware-version.txt` at build time, so reusing `0.5.0` would have given a
> **third** distinct build reporting `0x0500` — the 2026-08-29 protocol-5 build,
> the 2026-08-30 protocol-6 re-cut, and this protocol-7 one.  A patch bump costs
> nothing and keeps the marker meaningful: this artifact reports **`0x0501`**,
> which no other build does.
>
> **The GPE stamp is `0.149.74.0`, and the "follow the SemVer" convention is
> the thing that was wrong.**  0.4.0 stamped `0.4.0.0`, 0.4.1 stamped
> `0.4.1.0`, 0.5.0 stamped `0.5.0.0`, and an earlier cut of this release
> stamped `0.5.1.0` to match them.  **That convention is broken by
> construction.**  The GPE stamp is the CC35 SBL's **anti-rollback** field and
> it is enforced *per part, against that part's last-seen version* — even when
> every `*_rollback_protection_*` fuse reads `0`.  A unit that has ever been
> flashed at a `0.149.x` stamp will therefore **never** accept a `0.5.x.0`
> artifact again: it streams clean, exit 0, the full ~1.09 MB, and then the SBL
> refuses to boot it.  Dead link, empty XDS110 `query` table.
>
> It is not a documentation problem that a `STOP` block can carry.  Every bench
> or OTA-iterated unit climbs into the `0.149.x` range through
> `ti/regen_flashset.sh`, and the shipped artifact is then permanently
> unflashable on it without re-wrapping — which needs the signing assets a
> customer does not have.  0.6.0's `0.149.70.0` was the pragmatic choice, not
> the deviation.
>
> So this release stamps **`0.149.74.0`**: above the `0.149.70.0` this bench
> part was last seen at, and above the `0.149.73.0` a verification run flashed
> during the attempt to confirm it.  `major` stays `0` — a GPE major `>= 1`
> fails BL2 secure-boot with `AUTH_ERROR`.
>
> The firmware bytes are unchanged by this.  Re-stamping re-wraps the same
> `.out`: only the header hash, the four stamp bytes at file offset 36, and the
> in-band signature differ.

**Wire protocol 6 -> 7.** `CMD_SOCK_SEND` gains request identity: byte 3 of
`alp_cc3501e_sock_send_t` — previously `reserved`, always written 0 — is now a
`seq` the host assigns once per logical send. `handle_sock_send()` serves a
matching-seq retry from a 4-byte static cache instead of re-submitting the job.

Why it matters: alp-sdk's `poll_by_repeat()` re-sends the identical frame on
every poll, and `handle_worker_routed_payload_reply()`'s `WORKER_IDLE` edge
submits whatever it is handed. Once a send completed and the worker slot was
freed, the host's next poll was read as a NEW request and **the payload was
transmitted again** — so a single lost `RESP_OK` became an endless re-execute
and the host reported a timeout on a send that had succeeded. Root cause of
alp-sdk#1746; see issue #88.

**The protocol bump is semantic, not structural.** The frame layout is
unchanged. An OLD host writes `reserved = 0` on every request, so this firmware
reading byte 3 as a seq would see `seq == 0` forever and could serve a cached
reply for a genuinely new send. The version gate is what stops new firmware
misreading an old host. Host half: alp-sdk#1872.

Bench-measured on E1M-AEN801 serial 2617-0001 with both halves flashed, against
a bare `accept()` listener, 10 trials across two runs:

- full end-to-end successes (reply body retrieved): **6 of 10** — previously
  none confirmed across three runs on protocol 6.
- `-4` timeouts on completed sends: **zero in 10** — previously frequent.
- request reached the peer: **all 10**.

**The remaining 4 of 10 are now fixed too, host-side, and nothing here
changed for it.** When 0.6.0 was cut, `send failed (-1)`
(`RESP_ERR_INVALID`) still occurred on roughly 4 of 10 trials on transfers the
peer received in full, and it was recorded as a different, unexplained defect.
It has since been root-caused: the host's blind `CC3501E_PHASE_SETTLE_US` gap
between the request header and payload transfers was 40 us, which
intermittently out-ran this side's DMA re-arm, so the payload genuinely landed
short. **This firmware's strict length check was correct throughout** —
`req_len != sizeof(alp_cc3501e_sock_send_t) + data_len` was rejecting a frame
that really was incomplete, and loosening it would have accepted truncated
sends. Raising the host gap to 250 us (alp-sdk#1873) takes the same 10-trial
measurement to **10 of 10 end-to-end, zero `send failed`**. Nothing in
`src/protocol_sockets.c` was changed for it. alp-sdk#1746 and issue #90 are
both closed.

Two limits are still open and are not addressed by either half: socket **RX**
stalls at roughly 2 kB, and 250 us is an empirical upper bound paid per payload
phase, so it taxes anything that streams (alp-sdk#1677).

`SOCK_RECV` deliberately gets no seq: `alp_cc3501e_sock_recv_t` is
`{ handle, max_len }` and `max_len`'s high byte occupies offset 3, so there is
no spare byte and putting one there would be a real layout change.

Built with the `ti` backend (TI `ticlang` 5.1.1 + SimpleLink Wi-Fi SDK
10.10.01.08 + SysConfig 1.28.0), `build_ti.ps1 -Ble`, `0 error(s)` — then
wrapped as a TI `flash-images-builder` vendor image and signed with the Alp
Lab VALIDATION key at GPE stamp `0.149.74.0` (below). This artifact is the
**wrapped** kind, not the raw `build_ti.ps1` output; see
[`BUILD_RECIPE.md`](BUILD_RECIPE.md) for the exact two-stage recipe and the
byte-level evidence that confirms it (issue #94 — the raw output alone does
not reproduce this file, by design, and was never meant to). The `prebuilt
integrity` job now machine-checks that kind against `BUILD_RECIPE.md` rather
than trusting the prose (#97).

```
size    : 1099396 bytes
sha256  : d58d78a6697ea6d74c12ee51912dfb0434214f4244add2e6af03928919b0eb9f
marker  : fw_version 0.5.1 -> 0x0501
wire    : protocol 7
GPE     : 0.149.74.0   (anti-rollback stamp, enforced per part against that
                        part's last-seen version.  ONE field -- the artifact's
                        stamp IS the flash-set stamp; they are not separate.
                        Chosen above 0.149.73.0, the highest this bench part
                        has seen.  major MUST stay 0)
kind    : wrapped (TI flash-images-builder vendor_image, signed in-band)
```

**Not yet confirmed booting.** The AEN801 bench unit stopped booting on
2026-08-31 — SEROM cannot load SERAM from MRAM `0x0005ffff`, 0 of 10 cold
cycles (alp-sdk#1883) — and the Alif host is what releases the CC35 from
reset, so `alp companion` is unreachable and no post-flash check can run. The
image was written cleanly (all five transfers, `Saved report image bin file`),
its signature verifies, and its kind is machine-checked; what is **not**
established is that a part boots it. Treat this release as unverified on
silicon until that unit is repaired or replaced.

- `cc3501e-v0.5.1.bin`         -- signed vendor image (the wrapped kind; this
  is what `primary_vendor_image.sign.bin` expects, so the README recipe's `cp`
  is correct for it, which it was not for the raw 0.5.0 artifact -- issue #96)
- `cc3501e-v0.5.1.bin.sig`     -- detached **ECDSA-P256/SHA-256** signature
  (the VALIDATION vendor key -- a bench-grade artifact, not production-key)
- `cc3501e-v0.5.1.bin.sha256`  -- SHA-256 manifest

## [0.5.0] — 2026-08-30

> **Re-cut on 2026-08-30, before distribution.**  An earlier 0.5.0 artifact
> (`sha256 980db6c9...`, wire protocol 5) was built on 2026-08-29 and never
> left this repository, so the version number was reused rather than burned.
> The blob, its signature and its manifest under `prebuilt/` are the re-cut
> ones; nothing carrying the earlier bytes was ever released.
>
> **The re-cut changes the wire protocol under an unchanged app SemVer.**  A
> part flashed with the earlier 0.5.0 reports the same `fw_version=0x0500` as
> this one while speaking protocol **5**, so that marker no longer separates
> them.  `GET_VERSION` does, and it is the check that matters: the host refuses
> a protocol mismatch outright.  Use it, not `fw_version`, to identify what a
> part is running.

Built with the `ti` backend (TI `ticlang` 5.1.1 + SimpleLink Wi-Fi SDK
10.10.01.08 + SysConfig 1.28.0), `build_ti.ps1 -Ble`, `0 error(s)`.

```
size    : 1093668 bytes
sha256  : dee3c34c594ca14c0af5899b61a73b022ac184adef7670d3cfed2bf04dd8b8d9
marker  : fw_version 0.5.0 -> 0x0500
text    : 1092920
```

- `cc3501e-v0.5.0.bin`         -- signed firmware image (full shipped stack)
- `cc3501e-v0.5.0.bin.sig`     -- detached **ECDSA-P256/SHA-256** signature
  (the VALIDATION vendor key -- a bench-grade artifact, not production-key;
  verify with `openssl dgst -sha256 -verify keys/alp_cc3501e_vendor_VALIDATION_public.pem -signature
  cc3501e-v0.5.0.bin.sig cc3501e-v0.5.0.bin`)
- `cc3501e-v0.5.0.bin.sha256`  -- integrity manifest (`dee3c34c594ca14c0af5899b61a73b022ac184adef7670d3cfed2bf04dd8b8d9`)

| Number | Value | What it gates |
|---|---|---|
| App SemVer (`firmware-version.txt`) | **0.5.0** | the `fw_version` marker the bridge reports in `DIAG` (`0x0500`) |
| Wire protocol (`ALP_CC3501E_PROTOCOL_VERSION`) | **6** | `GET_VERSION`; the SPI1 passthrough opcodes bumped it from 5, so a 0.4.x or pre-re-cut host is refused |
| GPE image `--version` (this artifact's stamp) | **0.5.0.0** | the version the SBL compares against the part's last-seen version |

> **Anti-rollback still applies to this artifact.** The `0.5.0.0` stamp is far
> BELOW what a bench or OTA-iteration unit has already seen -- the bench part
> behind these results sits at `0.149.71.0`. Flashing `0.5.0.0` onto such a unit
> streams clean and then refuses to boot. Re-wrap the same image at a legal stamp
> (`VERSION=0.149.72.0 ti/regen_flashset.sh`, major MUST be `0`, every field
> <= 255). `README.md` and `BRINGUP_STATUS.md` carry the full rule.

**Bench-verified on an E1M-AEN801** (flashset `01496637`): 6/6 cold-cycle trials,
7/7 functional surfaces, **0 boot failures, 0 failures** — `ver`, `wifi scan`,
`ble enable`, `ble scan`, `ble scan-stop`, `ver` after the radio op, and the
GPIO proxy.

**The re-cut was re-verified on the same board** (2026-08-30, re-wrapped at
`VERSION=0.149.71.0` because the `0.5.0.0` stamp is far below that part's
history): full 144676 / 240 / 1340 / 12 / 1099332-byte programming run, 20 s
cold cycle, then bring-up `0`, `PING ok after 1 attempt`, `GET_VERSION ->
protocol v6 (host expects v6) -- match`, and the SPI1 group end to end
(`configure -> 0 (actual_hz=0 max_xfer=4088)`, a `9F` RDID transfer, a `05`
RDSR under CS_HOLD, a read under that hold, and a release that is idempotent on
repeat).

Minor bump rather than patch: this changes observable behaviour, not only
defects.

### Fixed

- **Socket EOF was unreachable.** `SOCK_RECV` answered `RESP_ERR_BUSY` forever
  after a peer close: the pump returns early on `peer_closed`, so the ring could
  never refill, and the empty-ring answer never consulted it. `poll_by_repeat`
  retries precisely on `BUSY`, so it span to timeout instead of seeing the
  0-byte close.
- **`WIFI_DISCONNECT` had no SPI re-sync at all.** It sat in the
  `body_already_reinit` skip list while `cc3501e_hw_wifi_disconnect()` contained
  no re-init — a `Wlan_Disconnect()` with no re-sync from either side.
- **`OPEN_DRAIN` was push-pull.** It drove the net HIGH, contending with any
  other driver pulling low. Now uses the pad's output-disable: assert LOW,
  release Hi-Z. A later fix corrected a pad-mask that aliased pads 32..37 onto
  0..5 (`GPIO_pinUpperBound` is 37, not "well under 32").
- **BLE `scan-stop` wedge.** Removed a `bridge_transport_spi_hw_reinit()` that
  entered the tree as a Wi-Fi `HIFInit` fix and was pattern-copied onto the BLE
  path. Bench: 3/32 scan-stop failures with it, 1/104 without.
- **A rollback GPE stamp default** in `ti/validate_gpio_bench.ps1`, the one
  script that reaches the part.
- **`DIAG_LOG_LEVEL` (0x71) discarded its argument** while answering `RESP_OK`.
- **Real reset cause** reported instead of a hardcoded `RESET_UNKNOWN`.
- **Both AP-role waits bounded** instead of `WLAN_WAIT_FOREVER`.

### Added

- ADC internal temperature channel instantiated (pinless) and converting;
  **raw code only, no unit** — the trip awaits TI's transfer function.
- Bench-probe capture of host-DMA channel state, raw SPI `RIS`, SoC `ERRSRIS`,
  and the watchdog registers.
- **The E1M connector's SPI1 relayed through the bridge** (#83): opcodes `0x55`
  / `0x56` / `0x57` (SPI1 CONFIGURE / TRANSFER / RELEASE) and the CC3501E SPI1
  master HAL behind `CC3501E_WIFI`.  This is what took the wire protocol to
  **6**.
- `ti/build_ti.ps1` no longer aborts when SysConfig emits a warning (#85).
  `CONFIG_SPI_1` added four pin warnings, and `$ErrorActionPreference = 'Stop'`
  turned the `0 error(s), 8 warning(s)` summary on stderr into a terminating
  error, so every build died before the compiler ran while a stale `.bin` sat
  in `build/ti/` looking flashable.  No effect on the image.

### Known limitations

- The residual `scan-stop` wedge is ~1% (1/147 booted trials), not zero.
- Cold-boot failures run ~20% on this part (PY25Q64LB Puya); the host
  hard-reset workaround is in `alp-sdk`'s `cc3501e_reset()`.

## [0.4.1]

Built with the `ti` backend (TI `ticlang` 5.1.1 + SimpleLink Wi-Fi SDK
10.10.01.08 + SysConfig 1.28.0 + Wi-Fi toolbox 4.2.4) via
`ti/build_ti.ps1 -Ble -AlpSdkRoot <alp-sdk>`.

- `cc3501e-v0.4.1.bin`         -- signed firmware image (full shipped stack)
- `cc3501e-v0.4.1.bin.sig`     -- detached **ECDSA-P256/SHA-256** signature
  (the VALIDATION vendor key -- a bench-grade artifact, not production-key;
  verify with `openssl dgst -sha256 -verify keys/alp_cc3501e_vendor_VALIDATION_public.pem -signature
  cc3501e-v0.4.1.bin.sig cc3501e-v0.4.1.bin`)
- `cc3501e-v0.4.1.bin.sha256`  -- integrity manifest (`7f550c79502f6b001f1e651d1229f61a0878cd29b78d33dae6621021148f324f`)

| Number | Value | What it gates |
|---|---|---|
| App SemVer (`firmware-version.txt`) | **0.4.1** | the `fw_version` marker the bridge reports in `DIAG` (`0x0401`) |
| Wire protocol (`ALP_CC3501E_PROTOCOL_VERSION`) | **5** | `GET_VERSION`; unchanged from 0.4.0, so a 0.4.0 host talks to this image |
| GPE image `--version` (this artifact's stamp) | **0.4.1.0** | the version the SBL compares against the part's last-seen version |

> **Anti-rollback -- read before flashing this artifact.** The stamp must be
> monotonically **>=** anything ever flashed on that unit, and the CC35 SBL
> enforces that **even when every `*_rollback_protection_*` fuse reads `0`**.
> A warm programming run burns no fuses, so an all-zero fuse report looks
> permissive and is not. A stamp below the unit's last-seen version streams
> clean (exit 0, the full ~1.09 MB) and then fails to boot: dead link, empty
> XDS110 `query` image table. Any unit used for OTA or flash iteration is far
> above `0.4.1.0` -- re-wrap this same image at a legal stamp
> (`VERSION=<higher> ti/regen_flashset.sh`) rather than assuming the binary is
> bad. `major` must stay `0`: a GPE major `>= 1` fails BL2 secure-boot with
> `AUTH_ERROR`.

**The soft-AP now accepts clients (alp-sdk#1562).** `cc3501e_hw_wifi_ap_start()`
built its role-up command as a zero-initialised `RoleUpApCmd_t` and filled only
`ssid`, `channel` and `secParams`. That left `sta_limit` at **0** -- a field TI's
header describes as "limits the number of stations that the AP's has", so the AP
was configured to admit **zero** clients. TI's own reference filler
(`ParseRoleUpApCmd()`) defaults it to 4 and clamps anything outside `[1, 8]` back
to 4, so 0 was never a legal value; it was simply the uninitialised value reaching
the NWP.

The AP beaconed perfectly throughout, which is why this survived so long: with one
radio a broken AP and a working one look identical. Measured with a second radio
(an Intel AX200 driven as a real client), from a cold boot:

| | 0.4.0 | 0.4.1 |
|---|---|---|
| WPA2 association | 0 of 16 attempts over 275 s | associated (t+13s, t+20s across runs) |
| Open association | 0 of 12 attempts over 180 s | associated at t+13s |
| firmware `wifievt` | frozen at 3 (role-up only) | 4 -- the station event |

Three sibling fields with the same zero-is-a-real-setting problem are now set from
TI's reference too: `countryDomain` (the `"00"` world domain), `sae_pwe`, and
`sae_anticlogging_threshold` (0 is `SAE_ANTI_CLOGGING_ALWAYS`, not "unset"). The
latter two affect the WPA3 security type only.

**Known limitation, unchanged:** a second `wifi ap` on a device already in AP role
does not take -- association fails until the part is cold-cycled. `ap-stop` is NOT
a workaround: it runs `WIFI_AP_STOP` in the SPI ISR and wedges the bridge
(alp-sdk#1564). Cold-cycle between AP experiments.

**Also unchanged from 0.4.0:** `ap start` still returns `-4 unconfirmed` even when
the AP is fully serviceable, because `WIFI_AP_START` has no status latch to confirm
against (alp-sdk#1385). And **sockets still do not connect** (alp-sdk#1746) -- that
is untouched by this release; do not read "0.4.1 fixes Wi-Fi" as covering sockets.

Everything else is identical to 0.4.0: same wire protocol, same host driver
contract, no ABI change.

## [0.4.0]

Built with the `ti` backend (TI `ticlang` 5.1.1 + SimpleLink Wi-Fi SDK
10.10.01.08 + SysConfig 1.28.0 + Wi-Fi toolbox 4.2.4) via
`ti/build_ti.ps1 -Ble`.

- `cc3501e-v0.4.0.bin`         -- signed firmware image (full shipped stack)
- `cc3501e-v0.4.0.bin.sig`     -- detached **ECDSA-P256/SHA-256** signature
  (the VALIDATION vendor key -- a bench-grade artifact, not production-key;
  verify with `openssl dgst -sha256 -verify keys/alp_cc3501e_vendor_VALIDATION_public.pem -signature
  cc3501e-v0.4.0.bin.sig cc3501e-v0.4.0.bin`)
- `cc3501e-v0.4.0.bin.sha256`  -- integrity manifest (`a73e0555841e90522eaa4c007445a4f08331092842267be200fd0c895ed4881a`)

**THREE version numbers, and they are not interchangeable.** Conflating them
has cost bench time repeatedly, so this release records all three:

| Number | Value | What it gates |
|---|---|---|
| App SemVer (`firmware-version.txt`) | **0.4.0** | the `fw_version` marker the bridge reports in `DIAG` (`0x0400`) |
| Wire protocol (`ALP_CC3501E_PROTOCOL_VERSION`) | **5** | `GET_VERSION`; a host on another version is refused outright |
| GPE image `--version` (this artifact's stamp) | **0.4.0.0** | the vendor-RoT **anti-rollback** floor burned into the part |

**Wire protocol 4 -> 5.** Adds `OTA_UPDATE_MODE` (opcode `0x47`): asks the
device to reboot into (or out of) update mode. A host built from this tree
expects **5** and will refuse the previous `cc3501e-v0.3.0.bin`, which answers
**4** -- upgrading host and companion is not optional across this boundary.

Also in this release, relative to 0.3.0:

- OTA over the bridge: 1373 s -> 8 s, and 16 defects fixed including a silent
  image splice (#1610, #1655).
- `PROMOTE` is now the sole OTA commit, confirmed from flash (#1123, #1714).
- Async events fan out to every subscriber instead of a single callback that
  one consumer could steal (#1724). The slave->master attention edge on the
  READY wire also landed (#130, #1721) -- but the firmware half is a
  **build-time opt-in** (`build_ti.ps1 -AttnPulse`, `-DCC3501E_ATTN_PULSE=1`),
  default OFF because the wire is a rev-1 bodge absent on the stock EVK, and
  `package_cc3501e_prod.ps1` does not pass it. Whether THIS artifact carries
  the pulse is unverified; assume it does not until an edge is observed on
  silicon, in which case a host that arms attention falls back to its timer
  poll. The host-side fan-out is unconditional either way.
- `WIFI_AP_START` has a real success path (#1696, #1709); a failed connect no
  longer leaves a stale association, and RSSI is validated (#1703).
- The Wi-Fi radio is driven from the power presets, on the task, with the
  result actually reported (#1681).
- The TI build no longer ships a stale image after a failed build (#1722,
  #1726) -- this one silently invalidated earlier bench results, because a
  build that died mid-compile left the *previous* `.out` flashable.

**Anti-rollback -- read this before flashing:**

- This artifact is stamped **`0.4.0.0`**, derived from the app SemVer with
  `major = 0`. A GPE major `>= 1` fails BL2 secure-boot with `AUTH_ERROR`.
- The stamp must be **monotonically >=** anything ever flashed on that part.
  A part whose floor is already higher (any unit used for OTA testing) will
  **refuse this artifact**: re-wrap the same `.out` at a legal stamp rather
  than assuming the binary is bad. Only the stamp differs.
- **The `--version` you build with must match the version stamped into the
  signed `programming_instructions`** in the flash-set, or the programmer
  reports success and the device silently keeps the old image.
- A rollback attempt streams clean and then refuses to boot. That reads as a
  dead part; it is not -- it is the anti-rollback gate doing its job.

**Do not derive a stamp from `regen_flashset.sh`'s epoch scheme for a
release.** `0.$(((e>>16)&255)).$(((e>>8)&255)).$((e&255))` wraps its high byte
every ~194 days, so it is *not* monotonic over a part's life: run today it
yields `0.144.51.196`, which is **below** a floor of `0.149.63.0` already
burned into this bench's unit. It is fine for same-day bench iteration, which
is all it claims; it is not a release scheme.

## [0.3.0]

Built on the bench with the `ti` backend (TI `ticlang` 5.1.1 +
SimpleLink Wi-Fi SDK 10.10.01.08 + SysConfig 1.28 + Wi-Fi toolbox
4.2.4) via `ti/build_ti.sh --wifi --ble`.

- `cc3501e-v0.3.0.bin`         -- signed firmware image (full shipped stack)
- `cc3501e-v0.3.0.bin.sig`     -- detached **ECDSA-P256/SHA-256** signature
  (the VALIDATION vendor key -- a bench-grade artifact, not production-key;
  verify with `openssl dgst -sha256 -verify keys/alp_cc3501e_vendor_VALIDATION_public.pem -signature
  cc3501e-v0.3.0.bin.sig cc3501e-v0.3.0.bin`)
- `cc3501e-v0.3.0.bin.sha256`  -- integrity manifest

**Wire protocol 3 -> 4.** Adds `OTA_PROMOTE` (opcode `0x46`): arms the
deferred swap-reboot for an image already committed to STAGED, without
opening a new OTA session. Before it existed there was no non-destructive
way to clear *or* promote a committed STAGED image over the bridge --
`OTA_ABORT` cancels only an in-flight session, not one already committed --
so a unit left with a stuck STAGED image (e.g. a bare reset mid-swap) had no
bridge-side recovery. `OTA_PROMOTE` must be present in the CC35's *running*
firmware to help: it cannot rescue a part that is already stuck on
pre-v0.3.0 firmware, only prevent the wedge going forward.

Fixes since [0.2.0]:

- Worker-route `WIFI_AP_STOP` off the SPI ISR -- the inline call killed the
  SPI-slave DMA and nothing re-armed it (#1564).
- Stop reporting the unpopulated `WIFI_STATUS` latch byte as an RSSI
  measurement (#1387, #1420).
- Implement the wire-version refusal `DESIGN.md` always claimed -- a host on
  an incompatible protocol version is now actually rejected instead of
  silently served (#1371, #1421).
- `OTA_BEGIN` clears a stuck STAGED slot and recovers an ambiguous/failed
  slot instead of bailing, and surfaces the swap-reboot refusal rc so the
  host can distinguish "refused" from "failed" (#493).
- Dynamic BLE GATT service registration (`BLE_GATT_REGISTER`, 0x38)
  replaces the fixed-demo-service stub, plus the follow-up distinct-status
  fix for register-while-advertising (#480, #892, #895).
- The TI SPI transport left the slave permanently unarmed after a failed
  `SPI_transfer()` arm; a failure counter now drives the same full SPI
  re-open recovery the existing resync path uses (#1133).

**KNOWN DEFECT -- #1562 is OPEN:** `wifi ap` brings up a soft-AP that stops
advertising after roughly 100 seconds. Station mode, scan, MAC, BLE, sockets
and OTA are unaffected; this is soft-AP advertising only.

**Anti-rollback -- read this before flashing over a link:**

- The OTA payload's signed version must **exceed** the running primary (the
  `psa_fwu` install-time gate); a downgrade is refused, not silently
  accepted.
- Over SWD, the GPE programming version must be **monotonically >=** anything
  ever flashed on that part, with **major = 0** -- a GPE major >= 1 fails
  BL2 secure-boot `AUTH_ERROR`.
- A rollback attempt streams clean and then refuses to boot. That reads as a
  dead part; it is not -- it is the anti-rollback gate doing its job.

## [0.2.0] - 2026-07-09

First signed prebuilt, built on the bench with the `ti` backend (TI
`ticlang` 5.1.1 + SimpleLink Wi-Fi SDK 10.10.01.08 + SysConfig 1.28 +
Wi-Fi toolbox 4.2.4) via `ti/build_ti.sh --wifi --ble`.

- `cc3501e-v0.2.0.bin`         -- signed firmware image (full shipped stack)
- `cc3501e-v0.2.0.bin.sig`     -- detached **ECDSA-P256/SHA-256** signature
  (the VALIDATION vendor key; verify with
  `openssl dgst -sha256 -verify keys/alp_cc3501e_vendor_VALIDATION_public.pem -signature cc3501e-v0.2.0.bin.sig
  cc3501e-v0.2.0.bin`)
- `cc3501e-v0.2.0.bin.sha256`  -- integrity manifest
  (`1dffbc30a306c5227578640d0a60b044edff1be38747ae9e57776d6b0989e9f4`)

Feature scope: the full CC3501E bridge — META (PING / GET_VERSION /
GET_MAC / RESET) + Wi-Fi station/AP + BLE (NimBLE) + sockets + OTA over
SPI. The signature is ECDSA-P256, not Ed25519 (the placeholder note in
earlier drafts was wrong — the VALIDATION vendor key is a P-256 EC key).

The AEN SoM presets' `helper_firmware.cc3501e_otp` now point
`firmware_path` at this blob; the CC3501E is never customer-flashed, so
there is no `flash_method` -- `update_channel: alp_ota_spi_otp` is the
whole story (see alp-sdk
[`metadata/e1m_modules/README.md`](https://github.com/alplabai/alp-sdk/blob/main/metadata/e1m_modules/README.md)).

**Full OTA cycle validated on hardware (2026-07-10):** stream → FINISH →
STAGED → the CC35's own `psa_fwu_request_reboot()` swap (the bridge drops,
then returns) → the swapped image runs and **self-accepts across a true
cold POR** (no rollback). Proven on the E1M-AEN801 EVK with a FORWARD
candidate — the OTA payload's signed version must EXCEED the running
primary (monotonic anti-rollback: a downgrade is refused at `psa_fwu`
install). A first OTA after a failed one recovers cleanly (no bridge
wedge, no CC35 reset). See `BRINGUP_STATUS.md` §5.
