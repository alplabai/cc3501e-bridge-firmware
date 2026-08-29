# CC3501E prebuilt firmware — release notes

Each entry corresponds to a tagged release of this repository's source.
The signed binary, its detached signature, and a SHA-256 manifest are
dropped into this directory and named `cc3501e-vX.Y.Z.bin` (matching
`firmware-version.txt` at the repo root).

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [0.5.0] — 2026-08-29

Built with the `ti` backend (TI `ticlang` 5.1.1 + SimpleLink Wi-Fi SDK
10.10.01.08 + SysConfig 1.28.0), `build_ti.ps1 -Ble`, `0 error(s)`.

```
size    : 1092384 bytes
sha256  : 980db6c9d5743581f68fe7a89119e06f29ff83273f6fe3ab723a496febc31109
marker  : fw_version 0.5.0 -> 0x0500
```

- `cc3501e-v0.5.0.bin`         -- signed firmware image (full shipped stack)
- `cc3501e-v0.5.0.bin.sig`     -- detached **ECDSA-P256/SHA-256** signature
  (the VALIDATION vendor key -- a bench-grade artifact, not production-key;
  verify with `openssl dgst -sha256 -verify <pub> -signature
  cc3501e-v0.5.0.bin.sig cc3501e-v0.5.0.bin`)
- `cc3501e-v0.5.0.bin.sha256`  -- integrity manifest (`980db6c9d5743581f68fe7a89119e06f29ff83273f6fe3ab723a496febc31109`)

| Number | Value | What it gates |
|---|---|---|
| App SemVer (`firmware-version.txt`) | **0.5.0** | the `fw_version` marker the bridge reports in `DIAG` (`0x0500`) |
| Wire protocol (`ALP_CC3501E_PROTOCOL_VERSION`) | **5** | `GET_VERSION`; unchanged from 0.4.x, so a 0.4.x host talks to this image |
| GPE image `--version` (this artifact's stamp) | **0.5.0.0** | the version the SBL compares against the part's last-seen version |

> **Anti-rollback still applies to this artifact.** The `0.5.0.0` stamp is far
> BELOW what a bench or OTA-iteration unit has already seen -- the bench part
> behind these results sits at `0.149.66.35`. Flashing `0.5.0.0` onto such a unit
> streams clean and then refuses to boot. Re-wrap the same image at a legal stamp
> (`VERSION=0.149.67.0 ti/regen_flashset.sh`, major MUST be `0`, every field
> <= 255). `README.md` and `BRINGUP_STATUS.md` carry the full rule.

**Bench-verified on an E1M-AEN801** (flashset `01496637`): 6/6 cold-cycle trials,
7/7 functional surfaces, **0 boot failures, 0 failures** — `ver`, `wifi scan`,
`ble enable`, `ble scan`, `ble scan-stop`, `ver` after the radio op, and the
GPIO proxy.

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
  verify with `openssl dgst -sha256 -verify <pub> -signature
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
  verify with `openssl dgst -sha256 -verify <pub> -signature
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
  verify with `openssl dgst -sha256 -verify <pub> -signature
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
  `openssl dgst -sha256 -verify <pub> -signature cc3501e-v0.2.0.bin.sig
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
