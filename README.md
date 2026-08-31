@page firmware_cc3501e_index CC3501E bridge firmware

# cc3501e-bridge

> **This repository holds the CC3501E bridge FIRMWARE only.**
> The wire contract, the host-side driver and all hardware metadata stay in
> [alplabai/alp-sdk](https://github.com/alplabai/alp-sdk) — the dependency runs
> firmware -> alp-sdk, never the other way. Extracted from
> `alp-sdk:firmware/cc3501e` with history intact (alp-sdk#1370).
>
> **`prebuilt/` ships signed binaries customers flash** as the
> `helper_firmware.cc3501e_otp` payload named by the AEN SoM presets
> (alp-sdk#1733). They are Alp Lab build output of the source here, linked
> against TI's SimpleLink SDK — which this repository does **not**
> redistribute; see
> `vendor/simplelink-cc33xx/README.md` for how to obtain it.
> The resulting images therefore also carry TI `wifi-uppermac`, lwIP and
> mbedTLS code; see `NOTICE`.

Firmware that runs on the **TI CC3501E** Wi-Fi 6 + BLE 5.4 coprocessor
populated on the E1M-AEN module family
(`E1M-AEN301/401/501/601/701/801`).  It is the SPI-slave (or, on
SDIO-routed boards, SDIO-slave) parser that fronts TI's SimpleLink
Wi-Fi + BLE stacks for the Alif Ensemble host.

This tree is a **separate compile artifact** with its own toolchain (TI
`ticlang` for the CC3501E Cortex-M33) and its own flash binary.  It is
**not** linked into the Zephyr-side alp-sdk library; the matching
host-side driver lives at [`chips/cc3501e/`](https://github.com/alplabai/alp-sdk/tree/main/chips/cc3501e/) and
the wire protocol at
[`include/alp/protocol/cc3501e.h`](https://github.com/alplabai/alp-sdk/blob/main/include/alp/protocol/cc3501e.h).

This firmware is its **own repository**, split out of
`alp-sdk:firmware/cc3501e` (alp-sdk#1370); the
[`gd32-bridge`](https://github.com/alplabai/alp-sdk/tree/main/firmware/gd32-bridge/)
is still embedded in alp-sdk.  The wire protocol stays single-source
across the split: the firmware `#include`s the canonical header out of an
alp-sdk checkout (`-DALP_SDK_ROOT=<path>`), never a mirrored copy, so
there is nothing here to drift.  What the split cost is **atomicity** --
an opcode change is now two commits in two repos instead of one -- so CI
builds against alp-sdk's *default branch* rather than a pinned SHA, and
the `protocol version parity` job fails the moment `protocol-version.txt`
and `ALP_CC3501E_PROTOCOL_VERSION` disagree.
[ADR 0015](https://github.com/alplabai/alp-sdk/blob/main/docs/adr/0015-cc3501e-firmware-embedded.md)
records the pre-split rationale.

## Tree layout

```
cc3501e-bridge-firmware/
├── CMakeLists.txt          ← backend (stub|ti) + transport (spi|sdio) selection; ALP_SDK_ROOT
├── README.md               ← this file
├── DESIGN.md               ← command scope, wire-reply contract, reconciliation items
├── BRINGUP_STATUS.md       ← the on-silicon bench record (link / Wi-Fi / BLE / OTA)
├── CONTRIBUTING.md         ← house style + review rules
├── NOTICE                  ← third-party code inside the SHIPPED BINARIES
├── firmware-version.txt    ← firmware RELEASE semver (own axis)
├── protocol-version.txt    ← wire-protocol version expected (= ALP_CC3501E_PROTOCOL_VERSION)
├── prebuilt/               ← signed release binaries + CHANGELOG.md
├── toolchain/              ← arm-none-eabi.cmake (stub/CI smoke) + ticlang.cmake (bench)
├── vendor/simplelink-cc33xx/ ← CMake shim onto a TI SDK you install yourself (no SDK sources)
├── ti/                     ← bench build + flash-set scripts, and the SysConfig board files
├── src/
│   ├── main.c              ← entry: hw init → selected transport init → WFI / FreeRTOS loop
│   ├── protocol.{c,h}      ← the opcode → handler switch + transport-agnostic framing
│   ├── protocol_internal.h ← handler declarations shared by the family TUs
│   ├── protocol_meta.c     ← PING / GET_VERSION / GET_MAC / RESET / STREAM_WRITE
│   ├── protocol_wifi.c     ← Wi-Fi station / AP / scan / status
│   ├── protocol_ble.c      ← BLE enable / advertise / scan / connect / GATT
│   ├── protocol_sockets.c  ← socket open / connect / send / recv / close
│   ├── protocol_gpio.c     ← GPIO proxy
│   ├── protocol_camera.c   ← camera-enable rails
│   ├── protocol_power.c    ← power policy (latched by the ISR, applied on the task)
│   ├── protocol_ota.c      ← OTA over the bridge + OTA update mode
│   ├── protocol_diag.c     ← diagnostics + the GET_PENDING_EVENTS drain
│   ├── event_ring.{c,h}    ← the async EVT_* ring the host drains
│   ├── worker.{c,h}        ← off-ISR seam for blocking radio / flash bodies
│   ├── transport.h         ← SPI + SDIO seam declarations
│   ├── transport_spi.c     ← SPI-slave staging (DEFAULT link)        — silicon-free
│   └── transport_sdio.c    ← SDIO-slave staging (OPTIONAL link)      — silicon-free
├── hal/
│   ├── cc3501e_hw.h        ← HAL contract (init/tick/MAC/reset + Wi-Fi, BLE, sockets,
│   │                         GPIO, camera, OTA, power, diagnostics)
│   ├── cc3501e_hw_stub.c   ← hardware-free backend (CI compile smoke)
│   └── ti/                 ← real TI SimpleLink / driverlib backend (bench build)
└── tests/
    ├── gen_protocol_vectors.py  ← canonical wire vectors (regenerate + --check)
    └── protocol_vectors.txt
```

Every transport calls the same `protocol_build_reply()` /
`protocol_dispatch()` in `src/protocol.c` -- one framing format, one
command set, one set of reply codes; only the byte-level transport
differs.

## Selectable host-control transport

The Alif↔CC3501E control link is **customer-selectable**:

| Transport | Role | Availability |
|-----------|------|--------------|
| **SPI0 slave** (CC35; Alif master = SPI1) | **DEFAULT** + always-available baseline/fallback | Always (CC3501E GPIO_27/28/29) |
| **SDIO slave** | OPTIONAL, higher throughput for Wi-Fi data | Only when the board routes the Alif's **single** SDIO controller to the CC3501E -- **mutually exclusive with a micro-SD card** (CC3501E GPIO_3/4/5/6/10/11) |

Because the Alif Ensemble has one SDIO controller shared at board level
with the SD slot, SDIO is available to the CC3501E only on boards
without an SD card; when an SD card is used, **SDIO is blocked and the
link falls back to SPI**.  Pick the transport at build time
(`-DCC3501E_CONTROL_TRANSPORT=spi|sdio`, default `spi`); in a studio
build the choice is sourced from the customer `board.yaml`.  Both
transports always compile; the selector only chooses which one `main()`
starts.

The current E1M-AEN rev (FIB v0.0.207, validated on E1M-AEN801) wires SPI with **hardware SS0 chip-select** (Alif `P14_7` = `SPI1_SS0_C`), per-phase READY gating, and deterministic framing per protocol phase (not fixed-count lockstep). See [`docs/cc3501e-bridge.md` § Current rev](https://github.com/alplabai/alp-sdk/blob/main/docs/cc3501e-bridge.md) for the validated link topology.

## Getting firmware onto a CC3501E

**Use the prebuilt blob.**  Building from source needs TI ticlang 5.1.1, the
SimpleLink Wi-Fi SDK 10.10.01.08, SysConfig 1.28.0 and the Wi-Fi toolbox 4.2.4
installed and on the right paths -- a long toolchain to require of anyone who
just wants a working companion.  The signed blob in `prebuilt/` is already
built and signed:

> **`cc3501e-v0.5.1.bin` is built from this tree's v0.5.1 release commit,
> which is current `main`.**  `cc3501e-v0.5.0.bin` and older are kept only for
> traceability, and note they are not all the same KIND of artifact: 0.2.0,
> 0.3.0 and 0.5.0 are raw `build_ti.ps1` output, while 0.4.0, 0.4.1 and this
> 0.5.1 are wrapped TI `flash-images-builder` vendor images.  Only the wrapped
> kind can be dropped straight into `primary_vendor_image.sign.bin` by the
> recipe below (#96).  `prebuilt/BUILD_RECIPE.md` records which is which and CI
> machine-checks it (#97).
>
> The `prebuilt freshness` CI job goes red when a change lands in `src/`,
> `hal/` or `ti/` that is neither released nor attested byte-identical in
> `prebuilt/BUILT_FROM`, so this is a checked claim rather than an asserted one
> -- that gap is what let 0.4.1 go stale under a green `prebuilt integrity`
> (#75).
>
> **Why 0.5.1 and not 0.6.0.**  This content shipped briefly as 0.6.0, which
> burned a minor version for no reason: neither 0.5.0 nor 0.6.0 was ever tagged
> or released, so a patch bump was the right step.  0.5.1 also keeps the
> `fw_version` marker unique -- it reads **`0x0501`**, where reusing 0.5.0 would
> have made a third distinct build report `0x0500`.
>
> **What this adds over 0.5.0:** request identity on `CMD_SOCK_SEND` (#89 +
> alp-sdk#1872), which bumps the wire protocol to **7**.  A host built against
> an older protocol is refused by `GET_VERSION`; pair this blob with alp-sdk at
> or after the matching host driver.  Bench-verified on the E1M-AEN801 serial
> `2617-0001`: `sock tcp-get` **10 of 10** end-to-end against a bare `accept()`
> listener with zero `send failed`, once the host-side phase settle was widened
> in alp-sdk#1873.
>
> **Two limits still open**, neither fixed here: socket **RX** stalls at
> roughly 2 kB, and the 250 us host phase settle is an empirical upper bound
> paid per payload phase, so it taxes anything that streams (alp-sdk#1677).

```sh
# 1. Verify what you are about to flash (never skip this).
openssl dgst -sha256 -verify keys/alp_cc3501e_vendor_VALIDATION_public.pem \
    -signature prebuilt/cc3501e-v0.5.1.bin.sig prebuilt/cc3501e-v0.5.1.bin
sha256sum -c <<<"$(cat prebuilt/cc3501e-v0.5.1.bin.sha256)  prebuilt/cc3501e-v0.5.1.bin"

# 2. Use a flash-set whose signed programming_instructions was generated at
#    THIS artifact's stamp (0.149.74.0) -- see the warning below.  An existing
#    flash-set built at another version will NOT do; regenerate it:
#      VERSION=0.149.74.0 ti/regen_flashset.sh
#
# 3. Drop the blob in as the primary vendor image, and remove any stale
#    pre-flattened image -- a leftover *.flashready.bin is used in preference
#    to the file you just copied.
#    This blob is the WRAPPED kind -- a TI flash-images-builder vendor_image,
#    signed in-band -- which is what this slot expects.  Do NOT do this with a
#    raw build_ti.ps1 image: the raw kind starts with a bare Cortex-M vector
#    table, not a signed container, and installing one here is a different file
#    format, not a different build (#96).  prebuilt/BUILD_RECIPE.md records
#    which release is which, and CI machine-checks it (#97).
cp prebuilt/cc3501e-v0.5.1.bin <flashset>/primary_vendor_image.sign.bin
rm -f <flashset>/*.flashready.bin

# 4. Program over XDS110/SWD (~18 s).
programmer -i XDS110 -param1 <PROBE_SN> programming --tool_settings <flashset>/tool_settings.warm.windows.json
```

> **Part not responding, or state unknown?** `docs/full-erase-and-flash.md`
> covers the recovery path -- full erase plus a complete four-component set --
> and the checks to run BEFORE erasing, because an erase removes the boot
> sector, the TBL and TI's wireless firmware, and a warm set cannot put them
> back.

**The image `--version` must match the version stamped into the signed
`programming_instructions` in that flash-set.**  If it does not, the programmer
reports success and the device silently keeps the old image -- the single most
expensive trap on this part.  `programming_instructions` is built from
`--version` plus the flash-discovery configs, so it must be regenerated at the
*same* `--version` as the vendor image (`ti/regen_flashset.sh`).

Verify the flash took by asking the device, not by trusting the programmer:
`GET_VERSION` must answer wire protocol **6**, and `GET_DIAG_INFO` must report
`fw_version=0x0500`.

Three distinct version numbers are in play here and they are **not**
interchangeable -- app SemVer (`0.5.1`), wire protocol (`7`), and the GPE
image stamp (`0.149.74.0`).  `prebuilt/CHANGELOG.md` has the table.

> **STOP -- check the unit's flash history before using this artifact's `0.149.74.0`
> stamp.**  The CC35 SBL enforces GPE-version **monotonicity against the last-seen
> version on that part**, and it does so **even when every `*_rollback_protection_*`
> fuse reads `0`**.  A warm programming run burns no fuses, so the report will show
> all-zero fuses and look permissive -- that is the trap, not the answer.  Flash a
> stamp LOWER than anything ever flashed on the unit and it streams clean (exit 0,
> the full ~1.09 MB) and then the SBL refuses to boot it: **dead link**, with an
> empty XDS110 `query` image table.  Bench units used for OTA or flash iteration sit
> at or above `0.149.74.0`.  The bench part here was last seen at `0.149.73.0`.
> For a unit above this artifact's stamp,
> re-wrap this same image at a legal stamp instead:
>
> ```sh
> VERSION=0.149.67.0 ti/regen_flashset.sh   # > the unit's last-seen version, major MUST be 0
> ```
>
> `major` must be `0`: a GPE major `>= 1` fails BL2 secure-boot with `AUTH_ERROR`.
> `BRINGUP_STATUS.md` "The #1 cause of *streams clean but dead link*" has the full
> rule and the recovery path.

Build from source only when you are changing the firmware -- see below.

## Build

The CMake build runs **outside** the Zephyr build (the Alif side's
`west build` does not descend here).  Two backends:

Run both from this repo's root.  `-DALP_SDK_ROOT` is **mandatory**: the
firmware compiles the canonical `<alp/protocol/cc3501e.h>` out of an
alp-sdk checkout, and CMake stops with a `FATAL_ERROR` when it cannot find
that header.  `CMAKE_TOOLCHAIN_FILE` must be **absolute** -- CMake resolves
a relative toolchain path against the *build* directory, not the working
directory.

```bash
# Host-side / CI compile smoke -- silicon-free, no TI SDK needed.
# ALP_SDK_ROOT is MANDATORY: this firmware compiles alp-sdk's canonical
# <alp/protocol/cc3501e.h> rather than a mirrored copy, and CMake FATAL_ERRORs
# without it.  The toolchain file must be an ABSOLUTE path -- CMake resolves a
# relative one against the BUILD directory, not the working directory.
cmake -B build/stub -S . \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/toolchain/arm-none-eabi.cmake" \
  -DCC3501E_HAL_BACKEND=stub \
  -DALP_SDK_ROOT=<path-to-alp-sdk>
cmake --build build/stub

# Production image (bench) -- needs TI ticlang + the SimpleLink CC33xx SDK.
# In practice use ti/build_ti.sh (or ti/build_ti.ps1): the CMake ti path does
# not apply the four post-generation patches those scripts do.
cmake -B build/ti -S . \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/toolchain/ticlang.cmake" \
  -DCC3501E_HAL_BACKEND=ti \
  -DALP_SDK_ROOT=<path-to-alp-sdk>
cmake --build build/ti
```

The `ti` backend links against TI's SimpleLink CC33xx SDK, which you
obtain from TI and install yourself.  It is **not vendored and not a
submodule**: `vendor/simplelink-cc33xx/` holds only a `CMakeLists.txt`
shim plus a `README.md` describing how to obtain the SDK -- no SDK
sources are redistributed here, and this repository makes no statement
about TI's licence terms.  The build uses `ticlang` (TI's Arm LLVM
compiler -- pin the exact version on the bench for reproducible images).
CI builds only the `stub` backend (that catches compile + framing errors
without the ~GB vendor SDK), in both transport configurations, and pins
`protocol-version.txt` against alp-sdk's `ALP_CC3501E_PROTOCOL_VERSION`.
The frame-level wire vectors live in `tests/protocol_vectors.txt` and are
pinned by the `wire vectors` job
(`ALP_SDK_ROOT=<path-to-alp-sdk> python3 tests/gen_protocol_vectors.py
--check`).  The full gate list is `stub build`, `protocol version parity`,
`wire vectors`, `clang-format (diff only)`, `no build artifacts or crash
dumps` and `prebuilt integrity`.

## Command scope

`protocol_dispatch()` routes **49 opcodes**, covering every command family
in the wire header: META (PING, GET_VERSION, GET_MAC, RESET,
STREAM_WRITE), Wi-Fi station/AP/scan/status, BLE (enable, advertise,
scan, connect, GATT), sockets, the GPIO proxy, camera enables, power
policy, diagnostics + `GET_PENDING_EVENTS`, and OTA including
`OTA_UPDATE_MODE`.  Routed is not the same as proven: the Status table
below and `BRINGUP_STATUS.md` record what is silicon-validated, and
**sockets do not connect** (alp-sdk#1746).

What survives from the bring-up contract is the rejection rule: an opcode
this firmware does not implement is answered with
`ALP_CC3501E_RESP_ERR_INVALID`, per the protocol header.  The roadmap
itself lives in
[`docs/cc3501e-bridge.md`](https://github.com/alplabai/alp-sdk/blob/main/docs/cc3501e-bridge.md).

## Source-of-truth contract

| Contract | Defined in |
|----------|------------|
| Wire protocol (commands, frames, events) | [`include/alp/protocol/cc3501e.h`](https://github.com/alplabai/alp-sdk/blob/main/include/alp/protocol/cc3501e.h) (included directly -- no mirror) |
| Bridge architecture + GPIO behaviour contract | [`docs/cc3501e-bridge.md`](https://github.com/alplabai/alp-sdk/blob/main/docs/cc3501e-bridge.md) |
| Inter-chip wiring (SPI1 / SDIO / control lines) | [`metadata/e1m_modules/aen/inter-chip.tsv`](https://github.com/alplabai/alp-sdk/blob/main/metadata/e1m_modules/aen/inter-chip.tsv) |
| E1M ↔ CC3501E pad routing | [`metadata/e1m_modules/aen/from-cc3501e.tsv`](https://github.com/alplabai/alp-sdk/blob/main/metadata/e1m_modules/aen/from-cc3501e.tsv) |
| Command scope + wire-reply contract (incl. the reply padding) + reconciliation items | `DESIGN.md` |

## Versioning

Three independent axes (same model as the gd32-bridge):

| Axis | Where | Bumps when |
|------|-------|-----------|
| Firmware release | `firmware-version.txt` | each firmware release -- names the tag + the `prebuilt/cc3501e-vX.Y.Z.bin` blob |
| Wire protocol | `ALP_CC3501E_PROTOCOL_VERSION` (`<alp/protocol/cc3501e.h>`) + `protocol-version.txt` | the wire format changes; the host refuses a mismatched version via GET_VERSION |
| Build / signature | the signed binary's `.sha256` in `prebuilt/` | every build |

## Firmware updates

The CC3501E ships **pre-flashed by Alp**, and normal firmware updates are
Alp-released and applied over the bridge SPI link, programming the chip's
own OTP; the SoM preset models this with
`helper_firmware[].update_channel: alp_ota_spi_otp` +
`flash_policy: recovery_only` (see alp-sdk
[`metadata/e1m_modules/README.md`](https://github.com/alplabai/alp-sdk/blob/main/metadata/e1m_modules/README.md)), and no
`flash_method`.  A customer flash is permitted **only** to recover a bricked
device, using Alp Lab-supplied binaries — it is not a routine flash target.
The signed
release blob is version-pinned at `prebuilt/cc3501e-vX.Y.Z.bin`.
`flash.py` is Alp's internal release/bench tool that produces and
validates that blob (relaying the image to the CC3501E over the
inter-chip link) -- it is not a customer-facing utility, and lives
in `alp-sdk-internal`, not this public tree.
`prebuilt/` holds the signed release blob; `cc3501e-v0.5.1.bin` is the
current one (wire protocol **7**).  Every older blob answers a DIFFERENT
protocol and a host built from this tree refuses all of them:
`cc3501e-v0.5.0.bin` answers **6**, `cc3501e-v0.4.1.bin` and
`cc3501e-v0.4.0.bin` answer **5**, `cc3501e-v0.3.0.bin` answers **4**.  They
are kept for traceability only -- and among them 0.4.1 predates the socket-EOF
repair (#32) and 0.4.0's soft-AP accepts no clients (alp-sdk#1562).

## Status

| Milestone | Status |
|-----------|--------|
| Wire-protocol header + host driver | ✅ landed (`include/alp/protocol/`, `chips/cc3501e/`) |
| Firmware tree (own repo) | ✅ this tree |
| META group (PING / GET_VERSION / GET_MAC / RESET) | ✅ silicon-free + stub backend; the frames are pinned by `tests/protocol_vectors.txt` (regenerate or `--check` with `tests/gen_protocol_vectors.py`; gated by the `wire vectors` CI job) |
| TI backend: SPI-slave + lifecycle (`hal/ti/`) | ✅ implemented against TI Drivers (`SPI_open` in `SPI_PERIPHERAL` + `SPI_MODE_CALLBACK`) + the CC35xx Wi-Fi host API (`Wlan_Start`, and `Wlan_Get(WLAN_GET_MACADDRESS)` for the factory MAC) + CMSIS reset. Hardware SS0 chip-select + per-phase READY framing (this rev wires SCLK/MOSI/MISO + `SPI1_SS0_C`); host `cc3501e_request()` reconciled to match. Compiles on the bench against the SimpleLink CC35xx SDK + a SysConfig board file (`CONFIG_SPI_0`). Bench-validated on E1M-AEN801 (FIB v0.0.207): survives radio ops and concurrent Wi-Fi/BLE scan; see [`docs/cc3501e-bridge.md`](https://github.com/alplabai/alp-sdk/blob/main/docs/cc3501e-bridge.md). |
| TI backend: SDIO-slave (`hal/ti/transport_hw_ti_sdio.c`) | 🟡 frame glue complete; the SDIO-**device** register bring-up needs SWRU626 §21 (no public SDK SDIO-device driver). Off the critical path — SPI is the default. |
| Async events: attention edge on READY | ❌ **NOT delivered by an edge on this board rev (#57, measured 2026-08-29).** Alif `P2_6` is an OPEN net at the host: with the Alif powered and only the CC35 held in nRESET, `P2_6` read HIGH in **48 of 48** samples across a 46 s window that spans the CC35's entire reset → `Board_init()` → first-arm sequence — the window in which `cc3501e_hw_init()` holds `cc3501e_bridge_busy()` (READY LOW) for *seconds*. The bridge answered `protocol v5` afterwards, so it really did reset and re-init in that window. A connected wire could not stay HIGH through it. This corroborates the independent "0 edges in 20000 samples" note in `src/worker.c` and `hal/ti/cc3501e_hw_ti_ble.c`, and **refutes** the previous row's claim of "135/135 firmware pushes delivered" *on the edge* — the host cannot observe edges on a net it does not see, so that delivery came from the timer poll, not the attention edge. The CC35-side pulse code is real and harmless; it is the HOST-side edge that does not exist here. Needs a board rev, or a dedicated HOST_IRQ pad. Build-time opt-in (`build_ti.ps1 -AttnPulse`, default OFF). |
| `flash.py` real flashing | 🔮 moved to `alp-sdk-internal` (Alp-internal OTA-build tooling); blocked on TI's `cc3501e-flasher` CLI (not public yet); manual SWD/J-Link is the interim bench path |
| `prebuilt/` populated | ✅ `cc3501e-v0.5.0.bin` signed (full bridge: META + Wi-Fi + BLE + OTA + the E1M SPI1 passthrough, **proto v6**). Matches `main` as of the v0.5.0 re-cut release commit, and `prebuilt freshness` keeps that true. Carries 0.5.0's socket-EOF repair (#32) and the BLE `scan-stop` re-init removal that took the #5 wedge from ~9% to ~1%. **Sockets still do not connect from the host** (alp-sdk#1746) — which is also why #32 is not yet exercised on silicon; see `prebuilt/CHANGELOG.md`. |
| Wi-Fi / BLE / GPIO-proxy groups | ✅ implemented and silicon-validated (alp-sdk v0.8.0 on E1M-AEN801): Wi-Fi scan with security decode, real BLE scan (ble_gap_disc), GPIO proxy warm-boot, OTA-over-bridge staged (see [`docs/cc3501e-bridge.md`](https://github.com/alplabai/alp-sdk/blob/main/docs/cc3501e-bridge.md)). |
