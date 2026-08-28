<!--
Copyright (c) 2026 Alp Lab AB
SPDX-License-Identifier: Apache-2.0
-->

# Security policy

## Reporting a vulnerability

**Do not open a public issue for a vulnerability.**

Use GitHub's private vulnerability reporting on this repository
(Security → Report a vulnerability), or email **security@alplab.ai**.

Include the firmware version (`firmware-version.txt`, or the `fw_version`
reported by `GET_DIAG_INFO`), the wire protocol version, and how the bridge was
reached — a fault reachable only over SWD with physical access is a different
severity from one reachable over the SPI link or over the air.

We will acknowledge within 5 working days.

## Scope

This repository is the **CC3501E bridge firmware**: the image running on the
TI CC3501E companion, its wire-protocol implementation, and its OTA path.

In scope here:

- The wire protocol as implemented in `src/` — framing, dispatch, and any
  handler that parses peer-supplied bytes.
- The OTA path (`hal/ti/cc3501e_hw_ti_ota.c` and the protocol side), including
  image acceptance and the anti-rollback behaviour described in
  `prebuilt/CHANGELOG.md`.
- The signed artifacts under `prebuilt/`.

**Not in scope here, report against the right repository:**

- The *host* driver, the portable `<alp/*>` API, and everything else in
  [`alplabai/alp-sdk`](https://github.com/alplabai/alp-sdk) — including
  `chips/cc3501e/`, which is the host half of this same protocol.
- The GD32 bridge firmware, in
  [`alplabai/gd32-bridge-firmware`](https://github.com/alplabai/gd32-bridge-firmware).
- TI SimpleLink SDK, lwIP, mbedTLS and the `wifi-uppermac` blob embedded in the
  shipped image. Those are upstream; report to their maintainers. We will
  forward and track anything that affects what we ship.

## Things worth knowing before you report

Some properties below look like vulnerabilities and are documented, deliberate
limitations of the current hardware revision. Reporting them is not wasted, but
the response will be a pointer here:

- **The bridge SPI link is not authenticated.** It is an inter-chip link on a
  single module; an attacker with physical access to those traces has already
  won more directly.
- **`prebuilt/` images are signed with a VALIDATION key, not a production key.**
  They are bench-grade artifacts. `prebuilt/CHANGELOG.md` says so per release.
- **A GPE stamp below the part's last-seen version is refused by the SBL**, and
  that is intended anti-rollback behaviour, not a bug — see the anti-rollback
  section in `prebuilt/CHANGELOG.md`.

## Disclosure

We aim to ship a fix or a documented mitigation within 90 days of a confirmed
report, and will credit reporters who want it. Because this firmware ships as a
signed binary flashed onto modules, a fix may also require a release artifact
and a field-update path, not only a source change — we will say so explicitly
rather than closing on the commit.
