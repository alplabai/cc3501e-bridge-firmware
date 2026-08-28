#!/usr/bin/env python3
# Copyright 2026 Alp Lab AB
# SPDX-License-Identifier: Apache-2.0
"""
Regenerate tests/protocol_vectors.txt.

Authoritative, human-readable source-of-truth for the cc3501e-bridge
wire frames, shared by the firmware transport tests
(tests/zephyr/cc3501e_bridge_transport/) and any future host-driver tests
under tests/zephyr/chips/cc3501e/.

The cc3501e frame is a 4-byte little-endian header + payload, with NO
start-of-frame byte and NO CRC (a short hardwired point-to-point link):

    REQUEST : cmd | flags | payload_len(LE16) | payload[payload_len]
    REPLY   : cmd | flags | payload_len(LE16) | status | data[...] | pad[...]

The reply echoes the request cmd, uses flags=0 (solicited), and carries
the response status (ALP_CC3501E_RESP_*) as the first payload byte.  The
reply payload is rounded up to a multiple of CC3501E_REPLY_PAD with zero
bytes -- see reply() below and protocol_build_reply() in src/protocol.c.

The canonical wire header include/alp/protocol/cc3501e.h lives in the
alp-sdk checkout, which is a SEPARATE repo from this firmware, so point
ALP_SDK_ROOT at it (the same variable the CMake build takes) and run from
the cc3501e-bridge-firmware repo root:

    ALP_SDK_ROOT=/path/to/alp-sdk python3 tests/gen_protocol_vectors.py

Use --check in CI to fail on drift.
"""

from __future__ import annotations

import argparse
import os
import pathlib
import re
import sys

_TESTS_DIR = pathlib.Path(__file__).resolve().parent
_HEADER_RELPATH = pathlib.Path("include") / "alp" / "protocol" / "cc3501e.h"


def _resolve_wire_header() -> pathlib.Path:
    """Locate the canonical wire header include/alp/protocol/cc3501e.h.

    It ships in the alp-sdk repo, not this one, so $ALP_SDK_ROOT (what the
    CMake build already passes as ALP_SDK_ROOT) wins.  The fallback is the
    pre-extraction in-tree layout, where this file sat at
    alp-sdk/firmware/cc3501e/tests/ and the sdk root was three levels up."""
    roots: list[pathlib.Path] = []
    env_root = os.environ.get("ALP_SDK_ROOT")
    if env_root:
        roots.append(pathlib.Path(env_root))
    roots.append(_TESTS_DIR.parents[2])

    for root in roots:
        candidate = root / _HEADER_RELPATH
        if candidate.is_file():
            return candidate

    tried = ", ".join(str(root / _HEADER_RELPATH) for root in roots)
    sys.exit(
        f"cannot find the canonical wire header {_HEADER_RELPATH.as_posix()} -- set "
        "ALP_SDK_ROOT to your alp-sdk checkout, e.g.\n"
        "    ALP_SDK_ROOT=/path/to/alp-sdk python3 tests/gen_protocol_vectors.py\n"
        f"tried: {tried}"
    )


_HEADER = _resolve_wire_header()
_PROTOCOL_VERSION_TXT = pathlib.Path(__file__).parent.parent / "protocol-version.txt"
_PROTOCOL_H = pathlib.Path(__file__).parent.parent / "src" / "protocol.h"

_DEFINE_RE = re.compile(r"^#define\s+ALP_CC3501E_PROTOCOL_VERSION\s+(\d+)", re.MULTILINE)
_REPLY_PAD_RE = re.compile(r"^#define\s+CC3501E_REPLY_PAD\s+(\d+)u?\b", re.MULTILINE)


def _read_protocol_version() -> int:
    """Single source of truth: parse ALP_CC3501E_PROTOCOL_VERSION out of the
    wire-protocol header instead of hardcoding it here, so a version bump
    that forgets this file fails --check instead of silently drifting.
    Also cross-checks protocol-version.txt against the same value."""
    match = _DEFINE_RE.search(_HEADER.read_text(encoding="utf-8"))
    if not match:
        sys.exit(f"cannot find #define ALP_CC3501E_PROTOCOL_VERSION in {_HEADER}")
    header_version = int(match.group(1))

    txt_version = _PROTOCOL_VERSION_TXT.read_text(encoding="utf-8").strip()
    if txt_version != str(header_version):
        sys.exit(
            f"DRIFT: {_PROTOCOL_VERSION_TXT} says {txt_version!r} but "
            f"{_HEADER} defines ALP_CC3501E_PROTOCOL_VERSION {header_version} -- "
            "update protocol-version.txt to match."
        )
    return header_version


def _read_reply_pad() -> int:
    """Single source of truth for the reply padding granularity: parse
    CC3501E_REPLY_PAD out of src/protocol.h instead of hardcoding 8, so a
    change there fails --check instead of silently drifting."""
    match = _REPLY_PAD_RE.search(_PROTOCOL_H.read_text(encoding="utf-8"))
    if not match:
        sys.exit(f"cannot find #define CC3501E_REPLY_PAD in {_PROTOCOL_H}")
    return int(match.group(1))


# --- Opcodes / codes -- keep aligned with include/alp/protocol/cc3501e.h.
CMD_PING = 0x00
CMD_GET_VERSION = 0x01
CMD_RESET = 0x02
CMD_GET_MAC = 0x03
CMD_GET_PENDING_EVENTS = 0x05  # async-event queue drain (host-polled)
CMD_WIFI_SCAN_START = 0x10  # worker-routed Wi-Fi scan (v0.2)

# Async event opcodes carried inside a GET_PENDING_EVENTS reply.
EVT_WIFI_CONNECTED = 0x19
EVT_WIFI_DISCONNECTED = 0x1A
CMD_SOCK_OPEN = 0x20
CMD_SOCK_CLOSE = 0x24

FLAG_SOLICITED = 0x00

RESP_OK = 0x00
RESP_ERR_INVALID = 0x01
RESP_ERR_BUSY = 0x02
RESP_ERR_NOT_READY = 0x05
RESP_ERR_PROTOCOL = 0x07

# Wire-protocol version GET_VERSION reports (ALP_CC3501E_PROTOCOL_VERSION).
# Sourced from the header, not hardcoded -- see _read_protocol_version().
PROTOCOL_VERSION = _read_protocol_version()

# Reply-payload padding granularity (CC3501E_REPLY_PAD).
# Sourced from src/protocol.h, not hardcoded -- see _read_reply_pad().
REPLY_PAD = _read_reply_pad()


def frame(cmd: int, flags: int, payload: bytes = b"") -> bytes:
    """Build a cc3501e frame: cmd | flags | payload_len(LE16) | payload."""
    return bytes([cmd, flags, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload


def reply(cmd: int, status: int, data: bytes = b"") -> bytes:
    """Build a solicited reply frame (payload = status + data, zero-padded).

    protocol_build_reply() in src/protocol.c rounds the reply payload
    (1 status byte + data) up to a multiple of CC3501E_REPLY_PAD and zero-fills
    the pad bytes, so the host can DMA the reply as one burst-aligned chunk.
    The pad bytes are never interpreted: every reply carries its own length.

    The firmware pads only when the padded frame still fits the caller's
    reply_cap.  This generator assumes the full CC3501E_FRAME_MAX_BYTES buffer
    that BOTH transports pass (spi_tx_buf in src/transport_spi.c,
    sdio_reply_buf in src/transport_sdio.c), so the pad always fits here."""
    payload = bytes([status]) + data
    rem = len(payload) % REPLY_PAD
    if rem:
        payload += bytes(REPLY_PAD - rem)
    return frame(cmd, FLAG_SOLICITED, payload)


def padded_len(data_len: int) -> int:
    """payload_len a reply carrying data_len DATA bytes puts on the wire."""
    n = 1 + data_len
    rem = n % REPLY_PAD
    return n + (REPLY_PAD - rem if rem else 0)


HEADER = f"""\
# cc3501e-bridge canonical wire-test vectors
#
# Consumed by the firmware transport tests
# (tests/zephyr/cc3501e_bridge_transport/) and any future host-driver
# tests (tests/zephyr/chips/cc3501e/), so the two sides cannot diverge.
#
# Frame: 4-byte LE header [cmd | flags | payload_len(LE16)] + payload.
# No SOF, no CRC.  Reply payload[0] is the response status.
#
# Reply payload_len is ROUNDED UP to a multiple of CC3501E_REPLY_PAD
# ({REPLY_PAD}, src/protocol.h) and the pad bytes are ZERO -- see
# protocol_build_reply() in src/protocol.c.  So a status-only reply is
# {REPLY_PAD} payload bytes on the wire, not 1.  Requests are NOT padded.
#
# Format: one `<name> = <hex>` vector per non-comment line; `#` comments.
# Regenerate with
# `ALP_SDK_ROOT=/path/to/alp-sdk python3 tests/gen_protocol_vectors.py`.
"""


def build_vectors() -> list[tuple[str, str, str | None]]:
    out: list[tuple[str, str, str | None]] = []
    pl = padded_len  # padded reply payload_len, for the per-vector comments

    out.append(("ping_request", frame(CMD_PING, 0).hex().upper(),
                "cmd=PING | flags=0 | len=0"))
    out.append(("ping_reply_ok", reply(CMD_PING, RESP_OK).hex().upper(),
                f"cmd=PING | flags=0 | len={pl(0)} | status=OK"))

    out.append(("get_version_request", frame(CMD_GET_VERSION, 0).hex().upper(),
                "cmd=GET_VERSION | flags=0 | len=0"))
    out.append((
        f"get_version_reply_proto{PROTOCOL_VERSION}",
        reply(CMD_GET_VERSION, RESP_OK,
              bytes([PROTOCOL_VERSION & 0xFF, (PROTOCOL_VERSION >> 8) & 0xFF])).hex().upper(),
        f"cmd=GET_VERSION | len={pl(2)} | status=OK | version={PROTOCOL_VERSION} (LE16)",
    ))

    out.append(("get_mac_request", frame(CMD_GET_MAC, 0).hex().upper(),
                "cmd=GET_MAC | flags=0 | len=0"))
    # GET_MAC is async (P0-4/P0-6): the first request submits the worker job
    # and replies BUSY; the host re-issues until the worker has the result.
    out.append(("get_mac_reply_busy_submitted", reply(CMD_GET_MAC, RESP_ERR_BUSY).hex().upper(),
                f"cmd=GET_MAC | len={pl(0)} | status=BUSY -- job submitted, host re-issues"))
    out.append(("get_mac_reply_not_ready_stub", reply(CMD_GET_MAC, RESP_ERR_NOT_READY).hex().upper(),
                f"cmd=GET_MAC | len={pl(0)} | status=NOT_READY -- re-issued; stub has no radio"))

    out.append(("reset_request", frame(CMD_RESET, 0).hex().upper(),
                "cmd=RESET | flags=0 | len=0"))
    out.append(("reset_reply_ok", reply(CMD_RESET, RESP_OK).hex().upper(),
                f"cmd=RESET | len={pl(0)} | status=OK -- firmware reboots after the ack is read"))

    # WIFI_SCAN_START (0x10) is implemented and worker-routed (handle_wifi_scan_start
    # -> handle_worker_routed): the real Wlan_Scan blocks for seconds, so the first
    # request submits the job and replies BUSY; the host re-issues to collect it.
    out.append(("wifi_scan_start_request", frame(CMD_WIFI_SCAN_START, 0).hex().upper(),
                "cmd=WIFI_SCAN_START | flags=0 | len=0"))
    out.append((
        "wifi_scan_start_reply_busy_submitted",
        reply(CMD_WIFI_SCAN_START, RESP_ERR_BUSY).hex().upper(),
        f"cmd=WIFI_SCAN_START | len={pl(0)} | status=BUSY -- job submitted, host re-issues",
    ))
    # Re-issued on the stub (no radio) -> NOTIMPL mapped to NOT_READY.
    out.append((
        "wifi_scan_start_reply_not_ready_stub",
        reply(CMD_WIFI_SCAN_START, RESP_ERR_NOT_READY).hex().upper(),
        f"cmd=WIFI_SCAN_START | len={pl(0)} | status=NOT_READY -- re-issued; stub has no radio",
    ))

    # TCP/UDP sockets (0x20..0x24): worker-routed, poll-by-repeat like GET_MAC.
    # SOCK_OPEN req = sock_open_t { family=IPV4(0) | type=STREAM(0) | proto=0 | rsvd }.
    out.append(("sock_open_tcp_request",
                frame(CMD_SOCK_OPEN, 0, bytes([0x00, 0x00, 0x00, 0x00])).hex().upper(),
                "cmd=SOCK_OPEN | len=4 | family=IPV4 type=STREAM proto=0"))
    # First request submits the worker job and replies BUSY; host re-issues.
    out.append(("sock_open_reply_busy_submitted", reply(CMD_SOCK_OPEN, RESP_ERR_BUSY).hex().upper(),
                f"cmd=SOCK_OPEN | len={pl(0)} | status=BUSY -- job submitted, host re-issues"))
    # Re-issued on the stub (no IP stack) -> NOT_READY.
    out.append(("sock_open_reply_not_ready_stub",
                reply(CMD_SOCK_OPEN, RESP_ERR_NOT_READY).hex().upper(),
                f"cmd=SOCK_OPEN | len={pl(0)} | status=NOT_READY -- re-issued; stub has no IP stack"))
    # SOCK_CLOSE req = sock_close_t { handle(LE16)=1 | reserved(LE16) } = 4 B.
    out.append(("sock_close_request",
                frame(CMD_SOCK_CLOSE, 0, bytes([0x01, 0x00, 0x00, 0x00])).hex().upper(),
                "cmd=SOCK_CLOSE | len=4 | handle=1"))
    # Bad length: a 3-byte SOCK_OPEN payload is rejected up front (not worker-routed).
    out.append(("sock_open_bad_len_reply_invalid",
                reply(CMD_SOCK_OPEN, RESP_ERR_INVALID).hex().upper(),
                f"cmd=SOCK_OPEN | len={pl(0)} | status=INVALID -- payload len != sizeof(sock_open_t)"))

    # Async-event queue drain (0x05, proto v3): the reply DATA is a packed list
    # of { evt_opcode(1) | len(1) | payload[len] } entries.
    out.append(("get_pending_events_request", frame(CMD_GET_PENDING_EVENTS, 0).hex().upper(),
                "cmd=GET_PENDING_EVENTS | flags=0 | len=0"))
    # Empty ring: status OK with zero data bytes.
    out.append(("get_pending_events_reply_empty",
                reply(CMD_GET_PENDING_EVENTS, RESP_OK).hex().upper(),
                f"cmd=GET_PENDING_EVENTS | len={pl(0)} | status=OK | no events queued"))
    # Two payloadless events queued: EVT_WIFI_CONNECTED then EVT_WIFI_DISCONNECTED,
    # each { evt_opcode | len=0 } back to back in the reply DATA.
    out.append((
        "get_pending_events_reply_wifi_conn_disc",
        reply(CMD_GET_PENDING_EVENTS, RESP_OK,
              bytes([EVT_WIFI_CONNECTED, 0x00, EVT_WIFI_DISCONNECTED, 0x00])).hex().upper(),
        f"cmd=GET_PENDING_EVENTS | len={pl(4)} | status=OK | "
        "[WIFI_CONNECTED len0][WIFI_DISCONNECTED len0] then zero pad",
    ))

    # Framing error: declared payload_len doesn't match the captured bytes.
    out.append((
        "ping_bad_len_reply_protocol",
        reply(CMD_PING, RESP_ERR_PROTOCOL).hex().upper(),
        "reply to a frame whose payload_len mismatches the byte count -> PROTOCOL error",
    ))

    return out


def emit(vectors: list[tuple[str, str, str | None]]) -> str:
    chunks = [HEADER]
    for name, value, comment in vectors:
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<34} = {value}")
    chunks.append("")
    return "\n".join(chunks)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true",
                        help="exit 1 if the on-disk file does not match the generated content")
    parser.add_argument("--out", type=pathlib.Path,
                        default=pathlib.Path(__file__).parent / "protocol_vectors.txt")
    args = parser.parse_args(argv)

    rendered = emit(build_vectors())
    if args.check:
        if not args.out.exists():
            print(f"missing: {args.out}", file=sys.stderr)
            return 1
        if args.out.read_text(encoding="utf-8") != rendered:
            print(f"DRIFT: {args.out} does not match generator output. Rerun without --check.",
                  file=sys.stderr)
            return 1
        print(f"OK: {args.out} matches generator output.")
        return 0

    args.out.write_text(rendered, encoding="utf-8", newline="")
    print(f"wrote {len(rendered)} bytes to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
