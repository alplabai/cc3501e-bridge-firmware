#!/usr/bin/env python3
# Copyright 2026 Alp Lab AB
# SPDX-License-Identifier: Apache-2.0
"""
Regenerate firmware/cc3501e/tests/protocol_vectors.txt.

Authoritative, human-readable source-of-truth for the cc3501e-bridge
wire frames, shared by the firmware transport tests
(tests/zephyr/cc3501e_bridge_transport/) and any future host-driver tests
under tests/zephyr/chips/cc3501e/.

The cc3501e frame is a 4-byte little-endian header + payload, with NO
start-of-frame byte and NO CRC (a short hardwired point-to-point link):

    REQUEST : cmd | flags | payload_len(LE16) | payload[payload_len]
    REPLY   : cmd | flags | payload_len(LE16) | status | data[...]

The reply echoes the request cmd, uses flags=0 (solicited), and carries
the response status (ALP_CC3501E_RESP_*) as the first payload byte.

Run from the alp-sdk repo root:

    python3 firmware/cc3501e/tests/gen_protocol_vectors.py

Use --check in CI to fail on drift.
"""

from __future__ import annotations

import argparse
import pathlib
import os
import re
import sys

# The protocol header lives in alp-sdk, not here: this firmware compiles the
# CANONICAL <alp/protocol/cc3501e.h> rather than a mirror, which is the whole
# point of the single-sourcing in ADR 0031.
#
# parents[3] assumed the pre-extraction alp-sdk/firmware/cc3501e/tests/ layout.
# Standalone it climbs TWO levels above the repo root, so --check died with
# FileNotFoundError instead of pinning anything -- and worse, if a stale
# include/alp/protocol/cc3501e.h ever existed at that out-of-tree path the
# script would silently source a FOREIGN protocol version and pass (#11).
#
# ALP_SDK_ROOT is the supported way to point at a checkout.  The fallback keeps
# a pre-extraction tree working (two levels above the REPO ROOT = alp-sdk root
# when this repo still sat at alp-sdk/firmware/cc3501e).
#
# The variable below is the REPO ROOT and must be parents[1]: this file is
# <repo>/tests/gen_protocol_vectors.py, so parents[0] is tests/ and parents[1] is
# the repo.  It was parents[2] -- the repo's PARENT -- which made the name a lie
# and pushed the fallback one level too far out: parents[2].parent.parent
# resolved ABOVE alp-sdk, i.e. one level further out than the parents[3] it
# replaced, so the pre-extraction fallback it exists to serve could never hit.
# Verified on both layouts:
#   alp-sdk/firmware/cc3501e/tests/... -> parents[1].parent.parent == alp-sdk  (right)
#                                      -> parents[2].parent.parent == above it (wrong)
_ENV_SDK_ROOT = os.environ.get("ALP_SDK_ROOT")
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
_HEADER = (
    pathlib.Path(_ENV_SDK_ROOT).expanduser() / "include" / "alp" / "protocol" / "cc3501e.h"
    if _ENV_SDK_ROOT
    else _REPO_ROOT.parent.parent / "include" / "alp" / "protocol" / "cc3501e.h"
)
_PROTOCOL_VERSION_TXT = pathlib.Path(__file__).parent.parent / "protocol-version.txt"

# The wire version is MAJOR.MINOR since ADR 0033; ALP_CC3501E_PROTOCOL_VERSION
# is now the COMPOSED expression `(MAJOR << 8) | MINOR`, not a literal, so the
# two halves are parsed instead and composed here the same way the header does.
_MAJOR_RE = re.compile(r"^#define\s+ALP_CC3501E_PROTOCOL_MAJOR\s+(\d+)", re.MULTILINE)
_MINOR_RE = re.compile(r"^#define\s+ALP_CC3501E_PROTOCOL_MINOR\s+(\d+)", re.MULTILINE)


def _read_protocol_version() -> int:
    """Single source of truth: parse ALP_CC3501E_PROTOCOL_VERSION out of the
    wire-protocol header instead of hardcoding it here, so a version bump
    that forgets this file fails --check instead of silently drifting.
    Also cross-checks protocol-version.txt against the same value."""
    if not _HEADER.is_file():
        # An actionable message, not a bare FileNotFoundError two levels above the
        # repo -- which is what this raised standalone before #11.
        sys.exit(
            "cannot find the canonical protocol header at:\n"
            "    %s\n"
            "This firmware compiles alp-sdk's <alp/protocol/cc3501e.h> rather than a\n"
            "mirror, so it has to come from an alp-sdk checkout.  Point ALP_SDK_ROOT\n"
            "at one:\n"
            "    ALP_SDK_ROOT=<path-to-alp-sdk> python3 tests/gen_protocol_vectors.py --check"
            % _HEADER
        )
    text = _HEADER.read_text(encoding="utf-8")
    major_m = _MAJOR_RE.search(text)
    minor_m = _MINOR_RE.search(text)
    if not major_m or not minor_m:
        sys.exit(
            f"cannot find #define ALP_CC3501E_PROTOCOL_MAJOR / _MINOR in {_HEADER}.\n"
            "The wire version has been MAJOR.MINOR since ADR 0033; a header that\n"
            "still carries only a flat ALP_CC3501E_PROTOCOL_VERSION literal is from\n"
            "before that and cannot be paired with this firmware."
        )
    major = int(major_m.group(1))
    minor = int(minor_m.group(1))
    if major < 1 or major > 255 or minor > 255:
        sys.exit(
            f"illegal wire version {major}.{minor} in {_HEADER}: each half must fit a "
            "byte, and MAJOR 0 is reserved to mean 'firmware predates the scheme'."
        )
    header_version = (major << 8) | minor

    # protocol-version.txt carries the HUMAN form "MAJOR.MINOR" -- the same
    # string a release note and `alp companion ver` show -- not the composed
    # integer, so the file stays readable by whoever is holding a board.
    txt_version = _PROTOCOL_VERSION_TXT.read_text(encoding="utf-8").strip()
    if txt_version != f"{major}.{minor}":
        sys.exit(
            f"DRIFT: {_PROTOCOL_VERSION_TXT} says {txt_version!r} but "
            f"{_HEADER} defines ALP_CC3501E_PROTOCOL_MAJOR.MINOR {major}.{minor} -- "
            "update protocol-version.txt to match."
        )
    return header_version


# --- Opcodes / codes -- keep aligned with include/alp/protocol/cc3501e.h.
CMD_PING = 0x00
CMD_GET_VERSION = 0x01
CMD_RESET = 0x02
CMD_GET_MAC = 0x03
CMD_GET_PENDING_EVENTS = 0x05  # async-event queue drain (host-polled)
CMD_WIFI_SCAN_START = 0x10  # representative not-yet-implemented v1 opcode

# Async event opcodes carried inside a GET_PENDING_EVENTS reply.
EVT_WIFI_CONNECTED = 0x19
EVT_WIFI_DISCONNECTED = 0x1A
CMD_SOCK_OPEN = 0x20
CMD_SOCK_CLOSE = 0x24
CMD_SOCK_BIND = 0x25  # listening path (proto v9)
CMD_SOCK_LISTEN = 0x26
EVT_SOCK_ACCEPTED = 0x2C  # async: a client connected to a listening socket
CMD_WIFI_GET_IP = 0x17
CMD_GET_CAPABILITIES = 0x06  # wire 3.1: which opcode families this build implements

# alp_cc3501e_capability_t bits (keep aligned with the header; bits are FOREVER).
CAP_WIFI_STA = 0x00000001
CAP_WIFI_AP = 0x00000002
CAP_SOCK_CLIENT = 0x00000004
CAP_SOCK_LISTEN = 0x00000008
CAP_BLE = 0x00000010
CAP_OTA = 0x00000020
CAP_GPIO_PROXY = 0x00000040
CAP_SPI1_MASTER = 0x00000080
CAP_CAMERA = 0x00000100
CAP_POWER_POLICY = 0x00000200
CAP_DIAG_STATS = 0x00000400
CAP_EVENTS = 0x00000800
CMD_SPI1_TRANSFER = 0x56  # SPI1 host passthrough (0x55..0x57); only TRANSFER is pinned here

FLAG_SOLICITED = 0x00

RESP_OK = 0x00
RESP_ERR_INVALID = 0x01
RESP_ERR_BUSY = 0x02
RESP_ERR_NOT_READY = 0x05
RESP_ERR_PROTOCOL = 0x07

# Wire-protocol version GET_VERSION reports (ALP_CC3501E_PROTOCOL_VERSION).
# Sourced from the header, not hardcoded -- see _read_protocol_version().
PROTOCOL_VERSION = _read_protocol_version()


def frame(cmd: int, flags: int, payload: bytes = b"") -> bytes:
    """Build a cc3501e frame: cmd | flags | payload_len(LE16) | payload."""
    return bytes([cmd, flags, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload


# src/protocol.h:80.  Reply payloads are padded up to a multiple of this so the
# host DW SSI can move one burst-aligned chunk; an odd length collapses its DMA
# burst to one transaction per byte.
REPLY_PAD = 8


def reply(cmd: int, status: int, data: bytes = b"") -> bytes:
    """Build a solicited reply frame (payload = status + data, zero-padded).

    The padding is NOT cosmetic and omitting it made these vectors describe
    frames the firmware never emits: src/protocol.c:470-473 rounds every reply
    payload up to a multiple of CC3501E_REPLY_PAD, so ping_reply_ok was pinned
    at payload_len=1 while the firmware sends 8 (#11).  Pad bytes are never
    interpreted -- every reply carries its own length -- but they ARE on the
    wire, and a vector file that disagrees with the wire pins nothing.
    """
    payload = bytes([status]) + data
    rem = len(payload) % REPLY_PAD
    if rem:
        payload += bytes(REPLY_PAD - rem)
    return frame(cmd, FLAG_SOLICITED, payload)


HEADER = """\
# cc3501e-bridge canonical wire-test vectors
#
# Consumed by the firmware transport tests
# (tests/zephyr/cc3501e_bridge_transport/) and any future host-driver
# tests (tests/zephyr/chips/cc3501e/), so the two sides cannot diverge.
#
# Frame: 4-byte LE header [cmd | flags | payload_len(LE16)] + payload.
# No SOF, no CRC.  Reply payload[0] is the response status.
#
# Format: one `<name> = <hex>` vector per non-comment line; `#` comments.
# Regenerate with `python3 firmware/cc3501e/tests/gen_protocol_vectors.py`.
"""


def build_vectors() -> list[tuple[str, str, str | None]]:
    out: list[tuple[str, str, str | None]] = []

    out.append(("ping_request", frame(CMD_PING, 0).hex().upper(),
                "cmd=PING | flags=0 | len=0"))
    out.append(("ping_reply_ok", reply(CMD_PING, RESP_OK).hex().upper(),
                "cmd=PING | flags=0 | len=1 | status=OK"))

    out.append(("get_version_request", frame(CMD_GET_VERSION, 0).hex().upper(),
                "cmd=GET_VERSION | flags=0 | len=0"))
    # The reply is the COMPOSED (MAJOR << 8) | MINOR, LE16 -- so wire 3.1 is
    # bytes 01 03, not 09.  Naming the vector after the human form keeps the
    # file readable by whoever is holding a board (ADR 0033).
    _major, _minor = PROTOCOL_VERSION >> 8, PROTOCOL_VERSION & 0xFF
    out.append((
        f"get_version_reply_wire{_major}_{_minor}",
        reply(CMD_GET_VERSION, RESP_OK,
              bytes([PROTOCOL_VERSION & 0xFF, (PROTOCOL_VERSION >> 8) & 0xFF])).hex().upper(),
        f"cmd=GET_VERSION | len=3 | status=OK | wire={_major}.{_minor} "
        f"= 0x{PROTOCOL_VERSION:04X} (LE16)",
    ))
    # A firmware from BEFORE ADR 0033 answers with its raw v1..v9 integer, which
    # decodes to MAJOR 0 -- pinned here because the host relies on that being
    # distinguishable to say "older than the scheme" instead of "corrupt".
    out.append((
        "get_version_reply_legacy_raw_v9",
        reply(CMD_GET_VERSION, RESP_OK, bytes([0x09, 0x00])).hex().upper(),
        "cmd=GET_VERSION | status=OK | legacy pre-ADR-0033 firmware: raw 9 -> major 0",
    ))

    # GET_CAPABILITIES (0x06, wire 3.1): reply DATA is
    # alp_cc3501e_capabilities_t { caps(LE32) | reserved(LE32) }.
    out.append(("get_capabilities_request", frame(CMD_GET_CAPABILITIES, 0).hex().upper(),
                "cmd=GET_CAPABILITIES | flags=0 | len=0"))
    # The STUB build's honest answer: EVENTS | DIAG_STATS only -- no HAL, so no
    # OTA/GPIO/camera/power, and no CC3501E_WIFI/BLE so no radio families.
    _stub_caps = CAP_EVENTS | CAP_DIAG_STATS
    out.append((
        "get_capabilities_reply_stub_build",
        reply(CMD_GET_CAPABILITIES, RESP_OK,
              _stub_caps.to_bytes(4, "little") + (0).to_bytes(4, "little")).hex().upper(),
        f"cmd=GET_CAPABILITIES | status=OK | caps=0x{_stub_caps:08X} "
        "(EVENTS|DIAG_STATS -- the stub backend implements nothing else)",
    ))

    out.append(("get_mac_request", frame(CMD_GET_MAC, 0).hex().upper(),
                "cmd=GET_MAC | flags=0 | len=0"))
    # GET_MAC is async (P0-4/P0-6): the first request submits the worker job
    # and replies BUSY; the host re-issues until the worker has the result.
    out.append(("get_mac_reply_busy_submitted", reply(CMD_GET_MAC, RESP_ERR_BUSY).hex().upper(),
                "cmd=GET_MAC | len=1 | status=BUSY -- job submitted, host re-issues"))
    out.append(("get_mac_reply_not_ready_stub", reply(CMD_GET_MAC, RESP_ERR_NOT_READY).hex().upper(),
                "cmd=GET_MAC | len=1 | status=NOT_READY -- re-issued; stub has no radio"))

    out.append(("reset_request", frame(CMD_RESET, 0).hex().upper(),
                "cmd=RESET | flags=0 | len=0"))
    out.append(("reset_reply_ok", reply(CMD_RESET, RESP_OK).hex().upper(),
                "cmd=RESET | len=1 | status=OK -- firmware reboots after the ack is read"))

    out.append((
        "wifi_scan_start_reply_invalid",
        reply(CMD_WIFI_SCAN_START, RESP_ERR_INVALID).hex().upper(),
        "cmd=WIFI_SCAN_START | len=1 | status=INVALID -- v1 opcode not implemented in v0.1",
    ))

    # TCP/UDP sockets (0x20..0x24): worker-routed, poll-by-repeat like GET_MAC.
    # SOCK_OPEN req = sock_open_t { family=IPV4(0) | type=STREAM(0) | proto=0 | rsvd }.
    out.append(("sock_open_tcp_request",
                frame(CMD_SOCK_OPEN, 0, bytes([0x00, 0x00, 0x00, 0x00])).hex().upper(),
                "cmd=SOCK_OPEN | len=4 | family=IPV4 type=STREAM proto=0"))
    # First request submits the worker job and replies BUSY; host re-issues.
    out.append(("sock_open_reply_busy_submitted", reply(CMD_SOCK_OPEN, RESP_ERR_BUSY).hex().upper(),
                "cmd=SOCK_OPEN | len=1 | status=BUSY -- job submitted, host re-issues"))
    # Re-issued on the stub (no IP stack) -> NOT_READY.
    out.append(("sock_open_reply_not_ready_stub",
                reply(CMD_SOCK_OPEN, RESP_ERR_NOT_READY).hex().upper(),
                "cmd=SOCK_OPEN | len=1 | status=NOT_READY -- re-issued; stub has no IP stack"))
    # SOCK_CLOSE req = sock_close_t { handle(LE16)=1 | reserved(LE16) } = 4 B.
    out.append(("sock_close_request",
                frame(CMD_SOCK_CLOSE, 0, bytes([0x01, 0x00, 0x00, 0x00])).hex().upper(),
                "cmd=SOCK_CLOSE | len=4 | handle=1"))
    # Bad length: a 3-byte SOCK_OPEN payload is rejected up front (not worker-routed).
    out.append(("sock_open_bad_len_reply_invalid",
                reply(CMD_SOCK_OPEN, RESP_ERR_INVALID).hex().upper(),
                "cmd=SOCK_OPEN | len=1 | status=INVALID -- payload length != sizeof(sock_open_t)"))

    # Listening path (proto v9).  SOCK_BIND req = sock_bind_t, byte-for-byte the
    # SOCK_CONNECT layout with the LOCAL endpoint: handle(LE16)=1 | reserved(2) |
    # local sock_addr { family=IPV4 | reserved | port(LE16)=80 | addr[16] }.
    # addr all-zero = INADDR_ANY, which is what a server on the soft-AP binds.
    sock_bind_payload = (bytes([0x01, 0x00, 0x00, 0x00])
                         + bytes([0x00, 0x00, 0x50, 0x00])
                         + bytes(16))
    out.append(("sock_bind_request",
                frame(CMD_SOCK_BIND, 0, sock_bind_payload).hex().upper(),
                "cmd=SOCK_BIND | len=24 | handle=1 family=IPV4 port=80 addr=INADDR_ANY"))
    out.append(("sock_bind_reply_busy_submitted", reply(CMD_SOCK_BIND, RESP_ERR_BUSY).hex().upper(),
                "cmd=SOCK_BIND | len=1 | status=BUSY -- job submitted, host re-issues"))
    # SOCK_LISTEN req = sock_listen_t { handle(LE16)=1 | backlog=4 | reserved }.
    out.append(("sock_listen_request",
                frame(CMD_SOCK_LISTEN, 0, bytes([0x01, 0x00, 0x04, 0x00])).hex().upper(),
                "cmd=SOCK_LISTEN | len=4 | handle=1 backlog=4"))
    out.append(("sock_listen_reply_not_ready_stub",
                reply(CMD_SOCK_LISTEN, RESP_ERR_NOT_READY).hex().upper(),
                "cmd=SOCK_LISTEN | len=1 | status=NOT_READY -- re-issued; stub has no IP stack"))
    # Handle 0 is the invalid handle, rejected up front (not worker-routed).
    out.append(("sock_listen_handle_zero_reply_invalid",
                reply(CMD_SOCK_LISTEN, RESP_ERR_INVALID).hex().upper(),
                "cmd=SOCK_LISTEN | len=1 | status=INVALID -- handle 0 is never valid"))

    # WIFI_GET_IP (0x17) with the v9 interface selector.  A ZERO-length request
    # keeps its pre-v9 meaning (STA); one byte selects the interface, and AP is
    # the address a serving application binds to.
    out.append(("wifi_get_ip_request_sta_legacy", frame(CMD_WIFI_GET_IP, 0).hex().upper(),
                "cmd=WIFI_GET_IP | len=0 | pre-v9 form, still means STA"))
    out.append(("wifi_get_ip_request_ap", frame(CMD_WIFI_GET_IP, 0, bytes([0x01])).hex().upper(),
                "cmd=WIFI_GET_IP | len=1 | iface=AP (v9)"))

    # Async-event queue drain (0x05, proto v3): the reply DATA is a packed list
    # of { evt_opcode(1) | len(1) | payload[len] } entries.
    out.append(("get_pending_events_request", frame(CMD_GET_PENDING_EVENTS, 0).hex().upper(),
                "cmd=GET_PENDING_EVENTS | flags=0 | len=0"))
    # Empty ring: status OK with zero data bytes.
    out.append(("get_pending_events_reply_empty",
                reply(CMD_GET_PENDING_EVENTS, RESP_OK).hex().upper(),
                "cmd=GET_PENDING_EVENTS | len=1 | status=OK | no events queued"))
    # Two payloadless events queued: EVT_WIFI_CONNECTED then EVT_WIFI_DISCONNECTED,
    # each { evt_opcode | len=0 } back to back in the reply DATA.
    out.append((
        "get_pending_events_reply_wifi_conn_disc",
        reply(CMD_GET_PENDING_EVENTS, RESP_OK,
              bytes([EVT_WIFI_CONNECTED, 0x00, EVT_WIFI_DISCONNECTED, 0x00])).hex().upper(),
        "cmd=GET_PENDING_EVENTS | status=OK | [WIFI_CONNECTED len0][WIFI_DISCONNECTED len0]",
    ))

    # One EVT_SOCK_ACCEPTED entry (proto v9): { opcode | len=12 | payload }, the
    # payload being alp_cc3501e_sock_accepted_evt_t -- listen_handle(LE16)=1 |
    # handle(LE16)=2 | peer_port(LE16, host order)=54321 | peer_family=IPV4 |
    # reserved | peer_addr[4]=192.168.1.14 (network order, MSB first).  12 bytes
    # is under the firmware ring's 16-byte per-entry payload cap, which is why
    # the peer address is carried in this compact form rather than as a
    # 20-byte sock_addr_t.
    sock_accepted_payload = (bytes([0x01, 0x00, 0x02, 0x00, 0x31, 0xD4, 0x00, 0x00])
                             + bytes([192, 168, 1, 14]))
    out.append((
        "get_pending_events_reply_sock_accepted",
        reply(CMD_GET_PENDING_EVENTS, RESP_OK,
              bytes([EVT_SOCK_ACCEPTED, 0x0C]) + sock_accepted_payload).hex().upper(),
        "cmd=GET_PENDING_EVENTS | status=OK | [SOCK_ACCEPTED len12 listen=1 handle=2 "
        "port=54321 192.168.1.14]",
    ))

    # SPI1 host passthrough (0x55..0x57): the only vector here is TRANSFER, and it
    # is the only executable check of the inline-TX request framing (the TX bytes
    # are packed straight after the 8-byte header, no padding) and the
    # self-delimiting RX reply framing (the reply's own len field is what lets a
    # reader stop at the real RX bytes instead of reading into the REPLY_PAD
    # padding).  len=4, flags=0 (single-shot), seq=1; tx=DEADBEEF looped back on
    # rx to match the stub HAL's wire-loop SPI1 body (tests/unit/transport_spi).
    spi1_xfer_tx = bytes([0xDE, 0xAD, 0xBE, 0xEF])
    out.append((
        "spi1_transfer_request",
        frame(CMD_SPI1_TRANSFER, 0,
              bytes([0x04, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00]) + spi1_xfer_tx).hex().upper(),
        "cmd=SPI1_TRANSFER | len=4 flags=0 seq=1 tx_fill=0 | tx=DEADBEEF packed inline, no padding",
    ))
    out.append((
        "spi1_transfer_reply_ok",
        reply(CMD_SPI1_TRANSFER, RESP_OK,
              bytes([0x04, 0x00, 0x00, 0x01]) + spi1_xfer_tx).hex().upper(),
        "cmd=SPI1_TRANSFER | status=OK | rx_len=4 flags=0 seq=1 | rx=DEADBEEF, self-delimiting "
        "before the REPLY_PAD padding",
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
