#!/usr/bin/env python3
# Copyright 2026 Alp Lab AB
# SPDX-License-Identifier: Apache-2.0
"""
Regenerate firmware/cc3501e/tests/protocol_vectors.txt (and, with --legacy,
tests/protocol_vectors_no_crc.txt).

Authoritative, human-readable source-of-truth for the cc3501e-bridge
wire frames, shared by the firmware transport tests
(tests/zephyr/cc3501e_bridge_transport/) and any future host-driver tests
under tests/zephyr/chips/cc3501e/.

The cc3501e frame is a 4-byte little-endian header + payload, with NO
start-of-frame byte (a short hardwired point-to-point link).  Wire MAJOR 4
(#2035) adds a mandatory 2-byte CRC-16/CCITT-FALSE trailer to every frame,
both directions, INSIDE the declared payload_len -- covering the header
plus every payload byte except the trailing 2 CRC bytes.  This firmware is
the strict side of that migration (CRC required unconditionally, no
dual-mode fallback) with ONE exception: CMD_GET_VERSION is accepted with or
without the trailer, because a host cannot know to send one before
GET_VERSION has told it the peer's wire major -- see
src/protocol.c's protocol_build_reply() and this file's
get_version_request_no_crc / get_version_request_crc vectors below.

    REQUEST : cmd | flags | payload_len(LE16) | payload[payload_len-2] | crc(LE16)
    REPLY   : cmd | flags | payload_len(LE16) | status | data[...] | pad | crc(LE16)

The reply echoes the request cmd, uses flags=0 (solicited), and carries
the response status (ALP_CC3501E_RESP_*) as the first payload byte.

CC3501E_WIRE_CRC=OFF (customer-selectable, CMakeLists.txt) is a DIFFERENT
wire shape, not the shape above with the trailer dropped: it is wire MAJOR 3
(ALP_CC3501E_PROTOCOL_MAJOR_LEGACY), byte-identical to the 3.1 wire this
firmware spoke before #2035 -- no trailer on either direction, and
RESP_OK_LEGACY (0x00) is the wire OK byte instead of RESP_OK (0x5A).  Pass
--legacy to generate THAT shape instead (see build_vectors(crc_enabled=)).

Run from the alp-sdk repo root:

    python3 firmware/cc3501e/tests/gen_protocol_vectors.py
    python3 firmware/cc3501e/tests/gen_protocol_vectors.py --legacy

Use --check in CI to fail on drift (run once per shape).
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
# The oldest major the (CRC-on) firmware/host still speak -- the CC3501E_WIRE_CRC=OFF
# build reports THIS major from GET_VERSION, not ALP_CC3501E_PROTOCOL_MAJOR.
_MAJOR_LEGACY_RE = re.compile(r"^#define\s+ALP_CC3501E_PROTOCOL_MAJOR_LEGACY\s+(\d+)", re.MULTILINE)


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


def _read_protocol_major_legacy() -> int:
    """ALP_CC3501E_PROTOCOL_MAJOR_LEGACY -- the wire major a CC3501E_WIRE_CRC=OFF
    firmware build reports from GET_VERSION instead of ALP_CC3501E_PROTOCOL_MAJOR.
    Parsed the same way _read_protocol_version() parses MAJOR/MINOR, so a header
    that drops or renumbers this constant fails --check loudly instead of the
    legacy vectors silently going stale."""
    text = _HEADER.read_text(encoding="utf-8")
    m = _MAJOR_LEGACY_RE.search(text)
    if not m:
        sys.exit(f"cannot find #define ALP_CC3501E_PROTOCOL_MAJOR_LEGACY in {_HEADER}.")
    return int(m.group(1))


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

# RESP_* and CRC_BYTES below are DUPLICATED from <alp/protocol/cc3501e.h> /
# <alp/protocol/crc16.h> (same discipline as the CMD_* opcodes above: "keep
# aligned with the header") -- they must move together with the header, by
# hand, the same change that bumps ALP_CC3501E_PROTOCOL_MAJOR/_MINOR.  There
# is no automated cross-check for these values the way _read_protocol_version()
# cross-checks the MAJOR.MINOR pair; a future drift-catcher belongs there too.
#
# Wire MAJOR 4 (#2035) moves the success status off 0x00 -- indistinguishable
# from a dead SPI phase clocking back nothing (#1378) -- to 0x5A.
# RESP_OK_LEGACY (0x00) survives only to describe the PRE-MAJOR-4 wire shape a
# real 4.0 firmware never emits (see get_version_reply_legacy_raw_v9 below,
# which models a firmware that predates the MAJOR.MINOR scheme entirely).
RESP_OK = 0x5A
RESP_OK_LEGACY = 0x00
RESP_ERR_INVALID = 0x01
RESP_ERR_BUSY = 0x02
RESP_ERR_NOT_READY = 0x05
RESP_ERR_PROTOCOL = 0x07

# <alp/protocol/cc3501e.h>'s ALP_CC3501E_CRC_BYTES: the wire MAJOR 4 trailer
# size, both directions.
CRC_BYTES = 2

# Wire-protocol version GET_VERSION reports (ALP_CC3501E_PROTOCOL_VERSION).
# Sourced from the header, not hardcoded -- see _read_protocol_version().
PROTOCOL_VERSION = _read_protocol_version()

# Wire-protocol version a CC3501E_WIRE_CRC=OFF firmware build reports instead:
# MAJOR_LEGACY.1 -- src/protocol_meta.c's CC3501E_FW_WIRE_VERSION.  The MINOR
# half (1) is NOT in the header any more (retired when the 4.0 bump reset
# ALP_CC3501E_PROTOCOL_MINOR to 0) -- it is the last minor wire 3.x actually
# held ("v9 = 3.1" in the header's version history), pinned here the same way
# the firmware pins it, rather than derived.
LEGACY_VERSION = (_read_protocol_major_legacy() << 8) | 1


def crc16_ccitt_false(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, non-reflected, no final
    XOR) -- the SAME algorithm <alp/protocol/crc16.h>'s alp_crc16_ccitt_false()
    implements, reimplemented here in Python because this generator has no C
    toolchain to call into.  Chainable: pass a prior call's return as `crc` to
    CRC a second buffer as if concatenated (mirrors alp_crc16_ccitt_false_update()).
    """
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def frame(cmd: int, flags: int, payload: bytes = b"") -> bytes:
    """Build a wire-MAJOR-4 REQUEST frame: header + payload + the mandatory
    2-byte CRC-16/CCITT-FALSE trailer (LE), covering the header + payload.

    Every call site below passes the opcode's actual (pre-MAJOR-4-shaped)
    payload; this appends the trailer and folds its 2 bytes into the
    declared payload_len, matching what src/protocol.c's protocol_build_reply()
    requires of every request except CMD_GET_VERSION (see
    get_version_request_no_crc for that one deliberate exception).
    """
    wire_len = len(payload) + CRC_BYTES
    header = bytes([cmd, flags, wire_len & 0xFF, (wire_len >> 8) & 0xFF])
    crc = crc16_ccitt_false(header + payload)
    return header + payload + crc.to_bytes(2, "little")


# src/protocol.h:CC3501E_REPLY_PAD.  Reply payloads are padded up to a
# multiple of this so the host DW SSI can move one burst-aligned chunk; an
# odd length collapses its DMA burst to one transaction per byte.
REPLY_PAD = 8


def reply(cmd: int, status: int, data: bytes = b"") -> bytes:
    """Build a wire-MAJOR-4 solicited REPLY frame: payload = status + data,
    zero-padded up to a multiple of REPLY_PAD, with the mandatory 2-byte
    CRC-16/CCITT-FALSE trailer occupying the LAST 2 bytes of that padded
    span (<alp/protocol/cc3501e.h>'s ALP_CC3501E_CRC_BYTES doc) -- so a
    bare-status reply, which already padded from 1 to REPLY_PAD bytes before
    MAJOR 4, costs zero extra wire bytes: the CRC simply consumes 2 of what
    used to be 7 pad bytes.

    The padding is NOT cosmetic and omitting it made these vectors describe
    frames the firmware never emits: src/protocol.c's protocol_build_reply()
    rounds every reply payload up to a multiple of CC3501E_REPLY_PAD, so
    ping_reply_ok was pinned at payload_len=1 while the firmware sends 8
    (#11).  Pad bytes are never interpreted -- every reply carries its own
    length -- but they ARE on the wire, and a vector file that disagrees
    with the wire pins nothing.
    """
    body = bytes([status]) + data
    unpadded = len(body) + CRC_BYTES
    rem = unpadded % REPLY_PAD
    pad = 0 if rem == 0 else REPLY_PAD - rem
    body_padded = body + bytes(pad)
    crc = crc16_ccitt_false(frame_header(cmd, FLAG_SOLICITED, len(body_padded) + CRC_BYTES) + body_padded)
    return frame_header(cmd, FLAG_SOLICITED, len(body_padded) + CRC_BYTES) + body_padded + crc.to_bytes(
        2, "little"
    )


def frame_header(cmd: int, flags: int, payload_len: int) -> bytes:
    """The bare 4-byte LE header, split out so reply() can build it once to
    CRC and once to prepend without duplicating the packing arithmetic."""
    return bytes([cmd, flags, payload_len & 0xFF, (payload_len >> 8) & 0xFF])


def legacy_reply(cmd: int, status: int, data: bytes = b"") -> bytes:
    """Build a PRE-MAJOR-4 (wire <= 3.1) solicited reply: payload = status +
    data, zero-padded, NO CRC trailer -- what a firmware from before this
    bump (or before the MAJOR.MINOR scheme existed at all) actually emits.
    Only get_version_reply_legacy_raw_v9 uses this: it models a DIFFERENT,
    older firmware's reply, not something this repo's current firmware ever
    sends, so it must NOT gain a wire-MAJOR-4 CRC trailer the way reply()
    would give it."""
    payload = bytes([status]) + data
    rem = len(payload) % REPLY_PAD
    if rem:
        payload += bytes(REPLY_PAD - rem)
    return frame_header(cmd, FLAG_SOLICITED, len(payload)) + payload


def request_no_crc(cmd: int, flags: int, payload: bytes = b"") -> bytes:
    """Build a PRE-MAJOR-4-shaped (wire 3.1) request: header + payload, NO
    CRC trailer.  Used only for the deliberate CMD_GET_VERSION exception
    (get_version_request_no_crc) and to show what a rejected legacy request
    looks like -- see src/protocol.c's protocol_build_reply() for exactly
    which opcode accepts this shape and which reject it with
    RESP_ERR_PROTOCOL."""
    return frame_header(cmd, flags, len(payload)) + payload


HEADER = """\
# cc3501e-bridge canonical wire-test vectors
#
# Consumed by the firmware transport tests
# (tests/zephyr/cc3501e_bridge_transport/) and any future host-driver
# tests (tests/zephyr/chips/cc3501e/), so the two sides cannot diverge.
#
# Frame: 4-byte LE header [cmd | flags | payload_len(LE16)] + payload.  No
# SOF.  Wire MAJOR 4 (#2035) adds a mandatory 2-byte CRC-16/CCITT-FALSE
# trailer to every frame, both directions, INSIDE payload_len -- see
# src/protocol.c's protocol_build_reply().  CMD_GET_VERSION is the ONE
# opcode accepted with or without that trailer (a host cannot know to send
# one before GET_VERSION has told it the peer's wire major); every other
# opcode requires it unconditionally.  Reply payload[0] is the response
# status; RESP_OK is 0x5A as of wire MAJOR 4 (was 0x00).
#
# Format: one `<name> = <hex>` vector per non-comment line; `#` comments.
# Regenerate with `python3 firmware/cc3501e/tests/gen_protocol_vectors.py`.
"""

HEADER_LEGACY = """\
# cc3501e-bridge canonical wire-test vectors -- CC3501E_WIRE_CRC=OFF
#
# Consumed by the firmware transport tests (tests/unit/transport_spi/), so
# the CRC-off build cannot silently drift from the shape it promises: BYTE-
# IDENTICAL to the wire 3.1 frames this firmware shipped before #2035.
#
# Frame: 4-byte LE header [cmd | flags | payload_len(LE16)] + payload.  No
# SOF, NO CRC trailer on either direction -- see src/protocol.c's
# protocol_build_reply() #else arm (CC3501E_WIRE_CRC=0).  CMD_GET_VERSION is
# still accepted with OR without an (unverified-by-need, but still checked
# when present) CRC trailer even on this wire -- see get_version_request_crc
# below and this repo's CC3501E_WIRE_CRC option comment in CMakeLists.txt.
# Reply payload[0] is the response status; RESP_OK_LEGACY (0x00) is the wire
# OK byte on this shape, not RESP_OK (0x5A).
#
# Format: one `<name> = <hex>` vector per non-comment line; `#` comments.
# Regenerate with `python3 firmware/cc3501e/tests/gen_protocol_vectors.py --legacy`.
"""


def build_vectors(crc_enabled: bool = True) -> list[tuple[str, str, str | None]]:
    """crc_enabled selects which CC3501E_WIRE_CRC build's wire shape to emit:
    True (default) = wire MAJOR 4 -- every request/reply carries the mandatory
    2-byte CRC-16/CCITT-FALSE trailer, RESP_OK is 0x5A.  False = wire MAJOR 3
    legacy (CC3501E_WIRE_CRC=OFF, CMakeLists.txt) -- byte-identical to the 3.1
    wire: no trailer on any frame, RESP_OK_LEGACY (0x00) is the wire OK byte.

    Both shapes come out of this ONE generator, opcode list and payload set,
    so the two builds cannot silently drift into describing two DIFFERENT
    protocols -- only the framing (_frame/_reply/_resp_ok/_version below)
    differs, the same substitution src/protocol.c's two protocol_build_reply()
    arms make in the firmware itself."""
    out: list[tuple[str, str, str | None]] = []

    _frame = frame if crc_enabled else request_no_crc
    _reply = reply if crc_enabled else legacy_reply
    _resp_ok = RESP_OK if crc_enabled else RESP_OK_LEGACY
    _version = PROTOCOL_VERSION if crc_enabled else LEGACY_VERSION

    out.append(("ping_request", _frame(CMD_PING, 0).hex().upper(),
                "cmd=PING | flags=0 | len=0"))
    out.append(("ping_reply_ok", _reply(CMD_PING, _resp_ok).hex().upper(),
                "cmd=PING | flags=0 | len=1 | status=OK"))

    # GET_VERSION's dual-shape carve-out (#2035) holds in BOTH builds, not just
    # the CRC-on one: a host cannot know a peer's wire major before GET_VERSION
    # answers, so its very first request to ANY fresh peer is, by construction,
    # this exact zero-payload, no-trailer shape -- both builds must accept it or
    # no host could ever discover them.  request_no_crc()/frame() (not the
    # active _frame) are used explicitly here because this pair of vectors
    # demonstrates BOTH shapes are accepted regardless of which build is
    # running, not just the active one.  See src/protocol.c's
    # protocol_build_reply() (both #if CC3501E_WIRE_CRC arms) for where this
    # carve-out lives on the firmware side.
    out.append(("get_version_request_no_crc", request_no_crc(CMD_GET_VERSION, 0).hex().upper(),
                "cmd=GET_VERSION | flags=0 | len=0 -- the CRC-less bootstrap shape every host "
                "sends before it knows the peer's wire major; accepted, not rejected, in BOTH "
                "the CRC-on and CRC-off (legacy) builds"))
    # A host with a STALE cached major (e.g. this board was OTA'd DOWN from a
    # MAJOR-4 image to a CRC-off/legacy one) may instead send the ordinary
    # CRC'd shape -- both builds also accept THIS (payload_len=2, logical
    # payload empty), so that host can still rediscover the board's real major
    # instead of being permanently stranded.
    out.append(("get_version_request_crc", frame(CMD_GET_VERSION, 0).hex().upper(),
                "cmd=GET_VERSION | flags=0 | len=2 (CRC only, no logical payload) -- the "
                "MAJOR-4-negotiated shape; ALSO accepted by a CRC-off (legacy) build, so a "
                "host with stale cached state can still rediscover a downgraded board"))
    # The reply is the COMPOSED (MAJOR << 8) | MINOR, LE16 -- so wire 4.0 is
    # bytes 00 04, not 09, and legacy 3.1 is bytes 01 03.  Naming the vector
    # after the human form keeps the file readable by whoever is holding a
    # board (ADR 0033).  The REPLY always follows the ACTIVE build's shape
    # (_reply/_resp_ok/_version) regardless of which request shape arrived --
    # a legacy build never answers with a CRC trailer just because the
    # request happened to carry one.
    _major, _minor = _version >> 8, _version & 0xFF
    out.append((
        f"get_version_reply_wire{_major}_{_minor}",
        _reply(CMD_GET_VERSION, _resp_ok,
              bytes([_version & 0xFF, (_version >> 8) & 0xFF])).hex().upper(),
        f"cmd=GET_VERSION | len=3 | status=OK | wire={_major}.{_minor} "
        f"= 0x{_version:04X} (LE16)",
    ))
    # A firmware from BEFORE ADR 0033 (and before wire MAJOR 4's CRC trailer)
    # answers with its raw v1..v9 integer over the LEGACY (no-CRC, RESP_OK_LEGACY
    # 0x00) shape, which decodes to MAJOR 0 -- pinned here because the host
    # relies on that being distinguishable to say "older than the scheme"
    # instead of "corrupt".  legacy_reply(), not _reply(): this models a
    # DIFFERENT, older (pre-ADR-0033) firmware's wire shape unconditionally --
    # not what either of THIS repo's two current build options emits -- so it
    # does not vary with crc_enabled. */
    out.append((
        "get_version_reply_legacy_raw_v9",
        legacy_reply(CMD_GET_VERSION, RESP_OK_LEGACY, bytes([0x09, 0x00])).hex().upper(),
        "cmd=GET_VERSION | status=OK(legacy 0x00) | pre-ADR-0033 firmware (predates even the "
        "MAJOR.MINOR scheme, not just the CRC trailer): raw 9 -> major 0, no CRC trailer",
    ))

    # GET_CAPABILITIES (0x06, wire 3.1): reply DATA is
    # alp_cc3501e_capabilities_t { caps(LE32) | reserved(LE32) }.
    out.append(("get_capabilities_request", _frame(CMD_GET_CAPABILITIES, 0).hex().upper(),
                "cmd=GET_CAPABILITIES | flags=0 | len=0"))
    # The STUB build's honest answer: EVENTS | DIAG_STATS only -- no HAL, so no
    # OTA/GPIO/camera/power, and no CC3501E_WIFI/BLE so no radio families.  The
    # capability BITMAP itself does not depend on CC3501E_WIRE_CRC -- that
    # option picks a wire shape, not which opcode families link.
    _stub_caps = CAP_EVENTS | CAP_DIAG_STATS
    out.append((
        "get_capabilities_reply_stub_build",
        _reply(CMD_GET_CAPABILITIES, _resp_ok,
              _stub_caps.to_bytes(4, "little") + (0).to_bytes(4, "little")).hex().upper(),
        f"cmd=GET_CAPABILITIES | status=OK | caps=0x{_stub_caps:08X} "
        "(EVENTS|DIAG_STATS -- the stub backend implements nothing else)",
    ))

    out.append(("get_mac_request", _frame(CMD_GET_MAC, 0).hex().upper(),
                "cmd=GET_MAC | flags=0 | len=0"))
    # GET_MAC is async (P0-4/P0-6): the first request submits the worker job
    # and replies BUSY; the host re-issues until the worker has the result.
    out.append(("get_mac_reply_busy_submitted", _reply(CMD_GET_MAC, RESP_ERR_BUSY).hex().upper(),
                "cmd=GET_MAC | len=1 | status=BUSY -- job submitted, host re-issues"))
    out.append(("get_mac_reply_not_ready_stub", _reply(CMD_GET_MAC, RESP_ERR_NOT_READY).hex().upper(),
                "cmd=GET_MAC | len=1 | status=NOT_READY -- re-issued; stub has no radio"))

    out.append(("reset_request", _frame(CMD_RESET, 0).hex().upper(),
                "cmd=RESET | flags=0 | len=0"))
    out.append(("reset_reply_ok", _reply(CMD_RESET, _resp_ok).hex().upper(),
                "cmd=RESET | len=1 | status=OK -- firmware reboots after the ack is read"))

    out.append((
        "wifi_scan_start_reply_invalid",
        _reply(CMD_WIFI_SCAN_START, RESP_ERR_INVALID).hex().upper(),
        "cmd=WIFI_SCAN_START | len=1 | status=INVALID -- v1 opcode not implemented in v0.1",
    ))

    # TCP/UDP sockets (0x20..0x24): worker-routed, poll-by-repeat like GET_MAC.
    # SOCK_OPEN req = sock_open_t { family=IPV4(0) | type=STREAM(0) | proto=0 | rsvd }.
    out.append(("sock_open_tcp_request",
                _frame(CMD_SOCK_OPEN, 0, bytes([0x00, 0x00, 0x00, 0x00])).hex().upper(),
                "cmd=SOCK_OPEN | len=4 | family=IPV4 type=STREAM proto=0"))
    # First request submits the worker job and replies BUSY; host re-issues.
    out.append(("sock_open_reply_busy_submitted", _reply(CMD_SOCK_OPEN, RESP_ERR_BUSY).hex().upper(),
                "cmd=SOCK_OPEN | len=1 | status=BUSY -- job submitted, host re-issues"))
    # Re-issued on the stub (no IP stack) -> NOT_READY.
    out.append(("sock_open_reply_not_ready_stub",
                _reply(CMD_SOCK_OPEN, RESP_ERR_NOT_READY).hex().upper(),
                "cmd=SOCK_OPEN | len=1 | status=NOT_READY -- re-issued; stub has no IP stack"))
    # SOCK_CLOSE req = sock_close_t { handle(LE16)=1 | reserved(LE16) } = 4 B.
    out.append(("sock_close_request",
                _frame(CMD_SOCK_CLOSE, 0, bytes([0x01, 0x00, 0x00, 0x00])).hex().upper(),
                "cmd=SOCK_CLOSE | len=4 | handle=1"))
    # Bad length: a 3-byte SOCK_OPEN payload is rejected up front (not worker-routed).
    out.append(("sock_open_bad_len_reply_invalid",
                _reply(CMD_SOCK_OPEN, RESP_ERR_INVALID).hex().upper(),
                "cmd=SOCK_OPEN | len=1 | status=INVALID -- payload length != sizeof(sock_open_t)"))

    # Listening path (proto v9).  SOCK_BIND req = sock_bind_t, byte-for-byte the
    # SOCK_CONNECT layout with the LOCAL endpoint: handle(LE16)=1 | reserved(2) |
    # local sock_addr { family=IPV4 | reserved | port(LE16)=80 | addr[16] }.
    # addr all-zero = INADDR_ANY, which is what a server on the soft-AP binds.
    sock_bind_payload = (bytes([0x01, 0x00, 0x00, 0x00])
                         + bytes([0x00, 0x00, 0x50, 0x00])
                         + bytes(16))
    out.append(("sock_bind_request",
                _frame(CMD_SOCK_BIND, 0, sock_bind_payload).hex().upper(),
                "cmd=SOCK_BIND | len=24 | handle=1 family=IPV4 port=80 addr=INADDR_ANY"))
    out.append(("sock_bind_reply_busy_submitted", _reply(CMD_SOCK_BIND, RESP_ERR_BUSY).hex().upper(),
                "cmd=SOCK_BIND | len=1 | status=BUSY -- job submitted, host re-issues"))
    # SOCK_LISTEN req = sock_listen_t { handle(LE16)=1 | backlog=4 | reserved }.
    out.append(("sock_listen_request",
                _frame(CMD_SOCK_LISTEN, 0, bytes([0x01, 0x00, 0x04, 0x00])).hex().upper(),
                "cmd=SOCK_LISTEN | len=4 | handle=1 backlog=4"))
    out.append(("sock_listen_reply_not_ready_stub",
                _reply(CMD_SOCK_LISTEN, RESP_ERR_NOT_READY).hex().upper(),
                "cmd=SOCK_LISTEN | len=1 | status=NOT_READY -- re-issued; stub has no IP stack"))
    # Handle 0 is the invalid handle, rejected up front (not worker-routed).
    out.append(("sock_listen_handle_zero_reply_invalid",
                _reply(CMD_SOCK_LISTEN, RESP_ERR_INVALID).hex().upper(),
                "cmd=SOCK_LISTEN | len=1 | status=INVALID -- handle 0 is never valid"))

    # WIFI_GET_IP (0x17) with the v9 interface selector.  A ZERO-length request
    # keeps its pre-v9 meaning (STA); one byte selects the interface, and AP is
    # the address a serving application binds to.
    out.append(("wifi_get_ip_request_sta_legacy", _frame(CMD_WIFI_GET_IP, 0).hex().upper(),
                "cmd=WIFI_GET_IP | len=0 | pre-v9 form, still means STA"))
    out.append(("wifi_get_ip_request_ap", _frame(CMD_WIFI_GET_IP, 0, bytes([0x01])).hex().upper(),
                "cmd=WIFI_GET_IP | len=1 | iface=AP (v9)"))

    # Async-event queue drain (0x05, proto v3): the reply DATA is a packed list
    # of { evt_opcode(1) | len(1) | payload[len] } entries.
    out.append(("get_pending_events_request", _frame(CMD_GET_PENDING_EVENTS, 0).hex().upper(),
                "cmd=GET_PENDING_EVENTS | flags=0 | len=0"))
    # Empty ring: status OK with zero data bytes.
    out.append(("get_pending_events_reply_empty",
                _reply(CMD_GET_PENDING_EVENTS, _resp_ok).hex().upper(),
                "cmd=GET_PENDING_EVENTS | len=1 | status=OK | no events queued"))
    # Two payloadless events queued: EVT_WIFI_CONNECTED then EVT_WIFI_DISCONNECTED,
    # each { evt_opcode | len=0 } back to back in the reply DATA.
    out.append((
        "get_pending_events_reply_wifi_conn_disc",
        _reply(CMD_GET_PENDING_EVENTS, _resp_ok,
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
        _reply(CMD_GET_PENDING_EVENTS, _resp_ok,
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
        _frame(CMD_SPI1_TRANSFER, 0,
              bytes([0x04, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00]) + spi1_xfer_tx).hex().upper(),
        "cmd=SPI1_TRANSFER | len=4 flags=0 seq=1 tx_fill=0 | tx=DEADBEEF packed inline, no padding",
    ))
    out.append((
        "spi1_transfer_reply_ok",
        _reply(CMD_SPI1_TRANSFER, _resp_ok,
              bytes([0x04, 0x00, 0x00, 0x01]) + spi1_xfer_tx).hex().upper(),
        "cmd=SPI1_TRANSFER | status=OK | rx_len=4 flags=0 seq=1 | rx=DEADBEEF, self-delimiting "
        "before the REPLY_PAD padding",
    ))

    # Framing error: declared payload_len doesn't match the captured bytes.
    # Unchanged by CC3501E_WIRE_CRC -- this check runs BEFORE the (CRC-on
    # build's) CRC check, so a frame that fails it never reaches CRC
    # verification at all, and the CRC-off build never has a CRC check to skip.
    out.append((
        "ping_bad_len_reply_protocol",
        _reply(CMD_PING, RESP_ERR_PROTOCOL).hex().upper(),
        "reply to a frame whose payload_len mismatches the byte count -> PROTOCOL error",
    ))

    if crc_enabled:
        # Wire MAJOR 4 CRC contract (#2035), the two request-side rejection
        # vectors the migration adds -- CRC-on-BUILD-SPECIFIC: the CRC-off
        # build has no CRC to be missing or corrupted, so neither applies
        # there (a CRC-less PING is simply the NORMAL shape on that wire; see
        # ping_request above).  Both produce the SAME reply as
        # ping_bad_len_reply_protocol above (RESP_ERR_PROTOCOL) -- the host
        # cannot and need not tell "no CRC" apart from "corrupted CRC" apart
        # from "bad length" on the wire; all three mean "this request was
        # never executed".
        out.append((
            "ping_request_no_crc_reply_protocol",
            request_no_crc(CMD_PING, 0).hex().upper(),
            "cmd=PING | flags=0 | len=0 -- a 3.1-shaped (no-CRC) request to a NON-GET_VERSION "
            "opcode on a CRC-on build; rejected with RESP_ERR_PROTOCOL, never executed "
            "(contrast get_version_request_no_crc above, the one opcode this DOES accept, "
            "and the CRC-off build's ping_request, which is this exact shape and IS executed)",
        ))
        _corrupt_hdr = frame_header(CMD_PING, 0, CRC_BYTES)
        _corrupt_crc = crc16_ccitt_false(_corrupt_hdr) ^ 0xFFFF  # flip every bit -> guaranteed wrong
        out.append((
            "ping_request_corrupted_crc",
            (_corrupt_hdr + _corrupt_crc.to_bytes(2, "little")).hex().upper(),
            "cmd=PING | flags=0 | len=2 | crc=deliberately wrong -- a PRESENT but corrupted CRC "
            "(link corruption, not a missing trailer); also rejected with RESP_ERR_PROTOCOL",
        ))

    return out


def emit(vectors: list[tuple[str, str, str | None]], legacy: bool = False) -> str:
    chunks = [HEADER_LEGACY if legacy else HEADER]
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
    parser.add_argument("--legacy", action="store_true",
                        help="emit the CC3501E_WIRE_CRC=OFF (wire MAJOR 3 legacy, "
                             "byte-identical-to-3.1) vector shape instead of the default "
                             "CC3501E_WIRE_CRC=ON (wire MAJOR 4) shape; changes the default "
                             "--out to protocol_vectors_no_crc.txt")
    parser.add_argument("--out", type=pathlib.Path, default=None)
    args = parser.parse_args(argv)
    if args.out is None:
        default_name = "protocol_vectors_no_crc.txt" if args.legacy else "protocol_vectors.txt"
        args.out = pathlib.Path(__file__).parent / default_name

    rendered = emit(build_vectors(crc_enabled=not args.legacy), legacy=args.legacy)
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
