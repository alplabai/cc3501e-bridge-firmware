#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Assert every prebuilt blob's artifact KIND matches what BUILD_RECIPE.md records.

A blob's kind matters as much as its hash.  `prebuilt/` has shipped two kinds
under one naming scheme -- a raw `build_ti.ps1` image and a TI
`flash-images-builder` vendor_image -- and README's flashing recipe copies the
blob straight in as `primary_vendor_image.sign.bin`, which is only correct for
the wrapped kind.  Installing a raw blob there puts a bare Cortex-M vector
table into the slot the programmer reads as a signed container: not a subtly
different build of the right thing, a different file format (#96).

`prebuilt/BUILD_RECIPE.md` records which release is which, in prose.  This
makes that record machine-checked, so a raw blob cannot again ship where a
wrapped one is expected -- and so a NEW release cannot ship without its kind
being written down at all.

The two kinds are distinguishable in the first 20 bytes, two ways that agree on
every blob shipped so far:

    wrapped -> bytes 16..19 are the container magic c2 47 0c 69
    raw     -> word 0 is a Cortex-M initial stack pointer (0x2000xxxx /
               0x2001xxxx) and bytes 16..19 hold a repeated exception-handler
               address instead

Only the magic is used as the discriminator; the stack pointer is checked as a
corroborating second opinion, and a disagreement between them is itself an
error rather than a silent pick-one.  That matters because the whole point is
to catch a file that is not the shape anyone assumed.

Lives under .github/scripts/ rather than ti/ deliberately.  The `prebuilt
freshness` gate treats every path under src/, hal/ and ti/ as
image-affecting, which is right for ti/build_ti.ps1 and the flashset
scripts -- they ARE the build recipe -- but wrong for a CI checker that
cannot touch the firmware.  Putting it here keeps that gate meaningful
instead of demanding an `inert:` attestation for a file that provably
changes nothing.

Exit 0 if every blob matches its recorded kind, 1 otherwise.
"""

from __future__ import annotations

import pathlib
import re
import sys

#: Container magic at file offset 16 in a TI flash-images-builder vendor_image.
#: Identical across v0.4.0, v0.4.1 and v0.6.0.
WRAPPED_MAGIC = bytes((0xC2, 0x47, 0x0C, 0x69))

#: A raw image starts with the Cortex-M vector table, whose first word is the
#: initial stack pointer.  Every raw blob shipped so far lands in SRAM at
#: 0x20002ff0 / 0x20013000, so the mask has to cover 0x2000xxxx AND
#: 0x2001xxxx -- a 0xFFFF0000 mask is too narrow and rejects the second.
RAW_SP_MASK = 0xFFF00000
RAW_SP_MATCH = 0x20000000

#: A wrapped image carries its GPE anti-rollback stamp as four bytes
#: (major, a, b, c) at file offset 36.  The CC35 SBL enforces monotonicity
#: against the part's last-seen version, permanently, even with every
#: *_rollback_protection_* fuse reading 0 -- so a stamp below what our units
#: have seen ships a blob that streams clean and then refuses to boot.
GPE_OFFSET = 36
GPE_LEN = 4

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
RECIPE = REPO / "prebuilt" / "BUILD_RECIPE.md"
BUILT_FROM = REPO / "prebuilt" / "BUILT_FROM"
PREBUILT = REPO / "prebuilt"

_VERSION = re.compile(r"\b\d+\.\d+\.\d+\b")
_BLOB = re.compile(r"^cc3501e-v(\d+\.\d+\.\d+)\.bin$")


def recorded_kinds(text: str) -> dict[str, str]:
    """Parse BUILD_RECIPE.md's kind table into {version: 'raw'|'wrapped'}.

    Rows look like `| 0.2.0, 0.3.0, 0.5.0 | **raw** -- ... | ... |`.  The first
    cell carries the versions, the second names the kind.
    """
    kinds: dict[str, str] = {}
    for line in text.splitlines():
        if not line.startswith("|"):
            continue
        cells = [c.strip() for c in line.strip("|").split("|")]
        if len(cells) < 2:
            continue
        versions = _VERSION.findall(cells[0])
        # Match the BOLDED kind token, not a bare substring: the raw row
        # describes itself as "unwrapped", which contains "wrapped".
        lowered = cells[1].lower()
        if "**wrapped**" in lowered:
            kind = "wrapped"
        elif "**raw**" in lowered:
            kind = "raw"
        else:
            continue
        for version in versions:
            kinds[version] = kind
    return kinds


def observed_kind(head: bytes) -> tuple[str | None, str]:
    """Return (kind, detail) for a blob's first 20 bytes.

    kind is None when the two discriminators disagree -- a shape nobody has
    seen, which is worth failing on rather than guessing about.
    """
    magic_says_wrapped = head[16:20] == WRAPPED_MAGIC
    word0 = int.from_bytes(head[0:4], "little")
    sp_says_raw = (word0 & RAW_SP_MASK) == RAW_SP_MATCH

    if magic_says_wrapped and not sp_says_raw:
        return "wrapped", f"container magic at 0x10, word0=0x{word0:08x}"
    if sp_says_raw and not magic_says_wrapped:
        return "raw", f"vector table, initial SP=0x{word0:08x}"
    return None, (
        f"indeterminate: magic@0x10={head[16:20].hex()} "
        f"word0=0x{word0:08x} -- matches both kinds or neither"
    )


def gpe_floor() -> tuple[int, ...] | None:
    """Parse `gpe-floor: 0.a.b.c` out of prebuilt/BUILT_FROM, or None."""
    if not BUILT_FROM.is_file():
        return None
    for line in BUILT_FROM.read_text(encoding="utf-8").splitlines():
        if line.startswith("gpe-floor:"):
            parts = line.split(":", 1)[1].strip().split(".")
            if len(parts) == 4 and all(p.isdigit() for p in parts):
                return tuple(int(p) for p in parts)
    return None


def newest_wrapped(blobs: list[pathlib.Path]) -> pathlib.Path | None:
    """The highest-versioned wrapped blob -- the one customers are told to flash."""
    best = None
    best_key: tuple[int, ...] = ()
    for b in blobs:
        m = _BLOB.match(b.name)
        if not m:
            continue
        head = b.read_bytes()[:20]
        if len(head) < 20 or head[16:20] != WRAPPED_MAGIC:
            continue
        key = tuple(int(x) for x in m.group(1).split("."))
        if key > best_key:
            best, best_key = b, key
    return best


def check_gpe(blob: pathlib.Path, floor: tuple[int, ...]) -> int:
    """Assert the shipped stamp has major 0 and clears the recorded floor."""
    raw = blob.read_bytes()[GPE_OFFSET:GPE_OFFSET + GPE_LEN]
    if len(raw) < GPE_LEN:
        print(f"::error::{blob.relative_to(REPO)} is too short to hold a GPE stamp")
        return 1
    stamp = tuple(raw)
    shown = ".".join(str(x) for x in stamp)
    rc = 0
    if stamp[0] != 0:
        print(f"::error::{blob.relative_to(REPO)} GPE stamp {shown} has major "
              f"{stamp[0]}; a GPE major >= 1 fails BL2 secure-boot with AUTH_ERROR")
        rc = 1
    if stamp < floor:
        print(f"::error::{blob.relative_to(REPO)} GPE stamp {shown} is BELOW the "
              f"recorded floor {'.'.join(str(x) for x in floor)} -- it would stream "
              f"clean and then refuse to boot on any unit already at that version. "
              f"Re-wrap at a higher stamp, or raise gpe-floor in prebuilt/BUILT_FROM "
              f"if the floor itself is stale")
        rc = 1
    if rc == 0:
        print(f"ok  {blob.relative_to(REPO)}  GPE {shown} >= floor "
              f"{'.'.join(str(x) for x in floor)}, major 0")
    return rc


def main() -> int:
    if not RECIPE.is_file():
        print(f"::error::{RECIPE.relative_to(REPO)} is missing -- "
              "the artifact-kind record it holds is what this gate checks")
        return 1

    kinds = recorded_kinds(RECIPE.read_text(encoding="utf-8"))
    if not kinds:
        print(f"::error::{RECIPE.relative_to(REPO)} declares no artifact kinds "
              "-- the kind table is missing or its shape changed")
        return 1

    blobs = sorted(PREBUILT.glob("cc3501e-v*.bin"))
    if not blobs:
        print("::error::no prebuilt blobs found to check")
        return 1

    rc = 0
    for blob in blobs:
        matched = _BLOB.match(blob.name)
        if not matched:
            continue
        version = matched.group(1)
        rel = blob.relative_to(REPO)

        head = blob.read_bytes()[:20]
        if len(head) < 20:
            print(f"::error::{rel} is shorter than 20 bytes -- not a firmware image")
            rc = 1
            continue

        actual, detail = observed_kind(head)
        want = kinds.get(version)

        if want is None:
            print(f"::error::{rel} is not listed in "
                  f"{RECIPE.relative_to(REPO)}'s kind table ({detail})")
            rc = 1
        elif actual is None:
            print(f"::error::{rel} {detail}")
            rc = 1
        elif actual != want:
            print(f"::error::{rel} is {actual} on disk but "
                  f"{RECIPE.relative_to(REPO)} records it as {want} ({detail})")
            rc = 1
        else:
            print(f"ok  {rel}  {actual}  ({detail})")

    floor = gpe_floor()
    if floor is None:
        print("::error::prebuilt/BUILT_FROM declares no `gpe-floor:` line -- "
              "without it nothing stops a release shipping a stamp our own "
              "units can never accept")
        return 1

    newest = newest_wrapped(blobs)
    if newest is None:
        print("::error::no wrapped blob found to check the GPE stamp of")
        return 1
    rc |= check_gpe(newest, floor)

    return rc


if __name__ == "__main__":
    sys.exit(main())
