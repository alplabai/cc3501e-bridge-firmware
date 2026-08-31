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

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
RECIPE = REPO / "prebuilt" / "BUILD_RECIPE.md"
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

    return rc


if __name__ == "__main__":
    sys.exit(main())
