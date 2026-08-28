## What this changes

<!-- One paragraph. If it fixes an alp-sdk issue, link it: alp-sdk#NNNN -->

## Bench evidence

<!--
REQUIRED for any change touching the radio, the SPI-slave transport, OTA, or
GPIO proxying. CI builds the stub backend only -- it cannot tell you whether the
CC35 still associates or whether the bridge survives a radio op.

Paste the actual console output, not a summary. Include BOTH sides where the
change has two (host counters and firmware counters in one place) -- measuring
one side alone has produced confident wrong conclusions in this repo before.
-->

- [ ] Ran on a real E1M-AEN801; output pasted above
- [ ] Not applicable (docs / build / stub-only change)

## What I did NOT verify

<!--
Say it plainly. "Not reproduced under control", "needs a second radio", "built
but not flashed" are useful results. A PASS a reader cannot check is not.
-->

## Wire protocol

- [ ] Unchanged
- [ ] Changed — `protocol-version.txt` AND alp-sdk's `ALP_CC3501E_PROTOCOL_VERSION`
      are bumped in lockstep (CI compares them; a stale-but-present header
      otherwise builds an image that answers `GET_VERSION` with one number and
      behaves like another)

## Prebuilt artifact

- [ ] No new artifact
- [ ] New artifact — `firmware-version.txt` bumped, `prebuilt/CHANGELOG.md`
      records all three version numbers (app SemVer, wire protocol, GPE stamp),
      and `.bin` + `.sig` + `.sha256` are all present

## Checks

- [ ] `cmake -DCC3501E_HAL_BACKEND=stub -DALP_SDK_ROOT=<alp-sdk>` builds
- [ ] `clang-format --dry-run --Werror` silent on changed C/H
- [ ] No build output, `*.stackdump`, or TI SDK content added
