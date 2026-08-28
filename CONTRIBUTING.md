<!--
Copyright (c) 2026 Alp Lab AB
SPDX-License-Identifier: Apache-2.0
-->

# Contributing to the CC3501E bridge firmware

This firmware runs on a TI CC3501E that sits next to an Alif Ensemble E8 on an
E1M-AEN SoM and speaks a wire protocol to a host driver in
[`alp-sdk`](https://github.com/alplabai/alp-sdk). Two consequences shape
everything below: **a change here ships as a signed binary that gets flashed
onto real silicon**, and **the wire contract has a second half in another
repository**.

## Branch strategy

`main` is the only long-lived branch, and it is protected. Every change —
including a one-line fix, a revert of your own work, and a docs typo — starts on
its own branch and lands through a pull request.

```sh
git checkout main && git pull
git checkout -b fix/<topic>        # or feat/, docs/, chore/
# ... work, run the gates below ...
git push -u origin fix/<topic>
gh pr create --base main
```

Prefixes: `fix/`, `feat/`, `docs/`, `chore/`, `perf/`. Reference the alp-sdk
issue where one exists (`alp-sdk#1562`) — most work here is tracked there,
because the defect is usually visible from the host side first.

**Why no `dev` branch**, unlike alp-sdk: this repo has a single long-lived
artifact line and a bench that serialises anyway. An integration branch would
add a merge step without adding a place for anything to be integrated. If that
changes — several people landing overlapping radio work at once — revisit it;
until then `main` plus short-lived branches is the honest shape.

Squash-merge is the default. Branch churn, the add-then-revert dead ends, and
the "try it on the bench" commits die with the branch instead of becoming
permanent history someone has to read.

## Before you open a PR

```sh
# stub build -- the same one CI runs; the ti backend needs non-redistributable
# TI tooling, so this is the compile coverage available to you.
#
# CMAKE_TOOLCHAIN_FILE is NOT optional (CMakeLists.txt's header comment says so
# too): these sources are compiled -mcpu=cortex-m33 -mthumb unconditionally, so
# the host compiler cannot build them even with the HAL stubbed.  Pass it as an
# ABSOLUTE path.  CMake resolves a relative toolchain path against the BUILD
# directory FIRST and only then against the source directory, so whether a bare
# `toolchain/arm-none-eabi.cmake` resolves depends on where you invoke it from.
# It happens to work from this repo root; it does NOT work from a parent
# workspace with `-S <subdir>`, which is the shape CI uses and where it was
# reported as "Could not find toolchain file" (see .github/workflows/ci.yml).
# An absolute path is unambiguous everywhere -- use one.
cmake -B build/stub -S . \
    -DCMAKE_TOOLCHAIN_FILE="$PWD/toolchain/arm-none-eabi.cmake" \
    -DCC3501E_HAL_BACKEND=stub \
    -DALP_SDK_ROOT=<path-to-alp-sdk>
cmake --build build/stub

# formatting, on the files you touched (clang-format 22.x)
clang-format -i <your files> && clang-format --dry-run --Werror <your files>
```

CI runs that configure with `-DCMAKE_C_FLAGS="-Werror"`, and repeats it with
`-DCC3501E_CONTROL_TRANSPORT=sdio` so the sdio branch of `src/main.c` is
compiled too. Neither is on by default locally; add both to the command above
when you want the same answer CI will give you.

If you changed the wire protocol, `protocol-version.txt` and alp-sdk's
`ALP_CC3501E_PROTOCOL_VERSION` must move **in the same change**. CI compares
them, and `src/protocol_meta.c` carries a compile-time assert as well — a header
that is merely *stale* rather than missing will otherwise build an image that
answers `GET_VERSION` with one number and behaves like another.

## Bench evidence is part of the change

A patch that touches the radio, the SPI-slave transport, OTA, or GPIO proxying
is **not reviewable without a bench run**. CI builds the stub backend; it cannot
tell you whether the CC35 still associates or whether the bridge survives a
radio op.

Paste the actual console output, not a summary. The bar this project has learned
to hold:

- Quote the counters, both sides. Host-side and firmware-side numbers in one
  place — measuring one side alone has produced confident wrong conclusions here
  repeatedly.
- State what you did **not** verify. "Not reproduced under control" and "needs a
  second radio" are useful results; a claim of PASS that a reader cannot check is
  not.
- If a fix is unverified on silicon, say so in the PR and in the changelog
  fragment. `prebuilt/CHANGELOG.md` and `BRINGUP_STATUS.md` are read by people
  deciding whether to trust a surface, so a shipped-but-unproven claim there costs
  someone a day.

## Releasing a prebuilt

`prebuilt/` holds signed images customers flash. If you add one:

- Bump `firmware-version.txt`, and add a `prebuilt/CHANGELOG.md` entry recording
  **all three** version numbers — app SemVer, wire protocol, and the GPE stamp
  on the artifact. They are not interchangeable and conflating them has cost
  bench time repeatedly.
- Ship the `.bin`, its `.sig` and its `.sha256`. CI verifies each blob against
  its own manifest.
- Read the anti-rollback rules in `prebuilt/CHANGELOG.md` before flashing the
  artifact anywhere. The SBL enforces GPE-version monotonicity against the
  part's last-seen version **even when every rollback-protection fuse reads 0**,
  so a warm programming run's all-zero fuse report is not permission to go
  backwards.

## What not to commit

Build output (`build/`), MSYS `*.stackdump` files, and anything from the TI SDK.
CI rejects tracked `.out`, `.map`, `.o` and `.stackdump` files because each of
those has been swept in by a `git add -A` on the Windows bench at least once.

## Attribution

Commits and PR bodies are attributed to Alp Lab AB and the human author. No
AI-assistant footers, co-author trailers, or session links.
