#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Assert every firmware commit since the prebuilt blob is released or attested.

`prebuilt integrity` verifies each blob against its OWN `.sha256`. That is
self-consistency, NOT currency: a prebuilt arbitrarily older than src/ passes
it green, and that is exactly how `cc3501e-v0.4.1.bin` came to sit 32 commits
behind main while CI stayed clean (#75). This job checks the property that
actually matters to a customer told by README to flash the newest blob:
whether it still corresponds to the source.

This started out comparing commit DATES: red whenever anything under src/,
hal/ or ti/ was newer than the prebuilt. Right property, wrong instrument --
it cannot tell a real firmware change from one that provably does not alter
the image. #79 was the second kind (it guarded a global the linker already
dead-stripped; the rebuilt .out was byte-identical), the gate went red, and it
STAYED red on every later PR. A permanently red gate is how a real staleness
gets waved through, which is the failure #75 was about, so the date test was
replaced by an explicit attestation in `prebuilt/BUILT_FROM`.

Nothing is auto-forgiven: every firmware commit after `built-from` must be
listed there as `inert:`, a deliberate human claim backed by a rebuild of
`raw-sha256` -- the stage-1 (unwrapped, unsigned) `build_ti.ps1` output, not
the shipped `image-sha256`. For a WRAPPED release the shipped blob's own
signature makes it unreproducible on any permitted host by construction
(#94, #92); `raw-sha256` is the half a source commit can actually affect, and
the only half `prebuilt/BUILD_RECIPE.md`'s stage 1 can rebuild. This script
cannot check that claim itself -- rebuilding needs license-gated TI ticlang,
which is not in CI -- so the ledger's evidence is the only thing standing
behind it.

Both `built-from` and every `inert:` SHA are copied from a PR branch, and this
repo SQUASH-MERGES: the moment that PR merges and the branch is deleted, the
pre-squash SHA a human typed into the ledger is reachable from nobody, even
though the release commit it became is sitting right there on main (this
happened for real -- see `prebuilt/BUILT_FROM`'s own header). Both are checked
for that BEFORE the attestation loop below, so a rotted ledger SHA reports
itself directly instead of surfacing as a confusing "unattested" mismatch
against the real, post-squash commit.

Lives here rather than inline in `.github/workflows/ci.yml` for the same
reason `check_prebuilt_kind.py` does: a heredoc inside that YAML file has
already produced a `yaml.reader.ReaderError` once. A standalone script is
parseable and testable on its own.

This job is deliberately NOT in the branch ruleset's required contexts. It
should go RED when firmware lands without a release or an attestation,
because that is true and worth seeing; it must not block every unrelated PR
until someone can sign a new blob (the signing key is not in CI).

Exit 0 if the newest prebuilt corresponds to HEAD, 1 otherwise.
"""

from __future__ import annotations

import pathlib
import re
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
LEDGER = REPO / "prebuilt" / "BUILT_FROM"
PREBUILT = REPO / "prebuilt"

#: Copied into every "this SHA does not resolve" error.  Both `built-from` and
#: `inert:` name a commit that started life on a PR branch, and this repo
#: squash-merges -- the branch SHA a human recorded is unreachable the moment
#: that PR lands and its branch is deleted.  This is the part everyone
#: re-learns the hard way (#92); say it every time rather than once.
SQUASH_TRAP = (
    "This repo SQUASH-MERGES: a SHA copied from a PR branch is rewritten the "
    "moment that PR merges and the branch is deleted, so the pre-squash SHA "
    "becomes unreachable from everyone. Repoint this line at the squash "
    "commit that actually landed on main -- same tree, same blob, a "
    "different SHA. See prebuilt/BUILT_FROM's own header for the exact "
    "precedent and how it was fixed last time."
)


def _git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=REPO, capture_output=True, text=True, check=False
    ).stdout.strip()


def commit_exists(sha: str) -> bool:
    return (
        subprocess.run(
            ["git", "cat-file", "-e", f"{sha}^{{commit}}"],
            cwd=REPO,
            capture_output=True,
            check=False,
        ).returncode
        == 0
    )


def parse_field(text: str, name: str) -> str | None:
    m = re.search(rf"^{re.escape(name)}:\s*(\S+)", text, re.MULTILINE)
    return m.group(1) if m else None


def parse_inert_shas(text: str) -> list[str]:
    return re.findall(r"^inert:\s*([0-9a-f]{40})\b", text, re.MULTILINE)


def newest_bin() -> pathlib.Path | None:
    blobs = sorted(PREBUILT.glob("cc3501e-*.bin"))
    return blobs[-1] if blobs else None


def main() -> int:
    blob = newest_bin()
    if blob is None:
        print("::error::no prebuilt/cc3501e-*.bin found")
        return 1

    if not LEDGER.is_file():
        print(
            f"::error::{LEDGER.relative_to(REPO)} is missing; it records which "
            f"commit {blob.relative_to(REPO)} was built from."
        )
        return 1

    text = LEDGER.read_text(encoding="utf-8")
    ledger_rel = LEDGER.relative_to(REPO)

    built_from = parse_field(text, "built-from")
    if not built_from:
        print(f"::error::{ledger_rel} has no 'built-from:' line.")
        return 1
    if not commit_exists(built_from):
        print(
            f"::error::{ledger_rel} built-from ({built_from}) is not a commit "
            "in this repository."
        )
        print(f"::error::{SQUASH_TRAP}")
        return 1

    # raw-sha256 is the stage-1 (unwrapped, unsigned) build_ti.ps1 output --
    # the only half of a WRAPPED release a source commit can affect, and the
    # only half reproducible on a permitted host.  Without it an `inert:`
    # claim has nothing checkable to rebuild against, so treat its absence
    # the same as a missing built-from: a hard fail, not a silent pass.
    if not parse_field(text, "raw-sha256"):
        print(
            f"::error::{ledger_rel} has no 'raw-sha256:' line. An `inert:` "
            "claim rebuilds the stage-1 (unwrapped) output and compares "
            "against THIS hash, not image-sha256 -- see "
            "prebuilt/BUILD_RECIPE.md. Record it beside image-sha256."
        )
        return 1

    # Every inert: SHA rots the same way built-from does (it is also copied
    # from a PR branch) -- check all of them before the attestation loop, so
    # a rotted line reports itself directly.
    inert_bad = 0
    for isha in parse_inert_shas(text):
        if not commit_exists(isha):
            print(
                f"::error::{ledger_rel} inert: ({isha}) is not a commit in "
                "this repository."
            )
            print(f"::error::{SQUASH_TRAP}")
            inert_bad += 1
    if inert_bad:
        return 1

    print(f"newest prebuilt : {blob.relative_to(REPO)}")
    print(f"built from      : {built_from}")

    rev_list = _git("rev-list", f"{built_from}..HEAD", "--", "src", "hal", "ti")
    shas = [s for s in rev_list.splitlines() if s]

    unattested = 0
    for sha in shas:
        if re.search(rf"^inert:\s*{re.escape(sha)}\b", text, re.MULTILINE):
            print(f"  attested inert : {_git('log', '-1', '--format=%h %s', sha)}")
        else:
            print(
                f"::error::unattested firmware commit: "
                f"{_git('log', '-1', '--format=%h %s', sha)}"
            )
            unattested += 1

    if unattested:
        print(
            f"::error::{unattested} firmware commit(s) since {built_from} are "
            "neither released nor attested."
        )
        print(
            f"::error::README tells customers to flash {blob.relative_to(REPO)}, "
            "so it must still match src/."
        )
        print(
            "::error::Either cut a signed release (bump firmware-version.txt, "
            "rebuild, .sha256, .sig, CHANGELOG, and point built-from at it), or "
            "-- ONLY if a rebuild of the RAW stage-1 output proves it is "
            f"unchanged against raw-sha256 -- add an 'inert: <full-sha>' line "
            f"to {ledger_rel} with that evidence."
        )
        return 1

    print(
        "ok  prebuilt corresponds to HEAD (every later firmware commit "
        "attested inert)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
