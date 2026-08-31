# Reproducing a signed `prebuilt/cc3501e-vX.Y.Z.bin`

`ti/build_ti.ps1` alone does **not** produce the file that ships under
`prebuilt/`. It produces `build/ti/cc3501e-bridge.{out,hex,bin}` — a raw,
unsigned Cortex-M33 image with no header, starting at byte 0 with the
vector table (initial SP, then the reset handler, both in the
`0x2000xxxx`/`0x1400xxxx` ranges). That raw `.bin` is **not** what most
releases in this directory actually are. Confusing the two is what caused
[alplabai/cc3501e-bridge-firmware#94](https://github.com/alplabai/cc3501e-bridge-firmware/issues/94):
the raw output was rebuilt, byte-compared against the shipped `v0.6.0` blob,
and found "5664 bytes apart, diverging from byte 0" — because the shipped
blob is a different *kind* of artifact, not a different build of the same
one.

## Two artifact kinds have shipped under the same filename pattern

| Release | Kind | How to tell |
|---|---|---|
| 0.2.0, 0.3.0, 0.5.0 | **raw** — `build_ti.ps1`'s `.bin` output, unwrapped, unsigned-in-band | byte 0 is the vector table directly: a stack-pointer word in `0x2000xxxx`/`0x2001xxxx`, then a reset-handler word in `0x1400xxxx` |
| 0.4.0, 0.4.1, 0.6.0 | **wrapped** — a TI `flash-images-builder` **vendor_image**, built from the same raw `.out` and then signed with the Alp Lab VALIDATION key | byte 0 is not a vector table; there is a small (~48-byte) header, then an `0xFF`-padded gap, then the vector table starts around offset 4100-4120; a `--version` stamp (see below) is embedded as 4 bytes at file offset 36 |

Nothing before this file recorded which kind a given release is. That is
itself the defect `prebuilt/BUILT_FROM` cannot see: its attestation only
asks whether the *source* changed, never what *kind* of artifact was
produced from it.

## The wrapped-kind recipe (0.4.0, 0.4.1, 0.6.0, and every release going forward unless a raw one is deliberately re-introduced and documented as such)

Two stages. Stage 1 needs the license-gated TI toolchain; stage 2 needs the
license-gated TI Wi-Fi Toolbox **and** two Alp Lab bench-only assets that are
deliberately not in this repository.

**Stage 1 — compile + link (`ti/build_ti.ps1`).**

```
build_ti.ps1 -Ble -AlpSdkRoot <alp-sdk checkout>
```

- `ticlang` 5.1.1, SysConfig 1.28.0, SimpleLink Wi-Fi SDK 10.10.01.08 (the
  versions this file's CHANGELOG entry records for the release in question).
- `-AlpSdkRoot` must point at a checkout whose
  `include/alp/protocol/cc3501e.h` `ALP_CC3501E_PROTOCOL_VERSION` matches
  this tree's `protocol-version.txt` — a mismatch fails a `_Static_assert`
  at compile time, so a successful build already proves the protocol
  version, but record it anyway: **protocol 7** for `v0.6.0`.
- No other switch (no `-AttnPulse`, no `-OtaSelftest`/`-OtaWindowSelftest`,
  default `-Transport spi`) — the release recipe never passed them, and the
  byte-level comparison below is evidence for that, not just the CHANGELOG's
  say-so: extra code from `-AttnPulse` would shift every symbol placed after
  it in link order (`build_ti.ps1` does not pass `--gc-sections`), and no
  such shift appears anywhere in the compiled-code region.
- Output: `build/ti/cc3501e-bridge.out`.

**Stage 2 — wrap + sign (`ti/deploy_validate.sh` / `ti/regen_flashset.sh`'s
`flash-images-builder` calls).**

```
simplelink-wifi-toolbox flash-images-builder build vendor_image \
    --version <GPE stamp — see below> \
    --public_key keys/alp_cc3501e_vendor_VALIDATION_public.pem \
    --vendor_out_file build/ti/cc3501e-bridge.out \
    --conf_bin_file <SoM-specific cc35xx-conf.bin> \
    --dir_out_path <out>

simplelink-wifi-toolbox flash-images-builder sign vendor_image \
    --unsign_image <out>/vendor_image.unsign.bin \
    --activation_type vendor_key \
    --signing_module <Alp Lab VALIDATION sign.py shim> \
    --public_key keys/alp_cc3501e_vendor_VALIDATION_public.pem \
    --dir_out_path <out>
```

`<out>/vendor_image.sign.bin` is `prebuilt/cc3501e-vX.Y.Z.bin`.

- **`--version`** is the GPE anti-rollback stamp (`a.b.c.d`, `a` always `0`;
  see `README.md`/`BRINGUP_STATUS.md` for why). It is **distinct** from the
  app SemVer and the wire protocol — CHANGELOG.md records it per release
  under `GPE:`. For `v0.6.0` it is **`0.149.70.0`**.
- **`--conf_bin_file`** is the SoM/board-specific flash-and-RF config
  (`cc35xx-conf.bin`). It is a bench asset, deliberately **not** committed
  here (`ti/deploy_validate.sh`'s own header: "NOT in the repo -- stage from
  the bench dir that ran the flash").
- **`--signing_module`** wraps the Alp Lab **VALIDATION** private key. Also
  deliberately not in this repository — only its public half is
  (`keys/alp_cc3501e_vendor_VALIDATION_public.pem`). This is a bench/staging
  key, not the production HSM key `ti/package_cc3501e_prod.ps1` uses.

Reproducing the exact signed bytes therefore needs bench access this
recipe's stage 2 cannot be run without. What stage 1 + the unsigned half of
stage 2 *can* prove — and does — is that this is the right mechanism.

## Evidence (issue #94)

Stage 1, run against `prebuilt/BUILT_FROM`'s `built-from` commit
(`3d963c98107178961ff7341ee8a9ac37d947ed75`, protocol-7 `alp-sdk`),
reproduces the raw image exactly:

```
sha256  64ad6abbc06df3b8537fc944ec53f23223dab2ac0a39ef723f8a9d322842a267
size    1093732
```

Running that `.out` through stage 2's `build vendor_image` (unsigned —
`--conf_bin_file` a stock TI-shipped CC3501E default, not the SoM-specific
one; no `--signing_module` available) produces an unsigned vendor image
that matches the shipped, signed `prebuilt/cc3501e-v0.6.0.bin`
(`91e3685c786f4bcfc8d8fb488f995a1fca2ddcc71f106ba53e9e06fa83bf94b6`, 1099396
bytes) **exactly in size** and in all but 106 of its 1,099,396 bytes:

| Bytes differing | Where | Why |
|---|---|---|
| 70 | the embedded ECDSA-P256 signature TLV | expected — no signing key available; ours is FIB's zero-filled unsigned placeholder |
| 32 | the SHA-256 TLV covering the signable region | expected — changes because the signature region does |
| 3 | inside the embedded `CONF_BIN` TLV | expected — the stock toolbox default `conf.bin` used here is not the SoM-specific one the release actually shipped with |
| 1 | inside `handle_get_diag_info` (`protocol_diag.c.o`) | **not accounted for.** A single byte (`0x4c` here vs `0x4a` shipped) with identical bytes immediately either side, and no address drift anywhere else in the image — inconsistent with a different build flag or a different alp-sdk root (either would ripple far past one byte with no `--gc-sections`). Unexplained; flagged rather than papered over. Does not change the conclusion below, given the other 105 bytes are fully and independently accounted for. |

That is every differing byte in the file — nothing else is unexplained at
the file-structure level.

**Conclusion:** `prebuilt/cc3501e-v0.6.0.bin` is the stage-2 wrapped/signed
output of exactly the recipe above, built from exactly the commit
`prebuilt/BUILT_FROM` names. The original "does not reproduce" finding
compared stage 1's output to a stage-2 artifact — not a source, flag, or
toolchain mismatch.
