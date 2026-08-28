<!--
Copyright (c) 2026 Alp Lab AB
SPDX-License-Identifier: Apache-2.0
-->

# Full erase and flash — CC3501E companion

How to take a CC3501E from an unknown or non-responding state to a known-good
image, over XDS110/SWD with the SimpleLink Wi-Fi Toolbox.

This is the **recovery** path. If the part already answers `GET_VERSION` and you
only want a newer application image, use the warm path in
[`../README.md`](../README.md) instead — it is faster and it cannot brick the
part, which this procedure can.

> **Verified end-to-end on an E1M-AEN801, 2026-08-28.** Programmed a complete
> set on a working part (booted), ran `full_flash_erase` (part dead,
> `get_version failed (-5)` on every attempt), then programmed the same set
> again and the part came back (`protocol v5`, `fw 0x0401`). The partial-write
> behaviour in step 5 was found during that run, not by reading the tool.

---

## Read this before you erase anything

**A full erase removes the boot sector, the TBL and TI's wireless firmware, not
just the application.** After it, the part has no bootable content at all. It is
recoverable *only* if you can program a **complete** flash set back. If all you
have is a warm (vendor-image-only) set, the part stays dead — a warm set cannot
supply the three components the erase removed.

So the order below is: **prove you can talk to the part → prove your key matches
the part → prove you have a complete set → only then erase.**

Three more things that have each cost a part or a day here:

- **The GPE stamp must be monotonically ≥ anything ever flashed on that unit**,
  and the SBL enforces this **even when every `*_rollback_protection_*` fuse
  reads `0`**. A warm programming run burns no fuses, so an all-zero fuse report
  looks permissive and is not. A stamp below the part's last-seen version
  streams clean, reports success, and then refuses to boot.
- **`major` must be `0`.** A GPE major `>= 1` fails BL2 secure-boot with
  `AUTH_ERROR 0x80`; the app core never launches and the host reads
  `get_version = -5`. Byte-identical firmware authenticated at `0.0.1.0` and
  failed at `1.0.0.0`.
- **Every field is one byte, so the ceiling is `0.255.255.255`.** Stamping near
  it burns the part's remaining update space permanently. Increment by the
  smallest step that clears the floor — `0.<n>.<m>.<k+1>`, not a round jump.

---

## 0. What you need

| Item | Value on the E1M-AEN SoM |
|---|---|
| Probe | TI XDS110 (LP-XDS110 works), over SWD |
| Flash part | `PY25Q64LB` (Puya, 64 Mbit / 8 MiB) |
| Toolbox | `simplelink-wifi-toolbox` 4.2.4 |
| Signing key | the vendor key whose SPKI hash is in the part's RoT fuse |

Set `TB` to the toolbox executable and `SN` to your probe's serial for the
commands below:

```sh
TB="/c/ti/simplelink_wifi_toolbox_4.2.4/simplelink_wifi_toolbox_win_4_2_4/simplelink-wifi-toolbox.exe"
SN=<your XDS110 serial>
```

---

## 1. Ask the part what it is, before deciding anything

> **Who can run this section.** `get_fuse_data` and `full_flash_erase` both sign
> an action request at run time, so they need the vendor **private** key via
> `--signing_module`. `programming` does not — it takes only `--tool_settings`
> and consumes pre-signed artefacts.
>
> That split matters when the person holding the part is not the person holding
> the key: a module can be **restored by whoever has a complete signed set**,
> with no key involved. If you are recovering someone else's module, build and
> sign the set for them and let them run step 5 alone — do not send a signing
> key to make an erase possible until you have established the erase is actually
> required.


```sh
"$TB" programmer -i XDS110 -param1 "$SN" get_fuse_data \
    --flash_type PY25Q64LB \
    --activation_type vendor_key \
    --signing_module <path-to-your-signing-module>
```

Two things in that output decide whether the rest of this guide can work:

- **The RoT fuse** is the DER-SPKI SHA-256 of the vendor **public** key the part
  was activated with. If it does not match the key you are about to sign with,
  **stop** — no image you produce will authenticate, and erasing first would
  leave you unable to restore anything. Get the matching key, or the part is
  factory-programmed to someone else's root of trust.
- **The `*_rollback_protection_*` fuses**, which on a warm-programmed part read
  `0`. As above, that is not permission to go backwards.

If the part still boots at all, also read the running image:

```sh
"$TB" programmer -i XDS110 -param1 "$SN" query \
    --query_action_req_path <path-to-a-signed-query-action-request>
```

An **empty image table** means no image has ever *booted*, which is different
from "no image is present" — a rollback-refused image is programmed and never
registers.

---

## 2. Establish the floor you must clear

You cannot read the last-seen GPE version out of the part directly. Use the
highest of:

- the stamp in the last set you programmed (byte offset **`0x24`** of
  `primary_vendor_image.sign.bin`):

  ```sh
  python3 -c "d=open('primary_vendor_image.sign.bin','rb').read(); \
      print('%d.%d.%d.%d' % (d[0x24], d[0x25], d[0x26], d[0x28]))"
  ```

  **The four fields are not contiguous.** `0x24` major, `0x25` minor, `0x26`
  patch, `0x27` **padding**, `0x28` the fourth field. The obvious slice
  `d[0x24:0x28]` returns *major.minor.patch.padding*, so it reports the fourth
  field as `0` whatever it really is. Determined by building `0.253.7.9` and
  reading back `00 fd 07 00 09`.

  Worth avoiding for a specific reason: that misread makes two genuinely
  different stamps look identical. `0.254.0.1` and `0.254.0.7` both display as
  `0.254.0.0` under the naive slice — so a set built to clear a floor can look
  like it does not, or one that does *not* clear it can look like it does.

  Offset `0x24` is the GPE version of the **vendor image** specifically. Do not
  read it from the other components and expect a version: the same offset in
  `boot_sector_image.bin`, `primary_tbl_image.bin` and
  `primary_ti_wireless_fw_image.bin` is ordinary payload, and it reads as
  plausible-looking nonsense (`252.239.85.0`, `0.1.0.0`, `0.1.8.0` on the sets
  here) — which is worse than an obvious error, because it looks like an answer.

- any stamp recorded in your flashing logs for that unit, and
- the stamps in `prebuilt/CHANGELOG.md` if you ever flashed a published artifact.

**If a unit has been used for OTA or flash iteration, assume its floor is high.**
Bench units here sit around `0.149.x.x`. A part flashed at `0.254.0.0` has a
floor of `0.254.0.0`, and every later image must be `>= 0.254.0.0`.

---

## 3. Build a COMPLETE set at a legal stamp

A complete set has four programmable components. A warm set has only the last
one:

| Component | In a warm set? | Needed after a full erase? |
|---|---|---|
| `boot_sector` | no | **yes** |
| `primary_tbl` | no | **yes** |
| `primary_ti_wsoc` (TI wireless firmware) | no | **yes** |
| `primary_vendor_image` (our application) | yes | yes |

Pick `VERSION` above the floor from step 2, with `major = 0` and every field
`<= 255`. **`ti/regen_flashset.sh` builds a WARM set** — correct for the fast
path in the README, wrong for this one. Use `ti/build_full_set.py`, which is the
script this procedure was validated with:

```sh
TOOLBOX=<path-to-simplelink-wifi-toolbox> \
SIGNING_DIR=<dir with the vendor key, sign module and cc35xx-conf.bin> \
REF_SET=<a prior COMPLETE set> \
python3 ti/build_full_set.py 0.149.65.0
```

It prints every component with its size so you can see the set is complete
before you erase, and it deletes stale `*.flashready.bin` for you.

Two couplings it handles that are easy to get wrong by hand:

- **The boot sector is version-coupled to the programming instructions.** It
  takes `--version` *and* embeds the instruction image, so a full set cannot
  reuse an old boot sector beside a new vendor image. Note also that
  `--programming_instruction_image_path` is **mutually exclusive** with the
  three `--flash_discovery_config_*` options — the signed instruction image
  already carries that configuration, and passing both is rejected.
- **TBL and TI wireless firmware carry TI's versions, not yours.** Their
  builders take a signed container and have no `--version` at all, so they are
  reused as-is from a prior complete set. You cannot re-stamp them and should
  not try.

Whatever tooling builds it, one rule is absolute:

> **`programming_instructions` must be built at the SAME `--version` as the
> vendor image.** It is derived from `--version` plus the flash-discovery
> configuration, and the programmer's version gate compares them. Mismatch it
> and the write is rejected — or worse, partially applied.

Then remove any stale pre-flattened images, which are used **in preference** to
the files you just built:

```sh
rm -f <flashset>/*.flashready.bin
```

Verify the stamp you actually produced before going near the part:

```sh
python3 -c "d=open('<flashset>/primary_vendor_image.sign.bin','rb').read(); \
    print('stamp %d.%d.%d.%d' % (d[0x24], d[0x25], d[0x26], d[0x28]))"
```

(`0x27` is padding — see step 2. `ti/build_full_set.py` prints this correctly.)

---

## 4. Erase

> **Point of no return.** After this the part has no bootable content. Do not
> run it until step 3 produced a complete set at a legal stamp, and step 1
> confirmed the RoT fuse matches your key.

```sh
"$TB" programmer -i XDS110 -param1 "$SN" full_flash_erase \
    --flash_type PY25Q64LB \
    --activation_type vendor_key \
    --signing_module <path-to-your-signing-module>
```

`--flash_type AUTO_DETECT` also works, but naming `PY25Q64LB` explicitly is
better here: an auto-detect that guesses wrong on a part you are about to erase
is a bad trade for the two seconds it saves.

---

## 5. Program the complete set

```sh
"$TB" programmer -i XDS110 -param1 "$SN" programming \
    --tool_settings <flashset>/tool_settings.json
```

Expect roughly 1.1 MB streamed for the vendor image alone, plus the boot sector,
TBL and TI wireless firmware. An intermittent ACK timeout before an image header
that clears on retry is benign; a timeout that repeats is not.

### Count the writes — a run can silently program nothing

**A programming run can finish, report no error, and write none of the four
images.** Seen on the first restore attempt of the validation run: the log
showed exactly three writes and then ended —

```
Writing binary size of 144676 bytes    <- ti_programmer, the toolbox's own loader
Writing binary size of 240 bytes       <- programming_action_request
Writing binary size of 1340 bytes      <- programming_instructions
```

No error, no `Saved report image bin file` line. A byte-identical second
invocation wrote everything. Cold-cycling after a run like that gives you a dead
part and no reason — which is how a recoverable module gets written off as
bricked.

**A restore that actually happened writes a `content_*` block per component.**
On this SoM:

| Component | Expected content write |
|---|---|
| `boot_sector` | 1084 B |
| `primary_ti_wsoc` | 171740 B |
| `primary_vendor_image` | 1097308 B (varies with your image) |
| `primary_tbl` | 135008 B |

each preceded by a 12-byte `header_*` write, and the run ending with
`Saved report image bin file at .../programming_report.txt`. If you do not see
those, **re-run before concluding anything** — and before touching the part's
power.

**Do not trust the exit status as proof either.** It reports success on writes
the SBL will later refuse. Device-side verification is step 6.

---

## 6. Verify against the device, not the programmer

Cold power-cycle the module — a warm reset is not enough, the Puya flash needs a
true cold boot — then ask the device:

```
alp companion ver          ->  CC3501E protocol v5
alp companion diag info    ->  fw: 0x0401 (v4.1)
```

Both must answer. `GET_VERSION` returning the expected wire protocol proves the
application is running, and `fw_version` proves *which* image is running — check
both, because they answer different questions.

### If PING returns `0xFF` on MISO

`0xFF` with the slave not driving the bus means **no application is running**.
In order of likelihood:

1. **The stamp was below the part's floor.** The write streamed clean and the
   SBL refused to boot it. Re-wrap the same image at a higher legal stamp and
   reprogram — the binary is not bad.
2. **`major >= 1`.** BL2 secure-boot rejected it (`AUTH_ERROR 0x80`).
3. **Key mismatch.** The signing key does not match the RoT fuse from step 1.
4. **An incomplete set.** A vendor image programmed onto a part whose
   `primary_tbl` / TI wireless firmware are from a much older release may not
   boot. This is the case a full erase plus a complete set exists to fix.

Note that (1) and (4) look identical from the host: both give a part that
programmed cleanly and does not answer. Step 1's `query` distinguishes them —
an empty image table points at (1).

---

## 7. Record what you flashed

Note the unit, the stamp, and the app version. The GPE stamp cannot be read back
out of the part, so your log is the only record of the floor the next person has
to clear. An unrecorded flash is how a unit becomes un-updatable by anyone who
does not have your notes.

---

## See also

- [`../README.md`](../README.md) — the warm path, for a part that already boots
- [`../prebuilt/CHANGELOG.md`](../prebuilt/CHANGELOG.md) — per-release stamps and
  the anti-rollback rules
- [`../BRINGUP_STATUS.md`](../BRINGUP_STATUS.md) — current surface status,
  including what is verified and what is not
