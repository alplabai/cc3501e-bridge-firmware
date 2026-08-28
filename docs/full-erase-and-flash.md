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
      print('.'.join(str(b) for b in d[0x24:0x28]))"
  ```

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
`<= 255`, then build the set. `ti/regen_flashset.sh` builds a **warm** set —
correct for the fast path in the README, **not** for this one; a full set is
generated with `prog_req_content` carrying `boot_sector`, `primary_tbl`,
`primary_ti_wsoc` **and** `primary_vendor_image` all true.

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
    print('stamp', '.'.join(str(b) for b in d[0x24:0x28]))"
```

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

**Do not trust the programmer's exit status as proof.** It reports success on
writes the SBL will later refuse. Verification is step 6.

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
