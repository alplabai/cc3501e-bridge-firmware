#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Linux port of validate_gpio_bench.ps1 step [2/5] -- WARM-flash the CC3501E:
# FIB build -> sign with the Alp VALIDATION key -> program over the XDS110.
# (The absent deploy_validate.ps1 named in package_cc3501e_prod.ps1 is this, on Linux.)
#
# WARM path: no cold POR forced here; the Alif host app drives the reset
# (WIFI_EN + nRESET) when it brings the link up, skipping the fuse-gated vendor SBL.
#
# Requires the bench signing assets (NOT in the repo -- stage from the bench dir
# that ran the 2026-06-18 flash; they are a matched set):
#   PUBLIC_KEY      -- Alp VALIDATION public key (PEM); the fresh unit's vendor RoT
#   SIGNING_MODULE  -- sign.py shim with PRIVATE_KEY/PUBLIC_KEY set to the validation keypair
#   CONF_BIN        -- cc35xx-conf.bin (SoM flash/memory config; must match this SoM)
#   TOOL_SETTINGS   -- tool_settings.json (device/key-specific programmer manifest)
#
# Usage:
#   PUBLIC_KEY=... SIGNING_MODULE=... CONF_BIN=... TOOL_SETTINGS=... VERSION=0.b.c.d \
#     ./deploy_validate.sh
#
# VERSION is MANDATORY (there is no safe default) -- see the GPE-version rule below.
set -euo pipefail

TOOLBOX="${TOOLBOX:?stage + set: SimpleLink Wi-Fi Toolbox launcher dir (the simplelink-wifi-toolbox executable)}"
PUBLIC_KEY="${PUBLIC_KEY:?stage + set: Alp validation public key (PEM)}"
SIGNING_MODULE="${SIGNING_MODULE:?stage + set: sign.py shim (keys wired to the validation keypair)}"
CONF_BIN="${CONF_BIN:?stage + set: cc35xx-conf.bin}"
TOOL_SETTINGS="${TOOL_SETTINGS:?stage + set: tool_settings.json}"
XDS_SERIAL="${XDS_SERIAL:-L50015YR}"     # CC3501E XDS110 on this bench

# GPE image/flash version = the CC35 vendor-RoT gate.  It is NOT the app SemVer
# (that lives in firmware-version.txt and is reported via GET_DIAG_INFO.fw_version)
# and NOT the wire ALP_CC3501E_PROTOCOL_VERSION.
#
# The four fields a.b.c.d are BYTE-SIZED (each 0..255).  THREE hard constraints,
# all bench-proven on the E1M-AEN801 (items 1-2 on 2026-07-05, item 3 on
# 2026-07-12):
#   1. Each field must be <=255.  A human date like 1.<yy>.<mmdd>.<hhmm> is INVALID
#      (mmdd=0705/hhmm=1531 overflow a byte and corrupt the version).
#   2. The MAJOR field (a) MUST be 0.  A vendor image whose major >= 1 FAILS the
#      SES/BL2 secure-boot AUTHENTICATION (boot report @0x28000104 sets AUTH_ERROR
#      0x80) and the app core never launches -- host reads get_version=-5, the CC35
#      never services the bridge.  Proven: byte-identical firmware authenticated at
#      0.0.1.0 but AUTH_ERROR'd at 1.0.0.0 and at 104.x.y.z.  (The OLD scheme here --
#      the big-endian 4 bytes of `date +%s`, high byte ~0x68=104 in the major slot --
#      is exactly this failure and silently bricked EVERY V3 image this session.)
#   3. The stamp MUST be GREATER than anything ever flashed on that unit.  The SBL
#      enforces monotonicity against the part's LAST-SEEN version even when every
#      *_rollback_protection_* fuse reads 0 (a WARM run burns no fuses, so the
#      all-zero programming_report.txt is the trap, not permission to go backwards).
#
# VERSION is therefore MANDATORY -- this script will not guess a stamp.  It used to
# default to MAJOR=0 plus the low 3 bytes of `date +%s`; that 24-bit window WRAPS every
# 194.18 days, so the stamp walks BACKWARDS across a wrap: on 2026-08-28 it evaluates to
# 0.145.198.56 -- below the bench part's last-seen 0.149.64.0 (README.md) and below the
# 0.149.63.0 floor recorded in prebuilt/CHANGELOG.md.  A set built from that default
# streams the full ~1.09 MB, exits 0, and the SBL then refuses to boot it: dead link.
version_rule() {
  cat >&2 <<'EOF'
GPE VERSION rule -- VERSION=a.b.c.d :
  * a (major) MUST be 0.  A GPE major >= 1 FAILS the SES/BL2 secure-boot
    AUTHENTICATION (boot report @0x28000104 sets AUTH_ERROR 0x80) and the app core
    never launches -- host reads get_version=-5.
  * every field is byte-sized: a, b, c and d must each be <= 255.
  * the value MUST be GREATER than anything ever flashed on that unit.  The SBL
    enforces monotonicity against the part's LAST-SEEN version even when every
    *_rollback_protection_* fuse reads 0 -- a WARM run burns no fuses, so the
    all-zero programming_report.txt is the trap, not permission to go backwards.
    A rollback streams the full ~1.09 MB, exits 0, and then refuses to boot.
Where to find the unit's last-seen version:
  * README.md -- "STOP -- check the unit's flash history"; this bench part is at 0.149.64.0
  * prebuilt/CHANGELOG.md -- the per-artifact stamp table and the 0.149.63.0 floor
  * BRINGUP_STATUS.md -- "The #1 cause of *streams clean but dead link*"
  * the XDS110 `query` image table read off the part in front of you
Example:
  VERSION=0.149.65.0 ./deploy_validate.sh
EOF
}
if [ -z "${VERSION:-}" ]; then
  echo "VERSION is MANDATORY -- refusing to guess a GPE stamp." >&2
  version_rule
  exit 2
fi
if [[ ! "$VERSION" =~ ^0\.([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})$ ]] ||
   [ "${BASH_REMATCH[1]}" -gt 255 ] || [ "${BASH_REMATCH[2]}" -gt 255 ] ||
   [ "${BASH_REMATCH[3]}" -gt 255 ]; then
  echo "invalid VERSION='$VERSION' -- not a legal GPE stamp." >&2
  version_rule
  exit 2
fi

# Output root is derived from this script's location (mirrors build_ti.sh's
# out="$fw/build/ti"), so the script targets THIS checkout, not a fixed path.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fw="$(cd "$HERE/.." && pwd)" # firmware/cc3501e
OUT="$fw/build/ti"
VOUT="$OUT/cc3501e-bridge.out"
PKG="$OUT/bench"
[ -f "$VOUT" ] || { echo "missing $VOUT -- run: firmware/cc3501e/ti/build_ti.sh --wifi"; exit 1; }
for f in "$PUBLIC_KEY" "$SIGNING_MODULE" "$CONF_BIN" "$TOOL_SETTINGS"; do
  [ -f "$f" ] || { echo "MISSING signing asset: $f"; exit 3; }
done
mkdir -p "$PKG"

echo "== FIB build vendor_image v$VERSION =="
"$TOOLBOX" flash-images-builder build vendor_image --version "$VERSION" \
    --public_key "$PUBLIC_KEY" --vendor_out_file "$VOUT" --conf_bin_file "$CONF_BIN" --dir_out_path "$PKG"

echo "== sign vendor_image (Alp VALIDATION key -- bench/staging) =="
"$TOOLBOX" flash-images-builder sign vendor_image \
    --unsign_image "$PKG/vendor_image.unsign.bin" --activation_type vendor_key \
    --signing_module "$SIGNING_MODULE" --public_key "$PUBLIC_KEY" --dir_out_path "$PKG"
# sign gotcha (BRINGUP_STATUS): output is named after the input base -> rename to
# the name tool_settings.json references before programming.
cp -f "$PKG/vendor_image.sign.bin" "$PKG/primary_vendor_image.sign.bin"

echo "== program over XDS110 ($XDS_SERIAL) =="
progargs=(programmer -i XDS110)
[ -n "$XDS_SERIAL" ] && progargs+=(-param1 "$XDS_SERIAL")
progargs+=(programming --tool_settings "$TOOL_SETTINGS")
prog_log="$PKG/program.log"
"$TOOLBOX" "${progargs[@]}" >"$prog_log" 2>&1 || {
  echo "program returned nonzero -- retry once (intermittent -1141 SECAP reject, per BRINGUP_STATUS)"
  "$TOOLBOX" "${progargs[@]}" >"$prog_log" 2>&1
}
cat "$prog_log"

# The programmer returns 0 even when it SKIPS the vendor image: if the staged
# tool_settings' programming_instructions/action_request are stale or image-coupled
# to a different build, the vendor-image write is silently no-op'd -- it streams only
# the ~1.3 KB programming-instructions and leaves the OLD resident image (this is #712,
# which cost a full bench session on #708).  A nonzero exit is NOT sufficient.
#
# Verify the FULL vendor image actually streamed.  NB: programming_report.txt's
# primary_vendor_image_done bit is NOT a usable signal on this bench -- it reads 0 on
# EVERY report, including the known-good REF_SET that brought Wi-Fi/BLE up.  The reliable
# discriminator is the streamed byte count vs the vendor image size: full stream (~1.09 MB
# decimal = 1.04 MiB, i.e. what `ls -lh` shows -- both are the same 1094764 B, don't read
# the MiB figure as a short write) = written, ~1.3 KB = skipped.
vsize=$(stat -c%s "$PKG/primary_vendor_image.sign.bin" 2>/dev/null || echo 0)
# NB the `|| true`: under `set -euo pipefail` a grep that matches NOTHING -- exactly the
# short-stream case this check exists to catch -- would kill the script HERE, before the
# diagnostic below could ever print.
streamed=$(grep -oiE 'Writing binary size of[[:space:]]+[0-9]+' "$prog_log" | grep -oE '[0-9]+' | sort -rn | head -1 || true)
streamed=${streamed:-0}
if [ "$vsize" -gt 0 ] && [ "$streamed" -ge $(( vsize / 2 )) ]; then
  echo "== CC3501E warm-flashed + vendor image streamed ($streamed B of $vsize B). =="
else
  echo "ERROR: programmer exited 0 but did NOT stream the vendor image (streamed=$streamed B, expected ~$vsize B)." >&2
  echo "       -> stale / image-coupled programming manifest: only the instructions were written and the" >&2
  echo "       OLD resident image survives (#712).  deploy_validate.sh re-signs only the vendor_image half;" >&2
  echo "       programming_instructions + action_request are image-coupled and are NOT regenerated here." >&2
  echo "       To deliver a fresh image, use the matched-full-set pipeline instead:" >&2
  echo "         firmware/cc3501e/ti/regen_flashset.sh" >&2
  exit 5
fi
