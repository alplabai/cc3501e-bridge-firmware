# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Alp Lab AB
#
# Produce a PRODUCTION-shippable CC3501E vendor image: build the full firmware
# (Wi-Fi + BLE + bridge + OTA), wrap it as a signed GPE vendor image, and
# factory-program a FRESH unit.  This is the reproducible release recipe; the
# actual signature is produced by the HSM (the production root-of-trust), which is
# NOT on a dev/bench machine -- so -SigningModule / -PublicKey MUST point at the
# HSM tooling + the production public key.  (Bench/staging signs with the Alp
# VALIDATION key instead -- see deploy_validate.ps1; those units are NOT
# production-shippable.)
#
# PRODUCTION UNIT REQUIREMENTS (see docs/cc3501e-production.md):
#   - FRESH (never-activated) CC3501E.  factory_programming activates it, burning
#     the HSM public key as the RoT and (with -RollbackProtection) the anti-rollback
#     fuses.  Already-activated parts reject factory_programming (-1141).
#   - Activate with vendor_sbl_container_enable=0 (or ship a TI vendor SBL) so the
#     cold boot + OTA swap-boot complete -- the bench unit was activated with that
#     fuse set WITHOUT an SBL, which breaks cold boot (project-cc3501e-firmware-bringup).
#
# Usage (HSM operator, fresh unit on the XDS110):
#   ./package_cc3501e_prod.ps1 -PublicKey <hsm_pub.pem> -SigningModule <hsm_sign.py> `
#       -ToolboxExe <simplelink-wifi-toolbox.exe> -ConfBin <cc35xx-conf.bin> -Program
param(
    [Parameter(Mandatory = $true)][string]$PublicKey,      # HSM production public key (PEM)
    [Parameter(Mandatory = $true)][string]$SigningModule,  # HSM signing module (sign.py shim)
    [Parameter(Mandatory = $true)][string]$ToolboxExe,     # simplelink-wifi-toolbox.exe
    [Parameter(Mandatory = $true)][string]$ConfBin,        # cc35xx-conf.bin (memory/flash config)
    # GPE image version = vendor-RoT ANTI-ROLLBACK gate (monotonic, >= the unit's),
    # DISTINCT from the app SemVer (firmware-version.txt / GET_DIAG_INFO.fw_version).
    # It MUST be monotonic vs anything ever flashed on the part
    # (the SBL enforces this against the last-seen version even when every
    # rollback-protection fuse reads 0), and MUST have major=0 -- a GPE major >= 1
    # fails BL2 secure-boot with AUTH_ERROR 0x80 and the app core never launches.
    # Every field must be <= 255.  Same a.b.c.d scheme as deploy_validate.sh and
    # regen_flashset.sh, which now REQUIRE an explicit VERSION because there is no
    # safe default (the old epoch-derived one wraps every 194.18 days and goes
    # BACKWARDS across a wrap).  There is no floor-safe default here either: a
    # low stamp is only ever valid on a FRESH never-activated unit, which is the
    # only target this script supports, and on anything with flash history it is
    # a rollback.  README.md and prebuilt/CHANGELOG.md record the last-seen
    # stamps (the bench part is at 0.149.64.0, above a 0.149.63.0 floor).
    [Parameter(Mandatory = $true)][string]$Version,
    [string]$FlashType     = "PY25Q64LB",
    [string]$XdsSerial     = "",         # XDS110 serial for -Program (e.g. L50015YR)
    [switch]$RollbackProtection,         # burn anti-rollback fuses (recommended for production)
    [switch]$Program                     # also factory_program a FRESH unit on the XDS110
)
$ErrorActionPreference = 'Stop'
$here = $PSScriptRoot
$fw   = Split-Path $here -Parent                 # firmware/cc3501e
$out  = "$fw\build\ti"
$vout = "$out\cc3501e-bridge.out"
$pkg  = "$out\prod"
New-Item -ItemType Directory -Force $pkg | Out-Null

# 1. Build the full production firmware (Wi-Fi + BLE + bridge + OTA).
Write-Host "== build full firmware (-Ble => Wi-Fi + BLE + bridge + OTA) =="
& "$here\build_ti.ps1" -Ble
if ($LASTEXITCODE -ne 0) { throw "firmware build failed" }
if (-not (Test-Path $vout)) { throw "missing $vout" }

# 2. FIB: wrap the ELF as a GPE vendor image at $Version.
Write-Host "== FIB build vendor_image v$Version =="
& $ToolboxExe flash-images-builder build vendor_image --version $Version `
    --public_key $PublicKey --vendor_out_file $vout --conf_bin_file $ConfBin --dir_out_path $pkg
if ($LASTEXITCODE -ne 0) { throw "FIB build failed" }

# 3. Sign with the PRODUCTION HSM key.
Write-Host "== sign vendor_image (HSM production key) =="
& $ToolboxExe flash-images-builder sign vendor_image `
    --unsign_image "$pkg\vendor_image.unsign.bin" --activation_type vendor_key `
    --signing_module $SigningModule --public_key $PublicKey --dir_out_path $pkg
if ($LASTEXITCODE -ne 0) { throw "HSM sign failed" }
$signed = "$pkg\vendor_image.sign.bin"
Write-Host ("signed production image: {0} ({1} bytes)" -f $signed, (Get-Item $signed).Length)

# 4. (optional) factory-program a FRESH unit: activates to the HSM key + writes
#    the boot sector / PS / vendor image atomically (the only flow that yields a
#    cold-bootable production part).
if ($Program) {
    if ($XdsSerial -eq "") { throw "-Program needs -XdsSerial" }
    $rb = if ($RollbackProtection) { "yes" } else { "no" }
    Write-Host "== factory_program FRESH unit (XDS110 $XdsSerial, rollback=$rb) =="
    # !!! KNOWN DEFECT -- READ BEFORE RUNNING WITH -Program !!!
    # The call below does NOT program the HSM-signed image built in steps 2-3, and it
    # passes NO --version, so $Version never reaches the part.  factory_programming is
    # handed the raw ELF ($vout) plus --signing_module and re-wraps/re-signs it itself,
    # at whatever GPE version the toolbox defaults to -- so the anti-rollback stamp
    # actually burned into a production unit is NOT the one printed above, and $signed
    # is written to disk and then ignored.  The script still reports success.
    # NOT repaired here, deliberately: the repair is either `--version $Version` or
    # feeding the pre-signed image, and neither flag spelling is verified against this
    # toolbox's factory_programming CLI anywhere in this repo.  factory_programming
    # ACTIVATES a fresh part and burns fuses -- one-shot per unit, not undoable -- so a
    # guessed flag risks scrapping production silicon.  Confirm the toolbox's
    # factory_programming options on the bench, then fix this call and delete this
    # block.  Until then treat -Program as UNVALIDATED for release use, and verify the
    # programmed stamp over the XDS110 `query` image table before shipping the unit.
    & $ToolboxExe programmer -i XDS110 -param1 $XdsSerial factory_programming `
        --activation_type vendor_key --flash_type $FlashType --enable_ota `
        --signing_module $SigningModule --vendor_out_file $vout --conf_bin_file $ConfBin `
        --rollback_protection $rb
    if ($LASTEXITCODE -ne 0) { throw "factory_programming failed (fresh unit required; -1141 = already activated)" }
    Write-Host ">> unit programmed -- cold-power-cycle to validate the launch"
}
Write-Host "== done: $signed =="
