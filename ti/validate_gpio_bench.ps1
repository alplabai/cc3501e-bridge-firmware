# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Alp Lab AB
#
# WARM-BOOT bench validation for the CC3501E GPIO proxy (synchronous
# gpio write / read + camera-enable).  This is the turnkey, human-run
# bench harness for the GPIO-proxy path: it builds + WARM-flashes the
# CC3501E bridge firmware, builds + flashes the Alif host example
# (examples/aen/aen-cc3501e-gpio), then captures the host serial console
# and asserts the GPIO_TEST: pass/fail contract.
#
# *** WARM-BOOT PATH ***  This harness validates the firmware over the
# WARM (debug-attached / host-driven-reset) boot path -- the path that
# SKIPS the cold Chain-of-Trust.  The bench unit was activated with
# vendor_sbl_container_enable=1 but no TI vendor SBL was ever programmed,
# so a COLD power-on POR never launches the image (the missing SBL breaks
# the chain).  Cold boot is therefore gated by that fuse and is OUT OF
# SCOPE here -- see docs/cc3501e-production.md ("Unit activation -- must be
# cold-bootable") and the firmware-bringup notes.  The GPIO proxy itself
# is identical on warm vs cold; warm is sufficient to validate the
# wire contract + the firmware GPIO/camera bodies on the current bench
# unit.
#
# *** SCOPE: SYNCHRONOUS write/read/cam ONLY ***  The current E1M-AEN rev
# wires NO host-IRQ line, so the async GPIO interrupt path
# (CMD_GPIO_SET_INTERRUPT -> EVT_GPIO_INTERRUPT, async push to the host)
# CANNOT be delivered to the host and is an r2 (next-board-rev) limitation.
# The host example still ARMS the IRQ (gpio_irq_arm step) to prove the
# config command round-trips, but no edge is asserted/awaited; this harness
# validates only the synchronous GPIO configure/write/read + camera-enable.
#
# Pad map under test (metadata/e1m_modules/aen/inter-chip.tsv):
#   * proxied GPIO  = E1M IO13 -> CC3501E GPIO13 (raw pad index 13) -- the
#     safe-to-toggle write->read loopback pad.
#   * camera enable = CAM_EN_LDO0 -> CC3501E GPIO0 (cam id 0).
#
# Usage (defaults match the 2026-06-16 bench install + the proven warm
# program recipe; override any path -- nothing is hardcoded to a user dir):
#
#   ./validate_gpio_bench.ps1 `
#       -ToolboxExe  <simplelink-wifi-toolbox.exe> `
#       -PublicKey   <validation-pubkey.pem> `
#       -SigningModule <sign.py> `
#       -ConfBin     <cc35xx-conf.bin> `
#       -ToolSettings <tool_settings.json> `
#       -CcXdsSerial <XDS110_SN> `
#       -HostSerialPort COM7
#
# Exit code: 0 iff the host prints "GPIO_TEST: SUMMARY pass=N fail=0"
# (all synchronous steps passed); non-zero otherwise.

param(
    # --- CC3501E firmware build (build_ti.ps1 passthrough; relative defaults) ---
    [string]$SdkDir       = "$env:USERPROFILE\Desktop\ti_simplelink_sdk\simplelink_wifi_sdk_10_10_01_08",
    [string]$TiclangRoot  = "C:\ti\ti-cgt-armllvm-5.1.1.LTS\ti-cgt-armllvm_5.1.1.LTS",
    [string]$SysconfigCli = "C:\ti\sysconfig-1.28.0\sysconfig_cli.bat",

    # --- WARM flash: FIB + sign + program (toolbox; Alp VALIDATION key for the bench) ---
    [Parameter(Mandatory = $true)][string]$ToolboxExe,     # simplelink-wifi-toolbox.exe
    [Parameter(Mandatory = $true)][string]$PublicKey,      # Alp VALIDATION public key (PEM) -- NOT the HSM production key
    [Parameter(Mandatory = $true)][string]$SigningModule,  # validation signing module (sign.py shim)
    [Parameter(Mandatory = $true)][string]$ConfBin,        # cc35xx-conf.bin (memory/flash config)
    [Parameter(Mandatory = $true)][string]$ToolSettings,   # tool_settings.json (references primary_vendor_image.sign.bin)
    # GPE image version = vendor-RoT ANTI-ROLLBACK gate (monotonic, >= the unit's),
    # DISTINCT from the app SemVer (firmware-version.txt / GET_DIAG_INFO.fw_version).
    # Its major MUST be 0 (major >= 1 fails BL2 secure-boot with
    # AUTH_ERROR 0x80), every field <= 255, and monotonic vs the part's last-seen
    # version even when the rollback fuses read 0.  Pass -Version explicitly on a
    # part with flash history.  Same a.b.c.d scheme as deploy_validate.sh and
    # regen_flashset.sh, which now REQUIRE an explicit VERSION because there is no
    # safe default; this bench part's last-seen stamp is 0.149.64.0 (README.md,
    # "STOP -- check the unit's flash history").
    [Parameter(Mandatory = $true)][string]$Version,
    [string]$CcXdsSerial   = "",         # XDS110 serial on the CC3501E (toolbox -param1); blank = single-probe auto

    # --- Alif host example build + flash ---
    [string]$BoardTarget   = "alp_e1m_aen801_m55_he/ae822fa0e5597ls0/rtss_he",
    [switch]$SkipHostFlash, # build the host app but don't flash it (e.g. it is already on the board)

    # --- Host serial capture (the carrier console the example prints GPIO_TEST: on) ---
    [Parameter(Mandatory = $true)][string]$HostSerialPort, # e.g. COM7
    [int]$HostBaud         = 115200,
    [int]$CaptureSeconds   = 45      # how long to listen for the SUMMARY line
)

$ErrorActionPreference = 'Stop'
$here    = $PSScriptRoot
$fw      = Split-Path $here -Parent                 # firmware/cc3501e
$repo    = (Resolve-Path "$fw\..\..").Path          # repo root
$out     = "$fw\build\ti"
$vout    = "$out\cc3501e-bridge.out"
$example = "$repo\examples\aen\aen-cc3501e-gpio"    # sibling task creates this app

Write-Host "================================================================"
Write-Host " CC3501E GPIO-proxy WARM-boot bench validation"
Write-Host "   firmware : $vout"
Write-Host "   host app : $example"
Write-Host "   contract : GPIO_TEST: SUMMARY pass=N fail=0  -> exit 0"
Write-Host "================================================================"

# ---------------------------------------------------------------------------
# 1. Build the CC3501E bridge firmware.
#
# The GPIO proxy + camera enables live in the DEFAULT (radio-free) bridge
# build (protocol.c 0x50..0x53 / 0x60..0x61) -- no Wi-Fi/BLE link is needed
# to exercise them, and the smaller image flashes faster on the bench.  Build
# the full -Ble image only if you also want the radio up in the same session.
# ---------------------------------------------------------------------------
Write-Host "`n== [1/5] build CC3501E bridge firmware (default bridge build) =="
& "$here\build_ti.ps1" -SdkDir $SdkDir -TiclangRoot $TiclangRoot -SysconfigCli $SysconfigCli
if ($LASTEXITCODE -ne 0) { throw "CC3501E firmware build failed" }
if (-not (Test-Path $vout)) { throw "missing $vout" }

# ---------------------------------------------------------------------------
# 2. WARM-flash the CC3501E (the proven validation-key program recipe).
#
# This reuses the bench warm recipe documented in BRINGUP_STATUS.md "Bench
# validate recipe": FIB build -> sign with the Alp VALIDATION key -> program
# over the XDS110.  WARM = no cold POR is forced here; the Alif host app drives
# the reset (WIFI_EN + nRESET, host-driven) when it brings the link up, which
# is the warm/debug path that skips the (fuse-gated) vendor SBL.
#
# GOTCHA (BRINGUP_STATUS.md): `sign` names its output after the input base
# name, so copy vendor_image.sign.bin -> primary_vendor_image.sign.bin (the
# name tool_settings.json references) before programming.
# ---------------------------------------------------------------------------
Write-Host "`n== [2/5] WARM-flash CC3501E (FIB -> sign[validation key] -> program) =="
$pkg = "$out\bench"
New-Item -ItemType Directory -Force $pkg | Out-Null

Write-Host "   FIB build vendor_image v$Version"
& $ToolboxExe flash-images-builder build vendor_image --version $Version `
    --public_key $PublicKey --vendor_out_file $vout --conf_bin_file $ConfBin --dir_out_path $pkg
if ($LASTEXITCODE -ne 0) { throw "FIB build failed" }

Write-Host "   sign vendor_image (Alp VALIDATION key -- bench/staging only)"
& $ToolboxExe flash-images-builder sign vendor_image `
    --unsign_image "$pkg\vendor_image.unsign.bin" --activation_type vendor_key `
    --signing_module $SigningModule --public_key $PublicKey --dir_out_path $pkg
if ($LASTEXITCODE -ne 0) { throw "validation sign failed" }
# Rename to the name tool_settings.json expects (the sign gotcha).
Copy-Item "$pkg\vendor_image.sign.bin" "$pkg\primary_vendor_image.sign.bin" -Force

$xdsLabel = if ([string]::IsNullOrEmpty($CcXdsSerial)) { "(auto)" } else { "($CcXdsSerial)" }
Write-Host "   program over XDS110 $xdsLabel"
$progArgs = @('programmer', '-i', 'XDS110')
if (-not [string]::IsNullOrEmpty($CcXdsSerial)) { $progArgs += @('-param1', $CcXdsSerial) }
$progArgs += @('programming', '--tool_settings', $ToolSettings)
$progLog = "$pkg\program.log"

# Run the programmer with its output teed to $progLog -- the streamed-byte check below
# needs the log, and the exit code alone is not a sufficient success signal (see #712
# under the check).  $ErrorActionPreference drops to 'Continue' across the call so that
# native stderr chatter merged in by 2>&1 cannot raise a terminating NativeCommandError
# and swallow the log; the exit code is returned and checked explicitly instead.
function Invoke-CcProgrammer {
    param([string]$Exe, [string[]]$ProgArgs, [string]$LogPath)
    $eap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $out = & $Exe @ProgArgs 2>&1 | ForEach-Object { "$_" }
        $rc  = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $eap
    }
    ($out -join [Environment]::NewLine) | Set-Content -Path $LogPath
    $out | ForEach-Object { Write-Host "   $_" }
    return $rc
}

$progRc = Invoke-CcProgrammer -Exe $ToolboxExe -ProgArgs $progArgs -LogPath $progLog
# -1141 = SECAP reject (intermittent); one retry per BRINGUP_STATUS.md.
if ($progRc -ne 0) {
    Write-Host "   program returned $progRc -- retrying once (intermittent -1141 SECAP reject)"
    $progRc = Invoke-CcProgrammer -Exe $ToolboxExe -ProgArgs $progArgs -LogPath $progLog
    if ($progRc -ne 0) { throw "CC3501E warm program failed" }
}

# The programmer returns 0 even when it SKIPS the vendor image: if the staged
# tool_settings' programming_instructions / action_request are stale or image-coupled to
# a different build, the vendor-image write is silently no-op'd -- it streams only the
# ~1.3 KB programming-instructions and leaves the OLD resident image (this is #712,
# which cost a full bench session on #708).  A zero exit code is NOT sufficient.
#
# Verify the FULL vendor image actually streamed.  NB: programming_report.txt's
# primary_vendor_image_done bit is NOT a usable signal on this bench -- it reads 0 on
# EVERY report, including the known-good set that brought Wi-Fi/BLE up.  The reliable
# discriminator is the streamed byte count vs the vendor image size: full stream
# (~1.09 MB decimal = 1.04 MiB, i.e. what a directory listing shows -- both are the same
# 1094764 B, don't read the MiB figure as a short write) = written, ~1.3 KB = skipped.
# This mirrors the check in deploy_validate.sh (its Linux twin).
$vsize    = (Get-Item "$pkg\primary_vendor_image.sign.bin").Length
$streamed = [int64]0
foreach ($m in (Select-String -Path $progLog -Pattern 'Writing binary size of\s+(\d+)' -AllMatches).Matches) {
    $n = [int64]$m.Groups[1].Value
    if ($n -gt $streamed) { $streamed = $n }
}
if ($vsize -gt 0 -and $streamed -ge ($vsize / 2)) {
    Write-Host "   CC3501E warm-flashed + vendor image streamed ($streamed B of $vsize B)."
} else {
    Write-Host "ERROR: programmer exited 0 but did NOT stream the vendor image (streamed=$streamed B, expected ~$vsize B)."
    Write-Host "       -> stale / image-coupled programming manifest: only the ~1.3 KB instructions were"
    Write-Host "       written and the OLD resident image survives (#712).  This harness re-signs only the"
    Write-Host "       vendor_image half; programming_instructions + action_request are image-coupled and"
    Write-Host "       are NOT regenerated here.  To deliver a fresh image, use the matched-full-set"
    Write-Host "       pipeline instead:"
    Write-Host "         firmware/cc3501e/ti/regen_flashset.sh"
    throw "CC3501E vendor image was NOT streamed (streamed=$streamed B of $vsize B) -- see #712"
}

# ---------------------------------------------------------------------------
# 3. Build the Alif host example (examples/aen/aen-cc3501e-gpio).
#
# Mirrors the aen-cc3501e-bringup recipe: the FULL qualified board target so
# the per-board overlay (boards/<target>.overlay) auto-applies.  west must be
# on PATH (run from the activated Zephyr/west env -- see the example README).
# ---------------------------------------------------------------------------
Write-Host "`n== [3/5] build Alif host example (west, $BoardTarget) =="
if (-not (Test-Path $example)) {
    throw "host example not found: $example (the aen-cc3501e-gpio app must exist first)"
}
& west build -p always -b $BoardTarget $example
if ($LASTEXITCODE -ne 0) { throw "host example build failed" }

# ---------------------------------------------------------------------------
# 4. Flash the Alif host example (over the Alif J-Link).
#
# Flashing the host app is what WARM-resets and re-powers the CC3501E (the
# host drives WIFI_EN + nRESET), so this is the warm boot of the coprocessor.
# ---------------------------------------------------------------------------
if ($SkipHostFlash) {
    Write-Host "`n== [4/5] skip host flash (-SkipHostFlash) -- assuming the app is already on the board =="
} else {
    Write-Host "`n== [4/5] flash Alif host example (Alif J-Link) =="
    & west flash
    if ($LASTEXITCODE -ne 0) { throw "host example flash failed" }
}

# ---------------------------------------------------------------------------
# 5. Capture the host serial console + assert the GPIO_TEST: contract.
#
# The example emits, in order, one line per step:
#   GPIO_TEST: <step> PASS|FAIL
# for: gpio_config_out, gpio_write_high, gpio_write_low, gpio_config_in,
#      gpio_read, cam_enable, cam_disable, gpio_irq_arm
# then a summary:
#   GPIO_TEST: SUMMARY pass=N fail=M
# PASS criteria: fail=0 (gpio_irq_arm only proves the config command
# round-trips; no async edge is delivered on this rev -- r2 limitation).
# ---------------------------------------------------------------------------
Write-Host "`n== [5/5] capture $HostSerialPort @ $HostBaud for up to ${CaptureSeconds}s =="
$port = New-Object System.IO.Ports.SerialPort($HostSerialPort, $HostBaud, 'None', 8, 'One')
$port.ReadTimeout = 1000
$port.NewLine     = "`n"
$port.Open()

$deadline   = (Get-Date).AddSeconds($CaptureSeconds)
$summary    = $null
$sawAnyStep = $false
try {
    while ((Get-Date) -lt $deadline) {
        try { $line = $port.ReadLine() } catch [TimeoutException] { continue }
        $line = $line.TrimEnd("`r", "`n")
        if ($line -match 'GPIO_TEST:\s*(\S+)\s+(PASS|FAIL)$') {
            $sawAnyStep = $true
            Write-Host "   $line"
        } elseif ($line -match 'GPIO_TEST:\s*SUMMARY\s+pass=(\d+)\s+fail=(\d+)') {
            $summary = [pscustomobject]@{ Pass = [int]$Matches[1]; Fail = [int]$Matches[2]; Line = $line }
            Write-Host "   $line"
            break
        }
    }
} finally {
    $port.Close()
}

Write-Host "`n================================================================"
if ($null -eq $summary) {
    if (-not $sawAnyStep) {
        Write-Host " RESULT: NO GPIO_TEST output seen in ${CaptureSeconds}s."
        Write-Host "   Check: host app flashed?  CC3501E warm-flashed + powered (WIFI_EN)?"
        Write-Host "   Right COM port/baud (carrier console)?  Link up (see witness over J-Link)?"
    } else {
        Write-Host " RESULT: per-step lines seen but no SUMMARY line within ${CaptureSeconds}s."
        Write-Host "   Increase -CaptureSeconds, or inspect the console for an early fault."
    }
    Write-Host "================================================================"
    exit 2
}
if ($summary.Fail -eq 0) {
    Write-Host (" RESULT: PASS  ({0})" -f $summary.Line)
    Write-Host "================================================================"
    exit 0
} else {
    Write-Host (" RESULT: FAIL  ({0})" -f $summary.Line)
    Write-Host "================================================================"
    exit 1
}
