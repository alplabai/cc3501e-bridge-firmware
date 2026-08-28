"""Build a COMPLETE (cold) CC3501E flash set at a legal GPE stamp.

Complete = boot_sector + primary_tbl + primary_ti_wsoc + primary_vendor_image.
That is what full_flash_erase destroys and what a WARM set cannot restore.

TBL and TI wireless FW carry TI's own versions -- their builders take a signed
container and have NO --version -- so they are reused as-is.  The boot sector
DOES take --version and embeds the programming-instruction image, so it must be
rebuilt against the new instructions; reusing an old boot sector alongside a new
vendor image would leave it pointing at a stale instruction image.

VALIDATED on an E1M-AEN801, 2026-08-28: a set built by this script programmed a
working part, survived a full_flash_erase that left the part dead
(`get_version failed (-5)`), and restored it (`protocol v5`, `fw 0x0401`).

Usage:

    TOOLBOX=<path-to-simplelink-wifi-toolbox>       SIGNING_DIR=<dir with the vendor key + sign module + cc35xx-conf.bin>     [REF_SET=<a prior COMPLETE set: fuses, temp/flash_disc_*.json, TBL,
              TI wireless FW, debug_action_request.sign.bin>]     [VENDOR_OUT=<cc3501e-bridge.out>]     py -3 ti/build_full_set.py 0.149.65.0

VERSION must exceed the part's last-seen stamp, have major == 0, and every field
<= 255.  See docs/full-erase-and-flash.md for how to establish the floor, and
why guessing it wrong is not recoverable by guessing again.
"""
import json
import os
import shutil
import subprocess
import sys

# Bench assets are NOT in this repo (signing keys, TI containers, the reference
# set).  Point these at your staged copies; every one is required.
_HERE = os.path.dirname(os.path.abspath(__file__))


def _env(name, default=None):
    v = os.environ.get(name, default)
    if not v:
        sys.exit("set %s -- see the module docstring" % name)
    return v


TB = _env("TOOLBOX")
SIGN = _env("SIGNING_DIR")
REF = _env("REF_SET", SIGN + "/gen-out/toolbox")
VOUT = _env("VENDOR_OUT", os.path.join(_HERE, "..", "build", "ti", "cc3501e-bridge.out"))
PUB = _env("PUBLIC_KEY", SIGN + "/alp_cc3501e_vendor_VALIDATION_public.pem")
MOD = _env("SIGNING_MODULE", SIGN + "/sign_win.py")
CONF = _env("CONF_BIN", SIGN + "/cc35xx-conf.bin")


def run(step, args):
    print("-- %s" % step, flush=True)
    r = subprocess.run([TB] + args, capture_output=True, text=True, timeout=600)
    if r.returncode != 0:
        print("FAILED (%d)" % r.returncode)
        print((r.stdout or "")[-1500:])
        print((r.stderr or "")[-1500:])
        sys.exit(1)


def stamp(path):
    with open(path, "rb") as f:
        d = f.read()
    return ".".join(str(b) for b in d[0x24:0x28]), len(d)


def main():
    version = sys.argv[1]
    parts = version.split(".")
    assert len(parts) == 4, "VERSION must be major.minor.patch.other"
    assert parts[0] == "0", "major MUST be 0 -- a GPE major >= 1 fails BL2 secure boot"
    assert all(0 <= int(p) <= 255 for p in parts), "every field must be <= 255"

    out = os.environ.get("DIR_OUT") or os.path.join(
        os.path.dirname(os.path.abspath(VOUT)), "fullset-" + version.replace(".", ""))
    os.makedirs(out, exist_ok=True)
    print("== complete set v%s -> %s" % (version, out), flush=True)

    run("1/5 vendor_image", [
        "flash-images-builder", "build", "vendor_image", "--version", version,
        "--public_key", PUB, "--vendor_out_file", VOUT,
        "--conf_bin_file", CONF, "--dir_out_path", out])
    run("    sign vendor_image", [
        "flash-images-builder", "sign", "vendor_image",
        "--unsign_image", out + "/vendor_image.unsign.bin",
        "--activation_type", "vendor_key", "--signing_module", MOD,
        "--public_key", PUB, "--dir_out_path", out])
    shutil.copyfile(out + "/vendor_image.sign.bin", out + "/primary_vendor_image.sign.bin")

    run("2/5 programming_instructions (matched)", [
        "flash-images-builder", "build", "programming_image", "--version", version,
        "--public_key", PUB,
        "--fuses_programming_instructions", REF + "/fuse_prog_inst_param.CC35XXE.mod.json",
        "--flash_discovery_config_otfde", REF + "/temp/flash_disc_param_otfde.json",
        "--flash_discovery_config_ext_mem", REF + "/temp/flash_disc_param_ext_mem.json",
        "--flash_discovery_config_xspi", REF + "/temp/flash_disc_param_xspi.json",
        "--dir_out_path", out, "--activation_type", "vendor_key", "--signing_module", MOD])
    run("    sign programming_instructions", [
        "flash-images-builder", "sign", "programming_image",
        "--unsign_image", out + "/programming_instructions_image.unsign.bin",
        "--activation_type", "vendor_key", "--signing_module", MOD,
        "--public_key", PUB, "--dir_out_path", out])

    # --programming_instruction_image_path is MUTUALLY EXCLUSIVE with the three
    # --flash_discovery_config_* options: the signed instruction image already
    # embeds that configuration, so passing both is rejected outright.  Feeding
    # it the instruction image is the right half of the choice here -- it is
    # what guarantees the boot sector and the instructions describe the same
    # flash layout at the same version.
    run("3/5 boot_sector (rebuilt against the NEW instructions)", [
        "flash-images-builder", "build", "boot_sector_image", "--version", version,
        "--programming_instruction_image_path", out + "/programming_instructions_image.sign.bin",
        "--dir_out_path", out])

    print("-- 4/5 reuse the TI-versioned components", flush=True)
    for n in ("primary_tbl_image.bin", "primary_ti_wireless_fw_image.bin",
              "debug_action_request.sign.bin"):
        shutil.copyfile(REF + "/" + n, out + "/" + n)

    print("-- 5/5 FULL action_request + tool_settings", flush=True)
    d = json.load(open(SIGN + "/gen-syscfg/action_params.json"))
    prc = d["programming"]["payload_param"]["payload"]["prog_req_content"]
    for k in list(prc):
        if isinstance(prc[k], bool):
            prc[k] = False
    for k in ("boot_sector", "primary_tbl", "primary_ti_wsoc", "primary_vendor_image"):
        prc[k] = True
    ap = out + "/action_params.full.json"
    json.dump(d, open(ap, "w"), indent=2)
    print("   prog_req_content: boot_sector/primary_tbl/primary_ti_wsoc/"
          "primary_vendor_image = True", flush=True)

    run("    build action_request", [
        "flash-images-builder", "build", "action_request", "--type", "programming",
        "--params_json", ap, "--dir_out_path", out])
    run("    sign action_request", [
        "flash-images-builder", "sign", "action_request",
        "--unsign_request", out + "/programming_action_request.unsign.bin",
        "--activation_type", "vendor_key", "--signing_module", MOD,
        "--public_key", PUB, "--dir_out_path", out])

    ts = {"programming_debug_and_ota_signed_components": {
        "tbl_container_programming": None,
        "programming_instructions": out + "/programming_instructions_image.sign.bin",
        "actions_req_paths": {
            "programming": out + "/programming_action_request.sign.bin",
            "debug": out + "/debug_action_request.sign.bin"},
        "primary_vendor_image": out + "/primary_vendor_image.sign.bin",
        "secondary_vendor_image": "",
        "boot_sector": out + "/boot_sector_image.bin",
        "primary_tbl": out + "/primary_tbl_image.bin",
        "secondary_tbl": "",
        "primary_ti_wsoc": out + "/primary_ti_wireless_fw_image.bin",
        "secondary_ti_wsoc": "",
        "protected_storage": ""}}
    json.dump(ts, open(out + "/tool_settings.full.json", "w"), indent=2)

    # A leftover pre-flattened image is used in PREFERENCE to the file just built.
    for f in os.listdir(out):
        if f.endswith(".flashready.bin"):
            os.remove(os.path.join(out, f))

    print("== built. components:", flush=True)
    ok = True
    for n in ("primary_vendor_image.sign.bin", "boot_sector_image.bin",
              "primary_tbl_image.bin", "primary_ti_wireless_fw_image.bin",
              "programming_instructions_image.sign.bin",
              "programming_action_request.sign.bin", "debug_action_request.sign.bin"):
        p = os.path.join(out, n)
        if os.path.exists(p):
            s, ln = stamp(p)
            print("   %-42s %9d B  @0x24=%s" % (n, ln, s))
        else:
            print("   MISSING %s" % n)
            ok = False
    print("== %s" % ("complete" if ok else "INCOMPLETE -- do not erase"), flush=True)
    sys.exit(0 if ok else 2)


main()
