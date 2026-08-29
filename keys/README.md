# Keys

## `alp_cc3501e_vendor_VALIDATION_public.pem`

The **public** half of the key every signed blob in `prebuilt/` is signed with.
It is here so that step 1 of the flashing recipe in `README.md` — the signature
check that section calls mandatory — can actually be run. Before this file was
committed the recipe named a `<VALIDATION_public.pem>` placeholder that appeared
nowhere in the repository, so the one step a consumer must not skip was the one
step they could not perform (#75 follow-up).

Verify any release against it:

```sh
openssl dgst -sha256 -verify keys/alp_cc3501e_vendor_VALIDATION_public.pem \
    -signature prebuilt/cc3501e-v0.5.0.bin.sig prebuilt/cc3501e-v0.5.0.bin
sha256sum -c <<<"$(cat prebuilt/cc3501e-v0.5.0.bin.sha256)  prebuilt/cc3501e-v0.5.0.bin"
```

All five shipped releases — 0.2.0, 0.3.0, 0.4.0, 0.4.1, 0.5.0 — verify against
this key. That is also how the key was identified when it came time to sign
0.5.0: the alp-sdk release-signing key returns `Verification Failure` on these
blobs, so it is emphatically *not* the signer here, and the two must not be
confused.

### What this key is NOT

- **Not a production root-of-trust anchor.** This is the vendor *validation*
  key, a bench-grade signing identity for development artifacts. It attests
  "this blob is the one Alp Lab built and published", not "this blob is
  authorised to boot on a provisioned production part". Do not wire it into a
  secure-boot chain.
- **Not a secret.** A public key is meant to be distributed; publishing it is
  its entire purpose. The **private** half is not in this repository and never
  will be.
- **Not the CC35 device RoT.** The part's own anti-rollback and secure-boot
  enforcement is separate and is described in `README.md` and
  `BRINGUP_STATUS.md`. Verifying a blob here says nothing about whether a given
  unit will accept its GPE version stamp — read the anti-rollback warning before
  flashing.
