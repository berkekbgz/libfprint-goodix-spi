<h1 align="center">
<img src="docs/assets/gdix51c0-sensor-capture-rounded-color.png" alt="False-color visualization of a real Goodix GDIX51C0 sensor capture" width="120">
<br>
<b>libfprint Goodix SPI Driver</b>
</h1>

<p align="center">
An out-of-tree libfprint project for Goodix host-processing fingerprint sensors that has official matcher!
</p>

<p align="center">
<a href="https://github.com/berkekbgz/libfprint-goodix-spi/actions/workflows/ci.yml"><img src="https://github.com/berkekbgz/libfprint-goodix-spi/actions/workflows/ci.yml/badge.svg" alt="CI"></a>
<a href="LICENSE"><img src="https://img.shields.io/badge/license-LGPL--2.1--or--later-blue.svg" alt="License: LGPL-2.1-or-later"></a>
<img src="https://img.shields.io/badge/status-pre--release-orange.svg" alt="Status: pre-release">
</p>

<br>

> Project is built with AI assistance, but everything validated byte for byte with official driver.\
> Took me too much time and too many tokens.
> Any support would be much appreciated.\
> Thank you!

<br>

## Supported devices

- Goodix GDIX51C0 <sup>* tested on Huawei Matebook 16s</sup>
- Goodix GXFP51A0 with HC32F460 MCU <sup>* tested on Huawei BOHB-WAX9, see [GXFP51A0 boards](#gxfp51a0-boards)</sup>

> [!NOTE]\
> For details, see the [support matrix](docs/SUPPORTED_DEVICES.md)
> and [current parity record](re/PARITY.md)

## Known limitations

- This is a pre-release. The driver has only been hardware-tested on the
  machines listed above, and packaging or integration details may still change.
  The driver id, storage paths, and enrolled-print format are treated as stable.
- The default GPIO lines match the MateBook 16s. Other laptops using the same
  sensor may require the `GDIX51C0_GPIOCHIP`, `GDIX51C0_IRQ_LINE`, and
  `GDIX51C0_RESET_LINE` overrides.
- The sensor must be exposed through Linux spidev; this project does not install
  a custom kernel driver.
- Integration is tested against the pinned libfprint and fprintd versions. Newer
  releases may require the integration patches to be updated.

## Security model

> [!IMPORTANT]\
> This sensor streams images to the host, and all matching runs on the host CPU.
> It is not a match-on-chip or secure-element design, and it cannot be made into
> one: the hardware has no command to store a template or match a finger.

- **A compromised host defeats it.** An attacker with root or kernel access can
  bypass authentication and read biometric data. Fingerprint unlock here defends
  against physical and opportunistic access to your machine, not against malware
  that is already running as you.
- **No liveness detection.** The sensor's raw anti-spoofing signals are never
  exposed to the host, so this driver cannot perform presentation-attack
  detection. A good fake finger may pass.
- **Templates and keys sit on disk.** Enrolled prints and the TLS PSK live under
  `/var/lib/fprint` protected only by filesystem permissions (`0600`).
- **What the native matcher does buy you** is a much lower false-accept rate than
  a generic image matcher achieves on a sensor this small — that is, the wrong
  finger is far less likely to unlock your machine.

Use fingerprint unlock as a convenience factor, not as your only line of
defense.

## Install

```bash
./install.sh
```

The installer:

1. Fetches pinned libfprint and fprintd sources into
   `${XDG_CACHE_HOME:-$HOME/.cache}/goodix-spi-driver/sources`,
2. Applies the libfprint and fprintd integration patches,
3. Builds a shadow libfprint under
   `/opt/goodix-spi-driver/libfprint`,
4. Builds the patched fprintd under `/opt/goodix-spi-driver/fprintd`,
5. Configures fprintd to use both shadow builds and prewarm GDIX51C0,
6. Restarts the daemon.

> [!WARNING]\
> The default installation patches fprintd so successful adaptive
> `templateStudy` updates are saved to disk and the sensor can initialize in
> the background. The current Chicago template can retain up to 50 captures.
> Use `./install.sh --without-fprintd` to keep the distribution fprintd; this
> disables both persistence of learning updates and background warm sessions.

<br>

The supported defaults are libfprint `v1.94.10` and fprintd `v1.94.5`. A
different libfprint checkout can be supplied directly:

```bash
./install.sh /path/to/libfprint
```

or fetched into a separate cache entry:

```bash
LIBFPRINT_REF=<tag-or-commit> ./scripts/fetch-libfprint.sh
```

Build dependencies include GLib/GIO, GUsb, GUdev, `libgpiod >= 2.0`, and
`OpenSSL >= 3.0`. The default patched fprintd build also requires Polkit's
GObject development files (`polkit-gobject-1 >= 0.91`). Verbose libfprint
diagnostics are disabled by default. Use
`GOODIX_SPI_DEBUG=1 ./install.sh` only for a debugging session.

### Developer builds

Normal builds do not contain capture dump writers, protocol tuning knobs, or
fault-injection hooks. To compile those tools for driver development:

```bash
GOODIX_SPI_DEVELOPER=1 GOODIX_SPI_DEBUG=1 ./install.sh
```

Developer diagnostics can write biometric captures and should not be left
enabled on an everyday installation. Runtime dump variables have no effect on
the default release build. CI checks both build profiles and rejects a release
library that still contains a developer hook.

### GXFP51A0 boards

Some Huawei and Honor laptops expose the same ChicagoHS sensor as `GXFP51A0`,
behind an HC32F460 MCU (firmware `GF_HC460SEC_APP_1410x`). The driver
recognises it by that ACPI id and enables the firmware workarounds it needs;
GDIX51C0 machines are unaffected. To check for it:

```bash
ls /sys/bus/spi/devices/ | grep GXFP51A0
cat /sys/class/dmi/id/product_name
```

The GPIO lines differ per board. On a board listed in
[`config/gdix51c0-boards.tsv`](config/gdix51c0-boards.tsv), `./install.sh`
picks them automatically. On any other board, read the reset and IRQ GPIOs from
the sensor's ACPI node (`_SB.PCI0.SPI1.FPNT` on BOHB-WAX9) and pass them:

```bash
GDIX51C0_IRQ_LINE=<irq> GDIX51C0_RESET_LINE=<reset> ./install.sh
```

If that works, please add a row for the board. On first start the driver reads
the sensor's factory TLS key, which takes about a minute, and caches it in
`/var/lib/fprint/gdix51c0-pmk.bin` (root only).

> [!WARNING]\
> Some laptops, such as the MateBook 13, report `GXFP51A0` with an STM32 MCU
> instead of the HC32. That variant is not supported.

## First run

The sensor also requires a 32-byte TLS PSK. If no key is supplied, the default
installation authorizes one random first-run provision and stores it as
`/var/lib/fprint/gdix51c0.psk` with mode `0600`. To use an existing key:

```bash
GOODIX_TLS_PSK_HEX=<64-hex> ./install.sh
```

> [!NOTE]\
> On a dual-boot system, an existing Windows key can be imported. If Linux and
> Windows use different keys, either OS may replace the key stored by the
> sensor. After a confirmed TLS authentication mismatch, Linux restores its
> persisted key once after a hard reset. The first fingerprint operation after
> switching operating systems may therefore take a little longer.

## Enroll and verify

Enroll a finger after installation:

```bash
fprintd-enroll -f right-index-finger
```

Then test it:

```bash
fprintd-verify
```

## Repository layout

```text
lib/goodix/common/          transport-neutral codecs and checksums
lib/goodix/chicago/         I/O-free native ChicagoHS biometric backend
drivers/gdix51c0/           GDIX51C0 transport, session, capture, and libfprint glue
patches/                    pinned libfprint and fprintd integration patches
scripts/                    fetch, build, install, and hardware-test helpers
config/                     supported-device metadata and install hooks
integration/                source-owned Meson registration fragments
tests/                      deterministic common and Chicago tests
docs/                       architecture, support, and development roadmap
re/                         source-only interoperability research and oracles
```

The [architecture](docs/ARCHITECTURE.md) explains the abstraction boundaries;
the [roadmap](docs/ROADMAP.md) covers additional sensors.

## Uninstall

```bash
./uninstall.sh
```

Use `./uninstall.sh --purge` only when the persisted PSK and sensor calibration
should also be removed.

This project is maintained out of tree today. Parts of the driver may
eventually be suitable for libfprint, but that is a possibility rather than a
promised distribution path. The SPI integration and adaptive-template
persistence would first need to be agreed with the libfprint and fprintd
maintainers.


## License and provenance

Unless otherwise noted, original project source is distributed under
LGPL-2.1-or-later; see [LICENSE](LICENSE). Adapted files retain their upstream
copyright notices, and patches for another project follow that target
project's license. [PROVENANCE.md](PROVENANCE.md) records the exact reference
repositories, revisions, licenses, and copied-versus-behavioral boundaries.

<sub>Made with ❤️ using too many tokens, by [Berkekbgz](https://berkekbgz.dev/)</sub>
