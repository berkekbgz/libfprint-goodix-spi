// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * Goodix GDIX51C0 SPI driver for libfprint
 *
 * Copyright (C) 2026 Berke Kabagöz <berkekbgz@gmail.com>
 */

#pragma once

#include <config.h>

#ifndef HAVE_UDEV
#error "gdix51c0 requires udev"
#endif

#include <fp-device.h>
#include <fpi-device.h>

#include "../../lib/goodix/common/goodix-command.h"

G_DECLARE_FINAL_TYPE (FpiDeviceGdix51c0, fpi_device_gdix51c0, FPI, DEVICE_GDIX51C0, FpDevice);

/* Sensor readout geometry.  SENSOR_WIDTH is the 64-sample fast axis of the
 * packed stream (used by the ChicagoHU regroup in the decoder); SENSOR_HEIGHT
 * is the 80-sample slow axis. */
#define GDIX51C0_SENSOR_WIDTH   64
#define GDIX51C0_SENSOR_HEIGHT  80
#define GDIX51C0_FRAME_PIXELS   (GDIX51C0_SENSOR_WIDTH * GDIX51C0_SENSOR_HEIGHT)
#define GDIX51C0_FRAME_BYTES    7680   /* 5120 px * 1.5 bytes/px */

/* Decoded-image geometry.  gdix51c0_decode_12bpp_to_16bpp applies Goodix's
 * ChicagoHUDataRegroup, which lays the samples out as a 64-row x 80-column
 * image (stride 80). This is the transpose of the raw readout axes and is
 * retained for debug-image consumers; production matching uses Chicago's
 * canonical 64x80 raw frame. */
#define GDIX51C0_IMAGE_WIDTH    GDIX51C0_SENSOR_HEIGHT  /* 80 columns */
#define GDIX51C0_IMAGE_HEIGHT   GDIX51C0_SENSOR_WIDTH   /* 64 rows    */

/* GPIO defaults recovered for the original test platform. The proper fix is to
 * resolve the IRQ line from the ACPI _CRS, but for our laptop these
 * values are stable. Override with env vars. */
#define GDIX51C0_DEFAULT_GPIOCHIP   "/dev/gpiochip0"
#define GDIX51C0_DEFAULT_IRQ_LINE   321
#define GDIX51C0_DEFAULT_RESET_LINE 140

#define GDIX51C0_ENV_GPIOCHIP   "GDIX51C0_GPIOCHIP"
#define GDIX51C0_ENV_IRQ_LINE   "GDIX51C0_IRQ_LINE"
#define GDIX51C0_ENV_RESET_LINE "GDIX51C0_RESET_LINE"
#define GDIX51C0_ENV_WARM_SESSION "GDIX51C0_WARM_SESSION"

/* Optional host-side TLS PSK override, encoded as 32 bytes of hex. Normal
 * operation provisions and loads the root-owned persisted state below. */
#define GDIX51C0_ENV_PSK_HEX "GOODIX_TLS_PSK_HEX"

/* (Re)provision the sensor.  "random" = generate a fresh PSK, but only if none
 * is already known (env or persisted state) — so it is safe to leave set as a
 * self-bootstrap.  A 64-hex value forces that specific PSK (rewrites the sensor
 * only if it differs from the persisted one).  The only way to get a working
 * PSK on a unit with no Windows (no DPAPI blob to unseal). */
#define GDIX51C0_ENV_PROVISION_PSK "GDIX51C0_PROVISION_PSK"

/* Persisted PSK.  After a successful (re)provision the plaintext PSK is written
 * here (root-owned, mode 0600) so it survives reboots without re-provisioning.
 * gdix51c0_load_psk() reads it as a fallback after $GOODIX_TLS_PSK_HEX.  This is
 * the machine-scoped, pre-login-available store — the Linux analogue of the
 * machine-scope DPAPI blob Windows keeps.  Override the path with the env var. */
#define GDIX51C0_ENV_PSK_STATE_FILE     "GDIX51C0_PSK_STATE_FILE"
#define GDIX51C0_DEFAULT_PSK_STATE_FILE "/var/lib/fprint/gdix51c0.psk"

/* Outer SPI wrapper bytes */
#define GDIX51C0_PKT_WRITE 0xa0
#define GDIX51C0_PKT_READ  0xb0

/* Inner command bytes we care about (cmd_byte = (cmd0<<4) | (cmd1<<1)) */
#define GDIX51C0_CMD_NAV_BASE   GOODIX_COMMAND_NAV
#define GDIX51C0_CMD_FDT_DOWN   GOODIX_COMMAND_MCU_SWITCH_TO_FDT_DOWN
#define GDIX51C0_CMD_FDT_UP     GOODIX_COMMAND_MCU_SWITCH_TO_FDT_UP
#define GDIX51C0_CMD_FDT_MANUAL GOODIX_COMMAND_MCU_SWITCH_TO_FDT_MODE
#define GDIX51C0_CMD_IMAGE_T0   GOODIX_COMMAND_MCU_GET_IMAGE /* background */
#define GDIX51C0_CMD_IMAGE_T1   GOODIX_WIRE_COMMAND_ID (0x2, 0x1) /* with finger */

#define GDIX51C0_BOOT_TIMEOUT_USEC    (3   * 1000 * 1000)
#define GDIX51C0_RESPONSE_TIMEOUT_USEC (1  * 1000 * 1000)
#define GDIX51C0_FDT_MANUAL_TIMEOUT_USEC (10 * 1000 * 1000)
#define GDIX51C0_IMAGE_TIMEOUT_USEC   GDIX51C0_RESPONSE_TIMEOUT_USEC
#define GDIX51C0_FINGER_TIMEOUT_USEC  (60  * 1000 * 1000)

/* spidev xfer chunk size — Linux SPI_IOC_MESSAGE caps at one page. */
#define GDIX51C0_SPI_CHUNK 2048

/* SPI bus settings recovered from the official transport. */
#define GDIX51C0_SPI_SPEED_HZ 10000000  /* 10 MHz */
#define GDIX51C0_SPI_MODE     0         /* CPOL=0, CPHA=0 */

#define GDIX51C0_UDEV_TYPES FPI_DEVICE_UDEV_SUBTYPE_SPIDEV

/* id_table driver_data flags */
#define GDIX51C0_QUIRK_LEVEL_IRQ  (1 << 0) /* IRQ pad reports level, not edges */
#define GDIX51C0_QUIRK_EXTRA_ACKS (1 << 1) /* fw ACKs 0xd5 and single-response cmds */
#define GDIX51C0_QUIRK_SLOW_TURNAROUND (1 << 2) /* needs read->write gap, slow replies */
#define GDIX51C0_QUIRK_FACTORY_PMK (1 << 3) /* PSK is the factory PMK; 0xe0 is a no-op */
#define GDIX51C0_QUIRK_CONFIG_256  (1 << 4) /* sensor config is a 256-byte 0x90 block */
#define GDIX51C0_QUIRK_PADDED_FRAME (1 << 5) /* image rows are 88 samples, 64 real */

/* Padded image format: 80 rows of 88 raw12 samples, the first 64 of each real. */
#define GDIX51C0_PADDED_ROW_STRIDE  88
#define GDIX51C0_PADDED_FRAME_BYTES (GDIX51C0_PADDED_ROW_STRIDE * GDIX51C0_SENSOR_HEIGHT * 3 / 2)

/* Cached factory PMK (48 bytes, root 0600), read from the sensor once via F4. */
#define GDIX51C0_DEFAULT_PMK_CACHE_FILE "/var/lib/fprint/gdix51c0-pmk.bin"

static const FpIdEntry gdix51c0_id_table[] = {
  { .udev_types = GDIX51C0_UDEV_TYPES, .spi_acpi_id = "GDIX51C0" },
  /* Same ChicagoHS die behind an HC32F460 MCU on some Huawei/Honor boards
   * (e.g. BOHB-WAX9, fw GF_HC460SEC_APP_14103). GPIO lines differ per board,
   * and firmware locks the IRQ pad as level-triggered. */
  { .udev_types = GDIX51C0_UDEV_TYPES, .spi_acpi_id = "GXFP51A0",
    .driver_data = GDIX51C0_QUIRK_LEVEL_IRQ | GDIX51C0_QUIRK_EXTRA_ACKS |
                   GDIX51C0_QUIRK_SLOW_TURNAROUND | GDIX51C0_QUIRK_FACTORY_PMK |
                   GDIX51C0_QUIRK_CONFIG_256 | GDIX51C0_QUIRK_PADDED_FRAME },
  { .udev_types = 0 }
};
