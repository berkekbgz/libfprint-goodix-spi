// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * Goodix GDIX51C0 SPI driver for libfprint
 *
 * Copyright (C) 2026 Berke Kabagöz <berkekbgz@gmail.com>
 */

#define FP_COMPONENT "gdix51c0"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <gpiod.h>
#include <linux/spi/spidev.h>

#include "drivers_api.h"
#include "fpi-spi-transfer.h"

#include "../../lib/goodix/common/goodix-image.h"
#include "../../lib/goodix/common/goodix-mcu-config.h"
#include "../../lib/goodix/common/goodix-sensor-profile.h"
#include "../../lib/goodix/common/goodix-wire.h"
#include "gdix51c0.h"
#include "../../lib/goodix/chicago/goodix-chicago-calibration.h"
#include "../../lib/goodix/chicago/goodix-chicago-enrollment.h"
#include "../../lib/goodix/chicago/goodix-chicago-preprocess.h"
#include "../../lib/goodix/chicago/goodix-chicago-runtime.h"
#include "../../lib/goodix/chicago/goodix-chicago-template.h"
#include "gdix51c0-listener.h"
#include "gdix51c0-proto.h"
#include "gdix51c0-tls.h"

#include <glib/gstdio.h>
#include <openssl/rand.h>

struct _FpiDeviceGdix51c0
{
  FpDevice                   parent;

  /* hardware handles */
  int                        spi_fd;
  struct gpiod_chip         *gpio_chip;
  struct gpiod_line_request *irq_req;
  unsigned int               irq_offset;
  unsigned int               reset_offset;
  struct gpiod_edge_event_buffer *irq_events;

  /* Async SPI listener — owns IRQ + SPI reads while a capture session is
   * active.  NULL outside of an activated session; sync helpers (boot probe,
   * init, TLS handshake) run with this NULL and use irq_req/spi_fd directly.
   * After tls_start succeeds, session_activate_once spawns the listener; it
   * is joined in session_deactivate before TLS teardown / reset pulse. */
  Gdix51c0Listener          *listener;

  /* TLS state (populated for each enroll/verify/identify session) */
  Gdix51c0Tls                   tls;
  gboolean                   tls_ready;

  gboolean                   skip_next_identify;
  gboolean                   session_desynced;
  gboolean                   psk_reprovision_pending;
  gboolean                   psk_reprovision_attempted_this_open;
  gboolean                   psk_reprovision_completed_this_open;
  GMainContext              *action_context;
  GCancellable              *action_cancellable;

  guint8                     fdt_down_regs[12];
  guint16                    fdt_down_sample[6];
  gboolean                   fdt_down_sample_valid;
  gboolean                   require_lift_gap;

  /* Live-measured no-finger FDT baseline (per zone), refreshed from every
   * zero-touch FDT-down response.  Windows (gf_get_fdtbase -> gfFDTDownbase)
   * arms the finger-down threshold at exactly baseline/2, which keeps the trip
   * margin constant across DAC changes; a static threshold table breaks when a
   * DAC change shifts the baseline (samples fall below a fixed trip point ->
   * every zone reads touched -> touchflag stuck 0x3f). See re/PARITY.md. */
  guint16                    fdt_baseline[6];
  gboolean                   fdt_baseline_valid;

  /* Effective main DAC applied this session (OTP self-cal / override / legacy).
   * Used post-TLS to decide whether to run the high-bias readout priming. */
  guint16                    active_dac_main;

  /* Resolved from the live cmd-0x82 register-zero response. This replaces the
   * former implicit assumption that every bound ACPI device is ChicagoHS. */
  const GoodixSensorProfile *sensor_profile;

  /* The EngineAdapter calibration file starts with this sensor-specific
   * 16-byte identifier (the leading OTP bytes).  Retain it locally too: a
   * raw ImageBase from another GDIX51C0 unit is not a valid preprocessing
   * reference for this one. */
  guint8                     sensor_id[GOODIX_CHICAGO_SENSOR_ID_LEN];
  gboolean                   sensor_id_valid;
  GBytes                    *chicago_calibration;
  GoodixChicagoPreprocessor *chicago_preprocessor;

  /* Raw no-finger ImageBase captured with cmd 0x20 and passed unmodified to
   * Chicago preprocessing. */
  guint16                    t0_baseline[GDIX51C0_FRAME_PIXELS];
  gboolean                   t0_baseline_valid;
  /* The official driver refreshes goodix.dat once when a cold driver instance
   * initializes, but retains it across ordinary capture/TLS reuse.  A
   * libfprint open/close cycle is the equivalent hardware-handle boundary. */
  gboolean                   image_base_refreshed_this_open;
};

G_DEFINE_TYPE (FpiDeviceGdix51c0, fpi_device_gdix51c0, FP_TYPE_DEVICE);

#define GDIX51C0_TOUCH_IGNORE 0xff
/* EngineAdapterIdentifyFeatureSet uses one initial capture plus up to two
 * RetryCaptureIMG passes (three attempts total) for the ordinary path. */
#define GDIX51C0_VERIFY_CAPTURE_ATTEMPTS 3

/* Persistent raw ImageBase file.  See WBDI-new.log:715-855 for the Windows
 * driver's gf_update_all_base flow: baseline is captured after a confirmed
 * finger-off check, then reused for subsequent capture.  The file is bound
 * to the 16-byte OTP sensor id, matching the binding of goodix_calib.dat.
 *
 * We persist to /var/lib/fprint (fprintd's storage dir, root-owned) and
 * load only after this unit's OTP has been read.  First capture happens
 * opportunistically the first time wait_for_lift confirms touchflag=0. */
#define GDIX51C0_BASELINE_PATH      "/var/lib/fprint/gdix51c0-image-base.bin"
#define GDIX51C0_BASELINE_MAGIC     0x30584447  /* "GDX0" little-endian */
#define GDIX51C0_BASELINE_VERSION   0x02
#define GDIX51C0_ENV_CHICAGO_CALIBRATION_FILE "GDIX51C0_CHICAGO_CALIBRATION_FILE"
#define GDIX51C0_DEFAULT_CHICAGO_CALIBRATION_FILE \
  "/var/lib/fprint/gdix51c0-calibration.bin"
#define GDIX51C0_FDT_UP_THRESHOLD_OFFSET 29
/* Minimum time to wait for an armed FDT-up event to fire so we can drain it.
 * The device typically fires ~70 ms after arming; if we abandon the read
 * earlier the response stays parked on the SPI bus and the next FDT-down arm
 * fails on a stuck-high IRQ. */
#define GDIX51C0_FDT_UP_DRAIN_USEC      (300 * 1000)
#define GDIX51C0_ENROLL_LIFT_TIMEOUT_USEC (2 * 1000 * 1000)
#define GDIX51C0_LIFT_CLEANUP_TIMEOUT_USEC (5 * 1000 * 1000)
#define GDIX51C0_CANCEL_POLL_USEC       (500 * 1000)
#define GDIX51C0_NAV_RESPONSE_LEN 2413
#define GDIX51C0_NO_MATCH_HOLD_MSEC     150
#ifdef GOODIX_SPI_DEVELOPER
#define GDIX51C0_FAULT_T0_DATA_EXHAUST_ONCE_ENV \
  "GDIX51C0_FAULT_T0_DATA_EXHAUST_ONCE"
#define GDIX51C0_FAULT_PSK_MISMATCH_ONCE_ENV \
  "GDIX51C0_FAULT_PSK_MISMATCH_ONCE"

/* The cold T0 transaction runs after listener startup but before ordinary
 * capture.  Keep this acceptance hook process-wide: activation 1 consumes both
 * image records, while activation 2 in the same daemon must run unmodified. */
static gint fault_t0_data_exhaust_used;
static gint fault_psk_mismatch_tls_starts;

static gboolean
gdix51c0_arm_t0_data_exhaust_once (guint8 cmd)
{
  const char *enabled = g_getenv (GDIX51C0_FAULT_T0_DATA_EXHAUST_ONCE_ENV);

  if (cmd != GDIX51C0_CMD_IMAGE_T0 || !enabled || !*enabled ||
      g_strcmp0 (enabled, "0") == 0)
    return FALSE;

  return g_atomic_int_compare_and_exchange (&fault_t0_data_exhaust_used, 0, 1);
}

static gboolean
gdix51c0_arm_psk_mismatch_once (void)
{
  const char *enabled = g_getenv (GDIX51C0_FAULT_PSK_MISMATCH_ONCE_ENV);
  gint64 requested_start;
  gint this_start;

  if (!enabled || !*enabled || g_strcmp0 (enabled, "0") == 0)
    return FALSE;

  /* The value selects which process-wide TLS start receives the one-shot
   * wrong in-memory key. "1" preserves the standalone test; "2" lets the
   * compound test consume a clean D4 activation failure first. */
  requested_start = g_ascii_strtoll (enabled, NULL, 0);
  if (requested_start < 1)
    requested_start = 1;
  requested_start = MIN (requested_start, G_MAXINT);

  this_start = g_atomic_int_add (&fault_psk_mismatch_tls_starts, 1) + 1;
  return this_start == (gint) requested_start;
}
#endif

#ifdef GOODIX_SPI_DEVELOPER
#define GDIX51C0_DEV_ENV(name) g_getenv (name)
#define GDIX51C0_DEV_ENV_INT(name, fallback) gdix51c0_env_int ((name), (fallback))
#else
#define GDIX51C0_DEV_ENV(name) NULL
#define GDIX51C0_DEV_ENV_INT(name, fallback) (fallback)
#endif

typedef enum {
  GDIX51C0_ACTION_ENROLL,
  GDIX51C0_ACTION_VERIFY,
  GDIX51C0_ACTION_IDENTIFY,
} Gdix51c0ActionKind;

typedef struct {
  FpDevice            *dev;
  Gdix51c0ActionKind   kind;
  GMainContext        *context;
  GCancellable        *cancellable;
  FpPrint             *enroll_print;
  FpPrint             *verify_print;
  GPtrArray           *identify_gallery;
} Gdix51c0ActionThread;

typedef struct {
  FpDevice     *dev;
  GMainContext *context;
  GCancellable *cancellable;
} Gdix51c0OpenThread;

typedef struct {
  FpDevice *dev;
  GError   *error;
} Gdix51c0OpenComplete;

static gboolean gdix51c0_action_check_cancelled (FpiDeviceGdix51c0 *self,
                                                 GError           **error);
static gboolean gdix51c0_session_activate (FpiDeviceGdix51c0 *self,
                                           GError           **error);
static void     gdix51c0_session_deactivate (FpiDeviceGdix51c0 *self);
static void     gdix51c0_baseline_load (FpiDeviceGdix51c0 *self);
static void     gdix51c0_baseline_save (FpiDeviceGdix51c0 *self);
static void     gdix51c0_baseline_adopt_and_save (
  FpiDeviceGdix51c0 *self,
  const guint16     *raw);
static void     gdix51c0_chicago_calibration_maybe_load_or_generate (
  FpiDeviceGdix51c0 *self);
static void     gdix51c0_chicago_preprocessor_maybe_init (FpiDeviceGdix51c0 *self);

static void
gdix51c0_mark_cold_boundary (FpiDeviceGdix51c0 *self,
                             const gchar       *reason)
{
  /* Keep the persisted/raw T0 as a failure fallback, but do not claim that
   * either the sensor FDT base or EngineAdapter ImageBase was refreshed for
   * this hardware handle.  libfprint bypasses driver suspend callbacks while
   * idle, so open() is the reliable post-resume boundary too. */
  self->image_base_refreshed_this_open = FALSE;
  self->fdt_baseline_valid = FALSE;
  self->fdt_down_sample_valid = FALSE;
  self->require_lift_gap = FALSE;
  fp_dbg ("gdix51c0: cold boundary (%s); ImageBase refresh pending",
          reason);
}

static const guint8 gdix51c0_default_fdt_down_regs[12] = {
  0x80, 0xad, 0x80, 0xbb, 0x80, 0xa2,
  0x80, 0xae, 0x80, 0xa3, 0x80, 0xae
};

/* ------------------------------------------------------------------ */
/* GPIO bring-up                                                       */
/* ------------------------------------------------------------------ */

static int
gdix51c0_env_int (const char *name, int fallback)
{
  const char *s = g_getenv (name);
  if (!s || !*s)
    return fallback;
  return (int) g_ascii_strtoll (s, NULL, 0);
}

static guint
gdix51c0_fdt_touch_count (guint8 touchflag)
{
  guint count = 0;

  for (guint8 zones = touchflag & 0x3f; zones; zones >>= 1)
    count += zones & 1;

  return count;
}

static guint
gdix51c0_fdt_trigger_min (FpiDeviceGdix51c0 *self)
{
  if (self->sensor_profile && self->sensor_profile->fdt_trigger_min > 0)
    return self->sensor_profile->fdt_trigger_min;

  return 5;
}

static gboolean
gdix51c0_fdt_touch_is_finger (FpiDeviceGdix51c0 *self,
                              guint8             touchflag)
{
  return gdix51c0_fdt_touch_count (touchflag) >=
         gdix51c0_fdt_trigger_min (self);
}

/* libgpiod v2: chip → line settings → line config → request.  For the
 * IRQ line we also enable both-edge detection so the kernel buffers
 * events for us; that beats polling for short MCU pulses. */
static struct gpiod_line_request *
gdix51c0_request_line (struct gpiod_chip   *chip,
                    unsigned int         offset,
                    enum gpiod_line_direction dir,
                    int                  initial_high,
                    gboolean             want_edges,
                    const char          *consumer,
                    GError             **err)
{
  struct gpiod_line_settings *settings = gpiod_line_settings_new ();
  struct gpiod_line_config   *line_cfg = gpiod_line_config_new ();
  struct gpiod_request_config *req_cfg = gpiod_request_config_new ();
  struct gpiod_line_request  *req      = NULL;

  if (!settings || !line_cfg || !req_cfg)
    {
      g_set_error (err, G_IO_ERROR, G_IO_ERROR_FAILED,
                   "gdix51c0: gpiod alloc failed");
      goto out;
    }

  gpiod_line_settings_set_direction (settings, dir);
  if (dir == GPIOD_LINE_DIRECTION_OUTPUT)
    gpiod_line_settings_set_output_value (settings,
                                          initial_high ? GPIOD_LINE_VALUE_ACTIVE
                                                       : GPIOD_LINE_VALUE_INACTIVE);
  if (want_edges)
    gpiod_line_settings_set_edge_detection (settings, GPIOD_LINE_EDGE_BOTH);

  if (gpiod_line_config_add_line_settings (line_cfg, &offset, 1, settings) < 0)
    {
      g_set_error (err, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: gpiod add line %u failed", offset);
      goto out;
    }
  gpiod_request_config_set_consumer (req_cfg, consumer);

  req = gpiod_chip_request_lines (chip, req_cfg, line_cfg);
  if (!req)
    g_set_error (err, G_IO_ERROR, g_io_error_from_errno (errno),
                 "gdix51c0: request line %u failed", offset);

out:
  if (settings) gpiod_line_settings_free (settings);
  if (line_cfg) gpiod_line_config_free (line_cfg);
  if (req_cfg)  gpiod_request_config_free (req_cfg);
  return req;
}

static gboolean
gdix51c0_open_gpios (FpiDeviceGdix51c0 *self, GError **err)
{
  const char *chip_path = g_getenv (GDIX51C0_ENV_GPIOCHIP);

  if (!chip_path || !*chip_path)
    chip_path = GDIX51C0_DEFAULT_GPIOCHIP;

  self->irq_offset   = gdix51c0_env_int (GDIX51C0_ENV_IRQ_LINE,   GDIX51C0_DEFAULT_IRQ_LINE);
  self->reset_offset = gdix51c0_env_int (GDIX51C0_ENV_RESET_LINE, GDIX51C0_DEFAULT_RESET_LINE);

  self->gpio_chip = gpiod_chip_open (chip_path);
  if (!self->gpio_chip)
    {
      g_set_error (err, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: open gpiochip %s failed", chip_path);
      return FALSE;
    }

  self->irq_req = gdix51c0_request_line (self->gpio_chip, self->irq_offset,
                                      GPIOD_LINE_DIRECTION_INPUT, 0,
                                      TRUE /* edges */,
                                      "gdix51c0-irq", err);
  if (!self->irq_req)
    return FALSE;

  /* Buffer up to 16 pending edges; we typically expect 2-4 per cmd. */
  self->irq_events = gpiod_edge_event_buffer_new (16);
  if (!self->irq_events)
    {
      g_set_error_literal (err, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: alloc edge event buffer failed");
      return FALSE;
    }

  /* Reset line is opened on-demand inside reset_pulse — see comment
   * there for why we don't keep a persistent OUTPUT request. */
  return TRUE;
}

static void
gdix51c0_close_gpios (FpiDeviceGdix51c0 *self)
{
  g_clear_pointer (&self->irq_req,    gpiod_line_request_release);
  g_clear_pointer (&self->irq_events, gpiod_edge_event_buffer_free);
  g_clear_pointer (&self->gpio_chip,  gpiod_chip_close);
}

static void
gdix51c0_open_thread_free (Gdix51c0OpenThread *open_thread)
{
  g_object_unref (open_thread->dev);
  g_clear_pointer (&open_thread->context, g_main_context_unref);
  g_clear_object (&open_thread->cancellable);
  g_free (open_thread);
}

static void
gdix51c0_open_complete_free (Gdix51c0OpenComplete *complete)
{
  g_object_unref (complete->dev);
  g_clear_error (&complete->error);
  g_free (complete);
}

static gboolean
gdix51c0_open_complete_main (gpointer user_data)
{
  Gdix51c0OpenComplete *complete = user_data;
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (complete->dev);

  g_clear_pointer (&self->action_context, g_main_context_unref);
  g_clear_object (&self->action_cancellable);

  if (complete->error)
    {
      /* The activation loop has already reset every failed session. Release
       * the host handles so a later Claim can retry open from a clean state. */
      g_clear_pointer (&self->listener, gdix51c0_listener_free);
      if (self->tls_ready)
        {
          gdix51c0_tls_free (&self->tls);
          self->tls_ready = FALSE;
        }
      g_clear_pointer (&self->chicago_preprocessor,
                       goodix_chicago_preprocessor_free);
      g_clear_pointer (&self->chicago_calibration, g_bytes_unref);
      if (self->spi_fd >= 0)
        {
          close (self->spi_fd);
          self->spi_fd = -1;
        }
      gdix51c0_close_gpios (self);
      fpi_device_open_complete (complete->dev,
                                g_steal_pointer (&complete->error));
      return G_SOURCE_REMOVE;
    }

  fp_info ("gdix51c0: background warm session ready (TLS + T0)");
  fpi_device_open_complete (complete->dev, NULL);
  return G_SOURCE_REMOVE;
}

static gpointer
gdix51c0_open_thread (gpointer user_data)
{
  Gdix51c0OpenThread *open_thread = user_data;
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (open_thread->dev);
  Gdix51c0OpenComplete *complete = g_new0 (Gdix51c0OpenComplete, 1);

  complete->dev = g_object_ref (open_thread->dev);
  if (!gdix51c0_session_activate (self, &complete->error))
    {
      /* open_complete_main decides success from the error alone; a failure
       * without one would be reported as a ready warm session. */
      if (!complete->error)
        g_set_error_literal (&complete->error, G_IO_ERROR, G_IO_ERROR_FAILED,
                             "gdix51c0: warm activation failed without an error");
      fp_warn ("gdix51c0: background warm activation failed: %s",
               complete->error->message);
    }

  g_main_context_invoke_full (
    open_thread->context,
    G_PRIORITY_DEFAULT,
    gdix51c0_open_complete_main,
    complete,
    (GDestroyNotify) gdix51c0_open_complete_free);
  gdix51c0_open_thread_free (open_thread);
  return NULL;
}

/* ------------------------------------------------------------------ */
/* Device open/close                                                   */
/* ------------------------------------------------------------------ */

static void
gdix51c0_open (FpDevice *dev)
{
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (dev);
  GError *err = NULL;

  G_DEBUG_HERE ();

  gdix51c0_mark_cold_boundary (self, "device open");
  self->psk_reprovision_pending = FALSE;
  self->psk_reprovision_attempted_this_open = FALSE;
  self->psk_reprovision_completed_this_open = FALSE;

  const char *spi_path = fpi_device_get_udev_data (dev, FPI_DEVICE_UDEV_SUBTYPE_SPIDEV);
  fp_dbg ("gdix51c0: opening spidev at %s", spi_path ? spi_path : "(null)");
#ifdef GOODIX_SPI_DEVELOPER
  fp_warn ("gdix51c0: developer diagnostics and fault injection are compiled in");
#endif
  if (!spi_path)
    {
      g_set_error_literal (&err, G_IO_ERROR, G_IO_ERROR_NOT_FOUND,
                           "gdix51c0: spidev node not found");
      fpi_device_open_complete (dev, err);
      return;
    }

  self->spi_fd = open (spi_path, O_RDWR);
  if (self->spi_fd < 0)
    {
      g_set_error (&err, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: open spidev %s failed", spi_path);
      fpi_device_open_complete (dev, err);
      return;
    }

  /* Match Python: 10 MHz, mode 0, 8 bits/word.  Without this we inherit
   * whatever the kernel set the fd to.  GDIX51C0_SPI_SPEED_HZ env lets us sweep
   * the bus clock — some Chicago parts derive the internal readout clock from
   * SPI, so this is the last knob for the 0x0b78 readout cliff. */
  guint32 speed = (guint32) GDIX51C0_DEV_ENV_INT ("GDIX51C0_SPI_SPEED_HZ",
                                                  GDIX51C0_SPI_SPEED_HZ);
  fp_dbg ("gdix51c0: SPI speed = %u Hz", speed);
  guint8  mode  = GDIX51C0_SPI_MODE;
  guint8  bpw   = 8;
  if (ioctl (self->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0 ||
      ioctl (self->spi_fd, SPI_IOC_WR_MODE,         &mode)  < 0 ||
      ioctl (self->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bpw)  < 0)
    {
      g_set_error (&err, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: spidev configure failed");
      close (self->spi_fd);
      self->spi_fd = -1;
      fpi_device_open_complete (dev, err);
      return;
    }
  /* Read back to confirm the kernel honored what we asked for. */
  guint32 actual_speed = 0;
  guint8  actual_mode  = 0xff;
  guint8  actual_bpw   = 0;
  ioctl (self->spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &actual_speed);
  ioctl (self->spi_fd, SPI_IOC_RD_MODE,         &actual_mode);
  ioctl (self->spi_fd, SPI_IOC_RD_BITS_PER_WORD, &actual_bpw);
  fp_dbg ("gdix51c0: spidev configured: speed=%u Hz mode=0x%02x bpw=%u",
          actual_speed, actual_mode, actual_bpw);

  if (!gdix51c0_open_gpios (self, &err))
    {
      close (self->spi_fd);
      self->spi_fd = -1;
      gdix51c0_close_gpios (self);
      fpi_device_open_complete (dev, err);
      return;
    }

  fpi_device_set_nr_enroll_stages (
    dev, GOODIX_CHICAGO_ENGINE_REQUIRED_SAMPLES);

  {
    const char *warm_session = g_getenv (GDIX51C0_ENV_WARM_SESSION);

    if (warm_session && *warm_session && g_strcmp0 (warm_session, "0") != 0)
      {
        Gdix51c0OpenThread *open_thread = g_new0 (Gdix51c0OpenThread, 1);
        GThread *thread;
        GCancellable *cancellable = fpi_device_get_cancellable (dev);

        open_thread->dev = g_object_ref (dev);
        open_thread->context = g_main_context_ref_thread_default ();
        open_thread->cancellable = cancellable ? g_object_ref (cancellable) : NULL;

        g_clear_pointer (&self->action_context, g_main_context_unref);
        g_clear_object (&self->action_cancellable);
        self->action_context = g_main_context_ref (open_thread->context);
        self->action_cancellable = open_thread->cancellable ?
          g_object_ref (open_thread->cancellable) : NULL;

        fp_info ("gdix51c0: starting background warm activation");
        thread = g_thread_try_new ("gdix51c0-open",
                                   gdix51c0_open_thread,
                                   open_thread,
                                   &err);
        if (!thread)
          {
            gdix51c0_open_thread_free (open_thread);
            g_clear_pointer (&self->action_context, g_main_context_unref);
            g_clear_object (&self->action_cancellable);
            close (self->spi_fd);
            self->spi_fd = -1;
            gdix51c0_close_gpios (self);
            fpi_device_open_complete (dev, err);
            return;
          }

        g_thread_unref (thread);
        return;
      }
  }

  fpi_device_open_complete (dev, NULL);
}

static void
gdix51c0_close (FpDevice *dev)
{
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (dev);

  G_DEBUG_HERE ();

  /* Defensive: if a session never got cleanly deactivated, the listener is
   * still running.  Stop it first so we don't release the IRQ line out from
   * underneath the reader thread. */
  g_clear_pointer (&self->listener, gdix51c0_listener_free);

  if (self->tls_ready)
    {
      gdix51c0_tls_free (&self->tls);
      self->tls_ready = FALSE;
    }

  g_clear_pointer (&self->chicago_preprocessor,
                   goodix_chicago_preprocessor_free);
  g_clear_pointer (&self->chicago_calibration, g_bytes_unref);

  if (self->spi_fd >= 0)
    {
      close (self->spi_fd);
      self->spi_fd = -1;
    }
  gdix51c0_close_gpios (self);

  fpi_device_close_complete (dev, NULL);
}

/* Reset + post-reset IRQ pulse + force-unlock-TLS (cmd 0xd5, no ack). */
static gboolean
gdix51c0_boot_probe (FpiDeviceGdix51c0 *self, GError **error)
{
  Gdix51c0Bus bus = {
    .dev = FP_DEVICE (self), .spi_fd = self->spi_fd,
    .irq_req = self->irq_req, .irq_offset = self->irq_offset,
    .irq_events = self->irq_events,
  };

  gdix51c0_irq_drain (self->irq_req, self->irq_events);
  if (!gdix51c0_reset_pulse (self->gpio_chip, self->reset_offset, error))
    return FALSE;

  if (!gdix51c0_irq_wait (self->irq_req, self->irq_events, TRUE,
                       self->irq_offset, GDIX51C0_BOOT_TIMEOUT_USEC,
                       "boot-rise", error))
    return FALSE;
  if (!gdix51c0_irq_wait (self->irq_req, self->irq_events, FALSE,
                       self->irq_offset, GDIX51C0_BOOT_TIMEOUT_USEC,
                       "boot-fall", error))
    return FALSE;

  /* Settle delay: the rise/fall we just caught may be a GPIO glitch from
   * our reset toggle, not the MCU's actual post-reset boot pulse.  Give
   * the MCU at least 100ms to fully boot, then drain any later edges so
   * the cmd flow starts from a clean kernel buffer. */
  g_usleep (200000);
  gdix51c0_irq_drain (self->irq_req, self->irq_events);

  static const guint8 d5_payload[] = { 0xd5, 0x03, 0x00, 0x00, 0x00, 0xd3 };
  return gdix51c0_cmd_no_ack (&bus, d5_payload, sizeof (d5_payload),
                           "force-unlock-tls", error);
}

static void
gdix51c0_debug_mcu_state_resp (const guint8 *buf, gsize len)
{
#ifdef GOODIX_SPI_DEVELOPER
  GString *s;
#endif

  if (!buf || len == 0)
    {
      fp_dbg ("gdix51c0: mcu-state: empty response");
      return;
    }

#ifdef GOODIX_SPI_DEVELOPER
  s = g_string_new (NULL);
  for (gsize i = 0; i < len; i++)
    g_string_append_printf (s, "%02x", buf[i]);

  fp_dbg ("gdix51c0: mcu-state raw response len=%zu: %s", len, s->str);
  g_string_free (s, TRUE);
#endif

  if (len < 4)
    {
      fp_warn ("gdix51c0: mcu-state response too short: %zu", len);
      return;
    }

  guint8 cmd = buf[0];
  guint16 inner_len = (guint16) buf[1] | ((guint16) buf[2] << 8);
  gsize expected_total = 3 + inner_len;

  if (cmd != 0xae)
    fp_warn ("gdix51c0: mcu-state response cmd is 0x%02x, expected 0xae", cmd);

  if (len != expected_total)
    {
      fp_warn ("gdix51c0: mcu-state total len=%zu, inner_len=%u, expected total=%zu",
               len, inner_len, expected_total);
      return;
    }

  fp_dbg ("gdix51c0: mcu-state packet OK: cmd=0x%02x inner_len=%u total=%zu",
          cmd, inner_len, len);
}

static gboolean
gdix51c0_cmd_required_same_irq (Gdix51c0Bus  *bus,
                                 const guint8 *payload,
                                 gsize         payload_len,
                                 guint         timeout_usec,
                                 const char   *label,
                                 GError      **error)
{
  gsize response_len = 0;
  g_autofree guint8 *response =
    gdix51c0_cmd_ack_resp_same_irq (bus, payload, payload_len,
                                    timeout_usec, &response_len,
                                    label, error);

  return response != NULL;
}

/* Full 14213 pre-TLS init sequence recovered from official-driver traces,
 * from get-EVK-version through MCU-config upload.
 * The MCU rejects cmd 0xd1 (TLS trigger) until this whole sequence has run. */
static gboolean
gdix51c0_init_sequence (FpiDeviceGdix51c0 *self, GError **error)
{
  Gdix51c0Bus bus = {
    .dev = FP_DEVICE (self), .spi_fd = self->spi_fd,
    .irq_req = self->irq_req, .irq_offset = self->irq_offset,
    .irq_events = self->irq_events,
  };

  /* "required for ..." prefix: 01 05 00 00 00 00 00 88 — no ack.
   * Sent once before get_evk_version and once before get_mcu_state. */
  static const guint8 prefix01[] = { 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88 };

  /* get_evk_version: NOP/prefix then cmd 0xa8.  The official SendNopCmd
   * sleeps 10 ms after its no-ACK send; the transport itself adds 1 ms before
   * each submit. */
  static const guint8 a8_payload[] = { 0xa8, 0x03, 0x00, 0x00, 0x00, 0xff };
  for (guint evk_attempt = 1; evk_attempt <= 3; evk_attempt++)
    {
      g_autoptr(GError) evk_error = NULL;

      if (gdix51c0_cmd_no_ack (&bus, prefix01, sizeof (prefix01),
                               "evk-prefix", &evk_error))
        {
          g_usleep (10000);
          if (gdix51c0_cmd_required_same_irq (&bus,
                                              a8_payload,
                                              sizeof (a8_payload),
                                              1000000,
                                              "get-evk-version",
                                              &evk_error))
            break;
        }

      if (evk_attempt == 3)
        {
          g_propagate_error (error, g_steal_pointer (&evk_error));
          return FALSE;
        }

      /* GetEvkVersionWithRetry uses three attempts and performs a hard reset
       * plus Sleep(500) between them.  This is the recovery boundary that was
       * missing when one lost A8 IRQ escalated to a complete activation retry. */
      fp_warn ("gdix51c0: get-EVK attempt %u/3 failed (%s); hard reset and "
               "500 ms retry like Windows",
               evk_attempt, evk_error ? evk_error->message : "?");
      if (!gdix51c0_reset_pulse (self->gpio_chip, self->reset_offset, error))
        return FALSE;
      g_usleep (500000);
      gdix51c0_irq_drain (self->irq_req, self->irq_events);
    }

  /* get_mcu_state: prefix → SendNopCmd's 10ms → cmd 0xaf with timestamp + ts-checksum
   * (single response, no ack — only one IRQ cycle).  We can reuse cmd_resp-
   * style flow: write, IRQ rise, read, IRQ fall.  No ack-only read.        */
  if (!gdix51c0_cmd_no_ack (&bus, prefix01, sizeof (prefix01), "mcu-state-prefix", error))
    return FALSE;
  g_usleep (10000);
  {
    GDateTime *now = g_date_time_new_now_utc ();
    guint ms = g_date_time_get_seconds (now) * 1000.0;
    g_date_time_unref (now);
    guint16 ms16 = (guint16) (ms & 0xffff);

    guint8 af_data[7];
    af_data[0] = 0xaf;
    af_data[1] = 0x06; af_data[2] = 0x00;        /* inner len */
    af_data[3] = 0x55;
    af_data[4] = ms16 & 0xff; af_data[5] = (ms16 >> 8) & 0xff;
    af_data[6] = 0x00;
    guint8 csum = gdix51c0_payload_checksum_ts (af_data, sizeof (af_data));
    guint8 af_full[9];
    memcpy (af_full, af_data, sizeof (af_data));
    af_full[7] = 0x00;        /* python: af 06 00 55 ms_lo ms_hi 00 00 csum */
    af_full[8] = csum;

    gsize mcu_state_len = 0;
    g_autofree guint8 *mcu_state = NULL;

    mcu_state = gdix51c0_cmd_single_resp_level (&bus,
                                                af_full,
                                                sizeof (af_full),
                                                GDIX51C0_RESPONSE_TIMEOUT_USEC,
                                                &mcu_state_len,
                                                "get-mcu-state",
                                                error);
    if (!mcu_state)
      return FALSE;

    gdix51c0_debug_mcu_state_resp (mcu_state, mcu_state_len);
  }

  /* reset_sensor (cmd 0xa2) — ack then response in separate IRQ cycles */
  guint8 reset_data[GOODIX_COMMAND_RESET_PAYLOAD_SIZE];
  guint8 a2_payload[GOODIX_COMMAND_RESET_PAYLOAD_SIZE +
                    GOODIX_WIRE_COMMAND_OVERHEAD];
  gsize a2_payload_size = 0;

  goodix_command_encode_reset (TRUE, FALSE, 0x14, reset_data);
  if (!goodix_wire_encode_command (GOODIX_COMMAND_RESET,
                                   reset_data, sizeof (reset_data),
                                   a2_payload, sizeof (a2_payload),
                                   &a2_payload_size) ||
      a2_payload_size != sizeof (a2_payload))
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: could not encode reset command");
      return FALSE;
    }
  if (!gdix51c0_cmd_ack_then_resp (&bus, a2_payload, sizeof (a2_payload),
                                "reset-sensor-1", error))
    return FALSE;

  /* Get MILAN_CHIPID (cmd 0x82). The official response contains the four-byte
   * LE chip id at register zero; 0x2504 resolves to sensor type 12/ChicagoHS. */
  guint8 register_data[GOODIX_COMMAND_REGISTER_PAYLOAD_SIZE];
  guint8 chip_payload[GOODIX_COMMAND_REGISTER_PAYLOAD_SIZE +
                      GOODIX_WIRE_COMMAND_OVERHEAD];
  gsize chip_payload_size = 0;
  gsize chip_response_size = 0;
  g_autofree guint8 *chip_response = NULL;
  const guint8 *chip_data = NULL;
  gsize chip_data_size = 0;
  guint8 chip_command = 0;
  guint16 chip_id_le;
  guint32 chip_id;

  goodix_command_encode_register_read (0, 4, register_data);
  if (!goodix_wire_encode_command (GOODIX_COMMAND_READ_SENSOR_REGISTER,
                                   register_data, sizeof (register_data),
                                   chip_payload, sizeof (chip_payload),
                                   &chip_payload_size) ||
      chip_payload_size != sizeof (chip_payload))
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: could not encode chip-id command");
      return FALSE;
    }
  chip_response = gdix51c0_cmd_ack_resp_same_irq (
    &bus, chip_payload, sizeof (chip_payload), 200000,
    &chip_response_size, "milan-chipid", error);
  if (!chip_response)
    return FALSE;
  if (!goodix_wire_decode_command (chip_response, chip_response_size, FALSE,
                                   &chip_command, &chip_data,
                                   &chip_data_size) ||
      chip_command != GOODIX_COMMAND_READ_SENSOR_REGISTER ||
      chip_data_size != 4)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                   "gdix51c0: malformed chip-id response (cmd=0x%02x, "
                   "payload=%zu, packet=%zu)",
                   chip_command, chip_data_size, chip_response_size);
      return FALSE;
    }
  /* The register-read payload is [status, value_lo, value_hi, reserved].
   * Live GDIX51C0 bytes are a2 04 25 00, yielding chip id 0x2504. */
  memcpy (&chip_id_le, chip_data + 1, sizeof (chip_id_le));
  chip_id = GUINT16_FROM_LE (chip_id_le);
  self->sensor_profile = goodix_sensor_profile_lookup (chip_id);
  if (!self->sensor_profile)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_NOT_SUPPORTED,
                   "gdix51c0: unsupported Goodix chip id 0x%08x", chip_id);
      return FALSE;
    }
  if (self->sensor_profile->algorithm !=
        GOODIX_SENSOR_ALGORITHM_CHICAGO_HS ||
      self->sensor_profile->width != GDIX51C0_IMAGE_WIDTH ||
      self->sensor_profile->height != GDIX51C0_IMAGE_HEIGHT)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_NOT_SUPPORTED,
                   "gdix51c0: chip 0x%08x resolved to incompatible profile %s "
                   "%ux%u", chip_id, self->sensor_profile->algorithm_name,
                   (guint) self->sensor_profile->width,
                   (guint) self->sensor_profile->height);
      return FALSE;
    }
  fp_info ("gdix51c0: sensor profile chip=0x%04x type=%u geometry=%ux%u "
           "otp=%u algorithm=%s",
           chip_id, (guint) self->sensor_profile->sensor_type,
           (guint) self->sensor_profile->width,
           (guint) self->sensor_profile->height,
           (guint) self->sensor_profile->otp_size,
           self->sensor_profile->algorithm_name);

  /* Per-unit analog DAC bias.  Self-calibrated from the sensor OTP below, exactly
   * like Windows (ChicagoHU_OTP_DAC_Check): the "ft dac" lives at OTP[0x32..0x35]
   * (mt-dac fallback at OTP[0x2e..0x31]), and
   *     main = (dac[0] << 4) | 8   -> reg 0x0220 / cmd-0x98 reg0
   *     dac1 = dac[1], dac2 = dac[2], dac3 = dac[3]  -> regs 0x0236/38/3a.
   * Legacy fallback 0x0b38/0xb5/0xb3/0xb3 is used only if the OTP read/parse
   * fails or GDIX51C0_DAC_FROM_OTP=0.  GDIX51C0_DAC="main,d1,d2,d3" force-overrides. */
  guint dac_main = 0x0b38, dac_1 = 0xb5, dac_2 = 0xb3, dac_3 = 0xb3;
  gboolean dac_from_otp = FALSE;

  /* get OTP (cmd 0xa6) — ack then response in separate IRQ cycles.  Response:
   *   [0xa6, len_lo, len_hi, <64-byte OTP>, csum] */
  {
    static const guint8 otp_payload[] = { 0xa6, 0x03, 0x00, 0x00, 0x00, 0x01 };
    gsize otp_len = 0;
    gsize otp_data_len = 0;
    const guint8 *otp_data = NULL;
    guint8 otp_command = 0;
    g_autofree guint8 *otp =
      gdix51c0_cmd_ack_then_resp_read (&bus, otp_payload, sizeof (otp_payload),
                                       500000, &otp_len, "get-otp", error);
    if (!otp)
      return FALSE;

    if (!goodix_wire_decode_command (otp, otp_len, FALSE, &otp_command,
                                     &otp_data, &otp_data_len) ||
        otp_command != GOODIX_COMMAND_READ_OTP ||
        otp_data_len != self->sensor_profile->otp_size)
      {
        g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                     "gdix51c0: malformed OTP response (cmd=0x%02x, "
                     "payload=%zu, expected=%u, packet=%zu)",
                     otp_command, otp_data_len,
                     (guint) self->sensor_profile->otp_size, otp_len);
        return FALSE;
      }

    if (self->sensor_id_valid &&
        memcmp (self->sensor_id, otp_data, sizeof (self->sensor_id)) != 0)
      {
        /* This should not happen for one open device, but never carry a base
         * frame into a different unit if the backing device changed. */
        fp_warn ("gdix51c0: active sensor id changed; discarding ImageBase");
        self->t0_baseline_valid = FALSE;
        self->image_base_refreshed_this_open = FALSE;
        g_clear_pointer (&self->chicago_preprocessor,
                         goodix_chicago_preprocessor_free);
        g_clear_pointer (&self->chicago_calibration, g_bytes_unref);
      }

    memcpy (self->sensor_id, otp_data, sizeof (self->sensor_id));
    self->sensor_id_valid = TRUE;
#ifdef GOODIX_SPI_DEVELOPER
    {
      g_autofree gchar *sensor_id_hex =
        g_compute_checksum_for_data (G_CHECKSUM_SHA256,
                                     self->sensor_id,
                                     sizeof (self->sensor_id));
      fp_dbg ("gdix51c0: Chicago sensor id SHA-256=%s",
              sensor_id_hex);
    }
#endif

    if (otp_data_len >= 54)
      {
        if (GDIX51C0_DEV_ENV_INT ("GDIX51C0_DAC_FROM_OTP", 1))
          {
            const guint8 *ftd = otp_data + 0x32;   /* ft dac */
            const guint8 *mtd = otp_data + 0x2e;   /* mt dac fallback */
            const guint8 *dac = NULL;
            if (ftd[0] && ftd[1] && ftd[2] && ftd[3]) dac = ftd;
            else if (mtd[0] && mtd[1] && mtd[2] && mtd[3]) dac = mtd;

            if (dac)
              {
                dac_main = ((guint) dac[0] << 4) | 8;
                dac_1 = dac[1]; dac_2 = dac[2]; dac_3 = dac[3];
                dac_from_otp = TRUE;
                fp_dbg ("gdix51c0: DAC self-calibrated from OTP (%s block)",
                        dac == ftd ? "ft" : "mt");
              }
            else
              fp_warn ("gdix51c0: OTP DAC block empty; using legacy default");
          }
      }
    else
      fp_warn ("gdix51c0: OTP response too short: %zu; using legacy DAC", otp_len);
  }

  /* The ImageBase is the native input to Chicago preprocessing.  Its file is
   * keyed by the just-read OTP id, so defer loading until sensor identity is
   * known instead of accepting an unbound frame during open(). */
  if (!self->t0_baseline_valid)
    gdix51c0_baseline_load (self);
  gdix51c0_chicago_calibration_maybe_load_or_generate (self);
  gdix51c0_chicago_preprocessor_maybe_init (self);

  /* reset_sensor again */
  if (!gdix51c0_cmd_ack_then_resp (&bus, a2_payload, sizeof (a2_payload),
                                "reset-sensor-2", error))
    return FALSE;

  /* setmode idle (cmd 0x70) — ack only, no separate response */
  guint8 idle_data[GOODIX_COMMAND_IDLE_PAYLOAD_SIZE];
  guint8 setmode_idle[GOODIX_COMMAND_IDLE_PAYLOAD_SIZE +
                      GOODIX_WIRE_COMMAND_OVERHEAD];
  gsize setmode_idle_size = 0;

  goodix_command_encode_idle (0x14, idle_data);
  if (!goodix_wire_encode_command (GOODIX_COMMAND_MCU_SWITCH_TO_IDLE_MODE,
                                   idle_data, sizeof (idle_data),
                                   setmode_idle, sizeof (setmode_idle),
                                   &setmode_idle_size) ||
      setmode_idle_size != sizeof (setmode_idle))
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: could not encode idle command");
      return FALSE;
    }
  if (!gdix51c0_cmd_ack (&bus, setmode_idle, sizeof (setmode_idle),
                      "setmode-idle", error))
    return FALSE;

  /* Apply the DAC (cmd 0x98 + config regs 0x0220/0x0236/0x0238/0x023a).  Value
   * is the OTP self-calibration derived above; GDIX51C0_DAC="main,d1,d2,d3"
   * force-overrides it (e.g. to pin the legacy 0x0b38 for full-frame readout).
   * NOTE: a higher DAC raises the FDT baseline — finger-down/up thresholds now
   * track it automatically via the dynamic cmd-0x36 base measurement. */
  {
    const char *dac_env = GDIX51C0_DEV_ENV ("GDIX51C0_DAC");
    if (dac_env && *dac_env)
      {
        guint v[4];
        if (sscanf (dac_env, "%i,%i,%i,%i", &v[0], &v[1], &v[2], &v[3]) == 4)
          {
            dac_main = v[0]; dac_1 = v[1]; dac_2 = v[2]; dac_3 = v[3];
            dac_from_otp = FALSE;
            fp_info ("gdix51c0: DAC override main=0x%04x d1=0x%02x d2=0x%02x d3=0x%02x",
                     dac_main, dac_1, dac_2, dac_3);
          }
        else
          fp_warn ("gdix51c0: bad GDIX51C0_DAC \"%s\" (want main,d1,d2,d3)", dac_env);
      }
  }

  /* send Dac (cmd 0x98) — ack+resp same cycle.  Checksum recomputed. */
  guint8 dac_payload[] = {
    0x98, 0x09, 0x00,
    (guint8) (dac_main & 0xff), (guint8) (dac_main >> 8),
    (guint8) dac_1, 0x00, (guint8) dac_2, 0x00, (guint8) dac_3, 0x00,
    0x00
  };
  dac_payload[sizeof (dac_payload) - 1] =
    gdix51c0_payload_checksum (dac_payload, sizeof (dac_payload) - 1);
  if (!gdix51c0_cmd_required_same_irq (&bus,
                                       dac_payload, sizeof (dac_payload),
                                       1000000, "send-dac", error))
    return FALSE;

  /* upload mcu config (cmd 0x90) — 232-byte payload, ack+resp same cycle.
   * The same four DAC values live in config regs 0x0220 (offsets 122/123),
   * 0x0236 (126), 0x0238 (130), 0x023a (134); patch + re-checksum below. */
  guint8 cfg_payload[] = {
    0x90, 0xE1, 0x00, 0x70, 0x11, 0x74, 0x85, 0x00, 0x85, 0x2C, 0xB1, 0x18, 0xC9, 0x14, 0xDD, 0x00,
    0xDD, 0x00, 0xDD, 0x00, 0xBA, 0x00, 0x01, 0x80, 0xCA, 0x00, 0x04, 0x00, 0x84, 0x00, 0x15, 0xB3,
    0x86, 0x00, 0x00, 0xC4, 0x88, 0x00, 0x00, 0xBA, 0x8A, 0x00, 0x00, 0xB2, 0x8C, 0x00, 0x00, 0xAA,
    0x8E, 0x00, 0x00, 0xC1, 0x90, 0x00, 0xBB, 0xBB, 0x92, 0x00, 0xB1, 0xB1, 0x94, 0x00, 0x00, 0xA8,
    0x96, 0x00, 0x00, 0xB6, 0x98, 0x00, 0x00, 0x00, 0x9A, 0x00, 0x00, 0x00, 0xD2, 0x00, 0x00, 0x00,
    0xD4, 0x00, 0x00, 0x00, 0xD6, 0x00, 0x00, 0x00, 0xD8, 0x00, 0x00, 0x00, 0x50, 0x00, 0x01, 0x05,
    0xD0, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x72, 0x00, 0x78, 0x56, 0x74, 0x00, 0x34, 0x12,
    0x20, 0x00, 0x10, 0x40, 0x5C, 0x00, 0x00, 0x01, 0x20, 0x02, 0x38, 0x0B, 0x36, 0x02, 0xB5, 0x00,
    0x38, 0x02, 0xB3, 0x00, 0x3A, 0x02, 0xB3, 0x00, 0x2A, 0x01, 0x82, 0x03, 0x22, 0x00, 0x01, 0x20,
    0x24, 0x00, 0x14, 0x00, 0x80, 0x00, 0x01, 0x00, 0x5C, 0x00, 0x00, 0x01, 0x56, 0x00, 0x04, 0x20,
    0x58, 0x00, 0x03, 0x02, 0x32, 0x00, 0x0C, 0x02, 0x66, 0x00, 0x03, 0x00, 0x7C, 0x00, 0x00, 0x58,
    0x82, 0x00, 0x80, 0x1B, 0x2A, 0x01, 0x08, 0x00, 0x54, 0x00, 0x10, 0x01, 0x62, 0x00, 0x04, 0x03,
    0x64, 0x00, 0x19, 0x00, 0x66, 0x00, 0x03, 0x00, 0x7C, 0x00, 0x00, 0x58, 0x2A, 0x01, 0x08, 0x00,
    0x52, 0x00, 0x08, 0x00, 0x54, 0x00, 0x00, 0x01, 0x66, 0x00, 0x03, 0x00, 0x7C, 0x00, 0x00, 0x58,
    0x00, 0x53, 0x66, 0x8F
  };
  /* Mirror the DAC into the config-blob regs and re-checksum. */
  cfg_payload[122] = (guint8) (dac_main & 0xff);
  cfg_payload[123] = (guint8) (dac_main >> 8);
  cfg_payload[126] = (guint8) dac_1;
  cfg_payload[130] = (guint8) dac_2;
  cfg_payload[134] = (guint8) dac_3;

  /* The DAC is not the only per-unit OTP value: Windows' modify_sensor_config
   * also writes reg 0x005c (tcode / integration time -> 0x0110 = 272) and
   * reg 0x0082 (readout timing -> 0x1d80).  These analog-timing registers must
   * track the DAC — the stock blob carries the OLD operating point's values
   * (0x005c=0x0100, 0x0082=0x1b80), and pairing the higher 0x0b78 bias with the
   * old timing leaves the frame readout unable to finish: the bottom ~60% of
   * every frame comes back as garbage.  Apply the OTP timing whenever we run a
   * DAC above the legacy 0x0b38 (i.e. the OTP self-cal or any override), so the
   * corrected bias and readout stay matched. */
  self->active_dac_main = (guint16) dac_main;
  if (dac_main != 0x0b38)
    {
      /* Readout-timing registers.  Windows raises 0x0082 (0x1580->0x1d80) and
       * 0x005c/tcode (0x0180->0x0110) for the higher DAC.  The suppress test
       * proved we're silent on the bus during readout, yet the frame still
       * cliffs at a fixed row — so the sensor finalises a partial frame on its
       * own readout-time budget.  These two regs are that budget.  Make them
       * sweepable so we can find the value that lets the full 80-row readout
       * finish at 0x0b78: GDIX51C0_REG0082 / GDIX51C0_REG005C (full u16 hex). */
      guint reg82 = (guint) GDIX51C0_DEV_ENV_INT ("GDIX51C0_REG0082", 0x1d80);
      guint reg5c = (guint) GDIX51C0_DEV_ENV_INT ("GDIX51C0_REG005C", 0x0110);
      cfg_payload[118] = (guint8) (reg5c & 0xff);   /* reg 0x005c val_lo */
      cfg_payload[119] = (guint8) (reg5c >> 8);     /* reg 0x005c val_hi */
      cfg_payload[178] = (guint8) (reg82 & 0xff);   /* reg 0x0082 val_lo */
      cfg_payload[179] = (guint8) (reg82 >> 8);     /* reg 0x0082 val_hi */
      fp_dbg ("gdix51c0: readout timing configured (dac_from_otp=%d)",
              dac_from_otp);
    }

  /* CRITICAL: recompute the config's INTERNAL 16-bit checksum after patching any
   * register.  Windows' modify_sensor_config recomputes it (gfspi.dll
   * FUN_1800161b0) on every reg write:
   *     crc = (-(0xa5a5 + sum(16-bit LE words over config bytes 3..224))) & 0xffff
   * stored little-endian at bytes 225-226.  If this is stale the MCU SILENTLY
   * REJECTS the whole config (the cmd-0x90 ACK still comes back) and keeps its
   * default readout timing — which is why the frame readout can't finish at the
   * higher 0x0b78 bias and every frame cliffs at row 31.  This was THE root
   * cause of the cliff.  Must run after the DAC/timing patches, before the
   * outer packet checksum. */
  if (!goodix_mcu_config_update_checksum (cfg_payload + 3,
                                          sizeof (cfg_payload) - 4))
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: invalid MCU configuration shape");
      return FALSE;
    }

  cfg_payload[sizeof (cfg_payload) - 1] =
    gdix51c0_payload_checksum (cfg_payload, sizeof (cfg_payload) - 1);
  if (!gdix51c0_cmd_required_same_irq (&bus,
                                       cfg_payload, sizeof (cfg_payload),
                                       1000000, "upload-mcu-config", error))
    return FALSE;

  g_usleep (200000);
  gdix51c0_irq_drain (self->irq_req, self->irq_events);

  fp_dbg ("gdix51c0: pre-TLS init sequence complete");
  return TRUE;
}

static gboolean
gdix51c0_parse_psk_hex (const char *hex, guint8 out[32], GError **err)
{
  if (strlen (hex) != 64)
    {
      g_set_error (err, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                   "gdix51c0: PSK must be 64 hex chars (got %zu)", strlen (hex));
      return FALSE;
    }
  for (int i = 0; i < 32; i++)
    {
      char b[3] = { hex[2*i], hex[2*i + 1], 0 };
      gchar *endp = NULL;
      out[i] = (guint8) g_ascii_strtoull (b, &endp, 16);
      if (!endp || *endp != 0)
        {
          g_set_error_literal (err, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                               "gdix51c0: bad hex in PSK");
          return FALSE;
        }
    }
  return TRUE;
}

static void
gdix51c0_psk_to_hex (const guint8 psk[32], char hex[65])
{
  for (int i = 0; i < 32; i++)
    g_snprintf (hex + 2 * i, 3, "%02x", psk[i]);
}

static char *
gdix51c0_psk_state_path (void)
{
  const char *p = g_getenv (GDIX51C0_ENV_PSK_STATE_FILE);
  if (p && *p)
    return g_strdup (p);
  return g_strdup (GDIX51C0_DEFAULT_PSK_STATE_FILE);
}

/* Read the persisted PSK, if any.  Returns FALSE (no error) when the file is
 * absent or malformed — the caller treats that as "no persisted PSK". */
static gboolean
gdix51c0_load_psk_state (guint8 out[32])
{
  g_autofree char *path = gdix51c0_psk_state_path ();
  g_autofree char *contents = NULL;

  if (!g_file_get_contents (path, &contents, NULL, NULL))
    return FALSE;
  g_strstrip (contents);
  if (strlen (contents) != 64)
    return FALSE;
  return gdix51c0_parse_psk_hex (contents, out, NULL);
}

/* Persist the PSK atomically as 0600.  Writes to a temp file then renames, and
 * chmods before the rename so the secret is never briefly world-readable. */
static gboolean
gdix51c0_save_psk_state (const guint8 psk[32], GError **err)
{
  g_autofree char *path = gdix51c0_psk_state_path ();
  g_autofree char *dir = g_path_get_dirname (path);
  g_autofree char *tmp = g_strconcat (path, ".tmp", NULL);
  char buf[65];
  int fd;

  if (g_mkdir_with_parents (dir, 0700) != 0)
    {
      g_set_error (err, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: cannot create state dir %s: %s", dir, g_strerror (errno));
      return FALSE;
    }

  fd = g_open (tmp, O_CREAT | O_WRONLY | O_TRUNC, 0600);
  if (fd < 0)
    {
      g_set_error (err, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: cannot write %s: %s", tmp, g_strerror (errno));
      return FALSE;
    }
  (void) fchmod (fd, 0600);

  gdix51c0_psk_to_hex (psk, buf);
  buf[64] = '\n';
  if (write (fd, buf, 65) != 65)
    {
      g_set_error (err, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: short write to %s", tmp);
      close (fd);
      g_unlink (tmp);
      return FALSE;
    }
  close (fd);

  if (g_rename (tmp, path) != 0)
    {
      g_set_error (err, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: cannot rename %s -> %s", tmp, path);
      g_unlink (tmp);
      return FALSE;
    }
  return TRUE;
}

static gboolean
gdix51c0_load_psk (guint8 out[32], GError **err)
{
  const char *hex = g_getenv (GDIX51C0_ENV_PSK_HEX);

  if (hex && *hex)
    return gdix51c0_parse_psk_hex (hex, out, err);

  if (gdix51c0_load_psk_state (out))
    return TRUE;

  g_set_error (err, G_IO_ERROR, G_IO_ERROR_NOT_INITIALIZED,
               "gdix51c0: no PSK: set $%s, or provision one with $%s=random "
               "(see re/PARITY.md)",
               GDIX51C0_ENV_PSK_HEX, GDIX51C0_ENV_PROVISION_PSK);
  return FALSE;
}

static guint16 *gdix51c0_capture_image_raw (FpiDeviceGdix51c0 *self,
                                            guint8             cmd,
                                            gboolean           retry_image,
                                            GError           **error);
static gboolean gdix51c0_fdt_measure_base (FpiDeviceGdix51c0 *self,
                                           gboolean          *finger_off,
                                           GError           **error);
static gboolean gdix51c0_capture_nav_data (FpiDeviceGdix51c0 *self, GError **error);

static gboolean
gdix51c0_session_tls_start (FpiDeviceGdix51c0 *self,
                            gboolean          *reset_required,
                            GError           **error)
{
  guint8 psk[32];
  guint8 mcu_psk[32];
  guint8 existing[32] = { 0 };
  gboolean mcu_present = FALSE;
  gboolean psk_ready = FALSE;
  gboolean have_existing = FALSE;
  gboolean repair_requested = self->psk_reprovision_pending;
  const char *prov = g_getenv (GDIX51C0_ENV_PROVISION_PSK);
  Gdix51c0Bus bus = {
    .dev = FP_DEVICE (self), .spi_fd = self->spi_fd,
    .irq_req = self->irq_req, .irq_offset = self->irq_offset,
    .irq_events = self->irq_events,
  };

  g_return_val_if_fail (reset_required != NULL, FALSE);
  *reset_required = FALSE;

  /* Known PSK before any provisioning: env override wins, else persisted state. */
  {
    const char *hex = g_getenv (GDIX51C0_ENV_PSK_HEX);
    if (hex && *hex)
      have_existing = gdix51c0_parse_psk_hex (hex, existing, NULL);
    if (!have_existing)
      have_existing = gdix51c0_load_psk_state (existing);
  }

  /* (Re)provision the sensor (destructive) when requested and actually needed.
   * "random" bootstraps if no PSK is known and restores that same persisted
   * key after a confirmed TLS authentication mismatch, so it is safe to leave
   * set; an explicit hex rewrites the sensor only when it differs from what we
   * hold or the sensor has proven that it no longer has that key.
   * On success the plaintext is persisted (root-owned, 0600) so it survives
   * reboots — the only workable path on units with no Windows/DPAPI. */
  if (prov && *prov)
    {
      guint8 want[32];
      gboolean do_provision = FALSE;

      if (g_ascii_strcasecmp (prov, "random") == 0)
        {
          if (repair_requested && have_existing)
            {
              /* "random" generated this persisted key originally.  Restore
               * that same strong key after another OS changed the sensor;
               * rotating the host copy too would add no security and could
               * lose the only recoverable plaintext. */
              memcpy (want, existing, sizeof (want));
              do_provision = TRUE;
            }
          else if (have_existing)
            fp_info ("gdix51c0: %s=random but a PSK already exists; skipping provision",
                     GDIX51C0_ENV_PROVISION_PSK);
          else if (RAND_bytes (want, sizeof (want)) != 1)
            {
              g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                                   "gdix51c0: provision: RAND_bytes failed");
              return FALSE;
            }
          else
            do_provision = TRUE;
        }
      else if (!gdix51c0_parse_psk_hex (prov, want, error))
        {
          return FALSE;
        }
      else if (!repair_requested && have_existing &&
               memcmp (want, existing, sizeof (want)) == 0)
        {
          fp_info ("gdix51c0: already provisioned with the requested PSK; skipping");
        }
      else
        {
          do_provision = TRUE;
        }

      if (do_provision)
        {
          if (repair_requested)
            {
              /* Consume the bounded repair before the destructive write.  A
               * failed write must not loop across all activation attempts. */
              self->psk_reprovision_pending = FALSE;
              self->psk_reprovision_attempted_this_open = TRUE;
              fp_warn ("gdix51c0: auto-restoring persisted PSK after "
                       "confirmed authentication mismatch");
            }
          fp_warn ("gdix51c0: PROVISIONING PSK to sensor (writes 0xbb010003)");
          if (!gdix51c0_provision_psk (&bus, want, TRUE, error))
            return FALSE;

          if (repair_requested)
            self->psk_reprovision_completed_this_open = TRUE;

          {
            g_autoptr(GError) save_err = NULL;
            g_autofree char *path = gdix51c0_psk_state_path ();
            if (gdix51c0_save_psk_state (want, &save_err))
              {
                fp_info ("gdix51c0: provisioned and persisted PSK to %s", path);
              }
            else
              {
                fp_warn ("gdix51c0: provisioned but could not persist PSK "
                         "state at %s (%s); the sensor must be reprovisioned "
                         "after restart",
                         path, save_err->message);
              }
          }

          memcpy (psk, want, sizeof (psk));
          memcpy (existing, want, sizeof (existing));
          have_existing = TRUE;
          psk_ready = TRUE;

          /* A verified stale-key rewrite starts a new authentication epoch
           * inside the MCU.  Real post-Windows hardware may acknowledge and
           * verify the WB write but never raise the first TLS IRQ until it has
           * crossed a hard-reset boundary.  Do not spend both 3 s TLS windows
           * discovering that indirectly: let the activation loop reset now
           * and run the complete pre-TLS sequence with the restored key. */
          if (repair_requested)
            {
              *reset_required = TRUE;
              fp_warn ("gdix51c0: verified PSK restoration requires a hard "
                       "reset before TLS");
              return TRUE;
            }
        }
    }

  /* Diagnostic: probe whether the sensor exposes a recoverable WB.  On known
   * hardware 0xbb010003 is write-only, so this reports "empty" — it is kept for
   * generality and to surface the register's readability in the log. */
  {
    g_autoptr(GError) rec_err = NULL;
    if (!gdix51c0_recover_psk_from_mcu (&bus, mcu_psk, &mcu_present, &rec_err))
      fp_info ("gdix51c0: sensor PSK recovery unavailable: %s",
               rec_err ? rec_err->message : "?");
  }

  /* Select the PSK for this session: just-provisioned > env/persisted >
   * (rarely) sensor-recovered > error. */
  if (psk_ready)
    {
      /* provisioned above */
    }
  else if (have_existing)
    {
      memcpy (psk, existing, sizeof (psk));
    }
  else if (mcu_present)
    {
      memcpy (psk, mcu_psk, sizeof (psk));
      fp_info ("gdix51c0: using PSK recovered from the sensor (0xbb010003)");
    }
  else if (!gdix51c0_load_psk (psk, error))   /* emits the "no PSK" error */
    {
      return FALSE;
    }

  /* Acceptance-only: make the host use a wrong key for one activation.  This
   * exercises the real OpenSSL Finished-record classifier without changing
   * the persisted key or the sensor before the bounded recovery path runs. */
#ifdef GOODIX_SPI_DEVELOPER
  if (gdix51c0_arm_psk_mismatch_once ())
    {
      psk[0] ^= 0x01;
      fp_warn ("gdix51c0: armed one-shot TLS PSK mismatch injection "
               "(host key altered in memory only)");
    }
#endif

  if (!gdix51c0_tls_init (&self->tls, FP_DEVICE (self), self->spi_fd,
                       self->irq_req, self->irq_events, self->irq_offset,
                       psk, sizeof (psk), error))
    return FALSE;

  {
    g_autoptr(GError) handshake_error = NULL;
    gboolean handshake_ok =
      gdix51c0_tls_handshake (&self->tls, &handshake_error);

    if (!handshake_ok)
      {
        gboolean auth_mismatch =
          g_error_matches (handshake_error,
                           G_IO_ERROR,
                           G_IO_ERROR_PERMISSION_DENIED);

        gdix51c0_tls_free (&self->tls);

        if (auth_mismatch && prov && *prov && have_existing &&
            !self->psk_reprovision_attempted_this_open)
          {
            self->psk_reprovision_pending = TRUE;
            fp_warn ("gdix51c0: confirmed stale TLS PSK; scheduling one "
                     "sensor reprovision after hard reset");
          }
        else if (auth_mismatch && (!prov || !*prov))
          {
            fp_warn ("gdix51c0: stale TLS PSK detected, but automatic repair "
                     "is disabled; set %s=random to authorize sensor reprovision",
                     GDIX51C0_ENV_PROVISION_PSK);
          }

        g_propagate_error (error, g_steal_pointer (&handshake_error));
        return FALSE;
      }
  }

  /* Post-TLS init: cmd 0xd4 (ack-only).  This is the MCU's
   * TLS_SUCCESSFULLY_ESTABLISHED transition, not an optional cleanup packet.
   * A TLS alert can occupy the expected ACK slot after a failed/rapid reopen;
   * accepting that as a usable session leaves every later command timing out.
   * Fail this activation attempt so the outer recovery performs a hard reset
   * and complete bring-up, matching the official state boundary. */
  g_usleep (100000);  /* python: manual_sleep(0.1) before cmd 0xd4 */

  static const guint8 d4_payload[] = {
    0xd4, 0x03, 0x00, 0x00, 0x00, 0xd3
  };

  {
    g_autoptr(GError) d4_error = NULL;

    if (!gdix51c0_cmd_ack (&bus,
                           d4_payload,
                           sizeof (d4_payload),
                           "post-tls-d4",
                           &d4_error))
      {
        g_prefix_error (&d4_error,
                        "gdix51c0: mandatory post-TLS D4 failed: ");
        gdix51c0_tls_free (&self->tls);
        g_propagate_error (error, g_steal_pointer (&d4_error));
        return FALSE;
      }
  }

  self->tls_ready = TRUE;
  fp_dbg ("gdix51c0: TLS ready; capture listener not started yet");
  return TRUE;
}

/* Replicate Windows' gf_update_all_base init calibration before the first
 * finger capture. Windows runs this after bring-up:
 *
 *   cmd 0x36 (FDT base) -> cmd 0x50 (nav base) -> cmd 0x36 (FDT base)
 *   -> cmd 0x20 (T0 background image) -> cmd 0x36 (FDT base)
 *
 * Image and FDT capture require the asynchronous listener. Keep this boundary
 * after listener startup; issuing T0 from session_tls_start used to fail on a
 * fresh installation with "image capture called before listener up". */
static gboolean
gdix51c0_session_prime_t0 (FpiDeviceGdix51c0 *self,
                           GError           **error)
{
  const char *prime = GDIX51C0_DEV_ENV ("GDIX51C0_PRIME_T0");
  const gboolean want_prime =
    prime ? (g_ascii_strtoll (prime, NULL, 0) != 0)
          : (!self->image_base_refreshed_this_open ||
             !self->t0_baseline_valid ||
             self->active_dac_main != 0x0b38);
  g_autoptr(GError) e1 = NULL, e2 = NULL, e3 = NULL, e4 = NULL, e5 = NULL;
  g_autofree guint16 *t0 = NULL;
  gboolean fdt1_finger_off = FALSE;
  gboolean fdt2_finger_off = FALSE;
  gboolean fdt3_finger_off = FALSE;

  if (!want_prime)
    return TRUE;

  gdix51c0_fdt_measure_base (self, &fdt1_finger_off, &e1); /* 0x36 */
  gdix51c0_capture_nav_data (self, &e2);                  /* 0x50 */
  gdix51c0_fdt_measure_base (self, &fdt2_finger_off, &e3); /* 0x36 */
  t0 = gdix51c0_capture_image_raw (self,
                                   GDIX51C0_CMD_IMAGE_T0,
                                   FALSE,
                                   &e4);                  /* 0x20 */
  gdix51c0_fdt_measure_base (self, &fdt3_finger_off, &e5); /* 0x36 */

  /* gf_update_all_base refreshes the persisted no-finger frame once for a
   * cold driver instance even when goodix.dat was valid.  Do the same once
   * per opened libfprint handle, then retain it across ordinary TLS restarts.
   * gdix51c0_baseline_adopt_and_save also performs EngineAdapter's effective
   * NeedUpdateImageBase transition by rebuilding and persisting Chicago state. */
  if (t0 && !self->image_base_refreshed_this_open &&
      fdt2_finger_off && fdt3_finger_off)
    {
      gdix51c0_baseline_adopt_and_save (self, t0);
      self->image_base_refreshed_this_open = TRUE;
    }
  else if (t0 && !self->image_base_refreshed_this_open)
    fp_warn ("gdix51c0: refusing cold ImageBase refresh without confirmed finger-off state "
             "(fdt1=%u fdt2=%u fdt3=%u); deferring to post-lift capture",
             fdt1_finger_off, fdt2_finger_off, fdt3_finger_off);

  fp_info ("gdix51c0: init calibration pass done "
           "(fdt1=%s nav=%s fdt2=%s T0=%s fdt3=%s)",
           e1 ? e1->message : "ok",
           e2 ? e2->message : "ok",
           e3 ? e3->message : "ok",
           t0 ? "ok" : (e4 ? e4->message : "failed"),
           e5 ? e5->message : "ok");

  /* A requested T0 capture is a stateful TLS/image transaction.  If both
   * official command attempts were ACKed but no image record arrived, the MCU
   * can remain in its image path and later acknowledge FDT-down without ever
   * producing a finger event (a late 0xda status packet is observed instead).
   * A persisted ImageBase makes the calibration data usable, but it does not
   * make that live MCU session safe to reuse.  Fail activation so its existing
   * recovery boundary tears down TLS, hard-resets, and repeats full bring-up. */
  if (!t0)
    {
      if (e4)
        g_propagate_error (error, g_steal_pointer (&e4));
      else
        g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_CONNECTION_CLOSED,
                             "gdix51c0: T0 capture failed; live session state is unknown");
      return FALSE;
    }

  if (!self->t0_baseline_valid)
    {
      if (!prime || g_ascii_strtoll (prime, NULL, 0) != 0)
        g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_NOT_INITIALIZED,
                             "gdix51c0: T0 ImageBase capture did not produce a frame");
      else
        g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_NOT_INITIALIZED,
                             "gdix51c0: T0 priming is disabled but no ImageBase exists");
      return FALSE;
    }

  return TRUE;
}

static void
gdix51c0_session_reset_capture_state (FpiDeviceGdix51c0 *self)
{
  self->session_desynced = FALSE;
  self->fdt_down_sample_valid = FALSE;
  self->require_lift_gap = FALSE;
  /* DO NOT reset the ImageBase lifecycle here.  Windows refreshes it on a
   * cold driver/engine boundary, not on every capture or TLS restart. */
  memcpy (self->fdt_down_regs,
          gdix51c0_default_fdt_down_regs,
          sizeof (self->fdt_down_regs));

  /* Finger-DOWN thresholds are baseline-relative; a DAC change shifts the
   * no-finger baseline and can push it above these static thresholds (every
   * zone then reads "touched").  GDIX51C0_FDT_DOWN_BIAS shifts all 6 zone
   * thresholds (the odd bytes) so we can re-fit after a DAC change. */
  gint down_bias = GDIX51C0_DEV_ENV_INT ("GDIX51C0_FDT_DOWN_BIAS", 0);
  if (down_bias != 0)
    for (guint i = 0; i < 6; i++)
      {
        gint t = (gint) self->fdt_down_regs[1 + i * 2] + down_bias;
        self->fdt_down_regs[1 + i * 2] = (guint8) CLAMP (t, 0, 0xff);
      }
}

static void
gdix51c0_baseline_adopt_and_save (FpiDeviceGdix51c0 *self,
                                  const guint16     *raw)
{
  const gchar *calibration_path;

  g_return_if_fail (raw != NULL);

  memcpy (self->t0_baseline, raw, sizeof (self->t0_baseline));
  self->t0_baseline_valid = TRUE;

  guint64 sum = 0;
  guint16 mn = 0xffff;
  guint16 mx = 0;
  for (gsize i = 0; i < GDIX51C0_FRAME_PIXELS; i++)
    {
      guint16 v = self->t0_baseline[i];
      sum += v;
      if (v < mn) mn = v;
      if (v > mx) mx = v;
    }
  fp_dbg ("gdix51c0: dark-frame baseline captured: mean=%u min=%u max=%u",
          (guint) (sum / GDIX51C0_FRAME_PIXELS), mn, mx);

#ifdef GOODIX_SPI_DEVELOPER
  /* Also dump the finger-off frame as a raw 16-bpp .raw (distinct "nofinger"
   * prefix) when GDIX51C0_DUMP_FRAMES is set.  This is the true no-finger
   * reference the Chicago preprocessor uses as its ImageBase/Kr. */
  if (g_getenv ("GDIX51C0_DUMP_FRAMES"))
    {
      const char *dump_dir = g_getenv ("GDIX51C0_DUMP_DIR");
      if (!dump_dir || !*dump_dir)
        dump_dir = "/var/lib/fprint";
      gint64 ts = g_get_monotonic_time ();
      g_autofree char *path =
        g_strdup_printf ("%s/gdix51c0_nofinger_%" G_GINT64_FORMAT ".raw",
                         dump_dir, ts);
      FILE *fp = fopen (path, "wb");
      if (fp)
        {
          fwrite (raw, sizeof (guint16), GDIX51C0_FRAME_PIXELS, fp);
          fclose (fp);
          fp_dbg ("gdix51c0: dumped no-finger frame %s (mean=%u)",
                  path, (guint) (sum / GDIX51C0_FRAME_PIXELS));
        }
    }
#endif

  gdix51c0_baseline_save (self);

  /* EngineAdapter reacts to NeedUpdateImageBase by running
   * preprocessor_init with the refreshed raw ImageBase and then "Save Kr to
   * file". Preserve accumulated gain/count/opaque calibration fields while
   * replacing only its prepared B/ImageBase plane. */
  calibration_path = g_getenv (GDIX51C0_ENV_CHICAGO_CALIBRATION_FILE);
  if (!calibration_path || !*calibration_path)
    calibration_path = GDIX51C0_DEFAULT_CHICAGO_CALIBRATION_FILE;
  if (self->chicago_calibration)
    {
      g_autoptr(GError) error = NULL;
      g_autoptr(GBytes) rebased = NULL;

      rebased = goodix_chicago_calibration_rebase (
        self->chicago_calibration, self->t0_baseline, &error);
      if (!rebased)
        fp_warn ("gdix51c0: could not rebase Chicago calibration on refreshed ImageBase: %s",
                 error ? error->message : "unknown error");
      else
        {
          g_clear_pointer (&self->chicago_calibration, g_bytes_unref);
          self->chicago_calibration = g_steal_pointer (&rebased);
          if (!goodix_chicago_calibration_save (
                calibration_path,
                self->sensor_id,
                self->chicago_calibration,
                &error))
            fp_warn ("gdix51c0: rebased Chicago state is usable in memory but could not be persisted to %s: %s",
                     calibration_path,
                     error ? error->message : "unknown error");
          else
            fp_info ("gdix51c0: NeedUpdateImageBase rebased and persisted Chicago state");
        }
    }

  gdix51c0_chicago_calibration_maybe_load_or_generate (self);
  g_clear_pointer (&self->chicago_preprocessor,
                   goodix_chicago_preprocessor_free);
  gdix51c0_chicago_preprocessor_maybe_init (self);
}

/* Best-effort: capture a finger-off baseline (cmd 0x20) and persist it to
 * disk. Called only when wait_for_lift confirms touchflag=0. The normal
 * first-run path adopts the equivalent T0 frame from session priming before
 * asking for a finger; this path remains for forced RE recaptures. */
static void
gdix51c0_baseline_capture_and_save (FpiDeviceGdix51c0 *self)
{
  g_autoptr(GError) error = NULL;
  g_autofree guint16 *raw = NULL;

  raw = gdix51c0_capture_image_raw (self, GDIX51C0_CMD_IMAGE_T0, FALSE, &error);
  if (!raw)
    {
      fp_warn ("gdix51c0: dark-frame capture failed: %s",
               error ? error->message : "?");
      return;
    }

  gdix51c0_baseline_adopt_and_save (self, raw);
  self->image_base_refreshed_this_open = TRUE;
}

static void
gdix51c0_baseline_load (FpiDeviceGdix51c0 *self)
{
  g_autoptr(GError) error = NULL;
  g_autofree gchar *contents = NULL;
  gsize length = 0;

  if (!g_file_get_contents (GDIX51C0_BASELINE_PATH, &contents, &length, &error))
    {
      fp_dbg ("gdix51c0: no persisted baseline at %s (%s) — will capture on first lift",
              GDIX51C0_BASELINE_PATH,
              error ? error->message : "?");
      return;
    }

  const gsize expected =
    sizeof (guint32) + 1 + 3 + sizeof (gint64) +
    sizeof (self->sensor_id) +
    GDIX51C0_FRAME_PIXELS * sizeof (guint16);
  if (length != expected)
    {
      fp_warn ("gdix51c0: baseline file %s has unexpected size %zu (want %zu) — ignoring",
               GDIX51C0_BASELINE_PATH, length, expected);
      return;
    }

  guint32 magic;
  memcpy (&magic, contents, sizeof (magic));
  if (magic != GDIX51C0_BASELINE_MAGIC)
    {
      fp_warn ("gdix51c0: baseline file %s magic mismatch — ignoring",
               GDIX51C0_BASELINE_PATH);
      return;
    }

  guint8 version = (guint8) contents[4];
  if (version != GDIX51C0_BASELINE_VERSION)
    {
      fp_warn ("gdix51c0: baseline file %s version %u != %u — ignoring",
               GDIX51C0_BASELINE_PATH, version, GDIX51C0_BASELINE_VERSION);
      return;
    }

  if (!self->sensor_id_valid)
    {
      fp_warn ("gdix51c0: cannot load ImageBase before reading sensor id");
      return;
    }

  if (memcmp (contents + 16, self->sensor_id, sizeof (self->sensor_id)) != 0)
    {
      fp_warn ("gdix51c0: persisted ImageBase belongs to another sensor; ignoring it");
      return;
    }

  gint64 captured_at;
  memcpy (&captured_at, contents + 8, sizeof (captured_at));

  memcpy (self->t0_baseline,
          contents + 16 + sizeof (self->sensor_id),
          GDIX51C0_FRAME_PIXELS * sizeof (guint16));
  self->t0_baseline_valid = TRUE;

  fp_info ("gdix51c0: loaded persisted Chicago ImageBase (captured at %" G_GINT64_FORMAT ")",
           captured_at);
}

static void
gdix51c0_chicago_calibration_maybe_load_or_generate (
  FpiDeviceGdix51c0 *self)
{
  const gchar *path;
  g_autoptr(GError) error = NULL;

  if (self->chicago_calibration || !self->sensor_id_valid)
    return;

  path = g_getenv (GDIX51C0_ENV_CHICAGO_CALIBRATION_FILE);
  if (!path || !*path)
    path = GDIX51C0_DEFAULT_CHICAGO_CALIBRATION_FILE;

  self->chicago_calibration =
    goodix_chicago_calibration_load (path, self->sensor_id, &error);
  if (self->chicago_calibration)
    {
      fp_info ("gdix51c0: loaded %zu-byte sensor-bound Chicago calibration",
               g_bytes_get_size (self->chicago_calibration));
      return;
    }

  if (!g_error_matches (error, G_FILE_ERROR, G_FILE_ERROR_NOENT) &&
      !g_error_matches (error, G_IO_ERROR, G_IO_ERROR_NOT_FOUND))
    {
      fp_warn ("gdix51c0: refusing to replace invalid Chicago calibration %s: %s",
               path, error ? error->message : "unknown error");
      return;
    }
  if (!self->t0_baseline_valid)
    {
      fp_dbg ("gdix51c0: no Chicago calibration at %s; waiting for T0 ImageBase",
              path);
      return;
    }

  g_clear_error (&error);
  self->chicago_calibration =
    goodix_chicago_calibration_generate (self->t0_baseline);
  if (!self->chicago_calibration)
    {
      fp_warn ("gdix51c0: could not generate native Chicago calibration");
      return;
    }

  if (!goodix_chicago_calibration_save (path,
                                         self->sensor_id,
                                         self->chicago_calibration,
                                         &error))
    fp_warn ("gdix51c0: generated calibration is usable in memory but could not be persisted to %s: %s",
             path, error ? error->message : "unknown error");
  else
    fp_info ("gdix51c0: generated and persisted first-run Chicago calibration at %s",
             path);
}

static void
gdix51c0_chicago_preprocessor_maybe_init (FpiDeviceGdix51c0 *self)
{
  g_autoptr(GError) error = NULL;

  if (self->chicago_preprocessor || !self->chicago_calibration ||
      !self->t0_baseline_valid)
    return;

  self->chicago_preprocessor =
    goodix_chicago_preprocessor_new (self->chicago_calibration,
                                        self->t0_baseline,
                                        &error);
  if (!self->chicago_preprocessor)
    {
      fp_warn ("gdix51c0: could not initialize native Chicago input state: %s",
               error ? error->message : "unknown error");
      return;
    }

  fp_info ("gdix51c0: native Chicago input state ready (calibration maps + ImageBase)");
}

static void
gdix51c0_baseline_save (FpiDeviceGdix51c0 *self)
{
  g_autoptr(GError) error = NULL;
  const gsize total =
    sizeof (guint32) + 1 + 3 + sizeof (gint64) +
    sizeof (self->sensor_id) +
    GDIX51C0_FRAME_PIXELS * sizeof (guint16);
  g_autofree gchar *buf = g_malloc0 (total);

  if (!self->sensor_id_valid)
    {
      fp_warn ("gdix51c0: refusing to persist ImageBase without sensor id");
      return;
    }

  guint32 magic = GDIX51C0_BASELINE_MAGIC;
  memcpy (buf, &magic, sizeof (magic));
  buf[4] = (gchar) GDIX51C0_BASELINE_VERSION;

  gint64 now = g_get_real_time ();
  memcpy (buf + 8, &now, sizeof (now));

  memcpy (buf + 16, self->sensor_id, sizeof (self->sensor_id));
  memcpy (buf + 16 + sizeof (self->sensor_id),
          self->t0_baseline,
          GDIX51C0_FRAME_PIXELS * sizeof (guint16));

  if (!g_file_set_contents (GDIX51C0_BASELINE_PATH, buf, total, &error))
    {
      fp_warn ("gdix51c0: failed to persist baseline to %s: %s",
               GDIX51C0_BASELINE_PATH,
               error ? error->message : "?");
      return;
    }

  fp_info ("gdix51c0: persisted Chicago ImageBase to %s",
           GDIX51C0_BASELINE_PATH);
}

static gboolean
gdix51c0_session_activate_once (FpiDeviceGdix51c0 *self, GError **error)
{
  gboolean reset_required = FALSE;

  G_DEBUG_HERE ();

  gdix51c0_session_reset_capture_state (self);

  if (!gdix51c0_boot_probe (self, error))
    return FALSE;

  if (!gdix51c0_init_sequence (self, error))
    return FALSE;

  if (!gdix51c0_session_tls_start (self, &reset_required, error))
    return FALSE;

  if (reset_required)
    {
      g_set_error_literal (error,
                           G_IO_ERROR,
                           G_IO_ERROR_BUSY,
                           "gdix51c0: PSK restoration completed; hard-reset "
                           "activation boundary required");
      return FALSE;
    }

  /* Bring the async listener up only after init+TLS finished synchronously.
   * From here until session_deactivate, every SPI read and IRQ wait goes
   * through the listener — no other code path may touch irq_req or call
   * gdix51c0_spi_read directly. */
  g_assert (self->listener == NULL);
  gdix51c0_irq_drain (self->irq_req, self->irq_events);
  self->listener = gdix51c0_listener_new (FP_DEVICE (self),
                                          self->spi_fd,
                                          self->irq_req,
                                          self->irq_offset,
                                          self->irq_events,
                                          error);
  if (!self->listener)
    return FALSE;

  if (!gdix51c0_session_prime_t0 (self, error))
    return FALSE;

  return TRUE;
}

static gboolean
gdix51c0_session_activate_with_attempts (FpiDeviceGdix51c0 *self,
                                         guint              max_attempts,
                                         GError           **error)
{
  guint failures = 0;
  gboolean repair_budget_granted = FALSE;

  max_attempts = MAX (max_attempts, 1);

  if (self->tls_ready)
    {
      gdix51c0_session_reset_capture_state (self);
      fp_dbg ("gdix51c0: reusing active TLS session");
      if (!self->image_base_refreshed_this_open &&
          !gdix51c0_session_prime_t0 (self, error))
        return FALSE;
      return TRUE;
    }

  while (failures < max_attempts)
    {
      g_autoptr(GError) local_error = NULL;
      gboolean repair_completed_before =
        self->psk_reprovision_completed_this_open;
      gboolean repair_completed_now;
      gboolean started_fresh_budget = FALSE;

      if (gdix51c0_action_check_cancelled (self, error))
        return FALSE;

      if (gdix51c0_session_activate_once (self, &local_error))
        return TRUE;

      repair_completed_now =
        !repair_completed_before &&
        self->psk_reprovision_completed_this_open;

      gdix51c0_session_deactivate (self);

      if (gdix51c0_action_check_cancelled (self, error))
        return FALSE;

      if (repair_completed_now && !repair_budget_granted)
        {
          /* A verified stale-key rewrite is a new sensor-authentication epoch,
           * not an ordinary transport retry. session_tls_start deliberately
           * stopped before TLS; session_deactivate has now hard-reset the MCU.
           * Give the restored sensor one fresh bounded activation budget and
           * continue immediately instead of adding the ordinary retry delay. */
          repair_budget_granted = TRUE;
          failures = 0;
          started_fresh_budget = TRUE;
          fp_warn ("gdix51c0: PSK restoration reset boundary complete; "
                   "starting one fresh %u-attempt recovery budget",
                   max_attempts);
        }
      else
        {
          failures++;
        }

      if (failures == max_attempts)
        {
          g_propagate_error (error, g_steal_pointer (&local_error));
          return FALSE;
        }

      if (!started_fresh_budget)
        fp_warn ("gdix51c0: activation failed on attempt %u/%u (%s); "
                 "resetting and retrying",
                 failures,
                 max_attempts,
                 local_error ? local_error->message : "?");

      if (started_fresh_budget)
        continue;

      for (guint slept_usec = 0; slept_usec < 1000000; slept_usec += 100000)
        {
          if (gdix51c0_action_check_cancelled (self, error))
            return FALSE;

          g_usleep (100000);
        }
    }

  g_set_error_literal (error,
                       G_IO_ERROR,
                       G_IO_ERROR_FAILED,
                       "gdix51c0: activation failed unexpectedly");
  return FALSE;
}

static gboolean
gdix51c0_session_activate (FpiDeviceGdix51c0 *self, GError **error)
{
  return gdix51c0_session_activate_with_attempts (self, 4, error);
}

static void
gdix51c0_session_deactivate (FpiDeviceGdix51c0 *self)
{
  G_DEBUG_HERE ();

  /* Stop the async listener BEFORE touching TLS / reset / SPI directly: the
   * listener's reader thread may currently hold the SPI mutex and we must not
   * race it for the IRQ line. */
  g_clear_pointer (&self->listener, gdix51c0_listener_free);

  if (self->tls_ready)
    {
      gdix51c0_tls_free (&self->tls);
      self->tls_ready = FALSE;
    }

  self->session_desynced = FALSE;

  gdix51c0_irq_drain (self->irq_req, self->irq_events);

  if (self->gpio_chip)
    {
      g_autoptr(GError) error = NULL;

      if (!gdix51c0_reset_pulse (self->gpio_chip,
                                 self->reset_offset,
                                 &error))
        {
          fp_dbg ("gdix51c0: reset during session deactivate failed: %s",
                  error ? error->message : "?");
        }

      /*
       * Important: 250ms is sometimes not enough. Logs show the next
       * activation can miss get-mcu-state or TLS ClientHello.
       */
      g_usleep (1000000);

      gdix51c0_irq_drain (self->irq_req, self->irq_events);
    }
}


/* ------------------------------------------------------------------ */
/* Image decode: 6 packed bytes -> 4 12-bit samples (kept as guint16   */
/* for downstream diff/normalization).  Mirrors image_decode.py.       */
/* ------------------------------------------------------------------ */
static void
gdix51c0_decode_12bpp_to_16bpp (const guint8 *packed, gsize packed_len,
                             guint16 *out16, gsize out_pixels)
{
  g_autofree guint16 *stream = g_new (guint16, out_pixels);
  gsize decoded;

  memset (out16, 0, out_pixels * sizeof (guint16));
  decoded = goodix_raw12_decode (packed, packed_len, stream, out_pixels);
  for (gsize n = 0; n < decoded; n++)
    {
      /* Goodix raw12 unpacking is common. ChicagoHUDataRegroup is not: the
       * Milan stream uses a 64-sample fast axis, so n maps into a 64-row by
       * 80-column matcher image. */
      gsize dst = (n % GDIX51C0_SENSOR_WIDTH) * GDIX51C0_SENSOR_HEIGHT
                  + (n / GDIX51C0_SENSOR_WIDTH);

      if (dst < out_pixels)
        out16[dst] = stream[n];
    }
}

#ifdef GOODIX_SPI_DEVELOPER
static void
gdix51c0_dump_image_blob (const char   *tag,
                          guint8        cmd,
                          const guint8 *data,
                          gsize         len)
{
  if (!g_getenv ("GDIX51C0_DUMP_FRAMES") || !data)
    return;

  gint64 ts = g_get_monotonic_time ();
  g_autofree char *path =
    g_strdup_printf ("/tmp/gdix51c0_%s_cmd%02x_%" G_GINT64_FORMAT ".bin",
                     tag, cmd, ts);

  if (!g_file_set_contents (path, (const char *) data, len, NULL))
    {
      fp_dbg ("gdix51c0: failed to dump %s image blob %s", tag, path);
      return;
    }

  fp_dbg ("gdix51c0: dumped %s image blob %s (%zu B)", tag, path, len);
}

static void
gdix51c0_log_raw_image_stats (const guint16 *raw)
{
  guint16 mn = 0xffff, mx = 0;
  guint clipped = 0;
  guint64 sum = 0;

  for (gsize i = 0; i < GDIX51C0_FRAME_PIXELS; i++)
    {
      guint16 v = raw[i];

      mn = MIN (mn, v);
      mx = MAX (mx, v);
      sum += v;
      if (v < 16 || v > 4080)
        clipped++;
    }

  fp_dbg ("gdix51c0: raw image stats min=%u max=%u mean=%u clipped=%u/%u",
          mn,
          mx,
          (guint) (sum / GDIX51C0_FRAME_PIXELS),
          clipped,
          GDIX51C0_FRAME_PIXELS);
}
#endif

/* Recompute the six finger-down thresholds from the live no-finger baseline:
 * threshold_hi = baseline/2 (Windows' gfFDTDownbase rule, verified against
 * fdt_base1), low byte fixed at 0x80.  This is what makes finger detection
 * DAC-independent.  GDIX51C0_FDT_DYNAMIC=0 forces the historical static table;
 * GDIX51C0_FDT_DOWN_BIAS nudges every threshold if the trip margin needs a
 * tweak.  Until a zero-touch baseline is seen we keep the static bootstrap. */
static void
gdix51c0_fdt_update_down_thresholds (FpiDeviceGdix51c0 *self)
{
  if (GDIX51C0_DEV_ENV_INT ("GDIX51C0_FDT_DYNAMIC", 1) == 0)
    return;
  if (!self->fdt_baseline_valid)
    return;

  gint bias = GDIX51C0_DEV_ENV_INT ("GDIX51C0_FDT_DOWN_BIAS", 0);
  for (guint i = 0; i < 6; i++)
    {
      gint thr = (gint) (self->fdt_baseline[i] / 2) + bias;
      self->fdt_down_regs[i * 2]     = 0x80;
      self->fdt_down_regs[i * 2 + 1] = (guint8) CLAMP (thr, 1, 0xff);
    }
}

static void
gdix51c0_fdt_store_zero_touch_baseline (FpiDeviceGdix51c0 *self,
                                        const guint16      sample[6],
                                        const char        *source,
                                        gboolean           announce)
{
  gboolean was_valid = self->fdt_baseline_valid;

  memcpy (self->fdt_baseline, sample, sizeof (self->fdt_baseline));
  self->fdt_baseline_valid = TRUE;

  if (announce || !was_valid)
    fp_dbg ("gdix51c0: FDT zero-touch base updated from %s "
            "(thresholds recalculated)",
            source);
}

static gboolean
gdix51c0_fdt_down_arm (FpiDeviceGdix51c0 *self,
                       GError           **error)
{
  guint8 fdt_data[2 + 12 + 2];
  gsize fdt_pkt_len = 0;
  g_autofree guint8 *fdt_packet = NULL;

  /* Windows ChicagoHUSetMode(Type 1): FDT-down mode byte 0x08, subcmd 1. */
  fdt_data[0] = 0x08;
  fdt_data[1] = 0x01;
  gdix51c0_fdt_update_down_thresholds (self);
  memcpy (&fdt_data[2], self->fdt_down_regs, sizeof (self->fdt_down_regs));

  /* Do not use a constant timestamp. */
  guint16 ts = (guint16) (g_get_monotonic_time () & 0xffff);
  fdt_data[14] = ts & 0xff;
  fdt_data[15] = ts >> 8;

  fdt_packet =
    gdix51c0_make_payload_packet (GDIX51C0_CMD_FDT_DOWN,
                                  fdt_data, sizeof (fdt_data),
                                  &fdt_pkt_len);

  if (G_UNLIKELY (!self->listener))
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_NOT_INITIALIZED,
                           "gdix51c0: FDT-down arm called before listener up");
      return FALSE;
    }

  /* Match the original 80 ms quiescence the MCU expects between FDT
   * arms.  The listener consumes any leftover packets while we sleep. */
  g_usleep (80000);

  /* Drop a stale finger packet before arming.  ACK draining and the exact
   * 1 ms / 1000 ms / one-resend policy live in the shared transport helper. */
  gdix51c0_listener_drain_cmd (self->listener, GDIX51C0_CMD_FDT_DOWN);
  return gdix51c0_listener_send_ack (self->listener,
                                     fdt_packet, fdt_pkt_len,
                                     "FDT-down", error);
}

static gboolean
gdix51c0_fdt_down_read (FpiDeviceGdix51c0 *self,
                        guint              finger_timeout_usec,
                        guint8            *touchflag,
                        GError           **error)
{
  *touchflag = 0;

  if (G_UNLIKELY (!self->listener))
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_NOT_INITIALIZED,
                           "gdix51c0: FDT-down read called before listener up");
      return FALSE;
    }

  {
    gsize n = 0;
    g_autofree guint8 *fdt_resp =
      gdix51c0_listener_await (self->listener, GDIX51C0_CMD_FDT_DOWN,
                               finger_timeout_usec, &n, error);

    if (!fdt_resp)
      return FALSE;

#ifdef GOODIX_SPI_DEVELOPER
    {
      guint16 inner_len = n >= 3 ? ((guint16) fdt_resp[1] | ((guint16) fdt_resp[2] << 8)) : 0;
      gboolean checksum_ok = n > 0 && gdix51c0_payload_checksum (fdt_resp, n - 1) == fdt_resp[n - 1];
      GString *s = g_string_sized_new (n * 3 + 1);

      for (gsize i = 0; i < n; i++)
        g_string_append_printf (s, "%02x%s", fdt_resp[i], i + 1 < n ? " " : "");

      fp_dbg ("gdix51c0: FDT response len=%zu cmd=0x%02x inner_len=%u checksum=%s data=%s",
              n,
              n > 0 ? fdt_resp[0] : 0,
              inner_len,
              checksum_ok ? "ok" : "bad",
              s->str);
      g_string_free (s, TRUE);
    }
#endif

    /* The listener filters by cmd byte so payload[0] is guaranteed to be
     * GDIX51C0_CMD_FDT_DOWN here.  Length-validate the rest before parse. */
    if (n < 17)
      {
        fp_dbg ("gdix51c0: ignoring short FDT-down response: len=%zu", n);
        *touchflag = GDIX51C0_TOUCH_IGNORE;
      }
    else
      {
        *touchflag = (n > 5) ? fdt_resp[5] : 0;

        self->fdt_down_sample_valid = FALSE;
        if (n >= 20)
          {
            for (guint i = 0; i < 6; i++)
              self->fdt_down_sample[i] = (guint16) fdt_resp[7 + i * 2] |
                                         ((guint16) fdt_resp[8 + i * 2] << 8);

            self->fdt_down_sample_valid = TRUE;

            /* Refresh the no-finger baseline from every clean (zero-touch)
             * reading so the dynamic finger-down threshold tracks the current
             * DAC and any slow drift.  Only touch-free frames are baseline. */
            if ((*touchflag & 0x3f) == 0)
              gdix51c0_fdt_store_zero_touch_baseline (self,
                                                      self->fdt_down_sample,
                                                      "FDT-down",
                                                      FALSE);
          }
      }

    /* Zero-touch samples are the only safe source for the no-finger base. */
  }

  return TRUE;
}

static gboolean
gdix51c0_parse_fdt_up_response (FpiDeviceGdix51c0 *self,
                                const guint8      *fdt_resp,
                                gsize              n,
                                guint8            *touchflag,
                                GError           **error)
{
  if (n < 17 || fdt_resp[0] != GDIX51C0_CMD_FDT_UP)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                   "gdix51c0: invalid FDT-up response len=%zu cmd=0x%02x",
                   n, n > 0 ? fdt_resp[0] : 0);
      return FALSE;
    }

  *touchflag = (n > 5) ? fdt_resp[5] : 0xff;

  if (n >= 20)
    {
      guint16 sample[6];

      for (guint i = 0; i < 6; i++)
        sample[i] = (guint16) fdt_resp[7 + i * 2] |
                    ((guint16) fdt_resp[8 + i * 2] << 8);

      if ((*touchflag & 0x3f) == 0)
        gdix51c0_fdt_store_zero_touch_baseline (self, sample, "FDT-up", TRUE);
      else
        {
          memcpy (self->fdt_down_sample, sample, sizeof (self->fdt_down_sample));
          self->fdt_down_sample_valid = TRUE;
        }
    }

#ifdef GOODIX_SPI_DEVELOPER
  {
    GString *s = g_string_sized_new (n * 3 + 1);
    for (gsize i = 0; i < n; i++)
      g_string_append_printf (s, "%02x%s", fdt_resp[i], i + 1 < n ? " " : "");
    fp_dbg ("gdix51c0: FDT-up response len=%zu touchflag=0x%02x data=%s",
            n, *touchflag, s->str);
    g_string_free (s, TRUE);
  }
#endif
  return TRUE;
}

static gboolean
gdix51c0_fdt_up_arm (FpiDeviceGdix51c0 *self,
                     GError           **error)
{
  guint8 fdt_data[2 + 12];
  gsize fdt_pkt_len = 0;
  g_autofree guint8 *fdt_packet = NULL;

  /* Windows ChicagoHUSetMode(Type 2): FDT-up mode byte 0x0a, subcmd 2. */
  fdt_data[0] = 0x0a;
  fdt_data[1] = 0x02;
  if (!self->fdt_down_sample_valid)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: FDT-up requested without FDT-down sample");
      return FALSE;
    }

  /* Windows' gfFDTUPbase rule is threshold_hi = touched_sample/2 + 29,
   * low byte fixed at 0x80.  Keep env overrides for experiments, but the
   * default must match the official driver; the previous midpoint formula
   * made FDT-up semantics diverge after the DAC change. */
  gboolean manual = GDIX51C0_DEV_ENV ("GDIX51C0_FDT_UP_NUM") ||
                    GDIX51C0_DEV_ENV ("GDIX51C0_FDT_UP_DEN") ||
                    GDIX51C0_DEV_ENV ("GDIX51C0_FDT_UP_OFF");
  guint num = (guint) GDIX51C0_DEV_ENV_INT ("GDIX51C0_FDT_UP_NUM", 1);
  guint den = (guint) GDIX51C0_DEV_ENV_INT ("GDIX51C0_FDT_UP_DEN", 2);
  gint  off = GDIX51C0_DEV_ENV_INT ("GDIX51C0_FDT_UP_OFF",
                                    GDIX51C0_FDT_UP_THRESHOLD_OFFSET);
  if (den == 0) den = 1;
  for (guint i = 0; i < 6; i++)
    {
      gint threshold;
      if (manual)
        threshold = (gint) (self->fdt_down_sample[i] * num / den) + off;
      else
        threshold = (gint) self->fdt_down_sample[i] / 2 +
                    GDIX51C0_FDT_UP_THRESHOLD_OFFSET;
      threshold = CLAMP (threshold, 1, 0xff);

      fdt_data[2 + i * 2] = 0x80;
      fdt_data[3 + i * 2] = (guint8) threshold;
    }

  fdt_packet =
    gdix51c0_make_payload_packet (GDIX51C0_CMD_FDT_UP,
                                  fdt_data, sizeof (fdt_data),
                                  &fdt_pkt_len);

  if (G_UNLIKELY (!self->listener))
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_NOT_INITIALIZED,
                           "gdix51c0: FDT-up arm called before listener up");
      return FALSE;
    }

  /* Drop any stale lift packet from an earlier transaction before arming. */
  gdix51c0_listener_drain_cmd (self->listener, GDIX51C0_CMD_FDT_UP);
  return gdix51c0_listener_send_ack (self->listener,
                                     fdt_packet, fdt_pkt_len,
                                     "FDT-up", error);
}

static gboolean
gdix51c0_fdt_up_read (FpiDeviceGdix51c0 *self,
                      guint              lift_timeout_usec,
                      guint8            *touchflag,
                      GError           **error)
{
  *touchflag = 0xff;

  if (G_UNLIKELY (!self->listener))
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_NOT_INITIALIZED,
                           "gdix51c0: FDT-up read called before listener up");
      return FALSE;
    }

  gsize n = 0;
  g_autofree guint8 *fdt_resp =
    gdix51c0_listener_await (self->listener, GDIX51C0_CMD_FDT_UP,
                             lift_timeout_usec, &n, error);

  if (!fdt_resp)
    return FALSE;

  return gdix51c0_parse_fdt_up_response (self, fdt_resp, n, touchflag, error);
}

static gboolean
gdix51c0_fdt_manual_read (FpiDeviceGdix51c0 *self,
                          guint8            *touchflag,
                          guint16            sample[6],
                          GError           **error)
{
  guint8 fdt_data[2 + 12];
  gsize pkt_len = 0;
  g_autofree guint8 *packet = NULL;

  *touchflag = 0xff;

  if (G_UNLIKELY (!self->listener))
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_NOT_INITIALIZED,
                           "gdix51c0: FDT manual read before listener up");
      return FALSE;
    }

  /* Windows ChicagoHUSetMode(Type 3): FDT-manual mode byte 0x09, subcmd 3. */
  fdt_data[0] = 0x09;
  fdt_data[1] = 0x03;
  memcpy (&fdt_data[2], self->fdt_down_regs, sizeof (self->fdt_down_regs));

  packet = gdix51c0_make_payload_packet (GDIX51C0_CMD_FDT_MANUAL,
                                         fdt_data, sizeof (fdt_data), &pkt_len);

  gsize n = 0;
  g_autofree guint8 *resp =
    gdix51c0_listener_command (self->listener, packet, pkt_len,
                               GDIX51C0_CMD_FDT_MANUAL,
                               GDIX51C0_FDT_MANUAL_TIMEOUT_USEC,
                               &n, "FDT-manual", error);
  if (!resp)
    return FALSE;

  if (n < 20)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                   "gdix51c0: short FDT manual response len=%zu", n);
      return FALSE;
    }

  *touchflag = (n > 5) ? resp[5] : 0xff;
  for (guint i = 0; i < 6; i++)
    sample[i] = (guint16) resp[7 + i * 2] |
                ((guint16) resp[8 + i * 2] << 8);

  fp_dbg ("gdix51c0: FDT-manual response touchflag=0x%02x zones=%u "
          "sample=[%u %u %u %u %u %u]",
          *touchflag, gdix51c0_fdt_touch_count (*touchflag),
          sample[0], sample[1], sample[2], sample[3], sample[4], sample[5]);
  return TRUE;
}

/* Probe FDT state on demand via cmd 0x36 (FDT_MANUAL, sub-cmd 3).  Windows
 * uses this as a branch point: touchflag==0 feeds gfFDTDownbase; touchflag
 * with enough zones feeds gfFDTUPbase. */
static gboolean
gdix51c0_fdt_measure_base (FpiDeviceGdix51c0 *self,
                           gboolean          *finger_off,
                           GError           **error)
{
  guint8 touchflag = 0xff;
  guint16 sample[6];

  if (finger_off)
    *finger_off = FALSE;

  if (!gdix51c0_fdt_manual_read (self, &touchflag, sample, error))
    return FALSE;

  if ((touchflag & 0x3f) != 0)
    {
      memcpy (self->fdt_down_sample, sample, sizeof (self->fdt_down_sample));
      self->fdt_down_sample_valid = TRUE;
      fp_dbg ("gdix51c0: FDT manual saw touchflag=0x%02x; not using it as no-finger base",
              touchflag);
      return TRUE;
    }

  if (finger_off)
    *finger_off = TRUE;
  gdix51c0_fdt_store_zero_touch_baseline (self, sample, "FDT-manual", TRUE);
  return TRUE;
}

typedef struct {
  FpDevice           *dev;
  FpFingerStatusFlags status;
} Gdix51c0FingerStatusEvent;

typedef struct {
  FpDevice *dev;
  gint      completed_stages;
  GError   *error;
} Gdix51c0EnrollProgressEvent;

typedef struct {
  FpDevice *dev;
  FpPrint  *print;
  GVariant *data;
  GError   *error;
} Gdix51c0EnrollCompleteEvent;

typedef struct {
  FpDevice            *dev;
  Gdix51c0ActionKind   kind;
  gboolean             report_result;
  FpiMatchResult       verify_result;
  FpPrint             *identify_match;
  FpPrint             *updated_print;
  GVariant            *updated_data;
  GError              *error;
} Gdix51c0MatchCompleteEvent;

static GMainContext *
gdix51c0_action_context_ref (FpiDeviceGdix51c0 *self)
{
  if (self->action_context)
    return g_main_context_ref (self->action_context);

  return g_main_context_ref (g_main_context_default ());
}

static void
gdix51c0_finger_status_event_free (Gdix51c0FingerStatusEvent *event)
{
  g_object_unref (event->dev);
  g_free (event);
}

static gboolean
gdix51c0_report_finger_status_main (gpointer user_data)
{
  Gdix51c0FingerStatusEvent *event = user_data;

  fpi_device_report_finger_status (event->dev, event->status);
  return G_SOURCE_REMOVE;
}

static void
gdix51c0_report_finger_status_on_main (FpiDeviceGdix51c0   *self,
                                       FpFingerStatusFlags  status)
{
  Gdix51c0FingerStatusEvent *event = g_new0 (Gdix51c0FingerStatusEvent, 1);
  GMainContext *context = gdix51c0_action_context_ref (self);

  event->dev = g_object_ref (FP_DEVICE (self));
  event->status = status;
  g_main_context_invoke_full (context,
                              G_PRIORITY_DEFAULT,
                              gdix51c0_report_finger_status_main,
                              event,
                              (GDestroyNotify) gdix51c0_finger_status_event_free);
  g_main_context_unref (context);
}

static void
gdix51c0_enroll_progress_event_free (Gdix51c0EnrollProgressEvent *event)
{
  g_object_unref (event->dev);
  g_clear_error (&event->error);
  g_free (event);
}

static gboolean
gdix51c0_enroll_progress_main (gpointer user_data)
{
  Gdix51c0EnrollProgressEvent *event = user_data;

  fpi_device_enroll_progress (event->dev,
                              event->completed_stages,
                              NULL,
                              g_steal_pointer (&event->error));
  return G_SOURCE_REMOVE;
}

static void
gdix51c0_enroll_progress_on_main (FpiDeviceGdix51c0 *self,
                                  gint               completed_stages,
                                  GError            *error)
{
  Gdix51c0EnrollProgressEvent *event = g_new0 (Gdix51c0EnrollProgressEvent, 1);
  GMainContext *context = gdix51c0_action_context_ref (self);

  event->dev = g_object_ref (FP_DEVICE (self));
  event->completed_stages = completed_stages;
  event->error = error;
  g_main_context_invoke_full (context,
                              G_PRIORITY_DEFAULT,
                              gdix51c0_enroll_progress_main,
                              event,
                              (GDestroyNotify) gdix51c0_enroll_progress_event_free);
  g_main_context_unref (context);
}

static void
gdix51c0_enroll_complete_event_free (Gdix51c0EnrollCompleteEvent *event)
{
  g_object_unref (event->dev);
  g_clear_object (&event->print);
  g_clear_pointer (&event->data, g_variant_unref);
  g_clear_error (&event->error);
  g_free (event);
}

static gboolean
gdix51c0_enroll_complete_main (gpointer user_data)
{
  Gdix51c0EnrollCompleteEvent *event = user_data;

  if (event->error)
    {
      fpi_device_enroll_complete (event->dev, NULL, g_steal_pointer (&event->error));
      return G_SOURCE_REMOVE;
    }

  if (!event->print || !event->data)
    {
      fpi_device_enroll_complete (event->dev, NULL,
                                  fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                                            "gdix51c0: missing enrollment result"));
      return G_SOURCE_REMOVE;
    }

  fpi_print_set_type (event->print, FPI_PRINT_RAW);
  g_object_set (G_OBJECT (event->print), "fpi-data", event->data, NULL);
  fpi_device_enroll_complete (event->dev, g_object_ref (event->print), NULL);
  return G_SOURCE_REMOVE;
}

static void
gdix51c0_enroll_complete_on_main (FpiDeviceGdix51c0 *self,
                                  FpPrint           *print,
                                  GVariant          *data,
                                  GError            *error)
{
  Gdix51c0EnrollCompleteEvent *event = g_new0 (Gdix51c0EnrollCompleteEvent, 1);
  GMainContext *context = gdix51c0_action_context_ref (self);

  event->dev = g_object_ref (FP_DEVICE (self));
  event->print = print ? g_object_ref (print) : NULL;
  event->data = data;
  event->error = error;
  g_main_context_invoke_full (context,
                              G_PRIORITY_DEFAULT,
                              gdix51c0_enroll_complete_main,
                              event,
                              (GDestroyNotify) gdix51c0_enroll_complete_event_free);
  g_main_context_unref (context);
}

static void
gdix51c0_match_complete_event_free (Gdix51c0MatchCompleteEvent *event)
{
  g_object_unref (event->dev);
  g_clear_object (&event->identify_match);
  g_clear_object (&event->updated_print);
  g_clear_pointer (&event->updated_data, g_variant_unref);
  g_clear_error (&event->error);
  g_free (event);
}

static gboolean
gdix51c0_match_complete_finish_main (gpointer user_data)
{
  Gdix51c0MatchCompleteEvent *event = user_data;

  if (event->kind == GDIX51C0_ACTION_VERIFY)
    fpi_device_verify_complete (event->dev, g_steal_pointer (&event->error));
  else
    fpi_device_identify_complete (event->dev, g_steal_pointer (&event->error));

  return G_SOURCE_REMOVE;
}

static gboolean
gdix51c0_match_complete_main (gpointer user_data)
{
  Gdix51c0MatchCompleteEvent *event = user_data;
  gboolean hold_no_match = FALSE;

  if (event->updated_print && event->updated_data)
    g_object_set (G_OBJECT (event->updated_print),
                  "fpi-data", event->updated_data, NULL);

  if (event->kind == GDIX51C0_ACTION_VERIFY)
    {
      if (!event->error && event->report_result)
        fpi_device_verify_report (event->dev, event->verify_result, NULL, NULL);

      if (!event->error)
        hold_no_match = event->verify_result == FPI_MATCH_FAIL;

    }
  else
    {
      if (!event->error && event->report_result)
        fpi_device_identify_report (event->dev, event->identify_match, NULL, NULL);

      if (!event->error)
        hold_no_match = event->identify_match == NULL;

    }

  if (hold_no_match)
    {
      Gdix51c0MatchCompleteEvent *finish_event = g_new0 (Gdix51c0MatchCompleteEvent, 1);
      GSource *source = g_timeout_source_new (GDIX51C0_NO_MATCH_HOLD_MSEC);

      finish_event->dev = g_object_ref (event->dev);
      finish_event->kind = event->kind;
      fp_dbg ("gdix51c0: holding no-match completion for %u ms",
              GDIX51C0_NO_MATCH_HOLD_MSEC);
      g_source_set_callback (source,
                             gdix51c0_match_complete_finish_main,
                             finish_event,
                             (GDestroyNotify) gdix51c0_match_complete_event_free);
      g_source_attach (source, NULL);
      g_source_unref (source);
      return G_SOURCE_REMOVE;
    }

  gdix51c0_match_complete_finish_main (event);
  return G_SOURCE_REMOVE;
}

static void
gdix51c0_match_complete_on_main (FpiDeviceGdix51c0 *self,
                                 Gdix51c0ActionKind kind,
                                 gboolean           report_result,
                                 FpiMatchResult     verify_result,
                                 FpPrint           *identify_match,
                                 FpPrint           *updated_print,
                                 GVariant          *updated_data,
                                 GError            *error)
{
  Gdix51c0MatchCompleteEvent *event = g_new0 (Gdix51c0MatchCompleteEvent, 1);
  GMainContext *context = gdix51c0_action_context_ref (self);

  event->dev = g_object_ref (FP_DEVICE (self));
  event->kind = kind;
  event->report_result = report_result;
  event->verify_result = verify_result;
  event->identify_match = identify_match ? g_object_ref (identify_match) : NULL;
  event->updated_print = updated_print ? g_object_ref (updated_print) : NULL;
  event->updated_data = updated_data ? g_variant_ref (updated_data) : NULL;
  event->error = error;
  g_main_context_invoke_full (context,
                              G_PRIORITY_DEFAULT,
                              gdix51c0_match_complete_main,
                              event,
                              (GDestroyNotify) gdix51c0_match_complete_event_free);
  g_main_context_unref (context);
}

static gboolean
gdix51c0_match_report_main (gpointer user_data)
{
  Gdix51c0MatchCompleteEvent *event = user_data;

  if (event->kind == GDIX51C0_ACTION_VERIFY)
    fpi_device_verify_report (
      event->dev,
      event->error ? FPI_MATCH_ERROR : event->verify_result,
      NULL, g_steal_pointer (&event->error));
  else
    fpi_device_identify_report (
      event->dev, event->identify_match, NULL,
      g_steal_pointer (&event->error));

  return G_SOURCE_REMOVE;
}

static void
gdix51c0_match_report_on_main (FpiDeviceGdix51c0 *self,
                               Gdix51c0ActionKind kind,
                               FpiMatchResult     verify_result,
                               FpPrint           *identify_match,
                               GError            *retry_error)
{
  Gdix51c0MatchCompleteEvent *event = g_new0 (Gdix51c0MatchCompleteEvent, 1);
  GMainContext *context = gdix51c0_action_context_ref (self);

  event->dev = g_object_ref (FP_DEVICE (self));
  event->kind = kind;
  event->verify_result = verify_result;
  event->identify_match = identify_match ? g_object_ref (identify_match) : NULL;
  event->error = retry_error;
  g_main_context_invoke_full (context,
                              G_PRIORITY_DEFAULT,
                              gdix51c0_match_report_main,
                              event,
                              (GDestroyNotify) gdix51c0_match_complete_event_free);
  g_main_context_unref (context);
}

static gboolean
gdix51c0_action_check_cancelled (FpiDeviceGdix51c0 *self,
                                 GError           **error)
{
  GCancellable *cancellable = self->action_cancellable;

  if (cancellable && g_cancellable_set_error_if_cancelled (cancellable, error))
    {
      fp_dbg ("gdix51c0: current action was cancelled");
      self->session_desynced = TRUE;
      return TRUE;
    }

  return FALSE;
}

static gboolean
gdix51c0_wait_for_finger (FpiDeviceGdix51c0 *self, GError **error)
{
  gint64 finger_deadline = g_get_monotonic_time () +
                           (gint64) GDIX51C0_FINGER_TIMEOUT_USEC;
  guint zero_touch_count = 0;
  guint retry_count = 0;
  guint nonready_count = 0;

  /* Seed the dynamic FDT threshold from a live no-finger baseline the first
   * time through (finger is off before the user is prompted).  cmd 0x36 reads
   * the base on demand; without it the threshold cannot self-calibrate at a
   * changed DAC.  Non-fatal — fall back to the static bootstrap table. */
  if (!self->fdt_baseline_valid &&
      GDIX51C0_DEV_ENV_INT ("GDIX51C0_FDT_DYNAMIC", 1) != 0)
    {
      g_autoptr(GError) berr = NULL;
      if (!gdix51c0_fdt_measure_base (self, NULL, &berr))
        fp_dbg ("gdix51c0: FDT base measure failed (%s); using static thresholds",
                berr ? berr->message : "?");
    }

  fp_dbg ("gdix51c0: arming FDT-down and waiting for finger GPIO IRQ...");

  if (!gdix51c0_fdt_down_arm (self, error))
    return FALSE;

  for (;;)
    {
      g_autoptr(GError) local_error = NULL;
      guint8 touchflag = 0;
      gint64 now = g_get_monotonic_time ();
      guint remaining_usec;
      guint wait_usec;

      if (gdix51c0_action_check_cancelled (self, error))
        return FALSE;

      if (now >= finger_deadline)
        {
          g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                               "gdix51c0: no finger detected within timeout");
          return FALSE;
        }

      remaining_usec = (guint) MIN (finger_deadline - now,
                                    (gint64) G_MAXUINT);
      wait_usec = MIN (remaining_usec, (guint) GDIX51C0_CANCEL_POLL_USEC);

      if (!gdix51c0_fdt_down_read (self, wait_usec,
                                   &touchflag, &local_error))
        {
          retry_count++;

          if (g_get_monotonic_time () >= finger_deadline)
            {
              g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                                   "gdix51c0: no finger detected within timeout");
              return FALSE;
            }

          if (g_error_matches (local_error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT))
            {
              if (retry_count == 1 || retry_count % 20 == 0)
                fp_dbg ("gdix51c0: FDT-down still armed; waiting for finger event (%u)",
                        retry_count);
              continue;
            }

          fp_dbg ("gdix51c0: FDT-down read failed; rearming (%u): %s",
                  retry_count, local_error ? local_error->message : "?");

          if (!gdix51c0_fdt_down_arm (self, error))
            return FALSE;

          continue;
        }

      retry_count = 0;

      if (touchflag == GDIX51C0_TOUCH_IGNORE)
        {
          fp_dbg ("gdix51c0: ignored async/non-FDT packet while waiting for finger");
          if (!gdix51c0_fdt_down_arm (self, error))
            return FALSE;
          continue;
        }

      if (self->require_lift_gap)
        {
          if (touchflag == 0x00)
            {
              fp_dbg ("gdix51c0: observed zero-touch FDT gap after lift timeout");
              self->require_lift_gap = FALSE;
            }
          else
            {
              fp_dbg ("gdix51c0: waiting for lift gap; ignoring FDT touchflag=0x%02x",
                      touchflag);

              if (!gdix51c0_fdt_down_arm (self, error))
                return FALSE;

              continue;
            }
        }

      if (gdix51c0_fdt_touch_is_finger (self, touchflag))
        {
          fp_dbg ("gdix51c0: finger detected, touchflag=0x%02x zones=%u",
                  touchflag, gdix51c0_fdt_touch_count (touchflag));
          return TRUE;
        }

      if (touchflag == 0x00)
        {
          zero_touch_count++;
          nonready_count = 0;

          fp_dbg ("gdix51c0: ignoring zero-touch FDT event %u",
                  zero_touch_count);

          if (zero_touch_count >= 3)
            zero_touch_count = 0;

          if (!gdix51c0_fdt_down_arm (self, error))
            return FALSE;

          continue;
        }

      /*
       * Non-zero but not 0x3f: contact-ish, but not known-good for image
       * capture. Do not start cmd 0x22 from this state.
       *
       * The UI may feel like it wants to keep scanning after no-match, but
       * The official sensor-type profile supplies the accepted trigger count
       * (5..6 for type 12); fewer zones are usually edge contact and produce
       * weak frames.
       */
      nonready_count++;
      zero_touch_count = 0;

      fp_dbg ("gdix51c0: ignoring non-ready FDT touchflag=0x%02x zones=%u "
              "event %u; require at least %u/%u zones",
              touchflag, gdix51c0_fdt_touch_count (touchflag), nonready_count,
              gdix51c0_fdt_trigger_min (self),
              self->sensor_profile ? self->sensor_profile->fdt_trigger_max : 6);

      if (!gdix51c0_fdt_down_arm (self, error))
        return FALSE;
    }
}

/* Wait for finger lift using the official FDT-up mode. */
static gboolean
gdix51c0_wait_for_lift (FpiDeviceGdix51c0 *self,
                        guint              timeout_usec,
                        GError           **error)
{
  gint64 lift_deadline = g_get_monotonic_time () +
                          (gint64) timeout_usec;
  guint8 touchflag = 0xff;

  g_usleep (20000);

  for (;;)
    {
      gint64 now = g_get_monotonic_time ();
      guint remaining_usec;

      if (gdix51c0_action_check_cancelled (self, error))
        return FALSE;

      if (now >= lift_deadline)
        {
          g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                               "gdix51c0: finger not lifted within timeout");
          return FALSE;
        }

      fp_dbg ("gdix51c0: arming FDT-up and waiting for finger lift IRQ...");

      if (!gdix51c0_fdt_up_arm (self, error))
        return FALSE;

      g_usleep (20000);

      fp_dbg ("gdix51c0: FDT-up armed; waiting for lift response");

      /* The device fires its FDT-up event ~70 ms after arming.  Always give
       * the read at least that much budget, even past lift_deadline, so we
       * don't leave a primed event undrained on the SPI bus.  An undrained
       * event keeps IRQ high and breaks the next FDT-down arm. */
      now = g_get_monotonic_time ();
      remaining_usec = (guint) (lift_deadline > now
                                ? MIN (lift_deadline - now, (gint64) G_MAXUINT)
                                : 0);
      if (remaining_usec < GDIX51C0_FDT_UP_DRAIN_USEC)
        remaining_usec = GDIX51C0_FDT_UP_DRAIN_USEC;
      remaining_usec = MIN (remaining_usec, (guint) GDIX51C0_CANCEL_POLL_USEC);

      {
        g_autoptr(GError) up_err = NULL;
        if (!gdix51c0_fdt_up_read (self, remaining_usec, &touchflag, &up_err))
          {
            if (g_error_matches (up_err, G_IO_ERROR, G_IO_ERROR_TIMED_OUT))
              {
                g_autoptr(GError) manual_err = NULL;
                guint8 manual_touchflag = 0xff;
                guint16 manual_sample[6];

                /* Windows does not treat silence as lift.  Its retry path
                 * probes FDT-manual and branches on touchflag: 0 -> down base
                 * / finger off, nonzero -> up base / finger still down. */
                gdix51c0_listener_drain_cmd (self->listener, GDIX51C0_CMD_FDT_UP);
                if (gdix51c0_fdt_manual_read (self,
                                              &manual_touchflag,
                                              manual_sample,
                                              &manual_err))
                  {
                    touchflag = manual_touchflag;
                    if ((touchflag & 0x3f) == 0)
                      gdix51c0_fdt_store_zero_touch_baseline (self,
                                                              manual_sample,
                                                              "FDT-manual lift",
                                                              TRUE);
                    else
                      {
                        memcpy (self->fdt_down_sample, manual_sample,
                                sizeof (self->fdt_down_sample));
                        self->fdt_down_sample_valid = TRUE;
                        fp_dbg ("gdix51c0: FDT-up timeout, but manual still sees "
                                "touchflag=0x%02x; continuing lift wait",
                                touchflag);
                      }
                  }
                else
                  {
                    fp_dbg ("gdix51c0: FDT-up timeout and manual probe failed (%s); retrying",
                            manual_err ? manual_err->message : "?");
                    touchflag = 0xff;
                  }
              }
            else
              {
                g_propagate_error (error, g_steal_pointer (&up_err));
                return FALSE;
              }
          }
      }

      if (touchflag == 0)
        {
          fp_dbg ("gdix51c0: finger lifted");

          /* Hardware-confirmed finger-off is the right moment to capture
           * the dark-frame baseline.  We only do this once per cold open:
           * the persisted frame remains the fallback until that refresh.
           * GDIX51C0_FORCE_DARK_FRAME recaptures on every lift (for RE /
           * grabbing a fresh no-finger reference). */
          if ((!self->image_base_refreshed_this_open ||
               GDIX51C0_DEV_ENV ("GDIX51C0_FORCE_DARK_FRAME") ||
               (!self->t0_baseline_valid &&
                GDIX51C0_DEV_ENV ("GDIX51C0_ENABLE_DARK_FRAME"))))
            gdix51c0_baseline_capture_and_save (self);

          return TRUE;
        }

      fp_dbg ("gdix51c0: FDT-up still sees touchflag=0x%02x; waiting", touchflag);
      g_usleep (50000);
    }
}

#ifdef GOODIX_SPI_DEVELOPER
static void
gdix51c0_dump_debug_views (const guint16 *raw)
{
  if (!g_getenv ("GDIX51C0_DUMP_FRAMES"))
    return;

  guint16 mn = 0xffff, mx = 0;

  for (gsize i = 0; i < GDIX51C0_FRAME_PIXELS; i++)
    {
      if (raw[i] < mn)
        mn = raw[i];
      if (raw[i] > mx)
        mx = raw[i];
    }

  guint span = mx > mn ? (guint) (mx - mn) : 1;

  guint8 normal[GDIX51C0_FRAME_PIXELS];
  guint8 inverted[GDIX51C0_FRAME_PIXELS];

  for (gsize i = 0; i < GDIX51C0_FRAME_PIXELS; i++)
    {
      guint v = ((guint) (raw[i] - mn) * 255u + span / 2) / span;
      v = MIN (v, 255u);

      normal[i] = (guint8) v;
      inverted[i] = (guint8) (255u - v);
    }

  gint64 ts = g_get_monotonic_time ();

  g_autofree char *path_normal =
    g_strdup_printf ("/tmp/gdix51c0_debug_normal_%" G_GINT64_FORMAT ".pgm", ts);
  g_autofree char *path_inverted =
    g_strdup_printf ("/tmp/gdix51c0_debug_inverted_%" G_GINT64_FORMAT ".pgm", ts);

  FILE *fp = fopen (path_normal, "wb");
  if (fp)
    {
      fprintf (fp, "P5\n%d %d\n255\n",
               GDIX51C0_IMAGE_WIDTH,
               GDIX51C0_IMAGE_HEIGHT);
      fwrite (normal, 1, GDIX51C0_FRAME_PIXELS, fp);
      fclose (fp);
    }

  fp = fopen (path_inverted, "wb");
  if (fp)
    {
      fprintf (fp, "P5\n%d %d\n255\n",
               GDIX51C0_IMAGE_WIDTH,
               GDIX51C0_IMAGE_HEIGHT);
      fwrite (inverted, 1, GDIX51C0_FRAME_PIXELS, fp);
      fclose (fp);
    }

  /* Raw 16-bpp frame exactly as AlgoMilan's preprocessor wants it: little-endian
   * guint16 x GDIX51C0_FRAME_PIXELS (= 10240 bytes for 64x80).  Feed this to
   * re/algo-oracle for the matching-algorithm RE.  Write into the fprintd
   * StateDirectory (/var/lib/fprint) since the service runs with PrivateTmp +
   * ProtectSystem=strict, which hide/deny /tmp.  Override dir with GDIX51C0_DUMP_DIR. */
  const char *dump_dir = g_getenv ("GDIX51C0_DUMP_DIR");
  if (!dump_dir || !*dump_dir)
    dump_dir = "/var/lib/fprint";
  g_autofree char *path_raw =
    g_strdup_printf ("%s/gdix51c0_frame_%" G_GINT64_FORMAT ".raw", dump_dir, ts);
  fp = fopen (path_raw, "wb");
  if (fp)
    {
      fwrite (raw, sizeof (guint16), GDIX51C0_FRAME_PIXELS, fp);
      fclose (fp);
    }

  fp_dbg ("gdix51c0: dumped debug views normal=%s inverted=%s raw=%s raw_mn=%u raw_mx=%u",
          path_normal, path_inverted, path_raw, mn, mx);
}
#endif

/* ------------------------------------------------------------------ */
/* Single image capture (cmd 0x22 -> ack -> read -> decrypt -> decode). */
/* Returns a fresh guint16[GDIX51C0_FRAME_PIXELS] on success.              */
/* ------------------------------------------------------------------ */
static guint16 *
gdix51c0_capture_image_raw (FpiDeviceGdix51c0 *self,
                            guint8             cmd,
                            gboolean           retry_image,
                            GError           **error)
{
#ifdef GOODIX_SPI_DEVELOPER
  gboolean exhaust_t0_data;
#endif
  guint8 img_setmode[] = {
    cmd,
    0x03, 0x00,
    0x01,
    0x00,
    0x00,
  };

  img_setmode[5] = gdix51c0_payload_checksum (img_setmode,
                                              sizeof (img_setmode) - 1);

#define GDIX51C0_IMAGE_PREFIX_LEN (3 + 5)

  /*
   * SpiSendDataToDeviceLock repeats the complete setmode operation once when
   * its required data event is absent.  That is distinct from the lower ACK
   * resend performed by gdix51c0_listener_send_ack().  We reproduce both
   * levels, but still never retry a record after it was received/decryption
   * began because that would advance the TLS sequence twice.
   */
  if (G_UNLIKELY (!self->listener))
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_NOT_INITIALIZED,
                           "gdix51c0: image capture called before listener up");
      return NULL;
    }

#ifdef GOODIX_SPI_DEVELOPER
  exhaust_t0_data = gdix51c0_arm_t0_data_exhaust_once (cmd);
  if (exhaust_t0_data)
    fp_warn ("gdix51c0: T0 armed one-shot image-data-exhaustion injection");
#endif

  /* Empirically required caller-level gap between the asynchronous FDT-down
   * event and the initial image setmode.  OnRetryCaptureIMG instead submits
   * 0x20 immediately after its synchronous FDT-manual result; the transport's
   * own official 1 ms delay remains in both paths. */
  if (!retry_image)
    g_usleep (10000);

  for (guint attempt = 1; attempt <= 2; attempt++)
    {
      g_autoptr(GError) local_error = NULL;

      fp_dbg ("gdix51c0: image capture attempt %u cmd=0x%02x%s",
              attempt, img_setmode[0],
              retry_image ? " (RetryCaptureIMG)" : "");

      /* Drop any stale ACK / image packets the MCU may have queued from a
       * previous transaction so they don't satisfy our await().  Image
       * payloads dispatch on the TLS appdata content type 0x17 (raw TLS
       * record) — the Goodix request cmd byte (0x22 / 0x20) is what we
       * SEND, not what we receive in the response payload. */
      gdix51c0_listener_drain_cmd (self->listener, 0x17);

      if (!gdix51c0_listener_send_ack (self->listener,
                                       img_setmode, sizeof (img_setmode),
                                       "img-setmode", &local_error))
        {
          g_propagate_error (error, g_steal_pointer (&local_error));
          return NULL;
        }

      /* The sensor now reads the array into its buffer over ~73 ms and only
       * then signals the image is ready.  Windows sits completely silent on
       * the SPI bus during this window; our async listener otherwise keeps
       * polling the bus (spi_read_typed even retries on idle), and that traffic
       * chops the analog readout at a fixed row — the 0x0b78 "row-31 cliff".
       * Suppress ALL listener SPI access for the readout duration, then release
       * it so the finished image is picked up on the next poll. */
      {
        guint settle_ms = (guint) GDIX51C0_DEV_ENV_INT (
          "GDIX51C0_CAPTURE_SETTLE_MS", 80);
        gdix51c0_listener_set_suppress (self->listener, TRUE);
        g_usleep ((gulong) settle_ms * 1000);
        gdix51c0_listener_set_suppress (self->listener, FALSE);
      }

      /* Only a missing data event follows the official full-command retry.
       * Once a TLS record is returned, every validation/decrypt failure is
       * terminal for this session because the record sequence may advance. */
      gsize record_len = 0;
      g_autofree guint8 *encrypted =
        gdix51c0_listener_await (self->listener, 0x17,
                                 GDIX51C0_IMAGE_TIMEOUT_USEC,
                                 &record_len, &local_error);

#ifdef GOODIX_SPI_DEVELOPER
      if (encrypted && exhaust_t0_data)
        {
          fp_warn ("gdix51c0: fault injection dropped T0 image record "
                   "on data attempt %u/2 (%zu B)",
                   attempt, record_len);
          g_clear_pointer (&encrypted, g_free);
          record_len = 0;
          g_usleep (GDIX51C0_IMAGE_TIMEOUT_USEC);
          g_set_error (&local_error,
                       G_IO_ERROR,
                       G_IO_ERROR_TIMED_OUT,
                       "gdix51c0: injected T0 image-data timeout on attempt %u/2",
                       attempt);
        }
#endif

      if (!encrypted)
        {
          if (attempt == 1 &&
              g_error_matches (local_error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT))
            {
              fp_dbg ("gdix51c0: image data response timed out; resubmitting "
                      "complete command once like SpiSendDataToDeviceLock");
              continue;
            }

          g_set_error (error,
                      G_IO_ERROR,
                      G_IO_ERROR_CONNECTION_CLOSED,
                      "gdix51c0: image read failed after setmode ACK: %s",
                      local_error ? local_error->message : "?");
          return NULL;
        }

      if (record_len < 5)
        {
          g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                       "gdix51c0: image packet too short: %zu", record_len);
          return NULL;
        }

      if (!gdix51c0_tls_record_header_ok (encrypted, record_len))
        {
          g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                       "gdix51c0: image packet is not a valid TLS record "
                       "(len=%zu)", record_len);
          return NULL;
        }

      if (encrypted[0] != 0x17)
        {
          g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                       "gdix51c0: image packet is TLS type 0x%02x, expected appdata 0x17",
                       encrypted[0]);
          return NULL;
        }

      gsize plain_len = 0;
      g_autofree guint8 *plain =
        gdix51c0_tls_decrypt_record (&self->tls,
                                     encrypted, record_len,
                                     &plain_len, &local_error);

      if (!plain)
        {
          g_set_error (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                       "gdix51c0: image TLS decrypt failed; session is no longer reusable: %s",
                       local_error ? local_error->message : "?");
          return NULL;
        }

      fp_dbg ("gdix51c0: image plaintext len=%zu", plain_len);

#ifdef GOODIX_SPI_DEVELOPER
      gdix51c0_dump_image_blob ("plain", cmd, plain, plain_len);

      if (plain_len >= GDIX51C0_IMAGE_PREFIX_LEN)
        {
          gsize image_payload_len = plain_len - GDIX51C0_IMAGE_PREFIX_LEN;
          gsize footer_len = image_payload_len > GDIX51C0_FRAME_BYTES ?
                             image_payload_len - GDIX51C0_FRAME_BYTES : 0;

          fp_dbg ("gdix51c0: image app prefix=%02x %02x %02x %02x %02x %02x %02x %02x "
                  "payload_after_prefix=%zu footer=%zu",
                  plain[0], plain[1], plain[2], plain[3],
                  plain[4], plain[5], plain[6], plain[7],
                  image_payload_len, footer_len);

          if (footer_len > 0)
            {
              const guint8 *footer = plain + GDIX51C0_IMAGE_PREFIX_LEN + GDIX51C0_FRAME_BYTES;
              gsize dump_len = MIN (footer_len, (gsize) 16);
              GString *s = g_string_sized_new (dump_len * 3 + 1);

              for (gsize i = 0; i < dump_len; i++)
                g_string_append_printf (s, "%02x%s", footer[i],
                                        i + 1 == dump_len ? "" : " ");
              fp_dbg ("gdix51c0: image footer bytes %s", s->str);
              g_string_free (s, TRUE);
            }
        }
#endif

      if (plain_len < GDIX51C0_IMAGE_PREFIX_LEN + GDIX51C0_FRAME_BYTES)
        {
          g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                       "gdix51c0: image plaintext too short: %zu, need at least %u",
                       plain_len,
                       (guint) (GDIX51C0_IMAGE_PREFIX_LEN + GDIX51C0_FRAME_BYTES));
          return NULL;
        }

      const guint8 *image_packed = plain + GDIX51C0_IMAGE_PREFIX_LEN;

#ifdef GOODIX_SPI_DEVELOPER
      gdix51c0_dump_image_blob ("packed", cmd, image_packed,
                                MIN (plain_len - GDIX51C0_IMAGE_PREFIX_LEN,
                                     (gsize) GDIX51C0_FRAME_BYTES + 16));
#endif

      guint16 *raw = g_malloc (GDIX51C0_FRAME_PIXELS * sizeof (guint16));
      gdix51c0_decode_12bpp_to_16bpp (image_packed,
                                      GDIX51C0_FRAME_BYTES,
                                      raw,
                                      GDIX51C0_FRAME_PIXELS);
#ifdef GOODIX_SPI_DEVELOPER
      gdix51c0_log_raw_image_stats (raw);
#endif
      return raw;
    }

  g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                       "gdix51c0: image setmode failed before ACK");
  return NULL;

#undef GDIX51C0_IMAGE_PREFIX_LEN
}

static gboolean
gdix51c0_capture_nav_data (FpiDeviceGdix51c0 *self, GError **error)
{
  const guint8 nav_data[] = { 0x00, 0x00 };
  gsize nav_pkt_len = 0;
  gsize nav_len = 0;
  g_autofree guint8 *nav_packet =
    gdix51c0_make_payload_packet (GDIX51C0_CMD_NAV_BASE,
                                  nav_data, sizeof (nav_data),
                                  &nav_pkt_len);
  g_autofree guint8 *nav = NULL;

  if (self->listener)
    {
      /* Once the session listener owns IRQ and SPI, both halves of this
       * request must be dispatched through it.  A synchronous read here
       * races the listener and can consume either the ACK or NAV response. */
      nav = gdix51c0_listener_command (self->listener,
                                       nav_packet, nav_pkt_len,
                                       GDIX51C0_CMD_NAV_BASE,
                                       GDIX51C0_RESPONSE_TIMEOUT_USEC,
                                       &nav_len, "nav-data", error);
    }
  else
    {
      Gdix51c0Bus bus = {
        .dev = FP_DEVICE (self),
        .spi_fd = self->spi_fd,
        .irq_req = self->irq_req,
        .irq_offset = self->irq_offset,
        .irq_events = self->irq_events,
      };

      /* Keep the synchronous form for pre-listener initialization callers. */
      nav = gdix51c0_cmd_ack_then_resp_read (&bus,
                                             nav_packet,
                                             nav_pkt_len,
                                             GDIX51C0_IMAGE_TIMEOUT_USEC,
                                             &nav_len,
                                             "nav-data",
                                             error);
    }

  if (!nav)
    return FALSE;

  if (nav_len < 3 || nav[0] != GDIX51C0_CMD_NAV_BASE)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                   "gdix51c0: invalid nav response len=%zu cmd=0x%02x",
                   nav_len, nav_len > 0 ? nav[0] : 0);
      return FALSE;
    }

  fp_dbg ("gdix51c0: nav response len=%zu cmd=0x%02x", nav_len, nav[0]);
  if (nav_len != GDIX51C0_NAV_RESPONSE_LEN)
    fp_dbg ("gdix51c0: nav response length was %zu, expected %u",
            nav_len, GDIX51C0_NAV_RESPONSE_LEN);

  return TRUE;
}

static gboolean
gdix51c0_capture_error_needs_session_restart (const GError *error)
{
  if (!error)
    return FALSE;

  if (g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CONNECTION_CLOSED))
    return TRUE;

  /*
   * After image setmode ACK, an all-FF/all-zero image read means the
   * current TLS/session is not trustworthy anymore.
   */
  if (error->message &&
      (strstr (error->message, "image read failed after setmode ACK") ||
       strstr (error->message, "image data IRQ did not arrive after setmode ACK") ||
       strstr (error->message, "FDT lost sync")))
    return TRUE;

  return FALSE;
}

static gboolean
gdix51c0_error_is_transient_verify_failure (const GError *error)
{
  if (!error || !error->message)
    return FALSE;

  if (strstr (error->message, "IRQ") &&
      strstr (error->message, "never went"))
    return TRUE;

  if (strstr (error->message, "TLS handshake did not receive MCU ClientHello"))
    return TRUE;

  return FALSE;
}

/* Canonical Chicago capture: wait for finger and return the decoded    */
/* 64x80, 12-bit-in-guint16 frame.  This is the direct Linux analogue  */
/* of gf_get_oneframe / RetryCaptureIMG's raw image payload and is the  */
/* only representation the future native Chicago preprocessor may use. */
/* ------------------------------------------------------------------ */
static guint16 *
gdix51c0_capture_one_raw16 (FpiDeviceGdix51c0 *self, GError **error)
{
  for (guint session_attempt = 0; session_attempt < 2; session_attempt++)
    {
      g_autoptr(GError) local_error = NULL;
      g_autofree guint16 *raw = NULL;

      if (!gdix51c0_wait_for_finger (self, &local_error))
        {
          if (gdix51c0_capture_error_needs_session_restart (local_error) &&
              session_attempt == 0)
            {
              fp_warn ("gdix51c0: capture wait lost sync; restarting session: %s",
                       local_error ? local_error->message : "?");

              gdix51c0_session_deactivate (self);

              if (!gdix51c0_session_activate (self, &local_error))
                {
                  g_propagate_error (error, g_steal_pointer (&local_error));
                  return NULL;
                }

              gdix51c0_report_finger_status_on_main (self, FP_FINGER_STATUS_NEEDED);
              continue;
            }

          g_propagate_error (error, g_steal_pointer (&local_error));
          return NULL;
        }

      gdix51c0_report_finger_status_on_main (self, FP_FINGER_STATUS_PRESENT);

      raw = gdix51c0_capture_image_raw (self, GDIX51C0_CMD_IMAGE_T1, FALSE, &local_error);
      if (!raw)
        {
          if (gdix51c0_capture_error_needs_session_restart (local_error) &&
              session_attempt == 0)
            {
              fp_warn ("gdix51c0: image capture lost sync; restarting session: %s",
                       local_error ? local_error->message : "?");

              gdix51c0_session_deactivate (self);

              if (!gdix51c0_session_activate (self, &local_error))
                {
                  g_propagate_error (error, g_steal_pointer (&local_error));
                  return NULL;
                }

              gdix51c0_report_finger_status_on_main (self, FP_FINGER_STATUS_NEEDED);
              continue;
            }

          g_propagate_error (error, g_steal_pointer (&local_error));
          return NULL;
        }

      /* Windows does not read nav data on the normal FDT-down -> image(0x22)
       * capture path (see WBDI-new.log:913-951).  Nav(0x50) is used in
       * base-update/base-validation flows (for example WBDI-new.log:746-756
       * and 1033-1044).  Keep it available for protocol debugging, but do
       * not make normal capture depend on this optional response. */
      if (GDIX51C0_DEV_ENV ("GDIX51C0_CAPTURE_NAV_DATA") &&
          !gdix51c0_capture_nav_data (self, &local_error))
        {
          fp_warn ("gdix51c0: optional nav-data capture failed: %s",
                   local_error ? local_error->message : "?");
          g_clear_error (&local_error);
        }

#ifdef GOODIX_SPI_DEVELOPER
      gdix51c0_dump_debug_views (raw);
#endif

      return g_steal_pointer (&raw);
    }

  g_set_error_literal (error,
                       G_IO_ERROR,
                       G_IO_ERROR_FAILED,
                       "gdix51c0: capture failed after session restart");
  return NULL;
}

/* Windows OnRetryCaptureIMG keeps the current press alive.  It switches to
 * FDT-manual, counts the set bits in the two-byte TouchFlag, compares that
 * count with the sensor profile's FDT trigger minimum, and captures the retry
 * frame with image command 0x20.  It does not require a lift/new FDT-down IRQ
 * between the initial 0x22 image and these retry images. */
static guint16 *
gdix51c0_capture_retry_raw16 (FpiDeviceGdix51c0 *self,
                              gboolean          *finger_still_down,
                              GError           **error)
{
  guint8 touchflag = 0xff;
  guint16 sample[6];
  guint touched;
  guint trigger_min = gdix51c0_fdt_trigger_min (self);

  g_return_val_if_fail (finger_still_down != NULL, NULL);
  *finger_still_down = FALSE;

  if (!gdix51c0_fdt_manual_read (self, &touchflag, sample, error))
    return NULL;

  touched = gdix51c0_fdt_touch_count (touchflag);
  fp_dbg ("gdix51c0: RetryCaptureIMG FDT-manual touchflag=0x%02x "
          "touched=%u trigger=%u/%u",
          touchflag, touched, trigger_min,
          self->sensor_profile ? self->sensor_profile->fdt_trigger_max : 6);

  if (touched < trigger_min)
    {
      fp_dbg ("gdix51c0: RetryCaptureIMG stopped because finger is no longer "
              "down (%u < %u zones)", touched, trigger_min);
      return NULL;
    }

  memcpy (self->fdt_down_sample, sample, sizeof (self->fdt_down_sample));
  self->fdt_down_sample_valid = TRUE;
  *finger_still_down = TRUE;

  return gdix51c0_capture_image_raw (self,
                                     GDIX51C0_CMD_IMAGE_T0,
                                     TRUE,
                                     error);
}

static gboolean
gdix51c0_wait_for_lift_report (FpiDeviceGdix51c0 *self,
                               guint              timeout_usec,
                               gboolean           allow_fdt_down_gap)
{
  GError *lift_err = NULL;

  if (!gdix51c0_wait_for_lift (self, timeout_usec, &lift_err))
    {
      if (allow_fdt_down_gap &&
          g_error_matches (lift_err, G_IO_ERROR, G_IO_ERROR_TIMED_OUT))
        {
          fp_dbg ("gdix51c0: FDT-up did not observe lift; requiring FDT-down zero-touch gap");
          self->require_lift_gap = TRUE;
          g_clear_error (&lift_err);
          gdix51c0_report_finger_status_on_main (self, FP_FINGER_STATUS_NONE);
          return TRUE;
        }

      fp_warn ("gdix51c0: lift wait failed (%s), reporting off anyway",
               lift_err ? lift_err->message : "?");
      g_clear_error (&lift_err);
      gdix51c0_report_finger_status_on_main (self, FP_FINGER_STATUS_NONE);
      return FALSE;
    }

  self->require_lift_gap = FALSE;
  gdix51c0_report_finger_status_on_main (self, FP_FINGER_STATUS_NONE);
  return TRUE;
}

static void
gdix51c0_runtime_match_print (
  FpiDeviceGdix51c0               *self,
  FpPrint                         *print,
  const GoodixChicagoRuntimeProbe *probe,
  GoodixChicagoMatchTemplateResult *result,
  GError                         **error)
{
  g_autoptr(GVariant) data = NULL;

  g_object_get (G_OBJECT (print), "fpi-data", &data, NULL);
  goodix_chicago_runtime_match_print_data (
    probe, data, self->sensor_id, self->chicago_calibration, result, error);
}

static void
gdix51c0_enroll_sync (FpDevice *dev,
                      FpPrint  *print)
{
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (dev);
  g_autoptr(GoodixChicagoEnrollment) enrollment = NULL;
  g_autoptr(GoodixChicagoRuntimeProbe) deferred_probe = NULL;
  g_autoptr(GVariant) data = NULL;
  GoodixChicagoEngineEnrollmentPolicy policy;
  GError *error = NULL;

  if (!gdix51c0_session_activate (self, &error))
    goto out;
  if (!self->chicago_preprocessor || !self->sensor_id_valid ||
      !self->chicago_calibration)
    {
      g_set_error_literal (&error, G_IO_ERROR, G_IO_ERROR_NOT_INITIALIZED,
                           "gdix51c0: native Chicago enrollment requires calibration and ImageBase");
      goto out;
    }

  enrollment = goodix_chicago_enrollment_new ();
  goodix_chicago_engine_enrollment_policy_init (&policy);
  while (!goodix_chicago_engine_enrollment_policy_complete (&policy))
    {
      g_autofree guint16 *raw = NULL;
      g_autoptr(GoodixChicagoRuntimeProbe) probe = NULL;
      const GoodixChicagoSubtemplateView *view;
      GoodixChicagoRuntimeReject reject;
      GoodixChicagoEnrollmentResult result;
      guint position_reject = 0;
      gboolean policy_accepted;

      gdix51c0_report_finger_status_on_main (self, FP_FINGER_STATUS_NEEDED);
      raw = gdix51c0_capture_one_raw16 (self, &error);
      if (!raw)
        goto out;
      probe = goodix_chicago_runtime_prepare_probe (
        self->chicago_preprocessor, raw, &reject, &error);
      if (!probe)
        {
          if (error)
            goto out;
          fp_dbg ("gdix51c0: Chicago enrollment retry reason=%s",
                  reject == GOODIX_CHICAGO_RUNTIME_REJECT_NO_FEATURES ?
                  "no-features" :
                  reject == GOODIX_CHICAGO_RUNTIME_REJECT_BAD_INPUT ?
                  "bad-input" : "poor-capture");
          gdix51c0_enroll_progress_on_main (
            self, policy.used,
            fpi_device_retry_new_msg (FP_DEVICE_RETRY_GENERAL,
                                      reject == GOODIX_CHICAGO_RUNTIME_REJECT_NO_FEATURES ?
                                      "No fingerprint features found; cover the sensor fully." :
                                      "Fingerprint image quality is too low; try again."));
          if (!gdix51c0_wait_for_lift_report (
                self, GDIX51C0_ENROLL_LIFT_TIMEOUT_USEC, TRUE))
            {
              g_set_error_literal (&error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                                   "gdix51c0: finger was not lifted after Chicago enrollment retry");
              goto out;
            }
          continue;
        }
      view = goodix_chicago_runtime_probe_get_view (probe);

      if (goodix_chicago_enrollment_get_count (enrollment) == 0)
        {
          if (!goodix_chicago_enrollment_insert_first (
                enrollment, view->records, view->record_count, view->active_count,
                view->quality, view->coverage, view->metric_data, &result, &error))
            goto out;
        }
      else if (!goodix_chicago_enrollment_insert_next (
                 enrollment, view->records, view->record_count, view->active_count,
                 view->quality, view->coverage, view->metric_data, &result, &error))
        goto out;

      policy_accepted = goodix_chicago_engine_enrollment_policy_accept (
        &policy, result.position_x, result.position_y, &position_reject);
      if (policy.defer_current_sample)
        {
          if (deferred_probe != NULL ||
              !goodix_chicago_enrollment_drop_last (enrollment, &error))
            {
              if (error == NULL)
                g_set_error_literal (&error, G_IO_ERROR, G_IO_ERROR_FAILED,
                                     "gdix51c0: duplicate deferred Chicago enrollment sample");
              goto out;
            }
          deferred_probe = g_steal_pointer (&probe);
        }
      if (policy.restore_deferred_sample)
        {
          GoodixChicagoEnrollmentResult deferred_result;
          const GoodixChicagoSubtemplateView *deferred_view;

          if (deferred_probe == NULL)
            {
              g_set_error_literal (&error, G_IO_ERROR, G_IO_ERROR_FAILED,
                                   "gdix51c0: missing deferred Chicago enrollment sample");
              goto out;
            }
          deferred_view = goodix_chicago_runtime_probe_get_view (
            deferred_probe);
          if (!goodix_chicago_enrollment_insert_next (
                enrollment, deferred_view->records,
                deferred_view->record_count, deferred_view->active_count,
                deferred_view->quality, deferred_view->coverage,
                deferred_view->metric_data, &deferred_result, &error))
            goto out;
          g_clear_pointer (&deferred_probe,
                           goodix_chicago_runtime_probe_free);
        }

      if (!policy_accepted)
        {
          const gchar *message = position_reject == 1 ?
            "Finger is too high; move it lower." : position_reject == 2 ?
            "Finger is too low; move it higher." : position_reject == 3 ?
            "Finger is too far left; move it right." :
            "Finger is too far right; move it left.";

          fp_dbg ("gdix51c0: Chicago enrollment retained attempt=%u "
                   "position=%u,%u reject=%u accepted=%u",
                   policy.touched, result.position_x, result.position_y,
                   position_reject, policy.used);
          gdix51c0_enroll_progress_on_main (
            self, policy.used,
            fpi_device_retry_new_msg (FP_DEVICE_RETRY_CENTER_FINGER,
                                      "%s", message));
        }
      else
        {
          fp_dbg ("gdix51c0: Chicago enrollment accepted=%u/%u attempt=%u "
                   "position=%u,%u",
                   policy.used, GOODIX_CHICAGO_ENGINE_REQUIRED_SAMPLES,
                   policy.touched, result.position_x, result.position_y);
          gdix51c0_enroll_progress_on_main (self, policy.used, NULL);
        }
      if (!goodix_chicago_engine_enrollment_policy_complete (&policy) &&
          !gdix51c0_wait_for_lift_report (
            self, GDIX51C0_ENROLL_LIFT_TIMEOUT_USEC, TRUE))
        {
          g_set_error_literal (&error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                               "gdix51c0: finger was not lifted between Chicago enrollment stages");
          goto out;
        }
    }

  {
    g_autoptr(GBytes) packed = goodix_chicago_enrollment_pack (
      enrollment, &error);

    if (!packed)
      goto out;
    data = g_variant_ref_sink (goodix_chicago_print_data_build (
      self->sensor_id, self->chicago_calibration, packed));
  }
  self->skip_next_identify = TRUE;
  fp_dbg ("gdix51c0: native Chicago enrollment complete with %u subtemplates, "
           "%u accepted and %u position retries",
           goodix_chicago_enrollment_get_count (enrollment), policy.used,
           policy.tipped);

out:
  gdix51c0_session_deactivate (self);
  gdix51c0_enroll_complete_on_main (self, print, g_steal_pointer (&data), error);
}

static void
gdix51c0_verify_or_identify (FpDevice            *dev,
                             Gdix51c0ActionKind   kind,
                             FpPrint             *verify_template,
                             GPtrArray           *identify_gallery)
{
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (dev);
  g_autofree guint16 *raw = NULL;
  g_autoptr(GoodixChicagoRuntimeProbe) probe = NULL;
  GError *error = NULL;
  gboolean saw_finger = FALSE;
  gboolean reported = FALSE;
  gboolean report_sent = FALSE;
  gboolean reported_match = FALSE;
  gboolean saw_valid_probe = FALSE;
  gboolean transient_failure = FALSE;
  gboolean lift_cleanup_ok = TRUE;
  FpiMatchResult final_verify_result = FPI_MATCH_FAIL;
  FpPrint *final_identify_match = NULL;
  FpPrint *updated_print = NULL;
  g_autoptr(GVariant) updated_data = NULL;
  GoodixChicagoMatchTemplateResult final_match_result = { 0, };
  gboolean final_match_result_valid = FALSE;
  GoodixChicagoRuntimeReject last_reject =
    GOODIX_CHICAGO_RUNTIME_REJECT_NONE;

  if (!gdix51c0_session_activate (self, &error))
    goto out;
  if (!self->chicago_preprocessor || !self->chicago_calibration ||
      !self->sensor_id_valid)
    {
      g_set_error_literal (&error, G_IO_ERROR, G_IO_ERROR_NOT_INITIALIZED,
                           "gdix51c0: Chicago matching requires calibration and ImageBase");
      goto out;
    }

  gdix51c0_report_finger_status_on_main (self, FP_FINGER_STATUS_NEEDED);
  for (guint attempt = 1; attempt <= GDIX51C0_VERIFY_CAPTURE_ATTEMPTS; attempt++)
    {
      g_clear_pointer (&raw, g_free);
      g_clear_pointer (&probe, goodix_chicago_runtime_probe_free);

      if (attempt > 1)
        {
          gboolean finger_still_down = FALSE;

          fp_dbg ("gdix51c0: Chicago RetryCaptureIMG attempt %u on current press",
                  attempt);
          raw = gdix51c0_capture_retry_raw16 (self,
                                              &finger_still_down,
                                              &error);
          if (!raw && !error && !finger_still_down)
            break;
        }
      else
        raw = gdix51c0_capture_one_raw16 (self, &error);

      if (!raw)
        {
          if (error)
            goto out;
          break;
        }
      saw_finger = TRUE;

      probe = goodix_chicago_runtime_prepare_probe (
        self->chicago_preprocessor, raw, &last_reject, &error);
      if (!probe)
        {
          if (error)
            goto out;
          fp_dbg ("gdix51c0: Chicago capture rejected attempt=%u reason=%s",
                   attempt,
                   last_reject == GOODIX_CHICAGO_RUNTIME_REJECT_NO_FEATURES ?
                   "no-features" :
                   last_reject == GOODIX_CHICAGO_RUNTIME_REJECT_BAD_INPUT ?
                   "bad-input" : "poor-capture");
          if (attempt < GDIX51C0_VERIFY_CAPTURE_ATTEMPTS)
            continue;
          break;
        }
      saw_valid_probe = TRUE;
      last_reject = GOODIX_CHICAGO_RUNTIME_REJECT_NONE;

      if (kind == GDIX51C0_ACTION_VERIFY)
        {
          GoodixChicagoMatchTemplateResult match_result = { 0, };
          gint32 score;
          FpiMatchResult result;

          gdix51c0_runtime_match_print (
            self, verify_template, probe, &match_result, &error);
          if (error)
            goto out;
          score = match_result.score;
          result = score > 0 ? FPI_MATCH_SUCCESS : FPI_MATCH_FAIL;
          fp_dbg ("gdix51c0: Chicago verify score=%d attempt=%u result=%s",
                   score, attempt,
                   result == FPI_MATCH_SUCCESS ? "MATCH" : "NO_MATCH");
          if (result == FPI_MATCH_SUCCESS ||
              attempt == GDIX51C0_VERIFY_CAPTURE_ATTEMPTS)
            {
              final_verify_result = result;
              reported = TRUE;
              reported_match = result == FPI_MATCH_SUCCESS;
              if (reported_match)
                {
                  final_match_result = match_result;
                  final_match_result_valid = TRUE;
                }
              if (!reported_match)
                {
                  gdix51c0_match_report_on_main (self,
                                                 GDIX51C0_ACTION_VERIFY,
                                                 FPI_MATCH_FAIL,
                                                 NULL,
                                                 NULL);
                  report_sent = TRUE;
                }
              break;
            }
        }
      else
        {
          FpPrint *match = NULL;
          gint32 best_score = G_MININT32;

          for (guint i = 0; identify_gallery && i < identify_gallery->len; i++)
            {
              FpPrint *candidate = g_ptr_array_index (identify_gallery, i);
              GoodixChicagoMatchTemplateResult match_result = { 0, };
              gint32 score;

              gdix51c0_runtime_match_print (
                self, candidate, probe, &match_result, &error);
              if (error)
                goto out;
              score = match_result.score;
              if (score > 0 && score > best_score)
                {
                  best_score = score;
                  match = candidate;
                  final_match_result = match_result;
                  final_match_result_valid = TRUE;
                }
            }

          fp_dbg ("gdix51c0: Chicago identify score=%d attempt=%u result=%s",
                   best_score, attempt, match ? "MATCH" : "NO_MATCH");
          if (match || attempt == GDIX51C0_VERIFY_CAPTURE_ATTEMPTS)
            {
              final_identify_match = match;
              reported = TRUE;
              reported_match = match != NULL;
              if (!reported_match)
                {
                  gdix51c0_match_report_on_main (self,
                                                 GDIX51C0_ACTION_IDENTIFY,
                                                 FPI_MATCH_FAIL,
                                                 NULL,
                                                 NULL);
                  report_sent = TRUE;
                }
              break;
            }
        }
    }

  if (reported_match && final_match_result_valid && probe)
    {
      g_autoptr(GError) study_error = NULL;
      g_autoptr(GVariant) print_data = NULL;

      updated_print = kind == GDIX51C0_ACTION_VERIFY ?
        verify_template : final_identify_match;
      g_object_get (G_OBJECT (updated_print), "fpi-data", &print_data, NULL);
      if (!goodix_chicago_runtime_study_print_data (
            probe, print_data, self->sensor_id, self->chicago_calibration,
            &final_match_result, &updated_data, &study_error))
        {
          fp_warn ("gdix51c0: templateStudy update failed after match: %s",
                   study_error ? study_error->message : "unknown error");
          updated_print = NULL;
        }
      else if (updated_data)
        {
          fp_dbg ("gdix51c0: templateStudy updated matched template");
        }
      else
        {
          updated_print = NULL;
          fp_dbg ("gdix51c0: matched scan was not eligible for templateStudy");
        }
    }

  if (!reported && !error)
    {
      reported = TRUE;
      if (!saw_valid_probe &&
          last_reject != GOODIX_CHICAGO_RUNTIME_REJECT_NONE)
        {
          fp_dbg ("gdix51c0: Chicago capture retries exhausted; reporting bad capture");
          gdix51c0_match_report_on_main (
            self, kind, FPI_MATCH_ERROR, NULL,
            fpi_device_retry_new_msg (
              FP_DEVICE_RETRY_GENERAL,
              "Fingerprint image quality is too low; try again."));
        }
      else
        gdix51c0_match_report_on_main (
          self, kind, FPI_MATCH_FAIL, NULL, NULL);
      report_sent = TRUE;
    }

out:
  g_clear_pointer (&probe, goodix_chicago_runtime_probe_free);
  transient_failure = gdix51c0_error_is_transient_verify_failure (error);
  if (transient_failure)
    {
      fp_warn ("gdix51c0: treating transient Chicago failure as no-match: %s",
               error ? error->message : "?");
      if (!reported)
        {
          reported = TRUE;
          gdix51c0_match_report_on_main (
            self, kind, FPI_MATCH_FAIL, NULL, NULL);
          report_sent = TRUE;
        }
    }

  if (saw_finger)
    {
      /* A finger may already cover the sensor while the cold init sequence
       * attempts T0.  Do not let a successful one-shot match skip the only
       * safe post-lift refresh opportunity for this hardware handle. */
      if (reported_match && !self->image_base_refreshed_this_open &&
          !self->session_desynced)
        lift_cleanup_ok = gdix51c0_wait_for_lift_report (
          self, GDIX51C0_LIFT_CLEANUP_TIMEOUT_USEC, FALSE);
      else if (reported_match || self->session_desynced)
        gdix51c0_report_finger_status_on_main (self, FP_FINGER_STATUS_NONE);
      else
        lift_cleanup_ok = gdix51c0_wait_for_lift_report (
          self, GDIX51C0_LIFT_CLEANUP_TIMEOUT_USEC, FALSE);
    }
  if (error || self->session_desynced || !lift_cleanup_ok)
    gdix51c0_session_deactivate (self);
  else if (!reported_match)
    fp_dbg ("gdix51c0: keeping TLS session active after clean no-match");

  if (transient_failure)
    g_clear_error (&error);
  gdix51c0_match_complete_on_main (self,
                                   kind,
                                   reported && !report_sent,
                                   final_verify_result,
                                   final_identify_match,
                                   updated_print,
                                   updated_data,
                                   error);
}

static void
gdix51c0_verify_sync (FpDevice *dev,
                      FpPrint  *verify_print)
{
  gdix51c0_verify_or_identify (dev, GDIX51C0_ACTION_VERIFY, verify_print, NULL);
}

static void
gdix51c0_identify_sync (FpDevice  *dev,
                        GPtrArray *identify_gallery)
{
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (dev);

  if (self->skip_next_identify)
    {
      self->skip_next_identify = FALSE;
      fp_dbg ("gdix51c0: skipping post-enrollment duplicate check");
      gdix51c0_match_complete_on_main (self,
                                       GDIX51C0_ACTION_IDENTIFY,
                                       TRUE,
                                       FPI_MATCH_FAIL,
                                       NULL,
                                       NULL,
                                       NULL,
                                       NULL);
      return;
    }

  gdix51c0_verify_or_identify (dev, GDIX51C0_ACTION_IDENTIFY, NULL, identify_gallery);
}

static void
gdix51c0_action_thread_free (Gdix51c0ActionThread *action)
{
  g_object_unref (action->dev);
  g_clear_pointer (&action->context, g_main_context_unref);
  g_clear_object (&action->cancellable);
  g_clear_object (&action->enroll_print);
  g_clear_object (&action->verify_print);
  g_clear_pointer (&action->identify_gallery, g_ptr_array_unref);
  g_free (action);
}

static gpointer
gdix51c0_action_thread (gpointer user_data)
{
  Gdix51c0ActionThread *action = user_data;
  FpDevice *dev = action->dev;
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (dev);

  switch (action->kind)
    {
    case GDIX51C0_ACTION_ENROLL:
      gdix51c0_enroll_sync (dev, action->enroll_print);
      break;

    case GDIX51C0_ACTION_VERIFY:
      gdix51c0_verify_sync (dev, action->verify_print);
      break;

    case GDIX51C0_ACTION_IDENTIFY:
      gdix51c0_identify_sync (dev, action->identify_gallery);
      break;
    }

  g_clear_pointer (&self->action_context, g_main_context_unref);
  g_clear_object (&self->action_cancellable);
  fp_dbg ("gdix51c0: action thread exited");

  gdix51c0_action_thread_free (action);
  return NULL;
}

static void
gdix51c0_run_action_thread (FpDevice          *dev,
                            Gdix51c0ActionKind kind,
                            const char        *name)
{
  Gdix51c0ActionThread *action = g_new0 (Gdix51c0ActionThread, 1);
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (dev);
  GThread *thread;
  GCancellable *cancellable;

  action->dev = g_object_ref (dev);
  action->kind = kind;
  action->context = g_main_context_get_thread_default ();
  if (action->context)
    g_main_context_ref (action->context);
  else
    action->context = g_main_context_ref (g_main_context_default ());

  cancellable = fpi_device_get_cancellable (dev);
  action->cancellable = cancellable ? g_object_ref (cancellable) : NULL;

  switch (kind)
    {
    case GDIX51C0_ACTION_ENROLL:
      {
        FpPrint *print = NULL;
        fpi_device_get_enroll_data (dev, &print);
        action->enroll_print = print ? g_object_ref (print) : NULL;
        break;
      }

    case GDIX51C0_ACTION_VERIFY:
      {
        FpPrint *print = NULL;
        fpi_device_get_verify_data (dev, &print);
        action->verify_print = print ? g_object_ref (print) : NULL;
        break;
      }

    case GDIX51C0_ACTION_IDENTIFY:
      {
        GPtrArray *gallery = NULL;

        fpi_device_get_identify_data (dev, &gallery);
        action->identify_gallery = g_ptr_array_new_with_free_func (g_object_unref);
        for (guint i = 0; gallery && i < gallery->len; i++)
          g_ptr_array_add (action->identify_gallery,
                           g_object_ref (g_ptr_array_index (gallery, i)));
        break;
      }
    }

  g_clear_pointer (&self->action_context, g_main_context_unref);
  g_clear_object (&self->action_cancellable);
  self->action_context = g_main_context_ref (action->context);
  self->action_cancellable = action->cancellable ? g_object_ref (action->cancellable) : NULL;

  thread = g_thread_new (name, gdix51c0_action_thread, action);
  g_thread_unref (thread);
}

static void
gdix51c0_suspend (FpDevice *dev)
{
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (dev);

  fp_dbg ("gdix51c0: suspend requested; parking active action until resume");
  self->session_desynced = TRUE;
  /* A power transition may discard the sensor-side bases. Keep the persisted
   * fallback, but require a new T0 before the next action just as the official
   * power-loss path clears base-inited. */
  gdix51c0_mark_cold_boundary (self, "active suspend");

  /* Do not complete suspend with an error here. libfprint would cancel the
   * action while the device is marked suspended; fprintd would then try to
   * close it immediately, and fp_device_close() correctly rejects every close
   * until resume. Keep the action parked across sleep and cancel it from the
   * resume callback, after libfprint has cleared the suspended state. */
  fpi_device_suspend_complete (dev, NULL);
}

static void
gdix51c0_resume (FpDevice *dev)
{
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (dev);
  g_autoptr(GCancellable) action_cancellable = NULL;

  fp_dbg ("gdix51c0: resume requested; cancelling parked action after resume");
  if (self->action_cancellable)
    action_cancellable = g_object_ref (self->action_cancellable);

  /* Clear libfprint's suspended state before the action completion reaches
   * fprintd. Its ensuing Release can then close the device normally instead
   * of leaving an open-but-unclaimed handle behind. */
  fpi_device_resume_complete (dev, NULL);

  if (action_cancellable)
    g_cancellable_cancel (action_cancellable);
}

static void
gdix51c0_enroll (FpDevice *dev)
{
  gdix51c0_run_action_thread (dev, GDIX51C0_ACTION_ENROLL, "gdix51c0-enroll");
}

static void
gdix51c0_verify (FpDevice *dev)
{
  gdix51c0_run_action_thread (dev, GDIX51C0_ACTION_VERIFY, "gdix51c0-verify");
}

static void
gdix51c0_identify (FpDevice *dev)
{
  gdix51c0_run_action_thread (dev, GDIX51C0_ACTION_IDENTIFY, "gdix51c0-identify");
}

/* ------------------------------------------------------------------ */
/* GObject boilerplate                                                 */
/* ------------------------------------------------------------------ */

static void
fpi_device_gdix51c0_init (FpiDeviceGdix51c0 *self)
{
  self->spi_fd = -1;
  self->image_base_refreshed_this_open = FALSE;
}

static void
fpi_device_gdix51c0_finalize (GObject *gobject)
{
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (gobject);

  gdix51c0_session_deactivate (self);
  g_clear_pointer (&self->action_context, g_main_context_unref);
  g_clear_object (&self->action_cancellable);
  G_OBJECT_CLASS (fpi_device_gdix51c0_parent_class)->finalize (gobject);
}

static void
fpi_device_gdix51c0_class_init (FpiDeviceGdix51c0Class *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);

  dev_class->id               = "gdix51c0";
  dev_class->full_name        = "Goodix GDIX51C0 Fingerprint Sensor";
  dev_class->type             = FP_DEVICE_TYPE_UDEV;
  dev_class->id_table         = gdix51c0_id_table;
  dev_class->scan_type        = FP_SCAN_TYPE_PRESS;
  dev_class->nr_enroll_stages =
    GOODIX_CHICAGO_ENGINE_REQUIRED_SAMPLES;
  dev_class->temp_hot_seconds = -1;
  dev_class->open             = gdix51c0_open;
  dev_class->close            = gdix51c0_close;
  dev_class->enroll           = gdix51c0_enroll;
  dev_class->verify           = gdix51c0_verify;
  dev_class->identify         = gdix51c0_identify;
  dev_class->suspend          = gdix51c0_suspend;
  dev_class->resume           = gdix51c0_resume;

  fpi_device_class_auto_initialize_features (dev_class);

  G_OBJECT_CLASS (klass)->finalize = fpi_device_gdix51c0_finalize;
}
