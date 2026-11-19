// SPDX-License-Identifier: LGPL-2.1-or-later
/* Copyright (C) 2026 Berke Kabagöz <berkekbgz@gmail.com> */

/*
 * Goodix GDIX51C0 wire protocol — implementation.
 *
 * Synchronous helpers used during open/activate.  We use libfprint's
 * fpi_spi_transfer_submit_sync for the actual SPI traffic so the driver
 * stays well-behaved under cancellation.
 */

#define FP_COMPONENT "gdix51c0"

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/spi/spidev.h>

#include "drivers_api.h"
#include "fpi-spi-transfer.h"

#include "../../lib/goodix/common/goodix-wire.h"
#include "gdix51c0.h"
#include "gdix51c0-proto.h"
#include "gdix51c0-wb.h"

/* GDIX51C0_QUIRK_* flags of the bound id-table entry; one device per process. */
static guint64 proto_quirks;

void
gdix51c0_proto_set_quirks (guint64 quirks)
{
  proto_quirks = quirks;
}

/* GDIX51C0_QUIRK_SLOW_TURNAROUND: a write sent within microseconds of a read
 * (or within a few ms of the previous reply, while the MCU is still busy) is
 * lost and desynchronises the next one, and a same-IRQ response can come
 * well after its ACK while IRQ stays high, so idle headers are polled longer. */
#define GDIX51C0_READ_TO_WRITE_GAP_USEC  2000
#define GDIX51C0_LOST_COMMAND_BACKOFF_USEC 20000
#define GDIX51C0_HEADER_TO_BODY_USEC     300
#define GDIX51C0_SLOW_IDLE_POLL_USEC     5000
#define GDIX51C0_SLOW_IDLE_BUDGET_USEC   (1000 * 1000)

static gint64 last_read_end_usec;

/* ---------------- packet construction ---------------- */

guint8
gdix51c0_payload_checksum (const guint8 *buf, gsize len)
{
  return goodix_wire_checksum (buf, len);
}

gboolean
gdix51c0_header_checksum_ok (const guint8 *hdr4)
{
  return goodix_wire_decode_outer_header (hdr4, NULL, NULL);
}

guint8 *
gdix51c0_make_header_packet (guint8 type, guint16 payload_len)
{
  guint8 *pkt = g_malloc (GOODIX_WIRE_OUTER_HEADER_SIZE);

  goodix_wire_encode_outer_header (type, payload_len, pkt);
  return pkt;
}

guint8 *
gdix51c0_make_payload_packet (guint8 type, const guint8 *data, gsize data_len,
                           gsize *out_len)
{
  g_return_val_if_fail (data_len <= G_MAXUINT16 - 1, NULL);

  gsize total = data_len + GOODIX_WIRE_COMMAND_OVERHEAD;
  guint8 *pkt = g_malloc (total);

  if (!goodix_wire_encode_command (type, data, data_len, pkt, total,
                                   out_len))
    {
      g_free (pkt);
      return NULL;
    }
  return pkt;
}

/* ---------------- SPI sync I/O ---------------- */

/* One write as a full-duplex transfer. With GDIX51C0_QUIRK_SLOW_TURNAROUND the
 * HC32F460 intermittently loses commands sent with half-duplex write(), while
 * full-duplex transfers (as the reads use) went through without loss. */
static ssize_t
gdix51c0_spi_write_duplex (int spi_fd, const guint8 *data, gsize len)
{
  g_autofree guint8 *rx = g_malloc (len);
  struct spi_ioc_transfer tr = {
    .tx_buf = (uintptr_t) data,
    .rx_buf = (uintptr_t) rx,
    .len = len,
    .bits_per_word = 8,
  };

  return ioctl (spi_fd, SPI_IOC_MESSAGE (1), &tr) < 1 ? -1 : (ssize_t) len;
}

static ssize_t
gdix51c0_spi_write_raw (int spi_fd, const guint8 *data, gsize len)
{
  if (proto_quirks & GDIX51C0_QUIRK_SLOW_TURNAROUND)
    return gdix51c0_spi_write_duplex (spi_fd, data, len);
  return write (spi_fd, data, len);
}

gboolean
gdix51c0_spi_write (FpDevice *dev, int spi_fd,
                 guint8 outer_type,
                 const guint8 *payload, gsize payload_len,
                 GError **error)
{
  /* Mirror python-spidev's writebytes() exactly: one ioctl per buffer.
   * Header and payload as two SPI transactions (two CS cycles).        */
  (void) dev;
  if (proto_quirks & GDIX51C0_QUIRK_SLOW_TURNAROUND)
    {
      gint64 since = g_get_monotonic_time () - last_read_end_usec;
      if (since < GDIX51C0_READ_TO_WRITE_GAP_USEC)
        g_usleep (GDIX51C0_READ_TO_WRITE_GAP_USEC - since);
    }
  g_autofree guint8 *hdr = gdix51c0_make_header_packet (outer_type, (guint16) payload_len);
  ssize_t n = gdix51c0_spi_write_raw (spi_fd, hdr, 4);
  if (n != 4)
    {
      g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: write header: %zd/%d", n, 4);
      return FALSE;
    }
  if (payload_len == 0)
    return TRUE;
  /* The HC32 needs a moment after the header to arm reception of the body;
   * a body clocked out within microseconds is lost with its command. */
  if (proto_quirks & GDIX51C0_QUIRK_SLOW_TURNAROUND)
    g_usleep (GDIX51C0_HEADER_TO_BODY_USEC);
  n = gdix51c0_spi_write_raw (spi_fd, payload, payload_len);
  if ((gsize) n != payload_len)
    {
      g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: write payload: %zd/%zu", n, payload_len);
      return FALSE;
    }
  return TRUE;
}

static gboolean
gdix51c0_spi_xfer_read (int spi_fd,
                        guint8 *rx,
                        gsize len,
                        GError **error)
{
  gsize done = 0;

  while (done < len)
    {
      gsize chunk = MIN (len - done, (gsize) 2048);
      g_autofree guint8 *tx = g_malloc (chunk);

      memset (tx, 0xff, chunk);

      struct spi_ioc_transfer tr = {
        .tx_buf = (uintptr_t) tx,
        .rx_buf = (uintptr_t) (rx + done),
        .len = chunk,
        .delay_usecs = 0,
        .speed_hz = 0,       /* use fd-configured speed */
        .bits_per_word = 8,
      };

      int ret = ioctl (spi_fd, SPI_IOC_MESSAGE (1), &tr);
      if (ret < 1)
        {
          g_set_error (error,
                       G_IO_ERROR,
                       g_io_error_from_errno (errno),
                       "gdix51c0: SPI_IOC_MESSAGE read failed at %zu/%zu",
                       done,
                       len);
          return FALSE;
        }

      done += chunk;
    }

  last_read_end_usec = g_get_monotonic_time ();
  return TRUE;
}

/* The longest packet the sensor sends is an image frame: 7680 bytes of raw
 * pixels plus the image prefix and TLS record overhead, just under 8 KiB.
 * Twice that covers one frame and a packet queued behind it. */
#define GDIX51C0_DRAIN_CHUNK     64
#define GDIX51C0_DRAIN_MAX_BYTES (16 * 1024)

/* A header that fails its checksum has an untrustworthy length, but the rest
 * of the real packet (and anything queued after it) is still waiting on the
 * bus.  Clock it out until the sensor returns idle filler, so the next read
 * starts at a packet boundary.  The callers then retry like a lost packet.
 * Returns the number of bytes discarded. */
static gsize
gdix51c0_spi_drain (int spi_fd)
{
  guint8 buf[GDIX51C0_DRAIN_CHUNK];
  gsize drained = 0;

  while (drained < GDIX51C0_DRAIN_MAX_BYTES)
    {
      gboolean idle = TRUE;

      if (!gdix51c0_spi_xfer_read (spi_fd, buf, sizeof (buf), NULL))
        break;
      drained += sizeof (buf);

      for (gsize i = 0; i < sizeof (buf); i++)
        {
          if (buf[i] != buf[0] || (buf[0] != 0x00 && buf[0] != 0xff))
            {
              idle = FALSE;
              break;
            }
        }

      if (idle)
        break;
    }

  return drained;
}

guint8 *
gdix51c0_spi_read (FpDevice *dev, int spi_fd, gsize *out_len, GError **error)
{
  return gdix51c0_spi_read_typed (dev, spi_fd, NULL, out_len, error);
}

guint8 *
gdix51c0_spi_read_typed (FpDevice *dev, int spi_fd,
                         guint8 *out_type,
                         gsize *out_len, GError **error)
{
  /*
   * Clock reads as full-duplex transfers filled with 0xff. Do not use plain
   * read(). On this platform, half-duplex read timing can
   * produce corrupted/truncated payload bytes, which makes decrypted image
   * frames look like noise even when the command sequence is correct.
   */
  (void) dev;

  guint8 hdr[4];
  gboolean saw_all_zero = FALSE;
  gboolean saw_all_ff = FALSE;
  const gboolean slow = (proto_quirks & GDIX51C0_QUIRK_SLOW_TURNAROUND) != 0;
  const gint64 slow_deadline = g_get_monotonic_time () + GDIX51C0_SLOW_IDLE_BUDGET_USEC;

  for (guint attempt = 0; ; attempt++)
    {
      if (!gdix51c0_spi_xfer_read (spi_fd, hdr, sizeof (hdr), error))
        return NULL;

      saw_all_zero = hdr[0] == 0 && hdr[1] == 0 && hdr[2] == 0 && hdr[3] == 0;
      saw_all_ff = hdr[0] == 0xff && hdr[1] == 0xff && hdr[2] == 0xff && hdr[3] == 0xff;

      if (!saw_all_zero && !saw_all_ff)
        break;

      if (slow ? g_get_monotonic_time () >= slow_deadline : attempt + 1 == 4)
        break;

      if (!slow || attempt < 4)
        fp_dbg ("gdix51c0: SPI read saw idle %s header, retrying",
                saw_all_ff ? "all-FF" : "all-zero");
      g_usleep (slow ? GDIX51C0_SLOW_IDLE_POLL_USEC : 5000 + attempt * 10000);
    }

  if (saw_all_zero)
    {
      g_set_error_literal (error,
                           G_IO_ERROR,
                           G_IO_ERROR_FAILED,
                           "gdix51c0: read returned all-zero header");
      return NULL;
    }

  guint16 length = (guint16) hdr[1] | ((guint16) hdr[2] << 8);

  if (saw_all_ff || length == 0xffff)
    {
      g_set_error_literal (error,
                           G_IO_ERROR,
                           G_IO_ERROR_FAILED,
                           "gdix51c0: read returned all-FF header");
      return NULL;
    }

  /* Do not trust the length of a header that fails its checksum.  Observed
   * corrupt headers are mostly real ones shifted by idle bytes (00 a0 06 00,
   * 00 00 a0 06), whose "length" is really the packet type: reading it would
   * swallow up to 64 KiB, including the packets queued behind it. */
  if (!gdix51c0_header_checksum_ok (hdr))
    {
      gsize drained = gdix51c0_spi_drain (spi_fd);

      fp_warn ("gdix51c0: header checksum mismatch (%02x %02x %02x %02x); "
               "discarded %zu B", hdr[0], hdr[1], hdr[2], hdr[3], drained);
      g_set_error (error,
                   G_IO_ERROR,
                   G_IO_ERROR_INVALID_DATA,
                   "gdix51c0: header checksum mismatch "
                   "(%02x %02x %02x %02x)",
                   hdr[0], hdr[1], hdr[2], hdr[3]);
      return NULL;
    }

  /* g_malloc (0) returns NULL, which every caller treats as a failure and
   * then reads error->message.  A header can pass its checksum and still
   * advertise zero length (a0 00 00 a0), so report it instead of returning
   * NULL with no error set. */
  if (length == 0)
    {
      g_set_error (error,
                   G_IO_ERROR,
                   G_IO_ERROR_INVALID_DATA,
                   "gdix51c0: read returned zero-length header "
                   "(%02x %02x %02x %02x)",
                   hdr[0], hdr[1], hdr[2], hdr[3]);
      return NULL;
    }

  /* g_malloc (0) returns NULL, which every caller treats as a failure and
   * then reads error->message.  A corrupted header such as 00 00 00 a0 gets
   * here, so report it instead of returning NULL with no error set. */
  if (length == 0)
    {
      g_set_error (error,
                   G_IO_ERROR,
                   G_IO_ERROR_INVALID_DATA,
                   "gdix51c0: read returned zero-length header "
                   "(%02x %02x %02x %02x)",
                   hdr[0], hdr[1], hdr[2], hdr[3]);
      return NULL;
    }

  /* ACK and response packets may be queued back-to-back in one IRQ-high
   * window. Read exactly the advertised length so this transfer cannot consume
   * the following packet's header. */
  guint8 *payload = g_malloc (length);
  if (!gdix51c0_spi_xfer_read (spi_fd, payload, length, error))
    {
      g_free (payload);
      return NULL;
    }

  if (out_len)
    *out_len = length;

  if (out_type)
    *out_type = hdr[0];

  return payload;
}

/* ---------------- IRQ + reset ---------------- */

/* On boards whose firmware locks the IRQ pad as level-triggered, the kernel
 * reports a RISING edge on every interrupt while the line is high and never a
 * FALLING one. Buffered rising events outlive the level that produced them, so
 * an edge wait for the next response fires on the previous one's leftovers.
 * In level mode, events only wake the waiter and the line level decides. */
#define GDIX51C0_IRQ_LEVEL_POLL_USEC 1000

static gboolean
gdix51c0_irq_wait_level (struct gpiod_line_request     *irq_req,
                         struct gpiod_edge_event_buffer *event_buf,
                         gboolean                        target_high,
                         unsigned int                    irq_offset,
                         gint64                          deadline,
                         const char                     *label,
                         GError                        **error)
{
  for (;;)
    {
      enum gpiod_line_value value =
        gpiod_line_request_get_value (irq_req, irq_offset);

      if (value < 0)
        {
          g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                       "gdix51c0: %s: get IRQ value failed", label);
          return FALSE;
        }
      if ((value == GPIOD_LINE_VALUE_ACTIVE) == target_high)
        return TRUE;

      gint64 now = g_get_monotonic_time ();
      if (now >= deadline)
        {
          g_set_error (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                       "gdix51c0: %s: IRQ level never went %s",
                       label, target_high ? "high" : "low");
          return FALSE;
        }

      gint64 slice = MIN (deadline - now, GDIX51C0_IRQ_LEVEL_POLL_USEC);
      int ready = gpiod_line_request_wait_edge_events (irq_req, slice * 1000);
      if (ready < 0)
        {
          g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                       "gdix51c0: %s: wait_edge_events failed", label);
          return FALSE;
        }
      if (ready > 0)
        gpiod_line_request_read_edge_events (irq_req, event_buf, 16);
    }
}

void
gdix51c0_irq_drain (struct gpiod_line_request *irq_req,
                 struct gpiod_edge_event_buffer *event_buf)
{
  if (!irq_req || !event_buf)
    return;
  /* Non-blocking poll for any already-buffered edges; discard. */
  while (gpiod_line_request_wait_edge_events (irq_req, 0) > 0)
    gpiod_line_request_read_edge_events (irq_req, event_buf, 16);
}

gboolean
gdix51c0_irq_wait (struct gpiod_line_request *irq_req,
                   struct gpiod_edge_event_buffer *event_buf,
                   gboolean target_high,
                   unsigned int irq_offset,
                   guint timeout_usec,
                   const char *label,
                   GError **error)
{
  enum gpiod_line_value value;
  enum gpiod_edge_event_type want = target_high
                                    ? GPIOD_EDGE_EVENT_RISING_EDGE
                                    : GPIOD_EDGE_EVENT_FALLING_EDGE;
  gint64 deadline = g_get_monotonic_time () + (gint64) timeout_usec;

  if (!irq_req || !event_buf)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_CLOSED,
                    "gdix51c0: %s: IRQ line is closed", label);
      return FALSE;
    }

  /* Critical fix: if the line is already at the desired level,
   * do not wait for an edge that has already happened. */
  value = gpiod_line_request_get_value (irq_req, irq_offset);
  if (value < 0)
    {
      g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: %s: get IRQ value failed", label);
      return FALSE;
    }

  if ((value == GPIOD_LINE_VALUE_ACTIVE) == target_high)
    return TRUE;

  if (proto_quirks & GDIX51C0_QUIRK_LEVEL_IRQ)
    return gdix51c0_irq_wait_level (irq_req, event_buf, target_high,
                                    irq_offset, deadline, label, error);

  for (;;)
    {
      gint64 now = g_get_monotonic_time ();

      if (now >= deadline)
        break;

      int ready = gpiod_line_request_wait_edge_events (
        irq_req, (deadline - now) * 1000);

      if (ready < 0)
        {
          g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                       "gdix51c0: %s: wait_edge_events failed", label);
          return FALSE;
        }

      if (ready == 0)
        break;

      int n = gpiod_line_request_read_edge_events (irq_req, event_buf, 1);
      if (n < 0)
        {
          g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                       "gdix51c0: %s: read_edge_events failed", label);
          return FALSE;
        }

      for (int i = 0; i < n; i++)
        {
          struct gpiod_edge_event *ev =
            gpiod_edge_event_buffer_get_event (event_buf, i);

          if (ev && gpiod_edge_event_get_event_type (ev) == want)
            return TRUE;
        }

      /* Also re-check physical level after any edge. This handles cases
       * where gpiod delivered the opposite edge from an old transition,
       * but the line is now already in the target state. */
      value = gpiod_line_request_get_value (irq_req, irq_offset);
      if (value < 0)
        {
          g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                       "gdix51c0: %s: get IRQ value failed", label);
          return FALSE;
        }

      if ((value == GPIOD_LINE_VALUE_ACTIVE) == target_high)
        return TRUE;
    }

  value = gpiod_line_request_get_value (irq_req, irq_offset);
  if (value >= 0)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                   "gdix51c0: %s: IRQ never went %s; final level=%s",
                   label,
                   target_high ? "high" : "low",
                   value == GPIOD_LINE_VALUE_ACTIVE ? "high" : "low");
    }
  else
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                   "gdix51c0: %s: IRQ never went %s",
                   label,
                   target_high ? "high" : "low");
    }

  return FALSE;
}

gboolean
gdix51c0_reset_pulse (struct gpiod_chip *chip, unsigned int offset, GError **error)
{
  if (!chip)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_CLOSED,
                          "gdix51c0: reset requested but gpio chip is closed");
      return FALSE;
    }
  /* Mirror python: open the line as OUTPUT(0), write 1, sleep, write 0,
   * sleep, then RELEASE the request entirely.  The release lets the
   * kernel return the line to its default state (INPUT/high-Z), and
   * the platform pull-up brings the MCU out of reset.  Reconfiguring
   * an existing OUTPUT(0) request to INPUT is NOT equivalent — the
   * kernel keeps remembering the OUTPUT-low intent on this platform. */
  struct gpiod_line_settings  *settings = gpiod_line_settings_new ();
  struct gpiod_line_config    *cfg      = gpiod_line_config_new ();
  struct gpiod_request_config *req_cfg  = gpiod_request_config_new ();
  struct gpiod_line_request   *req      = NULL;
  gboolean ok = FALSE;

  if (!settings || !cfg || !req_cfg)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: gpiod alloc failed");
      goto out;
    }

  gpiod_line_settings_set_direction    (settings, GPIOD_LINE_DIRECTION_OUTPUT);
  gpiod_line_settings_set_output_value (settings, GPIOD_LINE_VALUE_INACTIVE);
  if (gpiod_line_config_add_line_settings (cfg, &offset, 1, settings) < 0)
    {
      g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: reset add line %u failed", offset);
      goto out;
    }
  gpiod_request_config_set_consumer (req_cfg, "gdix51c0-reset");

  req = gpiod_chip_request_lines (chip, req_cfg, cfg);
  if (!req)
    {
      g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: reset request line %u failed", offset);
      goto out;
    }

  gpiod_line_request_set_value (req, offset, GPIOD_LINE_VALUE_ACTIVE);
  g_usleep (10000);
  gpiod_line_request_set_value (req, offset, GPIOD_LINE_VALUE_INACTIVE);
  g_usleep (10000);

  ok = TRUE;
out:
  if (req)      gpiod_line_request_release (req);
  if (settings) gpiod_line_settings_free (settings);
  if (cfg)      gpiod_line_config_free (cfg);
  if (req_cfg)  gpiod_request_config_free (req_cfg);
  return ok;
}

guint8
gdix51c0_payload_checksum_ts (const guint8 *buf, gsize len)
{
  return (guint8) (gdix51c0_payload_checksum (buf, len) + 1);
}

/* ---------------- 14213 init-sequence helpers ---------------- */

#define GDIX51C0_CMD_SEND_DELAY_USEC 1000
#define GDIX51C0_CMD_ACK_TIMEOUT_USEC (1000 * 1000)
#define GDIX51C0_CMD_MIN_RESPONSE_TIMEOUT_USEC (1000 * 1000)
#define GDIX51C0_CMD_RECOVERY_TIMEOUT_USEC (100 * 1000)
#define GDIX51C0_CMD_ATTEMPTS 2
#ifdef GOODIX_SPI_DEVELOPER
#define GDIX51C0_FAULT_SYNC_ACK_EXHAUST_ONCE_ENV \
  "GDIX51C0_FAULT_SYNC_ACK_EXHAUST_ONCE"
#define GDIX51C0_FAULT_EVK_FAILURES_ONCE_ENV \
  "GDIX51C0_FAULT_EVK_FAILURES_ONCE"

/* Synchronous initialization runs before a listener object exists.  Keep this
 * acceptance hook process-wide so exhausting both ACK attempts can fail the
 * first activation while the retry in the same daemon proceeds normally. */
static gint fault_sync_ack_exhaust_used;
static gint fault_evk_failures_claimed;

static gboolean
gdix51c0_cmd_arm_sync_ack_exhaust_once (const char *label)
{
  const char *target = g_getenv (GDIX51C0_FAULT_SYNC_ACK_EXHAUST_ONCE_ENV);

  if (!target || !*target || g_strcmp0 (target, label) != 0)
    return FALSE;

  return g_atomic_int_compare_and_exchange (&fault_sync_ack_exhaust_used, 0, 1);
}

/* Claim one complete EVK helper invocation for required-data exhaustion.
 * Setting the test value to 2 forces GetEvkVersionWithRetry to reach its third
 * outer attempt after two hard-reset + 500 ms recovery transitions. */
static guint
gdix51c0_cmd_claim_evk_failure (const char *label,
                                guint      *out_limit)
{
  const char *value = g_getenv (GDIX51C0_FAULT_EVK_FAILURES_ONCE_ENV);
  gint limit;

  if (g_strcmp0 (label, "get-evk-version") != 0 || !value || !*value)
    return 0;

  limit = (gint) CLAMP (g_ascii_strtoll (value, NULL, 0), 0, 2);
  if (out_limit)
    *out_limit = (guint) limit;

  for (;;)
    {
      gint claimed = g_atomic_int_get (&fault_evk_failures_claimed);

      if (claimed >= limit)
        return 0;
      if (g_atomic_int_compare_and_exchange (&fault_evk_failures_claimed,
                                             claimed,
                                             claimed + 1))
        return (guint) claimed + 1;
    }
}
#endif

static gboolean gdix51c0_read_and_drop (Gdix51c0Bus *bus,
                                        guint8        expected_command,
                                        const char   *label,
                                        GError      **error);

/* gfspi!SpiSendDataToDevice sleeps for 1 ms before its initial submit.
 * Commands which expect an ACK are then allowed 1000 ms and are submitted
 * once more, immediately, if the ACK is missing.  SpiSendDataToDeviceLock
 * separately invokes that whole lower layer again when a required data
 * response is missing.  Keep both retry levels inside the command boundary:
 * escalating one lost IRQ directly to a full sensor reset is observably
 * different from Windows. */
static void
gdix51c0_cmd_prepare_send (Gdix51c0Bus *bus)
{
  g_usleep (GDIX51C0_CMD_SEND_DELAY_USEC);
  gdix51c0_irq_drain (bus->irq_req, bus->irq_events);
}

static void
gdix51c0_cmd_prepare_response_retry (Gdix51c0Bus *bus,
                                     const char  *label,
                                     const GError *reason)
{
  g_autofree char *idle_label = g_strdup_printf ("%s retry-idle", label);

  fp_dbg ("gdix51c0: %s response wait failed (%s); resubmitting command "
          "once like SpiSendDataToDeviceLock",
          label,
          reason ? reason->message : "?");

  /* A failed read can leave a late falling edge queued.  Windows resets the
   * command event before its resend; on Linux the equivalent is a short
   * best-effort return-to-idle followed by draining old edge notifications. */
  gdix51c0_irq_wait (bus->irq_req, bus->irq_events, FALSE,
                     bus->irq_offset, GDIX51C0_CMD_RECOVERY_TIMEOUT_USEC,
                     idle_label, NULL);
  gdix51c0_irq_drain (bus->irq_req, bus->irq_events);
}

/* Reproduce one SpiSendDataToDevice invocation.  The initial submit gets the
 * 1 ms delay; a missed ACK gets exactly one immediate physical resend. */
static gboolean
gdix51c0_cmd_send_and_ack (Gdix51c0Bus *bus,
                           const guint8 *payload,
                           gsize         len,
                           gboolean      wait_for_fall,
                           const char   *label,
                           GError      **error)
{
  g_autofree char *ack_rise = g_strdup_printf ("%s ack-rise", label);
  g_autofree char *ack_fall = g_strdup_printf ("%s ack-fall", label);
#ifdef GOODIX_SPI_DEVELOPER
  gboolean exhaust_acks = gdix51c0_cmd_arm_sync_ack_exhaust_once (label);

  if (exhaust_acks)
    fp_warn ("gdix51c0: %s armed one-shot synchronous ACK-exhaustion injection",
             label);
#endif

  gdix51c0_cmd_prepare_send (bus);

  for (guint attempt = 1; attempt <= GDIX51C0_CMD_ATTEMPTS; attempt++)
    {
      g_autoptr(GError) local_error = NULL;
      gboolean received_ack;

      if (!gdix51c0_spi_write (bus->dev, bus->spi_fd,
                               GDIX51C0_PKT_WRITE, payload, len, error))
        return FALSE;

      received_ack =
        gdix51c0_irq_wait (bus->irq_req, bus->irq_events, TRUE,
                           bus->irq_offset,
                           GDIX51C0_CMD_ACK_TIMEOUT_USEC,
                           ack_rise, &local_error) &&
        gdix51c0_read_and_drop (bus, GDIX51C0_PKT_READ,
                                label, &local_error) &&
        (!wait_for_fall ||
         gdix51c0_irq_wait (bus->irq_req, bus->irq_events, FALSE,
                            bus->irq_offset,
                            GDIX51C0_CMD_ACK_TIMEOUT_USEC,
                            ack_fall, &local_error));

#ifdef GOODIX_SPI_DEVELOPER
      if (received_ack && exhaust_acks)
        {
          fp_warn ("gdix51c0: fault injection dropped %s ACK on transport "
                   "attempt %u/%u",
                   label, attempt, GDIX51C0_CMD_ATTEMPTS);
          g_usleep (GDIX51C0_CMD_ACK_TIMEOUT_USEC);
          g_set_error (&local_error,
                       G_IO_ERROR,
                       G_IO_ERROR_TIMED_OUT,
                       "gdix51c0: injected %s ACK timeout on attempt %u/%u",
                       label, attempt, GDIX51C0_CMD_ATTEMPTS);
          received_ack = FALSE;
        }
#endif

      if (received_ack)
        return TRUE;

      if (attempt < GDIX51C0_CMD_ATTEMPTS)
        {
          fp_dbg ("gdix51c0: %s ACK attempt 1/2 failed (%s); "
                  "same-command resend",
                  label, local_error ? local_error->message : "?");
          /* A command lost while the HC32 is still busy is lost again if
           * resent at once; give it time to finish first. */
          if (proto_quirks & GDIX51C0_QUIRK_SLOW_TURNAROUND)
            g_usleep (GDIX51C0_LOST_COMMAND_BACKOFF_USEC);
          continue;
        }

      g_propagate_error (error, g_steal_pointer (&local_error));
      return FALSE;
    }

  g_assert_not_reached ();
}

/* GF_HC460SEC_APP_14103 ACKs commands that 14213 answers without one: 0xd5
 * (ACK ~13 ms later) and single-response commands such as 0xaf, whose ACK
 * precedes the response. An ACK names the command with bit 0 cleared. */
#define GDIX51C0_EXTRA_ACK_WINDOW_USEC 50000

static gboolean
gdix51c0_is_ack_for (const guint8 *buf, gsize len, guint8 command)
{
  return len >= 4 && buf[0] == GDIX51C0_PKT_READ &&
         (buf[3] & 0xfe) == (command & 0xfe);
}

gboolean
gdix51c0_proto_is_extra_ack (const guint8 *buf, gsize len)
{
  /* A TLS record starts with its content type (0x14..0x17), never 0xb0. */
  return (proto_quirks & GDIX51C0_QUIRK_EXTRA_ACKS) && len >= 4 &&
         buf[0] == GDIX51C0_PKT_READ;
}

/* Read and drop a reply to a command the caller treats as no-ACK. Left unread
 * it holds IRQ high, and the next command takes it for its own reply. */
static void
gdix51c0_absorb_extra_ack (Gdix51c0Bus *bus, guint8 command, const char *label)
{
  g_autoptr(GError) local_error = NULL;
  g_autofree guint8 *buf = NULL;
  gsize n = 0;

  if (!gdix51c0_irq_wait (bus->irq_req, bus->irq_events, TRUE, bus->irq_offset,
                          GDIX51C0_EXTRA_ACK_WINDOW_USEC, label, NULL))
    return;

  buf = gdix51c0_spi_read (bus->dev, bus->spi_fd, &n, &local_error);
  if (!buf)
    fp_warn ("gdix51c0: %s: reading extra ACK failed: %s",
             label, local_error->message);
  else if (gdix51c0_is_ack_for (buf, n, command))
    fp_dbg ("gdix51c0: %s: dropped firmware ACK", label);
  else
    fp_warn ("gdix51c0: %s: dropped unexpected packet 0x%02x (%zu B)",
             label, buf[0], n);

  gdix51c0_irq_wait (bus->irq_req, bus->irq_events, FALSE, bus->irq_offset,
                     GDIX51C0_EXTRA_ACK_WINDOW_USEC, label, NULL);
}

gboolean
gdix51c0_cmd_no_ack (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                  const char *label, GError **error)
{
  fp_dbg ("gdix51c0: %s (no-ack, %zu B)", label, len);
  gdix51c0_cmd_prepare_send (bus);
  if (!gdix51c0_spi_write (bus->dev, bus->spi_fd, GDIX51C0_PKT_WRITE,
                           payload, len, error))
    return FALSE;
  if (proto_quirks & GDIX51C0_QUIRK_EXTRA_ACKS)
    gdix51c0_absorb_extra_ack (bus, payload[0], label);
  return TRUE;
}

static gboolean
gdix51c0_read_and_drop (Gdix51c0Bus *bus,
                        guint8        expected_command,
                        const char   *label,
                        GError      **error)
{
  gsize n = 0;
  g_autofree guint8 *buf = gdix51c0_spi_read (bus->dev, bus->spi_fd, &n, error);
  if (!buf)
    return FALSE;

  if (n == 0 || buf[0] != expected_command)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                   "gdix51c0: %s expected packet 0x%02x, got 0x%02x (%zu B)",
                   label, expected_command, n > 0 ? buf[0] : 0, n);
      return FALSE;
    }

  fp_dbg ("gdix51c0: %s read %zu B", label, n);
  return TRUE;
}

gboolean
gdix51c0_cmd_ack (Gdix51c0Bus *bus,
                  const guint8 *payload,
                  gsize len,
                  const char *label,
                  GError **error)
{
  return gdix51c0_cmd_send_and_ack (bus, payload, len, TRUE, label, error);
}

gboolean
gdix51c0_cmd_ack_resp (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                    const char *label, GError **error)
{
  fp_dbg ("gdix51c0: %s (ack then resp, %zu B)", label, len);
  if (!(proto_quirks & GDIX51C0_QUIRK_EXTRA_ACKS))
    return gdix51c0_cmd_ack_then_resp (bus, payload, len, label, error);

  /* fw 14103 answers the 0xe0 PSK write with an accepted ACK and no response
   * within a second; the caller verifies the write by reading it back. */
  if (!gdix51c0_cmd_send_and_ack (bus, payload, len, TRUE, label, error))
    return FALSE;
  if (!gdix51c0_irq_wait (bus->irq_req, bus->irq_events, TRUE, bus->irq_offset,
                          GDIX51C0_CMD_MIN_RESPONSE_TIMEOUT_USEC, label, NULL))
    {
      fp_dbg ("gdix51c0: %s: ACK only, no response", label);
      return TRUE;
    }

  gsize n = 0;
  g_autofree guint8 *response = gdix51c0_spi_read (bus->dev, bus->spi_fd, &n, error);
  if (!response)
    return FALSE;
  if (n == 0 || response[0] != payload[0])
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                   "gdix51c0: %s expected response 0x%02x, got 0x%02x (%zu B)",
                   label, payload[0], n > 0 ? response[0] : 0, n);
      return FALSE;
    }
  return gdix51c0_irq_wait (bus->irq_req, bus->irq_events, FALSE, bus->irq_offset,
                            GDIX51C0_CMD_MIN_RESPONSE_TIMEOUT_USEC, label, error);
}

guint8 *
gdix51c0_cmd_read (Gdix51c0Bus *bus,
                   const guint8 *payload,
                   gsize len,
                   guint timeout_usec,
                   gsize *out_len,
                   const char *label,
                   GError **error)
{
  fp_dbg ("gdix51c0: %s (read, %zu B)", label, len);
  return gdix51c0_cmd_single_resp (bus, payload, len, timeout_usec,
                                   out_len, label, error);
}

gboolean
gdix51c0_cmd_ack_then_resp (Gdix51c0Bus *bus,
                            const guint8 *payload,
                            gsize len,
                            const char *label,
                            GError **error)
{
  gsize response_len = 0;
  g_autofree guint8 *response =
    gdix51c0_cmd_ack_then_resp_read (bus, payload, len,
                                     GDIX51C0_CMD_MIN_RESPONSE_TIMEOUT_USEC,
                                     &response_len, label, error);

  return response != NULL;
}

guint8 *
gdix51c0_cmd_ack_then_resp_read (Gdix51c0Bus *bus,
                                 const guint8 *payload,
                                 gsize len,
                                 guint timeout_usec,
                                 gsize *out_len,
                                 const char *label,
                                 GError **error)
{
  g_autofree char *resp_rise = g_strdup_printf ("%s resp-rise", label);
  g_autofree char *resp_fall = g_strdup_printf ("%s resp-fall", label);

  guint response_timeout_usec =
    MAX (timeout_usec, GDIX51C0_CMD_MIN_RESPONSE_TIMEOUT_USEC);

  fp_dbg ("gdix51c0: %s (ack then resp read, %zu B)", label, len);

  for (guint attempt = 1; attempt <= GDIX51C0_CMD_ATTEMPTS; attempt++)
    {
      g_autoptr(GError) local_error = NULL;
      g_autofree guint8 *buf = NULL;
      gsize response_len = 0;

      if (!gdix51c0_cmd_send_and_ack (bus, payload, len, TRUE,
                                      label, &local_error))
        {
          g_propagate_error (error, g_steal_pointer (&local_error));
          return NULL;
        }

      if (gdix51c0_irq_wait (bus->irq_req, bus->irq_events, TRUE,
                             bus->irq_offset,
                             response_timeout_usec,
                             resp_rise, &local_error))
        {
          buf = gdix51c0_spi_read (bus->dev, bus->spi_fd,
                                   &response_len, &local_error);
          if (buf &&
              (response_len == 0 || buf[0] != payload[0]))
            {
              g_set_error (&local_error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                           "gdix51c0: %s expected response 0x%02x, "
                           "got 0x%02x (%zu B)", label, payload[0],
                           response_len > 0 ? buf[0] : 0, response_len);
              g_clear_pointer (&buf, g_free);
            }
          if (buf &&
              !gdix51c0_irq_wait (bus->irq_req, bus->irq_events, FALSE,
                                  bus->irq_offset,
                                  response_timeout_usec,
                                  resp_fall, &local_error))
            g_clear_pointer (&buf, g_free);
        }

      if (buf)
        {
          if (out_len)
            *out_len = response_len;
          fp_dbg ("gdix51c0: %s response read %zu B", label, response_len);
          return g_steal_pointer (&buf);
        }

      if (attempt < GDIX51C0_CMD_ATTEMPTS)
        {
          gdix51c0_cmd_prepare_response_retry (bus, label, local_error);
          continue;
        }

      g_propagate_error (error, g_steal_pointer (&local_error));
      return NULL;
    }

  g_assert_not_reached ();
}

guint8 *
gdix51c0_cmd_single_resp (Gdix51c0Bus *bus,
                          const guint8 *payload,
                          gsize len,
                          guint timeout_usec,
                          gsize *out_len,
                          const char *label,
                          GError **error)
{
  guint response_timeout_usec =
    MAX (timeout_usec, GDIX51C0_CMD_MIN_RESPONSE_TIMEOUT_USEC);

  fp_dbg ("gdix51c0: %s (single response, %zu B)", label, len);

  for (guint attempt = 1; attempt <= GDIX51C0_CMD_ATTEMPTS; attempt++)
    {
      g_autoptr(GError) local_error = NULL;
      g_autofree guint8 *buf = NULL;
      gsize response_len = 0;

      gdix51c0_cmd_prepare_send (bus);
      if (!gdix51c0_spi_write (bus->dev, bus->spi_fd,
                               GDIX51C0_PKT_WRITE, payload, len, error))
        return NULL;

      if (gdix51c0_irq_wait (bus->irq_req, bus->irq_events, TRUE,
                             bus->irq_offset, response_timeout_usec,
                             label, &local_error))
        {
          buf = gdix51c0_spi_read (bus->dev, bus->spi_fd,
                                   &response_len, &local_error);
          if (buf && (proto_quirks & GDIX51C0_QUIRK_EXTRA_ACKS) &&
              gdix51c0_is_ack_for (buf, response_len, payload[0]))
            {
              fp_dbg ("gdix51c0: %s: skipping firmware ACK before response",
                      label);
              g_clear_pointer (&buf, g_free);
              if (gdix51c0_irq_wait (bus->irq_req, bus->irq_events, TRUE,
                                     bus->irq_offset, response_timeout_usec,
                                     label, &local_error))
                buf = gdix51c0_spi_read (bus->dev, bus->spi_fd,
                                         &response_len, &local_error);
            }
          if (buf &&
              !gdix51c0_irq_wait (bus->irq_req, bus->irq_events, FALSE,
                                  bus->irq_offset, response_timeout_usec,
                                  label, &local_error))
            g_clear_pointer (&buf, g_free);
        }

      if (buf)
        {
          if (out_len)
            *out_len = response_len;
          fp_dbg ("gdix51c0: %s single response read %zu B",
                  label, response_len);
          return g_steal_pointer (&buf);
        }

      if (attempt < GDIX51C0_CMD_ATTEMPTS)
        {
          gdix51c0_cmd_prepare_response_retry (bus, label, local_error);
          continue;
        }

      g_propagate_error (error, g_steal_pointer (&local_error));
      return NULL;
    }

  g_assert_not_reached ();
}

guint8 *
gdix51c0_cmd_ack_resp_same_irq (Gdix51c0Bus *bus,
                                const guint8 *payload,
                                gsize len,
                                guint timeout_usec,
                                gsize *out_len,
                                const char *label,
                                GError **error)
{
  guint response_timeout_usec =
    MAX (timeout_usec, GDIX51C0_CMD_MIN_RESPONSE_TIMEOUT_USEC);
#ifdef GOODIX_SPI_DEVELOPER
  guint fault_evk_limit = 0;
  guint fault_evk_outer = gdix51c0_cmd_claim_evk_failure (label,
                                                          &fault_evk_limit);
#endif

  fp_dbg ("gdix51c0: %s (ack + required same-IRQ response, %zu B)",
          label, len);
#ifdef GOODIX_SPI_DEVELOPER
  if (fault_evk_outer != 0)
    fp_warn ("gdix51c0: get-EVK armed required-data exhaustion for outer "
             "attempt %u/%u",
             fault_evk_outer, fault_evk_limit);
#endif

  for (guint attempt = 1; attempt <= GDIX51C0_CMD_ATTEMPTS; attempt++)
    {
      g_autoptr(GError) local_error = NULL;
      g_autofree guint8 *response = NULL;
      gsize response_len = 0;

      if (!gdix51c0_cmd_send_and_ack (bus, payload, len, FALSE,
                                      label, &local_error))
        {
          g_propagate_error (error, g_steal_pointer (&local_error));
          return NULL;
        }

      {
        response = gdix51c0_spi_read (bus->dev, bus->spi_fd,
                                      &response_len, &local_error);
        if (response &&
            (response_len == 0 || response[0] != payload[0]))
        {
          g_set_error (&local_error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                       "gdix51c0: %s expected response 0x%02x, "
                       "got 0x%02x (%zu B)", label, payload[0],
                       response_len > 0 ? response[0] : 0, response_len);
          g_clear_pointer (&response, g_free);
        }
        if (response &&
            !gdix51c0_irq_wait (bus->irq_req, bus->irq_events, FALSE,
                                bus->irq_offset, response_timeout_usec,
                                label, &local_error))
          g_clear_pointer (&response, g_free);
      }

#ifdef GOODIX_SPI_DEVELOPER
      if (response && fault_evk_outer != 0)
        {
          fp_warn ("gdix51c0: fault injection dropped EVK required response "
                   "on outer %u/%u data attempt %u/%u (%zu B)",
                   fault_evk_outer, fault_evk_limit,
                   attempt, GDIX51C0_CMD_ATTEMPTS, response_len);
          g_clear_pointer (&response, g_free);
          response_len = 0;
          g_usleep (response_timeout_usec);
          g_set_error (&local_error,
                       G_IO_ERROR,
                       G_IO_ERROR_TIMED_OUT,
                       "gdix51c0: injected EVK response timeout on outer %u/%u "
                       "data attempt %u/%u",
                       fault_evk_outer, fault_evk_limit,
                       attempt, GDIX51C0_CMD_ATTEMPTS);
        }
#endif

      if (response)
        {
          if (out_len)
            *out_len = response_len;
          fp_dbg ("gdix51c0: %s required response read %zu B",
                  label, response_len);
          return g_steal_pointer (&response);
        }

      if (attempt < GDIX51C0_CMD_ATTEMPTS)
        {
          gdix51c0_cmd_prepare_response_retry (bus, label, local_error);
          continue;
        }

      g_propagate_error (error, g_steal_pointer (&local_error));
      return NULL;
    }

  g_assert_not_reached ();
}

/* Under GDIX51C0_QUIRK_EXTRA_ACKS, IRQ after the ACK tells whether a response
 * follows: it stays high until a pending response is read (0xa8, up to ~200 ms)
 * and drops when the ACK is all there is (0x98 send-dac on fw 14103). */
#define GDIX51C0_ACK_ONLY_SETTLE_USEC 10000

gboolean
gdix51c0_cmd_ack_maybe_resp (Gdix51c0Bus *bus,
                             const guint8 *payload,
                             gsize len,
                             guint timeout_usec,
                             const char *label,
                             GError **error)
{
  g_autofree guint8 *response = NULL;
  gsize response_len = 0;

  if (!(proto_quirks & GDIX51C0_QUIRK_EXTRA_ACKS))
    {
      response = gdix51c0_cmd_ack_resp_same_irq (bus, payload, len, timeout_usec,
                                                 &response_len, label, error);
      return response != NULL;
    }

  if (!gdix51c0_cmd_send_and_ack (bus, payload, len, FALSE, label, error))
    return FALSE;

  if (gdix51c0_irq_wait (bus->irq_req, bus->irq_events, FALSE, bus->irq_offset,
                         GDIX51C0_ACK_ONLY_SETTLE_USEC, label, NULL))
    {
      fp_dbg ("gdix51c0: %s: ACK only, no response pending", label);
      return TRUE;
    }

  response = gdix51c0_spi_read (bus->dev, bus->spi_fd, &response_len, error);
  if (!response)
    return FALSE;
  if (response_len == 0 || response[0] != payload[0])
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                   "gdix51c0: %s expected response 0x%02x, got 0x%02x (%zu B)",
                   label, payload[0], response_len > 0 ? response[0] : 0,
                   response_len);
      return FALSE;
    }
  fp_dbg ("gdix51c0: %s response read %zu B", label, response_len);
  return gdix51c0_irq_wait (bus->irq_req, bus->irq_events, FALSE,
                            bus->irq_offset,
                            MAX (timeout_usec, GDIX51C0_CMD_MIN_RESPONSE_TIMEOUT_USEC),
                            label, error);
}

/* F4 compares CRC32/MPEG-2 (poly 0x04c11db7, init 0xffffffff, no xorout) of any
 * memory range with a caller value and answers equal or not equal. On
 * GF_HC460SEC_APP_1410x it is the only way to read memory (F2 is not
 * implemented), so bytes are read one at a time: the CRC of a single byte is
 * unique per value, so at most 256 queries recover it. */
#define GDIX51C0_CRC_ORACLE_ATTEMPTS 4

static guint32
gdix51c0_crc_one_byte (guint8 value)
{
  guint32 c = (guint32) (0xffu ^ value) << 24;

  for (int i = 0; i < 8; i++)
    c = (c & 0x80000000u) ? (c << 1) ^ 0x04c11db7u : (c << 1);
  return 0xffffff00u ^ c;
}

static gboolean
gdix51c0_crc_oracle_query (Gdix51c0Bus *bus, guint32 addr, guint32 crc,
                           gboolean *equal, GError **error)
{
  guint8 pkt[16] = { 0xf4, 0x0d, 0x00 };

  memcpy (pkt + 3, (guint32[]) { GUINT32_TO_LE (addr) }, 4);
  memcpy (pkt + 7, (guint32[]) { GUINT32_TO_LE (1) }, 4);
  memcpy (pkt + 11, (guint32[]) { GUINT32_TO_LE (crc) }, 4);
  pkt[15] = gdix51c0_payload_checksum (pkt, 15);

  for (guint attempt = 1; ; attempt++)
    {
      g_autoptr(GError) local_error = NULL;
      g_autofree guint8 *ack = NULL, *resp = NULL;
      gsize ack_len = 0, resp_len = 0;

      gdix51c0_cmd_prepare_send (bus);
      if (gdix51c0_spi_write (bus->dev, bus->spi_fd, GDIX51C0_PKT_WRITE,
                              pkt, sizeof pkt, &local_error) &&
          gdix51c0_irq_wait (bus->irq_req, bus->irq_events, TRUE, bus->irq_offset,
                             GDIX51C0_CMD_ACK_TIMEOUT_USEC, "crc-oracle", &local_error) &&
          (ack = gdix51c0_spi_read (bus->dev, bus->spi_fd, &ack_len, &local_error)) &&
          (resp = gdix51c0_spi_read (bus->dev, bus->spi_fd, &resp_len, &local_error)))
        {
          if (resp_len >= 4 && resp[0] == 0xf4)
            {
              *equal = resp[3] == 1;
              gdix51c0_irq_wait (bus->irq_req, bus->irq_events, FALSE,
                                 bus->irq_offset, GDIX51C0_CMD_ACK_TIMEOUT_USEC,
                                 "crc-oracle", NULL);
              return TRUE;
            }
          g_set_error (&local_error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                       "gdix51c0: crc-oracle: unexpected reply 0x%02x (%zu B)",
                       resp_len ? resp[0] : 0, resp_len);
        }

      if (attempt == GDIX51C0_CRC_ORACLE_ATTEMPTS)
        {
          g_propagate_error (error, g_steal_pointer (&local_error));
          return FALSE;
        }
      /* A lost exchange: let the sensor settle and drop anything pending. */
      g_usleep (50000 * attempt);
      gdix51c0_spi_drain (bus->spi_fd);
    }
}

gboolean
gdix51c0_crc_oracle_read (Gdix51c0Bus *bus, guint32 addr, guint8 *out,
                          gsize len, GError **error)
{
  for (gsize i = 0; i < len; i++)
    {
      gboolean found = FALSE;

      for (guint v = 0; v < 256 && !found; v++)
        {
          gboolean equal = FALSE;

          if (!gdix51c0_crc_oracle_query (bus, addr + i, gdix51c0_crc_one_byte (v),
                                          &equal, error))
            return FALSE;
          if (equal)
            {
              out[i] = (guint8) v;
              found = TRUE;
            }
        }
      if (!found)
        {
          g_set_error (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                       "gdix51c0: crc-oracle: no byte value matched at 0x%08x",
                       addr + (guint32) i);
          return FALSE;
        }
    }
  return TRUE;
}

gboolean
gdix51c0_irq_wait_edge_strict (struct gpiod_line_request *irq_req,
                               struct gpiod_edge_event_buffer *event_buf,
                               gboolean target_high,
                               unsigned int irq_offset,
                               guint timeout_usec,
                               const char *label,
                               GError **error)
{
  enum gpiod_edge_event_type want =
    target_high ? GPIOD_EDGE_EVENT_RISING_EDGE : GPIOD_EDGE_EVENT_FALLING_EDGE;

  gint64 deadline = g_get_monotonic_time () + (gint64) timeout_usec;

  if (!irq_req || !event_buf)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_CLOSED,
                    "gdix51c0: %s: IRQ line is closed", label);
      return FALSE;
    }

  for (;;)
    {
      gint64 now = g_get_monotonic_time ();
      if (now >= deadline)
        break;

      int ready =
        gpiod_line_request_wait_edge_events (irq_req, (deadline - now) * 1000);

      if (ready < 0)
        {
          g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                       "gdix51c0: %s: wait_edge_events failed", label);
          return FALSE;
        }

      if (ready == 0)
        break;

      int n = gpiod_line_request_read_edge_events (irq_req, event_buf, 1);
      if (n < 0)
        {
          g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                       "gdix51c0: %s: read_edge_events failed", label);
          return FALSE;
        }

      for (int i = 0; i < n; i++)
        {
          struct gpiod_edge_event *ev =
            gpiod_edge_event_buffer_get_event (event_buf, i);

          if (!ev)
            continue;

          if (gpiod_edge_event_get_event_type (ev) == want)
            return TRUE;

          enum gpiod_line_value value =
            gpiod_line_request_get_value (irq_req, irq_offset);

          if (value < 0)
            {
              g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                           "gdix51c0: %s: get IRQ value failed", label);
              return FALSE;
            }

          if ((value == GPIOD_LINE_VALUE_ACTIVE) == target_high)
            return TRUE;
        }
    }

  enum gpiod_line_value value =
    gpiod_line_request_get_value (irq_req, irq_offset);

  if (value >= 0)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                   "gdix51c0: %s: IRQ edge never went %s; final level=%s",
                   label,
                   target_high ? "high" : "low",
                   value == GPIOD_LINE_VALUE_ACTIVE ? "high" : "low");
    }
  else
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                   "gdix51c0: %s: IRQ edge never went %s",
                   label,
                   target_high ? "high" : "low");
    }

  return FALSE;
}

guint8 *
gdix51c0_cmd_single_resp_level (Gdix51c0Bus *bus,
                                const guint8 *payload,
                                gsize len,
                                guint timeout_usec,
                                gsize *out_len,
                                const char *label,
                                GError **error)
{
  fp_dbg ("gdix51c0: %s (single response level-wait, %zu B)", label, len);
  return gdix51c0_cmd_single_resp (bus, payload, len, timeout_usec,
                                   out_len, label, error);
}

/* ---- PSK (re)provisioning ---------------------------------------- */
/*
 * Push a freshly-wrapped PSK WB to the MCU (cmd 0xe0 register-write to
 * 0xbb010003), matching the recovered Windows register-write contract.
 * With @verify, read back 0xbb020003 (the MCU-computed
 * SHA-256 of the stored WB) and compare against SHA-256 of what we wrote —
 * the definitive confirmation the MCU accepted the write (and did not
 * OTP-lock the register).
 *
 * We deliberately write only 0xbb010003.  The 0xbb010002 DPAPI blob is
 * host-side bookkeeping used by the Windows driver to recover the PSK; a
 * Linux-only stack holds the plaintext PSK directly, so it is not needed.
 */
gboolean
gdix51c0_provision_psk (Gdix51c0Bus *bus,
                        const guint8 *psk,
                        gboolean verify,
                        GError **error)
{
  g_autofree guint8 *data = NULL;
  g_autofree guint8 *pkt = NULL;
  gsize data_len = 0, pkt_len = 0;

  g_return_val_if_fail (bus != NULL, FALSE);
  g_return_val_if_fail (psk != NULL, FALSE);

  /* Refuse to touch the MCU if the linked crypto can't reproduce the known
   * wrap vector — better a clean error than a garbage PSK register. */
  if (!gdix51c0_wb_selftest (error))
    return FALSE;

  data = gdix51c0_wb_provision_data (psk, &data_len, error);
  if (!data)
    return FALSE;

  pkt = gdix51c0_make_payload_packet (0xe0, data, data_len, &pkt_len);

  fp_info ("gdix51c0: provisioning PSK WB -> MCU 0xbb010003 (%zu B)", data_len);
  if (!gdix51c0_cmd_ack_resp (bus, pkt, pkt_len, "provision-psk-write", error))
    return FALSE;

  if (!verify)
    return TRUE;

  /* Read back 0xbb020003 and compare to SHA-256(WB). */
  {
    guint8 wb[GDIX51C0_WB_LEN];
    guint8 want[32];
    /* Pre-framed 0xe4 read of 0xbb020003 (address LE, zero length, checksum). */
    static const guint8 read_hash_payload[] = {
      0xe4, 0x09, 0x00,
      0x03, 0x00, 0x02, 0xbb,
      0x00, 0x00, 0x00, 0x00,
      0xfd
    };
    gsize resp_len = 0;
    guint16 plen;
    guint32 dtype, dlen;
    g_autofree guint8 *resp = NULL;

    if (!gdix51c0_wb_wrap (psk, wb, error) ||
        !gdix51c0_wb_hash (wb, want, error))
      return FALSE;

    resp = gdix51c0_cmd_ack_then_resp_read (bus, read_hash_payload,
                                            sizeof (read_hash_payload),
                                            GDIX51C0_CMD_MIN_RESPONSE_TIMEOUT_USEC,
                                            &resp_len,
                                            "provision-psk-verify", error);
    if (!resp)
      return FALSE;

    if (resp_len < 6 || resp[0] != 0xe4)
      {
        g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                             "gdix51c0: provision verify: bad read response");
        return FALSE;
      }
    plen = (guint16) (resp[1] | (resp[2] << 8));
    if ((gsize) plen + 3 != resp_len)
      {
        g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                     "gdix51c0: provision verify: length mismatch (%u vs %zu)",
                     plen, resp_len);
        return FALSE;
      }
    if (plen == 3)
      {
        g_set_error (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                     "gdix51c0: provision verify: 0xbb020003 empty (status=%#04x) "
                     "- MCU did not store the WB (OTP-locked?)", resp[3]);
        return FALSE;
      }
    if (resp_len < 13)
      {
        g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                             "gdix51c0: provision verify: short data packet");
        return FALSE;
      }
    dtype = resp[4] | (resp[5] << 8) | (resp[6] << 16) | ((guint32) resp[7] << 24);
    dlen = resp[8] | (resp[9] << 8) | (resp[10] << 16) | ((guint32) resp[11] << 24);
    /* fw 14103 tags the 32-byte hash 0x0000aaaa instead of echoing the
     * address; the hash comparison below still decides. */
    if ((proto_quirks & GDIX51C0_QUIRK_EXTRA_ACKS) && resp[3] == 0 &&
        dtype == GDIX51C0_PSK_HASH_TAG_14103 && dlen == 32 &&
        (gsize) 12 + dlen + 1 <= resp_len)
      {
        fp_info ("gdix51c0: provision verify: 0x%08x-tagged hash %s SHA-256 of WB",
                 dtype, memcmp (resp + 12, want, 32) == 0 ? "matches" : "differs from");
        dtype = GDIX51C0_PSK_HASH_ADDR;
      }
    if (resp[3] != 0 || dtype != GDIX51C0_PSK_HASH_ADDR || dlen != 32 ||
        (gsize) 12 + dlen + 1 > resp_len)
      {
        g_set_error (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                     "gdix51c0: provision verify: unexpected read (status=%#04x "
                     "type=%#010x len=%u)", resp[3], dtype, dlen);
        return FALSE;
      }
    if (memcmp (resp + 12, want, 32) != 0)
      {
        g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                             "gdix51c0: provision verify: 0xbb020003 hash mismatch "
                             "- MCU rejected the WB write (OTP lock?)");
        return FALSE;
      }
  }

  fp_info ("gdix51c0: PSK provisioning verified (0xbb020003 matches SHA-256 of WB)");
  return TRUE;
}

/*
 * Try to recover the PSK straight from the sensor: read register 0xbb010003
 * (the WB) and unwrap it with our recovered key.  This is the Linux analogue
 * of the Windows "re-read the sealed blob and unseal it" path — except we need
 * no DPAPI and no host-side secret, because the WB is decryptable with a
 * constant key we already extracted (see re/PARITY.md).
 *
 * On success returns TRUE.  @out_present is set FALSE if the register is empty
 * (a blank/erased sensor that needs provisioning) and TRUE if a WB was present.
 * Returns FALSE (with @error) on an I/O failure, or when a WB is present but
 * fails authentication (not one of ours).
 *
 * GDIX51C0 hardware reports 0xbb010003 as empty even immediately after a
 * verified write. Keep this read helper only as a diagnostic for future
 * compatible sensor profiles; normal operation uses persisted host state.
 */
gboolean
gdix51c0_recover_psk_from_mcu (Gdix51c0Bus *bus,
                               guint8      *out_psk,
                               gboolean    *out_present,
                               GError     **error)
{
  /* Pre-framed 0xe4 read of 0xbb010003 (address LE, zero length, checksum). */
  static const guint8 read_wb_payload[] = {
    0xe4, 0x09, 0x00,
    0x03, 0x00, 0x01, 0xbb,
    0x00, 0x00, 0x00, 0x00,
    0xfe
  };
  gsize resp_len = 0;
  guint16 plen;
  guint32 dtype, dlen;
  g_autofree guint8 *resp = NULL;

  g_return_val_if_fail (bus != NULL, FALSE);
  g_return_val_if_fail (out_psk != NULL, FALSE);

  if (out_present)
    *out_present = FALSE;

  resp = gdix51c0_cmd_ack_then_resp_read (bus, read_wb_payload,
                                          sizeof (read_wb_payload),
                                          GDIX51C0_CMD_MIN_RESPONSE_TIMEOUT_USEC,
                                          &resp_len,
                                          "recover-psk-read", error);
  if (!resp)
    return FALSE;

  if (resp_len < 6 || resp[0] != 0xe4)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                           "gdix51c0: recover: bad read response "
                           "(is 0xbb010003 readable?)");
      return FALSE;
    }
  plen = (guint16) (resp[1] | (resp[2] << 8));
  if ((gsize) plen + 3 != resp_len)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                   "gdix51c0: recover: length mismatch (%u vs %zu)",
                   plen, resp_len);
      return FALSE;
    }
  if (plen == 3)
    {
      /* Register not populated: blank/erased sensor — caller should provision. */
      fp_info ("gdix51c0: 0xbb010003 empty (status=%#04x) — sensor unprovisioned",
               resp[3]);
      return TRUE;
    }
  if (resp_len < 13)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                           "gdix51c0: recover: short data packet");
      return FALSE;
    }
  dtype = resp[4] | (resp[5] << 8) | (resp[6] << 16) | ((guint32) resp[7] << 24);
  dlen = resp[8] | (resp[9] << 8) | (resp[10] << 16) | ((guint32) resp[11] << 24);
  if (resp[3] != 0 || dtype != GDIX51C0_PSK_WB_ADDR ||
      dlen != GDIX51C0_WB_LEN || (gsize) 12 + dlen + 1 > resp_len)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                   "gdix51c0: recover: unexpected read (status=%#04x "
                   "type=%#010x len=%u)", resp[3], dtype, dlen);
      return FALSE;
    }

  if (!gdix51c0_wb_unwrap (resp + 12, out_psk, error))
    return FALSE;

  if (out_present)
    *out_present = TRUE;
  fp_info ("gdix51c0: recovered PSK from sensor (0xbb010003 unwrapped OK)");
  return TRUE;
}
