// SPDX-License-Identifier: LGPL-2.1-or-later
/* Copyright (C) 2026 Berke Kabagöz <berkekbgz@gmail.com> */

/*
 * Goodix GDIX51C0 — async SPI listener thread.  See gdix51c0-listener.h.
 */

#define FP_COMPONENT "gdix51c0"

#include <errno.h>
#include <poll.h>
#include <string.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include "drivers_api.h"

#include "gdix51c0.h"
#include "gdix51c0-listener.h"
#include "gdix51c0-proto.h"

/* Maximum packets we'll keep buffered per cmd-byte queue.  The MCU should
 * never queue more than a handful of unread packets; if a sender disappears
 * we'd rather drop old packets than grow without bound. */
#define LISTENER_QUEUE_CAP 16

/* Hardware acceptance hooks are compiled out of release builds. */
#ifdef GOODIX_SPI_DEVELOPER
#define GDIX51C0_FAULT_ACK_TIMEOUT_ONCE_ENV \
  "GDIX51C0_FAULT_ACK_TIMEOUT_ONCE"
#define GDIX51C0_FAULT_RESPONSE_TIMEOUT_ONCE_ENV \
  "GDIX51C0_FAULT_RESPONSE_TIMEOUT_ONCE"
#endif

typedef struct
{
  guint8 *data;
  gsize   len;
} ListenerPacket;

struct Gdix51c0Listener
{
  FpDevice                       *dev;
  int                             spi_fd;
  struct gpiod_line_request      *irq_req;
  unsigned int                    irq_offset;
  struct gpiod_edge_event_buffer *irq_events;
  int                             wake_fd;

  GThread *thread;
  gint     stop_requested;  /* atomic */
  gint     awaiting;        /* atomic: callers blocked in listener_await */
  gint64   peek_holdoff_until_us; /* under dispatch_lock: no bus peeks before this */
  gint     suppress;        /* atomic: when set, the listener does NO SPI reads.
                             * Used during image capture so we stay silent on the
                             * bus (like Windows) while the sensor reads out the
                             * array — any SPI traffic in that window chops the
                             * readout at a fixed row. */

  /* spi_lock serializes any access to the SPI fd: listener reads in its
   * loop, callers write in gdix51c0_listener_write.  We *also* hold this
   * during the listener's write() so partial-write interleaving with reads
   * cannot happen. */
  GMutex spi_lock;

  /* dispatch_lock protects queues[] and is paired with cond. */
  GMutex   dispatch_lock;
  GCond    cond;
  GQueue  *queues[256];   /* one queue per inner-cmd byte, lazily allocated */

#ifdef GOODIX_SPI_DEVELOPER
  gint fault_ack_timeout_used;       /* atomic */
  gint fault_response_timeout_used;  /* atomic */
  gint fault_drop_ack_armed;         /* atomic boolean */
  gint fault_drop_response_kind;     /* atomic: inner cmd + 1, zero = off */
#endif
};

#ifdef GOODIX_SPI_DEVELOPER
static gboolean
listener_arm_timeout_once (const char *env_name,
                           const char *label,
                           gint       *used)
{
  const char *target = g_getenv (env_name);

  if (!target || !*target || g_strcmp0 (target, label) != 0)
    return FALSE;

  return g_atomic_int_compare_and_exchange (used, 0, 1);
}
#endif

static void
listener_packet_free (gpointer data)
{
  ListenerPacket *p = data;

  if (!p)
    return;

  g_free (p->data);
  g_free (p);
}

/* Caller must hold dispatch_lock. */
static GQueue *
listener_queue_for (Gdix51c0Listener *self, guint8 cmd)
{
  GQueue *q = self->queues[cmd];

  if (!q)
    {
      q = g_queue_new ();
      self->queues[cmd] = q;
    }

  return q;
}

/* Caller must hold dispatch_lock. */
static void
listener_enqueue_locked (Gdix51c0Listener *self,
                         guint8            cmd,
                         guint8           *data,
                         gsize             len)
{
  GQueue *q = listener_queue_for (self, cmd);

  if (g_queue_get_length (q) >= LISTENER_QUEUE_CAP)
    {
      ListenerPacket *old = g_queue_pop_head (q);

      fp_dbg ("gdix51c0: listener queue cmd=0x%02x full, dropping oldest "
              "(%zu B)", cmd, old ? old->len : 0);
      listener_packet_free (old);
    }

  ListenerPacket *p = g_new0 (ListenerPacket, 1);

  p->data = data;
  p->len  = len;
  g_queue_push_tail (q, p);
}

/* Route one packet read off the bus into its per-cmd queue.
 * Takes ownership of payload. */
static void
listener_dispatch (Gdix51c0Listener *self,
                   guint8            outer_type,
                   guint8           *payload,
                   gsize             n)
{
    /* Dispatch by payload[0] — that is the inner cmd byte the python
     * reference and Windows WBDI log call "packet type":
     *   0xb0 ACK, 0x32 FDT-down, 0x34 FDT-up, 0xae mcu state.
     * For image data the payload is a raw TLS record so payload[0] is
     * 0x17 (TLS appdata).  The outer header's type byte is always 0xa0
     * on this device and does not discriminate packet kinds, so we log
     * it for debugging but route only on payload[0]. */
    guint8 kind = payload[0];

#ifdef GOODIX_SPI_DEVELOPER
    if (kind == GDIX51C0_PKT_READ &&
        g_atomic_int_compare_and_exchange (&self->fault_drop_ack_armed,
                                           1, 0))
      {
        fp_warn ("gdix51c0: fault injection dropped one ACK packet");
        g_free (payload);
        return;
      }
#endif

#ifdef GOODIX_SPI_DEVELOPER
    gint response_fault = g_atomic_int_get (
      &self->fault_drop_response_kind);
    if (response_fault == (gint) kind + 1 &&
        g_atomic_int_compare_and_exchange (
          &self->fault_drop_response_kind, response_fault, 0))
      {
        fp_warn ("gdix51c0: fault injection dropped one response "
                 "packet kind=0x%02x", kind);
        g_free (payload);
        return;
      }
#endif

    g_mutex_lock (&self->dispatch_lock);
    listener_enqueue_locked (self, kind, payload, n);
    g_cond_broadcast (&self->cond);
    g_mutex_unlock (&self->dispatch_lock);

    fp_dbg ("gdix51c0: listener received outer=0x%02x kind=0x%02x len=%zu",
            outer_type, kind, n);
}

/* Read every pending packet while IRQ stays high.  Holds spi_lock for each
 * read but releases between packets so writers can interleave if the MCU
 * pauses. */
static void
listener_drain_irq_high (Gdix51c0Listener *self)
{
  for (;;)
    {
      if (g_atomic_int_get (&self->stop_requested))
        return;

      /* Stay completely off the SPI bus while a capture is in progress. */
      if (g_atomic_int_get (&self->suppress))
        return;

      enum gpiod_line_value v =
        gpiod_line_request_get_value (self->irq_req, self->irq_offset);

      if (v != GPIOD_LINE_VALUE_ACTIVE)
        return;

      g_mutex_lock (&self->spi_lock);

      gsize n = 0;
      guint8 outer_type = 0;
      g_autoptr(GError) err = NULL;
      /* Single header attempt: the retrying read would clock the bus
       * several times with sleeps in between, which damages an image
       * readout in progress.  An idle header just means "not yet". */
      guint8 *payload = gdix51c0_spi_peek_packet (self->spi_fd,
                                                  &outer_type, &n, &err);

      g_mutex_unlock (&self->spi_lock);

      if (!payload)
        {
          if (err)
            {
              fp_dbg ("gdix51c0: listener read failed: %s", err->message);
              g_usleep (5000);
              return;
            }
          /* IRQ high but idle header: back off, re-check level. */
          g_usleep (3000);
          continue;
        }

      if (n == 0)
        {
          g_free (payload);
          /* Idle/empty read while IRQ is still high.  Do NOT spin re-reading
           * the SPI bus: during an image capture the sensor holds IRQ high for
           * its ~73 ms row-by-row readout, and hammering SPI in that window
           * corrupts the readout partway (the frame comes back valid only down
           * to a fixed row, the rest garbage).  Windows stays silent on the bus
           * during capture.  Back off so we don't disturb the analog readout;
           * a real packet will still be drained on the next iteration. */
#ifdef GOODIX_SPI_DEVELOPER
          const char *idle_us = g_getenv ("GDIX51C0_LISTENER_IDLE_US");
          g_usleep (idle_us && *idle_us
                    ? (gulong) g_ascii_strtoull (idle_us, NULL, 0)
                    : 3000);
#else
          g_usleep (3000);
#endif
          continue;
        }

      listener_dispatch (self, outer_type, payload, n);
    }
}

/* Some GDIX51C0 boards (seen on a Huawei MateBook 16s 2023, CREFG-XX) leave
 * a reply ready on the SPI bus without asserting IRQ.  The level-driven loop
 * above never reads those, so the caller times out even though the packet is
 * there.  While somebody is blocked in gdix51c0_listener_await, peek the bus
 * every few milliseconds when IRQ is low.  An idle bus answers with all-zero
 * or all-FF headers, which the peek ignores. */
static void
listener_speculative_read (Gdix51c0Listener *self)
{
  static int enabled = -1;

  if (enabled < 0)
    {
      const char *v = g_getenv ("GDIX51C0_LISTENER_POLL");
      enabled = (v && g_strcmp0 (v, "0") == 0) ? 0 : 1;
    }
  if (!enabled)
    return;

  if (g_atomic_int_get (&self->stop_requested) ||
      g_atomic_int_get (&self->suppress) ||
      g_atomic_int_get (&self->awaiting) <= 0)
    return;

  g_mutex_lock (&self->dispatch_lock);
  gint64 holdoff = self->peek_holdoff_until_us;
  g_mutex_unlock (&self->dispatch_lock);
  if (g_get_monotonic_time () < holdoff)
    return;   /* capture readout window: no bus traffic unless IRQ says ready */

  if (gpiod_line_request_get_value (self->irq_req, self->irq_offset) ==
      GPIOD_LINE_VALUE_ACTIVE)
    return;   /* the normal IRQ-high drain handles this */

  g_mutex_lock (&self->spi_lock);

  gsize n = 0;
  guint8 outer_type = 0;
  g_autoptr(GError) err = NULL;
  guint8 *payload = gdix51c0_spi_peek_packet (self->spi_fd, &outer_type,
                                              &n, &err);

  g_mutex_unlock (&self->spi_lock);

  if (!payload)
    {
      if (err)
        {
          fp_dbg ("gdix51c0: listener poll read failed: %s", err->message);
          g_usleep (5000);
        }
      return;
    }

  if (n == 0)
    {
      g_free (payload);
      return;
    }

  fp_dbg ("gdix51c0: listener fetched a reply with IRQ low (%zu B)", n);
  listener_dispatch (self, outer_type, payload, n);
}

static gpointer
listener_thread_main (gpointer user_data)
{
  Gdix51c0Listener *self = user_data;
  const int gpio_fd = gpiod_line_request_get_fd (self->irq_req);
  struct pollfd poll_fds[] = {
    { .fd = gpio_fd,       .events = POLLIN },
    { .fd = self->wake_fd, .events = POLLIN },
  };

  fp_dbg ("gdix51c0: listener thread starting");

  /* A packet may already be holding IRQ high when the listener takes over
   * from the synchronous TLS path. Drain it once before blocking for a new
   * edge. */
  listener_drain_irq_high (self);

  while (!g_atomic_int_get (&self->stop_requested))
    {
      /* Block indefinitely when idle; poll the bus every 3 ms while a
       * caller waits for a reply (IRQ-low fallback, see above). */
      int timeout_ms = g_atomic_int_get (&self->awaiting) > 0 ? 3 : -1;
      int ready = poll (poll_fds, G_N_ELEMENTS (poll_fds), timeout_ms);

      if (g_atomic_int_get (&self->stop_requested))
        break;

      if (ready < 0)
        {
          if (errno == EINTR)
            continue;
          fp_warn ("gdix51c0: listener poll: errno=%d", errno);
          g_usleep (5000);
          continue;
        }

      if (ready == 0)
        {
          /* No new edge.  The level may still be high from an edge that
           * was consumed while reads were suppressed (image capture), so
           * drain on level first, then fall back to peeking the bus. */
          listener_drain_irq_high (self);
          listener_speculative_read (self);
          continue;
        }

      if (poll_fds[1].revents & POLLIN)
        {
          uint64_t wake_count;

          (void) read (self->wake_fd, &wake_count, sizeof (wake_count));
          poll_fds[1].revents = 0;
          /* A wake also follows suppress-off and await-start: pick up a
           * packet whose edge was consumed while we could not read. */
          listener_drain_irq_high (self);
          continue;
        }

      if (poll_fds[0].revents & POLLIN)
        {
          /* Drain the events buffer so it doesn't fill up; we don't
           * actually care which edges arrived — the IRQ level decides
           * whether there is a packet to read. */
          gpiod_line_request_read_edge_events (self->irq_req,
                                               self->irq_events, 16);
          poll_fds[0].revents = 0;
        }

      listener_drain_irq_high (self);
    }

  fp_dbg ("gdix51c0: listener thread exiting");
  return NULL;
}

Gdix51c0Listener *
gdix51c0_listener_new (FpDevice                       *dev,
                       int                             spi_fd,
                       struct gpiod_line_request      *irq_req,
                       unsigned int                    irq_offset,
                       struct gpiod_edge_event_buffer *irq_events,
                       GError                        **error)
{
  Gdix51c0Listener *self = g_new0 (Gdix51c0Listener, 1);

  self->dev         = dev;
  self->spi_fd      = spi_fd;
  self->irq_req     = irq_req;
  self->irq_offset  = irq_offset;
  self->irq_events  = irq_events;
  self->wake_fd     = eventfd (0, EFD_CLOEXEC | EFD_NONBLOCK);

  if (self->wake_fd < 0)
    {
      g_set_error (error,
                   G_IO_ERROR,
                   g_io_error_from_errno (errno),
                   "gdix51c0: listener eventfd failed: %s",
                   g_strerror (errno));
      g_free (self);
      return NULL;
    }

  g_mutex_init (&self->spi_lock);
  g_mutex_init (&self->dispatch_lock);
  g_cond_init  (&self->cond);

  self->thread = g_thread_try_new ("gdix51c0-listener",
                                   listener_thread_main, self, error);
  if (!self->thread)
    {
      close (self->wake_fd);
      g_mutex_clear (&self->spi_lock);
      g_mutex_clear (&self->dispatch_lock);
      g_cond_clear  (&self->cond);
      g_free (self);
      return NULL;
    }

  return self;
}

void
gdix51c0_listener_free (Gdix51c0Listener *self)
{
  if (!self)
    return;

  g_atomic_int_set (&self->stop_requested, 1);

  if (self->thread)
    {
      uint64_t wake_count = 1;

      /* Wake the blocking poll immediately. This keeps an idle session fully
       * event driven without making close wait for a periodic timeout. */
      (void) write (self->wake_fd, &wake_count, sizeof (wake_count));
      g_thread_join (self->thread);
      self->thread = NULL;
    }

  close (self->wake_fd);
  self->wake_fd = -1;

  for (guint i = 0; i < G_N_ELEMENTS (self->queues); i++)
    {
      if (self->queues[i])
        {
          g_queue_free_full (self->queues[i], listener_packet_free);
          self->queues[i] = NULL;
        }
    }

  g_mutex_clear (&self->spi_lock);
  g_mutex_clear (&self->dispatch_lock);
  g_cond_clear  (&self->cond);

  g_free (self);
}

void
gdix51c0_listener_drain_all (Gdix51c0Listener *self)
{
  if (!self)
    return;

  g_mutex_lock (&self->dispatch_lock);
  for (guint i = 0; i < G_N_ELEMENTS (self->queues); i++)
    {
      if (self->queues[i])
        {
          ListenerPacket *p;
          while ((p = g_queue_pop_head (self->queues[i])) != NULL)
            listener_packet_free (p);
        }
    }
  g_mutex_unlock (&self->dispatch_lock);
}

void
gdix51c0_listener_drain_cmd (Gdix51c0Listener *self, guint8 cmd)
{
  if (!self)
    return;

  g_mutex_lock (&self->dispatch_lock);
  GQueue *q = self->queues[cmd];
  if (q)
    {
      ListenerPacket *p;
      while ((p = g_queue_pop_head (q)) != NULL)
        listener_packet_free (p);
    }
  g_mutex_unlock (&self->dispatch_lock);
}

void
gdix51c0_listener_set_suppress (Gdix51c0Listener *self, gboolean on)
{
  if (!self)
    return;
  g_atomic_int_set (&self->suppress, on ? 1 : 0);
  if (!on && self->wake_fd >= 0)
    {
      uint64_t wake_count = 1;
      (void) write (self->wake_fd, &wake_count, sizeof (wake_count));
    }
}

void
gdix51c0_listener_hold_peeks (Gdix51c0Listener *self, guint usec)
{
  if (!self)
    return;
  g_mutex_lock (&self->dispatch_lock);
  self->peek_holdoff_until_us = g_get_monotonic_time () + (gint64) usec;
  g_mutex_unlock (&self->dispatch_lock);
}

static gboolean
gdix51c0_listener_write_internal (Gdix51c0Listener *self,
                                  const guint8     *payload,
                                  gsize             payload_len,
                                  gboolean          initial_submit,
                                  GError          **error)
{
  /* gfspi!SpiSendDataToDevice calls Sleep(1) once before the initial WDF
   * submit.  Its internal ACK-timeout resend is immediate. */
  if (initial_submit)
    g_usleep (1000);

  g_mutex_lock (&self->spi_lock);
  gboolean ok = gdix51c0_spi_write (self->dev, self->spi_fd,
                                    GDIX51C0_PKT_WRITE,
                                    payload, payload_len, error);
  g_mutex_unlock (&self->spi_lock);
  return ok;
}

gboolean
gdix51c0_listener_write (Gdix51c0Listener *self,
                         const guint8     *payload,
                         gsize             payload_len,
                         GError          **error)
{
  return gdix51c0_listener_write_internal (self, payload, payload_len,
                                            TRUE, error);
}

gboolean
gdix51c0_listener_send_ack (Gdix51c0Listener *self,
                            const guint8     *payload,
                            gsize             payload_len,
                            const char       *label,
                            GError          **error)
{
  g_autoptr(GError) local_error = NULL;

  gdix51c0_listener_drain_cmd (self, GDIX51C0_PKT_READ);

  for (guint attempt = 1; attempt <= 2; attempt++)
    {
      gsize ack_len = 0;
      g_autofree guint8 *ack = NULL;

#ifdef GOODIX_SPI_DEVELOPER
      if (attempt == 1 &&
          listener_arm_timeout_once (GDIX51C0_FAULT_ACK_TIMEOUT_ONCE_ENV,
                                     label,
                                     &self->fault_ack_timeout_used))
        {
          fp_warn ("gdix51c0: %s armed one-shot ACK-timeout injection",
                   label);
          g_atomic_int_set (&self->fault_drop_ack_armed, 1);
        }
#endif

      if (!gdix51c0_listener_write_internal (self, payload, payload_len,
                                              attempt == 1, error))
        {
#ifdef GOODIX_SPI_DEVELOPER
          g_atomic_int_set (&self->fault_drop_ack_armed, 0);
#endif
          return FALSE;
        }

      ack = gdix51c0_listener_await (self, GDIX51C0_PKT_READ,
                                     1000 * 1000, &ack_len,
                                     &local_error);
      if (ack)
        {
#ifdef GOODIX_SPI_DEVELOPER
          g_atomic_int_set (&self->fault_drop_ack_armed, 0);
#endif
          fp_dbg ("gdix51c0: %s ACK read %zu B on transport attempt %u/2",
                  label, ack_len, attempt);
          return TRUE;
        }

      /* The Windows resend is specific to ACK timeout.  A stopped listener
       * or another hard I/O error must still escape immediately. */
      if (!g_error_matches (local_error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT) ||
          attempt == 2)
        {
#ifdef GOODIX_SPI_DEVELOPER
          g_atomic_int_set (&self->fault_drop_ack_armed, 0);
#endif
          g_propagate_error (error, g_steal_pointer (&local_error));
          return FALSE;
        }

      fp_dbg ("gdix51c0: %s ACK timed out; immediate same-command resend",
              label);
      g_clear_error (&local_error);
    }

  g_assert_not_reached ();
}

guint8 *
gdix51c0_listener_command (Gdix51c0Listener *self,
                           const guint8     *payload,
                           gsize             payload_len,
                           guint8            response_command,
                           guint             response_timeout_usec,
                           gsize            *out_len,
                           const char       *label,
                           GError          **error)
{
  for (guint attempt = 1; attempt <= 2; attempt++)
    {
      g_autoptr(GError) local_error = NULL;
      gsize response_len = 0;
      g_autofree guint8 *response = NULL;

      /* ResetEvent(response_event) in SpiSendDataToDeviceLock. */
      gdix51c0_listener_drain_cmd (self, response_command);

#ifdef GOODIX_SPI_DEVELOPER
      if (attempt == 1 &&
          listener_arm_timeout_once (
            GDIX51C0_FAULT_RESPONSE_TIMEOUT_ONCE_ENV,
            label,
            &self->fault_response_timeout_used))
        {
          fp_warn ("gdix51c0: %s armed one-shot data-response-timeout "
                   "injection", label);
          g_atomic_int_set (&self->fault_drop_response_kind,
                            (gint) response_command + 1);
        }
#endif

      if (!gdix51c0_listener_send_ack (self, payload, payload_len,
                                        label, &local_error))
        {
#ifdef GOODIX_SPI_DEVELOPER
          g_atomic_int_set (&self->fault_drop_response_kind, 0);
#endif
          g_propagate_error (error, g_steal_pointer (&local_error));
          return NULL;
        }

      response = gdix51c0_listener_await (self, response_command,
                                          response_timeout_usec,
                                          &response_len, &local_error);
      if (response)
        {
#ifdef GOODIX_SPI_DEVELOPER
          g_atomic_int_set (&self->fault_drop_response_kind, 0);
#endif
          if (out_len)
            *out_len = response_len;
          return g_steal_pointer (&response);
        }

      if (!g_error_matches (local_error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT) ||
          attempt == 2)
        {
#ifdef GOODIX_SPI_DEVELOPER
          g_atomic_int_set (&self->fault_drop_response_kind, 0);
#endif
          g_propagate_error (error, g_steal_pointer (&local_error));
          return NULL;
        }

      fp_dbg ("gdix51c0: %s data response timed out; resubmitting complete "
              "command once like SpiSendDataToDeviceLock", label);
    }

  g_assert_not_reached ();
}

static guint8 *
listener_await_impl (Gdix51c0Listener *self,
                         guint8            cmd,
                         guint             timeout_usec,
                         gsize            *out_len,
                         GError          **error)
{
  gint64 deadline_us = g_get_monotonic_time () + (gint64) timeout_usec;

  g_mutex_lock (&self->dispatch_lock);

  for (;;)
    {
      GQueue *q = self->queues[cmd];

      if (q)
        {
          ListenerPacket *p = g_queue_pop_head (q);
          if (p)
            {
              guint8 *data = p->data;
              gsize   len  = p->len;

              g_free (p);
              g_mutex_unlock (&self->dispatch_lock);

              if (out_len)
                *out_len = len;
              return data;
            }
        }

      if (g_atomic_int_get (&self->stop_requested))
        {
          g_mutex_unlock (&self->dispatch_lock);
          g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_CLOSED,
                               "gdix51c0: listener stopped while awaiting");
          return NULL;
        }

      gint64 now_us = g_get_monotonic_time ();

      if (now_us >= deadline_us)
        {
          g_mutex_unlock (&self->dispatch_lock);
          g_set_error (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                       "gdix51c0: timed out awaiting cmd 0x%02x after %u us",
                       cmd, timeout_usec);
          return NULL;
        }

      gint64 wait_until_us = deadline_us;
      /* g_cond_wait_until takes monotonic microseconds. */
      g_cond_wait_until (&self->cond, &self->dispatch_lock, wait_until_us);
    }
}

guint8 *
gdix51c0_listener_await (Gdix51c0Listener *self,
                         guint8            cmd,
                         guint             timeout_usec,
                         gsize            *out_len,
                         GError          **error)
{
  uint64_t wake_count = 1;
  guint8  *data;

  g_atomic_int_inc (&self->awaiting);
  /* Kick the listener out of its indefinite poll so it starts the IRQ-low
   * polling fallback right away. */
  (void) write (self->wake_fd, &wake_count, sizeof (wake_count));

  data = listener_await_impl (self, cmd, timeout_usec, out_len, error);

  (void) g_atomic_int_dec_and_test (&self->awaiting);
  return data;
}
