// SPDX-License-Identifier: LGPL-2.1-or-later
/* Copyright (C) 2026 Berke Kabagöz <berkekbgz@gmail.com> */

/*
 * Goodix GDIX51C0 — async SPI listener thread.
 *
 * Mirrors the Windows driver's data_from_device thread (see WBDI-new.log,
 * TID:19288): a dedicated thread owns the IRQ line and the read side of the
 * SPI fd, drains every packet the MCU offers, and dispatches by inner cmd
 * byte (payload[0]) into per-cmd queues.  Senders write via the listener's
 * write API (which serializes against the listener's reads on the SPI fd)
 * and wait on a typed condvar for their expected response.
 *
 * Rationale: the synchronous helpers in gdix51c0-proto.c block the calling
 * thread on a hand-rolled rise/read/fall sequence per command.  That works
 * for init but loses unsolicited packets (mcu state, fdt events that fire
 * outside a fdt_read window) and forces the long FDT-down wait to monopolise
 * the SPI bus.  The listener decouples them: arming is "write + drain ack",
 * and waiting for the finger packet just blocks on a condvar until cmd 0x32
 * arrives.
 */

#pragma once

#include <glib.h>
#include <gpiod.h>

#include "fp-device.h"

G_BEGIN_DECLS

typedef struct Gdix51c0Listener Gdix51c0Listener;

/* Spawn the listener thread. Takes ownership of the IRQ resources and SPI fd
 * for the lifetime of the listener — callers must not call gpiod or spidev
 * read/write APIs directly on these handles while the listener is running.
 * The handles themselves are still owned by the caller (we do not close them
 * on free).
 *
 * Returns NULL on failure. */
Gdix51c0Listener *
gdix51c0_listener_new (FpDevice                       *dev,
                       int                             spi_fd,
                       struct gpiod_line_request      *irq_req,
                       unsigned int                    irq_offset,
                       struct gpiod_edge_event_buffer *irq_events,
                       GError                        **error);

/* Stop the listener thread, join it, and free the dispatcher state.  Safe to
 * call with NULL.  After this returns the IRQ and SPI fd handles are
 * unowned and can be used by sync helpers again. */
void gdix51c0_listener_free (Gdix51c0Listener *self);

/* Drop every queued packet across all cmd-byte queues. Useful when starting
 * a fresh transaction sequence so stale unsolicited packets don't satisfy
 * the next await(). */
void gdix51c0_listener_drain_all (Gdix51c0Listener *self);

/* Drop every queued packet for the given cmd byte. */
void gdix51c0_listener_drain_cmd (Gdix51c0Listener *self, guint8 cmd);

/* When enabled, the listener performs NO SPI reads at all — used to keep the
 * bus silent during an image capture readout. */
void gdix51c0_listener_set_suppress (Gdix51c0Listener *self, gboolean on);

/* Do not peek the bus (IRQ-low fallback) for @usec from now; IRQ-high
 * reads still happen.  Used after an image setmode so the sensor's readout
 * is not disturbed while a frame that does raise IRQ is still read at once. */
void gdix51c0_listener_hold_peeks (Gdix51c0Listener *self, guint usec);

/* Send a packet on the SPI bus (outer header type=0xa0, plus payload).
 * Serializes against listener reads via the bus mutex. */
gboolean gdix51c0_listener_write (Gdix51c0Listener *self,
                                  const guint8     *payload,
                                  gsize             payload_len,
                                  GError          **error);

/* Submit a command and wait for its 0xb0 ACK using the official transport
 * policy: 1 ms before the first submit, a 1000 ms ACK window, then one
 * immediate same-command resend if the ACK was not observed. */
gboolean gdix51c0_listener_send_ack (Gdix51c0Listener *self,
                                     const guint8     *payload,
                                     gsize             payload_len,
                                     const char       *label,
                                     GError          **error);

/* SpiSendDataToDeviceLock shape: run the ACK transport above, wait for the
 * typed data packet, and repeat that complete operation once on data timeout.
 * Caller owns the returned packet. */
guint8 *gdix51c0_listener_command (Gdix51c0Listener *self,
                                   const guint8     *payload,
                                   gsize             payload_len,
                                   guint8            response_command,
                                   guint             response_timeout_usec,
                                   gsize            *out_len,
                                   const char       *label,
                                   GError          **error);

/* Wait for the next packet whose payload[0] == cmd.  Returns the full
 * payload (caller frees with g_free); *out_len receives its length.
 * Returns NULL with G_IO_ERROR_TIMED_OUT on timeout, or with another
 * domain on a hard listener failure. */
guint8 *gdix51c0_listener_await (Gdix51c0Listener *self,
                                 guint8            cmd,
                                 guint             timeout_usec,
                                 gsize            *out_len,
                                 GError          **error);

G_END_DECLS
