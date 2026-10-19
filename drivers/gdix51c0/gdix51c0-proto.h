// SPDX-License-Identifier: LGPL-2.1-or-later
/* Copyright (C) 2026 Berke Kabagöz <berkekbgz@gmail.com> */

/*
 * Goodix GDIX51C0 wire protocol — header.
 */

#pragma once

#include <glib.h>
#include <gpiod.h>

#include "fp-device.h"

G_BEGIN_DECLS

/* ---- Packet construction ----------------------------------------- */
/*
 * Wire format (matches python make_header_packet / make_payload_packet):
 *   [header 4B]  type | LE16 length | header_csum=sum(type|length)&0xff
 *   [payload NB] type | LE16 inner_len | data... | payload_csum=(0xaa - sum_so_far) & 0xff
 *
 * Header `type` is the outer wrapper (0xa0=write, 0xb0=read-ack).
 * Payload `type` is the inner cmd byte (0x32 FDT-down, 0x22 image-T1, ...).
 * `inner_len` is len(data) + 1 to include the trailing csum byte.
 */
guint8 *gdix51c0_make_header_packet  (guint8 type, guint16 payload_len);
guint8 *gdix51c0_make_payload_packet (guint8 type, const guint8 *data, gsize data_len, gsize *out_len);

guint8  gdix51c0_payload_checksum    (const guint8 *buf, gsize len);
gboolean gdix51c0_header_checksum_ok (const guint8 *hdr4);

/* ---- Synchronous SPI I/O ----------------------------------------- */
/*
 * write a header (type=0xa0) + payload
 */
gboolean gdix51c0_spi_write (FpDevice *dev, int spi_fd,
                          guint8 outer_type,
                          const guint8 *payload, gsize payload_len,
                          GError **error);

/*
 * read a 4-byte header, then payload_length bytes. Returns the payload
 * (caller frees with g_free).  out_len is set to the payload length.
 */
guint8 *gdix51c0_spi_read (FpDevice *dev, int spi_fd,
                        gsize *out_len, GError **error);

/*
 * Same as gdix51c0_spi_read but also returns the outer header's type
 * byte (hdr[0]) — the byte the WBDI/Windows driver logs as "cmd".  The
 * type byte is what we use to route incoming packets in the listener
 * because some response kinds (notably 0x22 image data) put a raw TLS
 * record in the payload, so payload[0] is not the cmd byte for those.
 */
guint8 *gdix51c0_spi_read_typed (FpDevice *dev, int spi_fd,
                                 guint8 *out_type,
                                 gsize *out_len, GError **error);

/* Single-attempt read used by the listener's IRQ-low polling fallback.
 * Returns the payload when a well-formed packet header is on the bus.
 * Returns NULL with *out_len = 0 and no error when the bus is idle
 * (all-zero / all-FF header) or the header checksum fails; returns NULL
 * with an error only on a transfer failure. */
guint8 *gdix51c0_spi_peek_packet (int spi_fd,
                                  guint8 *out_type,
                                  gsize *out_len,
                                  GError **error);

/* Enable the IRQ-low polling fallback for the synchronous command path on
 * @spi_fd (pass -1 to disable, e.g. while the async listener owns the bus).
 * gdix51c0_irq_wait() then peeks the bus between short waits and parks any
 * reply it finds; the next gdix51c0_spi_read_typed() on that fd returns it. */
void gdix51c0_irq_poll_set_fd (int spi_fd);

/* ---- IRQ + reset ------------------------------------------------- */
/*
 * Wait for an edge event of the given direction on the IRQ line.
 * target_high=TRUE means "wait for next rising edge", FALSE means
 * "wait for next falling".  Callers that start a new command drain any stale
 * queued edges before writing, but do not drain between ACK and response
 * cycles because those edges can arrive back-to-back.
 */
gboolean gdix51c0_irq_wait (struct gpiod_line_request *irq_req,
                          struct gpiod_edge_event_buffer *event_buf,
                          gboolean target_high, unsigned int irq_offset,
                          guint timeout_usec, const char *label,
                          GError **error);

/*
 * Drain any currently-buffered edge events without blocking.  Use this
 * after the boot probe to discard the post-reset pulse before the cmd
 * pre-roll starts counting edges.
 */
void gdix51c0_irq_drain (struct gpiod_line_request *irq_req,
                      struct gpiod_edge_event_buffer *event_buf);

/*
 * Pulse the reset line high then low.  Opens a fresh request, drives
 * the pulse, then releases the request — exactly like Python's
 * CdevGPIO(...).write(1).write(0).close() sequence.  The release lets
 * the line go to its default state (INPUT/high-Z) so any external
 * pull-up can deassert reset.
 */
gboolean gdix51c0_reset_pulse (struct gpiod_chip *chip, unsigned int offset, GError **error);

/*
 * Variant of payload checksum used for the get-mcu-state cmd 0xaf:
 * standard checksum + 1.  See Python calculate_checksum_for_mcu_timestamp.
 */
guint8 gdix51c0_payload_checksum_ts (const guint8 *buf, gsize len);

/* ---- Convenience wrappers for the 14213 init sequence ---------- */

/*
 * Common context bundle so the init helpers don't need 6 params each.
 */
typedef struct
{
  FpDevice                       *dev;
  int                             spi_fd;
  struct gpiod_line_request      *irq_req;
  unsigned int                    irq_offset;
  struct gpiod_edge_event_buffer *irq_events;
} Gdix51c0Bus;

/*
 * write a payload (outer 0xa0). No IRQ wait, no read.
 */
gboolean gdix51c0_cmd_no_ack (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                           const char *label, GError **error);

/*
 * Write payload, wait up to 1 s for ACK, and immediately resend once when the
 * ACK is absent.  Discards the ACK and waits for IRQ to return low.
 */
gboolean gdix51c0_cmd_ack (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                        const char *label, GError **error);

/*
 * Compatibility name for an ACK followed by a required response in a second
 * IRQ cycle.  Both received buffers are discarded.
 */
gboolean gdix51c0_cmd_ack_resp (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                             const char *label, GError **error);

/*
 * write payload, wait IRQ rise, read ack, wait IRQ fall, wait next
 * IRQ rise, read response, wait IRQ fall. Used by cmd 0xa6 (OTP) and
 * reset_sensor (cmd 0xa2).
 */
gboolean gdix51c0_cmd_ack_then_resp (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                                   const char *label, GError **error);

guint8 *gdix51c0_cmd_ack_then_resp_read (Gdix51c0Bus *bus,
                                         const guint8 *payload,
                                         gsize len,
                                         guint timeout_usec,
                                         gsize *out_len,
                                         const char *label,
                                         GError **error);

/*
 * Write a command which has no ACK but does have a required response.  The
 * response wait is at least 1 s and the complete command is retried once when
 * that response is absent.  Caller frees the returned buffer with g_free().
 */
guint8 *gdix51c0_cmd_read (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                        guint timeout_usec, gsize *out_len,
                        const char *label, GError **error);

guint8 *
gdix51c0_cmd_single_resp (Gdix51c0Bus *bus,
                          const guint8 *payload,
                          gsize len,
                          guint timeout_usec,
                          gsize *out_len,
                          const char *label,
                          GError **error);

/*
 * Wrap @psk (32 bytes) into the MCU WB container and write it to register
 * 0xbb010003 (cmd 0xe0).  With @verify, read back 0xbb020003 and confirm it
 * equals SHA-256 of the written WB.  Returns FALSE (with @error) on any I/O
 * failure or if the MCU did not accept the write (e.g. an OTP-locked
 * register). See drivers/gdix51c0/gdix51c0-wb.c and re/PARITY.md.
 */
gboolean
gdix51c0_provision_psk (Gdix51c0Bus *bus,
                        const guint8 *psk,
                        gboolean verify,
                        GError **error);

/*
 * Attempt to recover the PSK directly from the sensor by reading register
 * 0xbb010003 (the WB) and unwrapping it with the recovered key.  @out_present
 * is set FALSE when the register is empty (unprovisioned sensor), TRUE when a
 * WB was read and unwrapped into @out_psk (32 bytes).  Returns FALSE on I/O
 * error or if a present WB fails authentication.
 */
gboolean
gdix51c0_recover_psk_from_mcu (Gdix51c0Bus *bus,
                               guint8 *out_psk,
                               gboolean *out_present,
                               GError **error);

/* Read a required ACK followed by a required response while IRQ remains high.
 * Several pre-TLS commands, including sensor-register read 0x82, expose both
 * packets in one IRQ window rather than using a second rising edge. */
guint8 *
gdix51c0_cmd_ack_resp_same_irq (Gdix51c0Bus *bus,
                                const guint8 *payload,
                                gsize len,
                                guint timeout_usec,
                                gsize *out_len,
                                const char *label,
                                GError **error);


gboolean
gdix51c0_irq_wait_edge_strict (struct gpiod_line_request *irq_req,
                               struct gpiod_edge_event_buffer *event_buf,
                               gboolean target_high,
                               unsigned int irq_offset,
                               guint timeout_usec,
                               const char *label,
                               GError **error);

guint8 *
gdix51c0_cmd_single_resp_level (Gdix51c0Bus *bus,
                                const guint8 *payload,
                                gsize len,
                                guint timeout_usec,
                                gsize *out_len,
                                const char *label,
                                GError **error);

G_END_DECLS
