// SPDX-License-Identifier: LGPL-2.1-or-later
/* Copyright (C) 2026 Berke Kabagöz <berkekbgz@gmail.com> */

/*
 * Goodix GDIX51C0 TLS layer.
 *
 * Custom BIO that reads/writes whole SPI packets to/from the MCU.
 * On the read side we apply the same workarounds as the Python script:
 *   - The MCU sends a malformed ChangeCipherSpec record with length 0;
 *     patch to length 1, payload 0x01 (per RFC 5246).
 *   - The MCU's ClientKeyExchange identity field has historically come
 *     back with the trailing bytes zeroed (spidev tail-dropout); we
 *     restore "Client_identity" defensively even though the host-level
 *     DMA fix should make this unnecessary on 14213.
 */

#define FP_COMPONENT "gdix51c0"

#include <errno.h>
#include <string.h>

#include <openssl/bio.h>
#include <openssl/err.h>
#include <openssl/ssl.h>
#include <openssl/sslerr.h>

#include "drivers_api.h"

#include "gdix51c0.h"
#include "gdix51c0-proto.h"
#include "gdix51c0-tls.h"

/* OpenSSL < 3.2 (e.g. Ubuntu 24.04's 3.0.x) does not define this reason code.
 * Use a value ERR_GET_REASON() can never return so the comparison is inert. */
#ifndef SSL_R_RECORD_LAYER_FAILURE
#define SSL_R_RECORD_LAYER_FAILURE (-1)
#endif

#define GDIX51C0_PSK_IDENTITY "Client_identity"

/* ---------------- Custom BIO ---------------- */

#define GDIX51C0_BIO_NAME "gdix51c0-spi"

static int
gdix51c0_bio_write (BIO *b, const char *buf, int len)
{
  Gdix51c0Tls *tls = BIO_get_data (b);
  GError *error = NULL;

  BIO_clear_retry_flags (b);
  if (!gdix51c0_spi_write (tls->dev, tls->spi_fd, GDIX51C0_PKT_READ /* 0xb0 */,
                        (const guint8 *) buf, (gsize) len, &error))
    {
      fp_warn ("gdix51c0: tls bio write failed: %s",
               error ? error->message : "?");
      g_clear_error (&error);
      return -1;
    }
  return len;
}

/* CKE record layout (TLS body 21 bytes):
 *   16 03 03 00 15            TLS handshake record header, len 21
 *   10 00 00 11               handshake type CKE, len 17
 *   00 0F                     PSK identity length 15
 *   <15 bytes>                "Client_identity"
 */
#define GDIX51C0_CKE_PREFIX_LEN 11
static const guint8 gdix51c0_cke_header[GDIX51C0_CKE_PREFIX_LEN] = {
  0x16, 0x03, 0x03, 0x00, 0x15, 0x10, 0x00, 0x00, 0x11, 0x00, 0x0f
};
static const guint8 gdix51c0_cke_identity[15] = "Client_identity";

static void
gdix51c0_patch_inbound (guint8 *buf, gsize len)
{
  /* Malformed empty ChangeCipherSpec: 14 03 03 00 00 00 -> 14 03 03 00 01 01 */
  if (len >= 6 &&
      buf[0] == 0x14 && buf[1] == 0x03 && buf[2] == 0x03 &&
      buf[3] == 0x00 && buf[4] == 0x00 && buf[5] == 0x00)
    {
      buf[4] = 0x01;
      buf[5] = 0x01;
      fp_dbg ("gdix51c0: patched malformed empty ChangeCipherSpec");
    }

  /* Truncated CKE identity -> restore "Client_identity". */
  gsize cke_full = GDIX51C0_CKE_PREFIX_LEN + sizeof (gdix51c0_cke_identity);
  if (len >= cke_full &&
      memcmp (buf, gdix51c0_cke_header, GDIX51C0_CKE_PREFIX_LEN) == 0 &&
      memcmp (buf + GDIX51C0_CKE_PREFIX_LEN, gdix51c0_cke_identity, sizeof (gdix51c0_cke_identity)) != 0)
    {
      memcpy (buf + GDIX51C0_CKE_PREFIX_LEN, gdix51c0_cke_identity, sizeof (gdix51c0_cke_identity));
      fp_dbg ("gdix51c0: restored truncated PSK identity in CKE");
    }
}

static void
gdix51c0_note_handshake_progress (Gdix51c0Tls *tls,
                                  const guint8 *buf,
                                  gsize         len)
{
  gsize offset = 0;

  while (offset + 5 <= len)
    {
      guint8 content_type = buf[offset];
      gsize record_len = ((gsize) buf[offset + 3] << 8) | buf[offset + 4];

      if (content_type == 0x14)
        tls->handshake_saw_change_cipher_spec = TRUE;
      else if (content_type == 0x16 && record_len >= 4 &&
               offset + 5 + record_len <= len)
        {
          guint8 handshake_type = buf[offset + 5];

          if (handshake_type == 0x01)
            tls->handshake_saw_client_hello = TRUE;
          else if (handshake_type == 0x10)
            tls->handshake_saw_client_key_exchange = TRUE;
        }

      /* The GDIX51C0 malformed CCS advertises length zero but still carries
       * one byte.  It is always delivered alone, so do not try to interpret
       * that payload as another TLS record. */
      if (record_len == 0 || offset + 5 + record_len > len)
        break;
      offset += 5 + record_len;
    }
}

static int
gdix51c0_bio_read (BIO *b, char *buf, int len)
{
  Gdix51c0Tls *tls = BIO_get_data (b);

  BIO_clear_retry_flags (b);

  if (tls->rd_off >= tls->rd_len)
    {
      /* Refill from one SPI packet.  Wait for IRQ rise → read → fall. */
      GError *error = NULL;

      if (!gdix51c0_irq_wait (tls->irq_req, tls->irq_events, TRUE,
                           tls->irq_offset, GDIX51C0_BOOT_TIMEOUT_USEC,
                           "tls-read-rise", &error))
        {
          fp_warn ("gdix51c0: tls bio read rise: %s", error ? error->message : "?");
          g_clear_error (&error);
          return -1;
        }

        gsize plen = 0;
        guint8 *pkt = gdix51c0_spi_read (tls->dev, tls->spi_fd, &plen, &error);
        if (!pkt)
          {
            fp_warn ("gdix51c0: tls bio spi_read first attempt: %s",
                    error ? error->message : "?");
            g_clear_error (&error);

            {
              g_autoptr(GError) idle_error = NULL;

              gdix51c0_irq_drain (tls->irq_req, tls->irq_events);

              if (!gdix51c0_irq_wait (tls->irq_req,
                                      tls->irq_events,
                                      FALSE,
                                      tls->irq_offset,
                                      500000,
                                      "tls-pre-idle",
                                      &idle_error))
                {
                  fp_dbg ("gdix51c0: TLS pre-idle wait failed, continuing anyway: %s",
                          idle_error ? idle_error->message : "?");
                }

              gdix51c0_irq_drain (tls->irq_req, tls->irq_events);
            }

            if (!gdix51c0_irq_wait (tls->irq_req,
                                    tls->irq_events,
                                    TRUE,
                                    tls->irq_offset,
                                    GDIX51C0_BOOT_TIMEOUT_USEC,
                                    "tls-read-rise-retry",
                                    &error))
              {
                fp_warn ("gdix51c0: tls bio retry rise: %s",
                        error ? error->message : "?");
                g_clear_error (&error);
                return -1;
              }

            pkt = gdix51c0_spi_read (tls->dev, tls->spi_fd, &plen, &error);
            if (!pkt)
              {
                fp_warn ("gdix51c0: tls bio spi_read retry failed: %s",
                        error ? error->message : "?");
                g_clear_error (&error);
                return -1;
              }
          }

      gdix51c0_irq_wait (tls->irq_req, tls->irq_events, FALSE,
                      tls->irq_offset, GDIX51C0_BOOT_TIMEOUT_USEC,
                      "tls-read-fall", &error);
      g_clear_error (&error);

      gdix51c0_note_handshake_progress (tls, pkt, plen);
      gdix51c0_patch_inbound (pkt, plen);

      g_free (tls->rd_buf);
      tls->rd_buf = pkt;
      tls->rd_off = 0;
      tls->rd_len = plen;
    }

  gsize avail = tls->rd_len - tls->rd_off;
  gsize n = MIN ((gsize) len, avail);
  memcpy (buf, tls->rd_buf + tls->rd_off, n);
  tls->rd_off += n;
  return (int) n;
}

static long
gdix51c0_bio_ctrl (BIO *b, int cmd, long larg, void *parg)
{
  (void) b; (void) larg; (void) parg;
  switch (cmd)
    {
    case BIO_CTRL_FLUSH:
      return 1;
    default:
      return 0;
    }
}

static int
gdix51c0_bio_create (BIO *b)
{
  BIO_set_init (b, 1);
  BIO_set_data (b, NULL);
  return 1;
}

static int
gdix51c0_bio_destroy (BIO *b)
{
  if (!b)
    return 0;
  return 1;
}

static BIO_METHOD *
gdix51c0_bio_method (void)
{
  static BIO_METHOD *m = NULL;
  if (!m)
    {
      m = BIO_meth_new (BIO_TYPE_SOURCE_SINK, GDIX51C0_BIO_NAME);
      BIO_meth_set_write   (m, gdix51c0_bio_write);
      BIO_meth_set_read    (m, gdix51c0_bio_read);
      BIO_meth_set_ctrl    (m, gdix51c0_bio_ctrl);
      BIO_meth_set_create  (m, gdix51c0_bio_create);
      BIO_meth_set_destroy (m, gdix51c0_bio_destroy);
    }
  return m;
}

/* ---------------- PSK callback ---------------- */

static unsigned int
gdix51c0_psk_server_cb (SSL *ssl,
                     const char *identity,
                     unsigned char *out,
                     unsigned int max_len)
{
  Gdix51c0Tls *tls = SSL_get_app_data (ssl);

  if (!identity || g_strcmp0 (identity, GDIX51C0_PSK_IDENTITY) != 0)
    {
      fp_warn ("gdix51c0: tls: unexpected PSK identity");
      return 0;
    }
  if (!tls || tls->psk_len == 0 || tls->psk_len > max_len)
    return 0;
  memcpy (out, tls->psk, tls->psk_len);
  return (unsigned int) tls->psk_len;
}

/* ---------------- Public API ---------------- */

static GError *
gdix51c0_openssl_error (const char *what)
{
  unsigned long e = ERR_get_error ();
  char buf[256];
  if (e)
    ERR_error_string_n (e, buf, sizeof (buf));
  else
    g_strlcpy (buf, "unknown OpenSSL error", sizeof (buf));
  return g_error_new (G_IO_ERROR, G_IO_ERROR_FAILED, "gdix51c0: %s: %s", what, buf);
}

gboolean
gdix51c0_tls_init (Gdix51c0Tls *tls,
                FpDevice *dev,
                int spi_fd,
                struct gpiod_line_request      *irq_req,
                struct gpiod_edge_event_buffer *irq_events,
                unsigned int                    irq_offset,
                const guint8 *psk, gsize psk_len,
                GError **error)
{
  memset (tls, 0, sizeof (*tls));
  tls->dev        = dev;
  tls->spi_fd     = spi_fd;
  tls->irq_req    = irq_req;
  tls->irq_events = irq_events;
  tls->irq_offset = irq_offset;
  if (psk_len > sizeof (tls->psk))
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                   "gdix51c0: PSK too long (%zu > %zu)", psk_len, sizeof (tls->psk));
      return FALSE;
    }
  memcpy (tls->psk, psk, psk_len);
  tls->psk_len = psk_len;

  tls->ctx = SSL_CTX_new (TLS_server_method ());
  if (!tls->ctx)
    {
      g_propagate_error (error, gdix51c0_openssl_error ("SSL_CTX_new"));
      return FALSE;
    }
  SSL_CTX_set_min_proto_version (tls->ctx, TLS1_2_VERSION);
  SSL_CTX_set_max_proto_version (tls->ctx, TLS1_2_VERSION);
  if (SSL_CTX_set_cipher_list (tls->ctx, "PSK-AES128-GCM-SHA256") != 1)
    {
      g_propagate_error (error, gdix51c0_openssl_error ("set_cipher_list"));
      return FALSE;
    }
  /* Tell OpenSSL to skip cert verification — we have no cert chain on
   * either side; PSK alone authenticates. */
  SSL_CTX_set_verify (tls->ctx, SSL_VERIFY_NONE, NULL);
  SSL_CTX_set_psk_server_callback (tls->ctx, gdix51c0_psk_server_cb);
  SSL_CTX_use_psk_identity_hint (tls->ctx, GDIX51C0_PSK_IDENTITY);

  tls->ssl = SSL_new (tls->ctx);
  if (!tls->ssl)
    {
      g_propagate_error (error, gdix51c0_openssl_error ("SSL_new"));
      return FALSE;
    }

  /* SSL_set_app_data is shorthand for ex_data slot 0 — that's how the
   * PSK callback recovers our Gdix51c0Tls (which holds the PSK). */
  SSL_set_app_data (tls->ssl, tls);

  tls->bio = BIO_new (gdix51c0_bio_method ());
  if (!tls->bio)
    {
      g_propagate_error (error, gdix51c0_openssl_error ("BIO_new"));
      return FALSE;
    }
  BIO_set_data (tls->bio, tls);
  SSL_set_bio (tls->ssl, tls->bio, tls->bio);
  /* tls->bio is now owned by ssl */
  SSL_set_accept_state (tls->ssl);
  return TRUE;
}

static gboolean
gdix51c0_tls_recreate_ssl (Gdix51c0Tls *tls, GError **error)
{
  g_free (tls->rd_buf);
  tls->rd_buf = NULL;
  tls->rd_off = 0;
  tls->rd_len = 0;
  tls->handshake_saw_client_hello = FALSE;
  tls->handshake_saw_client_key_exchange = FALSE;
  tls->handshake_saw_change_cipher_spec = FALSE;

  if (tls->ssl)
    {
      SSL_free (tls->ssl);
      tls->ssl = NULL;
      tls->bio = NULL; /* owned/freed by SSL */
    }

  tls->ssl = SSL_new (tls->ctx);
  if (!tls->ssl)
    {
      g_propagate_error (error, gdix51c0_openssl_error ("SSL_new retry"));
      return FALSE;
    }

  SSL_set_app_data (tls->ssl, tls);

  tls->bio = BIO_new (gdix51c0_bio_method ());
  if (!tls->bio)
    {
      g_propagate_error (error, gdix51c0_openssl_error ("BIO_new retry"));
      return FALSE;
    }

  BIO_set_data (tls->bio, tls);
  SSL_set_bio (tls->ssl, tls->bio, tls->bio);
  SSL_set_accept_state (tls->ssl);

  return TRUE;
}

gboolean
gdix51c0_tls_handshake (Gdix51c0Tls *tls, GError **error)
{
  static const guint8 d1_payload[] = {
    0xd1, 0x03, 0x00, 0x00, 0x00, 0xd7
  };
  gboolean first_complete_auth_record_failure = FALSE;

  for (guint attempt = 0; attempt < 2; attempt++)
    {
      g_autoptr(GError) local_error = NULL;

      fp_dbg ("gdix51c0: TLS handshake attempt %u", attempt + 1);

      if (attempt > 0)
        {
          if (!gdix51c0_tls_recreate_ssl (tls, &local_error))
            {
              g_propagate_error (error, g_steal_pointer (&local_error));
              return FALSE;
            }

          g_usleep (200000);
        }

      gdix51c0_irq_drain (tls->irq_req, tls->irq_events);

      if (!gdix51c0_irq_wait (tls->irq_req,
                              tls->irq_events,
                              FALSE,
                              tls->irq_offset,
                              500000,
                              "tls-pre-idle",
                              &local_error))
        {
          fp_dbg ("gdix51c0: TLS pre-idle wait failed, continuing anyway: %s",
                  local_error ? local_error->message : "?");
          g_clear_error (&local_error);
        }

      gdix51c0_irq_drain (tls->irq_req, tls->irq_events);

      /* The D0/no-ACK TLS trigger still goes through
       * SpiSendDataToDevice, which applies its 1 ms pre-submit delay. */
      g_usleep (1000);
      if (!gdix51c0_spi_write (tls->dev,
                               tls->spi_fd,
                               GDIX51C0_PKT_WRITE,
                               d1_payload,
                               sizeof (d1_payload),
                               &local_error))
        {
          g_propagate_error (error, g_steal_pointer (&local_error));
          return FALSE;
        }

      ERR_clear_error ();
      int rc = SSL_accept (tls->ssl);
      if (rc > 0)
        {
          fp_dbg ("gdix51c0: TLS handshake complete (cipher=%s)",
                  SSL_get_cipher (tls->ssl));

          g_free (tls->rd_buf);
          tls->rd_buf = NULL;
          tls->rd_off = 0;
          tls->rd_len = 0;

          gdix51c0_irq_drain (tls->irq_req, tls->irq_events);
          return TRUE;
        }

      int ssl_err = SSL_get_error (tls->ssl, rc);
      unsigned long queue_error;
      gboolean saw_concrete_auth_failure = FALSE;
      gboolean saw_record_layer_failure = FALSE;
      gboolean complete_client_flight =
        tls->handshake_saw_client_hello &&
        tls->handshake_saw_client_key_exchange &&
        tls->handshake_saw_change_cipher_spec;
      GString *queue_reasons = g_string_new (NULL);

      /* OpenSSL 3.5 adds SSL_R_RECORD_LAYER_FAILURE after the useful lower
       * record error. Consume and inspect the entire queue instead of looking
       * only at its final, generic entry. */
      while ((queue_error = ERR_get_error ()) != 0)
        {
          int queue_reason = ERR_GET_REASON (queue_error);
          const char *queue_reason_text = ERR_reason_error_string (queue_error);

          if (queue_reasons->len > 0)
            g_string_append (queue_reasons, " | ");
          g_string_append (queue_reasons,
                           queue_reason_text ? queue_reason_text : "unknown");

          if (queue_reason == SSL_R_DECRYPTION_FAILED_OR_BAD_RECORD_MAC ||
              queue_reason == SSL_R_DECRYPTION_FAILED)
            saw_concrete_auth_failure = TRUE;
          else if (queue_reason == SSL_R_RECORD_LAYER_FAILURE)
            saw_record_layer_failure = TRUE;
        }

      fp_warn ("gdix51c0: SSL_accept attempt %u rc=%d ssl_err=%d "
               "reasons=%s progress=hello:%u cke:%u ccs:%u",
               attempt + 1, rc, ssl_err,
               queue_reasons->len ? queue_reasons->str : "none",
               tls->handshake_saw_client_hello,
               tls->handshake_saw_client_key_exchange,
               tls->handshake_saw_change_cipher_spec);

      gdix51c0_irq_drain (tls->irq_req, tls->irq_events);

      if (attempt == 0)
        {
          /* A generic record-layer failure can also mean corruption. Never
           * authorize a PSK write from one observation: require the complete
           * client flight and the same failure on the independent retry. */
          first_complete_auth_record_failure =
            ssl_err == SSL_ERROR_SSL &&
            complete_client_flight &&
            (saw_concrete_auth_failure || saw_record_layer_failure);
          g_string_free (queue_reasons, TRUE);
          continue;
        }

      if (ssl_err == SSL_ERROR_SSL &&
          first_complete_auth_record_failure &&
          complete_client_flight &&
          (saw_concrete_auth_failure ||
           saw_record_layer_failure))
        {
          g_set_error (error,
                       G_IO_ERROR,
                       G_IO_ERROR_PERMISSION_DENIED,
                       "gdix51c0: TLS PSK authentication mismatch "
                       "after two complete client flights (%s)",
                       queue_reasons->len ? queue_reasons->str : "no reason");
          g_string_free (queue_reasons, TRUE);
          return FALSE;
        }

      g_set_error (error,
                   G_IO_ERROR,
                   G_IO_ERROR_CONNECTION_CLOSED,
                   "gdix51c0: TLS handshake failed without a confirmed PSK "
                   "mismatch (ssl_err=%d, reasons=%s)",
                   ssl_err,
                   queue_reasons->len ? queue_reasons->str : "none");
      g_string_free (queue_reasons, TRUE);
      return FALSE;
    }

  g_set_error_literal (error,
                       G_IO_ERROR,
                       G_IO_ERROR_CONNECTION_CLOSED,
                       "gdix51c0: TLS handshake failed");
  return FALSE;
}

gboolean
gdix51c0_tls_record_header_ok (const guint8 *record, gsize record_len)
{
  if (!record || record_len < 5)
    return FALSE;

  /* TLS record content types used here:
   * 0x14 ChangeCipherSpec
   * 0x15 Alert
   * 0x16 Handshake
   * 0x17 ApplicationData
   */
  if (record[0] != 0x14 &&
      record[0] != 0x15 &&
      record[0] != 0x16 &&
      record[0] != 0x17)
    return FALSE;

  if (record[1] != 0x03)
    return FALSE;

  guint16 tls_len = ((guint16) record[3] << 8) | record[4];

  if ((gsize) tls_len + 5 > record_len)
    return FALSE;

  return TRUE;
}

guint8 *
gdix51c0_tls_decrypt_record (Gdix51c0Tls *tls,
                             const guint8 *record,
                             gsize record_len,
                             gsize *out_len,
                             GError **error)
{
  if (!gdix51c0_tls_record_header_ok (record, record_len))
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                   "gdix51c0: invalid TLS record framing (len=%zu)",
                   record_len);
      return NULL;
    }

  g_free (tls->rd_buf);
  tls->rd_buf = g_memdup2 (record, record_len);
  tls->rd_off = 0;
  tls->rd_len = record_len;

  guint8 *plain = g_malloc (record_len);
  int n = SSL_read (tls->ssl, plain, (int) record_len);

  if (n <= 0)
    {
      int ssl_err = SSL_get_error (tls->ssl, n);
      g_free (plain);

      g_set_error (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                   "gdix51c0: SSL_read failed ssl_err=%d openssl=%s",
                   ssl_err,
                   ERR_error_string (ERR_get_error (), NULL));
      return NULL;
    }

  if (out_len)
    *out_len = (gsize) n;

  return plain;
}

void
gdix51c0_tls_free (Gdix51c0Tls *tls)
{
  if (!tls) return;
  if (tls->ssl)
    {
      SSL_free (tls->ssl);
      tls->ssl = NULL;
      tls->bio = NULL; /* freed via SSL_free */
    }
  if (tls->ctx)
    {
      SSL_CTX_free (tls->ctx);
      tls->ctx = NULL;
    }
  g_clear_pointer (&tls->rd_buf, g_free);
}
