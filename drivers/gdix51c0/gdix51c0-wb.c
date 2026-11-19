// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * Goodix GDIX51C0 — PSK white-box wrap (host-side re-provisioning)
 *
 * Copyright (C) 2026 Berke Kabagöz <berkekbgz@gmail.com>
 *
 * Reimplements gfspi.dll whitebox_wrap (@0x180006bb0).  Statically the routine
 * is a white-box, but it reduces to standard AES-256-GCM + HMAC-SHA256 with
 * three constants recovered from the DLL (re/PARITY.md). Layout of the
 * 102-byte container:
 *
 *   front(32) | hdr(2) | len(4) | h16(16) | ct(32) | tag(16)
 *
 *   hdr    = 02 ff
 *   len    = uint32_le(32)
 *   h16    = SHA256(hdr | len | psk[0:8] | (03000000)x16)[0:16]
 *   ct,tag = AES-256-GCM(KEY, iv=h16, aad=AAD, plaintext=psk)   (16-byte IV)
 *   front  = HMAC-SHA256(HMAC_KEY, hdr | len | ct | tag)
 *
 * The Python reference re/wb_wrap.py matches this byte-for-byte against the
 * DLL oracle for arbitrary PSKs; gdix51c0_wb_selftest() re-checks a synthetic
 * public vector at runtime.
 */

#include <string.h>

#include <gio/gio.h>
#include <openssl/crypto.h>
#include <openssl/evp.h>
#include <openssl/hmac.h>

#include "gdix51c0-wb.h"

/* 48-byte key buffer branch B derives (constant; dumped via re/wb_oracle.py).
 * KEY = KBUF[0:32], HMAC_KEY = KBUF[16:48]. */
static const guint8 GDIX51C0_WB_KBUF[48] = {
  0x58, 0xf0, 0x13, 0xee, 0xb7, 0xe2, 0x16, 0xe2,
  0xc1, 0xcd, 0x0e, 0x8f, 0xff, 0xa7, 0xdb, 0xd6,
  0x79, 0x9c, 0xd9, 0x2a, 0xa1, 0x49, 0x01, 0x3e,
  0xa0, 0xe9, 0xf9, 0xfd, 0x7d, 0xc6, 0xd9, 0x4e,
  0x3e, 0xf6, 0x3a, 0x75, 0x98, 0xc2, 0xa4, 0x93,
  0x31, 0x29, 0xe8, 0x71, 0xea, 0x03, 0x04, 0x3b,
};

/* Constant GCM additional-authenticated-data. */
static const guint8 GDIX51C0_WB_AAD[16] = {
  0x52, 0x2d, 0xc1, 0xf0, 0x99, 0x56, 0x7d, 0x07,
  0xf4, 0x7f, 0x37, 0xa3, 0x2a, 0x84, 0x42, 0x7d,
};

#define WB_KEY      (GDIX51C0_WB_KBUF)        /* AES-256 key, 32 bytes */
#define WB_HMAC_KEY (GDIX51C0_WB_KBUF + 16)   /* HMAC key,    32 bytes */

/* Field offsets inside the 102-byte container. */
#define WB_OFF_FRONT 0x00
#define WB_OFF_HDR   0x20
#define WB_OFF_LEN   0x22
#define WB_OFF_H16   0x26
#define WB_OFF_CT    0x36
#define WB_OFF_TAG   0x56

static gboolean
wb_sha256 (const guint8 *data, gsize len, guint8 out[32], GError **error)
{
  unsigned int outlen = 0;

  if (EVP_Digest (data, len, out, &outlen, EVP_sha256 (), NULL) != 1 ||
      outlen != 32)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: wb: SHA-256 failed");
      return FALSE;
    }
  return TRUE;
}

static gboolean
wb_aes256_gcm (const guint8  key[32],
               const guint8  iv[16],
               const guint8  aad[16],
               const guint8 *pt, gsize pt_len,
               guint8       *ct_out,   /* pt_len bytes */
               guint8        tag_out[16],
               GError      **error)
{
  EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new ();
  gboolean ok = FALSE;
  int outl = 0;

  if (!ctx)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: wb: EVP_CIPHER_CTX_new failed");
      return FALSE;
    }

  /* 16-byte IV (non-default), so set the length before binding key+IV. */
  if (EVP_EncryptInit_ex (ctx, EVP_aes_256_gcm (), NULL, NULL, NULL) != 1 ||
      EVP_CIPHER_CTX_ctrl (ctx, EVP_CTRL_GCM_SET_IVLEN, 16, NULL) != 1 ||
      EVP_EncryptInit_ex (ctx, NULL, NULL, key, iv) != 1)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: wb: GCM init failed");
      goto out;
    }

  if (EVP_EncryptUpdate (ctx, NULL, &outl, aad, 16) != 1 ||
      EVP_EncryptUpdate (ctx, ct_out, &outl, pt, (int) pt_len) != 1 ||
      EVP_EncryptFinal_ex (ctx, ct_out + outl, &outl) != 1 ||
      EVP_CIPHER_CTX_ctrl (ctx, EVP_CTRL_GCM_GET_TAG, 16, tag_out) != 1)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: wb: GCM encrypt failed");
      goto out;
    }

  ok = TRUE;
out:
  EVP_CIPHER_CTX_free (ctx);
  return ok;
}

gboolean
gdix51c0_wb_wrap (const guint8 *psk, guint8 *out_wb, GError **error)
{
  static const guint8 pad4[4] = { 0x03, 0x00, 0x00, 0x00 };
  guint8 sha_in[2 + 4 + 8 + 64];
  guint8 h32[32];
  guint8 *hdr = out_wb + WB_OFF_HDR;
  guint8 *len = out_wb + WB_OFF_LEN;
  gsize p = 0;
  unsigned int mac_len = 0;

  g_return_val_if_fail (psk != NULL, FALSE);
  g_return_val_if_fail (out_wb != NULL, FALSE);

  /* header + little-endian length, written in place. */
  hdr[0] = 0x02;
  hdr[1] = 0xff;
  len[0] = GDIX51C0_PSK_LEN;   /* 0x20 */
  len[1] = len[2] = len[3] = 0x00;

  /* h16 = SHA256(hdr | len | psk[0:8] | (03000000)x16)[0:16] */
  memcpy (sha_in + p, hdr, 2); p += 2;
  memcpy (sha_in + p, len, 4); p += 4;
  memcpy (sha_in + p, psk, 8); p += 8;
  for (int i = 0; i < 16; i++)
    {
      memcpy (sha_in + p, pad4, 4);
      p += 4;
    }
  if (!wb_sha256 (sha_in, p, h32, error))
    return FALSE;
  memcpy (out_wb + WB_OFF_H16, h32, 16);

  /* ct,tag = AES-256-GCM(KEY, iv=h16, aad=AAD, pt=psk) */
  if (!wb_aes256_gcm (WB_KEY, out_wb + WB_OFF_H16, GDIX51C0_WB_AAD,
                      psk, GDIX51C0_PSK_LEN,
                      out_wb + WB_OFF_CT, out_wb + WB_OFF_TAG, error))
    return FALSE;

  /* front = HMAC-SHA256(HMAC_KEY, hdr | len | ct | tag) */
  {
    guint8 mac_in[2 + 4 + 32 + 16];
    gsize m = 0;
    memcpy (mac_in + m, hdr, 2);                 m += 2;
    memcpy (mac_in + m, len, 4);                 m += 4;
    memcpy (mac_in + m, out_wb + WB_OFF_CT, 32); m += 32;
    memcpy (mac_in + m, out_wb + WB_OFF_TAG, 16); m += 16;

    if (HMAC (EVP_sha256 (), WB_HMAC_KEY, 32, mac_in, m,
              out_wb + WB_OFF_FRONT, &mac_len) == NULL || mac_len != 32)
      {
        g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                             "gdix51c0: wb: HMAC-SHA256 failed");
        return FALSE;
      }
  }

  return TRUE;
}

gboolean
gdix51c0_wb_hash (const guint8 *wb, guint8 out[32], GError **error)
{
  g_return_val_if_fail (wb != NULL, FALSE);
  return wb_sha256 (wb, GDIX51C0_WB_LEN, out, error);
}

static gboolean
wb_aes256_gcm_decrypt (const guint8  key[32],
                       const guint8  iv[16],
                       const guint8  aad[16],
                       const guint8 *ct, gsize ct_len,
                       const guint8  tag[16],
                       guint8       *pt_out,   /* ct_len bytes */
                       GError      **error)
{
  EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new ();
  gboolean ok = FALSE;
  int outl = 0;

  if (!ctx)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: wb: EVP_CIPHER_CTX_new failed");
      return FALSE;
    }

  if (EVP_DecryptInit_ex (ctx, EVP_aes_256_gcm (), NULL, NULL, NULL) != 1 ||
      EVP_CIPHER_CTX_ctrl (ctx, EVP_CTRL_GCM_SET_IVLEN, 16, NULL) != 1 ||
      EVP_DecryptInit_ex (ctx, NULL, NULL, key, iv) != 1 ||
      EVP_DecryptUpdate (ctx, NULL, &outl, aad, 16) != 1 ||
      EVP_DecryptUpdate (ctx, pt_out, &outl, ct, (int) ct_len) != 1 ||
      EVP_CIPHER_CTX_ctrl (ctx, EVP_CTRL_GCM_SET_TAG, 16, (void *) tag) != 1)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: wb: GCM decrypt setup failed");
      goto out;
    }

  /* Returns <= 0 when the GCM tag does not verify. */
  if (EVP_DecryptFinal_ex (ctx, pt_out + outl, &outl) <= 0)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: wb: GCM tag verification failed");
      goto out;
    }

  ok = TRUE;
out:
  EVP_CIPHER_CTX_free (ctx);
  return ok;
}

gboolean
gdix51c0_wb_unwrap (const guint8 *wb, guint8 out_psk[32], GError **error)
{
  static const guint8 pad4[4] = { 0x03, 0x00, 0x00, 0x00 };
  const guint8 *front = wb + WB_OFF_FRONT;
  const guint8 *hdr   = wb + WB_OFF_HDR;
  const guint8 *len   = wb + WB_OFF_LEN;
  const guint8 *h16   = wb + WB_OFF_H16;
  const guint8 *ct    = wb + WB_OFF_CT;
  const guint8 *tag   = wb + WB_OFF_TAG;
  guint8 mac_in[2 + 4 + 32 + 16];
  guint8 mac[32];
  unsigned int mac_len = 0;
  guint8 h32[32];
  guint8 sha_in[2 + 4 + 8 + 64];
  gsize p = 0;

  g_return_val_if_fail (wb != NULL, FALSE);
  g_return_val_if_fail (out_psk != NULL, FALSE);

  if (hdr[0] != 0x02 || hdr[1] != 0xff || len[0] != GDIX51C0_PSK_LEN)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                           "gdix51c0: wb: not a PSK WB (bad header/length)");
      return FALSE;
    }

  /* 1) authenticate front = HMAC-SHA256(HMAC_KEY, hdr|len|ct|tag). */
  memcpy (mac_in + 0, hdr, 2);
  memcpy (mac_in + 2, len, 4);
  memcpy (mac_in + 6, ct, 32);
  memcpy (mac_in + 38, tag, 16);
  if (HMAC (EVP_sha256 (), WB_HMAC_KEY, 32, mac_in, sizeof (mac_in),
            mac, &mac_len) == NULL || mac_len != 32)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: wb: HMAC-SHA256 failed");
      return FALSE;
    }
  if (CRYPTO_memcmp (front, mac, 32) != 0)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                           "gdix51c0: wb: front HMAC mismatch (wrong key or corrupt)");
      return FALSE;
    }

  /* 2) AES-256-GCM decrypt (iv = h16, aad = AAD); verifies the tag. */
  if (!wb_aes256_gcm_decrypt (WB_KEY, h16, GDIX51C0_WB_AAD, ct, 32, tag,
                              out_psk, error))
    return FALSE;

  /* 3) bind psk[0:8]: recompute h16 and compare. */
  memcpy (sha_in + p, hdr, 2); p += 2;
  memcpy (sha_in + p, len, 4); p += 4;
  memcpy (sha_in + p, out_psk, 8); p += 8;
  for (int i = 0; i < 16; i++)
    {
      memcpy (sha_in + p, pad4, 4);
      p += 4;
    }
  if (!wb_sha256 (sha_in, p, h32, error))
    return FALSE;
  if (CRYPTO_memcmp (h32, h16, 16) != 0)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                           "gdix51c0: wb: h16 mismatch");
      return FALSE;
    }

  return TRUE;
}

guint8 *
gdix51c0_wb_provision_data (const guint8 *psk, gsize *out_len, GError **error)
{
  guint8 wb[GDIX51C0_WB_LEN];
  /* addr(4) + len(4) + WB(102) = 110, padded up to a 4-byte boundary = 112. */
  const gsize head = 8;
  const gsize raw = head + GDIX51C0_WB_LEN;
  const gsize padded = (raw + 3) & ~((gsize) 3);
  guint8 *data;

  g_return_val_if_fail (psk != NULL, NULL);

  if (!gdix51c0_wb_wrap (psk, wb, error))
    return NULL;

  data = g_malloc0 (padded);
  /* little-endian addr + length, matching the Python 0xe0 register-write. */
  data[0] = (guint8) (GDIX51C0_PSK_WB_ADDR & 0xff);
  data[1] = (guint8) ((GDIX51C0_PSK_WB_ADDR >> 8) & 0xff);
  data[2] = (guint8) ((GDIX51C0_PSK_WB_ADDR >> 16) & 0xff);
  data[3] = (guint8) ((GDIX51C0_PSK_WB_ADDR >> 24) & 0xff);
  data[4] = (guint8) (GDIX51C0_WB_LEN & 0xff);
  data[5] = data[6] = data[7] = 0x00;
  memcpy (data + head, wb, GDIX51C0_WB_LEN);
  /* tail bytes [raw, padded) stay zero from g_malloc0. */

  if (out_len)
    *out_len = padded;
  return data;
}

gboolean
gdix51c0_wb_selftest (GError **error)
{
  static const guint8 sample_psk[GDIX51C0_PSK_LEN] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
  };
  static const guint8 sample_wb[GDIX51C0_WB_LEN] = {
    0x45, 0xa3, 0x73, 0x2a, 0xcf, 0x29, 0x90, 0xf4,
    0xce, 0xb3, 0x0e, 0x89, 0x96, 0xe4, 0xad, 0xd1,
    0x2e, 0x88, 0x71, 0x60, 0xd6, 0xdc, 0xaa, 0x99,
    0xc8, 0xbc, 0xd8, 0x33, 0x8c, 0x77, 0x43, 0xf2,
    0x02, 0xff, 0x20, 0x00, 0x00, 0x00, 0xe7, 0xbc,
    0x09, 0x96, 0x2a, 0xed, 0xfb, 0x47, 0x0f, 0xac,
    0x38, 0x87, 0xde, 0x32, 0x46, 0x5c, 0x5a, 0x71,
    0xc8, 0x8f, 0xe0, 0x1d, 0xd7, 0x76, 0xcf, 0xa5,
    0x19, 0xe7, 0x0c, 0x63, 0xf8, 0x24, 0x01, 0xf8,
    0x24, 0x73, 0x24, 0xfd, 0xe9, 0xc6, 0x56, 0x23,
    0x21, 0xd4, 0xd8, 0x2b, 0x0d, 0xd5, 0x52, 0xc0,
    0xbf, 0x1f, 0x74, 0x71, 0x60, 0x85, 0xb8, 0x52,
    0x65, 0x8b, 0xfe, 0x77, 0x29, 0x64,
  };
  guint8 wb[GDIX51C0_WB_LEN];

  if (!gdix51c0_wb_wrap (sample_psk, wb, error))
    return FALSE;

  if (memcmp (wb, sample_wb, GDIX51C0_WB_LEN) != 0)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: wb: self-test mismatch (bad KBUF/AAD or OpenSSL)");
      return FALSE;
    }
  return TRUE;
}

/* Family constant the firmware computes at runtime (seed generator at 0x2fc2c in
 * GF_HC460SEC_APP_14104, emulated); identical to the ST411 "fallback seed". */
static const guint8 GDIX51C0_FACTORY_SEED[16] = {
  0x5c, 0xba, 0x6e, 0x25, 0x81, 0x95, 0x18, 0xde,
  0x2d, 0x53, 0xe9, 0x6d, 0xc0, 0x34, 0x7a, 0xb0
};

gboolean
gdix51c0_factory_pmk_decrypt (const guint8 blob[GDIX51C0_FACTORY_BLOB_LEN],
                              guint8       pmk[GDIX51C0_FACTORY_PMK_LEN])
{
  guint8 kdf_in[80] = { 0 };
  guint8 digest[32];
  guint8 plain[GDIX51C0_FACTORY_BLOB_LEN - 16];
  unsigned int dlen = 0;
  int n1 = 0, n2 = 0;
  gboolean ok = FALSE;
  EVP_CIPHER_CTX *ctx = NULL;

  memcpy (kdf_in, blob, 16);
  memcpy (kdf_in + 64, GDIX51C0_FACTORY_SEED, sizeof GDIX51C0_FACTORY_SEED);
  if (EVP_Digest (kdf_in, sizeof kdf_in, digest, &dlen, EVP_sha256 (), NULL) != 1)
    goto out;

  ctx = EVP_CIPHER_CTX_new ();
  if (!ctx ||
      EVP_DecryptInit_ex (ctx, EVP_aes_128_cbc (), NULL, digest, blob) != 1 ||
      EVP_CIPHER_CTX_set_padding (ctx, 0) != 1 ||
      EVP_DecryptUpdate (ctx, plain, &n1, blob + 16, sizeof plain) != 1 ||
      EVP_DecryptFinal_ex (ctx, plain + n1, &n2) != 1 ||
      n1 + n2 != (int) sizeof plain)
    goto out;

  if (plain[0] != 0x00 || plain[1] != 0x0d ||
      plain[2] != 0x00 || plain[3] != 0x00 || plain[4] != 0x00 ||
      plain[5] != GDIX51C0_FACTORY_PMK_LEN)
    goto out;

  memcpy (pmk, plain + 6, GDIX51C0_FACTORY_PMK_LEN);
  ok = TRUE;
out:
  EVP_CIPHER_CTX_free (ctx);
  OPENSSL_cleanse (kdf_in, sizeof kdf_in);
  OPENSSL_cleanse (digest, sizeof digest);
  OPENSSL_cleanse (plain, sizeof plain);
  return ok;
}
