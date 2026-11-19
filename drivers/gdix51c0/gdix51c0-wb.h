// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * Goodix GDIX51C0 — PSK white-box wrap (host-side re-provisioning)
 *
 * Copyright (C) 2026 Berke Kabagöz <berkekbgz@gmail.com>
 */

#pragma once

#include <glib.h>

G_BEGIN_DECLS

#define GDIX51C0_PSK_LEN 32
#define GDIX51C0_WB_LEN  102

/* MCU register addresses (little-endian on the wire). */
#define GDIX51C0_PSK_WB_ADDR   0xbb010003u  /* stores the 102-byte WB */
#define GDIX51C0_PSK_HASH_ADDR 0xbb020003u  /* MCU-computed SHA-256(WB) */
#define GDIX51C0_PSK_HASH_TAG_14103 0x0000aaaau  /* type fw 14103 returns for it */

/*
 * Wrap a chosen 32-byte TLS PSK into the 102-byte white-box container the MCU
 * accepts on cmd 0xbb010003.  This is a full reimplementation of gfspi.dll's
 * whitebox_wrap (@0x180006bb0): it turned out to be standard AES-256-GCM plus
 * HMAC-SHA256 with three constants baked into the DLL — see re/PARITY.md and
 * re/wb_wrap.py (the byte-exact Python reference).
 *
 * out_wb must point to at least GDIX51C0_WB_LEN bytes.  Returns FALSE and sets
 * @error on any OpenSSL failure.
 */
gboolean gdix51c0_wb_wrap (const guint8 *psk,   /* GDIX51C0_PSK_LEN bytes */
                           guint8       *out_wb, /* GDIX51C0_WB_LEN bytes  */
                           GError      **error);

/*
 * SHA-256 of the 102-byte WB container.  This is exactly what the MCU stores
 * at 0xbb020003 after a successful write to 0xbb010003, so a read-back of that
 * register can be compared against this to confirm provisioning took.
 * @out must be 32 bytes.
 */
gboolean gdix51c0_wb_hash (const guint8 *wb, guint8 out[32], GError **error);

/*
 * Inverse of gdix51c0_wb_wrap: recover the 32-byte PSK from a 102-byte WB
 * container, authenticating it fully (front HMAC-SHA256, the AES-256-GCM tag,
 * and the psk[0:8]-binding h16). This is useful for operator-supplied or
 * captured containers. GDIX51C0 hardware reports register 0xbb010003 as empty,
 * so normal driver operation cannot use it to recover the sensor's live PSK.
 *
 * Returns FALSE (with @error) if any authentication check fails — i.e. the blob
 * is not a valid WB for our key.  @out_psk must be 32 bytes.
 */
gboolean gdix51c0_wb_unwrap (const guint8 *wb, guint8 out_psk[32], GError **error);

/* Factory PMK record (GXFP51A0 / GF_HC460SEC_APP_1410x, flash 0xC000):
 *   u32 type | u32 body_len | body = salt[16] || AES-128-CBC ciphertext
 * key = SHA256(salt || 48 zero bytes || family seed)[:16], iv = salt,
 * plaintext = 00 0d | u32 BE 0x30 | 48-byte PMK, which is the TLS PSK. */
#define GDIX51C0_FACTORY_PMK_ADDR     0x0000c000u
#define GDIX51C0_FACTORY_BODY_OFFSET  8u
#define GDIX51C0_FACTORY_BLOB_LEN     80u  /* salt + 4 blocks: enough for the PMK */
#define GDIX51C0_FACTORY_PMK_LEN      48u

gboolean gdix51c0_factory_pmk_decrypt (const guint8 blob[GDIX51C0_FACTORY_BLOB_LEN],
                                       guint8       pmk[GDIX51C0_FACTORY_PMK_LEN]);

/*
 * Build the payload *data* for the 0xe0 register-write that provisions the WB:
 *
 *   addr(LE32=0xbb010003) | len(LE32=102) | WB(102) | zero-pad to 4-byte align
 *
 * This is the argument you hand to gdix51c0_make_payload_packet(0xe0, ...).
 * Wraps @psk internally.  Returns a g_malloc'd buffer (caller frees) and sets
 * @out_len; NULL on error.
 */
guint8 *gdix51c0_wb_provision_data (const guint8 *psk, gsize *out_len, GError **error);

/*
 * Self-check: wrap a deliberately synthetic public PSK and compare against its
 * oracle-derived WB. Returns TRUE if the container matches byte-for-byte.
 * Cheap; call once before provisioning to fail loudly if OpenSSL misbehaves.
 */
gboolean gdix51c0_wb_selftest (GError **error);

G_END_DECLS
