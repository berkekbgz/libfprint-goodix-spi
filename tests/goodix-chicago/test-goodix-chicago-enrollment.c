// SPDX-License-Identifier: LGPL-2.1-or-later
/* Copyright (C) 2026 Berke Kabagöz <berkekbgz@gmail.com> */

#include <glib.h>
#include <gio/gio.h>
#include <stdio.h>
#include <string.h>

#include "goodix-chicago-enrollment.h"

typedef struct
{
  guint32 magic;
  guint32 version;
  gint32 transform[6];
  gint32 outputs[5];
  GoodixChicagoMetricData gallery;
  GoodixChicagoMetricData probe;
} IdentifyMetricVector;

typedef struct
{
  guint32 magic;
  guint32 version;
  guint8 auxiliary[6];
  guint8 reserved[2];
  gint32 transform[6];
  gint32 counts[3];
} LiveAuxiliaryCountVector;

G_STATIC_ASSERT (sizeof (GoodixChicagoMetricData) == 684);
G_STATIC_ASSERT (sizeof (IdentifyMetricVector) == 1420);
G_STATIC_ASSERT (sizeof (LiveAuxiliaryCountVector) == 52);

static guint32
test_packed_crc32 (const guint8 *data,
                   gsize         size)
{
  guint32 crc = G_MAXUINT32;

  for (gsize offset = 0; offset < size; offset++)
    {
      crc ^= data[offset];
      for (guint bit = 0; bit < 8; bit++)
        crc = (crc >> 1) ^ (0xedb88320u & -(crc & 1u));
    }
  return ~crc;
}

static void
test_adaptive_metadata_roundtrip (void)
{
  static const guint8 metadata_pattern[] = {
    0xba, 0, 0, 0, 0,
    0xbb, 0, 0, 0, 0,
    0xbc, 0, 0, 0, 0,
    0xbd, 0, 0, 0, 0,
    0xbe, 0, 0, 0, 0,
    0xc0, 0, 0, 0, 0,
  };
  g_autoptr(GoodixChicagoEnrollment) source =
    goodix_chicago_enrollment_new ();
  g_autoptr(GoodixChicagoEnrollment) parsed = NULL;
  GoodixChicagoFeatureRecord record = { 0, };
  GoodixChicagoMetricData metric_data = { 0, };
  GoodixChicagoEnrollmentResult result;
  GoodixChicagoSubtemplateView view;
  g_autoptr(GBytes) packed = NULL;
  g_autoptr(GBytes) repacked = NULL;
  g_autofree guint8 *modified = NULL;
  g_autoptr(GError) error = NULL;
  const guint8 *packed_data;
  const guint8 *repacked_data;
  gsize packed_size;
  gsize repacked_size;
  gsize metadata_offset = G_MAXSIZE;
  guint32 crc;

  g_assert_true (goodix_chicago_enrollment_insert_first (
    source, &record, 1, 1, 85, 100, &metric_data, &result, &error));
  g_assert_no_error (error);
  packed = goodix_chicago_enrollment_pack (source, &error);
  g_assert_no_error (error);
  packed_data = g_bytes_get_data (packed, &packed_size);
  modified = g_memdup2 (packed_data, packed_size);
  for (gsize offset = 0;
       offset + sizeof (metadata_pattern) <= packed_size;
       offset++)
    if (memcmp (modified + offset, metadata_pattern,
                sizeof (metadata_pattern)) == 0)
      {
        g_assert_cmpuint (metadata_offset, ==, G_MAXSIZE);
        metadata_offset = offset;
      }
  g_assert_cmpuint (metadata_offset, !=, G_MAXSIZE);

  modified[metadata_offset + 1] = 1;  /* appended by templateStudy */
  modified[metadata_offset + 6] = 2;
  modified[metadata_offset + 16] = 3;
  modified[metadata_offset + 21] = 4;
  modified[metadata_offset + 26] = 5;
  crc = GUINT32_TO_LE (test_packed_crc32 (modified + 10,
                                          packed_size - 10));
  memcpy (modified + 1, &crc, sizeof (crc));

  parsed = goodix_chicago_enrollment_unpack (modified, packed_size,
                                                &error);
  g_assert_no_error (error);
  g_assert_nonnull (parsed);
  g_assert_true (goodix_chicago_enrollment_get_subtemplate (parsed, 0,
                                                               &view));
  g_assert_cmpuint (view.study_state, ==, 1);
  g_assert_cmpuint (view.study_flags, ==, 2);
  g_assert_cmpuint (view.lineage_index, ==, 0);
  g_assert_cmpuint (view.replacement_count, ==, 3);
  g_assert_cmpuint (view.study_value_a, ==, 4);
  g_assert_cmpuint (view.study_value_b, ==, 5);

  repacked = goodix_chicago_enrollment_pack (parsed, &error);
  g_assert_no_error (error);
  repacked_data = g_bytes_get_data (repacked, &repacked_size);
  g_assert_cmpuint (repacked_size, ==, packed_size);
  g_assert_cmpmem (repacked_data, repacked_size, modified, packed_size);
}

static void
test_adaptive_study_append (void)
{
  g_autoptr(GoodixChicagoEnrollment) enrollment =
    goodix_chicago_enrollment_new ();
  GoodixChicagoFeatureRecord record = { 0, };
  GoodixChicagoMetricData metric_data = { 0, };
  GoodixChicagoEnrollmentResult insert_result;
  GoodixChicagoSubtemplateView probe;
  GoodixChicagoSubtemplateView view;
  GoodixChicagoRelation relations[2] = { 0, };
  GoodixChicagoRelation stored_relation;
  g_autoptr(GBytes) packed = NULL;
  g_autoptr(GError) error = NULL;
  const guint8 *packed_data;
  gsize packed_size;
  guint appended_index = G_MAXUINT;
  gboolean found_matcher_value_c = FALSE;

  g_assert_true (goodix_chicago_enrollment_insert_first (
    enrollment, &record, 1, 1, 85, 100, &metric_data,
    &insert_result, &error));
  g_assert_no_error (error);
  g_assert_true (goodix_chicago_enrollment_insert_next (
    enrollment, &record, 1, 1, 85, 100, &metric_data,
    &insert_result, &error));
  g_assert_no_error (error);
  g_assert_true (goodix_chicago_enrollment_get_subtemplate (
    enrollment, 0, &probe));
  relations[0].inlier_count = 31;
  memcpy (relations[0].transform,
          (const gint32[6]) { 0x100, 0, 0, 0, 0x100, 0 },
          sizeof (relations[0].transform));

  g_assert_true (goodix_chicago_enrollment_append_study (
    enrollment, &probe, 0, relations, G_N_ELEMENTS (relations),
    &appended_index, &error));
  g_assert_no_error (error);
  g_assert_cmpuint (appended_index, ==, 2);
  g_assert_cmpuint (goodix_chicago_enrollment_get_count (enrollment), ==, 3);
  g_assert_true (goodix_chicago_enrollment_get_subtemplate (
    enrollment, 0, &view));
  g_assert_cmpuint (view.group_state, ==, 1);
  g_assert_cmpuint (view.study_value_a, ==, 1);
  g_assert_true (goodix_chicago_enrollment_get_subtemplate (
    enrollment, 1, &view));
  g_assert_cmpuint (view.group_state, ==, 0);
  g_assert_true (goodix_chicago_enrollment_get_subtemplate (
    enrollment, 2, &view));
  g_assert_cmpuint (view.group_state, ==, 1);
  g_assert_cmpuint (view.study_state, ==, 1);
  g_assert_cmpuint (view.lineage_index, ==, 2);
  g_assert_cmpuint (view.replacement_count, ==, 0);
  g_assert_cmpuint (view.study_value_a, ==, 1);
  g_assert_true (goodix_chicago_enrollment_get_relation (
    enrollment, view.relation_base, &stored_relation));
  g_assert_cmpint (stored_relation.inlier_count, ==, 31);
  g_assert_cmpmem (stored_relation.transform,
                   sizeof (stored_relation.transform),
                   relations[0].transform, sizeof (relations[0].transform));

  packed = goodix_chicago_enrollment_pack (enrollment, &error);
  g_assert_no_error (error);
  g_assert_nonnull (packed);
  packed_data = g_bytes_get_data (packed, &packed_size);
  for (gsize offset = 0; offset + 5 <= packed_size; offset++)
    if (packed_data[offset] == 0xa7 &&
        memcmp (packed_data + offset + 1,
                (const guint8[4]) { 1, 0, 0, 0 }, 4) == 0)
      found_matcher_value_c = TRUE;
  g_assert_true (found_matcher_value_c);
}

static void
test_match_order_roundtrip (void)
{
  static const guint8 order_pattern[] = {
    0xa1, 200, 0, 0, 0,
    0, 0, 0, 0,
    1, 0, 0, 0,
  };
  g_autoptr(GoodixChicagoEnrollment) source =
    goodix_chicago_enrollment_new ();
  g_autoptr(GoodixChicagoEnrollment) parsed = NULL;
  GoodixChicagoFeatureRecord record = { 0, };
  GoodixChicagoMetricData metric_data = { 0, };
  GoodixChicagoEnrollmentResult result;
  g_autoptr(GBytes) packed = NULL;
  g_autoptr(GBytes) repacked = NULL;
  g_autofree guint8 *modified = NULL;
  g_autoptr(GError) error = NULL;
  const guint8 *packed_data;
  const guint8 *repacked_data;
  gsize packed_size;
  gsize repacked_size;
  gsize order_offset = G_MAXSIZE;
  guint gallery_index;
  guint32 crc;

  g_assert_true (goodix_chicago_enrollment_insert_first (
    source, &record, 1, 1, 85, 100, &metric_data, &result, &error));
  g_assert_no_error (error);
  g_assert_true (goodix_chicago_enrollment_insert_next (
    source, &record, 1, 1, 85, 100, &metric_data, &result, &error));
  g_assert_no_error (error);
  packed = goodix_chicago_enrollment_pack (source, &error);
  g_assert_no_error (error);
  packed_data = g_bytes_get_data (packed, &packed_size);
  modified = g_memdup2 (packed_data, packed_size);
  for (gsize offset = 0;
       offset + sizeof (order_pattern) <= packed_size;
       offset++)
    if (memcmp (modified + offset, order_pattern,
                sizeof (order_pattern)) == 0)
      {
        g_assert_cmpuint (order_offset, ==, G_MAXSIZE);
        order_offset = offset;
      }
  g_assert_cmpuint (order_offset, !=, G_MAXSIZE);
  modified[order_offset + 5] = 1;
  modified[order_offset + 9] = 0;
  crc = GUINT32_TO_LE (test_packed_crc32 (modified + 10,
                                          packed_size - 10));
  memcpy (modified + 1, &crc, sizeof (crc));

  parsed = goodix_chicago_enrollment_unpack (modified, packed_size,
                                                &error);
  g_assert_no_error (error);
  g_assert_nonnull (parsed);
  g_assert_true (goodix_chicago_enrollment_get_match_order_index (
    parsed, 0, &gallery_index));
  g_assert_cmpuint (gallery_index, ==, 1);
  g_assert_true (goodix_chicago_enrollment_get_match_order_index (
    parsed, 1, &gallery_index));
  g_assert_cmpuint (gallery_index, ==, 0);

  repacked = goodix_chicago_enrollment_pack (parsed, &error);
  g_assert_no_error (error);
  repacked_data = g_bytes_get_data (repacked, &repacked_size);
  g_assert_cmpuint (repacked_size, ==, packed_size);
  g_assert_cmpmem (repacked_data, repacked_size, modified, packed_size);
}

static void
test_packed_template_roundtrip (void)
{
  const gchar *path = g_getenv ("CHICAGO_PACKED_TEMPLATE");
  g_autofree gchar *contents = NULL;
  g_autofree guint8 *corrupted = NULL;
  g_autoptr(GoodixChicagoEnrollment) enrollment = NULL;
  g_autoptr(GoodixChicagoEnrollment) invalid = NULL;
  g_autoptr(GBytes) repacked = NULL;
  g_autoptr(GError) error = NULL;
  const guint8 *repacked_data;
  gsize size;
  gsize repacked_size;

  if (!path)
    {
      g_test_skip ("Chicago packed-template roundtrip was not requested");
      return;
    }
  g_assert_true (g_file_get_contents (path, &contents, &size, &error));
  g_assert_no_error (error);
  enrollment = goodix_chicago_enrollment_unpack (
    (const guint8 *) contents, size, &error);
  g_assert_no_error (error);
  g_assert_nonnull (enrollment);
  g_assert_cmpuint (goodix_chicago_enrollment_get_count (enrollment), ==, 8);
  g_assert_cmpuint (goodix_chicago_enrollment_get_capacity (enrollment), ==,
                    50);
  g_assert_cmpuint (
    goodix_chicago_enrollment_get_transform_count (enrollment), ==, 29);
  repacked = goodix_chicago_enrollment_pack (enrollment, &error);
  g_assert_no_error (error);
  g_assert_nonnull (repacked);
  repacked_data = g_bytes_get_data (repacked, &repacked_size);
  g_assert_cmpuint (repacked_size, ==, size);
  g_assert_cmpmem (repacked_data, repacked_size, contents, size);

  corrupted = g_memdup2 (contents, size);
  corrupted[size - 1] ^= 1;
  invalid = goodix_chicago_enrollment_unpack (corrupted, size, &error);
  g_assert_null (invalid);
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA);
}

static void
test_identify_metric_oracle (void)
{
  const gchar *path = g_getenv ("CHICAGO_IDENTIFY_METRIC_VECTOR");
  g_autofree gchar *contents = NULL;
  g_autoptr(GError) error = NULL;
  const IdentifyMetricVector *vector;
  gint metric_a;
  gint metric_b;
  gint selector;
  gsize size;

  if (!path)
    {
      g_test_skip ("Chicago identify-metric oracle was not requested");
      return;
    }
  g_assert_true (g_file_get_contents (path, &contents, &size, &error));
  g_assert_no_error (error);
  g_assert_cmpuint (size, ==, sizeof (*vector));
  vector = (const IdentifyMetricVector *) contents;
  g_assert_cmphex (vector->magic, ==, 0x544d4943);
  g_assert_cmpuint (vector->version, ==, 1);
  selector = goodix_chicago_enrollment_calculate_config1_metrics (
    &vector->probe, &vector->gallery, vector->transform,
    &metric_a, &metric_b);
  g_test_message ("identify metric native=%d,%d,%d official=%d,%d,%d",
                  selector, metric_a, metric_b, vector->outputs[0],
                  vector->outputs[3], vector->outputs[4]);
  g_assert_cmpint (selector, ==, vector->outputs[0]);
  g_assert_cmpint (metric_a, ==, vector->outputs[3]);
  g_assert_cmpint (metric_b, ==, vector->outputs[4]);
}

static void
test_study_metrics (void)
{
  static const gint32 identity[6] = { 0x100, 0, 0, 0, 0x100, 0 };
  GoodixChicagoMetricData gallery = { 0, };
  GoodixChicagoMetricData probe = { 0, };
  gint metric_18;
  gint metric_1c;
  gint metric_20;

  memset (gallery.coarse_mask, 0xff, sizeof (gallery.coarse_mask));
  memset (probe.coarse_mask, 0xff, sizeof (probe.coarse_mask));
  g_assert_true (goodix_chicago_enrollment_calculate_study_metrics (
    &probe, &gallery, identity, &metric_18, &metric_1c, &metric_20));
  g_assert_cmpint (metric_18, ==, 256);
  g_assert_cmpint (metric_1c, ==, 256);
  g_assert_cmpint (metric_20, ==, 0);

  memset (gallery.validity, 0xff, sizeof (gallery.validity));
  memset (probe.validity, 0xff, sizeof (probe.validity));
  g_assert_true (goodix_chicago_enrollment_calculate_study_metrics (
    &probe, &gallery, identity, &metric_18, &metric_1c, &metric_20));
  g_assert_cmpint (metric_18, ==, 256);
  g_assert_cmpint (metric_1c, ==, 0);
  g_assert_cmpint (metric_20, ==, 256);

  memset (probe.validity, 0, sizeof (probe.validity));
  g_assert_true (goodix_chicago_enrollment_calculate_study_metrics (
    &probe, &gallery, identity, &metric_18, &metric_1c, &metric_20));
  g_assert_cmpint (metric_18, ==, 0);
  g_assert_cmpint (metric_1c, ==, 0);
  g_assert_cmpint (metric_20, ==, 0);
}

static void
test_live_auxiliary_counts_type24 (void)
{
  static const struct
  {
    gint32 transform[6];
    gint expected_zero;
    gint expected_mixed;
    gint expected_one;
  } vectors[] = {
    { { 256, 0, 0, 0, 256, 0 }, 1278, 0, 2 },
    { { 259, 2, -6391, -5, 249, -10719 }, 278, 2, 0 },
    { { 251, 10, -9189, -5, 252, -2363 }, 606, 2, 0 },
    { { 257, -10, 1263, 7, 259, -2334 }, 1115, 1, 0 },
    { { 254, -15, 4015, 4, 270, -8144 }, 612, 0, 0 },
    { { 256, -13, 9272, 10, 301, -11516 }, 368, 0, 0 },
  };
  const guint8 auxiliary[6] = { 3, 0, 0, 4, 0, 0 };

  for (guint index = 0; index < G_N_ELEMENTS (vectors); index++)
    {
      gint count_zero;
      gint count_mixed;
      gint count_one;

      goodix_chicago_enrollment_calculate_live_auxiliary_counts_type24 (
        auxiliary, vectors[index].transform, &count_zero, &count_mixed,
        &count_one);
      g_assert_cmpint (count_zero, ==, vectors[index].expected_zero);
      g_assert_cmpint (count_mixed, ==, vectors[index].expected_mixed);
      g_assert_cmpint (count_one, ==, vectors[index].expected_one);
    }
}

static void
test_live_auxiliary_count_oracle_type24 (void)
{
  const gchar *path =
    g_getenv ("CHICAGO_LIVE_AUXILIARY_COUNT_VECTOR");
  g_autofree gchar *contents = NULL;
  g_autoptr(GError) error = NULL;
  const LiveAuxiliaryCountVector *vectors;
  gsize size;
  gsize count;

  if (!path)
    {
      g_test_skip ("Chicago live-auxiliary count oracle was not requested");
      return;
    }
  g_assert_true (g_file_get_contents (path, &contents, &size, &error));
  g_assert_no_error (error);
  g_assert_cmpuint (size % sizeof (*vectors), ==, 0);
  count = size / sizeof (*vectors);
  g_assert_cmpuint (count, >, 0);
  vectors = (const LiveAuxiliaryCountVector *) contents;
  for (gsize index = 0; index < count; index++)
    {
      gint count_zero;
      gint count_mixed;
      gint count_one;

      g_assert_cmphex (vectors[index].magic, ==, 0x43554143);
      g_assert_cmpuint (vectors[index].version, ==, 1);
      goodix_chicago_enrollment_calculate_live_auxiliary_counts_type24 (
        vectors[index].auxiliary, vectors[index].transform,
        &count_zero, &count_mixed, &count_one);
      g_assert_cmpint (count_zero, ==, vectors[index].counts[0]);
      g_assert_cmpint (count_mixed, ==, vectors[index].counts[1]);
      g_assert_cmpint (count_one, ==, vectors[index].counts[2]);
    }
  g_test_message ("type-24 live auxiliary count parity: %" G_GSIZE_FORMAT
                  "/%" G_GSIZE_FORMAT " exact", count, count);
}

static void
test_fallback_metric_records (void)
{
  const gchar *gallery_path =
    g_getenv ("CHICAGO_FALLBACK_METRIC_GALLERY_ENHANCED");
  const gchar *probe_path =
    g_getenv ("CHICAGO_FALLBACK_METRIC_PROBE_ENHANCED");
  const gchar *transform_value =
    g_getenv ("CHICAGO_FALLBACK_METRIC_TRANSFORM");
  g_autofree gchar *gallery_contents = NULL;
  g_autofree gchar *probe_contents = NULL;
  g_autoptr(GError) error = NULL;
  GoodixChicagoMetricData gallery;
  GoodixChicagoMetricData probe;
  gint32 transform[6];
  gint metric_a;
  gint metric_b;
  gint selector;
  gsize gallery_size;
  gsize probe_size;

  if (!gallery_path || !probe_path || !transform_value)
    {
      g_test_skip ("Chicago fallback metric check was not requested");
      return;
    }
  g_assert_cmpint (sscanf (transform_value, "%d,%d,%d,%d,%d,%d",
                           &transform[0], &transform[1], &transform[2],
                           &transform[3], &transform[4], &transform[5]), ==,
                   6);
  g_assert_true (g_file_get_contents (gallery_path, &gallery_contents,
                                      &gallery_size, &error));
  g_assert_no_error (error);
  g_assert_true (g_file_get_contents (probe_path, &probe_contents,
                                      &probe_size, &error));
  g_assert_no_error (error);
  g_assert_cmpuint (gallery_size, ==, GOODIX_CHICAGO_FEATURE_PIXELS);
  g_assert_cmpuint (probe_size, ==, GOODIX_CHICAGO_FEATURE_PIXELS);
  goodix_chicago_enrollment_build_metric_data (
    (const guint8 *) gallery_contents, &gallery);
  goodix_chicago_enrollment_build_metric_data (
    (const guint8 *) probe_contents, &probe);
  selector = goodix_chicago_enrollment_calculate_config1_metrics (
    &probe, &gallery, transform, &metric_a, &metric_b);
  g_test_message ("fallback metric native=%d/%d/%d official=128/225/113",
                  selector, metric_a, metric_b);
  g_assert_cmpint (selector, ==, 128);
  g_assert_cmpint (metric_a, ==, 225);
  g_assert_cmpint (metric_b, ==, 113);
}

static void
test_first_insert (void)
{
  g_autoptr(GoodixChicagoEnrollment) enrollment =
    goodix_chicago_enrollment_new ();
  GoodixChicagoFeatureRecord records[2] = { 0, };
  GoodixChicagoMetricData metric_data = { 0, };
  GoodixChicagoEnrollmentResult result;
  GoodixChicagoSubtemplateView view;
  GoodixChicagoRelation relation;
  g_autoptr(GError) error = NULL;

  ((guint8 *) &records[0])[0] = 0;
  ((guint8 *) &records[0])[10] = 0x55;
  ((guint8 *) &records[1])[0] = 1;
  ((guint8 *) &records[1])[10] = 0xaa;
  metric_data.primary[3] = 0x5a;
  metric_data.coarse_mask[2] = 0xa5;

  g_assert_cmpuint (goodix_chicago_enrollment_get_required_samples (
                      enrollment), ==, 8);
  g_assert_cmpuint (goodix_chicago_enrollment_get_capacity (enrollment),
                    ==, 50);
  g_assert_true (goodix_chicago_enrollment_insert_first (
    enrollment, records, 2, 1, 92, 100, &metric_data, &result, &error));
  g_assert_no_error (error);
  g_assert_cmphex (result.packed_position_detail, ==, 0x64000064);
  g_assert_cmpuint (result.progress, ==, 12);
  g_assert_cmpuint (result.position_x, ==, 0);
  g_assert_cmpuint (result.position_y, ==, 0);
  g_assert_cmpuint (goodix_chicago_enrollment_get_count (enrollment), ==, 1);
  g_assert_cmpuint (goodix_chicago_enrollment_get_transform_count (
                      enrollment), ==, 1);
  g_assert_true (goodix_chicago_enrollment_get_relation (enrollment, 0,
                                                            &relation));
  g_assert_cmpint (relation.inlier_count, ==, -1);
  g_assert_cmpint (relation.transform[0], ==, 0x100);
  g_assert_cmpint (relation.transform[4], ==, 0x100);
  g_assert_true (goodix_chicago_enrollment_get_subtemplate (enrollment, 0,
                                                               &view));
  g_assert_cmpuint (view.record_count, ==, 2);
  g_assert_cmpuint (view.active_count, ==, 1);
  g_assert_cmpuint (view.quality, ==, 92);
  g_assert_cmpuint (view.coverage, ==, 100);
  g_assert_cmpmem (view.records, sizeof (records), records, sizeof (records));
  g_assert_true (view.has_metric_data);
  g_assert_nonnull (view.metric_data);
  g_assert_cmpuint (view.metric_data->primary[3], ==, 0x5a);
  g_assert_cmpuint (view.metric_data->coarse_mask[2], ==, 0xa5);

  /* +0x19c00 owns a copy rather than retaining the extraction buffer. */
  memset (records, 0, sizeof (records));
  memset (&metric_data, 0, sizeof (metric_data));
  g_assert_cmpuint (((const guint8 *) &view.records[0])[10], ==, 0x55);
  g_assert_cmpuint (((const guint8 *) &view.records[1])[10], ==, 0xaa);
  g_assert_cmpuint (view.metric_data->primary[3], ==, 0x5a);

  g_assert_false (goodix_chicago_enrollment_insert_first (
    enrollment, records, 2, 1, 92, 100, NULL, &result, &error));
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_EXISTS);
}

static void
test_engine_adapter_enrollment_policy (void)
{
  static const struct
  {
    guint x;
    guint y;
    gboolean accepted;
    guint reject;
  } official_trace[] = {
    { 0, 0, TRUE, 0 },
    { 58, 56, TRUE, 0 },
    { 0, 0, TRUE, 0 },
    { 0, 99, TRUE, 0 },
    { 0, 0, TRUE, 0 },
    { 0, 39, FALSE, 4 },
    { 0, 0, TRUE, 0 },
    { 0, 0, TRUE, 0 },
    { 0, 0, TRUE, 0 },
    { 0, 0, TRUE, 0 },
    { 0, 0, TRUE, 0 },
    { 0, 78, FALSE, 1 },
    { 0, 62, FALSE, 1 },
    { 0, 0, TRUE, 0 },
    { 0, 100, FALSE, 3 },
    { 0, 100, FALSE, 3 },
    { 57, 100, TRUE, 0 },
  };
  GoodixChicagoEngineEnrollmentPolicy policy;

  goodix_chicago_engine_enrollment_policy_init (&policy);
  for (guint index = 0; index < G_N_ELEMENTS (official_trace); index++)
    {
      guint reject = G_MAXUINT;
      gboolean accepted = goodix_chicago_engine_enrollment_policy_accept (
        &policy, official_trace[index].x, official_trace[index].y, &reject);

      g_test_message ("attempt %u position=%u,%u accepted=%d reject=%u used=%u",
                      index + 1, official_trace[index].x,
                      official_trace[index].y, accepted, reject, policy.used);
      g_assert_cmpint (accepted, ==, official_trace[index].accepted);
      g_assert_cmpuint (reject, ==, official_trace[index].reject);
      if (index == 5)
        {
          g_assert_true (policy.defer_current_sample);
          g_assert_false (policy.restore_deferred_sample);
          g_assert_true (policy.deferred_pending);
        }
      else if (index == 6)
        {
          g_assert_false (policy.defer_current_sample);
          g_assert_true (policy.restore_deferred_sample);
          g_assert_false (policy.deferred_pending);
        }
      else
        {
          g_assert_false (policy.defer_current_sample);
          g_assert_false (policy.restore_deferred_sample);
        }
    }

  g_assert_cmpuint (policy.touched, ==, 17);
  g_assert_cmpuint (policy.enrolled, ==, 17);
  g_assert_cmpuint (policy.tipped, ==, 5);
  g_assert_cmpuint (policy.used, ==, 12);
  g_assert_cmpuint (policy.tip_index, ==, 3);
  g_assert_true (goodix_chicago_engine_enrollment_policy_complete (&policy));
}

static void
test_matches_first_sample_oracle (void)
{
  const gchar *path = g_getenv ("CHICAGO_ENROLLMENT_FEATURES");
  g_autofree gchar *contents = NULL;
  g_autoptr(GError) error = NULL;
  g_autoptr(GoodixChicagoEnrollment) enrollment = NULL;
  GoodixChicagoEnrollmentResult result;
  GoodixChicagoSubtemplateView view;
  guint32 count;
  guint32 quality;
  guint32 coverage;
  guint active = 0;
  gsize size;
  const GoodixChicagoFeatureRecord *records;

  if (!path)
    {
      g_test_skip ("Chicago Wine first-enrollment vector was not requested");
      return;
    }

  g_assert_true (g_file_get_contents (path, &contents, &size, &error));
  g_assert_no_error (error);
  g_assert_cmpuint (size, >=, 3 * sizeof (guint32));
  memcpy (&count, contents, sizeof (count));
  memcpy (&quality, contents + 4, sizeof (quality));
  memcpy (&coverage, contents + 8, sizeof (coverage));
  g_assert_cmpuint (size, ==,
                    3 * sizeof (guint32) + count * sizeof (*records));
  records = (const GoodixChicagoFeatureRecord *) (contents + 12);
  while (active < count && (((const guint8 *) &records[active])[0] & 3u) == 0)
    active++;

  enrollment = goodix_chicago_enrollment_new ();
  g_assert_true (goodix_chicago_enrollment_insert_first (
    enrollment, records, count, active, quality, coverage, NULL, &result,
    &error));
  g_assert_no_error (error);
  g_assert_cmpuint (result.progress, ==, 12);
  g_assert_cmphex (result.packed_position_detail, ==, 0x64000064);
  g_assert_true (goodix_chicago_enrollment_get_subtemplate (enrollment, 0,
                                                               &view));
  g_assert_cmpuint (view.record_count, ==, count);
  g_assert_cmpuint (view.active_count, ==, active);
  g_assert_cmpuint (view.quality, ==, quality);
  g_assert_cmpuint (view.coverage, ==, coverage);
  g_assert_cmpmem (view.records, count * sizeof (*records), records,
                   count * sizeof (*records));
}

static void
test_sequence_oracle_contract (void)
{
  static const guint32 expected_progress[8] = {
    12, 25, 37, 50, 62, 75, 87, 100,
  };
  static const guint32 expected_transforms[8] = {
    1, 2, 4, 7, 11, 16, 22, 29,
  };
  const gchar *path = g_getenv ("CHICAGO_ENROLLMENT_SEQUENCE");
  g_autofree gchar *contents = NULL;
  g_autoptr(GError) error = NULL;
  const guint32 *header;
  gsize size;

  if (!path)
    {
      g_test_skip ("Chicago Wine enrollment sequence was not requested");
      return;
    }
  g_assert_true (g_file_get_contents (path, &contents, &size, &error));
  g_assert_no_error (error);
  g_assert_cmpuint (size, >=, 3 * sizeof (guint32));
  header = (const guint32 *) contents;
  g_assert_cmphex (header[0], ==, 0x53524e45);
  g_assert_cmpuint (header[2], ==, 16 * sizeof (guint32));
  g_assert_cmpuint (size, ==, 3 * sizeof (guint32) + header[1] * header[2]);
  g_assert_cmpuint (header[1], >=, 8);

  for (guint step = 0; step < header[1]; step++)
    {
      const guint32 *state = (const guint32 *)
        (contents + 3 * sizeof (guint32) + step * header[2]);
      const guint accepted = MIN (step + 1, 8u);

      g_assert_cmpuint (state[1], ==, 8); /* required */
      g_assert_cmpuint (state[2], ==, accepted);
      g_assert_cmpuint (state[3], ==,
                        expected_progress[accepted - 1]);
      g_assert_cmpuint (state[4], ==, 0);
      g_assert_cmpuint (state[5], ==, 0);
      g_assert_cmpuint (state[6], ==, accepted);
      g_assert_cmpuint (state[7], ==, 50);
      g_assert_cmpuint (state[8], ==,
                        expected_transforms[accepted - 1]);
    }
}

static void
test_native_sequence_oracle (void)
{
  static const gchar *stems[8] = {
    "gdix51c0_frame_46937405529",
    "gdix51c0_frame_46938733390",
    "gdix51c0_frame_46939559130",
    "gdix51c0_frame_46940553896",
    "gdix51c0_frame_46941642439",
    "gdix51c0_frame_46942554085",
    "gdix51c0_frame_46943411976",
    "gdix51c0_frame_46944174638",
  };
  const gchar *feature_dir = g_getenv ("CHICAGO_NATIVE_FEATURE_DIR");
  const gchar *enhanced_dir = g_getenv ("CHICAGO_NATIVE_ENHANCED_DIR");
  const gchar *sequence_path = g_getenv ("CHICAGO_NATIVE_SEQUENCE");
  const gchar *packed_template_path =
    g_getenv ("CHICAGO_NATIVE_PACKED_TEMPLATE");
  const gchar *shifted_dir = g_getenv ("CHICAGO_NATIVE_SHIFTED_DIR");
  const gboolean repeat = g_getenv ("CHICAGO_NATIVE_REPEAT") != NULL;
  const gboolean mixed = g_getenv ("CHICAGO_NATIVE_MIXED") != NULL;
  g_autoptr(GoodixChicagoEnrollment) enrollment = NULL;
  g_autofree gchar *sequence = NULL;
  g_autoptr(GError) error = NULL;
  gsize sequence_size;
  guint steps;

  if (!feature_dir || !enhanced_dir || !sequence_path)
    {
      g_test_skip ("Chicago native sequence vector was not requested");
      return;
    }
  g_assert_true (g_file_get_contents (sequence_path, &sequence,
                                      &sequence_size, &error));
  g_assert_no_error (error);
  g_assert_cmpuint (sequence_size, >=, 12);
  g_assert_cmphex (*(const guint32 *) sequence, ==, 0x53524e45);
  steps = ((const guint32 *) sequence)[1];
  g_assert_cmpuint (sequence_size, ==,
                    12 + steps * 16 * sizeof (guint32));
  g_assert_cmpuint (steps, <=, 8);
  if (mixed)
    g_assert_nonnull (shifted_dir);
  enrollment = goodix_chicago_enrollment_new ();

  for (guint sample = 0; sample < steps; sample++)
    {
      const gboolean shifted = mixed && (sample == 1 || sample == 2);
      const gchar *axis = sample == 1 ? "x8" : "y8";
      const gchar *stem = repeat || mixed ? stems[0] : stems[sample];
      g_autofree gchar *feature_path = shifted ?
        g_strdup_printf ("%s/%s.features", shifted_dir, axis) :
        g_strdup_printf ("%s/%s.features", feature_dir, stem);
      g_autofree gchar *enhanced_path = shifted ?
        g_strdup_printf ("%s/%s.enhanced", shifted_dir, axis) :
        g_strdup_printf ("%s/%s.enh", enhanced_dir, stem);
      g_autofree gchar *features = NULL;
      g_autofree gchar *enhanced = NULL;
      GoodixChicagoMetricData metric_data;
      GoodixChicagoEnrollmentResult result;
      GoodixChicagoSubtemplateView view;
      const GoodixChicagoFeatureRecord *records;
      const guint32 *expected = (const guint32 *)
        (sequence + 12 + sample * 16 * sizeof (guint32));
      guint32 record_count;
      guint active_count = 0;
      gsize feature_size;
      gsize enhanced_size;

      g_assert_true (g_file_get_contents (feature_path, &features,
                                          &feature_size, &error));
      g_assert_no_error (error);
      g_assert_true (g_file_get_contents (enhanced_path, &enhanced,
                                          &enhanced_size, &error));
      g_assert_no_error (error);
      memcpy (&record_count, features, sizeof (record_count));
      g_assert_cmpuint (feature_size, ==,
                        12 + record_count * sizeof (*records));
      g_assert_cmpuint (enhanced_size, ==, GOODIX_CHICAGO_FEATURE_PIXELS);
      records = (const GoodixChicagoFeatureRecord *) (features + 12);
      while (active_count < record_count &&
             ((((const guint8 *) &records[active_count])[0] & 3u) == 0))
        active_count++;
      goodix_chicago_enrollment_build_metric_data (
        (const guint8 *) enhanced, &metric_data);

      if (sample == 0)
        g_assert_true (goodix_chicago_enrollment_insert_first (
          enrollment, records, record_count, active_count,
          *(const guint32 *) (features + 4),
          *(const guint32 *) (features + 8), &metric_data, &result, &error));
      else
        g_assert_true (goodix_chicago_enrollment_insert_next (
          enrollment, records, record_count, active_count,
          *(const guint32 *) (features + 4),
          *(const guint32 *) (features + 8), &metric_data, &result, &error));
      g_assert_no_error (error);
      g_test_message ("sample=%u native=%u,%u official=%u,%u state=%u",
                      sample, result.position_x, result.position_y,
                      expected[4], expected[5], expected[13]);
      g_assert_cmpuint (result.progress, ==, expected[3]);
      g_assert_cmpuint (result.position_x, ==, expected[4]);
      g_assert_cmpuint (result.position_y, ==, expected[5]);
      g_assert_cmpuint (goodix_chicago_enrollment_get_transform_count (
                          enrollment), ==, expected[8]);
      g_assert_true (goodix_chicago_enrollment_get_subtemplate (
        enrollment, sample, &view));
      g_assert_cmpuint (view.group_state, ==, expected[13]);
      g_assert_cmpuint (view.relation_base, ==, expected[14]);
    }

  if (packed_template_path)
    {
      g_autoptr(GBytes) native_packed = NULL;
      g_autofree gchar *packed_template = NULL;
      gconstpointer native_packed_data;
      gsize native_packed_size;
      gsize packed_template_size;

      g_assert_true (g_file_get_contents (packed_template_path,
                                          &packed_template,
                                          &packed_template_size,
                                          &error));
      g_assert_no_error (error);
      g_assert_cmpuint (goodix_chicago_enrollment_get_packed_size (
                          enrollment), ==, packed_template_size);
      native_packed = goodix_chicago_enrollment_pack (enrollment, &error);
      g_assert_no_error (error);
      g_assert_nonnull (native_packed);
      native_packed_data = g_bytes_get_data (native_packed,
                                             &native_packed_size);
      g_assert_cmpuint (native_packed_size, ==, packed_template_size);
      g_assert_cmpmem (native_packed_data, native_packed_size,
                       packed_template, packed_template_size);
    }
}

static void
test_correspondence_oracle (void)
{
  const gchar *old_path = g_getenv ("CHICAGO_CORRESPONDENCE_OLD");
  const gchar *new_path = g_getenv ("CHICAGO_CORRESPONDENCE_NEW");
  const gchar *oracle_path = g_getenv ("CHICAGO_CORRESPONDENCE_VECTOR");
  g_autofree gchar *old_contents = NULL;
  g_autofree gchar *new_contents = NULL;
  g_autofree gchar *oracle_contents = NULL;
  g_autoptr(GError) error = NULL;
  GoodixChicagoCorrespondence pairs[GOODIX_CHICAGO_CORRESPONDENCE_LIMIT];
  const GoodixChicagoFeatureRecord *old_records;
  const GoodixChicagoFeatureRecord *new_records;
  const gint32 *official_pairs;
  guint32 old_count;
  guint32 new_count;
  guint32 official_count;
  guint native_count;
  gsize old_size;
  gsize new_size;
  gsize oracle_size;

  if (!old_path || !new_path || !oracle_path)
    {
      g_test_skip ("Chicago Wine correspondence vector was not requested");
      return;
    }
  g_assert_true (g_file_get_contents (old_path, &old_contents, &old_size,
                                      &error));
  g_assert_no_error (error);
  g_assert_true (g_file_get_contents (new_path, &new_contents, &new_size,
                                      &error));
  g_assert_no_error (error);
  g_assert_true (g_file_get_contents (oracle_path, &oracle_contents,
                                      &oracle_size, &error));
  g_assert_no_error (error);

  memcpy (&old_count, old_contents, sizeof (old_count));
  memcpy (&new_count, new_contents, sizeof (new_count));
  g_assert_cmpuint (old_size, ==, 12 + old_count * sizeof (*old_records));
  g_assert_cmpuint (new_size, ==, 12 + new_count * sizeof (*new_records));
  g_assert_cmpuint (oracle_size, ==,
                    3 * sizeof (guint32) + 42 * 2 * sizeof (gint32));
  g_assert_cmpmem (oracle_contents, sizeof (old_count),
                   &old_count, sizeof (old_count));
  g_assert_cmpmem (oracle_contents + 4, sizeof (new_count),
                   &new_count, sizeof (new_count));
  memcpy (&official_count, oracle_contents + 8, sizeof (official_count));
  g_assert_cmpuint (official_count, <=,
                    GOODIX_CHICAGO_CORRESPONDENCE_LIMIT);
  official_pairs = (const gint32 *) (oracle_contents + 12);
  old_records = (const GoodixChicagoFeatureRecord *) (old_contents + 12);
  new_records = (const GoodixChicagoFeatureRecord *) (new_contents + 12);

  native_count = goodix_chicago_enrollment_find_correspondences (
    old_records, old_count, new_records, new_count, pairs);
  if (native_count != official_count)
    {
      for (guint index = 0; index < MAX (native_count, official_count); index++)
        g_test_message ("pair[%u] native=%d:%d official=%d:%d", index,
                        index < native_count ? (gint) pairs[index].old_index : -1,
                        index < native_count ? (gint) pairs[index].new_index : -1,
                        index < official_count ? official_pairs[index * 2] : -1,
                        index < official_count ? official_pairs[index * 2 + 1] : -1);
    }
  g_assert_cmpuint (native_count, ==, official_count);
  for (guint index = 0; index < native_count; index++)
    {
      g_assert_cmpuint (pairs[index].old_index, ==,
                        (guint) official_pairs[index * 2]);
      g_assert_cmpuint (pairs[index].new_index, ==,
                        (guint) official_pairs[index * 2 + 1]);
    }
}

static void
test_transform_oracle (void)
{
  const gchar *old_path = g_getenv ("CHICAGO_TRANSFORM_OLD");
  const gchar *new_path = g_getenv ("CHICAGO_TRANSFORM_NEW");
  const gchar *oracle_path = g_getenv ("CHICAGO_TRANSFORM_VECTOR");
  g_autofree gchar *old_contents = NULL;
  g_autofree gchar *new_contents = NULL;
  g_autofree gchar *oracle_contents = NULL;
  g_autoptr(GError) error = NULL;
  GoodixChicagoCorrespondence pairs[GOODIX_CHICAGO_CORRESPONDENCE_LIMIT];
  GoodixChicagoPoint source[GOODIX_CHICAGO_TRANSFORM_POINT_LIMIT];
  GoodixChicagoPoint target[GOODIX_CHICAGO_TRANSFORM_POINT_LIMIT];
  GoodixChicagoTransformResult result;
  const GoodixChicagoFeatureRecord *old_records;
  const GoodixChicagoFeatureRecord *new_records;
  const gint32 *official_transform;
  const guint8 *official_inliers;
  guint official_inlier_count = 0;
  guint32 old_count;
  guint32 new_count;
  guint32 official_count;
  gint32 official_error;
  guint pair_count;
  gsize old_size;
  gsize new_size;
  gsize oracle_size;

  if (!old_path || !new_path || !oracle_path)
    {
      g_test_skip ("Chicago Wine transform vector was not requested");
      return;
    }
  g_assert_true (g_file_get_contents (old_path, &old_contents, &old_size,
                                      &error));
  g_assert_no_error (error);
  g_assert_true (g_file_get_contents (new_path, &new_contents, &new_size,
                                      &error));
  g_assert_no_error (error);
  g_assert_true (g_file_get_contents (oracle_path, &oracle_contents,
                                      &oracle_size, &error));
  g_assert_no_error (error);
  memcpy (&old_count, old_contents, sizeof (old_count));
  memcpy (&new_count, new_contents, sizeof (new_count));
  g_assert_cmpuint (old_size, ==, 12 + old_count * sizeof (*old_records));
  g_assert_cmpuint (new_size, ==, 12 + new_count * sizeof (*new_records));
  g_assert_cmpuint (oracle_size, ==,
                    sizeof (guint32) + 6 * sizeof (gint32) +
                    sizeof (gint32) + 42 * sizeof (guint8));
  old_records = (const GoodixChicagoFeatureRecord *) (old_contents + 12);
  new_records = (const GoodixChicagoFeatureRecord *) (new_contents + 12);
  pair_count = goodix_chicago_enrollment_find_correspondences (
    old_records, old_count, new_records, new_count, pairs);
  memcpy (&official_count, oracle_contents, sizeof (official_count));
  g_assert_cmpuint (pair_count, ==, official_count);

  for (guint index = 0; index < pair_count; index++)
    {
      source[index].x = (guint16) new_records[pairs[index].new_index].refined_x;
      source[index].y = (guint16) new_records[pairs[index].new_index].refined_y;
      target[index].x = (guint16) old_records[pairs[index].old_index].refined_x;
      target[index].y = (guint16) old_records[pairs[index].old_index].refined_y;
    }
  goodix_chicago_enrollment_estimate_transform (source, target, pair_count,
                                                   &result);
  official_transform = (const gint32 *) (oracle_contents + 4);
  memcpy (&official_error, oracle_contents + 4 + 6 * sizeof (gint32),
          sizeof (official_error));
  official_inliers = (const guint8 *)
    (oracle_contents + 4 + 7 * sizeof (gint32));
  g_assert_cmpmem (result.values, sizeof (result.values), official_transform,
                   6 * sizeof (gint32));
  g_assert_cmpint (result.error, ==, official_error);
  g_assert_cmpmem (result.inliers, sizeof (result.inliers), official_inliers,
                   42 * sizeof (guint8));
  for (guint index = 0; index < 42; index++)
    official_inlier_count += official_inliers[index] != 0;
  g_assert_cmpuint (result.inlier_count, ==, official_inlier_count);
}

static void
test_evidence_gate (void)
{
  g_assert_false (goodix_chicago_enrollment_evidence_is_accepted (5, 1000,
                                                                     1000));
  g_assert_false (goodix_chicago_enrollment_evidence_is_accepted (6, 215,
                                                                     1000));
  g_assert_true (goodix_chicago_enrollment_evidence_is_accepted (6, 216, 0));
  g_assert_false (goodix_chicago_enrollment_evidence_is_accepted (7, 209,
                                                                     64));
  g_assert_true (goodix_chicago_enrollment_evidence_is_accepted (7, 209,
                                                                    65));
  g_assert_true (goodix_chicago_enrollment_evidence_is_accepted (11, 0, 0));
}

static void
test_relation_metric_oracle (void)
{
  const gchar *old_path = g_getenv ("CHICAGO_METRIC_OLD_OBJECT");
  const gchar *new_path = g_getenv ("CHICAGO_METRIC_NEW_OBJECT");
  const gchar *transform_path = g_getenv ("CHICAGO_METRIC_TRANSFORM");
  const gchar *oracle_path = g_getenv ("CHICAGO_METRIC_VECTOR");
  g_autofree gchar *old_contents = NULL;
  g_autofree gchar *new_contents = NULL;
  g_autofree gchar *transform_contents = NULL;
  g_autofree gchar *oracle_contents = NULL;
  g_autoptr(GError) error = NULL;
  const guint8 *old_primary;
  const guint8 *new_primary;
  const guint8 *old_mask;
  const guint8 *new_mask;
  const gint32 *transform;
  gint official_metric_a;
  gint official_metric_b;
  gint metric_a;
  gint metric_b;
  guint32 packed_bytes;
  gsize old_size;
  gsize new_size;
  gsize transform_size;
  gsize oracle_size;

  if (!old_path || !new_path || !transform_path || !oracle_path)
    {
      g_test_skip ("Chicago Wine relation metric vector was not requested");
      return;
    }
  g_assert_true (g_file_get_contents (old_path, &old_contents, &old_size,
                                      &error));
  g_assert_no_error (error);
  g_assert_true (g_file_get_contents (new_path, &new_contents, &new_size,
                                      &error));
  g_assert_no_error (error);
  g_assert_true (g_file_get_contents (transform_path, &transform_contents,
                                      &transform_size, &error));
  g_assert_no_error (error);
  g_assert_true (g_file_get_contents (oracle_path, &oracle_contents,
                                      &oracle_size, &error));
  g_assert_no_error (error);
  g_assert_cmpuint (old_size, >=, 4 + 0x160 + 4 + 32 + 160);
  g_assert_cmpuint (new_size, >=, 4 + 0x160 + 4 + 32 + 160);
  g_assert_cmpuint (transform_size, >=, 4 + 6 * sizeof (gint32));
  g_assert_cmpuint (oracle_size, ==, 2 * sizeof (gint32));
  g_assert_cmphex (*(const guint32 *) old_contents, ==, 0x4a424f53);
  g_assert_cmphex (*(const guint32 *) new_contents, ==, 0x4a424f53);
  memcpy (&packed_bytes, old_contents + 4 + 0x160,
          sizeof (packed_bytes));
  g_assert_cmpuint (packed_bytes, >=,
                    GOODIX_CHICAGO_METRIC_MAP_PACKED_BYTES);
  memcpy (&packed_bytes, new_contents + 4 + 0x160,
          sizeof (packed_bytes));
  g_assert_cmpuint (packed_bytes, >=,
                    GOODIX_CHICAGO_METRIC_MAP_PACKED_BYTES);

  old_mask = (const guint8 *) old_contents + 4 + 0x28;
  new_mask = (const guint8 *) new_contents + 4 + 0x28;
  old_primary = (const guint8 *) old_contents + 4 + 0x160 + 4 + 32;
  new_primary = (const guint8 *) new_contents + 4 + 0x160 + 4 + 32;
  transform = (const gint32 *) (transform_contents + 4);
  memcpy (&official_metric_a, oracle_contents, sizeof (official_metric_a));
  memcpy (&official_metric_b, oracle_contents + 4, sizeof (official_metric_b));
  goodix_chicago_enrollment_calculate_relation_metrics (
    new_primary, new_mask, old_primary, old_mask, transform,
    &metric_a, &metric_b);
  g_test_message ("native metrics=%d/%d official=%d/%d", metric_a, metric_b,
                  official_metric_a, official_metric_b);
  g_assert_cmpint (metric_a, ==, official_metric_a);
  g_assert_cmpint (metric_b, ==, official_metric_b);
}

static void
test_relation_metric_identity (void)
{
  guint8 primary[GOODIX_CHICAGO_METRIC_MAP_PACKED_BYTES] = { 0, };
  guint8 mask[GOODIX_CHICAGO_METRIC_MASK_PACKED_BYTES];
  const gint32 transform[6] = { 0x100, 0, 0, 0, 0x100, 0 };
  gint metric_a;
  gint metric_b;

  memset (mask, 0xff, sizeof (mask));
  goodix_chicago_enrollment_calculate_relation_metrics (
    primary, mask, primary, mask, transform, &metric_a, &metric_b);
  g_assert_cmpint (metric_a, ==, 246);
  g_assert_cmpint (metric_b, ==, 165);
}

static void
test_relation_builder_identity (void)
{
  GoodixChicagoFeatureRecord records[12] = { 0, };
  GoodixChicagoMetricData metric_data = { 0, };
  GoodixChicagoRelation relation;
  gboolean accepted = FALSE;
  gint metric_a;
  gint metric_b;

  memset (metric_data.coarse_mask, 0xff,
          sizeof (metric_data.coarse_mask));
  for (guint index = 0; index < G_N_ELEMENTS (records); index++)
    {
      records[index].refined_x = 0x1000 + (index % 4) * 0x400;
      records[index].refined_y = 0x1800 + (index / 4) * 0x400;
      records[index].descriptor[4] = index + 1;
    }

  goodix_chicago_enrollment_build_relation (
    records, G_N_ELEMENTS (records), &metric_data,
    records, G_N_ELEMENTS (records), &metric_data,
    &relation, &metric_a, &metric_b, &accepted);
  g_assert_cmpint (relation.inlier_count, ==, 12);
  g_assert_cmpint (relation.transform[0], ==, 0x100);
  g_assert_cmpint (relation.transform[1], ==, 0);
  g_assert_cmpint (relation.transform[2], ==, 0);
  g_assert_cmpint (relation.transform[3], ==, 0);
  g_assert_cmpint (relation.transform[4], ==, 0x100);
  g_assert_cmpint (relation.transform[5], ==, 0);
  g_assert_cmpint (metric_a, ==, 246);
  g_assert_cmpint (metric_b, ==, 165);
  g_assert_true (accepted);
}

static void
test_second_insert_identity (void)
{
  g_autoptr(GoodixChicagoEnrollment) enrollment =
    goodix_chicago_enrollment_new ();
  GoodixChicagoFeatureRecord records[12] = { 0, };
  GoodixChicagoMetricData metric_data = { 0, };
  GoodixChicagoEnrollmentResult result;
  GoodixChicagoRelation relation;
  g_autoptr(GError) error = NULL;

  memset (metric_data.coarse_mask, 0xff,
          sizeof (metric_data.coarse_mask));
  memset (metric_data.primary, 0xff, 40);
  for (guint index = 0; index < G_N_ELEMENTS (metric_data.position_map);
       index++)
    metric_data.position_map[index] = 0xff;
  for (guint index = 0; index < G_N_ELEMENTS (records); index++)
    {
      records[index].refined_x = 0x1000 + (index % 4) * 0x400;
      records[index].refined_y = 0x1800 + (index / 4) * 0x400;
      records[index].descriptor[4] = index + 1;
    }
  g_assert_true (goodix_chicago_enrollment_insert_first (
    enrollment, records, G_N_ELEMENTS (records), G_N_ELEMENTS (records),
    92, 100, &metric_data, &result, &error));
  g_assert_no_error (error);
  g_assert_true (goodix_chicago_enrollment_insert_second (
    enrollment, records, G_N_ELEMENTS (records), G_N_ELEMENTS (records),
    92, 100, &metric_data, &result, &error));
  g_assert_no_error (error);
  g_assert_cmphex (result.packed_position_detail, ==, 0x100);
  g_assert_cmpuint (result.position_x, ==, 100);
  g_assert_cmpuint (result.position_y, ==, 100);
  g_assert_cmpuint (result.progress, ==, 25);
  g_assert_cmpuint (goodix_chicago_enrollment_get_count (enrollment), ==, 2);
  g_assert_cmpuint (goodix_chicago_enrollment_get_transform_count (
                      enrollment), ==, 2);
  g_assert_true (goodix_chicago_enrollment_get_relation (enrollment, 1,
                                                            &relation));
  g_assert_cmpint (relation.inlier_count, ==, 12);
  g_assert_cmpint (relation.transform[0], ==, 0x100);
  g_assert_cmpint (relation.transform[4], ==, 0x100);

  /* EngineAdapter's first directional tip deletes this exact last image and
   * later re-adds it after the next capture. Both the triangular relation
   * storage and its public transform count must rewind with the image. */
  g_assert_true (goodix_chicago_enrollment_drop_last (enrollment, &error));
  g_assert_no_error (error);
  g_assert_cmpuint (goodix_chicago_enrollment_get_count (enrollment), ==, 1);
  g_assert_cmpuint (goodix_chicago_enrollment_get_transform_count (
                      enrollment), ==, 1);
  g_assert_false (goodix_chicago_enrollment_get_relation (enrollment, 1,
                                                             &relation));
  g_assert_true (goodix_chicago_enrollment_insert_next (
    enrollment, records, G_N_ELEMENTS (records), G_N_ELEMENTS (records),
    92, 100, &metric_data, &result, &error));
  g_assert_no_error (error);
  g_assert_cmpuint (goodix_chicago_enrollment_get_count (enrollment), ==, 2);
  g_assert_cmpuint (goodix_chicago_enrollment_get_transform_count (
                      enrollment), ==, 2);
}

static void
test_repeated_eight_insertions (void)
{
  static const guint expected_relations[8] = { 1, 2, 4, 7, 11, 16, 22, 29 };
  static const guint expected_progress[8] = { 12, 25, 37, 50, 62, 75, 87, 100 };
  g_autoptr(GoodixChicagoEnrollment) enrollment =
    goodix_chicago_enrollment_new ();
  GoodixChicagoFeatureRecord records[12] = { 0, };
  GoodixChicagoMetricData metric_data = { 0, };
  GoodixChicagoEnrollmentResult result;
  g_autoptr(GError) error = NULL;

  memset (metric_data.primary, 0xff, 40);
  memset (metric_data.coarse_mask, 0xff,
          sizeof (metric_data.coarse_mask));
  for (guint index = 0; index < G_N_ELEMENTS (metric_data.position_map);
       index++)
    metric_data.position_map[index] = 0xff;
  for (guint index = 0; index < G_N_ELEMENTS (records); index++)
    {
      records[index].refined_x = 0x1000 + (index % 4) * 0x400;
      records[index].refined_y = 0x1800 + (index / 4) * 0x400;
      records[index].descriptor[4] = index + 1;
    }

  for (guint sample = 0; sample < 8; sample++)
    {
      GoodixChicagoSubtemplateView view;

      if (sample == 0)
        g_assert_true (goodix_chicago_enrollment_insert_first (
          enrollment, records, G_N_ELEMENTS (records),
          G_N_ELEMENTS (records), 92, 100, &metric_data, &result, &error));
      else
        g_assert_true (goodix_chicago_enrollment_insert_next (
          enrollment, records, G_N_ELEMENTS (records),
          G_N_ELEMENTS (records), 92, 100, &metric_data, &result, &error));
      g_assert_no_error (error);
      g_assert_cmpuint (result.progress, ==, expected_progress[sample]);
      g_assert_cmpuint (goodix_chicago_enrollment_get_transform_count (
                          enrollment), ==, expected_relations[sample]);
      g_assert_true (goodix_chicago_enrollment_get_subtemplate (
        enrollment, sample, &view));
      g_assert_cmpuint (view.relation_base, ==,
                        sample == 0 ? 0 : expected_relations[sample - 1]);
      if (sample > 0)
        {
          g_assert_cmpuint (view.group_state, ==, 1);
          g_assert_cmpuint (result.position_x, ==, 100);
          g_assert_cmpuint (result.position_y, ==, 100);
        }
    }
  for (guint sample = 0; sample < 8; sample++)
    {
      GoodixChicagoSubtemplateView view;

      g_assert_true (goodix_chicago_enrollment_get_subtemplate (
        enrollment, sample, &view));
      g_assert_cmpuint (view.group_state, ==, 1);
    }
  g_assert_cmpuint (goodix_chicago_enrollment_get_packed_size (enrollment),
                    ==, 10310);
  {
    g_autoptr(GBytes) packed =
      goodix_chicago_enrollment_pack (enrollment, &error);

    g_assert_no_error (error);
    g_assert_nonnull (packed);
    g_assert_cmpuint (g_bytes_get_size (packed), ==, 10310);
  }
}

static void
test_second_insert_oracle (void)
{
  const gchar *old_features_path = g_getenv ("CHICAGO_SECOND_OLD_FEATURES");
  const gchar *new_features_path = g_getenv ("CHICAGO_SECOND_NEW_FEATURES");
  const gchar *old_enhanced_path = g_getenv ("CHICAGO_SECOND_OLD_ENHANCED");
  const gchar *new_enhanced_path = g_getenv ("CHICAGO_SECOND_NEW_ENHANCED");
  const gchar *sequence_path = g_getenv ("CHICAGO_SECOND_SEQUENCE");
  g_autofree gchar *old_features = NULL;
  g_autofree gchar *new_features = NULL;
  g_autofree gchar *old_enhanced = NULL;
  g_autofree gchar *new_enhanced = NULL;
  g_autofree gchar *sequence = NULL;
  g_autoptr(GError) error = NULL;
  g_autoptr(GoodixChicagoEnrollment) enrollment = NULL;
  GoodixChicagoMetricData old_metric_data;
  GoodixChicagoMetricData new_metric_data;
  GoodixChicagoEnrollmentResult result;
  GoodixChicagoRelation debug_relation;
  const GoodixChicagoFeatureRecord *old_records;
  const GoodixChicagoFeatureRecord *new_records;
  const guint32 *expected;
  guint32 old_count;
  guint32 new_count;
  guint old_active = 0;
  guint new_active = 0;
  gsize old_features_size;
  gsize new_features_size;
  gsize old_enhanced_size;
  gsize new_enhanced_size;
  gsize sequence_size;

  if (!old_features_path || !new_features_path || !old_enhanced_path ||
      !new_enhanced_path || !sequence_path)
    {
      g_test_skip ("Chicago Wine second-insert vector was not requested");
      return;
    }
  g_assert_true (g_file_get_contents (old_features_path, &old_features,
                                      &old_features_size, &error));
  g_assert_no_error (error);
  g_assert_true (g_file_get_contents (new_features_path, &new_features,
                                      &new_features_size, &error));
  g_assert_no_error (error);
  g_assert_true (g_file_get_contents (old_enhanced_path, &old_enhanced,
                                      &old_enhanced_size, &error));
  g_assert_no_error (error);
  g_assert_true (g_file_get_contents (new_enhanced_path, &new_enhanced,
                                      &new_enhanced_size, &error));
  g_assert_no_error (error);
  g_assert_true (g_file_get_contents (sequence_path, &sequence,
                                      &sequence_size, &error));
  g_assert_no_error (error);
  memcpy (&old_count, old_features, sizeof (old_count));
  memcpy (&new_count, new_features, sizeof (new_count));
  g_assert_cmpuint (old_features_size, ==,
                    12 + old_count * sizeof (*old_records));
  g_assert_cmpuint (new_features_size, ==,
                    12 + new_count * sizeof (*new_records));
  g_assert_cmpuint (old_enhanced_size, ==, GOODIX_CHICAGO_FEATURE_PIXELS);
  g_assert_cmpuint (new_enhanced_size, ==, GOODIX_CHICAGO_FEATURE_PIXELS);
  g_assert_cmpuint (sequence_size, >=, 12 + 2 * 16 * sizeof (guint32));
  g_assert_cmphex (*(const guint32 *) sequence, ==, 0x53524e45);
  old_records = (const GoodixChicagoFeatureRecord *) (old_features + 12);
  new_records = (const GoodixChicagoFeatureRecord *) (new_features + 12);
  while (old_active < old_count &&
         ((((const guint8 *) &old_records[old_active])[0] & 3u) == 0))
    old_active++;
  while (new_active < new_count &&
         ((((const guint8 *) &new_records[new_active])[0] & 3u) == 0))
    new_active++;
  goodix_chicago_enrollment_build_metric_data (
    (const guint8 *) old_enhanced, &old_metric_data);
  goodix_chicago_enrollment_build_metric_data (
    (const guint8 *) new_enhanced, &new_metric_data);
  {
    gint debug_metric_a;
    gint debug_metric_b;
    gboolean debug_evidence;

    goodix_chicago_enrollment_build_relation (
      old_records, old_count, &old_metric_data,
      new_records, new_count, &new_metric_data, &debug_relation,
      &debug_metric_a, &debug_metric_b, &debug_evidence);
    g_test_message ("relation=%d [%d,%d,%d,%d,%d,%d] metrics=%d/%d evidence=%d",
                    debug_relation.inlier_count,
                    debug_relation.transform[0], debug_relation.transform[1],
                    debug_relation.transform[2], debug_relation.transform[3],
                    debug_relation.transform[4], debug_relation.transform[5],
                    debug_metric_a, debug_metric_b, debug_evidence);
    goodix_chicago_enrollment_calculate_config1_metrics (
      &new_metric_data, &old_metric_data, debug_relation.transform,
      &debug_metric_a, &debug_metric_b);
    g_test_message ("config1 metrics=%d/%d", debug_metric_a,
                    debug_metric_b);
  }
  enrollment = goodix_chicago_enrollment_new ();
  g_assert_true (goodix_chicago_enrollment_insert_first (
    enrollment, old_records, old_count, old_active,
    *(const guint32 *) (old_features + 4),
    *(const guint32 *) (old_features + 8), &old_metric_data, &result, &error));
  g_assert_no_error (error);
  g_assert_true (goodix_chicago_enrollment_insert_second (
    enrollment, new_records, new_count, new_active,
    *(const guint32 *) (new_features + 4),
    *(const guint32 *) (new_features + 8), &new_metric_data, &result, &error));
  g_assert_no_error (error);
  expected = (const guint32 *) (sequence + 12 + 16 * sizeof (guint32));
  g_assert_cmpuint (result.progress, ==, expected[3]);
  g_assert_cmpuint (result.position_x, ==, expected[4]);
  g_assert_cmpuint (result.position_y, ==, expected[5]);
  g_assert_cmpuint (goodix_chicago_enrollment_get_transform_count (
                      enrollment), ==, expected[8]);
}

static void
test_relation_builder_oracle (void)
{
  const gchar *features_path = g_getenv ("CHICAGO_RELATION_FEATURES");
  const gchar *enhanced_path = g_getenv ("CHICAGO_RELATION_ENHANCED");
  const gchar *vector_path = g_getenv ("CHICAGO_RELATION_VECTOR");
  g_autofree gchar *features = NULL;
  g_autofree gchar *enhanced = NULL;
  g_autofree gchar *vector = NULL;
  g_autoptr(GError) error = NULL;
  GoodixChicagoMetricData metric_data;
  GoodixChicagoRelation relation;
  const GoodixChicagoFeatureRecord *records;
  const GoodixChicagoRelation *official;
  guint32 record_count;
  gboolean accepted = FALSE;
  gsize features_size;
  gsize enhanced_size;
  gsize vector_size;

  if (!features_path || !enhanced_path || !vector_path)
    {
      g_test_skip ("Chicago Wine relation-builder vector was not requested");
      return;
    }
  g_assert_true (g_file_get_contents (features_path, &features,
                                      &features_size, &error));
  g_assert_no_error (error);
  g_assert_true (g_file_get_contents (enhanced_path, &enhanced,
                                      &enhanced_size, &error));
  g_assert_no_error (error);
  g_assert_true (g_file_get_contents (vector_path, &vector,
                                      &vector_size, &error));
  g_assert_no_error (error);
  memcpy (&record_count, features, sizeof (record_count));
  g_assert_cmpuint (features_size, ==,
                    12 + record_count * sizeof (*records));
  g_assert_cmpuint (enhanced_size, ==, GOODIX_CHICAGO_FEATURE_PIXELS);
  g_assert_cmpuint (vector_size, >=, 12 + 64 + 0x1c0 +
                    2 * sizeof (*official));
  g_assert_cmphex (*(const guint32 *) vector, ==, 0x54414c52);
  records = (const GoodixChicagoFeatureRecord *) (features + 12);
  official = (const GoodixChicagoRelation *)
    (vector + 12 + 64 + 0x1c0 + sizeof (*official));
  goodix_chicago_enrollment_build_metric_data (
    (const guint8 *) enhanced, &metric_data);
  goodix_chicago_enrollment_build_relation (
    records, record_count, &metric_data, records, record_count, &metric_data,
    &relation, NULL, NULL, &accepted);
  g_assert_cmpmem (&relation, sizeof (relation), official, sizeof (*official));
  g_assert_true (accepted);
}

static void
test_metric_map_oracle (void)
{
  const gchar *enhanced_path = g_getenv ("CHICAGO_METRIC_MAP_ENHANCED");
  const gchar *object_path = g_getenv ("CHICAGO_METRIC_MAP_OBJECT");
  const gchar *position_path = g_getenv ("CHICAGO_POSITION_MAP_OBJECT");
  g_autofree gchar *enhanced = NULL;
  g_autofree gchar *object = NULL;
  g_autofree gchar *position = NULL;
  g_autoptr(GError) error = NULL;
  GoodixChicagoMetricData metric_data;
  guint32 stored_bytes;
  gsize enhanced_size;
  gsize object_size;
  gsize position_size = 0;
  const guint8 *official_primary;
  const guint8 *official_secondary;
  const guint8 *official_validity;
  const guint8 *official_mask;
  gsize object_offset;

  if (!enhanced_path || !object_path)
    {
      g_test_skip ("Chicago Wine metric map object was not requested");
      return;
    }
  g_assert_true (g_file_get_contents (enhanced_path, &enhanced,
                                      &enhanced_size, &error));
  g_assert_no_error (error);
  g_assert_true (g_file_get_contents (object_path, &object, &object_size,
                                      &error));
  g_assert_no_error (error);
  if (position_path)
    {
      g_assert_true (g_file_get_contents (position_path, &position,
                                          &position_size, &error));
      g_assert_no_error (error);
    }
  g_assert_cmpuint (enhanced_size, ==, GOODIX_CHICAGO_FEATURE_PIXELS);
  g_assert_cmpuint (object_size, >=, 4 + 0x160 + 4 + 32 + 160);
  g_assert_cmphex (*(const guint32 *) object, ==, 0x4a424f53);
  object_offset = 4 + 0x160;
  memcpy (&stored_bytes, object + object_offset, sizeof (stored_bytes));
  g_assert_cmpuint (stored_bytes, >=,
                    GOODIX_CHICAGO_METRIC_MAP_PACKED_BYTES);
  official_mask = (const guint8 *) object + 4 + 0x28;
  official_primary = (const guint8 *) object + object_offset + 4 + 32;
  object_offset += 4 + 32 + stored_bytes;
  g_assert_cmpuint (object_size, >=, object_offset + 4 + 32 +
                    GOODIX_CHICAGO_METRIC_MAP_PACKED_BYTES);
  memcpy (&stored_bytes, object + object_offset, sizeof (stored_bytes));
  official_secondary = (const guint8 *) object + object_offset + 4 + 32;
  object_offset += 4 + 32 + stored_bytes;
  g_assert_cmpuint (object_size, >=, object_offset + 4 + 32 +
                    GOODIX_CHICAGO_METRIC_MAP_PACKED_BYTES);
  memcpy (&stored_bytes, object + object_offset, sizeof (stored_bytes));
  official_validity = (const guint8 *) object + object_offset + 4 + 32;

  goodix_chicago_enrollment_build_metric_data (
    (const guint8 *) enhanced, &metric_data);
  g_assert_cmpmem (metric_data.primary, sizeof (metric_data.primary),
                   official_primary,
                   GOODIX_CHICAGO_METRIC_MAP_PACKED_BYTES);
  g_assert_cmpmem (metric_data.secondary, sizeof (metric_data.secondary),
                   official_secondary,
                   GOODIX_CHICAGO_METRIC_MAP_PACKED_BYTES);
  g_assert_cmpmem (metric_data.validity, sizeof (metric_data.validity),
                   official_validity,
                   GOODIX_CHICAGO_METRIC_MAP_PACKED_BYTES);
  g_assert_cmpmem (metric_data.coarse_mask, sizeof (metric_data.coarse_mask),
                   official_mask,
                   GOODIX_CHICAGO_METRIC_MASK_PACKED_BYTES);
  if (position)
    {
      g_assert_cmpuint (position_size, ==,
                        4 + 32 + GOODIX_CHICAGO_METRIC_MAP_PACKED_BYTES);
      g_assert_cmphex (*(const guint32 *) position, ==, 0x424f4d50);
      g_assert_cmpmem (metric_data.position_map,
                       sizeof (metric_data.position_map), position + 4 + 32,
                       GOODIX_CHICAGO_METRIC_MAP_PACKED_BYTES);
    }
}

int
main (int   argc,
      char *argv[])
{
  g_test_init (&argc, &argv, NULL);
  g_test_add_func ("/gdix51c0/chicago-enrollment/first-insert",
                   test_first_insert);
  g_test_add_func ("/gdix51c0/chicago-enrollment/engine-adapter-policy",
                   test_engine_adapter_enrollment_policy);
  g_test_add_func ("/gdix51c0/chicago-enrollment/matches-first-sample-oracle",
                   test_matches_first_sample_oracle);
  g_test_add_func ("/gdix51c0/chicago-enrollment/sequence-oracle-contract",
                   test_sequence_oracle_contract);
  g_test_add_func ("/gdix51c0/chicago-enrollment/native-sequence-oracle",
                   test_native_sequence_oracle);
  g_test_add_func ("/gdix51c0/chicago-enrollment/correspondence-oracle",
                   test_correspondence_oracle);
  g_test_add_func ("/gdix51c0/chicago-enrollment/transform-oracle",
                   test_transform_oracle);
  g_test_add_func ("/gdix51c0/chicago-enrollment/evidence-gate",
                   test_evidence_gate);
  g_test_add_func ("/gdix51c0/chicago-enrollment/relation-metric-oracle",
                   test_relation_metric_oracle);
  g_test_add_func ("/gdix51c0/chicago-enrollment/relation-metric-identity",
                   test_relation_metric_identity);
  g_test_add_func ("/gdix51c0/chicago-enrollment/relation-builder-identity",
                   test_relation_builder_identity);
  g_test_add_func ("/gdix51c0/chicago-enrollment/second-insert-identity",
                   test_second_insert_identity);
  g_test_add_func ("/gdix51c0/chicago-enrollment/repeated-eight-insertions",
                   test_repeated_eight_insertions);
  g_test_add_func ("/gdix51c0/chicago-enrollment/second-insert-oracle",
                   test_second_insert_oracle);
  g_test_add_func ("/gdix51c0/chicago-enrollment/relation-builder-oracle",
                   test_relation_builder_oracle);
  g_test_add_func ("/gdix51c0/chicago-enrollment/metric-map-oracle",
                   test_metric_map_oracle);
  g_test_add_func ("/gdix51c0/chicago-enrollment/identify-metric-oracle",
                   test_identify_metric_oracle);
  g_test_add_func ("/gdix51c0/chicago-enrollment/study-metrics",
                   test_study_metrics);
  g_test_add_func ("/gdix51c0/chicago-enrollment/live-auxiliary-counts-type24",
                   test_live_auxiliary_counts_type24);
  g_test_add_func ("/gdix51c0/chicago-enrollment/live-auxiliary-count-oracle-type24",
                   test_live_auxiliary_count_oracle_type24);
  g_test_add_func ("/gdix51c0/chicago-enrollment/packed-template-roundtrip",
                   test_packed_template_roundtrip);
  g_test_add_func ("/gdix51c0/chicago-enrollment/adaptive-metadata-roundtrip",
                   test_adaptive_metadata_roundtrip);
  g_test_add_func ("/gdix51c0/chicago-enrollment/adaptive-study-append",
                   test_adaptive_study_append);
  g_test_add_func ("/gdix51c0/chicago-enrollment/match-order-roundtrip",
                   test_match_order_roundtrip);
  g_test_add_func ("/gdix51c0/chicago-enrollment/fallback-metric-records",
                   test_fallback_metric_records);
  return g_test_run ();
}
