/*
 * PRECOMPUTED LOOKUP TABLES FOR ULTRA-FAST ADC CONVERSION
 *
 * Dynamic allocation of lookup tables based on ADC bit depth.
 * Supports 8-bit, 10-bit, 12-bit, 14-bit, and 16-bit ADCs.
 *
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "iqconverter_lut.h"
#include "iqconverter_common.h"
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

/* Default table, for converters used without a device */
iqconv_lut_t iqconv_lut = {
	.lut_float32 = NULL,
	.lut_int16 = NULL,
	.adc_bits = 0,
	.lut_size = 0,
	.midpoint = 0
};

/* One table per ADC width, built on first use, never rewritten */
static iqconv_lut_t *lut_tables[IQCONV_ADC_16BIT + 1];
static pthread_mutex_t lut_mutex = PTHREAD_MUTEX_INITIALIZER;

static void lut_table_free(iqconv_lut_t *table)
{
	if (!table)
		return;
	if (table->lut_float32)
		iqconv_aligned_free(table->lut_float32);
	if (table->lut_int16)
		iqconv_aligned_free(table->lut_int16);
	free(table);
}

static iqconv_lut_t *lut_table_build(int adc_bits)
{
	iqconv_lut_t *table;
	int i;
	int lut_size = 1 << adc_bits;		/* 2^adc_bits */
	int midpoint = lut_size >> 1;		/* lut_size / 2 */
	int shift = 16 - adc_bits;		/* 10-bit: 6, 12-bit: 4, 14-bit: 2, 16-bit: 0 */
	float scale_f = 1.0f / (float)midpoint;

	table = (iqconv_lut_t *)calloc(1, sizeof(*table));
	if (!table)
		return NULL;
	/* Max size, 64-byte aligned for SIMD */
	table->lut_float32 = (float *)iqconv_aligned_alloc(
		IQCONV_LUT_SIZE_MAX * sizeof(float));
	table->lut_int16 = (int16_t *)iqconv_aligned_alloc(
		IQCONV_LUT_SIZE_MAX * sizeof(int16_t));
	if (!table->lut_float32 || !table->lut_int16) {
		lut_table_free(table);
		return NULL;
	}

	/* Fill whole tables: codes above the ADC width map to the top code */

	/* Fill float32 LUT: lut[adc] = (adc - midpoint) / midpoint */
	for (i = 0; i < IQCONV_LUT_SIZE_MAX; i++) {
		int code = (i < lut_size) ? i : (lut_size - 1);
		table->lut_float32[i] = (float)(code - midpoint) * scale_f;
	}

	/* Fill int16 LUT: lut[adc] = (adc - midpoint) << shift
	 * (multiply: a left shift of a negative value is undefined) */
	for (i = 0; i < IQCONV_LUT_SIZE_MAX; i++) {
		int code = (i < lut_size) ? i : (lut_size - 1);
		int32_t val = (code - midpoint) * (1 << shift);
		/* Clamp to int16 range (shouldn't be needed, but safe) */
		if (val > 32767) val = 32767;
		if (val < -32768) val = -32768;
		table->lut_int16[i] = (int16_t)val;
	}

	table->adc_bits = adc_bits;
	table->lut_size = lut_size;
	table->midpoint = midpoint;
	return table;
}

/* With lut_mutex held */
static const iqconv_lut_t *lut_get_locked(int adc_bits)
{
	/* Validate ADC bits (8, 10, 12, 14, or 16) */
	if (adc_bits != IQCONV_ADC_8BIT && adc_bits != IQCONV_ADC_10BIT &&
	    adc_bits != IQCONV_ADC_12BIT && adc_bits != IQCONV_ADC_14BIT &&
	    adc_bits != IQCONV_ADC_16BIT)
		return NULL;

	if (!lut_tables[adc_bits])
		lut_tables[adc_bits] = lut_table_build(adc_bits);
	return lut_tables[adc_bits];
}

const iqconv_lut_t *iqconv_lut_get(int adc_bits)
{
	const iqconv_lut_t *table;

	pthread_mutex_lock(&lut_mutex);
	table = lut_get_locked(adc_bits);
	pthread_mutex_unlock(&lut_mutex);
	return table;
}

/*
 * Select the default table for specified ADC bit depth
 */
int iqconv_lut_init(int adc_bits)
{
	const iqconv_lut_t *table;

	pthread_mutex_lock(&lut_mutex);
	table = lut_get_locked(adc_bits);
	if (table)
		iqconv_lut = *table;
	pthread_mutex_unlock(&lut_mutex);
	return table ? 0 : -1;
}

/*
 * Free every table and reset the default
 */
void iqconv_lut_free(void)
{
	int i;

	pthread_mutex_lock(&lut_mutex);
	for (i = 0; i <= IQCONV_ADC_16BIT; i++) {
		lut_table_free(lut_tables[i]);
		lut_tables[i] = NULL;
	}
	memset(&iqconv_lut, 0, sizeof(iqconv_lut));
	pthread_mutex_unlock(&lut_mutex);
}

/*
 * Check if LUT is initialized
 */
int iqconv_lut_is_initialized(void)
{
	return (iqconv_lut.lut_float32 != NULL && iqconv_lut.lut_int16 != NULL);
}

/*
 * Get current ADC bit depth
 */
int iqconv_lut_get_adc_bits(void)
{
	return iqconv_lut.adc_bits;
}
