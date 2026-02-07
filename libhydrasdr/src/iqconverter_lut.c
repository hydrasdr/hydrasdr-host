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
#include <stdlib.h>
#include <string.h>

/* Global LUT instance */
iqconv_lut_t iqconv_lut = {
	.lut_float32 = NULL,
	.lut_int16 = NULL,
	.adc_bits = 0,
	.lut_size = 0,
	.midpoint = 0
};

/*
 * Initialize lookup tables for specified ADC bit depth
 */
int iqconv_lut_init(int adc_bits)
{
	int i;
	int lut_size;
	int midpoint;
	int shift;
	float scale_f;

	/* Validate ADC bits (8, 10, 12, 14, or 16) */
	if (adc_bits != IQCONV_ADC_8BIT && adc_bits != IQCONV_ADC_10BIT &&
	    adc_bits != IQCONV_ADC_12BIT && adc_bits != IQCONV_ADC_14BIT &&
	    adc_bits != IQCONV_ADC_16BIT)
		return -1;

	/* Free existing tables if re-initializing */
	iqconv_lut_free();

	/* Calculate parameters */
	lut_size = 1 << adc_bits;	/* 2^adc_bits */
	midpoint = lut_size >> 1;	/* lut_size / 2 */
	scale_f = 1.0f / (float)midpoint;

	/* Calculate shift for int16 scaling to maximize range */
	shift = 16 - adc_bits;		/* 10-bit: 6, 12-bit: 4, 14-bit: 2, 16-bit: 0 */

	/* Allocate float32 LUT (64-byte aligned for SIMD) */
	iqconv_lut.lut_float32 = (float *)iqconv_aligned_alloc(lut_size * sizeof(float));
	if (!iqconv_lut.lut_float32)
		return -1;

	/* Allocate int16 LUT (64-byte aligned for SIMD) */
	iqconv_lut.lut_int16 = (int16_t *)iqconv_aligned_alloc(lut_size * sizeof(int16_t));
	if (!iqconv_lut.lut_int16) {
		iqconv_aligned_free(iqconv_lut.lut_float32);
		iqconv_lut.lut_float32 = NULL;
		return -1;
	}

	/* Fill float32 LUT: lut[adc] = (adc - midpoint) / midpoint */
	for (i = 0; i < lut_size; i++) {
		iqconv_lut.lut_float32[i] = (float)(i - midpoint) * scale_f;
	}

	/* Fill int16 LUT: lut[adc] = (adc - midpoint) << shift */
	for (i = 0; i < lut_size; i++) {
		int32_t val = (i - midpoint) << shift;
		/* Clamp to int16 range (shouldn't be needed, but safe) */
		if (val > 32767) val = 32767;
		if (val < -32768) val = -32768;
		iqconv_lut.lut_int16[i] = (int16_t)val;
	}

	/* Store configuration */
	iqconv_lut.adc_bits = adc_bits;
	iqconv_lut.lut_size = lut_size;
	iqconv_lut.midpoint = midpoint;

	return 0;
}

/*
 * Free lookup tables and reset state
 */
void iqconv_lut_free(void)
{
	if (iqconv_lut.lut_float32) {
		iqconv_aligned_free(iqconv_lut.lut_float32);
		iqconv_lut.lut_float32 = NULL;
	}
	if (iqconv_lut.lut_int16) {
		iqconv_aligned_free(iqconv_lut.lut_int16);
		iqconv_lut.lut_int16 = NULL;
	}
	iqconv_lut.adc_bits = 0;
	iqconv_lut.lut_size = 0;
	iqconv_lut.midpoint = 0;
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
