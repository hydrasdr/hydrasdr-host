/*
 * PRECOMPUTED LOOKUP TABLES FOR ULTRA-FAST ADC CONVERSION
 *
 * Hardware-agnostic design supporting multiple ADC bit depths:
 *   - 8-bit ADC: 256 entries
 *   - 10-bit ADC: 1024 entries
 *   - 12-bit ADC: 4096 entries
 *   - 14-bit ADC: 16384 entries
 *   - 16-bit ADC: 65536 entries
 *
 * Tables are dynamically allocated at runtime based on configured ADC bits.
 *
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef IQCONVERTER_LUT_H
#define IQCONVERTER_LUT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Supported ADC bit depths
 */
#define IQCONV_ADC_8BIT   8
#define IQCONV_ADC_10BIT  10
#define IQCONV_ADC_12BIT  12
#define IQCONV_ADC_14BIT  14
#define IQCONV_ADC_16BIT  16

/*
 * Maximum LUT size (for 16-bit ADC)
 */
#define IQCONV_LUT_SIZE_MAX 65536

/*
 * LUT Context Structure
 *
 * Holds dynamically allocated lookup tables for a specific ADC configuration.
 * Tables are allocated with 64-byte alignment for SIMD optimization.
 */
typedef struct {
	float *lut_float32;      /* Float32 LUT: adc -> [-1.0, +1.0) */
	int16_t *lut_int16;      /* Int16 LUT: adc -> [-32768, +32767] */
	int adc_bits;            /* ADC bit depth (8, 10, 12, 14, or 16) */
	int lut_size;            /* LUT size = 2^adc_bits */
	int midpoint;            /* Midpoint = lut_size / 2 */
} iqconv_lut_t;

/*
 * Global LUT instance (initialized at startup)
 */
extern iqconv_lut_t iqconv_lut;

/*
 * Initialize lookup tables for specified ADC bit depth
 *
 * Parameters:
 *   adc_bits - ADC bit depth (8, 10, 12, 14, or 16)
 *
 * Returns:
 *   0 on success, -1 on failure (invalid adc_bits or allocation failure)
 *
 * LUT values:
 *   Float32: lut[adc] = (adc - midpoint) / midpoint
 *            Range: [-1.0, +1.0) (exactly -1.0 at 0, approaches +1.0 at max)
 *
 *   Int16:   lut[adc] = (adc - midpoint) * scale
 *            Scale chosen to maximize int16 range without overflow
 *            8-bit: << 8, 10-bit: << 6, 12-bit: << 4, 14-bit: << 2, 16-bit: << 0
 *
 * Memory usage:
 *   8-bit:  256 * 6 = 1.5 KB
 *   10-bit: 1024 * 6 = 6 KB
 *   12-bit: 4096 * 6 = 24 KB (fits in L1 cache)
 *   14-bit: 16384 * 6 = 96 KB (fits in L2 cache)
 *   16-bit: 65536 * 6 = 384 KB
 *
 * Example:
 *   iqconv_lut_init(IQCONV_ADC_12BIT);  // Initialize for 12-bit ADC
 *   float val = iqconv_lut.lut_float32[adc_sample];
 */
int iqconv_lut_init(int adc_bits);

/*
 * Free lookup tables and reset state
 */
void iqconv_lut_free(void);

/*
 * Check if LUT is initialized
 *
 * Returns:
 *   1 if initialized, 0 if not
 */
int iqconv_lut_is_initialized(void);

/*
 * Get current ADC bit depth
 *
 * Returns:
 *   ADC bit depth (8, 10, 12, 14, or 16) or 0 if not initialized
 */
int iqconv_lut_get_adc_bits(void);

/*
 * Convenience macros for direct LUT access (after initialization)
 *
 * Usage:
 *   float f = IQCONV_LUT_F32(adc_sample);
 *   int16_t i = IQCONV_LUT_I16(adc_sample);
 */
#define IQCONV_LUT_F32(adc) (iqconv_lut.lut_float32[(adc)])
#define IQCONV_LUT_I16(adc) (iqconv_lut.lut_int16[(adc)])

#ifdef __cplusplus
}
#endif

#endif /* IQCONVERTER_LUT_H */
