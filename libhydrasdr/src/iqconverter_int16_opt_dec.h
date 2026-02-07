/*
 * INT16 DDC WITH CASCADED DECIMATION
 *
 * Extends iqconverter_int16_opt with configurable decimation (1x, 2x, 4x, 8x, 16x, 32x, 64x).
 * Uses cascaded halfband stages for high-quality decimation.
 *
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef IQCONVERTER_INT16_OPT_DEC_H
#define IQCONVERTER_INT16_OPT_DEC_H

#include <stdint.h>
#include "iqconverter_int16_opt.h"
#include "iqconverter_decimator.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Int16 DDC with cascaded decimation context
 *
 * Combines base DDC (iqconverter_int16_opt_t) with up to DEC_MAX_STAGES cascaded
 * halfband decimation stages for 1x, 2x, 4x, 8x, 16x, 32x, or 64x decimation.
 */
typedef struct {
	iqconverter_int16_opt_t *base_ctx;   /* Base DDC instance */
	int decimation_factor;                /* Total decimation: 1, 2, 4, 8, 16, 32, or 64 */
	int num_dec_stages;                   /* Number of active stages: 0 to DEC_MAX_STAGES */
	dec_stage_int16_t stages[DEC_MAX_STAGES]; /* Cascaded stages */
	int16_t *inter_buf;                   /* Intermediate I/Q buffer */
	int inter_buf_len;                    /* Buffer length (in I/Q pairs) */
} iqconverter_int16_opt_dec_t;

/**
 * Create int16 DDC with decimation
 *
 * Parameters:
 *   hb_kernel   - Halfband filter coefficients for base DDC (Q15)
 *   len         - Filter length (33, 47, 65, or 83)
 *   decimation  - Decimation factor (1, 2, 4, 8, 16, 32, or 64)
 *
 * Returns:
 *   Pointer to DDC instance, or NULL on failure
 */
iqconverter_int16_opt_dec_t *iqconverter_int16_opt_dec_create(
	const int16_t *hb_kernel, int len, int decimation);

/**
 * Free int16 DDC with decimation
 */
void iqconverter_int16_opt_dec_free(iqconverter_int16_opt_dec_t *cnv);

/**
 * Reset int16 DDC with decimation
 */
void iqconverter_int16_opt_dec_reset(iqconverter_int16_opt_dec_t *cnv);

/**
 * Process ADC samples through DDC with decimation
 *
 * Parameters:
 *   cnv  - DDC instance
 *   src  - Raw uint16_t ADC samples
 *   dest - Output int16 I/Q samples (len/decimation pairs)
 *   len  - Number of input samples (must be multiple of 4 * decimation)
 *
 * Output length: len / decimation_factor I/Q pairs
 *               (len / decimation_factor * 2 int16s)
 */
void iqconverter_int16_opt_dec_process_u16(iqconverter_int16_opt_dec_t *cnv,
					   const uint16_t *src,
					   int16_t *dest, int len);

/**
 * Get decimation factor
 */
int iqconverter_int16_opt_dec_get_decimation(iqconverter_int16_opt_dec_t *cnv);

/**
 * Get output length for given input length
 */
int iqconverter_int16_opt_dec_get_output_len(iqconverter_int16_opt_dec_t *cnv,
					     int input_len);

/**
 * Get total group delay in samples at input rate (Fs)
 *
 * Returns the total delay through the entire DDC chain including:
 *   - Base DDC filter delay (depends on filter length)
 *   - All cascaded decimation stage delays
 *
 * Parameters:
 *   cnv             - DDC instance
 *   base_filter_len - Length of base DDC halfband filter (e.g., 47)
 *
 * Returns:
 *   Total delay in samples at input rate Fs
 *
 * To get delay at output rate: delay_output = delay_input / decimation
 *
 * Example for 47-tap DDC with 4x decimation:
 *   delay_fs = get_delay(cnv, 47);    // Returns 48 samples at Fs
 *   delay_out = 48 / 4;               // 12 samples at Fs/4 output
 */
int iqconverter_int16_opt_dec_get_delay(iqconverter_int16_opt_dec_t *cnv,
					int base_filter_len);

/**
 * Get group delay in samples at output rate
 *
 * Convenience function that returns the delay expressed at the output
 * sample rate. This is useful for timestamp adjustment.
 *
 * Parameters:
 *   cnv             - DDC instance
 *   base_filter_len - Length of base DDC halfband filter (e.g., 47)
 *
 * Returns:
 *   Delay in samples at output rate (Fs/decimation)
 *   May be fractional for some configurations.
 */
float iqconverter_int16_opt_dec_get_delay_output(iqconverter_int16_opt_dec_t *cnv,
						 int base_filter_len);

#ifdef __cplusplus
}
#endif

#endif /* IQCONVERTER_INT16_OPT_DEC_H */
