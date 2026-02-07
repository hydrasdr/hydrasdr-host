/*
 * OPTIMIZED FLOAT32 DDC IMPLEMENTATION - 65-TAP VARIANT
 *
 * Based on commit https://github.com/airspy/airspyone_host/commit/091c6f7449bc976f97bb327242532bfb1001d116
 *
 * Original work Copyright (C) 2014, Youssef Touil <youssef@airspy.com>
 * Modifications Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "iqconverter_float_opt65.h"
#include "iqconverter_common.h"
#include "iqconverter_lut.h"
#include <stdlib.h>
#include <string.h>

/* Power-of-2 delay for branchless modulo (65-tap specific) */
#define DELAY_SIZE 16
#define DELAY_MASK 15

iqconverter_float_opt65_t *iqconverter_float_opt65_create(const float *hb_kernel, int len)
{
	int i, j;
	iqconverter_float_opt65_t *cnv;

	/*
	 * Validate filter length - this algorithm is specifically for 65-tap filters
	 */
	if (len != 65)
		return NULL;

	cnv = (iqconverter_float_opt65_t *) iqconv_aligned_alloc(sizeof(iqconverter_float_opt65_t));
	if (OPT_UNLIKELY(!cnv))
		return NULL;

	cnv->total_taps = len;
	/*
	 * 65-tap halfband filter structure:
	 * - Non-zero coefficients at ODD indices: 1,3,5,...,63 = 32 taps
	 * - Center tap at index 32 = 0.5 (stored separately in hbc)
	 * - FIR uses 32-tap symmetric polyphase filter (16 pairs)
	 */
	cnv->len = (len - 1) / 2;	/* 65-tap -> 32 non-zero taps at odd indices */
	cnv->hbc = hb_kernel[len / 2];	/* Center tap (0.5) at index 32 */

	/* Allocate 64-byte aligned buffers */
	cnv->fir_kernel = (float *)iqconv_aligned_alloc(ALIGN64(cnv->len * sizeof(float)));
	cnv->fir_queue = (float *)iqconv_aligned_alloc(HISTORY_SIZE * 2 * sizeof(float));
	cnv->delay_line = (float *)iqconv_aligned_alloc(ALIGN64(DELAY_SIZE * sizeof(float)));

	if (OPT_UNLIKELY(!cnv->fir_kernel || !cnv->fir_queue || !cnv->delay_line)) {
		iqconverter_float_opt65_free(cnv);
		return NULL;
	}

	iqconverter_float_opt65_reset(cnv);

	/*
	 * Extract non-zero taps (every other coefficient)
	 * For 65-tap filter, non-zero coefficients are at ODD indices (1,3,5...63)
	 * Total: 32 coefficients at indices 1,3,5,...,63
	 */
	for (i = 0, j = 1; i < cnv->len; i++, j += 2)
		cnv->fir_kernel[i] = hb_kernel[j];

	return cnv;
}

void iqconverter_float_opt65_free(iqconverter_float_opt65_t *cnv)
{
	if (OPT_UNLIKELY(!cnv))
		return;

	if (cnv->fir_kernel)
		iqconv_aligned_free(cnv->fir_kernel);
	if (cnv->fir_queue)
		iqconv_aligned_free(cnv->fir_queue);
	if (cnv->delay_line)
		iqconv_aligned_free(cnv->delay_line);
	iqconv_aligned_free(cnv);
}

void iqconverter_float_opt65_reset(iqconverter_float_opt65_t *cnv)
{
	if (OPT_UNLIKELY(!cnv))
		return;

	cnv->avg = 0.0f;
	cnv->fir_index = 0;
	cnv->delay_index = 0;
	if (cnv->fir_queue)
		memset(cnv->fir_queue, 0, HISTORY_SIZE * 2 * sizeof(float));
	if (cnv->delay_line)
		memset(cnv->delay_line, 0, DELAY_SIZE * sizeof(float));
}

/*
 * FUSED PROCESSING: uint16_t ADC samples -> float32 I/Q output (65-tap variant)
 * Uses precomputed LUT for ultra-fast ADC -> float conversion.
 */
OPT_HOT
void iqconverter_float_opt65_process_u16(iqconverter_float_opt65_t *cnv,
					 const uint16_t * OPT_RESTRICT src,
					 float * OPT_RESTRICT dest, int len)
{
	if (OPT_UNLIKELY(!cnv || !src || !dest || (len & 3)))
		return;

	float avg = cnv->avg;
	const float hbc = cnv->hbc;

	/* Apply alignment hints to enable better vectorization */
	const float * OPT_RESTRICT kernel = (const float *)OPT_ALIGNED_64(cnv->fir_kernel);
	float * OPT_RESTRICT fir_hist = (float *)OPT_ALIGNED_64(cnv->fir_queue);
	float * OPT_RESTRICT dly_line = (float *)OPT_ALIGNED_64(cnv->delay_line);
	const float * OPT_RESTRICT lut = (const float *)OPT_ALIGNED_64(iqconv_lut.lut_float32);

	int fir_idx = cnv->fir_index;
	int dly_idx = cnv->delay_index;
	int i;

	float x0, x1, x2, x3;
	float fir_in0, fir_in1;
	float dly_in0, dly_in1;
	float acc0, acc1;
	float q0, q1;
	const float *src0, *src1;

	/* Prefetch first batch of LUT entries */
	OPT_PREFETCH_READ(&lut[src[0]]);
	OPT_PREFETCH_READ(&lut[src[1]]);

	for (i = 0; i < len; i += 4) {
		/* Prefetch next batch of LUT entries for next iteration */
		if (OPT_LIKELY(i + 8 < len)) {
			OPT_PREFETCH_READ(&lut[src[i + 4]]);
			OPT_PREFETCH_READ(&lut[src[i + 5]]);
			OPT_PREFETCH_READ(&lut[src[i + 6]]);
			OPT_PREFETCH_READ(&lut[src[i + 7]]);
		}
		/* FUSED: LUT-based uint16_t -> float + DC Removal */
		x0 = lut[src[i + 0]] - avg; avg += DC_REMOVAL_ALPHA * x0;
		x1 = lut[src[i + 1]] - avg; avg += DC_REMOVAL_ALPHA * x1;
		x2 = lut[src[i + 2]] - avg; avg += DC_REMOVAL_ALPHA * x2;
		x3 = lut[src[i + 3]] - avg; avg += DC_REMOVAL_ALPHA * x3;

		/* Polyphase Routing */
		fir_in0 = -x0;
		dly_in0 = -x1 * hbc;
		fir_in1 = x2;
		dly_in1 = x3 * hbc;

		/* FIR I-Channel (1) - 32-tap symmetric filter (16 pairs) */
		fir_hist[fir_idx] = fir_in0;
		fir_hist[fir_idx + HISTORY_SIZE] = fir_in0;
		src0 = &fir_hist[fir_idx + HISTORY_SIZE - 32 + 1];

		acc0 = kernel[0]  * (src0[0]  + src0[31])
		     + kernel[1]  * (src0[1]  + src0[30])
		     + kernel[2]  * (src0[2]  + src0[29])
		     + kernel[3]  * (src0[3]  + src0[28])
		     + kernel[4]  * (src0[4]  + src0[27])
		     + kernel[5]  * (src0[5]  + src0[26])
		     + kernel[6]  * (src0[6]  + src0[25])
		     + kernel[7]  * (src0[7]  + src0[24])
		     + kernel[8]  * (src0[8]  + src0[23])
		     + kernel[9]  * (src0[9]  + src0[22])
		     + kernel[10] * (src0[10] + src0[21])
		     + kernel[11] * (src0[11] + src0[20])
		     + kernel[12] * (src0[12] + src0[19])
		     + kernel[13] * (src0[13] + src0[18])
		     + kernel[14] * (src0[14] + src0[17])
		     + kernel[15] * (src0[15] + src0[16]);
		fir_idx = (fir_idx + 1) & FIR_MASK;

		/* FIR I-Channel (2) - 32-tap symmetric filter (16 pairs) */
		fir_hist[fir_idx] = fir_in1;
		fir_hist[fir_idx + HISTORY_SIZE] = fir_in1;
		src1 = &fir_hist[fir_idx + HISTORY_SIZE - 32 + 1];

		acc1 = kernel[0]  * (src1[0]  + src1[31])
		     + kernel[1]  * (src1[1]  + src1[30])
		     + kernel[2]  * (src1[2]  + src1[29])
		     + kernel[3]  * (src1[3]  + src1[28])
		     + kernel[4]  * (src1[4]  + src1[27])
		     + kernel[5]  * (src1[5]  + src1[26])
		     + kernel[6]  * (src1[6]  + src1[25])
		     + kernel[7]  * (src1[7]  + src1[24])
		     + kernel[8]  * (src1[8]  + src1[23])
		     + kernel[9]  * (src1[9]  + src1[22])
		     + kernel[10] * (src1[10] + src1[21])
		     + kernel[11] * (src1[11] + src1[20])
		     + kernel[12] * (src1[12] + src1[19])
		     + kernel[13] * (src1[13] + src1[18])
		     + kernel[14] * (src1[14] + src1[17])
		     + kernel[15] * (src1[15] + src1[16]);
		fir_idx = (fir_idx + 1) & FIR_MASK;

		/* Delay Q-Channel - BRANCHLESS with power-of-2 mask */
		q0 = dly_line[dly_idx];
		dly_line[dly_idx] = dly_in0;
		dly_idx = (dly_idx + 1) & DELAY_MASK;

		q1 = dly_line[dly_idx];
		dly_line[dly_idx] = dly_in1;
		dly_idx = (dly_idx + 1) & DELAY_MASK;

		/* Output */
		dest[i + 0] = acc0;
		dest[i + 1] = q0;
		dest[i + 2] = acc1;
		dest[i + 3] = q1;
	}

	cnv->avg = avg;
	cnv->fir_index = fir_idx;
	cnv->delay_index = dly_idx;
}
