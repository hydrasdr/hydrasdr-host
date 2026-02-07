/*
 * DDC DECIMATION STAGE IMPLEMENTATION
 *
 * Optimized 33-tap halfband decimation for cascaded stages.
 * Uses HB_KERNEL_*_33_FAST directly from filters_opt.h.
 *
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "iqconverter_decimator.h"
#include "compat_opt.h"
#include <stdlib.h>
#include <string.h>

/* ========================================================================
 * INT16 SATURATION MACRO
 *
 * Clamps int32 accumulator result to int16 range [-32768, +32767].
 * Required because the 2x gain per stage can cause overflow with strong
 * signals when AGC is disabled.
 *
 * Uses branchless implementation for minimal performance impact:
 * - If value is in range: returns value as int16_t
 * - If overflow (>32767): returns 32767
 * - If underflow (<-32768): returns -32768
 * ======================================================================== */
#define SAT_I16(x) ( \
	((x) > 32767) ? (int16_t)32767 : \
	(((x) < -32768) ? (int16_t)-32768 : (int16_t)(x)) \
)

/* ========================================================================
 * STAGE PARAMETER LOOKUP
 * ======================================================================== */

int dec_get_stage_params(int stage_num,
			 int *out_taps, int *out_sym_len,
			 int *out_buf_size, int *out_buf_mask)
{
	if (stage_num < 0 || stage_num >= DEC_MAX_STAGES)
		return -1;

	if (stage_num < DEC_STAGE_17TAP_THRESHOLD) {
		/* Stages 0-2: use 33-tap filter */
		if (out_taps)     *out_taps = DEC_HB33_TAPS;
		if (out_sym_len)  *out_sym_len = DEC_HB33_SYM_PAIRS;
		if (out_buf_size) *out_buf_size = DEC_HB33_BUF_SIZE;
		if (out_buf_mask) *out_buf_mask = DEC_HB33_BUF_MASK;
	} else {
		/* Stages 3+: use 17-tap filter */
		if (out_taps)     *out_taps = DEC_HB17_TAPS;
		if (out_sym_len)  *out_sym_len = DEC_HB17_SYM_PAIRS;
		if (out_buf_size) *out_buf_size = DEC_HB17_BUF_SIZE;
		if (out_buf_mask) *out_buf_mask = DEC_HB17_BUF_MASK;
	}
	return 0;
}

/* ========================================================================
 * FLOAT32 STAGE LIFECYCLE
 * ======================================================================== */

int dec_stage_float_init(dec_stage_float_t *stage, int stage_num)
{
	int buf_size;

	if (!stage)
		return -1;

	if (stage_num < 0 || stage_num >= DEC_MAX_STAGES)
		return -1;

	/* Set filter parameters based on stage number */
	if (stage_num < DEC_STAGE_17TAP_THRESHOLD) {
		/* Stages 0-2: use 33-tap filter */
		stage->taps = DEC_HB33_TAPS;
		stage->buf_size = DEC_HB33_BUF_SIZE;
		stage->buf_mask = DEC_HB33_BUF_MASK;
		stage->filter_type = DEC_FILTER_33TAP;
		buf_size = DEC_HB33_BUF_SIZE;
	} else {
		/* Stages 3+: use 17-tap filter */
		stage->taps = DEC_HB17_TAPS;
		stage->buf_size = DEC_HB17_BUF_SIZE;
		stage->buf_mask = DEC_HB17_BUF_MASK;
		stage->filter_type = DEC_FILTER_17TAP;
		buf_size = DEC_HB17_BUF_SIZE;
	}
	stage->fir_index = 0;

	/* Allocate interleaved I/Q buffer (mirrored: 2x size) */
	stage->queue_iq = (float *)opt_aligned_alloc(
		buf_size * 2 * 2 * sizeof(float), 64);

	if (!stage->queue_iq) {
		return -1;
	}

	dec_stage_float_reset(stage);
	return 0;
}

void dec_stage_float_free(dec_stage_float_t *stage)
{
	if (!stage)
		return;

	if (stage->queue_iq) {
		opt_aligned_free(stage->queue_iq);
		stage->queue_iq = NULL;
	}
}

void dec_stage_float_reset(dec_stage_float_t *stage)
{
	if (!stage)
		return;

	stage->fir_index = 0;

	if (stage->queue_iq)
		memset(stage->queue_iq, 0,
		       stage->buf_size * 2 * 2 * sizeof(float));
}

/* ========================================================================
 * INT16 STAGE LIFECYCLE
 * ======================================================================== */

int dec_stage_int16_init(dec_stage_int16_t *stage, int stage_num)
{
	int buf_size;

	if (!stage)
		return -1;

	if (stage_num < 0 || stage_num >= DEC_MAX_STAGES)
		return -1;

	/* Set filter parameters based on stage number */
	if (stage_num < DEC_STAGE_17TAP_THRESHOLD) {
		/* Stages 0-2: use 33-tap filter */
		stage->taps = DEC_HB33_TAPS;
		stage->buf_size = DEC_HB33_BUF_SIZE;
		stage->buf_mask = DEC_HB33_BUF_MASK;
		stage->filter_type = DEC_FILTER_33TAP;
		buf_size = DEC_HB33_BUF_SIZE;
	} else {
		/* Stages 3+: use 17-tap filter */
		stage->taps = DEC_HB17_TAPS;
		stage->buf_size = DEC_HB17_BUF_SIZE;
		stage->buf_mask = DEC_HB17_BUF_MASK;
		stage->filter_type = DEC_FILTER_17TAP;
		buf_size = DEC_HB17_BUF_SIZE;
	}
	stage->fir_index = 0;

	/*
	 * Allocate interleaved I/Q buffer (mirrored: 2x size)
	 * Using int16_t instead of int32_t halves memory bandwidth.
	 * Widening to int32_t happens only during MAC operations.
	 */
	stage->queue_iq = (int16_t *)opt_aligned_alloc(
		buf_size * 2 * 2 * sizeof(int16_t), 64);

	if (!stage->queue_iq) {
		return -1;
	}

	dec_stage_int16_reset(stage);
	return 0;
}

void dec_stage_int16_free(dec_stage_int16_t *stage)
{
	if (!stage)
		return;

	if (stage->queue_iq) {
		opt_aligned_free(stage->queue_iq);
		stage->queue_iq = NULL;
	}
}

void dec_stage_int16_reset(dec_stage_int16_t *stage)
{
	if (!stage)
		return;

	stage->fir_index = 0;

	if (stage->queue_iq)
		memset(stage->queue_iq, 0,
		       stage->buf_size * 2 * 2 * sizeof(int16_t));
}

/* ========================================================================
 * OPTIMIZED FLOAT32 33-TAP PROCESSING
 *
 * Uses HB_KERNEL_FLOAT_33_FAST directly from filters_opt.h.
 * Interleaved I/Q buffer + 4x unrolling (4 inputs -> 2 outputs per iter).
 *
 * 33-tap halfband: 8 symmetric pairs + center (0.5)
 * Non-zero coefficients at indices: 1,3,5,7,9,11,13,15,16,17,19,21,23,25,27,29,31
 *
 * Gain characteristics:
 *   - Coefficient sum: ~0.9992 (unity DC gain)
 *   - No output scaling applied (direct assignment)
 *   - SNR improvement from decimation manifests as lower noise floor
 *   - Use float32 as precision reference; int16 for max dynamic range
 * ======================================================================== */

OPT_HOT
void dec_hb33_float_process(dec_stage_float_t *stage,
			    const float *src, float *dest, int len)
{
	int i, out_idx;
	int idx;
	float *q;
	const int buf_size = DEC_HB33_BUF_SIZE;

	if (OPT_UNLIKELY(!src || !dest || len < 4))
		return;

	q = stage->queue_iq;
	idx = stage->fir_index;
	out_idx = 0;

	OPT_PREFETCH_READ(&src[0]);

	/* Process 4 input samples -> 2 output samples per iteration */
	for (i = 0; i < len; i += 4) {
		float i_in0, q_in0, i_in1, q_in1, i_in2, q_in2, i_in3, q_in3;
		float acc_i0, acc_q0, acc_i1, acc_q1;
		const float *s;
		int idx0, idx1;

		/* Load 4 input I/Q pairs */
		i_in0 = src[i * 2 + 0];
		q_in0 = src[i * 2 + 1];
		i_in1 = src[i * 2 + 2];
		q_in1 = src[i * 2 + 3];
		i_in2 = src[i * 2 + 4];
		q_in2 = src[i * 2 + 5];
		i_in3 = src[i * 2 + 6];
		q_in3 = src[i * 2 + 7];

		/* Prefetch next batch */
		if (OPT_LIKELY(i + 8 < len)) {
			OPT_PREFETCH_READ(&src[(i + 8) * 2]);
		}

		/* Store sample 0 (mirrored interleaved) */
		q[idx * 2]     = i_in0;
		q[idx * 2 + 1] = q_in0;
		q[(idx + buf_size) * 2]     = i_in0;
		q[(idx + buf_size) * 2 + 1] = q_in0;
		idx = (idx + 1) & DEC_HB33_BUF_MASK;

		/* Store sample 1 */
		q[idx * 2]     = i_in1;
		q[idx * 2 + 1] = q_in1;
		q[(idx + buf_size) * 2]     = i_in1;
		q[(idx + buf_size) * 2 + 1] = q_in1;
		idx0 = idx;
		idx = (idx + 1) & DEC_HB33_BUF_MASK;

		/* Store sample 2 */
		q[idx * 2]     = i_in2;
		q[idx * 2 + 1] = q_in2;
		q[(idx + buf_size) * 2]     = i_in2;
		q[(idx + buf_size) * 2 + 1] = q_in2;
		idx = (idx + 1) & DEC_HB33_BUF_MASK;

		/* Store sample 3 */
		q[idx * 2]     = i_in3;
		q[idx * 2 + 1] = q_in3;
		q[(idx + buf_size) * 2]     = i_in3;
		q[(idx + buf_size) * 2 + 1] = q_in3;
		idx1 = idx;
		idx = (idx + 1) & DEC_HB33_BUF_MASK;

		/*
		 * FIR for output 0 (after samples 0,1)
		 * 33-tap: 8 symmetric pairs, s[0]..s[64] (interleaved I/Q)
		 * Center at s[32],s[33] (I,Q)
		 */
		s = &q[(idx0 + buf_size - DEC_HB33_TAPS + 1) * 2];
		acc_i0 = HB_KERNEL_FLOAT_33_FAST[1]  * (s[2]  + s[62])
		       + HB_KERNEL_FLOAT_33_FAST[3]  * (s[6]  + s[58])
		       + HB_KERNEL_FLOAT_33_FAST[5]  * (s[10] + s[54])
		       + HB_KERNEL_FLOAT_33_FAST[7]  * (s[14] + s[50])
		       + HB_KERNEL_FLOAT_33_FAST[9]  * (s[18] + s[46])
		       + HB_KERNEL_FLOAT_33_FAST[11] * (s[22] + s[42])
		       + HB_KERNEL_FLOAT_33_FAST[13] * (s[26] + s[38])
		       + HB_KERNEL_FLOAT_33_FAST[15] * (s[30] + s[34])
		       + HB_KERNEL_FLOAT_33_FAST[16] * s[32];
		acc_q0 = HB_KERNEL_FLOAT_33_FAST[1]  * (s[3]  + s[63])
		       + HB_KERNEL_FLOAT_33_FAST[3]  * (s[7]  + s[59])
		       + HB_KERNEL_FLOAT_33_FAST[5]  * (s[11] + s[55])
		       + HB_KERNEL_FLOAT_33_FAST[7]  * (s[15] + s[51])
		       + HB_KERNEL_FLOAT_33_FAST[9]  * (s[19] + s[47])
		       + HB_KERNEL_FLOAT_33_FAST[11] * (s[23] + s[43])
		       + HB_KERNEL_FLOAT_33_FAST[13] * (s[27] + s[39])
		       + HB_KERNEL_FLOAT_33_FAST[15] * (s[31] + s[35])
		       + HB_KERNEL_FLOAT_33_FAST[16] * s[33];

		/* FIR for output 1 (after samples 2,3) */
		s = &q[(idx1 + buf_size - DEC_HB33_TAPS + 1) * 2];
		acc_i1 = HB_KERNEL_FLOAT_33_FAST[1]  * (s[2]  + s[62])
		       + HB_KERNEL_FLOAT_33_FAST[3]  * (s[6]  + s[58])
		       + HB_KERNEL_FLOAT_33_FAST[5]  * (s[10] + s[54])
		       + HB_KERNEL_FLOAT_33_FAST[7]  * (s[14] + s[50])
		       + HB_KERNEL_FLOAT_33_FAST[9]  * (s[18] + s[46])
		       + HB_KERNEL_FLOAT_33_FAST[11] * (s[22] + s[42])
		       + HB_KERNEL_FLOAT_33_FAST[13] * (s[26] + s[38])
		       + HB_KERNEL_FLOAT_33_FAST[15] * (s[30] + s[34])
		       + HB_KERNEL_FLOAT_33_FAST[16] * s[32];
		acc_q1 = HB_KERNEL_FLOAT_33_FAST[1]  * (s[3]  + s[63])
		       + HB_KERNEL_FLOAT_33_FAST[3]  * (s[7]  + s[59])
		       + HB_KERNEL_FLOAT_33_FAST[5]  * (s[11] + s[55])
		       + HB_KERNEL_FLOAT_33_FAST[7]  * (s[15] + s[51])
		       + HB_KERNEL_FLOAT_33_FAST[9]  * (s[19] + s[47])
		       + HB_KERNEL_FLOAT_33_FAST[11] * (s[23] + s[43])
		       + HB_KERNEL_FLOAT_33_FAST[13] * (s[27] + s[39])
		       + HB_KERNEL_FLOAT_33_FAST[15] * (s[31] + s[35])
		       + HB_KERNEL_FLOAT_33_FAST[16] * s[33];

		/* Store 2 outputs */
		dest[out_idx * 2 + 0] = acc_i0;
		dest[out_idx * 2 + 1] = acc_q0;
		dest[out_idx * 2 + 2] = acc_i1;
		dest[out_idx * 2 + 3] = acc_q1;
		out_idx += 2;
	}

	stage->fir_index = idx;
}

/* ========================================================================
 * OPTIMIZED INT16 33-TAP PROCESSING
 *
 * Uses HB_KERNEL_INT16_33_FAST directly from filters_opt.h.
 *
 * Fixed-point arithmetic:
 *   - Coefficients: Q15 format (scaled by 32768, center tap = 16384)
 *   - Output shift: >> 14 (intentional 2x gain for SNR preservation)
 *   - Each 2x decimation provides +3 dB SNR = +0.5 effective bits
 *   - The >> 14 shift converts this SNR gain into amplitude
 *   - With 64x decimation: +3 bits effective resolution gained
 *
 * Overflow considerations:
 *   - Int32 accumulator: safe (worst-case = 1.66B, max = 2.15B)
 *   - Int16 output: SATURATION enabled via SAT_I16() macro
 *   - Strong signals clip to +/-32767 instead of wrapping
 *
 * Optimizations:
 *   - int16_t buffer (halves memory bandwidth vs int32_t)
 *   - Pre-loaded coefficients as local int32_t
 *   - 8x unrolling (8 inputs -> 4 outputs per iteration)
 * ======================================================================== */

OPT_HOT
void dec_hb33_int16_process(dec_stage_int16_t *stage,
			    const int16_t *src, int16_t *dest, int len)
{
	int i, out_idx;
	int idx;
	int16_t *q;
	const int buf_size = DEC_HB33_BUF_SIZE;

	/* Pre-load coefficients as int32_t to avoid repeated casts */
	const int32_t h1  = (int32_t)HB_KERNEL_INT16_33_FAST[1];
	const int32_t h3  = (int32_t)HB_KERNEL_INT16_33_FAST[3];
	const int32_t h5  = (int32_t)HB_KERNEL_INT16_33_FAST[5];
	const int32_t h7  = (int32_t)HB_KERNEL_INT16_33_FAST[7];
	const int32_t h9  = (int32_t)HB_KERNEL_INT16_33_FAST[9];
	const int32_t h11 = (int32_t)HB_KERNEL_INT16_33_FAST[11];
	const int32_t h13 = (int32_t)HB_KERNEL_INT16_33_FAST[13];
	const int32_t h15 = (int32_t)HB_KERNEL_INT16_33_FAST[15];
	const int32_t h16 = (int32_t)HB_KERNEL_INT16_33_FAST[16];

	if (OPT_UNLIKELY(!src || !dest || len < 8))
		return;

	q = stage->queue_iq;
	idx = stage->fir_index;
	out_idx = 0;

	OPT_PREFETCH_READ(&src[0]);

	/* Process 8 input samples -> 4 output samples per iteration */
	for (i = 0; i < len; i += 8) {
		const int16_t *s;
		int idx0, idx1, idx2, idx3;
		int32_t acc_i, acc_q;
		int16_t i_val, q_val;

		/* Prefetch next batch */
		if (OPT_LIKELY(i + 16 < len)) {
			OPT_PREFETCH_READ(&src[(i + 16) * 2]);
		}

		/* Store 8 samples - fully unrolled for performance */
		/* Sample 0 */
		i_val = src[i * 2 + 0]; q_val = src[i * 2 + 1];
		q[idx * 2] = i_val; q[idx * 2 + 1] = q_val;
		q[(idx + buf_size) * 2] = i_val; q[(idx + buf_size) * 2 + 1] = q_val;
		idx = (idx + 1) & DEC_HB33_BUF_MASK;

		/* Sample 1 -> idx0 */
		i_val = src[i * 2 + 2]; q_val = src[i * 2 + 3];
		q[idx * 2] = i_val; q[idx * 2 + 1] = q_val;
		q[(idx + buf_size) * 2] = i_val; q[(idx + buf_size) * 2 + 1] = q_val;
		idx0 = idx;
		idx = (idx + 1) & DEC_HB33_BUF_MASK;

		/* Sample 2 */
		i_val = src[i * 2 + 4]; q_val = src[i * 2 + 5];
		q[idx * 2] = i_val; q[idx * 2 + 1] = q_val;
		q[(idx + buf_size) * 2] = i_val; q[(idx + buf_size) * 2 + 1] = q_val;
		idx = (idx + 1) & DEC_HB33_BUF_MASK;

		/* Sample 3 -> idx1 */
		i_val = src[i * 2 + 6]; q_val = src[i * 2 + 7];
		q[idx * 2] = i_val; q[idx * 2 + 1] = q_val;
		q[(idx + buf_size) * 2] = i_val; q[(idx + buf_size) * 2 + 1] = q_val;
		idx1 = idx;
		idx = (idx + 1) & DEC_HB33_BUF_MASK;

		/* Sample 4 */
		i_val = src[i * 2 + 8]; q_val = src[i * 2 + 9];
		q[idx * 2] = i_val; q[idx * 2 + 1] = q_val;
		q[(idx + buf_size) * 2] = i_val; q[(idx + buf_size) * 2 + 1] = q_val;
		idx = (idx + 1) & DEC_HB33_BUF_MASK;

		/* Sample 5 -> idx2 */
		i_val = src[i * 2 + 10]; q_val = src[i * 2 + 11];
		q[idx * 2] = i_val; q[idx * 2 + 1] = q_val;
		q[(idx + buf_size) * 2] = i_val; q[(idx + buf_size) * 2 + 1] = q_val;
		idx2 = idx;
		idx = (idx + 1) & DEC_HB33_BUF_MASK;

		/* Sample 6 */
		i_val = src[i * 2 + 12]; q_val = src[i * 2 + 13];
		q[idx * 2] = i_val; q[idx * 2 + 1] = q_val;
		q[(idx + buf_size) * 2] = i_val; q[(idx + buf_size) * 2 + 1] = q_val;
		idx = (idx + 1) & DEC_HB33_BUF_MASK;

		/* Sample 7 -> idx3 */
		i_val = src[i * 2 + 14]; q_val = src[i * 2 + 15];
		q[idx * 2] = i_val; q[idx * 2 + 1] = q_val;
		q[(idx + buf_size) * 2] = i_val; q[(idx + buf_size) * 2 + 1] = q_val;
		idx3 = idx;
		idx = (idx + 1) & DEC_HB33_BUF_MASK;

		/*
		 * FIR for 4 outputs using pre-loaded coefficients.
		 * 33-tap halfband: 8 symmetric pairs + center (0.5)
		 * Interleaved I/Q: even indices = I, odd = Q
		 */

		/* Output 0 */
		s = &q[(idx0 + buf_size - DEC_HB33_TAPS + 1) * 2];
		acc_i = h1  * ((int32_t)s[2]  + s[62])
		      + h3  * ((int32_t)s[6]  + s[58])
		      + h5  * ((int32_t)s[10] + s[54])
		      + h7  * ((int32_t)s[14] + s[50])
		      + h9  * ((int32_t)s[18] + s[46])
		      + h11 * ((int32_t)s[22] + s[42])
		      + h13 * ((int32_t)s[26] + s[38])
		      + h15 * ((int32_t)s[30] + s[34])
		      + h16 * (int32_t)s[32];
		acc_q = h1  * ((int32_t)s[3]  + s[63])
		      + h3  * ((int32_t)s[7]  + s[59])
		      + h5  * ((int32_t)s[11] + s[55])
		      + h7  * ((int32_t)s[15] + s[51])
		      + h9  * ((int32_t)s[19] + s[47])
		      + h11 * ((int32_t)s[23] + s[43])
		      + h13 * ((int32_t)s[27] + s[39])
		      + h15 * ((int32_t)s[31] + s[35])
		      + h16 * (int32_t)s[33];
		dest[out_idx * 2 + 0] = SAT_I16(acc_i >> 14);
		dest[out_idx * 2 + 1] = SAT_I16(acc_q >> 14);

		/* Output 1 */
		s = &q[(idx1 + buf_size - DEC_HB33_TAPS + 1) * 2];
		acc_i = h1  * ((int32_t)s[2]  + s[62])
		      + h3  * ((int32_t)s[6]  + s[58])
		      + h5  * ((int32_t)s[10] + s[54])
		      + h7  * ((int32_t)s[14] + s[50])
		      + h9  * ((int32_t)s[18] + s[46])
		      + h11 * ((int32_t)s[22] + s[42])
		      + h13 * ((int32_t)s[26] + s[38])
		      + h15 * ((int32_t)s[30] + s[34])
		      + h16 * (int32_t)s[32];
		acc_q = h1  * ((int32_t)s[3]  + s[63])
		      + h3  * ((int32_t)s[7]  + s[59])
		      + h5  * ((int32_t)s[11] + s[55])
		      + h7  * ((int32_t)s[15] + s[51])
		      + h9  * ((int32_t)s[19] + s[47])
		      + h11 * ((int32_t)s[23] + s[43])
		      + h13 * ((int32_t)s[27] + s[39])
		      + h15 * ((int32_t)s[31] + s[35])
		      + h16 * (int32_t)s[33];
		dest[out_idx * 2 + 2] = SAT_I16(acc_i >> 14);
		dest[out_idx * 2 + 3] = SAT_I16(acc_q >> 14);

		/* Output 2 */
		s = &q[(idx2 + buf_size - DEC_HB33_TAPS + 1) * 2];
		acc_i = h1  * ((int32_t)s[2]  + s[62])
		      + h3  * ((int32_t)s[6]  + s[58])
		      + h5  * ((int32_t)s[10] + s[54])
		      + h7  * ((int32_t)s[14] + s[50])
		      + h9  * ((int32_t)s[18] + s[46])
		      + h11 * ((int32_t)s[22] + s[42])
		      + h13 * ((int32_t)s[26] + s[38])
		      + h15 * ((int32_t)s[30] + s[34])
		      + h16 * (int32_t)s[32];
		acc_q = h1  * ((int32_t)s[3]  + s[63])
		      + h3  * ((int32_t)s[7]  + s[59])
		      + h5  * ((int32_t)s[11] + s[55])
		      + h7  * ((int32_t)s[15] + s[51])
		      + h9  * ((int32_t)s[19] + s[47])
		      + h11 * ((int32_t)s[23] + s[43])
		      + h13 * ((int32_t)s[27] + s[39])
		      + h15 * ((int32_t)s[31] + s[35])
		      + h16 * (int32_t)s[33];
		dest[out_idx * 2 + 4] = SAT_I16(acc_i >> 14);
		dest[out_idx * 2 + 5] = SAT_I16(acc_q >> 14);

		/* Output 3 */
		s = &q[(idx3 + buf_size - DEC_HB33_TAPS + 1) * 2];
		acc_i = h1  * ((int32_t)s[2]  + s[62])
		      + h3  * ((int32_t)s[6]  + s[58])
		      + h5  * ((int32_t)s[10] + s[54])
		      + h7  * ((int32_t)s[14] + s[50])
		      + h9  * ((int32_t)s[18] + s[46])
		      + h11 * ((int32_t)s[22] + s[42])
		      + h13 * ((int32_t)s[26] + s[38])
		      + h15 * ((int32_t)s[30] + s[34])
		      + h16 * (int32_t)s[32];
		acc_q = h1  * ((int32_t)s[3]  + s[63])
		      + h3  * ((int32_t)s[7]  + s[59])
		      + h5  * ((int32_t)s[11] + s[55])
		      + h7  * ((int32_t)s[15] + s[51])
		      + h9  * ((int32_t)s[19] + s[47])
		      + h11 * ((int32_t)s[23] + s[43])
		      + h13 * ((int32_t)s[27] + s[39])
		      + h15 * ((int32_t)s[31] + s[35])
		      + h16 * (int32_t)s[33];
		dest[out_idx * 2 + 6] = SAT_I16(acc_i >> 14);
		dest[out_idx * 2 + 7] = SAT_I16(acc_q >> 14);

		out_idx += 4;
	}

	stage->fir_index = idx;
}

/* ========================================================================
 * OPTIMIZED FLOAT32 17-TAP PROCESSING
 *
 * Uses HB_KERNEL_FLOAT_17_FAST directly from filters_opt.h.
 * 17-tap halfband: 4 symmetric pairs + center (0.5)
 * Non-zero coefficients at indices: 1,3,5,7,8,9,11,13,15
 *
 * Gain: Unity (~0.9989) - see 33-tap float32 comments for details.
 * ======================================================================== */

OPT_HOT
void dec_hb17_float_process(dec_stage_float_t *stage,
			    const float *src, float *dest, int len)
{
	int i, out_idx;
	int idx;
	float *q;
	const int buf_size = DEC_HB17_BUF_SIZE;

	if (OPT_UNLIKELY(!src || !dest || len < 4))
		return;

	q = stage->queue_iq;
	idx = stage->fir_index;
	out_idx = 0;

	OPT_PREFETCH_READ(&src[0]);

	/* Process 4 input samples -> 2 output samples per iteration */
	for (i = 0; i < len; i += 4) {
		float i_in0, q_in0, i_in1, q_in1, i_in2, q_in2, i_in3, q_in3;
		float acc_i0, acc_q0, acc_i1, acc_q1;
		const float *s;
		int idx0, idx1;

		/* Load 4 input I/Q pairs */
		i_in0 = src[i * 2 + 0];
		q_in0 = src[i * 2 + 1];
		i_in1 = src[i * 2 + 2];
		q_in1 = src[i * 2 + 3];
		i_in2 = src[i * 2 + 4];
		q_in2 = src[i * 2 + 5];
		i_in3 = src[i * 2 + 6];
		q_in3 = src[i * 2 + 7];

		/* Prefetch next batch */
		if (OPT_LIKELY(i + 8 < len)) {
			OPT_PREFETCH_READ(&src[(i + 8) * 2]);
		}

		/* Store sample 0 (mirrored interleaved) */
		q[idx * 2]     = i_in0;
		q[idx * 2 + 1] = q_in0;
		q[(idx + buf_size) * 2]     = i_in0;
		q[(idx + buf_size) * 2 + 1] = q_in0;
		idx = (idx + 1) & DEC_HB17_BUF_MASK;

		/* Store sample 1 */
		q[idx * 2]     = i_in1;
		q[idx * 2 + 1] = q_in1;
		q[(idx + buf_size) * 2]     = i_in1;
		q[(idx + buf_size) * 2 + 1] = q_in1;
		idx0 = idx;
		idx = (idx + 1) & DEC_HB17_BUF_MASK;

		/* Store sample 2 */
		q[idx * 2]     = i_in2;
		q[idx * 2 + 1] = q_in2;
		q[(idx + buf_size) * 2]     = i_in2;
		q[(idx + buf_size) * 2 + 1] = q_in2;
		idx = (idx + 1) & DEC_HB17_BUF_MASK;

		/* Store sample 3 */
		q[idx * 2]     = i_in3;
		q[idx * 2 + 1] = q_in3;
		q[(idx + buf_size) * 2]     = i_in3;
		q[(idx + buf_size) * 2 + 1] = q_in3;
		idx1 = idx;
		idx = (idx + 1) & DEC_HB17_BUF_MASK;

		/*
		 * FIR for output 0 (after samples 0,1)
		 * 17-tap: 4 symmetric pairs, s[0]..s[32] (interleaved I/Q)
		 * Center at s[16],s[17] (I,Q)
		 */
		s = &q[(idx0 + buf_size - DEC_HB17_TAPS + 1) * 2];
		acc_i0 = HB_KERNEL_FLOAT_17_FAST[1]  * (s[2]  + s[30])
		       + HB_KERNEL_FLOAT_17_FAST[3]  * (s[6]  + s[26])
		       + HB_KERNEL_FLOAT_17_FAST[5]  * (s[10] + s[22])
		       + HB_KERNEL_FLOAT_17_FAST[7]  * (s[14] + s[18])
		       + HB_KERNEL_FLOAT_17_FAST[8]  * s[16];
		acc_q0 = HB_KERNEL_FLOAT_17_FAST[1]  * (s[3]  + s[31])
		       + HB_KERNEL_FLOAT_17_FAST[3]  * (s[7]  + s[27])
		       + HB_KERNEL_FLOAT_17_FAST[5]  * (s[11] + s[23])
		       + HB_KERNEL_FLOAT_17_FAST[7]  * (s[15] + s[19])
		       + HB_KERNEL_FLOAT_17_FAST[8]  * s[17];

		/* FIR for output 1 (after samples 2,3) */
		s = &q[(idx1 + buf_size - DEC_HB17_TAPS + 1) * 2];
		acc_i1 = HB_KERNEL_FLOAT_17_FAST[1]  * (s[2]  + s[30])
		       + HB_KERNEL_FLOAT_17_FAST[3]  * (s[6]  + s[26])
		       + HB_KERNEL_FLOAT_17_FAST[5]  * (s[10] + s[22])
		       + HB_KERNEL_FLOAT_17_FAST[7]  * (s[14] + s[18])
		       + HB_KERNEL_FLOAT_17_FAST[8]  * s[16];
		acc_q1 = HB_KERNEL_FLOAT_17_FAST[1]  * (s[3]  + s[31])
		       + HB_KERNEL_FLOAT_17_FAST[3]  * (s[7]  + s[27])
		       + HB_KERNEL_FLOAT_17_FAST[5]  * (s[11] + s[23])
		       + HB_KERNEL_FLOAT_17_FAST[7]  * (s[15] + s[19])
		       + HB_KERNEL_FLOAT_17_FAST[8]  * s[17];

		/* Store 2 outputs */
		dest[out_idx * 2 + 0] = acc_i0;
		dest[out_idx * 2 + 1] = acc_q0;
		dest[out_idx * 2 + 2] = acc_i1;
		dest[out_idx * 2 + 3] = acc_q1;
		out_idx += 2;
	}

	stage->fir_index = idx;
}

/* ========================================================================
 * OPTIMIZED INT16 17-TAP PROCESSING
 *
 * Uses HB_KERNEL_INT16_17_FAST directly from filters_opt.h.
 *
 * Fixed-point arithmetic:
 *   - Coefficients: Q15 format (scaled by 32768, center tap = 16384)
 *   - Output shift: >> 14 (intentional 2x gain for SNR preservation)
 *   - See 33-tap comments for detailed explanation
 * ======================================================================== */

OPT_HOT
void dec_hb17_int16_process(dec_stage_int16_t *stage,
			    const int16_t *src, int16_t *dest, int len)
{
	int i, out_idx;
	int idx;
	int16_t *q;
	const int buf_size = DEC_HB17_BUF_SIZE;

	/* Pre-load coefficients as int32_t to avoid repeated casts */
	const int32_t h1 = (int32_t)HB_KERNEL_INT16_17_FAST[1];
	const int32_t h3 = (int32_t)HB_KERNEL_INT16_17_FAST[3];
	const int32_t h5 = (int32_t)HB_KERNEL_INT16_17_FAST[5];
	const int32_t h7 = (int32_t)HB_KERNEL_INT16_17_FAST[7];
	const int32_t h8 = (int32_t)HB_KERNEL_INT16_17_FAST[8];

	if (OPT_UNLIKELY(!src || !dest || len < 4))
		return;

	q = stage->queue_iq;
	idx = stage->fir_index;
	out_idx = 0;

	OPT_PREFETCH_READ(&src[0]);

	/* Process 4 input samples -> 2 output samples per iteration */
	for (i = 0; i < len; i += 4) {
		const int16_t *s;
		int idx0, idx1;
		int32_t acc_i, acc_q;
		int16_t i_val, q_val;

		/* Prefetch next batch */
		if (OPT_LIKELY(i + 8 < len)) {
			OPT_PREFETCH_READ(&src[(i + 8) * 2]);
		}

		/* Store sample 0 */
		i_val = src[i * 2 + 0]; q_val = src[i * 2 + 1];
		q[idx * 2] = i_val; q[idx * 2 + 1] = q_val;
		q[(idx + buf_size) * 2] = i_val; q[(idx + buf_size) * 2 + 1] = q_val;
		idx = (idx + 1) & DEC_HB17_BUF_MASK;

		/* Store sample 1 -> idx0 */
		i_val = src[i * 2 + 2]; q_val = src[i * 2 + 3];
		q[idx * 2] = i_val; q[idx * 2 + 1] = q_val;
		q[(idx + buf_size) * 2] = i_val; q[(idx + buf_size) * 2 + 1] = q_val;
		idx0 = idx;
		idx = (idx + 1) & DEC_HB17_BUF_MASK;

		/* Store sample 2 */
		i_val = src[i * 2 + 4]; q_val = src[i * 2 + 5];
		q[idx * 2] = i_val; q[idx * 2 + 1] = q_val;
		q[(idx + buf_size) * 2] = i_val; q[(idx + buf_size) * 2 + 1] = q_val;
		idx = (idx + 1) & DEC_HB17_BUF_MASK;

		/* Store sample 3 -> idx1 */
		i_val = src[i * 2 + 6]; q_val = src[i * 2 + 7];
		q[idx * 2] = i_val; q[idx * 2 + 1] = q_val;
		q[(idx + buf_size) * 2] = i_val; q[(idx + buf_size) * 2 + 1] = q_val;
		idx1 = idx;
		idx = (idx + 1) & DEC_HB17_BUF_MASK;

		/* FIR for output 0 - 17-tap: 4 symmetric pairs + center */
		s = &q[(idx0 + buf_size - DEC_HB17_TAPS + 1) * 2];
		acc_i = h1 * ((int32_t)s[2]  + s[30])
		      + h3 * ((int32_t)s[6]  + s[26])
		      + h5 * ((int32_t)s[10] + s[22])
		      + h7 * ((int32_t)s[14] + s[18])
		      + h8 * (int32_t)s[16];
		acc_q = h1 * ((int32_t)s[3]  + s[31])
		      + h3 * ((int32_t)s[7]  + s[27])
		      + h5 * ((int32_t)s[11] + s[23])
		      + h7 * ((int32_t)s[15] + s[19])
		      + h8 * (int32_t)s[17];
		dest[out_idx * 2 + 0] = SAT_I16(acc_i >> 14);
		dest[out_idx * 2 + 1] = SAT_I16(acc_q >> 14);

		/* FIR for output 1 */
		s = &q[(idx1 + buf_size - DEC_HB17_TAPS + 1) * 2];
		acc_i = h1 * ((int32_t)s[2]  + s[30])
		      + h3 * ((int32_t)s[6]  + s[26])
		      + h5 * ((int32_t)s[10] + s[22])
		      + h7 * ((int32_t)s[14] + s[18])
		      + h8 * (int32_t)s[16];
		acc_q = h1 * ((int32_t)s[3]  + s[31])
		      + h3 * ((int32_t)s[7]  + s[27])
		      + h5 * ((int32_t)s[11] + s[23])
		      + h7 * ((int32_t)s[15] + s[19])
		      + h8 * (int32_t)s[17];
		dest[out_idx * 2 + 2] = SAT_I16(acc_i >> 14);
		dest[out_idx * 2 + 3] = SAT_I16(acc_q >> 14);

		out_idx += 2;
	}

	stage->fir_index = idx;
}

/* ========================================================================
 * DISPATCH FUNCTIONS
 *
 * Automatically select 33-tap or 17-tap based on stage->filter_type.
 * ======================================================================== */

void dec_stage_float_process(dec_stage_float_t *stage,
			     const float *src, float *dest, int len)
{
	if (OPT_UNLIKELY(!stage))
		return;

	if (stage->filter_type == DEC_FILTER_17TAP)
		dec_hb17_float_process(stage, src, dest, len);
	else
		dec_hb33_float_process(stage, src, dest, len);
}

void dec_stage_int16_process(dec_stage_int16_t *stage,
			     const int16_t *src, int16_t *dest, int len)
{
	if (OPT_UNLIKELY(!stage))
		return;

	if (stage->filter_type == DEC_FILTER_17TAP)
		dec_hb17_int16_process(stage, src, dest, len);
	else
		dec_hb33_int16_process(stage, src, dest, len);
}
