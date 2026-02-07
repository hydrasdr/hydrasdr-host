/*
 * FLOAT32 DDC WITH CASCADED DECIMATION - IMPLEMENTATION
 *
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "iqconverter_float_opt_dec.h"
#include "compat_opt.h"
#include <stdlib.h>
#include <string.h>

iqconverter_float_opt_dec_t *iqconverter_float_opt_dec_create(
	const float *hb_kernel, int len, int decimation)
{
	iqconverter_float_opt_dec_t *cnv;
	int num_stages;
	int i;

	/* Validate decimation factor */
	if (!dec_valid_factor(decimation))
		return NULL;

	num_stages = dec_get_num_stages(decimation);
	if (num_stages < 0)
		return NULL;

	/* Allocate main context */
	cnv = (iqconverter_float_opt_dec_t *)opt_aligned_alloc(
		sizeof(iqconverter_float_opt_dec_t), 64);
	if (!cnv)
		return NULL;

	memset(cnv, 0, sizeof(iqconverter_float_opt_dec_t));

	/* Create base DDC */
	cnv->base_ctx = iqconverter_float_opt_create(hb_kernel, len);
	if (!cnv->base_ctx) {
		opt_aligned_free(cnv);
		return NULL;
	}

	cnv->decimation_factor = decimation;
	cnv->num_dec_stages = num_stages;

	/* Initialize decimation stages */
	for (i = 0; i < num_stages; i++) {
		if (dec_stage_float_init(&cnv->stages[i], i) != 0) {
			/* Cleanup on failure */
			int j;
			for (j = 0; j < i; j++)
				dec_stage_float_free(&cnv->stages[j]);
			iqconverter_float_opt_free(cnv->base_ctx);
			opt_aligned_free(cnv);
			return NULL;
		}
	}

	/* Allocate intermediate buffer if needed */
	if (num_stages > 0) {
		cnv->inter_buf_len = DEC_DEFAULT_INTER_BUF_LEN;
		cnv->inter_buf = (float *)opt_aligned_alloc(
			cnv->inter_buf_len * 2 * sizeof(float), 64);
		if (!cnv->inter_buf) {
			for (i = 0; i < num_stages; i++)
				dec_stage_float_free(&cnv->stages[i]);
			iqconverter_float_opt_free(cnv->base_ctx);
			opt_aligned_free(cnv);
			return NULL;
		}
	}

	return cnv;
}

void iqconverter_float_opt_dec_free(iqconverter_float_opt_dec_t *cnv)
{
	int i;

	if (!cnv)
		return;

	/* Free decimation stages */
	for (i = 0; i < cnv->num_dec_stages; i++)
		dec_stage_float_free(&cnv->stages[i]);

	/* Free intermediate buffer */
	if (cnv->inter_buf)
		opt_aligned_free(cnv->inter_buf);

	/* Free base DDC */
	if (cnv->base_ctx)
		iqconverter_float_opt_free(cnv->base_ctx);

	opt_aligned_free(cnv);
}

void iqconverter_float_opt_dec_reset(iqconverter_float_opt_dec_t *cnv)
{
	int i;

	if (!cnv)
		return;

	/* Reset base DDC */
	if (cnv->base_ctx)
		iqconverter_float_opt_reset(cnv->base_ctx);

	/* Reset decimation stages */
	for (i = 0; i < cnv->num_dec_stages; i++)
		dec_stage_float_reset(&cnv->stages[i]);
}

/*
 * Process a single chunk through all decimation stages.
 * Keeps data in cache between stages for better performance.
 */
OPT_HOT
static void process_chunk_float(iqconverter_float_opt_dec_t *cnv,
				const uint16_t *src, float *dest,
				int chunk_len, int *out_offset)
{
	int stage_len;
	int i;
	float * OPT_RESTRICT in_buf;
	float * OPT_RESTRICT out_buf;
	float * OPT_RESTRICT inter = cnv->inter_buf;

	/* Stage 0: Base DDC to intermediate buffer */
	iqconverter_float_opt_process_u16(cnv->base_ctx, src, inter, chunk_len);
	/*
	 * Base DDC outputs chunk_len floats = chunk_len/2 I/Q pairs.
	 * Decimation stages work on I/Q pair counts.
	 */
	stage_len = chunk_len / 2;

	/*
	 * Cascaded decimation stages:
	 * - Stages 0-2: 33-tap halfband (8 MACs, ~62 dB rejection)
	 * - Stages 3+:  17-tap halfband (4 MACs, ~59 dB rejection)
	 * This achieves < 0.1 dB total passband ripple while reducing
	 * computation by ~33% for high decimation factors.
	 */
	for (i = 0; i < cnv->num_dec_stages; i++) {
		int is_last = (i == cnv->num_dec_stages - 1);

		in_buf = inter;
		out_buf = is_last ? (dest + *out_offset * 2) : inter;

		/* Dispatch to 33-tap or 17-tap based on stage filter_type */
		dec_stage_float_process(&cnv->stages[i],
					in_buf, out_buf, stage_len);

		stage_len /= 2;
	}

	/* Update output offset (in I/Q pairs) */
	*out_offset += (chunk_len / 2) / cnv->decimation_factor;
}

OPT_HOT
void iqconverter_float_opt_dec_process_u16(iqconverter_float_opt_dec_t *cnv,
					   const uint16_t *src,
					   float *dest, int len)
{
	int offset, out_offset;
	int chunk_len;

	if (OPT_UNLIKELY(!cnv || !cnv->base_ctx || !src || !dest))
		return;

	/* Validate input length is multiple of 4 * decimation */
	if (OPT_UNLIKELY((len & 3) || (len % cnv->decimation_factor)))
		return;

	/* No decimation: direct pass-through */
	if (cnv->num_dec_stages == 0) {
		iqconverter_float_opt_process_u16(cnv->base_ctx, src, dest, len);
		return;
	}

	/* Ensure intermediate buffer is large enough for one chunk */
	if (OPT_UNLIKELY(DEC_CHUNK_SIZE > cnv->inter_buf_len)) {
		float *new_buf = (float *)opt_aligned_alloc(
			DEC_CHUNK_SIZE * 2 * sizeof(float), 64);
		if (!new_buf)
			return;
		if (cnv->inter_buf)
			opt_aligned_free(cnv->inter_buf);
		cnv->inter_buf = new_buf;
		cnv->inter_buf_len = DEC_CHUNK_SIZE;
	}

	/*
	 * Process in cache-friendly chunks.
	 * Each chunk fits in L1/L2 cache, so data stays hot between stages.
	 */
	out_offset = 0;
	for (offset = 0; offset < len; offset += DEC_CHUNK_SIZE) {
		/* Calculate chunk size (handle final partial chunk) */
		chunk_len = len - offset;
		if (chunk_len > DEC_CHUNK_SIZE)
			chunk_len = DEC_CHUNK_SIZE;

		/* Align chunk to decimation factor */
		chunk_len = (chunk_len / cnv->decimation_factor) *
			    cnv->decimation_factor;
		if (chunk_len == 0)
			break;

		process_chunk_float(cnv, src + offset, dest,
				    chunk_len, &out_offset);
	}
}

int iqconverter_float_opt_dec_get_decimation(iqconverter_float_opt_dec_t *cnv)
{
	if (!cnv)
		return 0;
	return cnv->decimation_factor;
}

int iqconverter_float_opt_dec_get_output_len(iqconverter_float_opt_dec_t *cnv,
					     int input_len)
{
	if (!cnv || cnv->decimation_factor == 0)
		return 0;
	return input_len / cnv->decimation_factor;
}

int iqconverter_float_opt_dec_get_delay(iqconverter_float_opt_dec_t *cnv,
					int base_filter_len)
{
	int base_delay;
	int dec_delay;

	if (!cnv)
		return 0;

	/* Base DDC filter delay: (N-1)/2 for linear-phase FIR */
	base_delay = (base_filter_len - 1) / 2;

	/* Get decimation stage delays */
	dec_delay = dec_get_decimation_delay(cnv->decimation_factor);
	if (dec_delay < 0)
		dec_delay = 0;

	return base_delay + dec_delay;
}

float iqconverter_float_opt_dec_get_delay_output(iqconverter_float_opt_dec_t *cnv,
						 int base_filter_len)
{
	int total_delay;

	if (!cnv || cnv->decimation_factor == 0)
		return 0.0f;

	total_delay = iqconverter_float_opt_dec_get_delay(cnv, base_filter_len);
	return (float)total_delay / (float)cnv->decimation_factor;
}
