/*
 * OPTIMIZED INT16 DDC IMPLEMENTATION
 *
 * Based on commit https://github.com/airspy/airspyone_host/commit/091c6f7449bc976f97bb327242532bfb1001d116
 *
 * Original work Copyright (C) 2014, Youssef Touil <youssef@airspy.com>
 * Modifications Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "iqconverter_int16_opt.h"
#include "iqconverter_common.h"
#include "iqconverter_lut.h"
#include <stdlib.h>
#include <string.h>

/* 47-tap filter: 12 symmetric pairs */
#define SYM_TAPS 12

iqconverter_int16_opt_t *iqconverter_int16_opt_create(const int16_t *hb_kernel,
						      int len)
{
	iqconverter_int16_opt_t *cnv;
	int i;

	if (OPT_UNLIKELY(len != 47)) return NULL;

	cnv = (iqconverter_int16_opt_t *)iqconv_aligned_alloc(sizeof(iqconverter_int16_opt_t));
	if (OPT_UNLIKELY(!cnv)) return NULL;

	memset(cnv, 0, sizeof(iqconverter_int16_opt_t));

	for (i = 0; i < SYM_TAPS; i++)
		cnv->sym_kernel[i] = (int32_t)hb_kernel[i * 2];

	cnv->dly_len = (len / 2 + 1) / 2;
	return cnv;
}

void iqconverter_int16_opt_free(iqconverter_int16_opt_t *cnv)
{
	if (OPT_LIKELY(cnv)) iqconv_aligned_free(cnv);
}

void iqconverter_int16_opt_reset(iqconverter_int16_opt_t *cnv)
{
	if (OPT_UNLIKELY(!cnv)) return;
	cnv->fir_head = 0;
	cnv->dly_head = 0;
	cnv->old_x = 0;
	cnv->old_y = 0;
	cnv->old_e = 0;
	memset(cnv->fir_buf, 0, sizeof(cnv->fir_buf));
	memset(cnv->dly_buf, 0, sizeof(cnv->dly_buf));
}

/*
 * FUSED PROCESSING: uint16_t ADC samples -> int16 I/Q output
 * Uses precomputed LUT for ultra-fast ADC -> int16 conversion.
 */
OPT_HOT
void iqconverter_int16_opt_process_u16(iqconverter_int16_opt_t * OPT_RESTRICT cnv,
				       const uint16_t * OPT_RESTRICT src,
				       int16_t * OPT_RESTRICT dest, int len)
{
	if (OPT_UNLIKELY(!cnv || !src || !dest || (len & 3)))
		return;

	int i;
	int16_t ox = cnv->old_x;
	int16_t oy = cnv->old_y;
	int32_t oe = cnv->old_e;
	int32_t f_head = cnv->fir_head;
	int32_t d_head = cnv->dly_head;

	/* Apply alignment hints for better vectorization */
	int32_t * OPT_RESTRICT fir_buf = (int32_t *)OPT_ALIGNED_64(cnv->fir_buf);
	const int32_t * OPT_RESTRICT kernel = (const int32_t *)OPT_ALIGNED_64(cnv->sym_kernel);
	int16_t * OPT_RESTRICT dly_buf = cnv->dly_buf;
	const int16_t * OPT_RESTRICT lut = (const int16_t *)OPT_ALIGNED_64(iqconv_lut.lut_int16);

	/* Prefetch first batch of LUT entries */
	OPT_PREFETCH_READ(&lut[src[0]]);
	OPT_PREFETCH_READ(&lut[src[1]]);

	for (i = 0; i < len; i += 4) {
		int16_t x0, x1, x2, x3;
		int16_t y0, y1, y2, y3;
		int32_t *fir_src;
		int32_t acc0, acc1;
		int16_t q_in0, q_in1;
		int16_t q_out0, q_out1;
		int16_t w, s;
		int32_t u;

		/* Prefetch next batch of LUT entries for next iteration */
		if (OPT_LIKELY(i + 8 < len)) {
			OPT_PREFETCH_READ(&lut[src[i + 4]]);
			OPT_PREFETCH_READ(&lut[src[i + 5]]);
			OPT_PREFETCH_READ(&lut[src[i + 6]]);
			OPT_PREFETCH_READ(&lut[src[i + 7]]);
		}

		/* FUSED: LUT-based uint16_t -> int16 + DC Removal */
		x0 = lut[src[i + 0]];
		w = x0 - ox;
		u = oe + (int32_t)oy * DC_ALPHA_Q15;
		s = (int16_t)(u >> 15);
		y0 = w + s;
		oe = u - ((int32_t)s << 15);
		ox = x0; oy = y0;

		x1 = lut[src[i + 1]];
		w = x1 - ox;
		u = oe + (int32_t)oy * DC_ALPHA_Q15;
		s = (int16_t)(u >> 15);
		y1 = w + s;
		oe = u - ((int32_t)s << 15);
		ox = x1; oy = y1;

		x2 = lut[src[i + 2]];
		w = x2 - ox;
		u = oe + (int32_t)oy * DC_ALPHA_Q15;
		s = (int16_t)(u >> 15);
		y2 = w + s;
		oe = u - ((int32_t)s << 15);
		ox = x2; oy = y2;

		x3 = lut[src[i + 3]];
		w = x3 - ox;
		u = oe + (int32_t)oy * DC_ALPHA_Q15;
		s = (int16_t)(u >> 15);
		y3 = w + s;
		oe = u - ((int32_t)s << 15);
		ox = x3; oy = y3;

		/* FIR I-Channel (Filter 0) - Symmetric FIR with coefficient folding */
		f_head = (f_head - 1) & FIR_BUF_MASK;
		fir_buf[f_head] = (int32_t)((int16_t)(-y0));
		fir_buf[f_head + FIR_BUF_SIZE] = fir_buf[f_head];

		fir_src = &fir_buf[f_head];
		acc0 = kernel[0]  * (fir_src[0]  + fir_src[23])
		     + kernel[1]  * (fir_src[1]  + fir_src[22])
		     + kernel[2]  * (fir_src[2]  + fir_src[21])
		     + kernel[3]  * (fir_src[3]  + fir_src[20])
		     + kernel[4]  * (fir_src[4]  + fir_src[19])
		     + kernel[5]  * (fir_src[5]  + fir_src[18])
		     + kernel[6]  * (fir_src[6]  + fir_src[17])
		     + kernel[7]  * (fir_src[7]  + fir_src[16])
		     + kernel[8]  * (fir_src[8]  + fir_src[15])
		     + kernel[9]  * (fir_src[9]  + fir_src[14])
		     + kernel[10] * (fir_src[10] + fir_src[13])
		     + kernel[11] * (fir_src[11] + fir_src[12]);

		/* FIR I-Channel (Filter 1) - Symmetric FIR with coefficient folding */
		f_head = (f_head - 1) & FIR_BUF_MASK;
		fir_buf[f_head] = (int32_t)y2;
		fir_buf[f_head + FIR_BUF_SIZE] = fir_buf[f_head];

		fir_src = &fir_buf[f_head];
		acc1 = kernel[0]  * (fir_src[0]  + fir_src[23])
		     + kernel[1]  * (fir_src[1]  + fir_src[22])
		     + kernel[2]  * (fir_src[2]  + fir_src[21])
		     + kernel[3]  * (fir_src[3]  + fir_src[20])
		     + kernel[4]  * (fir_src[4]  + fir_src[19])
		     + kernel[5]  * (fir_src[5]  + fir_src[18])
		     + kernel[6]  * (fir_src[6]  + fir_src[17])
		     + kernel[7]  * (fir_src[7]  + fir_src[16])
		     + kernel[8]  * (fir_src[8]  + fir_src[15])
		     + kernel[9]  * (fir_src[9]  + fir_src[14])
		     + kernel[10] * (fir_src[10] + fir_src[13])
		     + kernel[11] * (fir_src[11] + fir_src[12]);

		/* Mix & Delay Q-Channel */
		q_in0 = (int16_t)((-y1) >> 1);
		q_in1 = (int16_t)(y3 >> 1);

		q_out0 = dly_buf[d_head];
		dly_buf[d_head] = q_in0;
		if (OPT_UNLIKELY(++d_head >= 12)) d_head = 0;

		q_out1 = dly_buf[d_head];
		dly_buf[d_head] = q_in1;
		if (OPT_UNLIKELY(++d_head >= 12)) d_head = 0;

		/* Output */
		dest[i + 0] = (int16_t)(acc0 >> 15);
		dest[i + 1] = q_out0;
		dest[i + 2] = (int16_t)(acc1 >> 15);
		dest[i + 3] = q_out1;
	}

	cnv->old_x = ox; cnv->old_y = oy; cnv->old_e = oe;
	cnv->fir_head = f_head; cnv->dly_head = d_head;
}

/*
 * FUSED PROCESSING: uint8_t ADC samples (8-bit mode) -> int16 I/Q output
 * Uses precomputed 8-bit LUT (256 entries) for ultra-fast ADC -> int16 conversion.
 */
OPT_HOT
void iqconverter_int16_opt_process_u8(iqconverter_int16_opt_t * OPT_RESTRICT cnv,
				      const uint8_t * OPT_RESTRICT src,
				      int16_t * OPT_RESTRICT dest, int len)
{
	if (OPT_UNLIKELY(!cnv || !src || !dest || (len & 3)))
		return;

	/* Validate LUT is initialized for 8-bit ADC */
	if (OPT_UNLIKELY(iqconv_lut.adc_bits != IQCONV_ADC_8BIT))
		return;

	int i;
	int16_t ox = cnv->old_x;
	int16_t oy = cnv->old_y;
	int32_t oe = cnv->old_e;
	int32_t f_head = cnv->fir_head;
	int32_t d_head = cnv->dly_head;

	int32_t * OPT_RESTRICT fir_buf = (int32_t *)OPT_ALIGNED_64(cnv->fir_buf);
	const int32_t * OPT_RESTRICT kernel = (const int32_t *)OPT_ALIGNED_64(cnv->sym_kernel);
	int16_t * OPT_RESTRICT dly_buf = cnv->dly_buf;
	const int16_t * OPT_RESTRICT lut = (const int16_t *)OPT_ALIGNED_64(iqconv_lut.lut_int16);

	for (i = 0; i < len; i += 4) {
		int16_t x0, x1, x2, x3;
		int16_t y0, y1, y2, y3;
		int32_t *fir_src;
		int32_t acc0, acc1;
		int16_t q_in0, q_in1;
		int16_t q_out0, q_out1;
		int16_t w, s;
		int32_t u;

		/* FUSED: LUT-based uint8_t -> int16 + DC Removal */
		x0 = lut[src[i + 0]];
		w = x0 - ox;
		u = oe + (int32_t)oy * DC_ALPHA_Q15;
		s = (int16_t)(u >> 15);
		y0 = w + s;
		oe = u - ((int32_t)s << 15);
		ox = x0; oy = y0;

		x1 = lut[src[i + 1]];
		w = x1 - ox;
		u = oe + (int32_t)oy * DC_ALPHA_Q15;
		s = (int16_t)(u >> 15);
		y1 = w + s;
		oe = u - ((int32_t)s << 15);
		ox = x1; oy = y1;

		x2 = lut[src[i + 2]];
		w = x2 - ox;
		u = oe + (int32_t)oy * DC_ALPHA_Q15;
		s = (int16_t)(u >> 15);
		y2 = w + s;
		oe = u - ((int32_t)s << 15);
		ox = x2; oy = y2;

		x3 = lut[src[i + 3]];
		w = x3 - ox;
		u = oe + (int32_t)oy * DC_ALPHA_Q15;
		s = (int16_t)(u >> 15);
		y3 = w + s;
		oe = u - ((int32_t)s << 15);
		ox = x3; oy = y3;

		/* FIR I-Channel (Filter 0) */
		f_head = (f_head - 1) & FIR_BUF_MASK;
		fir_buf[f_head] = (int32_t)((int16_t)(-y0));
		fir_buf[f_head + FIR_BUF_SIZE] = fir_buf[f_head];

		fir_src = &fir_buf[f_head];
		acc0 = kernel[0]  * (fir_src[0]  + fir_src[23])
		     + kernel[1]  * (fir_src[1]  + fir_src[22])
		     + kernel[2]  * (fir_src[2]  + fir_src[21])
		     + kernel[3]  * (fir_src[3]  + fir_src[20])
		     + kernel[4]  * (fir_src[4]  + fir_src[19])
		     + kernel[5]  * (fir_src[5]  + fir_src[18])
		     + kernel[6]  * (fir_src[6]  + fir_src[17])
		     + kernel[7]  * (fir_src[7]  + fir_src[16])
		     + kernel[8]  * (fir_src[8]  + fir_src[15])
		     + kernel[9]  * (fir_src[9]  + fir_src[14])
		     + kernel[10] * (fir_src[10] + fir_src[13])
		     + kernel[11] * (fir_src[11] + fir_src[12]);

		/* FIR I-Channel (Filter 1) */
		f_head = (f_head - 1) & FIR_BUF_MASK;
		fir_buf[f_head] = (int32_t)y2;
		fir_buf[f_head + FIR_BUF_SIZE] = fir_buf[f_head];

		fir_src = &fir_buf[f_head];
		acc1 = kernel[0]  * (fir_src[0]  + fir_src[23])
		     + kernel[1]  * (fir_src[1]  + fir_src[22])
		     + kernel[2]  * (fir_src[2]  + fir_src[21])
		     + kernel[3]  * (fir_src[3]  + fir_src[20])
		     + kernel[4]  * (fir_src[4]  + fir_src[19])
		     + kernel[5]  * (fir_src[5]  + fir_src[18])
		     + kernel[6]  * (fir_src[6]  + fir_src[17])
		     + kernel[7]  * (fir_src[7]  + fir_src[16])
		     + kernel[8]  * (fir_src[8]  + fir_src[15])
		     + kernel[9]  * (fir_src[9]  + fir_src[14])
		     + kernel[10] * (fir_src[10] + fir_src[13])
		     + kernel[11] * (fir_src[11] + fir_src[12]);

		/* Mix & Delay Q-Channel */
		q_in0 = (int16_t)((-y1) >> 1);
		q_in1 = (int16_t)(y3 >> 1);

		q_out0 = dly_buf[d_head];
		dly_buf[d_head] = q_in0;
		if (OPT_UNLIKELY(++d_head >= 12)) d_head = 0;

		q_out1 = dly_buf[d_head];
		dly_buf[d_head] = q_in1;
		if (OPT_UNLIKELY(++d_head >= 12)) d_head = 0;

		/* Output */
		dest[i + 0] = (int16_t)(acc0 >> 15);
		dest[i + 1] = q_out0;
		dest[i + 2] = (int16_t)(acc1 >> 15);
		dest[i + 3] = q_out1;
	}

	cnv->old_x = ox; cnv->old_y = oy; cnv->old_e = oe;
	cnv->fir_head = f_head; cnv->dly_head = d_head;
}
