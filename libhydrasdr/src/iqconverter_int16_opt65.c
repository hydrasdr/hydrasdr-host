/*
 * OPTIMIZED INT16 DDC IMPLEMENTATION - 65-TAP VARIANT
 *
 * Based on commit https://github.com/airspy/airspyone_host/commit/091c6f7449bc976f97bb327242532bfb1001d116
 *
 * Original work Copyright (C) 2014, Youssef Touil <youssef@airspy.com>
 * Modifications Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "iqconverter_int16_opt65.h"
#include "iqconverter_common.h"
#include "iqconverter_lut.h"
#include <stdlib.h>
#include <string.h>

/* 65-tap filter: 16 symmetric pairs, power-of-2 delay for branchless modulo */
#define SYM_TAPS 16
#define DELAY_SIZE 16
#define DELAY_MASK 15

iqconverter_int16_opt65_t *iqconverter_int16_opt65_create(const int16_t *hb_kernel,
							  int len)
{
	iqconverter_int16_opt65_t *cnv;
	int i;

	if (len != 65) return NULL;

	cnv = (iqconverter_int16_opt65_t *)iqconv_aligned_alloc(sizeof(iqconverter_int16_opt65_t));
	if (!cnv) return NULL;

	memset(cnv, 0, sizeof(iqconverter_int16_opt65_t));

	/*
	 * Extract symmetric kernel coefficients (every other tap, first half)
	 * For 65-tap filter, non-zero coefficients are at ODD indices (1,3,5...)
	 */
	for (i = 0; i < SYM_TAPS; i++)
		cnv->sym_kernel[i] = (int32_t)hb_kernel[i * 2 + 1];

	cnv->dly_len = DELAY_SIZE;
	return cnv;
}

void iqconverter_int16_opt65_free(iqconverter_int16_opt65_t *cnv)
{
	if (cnv) iqconv_aligned_free(cnv);
}

void iqconverter_int16_opt65_reset(iqconverter_int16_opt65_t *cnv)
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
void iqconverter_int16_opt65_process_u16(iqconverter_int16_opt65_t * OPT_RESTRICT cnv,
					 const uint16_t * OPT_RESTRICT src,
					 int16_t * OPT_RESTRICT dest, int len)
{
	int i;
	int16_t ox = cnv->old_x;
	int16_t oy = cnv->old_y;
	int32_t oe = cnv->old_e;
	int32_t f_head = cnv->fir_head;
	int32_t d_head = cnv->dly_head;

	/* Apply alignment hints to enable better vectorization */
	int32_t * OPT_RESTRICT fir_buf = (int32_t *)OPT_ALIGNED_64(cnv->fir_buf);
	const int32_t * OPT_RESTRICT kernel = (const int32_t *)OPT_ALIGNED_64(cnv->sym_kernel);
	int16_t * OPT_RESTRICT dly_buf = (int16_t *)OPT_ALIGNED_64(cnv->dly_buf);
	const int16_t * OPT_RESTRICT lut = (const int16_t *)OPT_ALIGNED_64(iqconv_lut.lut_int16);

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
		int16_t x0, x1, x2, x3;
		int16_t y0, y1, y2, y3;
		int32_t *fir_src;
		int32_t acc0, acc1;
		int16_t q_in0, q_in1;
		int16_t q_out0, q_out1;
		int16_t w, s;
		int32_t u;

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

		/* FIR I-Channel (Filter 0) - 32-tap symmetric (16 pairs) */
		f_head = (f_head - 1) & FIR_BUF_MASK;
		fir_buf[f_head] = (int32_t)((int16_t)(-y0));
		fir_buf[f_head + FIR_BUF_SIZE] = fir_buf[f_head];

		fir_src = &fir_buf[f_head];
		acc0 = kernel[0]  * (fir_src[0]  + fir_src[31])
		     + kernel[1]  * (fir_src[1]  + fir_src[30])
		     + kernel[2]  * (fir_src[2]  + fir_src[29])
		     + kernel[3]  * (fir_src[3]  + fir_src[28])
		     + kernel[4]  * (fir_src[4]  + fir_src[27])
		     + kernel[5]  * (fir_src[5]  + fir_src[26])
		     + kernel[6]  * (fir_src[6]  + fir_src[25])
		     + kernel[7]  * (fir_src[7]  + fir_src[24])
		     + kernel[8]  * (fir_src[8]  + fir_src[23])
		     + kernel[9]  * (fir_src[9]  + fir_src[22])
		     + kernel[10] * (fir_src[10] + fir_src[21])
		     + kernel[11] * (fir_src[11] + fir_src[20])
		     + kernel[12] * (fir_src[12] + fir_src[19])
		     + kernel[13] * (fir_src[13] + fir_src[18])
		     + kernel[14] * (fir_src[14] + fir_src[17])
		     + kernel[15] * (fir_src[15] + fir_src[16]);

		/* FIR I-Channel (Filter 1) - 32-tap symmetric (16 pairs) */
		f_head = (f_head - 1) & FIR_BUF_MASK;
		fir_buf[f_head] = (int32_t)y2;
		fir_buf[f_head + FIR_BUF_SIZE] = fir_buf[f_head];

		fir_src = &fir_buf[f_head];
		acc1 = kernel[0]  * (fir_src[0]  + fir_src[31])
		     + kernel[1]  * (fir_src[1]  + fir_src[30])
		     + kernel[2]  * (fir_src[2]  + fir_src[29])
		     + kernel[3]  * (fir_src[3]  + fir_src[28])
		     + kernel[4]  * (fir_src[4]  + fir_src[27])
		     + kernel[5]  * (fir_src[5]  + fir_src[26])
		     + kernel[6]  * (fir_src[6]  + fir_src[25])
		     + kernel[7]  * (fir_src[7]  + fir_src[24])
		     + kernel[8]  * (fir_src[8]  + fir_src[23])
		     + kernel[9]  * (fir_src[9]  + fir_src[22])
		     + kernel[10] * (fir_src[10] + fir_src[21])
		     + kernel[11] * (fir_src[11] + fir_src[20])
		     + kernel[12] * (fir_src[12] + fir_src[19])
		     + kernel[13] * (fir_src[13] + fir_src[18])
		     + kernel[14] * (fir_src[14] + fir_src[17])
		     + kernel[15] * (fir_src[15] + fir_src[16]);

		/* Mix & Delay Q-Channel - BRANCHLESS with power-of-2 mask */
		q_in0 = (int16_t)((-y1) >> 1);
		q_in1 = (int16_t)(y3 >> 1);

		q_out0 = dly_buf[d_head];
		dly_buf[d_head] = q_in0;
		d_head = (d_head + 1) & DELAY_MASK;

		q_out1 = dly_buf[d_head];
		dly_buf[d_head] = q_in1;
		d_head = (d_head + 1) & DELAY_MASK;

		/* Output */
		dest[i + 0] = (int16_t)(acc0 >> 15);
		dest[i + 1] = q_out0;
		dest[i + 2] = (int16_t)(acc1 >> 15);
		dest[i + 3] = q_out1;
	}

	cnv->old_x = ox; cnv->old_y = oy; cnv->old_e = oe;
	cnv->fir_head = f_head; cnv->dly_head = d_head;
}
