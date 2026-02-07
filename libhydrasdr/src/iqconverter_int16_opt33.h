/*
 * OPTIMIZED INT16 DDC IMPLEMENTATION - 33-TAP VARIANT
 *
 * High-performance polyphase halfband DDC with:
 * - 33-tap filter (17 non-zero taps, 8 symmetric pairs)
 * - Power-of-2 delay (8 samples) for branchless wrap
 * - ~62 dB stopband rejection
 * - Q15 fixed-point arithmetic
 * - Mirrored FIR buffer for wrap-free access
 *
 * Based on commit https://github.com/airspy/airspyone_host/commit/091c6f7449bc976f97bb327242532bfb1001d116
 *
 * Original work Copyright (C) 2014, Youssef Touil <youssef@airspy.com>
 * Modifications Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef IQCONVERTER_INT16_OPT33_H
#define IQCONVERTER_INT16_OPT33_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Optimized Int16 DDC Context (33-tap variant)
 *
 * Key Features:
 * - 33 taps -> 17 non-zero taps -> 8 symmetric pairs
 * - Delay = 8 samples (power-of-2, branchless modulo)
 * - Lower rejection (~62 dB) but fastest processing
 * - Ideal for low-latency applications
 *
 * Memory Layout (64-byte aligned for AVX-512):
 * - sym_kernel: 32 bytes (8 int32_t for symmetric coefficients)
 * - fir_buf: 4096 bytes (1024 int32_t, mirrored buffer)
 * - dly_buf: 32 bytes (8 int16_t + padding)
 */
#if defined(_MSC_VER)
__declspec(align(64))
#endif
typedef struct {
	/* Filter coefficients - 32 bytes (8 x int32_t) */
	int32_t sym_kernel[8]
#if defined(__GNUC__) || defined(__clang__)
		__attribute__((aligned(64)))
#endif
	;

	/* FIR history buffer (mirrored for wrap-free access) - 64-byte aligned */
	int32_t fir_buf[1024]
#if defined(__GNUC__) || defined(__clang__)
		__attribute__((aligned(64)))
#endif
	;	/* Size: 2 * 512 */

	/* Delay line for Q-channel - 32 bytes (16 x int16_t, using first 8) */
	int16_t dly_buf[16]
#if defined(__GNUC__) || defined(__clang__)
		__attribute__((aligned(64)))
#endif
	;

	/* Circular buffer indices */
	int32_t fir_head;	/* FIR buffer write position */
	int32_t dly_head;	/* Delay buffer write position */

	/* DC removal state (Q15 fixed-point IIR) */
	int16_t old_x;		/* Previous input sample x[n-1] */
	int16_t old_y;		/* Previous output sample y[n-1] */
	int32_t old_e;		/* Accumulator error term */

	/* Delay line length (for 33-tap: 8, power-of-2) */
	int dly_len;
}
#if defined(__GNUC__) || defined(__clang__)
__attribute__((aligned(64)))
#endif
iqconverter_int16_opt33_t;

/*
 * Create optimized int16 DDC instance (33-tap variant)
 *
 * Parameters:
 *   hb_kernel - 33-tap halfband filter coefficients (Q15 format)
 *   len       - Filter length (must be 33)
 *
 * Returns:
 *   Pointer to DDC instance, or NULL on allocation failure
 */
iqconverter_int16_opt33_t *iqconverter_int16_opt33_create(const int16_t *hb_kernel,
							  int len);

/*
 * Free optimized int16 DDC instance (33-tap variant)
 */
void iqconverter_int16_opt33_free(iqconverter_int16_opt33_t *cnv);

/*
 * Reset optimized int16 DDC state (33-tap variant)
 */
void iqconverter_int16_opt33_reset(iqconverter_int16_opt33_t *cnv);

/*
 * FUSED Processing: uint16_t ADC samples -> int16 I/Q output (33-tap variant)
 *
 * Uses precomputed LUT for ultra-fast ADC -> int16 conversion.
 *
 * Key Optimization: Power-of-2 Delay
 * ----------------------------------
 * With delay=8, the delay index wrap uses branchless AND:
 *   dly_idx = (dly_idx + 1) & 7;  // No branch!
 */
void iqconverter_int16_opt33_process_u16(iqconverter_int16_opt33_t *cnv,
					 const uint16_t *src,
					 int16_t *dest, int len);

#ifdef __cplusplus
}
#endif

#endif /* IQCONVERTER_INT16_OPT33_H */
