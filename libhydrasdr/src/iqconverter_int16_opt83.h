/*
 * OPTIMIZED INT16 DDC IMPLEMENTATION - 83-TAP VARIANT
 *
 * High-performance polyphase halfband DDC with:
 * - 83-tap filter (42 non-zero taps, 21 symmetric pairs)
 * - Delay = 21 samples (NOT power-of-2, requires branch)
 * - ~103 dB stopband rejection (high dynamic range)
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

#ifndef IQCONVERTER_INT16_OPT83_H
#define IQCONVERTER_INT16_OPT83_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Optimized Int16 DDC Context (83-tap variant)
 *
 * Key Features:
 * - 83 taps -> 42 non-zero taps -> 21 symmetric pairs
 * - Delay = 21 samples (NOT power-of-2, uses branch)
 * - Highest rejection (~103 dB) for high dynamic range
 * - Ideal for high-fidelity applications
 *
 * Memory Layout (64-byte aligned for AVX-512):
 * - sym_kernel: 128 bytes (21 int32_t + padding for symmetric coefficients)
 * - fir_buf: 4096 bytes (1024 int32_t, mirrored buffer)
 * - dly_buf: 64 bytes (21 int16_t + padding)
 */
#if defined(_MSC_VER)
__declspec(align(64))
#endif
typedef struct {
	/* Filter coefficients - 128 bytes (21 x int32_t + padding) */
	int32_t sym_kernel[32]
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

	/* Delay line for Q-channel - 64 bytes (32 x int16_t, using first 21) */
	int16_t dly_buf[32]
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

	/* Delay line length (for 83-tap: 21, NOT power-of-2) */
	int dly_len;
}
#if defined(__GNUC__) || defined(__clang__)
__attribute__((aligned(64)))
#endif
iqconverter_int16_opt83_t;

/*
 * Create optimized int16 DDC instance (83-tap variant)
 *
 * Parameters:
 *   hb_kernel - 83-tap halfband filter coefficients (Q15 format)
 *   len       - Filter length (must be 83)
 *
 * Returns:
 *   Pointer to DDC instance, or NULL on allocation failure
 */
iqconverter_int16_opt83_t *iqconverter_int16_opt83_create(const int16_t *hb_kernel,
							  int len);

/*
 * Free optimized int16 DDC instance (83-tap variant)
 */
void iqconverter_int16_opt83_free(iqconverter_int16_opt83_t *cnv);

/*
 * Reset optimized int16 DDC state (83-tap variant)
 */
void iqconverter_int16_opt83_reset(iqconverter_int16_opt83_t *cnv);

/*
 * FUSED Processing: uint16_t ADC samples -> int16 I/Q output (83-tap variant)
 *
 * Combines LUT-based ADC conversion with DDC processing in a single loop
 * for maximum cache efficiency.
 *
 * Parameters:
 *   cnv  - DDC instance
 *   src  - Raw ADC samples (uint16_t, supports 8, 10, 12, 14, or 16-bit via LUT)
 *   dest - Output int16 I/Q samples buffer
 *   len  - Number of input samples (must be multiple of 4)
 *
 * Note: Delay (21 samples) is not power-of-2, uses conditional branch
 */
void iqconverter_int16_opt83_process_u16(iqconverter_int16_opt83_t *cnv,
					 const uint16_t *src,
					 int16_t *dest, int len);

#ifdef __cplusplus
}
#endif

#endif /* IQCONVERTER_INT16_OPT83_H */
