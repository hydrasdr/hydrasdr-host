/*
 * OPTIMIZED FLOAT32 DDC IMPLEMENTATION - 83-TAP VARIANT
 *
 * High-performance polyphase halfband DDC with:
 * - 83-tap filter (42 non-zero taps, 21 symmetric pairs)
 * - Delay = 21 samples (NOT power-of-2, requires branch)
 * - ~103 dB stopband rejection (high dynamic range)
 * - Dual-buffered FIR history (linear memory access)
 * - Fused DC+Mix+FIR+Delay processing
 *
 * Based on commit https://github.com/airspy/airspyone_host/commit/091c6f7449bc976f97bb327242532bfb1001d116
 *
 * Original work Copyright (C) 2014, Youssef Touil <youssef@airspy.com>
 * Modifications Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef IQCONVERTER_FLOAT_OPT83_H
#define IQCONVERTER_FLOAT_OPT83_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Optimized Float32 DDC Context (83-tap variant)
 *
 * Key Features:
 * - 83 taps -> 42 non-zero taps -> 21 symmetric pairs
 * - Delay = 21 samples (NOT power-of-2, uses branch)
 * - Highest rejection (~103 dB) for high dynamic range
 * - Ideal for high-fidelity applications
 *
 * Memory Layout (64-byte aligned for AVX-512):
 * - fir_kernel: 128 bytes (21 floats + padding for symmetric pairs)
 * - fir_queue: 4096 bytes (2*512 floats) - 64-byte aligned
 * - delay_line: 128 bytes (21 floats + padding)
 */
#if defined(_MSC_VER)
__declspec(align(64))
#endif
typedef struct {
	/* Filter buffers (64-byte aligned for AVX-512) */
	float *fir_kernel;	/* Filter coefficients (21 non-zero taps) */
	float *fir_queue;	/* Dual-buffered FIR history (2*512) */
	float *delay_line;	/* Delay line for Q-channel (21 samples) */

	/* DC removal state */
	float avg;		/* Running average for DC estimation */

	/* Filter parameters */
	float hbc;		/* Halfband center coefficient (0.5) */
	int len;		/* Number of non-zero taps (42) */
	int total_taps;		/* Total filter length (83) */

	/* Circular buffer indices */
	int fir_index;		/* FIR buffer write position */
	int delay_index;	/* Delay buffer write position */
}
#if defined(__GNUC__) || defined(__clang__)
__attribute__((aligned(64)))
#endif
iqconverter_float_opt83_t;

/*
 * Create optimized float32 DDC instance (83-tap variant)
 *
 * Parameters:
 *   hb_kernel - 83-tap halfband filter coefficients
 *   len       - Filter length (must be 83)
 *
 * Returns:
 *   Pointer to DDC instance, or NULL on allocation failure
 */
iqconverter_float_opt83_t *iqconverter_float_opt83_create(const float *hb_kernel,
							  int len);

/*
 * Free optimized float32 DDC instance (83-tap variant)
 */
void iqconverter_float_opt83_free(iqconverter_float_opt83_t *cnv);

/*
 * Reset optimized float32 DDC state (83-tap variant)
 */
void iqconverter_float_opt83_reset(iqconverter_float_opt83_t *cnv);

/*
 * FUSED Processing: uint16_t ADC samples -> float32 I/Q output (83-tap variant)
 *
 * Note: Delay (21 samples) is not power-of-2, uses conditional branch
 */
void iqconverter_float_opt83_process_u16(iqconverter_float_opt83_t *cnv,
					 const uint16_t *src,
					 float *dest, int len);

#ifdef __cplusplus
}
#endif

#endif /* IQCONVERTER_FLOAT_OPT83_H */
