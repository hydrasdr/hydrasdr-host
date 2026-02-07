/*
 * OPTIMIZED FLOAT32 DDC IMPLEMENTATION - 65-TAP VARIANT
 *
 * High-performance polyphase halfband DDC with:
 * - 65-tap filter (33 non-zero taps, 16 symmetric pairs)
 * - Power-of-2 delay (16 samples) for branchless wrap
 * - ~66 dB stopband rejection
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

#ifndef IQCONVERTER_FLOAT_OPT65_H
#define IQCONVERTER_FLOAT_OPT65_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Optimized Float32 DDC Context (65-tap variant)
 *
 * Key Differences from 47-tap:
 * - 65 taps -> 33 non-zero taps -> 16 symmetric pairs
 * - Delay = 16 samples (power-of-2, branchless modulo)
 * - Slightly higher rejection (~66 dB vs ~63 dB)
 * - 33% more MACs but better branch prediction
 *
 * Memory Layout (64-byte aligned for AVX-512):
 * - fir_kernel: 64 bytes (16 floats for symmetric pairs)
 * - fir_queue: 4096 bytes (2*512 floats) - 64-byte aligned
 * - delay_line: 64 bytes (16 floats - exact fit for delay=16)
 */
#if defined(_MSC_VER)
__declspec(align(64))
#endif
typedef struct {
	/* Filter buffers (64-byte aligned for AVX-512) */
	float *fir_kernel;	/* Filter coefficients (16 non-zero taps) */
	float *fir_queue;	/* Dual-buffered FIR history (2*512) */
	float *delay_line;	/* Delay line for Q-channel (16 samples) */

	/* DC removal state */
	float avg;		/* Running average for DC estimation */

	/* Filter parameters */
	float hbc;		/* Halfband center coefficient (0.5) */
	int len;		/* Number of non-zero taps (33) */
	int total_taps;		/* Total filter length (65) */

	/* Circular buffer indices */
	int fir_index;		/* FIR buffer write position */
	int delay_index;	/* Delay buffer write position */
}
#if defined(__GNUC__) || defined(__clang__)
__attribute__((aligned(64)))
#endif
iqconverter_float_opt65_t;

/*
 * Create optimized float32 DDC instance (65-tap variant)
 *
 * Parameters:
 *   hb_kernel - 65-tap halfband filter coefficients
 *   len       - Filter length (must be 65)
 *
 * Returns:
 *   Pointer to DDC instance, or NULL on allocation failure
 */
iqconverter_float_opt65_t *iqconverter_float_opt65_create(const float *hb_kernel,
							  int len);

/*
 * Free optimized float32 DDC instance (65-tap variant)
 */
void iqconverter_float_opt65_free(iqconverter_float_opt65_t *cnv);

/*
 * Reset optimized float32 DDC state (65-tap variant)
 */
void iqconverter_float_opt65_reset(iqconverter_float_opt65_t *cnv);

/*
 * FUSED Processing: uint16_t ADC samples -> float32 I/Q output (65-tap variant)
 *
 * Key Optimization: Power-of-2 Delay
 * ----------------------------------
 * With delay=16, the delay index wrap uses branchless AND:
 *   dly_idx = (dly_idx + 1) & 15;  // No branch!
 *
 * vs 47-tap with delay=12:
 *   if (++dly_idx >= 12) dly_idx = 0;  // Branch
 *
 * This eliminates branch misprediction penalty (~5-10 cycles per miss).
 */
void iqconverter_float_opt65_process_u16(iqconverter_float_opt65_t *cnv,
					 const uint16_t *src,
					 float *dest, int len);

#ifdef __cplusplus
}
#endif

#endif /* IQCONVERTER_FLOAT_OPT65_H */
