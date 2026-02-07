/*
 * OPTIMIZED INT16 DDC IMPLEMENTATION
 *
 * High-performance polyphase halfband DDC with:
 * - Symmetric FIR coefficient folding (50% MAC reduction)
 * - Mirrored ring buffer (zero-overhead wrapping)
 * - Fused DC+Mix+FIR+Delay processing
 *
 * Original work Copyright (C) 2014, Youssef Touil <youssef@airspy.com>
 * Modifications Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef IQCONVERTER_INT16_OPT_H
#define IQCONVERTER_INT16_OPT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Optimized Int16 DDC Context
 *
 * Implementation Details:
 * ----------------------
 * - Uses symmetric FIR folding: acc += k[i] * (x[i] + x[N-1-i])
 * - FIR buffer is mirrored: write to both [idx] and [idx+BUFFER_SIZE]
 * - DC removal: 1st-order IIR highpass (alpha = 32100/32768 ~ 0.98)
 * - Fs/4 mixing: sign pattern [-I, -Q>>1, +I, +Q>>1]
 * - Q15 arithmetic: coefficients scaled by 32768
 *
 * Memory Layout (64-byte aligned for AVX-512):
 * - sym_kernel: 16 int32_t (64 bytes) - padded from 12
 * - fir_buf: 1024 int32_t (4096 bytes) - 64-byte aligned
 * - dly_buf: 32 int16_t (64 bytes) - padded from 12
 */
#if defined(_MSC_VER)
__declspec(align(64))
#endif
typedef struct {
	/* Filter coefficients - padded to 64 bytes (16 x int32_t) */
	int32_t sym_kernel[16]
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

	/* Delay line for Q-channel - padded to 64 bytes (32 x int16_t) */
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

	/* Delay line length (depends on filter length) */
	int dly_len;
}
#if defined(__GNUC__) || defined(__clang__)
__attribute__((aligned(64)))
#endif
iqconverter_int16_opt_t;

/*
 * Create optimized int16 DDC instance
 *
 * Parameters:
 *   hb_kernel - 47-tap halfband filter coefficients (Q15 format)
 *   len       - Filter length (must be 47)
 *
 * Returns:
 *   Pointer to DDC instance, or NULL on allocation failure
 *
 * Notes:
 *   - Extracts symmetric coefficient pairs for folded FIR
 *   - Allocates aligned memory for SIMD optimization
 *   - Initializes all state to zero
 */
iqconverter_int16_opt_t *iqconverter_int16_opt_create(const int16_t *hb_kernel,
						      int len);

/*
 * Free optimized int16 DDC instance
 *
 * Parameters:
 *   cnv - DDC instance (can be NULL)
 */
void iqconverter_int16_opt_free(iqconverter_int16_opt_t *cnv);

/*
 * Reset optimized int16 DDC state
 *
 * Parameters:
 *   cnv - DDC instance
 *
 * Resets:
 *   - FIR history buffer to zero
 *   - Delay line to zero
 *   - DC removal state to zero
 *   - Circular buffer indices to initial positions
 */
void iqconverter_int16_opt_reset(iqconverter_int16_opt_t *cnv);

/*
 * FUSED Processing: uint16_t ADC samples -> int16 I/Q output
 *
 * Uses precomputed LUT for ultra-fast ADC -> int16 conversion.
 *
 * Processing Pipeline (Fused):
 * ---------------------------
 * For each 4-sample ADC block [x0, x1, x2, x3]:
 *
 * 1. LUT-based ADC Conversion + DC Removal (1st-order IIR highpass):
 *    x_int16 = lut[adc_sample]
 *    y[n] = x[n] - x[n-1] + 0.98*y[n-1]
 *
 * 2. Fs/4 Mixing with center tap scaling:
 *    I: [-x0, +x2] -> FIR path (I-channel)
 *    Q: [-x1>>1, +x3>>1] -> Delay path (Q-channel)
 *
 * 3. Symmetric FIR (I-channel):
 *    acc = sum k[i] * (x[i] + x[23-i]) for i=0..11
 *
 * 4. Delay compensation (Q-channel):
 *    Delay = (47-1)/4 = 11.5 -> 12 samples
 *
 * 5. Output (complex I/Q):
 *    [I_filtered0, Q_delayed0, I_filtered1, Q_delayed1]
 *
 * Performance:
 * -----------
 * - ~400 MSPS on modern x86_64 (vs ~170 MSPS legacy)
 * - ~2.4x performance improvement over legacy
 */
void iqconverter_int16_opt_process_u16(iqconverter_int16_opt_t *cnv,
				       const uint16_t *src,
				       int16_t *dest, int len);

/*
 * FUSED Processing: uint8_t ADC samples (8-bit mode @ 40 MSPS) -> int16 I/Q output
 *
 * Same processing pipeline as _u16 variant but with 8-bit ADC input.
 * Uses 8-bit LUT (256 entries) for ADC -> int16 conversion.
 *
 * Parameters:
 *   cnv  - DDC instance
 *   src  - Raw uint8_t ADC samples (input)
 *   dest - int16_t I/Q output buffer
 *   len  - Number of input samples (must be multiple of 4)
 */
void iqconverter_int16_opt_process_u8(iqconverter_int16_opt_t *cnv,
				      const uint8_t *src,
				      int16_t *dest, int len);

#ifdef __cplusplus
}
#endif

#endif /* IQCONVERTER_INT16_OPT_H */
