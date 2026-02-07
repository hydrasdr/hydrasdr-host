/*
 * OPTIMIZED FLOAT32 DDC IMPLEMENTATION
 *
 * High-performance polyphase halfband DDC with:
 * - Dual-buffered FIR history (linear memory access)
 * - Fused DC+Mix+FIR+Delay processing
 * - ~1.9x performance improvement over legacy
 *
 * Copyright (C) 2014, Youssef Touil <youssef@airspy.com>
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef IQCONVERTER_FLOAT_OPT_H
#define IQCONVERTER_FLOAT_OPT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Optimized Float32 DDC Context
 *
 * Implementation Details:
 * ----------------------
 * - Dual-buffered FIR history: write to [idx] and [idx+HISTORY_SIZE]
 * - Allows linear convolution without wrap-around checks
 * - Power-of-2 buffer sizes for efficient masking
 * - DC removal: 1st-order IIR highpass (alpha = 0.01)
 * - Fs/4 mixing with center tap scaling
 * - All stages fused into single processing loop
 *
 * Memory Layout (64-byte aligned for AVX-512):
 * - fir_kernel: 64 bytes (16 floats, padded from 24)
 * - fir_queue: 4096 bytes (2*512 floats) - 64-byte aligned
 * - delay_line: 64 bytes (16 floats, padded from 12)
 */
#if defined(_MSC_VER)
__declspec(align(64))
#endif
typedef struct {
	/* Filter buffers (64-byte aligned for AVX-512) */
	float *fir_kernel; /* Filter coefficients (non-zero taps) */
	float *fir_queue; /* Dual-buffered FIR history (2*512) */
	float *delay_line; /* Delay line for Q-channel */

	/* DC removal state */
	float avg; /* Running average for DC estimation */

	/* Filter parameters */
	float hbc; /* Halfband center coefficient (0.5) */
	int len; /* Number of non-zero taps (24) */
	int total_taps; /* Total filter length (47) */

	/* Circular buffer indices */
	int fir_index; /* FIR buffer write position */
	int delay_index; /* Delay buffer write position */
}
#if defined(__GNUC__) || defined(__clang__)
__attribute__((aligned(64)))
#endif
iqconverter_float_opt_t;

/*
 * Create optimized float32 DDC instance
 *
 * Parameters:
 *   hb_kernel - 47-tap halfband filter coefficients
 *   len       - Filter length (must be 47)
 *
 * Returns:
 *   Pointer to DDC instance, or NULL on allocation failure
 *
 * Notes:
 *   - Allocates aligned memory for SIMD optimization
 *   - Extracts non-zero coefficients (every other tap)
 *   - Initializes dual-buffered history
 */
iqconverter_float_opt_t *iqconverter_float_opt_create(const float *hb_kernel,
						      int len);

/*
 * Free optimized float32 DDC instance
 */
void iqconverter_float_opt_free(iqconverter_float_opt_t *cnv);

/*
 * Reset optimized float32 DDC state
 */
void iqconverter_float_opt_reset(iqconverter_float_opt_t *cnv);

/*
 * FUSED Processing: uint16_t ADC samples -> float32 I/Q output
 *
 * Combines sample conversion (uint16_t -> float) with DDC processing
 * in a single loop for maximum cache efficiency.
 *
 * Parameters:
 *   cnv  - DDC instance
 *   src  - Raw ADC samples (uint16_t, supports 8, 10, 12, 14, or 16-bit via LUT)
 *   dest - Output float I/Q samples buffer
 *   len  - Number of input samples (must be multiple of 4)
 *
 * Processing Pipeline (Fused):
 * ---------------------------
 * For each 4-sample ADC block [x0, x1, x2, x3]:
 *
 * 1. LUT-based ADC Conversion + DC Removal:
 *    x_float = lut[adc_sample] - avg
 *    avg += alpha * x_float  (alpha = 0.01)
 *
 * 2. Polyphase routing with Fs/4 mixing:
 *    Type-0 (FIR): [-x0, +x2] with full convolution -> I-channel
 *    Type-1 (Delay): [-x1*hbc, +x3*hbc] with delay compensation -> Q-channel
 *
 * 3. FIR Convolution (I-channel):
 *    acc = sum k[i] * x[i] for i=0..23
 *    Uses dual-buffer technique for linear access
 *
 * 4. Delay Compensation (Q-channel):
 *    Delay = (47-1)/4 = 11.5 -> 12 samples
 *
 * 5. Output interleaving (complex I/Q):
 *    [I_filtered0, Q_delayed0, I_filtered1, Q_delayed1]
 *
 * Performance:
 * -----------
 * - ~500 MSPS on modern x86_64 (vs ~260 MSPS legacy)
 * - ~2x speedup over legacy implementation
 * - Eliminates intermediate float buffer allocation
 * - Single memory pass instead of two
 */
void iqconverter_float_opt_process_u16(iqconverter_float_opt_t *cnv,
				       const uint16_t *src,
				       float *dest, int len);

/*
 * FUSED Processing: uint8_t ADC samples (8-bit mode @ 40 MSPS) -> float32 I/Q output
 *
 * Same processing pipeline as _u16 variant but with 8-bit ADC input.
 * Uses 8-bit LUT (256 entries) for ADC -> float32 conversion.
 */
void iqconverter_float_opt_process_u8(iqconverter_float_opt_t *cnv,
				      const uint8_t *src,
				      float *dest, int len);

#ifdef __cplusplus
}
#endif

#endif /* IQCONVERTER_FLOAT_OPT_H */
