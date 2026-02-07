/*
 * DDC DECIMATION STAGE API
 *
 * Provides data structures and functions for cascaded halfband decimation.
 * Used internally by iqconverter_float_opt_dec.c and iqconverter_int16_opt_dec.c
 *
 * Architecture:
 *   Mixed 33/17-tap halfband filters optimized for < 0.1 dB passband ripple:
 *   - Stages 0-2: 33-tap (pb=0.195, 0.0137 dB ripple, 62 dB rejection, 8 MACs)
 *   - Stages 3-5: 17-tap (pb=0.148, 0.0192 dB ripple, 59 dB rejection, 4 MACs)
 *
 *   Cascade ripple at 64x (3x33 + 3x17):
 *   - 33-tap contribution: 3 x 0.0137 = 0.0411 dB
 *   - 17-tap contribution: 3 x 0.0192 = 0.0576 dB
 *   - Total: ~0.083 dB (well under 0.1 dB requirement)
 *
 *   Both filter types maintain power-of-2 symmetry for branchless modulo.
 *
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef IQCONVERTER_DECIMATOR_H
#define IQCONVERTER_DECIMATOR_H

#include <stdint.h>
#include "filters_opt.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * FILTER CONSTANTS
 *
 * Two filter types available:
 *   - 33-tap: Higher rejection (~62 dB), 8 MACs, used for stages 0-2
 *   - 17-tap: Faster (~59 dB rejection), 4 MACs, used for stages 3+
 *
 * Optimization: Later stages (3+) operate at lower sample rates where the
 * narrower passband of the 17-tap filter is still adequate. Using 17-tap
 * for stages 3+ reduces computation while maintaining < 0.1 dB total ripple.
 *
 * Both filters use power-of-2 buffer sizes for branchless modulo operation.
 * ======================================================================== */

/* 33-tap halfband (for stages 0-2) - optimized pb=0.195 */
#define DEC_HB33_TAPS      33   /* Total filter taps */
#define DEC_HB33_SYM_PAIRS 8    /* Number of symmetric coefficient pairs */
#define DEC_HB33_BUF_SIZE  32   /* Buffer size (power of 2) */
#define DEC_HB33_BUF_MASK  31   /* Mask for branchless wrap */
#define DEC_HB33_DELAY     16   /* Group delay: (33-1)/2 samples */

/* 17-tap halfband (for stages 3+) - low ripple pb=0.148 */
#define DEC_HB17_TAPS      17   /* Total filter taps */
#define DEC_HB17_SYM_PAIRS 4    /* Number of symmetric coefficient pairs */
#define DEC_HB17_BUF_SIZE  16   /* Buffer size (power of 2) */
#define DEC_HB17_BUF_MASK  15   /* Mask for branchless wrap */
#define DEC_HB17_DELAY     8    /* Group delay: (17-1)/2 samples */

/* Stage threshold for 17-tap filter (stages >= this use 17-tap) */
#define DEC_STAGE_17TAP_THRESHOLD 3

/* Maximum number of cascaded decimation stages */
#define DEC_MAX_STAGES 6

/* Number of supported decimation factors: 1, 2, 4, 8, 16, 32, 64 */
#define DEC_NUM_FACTORS 7

/* Default intermediate buffer size (enough for typical block sizes) */
#define DEC_DEFAULT_INTER_BUF_LEN 65536

/*
 * Chunk size for cache-optimized processing.
 * Process in chunks that fit in L2 cache to keep data hot between stages.
 * 4096 I/Q pairs fits in L1 cache (32KB for float, 16KB for int16).
 */
#define DEC_CHUNK_SIZE 4096

/* Filter type indicator */
#define DEC_FILTER_33TAP 0
#define DEC_FILTER_17TAP 1

/* ========================================================================
 * GROUP DELAY CONSTANTS
 *
 * Total cascaded delays (at input Fs) for each decimation factor.
 * Does NOT include base DDC delay (depends on base filter length).
 *
 * With mixed filters (33-tap for stages 0-2, 17-tap for stages 3+):
 *   Stage 0: 33-tap, delay = 16 samples at Fs
 *   Stage 1: 33-tap, delay = 16 samples at Fs/2 = 32 at Fs
 *   Stage 2: 33-tap, delay = 16 samples at Fs/4 = 64 at Fs
 *   Stage 3: 17-tap, delay = 8 samples at Fs/8 = 64 at Fs
 *   Stage 4: 17-tap, delay = 8 samples at Fs/16 = 128 at Fs
 *   Stage 5: 17-tap, delay = 8 samples at Fs/32 = 256 at Fs
 *
 * Total delays:
 *   1x:  0 samples
 *   2x:  16 samples
 *   4x:  16 + 32 = 48 samples
 *   8x:  48 + 64 = 112 samples (all 33-tap)
 *   16x: 112 + 64 = 176 samples
 *   32x: 176 + 128 = 304 samples
 *   64x: 304 + 256 = 560 samples
 * ======================================================================== */

#define DEC_TOTAL_DELAY_1X  0
#define DEC_TOTAL_DELAY_2X  (DEC_HB33_DELAY)
#define DEC_TOTAL_DELAY_4X  (DEC_HB33_DELAY + DEC_HB33_DELAY * 2)
#define DEC_TOTAL_DELAY_8X  (DEC_TOTAL_DELAY_4X + DEC_HB33_DELAY * 4)
#define DEC_TOTAL_DELAY_16X (DEC_TOTAL_DELAY_8X + DEC_HB17_DELAY * 8)
#define DEC_TOTAL_DELAY_32X (DEC_TOTAL_DELAY_16X + DEC_HB17_DELAY * 16)
#define DEC_TOTAL_DELAY_64X (DEC_TOTAL_DELAY_32X + DEC_HB17_DELAY * 32)

/* ========================================================================
 * FLOAT32 DECIMATION STAGE
 * ======================================================================== */

/**
 * Float32 decimation stage context
 *
 * Implements a single 2:1 decimation using a complex halfband FIR filter.
 * Uses mirrored circular buffer with interleaved I/Q for cache efficiency.
 *
 * Stages 0-2 use 33-tap filter (8 MACs, ~62 dB rejection).
 * Stages 3+ use 17-tap filter (4 MACs, ~59 dB rejection).
 */
typedef struct {
	float *queue_iq;        /* Interleaved I/Q buffer (mirrored, 2x size) */
	int fir_index;          /* Circular buffer write index */
	int taps;               /* Total filter taps (33 or 17) */
	int buf_size;           /* Buffer size (32 or 16) */
	int buf_mask;           /* Mask for branchless wrap (31 or 15) */
	int filter_type;        /* DEC_FILTER_33TAP or DEC_FILTER_17TAP */
} dec_stage_float_t;

/* ========================================================================
 * INT16 DECIMATION STAGE
 * ======================================================================== */

/**
 * Int16 decimation stage context
 *
 * Fixed-point arithmetic:
 *   - Coefficients: Q15 format (scaled by 32768)
 *   - Accumulators: 32-bit (safe from overflow)
 *   - Output shift: >> 14 (intentional 2x gain for SNR preservation)
 *
 * The >> 14 shift converts decimation SNR gain (+3 dB per stage) into
 * amplitude, providing +0.5 effective bits per decimation stage.
 * Requires AGC to limit input headroom based on decimation factor.
 *
 * Stages 0-2 use 33-tap filter (8 MACs, ~62 dB rejection).
 * Stages 3+ use 17-tap filter (4 MACs, ~59 dB rejection).
 */
typedef struct {
	int16_t *queue_iq;      /* Interleaved I/Q buffer (mirrored, 2x size) */
	int fir_index;          /* Circular buffer write index */
	int taps;               /* Total filter taps (33 or 17) */
	int buf_size;           /* Buffer size (32 or 16) */
	int buf_mask;           /* Mask for branchless wrap (31 or 15) */
	int filter_type;        /* DEC_FILTER_33TAP or DEC_FILTER_17TAP */
} dec_stage_int16_t;

/* ========================================================================
 * STAGE LIFECYCLE
 * ======================================================================== */

/**
 * Initialize a float32 decimation stage
 *
 * Parameters:
 *   stage      - Stage context to initialize
 *   stage_num  - Stage number (0-5)
 *
 * Returns:
 *   0 on success, -1 on failure
 */
int dec_stage_float_init(dec_stage_float_t *stage, int stage_num);

/**
 * Initialize an int16 decimation stage
 */
int dec_stage_int16_init(dec_stage_int16_t *stage, int stage_num);

/**
 * Free decimation stage resources
 */
void dec_stage_float_free(dec_stage_float_t *stage);
void dec_stage_int16_free(dec_stage_int16_t *stage);

/**
 * Reset decimation stage state (clear history buffers)
 */
void dec_stage_float_reset(dec_stage_float_t *stage);
void dec_stage_int16_reset(dec_stage_int16_t *stage);

/* ========================================================================
 * STAGE PROCESSING
 *
 * Two filter variants available:
 *   - 33-tap: Higher quality, 8 MACs per output
 *   - 17-tap: Faster, 4 MACs per output
 *
 * Use dec_stage_float_process() / dec_stage_int16_process() for automatic
 * dispatch based on stage->filter_type.
 * ======================================================================== */

/**
 * Process complex I/Q through float32 halfband decimator (2:1)
 * Automatically selects 33-tap or 17-tap based on stage->filter_type.
 *
 * Parameters:
 *   stage    - Decimation stage context
 *   src      - Input I/Q pairs (I0,Q0,I1,Q1,...) - len*2 floats
 *   dest     - Output I/Q pairs - (len/2)*2 floats
 *   len      - Number of input I/Q PAIRS (must be multiple of 4)
 */
void dec_stage_float_process(dec_stage_float_t *stage,
			     const float *src, float *dest, int len);

/**
 * Process complex I/Q through int16 halfband decimator (2:1)
 * Automatically selects 33-tap or 17-tap based on stage->filter_type.
 */
void dec_stage_int16_process(dec_stage_int16_t *stage,
			     const int16_t *src, int16_t *dest, int len);

/* Direct 33-tap processing (for explicit use or testing) */
void dec_hb33_float_process(dec_stage_float_t *stage,
			    const float *src, float *dest, int len);
void dec_hb33_int16_process(dec_stage_int16_t *stage,
			    const int16_t *src, int16_t *dest, int len);

/* Direct 17-tap processing (for explicit use or testing) */
void dec_hb17_float_process(dec_stage_float_t *stage,
			    const float *src, float *dest, int len);
void dec_hb17_int16_process(dec_stage_int16_t *stage,
			    const int16_t *src, int16_t *dest, int len);

/* ========================================================================
 * HELPER FUNCTIONS
 * ======================================================================== */

/**
 * Get filter parameters for a stage number
 *
 * Returns:
 *   0 on success, -1 if stage_num is invalid (must be 0-5)
 */
int dec_get_stage_params(int stage_num,
			 int *out_taps, int *out_sym_len,
			 int *out_buf_size, int *out_buf_mask);

/**
 * Calculate number of stages needed for decimation factor
 *
 * Returns:
 *   Number of stages (0-6), or -1 if invalid
 */
static inline int dec_get_num_stages(int decimation)
{
	switch (decimation) {
	case 1:  return 0;
	case 2:  return 1;
	case 4:  return 2;
	case 8:  return 3;
	case 16: return 4;
	case 32: return 5;
	case 64: return 6;
	default: return -1;
	}
}

/**
 * Validate decimation factor
 *
 * Returns:
 *   1 if valid (1, 2, 4, 8, 16, 32, or 64), 0 otherwise
 */
static inline int dec_valid_factor(int decimation)
{
	return (decimation == 1 || decimation == 2 ||
		decimation == 4 || decimation == 8 ||
		decimation == 16 || decimation == 32 ||
		decimation == 64);
}

/**
 * Get total decimation stage delay in samples at input rate (Fs)
 */
static inline int dec_get_decimation_delay(int decimation)
{
	switch (decimation) {
	case 1:  return DEC_TOTAL_DELAY_1X;
	case 2:  return DEC_TOTAL_DELAY_2X;
	case 4:  return DEC_TOTAL_DELAY_4X;
	case 8:  return DEC_TOTAL_DELAY_8X;
	case 16: return DEC_TOTAL_DELAY_16X;
	case 32: return DEC_TOTAL_DELAY_32X;
	case 64: return DEC_TOTAL_DELAY_64X;
	default: return -1;
	}
}

/**
 * Get stage delay for a single decimation stage (at stage's rate)
 * Stages 0-2 use 33-tap (delay=16), stages 3+ use 17-tap (delay=8)
 */
static inline int dec_get_stage_delay(int stage_num)
{
	if (stage_num < 0 || stage_num >= DEC_MAX_STAGES)
		return -1;
	return (stage_num < DEC_STAGE_17TAP_THRESHOLD) ?
		DEC_HB33_DELAY : DEC_HB17_DELAY;
}

/**
 * Get filter type for a stage number
 * Returns DEC_FILTER_33TAP or DEC_FILTER_17TAP
 */
static inline int dec_get_filter_type(int stage_num)
{
	if (stage_num < 0 || stage_num >= DEC_MAX_STAGES)
		return -1;
	return (stage_num < DEC_STAGE_17TAP_THRESHOLD) ?
		DEC_FILTER_33TAP : DEC_FILTER_17TAP;
}

#ifdef __cplusplus
}
#endif

#endif /* IQCONVERTER_DECIMATOR_H */
