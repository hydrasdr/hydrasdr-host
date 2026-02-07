/*
 * DDC ABSTRACTION LAYER API
 *
 * Provides unified interface for multiple DDC implementations.
 * Uses function pointer dispatch for easy extensibility.
 * New algorithms can be added without modifying core code.
 *
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef IQCONVERTER_H
#define IQCONVERTER_H

#include <stdint.h>

/* Valid halfband filter tap counts */
#define IQCONV_TAPS_FAST (33)       /* 33-tap fast filter */
#define IQCONV_TAPS_OPT (47)        /* 47-tap optimized filter */
#define IQCONV_TAPS_POW2 (65)       /* 65-tap power-of-2 delay filter */
#define IQCONV_TAPS_HIGHPERF (83)   /* 83-tap high performance filter */

/* Validate filter tap count (returns true if valid) */
#define IQCONV_VALID_TAPS(len) \
	((len) == IQCONV_TAPS_FAST || (len) == IQCONV_TAPS_OPT || \
	 (len) == IQCONV_TAPS_POW2 || (len) == IQCONV_TAPS_HIGHPERF)

/* ========================================================================
 * DECIMATION SUPPORT
 * ========================================================================
 */

/**
 * Decimation factor for DDC output
 *
 * Controls the output sample rate relative to the base DDC rate.
 * Higher decimation provides narrower bandwidth but lower sample rate.
 *
 * Example: For 10 MSPS base rate:
 *   IQCONV_DEC_1X  -> 10 MSPS output (no decimation)
 *   IQCONV_DEC_2X  -> 5 MSPS output
 *   IQCONV_DEC_4X  -> 2.5 MSPS output
 *   IQCONV_DEC_8X  -> 1.25 MSPS output
 *   IQCONV_DEC_16X -> 625 kSPS output
 *   IQCONV_DEC_32X -> 312.5 kSPS output
 *   IQCONV_DEC_64X -> 156.25 kSPS output
 */
typedef enum {
	IQCONV_DEC_1X = 1,   /* No decimation (current behavior) - Fs output */
	IQCONV_DEC_2X = 2,   /* 2:1 decimation (1 stage) - Fs/2 output */
	IQCONV_DEC_4X = 4,   /* 4:1 decimation (2 stages) - Fs/4 output */
	IQCONV_DEC_8X = 8,   /* 8:1 decimation (3 stages) - Fs/8 output */
	IQCONV_DEC_16X = 16, /* 16:1 decimation (4 stages) - Fs/16 output */
	IQCONV_DEC_32X = 32, /* 32:1 decimation (5 stages) - Fs/32 output */
	IQCONV_DEC_64X = 64  /* 64:1 decimation (6 stages) - Fs/64 output */
} iqconv_decimation_t;

/* Validate decimation factor */
#define IQCONV_VALID_DECIMATION(d) \
	((d) == IQCONV_DEC_1X || (d) == IQCONV_DEC_2X || \
	 (d) == IQCONV_DEC_4X || (d) == IQCONV_DEC_8X || \
	 (d) == IQCONV_DEC_16X || (d) == IQCONV_DEC_32X || \
	 (d) == IQCONV_DEC_64X)

/*
 * iqconverter is an internal API - not exported from DLL.
 * Tools using iqconverter directly must link against the static library.
 */
#define IQCONV_API
#define IQCONV_CALL

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declaration of opaque handle */
typedef struct iqconverter_s iqconverter_t;

/*
 * DDC Algorithm Identifier
 *
 * Each algorithm is uniquely identified by a string name.
 * Standard algorithms provided:
 *   "int16_legacy"   - Original Airspy int16 implementation
 *   "int16_opt"      - Optimized HydraSDR int16 implementation
 *   "float32_legacy" - Original Airspy float32 implementation
 *   "float32_opt"    - Optimized HydraSDR float32 implementation
 *
 * To add a new algorithm, simply register it with iqconverter_register().
 */

/*
 * Create DDC instance with specified algorithm
 *
 * Parameters:
 *   hb_kernel - Halfband filter coefficients (47 taps)
 *   len       - Filter length (must be 47)
 *   algorithm - Algorithm name (e.g., "int16_opt", "float32_opt")
 *
 * Returns:
 *   Pointer to DDC instance, or NULL on failure
 *
 * Example:
 *   iqconverter_t *ddc = iqconverter_create(kernel, 47, "float32_opt");
 */
extern IQCONV_API iqconverter_t * IQCONV_CALL iqconverter_create(
	const void *hb_kernel, int len, const char *algorithm);

/*
 * Create DDC instance with specified algorithm and decimation factor
 *
 * Parameters:
 *   hb_kernel   - Halfband filter coefficients for base DDC
 *   len         - Filter length (33, 47, 65, or 83)
 *   algorithm   - Algorithm name (e.g., "float32_opt_dec", "int16_opt_dec")
 *   decimation  - Decimation factor (IQCONV_DEC_1X/2X/4X/8X)
 *
 * Returns:
 *   Pointer to DDC instance, or NULL on failure
 *
 * Note: For decimation algorithms (ending in "_dec"), use this function.
 *       For non-decimation algorithms, decimation parameter is ignored.
 *
 * Example:
 *   // Create DDC with 4x decimation (Fs/4 output rate)
 *   iqconverter_t *ddc = iqconverter_create_ex(
 *       HB_KERNEL_FLOAT_47_OPT, 47, "float32_opt_dec", IQCONV_DEC_4X);
 */
extern IQCONV_API iqconverter_t * IQCONV_CALL iqconverter_create_ex(
	const void *hb_kernel, int len, const char *algorithm,
	iqconv_decimation_t decimation);

/*
 * Free DDC instance
 */
extern IQCONV_API void IQCONV_CALL iqconverter_free(iqconverter_t *cnv);

/*
 * Reset DDC state (clear history buffers)
 */
extern IQCONV_API void IQCONV_CALL iqconverter_reset(iqconverter_t *cnv);

/*
 * Process raw ADC samples through DDC
 *
 * Unified processing function that handles both optimized fused algorithms
 * and legacy algorithms transparently. For optimized algorithms, uses the
 * fused ADC->IQ path directly. For legacy algorithms, converts via Legacy 
 * conversion then processes in-place.
 *
 * Parameters:
 *   cnv  - DDC instance
 *   src  - Raw uint16_t ADC samples (input, unchanged)
 *   dest - Output buffer (float32 or int16 I/Q samples)
 *   len  - Number of samples (must be multiple of 4)
 *
 * Note: LUT is auto-initialized when creating fused algorithms (12-bit default)
 *
 * Example:
 *   iqconverter_t *ddc = iqconverter_create(kernel, 47, "float32_opt");
 *   iqconverter_process(ddc, adc_samples, iq_output, count);
 */
extern IQCONV_API void IQCONV_CALL iqconverter_process(
	iqconverter_t *cnv, const uint16_t *src, void *dest, int len);

/*
 * Process 8-bit raw ADC samples through DDC (for 8-bit ADC mode @ 40 MSPS)
 *
 * Parameters:
 *   cnv  - DDC instance
 *   src  - Raw uint8_t ADC samples (input, unchanged)
 *   dest - Output buffer (float32 or int16 I/Q samples)
 *   len  - Number of samples (must be multiple of 4)
 *
 * Note: Requires 8-bit LUT to be initialized via iqconv_lut_init(IQCONV_ADC_8BIT)
 */
extern IQCONV_API void IQCONV_CALL iqconverter_process_u8(
	iqconverter_t *cnv, const uint8_t *src, void *dest, int len);

/*
 * Get algorithm description string
 *
 * Returns:
 *   Human-readable description of the algorithm
 */
extern IQCONV_API const char * IQCONV_CALL iqconverter_get_info(
	iqconverter_t *cnv);

/*
 * Get algorithm name
 *
 * Returns:
 *   Algorithm identifier string (e.g., "float32_opt")
 */
extern IQCONV_API const char * IQCONV_CALL iqconverter_get_algorithm(
	iqconverter_t *cnv);

/*
 * Get decimation factor
 *
 * Returns:
 *   Decimation factor (1, 2, 4, or 8), or 0 if not applicable
 */
extern IQCONV_API int IQCONV_CALL iqconverter_get_decimation(
	iqconverter_t *cnv);

/*
 * Calculate output length for given input length
 *
 * Parameters:
 *   cnv       - DDC instance
 *   input_len - Number of input samples
 *
 * Returns:
 *   Number of output I/Q pairs (input_len / decimation_factor)
 *   For non-decimation algorithms, returns input_len.
 */
extern IQCONV_API int IQCONV_CALL iqconverter_get_output_len(
	iqconverter_t *cnv, int input_len);

/* ========================================================================
 * ALGORITHM REGISTRATION API (for adding new implementations)
 * ========================================================================
 */

/*
 * DDC Algorithm Virtual Function Table
 *
 * Each DDC implementation provides these function pointers.
 * This allows new algorithms to be added without modifying core code.
 */
typedef struct {
	/* Algorithm metadata */
	const char *name;        /* Unique identifier (e.g., "float32_opt") */
	const char *description; /* Human-readable description */

	/* Algorithm operations */
	void *(*create)(const void *hb_kernel, int len);
	void (*free)(void *ctx);
	void (*reset)(void *ctx);
	void (*process)(void *ctx, void *samples, int len);
	void (*process_u16)(void *ctx, const void *src, void *dest, int len);
	void (*process_u8)(void *ctx, const void *src, void *dest, int len);  /* 8-bit ADC input */
} iqconv_algorithm_t;

/*
 * Register a new DDC algorithm
 *
 * Parameters:
 *   algo - Algorithm vtable with function pointers
 *
 * Returns:
 *   0 on success, -1 on failure (name collision or table full)
 *
 * Example:
 *   static void *my_create(const void *k, int l) { ... }
 *   static void my_free(void *c) { ... }
 *   static void my_reset(void *c) { ... }
 *   static void my_process(void *c, void *s, int l) { ... }
 *
 *   iqconv_algorithm_t my_algo = {
 *       .name = "my_custom_ddc",
 *       .description = "My Custom DDC Algorithm",
 *       .create = my_create,
 *       .free = my_free,
 *       .reset = my_reset,
 *       .process = my_process
 *   };
 *
 *   iqconverter_register(&my_algo);
 */
extern IQCONV_API int IQCONV_CALL iqconverter_register(
	const iqconv_algorithm_t *algo);

/*
 * List all registered algorithms
 *
 * Parameters:
 *   names  - Array to store algorithm name pointers
 *   max    - Maximum number of names to return
 *
 * Returns:
 *   Number of algorithms returned
 *
 * Example:
 *   const char *names[16];
 *   int count = iqconverter_list_algorithms(names, 16);
 *   for (int i = 0; i < count; i++)
 *       printf("%s\n", names[i]);
 */
extern IQCONV_API int IQCONV_CALL iqconverter_list_algorithms(
	const char **names, int max);

#ifdef __cplusplus
}
#endif

#endif /* IQCONVERTER_H */
