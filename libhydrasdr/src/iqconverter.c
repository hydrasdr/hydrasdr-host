/*
 * DDC ABSTRACTION LAYER IMPLEMENTATION
 *
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "iqconverter.h"
#include "iqconverter_float.h"
#include "iqconverter_int16.h"
#include "iqconverter_float_opt.h"
#include "iqconverter_int16_opt.h"
#include "iqconverter_float_opt33.h"
#include "iqconverter_int16_opt33.h"
#include "iqconverter_float_opt65.h"
#include "iqconverter_int16_opt65.h"
#include "iqconverter_float_opt83.h"
#include "iqconverter_int16_opt83.h"
#include "iqconverter_float_opt_dec.h"
#include "iqconverter_int16_opt_dec.h"
#include "iqconverter_lut.h"
#include "compat_opt.h"
#include <stdlib.h>
#include <string.h>

/* Maximum number of registered algorithms */
#define MAX_ALGORITHMS 24

/* Initialization state (NOT thread-safe - call from main thread only) */
#define INIT_STATE_NONE 0
#define INIT_STATE_DONE 1

/* Algorithm registry */
static const iqconv_algorithm_t *algorithm_registry[MAX_ALGORITHMS];
static int num_algorithms = 0;

/* Main DDC structure - holds vtable and context */
struct iqconverter_s {
	const iqconv_algorithm_t *algo;  /* Virtual function table */
	void *ctx;                       /* Algorithm-specific context */
	int decimation;                  /* Decimation factor (1, 2, 4, 8, 16, or 32) */
	int is_dec_algo;                 /* Non-zero if decimation algorithm */
};

/* ========================================================================
 * BUILT-IN ALGORITHM DEFINITIONS (Direct Function Pointers)
 * ========================================================================
 */

/*
 * Cast helper macros for type-safe function pointer conversion
 *
 * These casts are safe because:
 * - All pointer types have the same size and representation in C
 * - We're only casting away type information, not changing behavior
 * - The actual functions are called with correct types at runtime
 * - This is a standard C technique for polymorphism
 */
#define CREATE_FN(fn)      ((void *(*)(const void *, int))(fn))
#define FREE_FN(fn)        ((void (*)(void *))(fn))
#define RESET_FN(fn)       ((void (*)(void *))(fn))
#define PROCESS_FN(fn)     ((void (*)(void *, void *, int))(fn))
#define PROCESS_U16_FN(fn) ((void (*)(void *, const void *, void *, int))(fn))
#define PROCESS_U8_FN(fn)  ((void (*)(void *, const void *, void *, int))(fn))

/* Int16 Legacy Algorithm */
static const iqconv_algorithm_t algo_int16_legacy = {
	.name = "int16_legacy",
	.description = "Int16 Legacy (Airspy Reference)",
	.create = CREATE_FN(iqconverter_int16_create),
	.free = FREE_FN(iqconverter_int16_free),
	.reset = RESET_FN(iqconverter_int16_reset),
	.process = PROCESS_FN(iqconverter_int16_process),
	.process_u16 = NULL,  /* Legacy has no fused processing */
	.process_u8 = NULL
};

/* Int16 Optimized Algorithm */
static const iqconv_algorithm_t algo_int16_opt = {
	.name = "int16_opt",
	.description = "Int16 Optimized (HydraSDR Fused Processing)",
	.create = CREATE_FN(iqconverter_int16_opt_create),
	.free = FREE_FN(iqconverter_int16_opt_free),
	.reset = RESET_FN(iqconverter_int16_opt_reset),
	.process = NULL,  /* Fused-only: use process_u16 */
	.process_u16 = PROCESS_U16_FN(iqconverter_int16_opt_process_u16),
	.process_u8 = PROCESS_U8_FN(iqconverter_int16_opt_process_u8)
};

/* Float32 Legacy Algorithm */
static const iqconv_algorithm_t algo_float32_legacy = {
	.name = "float32_legacy",
	.description = "Float32 Legacy (Airspy Reference)",
	.create = CREATE_FN(iqconverter_float_create),
	.free = FREE_FN(iqconverter_float_free),
	.reset = RESET_FN(iqconverter_float_reset),
	.process = PROCESS_FN(iqconverter_float_process),
	.process_u16 = NULL,  /* Legacy has no fused processing */
	.process_u8 = NULL
};

/* Float32 Optimized Algorithm */
static const iqconv_algorithm_t algo_float32_opt = {
	.name = "float32_opt",
	.description = "Float32 Optimized (HydraSDR Fused Processing)",
	.create = CREATE_FN(iqconverter_float_opt_create),
	.free = FREE_FN(iqconverter_float_opt_free),
	.reset = RESET_FN(iqconverter_float_opt_reset),
	.process = NULL,  /* Fused-only: use process_u16 */
	.process_u16 = PROCESS_U16_FN(iqconverter_float_opt_process_u16),
	.process_u8 = PROCESS_U8_FN(iqconverter_float_opt_process_u8)
};

/* Int16 Optimized 33-tap Algorithm (power-of-2 delay) */
static const iqconv_algorithm_t algo_int16_opt33 = {
	.name = "int16_opt33",
	.description = "Int16 Optimized 33-tap (Power-of-2 Delay)",
	.create = CREATE_FN(iqconverter_int16_opt33_create),
	.free = FREE_FN(iqconverter_int16_opt33_free),
	.reset = RESET_FN(iqconverter_int16_opt33_reset),
	.process = NULL,  /* Fused-only: use process_u16 */
	.process_u16 = PROCESS_U16_FN(iqconverter_int16_opt33_process_u16),
	.process_u8 = NULL
};

/* Float32 Optimized 33-tap Algorithm (power-of-2 delay) */
static const iqconv_algorithm_t algo_float32_opt33 = {
	.name = "float32_opt33",
	.description = "Float32 Optimized 33-tap (Power-of-2 Delay)",
	.create = CREATE_FN(iqconverter_float_opt33_create),
	.free = FREE_FN(iqconverter_float_opt33_free),
	.reset = RESET_FN(iqconverter_float_opt33_reset),
	.process = NULL,  /* Fused-only: use process_u16 */
	.process_u16 = PROCESS_U16_FN(iqconverter_float_opt33_process_u16),
	.process_u8 = NULL
};

/* Int16 Optimized 65-tap Algorithm (power-of-2 delay) */
static const iqconv_algorithm_t algo_int16_opt65 = {
	.name = "int16_opt65",
	.description = "Int16 Optimized 65-tap (Power-of-2 Delay)",
	.create = CREATE_FN(iqconverter_int16_opt65_create),
	.free = FREE_FN(iqconverter_int16_opt65_free),
	.reset = RESET_FN(iqconverter_int16_opt65_reset),
	.process = NULL,  /* Fused-only: use process_u16 */
	.process_u16 = PROCESS_U16_FN(iqconverter_int16_opt65_process_u16),
	.process_u8 = NULL
};

/* Float32 Optimized 65-tap Algorithm (power-of-2 delay) */
static const iqconv_algorithm_t algo_float32_opt65 = {
	.name = "float32_opt65",
	.description = "Float32 Optimized 65-tap (Power-of-2 Delay)",
	.create = CREATE_FN(iqconverter_float_opt65_create),
	.free = FREE_FN(iqconverter_float_opt65_free),
	.reset = RESET_FN(iqconverter_float_opt65_reset),
	.process = NULL,  /* Fused-only: use process_u16 */
	.process_u16 = PROCESS_U16_FN(iqconverter_float_opt65_process_u16),
	.process_u8 = NULL
};

/* Int16 Optimized 83-tap Algorithm (high performance) */
static const iqconv_algorithm_t algo_int16_opt83 = {
	.name = "int16_opt83",
	.description = "Int16 Optimized 83-tap (High Dynamic Range)",
	.create = CREATE_FN(iqconverter_int16_opt83_create),
	.free = FREE_FN(iqconverter_int16_opt83_free),
	.reset = RESET_FN(iqconverter_int16_opt83_reset),
	.process = NULL,  /* Fused-only: use process_u16 */
	.process_u16 = PROCESS_U16_FN(iqconverter_int16_opt83_process_u16),
	.process_u8 = NULL
};

/* Float32 Optimized 83-tap Algorithm (high performance) */
static const iqconv_algorithm_t algo_float32_opt83 = {
	.name = "float32_opt83",
	.description = "Float32 Optimized 83-tap (High Dynamic Range)",
	.create = CREATE_FN(iqconverter_float_opt83_create),
	.free = FREE_FN(iqconverter_float_opt83_free),
	.reset = RESET_FN(iqconverter_float_opt83_reset),
	.process = NULL,  /* Fused-only: use process_u16 */
	.process_u16 = PROCESS_U16_FN(iqconverter_float_opt83_process_u16),
	.process_u8 = NULL
};

/* ========================================================================
 * DECIMATION ALGORITHM DEFINITIONS
 *
 * These algorithms support configurable decimation (1x, 2x, 4x, 8x, 16x, 32x, 64x).
 * Use iqconverter_create_ex() to specify decimation factor.
 * ========================================================================
 */

/*
 * Float32 Decimation Algorithm
 * Note: The actual decimation factor is set via iqconverter_create_ex().
 * This registration uses a dummy create function - actual creation
 * happens through the extended API.
 */
static void *float32_opt_dec_create_dummy(const void *hb_kernel, int len)
{
	/* Default to 1x decimation when created via iqconverter_create() */
	return iqconverter_float_opt_dec_create((const float *)hb_kernel, len, 1);
}

static const iqconv_algorithm_t algo_float32_opt_dec = {
	.name = "float32_opt_dec",
	.description = "Float32 Optimized with Decimation (1x to 64x)",
	.create = CREATE_FN(float32_opt_dec_create_dummy),
	.free = FREE_FN(iqconverter_float_opt_dec_free),
	.reset = RESET_FN(iqconverter_float_opt_dec_reset),
	.process = NULL,
	.process_u16 = PROCESS_U16_FN(iqconverter_float_opt_dec_process_u16),
	.process_u8 = NULL  /* TODO: Add 8-bit decimation support */
};

/*
 * Int16 Decimation Algorithm
 */
static void *int16_opt_dec_create_dummy(const void *hb_kernel, int len)
{
	/* Default to 1x decimation when created via iqconverter_create() */
	return iqconverter_int16_opt_dec_create((const int16_t *)hb_kernel, len, 1);
}

static const iqconv_algorithm_t algo_int16_opt_dec = {
	.name = "int16_opt_dec",
	.description = "Int16 Optimized with Decimation (1x to 64x)",
	.create = CREATE_FN(int16_opt_dec_create_dummy),
	.free = FREE_FN(iqconverter_int16_opt_dec_free),
	.reset = RESET_FN(iqconverter_int16_opt_dec_reset),
	.process = NULL,
	.process_u16 = PROCESS_U16_FN(iqconverter_int16_opt_dec_process_u16),
	.process_u8 = NULL  /* TODO: Add 8-bit decimation support */
};

/* ========================================================================
 * ALGORITHM REGISTRATION
 * ========================================================================
 */

/* Initialization state */
static int init_state = INIT_STATE_NONE;

/*
 * Initialize built-in algorithms on first use
 *
 * NOTE: NOT thread-safe. This function should only be called from the main
 * thread during library initialization. It is automatically called by
 * iqconverter_create() and iqconverter_list_algorithms().
 */
static void init_builtin_algorithms(void)
{
	if (init_state == INIT_STATE_DONE)
		return;

	/* Register built-in algorithms */
	iqconverter_register(&algo_int16_legacy);
	iqconverter_register(&algo_int16_opt);
	iqconverter_register(&algo_float32_legacy);
	iqconverter_register(&algo_float32_opt);
	iqconverter_register(&algo_int16_opt33);
	iqconverter_register(&algo_float32_opt33);
	iqconverter_register(&algo_int16_opt65);
	iqconverter_register(&algo_float32_opt65);
	iqconverter_register(&algo_int16_opt83);
	iqconverter_register(&algo_float32_opt83);

	/* Register decimation algorithms */
	iqconverter_register(&algo_float32_opt_dec);
	iqconverter_register(&algo_int16_opt_dec);

	init_state = INIT_STATE_DONE;
}

IQCONV_API int IQCONV_CALL iqconverter_register(const iqconv_algorithm_t *algo)
{
	int i;

	/* Require name, create, free, reset; at least one of process/process_u16 */
	if (!algo || !algo->name || !algo->create || !algo->free ||
	    !algo->reset || (!algo->process && !algo->process_u16)) {
		return -1;
	}

	/* Check for duplicate name */
	for (i = 0; i < num_algorithms; i++) {
		if (strcmp(algorithm_registry[i]->name, algo->name) == 0)
			return -1; /* Name collision */
	}

	/* Check if registry is full */
	if (num_algorithms >= MAX_ALGORITHMS)
		return -1;

	/* Register algorithm */
	algorithm_registry[num_algorithms++] = algo;
	return 0;
}

/*
 * Find algorithm by name
 */
static const iqconv_algorithm_t *find_algorithm(const char *name)
{
	int i;

	if (!name)
		return NULL;

	for (i = 0; i < num_algorithms; i++) {
		if (strcmp(algorithm_registry[i]->name, name) == 0)
			return algorithm_registry[i];
	}

	return NULL;
}

IQCONV_API int IQCONV_CALL iqconverter_list_algorithms(const char **names, int max)
{
	int i;
	int count = 0;

	init_builtin_algorithms();

	for (i = 0; i < num_algorithms && count < max; i++) {
		names[count++] = algorithm_registry[i]->name;
	}

	return count;
}

/* ========================================================================
 * PUBLIC API IMPLEMENTATION
 * ========================================================================
 */

/*
 * Helper to check if algorithm name ends with "_dec"
 */
static int is_decimation_algorithm(const char *name)
{
	size_t len;
	if (!name)
		return 0;
	len = strlen(name);
	if (len < 4)
		return 0;
	return (strcmp(name + len - 4, "_dec") == 0);
}

IQCONV_API iqconverter_t * IQCONV_CALL iqconverter_create(
	const void *hb_kernel, int len, const char *algorithm)
{
	/* Delegate to create_ex with default 1x decimation */
	return iqconverter_create_ex(hb_kernel, len, algorithm, IQCONV_DEC_1X);
}

IQCONV_API iqconverter_t * IQCONV_CALL iqconverter_create_ex(
	const void *hb_kernel, int len, const char *algorithm,
	iqconv_decimation_t decimation)
{
	iqconverter_t *cnv;
	const iqconv_algorithm_t *algo;
	int is_dec_algo;

	/* Validate parameters */
	if (!hb_kernel || !algorithm)
		return NULL;

	/* Check for valid filter lengths: 33, 47, 65, 83 tap halfband filters */
	if (!IQCONV_VALID_TAPS(len))
		return NULL;

	/* Validate decimation factor */
	if (!IQCONV_VALID_DECIMATION(decimation))
		return NULL;

	/* Initialize built-in algorithms */
	init_builtin_algorithms();

	/* Find requested algorithm */
	algo = find_algorithm(algorithm);
	if (!algo)
		return NULL; /* Algorithm not found */

	/*
	 * Auto-initialize LUT for fused algorithms (those with process_u16).
	 * Default to 12-bit ADC which is standard for HydraSDR/Airspy hardware.
	 * This ensures process_u16() works without manual LUT initialization.
	 */
	if (algo->process_u16 && !iqconv_lut_is_initialized()) {
		if (iqconv_lut_init(IQCONV_ADC_12BIT) != 0)
			return NULL; /* LUT allocation failed */
	}

	/* Allocate main structure */
	cnv = (iqconverter_t *)malloc(sizeof(iqconverter_t));
	if (!cnv)
		return NULL;

	/* Check if this is a decimation algorithm */
	is_dec_algo = is_decimation_algorithm(algorithm);

	/* Create algorithm-specific context */
	cnv->algo = algo;

	if (is_dec_algo) {
		/* For decimation algorithms, create with specified decimation */
		if (strstr(algorithm, "float32")) {
			cnv->ctx = iqconverter_float_opt_dec_create(
				(const float *)hb_kernel, len, (int)decimation);
		} else {
			cnv->ctx = iqconverter_int16_opt_dec_create(
				(const int16_t *)hb_kernel, len, (int)decimation);
		}
		cnv->decimation = (int)decimation;
		cnv->is_dec_algo = 1;
	} else {
		/* For regular algorithms, use standard create */
		cnv->ctx = algo->create(hb_kernel, len);
		cnv->decimation = 1;  /* No decimation */
		cnv->is_dec_algo = 0;
	}

	if (!cnv->ctx) {
		free(cnv);
		return NULL;
	}

	return cnv;
}

IQCONV_API void IQCONV_CALL iqconverter_free(iqconverter_t *cnv)
{
	if (!cnv)
		return;

	if (cnv->algo && cnv->ctx)
		cnv->algo->free(cnv->ctx);

	free(cnv);
}

IQCONV_API void IQCONV_CALL iqconverter_reset(iqconverter_t *cnv)
{
	if (!cnv || !cnv->algo || !cnv->ctx)
		return;

	cnv->algo->reset(cnv->ctx);
}

/*
 * Helper to check if algorithm is float32 type (vs int16)
 */
static int is_float32_algorithm(const char *name)
{
	if (!name)
		return 0;
	return (strstr(name, "float32") != NULL);
}


IQCONV_API void IQCONV_CALL iqconverter_process(iqconverter_t *cnv,
						const uint16_t *src,
						void *dest, int len)
{
	if (!cnv || !cnv->algo || !cnv->ctx || !src || !dest)
		return;

	/* Path 1: Fused processing (optimized algorithms) */
	if (cnv->algo->process_u16) {
		cnv->algo->process_u16(cnv->ctx, src, dest, len);
		return;
	}

	/* Path 2: Legacy fallback - use LUT for ADC conversion then process in-place */
	if (cnv->algo->process) {
		int i;

		if (is_float32_algorithm(cnv->algo->name)) {
			/* Float32 legacy: convert via precomputed LUT */
			float *out = (float *)dest;

			for (i = 0; i < len; i += 4) {
				out[i + 0] = IQCONV_LUT_F32(src[i + 0]);
				out[i + 1] = IQCONV_LUT_F32(src[i + 1]);
				out[i + 2] = IQCONV_LUT_F32(src[i + 2]);
				out[i + 3] = IQCONV_LUT_F32(src[i + 3]);
			}

			/* Process in-place */
			cnv->algo->process(cnv->ctx, dest, len);
		} else {
			/* Int16 legacy: convert via precomputed LUT */
			int16_t *out = (int16_t *)dest;

			for (i = 0; i < len; i += 4) {
				out[i + 0] = IQCONV_LUT_I16(src[i + 0]);
				out[i + 1] = IQCONV_LUT_I16(src[i + 1]);
				out[i + 2] = IQCONV_LUT_I16(src[i + 2]);
				out[i + 3] = IQCONV_LUT_I16(src[i + 3]);
			}

			/* Process in-place */
			cnv->algo->process(cnv->ctx, dest, len);
		}
	}
}

IQCONV_API void IQCONV_CALL iqconverter_process_u8(iqconverter_t *cnv,
						   const uint8_t *src,
						   void *dest, int len)
{
	if (!cnv || !cnv->algo || !cnv->ctx || !src || !dest)
		return;

	/* Validate LUT is initialized for 8-bit ADC */
	if (iqconv_lut.adc_bits != IQCONV_ADC_8BIT)
		return;

	/* Path 1: Fused processing (optimized algorithms with 8-bit support) */
	if (cnv->algo->process_u8) {
		cnv->algo->process_u8(cnv->ctx, src, dest, len);
		return;
	}

	/*
	 * Path 2: Legacy fallback - use LUT for ADC conversion then process in-place
	 * Note: LUT must be initialized with IQCONV_ADC_8BIT before calling this
	 */
	if (cnv->algo->process) {
		int i;

		if (is_float32_algorithm(cnv->algo->name)) {
			/* Float32 legacy: convert via precomputed LUT */
			float *out = (float *)dest;

			for (i = 0; i < len; i += 4) {
				out[i + 0] = IQCONV_LUT_F32(src[i + 0]);
				out[i + 1] = IQCONV_LUT_F32(src[i + 1]);
				out[i + 2] = IQCONV_LUT_F32(src[i + 2]);
				out[i + 3] = IQCONV_LUT_F32(src[i + 3]);
			}

			/* Process in-place */
			cnv->algo->process(cnv->ctx, dest, len);
		} else {
			/* Int16 legacy: convert via precomputed LUT */
			int16_t *out = (int16_t *)dest;

			for (i = 0; i < len; i += 4) {
				out[i + 0] = IQCONV_LUT_I16(src[i + 0]);
				out[i + 1] = IQCONV_LUT_I16(src[i + 1]);
				out[i + 2] = IQCONV_LUT_I16(src[i + 2]);
				out[i + 3] = IQCONV_LUT_I16(src[i + 3]);
			}

			/* Process in-place */
			cnv->algo->process(cnv->ctx, dest, len);
		}
	}
}

IQCONV_API const char * IQCONV_CALL iqconverter_get_info(iqconverter_t *cnv)
{
	if (!cnv || !cnv->algo)
		return "Invalid";

	return cnv->algo->description ? cnv->algo->description : cnv->algo->name;
}

IQCONV_API const char * IQCONV_CALL iqconverter_get_algorithm(iqconverter_t *cnv)
{
	if (!cnv || !cnv->algo)
		return "Invalid";

	return cnv->algo->name;
}

IQCONV_API int IQCONV_CALL iqconverter_get_decimation(iqconverter_t *cnv)
{
	if (!cnv)
		return 0;

	return cnv->decimation;
}

IQCONV_API int IQCONV_CALL iqconverter_get_output_len(iqconverter_t *cnv,
						      int input_len)
{
	if (!cnv || cnv->decimation == 0)
		return 0;

	return input_len / cnv->decimation;
}
