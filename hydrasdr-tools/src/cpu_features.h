/*
 * CPU FEATURE DETECTION
 *
 * Runtime detection of CPU capabilities for optimal code path selection.
 * Supports: Windows, Linux, macOS on x86-64, ARM64, ARM32
 *
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef CPU_FEATURES_H
#define CPU_FEATURES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CPU feature flags */
typedef struct {
	/* x86/x86-64 features */
	uint32_t sse2      : 1;
	uint32_t sse3      : 1;
	uint32_t ssse3     : 1;
	uint32_t sse41     : 1;
	uint32_t sse42     : 1;
	uint32_t avx       : 1;
	uint32_t avx2      : 1;
	uint32_t avx512f   : 1;
	uint32_t avx512vl  : 1;
	uint32_t fma       : 1;
	uint32_t popcnt    : 1;
	uint32_t bmi1      : 1;
	uint32_t bmi2      : 1;

	/* ARM features */
	uint32_t neon      : 1;
	uint32_t sve       : 1;
	uint32_t sve2      : 1;

	/* Reserved for future use */
	uint32_t reserved  : 16;
} cpu_features_t;

/* CPU architecture enum */
typedef enum {
	CPU_ARCH_UNKNOWN = 0,
	CPU_ARCH_X86,
	CPU_ARCH_X86_64,
	CPU_ARCH_ARM32,
	CPU_ARCH_ARM64,
	CPU_ARCH_RISCV
} cpu_arch_t;

/* SIMD capability level (for easy comparison) */
typedef enum {
	SIMD_NONE = 0,
	/* x86-64 levels */
	SIMD_X64_SSE2,        /* x86-64 baseline */
	SIMD_X64_SSE42,       /* Nehalem 2008+ */
	SIMD_X64_AVX,         /* Sandy Bridge 2011+ */
	SIMD_X64_AVX2,        /* Haswell 2013+ */
	SIMD_X64_AVX512,      /* Skylake-X 2017+ */
	/* ARM64 levels */
	SIMD_ARM64_NEON,        /* ARM NEON (always on ARM64) */
	SIMD_ARM64_SVE,         /* ARM SVE */
	SIMD_ARM64_SVE2         /* ARM SVE2 */
} simd_level_t;

/*
 * Detect CPU features (cached after first call)
 * Thread-safe, can be called from multiple threads
 */
const cpu_features_t *cpu_features_detect(void);

/*
 * Get CPU architecture
 */
cpu_arch_t cpu_get_arch(void);

/*
 * Get highest SIMD level supported
 */
simd_level_t cpu_get_simd_level(void);

/*
 * Get CPU architecture name as string
 */
const char *cpu_get_arch_name(void);

/*
 * Get SIMD level name as string
 */
const char *cpu_get_simd_name(void);

/*
 * Check if specific feature is available
 */
static inline int cpu_has_sse2(void)    { return cpu_features_detect()->sse2; }
static inline int cpu_has_sse42(void)   { return cpu_features_detect()->sse42; }
static inline int cpu_has_avx(void)     { return cpu_features_detect()->avx; }
static inline int cpu_has_avx2(void)    { return cpu_features_detect()->avx2; }
static inline int cpu_has_avx512(void)  { return cpu_features_detect()->avx512f; }
static inline int cpu_has_fma(void)     { return cpu_features_detect()->fma; }
static inline int cpu_has_neon(void)    { return cpu_features_detect()->neon; }
static inline int cpu_has_sve(void)     { return cpu_features_detect()->sve; }
static inline int cpu_has_sve2(void)    { return cpu_features_detect()->sve2; }

#ifdef __cplusplus
}
#endif

#endif /* CPU_FEATURES_H */
