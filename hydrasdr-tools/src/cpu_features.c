/*
 * CPU FEATURE DETECTION IMPLEMENTATION
 *
 * Runtime detection of CPU capabilities for optimal code path selection.
 * Supports: Windows, Linux, macOS on x86-64, ARM64, ARM32
 *
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "cpu_features.h"
#include <string.h>

/* Platform includes */
#if defined(_WIN32)
#include <windows.h>
#endif

/* CPUID support for x86/x86-64 */
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
#include <intrin.h>
#define HAVE_CPUID 1
#elif (defined(__GNUC__) || defined(__clang__)) && (defined(__x86_64__) || defined(__i386__))
#include <cpuid.h>
#define HAVE_CPUID 1
#endif

/* ARM feature detection on Linux */
#if defined(__linux__) && (defined(__aarch64__) || defined(__arm__))
#include <sys/auxv.h>
#include <asm/hwcap.h>
#endif

/* macOS ARM feature detection */
#if defined(__APPLE__) && defined(__aarch64__)
#include <sys/sysctl.h>
#endif

/* Static storage for detected features */
static cpu_features_t g_cpu_features;

/*
 * Thread-safe initialization state:
 * 0 = uninitialized
 * 1 = initializing (one thread is working)
 * 2 = initialized (done)
 */
static volatile int g_cpu_features_state = 0;

/* Thread-safe initialization using atomic compare-and-swap */
#if defined(_MSC_VER)
#include <intrin.h>
#define MEMORY_BARRIER() _ReadWriteBarrier()
#define ATOMIC_CAS(ptr, expected, desired) \
	(InterlockedCompareExchange((volatile long*)(ptr), (desired), (expected)) == (expected))
#define ATOMIC_LOAD(ptr) InterlockedCompareExchange((volatile long*)(ptr), 0, 0)
#elif defined(__GNUC__) || defined(__clang__)
#define MEMORY_BARRIER() __atomic_thread_fence(__ATOMIC_SEQ_CST)
#define ATOMIC_CAS(ptr, expected, desired) \
	__sync_bool_compare_and_swap((ptr), (expected), (desired))
#define ATOMIC_LOAD(ptr) __atomic_load_n((ptr), __ATOMIC_ACQUIRE)
#else
#define MEMORY_BARRIER() ((void)0)
#define ATOMIC_CAS(ptr, expected, desired) ((*(ptr) == (expected)) ? (*(ptr) = (desired), 1) : 0)
#define ATOMIC_LOAD(ptr) (*(ptr))
#endif

/*
 * x86/x86-64 CPUID-based detection
 */
#if defined(HAVE_CPUID)
static void detect_x86_features(cpu_features_t *features)
{
	unsigned int eax = 0, ebx = 0, ecx = 0, edx = 0;
	unsigned int max_level = 0;

#if defined(_MSC_VER)
	int cpuinfo[4];

	/* Get maximum supported CPUID level */
	__cpuid(cpuinfo, 0);
	max_level = cpuinfo[0];

	if (max_level >= 1) {
		__cpuid(cpuinfo, 1);
		ecx = cpuinfo[2];
		edx = cpuinfo[3];

		features->sse2   = (edx >> 26) & 1;
		features->sse3   = (ecx >> 0) & 1;
		features->ssse3  = (ecx >> 9) & 1;
		features->sse41  = (ecx >> 19) & 1;
		features->sse42  = (ecx >> 20) & 1;
		features->popcnt = (ecx >> 23) & 1;
		features->avx    = (ecx >> 28) & 1;
		features->fma    = (ecx >> 12) & 1;
	}

	if (max_level >= 7) {
		__cpuidex(cpuinfo, 7, 0);
		ebx = cpuinfo[1];
		features->avx2     = (ebx >> 5) & 1;
		features->bmi1     = (ebx >> 3) & 1;
		features->bmi2     = (ebx >> 8) & 1;
		features->avx512f  = (ebx >> 16) & 1;
		features->avx512vl = (ebx >> 31) & 1;
	}

#else /* GCC/Clang */
	if (__get_cpuid(0, &eax, &ebx, &ecx, &edx)) {
		max_level = eax;

		if (max_level >= 1) {
			__get_cpuid(1, &eax, &ebx, &ecx, &edx);
			features->sse2   = (edx >> 26) & 1;
			features->sse3   = (ecx >> 0) & 1;
			features->ssse3  = (ecx >> 9) & 1;
			features->sse41  = (ecx >> 19) & 1;
			features->sse42  = (ecx >> 20) & 1;
			features->popcnt = (ecx >> 23) & 1;
			features->avx    = (ecx >> 28) & 1;
			features->fma    = (ecx >> 12) & 1;
		}

		if (max_level >= 7) {
			__cpuid_count(7, 0, eax, ebx, ecx, edx);
			features->avx2     = (ebx >> 5) & 1;
			features->bmi1     = (ebx >> 3) & 1;
			features->bmi2     = (ebx >> 8) & 1;
			features->avx512f  = (ebx >> 16) & 1;
			features->avx512vl = (ebx >> 31) & 1;
		}
	}
#endif

	/*
	 * Additional OS support check for AVX/AVX2/AVX-512
	 * The CPU may support these but the OS might not save/restore
	 * the extended registers (XMM/YMM/ZMM) on context switch.
	 */
#if defined(_MSC_VER)
	/* Check OSXSAVE bit and verify OS support via XGETBV */
	if (features->avx) {
		int cpuinfo_1[4];
		__cpuid(cpuinfo_1, 1);
		int osxsave = (cpuinfo_1[2] >> 27) & 1;
		if (osxsave) {
			unsigned long long xcr0 = _xgetbv(0);
			/* Check if OS saves XMM (bit 1) and YMM (bit 2) */
			if ((xcr0 & 0x6) != 0x6) {
				features->avx = 0;
				features->avx2 = 0;
				features->fma = 0;
			}
			/* Check if OS saves ZMM (bits 5,6,7) for AVX-512 */
			if ((xcr0 & 0xE0) != 0xE0) {
				features->avx512f = 0;
				features->avx512vl = 0;
			}
		} else {
			features->avx = 0;
			features->avx2 = 0;
			features->avx512f = 0;
			features->avx512vl = 0;
			features->fma = 0;
		}
	}
#elif defined(__GNUC__) || defined(__clang__)
	/* GCC/Clang: Check OSXSAVE and XGETBV */
	if (features->avx) {
		unsigned int eax_1 = 0, ebx_1 = 0, ecx_1 = 0, edx_1 = 0;
		__get_cpuid(1, &eax_1, &ebx_1, &ecx_1, &edx_1);
		int osxsave = (ecx_1 >> 27) & 1;
		if (osxsave) {
			unsigned int xcr0_lo, xcr0_hi;
			__asm__ volatile("xgetbv" : "=a"(xcr0_lo), "=d"(xcr0_hi) : "c"(0));
			/* Check if OS saves XMM (bit 1) and YMM (bit 2) */
			if ((xcr0_lo & 0x6) != 0x6) {
				features->avx = 0;
				features->avx2 = 0;
				features->fma = 0;
			}
			/* Check if OS saves ZMM (bits 5,6,7) for AVX-512 */
			if ((xcr0_lo & 0xE0) != 0xE0) {
				features->avx512f = 0;
				features->avx512vl = 0;
			}
		} else {
			features->avx = 0;
			features->avx2 = 0;
			features->avx512f = 0;
			features->avx512vl = 0;
			features->fma = 0;
		}
	}
#endif
}
#endif /* HAVE_CPUID */

/*
 * ARM64 feature detection
 */
#if defined(__aarch64__) || defined(_M_ARM64)
static void detect_arm64_features(cpu_features_t *features)
{
	/* NEON is mandatory on ARM64 */
	features->neon = 1;

#if defined(__linux__)
	/* Linux: Use getauxval for feature detection */
	unsigned long hwcap = getauxval(AT_HWCAP);
	unsigned long hwcap2 = getauxval(AT_HWCAP2);

	/* SVE detection */
#ifdef HWCAP_SVE
	features->sve = (hwcap & HWCAP_SVE) ? 1 : 0;
#endif

	/* SVE2 detection */
#ifdef HWCAP2_SVE2
	features->sve2 = (hwcap2 & HWCAP2_SVE2) ? 1 : 0;
#endif

	/* Suppress unused warnings if HWCAP macros not defined */
	(void)hwcap;
	(void)hwcap2;

#elif defined(__APPLE__)
	/* macOS: All Apple Silicon has NEON, no SVE yet */
	features->neon = 1;
	features->sve = 0;
	features->sve2 = 0;

#elif defined(_WIN32)
	/* Windows ARM64: NEON is mandatory */
	features->neon = 1;
	/* SVE detection on Windows ARM64 - not widely available yet */
	features->sve = 0;
	features->sve2 = 0;
#endif
}
#endif /* ARM64 */

/*
 * ARM32 feature detection
 */
#if defined(__arm__) || defined(_M_ARM)
static void detect_arm32_features(cpu_features_t *features)
{
#if defined(__linux__)
	/* Linux: Use getauxval for NEON detection */
	unsigned long hwcap = getauxval(AT_HWCAP);

#ifdef HWCAP_NEON
	features->neon = (hwcap & HWCAP_NEON) ? 1 : 0;
#else
	features->neon = 0;
	(void)hwcap; /* Suppress unused warning if HWCAP_NEON not defined */
#endif

#elif defined(__APPLE__)
	/* iOS: All supported iOS devices have NEON */
	features->neon = 1;

#elif defined(_WIN32)
	/* Windows ARM32: Check for NEON support */
	features->neon = IsProcessorFeaturePresent(PF_ARM_NEON_INSTRUCTIONS_AVAILABLE) ? 1 : 0;

#else
	/* Compile-time detection fallback */
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
	features->neon = 1;
#else
	features->neon = 0;
#endif
#endif
}
#endif /* ARM32 */

/*
 * Main detection function - thread-safe
 */
const cpu_features_t *cpu_features_detect(void)
{
	/* Fast path: already initialized */
	if (ATOMIC_LOAD(&g_cpu_features_state) == 2) {
		return &g_cpu_features;
	}

	/* Try to become the initializer (transition 0 -> 1) */
	if (ATOMIC_CAS(&g_cpu_features_state, 0, 1)) {
		/* We won the race - do initialization */
		memset(&g_cpu_features, 0, sizeof(g_cpu_features));

		/* Detect based on architecture */
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#if defined(HAVE_CPUID)
		detect_x86_features(&g_cpu_features);
#endif

#elif defined(__aarch64__) || defined(_M_ARM64)
		detect_arm64_features(&g_cpu_features);

#elif defined(__arm__) || defined(_M_ARM)
		detect_arm32_features(&g_cpu_features);

#endif

		/* Mark as fully initialized (transition 1 -> 2) */
		MEMORY_BARRIER();
		g_cpu_features_state = 2;
	} else {
		/* Another thread is initializing - wait for completion */
		while (ATOMIC_LOAD(&g_cpu_features_state) != 2) {
			/* Spin-wait (acceptable for one-time initialization) */
#if defined(_MSC_VER)
			_mm_pause();
#elif defined(__GNUC__) || defined(__clang__)
#if defined(__x86_64__) || defined(__i386__)
			__builtin_ia32_pause();
#elif defined(__aarch64__) || defined(__arm__)
			__asm__ volatile("yield" ::: "memory");
#endif
#endif
		}
	}

	return &g_cpu_features;
}

/*
 * Get CPU architecture
 */
cpu_arch_t cpu_get_arch(void)
{
#if defined(__x86_64__) || defined(_M_X64)
	return CPU_ARCH_X86_64;
#elif defined(__i386__) || defined(_M_IX86)
	return CPU_ARCH_X86;
#elif defined(__aarch64__) || defined(_M_ARM64)
	return CPU_ARCH_ARM64;
#elif defined(__arm__) || defined(_M_ARM)
	return CPU_ARCH_ARM32;
#elif defined(__riscv)
	return CPU_ARCH_RISCV;
#else
	return CPU_ARCH_UNKNOWN;
#endif
}

/*
 * Get highest SIMD level supported
 */
simd_level_t cpu_get_simd_level(void)
{
	const cpu_features_t *f = cpu_features_detect();

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
	if (f->avx512f && f->avx512vl)
		return SIMD_X64_AVX512;
	if (f->avx2 && f->fma)
		return SIMD_X64_AVX2;
	if (f->avx)
		return SIMD_X64_AVX;
	if (f->sse42)
		return SIMD_X64_SSE42;
	if (f->sse2)
		return SIMD_X64_SSE2;
	return SIMD_NONE;

#elif defined(__aarch64__) || defined(_M_ARM64) || defined(__arm__) || defined(_M_ARM)
	if (f->sve2)
		return SIMD_ARM64_SVE2;
	if (f->sve)
		return SIMD_ARM64_SVE;
	if (f->neon)
		return SIMD_ARM64_NEON;
	return SIMD_NONE;

#else
	(void)f;
	return SIMD_NONE;
#endif
}

/*
 * Get CPU architecture name as string
 */
const char *cpu_get_arch_name(void)
{
	switch (cpu_get_arch()) {
	case CPU_ARCH_X86:     return "x86";
	case CPU_ARCH_X86_64:  return "x86-64";
	case CPU_ARCH_ARM32:   return "ARM32";
	case CPU_ARCH_ARM64:   return "ARM64";
	case CPU_ARCH_RISCV:   return "RISC-V";
	default:               return "Unknown";
	}
}

/*
 * Get SIMD level name as string
 */
const char *cpu_get_simd_name(void)
{
	switch (cpu_get_simd_level()) {
	case SIMD_NONE:         return "None";
	case SIMD_X64_SSE2:     return "SSE2";
	case SIMD_X64_SSE42:    return "SSE4.2";
	case SIMD_X64_AVX:      return "AVX";
	case SIMD_X64_AVX2:     return "AVX2+FMA";
	case SIMD_X64_AVX512:   return "AVX-512";
	case SIMD_ARM64_NEON:   return "NEON";
	case SIMD_ARM64_SVE:    return "SVE";
	case SIMD_ARM64_SVE2:   return "SVE2";
	default:                return "Unknown";
	}
}
