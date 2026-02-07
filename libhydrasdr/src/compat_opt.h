/*
 * PORTABLE OPTIMIZATION COMPATIBILITY HEADER
 *
 * Cross-platform macros for performance optimization hints that work on:
 * - GCC (Linux, MinGW, ARM)
 * - Clang (Linux, macOS, iOS)
 * - MSVC (Visual Studio 2019/2022+)
 * - ARM compilers (RPi3/4/5, Apple Silicon)
 *
 * These hints help compilers generate better code without requiring
 * platform-specific SIMD intrinsics.
 *
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef COMPAT_OPT_H
#define COMPAT_OPT_H

#include <stdint.h>
#include <stddef.h>

/* ========================================================================
 * COMPILER DETECTION
 * ======================================================================== */

#if defined(_MSC_VER)
	#define COMPILER_MSVC 1
	#define COMPILER_VERSION _MSC_VER
#elif defined(__clang__)
	#define COMPILER_CLANG 1
	#define COMPILER_VERSION (__clang_major__ * 10000 + __clang_minor__ * 100)
#elif defined(__GNUC__)
	#define COMPILER_GCC 1
	#define COMPILER_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100)
#else
	#define COMPILER_UNKNOWN 1
	#define COMPILER_VERSION 0
#endif

/* ========================================================================
 * ARCHITECTURE DETECTION
 * ======================================================================== */

#if defined(__x86_64__) || defined(_M_X64)
	#define ARCH_X86_64 1
#elif defined(__i386__) || defined(_M_IX86)
	#define ARCH_X86_32 1
#elif defined(__aarch64__) || defined(_M_ARM64)
	#define ARCH_ARM64 1
#elif defined(__arm__) || defined(_M_ARM)
	#define ARCH_ARM32 1
#elif defined(__riscv)
	#define ARCH_RISCV 1
#else
	#define ARCH_UNKNOWN 1
#endif

/* ========================================================================
 * RESTRICT KEYWORD (Pointer Aliasing Hint)
 * Tells compiler that pointers don't alias, enabling better optimization
 * ======================================================================== */

#if defined(COMPILER_MSVC)
	#define OPT_RESTRICT __restrict
#elif defined(COMPILER_GCC) || defined(COMPILER_CLANG)
	#define OPT_RESTRICT __restrict__
#else
	#define OPT_RESTRICT
#endif

/* ========================================================================
 * INLINE HINTS
 * Force/suggest inlining for performance-critical small functions
 * ======================================================================== */

#if defined(COMPILER_MSVC)
	#define OPT_INLINE __forceinline
	#define OPT_NOINLINE __declspec(noinline)
#elif defined(COMPILER_GCC) || defined(COMPILER_CLANG)
	#define OPT_INLINE inline __attribute__((always_inline))
	#define OPT_NOINLINE __attribute__((noinline))
#else
	#define OPT_INLINE inline
	#define OPT_NOINLINE
#endif

/* ========================================================================
 * BRANCH PREDICTION HINTS
 * Help branch predictor by marking likely/unlikely conditions
 * ======================================================================== */

#if defined(COMPILER_GCC) || defined(COMPILER_CLANG)
	#define OPT_LIKELY(x)   __builtin_expect(!!(x), 1)
	#define OPT_UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
	/* MSVC doesn't have direct equivalent, but optimizer is usually good */
	#define OPT_LIKELY(x)   (x)
	#define OPT_UNLIKELY(x) (x)
#endif

/* ========================================================================
 * SOFTWARE PREFETCH
 * Bring data into cache before it's needed
 *
 * Parameters:
 *   addr - address to prefetch
 *   rw   - 0=read, 1=write
 *   locality - 0=non-temporal (use once), 1-3=temporal (keep in cache)
 *              3=high locality (L1), 2=medium (L2), 1=low (L3), 0=NTA
 * ======================================================================== */

#if defined(COMPILER_GCC) || defined(COMPILER_CLANG)
	#define OPT_PREFETCH(addr, rw, locality) \
		__builtin_prefetch((const void *)(addr), (rw), (locality))
	#define OPT_PREFETCH_READ(addr)  __builtin_prefetch((const void *)(addr), 0, 3)
	#define OPT_PREFETCH_WRITE(addr) __builtin_prefetch((const void *)(addr), 1, 3)
	#define OPT_PREFETCH_NTA(addr)   __builtin_prefetch((const void *)(addr), 0, 0)
#elif defined(COMPILER_MSVC)
	#include <intrin.h>
	/* MSVC prefetch: _MM_HINT_T0=L1, _MM_HINT_T1=L2, _MM_HINT_T2=L3, _MM_HINT_NTA */
	#define OPT_PREFETCH(addr, rw, locality) \
		_mm_prefetch((const char *)(addr), (locality) == 3 ? _MM_HINT_T0 : \
		                                   (locality) == 2 ? _MM_HINT_T1 : \
		                                   (locality) == 1 ? _MM_HINT_T2 : _MM_HINT_NTA)
	#define OPT_PREFETCH_READ(addr)  _mm_prefetch((const char *)(addr), _MM_HINT_T0)
	#define OPT_PREFETCH_WRITE(addr) _mm_prefetch((const char *)(addr), _MM_HINT_T0)
	#define OPT_PREFETCH_NTA(addr)   _mm_prefetch((const char *)(addr), _MM_HINT_NTA)
#else
	#define OPT_PREFETCH(addr, rw, locality) ((void)0)
	#define OPT_PREFETCH_READ(addr)  ((void)0)
	#define OPT_PREFETCH_WRITE(addr) ((void)0)
	#define OPT_PREFETCH_NTA(addr)   ((void)0)
#endif

/* ========================================================================
 * ALIGNMENT HINTS
 * Tell compiler that pointer is aligned, enabling SIMD operations
 * ======================================================================== */

#if defined(COMPILER_GCC) || defined(COMPILER_CLANG)
	#define OPT_ASSUME_ALIGNED(ptr, alignment) \
		__builtin_assume_aligned((ptr), (alignment))
#elif defined(COMPILER_MSVC)
	/* MSVC uses __assume for alignment hints */
	#define OPT_ASSUME_ALIGNED(ptr, alignment) \
		(__assume(((uintptr_t)(ptr) & ((alignment) - 1)) == 0), (ptr))
#else
	#define OPT_ASSUME_ALIGNED(ptr, alignment) (ptr)
#endif

/* Convenience macros for common alignments */
#define OPT_ALIGNED_16(ptr)  OPT_ASSUME_ALIGNED((ptr), 16)
#define OPT_ALIGNED_32(ptr)  OPT_ASSUME_ALIGNED((ptr), 32)
#define OPT_ALIGNED_64(ptr)  OPT_ASSUME_ALIGNED((ptr), 64)

/* ========================================================================
 * MEMORY ALIGNMENT ATTRIBUTES
 * Declare variables/structs with specific alignment
 * ======================================================================== */

#if defined(COMPILER_MSVC)
	#define OPT_ALIGN(n) __declspec(align(n))
#elif defined(COMPILER_GCC) || defined(COMPILER_CLANG)
	#define OPT_ALIGN(n) __attribute__((aligned(n)))
#else
	#define OPT_ALIGN(n)
#endif

/* ========================================================================
 * VECTORIZATION PRAGMAS
 * Hint to compiler that loop can be safely vectorized
 * ======================================================================== */

/* Helper macros for pragma stringification */
#define OPT_PRAGMA_STR_HELPER(x) #x
#define OPT_PRAGMA_STR(x) OPT_PRAGMA_STR_HELPER(x)
#define OPT_PRAGMA_DO(x) _Pragma(OPT_PRAGMA_STR(x))

#if defined(COMPILER_CLANG)
	/* Clang-specific vectorization hints */
	#define OPT_PRAGMA_VECTORIZE \
		_Pragma("clang loop vectorize(enable) interleave(enable)")
	#define OPT_PRAGMA_UNROLL(n) \
		OPT_PRAGMA_DO(clang loop unroll_count(n))
	#define OPT_PRAGMA_UNROLL_FULL \
		_Pragma("clang loop unroll(full)")
#elif defined(COMPILER_GCC) && COMPILER_VERSION >= 80000
	/* GCC 8+ vectorization hints (GCC unroll pragma requires GCC 8+) */
	#define OPT_PRAGMA_VECTORIZE \
		_Pragma("GCC ivdep")
	#define OPT_PRAGMA_UNROLL(n) \
		OPT_PRAGMA_DO(GCC unroll n)
	#define OPT_PRAGMA_UNROLL_FULL \
		_Pragma("GCC unroll 128")
#elif defined(COMPILER_GCC) && COMPILER_VERSION >= 40900
	/* GCC 4.9-7.x: only ivdep supported */
	#define OPT_PRAGMA_VECTORIZE \
		_Pragma("GCC ivdep")
	#define OPT_PRAGMA_UNROLL(n) /* Use -funroll-loops flag */
	#define OPT_PRAGMA_UNROLL_FULL /* Use -funroll-loops flag */
#elif defined(COMPILER_MSVC)
	/* MSVC vectorization hints */
	#define OPT_PRAGMA_VECTORIZE \
		__pragma(loop(ivdep))
	/* MSVC doesn't support counted unroll - use /O2 for auto-unrolling */
	#define OPT_PRAGMA_UNROLL(n)
	#define OPT_PRAGMA_UNROLL_FULL
#else
	#define OPT_PRAGMA_VECTORIZE
	#define OPT_PRAGMA_UNROLL(n)
	#define OPT_PRAGMA_UNROLL_FULL
#endif

/* ========================================================================
 * FAST MATH HINTS (Use with caution!)
 * Allow compiler to reorder/optimize floating-point operations
 * Only use when strict IEEE-754 compliance is not required
 * ======================================================================== */

/* These are controlled via compiler flags, but we can hint function-level */
#if defined(COMPILER_GCC)
	/* GCC supports pragma optimize */
	#define OPT_FAST_MATH_BEGIN \
		_Pragma("GCC push_options") \
		_Pragma("GCC optimize (\"fast-math\")")
	#define OPT_FAST_MATH_END \
		_Pragma("GCC pop_options")
#elif defined(COMPILER_CLANG)
	/* Clang: Use -ffast-math flag at compile time, or per-function attribute */
	/* Note: Clang supports __attribute__((optimize)) but not pragma push/pop */
	#define OPT_FAST_MATH_BEGIN
	#define OPT_FAST_MATH_END
#else
	/* MSVC: Use /fp:fast flag at compile time instead */
	#define OPT_FAST_MATH_BEGIN
	#define OPT_FAST_MATH_END
#endif

/* ========================================================================
 * HOT/COLD FUNCTION ATTRIBUTES
 * Mark functions as frequently/rarely called for better code placement
 * ======================================================================== */

#if defined(COMPILER_GCC) || defined(COMPILER_CLANG)
	#define OPT_HOT  __attribute__((hot))
	#define OPT_COLD __attribute__((cold))
#else
	#define OPT_HOT
	#define OPT_COLD
#endif

/* ========================================================================
 * PURE/CONST FUNCTION ATTRIBUTES
 * Mark functions with no side effects for better optimization
 * ======================================================================== */

#if defined(COMPILER_GCC) || defined(COMPILER_CLANG)
	/* Pure: no side effects, result depends only on arguments and global memory */
	#define OPT_PURE __attribute__((pure))
	/* Const: no side effects, result depends only on arguments (stricter) */
	#define OPT_CONST __attribute__((const))
#else
	#define OPT_PURE
	#define OPT_CONST
#endif

/* ========================================================================
 * UNREACHABLE CODE HINT
 * Tell compiler that code path should never be reached
 * ======================================================================== */

#if defined(COMPILER_GCC) || defined(COMPILER_CLANG)
	#define OPT_UNREACHABLE() __builtin_unreachable()
#elif defined(COMPILER_MSVC)
	#define OPT_UNREACHABLE() __assume(0)
#else
	#define OPT_UNREACHABLE() ((void)0)
#endif

/* ========================================================================
 * FMA (FUSED MULTIPLY-ADD) HINT
 * Portable FMA that uses hardware FMA when available, falls back to mul+add
 *
 * FMA computes (a * b) + c with single rounding, which is:
 * - More accurate (single rounding instead of two)
 * - Often faster on modern CPUs (single instruction)
 * ======================================================================== */

#if defined(COMPILER_GCC) || defined(COMPILER_CLANG)
	/* Use compiler builtin that will use FMA instruction when available */
	#define OPT_FMA_F32(a, b, c) __builtin_fmaf((a), (b), (c))
	#define OPT_FMA_F64(a, b, c) __builtin_fma((a), (b), (c))
#elif defined(COMPILER_MSVC)
	#include <math.h>
	/* MSVC will optimize to FMA with /fp:fast and appropriate /arch */
	#define OPT_FMA_F32(a, b, c) fmaf((a), (b), (c))
	#define OPT_FMA_F64(a, b, c) fma((a), (b), (c))
#else
	/* Fallback: explicit multiply-add (compiler may still optimize) */
	#define OPT_FMA_F32(a, b, c) ((a) * (b) + (c))
	#define OPT_FMA_F64(a, b, c) ((a) * (b) + (c))
#endif

/* ========================================================================
 * COMPILER MEMORY BARRIER
 * Prevent compiler from reordering memory operations
 * ======================================================================== */

#if defined(COMPILER_GCC) || defined(COMPILER_CLANG)
	#define OPT_COMPILER_BARRIER() __asm__ __volatile__("" ::: "memory")
#elif defined(COMPILER_MSVC)
	#define OPT_COMPILER_BARRIER() _ReadWriteBarrier()
#else
	#define OPT_COMPILER_BARRIER() ((void)0)
#endif

/* ========================================================================
 * CACHE LINE SIZE
 * Used for padding to avoid false sharing in multi-threaded code
 * ======================================================================== */

#if defined(ARCH_ARM32)
	/* ARM Cortex-A7/A53 (RPi3) typically have 64-byte cache lines */
	#define OPT_CACHE_LINE_SIZE 64
#elif defined(ARCH_ARM64)
	/* ARM Cortex-A72/A76 (RPi4/5) and Apple Silicon have 64 or 128-byte lines */
	/* Use 128 to be safe for Apple M-series */
	#define OPT_CACHE_LINE_SIZE 128
#else
	/* x86/x86-64 typically have 64-byte cache lines */
	#define OPT_CACHE_LINE_SIZE 64
#endif

/* ========================================================================
 * SIMD WIDTH HINT
 * Suggests natural SIMD width for the target architecture
 * Used for manual loop unrolling to match vector width
 * ======================================================================== */

#if defined(ARCH_X86_64) || defined(ARCH_X86_32)
	/* x86: Assume at least SSE2 (128-bit = 4 floats) */
	/* AVX/AVX2 (256-bit = 8 floats) and AVX-512 (512-bit = 16 floats) may be available */
	#define OPT_SIMD_WIDTH_F32 4  /* Conservative: SSE2 baseline */
	#define OPT_SIMD_WIDTH_F64 2
#elif defined(ARCH_ARM64)
	/* ARM64: NEON is mandatory (128-bit = 4 floats) */
	#define OPT_SIMD_WIDTH_F32 4
	#define OPT_SIMD_WIDTH_F64 2
#elif defined(ARCH_ARM32)
	/* ARM32: NEON may be available (128-bit = 4 floats) */
	#define OPT_SIMD_WIDTH_F32 4
	#define OPT_SIMD_WIDTH_F64 2
#else
	/* Conservative default */
	#define OPT_SIMD_WIDTH_F32 4
	#define OPT_SIMD_WIDTH_F64 2
#endif

/* ========================================================================
 * DIAGNOSTIC PRAGMAS
 * Suppress/enable specific compiler warnings
 * ======================================================================== */

#if defined(COMPILER_CLANG)
	#define OPT_DIAG_PUSH _Pragma("clang diagnostic push")
	#define OPT_DIAG_POP  _Pragma("clang diagnostic pop")
	#define OPT_DIAG_IGNORE_FLOAT_EQUAL \
		_Pragma("clang diagnostic ignored \"-Wfloat-equal\"")
#elif defined(COMPILER_GCC)
	#define OPT_DIAG_PUSH _Pragma("GCC diagnostic push")
	#define OPT_DIAG_POP  _Pragma("GCC diagnostic pop")
	#define OPT_DIAG_IGNORE_FLOAT_EQUAL \
		_Pragma("GCC diagnostic ignored \"-Wfloat-equal\"")
#elif defined(COMPILER_MSVC)
	#define OPT_DIAG_PUSH __pragma(warning(push))
	#define OPT_DIAG_POP  __pragma(warning(pop))
	#define OPT_DIAG_IGNORE_FLOAT_EQUAL __pragma(warning(disable: 4056))
#else
	#define OPT_DIAG_PUSH
	#define OPT_DIAG_POP
	#define OPT_DIAG_IGNORE_FLOAT_EQUAL
#endif

/* ========================================================================
 * ASSUME HINT
 * Tell compiler to assume a condition is true for optimization
 * ======================================================================== */

#if defined(COMPILER_MSVC)
	#define OPT_ASSUME(cond) __assume(cond)
#elif defined(COMPILER_CLANG)
	/* Clang has __builtin_assume */
	#define OPT_ASSUME(cond) __builtin_assume(cond)
#elif defined(COMPILER_GCC)
	/* GCC: Use __builtin_unreachable() pattern (works on all GCC versions) */
	#define OPT_ASSUME(cond) do { if (!(cond)) __builtin_unreachable(); } while(0)
#else
	/* Fallback: void cast to avoid unused warnings */
	#define OPT_ASSUME(cond) ((void)(cond))
#endif

/* ========================================================================
 * ALIGNED MEMORY ALLOCATION
 * Cross-platform aligned memory allocation for SIMD and buffers
 * ======================================================================== */

#include <stdlib.h>

/* Windows: Include malloc.h for _aligned_malloc/_aligned_free */
#if defined(_WIN32) || defined(__MINGW32__) || defined(__MINGW64__)
#include <malloc.h>
#endif

/* POSIX: Include unistd.h for posix_memalign detection */
#if !defined(_WIN32) && !defined(__MINGW32__) && !defined(__MINGW64__)
#include <unistd.h>
#endif

/**
 * @brief Allocate aligned memory
 * @param size Size in bytes to allocate
 * @param alignment Alignment requirement (must be power of 2)
 * @return Pointer to aligned memory, or NULL on failure
 *
 * @note Memory must be freed with opt_aligned_free()
 */
static OPT_INLINE void *opt_aligned_alloc(size_t size, size_t alignment)
{
    void *ptr = NULL;

    /* Handle size 0 - return NULL (consistent across platforms) */
    if (size == 0) {
        return NULL;
    }

    /* Ensure alignment is at least sizeof(void*) and power of 2 */
    if (alignment < sizeof(void*)) {
        alignment = sizeof(void*);
    }

#if defined(_WIN32) || defined(__MINGW32__) || defined(__MINGW64__)
    /* Windows (MSVC and MinGW64): Use _aligned_malloc from <malloc.h> */
    ptr = _aligned_malloc(size, alignment);
#elif defined(_POSIX_VERSION) && _POSIX_VERSION >= 200112L
    /* POSIX: posix_memalign available */
    if (posix_memalign(&ptr, alignment, size) != 0) {
        ptr = NULL;
    }
#else
    /* Fallback: manual alignment */
    void *raw = malloc(size + alignment + sizeof(void*));
    if (raw) {
        /* Align pointer and store original for free */
        uintptr_t aligned = ((uintptr_t)raw + alignment + sizeof(void*)) & ~(alignment - 1);
        ((void**)aligned)[-1] = raw;
        ptr = (void*)aligned;
    }
#endif

    return ptr;
}

/**
 * @brief Free aligned memory
 * @param ptr Pointer returned by opt_aligned_alloc (NULL is safe)
 */
static OPT_INLINE void opt_aligned_free(void *ptr)
{
    if (!ptr) return;

#if defined(_WIN32) || defined(__MINGW32__) || defined(__MINGW64__)
    /* Windows (MSVC and MinGW64): Use _aligned_free from <malloc.h> */
    _aligned_free(ptr);
#elif defined(_POSIX_VERSION) && _POSIX_VERSION >= 200112L
    free(ptr);
#else
    /* Fallback: retrieve original pointer */
    free(((void**)ptr)[-1]);
#endif
}

/**
 * @brief Allocate cache-line aligned memory
 * Convenience macro for common case of cache-line alignment
 */
#define opt_cache_aligned_alloc(size) \
    opt_aligned_alloc((size), OPT_CACHE_LINE_SIZE)

/**
 * @brief Allocate SIMD-aligned memory (64 bytes for AVX-512)
 */
#define opt_simd_aligned_alloc(size) \
    opt_aligned_alloc((size), 64)

/* ========================================================================
 * PORTABLE ATOMIC OPERATIONS
 *
 * Centralized atomic operations for lock-free data structures.
 * Used by spsc_queue.h and buffer_pool.h for thread-safe operations.
 *
 * Priority order:
 * 1. C11 stdatomic (most portable, compiler-supported)
 * 2. GCC/Clang __atomic builtins
 * 3. MSVC intrinsics with proper hardware barriers
 * 4. Fallback with volatile + compiler barrier (x86 only)
 * ======================================================================== */

/* Check for C11 stdatomic support */
#if !defined(__STDC_NO_ATOMICS__) && defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#define OPT_USE_C11_ATOMICS 1
#elif defined(__has_include)
#if __has_include(<stdatomic.h>) && !defined(_MSC_VER)
#define OPT_USE_C11_ATOMICS 1
#endif
#endif

#ifdef OPT_USE_C11_ATOMICS
/* ---- C11 Atomics (preferred) ---- */
#include <stdatomic.h>

#define OPT_ATOMIC_LOAD_ACQ(ptr) \
    atomic_load_explicit((_Atomic uint32_t*)(ptr), memory_order_acquire)

#define OPT_ATOMIC_STORE_REL(ptr, val) \
    atomic_store_explicit((_Atomic uint32_t*)(ptr), (val), memory_order_release)

#define OPT_MEMORY_FENCE() atomic_thread_fence(memory_order_seq_cst)

#elif defined(COMPILER_MSVC)
/* ---- MSVC Intrinsics ---- */
/* Note: intrin.h already included above for prefetch */

#if defined(_M_ARM64) || defined(_M_ARM)
/* ARM memory barriers */
#ifndef _ARM64_BARRIER_ISH
#define _ARM64_BARRIER_ISH   0xB
#define _ARM64_BARRIER_ISHLD 0x9
#define _ARM64_BARRIER_ISHST 0xA
#endif

static __forceinline uint32_t opt_atomic_load_acq(volatile uint32_t *ptr) {
    uint32_t val = *ptr;
    __dmb(_ARM64_BARRIER_ISHLD);
    return val;
}

static __forceinline void opt_atomic_store_rel(volatile uint32_t *ptr, uint32_t val) {
    __dmb(_ARM64_BARRIER_ISHST);
    *ptr = val;
}

#define OPT_MEMORY_FENCE() do { _ReadWriteBarrier(); __dmb(_ARM64_BARRIER_ISH); } while(0)

#else /* x86/x64 */
#include <emmintrin.h>

static __forceinline uint32_t opt_atomic_load_acq(volatile uint32_t *ptr) {
    uint32_t val = *ptr;
    _ReadWriteBarrier();
    _mm_lfence();
    return val;
}

static __forceinline void opt_atomic_store_rel(volatile uint32_t *ptr, uint32_t val) {
    _mm_sfence();
    _ReadWriteBarrier();
    *ptr = val;
}

#define OPT_MEMORY_FENCE() do { _ReadWriteBarrier(); _mm_mfence(); } while(0)

#endif /* _M_ARM64 || _M_ARM */

#define OPT_ATOMIC_LOAD_ACQ(ptr) opt_atomic_load_acq((volatile uint32_t*)(ptr))
#define OPT_ATOMIC_STORE_REL(ptr, val) opt_atomic_store_rel((volatile uint32_t*)(ptr), (val))

#elif defined(COMPILER_GCC) || defined(COMPILER_CLANG)
/* ---- GCC/Clang __atomic builtins ---- */

#define OPT_ATOMIC_LOAD_ACQ(ptr) \
    __atomic_load_n((volatile uint32_t*)(ptr), __ATOMIC_ACQUIRE)

#define OPT_ATOMIC_STORE_REL(ptr, val) \
    __atomic_store_n((volatile uint32_t*)(ptr), (val), __ATOMIC_RELEASE)

#define OPT_MEMORY_FENCE() __atomic_thread_fence(__ATOMIC_SEQ_CST)

#else
/* ---- Fallback: volatile + compiler barrier (x86 only) ---- */
#warning "Using fallback atomics - only safe on x86/x64 with strong memory model!"

#define OPT_ATOMIC_LOAD_ACQ(ptr) (OPT_COMPILER_BARRIER(), *(volatile uint32_t*)(ptr))
#define OPT_ATOMIC_STORE_REL(ptr, val) (*(volatile uint32_t*)(ptr) = (val), OPT_COMPILER_BARRIER())
#define OPT_MEMORY_FENCE() OPT_COMPILER_BARRIER()

#endif

/* ========================================================================
 * CROSS-PLATFORM STRING UTILITIES
 * ======================================================================== */

#include <ctype.h>

/**
 * @brief Case-insensitive string comparison (portable)
 *
 * Windows/MSVC doesn't have strcasecmp in standard headers.
 * This provides a portable implementation that works everywhere.
 *
 * @param s1 First string to compare
 * @param s2 Second string to compare
 * @return <0 if s1 < s2, 0 if s1 == s2, >0 if s1 > s2
 */
static OPT_INLINE int opt_strcasecmp(const char *s1, const char *s2)
{
	while (*s1 && *s2) {
		int c1 = tolower((unsigned char)*s1);
		int c2 = tolower((unsigned char)*s2);
		if (c1 != c2)
			return c1 - c2;
		s1++;
		s2++;
	}
	return tolower((unsigned char)*s1) - tolower((unsigned char)*s2);
}

#endif /* COMPAT_OPT_H */
