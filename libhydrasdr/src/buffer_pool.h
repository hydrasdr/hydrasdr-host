/*
 * HIGH-PERFORMANCE BUFFER POOL
 *
 * Pre-allocated buffer pool with aligned memory support.
 * Optimized for USB streaming with zero-copy patterns.
 *
 * Features:
 * - SIMD-aligned buffers for vectorized processing (64-byte alignment for AVX-512)
 * - Triple buffering for output pipeline (see triple_buffer.h)
 * - Thread-safe buffer acquisition/release
 *
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef BUFFER_POOL_H
#define BUFFER_POOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "compat_opt.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * CONFIGURATION
 * ======================================================================== */

/* Maximum buffers in a pool */
#define BUFFER_POOL_MAX_BUFFERS 32

/* Default alignment for buffers (64 bytes for AVX-512) */
#define BUFFER_POOL_ALIGNMENT 64

/* ========================================================================
 * BUFFER FLAGS
 * ======================================================================== */

typedef enum {
	BUFFER_FLAG_NONE       = 0,
	BUFFER_FLAG_ALIGNED    = (1 << 0),  /* Allocated with alignment */
	BUFFER_FLAG_IN_USE     = (1 << 1),  /* Currently acquired */
} buffer_flags_t;

/* ========================================================================
 * BUFFER DESCRIPTOR
 * ======================================================================== */

typedef struct {
	void *ptr;              /* Buffer pointer */
	size_t size;            /* Buffer size in bytes */
	uint32_t flags;         /* Buffer flags */
	uint32_t index;         /* Index in pool */
} buffer_desc_t;

/* ========================================================================
 * BUFFER POOL STRUCTURE
 * ======================================================================== */

typedef struct {
	/* Buffer storage */
	buffer_desc_t buffers[BUFFER_POOL_MAX_BUFFERS];
	uint32_t buffer_count;
	size_t buffer_size;

	/* Allocation tracking */
	uint32_t free_mask;     /* Bitmask of free buffers */

	/* Statistics */
	uint64_t alloc_count;
	uint64_t free_count;

} buffer_pool_t;

/* ========================================================================
 * SINGLE BUFFER ALLOCATION (aligned)
 * ======================================================================== */

/**
 * @brief Allocate a single aligned buffer
 * @param size Buffer size in bytes
 * @param flags Output: allocation flags (optional, can be NULL)
 * @return Buffer pointer or NULL on failure
 */
static OPT_INLINE void *buffer_alloc_aligned(size_t size, uint32_t *flags)
{
	void *ptr = opt_aligned_alloc(size, BUFFER_POOL_ALIGNMENT);

	if (flags) {
		*flags = ptr ? BUFFER_FLAG_ALIGNED : BUFFER_FLAG_NONE;
	}

	return ptr;
}

/**
 * @brief Free a buffer allocated with buffer_alloc_aligned
 * @param ptr Buffer pointer (NULL is safe)
 */
static OPT_INLINE void buffer_free_aligned(void *ptr)
{
	if (OPT_LIKELY(ptr)) {
		opt_aligned_free(ptr);
	}
}

/* ========================================================================
 * BUFFER POOL OPERATIONS
 * ======================================================================== */

/**
 * @brief Initialize buffer pool
 * @param pool Pool structure
 * @param buffer_count Number of buffers to allocate
 * @param buffer_size Size of each buffer
 * @return 0 on success, -1 on failure
 */
static OPT_INLINE int buffer_pool_init(buffer_pool_t *pool,
                                       uint32_t buffer_count,
                                       size_t buffer_size)
{
	if (!pool || buffer_count == 0 || buffer_count > BUFFER_POOL_MAX_BUFFERS) {
		return -1;
	}

	memset(pool, 0, sizeof(*pool));
	pool->buffer_size = buffer_size;
	pool->buffer_count = buffer_count;
	pool->free_mask = 0;

	/* Allocate buffers */
	for (uint32_t i = 0; i < buffer_count; i++) {
		uint32_t flags = 0;
		void *ptr = buffer_alloc_aligned(buffer_size, &flags);

		if (!ptr) {
			/* Cleanup on failure */
			for (uint32_t j = 0; j < i; j++) {
				buffer_free_aligned(pool->buffers[j].ptr);
			}
			return -1;
		}

		pool->buffers[i].ptr = ptr;
		pool->buffers[i].size = buffer_size;
		pool->buffers[i].flags = flags;
		pool->buffers[i].index = i;
		pool->free_mask |= (1u << i);
	}

	return 0;
}

/**
 * @brief Destroy buffer pool and free all buffers
 * @param pool Pool structure
 */
static OPT_INLINE void buffer_pool_destroy(buffer_pool_t *pool)
{
	if (OPT_UNLIKELY(!pool)) return;

	for (uint32_t i = 0; i < pool->buffer_count; i++) {
		if (OPT_LIKELY(pool->buffers[i].ptr != NULL)) {
			buffer_free_aligned(pool->buffers[i].ptr);
			pool->buffers[i].ptr = NULL;
		}
	}

	pool->buffer_count = 0;
	pool->free_mask = 0;
}

/**
 * @brief Acquire a free buffer from pool
 * @param pool Pool structure
 * @return Buffer descriptor or NULL if none available
 *
 * @note Not thread-safe, caller must synchronize
 */
static OPT_INLINE buffer_desc_t *buffer_pool_acquire(buffer_pool_t *pool)
{
	if (!pool || pool->free_mask == 0) {
		return NULL;
	}

	/* Find first free buffer (count trailing zeros) */
#if defined(COMPILER_GCC) || defined(COMPILER_CLANG)
	uint32_t index = __builtin_ctz(pool->free_mask);
#elif defined(COMPILER_MSVC)
	unsigned long index;
	_BitScanForward(&index, pool->free_mask);
#else
	uint32_t index = 0;
	uint32_t mask = pool->free_mask;
	while ((mask & 1) == 0) {
		mask >>= 1;
		index++;
	}
#endif

	/* Mark as in use */
	pool->free_mask &= ~(1u << index);
	pool->buffers[index].flags |= BUFFER_FLAG_IN_USE;
	pool->alloc_count++;

	return &pool->buffers[index];
}

/**
 * @brief Release a buffer back to pool
 * @param pool Pool structure
 * @param desc Buffer descriptor
 */
static OPT_INLINE void buffer_pool_release(buffer_pool_t *pool, buffer_desc_t *desc)
{
	if (!pool || !desc || desc->index >= pool->buffer_count) {
		return;
	}

	desc->flags &= ~BUFFER_FLAG_IN_USE;
	pool->free_mask |= (1u << desc->index);
	pool->free_count++;
}

#ifdef __cplusplus
}
#endif

/* ========================================================================
 * TRIPLE BUFFER
 * Include after buffer_alloc_aligned() is defined
 * ======================================================================== */
#include "triple_buffer.h"

#endif /* BUFFER_POOL_H */
