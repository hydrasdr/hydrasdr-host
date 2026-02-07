/*
 * LOCK-FREE TRIPLE BUFFER
 *
 * Wait-free triple buffering for producer/consumer patterns.
 * Optimized for real-time streaming with zero-copy semantics.
 *
 * Features:
 * - Lock-free: no mutexes, uses atomic operations
 * - Wait-free: both producer and consumer always complete in bounded time
 * - Zero-copy: buffer pointers are swapped, data is never copied
 * - Cache-friendly: indices fit in single cache line
 *
 * Thread model:
 * - Single producer, single consumer (different threads)
 * - Or same-thread publish+read with optimized API
 *
 * Note: Include buffer_pool.h instead of this file directly.
 *       This header requires buffer_alloc_aligned() to be defined first.
 *
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TRIPLE_BUFFER_H
#define TRIPLE_BUFFER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "compat_opt.h"

/* ========================================================================
 * ATOMIC OPERATIONS FOR TRIPLE BUFFER
 * Uses centralized atomic ops from compat_opt.h
 * ======================================================================== */

#define TB_ATOMIC_LOAD(ptr)       OPT_ATOMIC_LOAD_ACQ(ptr)
#define TB_ATOMIC_STORE(ptr, val) OPT_ATOMIC_STORE_REL((ptr), (val))

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * CONFIGURATION
 * ======================================================================== */

/* Triple buffer count for output pipeline */
#define TRIPLE_BUFFER_COUNT 3

/* ========================================================================
 * TRIPLE BUFFER STRUCTURE
 * For output pipeline with producer/consumer pattern
 * ======================================================================== */

typedef struct {
	/* Buffer pointers */
	void *buffers[TRIPLE_BUFFER_COUNT];
	size_t buffer_size;

	/* Indices - use TB_ATOMIC_LOAD/STORE for thread-safe access */
	volatile uint32_t write_index;          /* Producer writes here (producer-only) */
	volatile uint32_t read_index;           /* Consumer reads here (consumer-only) */
	volatile uint32_t ready_index;          /* Shared: ready for consumer (atomic) */

	/* New data flag (atomic: set by publish, cleared by acquire) */
	volatile uint32_t new_data_available;

	/* Flags */
	uint32_t flags;

	/* Per-buffer allocation flags (for proper cleanup) */
	uint32_t buffer_flags[TRIPLE_BUFFER_COUNT];

	/* Statistics */
	uint64_t swaps;

} triple_buffer_t;

/* ========================================================================
 * TRIPLE BUFFER OPERATIONS
 * Requires buffer_alloc_aligned() and buffer_free_aligned() from buffer_pool.h
 * ======================================================================== */

/**
 * @brief Initialize triple buffer
 * @param tb Triple buffer structure
 * @param buffer_size Size of each buffer
 * @return 0 on success, -1 on failure
 */
static OPT_INLINE int triple_buffer_init(triple_buffer_t *tb,
                                         size_t buffer_size)
{
	if (!tb || buffer_size == 0) {
		return -1;
	}

	memset(tb, 0, sizeof(*tb));
	tb->buffer_size = buffer_size;

	/* Allocate three buffers */
	for (int i = 0; i < TRIPLE_BUFFER_COUNT; i++) {
		uint32_t flags = 0;
		tb->buffers[i] = buffer_alloc_aligned(buffer_size, &flags);
		tb->buffer_flags[i] = flags;

		if (!tb->buffers[i]) {
			/* Cleanup on failure */
			for (int j = 0; j < i; j++) {
				buffer_free_aligned(tb->buffers[j]);
				tb->buffers[j] = NULL;
			}
			return -1;
		}

		tb->flags |= flags;
	}

	/* Initialize indices */
	tb->write_index = 0;
	tb->read_index = 2;
	tb->ready_index = 1;
	TB_ATOMIC_STORE(&tb->new_data_available, 0);

	return 0;
}

/**
 * @brief Destroy triple buffer
 * @param tb Triple buffer structure
 */
static OPT_INLINE void triple_buffer_destroy(triple_buffer_t *tb)
{
	if (OPT_UNLIKELY(!tb)) return;

	for (int i = 0; i < TRIPLE_BUFFER_COUNT; i++) {
		if (OPT_LIKELY(tb->buffers[i] != NULL)) {
			buffer_free_aligned(tb->buffers[i]);
			tb->buffers[i] = NULL;
		}
	}
}

/**
 * @brief Get write buffer (producer)
 * @param tb Triple buffer structure
 * @return Write buffer pointer
 */
static OPT_INLINE void *triple_buffer_get_write(triple_buffer_t *tb)
{
	return tb->buffers[tb->write_index];
}

/**
 * @brief Publish write buffer (producer done writing)
 * Swaps write and ready buffers atomically
 * @param tb Triple buffer structure
 *
 * Thread safety: Call only from producer thread
 */
static OPT_INLINE void triple_buffer_publish(triple_buffer_t *tb)
{
	/* Swap write and ready indices atomically */
	uint32_t old_ready = TB_ATOMIC_LOAD(&tb->ready_index);
	TB_ATOMIC_STORE(&tb->ready_index, tb->write_index);
	tb->write_index = old_ready;

	/* Signal new data available with release semantics */
	TB_ATOMIC_STORE(&tb->new_data_available, 1);
	tb->swaps++;
}

/**
 * @brief Get read buffer (consumer)
 * @param tb Triple buffer structure
 * @return Read buffer pointer
 */
static OPT_INLINE void *triple_buffer_get_read(triple_buffer_t *tb)
{
	return tb->buffers[tb->read_index];
}

/**
 * @brief Acquire latest ready buffer (consumer)
 * Swaps read and ready buffers if new data available
 * @param tb Triple buffer structure
 * @return true if new data was acquired
 *
 * Thread safety: Call only from consumer thread
 */
static OPT_INLINE bool triple_buffer_acquire(triple_buffer_t *tb)
{
	/* Check if new data is available with acquire semantics */
	if (!TB_ATOMIC_LOAD(&tb->new_data_available)) {
		return false;
	}

	/* Clear flag atomically */
	TB_ATOMIC_STORE(&tb->new_data_available, 0);

	/* Swap read and ready indices atomically */
	uint32_t old_ready = TB_ATOMIC_LOAD(&tb->ready_index);
	TB_ATOMIC_STORE(&tb->ready_index, tb->read_index);
	tb->read_index = old_ready;

	return true;
}

/**
 * @brief Same-thread optimization: publish and get read buffer in one call
 *
 * Use this when producer and consumer are the same thread (sequential
 * publish + read pattern). Bypasses ready buffer and new_data_available
 * flag for maximum performance.
 *
 * @param tb Triple buffer structure
 * @return Read buffer pointer (the buffer that was just written)
 *
 * Thread safety: Call only when same thread does write+read
 */
static OPT_INLINE void *triple_buffer_publish_for_read(triple_buffer_t *tb)
{
	/* Direct swap: write buffer becomes read buffer */
	uint32_t old_write = tb->write_index;
	tb->write_index = tb->read_index;
	tb->read_index = old_write;
	tb->swaps++;
	return tb->buffers[old_write];
}

/**
 * @brief Reset triple buffer indices
 * @param tb Triple buffer structure
 *
 * Thread safety: Call only when no concurrent access (e.g., during init/cleanup)
 */
static OPT_INLINE void triple_buffer_reset(triple_buffer_t *tb)
{
	tb->write_index = 0;
	tb->read_index = 2;
	TB_ATOMIC_STORE(&tb->ready_index, 1);
	TB_ATOMIC_STORE(&tb->new_data_available, 0);
	tb->swaps = 0;
}

#ifdef __cplusplus
}
#endif

#endif /* TRIPLE_BUFFER_H */
