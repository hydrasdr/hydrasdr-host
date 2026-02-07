/*
 * LOCK-FREE SINGLE PRODUCER SINGLE CONSUMER (SPSC) QUEUE
 *
 * High-performance lock-free ring buffer for streaming applications.
 * Optimized for the USB callback → Consumer thread pattern.
 *
 * Features:
 * - Lock-free: No mutexes, uses atomic operations only
 * - Cache-line aligned: Prevents false sharing between producer/consumer
 * - Wait-free for producer: enqueue never blocks
 * - Bounded: Fixed-size ring buffer with overflow detection
 *
 * Thread Safety Guarantees:
 * - Single producer thread may call: enqueue, enqueue_swap, can_enqueue
 * - Single consumer thread may call: dequeue, peek, commit_dequeue, can_dequeue
 * - Any thread may call: is_empty, is_full, count, stats (approximate values)
 * - init/reset must be called with no concurrent operations
 *
 * Memory Ordering:
 * - Producer uses release semantics when publishing (head update)
 * - Consumer uses acquire semantics when reading (head read)
 * - Consumer uses release semantics when releasing (tail update)
 * - Producer uses acquire semantics when checking space (tail read)
 * - Data in slots is fully visible before index is updated
 *
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SPSC_QUEUE_H
#define SPSC_QUEUE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "compat_opt.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * ATOMIC OPERATIONS
 * Uses centralized atomic ops from compat_opt.h
 * ======================================================================== */

#define SPSC_ATOMIC_LOAD_ACQ(ptr)       OPT_ATOMIC_LOAD_ACQ(ptr)
#define SPSC_ATOMIC_STORE_REL(ptr, val) OPT_ATOMIC_STORE_REL((ptr), (val))
#define SPSC_MEMORY_FENCE()             OPT_MEMORY_FENCE()
#define SPSC_COMPILER_BARRIER()         OPT_COMPILER_BARRIER()

/* ========================================================================
 * SPSC QUEUE CONFIGURATION
 * ======================================================================== */

/* Maximum slots in the queue (must be power of 2) */
#ifndef SPSC_QUEUE_CAPACITY
#define SPSC_QUEUE_CAPACITY 16
#endif

/* Compile-time assertion helper */
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#define SPSC_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#elif defined(__cplusplus) && __cplusplus >= 201103L
#define SPSC_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#elif defined(_MSC_VER)
#define SPSC_STATIC_ASSERT(cond, msg) typedef char spsc_static_assert_##__LINE__[(cond)?1:-1]
#else
#define SPSC_STATIC_ASSERT(cond, msg) typedef char spsc_static_assert_##__LINE__[(cond)?1:-1]
#endif

/* Verify capacity is power of 2 */
SPSC_STATIC_ASSERT((SPSC_QUEUE_CAPACITY > 0) &&
                   ((SPSC_QUEUE_CAPACITY & (SPSC_QUEUE_CAPACITY - 1)) == 0),
                   "SPSC_QUEUE_CAPACITY must be a power of 2");

/* Verify capacity is reasonable */
SPSC_STATIC_ASSERT(SPSC_QUEUE_CAPACITY >= 2 && SPSC_QUEUE_CAPACITY <= 65536,
                   "SPSC_QUEUE_CAPACITY must be between 2 and 65536");

/* ========================================================================
 * SPSC QUEUE STRUCTURE
 *
 * Layout optimized to prevent false sharing:
 * - Producer data on one cache line (only written by producer)
 * - Consumer data on another cache line (only written by consumer)
 * - Shared read-only data separate
 * - Slot array on separate cache lines
 *
 * Memory ordering invariants:
 * - head is updated with release after slot data is written
 * - tail is updated with release after slot data is read
 * - Cross-thread reads use acquire semantics
 * ======================================================================== */

/* Slot structure - holds pointer and metadata
 * Padded to reduce false sharing between adjacent slots */
typedef struct {
    void *buffer;           /* Pointer to sample buffer */
    uint32_t dropped;       /* Dropped buffers count for this slot */
    uint32_t _pad[1];       /* Padding for alignment */
} spsc_slot_t;

/* SPSC Queue structure */
typedef struct {
    /* ---- Producer cache line (USB callback thread) ----
     * Only the producer thread writes to these fields.
     * Consumer reads head with acquire semantics. */
    OPT_ALIGN(OPT_CACHE_LINE_SIZE) struct {
        volatile uint32_t head;     /* Write position (only producer writes) */
        uint32_t cached_tail;       /* Local cache of tail for fast space check */
        uint64_t enqueue_count;     /* Total successful enqueues (stats) */
        uint64_t overflow_count;    /* Overflow events when queue full (stats) */
    } producer;

    /* ---- Consumer cache line (consumer thread) ----
     * Only the consumer thread writes to these fields.
     * Producer reads tail with acquire semantics. */
    OPT_ALIGN(OPT_CACHE_LINE_SIZE) struct {
        volatile uint32_t tail;     /* Read position (only consumer writes) */
        uint32_t cached_head;       /* Local cache of head for fast data check */
        uint64_t dequeue_count;     /* Total successful dequeues (stats) */
    } consumer;

    /* ---- Shared data (read-only after init) ----
     * These fields are set during init and never modified.
     * No synchronization needed for reads. */
    OPT_ALIGN(OPT_CACHE_LINE_SIZE) struct {
        uint32_t capacity;          /* Queue capacity (power of 2) */
        uint32_t mask;              /* capacity - 1 for fast modulo */
        uint32_t initialized;       /* Magic value to detect uninitialized queue */
    } shared;

    /* ---- Slot array (separate cache lines) ----
     * Producer writes to slots[head], consumer reads from slots[tail].
     * No concurrent access to same slot due to head/tail separation. */
    OPT_ALIGN(OPT_CACHE_LINE_SIZE) spsc_slot_t slots[SPSC_QUEUE_CAPACITY];

} spsc_queue_t;

/* Magic value for initialized queue */
#define SPSC_QUEUE_MAGIC 0x53505343  /* "SPSC" */

/* ========================================================================
 * INITIALIZATION
 * ======================================================================== */

/**
 * @brief Initialize SPSC queue
 *
 * Must be called before any other operations. Not thread-safe.
 * Call only once per queue, or after spsc_queue_destroy().
 *
 * @param q Pointer to queue structure
 * @return 0 on success, -1 on error
 */
static OPT_INLINE int spsc_queue_init(spsc_queue_t *q)
{
    if (!q) return -1;

    /* Initialize producer state */
    q->producer.head = 0;
    q->producer.cached_tail = 0;
    q->producer.enqueue_count = 0;
    q->producer.overflow_count = 0;

    /* Initialize consumer state */
    q->consumer.tail = 0;
    q->consumer.cached_head = 0;
    q->consumer.dequeue_count = 0;

    /* Set capacity (compile-time constant, power of 2) */
    q->shared.capacity = SPSC_QUEUE_CAPACITY;
    q->shared.mask = SPSC_QUEUE_CAPACITY - 1;

    /* Clear slots */
    for (uint32_t i = 0; i < SPSC_QUEUE_CAPACITY; i++) {
        q->slots[i].buffer = NULL;
        q->slots[i].dropped = 0;
    }

    /* Full fence to ensure all writes are visible */
    SPSC_MEMORY_FENCE();

    /* Mark as initialized (after fence so all init is visible) */
    q->shared.initialized = SPSC_QUEUE_MAGIC;

    return 0;
}

/**
 * @brief Check if queue is properly initialized
 * @param q Pointer to queue structure
 * @return true if initialized, false otherwise
 */
static OPT_INLINE bool spsc_queue_is_initialized(const spsc_queue_t *q)
{
    return q && q->shared.initialized == SPSC_QUEUE_MAGIC;
}

/**
 * @brief Reset queue to empty state
 *
 * WARNING: Call only when both producer and consumer are idle!
 * This is NOT thread-safe and will corrupt data if called during operation.
 *
 * @param q Pointer to queue structure
 */
static OPT_INLINE void spsc_queue_reset(spsc_queue_t *q)
{
    if (!q) return;

    /* Reset indices */
    q->producer.head = 0;
    q->producer.cached_tail = 0;
    q->consumer.tail = 0;
    q->consumer.cached_head = 0;

    /* Full fence to make reset visible to all threads */
    SPSC_MEMORY_FENCE();
}

/**
 * @brief Destroy queue (mark as uninitialized)
 *
 * Call only when both producer and consumer are idle.
 * After this, the queue must be re-initialized before use.
 *
 * @param q Pointer to queue structure
 */
static OPT_INLINE void spsc_queue_destroy(spsc_queue_t *q)
{
    if (!q) return;
    q->shared.initialized = 0;
    SPSC_MEMORY_FENCE();
}

/* ========================================================================
 * PRODUCER OPERATIONS (USB callback thread)
 * ======================================================================== */

/**
 * @brief Enqueue a buffer (producer only, wait-free)
 *
 * Thread safety: Only call from the producer thread.
 * Wait-free: Returns immediately, never blocks.
 *
 * @param q Pointer to queue structure
 * @param buffer Buffer pointer to enqueue
 * @param dropped Dropped buffer count for this slot
 * @return true on success, false if queue is full
 */
static OPT_INLINE bool spsc_queue_enqueue(spsc_queue_t *q, void *buffer, uint32_t dropped)
{
    uint32_t head = q->producer.head;
    uint32_t next_head = (head + 1) & q->shared.mask;

    /* Check space (use cached tail for speed) */
    if (OPT_UNLIKELY(next_head == q->producer.cached_tail)) {
        /* Refresh cached tail with acquire to see consumer's latest progress */
        q->producer.cached_tail = SPSC_ATOMIC_LOAD_ACQ(&q->consumer.tail);
        if (next_head == q->producer.cached_tail) {
            /* Queue is full */
            q->producer.overflow_count++;
            return false;
        }
    }

    /* Store data in slot
     * These writes must complete before head is updated */
    q->slots[head].buffer = buffer;
    q->slots[head].dropped = dropped;

    /* Compiler barrier: ensure slot writes are not reordered past head update
     * The ATOMIC_STORE_REL provides the necessary memory ordering for other
     * threads, but we need a compiler barrier to prevent the compiler from
     * reordering the slot writes after the head update */
    SPSC_COMPILER_BARRIER();

    /* Publish: update head with release semantics
     * This ensures all prior writes (slot data) are visible before head update */
    SPSC_ATOMIC_STORE_REL(&q->producer.head, next_head);
    q->producer.enqueue_count++;

    return true;
}

/**
 * @brief Swap buffer into queue slot (zero-copy enqueue)
 *
 * Atomically swaps the provided buffer with an empty slot's buffer.
 * Used for USB callback zero-copy pattern where the same buffers
 * are recycled between producer and consumer.
 *
 * Thread safety: Only call from the producer thread.
 * Wait-free: Returns immediately, never blocks.
 *
 * @param q Pointer to queue structure
 * @param buffer_ptr Pointer to buffer pointer (will be swapped with slot's buffer)
 * @param dropped Dropped buffer count for this slot
 * @return true on success, false if queue is full
 */
static OPT_INLINE bool spsc_queue_enqueue_swap(spsc_queue_t *q, void **buffer_ptr, uint32_t dropped)
{
    uint32_t head = q->producer.head;
    uint32_t next_head = (head + 1) & q->shared.mask;

    /* Check space */
    if (OPT_UNLIKELY(next_head == q->producer.cached_tail)) {
        q->producer.cached_tail = SPSC_ATOMIC_LOAD_ACQ(&q->consumer.tail);
        if (next_head == q->producer.cached_tail) {
            q->producer.overflow_count++;
            return false;
        }
    }

    /* Swap buffers (zero-copy)
     * Get the old buffer from slot before overwriting */
    void *temp = q->slots[head].buffer;
    q->slots[head].buffer = *buffer_ptr;
    q->slots[head].dropped = dropped;
    *buffer_ptr = temp;

    /* Compiler barrier before publishing */
    SPSC_COMPILER_BARRIER();

    /* Publish: update head with release semantics */
    SPSC_ATOMIC_STORE_REL(&q->producer.head, next_head);
    q->producer.enqueue_count++;

    return true;
}

/* ========================================================================
 * CONSUMER OPERATIONS (consumer thread)
 * ======================================================================== */

/**
 * @brief Get number of items in queue (approximate, consumer side)
 * @param q Pointer to queue structure
 * @return Number of items (may be stale)
 */
static OPT_INLINE uint32_t spsc_queue_count(spsc_queue_t *q)
{
    uint32_t head = SPSC_ATOMIC_LOAD_ACQ(&q->producer.head);
    uint32_t tail = q->consumer.tail;
    return (head - tail) & q->shared.mask;
}

/**
 * @brief Dequeue a buffer (consumer only)
 *
 * Thread safety: Only call from the consumer thread.
 * Wait-free: Returns immediately, never blocks.
 *
 * @param q Pointer to queue structure
 * @param buffer_out Output: buffer pointer (must not be NULL)
 * @param dropped_out Output: dropped count (can be NULL)
 * @return true on success, false if queue is empty
 */
static OPT_INLINE bool spsc_queue_dequeue(spsc_queue_t *q, void **buffer_out, uint32_t *dropped_out)
{
    uint32_t tail = q->consumer.tail;

    /* Check data available (use cached head for speed) */
    if (OPT_UNLIKELY(tail == q->consumer.cached_head)) {
        /* Refresh cached head with acquire to see producer's latest progress */
        q->consumer.cached_head = SPSC_ATOMIC_LOAD_ACQ(&q->producer.head);
        if (tail == q->consumer.cached_head) {
            /* Queue is empty */
            return false;
        }
    }

    /* Compiler barrier: ensure we read slot data after confirming data is available
     * The acquire on cached_head provides memory ordering, but compiler barrier
     * prevents speculative reads before the check */
    SPSC_COMPILER_BARRIER();

    /* Read data from slot
     * Safe because producer won't touch this slot until we update tail */
    *buffer_out = q->slots[tail].buffer;
    if (dropped_out) {
        *dropped_out = q->slots[tail].dropped;
    }

    /* Calculate next position */
    uint32_t next_tail = (tail + 1) & q->shared.mask;

    /* Prefetch next slot for performance */
    OPT_PREFETCH_READ(&q->slots[next_tail]);

    /* Compiler barrier: ensure slot reads complete before tail update */
    SPSC_COMPILER_BARRIER();

    /* Release: update tail with release semantics
     * This signals to producer that slot is now available for reuse */
    SPSC_ATOMIC_STORE_REL(&q->consumer.tail, next_tail);
    q->consumer.dequeue_count++;

    return true;
}

/**
 * @brief Peek at front buffer without removing (consumer only)
 * @param q Pointer to queue structure
 * @param buffer_out Output: buffer pointer
 * @param dropped_out Output: dropped count (can be NULL)
 * @return true on success, false if queue is empty
 */
static OPT_INLINE bool spsc_queue_peek(spsc_queue_t *q, void **buffer_out, uint32_t *dropped_out)
{
    uint32_t tail = q->consumer.tail;

    /* Check data available */
    if (tail == q->consumer.cached_head) {
        q->consumer.cached_head = SPSC_ATOMIC_LOAD_ACQ(&q->producer.head);
        if (tail == q->consumer.cached_head) {
            return false;
        }
    }

    /* Read data without advancing tail */
    *buffer_out = q->slots[tail].buffer;
    if (dropped_out) {
        *dropped_out = q->slots[tail].dropped;
    }

    return true;
}

/* ========================================================================
 * STATISTICS (Thread-safe reads)
 * ======================================================================== */

/**
 * @brief Get queue statistics
 * @param q Pointer to queue structure
 * @param enqueues Output: total enqueue operations
 * @param dequeues Output: total dequeue operations
 * @param overflows Output: overflow events (queue was full)
 */
static OPT_INLINE void spsc_queue_stats(const spsc_queue_t *q,
                                        uint64_t *enqueues,
                                        uint64_t *dequeues,
                                        uint64_t *overflows)
{
    if (enqueues)  *enqueues = q->producer.enqueue_count;
    if (dequeues)  *dequeues = q->consumer.dequeue_count;
    if (overflows) *overflows = q->producer.overflow_count;
}

/**
 * @brief Check if queue is empty (approximate, thread-safe)
 *
 * This is an approximate check - the queue state may change immediately
 * after this function returns. Safe to call from any thread.
 *
 * @param q Pointer to queue structure
 * @return true if queue appears empty
 */
static OPT_INLINE bool spsc_queue_is_empty(const spsc_queue_t *q)
{
    /* Use explicit volatile reads to avoid casting away const */
    volatile const uint32_t *head_ptr = &q->producer.head;
    volatile const uint32_t *tail_ptr = &q->consumer.tail;

    uint32_t head = *head_ptr;
    SPSC_COMPILER_BARRIER();
    uint32_t tail = *tail_ptr;

    return head == tail;
}

/**
 * @brief Check if queue is full (approximate, thread-safe)
 *
 * This is an approximate check - the queue state may change immediately
 * after this function returns. Safe to call from any thread.
 *
 * @param q Pointer to queue structure
 * @return true if queue appears full
 */
static OPT_INLINE bool spsc_queue_is_full(const spsc_queue_t *q)
{
    /* Use explicit volatile reads to avoid casting away const */
    volatile const uint32_t *head_ptr = &q->producer.head;
    volatile const uint32_t *tail_ptr = &q->consumer.tail;

    uint32_t head = *head_ptr;
    SPSC_COMPILER_BARRIER();
    uint32_t tail = *tail_ptr;

    uint32_t next_head = (head + 1) & q->shared.mask;
    return (next_head == tail);
}

#ifdef __cplusplus
}
#endif

#endif /* SPSC_QUEUE_H */
