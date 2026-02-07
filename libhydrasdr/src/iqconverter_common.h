/*
 * IQCONVERTER COMMON DEFINITIONS
 *
 * Shared constants and macros for all DDC implementations.
 *
 * Copyright (C) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef IQCONVERTER_COMMON_H
#define IQCONVERTER_COMMON_H

#include "compat_opt.h"

/*
 * DC Removal Configuration
 * Alpha = 0.01 gives ~100 sample time constant for DC tracking
 */
#define DC_REMOVAL_ALPHA 0.01f

/*
 * FIR History Buffer Configuration
 * Must be power-of-2 for efficient circular buffer indexing
 */
#define HISTORY_SIZE 512
#define FIR_MASK (HISTORY_SIZE - 1)

/*
 * Memory Alignment Macros
 * 64-byte alignment for AVX-512 compatibility
 */
#define ALIGN64(x) (((x) + 63) & ~63)

/*
 * Aligned Memory Allocation
 * Use compat_opt.h functions for cross-platform support
 */
#define iqconv_aligned_alloc(size) opt_aligned_alloc((size), 64)
#define iqconv_aligned_free(ptr)   opt_aligned_free(ptr)

/*
 * Int16 DDC Configuration
 * Fixed-point DC removal alpha coefficient (Q15 format)
 * DC_ALPHA_Q15 = 32100 ~ 0.979 gives fast DC tracking
 */
#define DC_ALPHA_Q15 32100

/*
 * Int16 FIR Circular Buffer Configuration
 * Power-of-2 size for efficient branchless modulo via bitmask
 */
#define FIR_BUF_SIZE 512
#define FIR_BUF_MASK 511

#endif /* IQCONVERTER_COMMON_H */
