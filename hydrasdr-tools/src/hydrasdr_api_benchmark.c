/*
 * Copyright (c) 2025-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * This file is part of HydraSDR.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * HydraSDR API Benchmark Tool
 *
 * Benchmarks all exported hydrasdr.h APIs with timing statistics.
 * Measures USB round-trip latency for device operations.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#endif

#include <hydrasdr.h>

/* Include internal headers for DDC benchmarking (requires static linking) */
#include "iqconverter.h"
#include "cpu_features.h"
#include "filters.h"
#include "filters_opt.h"

/* ============================================================================
 * System Information
 * ========================================================================== */

static void print_system_info(void)
{
	const cpu_features_t *cpu = cpu_features_detect();

	printf("System Information:\n");
	printf("  Architecture:    %s\n", cpu_get_arch_name());
	printf("  SIMD Level:      %s\n", cpu_get_simd_name());

	/* Print detected CPU features */
	printf("  CPU Features:    ");
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
	if (cpu->sse2)    printf("SSE2 ");
	if (cpu->sse42)   printf("SSE4.2 ");
	if (cpu->avx)     printf("AVX ");
	if (cpu->avx2)    printf("AVX2 ");
	if (cpu->fma)     printf("FMA ");
	if (cpu->avx512f) printf("AVX-512 ");
#elif defined(__aarch64__) || defined(_M_ARM64) || defined(__arm__) || defined(_M_ARM)
	if (cpu->neon)    printf("NEON ");
	if (cpu->sve)     printf("SVE ");
	if (cpu->sve2)    printf("SVE2 ");
#endif
	printf("\n\n");
}

/* ============================================================================
 * Timing Utilities
 * ========================================================================== */

#ifdef _WIN32
static LARGE_INTEGER perf_freq;
static bool perf_freq_init = false;

static void init_timer(void)
{
	if (!perf_freq_init) {
		QueryPerformanceFrequency(&perf_freq);
		perf_freq_init = true;
	}
}

static double get_time_us(void)
{
	LARGE_INTEGER counter;
	QueryPerformanceCounter(&counter);
	return (double)counter.QuadPart * 1000000.0 / (double)perf_freq.QuadPart;
}
#else
static void init_timer(void) { }

static double get_time_us(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (double)ts.tv_sec * 1000000.0 + (double)ts.tv_nsec / 1000.0;
}
#endif

/* ============================================================================
 * Statistics
 * ========================================================================== */

typedef struct {
	const char *name;
	int iterations;
	int success_count;
	int fail_count;
	double min_us;
	double max_us;
	double sum_us;
	double sum_sq_us;
} bench_stats_t;

static void stats_init(bench_stats_t *s, const char *name)
{
	memset(s, 0, sizeof(*s));
	s->name = name;
	s->min_us = 1e12;
	s->max_us = 0.0;
}

static void stats_add(bench_stats_t *s, double elapsed_us, bool success)
{
	s->iterations++;
	if (success) {
		s->success_count++;
		if (elapsed_us < s->min_us) s->min_us = elapsed_us;
		if (elapsed_us > s->max_us) s->max_us = elapsed_us;
		s->sum_us += elapsed_us;
		s->sum_sq_us += elapsed_us * elapsed_us;
	} else {
		s->fail_count++;
	}
}

static void stats_print(const bench_stats_t *s)
{
	if (s->success_count == 0) {
		printf("%-34s FAILED (%d attempts)\n", s->name, s->iterations);
		return;
	}

	double avg = s->sum_us / s->success_count;
	double variance = (s->sum_sq_us / s->success_count) - (avg * avg);
	double stddev = (variance > 0) ? sqrt(variance) : 0.0;

	printf("%-34s min:%7.1fus max:%7.1fus avg:%7.1fus stddev:%6.1fus (%d/%d ok)\n",
	       s->name, s->min_us, s->max_us, avg, stddev,
	       s->success_count, s->iterations);
}

/* ============================================================================
 * Benchmark Macros
 * ========================================================================== */

#define BENCHMARK_ITERATIONS 100
#define BENCHMARK_ITERATIONS_SLOW 10  /* For slow peripheral APIs */
#define WARMUP_ITERATIONS 5
#define WARMUP_ITERATIONS_SLOW 2      /* For slow peripheral APIs */

#define BENCH_START(stats_ptr, func_name) \
	do { \
		stats_init((stats_ptr), (func_name)); \
	} while (0)

#define BENCH_ITER(stats_ptr, code, success_cond) \
	do { \
		double _start = get_time_us(); \
		code; \
		double _elapsed = get_time_us() - _start; \
		stats_add((stats_ptr), _elapsed, (success_cond)); \
	} while (0)

/* ============================================================================
 * Library-only Benchmarks (no device required)
 * ========================================================================== */

static void benchmark_library_apis(void)
{
	bench_stats_t stats;
	hydrasdr_lib_version_t ver;
	int i;

	printf("\nLibrary API Benchmarks (no device required)\n\n");

	/* hydrasdr_lib_version */
	BENCH_START(&stats, "hydrasdr_lib_version()");
	for (i = 0; i < WARMUP_ITERATIONS; i++) {
		hydrasdr_lib_version(&ver);
	}
	for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
		BENCH_ITER(&stats, hydrasdr_lib_version(&ver), true);
	}
	stats_print(&stats);

	/* hydrasdr_error_name */
	BENCH_START(&stats, "hydrasdr_error_name()");
	for (i = 0; i < WARMUP_ITERATIONS; i++) {
		(void)hydrasdr_error_name(HYDRASDR_SUCCESS);
	}
	for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
		const char *name;
		BENCH_ITER(&stats, name = hydrasdr_error_name(HYDRASDR_SUCCESS), name != NULL);
	}
	stats_print(&stats);

	/* hydrasdr_list_devices (USB enumeration) */
	BENCH_START(&stats, "hydrasdr_list_devices()");
	for (i = 0; i < WARMUP_ITERATIONS; i++) {
		(void)hydrasdr_list_devices(NULL, 0);
	}
	for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
		int count;
		BENCH_ITER(&stats, count = hydrasdr_list_devices(NULL, 0), count >= 0);
	}
	stats_print(&stats);

	/* iqconverter_list_algorithms */
	BENCH_START(&stats, "iqconverter_list_algorithms()");
	for (i = 0; i < WARMUP_ITERATIONS; i++) {
		(void)iqconverter_list_algorithms(NULL, 0);
	}
	for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
		int count;
		BENCH_ITER(&stats, count = iqconverter_list_algorithms(NULL, 0), count >= 0);
	}
	stats_print(&stats);

	/* hydrasdr_list_conversion_algorithms */
	BENCH_START(&stats, "hydrasdr_list_conversion_algorithms()");
	for (i = 0; i < WARMUP_ITERATIONS; i++) {
		(void)hydrasdr_list_conversion_algorithms(NULL, NULL, 0);
	}
	for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
		int count;
		BENCH_ITER(&stats, count = hydrasdr_list_conversion_algorithms(NULL, NULL, 0), count >= 0);
	}
	stats_print(&stats);

	/* iqconverter_create/free cycle */
	BENCH_START(&stats, "iqconverter_create()+free()");
	for (i = 0; i < WARMUP_ITERATIONS; i++) {
		iqconverter_t *cnv = iqconverter_create(HB_KERNEL_FLOAT_47_OPT,
							HB_KERNEL_FLOAT_LEN_47_OPT,
							"float32_legacy");
		if (cnv) iqconverter_free(cnv);
	}
	for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
		iqconverter_t *cnv;
		BENCH_ITER(&stats,
		           cnv = iqconverter_create(HB_KERNEL_FLOAT_47_OPT,
		                                    HB_KERNEL_FLOAT_LEN_47_OPT,
		                                    "float32_legacy");
		           if (cnv) iqconverter_free(cnv),
		           cnv != NULL);
	}
	stats_print(&stats);
}

/* ============================================================================
 * Device API Benchmarks (requires connected device)
 * ========================================================================== */

static void benchmark_device_info_apis(struct hydrasdr_device *dev)
{
	bench_stats_t stats;
	int i, result;

	printf("\nDevice Information API Benchmarks\n\n");

	/* hydrasdr_get_device_info - returns cached data from init */
	{
		hydrasdr_device_info_t info;
		BENCH_START(&stats, "hydrasdr_get_device_info() [cached]");
		for (i = 0; i < WARMUP_ITERATIONS; i++) {
			hydrasdr_get_device_info(dev, &info);
		}
		for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
			BENCH_ITER(&stats,
			           result = hydrasdr_get_device_info(dev, &info),
			           result == HYDRASDR_SUCCESS);
		}
		stats_print(&stats);
	}

	/* hydrasdr_get_samplerates - returns cached data from init */
	{
		uint32_t rates[32];
		uint32_t count = 0;
		hydrasdr_get_samplerates(dev, &count, 0);
		if (count > 32) count = 32;
		BENCH_START(&stats, "hydrasdr_get_samplerates() [cached]");
		for (i = 0; i < WARMUP_ITERATIONS; i++) {
			hydrasdr_get_samplerates(dev, rates, count);
		}
		for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
			BENCH_ITER(&stats,
			           result = hydrasdr_get_samplerates(dev, rates, count),
			           result >= 0);
		}
		stats_print(&stats);
	}

	/* hydrasdr_get_bandwidths - returns cached data from init */
	{
		uint32_t bws[32];
		uint32_t count = 0;
		hydrasdr_get_bandwidths(dev, &count, 0);
		if (count > 32) count = 32;
		BENCH_START(&stats, "hydrasdr_get_bandwidths() [cached]");
		for (i = 0; i < WARMUP_ITERATIONS; i++) {
			hydrasdr_get_bandwidths(dev, bws, count);
		}
		for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
			BENCH_ITER(&stats,
			           result = hydrasdr_get_bandwidths(dev, bws, count),
			           result >= 0);
		}
		stats_print(&stats);
	}

	/* hydrasdr_is_streaming - returns local state, no USB */
	BENCH_START(&stats, "hydrasdr_is_streaming() [local]");
	for (i = 0; i < WARMUP_ITERATIONS; i++) {
		hydrasdr_is_streaming(dev);
	}
	for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
		BENCH_ITER(&stats,
		           result = hydrasdr_is_streaming(dev),
		           result == 0 || result == 1);
	}
	stats_print(&stats);
}

static void benchmark_device_config_apis(struct hydrasdr_device *dev)
{
	bench_stats_t stats;
	int i, result;
	hydrasdr_device_info_t info;

	printf("\nDevice Configuration API Benchmarks (USB round-trip)\n\n");

	/* Get device info for capability checking */
	hydrasdr_get_device_info(dev, &info);

	/* hydrasdr_set_freq - use device's actual frequency range */
	if (info.min_frequency > 0 && info.max_frequency > info.min_frequency) {
		uint64_t freq_min = info.min_frequency;
		uint64_t freq_max = info.max_frequency;
		uint64_t freq_step = (freq_max - freq_min) / BENCHMARK_ITERATIONS;

		if (freq_step == 0)
			freq_step = 1000000; /* Minimum 1 MHz step */

		/* Linear sweep benchmark */
		BENCH_START(&stats, "hydrasdr_set_freq() sweep");
		for (i = 0; i < WARMUP_ITERATIONS; i++) {
			hydrasdr_set_freq(dev, freq_min + freq_step * i);
		}
		for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
			/* Sweep from min to max frequency */
			uint64_t freq = freq_min + freq_step * i;
			BENCH_ITER(&stats,
				   result = hydrasdr_set_freq(dev, freq),
				   result == HYDRASDR_SUCCESS);
		}
		stats_print(&stats);
		printf("  Range: %llu - %llu MHz (step: %llu MHz)\n",
		       (unsigned long long)(freq_min / 1000000),
		       (unsigned long long)(freq_max / 1000000),
		       (unsigned long long)(freq_step / 1000000));

		/* Big jump benchmark (min <-> max) - measures PLL worst-case settling */
		BENCH_START(&stats, "hydrasdr_set_freq() jump");
		for (i = 0; i < WARMUP_ITERATIONS; i++) {
			hydrasdr_set_freq(dev, (i & 1) ? freq_max : freq_min);
		}
		for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
			/* Alternate between min and max frequency */
			uint64_t freq = (i & 1) ? freq_max : freq_min;
			BENCH_ITER(&stats,
				   result = hydrasdr_set_freq(dev, freq),
				   result == HYDRASDR_SUCCESS);
		}
		stats_print(&stats);
		printf("  Jump: %llu MHz <-> %llu MHz (PLL worst-case)\n",
		       (unsigned long long)(freq_min / 1000000),
		       (unsigned long long)(freq_max / 1000000));
	} else {
		printf("hydrasdr_set_freq()                SKIPPED (no frequency range in device info)\n");
	}

	/* hydrasdr_set_bandwidth - set BEFORE samplerate (v1.1.0 pre-streaming order) */
	if (info.features & HYDRASDR_CAP_BANDWIDTH) {
		uint32_t bws[32];
		uint32_t bw_count = 0;

		/* Query available bandwidths */
		hydrasdr_get_bandwidths(dev, &bw_count, 0);
		if (bw_count > 0 && bw_count <= 32) {
			hydrasdr_get_bandwidths(dev, bws, bw_count);

			/* Use middle bandwidth for benchmark */
			uint32_t test_bw = bws[bw_count / 2];

			BENCH_START(&stats, "hydrasdr_set_bandwidth()");
			for (i = 0; i < WARMUP_ITERATIONS; i++) {
				hydrasdr_set_bandwidth(dev, test_bw);
			}
			for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
				/* Cycle through available bandwidths */
				uint32_t bw = bws[i % bw_count];
				BENCH_ITER(&stats,
					   result = hydrasdr_set_bandwidth(dev, bw),
					   result == HYDRASDR_SUCCESS);
			}
			stats_print(&stats);
			printf("  Available bandwidths: %u (using %u Hz)\n", bw_count, test_bw);
		} else {
			printf("hydrasdr_set_bandwidth()           SKIPPED (no bandwidths available)\n");
		}
	}

	/* hydrasdr_set_samplerate - set AFTER bandwidth (v1.1.0 pre-streaming order) */
	{
		uint32_t rates[32];
		uint32_t rate_count = 0;

		/* Query available sample rates */
		hydrasdr_get_samplerates(dev, &rate_count, 0);
		if (rate_count > 0 && rate_count <= 32) {
			hydrasdr_get_samplerates(dev, rates, rate_count);

			/* Use middle sample rate for benchmark */
			uint32_t test_rate = rates[rate_count / 2];

			BENCH_START(&stats, "hydrasdr_set_samplerate()");
			for (i = 0; i < WARMUP_ITERATIONS; i++) {
				hydrasdr_set_samplerate(dev, test_rate);
			}
			for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
				/* Cycle through available sample rates */
				uint32_t rate = rates[i % rate_count];
				BENCH_ITER(&stats,
					   result = hydrasdr_set_samplerate(dev, rate),
					   result == HYDRASDR_SUCCESS);
			}
			stats_print(&stats);
			printf("  Available rates: %u (using %u Hz)\n", rate_count, test_rate);
		} else {
			printf("hydrasdr_set_samplerate()          SKIPPED (no sample rates available)\n");
		}
	}

	/* hydrasdr_set_sample_type */
	BENCH_START(&stats, "hydrasdr_set_sample_type()");
	for (i = 0; i < WARMUP_ITERATIONS; i++) {
		hydrasdr_set_sample_type(dev, HYDRASDR_SAMPLE_FLOAT32_IQ);
	}
	for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
		BENCH_ITER(&stats,
		           result = hydrasdr_set_sample_type(dev, HYDRASDR_SAMPLE_FLOAT32_IQ),
		           result == HYDRASDR_SUCCESS);
	}
	stats_print(&stats);

	/* hydrasdr_set_packing (if supported) */
	if (info.features & HYDRASDR_CAP_PACKING) {
		BENCH_START(&stats, "hydrasdr_set_packing()");
		for (i = 0; i < WARMUP_ITERATIONS; i++) {
			hydrasdr_set_packing(dev, 1);
		}
		for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
			BENCH_ITER(&stats,
			           result = hydrasdr_set_packing(dev, 1),
			           result == HYDRASDR_SUCCESS);
		}
		stats_print(&stats);
	}

	/* NOTE: hydrasdr_set_rf_bias() is NOT benchmarked - enabling bias tee
	 * can damage equipment connected to the RF port */
}

static void benchmark_gain_apis(struct hydrasdr_device *dev)
{
	bench_stats_t stats;
	int i, result;
	hydrasdr_device_info_t info;

	printf("\nGain Control API Benchmarks (USB round-trip)\n\n");

	hydrasdr_get_device_info(dev, &info);

	/* VGA gain (most devices support this) */
	if (info.features & HYDRASDR_CAP_VGA_GAIN) {
		BENCH_START(&stats, "hydrasdr_set_vga_gain()");
		for (i = 0; i < WARMUP_ITERATIONS; i++) {
			hydrasdr_set_vga_gain(dev, 5);
		}
		for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
			uint8_t gain = (uint8_t)(i % (info.vga_gain.max_value + 1));
			BENCH_ITER(&stats,
			           result = hydrasdr_set_vga_gain(dev, gain),
			           result == HYDRASDR_SUCCESS);
		}
		stats_print(&stats);
	}

	/* LNA gain */
	if (info.features & HYDRASDR_CAP_LNA_GAIN) {
		BENCH_START(&stats, "hydrasdr_set_lna_gain()");
		for (i = 0; i < WARMUP_ITERATIONS; i++) {
			hydrasdr_set_lna_gain(dev, 5);
		}
		for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
			uint8_t gain = (uint8_t)(i % (info.lna_gain.max_value + 1));
			BENCH_ITER(&stats,
			           result = hydrasdr_set_lna_gain(dev, gain),
			           result == HYDRASDR_SUCCESS);
		}
		stats_print(&stats);
	}

	/* Mixer gain */
	if (info.features & HYDRASDR_CAP_MIXER_GAIN) {
		BENCH_START(&stats, "hydrasdr_set_mixer_gain()");
		for (i = 0; i < WARMUP_ITERATIONS; i++) {
			hydrasdr_set_mixer_gain(dev, 5);
		}
		for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
			uint8_t gain = (uint8_t)(i % (info.mixer_gain.max_value + 1));
			BENCH_ITER(&stats,
			           result = hydrasdr_set_mixer_gain(dev, gain),
			           result == HYDRASDR_SUCCESS);
		}
		stats_print(&stats);
	}

	/* RF gain */
	if (info.features & HYDRASDR_CAP_RF_GAIN) {
		BENCH_START(&stats, "hydrasdr_set_gain(RF)");
		for (i = 0; i < WARMUP_ITERATIONS; i++) {
			hydrasdr_set_gain(dev, HYDRASDR_GAIN_TYPE_RF, 5);
		}
		for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
			uint8_t gain = (uint8_t)(i % (info.rf_gain.max_value + 1));
			BENCH_ITER(&stats,
			           result = hydrasdr_set_gain(dev, HYDRASDR_GAIN_TYPE_RF, gain),
			           result == HYDRASDR_SUCCESS);
		}
		stats_print(&stats);
	}

	/* Filter gain */
	if (info.features & HYDRASDR_CAP_FILTER_GAIN) {
		BENCH_START(&stats, "hydrasdr_set_gain(FILTER)");
		for (i = 0; i < WARMUP_ITERATIONS; i++) {
			hydrasdr_set_gain(dev, HYDRASDR_GAIN_TYPE_FILTER, 5);
		}
		for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
			uint8_t gain = (uint8_t)(i % (info.filter_gain.max_value + 1));
			BENCH_ITER(&stats,
			           result = hydrasdr_set_gain(dev, HYDRASDR_GAIN_TYPE_FILTER, gain),
			           result == HYDRASDR_SUCCESS);
		}
		stats_print(&stats);
	}

	/* Linearity gain mode */
	if (info.features & HYDRASDR_CAP_LINEARITY_GAIN) {
		BENCH_START(&stats, "hydrasdr_set_linearity_gain()");
		for (i = 0; i < WARMUP_ITERATIONS; i++) {
			hydrasdr_set_linearity_gain(dev, 10);
		}
		for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
			uint8_t gain = (uint8_t)(i % (info.linearity_gain.max_value + 1));
			BENCH_ITER(&stats,
			           result = hydrasdr_set_linearity_gain(dev, gain),
			           result == HYDRASDR_SUCCESS);
		}
		stats_print(&stats);
	}

	/* Sensitivity gain mode */
	if (info.features & HYDRASDR_CAP_SENSITIVITY_GAIN) {
		BENCH_START(&stats, "hydrasdr_set_sensitivity_gain()");
		for (i = 0; i < WARMUP_ITERATIONS; i++) {
			hydrasdr_set_sensitivity_gain(dev, 10);
		}
		for (i = 0; i < BENCHMARK_ITERATIONS; i++) {
			uint8_t gain = (uint8_t)(i % (info.sensitivity_gain.max_value + 1));
			BENCH_ITER(&stats,
			           result = hydrasdr_set_sensitivity_gain(dev, gain),
			           result == HYDRASDR_SUCCESS);
		}
		stats_print(&stats);
	}
}

static void benchmark_peripheral_apis(struct hydrasdr_device *dev)
{
	bench_stats_t stats;
	int i, result;
	hydrasdr_device_info_t info;

	printf("\nPeripheral API Benchmarks (USB round-trip, %d iterations)\n\n",
	       BENCHMARK_ITERATIONS_SLOW);

	hydrasdr_get_device_info(dev, &info);

	/* GPIO read/write */
	if (info.features & HYDRASDR_CAP_GPIO) {
		uint8_t val;

		BENCH_START(&stats, "hydrasdr_gpio_read()");
		for (i = 0; i < WARMUP_ITERATIONS_SLOW; i++) {
			hydrasdr_gpio_read(dev, HYDRASDR_GPIO_PORT0, HYDRASDR_GPIO_PIN0, &val);
		}
		for (i = 0; i < BENCHMARK_ITERATIONS_SLOW; i++) {
			BENCH_ITER(&stats,
			           result = hydrasdr_gpio_read(dev, HYDRASDR_GPIO_PORT0, HYDRASDR_GPIO_PIN0, &val),
			           result == HYDRASDR_SUCCESS);
		}
		stats_print(&stats);

		BENCH_START(&stats, "hydrasdr_gpiodir_read()");
		for (i = 0; i < WARMUP_ITERATIONS_SLOW; i++) {
			hydrasdr_gpiodir_read(dev, HYDRASDR_GPIO_PORT0, HYDRASDR_GPIO_PIN0, &val);
		}
		for (i = 0; i < BENCHMARK_ITERATIONS_SLOW; i++) {
			BENCH_ITER(&stats,
			           result = hydrasdr_gpiodir_read(dev, HYDRASDR_GPIO_PORT0, HYDRASDR_GPIO_PIN0, &val),
			           result == HYDRASDR_SUCCESS);
		}
		stats_print(&stats);
	}

	/* Clock generator read - comprehensive test with different access patterns */
	if (info.features & HYDRASDR_CAP_CLOCKGEN) {
		uint8_t val;
		uint32_t reg_count = 0;

		/* Get clock generator register count from component info */
		for (i = 0; i < info.component_count && i < HYDRASDR_MAX_COMPONENTS; i++) {
			if (info.components[i].type == HYDRASDR_COMP_CLOCKGEN) {
				reg_count = info.components[i].register_count;
				printf("Clock Generator: %s (%u registers)\n",
				       info.components[i].name, reg_count);
				break;
			}
		}

		if (reg_count == 0) {
			printf("hydrasdr_clockgen_read()       SKIPPED (no register count)\n");
		} else {
			/* Single register read (register 0) */
			BENCH_START(&stats, "hydrasdr_clockgen_read(1 reg)");
			for (i = 0; i < WARMUP_ITERATIONS_SLOW; i++) {
				hydrasdr_clockgen_read(dev, 0, &val);
			}
			for (i = 0; i < BENCHMARK_ITERATIONS_SLOW; i++) {
				BENCH_ITER(&stats,
					   result = hydrasdr_clockgen_read(dev, 0, &val),
					   result == HYDRASDR_SUCCESS);
			}
			stats_print(&stats);

			/* Read 4 consecutive registers */
			if (reg_count >= 4) {
				BENCH_START(&stats, "hydrasdr_clockgen_read(4 regs)");
				for (i = 0; i < WARMUP_ITERATIONS_SLOW; i++) {
					hydrasdr_clockgen_read(dev, 0, &val);
					hydrasdr_clockgen_read(dev, 1, &val);
					hydrasdr_clockgen_read(dev, 2, &val);
					hydrasdr_clockgen_read(dev, 3, &val);
				}
				for (i = 0; i < BENCHMARK_ITERATIONS_SLOW; i++) {
					double _start = get_time_us();
					result = hydrasdr_clockgen_read(dev, 0, &val);
					result |= hydrasdr_clockgen_read(dev, 1, &val);
					result |= hydrasdr_clockgen_read(dev, 2, &val);
					result |= hydrasdr_clockgen_read(dev, 3, &val);
					double _elapsed = get_time_us() - _start;
					stats_add(&stats, _elapsed, result == HYDRASDR_SUCCESS);
				}
				stats_print(&stats);
			}

			/* Read 16 consecutive registers */
			if (reg_count >= 16) {
				BENCH_START(&stats, "hydrasdr_clockgen_read(16 regs)");
				for (i = 0; i < WARMUP_ITERATIONS_SLOW; i++) {
					int r;
					for (r = 0; r < 16; r++)
						hydrasdr_clockgen_read(dev, r, &val);
				}
				for (i = 0; i < BENCHMARK_ITERATIONS_SLOW; i++) {
					double _start = get_time_us();
					int r;
					result = HYDRASDR_SUCCESS;
					for (r = 0; r < 16; r++) {
						if (hydrasdr_clockgen_read(dev, r, &val) != HYDRASDR_SUCCESS)
							result = HYDRASDR_ERROR_LIBUSB;
					}
					double _elapsed = get_time_us() - _start;
					stats_add(&stats, _elapsed, result == HYDRASDR_SUCCESS);
				}
				stats_print(&stats);
			}

			/* Read all registers (sequential scan) */
			{
				char bench_name[64];
				int max_regs = (reg_count > 256) ? 256 : reg_count;
				snprintf(bench_name, sizeof(bench_name),
					 "hydrasdr_clockgen_read(%d regs)", max_regs);

				BENCH_START(&stats, bench_name);
				for (i = 0; i < WARMUP_ITERATIONS_SLOW; i++) {
					int r;
					for (r = 0; r < max_regs; r++)
						hydrasdr_clockgen_read(dev, r, &val);
				}
				for (i = 0; i < BENCHMARK_ITERATIONS_SLOW; i++) {
					double _start = get_time_us();
					int r;
					result = HYDRASDR_SUCCESS;
					for (r = 0; r < max_regs; r++) {
						if (hydrasdr_clockgen_read(dev, r, &val) != HYDRASDR_SUCCESS)
							result = HYDRASDR_ERROR_LIBUSB;
					}
					double _elapsed = get_time_us() - _start;
					stats_add(&stats, _elapsed, result == HYDRASDR_SUCCESS);
				}
				stats_print(&stats);
				printf("  Per-register avg: %.1fus\n",
				       (stats.sum_us / stats.success_count) / max_regs);
			}
		}
	}

	/* RF frontend read - comprehensive test with different access patterns */
	if (info.features & HYDRASDR_CAP_RF_FRONTEND) {
		uint32_t val;
		uint32_t reg_count = 0;

		/* Get RF frontend register count from component info */
		for (i = 0; i < info.component_count && i < HYDRASDR_MAX_COMPONENTS; i++) {
			if (info.components[i].type == HYDRASDR_COMP_RF_FRONTEND) {
				reg_count = info.components[i].register_count;
				printf("RF Frontend: %s (%u registers)\n",
				       info.components[i].name, reg_count);
				break;
			}
		}

		if (reg_count == 0) {
			printf("hydrasdr_rf_frontend_read()    SKIPPED (no register count)\n");
		} else {
			/* Single register read (register 0) */
			BENCH_START(&stats, "hydrasdr_rf_frontend_read(1 reg)");
			for (i = 0; i < WARMUP_ITERATIONS_SLOW; i++) {
				hydrasdr_rf_frontend_read(dev, 0, &val);
			}
			for (i = 0; i < BENCHMARK_ITERATIONS_SLOW; i++) {
				BENCH_ITER(&stats,
					   result = hydrasdr_rf_frontend_read(dev, 0, &val),
					   result == HYDRASDR_SUCCESS);
			}
			stats_print(&stats);

			/* Read 4 consecutive registers */
			if (reg_count >= 4) {
				BENCH_START(&stats, "hydrasdr_rf_frontend_read(4 regs)");
				for (i = 0; i < WARMUP_ITERATIONS_SLOW; i++) {
					hydrasdr_rf_frontend_read(dev, 0, &val);
					hydrasdr_rf_frontend_read(dev, 1, &val);
					hydrasdr_rf_frontend_read(dev, 2, &val);
					hydrasdr_rf_frontend_read(dev, 3, &val);
				}
				for (i = 0; i < BENCHMARK_ITERATIONS_SLOW; i++) {
					double _start = get_time_us();
					result = hydrasdr_rf_frontend_read(dev, 0, &val);
					result |= hydrasdr_rf_frontend_read(dev, 1, &val);
					result |= hydrasdr_rf_frontend_read(dev, 2, &val);
					result |= hydrasdr_rf_frontend_read(dev, 3, &val);
					double _elapsed = get_time_us() - _start;
					stats_add(&stats, _elapsed, result == HYDRASDR_SUCCESS);
				}
				stats_print(&stats);
			}

			/* Read 16 consecutive registers */
			if (reg_count >= 16) {
				BENCH_START(&stats, "hydrasdr_rf_frontend_read(16 regs)");
				for (i = 0; i < WARMUP_ITERATIONS_SLOW; i++) {
					int r;
					for (r = 0; r < 16; r++)
						hydrasdr_rf_frontend_read(dev, r, &val);
				}
				for (i = 0; i < BENCHMARK_ITERATIONS_SLOW; i++) {
					double _start = get_time_us();
					int r;
					result = HYDRASDR_SUCCESS;
					for (r = 0; r < 16; r++) {
						if (hydrasdr_rf_frontend_read(dev, r, &val) != HYDRASDR_SUCCESS)
							result = HYDRASDR_ERROR_LIBUSB;
					}
					double _elapsed = get_time_us() - _start;
					stats_add(&stats, _elapsed, result == HYDRASDR_SUCCESS);
				}
				stats_print(&stats);
			}

			/* Read all registers (sequential scan) */
			{
				char bench_name[64];
				int max_regs = (reg_count > 256) ? 256 : reg_count;
				snprintf(bench_name, sizeof(bench_name),
					 "hydrasdr_rf_frontend_read(%d regs)", max_regs);

				BENCH_START(&stats, bench_name);
				for (i = 0; i < WARMUP_ITERATIONS_SLOW; i++) {
					int r;
					for (r = 0; r < max_regs; r++)
						hydrasdr_rf_frontend_read(dev, r, &val);
				}
				for (i = 0; i < BENCHMARK_ITERATIONS_SLOW; i++) {
					double _start = get_time_us();
					int r;
					result = HYDRASDR_SUCCESS;
					for (r = 0; r < max_regs; r++) {
						if (hydrasdr_rf_frontend_read(dev, r, &val) != HYDRASDR_SUCCESS)
							result = HYDRASDR_ERROR_LIBUSB;
					}
					double _elapsed = get_time_us() - _start;
					stats_add(&stats, _elapsed, result == HYDRASDR_SUCCESS);
				}
				stats_print(&stats);
				printf("  Per-register avg: %.1fus\n",
				       (stats.sum_us / stats.success_count) / max_regs);
			}
		}
	}

	/* Temperature sensor */
	if (info.features & HYDRASDR_CAP_TEMPERATURE_SENSOR) {
		hydrasdr_temperature_t temp;
		float temp_min = 1000.0f;
		float temp_max = -1000.0f;

		BENCH_START(&stats, "hydrasdr_get_temperature()");
		for (i = 0; i < WARMUP_ITERATIONS_SLOW; i++) {
			hydrasdr_get_temperature(dev, &temp);
		}
		for (i = 0; i < BENCHMARK_ITERATIONS_SLOW; i++) {
			BENCH_ITER(&stats,
			           result = hydrasdr_get_temperature(dev, &temp),
			           result == HYDRASDR_SUCCESS);
			if (result == HYDRASDR_SUCCESS) {
				if (temp.temperature_celsius < temp_min)
					temp_min = temp.temperature_celsius;
				if (temp.temperature_celsius > temp_max)
					temp_max = temp.temperature_celsius;
			}
		}
		stats_print(&stats);
		printf("  Temperature range: %.2f - %.2f C\n", temp_min, temp_max);
	}
}

static void benchmark_spiflash_read(struct hydrasdr_device *dev)
{
	bench_stats_t stats;
	int i, result;
	hydrasdr_device_info_t info;
	unsigned char buf[256];

	hydrasdr_get_device_info(dev, &info);

	if (!(info.features & HYDRASDR_CAP_SPIFLASH)) {
		return;
	}

	printf("\nSPI Flash Read Benchmark (USB bulk transfer, %d iterations)\n\n",
	       BENCHMARK_ITERATIONS_SLOW);

	/* 1 byte read */
	BENCH_START(&stats, "hydrasdr_spiflash_read(1 byte)");
	for (i = 0; i < WARMUP_ITERATIONS_SLOW; i++) {
		hydrasdr_spiflash_read(dev, 0, 1, buf);
	}
	for (i = 0; i < BENCHMARK_ITERATIONS_SLOW; i++) {
		BENCH_ITER(&stats,
		           result = hydrasdr_spiflash_read(dev, 0, 1, buf),
		           result == HYDRASDR_SUCCESS);
	}
	stats_print(&stats);

	/* 4 bytes read */
	BENCH_START(&stats, "hydrasdr_spiflash_read(4 bytes)");
	for (i = 0; i < WARMUP_ITERATIONS_SLOW; i++) {
		hydrasdr_spiflash_read(dev, 0, 4, buf);
	}
	for (i = 0; i < BENCHMARK_ITERATIONS_SLOW; i++) {
		BENCH_ITER(&stats,
		           result = hydrasdr_spiflash_read(dev, 0, 4, buf),
		           result == HYDRASDR_SUCCESS);
	}
	stats_print(&stats);

	/* 16 bytes read */
	BENCH_START(&stats, "hydrasdr_spiflash_read(16 bytes)");
	for (i = 0; i < WARMUP_ITERATIONS_SLOW; i++) {
		hydrasdr_spiflash_read(dev, 0, 16, buf);
	}
	for (i = 0; i < BENCHMARK_ITERATIONS_SLOW; i++) {
		BENCH_ITER(&stats,
		           result = hydrasdr_spiflash_read(dev, 0, 16, buf),
		           result == HYDRASDR_SUCCESS);
	}
	stats_print(&stats);

	/* 64 bytes read */
	BENCH_START(&stats, "hydrasdr_spiflash_read(64 bytes)");
	for (i = 0; i < WARMUP_ITERATIONS_SLOW; i++) {
		hydrasdr_spiflash_read(dev, 0, 64, buf);
	}
	for (i = 0; i < BENCHMARK_ITERATIONS_SLOW; i++) {
		BENCH_ITER(&stats,
		           result = hydrasdr_spiflash_read(dev, 0, 64, buf),
		           result == HYDRASDR_SUCCESS);
	}
	stats_print(&stats);

	/* 256 bytes read */
	BENCH_START(&stats, "hydrasdr_spiflash_read(256 bytes)");
	for (i = 0; i < WARMUP_ITERATIONS_SLOW; i++) {
		hydrasdr_spiflash_read(dev, 0, 256, buf);
	}
	for (i = 0; i < BENCHMARK_ITERATIONS_SLOW; i++) {
		BENCH_ITER(&stats,
		           result = hydrasdr_spiflash_read(dev, 0, 256, buf),
		           result == HYDRASDR_SUCCESS);
	}
	stats_print(&stats);
}


/* ============================================================================
 * DDC Processing Benchmarks
 * ========================================================================== */

/*
 * Algorithm configuration table for DDC benchmarking.
 * Maps high-level algorithm names to filter coefficients and iqconverter algorithm names.
 */
typedef struct {
	const char *name;		/* Algorithm name from hydrasdr_list_conversion_algorithms() */
	const float *kernel_f;		/* Float32 filter coefficients */
	const int16_t *kernel_i;	/* Int16 filter coefficients */
	int len;			/* Filter length (taps) */
	const char *algo_f;		/* Internal iqconverter float32 algorithm name */
	const char *algo_i;		/* Internal iqconverter int16 algorithm name */
} ddc_algo_config_t;

static const ddc_algo_config_t ddc_algo_configs[] = {
	{
		"47_opt",
		HB_KERNEL_FLOAT_47_OPT, HB_KERNEL_INT16_47_OPT, HB_KERNEL_FLOAT_LEN_47_OPT,
		"float32_opt", "int16_opt"
	},
	{
		"33_fast",
		HB_KERNEL_FLOAT_33_FAST, HB_KERNEL_INT16_33_FAST, HB_KERNEL_FLOAT_LEN_33_FAST,
		"float32_opt33", "int16_opt33"
	},
	{
		"65_pow2",
		HB_KERNEL_FLOAT_65_POW2, HB_KERNEL_INT16_65_POW2, HB_KERNEL_FLOAT_LEN_65_POW2,
		"float32_opt65", "int16_opt65"
	},
	{
		"83_highperf",
		HB_KERNEL_FLOAT_83_HIGHPERF, HB_KERNEL_INT16_83_HIGHPERF, HB_KERNEL_FLOAT_LEN_83_HIGHPERF,
		"float32_opt83", "int16_opt83"
	},
	{
		"legacy",
		HB_KERNEL_FLOAT_47_OPT, HB_KERNEL_INT16_47_OPT, HB_KERNEL_FLOAT_LEN_47_OPT,
		"float32_legacy", "int16_legacy"
	},
	{ NULL, NULL, NULL, 0, NULL, NULL }  /* Sentinel */
};

/* Find algorithm config by name */
static const ddc_algo_config_t *find_ddc_algo_config(const char *name)
{
	for (int i = 0; ddc_algo_configs[i].name != NULL; i++) {
		if (strcmp(ddc_algo_configs[i].name, name) == 0)
			return &ddc_algo_configs[i];
	}
	return NULL;
}

static void benchmark_ddc_processing(void)
{
	bench_stats_t stats;
	int i, j;
	const int sample_count = 10000000;  /* 10M samples */
	const int iterations = 10;

	printf("\nDDC Processing Benchmarks (CPU-only, %d samples)\n\n", sample_count);
	printf("Note: LUT auto-initialized when creating fused algorithms (12-bit ADC)\n\n");

	/* List available algorithms from hydrasdr API */
	const char *algo_names[16];
	const char *algo_descs[16];
	int algo_count = hydrasdr_list_conversion_algorithms(algo_names, algo_descs, 16);
	printf("Available DDC algorithms: %d\n", algo_count);
	for (i = 0; i < algo_count; i++) {
		printf("  %-12s : %s\n", algo_names[i], algo_descs[i]);
	}
	printf("\n");

	/* Also list low-level iqconverter algorithms */
	const char *iq_algo_names[24];
	int iq_algo_count = iqconverter_list_algorithms(iq_algo_names, 24);
	printf("Available iqconverter algorithms: %d\n", iq_algo_count);
	for (i = 0; i < iq_algo_count; i++) {
		printf("  [%2d] %s\n", i, iq_algo_names[i]);
	}
	printf("\n");

	/* Allocate buffers */
	uint16_t *input = (uint16_t *)malloc(sample_count * sizeof(uint16_t));
	float *output_f = (float *)malloc(sample_count * sizeof(float));
	int16_t *output_i = (int16_t *)malloc(sample_count * sizeof(int16_t));

	if (!input || !output_f || !output_i) {
		printf("Memory allocation failed\n");
		free(input);
		free(output_f);
		free(output_i);
		return;
	}

	/* Initialize input data (simulated 12-bit ADC samples) */
	for (i = 0; i < sample_count; i++) {
		input[i] = (uint16_t)(i & 0xFFF);  /* 12-bit range: 0-4095 */
	}

	/* Benchmark each algorithm returned by hydrasdr_list_conversion_algorithms() */
	for (j = 0; j < algo_count; j++) {
		const ddc_algo_config_t *cfg = find_ddc_algo_config(algo_names[j]);
		if (!cfg) {
			printf("%-12s: SKIPPED (no config found)\n\n", algo_names[j]);
			continue;
		}

		printf("=== %s (%d-tap) ===\n", algo_names[j], cfg->len);

		/* Benchmark Float32 implementation */
		{
			iqconverter_t *cnv = iqconverter_create(cfg->kernel_f, cfg->len, cfg->algo_f);
			if (!cnv) {
				printf("  Float32: SKIPPED (create failed)\n");
			} else {
				char name[64];
				snprintf(name, sizeof(name), "  Float32 (%s)", cfg->algo_f);

				/* Warmup - unified API handles both fused and legacy */
				for (i = 0; i < 3; i++) {
					iqconverter_process(cnv, input, output_f, sample_count);
				}

				/* Benchmark */
				BENCH_START(&stats, name);
				for (i = 0; i < iterations; i++) {
					BENCH_ITER(&stats,
						   iqconverter_process(cnv, input, output_f, sample_count),
						   true);
				}
				stats_print(&stats);

				double avg_us = stats.sum_us / stats.success_count;
				double msps = (double)sample_count / avg_us;
				printf("    Throughput: %.1f MSPS (%.1f MB/s)\n",
				       msps, msps * sizeof(float) * 2);

				iqconverter_free(cnv);
			}
		}

		/* Benchmark Int16 implementation */
		{
			iqconverter_t *cnv = iqconverter_create(cfg->kernel_i, cfg->len, cfg->algo_i);
			if (!cnv) {
				printf("  Int16: SKIPPED (create failed)\n");
			} else {
				char name[64];
				snprintf(name, sizeof(name), "  Int16 (%s)", cfg->algo_i);

				/* Warmup - unified API handles both fused and legacy */
				for (i = 0; i < 3; i++) {
					iqconverter_process(cnv, input, output_i, sample_count);
				}

				/* Benchmark */
				BENCH_START(&stats, name);
				for (i = 0; i < iterations; i++) {
					BENCH_ITER(&stats,
						   iqconverter_process(cnv, input, output_i, sample_count),
						   true);
				}
				stats_print(&stats);

				double avg_us = stats.sum_us / stats.success_count;
				double msps = (double)sample_count / avg_us;
				printf("    Throughput: %.1f MSPS (%.1f MB/s)\n",
				       msps, msps * sizeof(int16_t) * 2);

				iqconverter_free(cnv);
			}
		}
		printf("\n");
	}

	free(input);
	free(output_f);
	free(output_i);
	/* LUT is internal to library - no cleanup needed here */
}

/* ============================================================================
 * Main
 * ========================================================================== */

static void print_usage(const char *progname)
{
	printf("HydraSDR API Benchmark Tool\n\n");
	printf("Usage: %s [options]\n\n", progname);
	printf("Options:\n");
	printf("  -h         Show this help\n");
	printf("  -l         Library-only benchmarks (no device required)\n");
	printf("  -d         Device benchmarks (requires connected device)\n");
	printf("  -a         All benchmarks (default)\n");
	printf("  -i <num>   Number of iterations (default: %d)\n", BENCHMARK_ITERATIONS);
	printf("\n");
	printf("Use hydrasdr_test_robustness for API robustness testing.\n");
	printf("\n");
}

/* Warmup: call various APIs to prime USB and system caches */
static void device_warmup(struct hydrasdr_device *dev)
{
	hydrasdr_device_info_t info;
	uint32_t rates[8];
	uint32_t rate_count = 0;
	uint8_t gpio_val;
	int i;

	printf("\nDevice Warmup Phase\n");
	printf("Priming USB and system caches...\n");

	/* Query rate count first */
	hydrasdr_get_samplerates(dev, &rate_count, 0);
	if (rate_count > 8) rate_count = 8;

	/* Call various APIs multiple times to warm up */
	for (i = 0; i < 10; i++) {
		hydrasdr_get_device_info(dev, &info);
		if (rate_count > 0) {
			hydrasdr_get_samplerates(dev, rates, rate_count);
		}
		hydrasdr_is_streaming(dev);
		hydrasdr_gpio_read(dev, HYDRASDR_GPIO_PORT0, HYDRASDR_GPIO_PIN0, &gpio_val);
	}

	printf("Warmup complete.\n");
}

int main(int argc, char **argv)
{
	bool lib_only = false;
	bool dev_only = false;
	int opt;

	init_timer();

	/* Parse arguments */
	for (opt = 1; opt < argc; opt++) {
		if (strcmp(argv[opt], "-h") == 0) {
			print_usage(argv[0]);
			return 0;
		} else if (strcmp(argv[opt], "-l") == 0) {
			lib_only = true;
		} else if (strcmp(argv[opt], "-d") == 0) {
			dev_only = true;
		} else if (strcmp(argv[opt], "-a") == 0) {
			lib_only = false;
			dev_only = false;
		}
	}

	/* Print header and check library version compatibility */
	hydrasdr_lib_version_t ver;
	hydrasdr_lib_version(&ver);

	printf("========================================\n");
	printf("HydraSDR API Benchmark Tool\n");
	printf("libhydrasdr v%d.%d.%d\n",
	       ver.major_version, ver.minor_version, ver.revision);
	printf("========================================\n\n");

	{
		uint32_t runtime_ver = HYDRASDR_MAKE_VERSION(ver.major_version,
		                                              ver.minor_version,
		                                              ver.revision);
		uint32_t min_ver = HYDRASDR_MAKE_VERSION(1, 1, 0);
		if (runtime_ver < min_ver) {
			fprintf(stderr, "[WARN] Library version too old: need v1.1.0+\n\n");
		}
	}

	/* Print system information */
	print_system_info();

	printf("Benchmark iterations: %d (warmup: %d)\n", BENCHMARK_ITERATIONS, WARMUP_ITERATIONS);

	/* Library-only benchmarks */
	if (!dev_only) {
		benchmark_library_apis();
		benchmark_ddc_processing();
	}

	/* Device benchmarks */
	if (!lib_only) {
		struct hydrasdr_device *dev = NULL;
		int result;

		printf("\nOpening device\n");
		result = hydrasdr_open(&dev);
		if (result != HYDRASDR_SUCCESS) {
			printf("No device found (error: %s)\n", hydrasdr_error_name(result));
			printf("Skipping device benchmarks.\n");
		} else {
			hydrasdr_device_info_t info;
			hydrasdr_get_device_info(dev, &info);
			printf("Device: %s\n", info.board_name);
			printf("Firmware: %s\n", info.firmware_version);
			printf("Serial: 0x%08X%08X%08X%08X\n",
			       info.part_serial.serial_no[0],
			       info.part_serial.serial_no[1],
			       info.part_serial.serial_no[2],
			       info.part_serial.serial_no[3]);
			printf("Frequency: %llu - %llu MHz\n",
			       (unsigned long long)(info.min_frequency / 1000000),
			       (unsigned long long)(info.max_frequency / 1000000));

			/* Device warmup (prime USB and caches) */
			device_warmup(dev);

			/* Timing benchmarks on critical APIs */
			benchmark_device_info_apis(dev);
			benchmark_device_config_apis(dev);
			benchmark_gain_apis(dev);
			benchmark_peripheral_apis(dev);
			benchmark_spiflash_read(dev);

			hydrasdr_close(dev);
		}
	}

	printf("\nBenchmark complete.\n");

	return 0;
}
