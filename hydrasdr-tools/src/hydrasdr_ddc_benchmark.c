/*
 * HydraSDR DDC Benchmark Tool
 *
 * Benchmarks different DDC (Digital Down Converter) algorithm implementations.
 * Compares performance and verifies correctness of legacy vs optimized versions.
 *
 * Copyright (C) 2025-2026 Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <hydrasdr.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(_WIN32)
#include <windows.h>
#else
#include <sys/time.h>
#include <unistd.h>
#include <sys/utsname.h>
#endif

#if defined(__APPLE__) || defined(__FreeBSD__)
#include <sys/sysctl.h>
#endif
#if defined(__FreeBSD__)
#include <sys/utsname.h>
#endif

/* CPU feature detection */
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
#include <intrin.h>
#define HAVE_CPUID 1
#elif (defined(__GNUC__) || defined(__clang__)) && (defined(__x86_64__) || defined(__i386__))
#include <cpuid.h>
#define HAVE_CPUID 1
#endif

/* Include internal headers for direct DDC access */
#include "iqconverter.h"
#include "iqconverter_decimator.h"
#include "cpu_features.h"

/* Include comprehensive filter definitions for all configurations */
#include "filters_opt.h"

/* Decimation factors to benchmark */
static const int DEC_FACTORS[] = { 2, 4, 8, 16, 32, 64 };
#define NUM_DEC_FACTORS (sizeof(DEC_FACTORS) / sizeof(DEC_FACTORS[0]))

/* Platform-portable strdup */
#ifdef _MSC_VER
#define STRDUP _strdup
#else
#define STRDUP strdup
#endif

/* Aliases for backward compatibility with existing benchmark code (47-tap default) */
#define HB_KERNEL_FLOAT HB_KERNEL_FLOAT_47_OPT
#define HB_KERNEL_INT16 HB_KERNEL_INT16_47_OPT
#define HB_KERNEL_FLOAT_LEN HB_KERNEL_FLOAT_LEN_47_OPT
#define HB_KERNEL_INT16_LEN HB_KERNEL_INT16_LEN_47_OPT

#if defined(_WIN32)
#include <windows.h>
#endif

/* ========================================================================
 * SYSTEM INFORMATION DETECTION
 * ======================================================================== */

/* Get architecture string */
static const char *get_arch_string(void)
{
#if defined(__x86_64__) || defined(_M_X64)
	return "x86_64";
#elif defined(__i386__) || defined(_M_IX86)
	return "x86";
#elif defined(__aarch64__) || defined(_M_ARM64)
	return "ARM64";
#elif defined(__arm__) || defined(_M_ARM)
	return "ARM32";
#elif defined(__riscv)
	return "RISC-V";
#elif defined(__powerpc64__)
	return "PPC64";
#else
	return "Unknown";
#endif
}

/* Get OS name and version */
static void get_os_info(char *buffer, size_t size)
{
#if defined(_WIN32)
	/* Try to get Windows version from registry (more reliable) */
	{
		HKEY hKey;
		if (RegOpenKeyExW(HKEY_LOCAL_MACHINE,
				  L"SOFTWARE\\Microsoft\\Windows NT\\CurrentVersion",
				  0, KEY_READ, &hKey) == ERROR_SUCCESS) {
			DWORD dwType, dwSize;
			WCHAR szProductName[256] = {0};
			WCHAR szDisplayVersion[64] = {0};
			WCHAR szCurrentBuild[32] = {0};
			DWORD dwCurrentBuild = 0;

			dwSize = sizeof(szProductName);
			RegQueryValueExW(hKey, L"ProductName", NULL, &dwType,
					 (LPBYTE)szProductName, &dwSize);

			dwSize = sizeof(szDisplayVersion);
			RegQueryValueExW(hKey, L"DisplayVersion", NULL, &dwType,
					 (LPBYTE)szDisplayVersion, &dwSize);

			dwSize = sizeof(szCurrentBuild);
			if (RegQueryValueExW(hKey, L"CurrentBuildNumber", NULL, &dwType,
					     (LPBYTE)szCurrentBuild, &dwSize) == ERROR_SUCCESS) {
				dwCurrentBuild = _wtoi(szCurrentBuild);
			}

			RegCloseKey(hKey);

			/* Convert to UTF-8 */
			char product_name[256] = {0};
			char display_ver[64] = {0};
			WideCharToMultiByte(CP_UTF8, 0, szProductName, -1,
					    product_name, sizeof(product_name), NULL, NULL);
			WideCharToMultiByte(CP_UTF8, 0, szDisplayVersion, -1,
					    display_ver, sizeof(display_ver), NULL, NULL);

			/* Fix Windows 11 detection: registry ProductName may still say
			 * "Windows 10" due to backward compatibility. Windows 11 has
			 * build >= 22000 */
			if (dwCurrentBuild >= 22000) {
				char *win10_pos = strstr(product_name, "Windows 10");
				if (win10_pos) {
					/* Replace "Windows 10" with "Windows 11" in place */
					win10_pos[9] = '1';
				}
			}

			if (product_name[0] && display_ver[0]) {
				snprintf(buffer, size, "%s %s (Build %lu)",
					 product_name, display_ver, (unsigned long)dwCurrentBuild);
				return;
			}
		}
	}

	/* Fallback */
	snprintf(buffer, size, "Windows");

#elif defined(__APPLE__)
	char osproductversion[64] = {0};
	char osversion[64] = {0};
	size_t len;

	len = sizeof(osproductversion);
	if (sysctlbyname("kern.osproductversion", osproductversion, &len, NULL, 0) == 0) {
		len = sizeof(osversion);
		sysctlbyname("kern.osversion", osversion, &len, NULL, 0);
		snprintf(buffer, size, "macOS %s (%s)", osproductversion, osversion);
	} else {
		snprintf(buffer, size, "macOS");
	}

#elif defined(__linux__)
	FILE *fp = fopen("/etc/os-release", "r");
	if (fp) {
		char line[256];
		char pretty_name[256] = {0};

		while (fgets(line, sizeof(line), fp)) {
			if (strncmp(line, "PRETTY_NAME=", 12) == 0) {
				char *start = line + 12;
				if (*start == '"') start++;
				snprintf(pretty_name, sizeof(pretty_name), "%s", start);
				/* Remove trailing quote and newline */
				char *end = pretty_name + strlen(pretty_name) - 1;
				while (end > pretty_name && (*end == '"' || *end == '\n' || *end == '\r'))
					*end-- = '\0';
				break;
			}
		}
		fclose(fp);

		if (pretty_name[0]) {
			snprintf(buffer, size, "%s", pretty_name);
			return;
		}
	}

	/* Fallback: use uname */
	{
		struct utsname uts;
		if (uname(&uts) == 0) {
			snprintf(buffer, size, "%s %s", uts.sysname, uts.release);
		} else {
			snprintf(buffer, size, "Linux");
		}
	}
#elif defined(__FreeBSD__)
	struct utsname name;
	memset(&name, 0, sizeof name);
	uname(&name);
	snprintf(buffer, size, "%s", name.version);
#else
	snprintf(buffer, size, "Unknown OS");
#endif
}

/* Get CPU model name */
static void get_cpu_model(char *buffer, size_t size)
{
#if defined(_WIN32)
	HKEY hKey;
	if (RegOpenKeyExW(HKEY_LOCAL_MACHINE,
			  L"HARDWARE\\DESCRIPTION\\System\\CentralProcessor\\0",
			  0, KEY_READ, &hKey) == ERROR_SUCCESS) {
		DWORD dwType, dwSize = (DWORD)size;
		WCHAR szValue[256] = {0};
		dwSize = sizeof(szValue);
		if (RegQueryValueExW(hKey, L"ProcessorNameString", NULL, &dwType,
				     (LPBYTE)szValue, &dwSize) == ERROR_SUCCESS) {
			WideCharToMultiByte(CP_UTF8, 0, szValue, -1, buffer, (int)size, NULL, NULL);
			/* Trim leading/trailing spaces */
			char *start = buffer;
			while (*start == ' ') start++;
			if (start != buffer) memmove(buffer, start, strlen(start) + 1);
			char *end = buffer + strlen(buffer) - 1;
			while (end > buffer && *end == ' ') *end-- = '\0';
		}
		RegCloseKey(hKey);
		if (buffer[0]) return;
	}
	snprintf(buffer, size, "Unknown CPU");

#elif defined(__APPLE__)
	size_t len = size;
	if (sysctlbyname("machdep.cpu.brand_string", buffer, &len, NULL, 0) != 0) {
		/* ARM Macs don't have brand_string, try chip info */
		char chip[64] = {0};
		len = sizeof(chip);
		if (sysctlbyname("machdep.cpu.brand", chip, &len, NULL, 0) == 0) {
			snprintf(buffer, size, "%s", chip);
		} else {
			/* Last resort: get hw.model */
			len = size;
			if (sysctlbyname("hw.model", buffer, &len, NULL, 0) != 0) {
				snprintf(buffer, size, "Apple Silicon");
			}
		}
	}

#elif defined(__linux__)
	FILE *fp = fopen("/proc/cpuinfo", "r");
	if (fp) {
		char line[256];
		while (fgets(line, sizeof(line), fp)) {
			/* x86: "model name" */
			if (strncmp(line, "model name", 10) == 0) {
				char *colon = strchr(line, ':');
				if (colon) {
					colon++;
					while (*colon == ' ' || *colon == '\t') colon++;
					strncpy(buffer, colon, size - 1);
					buffer[size - 1] = '\0';
					/* Remove newline */
					char *nl = strchr(buffer, '\n');
					if (nl) *nl = '\0';
					fclose(fp);
					return;
				}
			}
			/* ARM: "Model" or "Hardware" */
			if (strncmp(line, "Model", 5) == 0 || strncmp(line, "Hardware", 8) == 0) {
				char *colon = strchr(line, ':');
				if (colon) {
					colon++;
					while (*colon == ' ' || *colon == '\t') colon++;
					strncpy(buffer, colon, size - 1);
					buffer[size - 1] = '\0';
					char *nl = strchr(buffer, '\n');
					if (nl) *nl = '\0';
					fclose(fp);
					return;
				}
			}
		}
		fclose(fp);
	}
	snprintf(buffer, size, "Unknown CPU");
#elif defined(__FreeBSD__)
	char text[256];
	size_t len = sizeof text;
	if (sysctlbyname("hw.model", &text, &len, NULL, 0) == 0) {
		snprintf(buffer, size, "%s", text);
	}
	else {
		snprintf(buffer, size, "Unknown CPU");
	}
#else
	snprintf(buffer, size, "Unknown CPU");
#endif
}

/* Get number of CPU cores */
static int get_cpu_cores(void)
{
#if defined(_WIN32)
	SYSTEM_INFO sysinfo;
	GetSystemInfo(&sysinfo);
	return (int)sysinfo.dwNumberOfProcessors;
#elif defined(__APPLE__) || defined(__FreeBSD__)
	int cores = 0;
	size_t len = sizeof(cores);
	if (sysctlbyname("hw.ncpu", &cores, &len, NULL, 0) == 0)
		return cores;
	return 1;
#elif defined(__linux__)
	return (int)sysconf(_SC_NPROCESSORS_ONLN);
#else
	return 1;
#endif
}

/* Local CPU feature flags (separate from cpu_features.h) */
typedef struct {
	/* x86 features */
	int sse2;
	int sse3;
	int ssse3;
	int sse41;
	int sse42;
	int avx;
	int avx2;
	int avx512f;
	int fma;
	/* ARM features */
	int neon;
	int sve;
} local_cpu_features_t;

static void detect_cpu_features(local_cpu_features_t *features)
{
	memset(features, 0, sizeof(*features));

#if defined(HAVE_CPUID) && (defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86))

#if defined(_MSC_VER)
	int cpuinfo[4];
	unsigned int max_level;
	unsigned int ebx, ecx, edx;

	__cpuid(cpuinfo, 0);
	max_level = cpuinfo[0];

	if (max_level >= 1) {
		__cpuid(cpuinfo, 1);
		ecx = cpuinfo[2];
		edx = cpuinfo[3];

		features->sse2  = (edx >> 26) & 1;
		features->sse3  = (ecx >> 0) & 1;
		features->ssse3 = (ecx >> 9) & 1;
		features->sse41 = (ecx >> 19) & 1;
		features->sse42 = (ecx >> 20) & 1;
		features->avx   = (ecx >> 28) & 1;
		features->fma   = (ecx >> 12) & 1;
	}

	if (max_level >= 7) {
		__cpuidex(cpuinfo, 7, 0);
		ebx = cpuinfo[1];
		features->avx2    = (ebx >> 5) & 1;
		features->avx512f = (ebx >> 16) & 1;
	}
#else
	/* GCC/Clang */
	unsigned int eax, ebx, ecx, edx;
	unsigned int max_level;

	if (__get_cpuid(0, &eax, &ebx, &ecx, &edx)) {
		max_level = eax;

		if (max_level >= 1) {
			__get_cpuid(1, &eax, &ebx, &ecx, &edx);
			features->sse2  = (edx >> 26) & 1;
			features->sse3  = (ecx >> 0) & 1;
			features->ssse3 = (ecx >> 9) & 1;
			features->sse41 = (ecx >> 19) & 1;
			features->sse42 = (ecx >> 20) & 1;
			features->avx   = (ecx >> 28) & 1;
			features->fma   = (ecx >> 12) & 1;
		}

		if (max_level >= 7) {
			__cpuid_count(7, 0, eax, ebx, ecx, edx);
			features->avx2    = (ebx >> 5) & 1;
			features->avx512f = (ebx >> 16) & 1;
		}
	}
#endif

#elif defined(__aarch64__) || defined(_M_ARM64)
	/* ARM64 always has NEON */
	features->neon = 1;

#if defined(__linux__)
	/* Check for SVE on Linux */
	FILE *fp = fopen("/proc/cpuinfo", "r");
	if (fp) {
		char line[256];
		while (fgets(line, sizeof(line), fp)) {
			if (strstr(line, "Features") && strstr(line, "sve")) {
				features->sve = 1;
				break;
			}
		}
		fclose(fp);
	}
#endif

#elif defined(__arm__) || defined(_M_ARM)
	/* ARM32: Check for NEON */
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
	features->neon = 1;
#elif defined(__linux__)
	FILE *fp = fopen("/proc/cpuinfo", "r");
	if (fp) {
		char line[256];
		while (fgets(line, sizeof(line), fp)) {
			if (strstr(line, "Features") && strstr(line, "neon")) {
				features->neon = 1;
				break;
			}
		}
		fclose(fp);
	}
#endif
#endif
}

/* Get compiler info */
static void get_compiler_info(char *buffer, size_t size)
{
#if defined(_MSC_VER)
	snprintf(buffer, size, "MSVC %d", _MSC_VER);
#elif defined(__clang__)
	snprintf(buffer, size, "Clang %d.%d.%d", __clang_major__, __clang_minor__, __clang_patchlevel__);
#elif defined(__GNUC__)
	snprintf(buffer, size, "GCC %d.%d.%d", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
#else
	snprintf(buffer, size, "Unknown compiler");
#endif
}

/* Get optimization level */
static const char *get_optimization_level(void)
{
#if defined(BUILD_OPT_FLAGS)
	/* Use build-time defined flags from CMake */
	return BUILD_OPT_FLAGS;
#elif defined(_MSC_VER)
	/* MSVC optimization detection */
	#if defined(_DEBUG)
		return "Debug";
	#elif defined(NDEBUG)
		return "Release (optimized)";
	#else
		return "Unknown";
	#endif
#elif defined(__GNUC__) || defined(__clang__)
	/* GCC/Clang optimization detection */
	#if defined(__OPTIMIZE_SIZE__)
		return "-Os (size)";
	#elif defined(__OPTIMIZE__)
		#if defined(__FAST_MATH__)
			return "-O2/-O3 + fast-math";
		#else
			return "-O1/-O2/-O3 (enabled)";
		#endif
	#else
		return "-O0 (no optimization)";
	#endif
#else
	return "Unknown";
#endif
}

/* Print system information */
static void print_system_info(void)
{
	char os_info[256];
	char cpu_model[256];
	char compiler_info[64];
	local_cpu_features_t features;
	int cores;

	get_os_info(os_info, sizeof(os_info));
	get_cpu_model(cpu_model, sizeof(cpu_model));
	get_compiler_info(compiler_info, sizeof(compiler_info));
	detect_cpu_features(&features);
	cores = get_cpu_cores();

	printf("System Information:\n");
	printf("-------------------\n");
	printf("  OS:           %s\n", os_info);
	printf("  Architecture: %s\n", get_arch_string());
	printf("  CPU:          %s\n", cpu_model);
	printf("  CPU Cores:    %d\n", cores);

	/* Print CPU features */
	printf("  CPU Features: ");
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
	if (features.sse2)    printf("SSE2 ");
	if (features.sse3)    printf("SSE3 ");
	if (features.ssse3)   printf("SSSE3 ");
	if (features.sse41)   printf("SSE4.1 ");
	if (features.sse42)   printf("SSE4.2 ");
	if (features.avx)     printf("AVX ");
	if (features.avx2)    printf("AVX2 ");
	if (features.fma)     printf("FMA ");
	if (features.avx512f) printf("AVX-512 ");
#elif defined(__aarch64__) || defined(_M_ARM64) || defined(__arm__) || defined(_M_ARM)
	if (features.neon)    printf("NEON ");
	if (features.sve)     printf("SVE ");
#endif
	printf("\n");

	printf("  Compiler:     %s\n", compiler_info);
	printf("  Optimization: %s\n", get_optimization_level());
	printf("\n");
}

/* ========================================================================
 * BENCHMARK CONFIGURATION
 * ======================================================================== */

/* Benchmark configuration */
#define NUM_SAMPLES      (2 * 1024 * 1024)  /* 2M samples per iteration (~200 MSPS realistic) */
#define NUM_ITERATIONS   10                /* Number of benchmark iterations */
#define WARMUP_ITERATIONS 10                /* Warmup iterations for stable results */
#define NUM_RUNS         3                  /* Number of benchmark runs for min/max/avg */

/* Verification configuration */
#define VERIFY_SAMPLES   65536          /* Samples for verification */
#define FLOAT_TOLERANCE  1e-5f          /* Max allowed difference for float */
#define INT16_TOLERANCE  1              /* Max allowed difference for int16 */

/*
 * Generate uint16_t ADC test data (12-bit range: 0-4095)
 */
static void generate_test_data_u16(uint16_t *buffer, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		/* Mix of random noise and edge cases */
		if (i % 1000 == 0) {
			buffer[i] = 4095;      /* Max ADC value */
		} else if (i % 1001 == 0) {
			buffer[i] = 0;         /* Min ADC value */
		} else if (i % 1002 == 0) {
			buffer[i] = 2048;      /* Midpoint */
		} else {
			/* Pseudo-random noise in 12-bit range */
			buffer[i] = (uint16_t)(rand() % 4096);
		}
	}
}

/* High-resolution timer */
static double get_time_seconds(void)
{
#if defined(_WIN32)
	static LARGE_INTEGER frequency;
	static int initialized = 0;
	LARGE_INTEGER counter;

	if (!initialized) {
		QueryPerformanceFrequency(&frequency);
		initialized = 1;
	}

	QueryPerformanceCounter(&counter);
	return (double)counter.QuadPart / (double)frequency.QuadPart;
#else
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec + tv.tv_usec * 1e-6;
#endif
}

/* Generate test data with random noise and edge cases */
#ifdef _MSC_VER
static void
#else
static void __attribute__((unused))
#endif
generate_test_data_float(float *buffer, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		/* Mix of random noise and edge cases */
		if (i % 1000 == 0) {
			buffer[i] = 1.0f;      /* Max positive */
		} else if (i % 1001 == 0) {
			buffer[i] = -1.0f;     /* Max negative */
		} else if (i % 1002 == 0) {
			buffer[i] = 0.0f;      /* Zero */
		} else {
			/* Pseudo-random noise */
			buffer[i] = ((float)(rand() % 65536) / 32768.0f) - 1.0f;
		}
	}
}

#ifdef _MSC_VER
static void
#else
static void __attribute__((unused))
#endif
generate_test_data_int16(int16_t *buffer, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		/* Mix of random noise and edge cases */
		if (i % 1000 == 0) {
			buffer[i] = 32767;     /* Max positive */
		} else if (i % 1001 == 0) {
			buffer[i] = -32768;    /* Max negative */
		} else if (i % 1002 == 0) {
			buffer[i] = 0;         /* Zero */
		} else {
			/* Pseudo-random noise */
			buffer[i] = (int16_t)((rand() % 65536) - 32768);
		}
	}
}

/* Verification result */
typedef struct {
	int passed;
	int total_samples;
	int mismatches;
	float max_diff;
	int first_mismatch_idx;
} verify_result_t;

/*
 * Verify float32 algorithm outputs match between legacy and optimized
 * Both algorithms should produce identical (or nearly identical) results
 *
 * Uses uint16_t ADC input for both:
 * - Legacy: iqconverter_process() with internal LUT conversion
 * - Optimized: iqconverter_process() with fused processing
 */
static verify_result_t verify_float32_algorithms(void)
{
	verify_result_t result;
	iqconverter_t *cnv_legacy, *cnv_opt;
	uint16_t *input_adc;
	float *output_legacy, *output_opt;
	int i;
	unsigned int seed;

	memset(&result, 0, sizeof(result));
	result.total_samples = VERIFY_SAMPLES;
	result.first_mismatch_idx = -1;

	/* Allocate buffers */
	input_adc = (uint16_t *)malloc(VERIFY_SAMPLES * sizeof(uint16_t));
	output_legacy = (float *)malloc(VERIFY_SAMPLES * sizeof(float));
	output_opt = (float *)malloc(VERIFY_SAMPLES * sizeof(float));
	if (!input_adc || !output_legacy || !output_opt) {
		fprintf(stderr, "Failed to allocate verification buffers\n");
		free(input_adc);
		free(output_legacy);
		free(output_opt);
		return result;
	}

	/* Create both DDC instances */
	cnv_legacy = iqconverter_create(HB_KERNEL_FLOAT, HB_KERNEL_FLOAT_LEN, "float32_legacy");
	cnv_opt = iqconverter_create(HB_KERNEL_FLOAT, HB_KERNEL_FLOAT_LEN, "float32_opt");
	if (!cnv_legacy || !cnv_opt) {
		fprintf(stderr, "Failed to create DDC instances for verification\n");
		iqconverter_free(cnv_legacy);
		iqconverter_free(cnv_opt);
		free(input_adc);
		free(output_legacy);
		free(output_opt);
		return result;
	}

	/* Generate uint16_t ADC input data with fixed seed */
	seed = 12345;
	srand(seed);
	generate_test_data_u16(input_adc, VERIFY_SAMPLES);

	/*
	 * Process through legacy algorithm using unified API
	 * (internally converts via LUT then processes in-place)
	 */
	iqconverter_process(cnv_legacy, input_adc, output_legacy, VERIFY_SAMPLES);

	/*
	 * Process through optimized algorithm using unified API
	 * (uses fused processing path)
	 */
	iqconverter_process(cnv_opt, input_adc, output_opt, VERIFY_SAMPLES);

	/* Compare outputs */
	result.passed = 1;
	for (i = 0; i < VERIFY_SAMPLES; i++) {
		float diff = fabsf(output_legacy[i] - output_opt[i]);
		if (diff > result.max_diff)
			result.max_diff = diff;
		if (diff > FLOAT_TOLERANCE) {
			result.mismatches++;
			if (result.first_mismatch_idx < 0) {
				result.first_mismatch_idx = i;
				printf("    Float32 mismatch at index %d: legacy=%.8f, opt=%.8f, diff=%.8f\n",
				       i, output_legacy[i], output_opt[i], diff);
			}
			result.passed = 0;
		}
	}

	/* Cleanup */
	iqconverter_free(cnv_legacy);
	iqconverter_free(cnv_opt);
	free(input_adc);
	free(output_legacy);
	free(output_opt);

	return result;
}

/*
 * Verify int16 algorithm outputs match between legacy and optimized
 *
 * Uses uint16_t ADC input for both:
 * - Legacy: iqconverter_process() with internal LUT conversion
 * - Optimized: iqconverter_process() with fused processing
 */
static verify_result_t verify_int16_algorithms(void)
{
	verify_result_t result;
	iqconverter_t *cnv_legacy, *cnv_opt;
	uint16_t *input_adc;
	int16_t *output_legacy, *output_opt;
	int i;
	unsigned int seed;

	memset(&result, 0, sizeof(result));
	result.total_samples = VERIFY_SAMPLES;
	result.first_mismatch_idx = -1;

	/* Allocate buffers */
	input_adc = (uint16_t *)malloc(VERIFY_SAMPLES * sizeof(uint16_t));
	output_legacy = (int16_t *)malloc(VERIFY_SAMPLES * sizeof(int16_t));
	output_opt = (int16_t *)malloc(VERIFY_SAMPLES * sizeof(int16_t));
	if (!input_adc || !output_legacy || !output_opt) {
		fprintf(stderr, "Failed to allocate verification buffers\n");
		free(input_adc);
		free(output_legacy);
		free(output_opt);
		return result;
	}

	/* Create both DDC instances */
	cnv_legacy = iqconverter_create(HB_KERNEL_INT16, HB_KERNEL_INT16_LEN, "int16_legacy");
	cnv_opt = iqconverter_create(HB_KERNEL_INT16, HB_KERNEL_INT16_LEN, "int16_opt");
	if (!cnv_legacy || !cnv_opt) {
		fprintf(stderr, "Failed to create DDC instances for verification\n");
		iqconverter_free(cnv_legacy);
		iqconverter_free(cnv_opt);
		free(input_adc);
		free(output_legacy);
		free(output_opt);
		return result;
	}

	/* Generate uint16_t ADC input data with fixed seed */
	seed = 12345;
	srand(seed);
	generate_test_data_u16(input_adc, VERIFY_SAMPLES);

	/*
	 * Process through legacy algorithm using unified API
	 * (internally converts via LUT then processes in-place)
	 */
	iqconverter_process(cnv_legacy, input_adc, output_legacy, VERIFY_SAMPLES);

	/*
	 * Process through optimized algorithm using unified API
	 * (uses fused processing path)
	 */
	iqconverter_process(cnv_opt, input_adc, output_opt, VERIFY_SAMPLES);

	/* Compare outputs */
	result.passed = 1;
	for (i = 0; i < VERIFY_SAMPLES; i++) {
		int diff = abs(output_legacy[i] - output_opt[i]);
		if ((float)diff > result.max_diff)
			result.max_diff = (float)diff;
		if (diff > INT16_TOLERANCE) {
			result.mismatches++;
			if (result.mismatches <= 10) {
				printf("    Int16 mismatch at index %d: legacy=%d, opt=%d, diff=%d\n",
				       i, output_legacy[i], output_opt[i], diff);
			}
			if (result.first_mismatch_idx < 0)
				result.first_mismatch_idx = i;
			result.passed = 0;
		}
	}

	/* Cleanup */
	iqconverter_free(cnv_legacy);
	iqconverter_free(cnv_opt);
	free(input_adc);
	free(output_legacy);
	free(output_opt);

	return result;
}

/* Benchmark a single DDC algorithm */
typedef struct {
	const char *name;
	double total_time;
	double msps;
	double msps_min;
	double msps_max;
	double msps_avg;
	int valid;
} benchmark_result_t;

static benchmark_result_t benchmark_float_algorithm_ex(const char *display_name,
						       const char *algorithm,
						       const float *kernel,
						       int kernel_len)
{
	benchmark_result_t result;
	iqconverter_t *cnv;
	uint16_t *input_adc = NULL;
	float *output_buffer = NULL;
	double start_time, end_time;
	double run_msps[NUM_RUNS];
	int i, run;

	memset(&result, 0, sizeof(result));
	result.name = display_name;

	/* Allocate test buffers - unified API always uses ADC input */
	input_adc = (uint16_t *)malloc(NUM_SAMPLES * sizeof(uint16_t));
	output_buffer = (float *)malloc(NUM_SAMPLES * sizeof(float));
	if (!input_adc || !output_buffer) {
		fprintf(stderr, "Failed to allocate buffer for %s\n", display_name);
		free(input_adc);
		free(output_buffer);
		return result;
	}

	/* Create DDC instance */
	cnv = iqconverter_create(kernel, kernel_len, algorithm);
	if (!cnv) {
		fprintf(stderr, "Failed to create DDC for algorithm: %s\n", algorithm);
		free(input_adc);
		free(output_buffer);
		return result;
	}

	printf("  Testing %s (%d runs x %d iterations)...\n",
	       display_name, NUM_RUNS, NUM_ITERATIONS);

	/* Pre-generate test data once (to exclude data generation from timing) */
	generate_test_data_u16(input_adc, NUM_SAMPLES);

	/* Extended warmup phase */
	for (i = 0; i < WARMUP_ITERATIONS; i++) {
		iqconverter_process(cnv, input_adc, output_buffer, NUM_SAMPLES);
		iqconverter_reset(cnv);
	}

	/* Multiple benchmark runs */
	result.msps_min = 1e12;
	result.msps_max = 0;
	result.msps_avg = 0;

	for (run = 0; run < NUM_RUNS; run++) {
		/* Reset before each run */
		iqconverter_reset(cnv);

		/* Benchmark phase - pure processing time */
		start_time = get_time_seconds();
		for (i = 0; i < NUM_ITERATIONS; i++) {
			iqconverter_process(cnv, input_adc, output_buffer, NUM_SAMPLES);
		}
		end_time = get_time_seconds();

		/* Calculate this run's MSPS */
		run_msps[run] = ((double)NUM_SAMPLES * NUM_ITERATIONS) /
				((end_time - start_time) * 1e6);

		if (run_msps[run] < result.msps_min)
			result.msps_min = run_msps[run];
		if (run_msps[run] > result.msps_max)
			result.msps_max = run_msps[run];
		result.msps_avg += run_msps[run];
	}

	result.msps_avg /= NUM_RUNS;
	result.msps = result.msps_avg; /* Use average as primary result */
	result.total_time = ((double)NUM_SAMPLES * NUM_ITERATIONS * NUM_RUNS) /
			    (result.msps_avg * 1e6);
	result.valid = 1;

	/* Cleanup */
	iqconverter_free(cnv);
	free(input_adc);
	free(output_buffer);

	return result;
}

/* Wrapper for backward compatibility (display_name == algorithm) */
static benchmark_result_t benchmark_float_algorithm(const char *algorithm,
						    const float *kernel,
						    int kernel_len)
{
	return benchmark_float_algorithm_ex(algorithm, algorithm, kernel, kernel_len);
}

static benchmark_result_t benchmark_int16_algorithm_ex(const char *display_name,
						       const char *algorithm,
						       const int16_t *kernel,
						       int kernel_len)
{
	benchmark_result_t result;
	iqconverter_t *cnv;
	uint16_t *input_adc = NULL;
	int16_t *output_buffer = NULL;
	double start_time, end_time;
	double run_msps[NUM_RUNS];
	int i, run;

	memset(&result, 0, sizeof(result));
	result.name = display_name;

	/* Allocate test buffers - unified API always uses ADC input */
	input_adc = (uint16_t *)malloc(NUM_SAMPLES * sizeof(uint16_t));
	output_buffer = (int16_t *)malloc(NUM_SAMPLES * sizeof(int16_t));
	if (!input_adc || !output_buffer) {
		fprintf(stderr, "Failed to allocate buffer for %s\n", display_name);
		free(input_adc);
		free(output_buffer);
		return result;
	}

	/* Create DDC instance */
	cnv = iqconverter_create(kernel, kernel_len, algorithm);
	if (!cnv) {
		fprintf(stderr, "Failed to create DDC for algorithm: %s\n", algorithm);
		free(input_adc);
		free(output_buffer);
		return result;
	}

	printf("  Testing %s (%d runs x %d iterations)...\n",
	       display_name, NUM_RUNS, NUM_ITERATIONS);

	/* Pre-generate test data once (to exclude data generation from timing) */
	generate_test_data_u16(input_adc, NUM_SAMPLES);

	/* Extended warmup phase */
	for (i = 0; i < WARMUP_ITERATIONS; i++) {
		iqconverter_process(cnv, input_adc, output_buffer, NUM_SAMPLES);
		iqconverter_reset(cnv);
	}

	/* Multiple benchmark runs */
	result.msps_min = 1e12;
	result.msps_max = 0;
	result.msps_avg = 0;

	for (run = 0; run < NUM_RUNS; run++) {
		/* Reset before each run */
		iqconverter_reset(cnv);

		/* Benchmark phase - pure processing time */
		start_time = get_time_seconds();
		for (i = 0; i < NUM_ITERATIONS; i++) {
			iqconverter_process(cnv, input_adc, output_buffer, NUM_SAMPLES);
		}
		end_time = get_time_seconds();

		/* Calculate this run's MSPS */
		run_msps[run] = ((double)NUM_SAMPLES * NUM_ITERATIONS) /
				((end_time - start_time) * 1e6);

		if (run_msps[run] < result.msps_min)
			result.msps_min = run_msps[run];
		if (run_msps[run] > result.msps_max)
			result.msps_max = run_msps[run];
		result.msps_avg += run_msps[run];
	}

	result.msps_avg /= NUM_RUNS;
	result.msps = result.msps_avg; /* Use average as primary result */
	result.total_time = ((double)NUM_SAMPLES * NUM_ITERATIONS * NUM_RUNS) /
			    (result.msps_avg * 1e6);
	result.valid = 1;

	/* Cleanup */
	iqconverter_free(cnv);
	free(input_adc);
	free(output_buffer);

	return result;
}

/* Wrapper for backward compatibility (display_name == algorithm) */
static benchmark_result_t benchmark_int16_algorithm(const char *algorithm,
						    const int16_t *kernel,
						    int kernel_len)
{
	return benchmark_int16_algorithm_ex(algorithm, algorithm, kernel, kernel_len);
}

/* ========================================================================
 * DECIMATION BENCHMARKS
 *
 * Benchmarks DDC with cascaded decimation (2x to 64x).
 * Uses mixed 33-tap (stages 0-2) + 17-tap (stages 3+) halfband filters.
 * Achieves < 0.1 dB passband ripple at 64x decimation.
 * ======================================================================== */

static benchmark_result_t benchmark_float_decimation(int decimation)
{
	benchmark_result_t result;
	iqconverter_t *cnv;
	uint16_t *input_adc = NULL;
	float *output_buffer = NULL;
	double start_time, end_time;
	double run_msps[NUM_RUNS];
	int i, run;
	int output_len;
	char name[32];

	snprintf(name, sizeof(name), "float32_dec_%dx", decimation);

	memset(&result, 0, sizeof(result));
	result.name = STRDUP(name);

	/* Calculate output length after decimation */
	output_len = NUM_SAMPLES / decimation;

	/* Allocate test buffers */
	input_adc = (uint16_t *)malloc(NUM_SAMPLES * sizeof(uint16_t));
	output_buffer = (float *)malloc(output_len * sizeof(float));
	if (!input_adc || !output_buffer) {
		fprintf(stderr, "Failed to allocate buffer for %s\n", name);
		free(input_adc);
		free(output_buffer);
		return result;
	}

	/* Create DDC instance with decimation */
	cnv = iqconverter_create_ex(HB_KERNEL_FLOAT, HB_KERNEL_FLOAT_LEN,
				    "float32_opt_dec", decimation);
	if (!cnv) {
		fprintf(stderr, "Failed to create DDC for decimation %dx\n", decimation);
		free(input_adc);
		free(output_buffer);
		return result;
	}

	printf("  Testing %s (%d runs x %d iterations)...\n",
	       name, NUM_RUNS, NUM_ITERATIONS);

	/* Pre-generate test data */
	generate_test_data_u16(input_adc, NUM_SAMPLES);

	/* Warmup phase */
	for (i = 0; i < WARMUP_ITERATIONS; i++) {
		iqconverter_process(cnv, input_adc, output_buffer, NUM_SAMPLES);
		iqconverter_reset(cnv);
	}

	/* Multiple benchmark runs */
	result.msps_min = 1e12;
	result.msps_max = 0;
	result.msps_avg = 0;

	for (run = 0; run < NUM_RUNS; run++) {
		iqconverter_reset(cnv);

		start_time = get_time_seconds();
		for (i = 0; i < NUM_ITERATIONS; i++) {
			iqconverter_process(cnv, input_adc, output_buffer, NUM_SAMPLES);
		}
		end_time = get_time_seconds();

		/* MSPS based on INPUT sample rate */
		run_msps[run] = ((double)NUM_SAMPLES * NUM_ITERATIONS) /
				((end_time - start_time) * 1e6);

		if (run_msps[run] < result.msps_min)
			result.msps_min = run_msps[run];
		if (run_msps[run] > result.msps_max)
			result.msps_max = run_msps[run];
		result.msps_avg += run_msps[run];
	}

	result.msps_avg /= NUM_RUNS;
	result.msps = result.msps_avg;
	result.total_time = ((double)NUM_SAMPLES * NUM_ITERATIONS * NUM_RUNS) /
			    (result.msps_avg * 1e6);
	result.valid = 1;

	iqconverter_free(cnv);
	free(input_adc);
	free(output_buffer);

	return result;
}

static benchmark_result_t benchmark_int16_decimation(int decimation)
{
	benchmark_result_t result;
	iqconverter_t *cnv;
	uint16_t *input_adc = NULL;
	int16_t *output_buffer = NULL;
	double start_time, end_time;
	double run_msps[NUM_RUNS];
	int i, run;
	int output_len;
	char name[32];

	snprintf(name, sizeof(name), "int16_dec_%dx", decimation);

	memset(&result, 0, sizeof(result));
	result.name = STRDUP(name);

	/* Calculate output length after decimation */
	output_len = NUM_SAMPLES / decimation;

	/* Allocate test buffers */
	input_adc = (uint16_t *)malloc(NUM_SAMPLES * sizeof(uint16_t));
	output_buffer = (int16_t *)malloc(output_len * sizeof(int16_t));
	if (!input_adc || !output_buffer) {
		fprintf(stderr, "Failed to allocate buffer for %s\n", name);
		free(input_adc);
		free(output_buffer);
		return result;
	}

	/* Create DDC instance with decimation */
	cnv = iqconverter_create_ex(HB_KERNEL_INT16, HB_KERNEL_INT16_LEN,
				    "int16_opt_dec", decimation);
	if (!cnv) {
		fprintf(stderr, "Failed to create DDC for decimation %dx\n", decimation);
		free(input_adc);
		free(output_buffer);
		return result;
	}

	printf("  Testing %s (%d runs x %d iterations)...\n",
	       name, NUM_RUNS, NUM_ITERATIONS);

	/* Pre-generate test data */
	generate_test_data_u16(input_adc, NUM_SAMPLES);

	/* Warmup phase */
	for (i = 0; i < WARMUP_ITERATIONS; i++) {
		iqconverter_process(cnv, input_adc, output_buffer, NUM_SAMPLES);
		iqconverter_reset(cnv);
	}

	/* Multiple benchmark runs */
	result.msps_min = 1e12;
	result.msps_max = 0;
	result.msps_avg = 0;

	for (run = 0; run < NUM_RUNS; run++) {
		iqconverter_reset(cnv);

		start_time = get_time_seconds();
		for (i = 0; i < NUM_ITERATIONS; i++) {
			iqconverter_process(cnv, input_adc, output_buffer, NUM_SAMPLES);
		}
		end_time = get_time_seconds();

		/* MSPS based on INPUT sample rate */
		run_msps[run] = ((double)NUM_SAMPLES * NUM_ITERATIONS) /
				((end_time - start_time) * 1e6);

		if (run_msps[run] < result.msps_min)
			result.msps_min = run_msps[run];
		if (run_msps[run] > result.msps_max)
			result.msps_max = run_msps[run];
		result.msps_avg += run_msps[run];
	}

	result.msps_avg /= NUM_RUNS;
	result.msps = result.msps_avg;
	result.total_time = ((double)NUM_SAMPLES * NUM_ITERATIONS * NUM_RUNS) /
			    (result.msps_avg * 1e6);
	result.valid = 1;

	iqconverter_free(cnv);
	free(input_adc);
	free(output_buffer);

	return result;
}

static void print_usage(void)
{
	printf("HydraSDR DDC Benchmark Tool\n");
	printf("\n");
	printf("Usage: hydrasdr_ddc_benchmark [options]\n");
	printf("\n");
	printf("Options:\n");
	printf("  -h         Show this help\n");
	printf("  -l         List available DDC algorithms\n");
	printf("  -a <name>  Benchmark specific algorithm only\n");
	printf("  -n <num>   Number of iterations (default: %d)\n", NUM_ITERATIONS);
	printf("  -v         Run correctness verification only (no benchmark)\n");
	printf("\n");
	printf("This tool benchmarks the DDC (Digital Down Converter) implementations.\n");
	printf("DDC converts real ADC samples to complex I/Q samples with 2:1 decimation.\n");
}

static void list_algorithms(void)
{
	const char *names[16];
	int count, i;

	count = iqconverter_list_algorithms(names, 16);

	printf("Available DDC Algorithms:\n");
	printf("-------------------------\n");
	for (i = 0; i < count; i++) {
		iqconverter_t *cnv = NULL;

		/* Try 47-tap float kernel */
		cnv = iqconverter_create(HB_KERNEL_FLOAT, HB_KERNEL_FLOAT_LEN, names[i]);
		if (!cnv) {
			/* Try 47-tap int16 kernel */
			cnv = iqconverter_create(HB_KERNEL_INT16, HB_KERNEL_INT16_LEN, names[i]);
		}
		if (!cnv) {
			/* Try 65-tap float kernel */
			cnv = iqconverter_create(HB_KERNEL_FLOAT_65_POW2,
						 HB_KERNEL_FLOAT_LEN_65_POW2, names[i]);
		}
		if (!cnv) {
			/* Try 65-tap int16 kernel */
			cnv = iqconverter_create(HB_KERNEL_INT16_65_POW2,
						 HB_KERNEL_INT16_LEN_65_POW2, names[i]);
		}

		if (cnv) {
			printf("  %-20s - %s\n", names[i], iqconverter_get_info(cnv));
			iqconverter_free(cnv);
		} else {
			printf("  %-20s\n", names[i]);
		}
	}
	printf("\n");
}

int main(int argc, char **argv)
{
	benchmark_result_t results[24];	/* Extended for all filter variants: 33, 47, 65, 83 tap */
	int num_results = 0;
	int i;
	const char *specific_algo = NULL;
	int verify_only = 0;
	hydrasdr_lib_version_t lib_version;

#if defined(_WIN32)
	SetConsoleOutputCP(CP_UTF8);
#endif

	/* Parse command line */
	for (i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			print_usage();
			return 0;
		} else if (strcmp(argv[i], "-l") == 0) {
			list_algorithms();
			return 0;
		} else if (strcmp(argv[i], "-a") == 0 && i + 1 < argc) {
			specific_algo = argv[++i];
		} else if (strcmp(argv[i], "-v") == 0) {
			verify_only = 1;
		}
	}

	/* Print header */
	printf("========================================\n");
	printf("HydraSDR DDC Benchmark Tool\n");
	printf("========================================\n\n");

	hydrasdr_lib_version(&lib_version);
	printf("Library Version: %d.%d.%d\n",
	       lib_version.major_version,
	       lib_version.minor_version,
	       lib_version.revision);
	{
		uint32_t runtime_ver = HYDRASDR_MAKE_VERSION(lib_version.major_version,
		                                              lib_version.minor_version,
		                                              lib_version.revision);
		uint32_t min_ver = HYDRASDR_MAKE_VERSION(1, 1, 0);
		if (runtime_ver < min_ver) {
			fprintf(stderr, "[WARN] Library version too old: need v1.1.0+\n");
		}
	}
	printf("\n");

	/* Print system information */
	print_system_info();

	printf("Configuration:\n");
	printf("  Samples per iteration: %d (%.2f MB float, %.2f MB int16)\n",
	       NUM_SAMPLES,
	       (double)NUM_SAMPLES * sizeof(float) / (1024 * 1024),
	       (double)NUM_SAMPLES * sizeof(int16_t) / (1024 * 1024));
	printf("  Iterations per run:    %d\n", NUM_ITERATIONS);
	printf("  Number of runs:        %d\n", NUM_RUNS);
	printf("  Warmup iterations:     %d\n", WARMUP_ITERATIONS);
	printf("  Total samples tested:  %.0f M\n\n",
	       (double)NUM_SAMPLES * NUM_ITERATIONS * NUM_RUNS / 1e6);

	/* List available algorithms */
	list_algorithms();

	/* Run correctness verification */
	printf("========================================\n");
	printf("Correctness Verification\n");
	printf("========================================\n\n");

	printf("Verifying Float32 algorithms (legacy vs optimized)...\n");
	{
		verify_result_t vr = verify_float32_algorithms();
		if (vr.passed) {
			printf("  PASSED: %d samples compared, max_diff=%.8f\n\n",
			       vr.total_samples, vr.max_diff);
		} else {
			printf("  FAILED: %d mismatches out of %d samples, max_diff=%.8f\n\n",
			       vr.mismatches, vr.total_samples, vr.max_diff);
		}
	}

	printf("Verifying Int16 algorithms (legacy vs optimized)...\n");
	{
		verify_result_t vr = verify_int16_algorithms();
		if (vr.passed) {
			printf("  PASSED: %d samples compared, max_diff=%.0f\n\n",
			       vr.total_samples, vr.max_diff);
		} else {
			printf("  FAILED: %d mismatches out of %d samples, max_diff=%.0f\n\n",
			       vr.mismatches, vr.total_samples, vr.max_diff);
		}
	}

	if (verify_only) {
		printf("Verification complete (benchmark skipped with -v flag).\n");
		return 0;
	}

	/* Run benchmarks */
	printf("Running Benchmarks...\n");
	printf("---------------------\n\n");

	/*
	 * Filter configurations summary:
	 * | Config      | Taps | Delay | Pow2 | Rejection | MACs |
	 * |-------------|------|-------|------|-----------|------|
	 * | 33_FAST     |   33 |     8 | YES  |   ~62 dB  |    8 |
	 * | 47_OPT      |   47 |    12 | NO   |   ~63 dB  |   12 |
	 * | 65_POW2     |   65 |    16 | YES  |   ~67 dB  |   16 |
	 * | 83_HIGHPERF |   83 |    21 | NO   |  ~103 dB  |   21 |
	 */

	if (specific_algo == NULL || strstr(specific_algo, "float32")) {
		/* 33-tap FAST (power-of-2 delay=8) */
		printf("Float32 Algorithms (33-tap FAST, pow2 delay=8, ~62dB):\n");
		if (specific_algo == NULL) {
			results[num_results++] = benchmark_float_algorithm_ex(
				"float32_leg_33", "float32_legacy",
				HB_KERNEL_FLOAT_33_FAST, HB_KERNEL_FLOAT_LEN_33_FAST);
			results[num_results++] = benchmark_float_algorithm(
				"float32_opt33", HB_KERNEL_FLOAT_33_FAST,
				HB_KERNEL_FLOAT_LEN_33_FAST);
		}
		printf("\n");

		/* 47-tap OPT (delay=12) */
		printf("Float32 Algorithms (47-tap OPT, delay=12, ~63dB):\n");
		if (specific_algo == NULL || strcmp(specific_algo, "float32_legacy") == 0) {
			results[num_results++] = benchmark_float_algorithm(
				"float32_legacy", HB_KERNEL_FLOAT, HB_KERNEL_FLOAT_LEN);
		}
		if (specific_algo == NULL || strcmp(specific_algo, "float32_opt") == 0) {
			results[num_results++] = benchmark_float_algorithm(
				"float32_opt", HB_KERNEL_FLOAT, HB_KERNEL_FLOAT_LEN);
		}
		printf("\n");

		/* 65-tap POW2 (power-of-2 delay=16) */
		printf("Float32 Algorithms (65-tap POW2, delay=16, ~67dB):\n");
		if (specific_algo == NULL) {
			results[num_results++] = benchmark_float_algorithm_ex(
				"float32_leg_65", "float32_legacy",
				HB_KERNEL_FLOAT_65_POW2, HB_KERNEL_FLOAT_LEN_65_POW2);
		}
		if (specific_algo == NULL || strcmp(specific_algo, "float32_opt65") == 0) {
			results[num_results++] = benchmark_float_algorithm(
				"float32_opt65", HB_KERNEL_FLOAT_65_POW2,
				HB_KERNEL_FLOAT_LEN_65_POW2);
		}
		printf("\n");

		/* 83-tap HIGHPERF (delay=21, >100dB) */
		printf("Float32 Algorithms (83-tap HIGHPERF, delay=21, ~103dB):\n");
		if (specific_algo == NULL) {
			results[num_results++] = benchmark_float_algorithm_ex(
				"float32_leg_83", "float32_legacy",
				HB_KERNEL_FLOAT_83_HIGHPERF, HB_KERNEL_FLOAT_LEN_83_HIGHPERF);
			results[num_results++] = benchmark_float_algorithm(
				"float32_opt83", HB_KERNEL_FLOAT_83_HIGHPERF,
				HB_KERNEL_FLOAT_LEN_83_HIGHPERF);
		}
		printf("\n");
	}

	if (specific_algo == NULL || strstr(specific_algo, "int16")) {
		/* 33-tap FAST (power-of-2 delay=8) */
		printf("Int16 Algorithms (33-tap FAST, pow2 delay=8, ~62dB):\n");
		if (specific_algo == NULL) {
			results[num_results++] = benchmark_int16_algorithm_ex(
				"int16_leg_33", "int16_legacy",
				HB_KERNEL_INT16_33_FAST, HB_KERNEL_INT16_LEN_33_FAST);
			results[num_results++] = benchmark_int16_algorithm(
				"int16_opt33", HB_KERNEL_INT16_33_FAST,
				HB_KERNEL_INT16_LEN_33_FAST);
		}
		printf("\n");

		/* 47-tap OPT (delay=12) */
		printf("Int16 Algorithms (47-tap OPT, delay=12, ~63dB):\n");
		if (specific_algo == NULL || strcmp(specific_algo, "int16_legacy") == 0) {
			results[num_results++] = benchmark_int16_algorithm(
				"int16_legacy", HB_KERNEL_INT16, HB_KERNEL_INT16_LEN);
		}
		if (specific_algo == NULL || strcmp(specific_algo, "int16_opt") == 0) {
			results[num_results++] = benchmark_int16_algorithm(
				"int16_opt", HB_KERNEL_INT16, HB_KERNEL_INT16_LEN);
		}
		printf("\n");

		/* 65-tap POW2 (power-of-2 delay=16) */
		printf("Int16 Algorithms (65-tap POW2, delay=16, ~67dB):\n");
		if (specific_algo == NULL) {
			results[num_results++] = benchmark_int16_algorithm_ex(
				"int16_leg_65", "int16_legacy",
				HB_KERNEL_INT16_65_POW2, HB_KERNEL_INT16_LEN_65_POW2);
		}
		if (specific_algo == NULL || strcmp(specific_algo, "int16_opt65") == 0) {
			results[num_results++] = benchmark_int16_algorithm(
				"int16_opt65", HB_KERNEL_INT16_65_POW2,
				HB_KERNEL_INT16_LEN_65_POW2);
		}
		printf("\n");

		/* 83-tap HIGHPERF (delay=21, >100dB) */
		printf("Int16 Algorithms (83-tap HIGHPERF, delay=21, ~103dB):\n");
		if (specific_algo == NULL) {
			results[num_results++] = benchmark_int16_algorithm_ex(
				"int16_leg_83", "int16_legacy",
				HB_KERNEL_INT16_83_HIGHPERF, HB_KERNEL_INT16_LEN_83_HIGHPERF);
			results[num_results++] = benchmark_int16_algorithm(
				"int16_opt83", HB_KERNEL_INT16_83_HIGHPERF,
				HB_KERNEL_INT16_LEN_83_HIGHPERF);
		}
		printf("\n");
	}

	/* Run decimation benchmarks (x2 to x64) */
	if (specific_algo == NULL) {
		benchmark_result_t dec_results_float[NUM_DEC_FACTORS];
		benchmark_result_t dec_results_int16[NUM_DEC_FACTORS];
		size_t d;

		printf("========================================\n");
		printf("Decimation Benchmarks (x2 to x64)\n");
		printf("========================================\n\n");
		printf("Using mixed 33-tap (stages 0-2) + 17-tap (stages 3+) halfband filters.\n");
		printf("Achieves < 0.1 dB passband ripple at 64x decimation.\n\n");

		printf("Float32 Decimation:\n");
		for (d = 0; d < NUM_DEC_FACTORS; d++) {
			dec_results_float[d] = benchmark_float_decimation(DEC_FACTORS[d]);
		}
		printf("\n");

		printf("Int16 Decimation:\n");
		for (d = 0; d < NUM_DEC_FACTORS; d++) {
			dec_results_int16[d] = benchmark_int16_decimation(DEC_FACTORS[d]);
		}
		printf("\n");

		/* Print decimation results table */
		printf("Decimation Results Summary:\n");
		printf("---------------------------\n\n");

		printf("%-20s | %10s | %10s | %10s | %10s\n",
		       "Decimation", "Min MSPS", "Max MSPS", "Avg MSPS", "Time (s)");
		printf("%-20s-+-%10s-+-%10s-+-%10s-+-%10s\n",
		       "--------------------", "----------", "----------", "----------", "----------");

		for (d = 0; d < NUM_DEC_FACTORS; d++) {
			if (dec_results_float[d].valid) {
				char name[32];
				snprintf(name, sizeof(name), "float32_dec_%dx", DEC_FACTORS[d]);
				printf("%-20s | %10.1f | %10.1f | %10.1f | %10.3f\n",
				       name,
				       dec_results_float[d].msps_min,
				       dec_results_float[d].msps_max,
				       dec_results_float[d].msps_avg,
				       dec_results_float[d].total_time);
			}
		}
		for (d = 0; d < NUM_DEC_FACTORS; d++) {
			if (dec_results_int16[d].valid) {
				char name[32];
				snprintf(name, sizeof(name), "int16_dec_%dx", DEC_FACTORS[d]);
				printf("%-20s | %10.1f | %10.1f | %10.1f | %10.3f\n",
				       name,
				       dec_results_int16[d].msps_min,
				       dec_results_int16[d].msps_max,
				       dec_results_int16[d].msps_avg,
				       dec_results_int16[d].total_time);
			}
		}
		printf("\n");

		/* Free strdup'd names */
		for (d = 0; d < NUM_DEC_FACTORS; d++) {
			if (dec_results_float[d].name)
				free((void *)dec_results_float[d].name);
			if (dec_results_int16[d].name)
				free((void *)dec_results_int16[d].name);
		}
	}

	/* Print results table */
	printf("========================================\n");
	printf("Results Summary\n");
	printf("========================================\n\n");

	printf("%-20s | %10s | %10s | %10s | %10s\n",
	       "Algorithm", "Min MSPS", "Max MSPS", "Avg MSPS", "Time (s)");
	printf("%-20s-+-%10s-+-%10s-+-%10s-+-%10s\n",
	       "--------------------", "----------", "----------", "----------", "----------");

	for (i = 0; i < num_results; i++) {
		if (results[i].valid) {
			printf("%-20s | %10.1f | %10.1f | %10.1f | %10.3f\n",
			       results[i].name,
			       results[i].msps_min,
			       results[i].msps_max,
			       results[i].msps_avg,
			       results[i].total_time);
		} else {
			printf("%-20s | %10s | %10s | %10s | %10s\n",
			       results[i].name, "FAILED", "FAILED", "N/A", "N/A");
		}
	}
	printf("\n");

	/* Calculate speedups - helper to find result by name */
	#define FIND_RESULT(name_str, out_idx) \
		do { \
			out_idx = -1; \
			for (int k = 0; k < num_results; k++) { \
				if (results[k].valid && strcmp(results[k].name, name_str) == 0) { \
					out_idx = k; break; \
				} \
			} \
		} while(0)

	if (num_results >= 2) {
		int idx_leg, idx_opt;
		double speedup;

		printf("Performance Comparison (opt vs legacy):\n");
		printf("----------------------------------------\n");

		/* 33-tap Float32: float32_opt33 vs float32_leg_33 */
		FIND_RESULT("float32_leg_33", idx_leg);
		FIND_RESULT("float32_opt33", idx_opt);
		if (idx_leg >= 0 && idx_opt >= 0) {
			speedup = results[idx_opt].msps_avg / results[idx_leg].msps_avg;
			printf("  33-tap float32_opt33 vs float32_leg_33:  %.2fx\n", speedup);
		}

		/* 47-tap Float32: float32_opt vs float32_legacy */
		FIND_RESULT("float32_legacy", idx_leg);
		FIND_RESULT("float32_opt", idx_opt);
		if (idx_leg >= 0 && idx_opt >= 0) {
			speedup = results[idx_opt].msps_avg / results[idx_leg].msps_avg;
			printf("  47-tap float32_opt   vs float32_legacy:  %.2fx\n", speedup);
		}

		/* 65-tap Float32: float32_opt65 vs float32_leg_65 */
		FIND_RESULT("float32_leg_65", idx_leg);
		FIND_RESULT("float32_opt65", idx_opt);
		if (idx_leg >= 0 && idx_opt >= 0) {
			speedup = results[idx_opt].msps_avg / results[idx_leg].msps_avg;
			printf("  65-tap float32_opt65 vs float32_leg_65:  %.2fx\n", speedup);
		}

		/* 83-tap Float32: float32_opt83 vs float32_leg_83 */
		FIND_RESULT("float32_leg_83", idx_leg);
		FIND_RESULT("float32_opt83", idx_opt);
		if (idx_leg >= 0 && idx_opt >= 0) {
			speedup = results[idx_opt].msps_avg / results[idx_leg].msps_avg;
			printf("  83-tap float32_opt83 vs float32_leg_83:  %.2fx\n", speedup);
		}

		/* Cross-comparison: 33-tap opt vs 47-tap legacy (shows optimization impact) */
		FIND_RESULT("float32_legacy", idx_leg);
		FIND_RESULT("float32_opt33", idx_opt);
		if (idx_leg >= 0 && idx_opt >= 0) {
			speedup = results[idx_opt].msps_avg / results[idx_leg].msps_avg;
			printf("  float32_opt33 vs float32_legacy:         %.2fx\n", speedup);
		}

		printf("\n");

		/* 33-tap Int16: int16_opt33 vs int16_leg_33 */
		FIND_RESULT("int16_leg_33", idx_leg);
		FIND_RESULT("int16_opt33", idx_opt);
		if (idx_leg >= 0 && idx_opt >= 0) {
			speedup = results[idx_opt].msps_avg / results[idx_leg].msps_avg;
			printf("  33-tap int16_opt33   vs int16_leg_33:    %.2fx\n", speedup);
		}

		/* 47-tap Int16: int16_opt vs int16_legacy */
		FIND_RESULT("int16_legacy", idx_leg);
		FIND_RESULT("int16_opt", idx_opt);
		if (idx_leg >= 0 && idx_opt >= 0) {
			speedup = results[idx_opt].msps_avg / results[idx_leg].msps_avg;
			printf("  47-tap int16_opt     vs int16_legacy:    %.2fx\n", speedup);
		}

		/* 65-tap Int16: int16_opt65 vs int16_leg_65 */
		FIND_RESULT("int16_leg_65", idx_leg);
		FIND_RESULT("int16_opt65", idx_opt);
		if (idx_leg >= 0 && idx_opt >= 0) {
			speedup = results[idx_opt].msps_avg / results[idx_leg].msps_avg;
			printf("  65-tap int16_opt65   vs int16_leg_65:    %.2fx\n", speedup);
		}

		/* 83-tap Int16: int16_opt83 vs int16_leg_83 */
		FIND_RESULT("int16_leg_83", idx_leg);
		FIND_RESULT("int16_opt83", idx_opt);
		if (idx_leg >= 0 && idx_opt >= 0) {
			speedup = results[idx_opt].msps_avg / results[idx_leg].msps_avg;
			printf("  83-tap int16_opt83   vs int16_leg_83:    %.2fx\n", speedup);
		}

		/* Cross-comparison: 33-tap opt vs 47-tap legacy (shows optimization impact) */
		FIND_RESULT("int16_legacy", idx_leg);
		FIND_RESULT("int16_opt33", idx_opt);
		if (idx_leg >= 0 && idx_opt >= 0) {
			speedup = results[idx_opt].msps_avg / results[idx_leg].msps_avg;
			printf("  int16_opt33 vs int16_legacy:             %.2fx\n", speedup);
		}

		printf("\nFilter Complexity Comparison:\n");
		printf("-----------------------------\n");

		/* Compare different filter lengths using legacy algorithms */
		int idx_33, idx_47, idx_65, idx_83;

		/* Float32 legacy across filter lengths */
		FIND_RESULT("float32_leg_33", idx_33);
		FIND_RESULT("float32_legacy", idx_47);
		FIND_RESULT("float32_leg_65", idx_65);
		FIND_RESULT("float32_leg_83", idx_83);

		if (idx_33 >= 0 && idx_47 >= 0 && idx_65 >= 0 && idx_83 >= 0) {
			printf("  Float32 legacy:  33-tap=%6.1f  47-tap=%6.1f  65-tap=%6.1f  83-tap=%6.1f MSPS\n",
			       results[idx_33].msps_avg, results[idx_47].msps_avg,
			       results[idx_65].msps_avg, results[idx_83].msps_avg);
		}

		/* Float32 optimized across filter lengths */
		FIND_RESULT("float32_opt33", idx_33);
		FIND_RESULT("float32_opt", idx_47);
		FIND_RESULT("float32_opt65", idx_65);
		FIND_RESULT("float32_opt83", idx_83);

		if (idx_33 >= 0 && idx_47 >= 0 && idx_65 >= 0 && idx_83 >= 0) {
			printf("  Float32 opt:     33-tap=%6.1f  47-tap=%6.1f  65-tap=%6.1f  83-tap=%6.1f MSPS\n",
			       results[idx_33].msps_avg, results[idx_47].msps_avg,
			       results[idx_65].msps_avg, results[idx_83].msps_avg);
		}

		/* Int16 legacy across filter lengths */
		FIND_RESULT("int16_leg_33", idx_33);
		FIND_RESULT("int16_legacy", idx_47);
		FIND_RESULT("int16_leg_65", idx_65);
		FIND_RESULT("int16_leg_83", idx_83);

		if (idx_33 >= 0 && idx_47 >= 0 && idx_65 >= 0 && idx_83 >= 0) {
			printf("  Int16 legacy:    33-tap=%6.1f  47-tap=%6.1f  65-tap=%6.1f  83-tap=%6.1f MSPS\n",
			       results[idx_33].msps_avg, results[idx_47].msps_avg,
			       results[idx_65].msps_avg, results[idx_83].msps_avg);
		}

		/* Int16 optimized across filter lengths */
		FIND_RESULT("int16_opt33", idx_33);
		FIND_RESULT("int16_opt", idx_47);
		FIND_RESULT("int16_opt65", idx_65);
		FIND_RESULT("int16_opt83", idx_83);

		if (idx_33 >= 0 && idx_47 >= 0 && idx_65 >= 0 && idx_83 >= 0) {
			printf("  Int16 opt:       33-tap=%6.1f  47-tap=%6.1f  65-tap=%6.1f  83-tap=%6.1f MSPS\n",
			       results[idx_33].msps_avg, results[idx_47].msps_avg,
			       results[idx_65].msps_avg, results[idx_83].msps_avg);
		}

		printf("\n");
	}
	#undef FIND_RESULT

	printf("Benchmark complete.\n");

	return 0;
}
