/**
 * Copyright 2025-2026 Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * @file hydrasdr_async_rx.c
 * @brief Cross-platform asynchronous streaming example for HydraSDR devices.
 *
 * @details
 * This application demonstrates how to use the HydraSDR v1.1.0 API:
 *
 * **Device Management:**
 * 1. Query library version via hydrasdr_lib_version() with HYDRASDR_MAKE_VERSION compatibility check
 * 2. Open device and query capabilities via hydrasdr_get_device_info()
 *
 * **Hardware-Agnostic Configuration:**
 * 3. Configure RF parameters (Frequency, Gain, Bias-T, RF Port, Packing)
 * 4. Configure Data parameters (Sample Rate, Sample Type)
 * 5. All parameters validated against device capabilities before use
 *
 * **Streaming (v1.1.0 decoupled architecture):**
 * 6. Perform non-blocking (async) streaming to a file via hydrasdr_start_rx()
 * 7. Monitor streaming statistics via hydrasdr_get_streaming_stats() API
 * 8. Deduce sample buffer sizes dynamically from hydrasdr_transfer_t
 *
 * Usage:
 * ./hydrasdr_async_rx -o <filename> [-f freq_MHz] [-s rate_sps] [-t sample_type]
 *                     [-g linearity] [-G sensitivity] [-l lna] [-r rf] [-m mixer]
 *                     [-F filter] [-v vga] [-a lna_agc] [-A mixer_agc]
 *                     [-b bias_on_off] [-p packing] [-P rf_port] [-D <0/1>]
 *
 * All defaults are obtained dynamically from the connected device via
 * hydrasdr_get_device_info(). Run without arguments with a device connected
 * to see device-specific options and supported values.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include <errno.h>
#include <time.h>

/* Include the HydraSDR API definition */
#include "hydrasdr.h"

#define HYDRASDR_ASYNC_RX_VERSION "1.1.0 2025-2026"

/* Minimum library version required (uses HYDRASDR_MAKE_VERSION macro) */
#define MIN_LIB_VERSION HYDRASDR_MAKE_VERSION(1, 1, 0)

#if !defined(__STDC_VERSION__) || __STDC_VERSION__ < 202311L
#ifndef bool
typedef int bool;
#define true 1
#define false 0
#endif
#endif

// Cross-platform utilities
#ifdef _WIN32
	#include <windows.h>
	#define SLEEP_MS(ms) Sleep(ms)
	typedef volatile long sig_atomic_bool_t;

	static double get_time_sec(void)
	{
		LARGE_INTEGER t, f;
		QueryPerformanceCounter(&t);
		QueryPerformanceFrequency(&f);
		return (double)t.QuadPart / (double)f.QuadPart;
	}
#else /* Linux / macOS */
	#include <unistd.h>
	#include <pthread.h>
	#include <sys/time.h>

	#define SLEEP_MS(ms) usleep((ms) * 1000)
	typedef volatile sig_atomic_t sig_atomic_bool_t;

	static double get_time_sec(void)
	{
		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC, &ts);
		return ts.tv_sec + (ts.tv_nsec / 1e9);
	}
#endif

// Global Application State
static struct hydrasdr_device *g_dev = NULL;
static FILE *g_out = NULL;

static sig_atomic_bool_t g_exit_requested = 0;

/* Thread-safe 64-bit counter using atomic operations */
static volatile uint64_t g_total_bytes = 0;

/* Cross-platform atomic helpers for 64-bit counters */
#if defined(_MSC_VER)
  #define ATOMIC_ADD64(ptr, val) InterlockedExchangeAdd64((volatile LONG64*)(ptr), (LONG64)(val))
  #define ATOMIC_LOAD64(ptr) InterlockedCompareExchange64((volatile LONG64*)(ptr), 0, 0)
#elif defined(__GNUC__) || defined(__clang__)
  #define ATOMIC_ADD64(ptr, val) __atomic_fetch_add((ptr), (val), __ATOMIC_RELAXED)
  #define ATOMIC_LOAD64(ptr) __atomic_load_n((ptr), __ATOMIC_RELAXED)
#else
  /* Fallback for non-atomic (single-threaded only) */
  #define ATOMIC_ADD64(ptr, val) (*(ptr) += (val))
  #define ATOMIC_LOAD64(ptr) (*(ptr))
#endif

// Device info (global for usage display)
static hydrasdr_device_info_t g_device_info;
static bool g_have_device_info = false;

// Signal Handling
#ifdef _WIN32
BOOL WINAPI
windows_signal_handler(DWORD signum)
{
	if (!g_exit_requested)
		fprintf(stderr, "\nCaught signal %lu\n", (unsigned long)signum);

	g_exit_requested = 1;
	return TRUE;
}
#else
static void posix_signal_handler(int signum)
{
	if (!g_exit_requested)
		fprintf(stderr, "\nCaught signal %d\n", signum);

	g_exit_requested = 1;
}
#endif

/**
 * @brief Helper to calculate bytes per sample based on API enum.
 *
 * @param type The hydrasdr_sample_type enum from the transfer struct.
 * @return size_t Number of bytes per single sample (I+Q combined if applicable).
 */
static inline size_t bytes_per_sample(enum hydrasdr_sample_type type)
{
	switch (type) {
		case HYDRASDR_SAMPLE_FLOAT32_IQ:   return 8; // 4 bytes I + 4 bytes Q
		case HYDRASDR_SAMPLE_FLOAT32_REAL: return 4;
		case HYDRASDR_SAMPLE_INT16_IQ:     return 4; // 2 bytes I + 2 bytes Q
		case HYDRASDR_SAMPLE_INT16_REAL:   return 2;
		case HYDRASDR_SAMPLE_UINT16_REAL:  return 2;
		case HYDRASDR_SAMPLE_RAW:          return 2; /* Raw device stream */
		case HYDRASDR_SAMPLE_INT8_IQ:      return 2; // 1 byte I + 1 byte Q
		case HYDRASDR_SAMPLE_UINT8_IQ:     return 2; // 1 byte I + 1 byte Q
		case HYDRASDR_SAMPLE_INT8_REAL:    return 1;
		case HYDRASDR_SAMPLE_UINT8_REAL:   return 1;
		default:                           return 2;
	}
}

/**
 * @brief Get sample type name string
 */
static const char *sample_type_name(enum hydrasdr_sample_type type)
{
	switch (type) {
	case HYDRASDR_SAMPLE_FLOAT32_IQ:   return "FLOAT32_IQ";
	case HYDRASDR_SAMPLE_FLOAT32_REAL: return "FLOAT32_REAL";
	case HYDRASDR_SAMPLE_INT16_IQ:     return "INT16_IQ";
	case HYDRASDR_SAMPLE_INT16_REAL:   return "INT16_REAL";
	case HYDRASDR_SAMPLE_UINT16_REAL:  return "UINT16_REAL";
	case HYDRASDR_SAMPLE_RAW:          return "RAW";
	case HYDRASDR_SAMPLE_INT8_IQ:      return "INT8_IQ";
	case HYDRASDR_SAMPLE_UINT8_IQ:     return "UINT8_IQ";
	case HYDRASDR_SAMPLE_INT8_REAL:    return "INT8_REAL";
	case HYDRASDR_SAMPLE_UINT8_REAL:   return "UINT8_REAL";
	default:                           return "UNKNOWN";
	}
}

/**
 * @brief Static help (no device connected)
 */
static void print_usage_static(const char *prog)
{
	printf("Usage: %s -o <file> [options]\n", prog);
	printf("\nRequired:\n");
	printf(" -o <file>    Output file for captured data\n");
	printf("\nOptional:\n");
	printf(" -f <MHz>     Set RF frequency in MHz (e.g., 100.5)\n");
	printf(" -s <SPS>     Set sample rate\n");
	printf(" -B <Hz>      Set bandwidth (if supported by device)\n");
	printf(" -t <type>    Set sample type (0=FloatIQ, 1=FloatReal, 2=Int16IQ, 3=Int16Real, 5=Raw)\n");
	printf("\n Gain control (device-specific, in priority order):\n");
	printf(" -g <value>   Set linearity gain (if supported)\n");
	printf(" -G <value>   Set sensitivity gain (if supported)\n");
	printf(" -l <value>   Set LNA gain (if supported)\n");
	printf(" -r <value>   Set RF gain (if supported)\n");
	printf(" -m <value>   Set Mixer gain (if supported)\n");
	printf(" -F <value>   Set Filter gain (if supported)\n");
	printf(" -v <value>   Set VGA gain (if supported)\n");
	printf(" -a <0/1>     LNA AGC off/on (if supported)\n");
	printf(" -A <0/1>     Mixer AGC off/on (if supported)\n");
	printf("\n");
	printf(" -b <0/1>     Bias-T off/on (default: 0)\n");
	printf(" -p <0/1>     Packing mode: 0=16-bit, 1=12-bit packed (if supported)\n");
	printf(" -P <port>    Select RF port (0-based index)\n");
	printf(" -D <0/1>     Decimation mode: 0=Low Bandwidth (default), 1=High Definition\n");
	printf("              High Definition uses highest HW sample rate with decimation\n");
	printf(" -h           Show help\n");
	printf("\nNote: Connect a device and run with -h for device-specific defaults and ranges.\n");
}

/**
 * @brief Dynamic help (device connected, shows device-specific info)
 */
static void print_usage_dynamic(const char *prog, const hydrasdr_device_info_t *info)
{
	printf("Usage: %s -o <file> [options]\n", prog);
	printf("\nDevice: %s (Board ID: %u)\n", info->board_name, info->board_id);
	printf("Firmware: %s\n", info->firmware_version);
	printf("Frequency range: %.6f - %.6f MHz\n",
	       (double)info->min_frequency / 1e6,
	       (double)info->max_frequency / 1e6);
	printf("\nRequired:\n");
	printf(" -o <file>    Output file for captured data\n");
	printf("\nOptional:\n");
	printf(" -f <MHz>     Set RF frequency in MHz (default: %.6f MHz)\n",
	       (double)info->min_frequency / 1e6);
	printf(" -s <SPS>     Set sample rate (query device for available rates)\n");

	/* Show bandwidth option if supported */
	if (info->features & HYDRASDR_CAP_BANDWIDTH) {
		printf(" -B <Hz>      Set bandwidth (device supports bandwidth control)\n");
	}

	/* Show supported sample types */
	printf(" -t <type>    Set sample type. Supported types:\n");
	for (int i = 0; i < HYDRASDR_SAMPLE_END; i++) {
		if (info->sample_types & (1 << i)) {
			printf("              %d = %s%s\n", i, sample_type_name((enum hydrasdr_sample_type)i),
			       (i == HYDRASDR_SAMPLE_INT16_IQ) ? " (default)" : "");
		}
	}

	/* Show gain options based on device capabilities */
	printf("\n Gain control options (in priority order):\n");
	if (info->features & HYDRASDR_CAP_LINEARITY_GAIN) {
		printf(" -g <%u-%u>    Set linearity gain (default: %u) [PRIORITY 1]\n",
		       info->linearity_gain.min_value,
		       info->linearity_gain.max_value,
		       info->linearity_gain.default_value);
	}
	if (info->features & HYDRASDR_CAP_SENSITIVITY_GAIN) {
		printf(" -G <%u-%u>    Set sensitivity gain (default: %u) [PRIORITY 2]\n",
		       info->sensitivity_gain.min_value,
		       info->sensitivity_gain.max_value,
		       info->sensitivity_gain.default_value);
	}
	if (info->features & HYDRASDR_CAP_LNA_GAIN) {
		printf(" -l <%u-%u>    Set LNA gain (default: %u)\n",
		       info->lna_gain.min_value,
		       info->lna_gain.max_value,
		       info->lna_gain.default_value);
	}
	if (info->features & HYDRASDR_CAP_RF_GAIN) {
		printf(" -r <%u-%u>    Set RF gain (default: %u)\n",
		       info->rf_gain.min_value,
		       info->rf_gain.max_value,
		       info->rf_gain.default_value);
	}
	if (info->features & HYDRASDR_CAP_MIXER_GAIN) {
		printf(" -m <%u-%u>    Set Mixer gain (default: %u)\n",
		       info->mixer_gain.min_value,
		       info->mixer_gain.max_value,
		       info->mixer_gain.default_value);
	}
	if (info->features & HYDRASDR_CAP_FILTER_GAIN) {
		printf(" -F <%u-%u>    Set Filter gain (default: %u)\n",
		       info->filter_gain.min_value,
		       info->filter_gain.max_value,
		       info->filter_gain.default_value);
	}
	if (info->features & HYDRASDR_CAP_VGA_GAIN) {
		printf(" -v <%u-%u>    Set VGA gain (default: %u)\n",
		       info->vga_gain.min_value,
		       info->vga_gain.max_value,
		       info->vga_gain.default_value);
	}
	if (info->features & HYDRASDR_CAP_LNA_AGC) {
		printf(" -a <0/1>     LNA AGC off/on (default: 0)\n");
	}
	if (info->features & HYDRASDR_CAP_MIXER_AGC) {
		printf(" -A <0/1>     Mixer AGC off/on (default: 0)\n");
	}

	/* Show bias tee option if supported */
	if (info->features & HYDRASDR_CAP_BIAS_TEE) {
		printf(" -b <0/1>     Bias-T off/on (default: 0)\n");
	}

	/* Show packing option if supported */
	if (info->features & HYDRASDR_CAP_PACKING) {
		printf(" -p <0/1>     Packing mode: 0=16-bit (default), 1=12-bit packed\n");
	}

	/* Decimation mode (v1.1.0+ API feature) */
	printf(" -D <0/1>     Decimation mode: 0=Low Bandwidth (default), 1=High Definition\n");
	printf("              High Definition uses highest HW sample rate with decimation\n");

	/* Show RF port selection if supported */
	if ((info->features & HYDRASDR_CAP_RF_PORT_SELECT) && info->rf_port_count > 0) {
		printf(" -P <port>    Select RF port (0-%u). Available ports:\n",
		       info->rf_port_count - 1);
		for (uint8_t i = 0; i < info->rf_port_count; i++) {
			const hydrasdr_rf_port_info_t *port = &info->rf_port_info[i];
			printf("              %u: %s", i, port->name);
			if (port->min_frequency > 0 || port->max_frequency > 0) {
				printf(" (%.6f-%.6f MHz)",
				       (double)port->min_frequency / 1e6,
				       (double)port->max_frequency / 1e6);
			}
			if (port->has_bias_tee) {
				printf(" [Bias-T]");
			}
			printf("\n");
		}
	}

	printf(" -h           Show help\n");
}

/**
 * @brief Asynchronous Callback Function.
 *
 * @details This function is invoked by the HydraSDR library thread.
 * Critical Section: Execution time must be minimized.
 *
 * @param transfer Pointer to the transfer structure containing data and metadata.
 * @return int 0 to continue streaming, non-zero to request stop (internally).
 */
int rx_callback(hydrasdr_transfer_t *t)
{
	if (g_exit_requested)
		return 0;

	/* Note: Buffer drops are tracked by the library and can be queried via
	 * hydrasdr_get_streaming_stats(). The t->dropped_samples field is also
	 * available here for per-callback tracking if needed. */

	const size_t bps = bytes_per_sample(t->sample_type);
	const size_t chunk_bytes = (size_t)t->sample_count * bps;

	if (g_out && chunk_bytes > 0) {
		const size_t w = fwrite(t->samples, 1, chunk_bytes, g_out);
		if (w != chunk_bytes)
			fprintf(stderr, "Disk write error\n");
	}

	ATOMIC_ADD64(&g_total_bytes, chunk_bytes);

	return 0;
}

/**
 * @brief Main Entry Point
 */

int main(int argc, char **argv)
{
	/* Variables - initialized to 0/NULL, will get defaults from device_info */
	uint64_t freq_hz = 0;
	uint32_t samplerate = 0;
	uint32_t bandwidth = 0;
	uint8_t bias = 0;
	uint8_t packing = 0;   /* 0=16-bit (default), 1=12-bit packed */
	int sample_type = -1;  /* -1 means use device default */
	int rf_port = -1;      /* -1 means use first port */
	int decimation_mode = -1; /* -1 means not set, 0=LOW_BANDWIDTH, 1=HIGH_DEFINITION */
	const char *filename = "capture.bin";

	/* Gain control variables */
	uint8_t linearity_gain = 0;
	uint8_t sensitivity_gain = 0;
	uint8_t lna_gain = 0;
	uint8_t rf_gain = 0;
	uint8_t mixer_gain = 0;
	uint8_t filter_gain = 0;
	uint8_t vga_gain = 0;
	uint8_t lna_agc = 0;
	uint8_t mixer_agc = 0;

	/* Track which parameters were explicitly set by user */
	bool freq_set = false;
	bool samplerate_set = false;
	bool bandwidth_set = false;
	bool linearity_gain_set = false;
	bool sensitivity_gain_set = false;
	bool lna_gain_set = false;
	bool rf_gain_set = false;
	bool mixer_gain_set = false;
	bool filter_gain_set = false;
	bool vga_gain_set = false;
	bool lna_agc_set = false;
	bool mixer_agc_set = false;
	bool sample_type_set = false;
	bool rf_port_set = false;
	bool packing_set = false;
	bool decimation_mode_set = false;
	bool help_requested = false;

	// 1. Parse Command Line Arguments (Manual parsing for no deps)
	//    Show help if no arguments provided
	if (argc == 1) {
		/* Try to connect to device and show device-specific options */
		int ret = hydrasdr_open(&g_dev);
		if (ret == HYDRASDR_SUCCESS) {
			ret = hydrasdr_get_device_info(g_dev, &g_device_info);
			if (ret == HYDRASDR_SUCCESS) {
				g_have_device_info = true;
				print_usage_dynamic(argv[0], &g_device_info);

				/* Query and display supported sample rates */
				uint32_t sr_count = 0;
				ret = hydrasdr_get_samplerates(g_dev, &sr_count, 0);
				if (ret == HYDRASDR_SUCCESS && sr_count > 0) {
					uint32_t *rates = malloc(sr_count * sizeof(uint32_t));
					if (rates) {
						ret = hydrasdr_get_samplerates(g_dev, rates, sr_count);
						if (ret == HYDRASDR_SUCCESS) {
							printf("\nSupported sample rates (-s option):\n");
							for (uint32_t i = 0; i < sr_count; i++) {
								if (rates[i] >= 1000000) {
									printf("  %u = %.2f MHz\n", i, (double)rates[i] / 1e6);
								} else {
									printf("  %u = %u Hz\n", i, rates[i]);
								}
							}
						}
						free(rates);
					}
				}

				/* Query and display supported bandwidths if supported */
				if (g_device_info.features & HYDRASDR_CAP_BANDWIDTH) {
					uint32_t bw_count = 0;
					ret = hydrasdr_get_bandwidths(g_dev, &bw_count, 0);
					if (ret == HYDRASDR_SUCCESS && bw_count > 0) {
						uint32_t *bws = malloc(bw_count * sizeof(uint32_t));
						if (bws) {
							ret = hydrasdr_get_bandwidths(g_dev, bws, bw_count);
							if (ret == HYDRASDR_SUCCESS) {
								printf("\nSupported bandwidths (-B option):\n");
								for (uint32_t i = 0; i < bw_count; i++) {
									if (bws[i] >= 1000000) {
										printf("  %u = %.2f MHz\n", i, (double)bws[i] / 1e6);
									} else {
										printf("  %u = %u Hz\n", i, bws[i]);
									}
								}
							}
							free(bws);
						}
					}
				}
			}
			hydrasdr_close(g_dev);
			g_dev = NULL;
		} else {
			/* No device detected, show generic help */
			print_usage_static(argv[0]);
		}
		return 0;
	}

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-f") && i + 1 < argc) {
			freq_hz = (uint64_t)(strtod(argv[++i], NULL) * 1e6);
			freq_set = true;
		}
		else if (!strcmp(argv[i], "-s") && i + 1 < argc) {
			samplerate = (uint32_t)strtoul(argv[++i], NULL, 10);
			samplerate_set = true;
		}
		else if (!strcmp(argv[i], "-B") && i + 1 < argc) {
			bandwidth = (uint32_t)strtoul(argv[++i], NULL, 10);
			bandwidth_set = true;
		}
		else if (!strcmp(argv[i], "-t") && i + 1 < argc) {
			sample_type = atoi(argv[++i]);
			sample_type_set = true;
		}
		/* Gain control options */
		else if (!strcmp(argv[i], "-g") && i + 1 < argc) {
			linearity_gain = (uint8_t)atoi(argv[++i]);
			linearity_gain_set = true;
		}
		else if (!strcmp(argv[i], "-G") && i + 1 < argc) {
			sensitivity_gain = (uint8_t)atoi(argv[++i]);
			sensitivity_gain_set = true;
		}
		else if (!strcmp(argv[i], "-l") && i + 1 < argc) {
			lna_gain = (uint8_t)atoi(argv[++i]);
			lna_gain_set = true;
		}
		else if (!strcmp(argv[i], "-r") && i + 1 < argc) {
			rf_gain = (uint8_t)atoi(argv[++i]);
			rf_gain_set = true;
		}
		else if (!strcmp(argv[i], "-m") && i + 1 < argc) {
			mixer_gain = (uint8_t)atoi(argv[++i]);
			mixer_gain_set = true;
		}
		else if (!strcmp(argv[i], "-F") && i + 1 < argc) {
			filter_gain = (uint8_t)atoi(argv[++i]);
			filter_gain_set = true;
		}
		else if (!strcmp(argv[i], "-v") && i + 1 < argc) {
			vga_gain = (uint8_t)atoi(argv[++i]);
			vga_gain_set = true;
		}
		else if (!strcmp(argv[i], "-a") && i + 1 < argc) {
			lna_agc = (uint8_t)atoi(argv[++i]);
			lna_agc_set = true;
		}
		else if (!strcmp(argv[i], "-A") && i + 1 < argc) {
			mixer_agc = (uint8_t)atoi(argv[++i]);
			mixer_agc_set = true;
		}
		else if (!strcmp(argv[i], "-b") && i + 1 < argc) {
			bias = (uint8_t)atoi(argv[++i]);
		}
		else if (!strcmp(argv[i], "-p") && i + 1 < argc) {
			packing = (uint8_t)atoi(argv[++i]);
			packing_set = true;
		}
		else if (!strcmp(argv[i], "-P") && i + 1 < argc) {
			rf_port = atoi(argv[++i]);
			rf_port_set = true;
		}
		else if (!strcmp(argv[i], "-D") && i + 1 < argc) {
			decimation_mode = atoi(argv[++i]);
			decimation_mode_set = true;
		}
		else if (!strcmp(argv[i], "-o") && i + 1 < argc) {
			filename = argv[++i];
		}
		else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
			help_requested = true;
		}
		else {
			fprintf(stderr, "Unknown option: %s\n", argv[i]);
			print_usage_static(argv[0]);
			return 1;
		}
	}

	// 2. Setup Signal Handling (Ctrl+C)
#ifdef _WIN32
	SetConsoleCtrlHandler(windows_signal_handler, TRUE);
#else
	signal(SIGINT, posix_signal_handler);
	signal(SIGTERM, posix_signal_handler);
	signal(SIGABRT, posix_signal_handler);
#endif

	/* Display tool and library version info */
	hydrasdr_lib_version_t lib_ver;
	hydrasdr_lib_version(&lib_ver);
	printf("HydraSDR Async RX Tool v%s (libhydrasdr v%u.%u.%u)\n",
	       HYDRASDR_ASYNC_RX_VERSION,
	       lib_ver.major_version, lib_ver.minor_version, lib_ver.revision);

	/* Check library version compatibility using HYDRASDR_MAKE_VERSION macro */
	uint32_t runtime_ver = HYDRASDR_MAKE_VERSION(lib_ver.major_version,
	                                              lib_ver.minor_version,
	                                              lib_ver.revision);
	if (runtime_ver < MIN_LIB_VERSION) {
		fprintf(stderr, "[WARN] Library version too old: need v1.1.0+, got v%u.%u.%u\n",
		        lib_ver.major_version, lib_ver.minor_version, lib_ver.revision);
	}

	// 3. Open Device
	// Note: hydrasdr_open opens the first available device
	int ret = hydrasdr_open(&g_dev);
	if (ret != HYDRASDR_SUCCESS) {
		if (help_requested) {
			/* No device, show static help */
			print_usage_static(argv[0]);
			return 0;
		}
		fprintf(stderr, "ERROR: hydrasdr_open failed: %s\n",
				hydrasdr_error_name(ret));
		return EXIT_FAILURE;
	}
	printf("[INFO] Device opened.\n");

	// 4. Get device info for hardware-agnostic configuration
	ret = hydrasdr_get_device_info(g_dev, &g_device_info);
	if (ret != HYDRASDR_SUCCESS) {
		fprintf(stderr, "ERROR: hydrasdr_get_device_info failed: %s\n",
				hydrasdr_error_name(ret));
		goto error;
	}
	g_have_device_info = true;

	/* If help was requested, show dynamic help and exit */
	if (help_requested) {
		print_usage_dynamic(argv[0], &g_device_info);
		hydrasdr_close(g_dev);
		return 0;
	}

	printf("[INFO] Device: %s (FW: %s)\n",
	       g_device_info.board_name, g_device_info.firmware_version);

	// 5. Apply device defaults for parameters not set by user
	if (!freq_set) {
		freq_hz = g_device_info.min_frequency;
	}

	/* Apply default gain values from device info if not set by user */
	if (!linearity_gain_set && (g_device_info.features & HYDRASDR_CAP_LINEARITY_GAIN)) {
		linearity_gain = g_device_info.linearity_gain.default_value;
	}
	if (!sensitivity_gain_set && (g_device_info.features & HYDRASDR_CAP_SENSITIVITY_GAIN)) {
		sensitivity_gain = g_device_info.sensitivity_gain.default_value;
	}
	if (!lna_gain_set && (g_device_info.features & HYDRASDR_CAP_LNA_GAIN)) {
		lna_gain = g_device_info.lna_gain.default_value;
	}
	if (!rf_gain_set && (g_device_info.features & HYDRASDR_CAP_RF_GAIN)) {
		rf_gain = g_device_info.rf_gain.default_value;
	}
	if (!mixer_gain_set && (g_device_info.features & HYDRASDR_CAP_MIXER_GAIN)) {
		mixer_gain = g_device_info.mixer_gain.default_value;
	}
	if (!filter_gain_set && (g_device_info.features & HYDRASDR_CAP_FILTER_GAIN)) {
		filter_gain = g_device_info.filter_gain.default_value;
	}
	if (!vga_gain_set && (g_device_info.features & HYDRASDR_CAP_VGA_GAIN)) {
		vga_gain = g_device_info.vga_gain.default_value;
	}
	if (!sample_type_set) {
		/* Find first supported sample type, prefer INT16_IQ */
		if (g_device_info.sample_types & (1 << HYDRASDR_SAMPLE_INT16_IQ)) {
			sample_type = HYDRASDR_SAMPLE_INT16_IQ;
		} else {
			for (int i = 0; i < HYDRASDR_SAMPLE_END; i++) {
				if (g_device_info.sample_types & (1 << i)) {
					sample_type = i;
					break;
				}
			}
		}
	}
	if (!rf_port_set) {
		rf_port = 0;  /* Use first port by default */
	}

	// 6. Print supported samplerates and get default if not set
	uint32_t count = 0;
	hydrasdr_get_samplerates(g_dev, &count, 0);

	if (count > 0) {
		uint32_t *rates = malloc(count * sizeof(uint32_t));
		if (rates) {
			hydrasdr_get_samplerates(g_dev, rates, count);
			printf("Available sample rates:\n");
			for (uint32_t i = 0; i < count; i++)
				printf("  %u (%.3f MSPS)\n", rates[i], rates[i] / 1e6);

			/* Use first available rate if not set by user */
			if (!samplerate_set && count > 0) {
				samplerate = rates[0];
			}

			/* Resolve sample rate: support both index and direct SPS value */
			if (samplerate_set) {
				/* First check if it matches an available rate directly */
				bool found = false;
				for (uint32_t i = 0; i < count; i++) {
					if (samplerate == rates[i]) {
						found = true;
						break;
					}
				}
				/* If not found and value is small, treat as index */
				if (!found && samplerate < count) {
					printf("[INFO] Interpreting -s %u as index -> %u SPS\n",
					       samplerate, rates[samplerate]);
					samplerate = rates[samplerate];
				}
			}
			free(rates);
		}
	}
	printf("\n");

	// 7. Apply Configuration

	// A. Validate and set frequency
	if (freq_hz < g_device_info.min_frequency || freq_hz > g_device_info.max_frequency) {
		fprintf(stderr, "[ERROR] Frequency %.6f MHz out of range (%.6f - %.6f MHz)\n",
			(double)freq_hz / 1e6,
			(double)g_device_info.min_frequency / 1e6,
			(double)g_device_info.max_frequency / 1e6);
		ret = HYDRASDR_ERROR_INVALID_PARAM;
		goto error;
	}
	printf("[CONF] Frequency:   %.6f MHz\n", (double)freq_hz / 1e6);
	ret = hydrasdr_set_freq(g_dev, freq_hz);
	if (ret != HYDRASDR_SUCCESS) {
		fprintf(stderr, "[ERROR] Failed set frequency: %s\n", hydrasdr_error_name(ret));
		goto error;
	}

	// B. Decimation Mode (must be set before sample rate)
	if (decimation_mode_set) {
		if (decimation_mode < 0 || decimation_mode > 1) {
			fprintf(stderr, "[ERROR] Invalid decimation mode %d (must be 0 or 1)\n", decimation_mode);
			ret = HYDRASDR_ERROR_INVALID_PARAM;
			goto error;
		}
		printf("[CONF] Decimation:  %s\n",
		       decimation_mode ? "High Definition (max HW rate + decimation)" : "Low Bandwidth (min HW rate)");
		ret = hydrasdr_set_decimation_mode(g_dev, (enum hydrasdr_decimation_mode)decimation_mode);
		if (ret != HYDRASDR_SUCCESS) {
			fprintf(stderr, "[ERROR] Failed set decimation mode: %s\n", hydrasdr_error_name(ret));
			goto error;
		}
	}

	// C. Bandwidth (must be set before sample rate for auto-bandwidth to work correctly)
	if (g_device_info.features & HYDRASDR_CAP_BANDWIDTH) {
		uint32_t bw_count = 0;
		uint32_t *supported_bandwidths = NULL;

		/* Query available bandwidths */
		hydrasdr_get_bandwidths(g_dev, &bw_count, 0);
		if (bw_count > 0) {
			supported_bandwidths = (uint32_t *)malloc(bw_count * sizeof(uint32_t));
			if (supported_bandwidths) {
				hydrasdr_get_bandwidths(g_dev, supported_bandwidths, bw_count);

				printf("Available bandwidths:\n");
				for (uint32_t i = 0; i < bw_count; i++) {
					printf("  %u (%.3f MHz)\n", supported_bandwidths[i],
						supported_bandwidths[i] / 1e6);
				}

				/* Set bandwidth if user specified one */
				if (bandwidth_set) {
					/* Resolve bandwidth: support both index and direct Hz value */
					uint32_t bw_to_set = bandwidth;
					bool found = false;
					for (uint32_t i = 0; i < bw_count; i++) {
						if (bandwidth == supported_bandwidths[i]) {
							found = true;
							break;
						}
					}
					/* If not found and value is small, treat as index */
					if (!found && bandwidth < bw_count) {
						printf("[INFO] Interpreting -B %u as index -> %u Hz\n",
						       bandwidth, supported_bandwidths[bandwidth]);
						bw_to_set = supported_bandwidths[bandwidth];
					}

					printf("[CONF] Bandwidth:   %u Hz (%.3f MHz)\n", bw_to_set, bw_to_set / 1e6);
					ret = hydrasdr_set_bandwidth(g_dev, bw_to_set);
					if (ret != HYDRASDR_SUCCESS) {
						fprintf(stderr, "[ERROR] Failed set bandwidth: %s\n",
							hydrasdr_error_name(ret));
						free(supported_bandwidths);
						goto error;
					}
				}
				free(supported_bandwidths);
			}
		}
	} else if (bandwidth_set) {
		fprintf(stderr, "[WARN] Bandwidth control not supported on this device.\n");
	}

	// D. Sample Rate (after decimation mode and bandwidth)
	printf("[CONF] Samplerate:  %u SPS (%.3f MSPS)\n", samplerate, samplerate / 1e6);
	ret = hydrasdr_set_samplerate(g_dev, samplerate);
	if (ret != HYDRASDR_SUCCESS) {
		fprintf(stderr, "[ERROR] Failed set sample rate: %s\n", hydrasdr_error_name(ret));
		goto error;
	}

	// E. Validate and set sample type
	if (sample_type < 0 || sample_type >= HYDRASDR_SAMPLE_END) {
		fprintf(stderr, "[ERROR] Invalid sample type %d\n", sample_type);
		ret = HYDRASDR_ERROR_INVALID_PARAM;
		goto error;
	}
	if (!(g_device_info.sample_types & (1 << sample_type))) {
		fprintf(stderr, "[ERROR] Sample type %d (%s) not supported by this device.\n",
			sample_type, sample_type_name((enum hydrasdr_sample_type)sample_type));
		fprintf(stderr, "        Supported types:\n");
		for (int i = 0; i < HYDRASDR_SAMPLE_END; i++) {
			if (g_device_info.sample_types & (1 << i)) {
				fprintf(stderr, "          %d = %s\n", i,
					sample_type_name((enum hydrasdr_sample_type)i));
			}
		}
		ret = HYDRASDR_ERROR_INVALID_PARAM;
		goto error;
	}
	printf("[CONF] Sample type: %d (%s)\n", sample_type,
	       sample_type_name((enum hydrasdr_sample_type)sample_type));
	ret = hydrasdr_set_sample_type(g_dev, (enum hydrasdr_sample_type)sample_type);
	if (ret != HYDRASDR_SUCCESS) {
		fprintf(stderr, "[ERROR] Failed set sample type: %s\n", hydrasdr_error_name(ret));
		goto error;
	}

	// F. Gain Configuration (hardware-agnostic - apply all supported gain controls)
	//    Priority order: Linearity > Sensitivity > Individual gains (LNA, Mixer, VGA)
	//    Note: Linearity/Sensitivity are mutually exclusive presets that configure
	//          internal LNA/Mixer/VGA automatically. If user sets individual gains,
	//          they override any preset mode.

	bool using_preset_gain = false;

	/* Priority 1: Linearity gain preset (if supported and requested or default) */
	if ((g_device_info.features & HYDRASDR_CAP_LINEARITY_GAIN) &&
	    (linearity_gain_set || (!sensitivity_gain_set && !lna_gain_set &&
	     !rf_gain_set && !mixer_gain_set && !filter_gain_set && !vga_gain_set))) {
		printf("[CONF] Linearity Gain: %u (range: %u-%u)\n", linearity_gain,
		       g_device_info.linearity_gain.min_value,
		       g_device_info.linearity_gain.max_value);
		ret = hydrasdr_set_gain(g_dev, HYDRASDR_GAIN_TYPE_LINEARITY, linearity_gain);
		if (ret != HYDRASDR_SUCCESS) {
			fprintf(stderr, "[ERROR] Failed set linearity gain: %s\n",
				hydrasdr_error_name(ret));
			goto error;
		}
		using_preset_gain = true;
	}
	/* Priority 2: Sensitivity gain preset (if supported and requested) */
	else if ((g_device_info.features & HYDRASDR_CAP_SENSITIVITY_GAIN) &&
	         sensitivity_gain_set) {
		printf("[CONF] Sensitivity Gain: %u (range: %u-%u)\n", sensitivity_gain,
		       g_device_info.sensitivity_gain.min_value,
		       g_device_info.sensitivity_gain.max_value);
		ret = hydrasdr_set_gain(g_dev, HYDRASDR_GAIN_TYPE_SENSITIVITY, sensitivity_gain);
		if (ret != HYDRASDR_SUCCESS) {
			fprintf(stderr, "[ERROR] Failed set sensitivity gain: %s\n",
				hydrasdr_error_name(ret));
			goto error;
		}
		using_preset_gain = true;
	}

	/* Individual gain controls (if not using preset, or if explicitly set by user) */
	if (!using_preset_gain || lna_gain_set || rf_gain_set || mixer_gain_set ||
	    filter_gain_set || vga_gain_set) {
		/* LNA Gain */
		if (g_device_info.features & HYDRASDR_CAP_LNA_GAIN) {
			printf("[CONF] LNA Gain:    %u (range: %u-%u)\n", lna_gain,
			       g_device_info.lna_gain.min_value,
			       g_device_info.lna_gain.max_value);
			ret = hydrasdr_set_gain(g_dev, HYDRASDR_GAIN_TYPE_LNA, lna_gain);
			if (ret != HYDRASDR_SUCCESS) {
				fprintf(stderr, "[ERROR] Failed set LNA gain: %s\n",
					hydrasdr_error_name(ret));
				goto error;
			}
		}

		/* RF Gain */
		if (g_device_info.features & HYDRASDR_CAP_RF_GAIN) {
			printf("[CONF] RF Gain:     %u (range: %u-%u)\n", rf_gain,
			       g_device_info.rf_gain.min_value,
			       g_device_info.rf_gain.max_value);
			ret = hydrasdr_set_gain(g_dev, HYDRASDR_GAIN_TYPE_RF, rf_gain);
			if (ret != HYDRASDR_SUCCESS) {
				fprintf(stderr, "[ERROR] Failed set RF gain: %s\n",
					hydrasdr_error_name(ret));
				goto error;
			}
		}

		/* Mixer Gain */
		if (g_device_info.features & HYDRASDR_CAP_MIXER_GAIN) {
			printf("[CONF] Mixer Gain:  %u (range: %u-%u)\n", mixer_gain,
			       g_device_info.mixer_gain.min_value,
			       g_device_info.mixer_gain.max_value);
			ret = hydrasdr_set_gain(g_dev, HYDRASDR_GAIN_TYPE_MIXER, mixer_gain);
			if (ret != HYDRASDR_SUCCESS) {
				fprintf(stderr, "[ERROR] Failed set Mixer gain: %s\n",
					hydrasdr_error_name(ret));
				goto error;
			}
		}

		/* Filter Gain */
		if (g_device_info.features & HYDRASDR_CAP_FILTER_GAIN) {
			printf("[CONF] Filter Gain: %u (range: %u-%u)\n", filter_gain,
			       g_device_info.filter_gain.min_value,
			       g_device_info.filter_gain.max_value);
			ret = hydrasdr_set_gain(g_dev, HYDRASDR_GAIN_TYPE_FILTER, filter_gain);
			if (ret != HYDRASDR_SUCCESS) {
				fprintf(stderr, "[ERROR] Failed set Filter gain: %s\n",
					hydrasdr_error_name(ret));
				goto error;
			}
		}

		/* VGA Gain */
		if (g_device_info.features & HYDRASDR_CAP_VGA_GAIN) {
			printf("[CONF] VGA Gain:    %u (range: %u-%u)\n", vga_gain,
			       g_device_info.vga_gain.min_value,
			       g_device_info.vga_gain.max_value);
			ret = hydrasdr_set_gain(g_dev, HYDRASDR_GAIN_TYPE_VGA, vga_gain);
			if (ret != HYDRASDR_SUCCESS) {
				fprintf(stderr, "[ERROR] Failed set VGA gain: %s\n",
					hydrasdr_error_name(ret));
				goto error;
			}
		}
	}

	/* AGC Controls (independent of gain presets) */
	if (g_device_info.features & HYDRASDR_CAP_LNA_AGC) {
		if (lna_agc_set || lna_agc) {
			printf("[CONF] LNA AGC:     %s\n", lna_agc ? "ON" : "OFF");
			ret = hydrasdr_set_gain(g_dev, HYDRASDR_GAIN_TYPE_LNA_AGC, lna_agc);
			if (ret != HYDRASDR_SUCCESS) {
				fprintf(stderr, "[ERROR] Failed set LNA AGC: %s\n",
					hydrasdr_error_name(ret));
				goto error;
			}
		}
	}

	if (g_device_info.features & HYDRASDR_CAP_MIXER_AGC) {
		if (mixer_agc_set || mixer_agc) {
			printf("[CONF] Mixer AGC:   %s\n", mixer_agc ? "ON" : "OFF");
			ret = hydrasdr_set_gain(g_dev, HYDRASDR_GAIN_TYPE_MIXER_AGC, mixer_agc);
			if (ret != HYDRASDR_SUCCESS) {
				fprintf(stderr, "[ERROR] Failed set Mixer AGC: %s\n",
					hydrasdr_error_name(ret));
				goto error;
			}
		}
	}

	// G. RF Port (if supported)
	if ((g_device_info.features & HYDRASDR_CAP_RF_PORT_SELECT) &&
	    g_device_info.rf_port_count > 0) {
		if (rf_port < 0 || (uint8_t)rf_port >= g_device_info.rf_port_count) {
			fprintf(stderr, "[ERROR] RF port %d out of range (0-%u)\n",
				rf_port, g_device_info.rf_port_count - 1);
			ret = HYDRASDR_ERROR_INVALID_PARAM;
			goto error;
		}
		printf("[CONF] RF Port:     %d (%s)\n", rf_port,
		       g_device_info.rf_port_info[rf_port].name);
		ret = hydrasdr_set_rf_port(g_dev, (hydrasdr_rf_port_t)rf_port);
		if (ret != HYDRASDR_SUCCESS) {
			fprintf(stderr, "[ERROR] Failed set RF port: %s\n",
				hydrasdr_error_name(ret));
			goto error;
		}
	}

	// H. Bias Tee (if supported)
	if (g_device_info.features & HYDRASDR_CAP_BIAS_TEE) {
		printf("[CONF] Bias-T:      %u\n", bias);
		ret = hydrasdr_set_rf_bias(g_dev, bias);
		if (ret != HYDRASDR_SUCCESS) {
			fprintf(stderr, "[ERROR] Failed set Bias Tee: %s\n",
				hydrasdr_error_name(ret));
			goto error;
		}
		if (bias)
			printf("[WARN] Bias-T ENABLED.\n");
	} else if (bias) {
		fprintf(stderr, "[WARN] Bias-T not supported on this device.\n");
	}

	// I. Packing Mode (if supported)
	if (g_device_info.features & HYDRASDR_CAP_PACKING) {
		printf("[CONF] Packing:     %s\n", packing ? "12-bit packed" : "16-bit");
		ret = hydrasdr_set_packing(g_dev, packing);
		if (ret != HYDRASDR_SUCCESS) {
			fprintf(stderr, "[ERROR] Failed set packing mode: %s\n",
				hydrasdr_error_name(ret));
			goto error;
		}
	} else if (packing_set && packing) {
		fprintf(stderr, "[WARN] Packing mode not supported on this device.\n");
	}

	// J. Open Output File
	g_out = fopen(filename, "wb");
	if (!g_out) {
		fprintf(stderr, "ERROR: Cannot open '%s': %s\n",
				filename, strerror(errno));
		ret = HYDRASDR_ERROR_OTHER;
		goto error;
	}
	printf("[CONF] Output file: %s\n", filename);

	// 8. Start Streaming (Async)
	printf("\n[INFO] Starting stream... (Press Ctrl+C to stop)\n");

	// Pass 'NULL' as user context (last arg) since we use global vars for this simple example
	ret = hydrasdr_start_rx(g_dev, rx_callback, NULL);
	if (ret) {
		fprintf(stderr, "ERROR: start_rx failed: %s\n",
				hydrasdr_error_name(ret));
		goto error;
	}

	// 9. Monitoring Loop (uses hydrasdr_get_streaming_stats() API for buffer stats)
	const double t_start = get_time_sec();
	double t_last = t_start;
	uint64_t last_bytes = 0;
	hydrasdr_streaming_stats_t stats;

	const size_t bps = bytes_per_sample((enum hydrasdr_sample_type)sample_type);

	while (!g_exit_requested) {
		SLEEP_MS(1000);

		if (hydrasdr_is_streaming(g_dev) != HYDRASDR_TRUE) {
			fprintf(stderr, "\n[ERROR] Device stopped streaming.\n");
			g_exit_requested = 1;
			break;
		}

		double t_now = get_time_sec();
		double dt = t_now - t_last;

		if (dt >= 1.0 && !g_exit_requested) {
			const uint64_t bytes_now = ATOMIC_LOAD64(&g_total_bytes);
			const uint64_t dbytes = bytes_now - last_bytes;

			const double inst_msps = (dbytes / (double)bps) / (dt * 1e6);
			const double avg_msps  = (bytes_now / (double)bps) / ((t_now - t_start) * 1e6);

			/* Get streaming stats from library API */
			uint64_t buf_dropped = 0;
			if (hydrasdr_get_streaming_stats(g_dev, &stats) == HYDRASDR_SUCCESS) {
				buf_dropped = stats.buffers_dropped;
			}

			printf("Time %4.0fs | Inst %5.2f MSPS | Avg %5.2f MSPS | Vol %7.2f MB | Drops %llu\r",
				   t_now - t_start,
				   inst_msps,
				   avg_msps,
				   bytes_now / (1024.0 * 1024.0),
				   (unsigned long long)buf_dropped);
			fflush(stdout);

			last_bytes = bytes_now;
			t_last = t_now;
		}
	}

	printf("\n\n[INFO] Stopping ...\n");

	/* Print final streaming statistics from library API */
	if (hydrasdr_get_streaming_stats(g_dev, &stats) == HYDRASDR_SUCCESS) {
		printf("\n=== Streaming Statistics ===\n");
		printf("  Buffers received:  %llu\n", (unsigned long long)stats.buffers_received);
		printf("  Buffers processed: %llu\n", (unsigned long long)stats.buffers_processed);
		printf("  Buffers dropped:   %llu\n", (unsigned long long)stats.buffers_dropped);
		if (stats.buffers_dropped > 0) {
			printf("  [WARN] Data loss detected - callback processing too slow\n");
		}
	}

error:
	if (g_dev) {
		hydrasdr_stop_rx(g_dev);
		hydrasdr_close(g_dev);
	}
	if (g_out)
		fclose(g_out);

	printf("[INFO] Done.\n");
	return ret == HYDRASDR_SUCCESS ? EXIT_SUCCESS : EXIT_FAILURE;
}
