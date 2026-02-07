/*
 * HydraSDR Shared Utilities
 *
 * Copyright (C) 2013-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HYDRASDR_SHARED_H
#define HYDRASDR_SHARED_H

#include "hydrasdr_internal.h"
#include "compat_opt.h"
#include "spsc_queue.h"
#define HAVE_LIBUSB 1  /* Enable libusb integration in buffer_pool.h */
#include "buffer_pool.h"
#include <pthread.h>

/* Sample packing constants */
#define UNPACKED_SIZE (16)
#define PACKED_SIZE (12)
#define RAW_BUFFER_COUNT (8)
#define SAMPLE_RESOLUTION 12
#define SAMPLE_ENCAPSULATION 16
#define SAMPLE_SHIFT (SAMPLE_ENCAPSULATION - SAMPLE_RESOLUTION)
#define SAMPLE_SCALE (1.0f / (1 << (15 - SAMPLE_SHIFT)))

/* Rate/bandwidth validation limits */
#define MIN_SAMPLERATE_BY_VALUE (10000)    /* 10 kHz - minimum rate specified by value */
#define MIN_BANDWIDTH_BY_VALUE (1000)      /* 1 kHz - minimum bandwidth specified by value */
#define MAX_SAMPLERATE_HZ (100000000)      /* 100 MHz - max valid samplerate/bandwidth */
#define SAMPLE_TYPE_IS_IQ(x) ((x) == HYDRASDR_SAMPLE_FLOAT32_IQ || (x) == HYDRASDR_SAMPLE_INT16_IQ || \
                              (x) == HYDRASDR_SAMPLE_INT8_IQ || (x) == HYDRASDR_SAMPLE_UINT8_IQ)

/*
 * Data format from firmware (returned by HYDRASDR_SET_SAMPLERATE)
 * Determines whether DDC processing is needed or data is already IQ.
 */
#define HYDRASDR_DATA_RAW_ADC    0  /* Raw ADC samples -> needs DDC processing */
#define HYDRASDR_DATA_IQ_DIRECT  1  /* Direct IQ from firmware -> skip DDC */

/* Threading timeout constants (microseconds) */
#define TRANSFER_THREAD_TIMEOUT_US (100000)  /* 100ms - fast response to stop */
#define EVENT_DRAIN_TIMEOUT_US (50000)       /* 50ms - event drain per iteration */
#define EVENT_DRAIN_ITERATIONS (2)           /* Number of drain iterations */

/* Buffer size constants */
#define DEFAULT_BUFFER_SIZE (262144)              /* 256KB - default USB buffer size */
#define PACKED_BUFFER_SIZE (6144 * 24)            /* 147KB - packed mode buffer size */
#define MAX_SUPPORTED_RATE_COUNT (100)            /* Sanity limit for rate/bandwidth arrays */

/* Version string buffer size */
#define VERSION_STRING_SIZE (128)

/*
 * Generic streaming data structure
 *
 * IMPORTANT: Structure is organized to minimize false sharing between
 * USB callback thread and consumer thread. Hot variables are separated
 * onto different cache lines.
 *
 * Uses lock-free SPSC queue for zero-copy buffer passing between USB
 * callback and consumer thread, providing low latency streaming.
 */
typedef struct {
	/* ---- Read-mostly / initialization data ---- */
	struct libusb_transfer** transfers;
	hydrasdr_sample_block_cb_fn callback;
	pthread_t transfer_thread;
	pthread_t consumer_thread;
	bool transfer_thread_running;
	bool consumer_thread_running;
	uint32_t transfer_count;
	uint32_t buffer_size;
	bool packing_enabled;
	uint8_t adc_bits;	/* ADC bit depth from firmware: 8, 10, 12, 14, or 16 */
	uint8_t data_format;	/* Data format from firmware: RAW_ADC or IQ_DIRECT */
	enum hydrasdr_sample_type sample_type;
	struct hydrasdr_device* dev;
	void* ctx;

	/* Supported rates/bandwidths (read-only after init) */
	uint32_t supported_samplerate_count;
	uint32_t *supported_samplerates;
	hydrasdr_samplerate_info_t *samplerate_info;	/* Extended info per rate (if HYDRASDR_CAP_EXTENDED_SAMPLERATES) */
	bool has_extended_samplerate_info;	/* true if samplerate_info is populated */
	uint32_t supported_bandwidth_count;
	uint32_t *supported_bandwidths;

	/* Cached device info (read-only after init, avoids repeated USB queries) */
	uint8_t cached_board_id;
	char cached_version_string[VERSION_STRING_SIZE];
	hydrasdr_read_partid_serialno_t cached_part_serial;
	uint32_t cached_features;
	uint32_t cached_features_reserved[3];
	bool device_info_cached;

	/* Cached gain info (updated on set, avoids USB queries on get) */
	hydrasdr_gain_info_t cached_gain_info[HYDRASDR_GAIN_TYPE_COUNT];
	bool gains_cached[HYDRASDR_GAIN_TYPE_COUNT];

	/* Ring buffer storage (accessed by both but via indices) */
	uint16_t *received_samples_queue[RAW_BUFFER_COUNT];

	/* DDC instances */
	iqconverter_t *cnv_f;	/* Float32 DDC instance */
	iqconverter_t *cnv_i;	/* Int16 DDC instance */

	/*
	 * ---- Decimation state (for virtual sample rates) ----
	 * Enables lower USB bandwidth by using hardware rate with DDC decimation.
	 * Example: 10 MHz HW rate with 4x decimation = 2.5 MHz effective rate.
	 */
	uint32_t hardware_samplerate;	/* Actual ADC rate sent to hardware */
	uint32_t effective_samplerate;	/* User-requested rate after decimation */
	uint32_t decimation_factor;	/* 1, 2, 4, 8, 16, 32, or 64 */
	uint32_t decimation_mode;	/* enum hydrasdr_decimation_mode */

	/* Bandwidth auto-selection tracking */
	bool bandwidth_explicitly_set;	/* true if user called set_bandwidth() */
	uint32_t current_bandwidth;	/* Currently set bandwidth in Hz (0 if not set) */
	uint32_t pending_auto_bandwidth_idx;	/* Auto-selected BW index to apply after streaming starts (0 = none) */
	uint32_t pending_auto_bandwidth_hz;	/* Auto-selected BW in Hz (for tracking) */

	/* Virtual sample rate table (hardware rates + decimated variants) */
	uint32_t *virtual_samplerates;
	uint32_t virtual_samplerate_count;

	/* Output triple buffer (allows callback to hold buffer while next processes) */
	triple_buffer_t output_triple;
	uint16_t *unpacked_samples;

	/*
	 * ---- Lock-free SPSC queue ----
	 * Zero-copy buffer passing between USB callback and consumer
	 */
	OPT_ALIGN(OPT_CACHE_LINE_SIZE) spsc_queue_t spsc;

	/*
	 * ---- Buffer tracking for cleanup ----
	 * Tracks ALL allocated buffers to ensure proper cleanup regardless of
	 * swap state. Buffers may move between USB transfers and SPSC slots
	 * during streaming, but this array always holds the original pointers.
	 * Max: RAW_BUFFER_COUNT + transfer_count + SPSC_QUEUE_CAPACITY = 8+16+16 = 40
	 */
	#define BUFFER_TRACKING_MAX 64
	void *tracked_buffers[BUFFER_TRACKING_MAX];
	uint32_t tracked_buffer_count;

	/*
	 * ---- USB callback thread hot data (own cache line) ----
	 * Written by: USB callback thread
	 * Read by: Consumer thread
	 *
	 * IMPORTANT: streaming and stop_requested use atomic operations
	 * via OPT_ATOMIC_LOAD_ACQ/OPT_ATOMIC_STORE_REL for thread safety.
	 */
	OPT_ALIGN(OPT_CACHE_LINE_SIZE) struct {
		uint32_t dropped_buffers;         /* Overflow counter */
		uint32_t streaming;               /* Streaming active flag (atomic) */
		uint32_t stop_requested;          /* Stop request flag (atomic) */
	} usb_hot;

	/*
	 * ---- Event signaling for consumer wakeup ----
	 * Hybrid approach: lock-free SPSC queue + event for low-latency wakeup
	 */
	pthread_mutex_t consumer_mutex;
	pthread_cond_t consumer_cond;

	/*
	 * ---- Statistics (own cache line) ----
	 * Performance monitoring counters
	 */
	OPT_ALIGN(OPT_CACHE_LINE_SIZE) struct {
		uint64_t total_buffers_received;
		uint64_t total_buffers_processed;
		uint64_t total_dropped_buffers;
	} stats;

} hydrasdr_streaming_t;

/* Streaming control */
int hydrasdr_generic_start_rx(struct hydrasdr_device* dev, hydrasdr_sample_block_cb_fn callback, void* rx_ctx, uint8_t endpoint);
int hydrasdr_generic_stop_rx(struct hydrasdr_device* dev);
int hydrasdr_generic_is_streaming(struct hydrasdr_device* dev);

/* Generic device initialization/cleanup helpers */
int hydrasdr_generic_device_init(struct hydrasdr_device* dev, uint32_t transfer_count, uint32_t buffer_size, uint8_t enable_packing_default, uint32_t fallback_caps);
int hydrasdr_generic_device_exit(struct hydrasdr_device* dev);
int hydrasdr_generic_init_streaming(struct hydrasdr_device* dev, uint32_t transfer_count, uint32_t buffer_size, enum hydrasdr_sample_type default_sample_type, uint32_t fallback_caps);
int hydrasdr_generic_cleanup_streaming(struct hydrasdr_device* dev);

/* Generic device control functions */
int hydrasdr_generic_set_freq(struct hydrasdr_device* dev, const uint64_t freq_hz);
int hydrasdr_generic_set_samplerate(struct hydrasdr_device* dev, uint32_t samplerate);
int hydrasdr_generic_get_samplerates(struct hydrasdr_device* dev, uint32_t* buffer, const uint32_t len);

/* Decimation-aware sample rate API */
int hydrasdr_generic_build_virtual_samplerates(struct hydrasdr_device* dev);
int hydrasdr_generic_get_decimation_factor(struct hydrasdr_device* dev, uint32_t* factor);
int hydrasdr_generic_get_hardware_samplerate(struct hydrasdr_device* dev, uint32_t* rate);
int hydrasdr_generic_set_decimation_mode(struct hydrasdr_device* dev, uint32_t mode);
int hydrasdr_generic_get_decimation_mode(struct hydrasdr_device* dev, uint32_t* mode);
int hydrasdr_generic_set_bandwidth(struct hydrasdr_device* dev, uint32_t bandwidth);
int hydrasdr_generic_get_bandwidths(struct hydrasdr_device* dev, uint32_t* buffer, const uint32_t len);
int hydrasdr_generic_set_rf_bias(struct hydrasdr_device* dev, uint8_t value);
int hydrasdr_generic_set_packing(struct hydrasdr_device* dev, uint8_t value);
int hydrasdr_generic_set_rf_port(struct hydrasdr_device* dev, hydrasdr_rf_port_t rf_port);
int hydrasdr_generic_reset(struct hydrasdr_device* dev);
int hydrasdr_generic_set_sample_type(struct hydrasdr_device* dev, enum hydrasdr_sample_type sample_type);

/* Generic board information functions */
int hydrasdr_generic_board_id_read(struct hydrasdr_device* dev, uint8_t* value);
int hydrasdr_generic_version_string_read(struct hydrasdr_device* dev, char* version, uint8_t length);
int hydrasdr_generic_board_partid_serialno_read(struct hydrasdr_device* dev, hydrasdr_read_partid_serialno_t* read_partid_serialno);

/* Generic capability functions */
int hydrasdr_generic_get_capabilities(struct hydrasdr_device* dev, uint32_t* features, uint32_t* features_reserved, uint32_t fallback_caps);

/* Generic gain control functions (legacy - separate commands per gain type) */
int hydrasdr_generic_set_lna_gain(struct hydrasdr_device* dev, uint8_t value, uint8_t max_value);
int hydrasdr_generic_set_mixer_gain(struct hydrasdr_device* dev, uint8_t value, uint8_t max_value);
int hydrasdr_generic_set_vga_gain(struct hydrasdr_device* dev, uint8_t value, uint8_t max_value);
int hydrasdr_generic_set_lna_agc(struct hydrasdr_device* dev, uint8_t value);
int hydrasdr_generic_set_mixer_agc(struct hydrasdr_device* dev, uint8_t value);

/* Unified gain control functions (HYDRASDR_CAP_EXTENDED_GAIN) */
int hydrasdr_generic_get_gain(struct hydrasdr_device* dev, hydrasdr_gain_type_t type, hydrasdr_gain_info_t* info);
int hydrasdr_generic_set_gain(struct hydrasdr_device* dev, hydrasdr_gain_type_t type, uint8_t value);
int hydrasdr_generic_set_gain_info(struct hydrasdr_device* dev, hydrasdr_gain_type_t type, const hydrasdr_gain_info_t* info);
int hydrasdr_generic_get_all_gains(struct hydrasdr_device* dev, hydrasdr_gain_info_t* gains, uint8_t* count);

/* Generic conversion filter management */
int hydrasdr_generic_set_conversion_filter_float32(struct hydrasdr_device* dev, const float* kernel, const uint32_t len);
int hydrasdr_generic_set_conversion_filter_int16(struct hydrasdr_device* dev, const int16_t* kernel, const uint32_t len);
int hydrasdr_generic_set_conversion_algorithm(struct hydrasdr_device* dev, const char* algorithm);
int hydrasdr_generic_list_conversion_algorithms(const char **names, const char **descriptions, int max);

/* Generic device-specific operations */
int hydrasdr_generic_set_receiver_mode(struct hydrasdr_device* dev, hydrasdr_receiver_mode_t value);

/* ========================================================================
 * Peripheral Module Functions
 * ======================================================================== */

/* GPIO value functions */
int hydrasdr_generic_gpio_write(struct hydrasdr_device* dev, hydrasdr_gpio_port_t port, hydrasdr_gpio_pin_t pin, uint8_t val);
int hydrasdr_generic_gpio_read(struct hydrasdr_device* dev, hydrasdr_gpio_port_t port, hydrasdr_gpio_pin_t pin, uint8_t* val);

/* GPIO direction functions */
int hydrasdr_generic_gpiodir_write(struct hydrasdr_device* dev, hydrasdr_gpio_port_t port, hydrasdr_gpio_pin_t pin, uint8_t val);
int hydrasdr_generic_gpiodir_read(struct hydrasdr_device* dev, hydrasdr_gpio_port_t port, hydrasdr_gpio_pin_t pin, uint8_t* val);

/* Clock generator register access functions */
int hydrasdr_generic_clockgen_write(struct hydrasdr_device* dev, uint8_t reg, uint8_t val);
int hydrasdr_generic_clockgen_read(struct hydrasdr_device* dev, uint8_t reg, uint8_t* val);

/* SPI flash functions */
int hydrasdr_generic_spiflash_erase(struct hydrasdr_device* dev);
int hydrasdr_generic_spiflash_write(struct hydrasdr_device* dev, const uint32_t addr, const uint16_t len, unsigned char* const data);
int hydrasdr_generic_spiflash_read(struct hydrasdr_device* dev, const uint32_t addr, const uint16_t len, unsigned char* data);
int hydrasdr_generic_spiflash_erase_sector(struct hydrasdr_device* dev, const uint16_t sector_num);

/* RF frontend register access functions (future-proof types) */
int hydrasdr_generic_rf_frontend_write(struct hydrasdr_device* dev, uint16_t reg, uint32_t val);
int hydrasdr_generic_rf_frontend_read(struct hydrasdr_device* dev, uint16_t reg, uint32_t* val);

/* ========================================================================
 * Sample Unpacking Functions (12-bit packed to 16-bit)
 * ======================================================================== */

/* MSVC doesn't support C99 restrict in C mode, use __restrict instead */
#if defined(_MSC_VER) && !defined(__cplusplus)
#define HYDRASDR_RESTRICT __restrict
#else
#define HYDRASDR_RESTRICT restrict
#endif

/**
 * @brief Unpack 12-bit samples from packed format (optimized version)
 *
 * Unpacks 8 x 12-bit samples from 3 x 32-bit words (96 bits total).
 * This version is optimized with loop unrolling and local caching.
 *
 * @param input   Packed input buffer (must be 32-bit aligned)
 * @param output  Unpacked output buffer (16-bit per sample)
 * @param length  Number of samples to unpack (must be multiple of 8)
 *
 * @note Internal function - not exported from DLL
 */
void hydrasdr_unpack_samples(
	const uint16_t* HYDRASDR_RESTRICT input,
	uint16_t* HYDRASDR_RESTRICT output,
	int length);

/**
 * @brief Unpack 12-bit samples (reference implementation for testing)
 *
 * Simple, unoptimized implementation used as reference for validation.
 *
 * @param input   Packed input buffer
 * @param output  Unpacked output buffer
 * @param length  Number of samples to unpack (must be multiple of 8)
 *
 * @note Internal function - not exported from DLL
 */
void hydrasdr_unpack_samples_reference(
	const uint16_t* input,
	uint16_t* output,
	int length);

#endif /* HYDRASDR_SHARED_H */
