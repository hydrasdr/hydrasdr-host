/*
 * HydraSDR Internal API
 *
 * Copyright (C) 2013-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef __HYDRASDR_INTERNAL_H__
#define __HYDRASDR_INTERNAL_H__

#include "hydrasdr.h"
#include <libusb.h>
#include <pthread.h>
#include "iqconverter.h"
#include "iqconverter_float.h"
#include "iqconverter_int16.h"
#include "iqconverter_float_opt.h"
#include "iqconverter_int16_opt.h"

#if !defined(__STDC_VERSION__) || __STDC_VERSION__ < 202311L
#ifndef bool
typedef int bool;
#define true 1
#define false 0
#endif
#endif

#ifdef HYDRASDR_BIG_ENDIAN
#define TO_LE_32(x) __builtin_bswap32(x)
#define TO_LE_64(x) __builtin_bswap64(x)
#else
#define TO_LE_32(x) x
#define TO_LE_64(x) x
#endif

/* Temperature conversion macro (internal use only) */
#define CELSIUS_TO_FAHRENHEIT(c) (((c) * 1.8f) + 32.0f)

#define LIBUSB_CTRL_TIMEOUT_MS (500)
#define LIBUSB_CTRL_TIMEOUT_CHIPERASE_MS (32000) // W25Q80DV Chip Erase Time up to 8s or 64KB Erase Block(s)(16blocks of 64KB) 32s max

struct hydrasdr_device;

// The Hardware Abstraction Layer (HAL) "interface"
typedef struct {
	int (*init)(struct hydrasdr_device* dev);
	int (*exit)(struct hydrasdr_device* dev);
	int (*board_id_read)(struct hydrasdr_device* dev, uint8_t* value);
	int (*version_string_read)(struct hydrasdr_device* dev, char* version, uint8_t length);
	int (*board_partid_serialno_read)(struct hydrasdr_device* dev, hydrasdr_read_partid_serialno_t* data);
	int (*get_samplerates)(struct hydrasdr_device* dev, uint32_t* buffer, const uint32_t len);
	int (*set_samplerate)(struct hydrasdr_device* dev, uint32_t samplerate_hz);
	int (*get_bandwidths)(struct hydrasdr_device* dev, uint32_t* buffer, const uint32_t len);
	int (*set_bandwidth)(struct hydrasdr_device* dev, uint32_t bandwidth_hz);
	int (*set_freq)(struct hydrasdr_device* dev, const uint64_t freq_hz);
	int (*set_lna_gain)(struct hydrasdr_device* dev, uint8_t value);
	int (*set_rf_gain)(struct hydrasdr_device* dev, uint8_t value);
	int (*set_mixer_gain)(struct hydrasdr_device* dev, uint8_t value);
	int (*set_filter_gain)(struct hydrasdr_device* dev, uint8_t value);
	int (*set_vga_gain)(struct hydrasdr_device* dev, uint8_t value);
	int (*set_lna_agc)(struct hydrasdr_device* dev, uint8_t value);
	int (*set_rf_agc)(struct hydrasdr_device* dev, uint8_t value);
	int (*set_mixer_agc)(struct hydrasdr_device* dev, uint8_t value);
	int (*set_filter_agc)(struct hydrasdr_device* dev, uint8_t value);
	int (*set_linearity_gain)(struct hydrasdr_device* dev, uint8_t value);
	int (*set_sensitivity_gain)(struct hydrasdr_device* dev, uint8_t value);
	int (*set_rf_bias)(struct hydrasdr_device* dev, uint8_t value);
	int (*set_packing)(struct hydrasdr_device* dev, uint8_t value);
	int (*set_rf_port)(struct hydrasdr_device* dev, hydrasdr_rf_port_t port);
	int (*reset)(struct hydrasdr_device* dev);
	int (*get_temperature)(struct hydrasdr_device* dev, hydrasdr_temperature_t* temperature);
	int (*clockgen_write)(struct hydrasdr_device* dev, uint8_t reg, uint8_t val);
	int (*clockgen_read)(struct hydrasdr_device* dev, uint8_t reg, uint8_t* val);
	int (*rf_frontend_write)(struct hydrasdr_device* dev, uint16_t reg, uint32_t val);
	int (*rf_frontend_read)(struct hydrasdr_device* dev, uint16_t reg, uint32_t* val);
	int (*gpio_write)(struct hydrasdr_device* dev, hydrasdr_gpio_port_t port, hydrasdr_gpio_pin_t pin, uint8_t val);
	int (*gpio_read)(struct hydrasdr_device* dev, hydrasdr_gpio_port_t port, hydrasdr_gpio_pin_t pin, uint8_t* value);
	int (*gpiodir_write)(struct hydrasdr_device* dev, hydrasdr_gpio_port_t port, hydrasdr_gpio_pin_t pin, uint8_t val);
	int (*gpiodir_read)(struct hydrasdr_device* dev, hydrasdr_gpio_port_t port, hydrasdr_gpio_pin_t pin, uint8_t* val);
	int (*spiflash_erase)(struct hydrasdr_device* dev);
	int (*spiflash_write)(struct hydrasdr_device* dev, const uint32_t addr, const uint16_t len, unsigned char* const data);
	int (*spiflash_read)(struct hydrasdr_device* dev, const uint32_t addr, const uint16_t len, unsigned char* data);
	int (*spiflash_erase_sector)(struct hydrasdr_device* dev, const uint16_t sector_num);
	int (*start_rx)(struct hydrasdr_device* dev, hydrasdr_sample_block_cb_fn callback, void* rx_ctx);
	int (*stop_rx)(struct hydrasdr_device* dev);
	int (*set_sample_type)(struct hydrasdr_device* device, enum hydrasdr_sample_type sample_type);
	int (*set_conversion_algorithm)(struct hydrasdr_device* device, const char* algorithm);
	int (*set_conversion_filter_float32)(struct hydrasdr_device* device, const float* kernel, const uint32_t len);
	int (*set_conversion_filter_int16)(struct hydrasdr_device* device, const int16_t* kernel, const uint32_t len);
	int (*get_device_info)(struct hydrasdr_device* dev, hydrasdr_device_info_t* info);
} hydrasdr_hal_t;

// The full, internal device structure
struct hydrasdr_device {
	libusb_context* usb_context;
	libusb_device_handle* usb_handle;
	const hydrasdr_hal_t* hal;
	void* private_data; // Pointer to a struct with device-specific data (e.g., transfer pool)

	/* Generic Software State (Managed by hydrasdr.c) */
	enum hydrasdr_sample_type sample_type; // Caching sample type for get_samplerates() logic
	bool reset_command; /* HYDRASDR_RESET command executed ? */
};

/* Supported VID/PID combinations */
typedef struct {
	uint16_t vid;
	uint16_t pid;
	const char* description;
} hydrasdr_usb_device_id_t;

/*
 * Gain type enumeration is now defined in hydrasdr_commands.h as hydrasdr_gain_type_t.
 * Legacy aliases for backward compatibility with internal code:
 */
#define HYDRASDR_GAIN_LNA         HYDRASDR_GAIN_TYPE_LNA
#define HYDRASDR_GAIN_RF          HYDRASDR_GAIN_TYPE_RF
#define HYDRASDR_GAIN_MIXER       HYDRASDR_GAIN_TYPE_MIXER
#define HYDRASDR_GAIN_FILTER      HYDRASDR_GAIN_TYPE_FILTER
#define HYDRASDR_GAIN_VGA         HYDRASDR_GAIN_TYPE_VGA
#define HYDRASDR_GAIN_LINEARITY   HYDRASDR_GAIN_TYPE_LINEARITY
#define HYDRASDR_GAIN_SENSITIVITY HYDRASDR_GAIN_TYPE_SENSITIVITY

#endif // __HYDRASDR_INTERNAL_H__