/*
 * HydraSDR USB Commands
 *
 * Copyright (C) 2013-2026, Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __HYDRASDR_COMMANDS_H__
#define __HYDRASDR_COMMANDS_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
	HYDRASDR_RECEIVER_MODE_OFF = 0,
	HYDRASDR_RECEIVER_MODE_RX = 1
} hydrasdr_receiver_mode_t;

/* Commands (usb vendor request) shared between Firmware and Host. */
typedef enum
{
	HYDRASDR_RESET = 0,
	HYDRASDR_RECEIVER_MODE = 1,
	HYDRASDR_CLOCKGEN_WRITE = 2,
	HYDRASDR_CLOCKGEN_READ = 3,
	HYDRASDR_RF_FRONTEND_WRITE = 4,
	HYDRASDR_RF_FRONTEND_READ = 5,
	HYDRASDR_SPIFLASH_ERASE = 6,
	HYDRASDR_SPIFLASH_WRITE = 7,
	HYDRASDR_SPIFLASH_READ = 8,
	HYDRASDR_BOARD_ID_READ = 9,
	HYDRASDR_VERSION_STRING_READ = 10,
	HYDRASDR_BOARD_PARTID_SERIALNO_READ = 11,
	HYDRASDR_SET_SAMPLERATE = 12,
	HYDRASDR_SET_FREQ = 13,
	HYDRASDR_SET_LNA_GAIN = 14,
	HYDRASDR_SET_MIXER_GAIN = 15,
	HYDRASDR_SET_VGA_GAIN = 16,
	HYDRASDR_SET_LNA_AGC = 17,
	HYDRASDR_SET_MIXER_AGC = 18,
	HYDRASDR_MS_VENDOR_CMD = 19,
	HYDRASDR_SET_RF_BIAS_CMD = 20,
	HYDRASDR_GPIO_WRITE = 21,
	HYDRASDR_GPIO_READ = 22,
	HYDRASDR_GPIODIR_WRITE = 23,
	HYDRASDR_GPIODIR_READ = 24,
	HYDRASDR_GET_SAMPLERATES = 25,
	HYDRASDR_SET_PACKING = 26,
	HYDRASDR_SPIFLASH_ERASE_SECTOR = 27,
	HYDRASDR_SET_RF_PORT = 28,
	/* New commands since v1.1.0 */
	HYDRASDR_GET_CAPABILITIES = 29,
	HYDRASDR_SET_BANDWIDTH = 30,
	HYDRASDR_GET_BANDWIDTHS = 31,
	HYDRASDR_GET_TEMPERATURE = 32,
	HYDRASDR_SET_GAIN = 33, /**< Unified gain set (if HYDRASDR_CAP_EXTENDED_GAIN) */
	HYDRASDR_VENDOR_REQUEST_COUNT /* Sentinel - total number of vendor requests */
} hydrasdr_vendor_request;

typedef enum
{
	HYDRASDR_GPIO_PORT0 = 0,
	HYDRASDR_GPIO_PORT1 = 1,
	HYDRASDR_GPIO_PORT2 = 2,
	HYDRASDR_GPIO_PORT3 = 3,
	HYDRASDR_GPIO_PORT4 = 4,
	HYDRASDR_GPIO_PORT5 = 5,
	HYDRASDR_GPIO_PORT6 = 6,
	HYDRASDR_GPIO_PORT7 = 7
} hydrasdr_gpio_port_t;

typedef enum
{
	HYDRASDR_GPIO_PIN0 = 0,
	HYDRASDR_GPIO_PIN1 = 1,
	HYDRASDR_GPIO_PIN2 = 2,
	HYDRASDR_GPIO_PIN3 = 3,
	HYDRASDR_GPIO_PIN4 = 4,
	HYDRASDR_GPIO_PIN5 = 5,
	HYDRASDR_GPIO_PIN6 = 6,
	HYDRASDR_GPIO_PIN7 = 7,
	HYDRASDR_GPIO_PIN8 = 8,
	HYDRASDR_GPIO_PIN9 = 9,
	HYDRASDR_GPIO_PIN10 = 10,
	HYDRASDR_GPIO_PIN11 = 11,
	HYDRASDR_GPIO_PIN12 = 12,
	HYDRASDR_GPIO_PIN13 = 13,
	HYDRASDR_GPIO_PIN14 = 14,
	HYDRASDR_GPIO_PIN15 = 15,
	HYDRASDR_GPIO_PIN16 = 16,
	HYDRASDR_GPIO_PIN17 = 17,
	HYDRASDR_GPIO_PIN18 = 18,
	HYDRASDR_GPIO_PIN19 = 19,
	HYDRASDR_GPIO_PIN20 = 20,
	HYDRASDR_GPIO_PIN21 = 21,
	HYDRASDR_GPIO_PIN22 = 22,
	HYDRASDR_GPIO_PIN23 = 23,
	HYDRASDR_GPIO_PIN24 = 24,
	HYDRASDR_GPIO_PIN25 = 25,
	HYDRASDR_GPIO_PIN26 = 26,
	HYDRASDR_GPIO_PIN27 = 27,
	HYDRASDR_GPIO_PIN28 = 28,
	HYDRASDR_GPIO_PIN29 = 29,
	HYDRASDR_GPIO_PIN30 = 30,
	HYDRASDR_GPIO_PIN31 = 31
} hydrasdr_gpio_pin_t;

/**
 * @brief RF port identifiers (hardware-agnostic)
 *
 * Generic RF port indices. Use with hydrasdr_set_rf_port() to select
 * the active RF input.
 *
 * Port characteristics vary by device - use hydrasdr_get_device_info() to
 * query rf_port_count and rf_port_info[] for hardware-specific details:
 *   - rf_port_info[port].name: Human-readable port name
 *   - rf_port_info[port].min_frequency / max_frequency: Recommended range
 *   - rf_port_info[port].has_bias_tee: Bias tee availability
 *
 * @see hydrasdr_set_rf_port()
 * @see hydrasdr_rf_port_info_t
 * @see hydrasdr_device_info_t
 */
typedef enum
{
	HYDRASDR_RF_PORT_RX0 = 0, /**< RF Port 0 (see rf_port_info[0] for details) */
	HYDRASDR_RF_PORT_RX1 = 1, /**< RF Port 1 (see rf_port_info[1] for details) */
	HYDRASDR_RF_PORT_RX2 = 2, /**< RF Port 2 (see rf_port_info[2] for details) */
	HYDRASDR_RF_PORT_MAX = 31 /**< Maximum supported RF port index */
} hydrasdr_rf_port_t;

/**
 * @brief Generate bitmask for N consecutive RF ports
 *
 * Creates a bitmask with bits 0 to (n-1) set, representing N available ports.
 * Use with rf_ports field in hydrasdr_device_info_t.
 *
 * @param n Number of RF ports (1 to 32)
 * @return Bitmask with n consecutive bits set starting from bit 0
 *
 * Example:
 *   HYDRASDR_RF_PORTS_MASK(3) = 0x07 (bits 0,1,2 set)
 *   HYDRASDR_RF_PORTS_MASK(4) = 0x0F (bits 0,1,2,3 set)
 */
#define HYDRASDR_RF_PORTS_MASK(n)  ((1U << (n)) - 1)

/* ========================================================================
 * Capability Discovery API Types
 * ======================================================================== */

/**
 * @brief Feature capability flags
 *
 * Bitmask values representing device capabilities. These flags indicate which
 * features and functions are supported by the connected device.
 *
 * Use with hydrasdr_get_device_info() to query device capabilities at runtime.
 * Check capabilities before calling related functions to avoid errors.
 *
 * @note Capabilities vary by device model
 * @note Always check capabilities rather than assuming based on device type
 * @note Future hardware may add new capability flags
 *
 * @since v1.1.0
 */
typedef enum
{
	HYDRASDR_CAP_LNA_GAIN                 = (1 << 0),  /**< Manual LNA gain control via hydrasdr_set_gain(HYDRASDR_GAIN_TYPE_LNA) */
	HYDRASDR_CAP_RF_GAIN                  = (1 << 1),  /**< Manual RF gain control via hydrasdr_set_gain(HYDRASDR_GAIN_TYPE_RF) */
	HYDRASDR_CAP_MIXER_GAIN               = (1 << 2),  /**< Manual Mixer gain control via hydrasdr_set_gain(HYDRASDR_GAIN_TYPE_MIXER) */
	HYDRASDR_CAP_FILTER_GAIN              = (1 << 3),  /**< Manual Filter gain control via hydrasdr_set_gain(HYDRASDR_GAIN_TYPE_FILTER) */
	HYDRASDR_CAP_VGA_GAIN                 = (1 << 4),  /**< Manual VGA gain control via hydrasdr_set_gain(HYDRASDR_GAIN_TYPE_VGA) */
	HYDRASDR_CAP_LNA_AGC                  = (1 << 5),  /**< LNA automatic gain control via hydrasdr_set_gain(HYDRASDR_GAIN_TYPE_LNA_AGC) */
	HYDRASDR_CAP_RF_AGC                   = (1 << 6),  /**< RF automatic gain control via hydrasdr_set_gain(HYDRASDR_GAIN_TYPE_RF_AGC) */
	HYDRASDR_CAP_MIXER_AGC                = (1 << 7),  /**< Mixer automatic gain control via hydrasdr_set_gain(HYDRASDR_GAIN_TYPE_MIXER_AGC) */
	HYDRASDR_CAP_FILTER_AGC               = (1 << 8),  /**< Filter automatic gain control via hydrasdr_set_gain(HYDRASDR_GAIN_TYPE_FILTER_AGC) */
	HYDRASDR_CAP_LINEARITY_GAIN           = (1 << 9),  /**< Global Linearity gain mode via hydrasdr_set_gain(HYDRASDR_GAIN_TYPE_LINEARITY) */
	HYDRASDR_CAP_SENSITIVITY_GAIN         = (1 << 10), /**< Global Sensitivity gain mode via hydrasdr_set_gain(HYDRASDR_GAIN_TYPE_SENSITIVITY) */
	HYDRASDR_CAP_BIAS_TEE                 = (1 << 11), /**< Bias tee control via hydrasdr_set_rf_bias() */
	HYDRASDR_CAP_PACKING                  = (1 << 12), /**< Sample packing support via hydrasdr_set_packing() */
	HYDRASDR_CAP_RF_PORT_SELECT           = (1 << 13), /**< RF port selection via hydrasdr_set_rf_port() */
	HYDRASDR_CAP_GPIO                     = (1 << 14), /**< GPIO control via hydrasdr_gpio_*() functions */
	HYDRASDR_CAP_SPIFLASH                 = (1 << 15), /**< SPI flash access via hydrasdr_spiflash_*() functions */
	HYDRASDR_CAP_CLOCKGEN                 = (1 << 16), /**< Clock generator access via hydrasdr_clockgen_*() */
	HYDRASDR_CAP_RF_FRONTEND              = (1 << 17), /**< RF frontend register access via hydrasdr_rf_frontend_*() */
	HYDRASDR_CAP_BANDWIDTH                = (1 << 18), /**< Configurable Bandwidth via hydrasdr_get_bandwidths()/hydrasdr_set_bandwidth() */
	HYDRASDR_CAP_TEMPERATURE_SENSOR       = (1 << 19), /**< Temperature sensor via hydrasdr_get_temperature() */
	HYDRASDR_CAP_RX                       = (1 << 20), /**< Device supports receive (RX) operation */
	HYDRASDR_CAP_EXTENDED_SAMPLERATES     = (1 << 21), /**< Extended sample rate info with ADC config per rate */
	HYDRASDR_CAP_EXTENDED_GAIN            = (1 << 22), /**< Unified gain API via HYDRASDR_SET_GAIN */
	/* Bits 23-31 reserved for future capabilities */
} hydrasdr_capability_t;

/**
 * @brief Gain type identifiers for unified gain API
 *
 * Used with HYDRASDR_SET_GAIN command when
 * HYDRASDR_CAP_EXTENDED_GAIN capability is present.
 *
 * Type values 0-4 are manual gain controls (value = gain level).
 * Type values 5-6 are composite gain modes (value = preset index).
 * Type values 7-10 are AGC enables (value = 0=off, 1=on).
 *
 * @since v1.1.0
 */
typedef enum {
	HYDRASDR_GAIN_TYPE_LNA         = 0,  /**< LNA manual gain (maps to HYDRASDR_CAP_LNA_GAIN) */
	HYDRASDR_GAIN_TYPE_RF          = 1,  /**< RF manual gain (maps to HYDRASDR_CAP_RF_GAIN) */
	HYDRASDR_GAIN_TYPE_MIXER       = 2,  /**< Mixer manual gain (maps to HYDRASDR_CAP_MIXER_GAIN) */
	HYDRASDR_GAIN_TYPE_FILTER      = 3,  /**< Filter manual gain (maps to HYDRASDR_CAP_FILTER_GAIN) */
	HYDRASDR_GAIN_TYPE_VGA         = 4,  /**< VGA manual gain (maps to HYDRASDR_CAP_VGA_GAIN) */
	HYDRASDR_GAIN_TYPE_LINEARITY   = 5,  /**< Linearity preset (maps to HYDRASDR_CAP_LINEARITY_GAIN) */
	HYDRASDR_GAIN_TYPE_SENSITIVITY = 6,  /**< Sensitivity preset (maps to HYDRASDR_CAP_SENSITIVITY_GAIN) */
	HYDRASDR_GAIN_TYPE_LNA_AGC     = 7,  /**< LNA AGC enable (maps to HYDRASDR_CAP_LNA_AGC) */
	HYDRASDR_GAIN_TYPE_RF_AGC      = 8,  /**< RF AGC enable (maps to HYDRASDR_CAP_RF_AGC) */
	HYDRASDR_GAIN_TYPE_MIXER_AGC   = 9,  /**< Mixer AGC enable (maps to HYDRASDR_CAP_MIXER_AGC) */
	HYDRASDR_GAIN_TYPE_FILTER_AGC  = 10, /**< Filter AGC enable (maps to HYDRASDR_CAP_FILTER_AGC) */
	HYDRASDR_GAIN_TYPE_COUNT       = 11  /**< Total number of gain types */
} hydrasdr_gain_type_t;

/**
 * @brief Extended gain information structure
 *
 * Used by hydrasdr_get_gain() and hydrasdr_get_all_gains() to return
 * cached gain values. Gain values are cached on the host when
 * hydrasdr_set_gain() is called.
 *
 * Protocol:
 *   SET_GAIN wValue=type, wIndex=value -> sets gain, returns status
 *
 * @since v1.1.0
 */
typedef struct {
	uint8_t type;          /**< Gain type (hydrasdr_gain_type_t) */
	uint8_t value;         /**< Current gain value */
	uint8_t min_value;     /**< Minimum valid value */
	uint8_t max_value;     /**< Maximum valid value */
	uint8_t step_value;    /**< Step size between valid values (0 = any) */
	uint8_t default_value; /**< Default/recommended value */
	uint8_t flags;         /**< Flags: bit0=AGC type, bit1-7=reserved */
	uint8_t reserved;      /**< Reserved for alignment */
} hydrasdr_gain_info_t;

/** Gain info flags */
#define HYDRASDR_GAIN_FLAG_IS_AGC    (1 << 0)  /**< This is an AGC enable (value: 0=off, 1=on) */
#define HYDRASDR_GAIN_FLAG_IS_PRESET (1 << 1)  /**< This is a composite preset (LINEARITY/SENSITIVITY) */

/**
 * @brief Extended sample rate information
 *
 * When HYDRASDR_CAP_EXTENDED_SAMPLERATES is set, firmware returns this
 * extended format with ADC configuration per sample rate via
 * HYDRASDR_GET_SAMPLERATES command with wValue=1.
 *
 * This enables hardware-agnostic ADC mode detection:
 *
 * @since v1.1.0
 */
typedef struct {
	uint32_t rate_hz;       /**< Sample rate in Hz */
	uint8_t  adc_bits;      /**< ADC bit depth: 8, 10, 12, 14, or 16 (use IQCONV_ADC_* constants) */
	uint8_t  data_format;   /**< Data format: 0=RAW_ADC (needs DDC), 1=IQ_DIRECT (skip DDC) */
	uint16_t reserved;      /**< Reserved for future use (alignment) */
} hydrasdr_samplerate_info_t;

#ifdef __cplusplus
} // __cplusplus defined.
#endif

#endif//__HYDRASDR_COMMANDS_H__
