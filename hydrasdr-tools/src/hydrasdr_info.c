/*
 * Copyright 2012 Jared Boone <jared@sharebrained.com>
 * Copyright 2013 Michael Ossmann <mike@ossmann.com>
 * Copyright 2013-2026 Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * This file is part of HydraSDR (based on HackRF project).
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <hydrasdr.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>

#if defined(_WIN32)
#include <windows.h>
#endif

#if !defined(__STDC_VERSION__) || __STDC_VERSION__ < 202311L
#ifndef bool
typedef int bool;
#define true 1
#define false 0
#endif
#endif

#define HYDRASDR_MAX_DEVICE (32)

static uint64_t serials[HYDRASDR_MAX_DEVICE+1];

/* Helper function prototypes */
static void print_capability(uint32_t features, uint32_t cap, const char* name);
static void print_gain_range(const char* name, const hydrasdr_gain_range_t* range, int supported);
static void print_device_capabilities(struct hydrasdr_device* device, int device_num);
static const char* component_type_name(uint8_t type);

int parse_u64(char* s, uint64_t* const value)
{
	uint_fast8_t base = 10;
	char* s_end;
	uint64_t u64_value;

	if (strlen(s) > 2) {
		if (s[0] == '0') {
			if ((s[1] == 'x') || (s[1] == 'X')) {
				base = 16;
				s += 2;
			} else if ((s[1] == 'b') || (s[1] == 'B')) {
				base = 2;
				s += 2;
			}
		}
	}

	s_end = s;
	u64_value = strtoull(s, &s_end, base);
	if ((s != s_end) && (*s_end == 0)) {
		*value = u64_value;
		return HYDRASDR_SUCCESS;
	} else {
		return HYDRASDR_ERROR_INVALID_PARAM;
	}
}

static void usage(void)
{
	printf("Usage:\n");
	printf("\t[-s serial_number_64bits]: Open board with specified 64bits serial number.\n");
}

/* Helper function to print a single capability */
static void print_capability(uint32_t features, uint32_t cap, const char* name)
{
	if (features & cap)
		printf("%-30s\n", name);
}

/* Helper function to print gain range information */
static void print_gain_range(const char* name, const hydrasdr_gain_range_t* range, int supported)
{
	if (supported) {
		printf("%-20s: %d - %d (step %d, default %d)\n",
		       name,
		       (int)range->min_value,
		       (int)range->max_value,
		       (int)range->step_value,
		       (int)range->default_value);
	} else {
		printf("%-20s: Not supported\n", name);
	}
}

/* Helper function to convert component type to string */
static const char* component_type_name(uint8_t type)
{
	switch (type) {
	case HYDRASDR_COMP_RF_FRONTEND: return "RF Frontend";
	case HYDRASDR_COMP_CLOCKGEN:    return "Clock Generator";
	case HYDRASDR_COMP_LNA:         return "LNA";
	case HYDRASDR_COMP_PA:          return "Power Amplifier";
	case HYDRASDR_COMP_ADC:         return "ADC";
	case HYDRASDR_COMP_DAC:         return "DAC";
	case HYDRASDR_COMP_FILTER:      return "Filter";
	case HYDRASDR_COMP_MIXER:       return "Mixer";
	default:                        return "Unknown";
	}
}

/* Print comprehensive device capabilities */
static void print_device_capabilities(struct hydrasdr_device* device, int device_num)
{
	hydrasdr_device_info_t info;
	hydrasdr_temperature_t temp;
	uint32_t *samplerates;
	uint32_t *bandwidths;
	uint32_t count;
	uint32_t nb_rf_ports;
	uint32_t i;
	int result;

	printf("============================================\n");
	printf(" HydraSDR Board %d Information\n", device_num);

	/* === Get Device Info (single API call for all device information) === */
	result = hydrasdr_get_device_info(device, &info);
	if (result != HYDRASDR_SUCCESS) {
		fprintf(stderr, "hydrasdr_get_device_info() failed: %s (%d)\n",
			hydrasdr_error_name(result), result);
		return;
	}

	printf("\n=== Device Identification ===\n");
	printf("Board ID: %d\n", info.board_id);
	printf("Board Name: %s\n", info.board_name);
	printf("Firmware Version: %s\n", info.firmware_version);
	printf("Part ID Number: 0x%08X 0x%08X\n",
		info.part_serial.part_id[0],
		info.part_serial.part_id[1]);
	printf("Serial Number: 0x%08X%08X\n",
		info.part_serial.serial_no[2],
		info.part_serial.serial_no[3]);
	printf("Struct version: %d.%d.%d\n",
			info.struct_version_major,
			info.struct_version_minor,
			info.struct_version_revision);

	/* Display capability flags */
	printf("\n=== Hardware Capabilities ===\n");
	print_capability(info.features, HYDRASDR_CAP_RX, "Receive (RX)");
	print_capability(info.features, HYDRASDR_CAP_LNA_GAIN, "LNA Manual Gain Control");
	print_capability(info.features, HYDRASDR_CAP_RF_GAIN, "RF Manual Gain Control");
	print_capability(info.features, HYDRASDR_CAP_MIXER_GAIN, "Mixer Manual Gain Control");
	print_capability(info.features, HYDRASDR_CAP_FILTER_GAIN, "Filter Manual Gain Control");
	print_capability(info.features, HYDRASDR_CAP_VGA_GAIN, "VGA Manual Gain Control");
	print_capability(info.features, HYDRASDR_CAP_LNA_AGC, "LNA Automatic Gain Control");
	print_capability(info.features, HYDRASDR_CAP_RF_AGC, "RF Automatic Gain Control");
	print_capability(info.features, HYDRASDR_CAP_MIXER_AGC, "Mixer Automatic Gain Control");
	print_capability(info.features, HYDRASDR_CAP_FILTER_AGC, "Filter Automatic Gain Control");
	print_capability(info.features, HYDRASDR_CAP_LINEARITY_GAIN, "Linearity Gain Mode");
	print_capability(info.features, HYDRASDR_CAP_SENSITIVITY_GAIN, "Sensitivity Gain Mode");
	print_capability(info.features, HYDRASDR_CAP_BIAS_TEE, "Bias Tee");
	print_capability(info.features, HYDRASDR_CAP_PACKING, "Sample Packing");
	print_capability(info.features, HYDRASDR_CAP_RF_PORT_SELECT, "RF Port Selection");
	print_capability(info.features, HYDRASDR_CAP_GPIO, "GPIO Control");
	print_capability(info.features, HYDRASDR_CAP_SPIFLASH, "SPI Flash Access");
	print_capability(info.features, HYDRASDR_CAP_CLOCKGEN, "Clock Generator Access");
	print_capability(info.features, HYDRASDR_CAP_RF_FRONTEND, "RF Frontend Register Access");
	print_capability(info.features, HYDRASDR_CAP_BANDWIDTH, "Bandwidth Selection");
	print_capability(info.features, HYDRASDR_CAP_TEMPERATURE_SENSOR, "Temperature Sensor");

	/* Display gain ranges */
	printf("\n=== Gain Control Ranges ===\n");
	print_gain_range("LNA Gain", &info.lna_gain,
	                 info.features & HYDRASDR_CAP_LNA_GAIN);
	print_gain_range("RF Gain", &info.rf_gain,
	                 info.features & HYDRASDR_CAP_RF_GAIN);
	print_gain_range("Mixer Gain", &info.mixer_gain,
	                 info.features & HYDRASDR_CAP_MIXER_GAIN);
	print_gain_range("Filter Gain", &info.filter_gain,
	                 info.features & HYDRASDR_CAP_FILTER_GAIN);
	print_gain_range("VGA Gain", &info.vga_gain,
	                 info.features & HYDRASDR_CAP_VGA_GAIN);
	print_gain_range("Linearity Gain", &info.linearity_gain,
	                 info.features & HYDRASDR_CAP_LINEARITY_GAIN);
	print_gain_range("Sensitivity Gain", &info.sensitivity_gain,
	                 info.features & HYDRASDR_CAP_SENSITIVITY_GAIN);

	/* RF Frontend Components */
	if (info.component_count > 0) {
		printf("\n=== RF Frontend Components ===\n");
		for (i = 0; i < info.component_count; i++) {
			printf("%s: %s",
			       component_type_name(info.components[i].type),
			       info.components[i].name);
			if (info.components[i].register_count > 0) {
				printf(" (%u registers)", info.components[i].register_count);
			}
			printf("\n");
		}
	}
			   
	printf("\n=== Frequency range ===\n");
	printf("%.1f MHz - %.1f MHz\n",
	       (double)info.min_frequency / 1e6,
	       (double)info.max_frequency / 1e6);

	nb_rf_ports = info.rf_port_count;
	printf("\n=== RF ports ===\n");
	printf("Available RF Ports: %u\n", nb_rf_ports);
	for(i = 0; i < nb_rf_ports; i++) {
		if(info.rf_port_info[i].has_bias_tee == 1) {
			printf("RFPort%u: %s %.1f MHz - %.1f MHz, Bias tee %.2f V, max current: %.2f mA\n", i,
					info.rf_port_info[i].name,
					(double)info.rf_port_info[i].min_frequency / 1e6,
					(double)info.rf_port_info[i].max_frequency / 1e6,
					info.rf_port_info[i].bias_tee.voltage,
					info.rf_port_info[i].bias_tee.max_current_milliamp);
		} else {
			printf("RFPort%u: %s %.1f MHz - %.1f MHz\n", i,
					info.rf_port_info[i].name,
					(double)info.rf_port_info[i].min_frequency / 1e6,
					(double)info.rf_port_info[i].max_frequency / 1e6);
		}
	}

	if(info.features & HYDRASDR_CAP_GPIO) {
		printf("\n=== GPIO ===\n");
		printf("Nb GPIOs: %d\n", (int)info.gpio_count);
	}

	/* Display sample types */
	printf("\n=== Supported Sample Types ===\n");
	if (info.sample_types & (1 << HYDRASDR_SAMPLE_FLOAT32_IQ))
		printf("FLOAT32_IQ (2 x 32-bit float per sample)\n");
	if (info.sample_types & (1 << HYDRASDR_SAMPLE_FLOAT32_REAL))
		printf("FLOAT32_REAL (1 x 32-bit float per sample)\n");
	if (info.sample_types & (1 << HYDRASDR_SAMPLE_INT16_IQ))
		printf("INT16_IQ (2 x 16-bit int per sample)\n");
	if (info.sample_types & (1 << HYDRASDR_SAMPLE_INT16_REAL))
		printf("INT16_REAL (1 x 16-bit int per sample)\n");
	if (info.sample_types & (1 << HYDRASDR_SAMPLE_UINT16_REAL))
		printf("UINT16_REAL (1 x 16-bit unsigned int per sample)\n");
	if (info.sample_types & (1 << HYDRASDR_SAMPLE_RAW))
		printf("RAW (packed samples from device)\n");
	if (info.sample_types & (1 << HYDRASDR_SAMPLE_INT8_IQ))
		printf("INT8_IQ (2 x 8-bit signed int per sample)\n");
	if (info.sample_types & (1 << HYDRASDR_SAMPLE_UINT8_IQ))
		printf("UINT8_IQ (2 x 8-bit unsigned int per sample)\n");
	if (info.sample_types & (1 << HYDRASDR_SAMPLE_INT8_REAL))
		printf("INT8_REAL (1 x 8-bit signed int per sample)\n");
	if (info.sample_types & (1 << HYDRASDR_SAMPLE_UINT8_REAL))
		printf("UINT8_REAL (1 x 8-bit unsigned int per sample)\n");

	printf("\n=== Power and Thermal Information ===\n");
	printf("typical_power_mw: %.0f mW\n", info.typical_power_mw);
	printf("max_power_mw: %.0f mW\n", info.max_power_mw);
	printf("max_safe_temp_celsius: %.2f DegC (%.2f DegF)\n",
			info.max_safe_temp_celsius,
			info.max_safe_temp_fahrenheit);

	/* Display supported sample rates */
	printf("\n=== Supported Sample Rates ===\n");
	hydrasdr_get_samplerates(device, &count, 0);
	samplerates = (uint32_t *) malloc(count * sizeof(uint32_t));
	if (samplerates != NULL) {
		hydrasdr_get_samplerates(device, samplerates, count);
		for (i = 0; i < count; i++) {
			printf("%f MSPS\n", samplerates[i] * 0.000001f);
		}
		free(samplerates);
	}

	/* Display supported bandwidths */
	if (info.features & HYDRASDR_CAP_BANDWIDTH) {
		printf("\n=== Supported Bandwidths ===\n");
		hydrasdr_get_bandwidths(device, &count, 0);
		bandwidths = (uint32_t *) malloc(count * sizeof(uint32_t));
		if (bandwidths != NULL) {
			hydrasdr_get_bandwidths(device, bandwidths, count);
			for (i = 0; i < count; i++) {
				printf("%f MHz\n", bandwidths[i] * 0.000001f);
			}
			free(bandwidths);
		}
	}

	/* Display temperature if available */
	if (info.features & HYDRASDR_CAP_TEMPERATURE_SENSOR) {
		printf("\n=== Temperature Sensor ===\n");

		/* Get current temperature reading */
		result = hydrasdr_get_temperature(device, &temp);
		if (result == HYDRASDR_SUCCESS && temp.valid) {
			printf("Current Temperature:   %.2f DegC (%.2f DegF)\n",
			       temp.temperature_celsius,
			       temp.temperature_fahrenheit);

			/* Provide thermal status */
			if (temp.temperature_celsius < 60.0f) {
				printf("Thermal Status:        Normal\n");
			} else if (temp.temperature_celsius < 75.0f) {
				printf("Thermal Status:        Warm\n");
			} else {
				printf("Thermal Status:        HOT - Consider cooling\n");
			}
		} else {
			printf("Temperature:           Not available\n");
		}
	}
	printf("Closing board %d...\n", device_num);
	printf("============================================\n");
}

int main(int argc, char** argv)
{
	int i;
	int result;
	int opt;
	int device_count = 0;
	uint32_t serial_number_msb_val = 0;
	uint32_t serial_number_lsb_val = 0;
	hydrasdr_lib_version_t lib_version;
	uint64_t serial_number_val = 0;

#if defined(_WIN32)
    SetConsoleOutputCP(CP_UTF8);
#endif

	while ((opt = getopt(argc, argv, "s:")) != EOF) {
		result = HYDRASDR_SUCCESS;
		switch (opt) {
		case 's':
			result = parse_u64(optarg, &serial_number_val);
			serial_number_msb_val = (uint32_t)(serial_number_val >> 32);
			serial_number_lsb_val = (uint32_t)(serial_number_val & 0xFFFFFFFF);
			printf("Board serial number to open: 0x%08X%08X\n",
				serial_number_msb_val, serial_number_lsb_val);
			serials[device_count++] = serial_number_val;
			break;

		default:
			printf("unknown argument '-%c %s'\n", opt, optarg);
			usage();
			return EXIT_FAILURE;
		}

		if (result != HYDRASDR_SUCCESS) {
			printf("argument error: '-%c %s' %s (%d)\n",
				opt, optarg, hydrasdr_error_name(result), result);
			usage();
			return EXIT_FAILURE;
		}
	}

	printf("HydraSDR Device Information & Capabilities\n");
	hydrasdr_lib_version(&lib_version);
	printf("hydrasdr library version: %d.%d.%d\n",
		lib_version.major_version, lib_version.minor_version, lib_version.revision);

	/* Check library version compatibility using HYDRASDR_MAKE_VERSION macro */
	{
		uint32_t runtime_ver = HYDRASDR_MAKE_VERSION(lib_version.major_version,
		                                              lib_version.minor_version,
		                                              lib_version.revision);
		uint32_t min_ver = HYDRASDR_MAKE_VERSION(1, 1, 0);
		if (runtime_ver < min_ver) {
			fprintf(stderr, "[WARN] Library version too old: need v1.1.0+, got v%d.%d.%d\n",
			        lib_version.major_version, lib_version.minor_version, lib_version.revision);
		}
	}
	printf("\n");

	if (device_count == 0)
	{
		device_count = hydrasdr_list_devices(serials, HYDRASDR_MAX_DEVICE);
	}

	/* Enumerate and open devices */
	for (i = 0; i < device_count; i++) {
		struct hydrasdr_device *device;
		result = hydrasdr_open_sn(&device, serials[i]);
		if (result != HYDRASDR_SUCCESS) {
			if (i == 0) {
				fprintf(stderr, "hydrasdr_open() board %d failed: %s (%d)\n",
					i+1, hydrasdr_error_name(result), result);
			}
		}
		else {
			print_device_capabilities(device, i + 1);
			result = hydrasdr_close(device);
			if (result != HYDRASDR_SUCCESS) {
				fprintf(stderr, "hydrasdr_close() board %d failed: %s (%d)\n",
					i+1, hydrasdr_error_name(result), result);
			}
			printf("\n");
		}
	}

	printf("Total HydraSDR devices found: %d\n", device_count);

	return EXIT_SUCCESS;
}
