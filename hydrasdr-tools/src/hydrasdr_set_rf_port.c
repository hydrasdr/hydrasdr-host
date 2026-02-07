/*
 * Copyright 2024-2026 Benjamin Vernoux <bvernoux@hydrasdr.com>
 *
 * This file is part of HydraSDR.
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

#if !defined(__STDC_VERSION__) || __STDC_VERSION__ < 202311L
#ifndef bool
typedef int bool;
#define true 1
#define false 0
#endif
#endif

#define RF_PORT_INVALID 255

bool serial_number = false;
uint64_t serial_number_val;
int rf_port = -1;  /* -1 means not set */

struct hydrasdr_device* device = NULL;

int parse_u8(char* const s, uint8_t* const value) {
	char* s_end = s;
	const long int long_value = strtol(s, &s_end, 10);
	if( (s != s_end) && (*s_end == 0) ) {
		if((long_value >=0 ) && (long_value < 256)) {
			*value = (uint8_t)long_value;
			return HYDRASDR_SUCCESS;
		} else {
			return HYDRASDR_ERROR_INVALID_PARAM;
		}
	} else {
		return HYDRASDR_ERROR_INVALID_PARAM;
	}
}

int parse_u64(char* s, uint64_t* const value) {
	uint_fast8_t base = 10;
	char* s_end;
	uint64_t u64_value;

	if( strlen(s) > 2 ) {
		if( s[0] == '0' ) {
			if( (s[1] == 'x') || (s[1] == 'X') ) {
				base = 16;
				s += 2;
			} else if( (s[1] == 'b') || (s[1] == 'B') ) {
				base = 2;
				s += 2;
			}
		}
	}

	s_end = s;
	u64_value = strtoull(s, &s_end, base);
	if( (s != s_end) && (*s_end == 0) ) {
		*value = u64_value;
		return HYDRASDR_SUCCESS;
	} else {
		return HYDRASDR_ERROR_INVALID_PARAM;
	}
}

static void usage(void)
{
	printf("Usage:\n");
	printf("\t-p <port>: set RF port (0-based index)\n");
	printf("\t[-s serial_number_64bits]: Open board with specified 64bits serial number.\n");
	printf("\nNote: Connect a device and run without -p to see available RF ports.\n");
}

int main(int argc, char** argv)
{
	int result;
	int opt;
	uint8_t port_val;
	uint32_t serial_number_msb_val;
	uint32_t serial_number_lsb_val;
	hydrasdr_lib_version_t lib_version;
	hydrasdr_device_info_t device_info;
	uint8_t i;

	/* Display library version and check compatibility */
	hydrasdr_lib_version(&lib_version);
	printf("HydraSDR RF Port Tool (libhydrasdr v%d.%d.%d)\n",
	       lib_version.major_version, lib_version.minor_version, lib_version.revision);
	{
		uint32_t runtime_ver = HYDRASDR_MAKE_VERSION(lib_version.major_version,
		                                              lib_version.minor_version,
		                                              lib_version.revision);
		uint32_t min_ver = HYDRASDR_MAKE_VERSION(1, 1, 0);
		if (runtime_ver < min_ver) {
			fprintf(stderr, "[WARN] Library version too old: need v1.1.0+\n");
		}
	}

	/* Parse command line arguments (port validation done after device open) */
	while ((opt = getopt(argc, argv, "s:p:h")) != EOF)
	{
		switch (opt)
		{
		case 's':
			serial_number = true;
			result = parse_u64(optarg, &serial_number_val);
			if (result != HYDRASDR_SUCCESS) {
				fprintf(stderr, "Invalid serial number: %s\n", optarg);
				usage();
				return EXIT_FAILURE;
			}
			serial_number_msb_val = (uint32_t)(serial_number_val >> 32);
			serial_number_lsb_val = (uint32_t)(serial_number_val & 0xFFFFFFFF);
			printf("Board serial number: 0x%08X%08X\n", serial_number_msb_val, serial_number_lsb_val);
			break;

		case 'p':
			result = parse_u8(optarg, &port_val);
			if (result != HYDRASDR_SUCCESS) {
				fprintf(stderr, "Invalid port number: %s\n", optarg);
				usage();
				return EXIT_FAILURE;
			}
			rf_port = port_val;
			break;

		case 'h':
			usage();
			return EXIT_SUCCESS;

		default:
			usage();
			return EXIT_FAILURE;
		}
	}

	/* Open device */
	if (serial_number == true) {
		result = hydrasdr_open_sn(&device, serial_number_val);
	} else {
		result = hydrasdr_open(&device);
	}
	if (result != HYDRASDR_SUCCESS) {
		fprintf(stderr, "hydrasdr_open() failed: %s (%d)\n",
		        hydrasdr_error_name(result), result);
		return EXIT_FAILURE;
	}

	/* Get device info */
	result = hydrasdr_get_device_info(device, &device_info);
	if (result != HYDRASDR_SUCCESS) {
		fprintf(stderr, "hydrasdr_get_device_info() failed: %s (%d)\n",
		        hydrasdr_error_name(result), result);
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}

	printf("Device: %s (FW: %s)\n", device_info.board_name, device_info.firmware_version);

	/* Check RF port selection capability */
	if (!(device_info.features & HYDRASDR_CAP_RF_PORT_SELECT)) {
		fprintf(stderr, "Error: RF port selection not supported on this device.\n");
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}

	/* Show available RF ports */
	printf("\nAvailable RF Ports (%u):\n", device_info.rf_port_count);
	for (i = 0; i < device_info.rf_port_count; i++) {
		const hydrasdr_rf_port_info_t *port = &device_info.rf_port_info[i];
		printf("  %u: %s", i, port->name);
		if (port->min_frequency > 0 || port->max_frequency > 0) {
			printf(" (%.1f - %.1f MHz)",
			       (double)port->min_frequency / 1e6,
			       (double)port->max_frequency / 1e6);
		}
		if (port->has_bias_tee) {
			printf(" [Bias-T]");
		}
		printf("\n");
	}

	/* If no port specified, just show info and exit */
	if (rf_port < 0) {
		printf("\nNo port specified. Use -p <port> to set RF port.\n");
		hydrasdr_close(device);
		return EXIT_SUCCESS;
	}

	/* Validate port against actual device port count */
	if ((uint8_t)rf_port >= device_info.rf_port_count) {
		fprintf(stderr, "\nError: Port %d out of range (0-%u)\n",
		        rf_port, device_info.rf_port_count - 1);
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}

	/* Set RF port */
	printf("\nSetting RF port to %d (%s)...\n", rf_port,
	       device_info.rf_port_info[rf_port].name);
	result = hydrasdr_set_rf_port(device, (hydrasdr_rf_port_t)rf_port);
	if (result != HYDRASDR_SUCCESS) {
		fprintf(stderr, "hydrasdr_set_rf_port() failed: %s (%d)\n",
		        hydrasdr_error_name(result), result);
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}
	printf("RF port set successfully.\n");

	/* Cleanup */
	result = hydrasdr_close(device);
	if (result != HYDRASDR_SUCCESS) {
		fprintf(stderr, "hydrasdr_close() failed: %s (%d)\n",
		        hydrasdr_error_name(result), result);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
