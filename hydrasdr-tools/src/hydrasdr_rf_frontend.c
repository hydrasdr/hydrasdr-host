/*
 * Copyright 2013-2026 Benjamin Vernoux <bvernoux@hydrasdr.com>
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

#define REGISTER_NUM_MIN (0)
int register_num_max = 0;

static void usage() {
	printf("Usage:\n");
	printf("\t-n, --register <n>: set register <n>[%d,%d] for subsequent read/write operations\n", REGISTER_NUM_MIN, register_num_max);
	printf("\t-r, --read: read register specified by last -n argument, or all registers\n");
	printf("\t-w, --write <v>: write register specified by last -n argument with value <v>[0,255]\n");
	printf("\t[-s serial_number_64bits]: Open board with specified 64bits serial number.\n");
	printf("\nExamples:\n");
	printf("\t<command> -n 12 -r    # reads from register 12\n");
	printf("\t<command> -r          # reads all registers\n");
	printf("\t<command> -n 10 -w 22 # writes register 10 with 22 decimal\n");
}

static struct option long_options[] = {
	{ "register", required_argument, 0, 'n' },
	{ "write", required_argument, 0, 'w' },
	{ "read", no_argument, 0, 'r' },
	{ 0, 0, 0, 0 },
};

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

int dump_register(struct hydrasdr_device* device, const uint16_t register_number)
{
	uint32_t register_value = 0;
	int result = hydrasdr_rf_frontend_read(device, register_number, &register_value);

	if( result == HYDRASDR_SUCCESS ) {
		printf("[%3u] -> 0x%02X\n", register_number, (uint8_t)(register_value & 0xFF));
	} else {
		printf("hydrasdr_rf_frontend_read() failed: %s (%d)\n", hydrasdr_error_name(result), result);
	}

	return result;
}

int dump_registers(struct hydrasdr_device* device)
{
	int reg;
	int result = HYDRASDR_SUCCESS;

	/* Use register_num_max from device info instead of hardcoded value */
	for (reg = 0; reg < register_num_max; reg++)
	{
		result = dump_register(device, (uint16_t)reg);
		if (result != HYDRASDR_SUCCESS) {
			break;
		}
	}

	return result;
}

int write_register(struct hydrasdr_device* device, const uint16_t register_number, const uint32_t register_value)
{
	int result = HYDRASDR_SUCCESS;
	result = hydrasdr_rf_frontend_write(device, register_number, register_value);

	if( result == HYDRASDR_SUCCESS ) {
		printf("0x%02X -> [%3u]\n", (uint8_t)(register_value & 0xFF), register_number);
	} else {
		printf("hydrasdr_rf_frontend_write() failed: %s (%d)\n", hydrasdr_error_name(result), result);
	}

	return result;
}

#define REGISTER_INVALID 255
bool serial_number = false;
uint64_t serial_number_val;

int main(int argc, char** argv) {
	int opt;
	uint8_t register_number = REGISTER_INVALID;
	uint8_t register_value;
	struct hydrasdr_device* device = NULL;
	int option_index;
	uint32_t serial_number_msb_val;
	uint32_t serial_number_lsb_val;
	int result;
	hydrasdr_device_info_t info;
	hydrasdr_lib_version_t lib_version;

	/* Display library version and check compatibility */
	hydrasdr_lib_version(&lib_version);
	printf("HydraSDR RF Frontend Tool (libhydrasdr v%d.%d.%d)\n",
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

	option_index = 0;
	while( (opt = getopt_long(argc, argv, "n:rw:s:", long_options, &option_index)) != EOF )
	{
		switch( opt ) {

		case 's':
			serial_number = true;
			result = parse_u64(optarg, &serial_number_val);
			serial_number_msb_val = (uint32_t)(serial_number_val >> 32);
			serial_number_lsb_val = (uint32_t)(serial_number_val & 0xFFFFFFFF);
			printf("Board serial number to open: 0x%08X%08X\n", serial_number_msb_val, serial_number_lsb_val);
			break;
		}
	}

	if(serial_number == true)
	{
		result = hydrasdr_open_sn(&device, serial_number_val);
		if( result != HYDRASDR_SUCCESS ) {
			printf("hydrasdr_open_sn() failed: %s (%d)\n", hydrasdr_error_name(result), result);
			usage();
			return EXIT_FAILURE;
		}
	}else
	{
		result = hydrasdr_open(&device);
		if( result != HYDRASDR_SUCCESS ) {
			printf("hydrasdr_open() failed: %s (%d)\n", hydrasdr_error_name(result), result);
			usage();
			return EXIT_FAILURE;
		}
	}

	result = hydrasdr_get_device_info(device, &info);
	if (result != HYDRASDR_SUCCESS) {
		fprintf(stderr, "hydrasdr_get_device_info() failed: %s (%d)\n",
			hydrasdr_error_name(result), result);
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}

	printf("Device: %s (FW: %s)\n", info.board_name, info.firmware_version);

	/* Check RF frontend capability */
	if (!(info.features & HYDRASDR_CAP_RF_FRONTEND)) {
		fprintf(stderr, "Error: This device does not support RF frontend register access.\n");
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}

	/* Find RF frontend component and get register count */
	register_num_max = 32; /* Default fallback */
	for (uint8_t ci = 0; ci < info.component_count; ci++) {
		if (info.components[ci].type == HYDRASDR_COMP_RF_FRONTEND) {
			register_num_max = info.components[ci].register_count;
			printf("RF Frontend: %s (%d registers)\n",
			       info.components[ci].name, register_num_max);
			break;
		}
	}
	
	result = HYDRASDR_ERROR_OTHER;
	option_index = 0;
	optind = 0;
	while( (opt = getopt_long(argc, argv, "n:rw:", long_options, &option_index)) != EOF )
	{
		switch( opt ) {
		case 'n':
			result = parse_u8(optarg, &register_number);
			if((result != HYDRASDR_SUCCESS) || (register_number > register_num_max))
			{
				register_number = REGISTER_INVALID;
				printf("Error parameter -n shall be between %d and %d\n", REGISTER_NUM_MIN, register_num_max);
				result = HYDRASDR_ERROR_OTHER;
			}
			break;

		case 'w':
			result = parse_u8(optarg, &register_value);
			if( result == HYDRASDR_SUCCESS ) {
				result = write_register(device, register_number, register_value);
			}else
			{
				printf("Error parameter -w shall be between 0 and 255\n");
				result = HYDRASDR_ERROR_OTHER;
			}
			break;

		case 'r':
			if( register_number == REGISTER_INVALID ) {
				result = dump_registers(device);
			} else {
				result = dump_register(device, register_number);
			}
			break;
		}

		if( result != HYDRASDR_SUCCESS )
		{
			break;
		}
	}

	if( result != HYDRASDR_SUCCESS )
	{
		usage();
	}

	result = hydrasdr_close(device);
	if( result ) {
		printf("hydrasdr_close() failed: %s (%d)\n", hydrasdr_error_name(result), result);
		return EXIT_FAILURE;
	}

	return 0;
}

