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

#define HYDRASDR_MAX_DEVICE (32)

struct hydrasdr_device* devices[HYDRASDR_MAX_DEVICE+1] = { NULL };
hydrasdr_device_info_t device_infos[HYDRASDR_MAX_DEVICE+1];

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
	printf("\tIf no parameter are used open and reset all boards found.\n");
	printf("\t[-s serial_number_64bits]: Open and reset board with specified 64bits serial number.\n");
}

bool serial_number = false;
uint64_t serial_number_val;

int main(int argc, char** argv)
{
	int i;
	int result;
	int opt;
	uint32_t serial_number_msb_val;
	uint32_t serial_number_lsb_val;
	hydrasdr_lib_version_t lib_version;

	/* Display library version and check compatibility */
	hydrasdr_lib_version(&lib_version);
	printf("HydraSDR Reset Tool (libhydrasdr v%d.%d.%d)\n",
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

	while( (opt = getopt(argc, argv, "s:")) != EOF )
	{
		result = HYDRASDR_SUCCESS;
		switch( opt )
		{
		case 's':
			serial_number = true;
			result = parse_u64(optarg, &serial_number_val);
			serial_number_msb_val = (uint32_t)(serial_number_val >> 32);
			serial_number_lsb_val = (uint32_t)(serial_number_val & 0xFFFFFFFF);
			printf("Board serial number to open: 0x%08X%08X\n", serial_number_msb_val, serial_number_lsb_val);
			break;

		default:
			printf("unknown argument '-%c %s'\n", opt, optarg);
			usage();
			return EXIT_FAILURE;
		}

		if( result != HYDRASDR_SUCCESS ) {
			printf("argument error: '-%c %s' %s (%d)\n", opt, optarg, hydrasdr_error_name(result), result);
			usage();
			return EXIT_FAILURE;
		}
	} 

	for (i = 0; i < HYDRASDR_MAX_DEVICE; i++)
	{
		if(serial_number == true)
		{
			result = hydrasdr_open_sn(&devices[i], serial_number_val);
		}else
		{
			result = hydrasdr_open(&devices[i]);
		}
		if (result != HYDRASDR_SUCCESS)
		{
			if(i == 0)
			{
				fprintf(stderr, "hydrasdr_open() board %d failed: %s (%d)\n",
						i+1, hydrasdr_error_name(result), result);
			}
			break;
		}
	}

	for(i = 0; i < HYDRASDR_MAX_DEVICE; i++)
	{
		if(devices[i] != NULL)
		{
			hydrasdr_device_info_t *info = &device_infos[i];

			printf("\n=== Board %d ===\n", i + 1);
			fflush(stdout);

			/* Get device info */
			result = hydrasdr_get_device_info(devices[i], info);
			if (result != HYDRASDR_SUCCESS) {
				fprintf(stderr, "hydrasdr_get_device_info() failed: %s (%d)\n",
				        hydrasdr_error_name(result), result);
				hydrasdr_close(devices[i]);
				continue;
			}

			printf("Device: %s (FW: %s)\n", info->board_name, info->firmware_version);
			printf("Executing reset...\n");
			fflush(stdout);

			result = hydrasdr_reset(devices[i]);
			if (result != HYDRASDR_SUCCESS) {
				fprintf(stderr, "hydrasdr_reset() failed: %s (%d)\n",
					hydrasdr_error_name(result), result);
				hydrasdr_close(devices[i]);
				continue;
			}

			printf("Reset successful, closing board %d\n", i+1);
			result = hydrasdr_close(devices[i]);
			if (result != HYDRASDR_SUCCESS) {
				fprintf(stderr, "hydrasdr_close() board %d failed: %s (%d)\n",
						i+1, hydrasdr_error_name(result), result);
				continue;
			}
		}
	}

	return EXIT_SUCCESS;
}
