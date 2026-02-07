/*
 * Copyright 2014-2026 Benjamin Vernoux <bvernoux@hydrasdr.com>
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
#include <inttypes.h>

int main(int argc, char** argv)
{
	hydrasdr_lib_version_t lib_version;
	uint64_t *serials = NULL;
	int result;
	int count;
	int i;
	uint32_t serial_number_msb_val;
	uint32_t serial_number_lsb_val;
	(void)argc;
	(void)argv;

	/* Display library version */
	hydrasdr_lib_version(&lib_version);
	printf("HydraSDR lib version: %d.%d.%d\n",
	       lib_version.major_version,
	       lib_version.minor_version,
	       lib_version.revision);

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

	/* Query number of devices */
	result = hydrasdr_list_devices(NULL, 0);

	if (result < 0) {
		fprintf(stderr, "Error: %s (%d)\n",
		        hydrasdr_error_name(result), result);
		return EXIT_FAILURE;
	}

	if (result == 0) {
		printf("No HydraSDR devices found.\n");
		return EXIT_SUCCESS;
	}

	count = result;
	printf("Found %d HydraSDR device%s:\n", count, (count > 1) ? "s" : "");

	/* Allocate memory for serial numbers */
	serials = (uint64_t *)malloc(count * sizeof(uint64_t));
	if (serials == NULL) {
		fprintf(stderr, "Error: Failed to allocate memory\n");
		return EXIT_FAILURE;
	}

	/* Retrieve serial numbers */
	result = hydrasdr_list_devices(serials, count);
	if (result < 0) {
		fprintf(stderr, "Error: %s (%d)\n",
		        hydrasdr_error_name(result), result);
		free(serials);
		return EXIT_FAILURE;
	}

	/* List each device with detailed information from hydrasdr_get_device_info() */
	for (i = 0; i < count; i++) {
		struct hydrasdr_device *dev = NULL;
		hydrasdr_device_info_t info;

		serial_number_msb_val = (uint32_t)(serials[i] >> 32);
		serial_number_lsb_val = (uint32_t)(serials[i] & 0xFFFFFFFF);

		/* Try to open device and get detailed info */
		result = hydrasdr_open_sn(&dev, serials[i]);
		if (result == HYDRASDR_SUCCESS && dev != NULL) {
			result = hydrasdr_get_device_info(dev, &info);
			if (result == HYDRASDR_SUCCESS) {
				printf("  #%d: %s (FW: %s) Serial: 0x%08X%08X\n",
				       i + 1,
				       info.board_name,
				       info.firmware_version,
				       serial_number_msb_val,
				       serial_number_lsb_val);
			} else {
				/* Fallback if device_info fails */
				printf("  #%d: <unknown> Serial: 0x%08X%08X\n",
				       i + 1,
				       serial_number_msb_val,
				       serial_number_lsb_val);
			}
			hydrasdr_close(dev);
		} else {
			/* Fallback if open fails */
			printf("  #%d: <unable to open> Serial: 0x%08X%08X\n",
			       i + 1,
			       serial_number_msb_val,
			       serial_number_lsb_val);
		}
	}

	/* Cleanup */
	free(serials);

	return EXIT_SUCCESS;
}
