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
#include <sys/types.h>
#include <time.h>

#if !defined(__STDC_VERSION__) || __STDC_VERSION__ < 202311L
#ifndef bool
typedef int bool;
#define true 1
#define false 0
#endif
#endif

#define HYDRASDR_FLASH_CALIB_OFFSET (0x20000) /* After 128KB (Reserved for Firmware + 64KB Spare) */
#define HYDRASDR_FLASH_CALIB_HEADER (0xCA1B0001)

typedef struct
{
	uint32_t header; /* Shall be equal to HYDRASDR_FLASH_CALIB_HEADER */
	uint32_t timestamp; /* Epoch Unix Time Stamp */
	int32_t correction_ppb;
} hydrasdr_calib_t;

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

static void usage()
{
	printf("Usage:\n");
	printf("\t-r: Read and display calibration data.\n");
	printf("\t-w <calibration in ppb>: Erase and Write calibration in ppb.\n");
	printf("\t[-s serial_number_64bits]: Open board with specified 64bits serial number.\n");
}

int main(int argc, char **argv)
{
	int opt;
	struct hydrasdr_device* device = NULL;
	int result;
	bool read = false;
	bool write = false;
	int32_t calibration_ppb = 0;
	hydrasdr_calib_t calib;
	hydrasdr_lib_version_t lib_version;
	hydrasdr_device_info_t device_info;
	bool serial_number = false;
	uint64_t serial_number_val = 0;

	/* Display library version and check compatibility */
	hydrasdr_lib_version(&lib_version);
	printf("HydraSDR Calibration Tool (libhydrasdr v%d.%d.%d)\n",
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

	while ((opt = getopt(argc, argv, "rw:s:")) != EOF)
	{
		switch (opt) {
		case 'r':
			read = true;
			break;

		case 'w':
			write = true;
			calibration_ppb = atoi(optarg);
			break;

		case 's':
			serial_number = true;
			result = parse_u64(optarg, &serial_number_val);
			if (result != HYDRASDR_SUCCESS) {
				fprintf(stderr, "Invalid serial number: %s\n", optarg);
				usage();
				return EXIT_FAILURE;
			}
			break;

		default:
			fprintf(stderr, "opt error: %d\n", opt);
			usage();
			return EXIT_FAILURE;
		}
	}

	if (write == read) {
		if (write == true) {
			fprintf(stderr, "Read and write options are mutually exclusive.\n");
		} else {
			fprintf(stderr, "Specify either read or write option.\n");
		}
		usage();
		return EXIT_FAILURE;
	}

	if (serial_number) {
		result = hydrasdr_open_sn(&device, serial_number_val);
	} else {
		result = hydrasdr_open(&device);
	}
	if (result != HYDRASDR_SUCCESS) {
		fprintf(stderr, "hydrasdr_open() failed: %s (%d)\n",
		        hydrasdr_error_name(result), result);
		return EXIT_FAILURE;
	}

	/* Get device info and check SPIFLASH capability */
	result = hydrasdr_get_device_info(device, &device_info);
	if (result != HYDRASDR_SUCCESS) {
		fprintf(stderr, "hydrasdr_get_device_info() failed: %s (%d)\n",
		        hydrasdr_error_name(result), result);
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}

	printf("Device: %s (FW: %s)\n", device_info.board_name, device_info.firmware_version);

	/* Check SPI flash capability */
	if (!(device_info.features & HYDRASDR_CAP_SPIFLASH)) {
		fprintf(stderr, "Error: SPI flash access not supported on this device.\n");
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}

	if (read) {
		printf("Reading %d bytes from 0x%06x.\n", (int)sizeof(calib), HYDRASDR_FLASH_CALIB_OFFSET);
		result = hydrasdr_spiflash_read(device, HYDRASDR_FLASH_CALIB_OFFSET, (int)sizeof(calib), (uint8_t *)&calib);
		if (result != HYDRASDR_SUCCESS) {
			fprintf(stderr, "hydrasdr_spiflash_read() failed: %s (%d)\n", hydrasdr_error_name(result), result);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
		time_t epoch_time = calib.timestamp;
		struct tm *local_time = localtime(&epoch_time);
		printf("Calibration timestamp: %04d/%02d/%02d %02d:%02d:%02d\nCalibration correction in ppb: %d\n", 
				local_time->tm_year + 1900,
				local_time->tm_mon + 1,
				local_time->tm_mday,
				local_time->tm_hour,
				local_time->tm_min,
				local_time->tm_sec,
				calib.correction_ppb);
	}
	if(write) {
		printf("Erasing sector 2 (calibration) in SPI flash.\n");
		result = hydrasdr_spiflash_erase_sector(device, 2);
		if (result != HYDRASDR_SUCCESS) {
			fprintf(stderr, "hydrasdr_spiflash_erase_sector() failed: %s (%d)\n",
			        hydrasdr_error_name(result), result);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}

		calib.header = HYDRASDR_FLASH_CALIB_HEADER;
		calib.timestamp = (uint32_t)time(NULL);
		calib.correction_ppb = calibration_ppb;

		printf("Writing calibration %d bytes at 0x%06x.\n", (int)sizeof(calib), HYDRASDR_FLASH_CALIB_OFFSET);
		time_t epoch_time = calib.timestamp;
		struct tm *local_time = localtime(&epoch_time);
		printf("Calibration timestamp: %04d/%02d/%02d %02d:%02d:%02d\nCalibration correction in ppb: %d\n",
				local_time->tm_year + 1900,
				local_time->tm_mon + 1,
				local_time->tm_mday,
				local_time->tm_hour,
				local_time->tm_min,
				local_time->tm_sec,
				calib.correction_ppb);
		result = hydrasdr_spiflash_write(device, HYDRASDR_FLASH_CALIB_OFFSET, sizeof(calib), (uint8_t *)&calib);
		if (result != HYDRASDR_SUCCESS) {
			fprintf(stderr, "hydrasdr_spiflash_write() failed: %s (%d)\n",
			        hydrasdr_error_name(result), result);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
	}

	result = hydrasdr_close(device);
	if (result != HYDRASDR_SUCCESS) {
		fprintf(stderr, "hydrasdr_close() failed: %s (%d)\n", hydrasdr_error_name(result), result);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

