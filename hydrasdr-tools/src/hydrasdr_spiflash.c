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
#include <sys/types.h>
#include <time.h>

#define HYDRASDR_SPIFLASH_VERSION "1.1.0 2025-2026"
#define MAX_FIRMWARE_BIN_SIZE (64*1024)

#if !defined(__STDC_VERSION__) || __STDC_VERSION__ < 202311L
#ifndef bool
typedef int bool;
#define true 1
#define false 0
#endif
#endif

#ifdef _WIN32
#include <windows.h>
	#ifdef _MSC_VER
		#ifdef _WIN64
		typedef int64_t ssize_t;
		#else
		typedef int32_t ssize_t;
		#endif
	#endif

#define sleep(a) Sleep( (a*1000) )
#endif

/* 8 Mbit flash */
#define MAX_LENGTH 0x100000

static struct option long_options[] = {
	{ "address", required_argument, 0, 'a' },
	{ "length", required_argument, 0, 'l' },
	{ "read", required_argument, 0, 'r' },
	{ "write", required_argument, 0, 'w' },
	{ "reset", no_argument, 0, 't' },
	{ "force", no_argument, 0, 'f' },
	{ 0, 0, 0, 0 },
};

int parse_u32(char* s, uint32_t* const value)
{
	char* s_end;
	uint_fast8_t base = 10;
	uint32_t u32_value;

	if (strlen(s) > 2) {
		if (s[0] == '0')  {
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
	u32_value = strtoul(s, &s_end, base);
	if ((s != s_end) && (*s_end == 0)) {
		*value = u32_value;
		return HYDRASDR_SUCCESS;
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

static void usage()
{
	printf("Usage:\n");
	printf("\t-a, --address <n>: starting address (default: 0)\n");
	printf("\t-l, --length <n>: number of bytes to read (default: 0)\n");
	printf("\t-r <filename>: Read data into file (SPIFI@0x80000000).\n");
	printf("\t-w <filename>: Write data from file.\n");
	printf("\t[-s serial_number_64bits]: Open board with specified 64bits serial number.\n");
	printf("\t[-f, --force]: Force write without board compatibility verification.\n");
}

bool serial_number = false;
uint64_t serial_number_val;

/**
 * Search for a string in binary data (case-sensitive)
 * Returns pointer to first occurrence, or NULL if not found
 */
const uint8_t* memmem_case_sensitive(const uint8_t* haystack, size_t haystack_len,
				     const uint8_t* needle, size_t needle_len)
{
	size_t i;
	
	if (needle_len == 0 || needle_len > haystack_len) {
		return NULL;
	}
	
	for (i = 0; i <= haystack_len - needle_len; i++) {
		if (memcmp(&haystack[i], needle, needle_len) == 0) {
			return &haystack[i];
		}
	}
	
	return NULL;
}

#if 0
typedef enum {
  USB_DESCRIPTOR_TYPE_DEVICE = 1,
  USB_DESCRIPTOR_TYPE_CONFIGURATION = 2,
  USB_DESCRIPTOR_TYPE_STRING = 3,
  USB_DESCRIPTOR_TYPE_INTERFACE = 4,
  USB_DESCRIPTOR_TYPE_ENDPOINT = 5,
  USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER = 6,
  USB_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION = 7,
  USB_DESCRIPTOR_TYPE_INTERFACE_POWER = 8,
} usb_descriptor_type_t;

uint8_t usb_descriptor_string_serial_number[] =
{
  58,                         // bLength
  USB_DESCRIPTOR_TYPE_STRING, // bDescriptorType
  'H', 0x00, /* 4  */
  'Y', 0x00, /* 6  */
  'D', 0x00, /* 8  */
  'R', 0x00, /* 10 */
  'A', 0x00, /* 12 */
  'S', 0x00, /* 14 */
  'D', 0x00, /* 16 */
  'R', 0x00, /* 18 */
  ' ', 0x00, /* 20 */
  'S', 0x00, /* 22 */
  'N', 0x00, /* 24 */
  ':', 0x00, /* 26 */
  /* Data filled by CPU with 64bits Serial Number from SPIFI to ASCII HEX */
  ' ', 0x00, /* 28 */
  ' ', 0x00, /* 30 */
  ' ', 0x00, /* 32 */
  ' ', 0x00, /* 34 */
  ' ', 0x00, /* 36 */
  ' ', 0x00, /* 38 */
  ' ', 0x00, /* 40 */
  ' ', 0x00, /* 42 */
  ' ', 0x00, /* 44 */
  ' ', 0x00, /* 46 */
  ' ', 0x00, /* 48 */
  ' ', 0x00, /* 50 */
  ' ', 0x00, /* 52 */
  ' ', 0x00, /* 54 */
  ' ', 0x00, /* 56 */
  ' ', 0x00  /* 58 */
};
#endif
static uint8_t board_fw_compatibility_check_usb_descriptor_string_serial_number[58] = {
    0x3A, 0x03, 0x48, 0x00, 0x59, 0x00, 0x44, 0x00, 0x52, 0x00, 0x41, 0x00, 0x53, 0x00, 0x44, 0x00,
    0x52, 0x00, 0x20, 0x00, 0x53, 0x00, 0x4E, 0x00, 0x3A, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00,
    0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00,
    0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00 
};

/**
 * Verify that firmware file contains the board name string
 * Returns HYDRASDR_SUCCESS if compatible, error otherwise
 */
int verify_board_compatibility(struct hydrasdr_device* device, const uint8_t* data, uint32_t length)
{
	int result;
	hydrasdr_device_info_t info;
	uint8_t device_board_id;
	const char* board_name;
	const uint8_t* found;
	size_t len;

	/* Read Device Informations */
	result = hydrasdr_get_device_info(device, &info);
	if (result != HYDRASDR_SUCCESS) {
		fprintf(stderr, "hydrasdr_get_device_info() failed: %s (%d)\n",
			hydrasdr_error_name(result), result);
		return result;
	}
	device_board_id = info.board_id;
	board_name = info.board_name;
	if (board_name == NULL || board_name[0] == '\0') {
		fprintf(stderr, "ERROR: Invalid board name with board ID %d\n", device_board_id);
		return HYDRASDR_ERROR_INVALID_PARAM;
	}

	printf("Connected Board ID: %d (%s)\n", device_board_id, board_name);
	printf("Searching for usb_descriptor_string_serial_number data and board name \"%s\" in firmware binary...\n", board_name);

	/* Search for usb_descriptor_string_serial_number in firmware data (case-sensitive) */		
	len = sizeof(board_fw_compatibility_check_usb_descriptor_string_serial_number);
	found = memmem_case_sensitive(data, length, board_fw_compatibility_check_usb_descriptor_string_serial_number, len);
	if (found == NULL) {
		fprintf(stderr, "\nERROR: Board compatibility check failed!\n");
		fprintf(stderr, "Board usb_descriptor_string_serial_number data not found in firmware binary.\n");
		fprintf(stderr, "\nThis firmware is NOT compatible with your board!\n");
		fprintf(stderr, "Connected board: %s (ID=%d)\n", board_name, device_board_id);
		fprintf(stderr, "\nFlashing incompatible firmware may brick your device!\n");
		fprintf(stderr, "Use --force to override this check (not recommended).\n");
		return HYDRASDR_ERROR_INVALID_PARAM;
	}
	printf("Board usb_descriptor_string_serial_number data found at offset 0x%08lx (len=%zu) in firmware.\n",
	       (unsigned long)(found - data), len);

	/* Search for board name string in firmware data (case-sensitive) */
	len = strlen(board_name);
	found = memmem_case_sensitive(data, length, (const uint8_t*)board_name, len);
	if (found == NULL) {
		fprintf(stderr, "\nERROR: Board compatibility check failed!\n");
		fprintf(stderr, "Board name \"%s\" not found in firmware binary.\n", board_name);
		fprintf(stderr, "\nThis firmware is NOT compatible with your board!\n");
		fprintf(stderr, "Connected board: %s (ID=%d)\n", board_name, device_board_id);
		fprintf(stderr, "\nFlashing incompatible firmware may brick your device!\n");
		fprintf(stderr, "Use --force to override this check (not recommended).\n");
		return HYDRASDR_ERROR_INVALID_PARAM;
	}
	printf("Board name found at offset 0x%08lx (len=%zu) in firmware.\n",
	       (unsigned long)(found - data), len);


	printf("Board compatibility verified: OK\n");

	return HYDRASDR_SUCCESS;
}

int main(int argc, char** argv)
{
	int opt;
	uint32_t address = 0;
	uint32_t length = 0;
	uint32_t tmp_length;
	uint16_t xfer_len = 0;
	const char* path = NULL;
	struct hydrasdr_device* device = NULL;
	int result = HYDRASDR_SUCCESS;
	int option_index = 0;
	static uint8_t data[MAX_LENGTH];
	uint8_t* pdata = &data[0];
	FILE* fd = NULL;
	bool read = false;
	bool write = false;
	bool force = false;
	uint32_t serial_number_msb_val;
	uint32_t serial_number_lsb_val;
	hydrasdr_lib_version_t lib_version;
	hydrasdr_device_info_t device_info;

	/* Display library version and check compatibility */
	hydrasdr_lib_version(&lib_version);
	printf("HydraSDR SPI Flash Tool v%s (libhydrasdr v%d.%d.%d)\n",
	       HYDRASDR_SPIFLASH_VERSION,
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

	while ((opt = getopt_long(argc, argv, "a:l:r:w:s:f", long_options, &option_index)) != EOF)
	{
		switch (opt) {
		case 'a':
			result = parse_u32(optarg, &address);
			break;

		case 'l':
			result = parse_u32(optarg, &length);
			break;

		case 'r':
			read = true;
			path = optarg;
			break;

		case 'w':
			write = true;
			path = optarg;
			break;

		case 's':
			serial_number = true;
			result = parse_u64(optarg, &serial_number_val);
			serial_number_msb_val = (uint32_t)(serial_number_val >> 32);
			serial_number_lsb_val = (uint32_t)(serial_number_val & 0xFFFFFFFF);
			printf("Board serial number to open: 0x%08X%08X\n", serial_number_msb_val, serial_number_lsb_val);
			break;

		case 'f':
			force = true;
			break;

		default:
			fprintf(stderr, "opt error: %d\n", opt);
			usage();
			return EXIT_FAILURE;
		}

		if (result != HYDRASDR_SUCCESS) {
			fprintf(stderr, "argument error: %s (%d)\n", hydrasdr_error_name(result), result);
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
	
	if (path == NULL) {
		fprintf(stderr, "Specify a path to a file.\n");
		usage();
		return EXIT_FAILURE;
	} 
	
	if( write )
	{
		fd = fopen(path, "rb");
		if(fd == NULL)
		{
			printf("Error to open file %s\n", path);
			return EXIT_FAILURE;
		}
		/* Get size of the file  */
		fseek(fd, 0, SEEK_END); /* Not really portable but work on major OS Linux/Win32 */
		length = ftell(fd);
		/* Move to start */
		rewind(fd);
		printf("File size %d bytes.\n", length);
	}

	if (length == 0) {
		fprintf(stderr, "Requested transfer of zero bytes.\n");
		if(fd != NULL)
			fclose(fd);
		usage();
		return EXIT_FAILURE;
	}

	if(length > MAX_LENGTH)
	{
		fprintf(stderr, "Request exceeds size of flash memory.\n");
		fprintf(stderr, "address=0x%08X size=%d Bytes.\n", address, length);
		if(fd != NULL)
			fclose(fd);
		usage();
		return EXIT_FAILURE;
	}

	if (read) {
		fd = fopen(path, "wb");
		if(fd == NULL)
		{
			printf("Error to open file %s\n", path);
			return EXIT_FAILURE;
		}
	}

	if (fd == NULL) {
		fprintf(stderr, "Failed to open file: %s\n", path);
		return EXIT_FAILURE;
	}

	if(serial_number == true)
	{
		result = hydrasdr_open_sn(&device, serial_number_val);
		if( result != HYDRASDR_SUCCESS ) {
			printf("hydrasdr_open_sn() failed: %s (%d)\n", hydrasdr_error_name(result), result);
			usage();
			fclose(fd);
			return EXIT_FAILURE;
		}
	}else
	{
		result = hydrasdr_open(&device);
		if( result != HYDRASDR_SUCCESS ) {
			printf("hydrasdr_open() failed: %s (%d)\n", hydrasdr_error_name(result), result);
			usage();
			fclose(fd);
			return EXIT_FAILURE;
		}
	}

	/* Get device info and check SPIFLASH capability */
	result = hydrasdr_get_device_info(device, &device_info);
	if (result != HYDRASDR_SUCCESS) {
		fprintf(stderr, "hydrasdr_get_device_info() failed: %s (%d)\n",
		        hydrasdr_error_name(result), result);
		fclose(fd);
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}

	printf("Device: %s (FW: %s)\n", device_info.board_name, device_info.firmware_version);

	/* Check SPI flash capability */
	if (!(device_info.features & HYDRASDR_CAP_SPIFLASH)) {
		fprintf(stderr, "Error: SPI flash access not supported on this device.\n");
		fclose(fd);
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}

	if (read) 
	{
		ssize_t bytes_written;
		tmp_length = length;
		while (tmp_length) 
		{
			xfer_len = (tmp_length > 256) ? 256 : tmp_length;
			printf("Reading %d bytes from 0x%06x.\n", xfer_len, address);
			result = hydrasdr_spiflash_read(device, address, xfer_len, pdata);
			if (result != HYDRASDR_SUCCESS) {
				fprintf(stderr, "hydrasdr_spiflash_read() failed: %s (%d)\n", hydrasdr_error_name(result), result);
				fclose(fd);
				hydrasdr_close(device);
				return EXIT_FAILURE;
			}     
			address += xfer_len;
			pdata += xfer_len;
			tmp_length -= xfer_len;
		}
		bytes_written = fwrite(data, 1, length, fd);
		if (bytes_written != length) {
			fprintf(stderr, "Failed write to file (wrote %d bytes).\n", (int)bytes_written);
			fclose(fd);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
	} else
	{
		/* Write Firmware */
		ssize_t bytes_read = fread(data, 1, length, fd);
		if (bytes_read != length) {
			fprintf(stderr, "Failed read file (read %d bytes).\n", (int)bytes_read);
			fclose(fd);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
		if(bytes_read > MAX_FIRMWARE_BIN_SIZE) {
			fprintf(stderr, "\nFlashing aborted due to firmware bin size > MAX_FIRMWARE_BIN_SIZE (%d).\n", MAX_FIRMWARE_BIN_SIZE);
			fclose(fd);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}

		/* Verify board compatibility before flashing */
		if (!force) {
			printf("\n=== Board Compatibility Check ===\n");
			result = verify_board_compatibility(device, data, length);
			if (result != HYDRASDR_SUCCESS) {
				fprintf(stderr, "\nFlashing aborted due to board compatibility check failure.\n");
				fclose(fd);
				hydrasdr_close(device);
				return EXIT_FAILURE;
			}
			printf("=================================\n\n");
		} else {
			printf("\nWARNING: Board compatibility check bypassed (--force used).\n");
			printf("Proceeding with flash operation...\n\n");
		}

		printf("Erasing 1st 64KB in SPI flash.\n");
		result = hydrasdr_spiflash_erase(device);
		if (result != HYDRASDR_SUCCESS) {
			fprintf(stderr, "hydrasdr_spiflash_erase() failed: %s (%d)\n", hydrasdr_error_name(result), result);
			fclose(fd);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
		while (length)
		{
			xfer_len = (length > 256) ? 256 : length;
			printf("Writing %d bytes at 0x%06x.\n", xfer_len, address);
			result = hydrasdr_spiflash_write(device, address, xfer_len, pdata);
			if (result != HYDRASDR_SUCCESS) {
				fprintf(stderr, "hydrasdr_spiflash_write() failed: %s (%d)\n", hydrasdr_error_name(result), result);
				fclose(fd);
				hydrasdr_close(device);
				return EXIT_FAILURE;
			}
			address += xfer_len;
			pdata += xfer_len;
			length -= xfer_len;
		}
		printf("\nFirmware flashing completed successfully.\n");
	}

	result = hydrasdr_close(device);
	if (result != HYDRASDR_SUCCESS) {
		fprintf(stderr, "hydrasdr_close() failed: %s (%d)\n", hydrasdr_error_name(result), result);
		fclose(fd);
		return EXIT_FAILURE;
	}

	if (fd != NULL) {
		fclose(fd);
	}

	return EXIT_SUCCESS;
}
