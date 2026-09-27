/*
 * Copyright 2026 Bertold Van den Bergh <vandenbergh@bertold.org>
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

/* Device memory access and function calls, for firmware debugging */

#include <hydrasdr.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

static uint32_t num(const char* s) { return (uint32_t)strtoul(s, NULL, 0); }

static int usage(void)
{
	fprintf(stderr, "usage: hydrasdr_debug read <addr> [words] | bytes <addr> [count] | write <addr> <word>... | call <core> <addr> [r0 r1 r2 r3]\n");
	fprintf(stderr, "\tcore: 0 = M0 (USB request handler), 1 = M4 (main loop)\n");
	return 1;
}

int main(int argc, char** argv)
{
	struct hydrasdr_device* device = NULL;
	int result, ret = 0;

	if (argc < 3)
		return usage();
	result = hydrasdr_open(&device);
	if (result != HYDRASDR_SUCCESS) {
		fprintf(stderr, "hydrasdr_open() failed: %s (%d)\n", hydrasdr_error_name(result), result);
		return 1;
	}

	if (strcmp(argv[1], "read") == 0 || strcmp(argv[1], "bytes") == 0) {
		int words = strcmp(argv[1], "read") == 0;
		uint32_t addr = num(argv[2]);
		uint32_t count = argc > 3 ? num(argv[3]) : 1;
		uint32_t total = words ? count * 4 : count;
		uint32_t done = 0;
		uint8_t buf[64];

		while (done < total) {
			uint32_t n = total - done > 64 ? 64 : total - done;
			uint32_t i;

			result = hydrasdr_mem_read(device, addr + done, buf, (uint16_t)n);
			if (result != HYDRASDR_SUCCESS) {
				fprintf(stderr, "read at 0x%08x failed: %s\n", addr + done, hydrasdr_error_name(result));
				ret = 1;
				break;
			}
			if (words) {
				for (i = 0; i < n; i += 4) {
					uint32_t w;
					memcpy(&w, &buf[i], 4);
					if (((done + i) % 16) == 0)
						printf("%s%08x:", (done + i) ? "\n" : "", addr + done + i);
					printf(" %08x", w);
				}
			} else {
				for (i = 0; i < n; i++) {
					if (((done + i) % 16) == 0)
						printf("%s%08x:", (done + i) ? "\n" : "", addr + done + i);
					printf(" %02x", buf[i]);
				}
			}
			done += n;
		}
		printf("\n");
	} else if (strcmp(argv[1], "write") == 0) {
		uint32_t addr = num(argv[2]);
		int i;

		for (i = 3; i < argc; i++) {
			uint32_t w = num(argv[i]);
			result = hydrasdr_mem_write(device, addr, &w, 4);
			if (result != HYDRASDR_SUCCESS) {
				fprintf(stderr, "write at 0x%08x failed: %s\n", addr, hydrasdr_error_name(result));
				ret = 1;
				break;
			}
			addr += 4;
		}
	} else if (strcmp(argv[1], "call") == 0 && argc >= 4) {
		uint32_t args[4] = { 0, 0, 0, 0 };
		uint32_t r0 = 0;
		int i;

		for (i = 0; i < 4 && argc > 4 + i; i++)
			args[i] = num(argv[4 + i]);
		if (num(argv[3]) < 0x1000) {
			fprintf(stderr, "refusing to call address 0x%08x\n", num(argv[3]));
			hydrasdr_close(device);
			return 1;
		}
		result = hydrasdr_call(device, num(argv[2]), num(argv[3]), args, &r0, 2000);
		if (result != HYDRASDR_SUCCESS) {
			fprintf(stderr, "call failed: %s\n", hydrasdr_error_name(result));
			ret = 1;
		} else {
			printf("r0 = 0x%08x (%u)\n", r0, r0);
		}
	} else {
		ret = usage();
	}

	hydrasdr_close(device);
	return ret;
}
