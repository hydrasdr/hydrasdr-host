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

int main(int argc, char** argv)
{
	hydrasdr_lib_version_t lib_version;
	uint32_t runtime_ver;

	(void)argc;  /* Unused */
	(void)argv;  /* Unused */

	/* Compile-time version (from header at build time) */
	printf("=== Compile-time (header) ===\n");
	printf("  HYDRASDR_VERSION:     %s\n", HYDRASDR_VERSION);
	printf("  HYDRASDR_VERSION_NUM: 0x%06X (%d.%d.%d)\n",
	       HYDRASDR_VERSION_NUM,
	       HYDRASDR_VER_MAJOR, HYDRASDR_VER_MINOR, HYDRASDR_VER_REVISION);

	/* Runtime version (from linked library) */
	hydrasdr_lib_version(&lib_version);
	runtime_ver = HYDRASDR_MAKE_VERSION(lib_version.major_version,
	                                     lib_version.minor_version,
	                                     lib_version.revision);

	printf("\n=== Runtime (library) ===\n");
	printf("  Version:              %d.%d.%d\n",
	       lib_version.major_version,
	       lib_version.minor_version,
	       lib_version.revision);
	printf("  Version (packed):     0x%06X\n", runtime_ver);

	/* Check for header/library mismatch */
	if (runtime_ver != HYDRASDR_VERSION_NUM) {
		printf("\n[WARN] Header/library version MISMATCH!\n");
		printf("  Header:  %d.%d.%d (0x%06X)\n",
		       HYDRASDR_VER_MAJOR, HYDRASDR_VER_MINOR, HYDRASDR_VER_REVISION,
		       HYDRASDR_VERSION_NUM);
		printf("  Library: %d.%d.%d (0x%06X)\n",
		       lib_version.major_version, lib_version.minor_version,
		       lib_version.revision, runtime_ver);
		return EXIT_FAILURE;
	}

	printf("\n[OK] Header and library versions match.\n");
	return EXIT_SUCCESS;
}
