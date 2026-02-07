/*
 * Copyright 2012 Jared Boone <jared@sharebrained.com>
 * Copyright 2014-2026 Benjamin Vernoux <bvernoux@hydrasdr.com>
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
#include <time.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <limits.h>

#define HYDRASDR_RX_VERSION "1.1.0 2025-2026"

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

#define strtoull _strtoui64
#define snprintf _snprintf

int gettimeofday(struct timeval *tv, void* ignored)
{
	FILETIME ft;
	unsigned __int64 tmp = 0;
	if (NULL != tv) {
		GetSystemTimeAsFileTime(&ft);
		tmp |= ft.dwHighDateTime;
		tmp <<= 32;
		tmp |= ft.dwLowDateTime;
		tmp /= 10;
		tmp -= 11644473600000000Ui64;
		tv->tv_sec = (long)(tmp / 1000000UL);
		tv->tv_usec = (long)(tmp % 1000000UL);
	}
	return 0;
}

#endif
#endif

#if defined(__GNUC__)
#include <unistd.h>
#include <sys/time.h>
#endif

#include <signal.h>

#if defined _WIN32
	#define sleep(a) Sleep( (a*1000) )
#endif

#define SAMPLE_SCALE_FLOAT_TO_INT ( (8192.0f) )

#define FLOAT32_EL_SIZE_BYTE (4)	/* 4bytes = 32bit float */
#define INT16_EL_SIZE_BYTE (2)   /* 2bytes = 16bit int */
#define INT12_EL_SIZE_BITS (12)
#define INT8_EL_SIZE_BITS (8)

#define FD_BUFFER_SIZE (16*1024)

#define FREQ_ONE_MHZ (1000000ull)

#define PACKING_MAX (0xffffffff)
#define SAMPLE_TYPE_MAX (HYDRASDR_SAMPLE_END-1)
#define BIAST_MAX (1)
#define SAMPLES_TO_XFER_MAX_U64 (0x8000000000000000ull) /* Max value */

#define MIN_SAMPLERATE_BY_VALUE (100000ul)

/* WAVE or RIFF WAVE file format containing data for HydraSDR compatible with SDR++ Wav IQ file */
typedef struct 
{
		char groupID[4]; /* 'RIFF' */
		uint32_t size; /* File size + 8bytes */
		char riffType[4]; /* 'WAVE'*/
} t_WAVRIFF_hdr;

#define FormatID "fmt "   /* chunkID for Format Chunk. NOTE: There is a space at the end of this ID. */

typedef struct {
	char chunkID[4]; /* 'fmt ' */
	uint32_t chunkSize; /* 16 fixed */

	uint16_t wFormatTag; /* 1=PCM8/16, 3=Float32 */
	uint16_t wChannels;
	uint32_t dwSamplesPerSec; /* Freq Hz sampling */
	uint32_t dwAvgBytesPerSec; /* Freq Hz sampling x 2 */
	uint16_t wBlockAlign;
	uint16_t wBitsPerSample;
} t_FormatChunk;

typedef struct 
{
		char chunkID[4]; /* 'data' */
		uint32_t chunkSize; /* Size of data in bytes */
	/* For IQ samples I(16 or 32bits) then Q(16 or 32bits), I, Q ... */
} t_DataChunk;

typedef struct
{
	t_WAVRIFF_hdr hdr;
	t_FormatChunk fmt_chunk;
	t_DataChunk data_chunk;
} t_wav_file_hdr;

t_wav_file_hdr wave_file_hdr = 
{
	/* t_WAVRIFF_hdr */
	{
		{ 'R', 'I', 'F', 'F' }, /* groupID */
		0, /* size to update later */
		{ 'W', 'A', 'V', 'E' }
	},
	/* t_FormatChunk */
	{
		{ 'f', 'm', 't', ' ' }, /* char		chunkID[4];  */
		16, /* uint32_t chunkSize; */
		0, /* uint16_t wFormatTag; to update later */
		0, /* uint16_t wChannels; to update later */
		0, /* uint32_t dwSamplesPerSec; Freq Hz sampling to update later */
		0, /* uint32_t dwAvgBytesPerSec; to update later */
		0, /* uint16_t wBlockAlign; to update later */
		0, /* uint16_t wBitsPerSample; to update later  */
	},
	/* t_DataChunk */
	{
		{ 'd', 'a', 't', 'a' }, /* char chunkID[4]; */
		0, /* uint32_t	chunkSize; to update later */
	}
};

#define U64TOA_MAX_DIGIT (31)
typedef struct 
{
		char data[U64TOA_MAX_DIGIT+1];
} t_u64toa;

t_u64toa ascii_u64_data1;
t_u64toa ascii_u64_data2;
t_u64toa ascii_u64_data3;

hydrasdr_receiver_mode_t receiver_mode = HYDRASDR_RECEIVER_MODE_RX;

/* Gain values (initialized to 0, will use device defaults from device_info API) */
uint32_t vga_gain = 0;
uint32_t lna_gain = 0;
uint32_t mixer_gain = 0;
uint32_t rf_gain = 0;
uint32_t filter_gain = 0;

uint32_t linearity_gain_val;
bool linearity_gain = false;

uint32_t sensitivity_gain_val;
bool sensitivity_gain = false;

/* Flags for which gains were explicitly set by user */
bool vga_gain_set = false;
bool lna_gain_set = false;
bool mixer_gain_set = false;
bool rf_gain_set = false;
bool filter_gain_set = false;

/* WAV default values */
uint16_t wav_format_tag=1; /* PCM8 or PCM16 */
uint16_t wav_nb_channels=2;
uint32_t wav_sample_per_sec;
uint16_t wav_nb_byte_per_sample=2;
uint16_t wav_nb_bits_per_sample=16;

/* Device info for capability-based gain validation */
hydrasdr_device_info_t device_info;

/* Temperature statistics */
float temp_min_celsius = 1000.0f;
float temp_max_celsius = -1000.0f;
uint32_t temp_sample_count = 0;
time_t temp_min_time = 0;
time_t temp_max_time = 0;

volatile bool do_exit = false;

FILE* fd = NULL;

bool verbose = false;
bool receive = false;
bool receive_wav = false;

struct timeval time_start;
struct timeval t_start;

bool got_first_packet = false;
float average_rate = 0.0f;
float global_average_rate = 0.0f;
uint32_t rate_samples = 0;
uint32_t buffer_count = 0;
uint32_t sample_count = 0;
	
bool freq = false;
uint64_t freq_hz;

bool limit_num_samples = false;
uint64_t samples_to_xfer = 0;
uint64_t bytes_to_xfer = 0;

bool call_set_packing = false;
uint32_t packing_val = 0;

bool sample_rate = false;
uint32_t sample_rate_val = 0;  /* Default to index 0 (first supported rate) */

bool sample_type = false;
enum hydrasdr_sample_type sample_type_val;

bool biast = false;
uint32_t biast_val;

bool rf_port = false;
uint32_t rf_port_val = 0;

bool bandwidth = false;
uint32_t bandwidth_val = 0;

bool decimation_mode = false;
uint32_t decimation_mode_val = 0;

bool serial_number = false;
uint64_t serial_number_val;

/* Dropped samples tracking - thread-safe 64-bit counter */
volatile uint64_t total_dropped_samples = 0;

/* Cross-platform atomic helpers for 64-bit counters */
#if defined(_MSC_VER)
  #define ATOMIC_ADD64(ptr, val) InterlockedExchangeAdd64((volatile LONG64*)(ptr), (LONG64)(val))
#elif defined(__GNUC__) || defined(__clang__)
  #define ATOMIC_ADD64(ptr, val) __atomic_fetch_add((ptr), (val), __ATOMIC_RELAXED)
#else
  #define ATOMIC_ADD64(ptr, val) (*(ptr) += (val))
#endif

static float
TimevalDiff(const struct timeval *a, const struct timeval *b)
{
	return (a->tv_sec - b->tv_sec) + 1e-6f * (a->tv_usec - b->tv_usec);
}

/**
 * @brief Helper to calculate bytes per sample based on API enum.
 * @param type The hydrasdr_sample_type enum from the transfer struct.
 * @return Number of bytes per single sample (I+Q combined if applicable).
 */
static inline size_t bytes_per_sample(enum hydrasdr_sample_type type)
{
	switch (type) {
		case HYDRASDR_SAMPLE_FLOAT32_IQ:   return 8; /* 4 bytes I + 4 bytes Q */
		case HYDRASDR_SAMPLE_FLOAT32_REAL: return 4;
		case HYDRASDR_SAMPLE_INT16_IQ:     return 4; /* 2 bytes I + 2 bytes Q */
		case HYDRASDR_SAMPLE_INT16_REAL:   return 2;
		case HYDRASDR_SAMPLE_UINT16_REAL:  return 2;
		case HYDRASDR_SAMPLE_RAW:          return 2; /* Raw device stream (or 1.5 if packed) */
		case HYDRASDR_SAMPLE_INT8_IQ:      return 2; /* 1 byte I + 1 byte Q */
		case HYDRASDR_SAMPLE_UINT8_IQ:     return 2; /* 1 byte I + 1 byte Q */
		case HYDRASDR_SAMPLE_INT8_REAL:    return 1;
		case HYDRASDR_SAMPLE_UINT8_REAL:   return 1;
		default:                           return 2;
	}
}

/**
 * @brief Get sample type name string
 */
static const char* sample_type_name(enum hydrasdr_sample_type type)
{
	switch (type) {
		case HYDRASDR_SAMPLE_FLOAT32_IQ:   return "FLOAT32_IQ";
		case HYDRASDR_SAMPLE_FLOAT32_REAL: return "FLOAT32_REAL";
		case HYDRASDR_SAMPLE_INT16_IQ:     return "INT16_IQ";
		case HYDRASDR_SAMPLE_INT16_REAL:   return "INT16_REAL";
		case HYDRASDR_SAMPLE_UINT16_REAL:  return "UINT16_REAL";
		case HYDRASDR_SAMPLE_RAW:          return "RAW";
		case HYDRASDR_SAMPLE_INT8_IQ:      return "INT8_IQ";
		case HYDRASDR_SAMPLE_UINT8_IQ:     return "UINT8_IQ";
		case HYDRASDR_SAMPLE_INT8_REAL:    return "INT8_REAL";
		case HYDRASDR_SAMPLE_UINT8_REAL:   return "UINT8_REAL";
		default:                           return "UNKNOWN";
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

int parse_u32(char* s, uint32_t* const value)
{
	uint_fast8_t base = 10;
	char* s_end;
	uint64_t ulong_value;

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
	ulong_value = strtoul(s, &s_end, base);
	if( (s != s_end) && (*s_end == 0) ) {
		*value = (uint32_t)ulong_value;
		return HYDRASDR_SUCCESS;
	} else {
		return HYDRASDR_ERROR_INVALID_PARAM;
	}
}

static char *stringrev(char *str)
{
	char *p1, *p2;

	if(! str || ! *str)
		return str;

	for(p1 = str, p2 = str + strlen(str) - 1; p2 > p1; ++p1, --p2)
	{
		*p1 ^= *p2;
		*p2 ^= *p1;
		*p1 ^= *p2;
	}
	return str;
}

char* u64toa(uint64_t val, t_u64toa* str)
{
	#define BASE (10ull) /* Base10 by default */
	uint64_t sum;
	int pos;
	int digit;
	int max_len;
	char* res;

	sum = val;
	max_len = U64TOA_MAX_DIGIT;
	pos = 0;

	do
	{
		digit = (sum % BASE);
		str->data[pos] = digit + '0';
		pos++;

		sum /= BASE;
	}while( (sum>0) && (pos < max_len) );

	if( (pos == max_len) && (sum>0) )
		return NULL;

	str->data[pos] = '\0';
	res = stringrev(str->data);

	return res;
}

/**
 * @brief Asynchronous RX Callback Function.
 *
 * @details This function is invoked by the HydraSDR library thread.
 * Critical Section: Execution time must be minimized.
 * Uses transfer->sample_type for dynamic bytes calculation.
 *
 * @param transfer Pointer to the transfer structure containing data and metadata.
 * @return int 0 to continue streaming, non-zero to request stop.
 */
int rx_callback(hydrasdr_transfer_t* transfer)
{
	size_t bytes_to_write;
	size_t bytes_written;
	struct timeval time_now;
	float time_difference, rate;

	if (do_exit)
		return -1;

	/* Track dropped samples */
	if (transfer->dropped_samples)
		ATOMIC_ADD64(&total_dropped_samples, transfer->dropped_samples);

	if (fd == NULL)
		return -1;

	/* Calculate bytes using transfer's sample_type (hardware-agnostic) */
	const size_t bps = bytes_per_sample(transfer->sample_type);

	/* Handle RAW mode with packing (12-bit packed) */
	if (transfer->sample_type == HYDRASDR_SAMPLE_RAW && packing_val) {
		bytes_to_write = (size_t)transfer->sample_count * INT12_EL_SIZE_BITS / INT8_EL_SIZE_BITS;
	} else {
		bytes_to_write = (size_t)transfer->sample_count * bps;
	}

	/* Update timing statistics */
	gettimeofday(&time_now, NULL);

	if (!got_first_packet) {
		t_start = time_now;
		time_start = time_now;
		got_first_packet = true;
	} else {
		buffer_count++;
		sample_count += transfer->sample_count;
		if (buffer_count == 50) {
			time_difference = TimevalDiff(&time_now, &time_start);
			rate = (float) sample_count / time_difference;
			average_rate += 0.2f * (rate - average_rate);
			global_average_rate += average_rate;
			rate_samples++;
			time_start = time_now;
			sample_count = 0;
			buffer_count = 0;
		}
	}

	/* Handle sample limit */
	if (limit_num_samples) {
		if (bytes_to_write >= bytes_to_xfer) {
			bytes_to_write = (size_t)bytes_to_xfer;
		}
		bytes_to_xfer -= bytes_to_write;
	}

	/* Write samples to file */
	if (bytes_to_write > 0 && transfer->samples != NULL) {
		bytes_written = fwrite(transfer->samples, 1, bytes_to_write, fd);
		if (bytes_written != bytes_to_write) {
			fprintf(stderr, "Disk write error\n");
			return -1;
		}
	}

	/* Check if transfer complete */
	if (limit_num_samples && bytes_to_xfer == 0)
		return -1;

	return 0;
}

/* Static usage for when device is not open yet */
static void usage_static(void)
{
	fprintf(stderr, "Usage:\n");
	fprintf(stderr, "-r <filename>: Receive data into file\n");
	fprintf(stderr, "-w Receive data into file with WAV header and automatic name\n");
	fprintf(stderr, " This is for SDR++ compatibility and may not work with other software\n");
	fprintf(stderr, "[-s serial_number_64bits]: Open device with specified 64bits serial number\n");
	fprintf(stderr, "[-p packing]: Set packing for samples, \n");
	fprintf(stderr, " 1=enabled(12bits packed), 0=disabled(default 16bits not packed)\n");
	fprintf(stderr, "[-f frequency_MHz]: Set frequency in MHz (device-specific range)\n");
	fprintf(stderr, "[-a sample_rate]: Set sample rate\n");
	fprintf(stderr, "[-B bandwidth]: Set bandwidth (if supported by device)\n");
	fprintf(stderr, "[-t sample_type]: Set sample type (device-specific supported types)\n");
	fprintf(stderr, "[-P rf_port]: Set RF port (0=first port, device-specific)\n");
	fprintf(stderr, "[-b biast]: Set Bias Tee, 1=enabled, 0=disabled(default)\n");
	fprintf(stderr, "Gain options (availability and ranges depend on device):\n");
	fprintf(stderr, "[-v vga_gain]: Set VGA/IF gain\n");
	fprintf(stderr, "[-l lna_gain]: Set LNA gain\n");
	fprintf(stderr, "[-m mixer_gain]: Set Mixer gain\n");
	fprintf(stderr, "[-R rf_gain]: Set RF gain\n");
	fprintf(stderr, "[-F filter_gain]: Set Filter gain\n");
	fprintf(stderr, "[-g linearity_gain]: Set linearity simplified gain\n");
	fprintf(stderr, "[-i sensitivity_gain]: Set sensitivity simplified gain\n");
	fprintf(stderr, "[-D decimation_mode]: Set decimation mode: 0=Low Bandwidth (default), 1=High Definition\n");
	fprintf(stderr, "   High Definition uses highest HW sample rate with decimation\n");
	fprintf(stderr, "[-n num_samples]: Number of samples to transfer (default is unlimited)\n");
	fprintf(stderr, "[-d]: Verbose mode\n");
	fprintf(stderr, "\nNote: Connect device to see device-specific options, sample types, and RF ports.\n");
}

/* Dynamic usage showing device-specific capabilities and ranges */
static void usage_dynamic(const hydrasdr_device_info_t *info)
{
	uint8_t i;

	fprintf(stderr, "Device: %s\n\n", info->board_name);
	fprintf(stderr, "Usage:\n");
	fprintf(stderr, "-r <filename>: Receive data into file\n");
	fprintf(stderr, "-w Receive data into file with WAV header and automatic name\n");
	fprintf(stderr, " This is for SDR++ compatibility and may not work with other software\n");
	fprintf(stderr, "[-s serial_number_64bits]: Open device with specified 64bits serial number\n");
	fprintf(stderr, "[-p packing]: Set packing for samples, \n");
	fprintf(stderr, " 1=enabled(12bits packed), 0=disabled(default 16bits not packed)\n");
	fprintf(stderr, "[-f frequency_MHz]: Set frequency in MHz between [%.1f, %.1f]\n",
		(double)info->min_frequency / 1e6,
		(double)info->max_frequency / 1e6);
	fprintf(stderr, "[-a sample_rate]: Set sample rate\n");

	/* Display bandwidth option if supported */
	if (info->features & HYDRASDR_CAP_BANDWIDTH) {
		fprintf(stderr, "[-B bandwidth]: Set bandwidth (device supports bandwidth control)\n");
	}

	/* Display supported sample types from device_info */
	fprintf(stderr, "[-t sample_type]: Set sample type. Supported types:\n");
	for (i = 0; i < HYDRASDR_SAMPLE_END; i++) {
		if (info->sample_types & (1 << i)) {
			fprintf(stderr, "   %d=%s%s\n", i, sample_type_name((enum hydrasdr_sample_type)i),
				(i == HYDRASDR_SAMPLE_INT16_IQ) ? " (default)" : "");
		}
	}

	/* Display RF ports if supported */
	if ((info->features & HYDRASDR_CAP_RF_PORT_SELECT) && info->rf_port_count > 0) {
		fprintf(stderr, "[-P rf_port]: Set RF port (0-%d). Available ports:\n", info->rf_port_count - 1);
		for (i = 0; i < info->rf_port_count; i++) {
			fprintf(stderr, "   %d=%s [%.1f-%.1f MHz]%s\n", i,
				info->rf_port_info[i].name,
				(double)info->rf_port_info[i].min_frequency / 1e6,
				(double)info->rf_port_info[i].max_frequency / 1e6,
				info->rf_port_info[i].has_bias_tee ? " (bias-tee)" : "");
		}
	}

	if (info->features & HYDRASDR_CAP_BIAS_TEE) {
		fprintf(stderr, "[-b biast]: Set Bias Tee, 1=enabled, 0=disabled(default)\n");
	}

	fprintf(stderr, "\nGain options for %s:\n", info->board_name);

	if (info->features & HYDRASDR_CAP_VGA_GAIN) {
		fprintf(stderr, "[-v vga_gain]: Set VGA/IF gain, %d-%d (default %d)\n",
			(int)info->vga_gain.min_value, (int)info->vga_gain.max_value, (int)info->vga_gain.default_value);
	}
	if (info->features & HYDRASDR_CAP_LNA_GAIN) {
		fprintf(stderr, "[-l lna_gain]: Set LNA gain, %d-%d (default %d)\n",
			(int)info->lna_gain.min_value, (int)info->lna_gain.max_value, (int)info->lna_gain.default_value);
	}
	if (info->features & HYDRASDR_CAP_MIXER_GAIN) {
		fprintf(stderr, "[-m mixer_gain]: Set Mixer gain, %d-%d (default %d)\n",
			(int)info->mixer_gain.min_value, (int)info->mixer_gain.max_value, (int)info->mixer_gain.default_value);
	}
	if (info->features & HYDRASDR_CAP_RF_GAIN) {
		fprintf(stderr, "[-R rf_gain]: Set RF gain, %d-%d (default %d)\n",
			(int)info->rf_gain.min_value, (int)info->rf_gain.max_value, (int)info->rf_gain.default_value);
	}
	if (info->features & HYDRASDR_CAP_FILTER_GAIN) {
		fprintf(stderr, "[-F filter_gain]: Set Filter gain, %d-%d (default %d)\n",
			(int)info->filter_gain.min_value, (int)info->filter_gain.max_value, (int)info->filter_gain.default_value);
	}
	if (info->features & HYDRASDR_CAP_LINEARITY_GAIN) {
		fprintf(stderr, "[-g linearity_gain]: Set linearity simplified gain, %d-%d (default %d)\n",
			(int)info->linearity_gain.min_value, (int)info->linearity_gain.max_value, (int)info->linearity_gain.default_value);
	}
	if (info->features & HYDRASDR_CAP_SENSITIVITY_GAIN) {
		fprintf(stderr, "[-i sensitivity_gain]: Set sensitivity simplified gain, %d-%d (default %d)\n",
			(int)info->sensitivity_gain.min_value, (int)info->sensitivity_gain.max_value, (int)info->sensitivity_gain.default_value);
	}

	/* Decimation mode (v1.1.0+ API feature) */
	fprintf(stderr, "[-D decimation_mode]: Set decimation mode: 0=Low Bandwidth (default), 1=High Definition\n");
	fprintf(stderr, "   High Definition uses highest HW sample rate with decimation\n");

	fprintf(stderr, "\n[-n num_samples]: Number of samples to transfer (default is unlimited)\n");
	fprintf(stderr, "[-d]: Verbose mode\n");
}

struct hydrasdr_device* device = NULL;

#ifdef _MSC_VER
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Caught signal %d\n", signum);
		do_exit = true;
		return TRUE;
	}
	return FALSE;
}
#else
void sigint_callback_handler(int signum) 
{
	fprintf(stderr, "Caught signal %d\n", signum);
	do_exit = true;
}
#endif

#define PATH_FILE_MAX_LEN (FILENAME_MAX)
#define DATE_TIME_MAX_LEN (32)

int main(int argc, char** argv)
{
	int opt;
	char path_file[PATH_FILE_MAX_LEN];
	char date_time[DATE_TIME_MAX_LEN];
	char sample_type_str[16];
	char channels_str[8];
	const char* path = NULL;
	int result;
	time_t rawtime;
	struct tm * timeinfo;
	struct timeval t_end;
	float time_diff;
	uint32_t file_pos;
	int exit_code = EXIT_SUCCESS;

	uint32_t count;
	uint32_t packing_val_u32;
	uint32_t *supported_samplerates;
	uint32_t sample_rate_u32;
	uint32_t sample_type_u32;
	double freq_hz_temp;
	char str[20];
	hydrasdr_lib_version_t lib_version;

	/* Display library version and check compatibility */
	hydrasdr_lib_version(&lib_version);
	fprintf(stderr, "hydrasdr_rx v%s (libhydrasdr v%d.%d.%d)\n",
	        HYDRASDR_RX_VERSION,
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

	/* Default sample type int16 IQ */
	sample_type_val = HYDRASDR_SAMPLE_INT16_IQ;
	snprintf(sample_type_str, sizeof(sample_type_str), "int16");
	snprintf(channels_str, sizeof(channels_str), "IQ");

	/* Show help if no arguments provided */
	if (argc == 1) {
		/* Try to connect to device and show device-specific options */
		result = hydrasdr_open(&device);
		if (result == HYDRASDR_SUCCESS) {
			hydrasdr_device_info_t device_info;
			result = hydrasdr_get_device_info(device, &device_info);
			if (result == HYDRASDR_SUCCESS) {
				usage_dynamic(&device_info);

				/* Query and display supported sample rates */
				count = 0;
				result = hydrasdr_get_samplerates(device, &count, 0);
				if (result == HYDRASDR_SUCCESS && count > 0) {
					supported_samplerates = malloc(count * sizeof(uint32_t));
					if (supported_samplerates) {
						result = hydrasdr_get_samplerates(device, supported_samplerates, count);
						if (result == HYDRASDR_SUCCESS) {
							fprintf(stderr, "\nSupported sample rates (-a option):\n");
							for (uint32_t i = 0; i < count; i++) {
								if (supported_samplerates[i] >= 1000000) {
									fprintf(stderr, "  %u = %.2f MHz\n", i,
									        (double)supported_samplerates[i] / 1e6);
								} else {
									fprintf(stderr, "  %u = %u Hz\n", i, supported_samplerates[i]);
								}
							}
						}
						free(supported_samplerates);
					}
				}

				/* Query and display supported bandwidths if supported */
				if (device_info.features & HYDRASDR_CAP_BANDWIDTH) {
					uint32_t bw_count = 0;
					result = hydrasdr_get_bandwidths(device, &bw_count, 0);
					if (result == HYDRASDR_SUCCESS && bw_count > 0) {
						uint32_t *bandwidths = malloc(bw_count * sizeof(uint32_t));
						if (bandwidths) {
							result = hydrasdr_get_bandwidths(device, bandwidths, bw_count);
							if (result == HYDRASDR_SUCCESS) {
								fprintf(stderr, "\nSupported bandwidths (-B option):\n");
								for (uint32_t i = 0; i < bw_count; i++) {
									if (bandwidths[i] >= 1000000) {
										fprintf(stderr, "  %u = %.2f MHz\n", i,
										        (double)bandwidths[i] / 1e6);
									} else {
										fprintf(stderr, "  %u = %u Hz\n", i, bandwidths[i]);
									}
								}
							}
							free(bandwidths);
						}
					}
				}
			}
			hydrasdr_close(device);
		} else {
			/* No device detected, show generic help */
			usage_static();
		}
		return EXIT_SUCCESS;
	}

	while( (opt = getopt(argc, argv, "r:ws:p:f:a:B:t:P:b:v:m:l:R:F:g:i:D:n:d")) != EOF )
	{
		result = HYDRASDR_SUCCESS;
		switch( opt )
		{
			case 'r':
				receive = true;
				path = optarg;
			break;

			case 'w':
				receive_wav = true;
			 break;

			case 's':
				serial_number = true;
				result = parse_u64(optarg, &serial_number_val);
			break;

			case 'p': /* packing */
				result = parse_u32(optarg, &packing_val_u32);
				switch (packing_val_u32)
				{
					case 0:
					case 1:
						packing_val = packing_val_u32;
						call_set_packing = true;
					break;

					default:
						/* Invalid value will display error */
						packing_val = PACKING_MAX;
						call_set_packing = false;
					break;
				}
			break;

			case 'f':
				freq = true;
				freq_hz_temp = strtod(optarg, NULL) * (double)FREQ_ONE_MHZ;
				freq_hz = (uint64_t)freq_hz_temp;
			break;

			case 'a': /* Sample rate */
				sample_rate = true;
				result = parse_u32(optarg, &sample_rate_u32);
			break;

			case 'B': /* Bandwidth */
				bandwidth = true;
				result = parse_u32(optarg, &bandwidth_val);
			break;

			case 't': /* Sample type see also hydrasdr_sample_type */
				result = parse_u32(optarg, &sample_type_u32);
				switch (sample_type_u32)
				{
					case 0:
						sample_type_val = HYDRASDR_SAMPLE_FLOAT32_IQ;
						wav_format_tag = 3; /* Float32 */
						wav_nb_channels = 2;
						wav_nb_bits_per_sample = 32;
						wav_nb_byte_per_sample = (wav_nb_bits_per_sample / 8);
						snprintf(sample_type_str, sizeof(sample_type_str), "float32");
						snprintf(channels_str, sizeof(channels_str), "IQ");
					break;

					case 1:
						sample_type_val = HYDRASDR_SAMPLE_FLOAT32_REAL;
						wav_format_tag = 3; /* Float32 */
						wav_nb_channels = 1;
						wav_nb_bits_per_sample = 32;
						wav_nb_byte_per_sample = (wav_nb_bits_per_sample / 8);
						snprintf(sample_type_str, sizeof(sample_type_str), "float32");
						snprintf(channels_str, sizeof(channels_str), "REAL");
					break;

					case 2:
						sample_type_val = HYDRASDR_SAMPLE_INT16_IQ;
						wav_format_tag = 1; /* PCM8 or PCM16 */
						wav_nb_channels = 2;
						wav_nb_bits_per_sample = 16;
						wav_nb_byte_per_sample = (wav_nb_bits_per_sample / 8);
						snprintf(sample_type_str, sizeof(sample_type_str), "int16");
						snprintf(channels_str, sizeof(channels_str), "IQ");
					break;

					case 3:
						sample_type_val = HYDRASDR_SAMPLE_INT16_REAL;
						wav_format_tag = 1; /* PCM8 or PCM16 */
						wav_nb_channels = 1;
						wav_nb_bits_per_sample = 16;
						wav_nb_byte_per_sample = (wav_nb_bits_per_sample / 8);
						snprintf(sample_type_str, sizeof(sample_type_str), "int16");
						snprintf(channels_str, sizeof(channels_str), "REAL");
					break;

					case 4:
						sample_type_val = HYDRASDR_SAMPLE_UINT16_REAL;
						wav_format_tag = 1; /* PCM8 or PCM16 */
						wav_nb_channels = 1;
						wav_nb_bits_per_sample = 16;
						wav_nb_byte_per_sample = (wav_nb_bits_per_sample / 8);
						snprintf(sample_type_str, sizeof(sample_type_str), "uint16");
						snprintf(channels_str, sizeof(channels_str), "REAL");
					break;

					case 5:
						sample_type_val = HYDRASDR_SAMPLE_RAW;
						wav_nb_bits_per_sample = 12;
						wav_nb_channels = 1;
						snprintf(sample_type_str, sizeof(sample_type_str), "raw12");
						snprintf(channels_str, sizeof(channels_str), "RAW");
						break;

					case 6:
						sample_type_val = HYDRASDR_SAMPLE_INT8_IQ;
						wav_format_tag = 1; /* PCM8 */
						wav_nb_channels = 2;
						wav_nb_bits_per_sample = 8;
						wav_nb_byte_per_sample = (wav_nb_bits_per_sample / 8);
						snprintf(sample_type_str, sizeof(sample_type_str), "int8");
						snprintf(channels_str, sizeof(channels_str), "IQ");
						break;

					case 7:
						sample_type_val = HYDRASDR_SAMPLE_UINT8_IQ;
						wav_format_tag = 1; /* PCM8 */
						wav_nb_channels = 2;
						wav_nb_bits_per_sample = 8;
						wav_nb_byte_per_sample = (wav_nb_bits_per_sample / 8);
						snprintf(sample_type_str, sizeof(sample_type_str), "uint8");
						snprintf(channels_str, sizeof(channels_str), "IQ");
						break;

					case 8:
						sample_type_val = HYDRASDR_SAMPLE_INT8_REAL;
						wav_format_tag = 1; /* PCM8 */
						wav_nb_channels = 1;
						wav_nb_bits_per_sample = 8;
						wav_nb_byte_per_sample = (wav_nb_bits_per_sample / 8);
						snprintf(sample_type_str, sizeof(sample_type_str), "int8");
						snprintf(channels_str, sizeof(channels_str), "REAL");
						break;

					case 9:
						sample_type_val = HYDRASDR_SAMPLE_UINT8_REAL;
						wav_format_tag = 1; /* PCM8 */
						wav_nb_channels = 1;
						wav_nb_bits_per_sample = 8;
						wav_nb_byte_per_sample = (wav_nb_bits_per_sample / 8);
						snprintf(sample_type_str, sizeof(sample_type_str), "uint8");
						snprintf(channels_str, sizeof(channels_str), "REAL");
						break;

					default:
						/* Invalid value will display error */
						sample_type_val = SAMPLE_TYPE_MAX+1;
					break;
				}
			break;

			case 'P': /* RF port selection */
				rf_port = true;
				result = parse_u32(optarg, &rf_port_val);
			break;

			case 'b':
				biast = true;
				result = parse_u32(optarg, &biast_val);
			break;

			case 'v':
				vga_gain_set = true;
				result = parse_u32(optarg, &vga_gain);
			break;

			case 'm':
				mixer_gain_set = true;
				result = parse_u32(optarg, &mixer_gain);
			break;

			case 'l':
				lna_gain_set = true;
				result = parse_u32(optarg, &lna_gain);
			break;

			case 'R':
				rf_gain_set = true;
				result = parse_u32(optarg, &rf_gain);
			break;

			case 'F':
				filter_gain_set = true;
				result = parse_u32(optarg, &filter_gain);
			break;

			case 'g':
				linearity_gain = true;
				result = parse_u32(optarg, &linearity_gain_val);
			break;

			case 'i':
				sensitivity_gain = true;
				result = parse_u32(optarg, &sensitivity_gain_val);
			break;

			case 'D':
				decimation_mode = true;
				result = parse_u32(optarg, &decimation_mode_val);
				if (result == HYDRASDR_SUCCESS && decimation_mode_val > 1) {
					fprintf(stderr, "argument error: decimation_mode must be 0 or 1\n");
					result = HYDRASDR_ERROR_INVALID_PARAM;
				}
			break;

			case 'n':
				limit_num_samples = true;
				result = parse_u64(optarg, &samples_to_xfer);
			break;

			case 'd':
				verbose = true;
			break;

			default:
				fprintf(stderr, "unknown argument '-%c %s'\n", opt, optarg);
				usage_static();
				return EXIT_FAILURE;
		}
		
		if( result != HYDRASDR_SUCCESS ) {
			fprintf(stderr, "argument error: '-%c %s' %s (%d)\n", opt, optarg, hydrasdr_error_name(result), result);
			usage_static();
			return EXIT_FAILURE;
		}
	}

	if (sample_rate)
	{
		sample_rate_val = sample_rate_u32;
	}

	bytes_to_xfer = samples_to_xfer * wav_nb_bits_per_sample * wav_nb_channels / 8;

	if (samples_to_xfer >= SAMPLES_TO_XFER_MAX_U64) {
		fprintf(stderr, "argument error: num_samples must be less than %s/%sMio\n",
				u64toa(SAMPLES_TO_XFER_MAX_U64, &ascii_u64_data1),
				u64toa((SAMPLES_TO_XFER_MAX_U64/FREQ_ONE_MHZ), &ascii_u64_data2) );
		usage_static();
		return EXIT_FAILURE;
	}

	/* Frequency default and WAV filename generation deferred to after device_info is obtained */

	receiver_mode = HYDRASDR_RECEIVER_MODE_RX;
	if( receive_wav )
	{
		if (sample_type_val == HYDRASDR_SAMPLE_RAW)
		{
			fprintf(stderr, "The RAW sampling mode is not compatible with Wave files\n");
			usage_static();
			return EXIT_FAILURE;
		}
		/* WAV filename will be generated after device_info is obtained */
	}

	if( path == NULL && !receive_wav ) {
		fprintf(stderr, "error: you shall specify at least -r <with filename> or -w option\n");
		usage_static();
		return EXIT_FAILURE;
	}

	if(packing_val == PACKING_MAX) {
		fprintf(stderr, "argument error: packing out of range\n");
		usage_static();
		return EXIT_FAILURE;
	}

	if(sample_type_val > SAMPLE_TYPE_MAX) {
		fprintf(stderr, "argument error: sample_type out of range\n");
		usage_static();
		return EXIT_FAILURE;
	}

	if(biast_val > BIAST_MAX) {
		fprintf(stderr, "argument error: biast_val out of range\n");
		usage_static();
		return EXIT_FAILURE;
	}

	/* Gain validation deferred to after device info is obtained */

	if( (linearity_gain == true) && (sensitivity_gain == true) )
	{
		fprintf(stderr, "argument error: linearity_gain and sensitivity_gain are both set (choose only one option)\n");
		usage_static();
		return EXIT_FAILURE;
	}

	if(verbose == true)
	{
		uint32_t serial_number_msb_val;
		uint32_t serial_number_lsb_val;

		serial_number_msb_val = (uint32_t)(serial_number_val >> 32);
		serial_number_lsb_val = (uint32_t)(serial_number_val & 0xFFFFFFFF);
		if(serial_number)
			fprintf(stderr, "serial_number_64bits -s 0x%08X%08X\n", serial_number_msb_val, serial_number_lsb_val);
		fprintf(stderr, "packing -p %d\n", packing_val);
		fprintf(stderr, "frequency_MHz -f %sMHz (%sHz)\n",
			u64toa((freq_hz/FREQ_ONE_MHZ), &ascii_u64_data1),
			u64toa(freq_hz, &ascii_u64_data2) );
		fprintf(stderr, "sample_type -t %d\n", sample_type_val);
		fprintf(stderr, "biast -b %d\n", biast_val);

		if( (linearity_gain == false) && (sensitivity_gain == false) )
		{
			fprintf(stderr, "vga_gain -v %u\n", vga_gain);
			fprintf(stderr, "mixer_gain -m %u\n", mixer_gain);
			fprintf(stderr, "lna_gain -l %u\n", lna_gain);
		} else
		{
			if( linearity_gain == true)
			{
				fprintf(stderr, "linearity_gain -g %u\n", linearity_gain_val);
			}

			if( sensitivity_gain == true)
			{
				fprintf(stderr, "sensitivity_gain -h %u\n", sensitivity_gain_val);
			}
		}

		if( limit_num_samples ) {
			fprintf(stderr, "num_samples -n %s (%sM)\n",
					u64toa(samples_to_xfer, &ascii_u64_data1),
					u64toa((samples_to_xfer/FREQ_ONE_MHZ), &ascii_u64_data2));
		}
	}

	if(serial_number == true)
	{
		result = hydrasdr_open_sn(&device, serial_number_val);
		if( result != HYDRASDR_SUCCESS ) {
			fprintf(stderr, "hydrasdr_open_sn() failed: %s (%d)\n", hydrasdr_error_name(result), result);
			return EXIT_FAILURE;
		}
	}else
	{
		result = hydrasdr_open(&device);
		if( result != HYDRASDR_SUCCESS ) {
			fprintf(stderr, "hydrasdr_open() failed: %s (%d)\n", hydrasdr_error_name(result), result);
			return EXIT_FAILURE;
		}
	}

	/* Get device info early for capability-based configuration (v1.1.0+ API order) */
	result = hydrasdr_get_device_info(device, &device_info);
	if (result != HYDRASDR_SUCCESS) {
		fprintf(stderr, "hydrasdr_get_device_info() failed: %s (%d)\n",
			hydrasdr_error_name(result), result);
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}
	fprintf(stderr, "Device: %s\n", device_info.board_name);
	fprintf(stderr, "Firmware: %s\n", device_info.firmware_version);
	fprintf(stderr, "Serial Number: 0x%08X%08X\n",
		device_info.part_serial.serial_no[2],
		device_info.part_serial.serial_no[3]);

	result = hydrasdr_set_sample_type(device, sample_type_val);
	if (result != HYDRASDR_SUCCESS) {
		fprintf(stderr, "hydrasdr_set_sample_type() failed: %s (%d)\n", hydrasdr_error_name(result), result);
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}

	hydrasdr_get_samplerates(device, &count, 0);

	supported_samplerates = (uint32_t *) malloc(count * sizeof(uint32_t));
	hydrasdr_get_samplerates(device, supported_samplerates, count);

	if (sample_rate_val <= MIN_SAMPLERATE_BY_VALUE)
	{
		if (sample_rate_val < count)
		{
			wav_sample_per_sec = supported_samplerates[sample_rate_val];
		}
		else
		{
			free(supported_samplerates);
			fprintf(stderr, "argument error: unsupported sample rate\n");
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
	}
	else
	{
		wav_sample_per_sec = sample_rate_val;
	}

	free(supported_samplerates);

	/* Set decimation mode before sample rate (v1.1.0+ API) */
	if (decimation_mode) {
		result = hydrasdr_set_decimation_mode(device, (enum hydrasdr_decimation_mode)decimation_mode_val);
		if (result != HYDRASDR_SUCCESS) {
			fprintf(stderr, "hydrasdr_set_decimation_mode() failed: %s (%d)\n", hydrasdr_error_name(result), result);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
		if (verbose) {
			fprintf(stderr, "Decimation mode: %s\n",
				decimation_mode_val ? "High Definition (max HW rate + decimation)" : "Low Bandwidth (min HW rate)");
		}
	}

	/* Set bandwidth before sample rate (v1.1.0+ API order for auto-bandwidth) */
	if (device_info.features & HYDRASDR_CAP_BANDWIDTH) {
		uint32_t bw_count = 0;
		uint32_t *supported_bandwidths = NULL;

		/* Query available bandwidths */
		hydrasdr_get_bandwidths(device, &bw_count, 0);
		if (bw_count > 0) {
			supported_bandwidths = (uint32_t *)malloc(bw_count * sizeof(uint32_t));
			if (supported_bandwidths) {
				hydrasdr_get_bandwidths(device, supported_bandwidths, bw_count);

				if (verbose) {
					fprintf(stderr, "Supported bandwidths:\n");
					for (uint32_t i = 0; i < bw_count; i++) {
						fprintf(stderr, "  [%u] %u Hz (%.3f MHz)\n", i,
							supported_bandwidths[i],
							supported_bandwidths[i] / 1e6);
					}
				}

				/* Set bandwidth if user specified one */
				if (bandwidth) {
					/* Resolve bandwidth: support both index and direct Hz value */
					uint32_t bw_to_set = bandwidth_val;
					bool found = false;
					for (uint32_t i = 0; i < bw_count; i++) {
						if (bandwidth_val == supported_bandwidths[i]) {
							found = true;
							break;
						}
					}
					/* If not found and value is small, treat as index */
					if (!found && bandwidth_val < bw_count) {
						bw_to_set = supported_bandwidths[bandwidth_val];
						if (verbose) {
							fprintf(stderr, "Interpreting -B %u as index -> %u Hz\n",
								bandwidth_val, bw_to_set);
						}
					}

					result = hydrasdr_set_bandwidth(device, bw_to_set);
					if (result != HYDRASDR_SUCCESS) {
						fprintf(stderr, "hydrasdr_set_bandwidth() failed: %s (%d)\n",
							hydrasdr_error_name(result), result);
						free(supported_bandwidths);
						hydrasdr_close(device);
						return EXIT_FAILURE;
					}
					if (verbose) {
						fprintf(stderr, "Bandwidth set to %u Hz (%.3f MHz)\n",
							bw_to_set, bw_to_set / 1e6);
					}
				}
				free(supported_bandwidths);
			}
		}
	} else if (bandwidth) {
		fprintf(stderr, "Warning: -B (Bandwidth) not supported on this device, ignored\n");
	}

	result = hydrasdr_set_samplerate(device, sample_rate_val);
	if (result != HYDRASDR_SUCCESS) {
		fprintf(stderr, "hydrasdr_set_samplerate() failed: %s (%d)\n", hydrasdr_error_name(result), result);
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}

	if (verbose)
	{
		fprintf(stderr, "sample_rate -a %d (%f MSPS %s)\n", sample_rate_val, (wav_sample_per_sec * 0.000001f), wav_nb_channels == 1 ? "Real" : "IQ");
	}

	/* === Apply defaults from device_info === */

	/* Set default frequency if not specified by user */
	if( !freq ) {
		/* Use device's min_frequency as default (start of valid range) */
		freq_hz = device_info.min_frequency;
	}

	/* Set default gains from device_info if not explicitly set by user */
	if (!vga_gain_set && (device_info.features & HYDRASDR_CAP_VGA_GAIN)) {
		vga_gain = (uint32_t)device_info.vga_gain.default_value;
	}
	if (!lna_gain_set && (device_info.features & HYDRASDR_CAP_LNA_GAIN)) {
		lna_gain = (uint32_t)device_info.lna_gain.default_value;
	}
	if (!mixer_gain_set && (device_info.features & HYDRASDR_CAP_MIXER_GAIN)) {
		mixer_gain = (uint32_t)device_info.mixer_gain.default_value;
	}
	if (!rf_gain_set && (device_info.features & HYDRASDR_CAP_RF_GAIN)) {
		rf_gain = (uint32_t)device_info.rf_gain.default_value;
	}
	if (!filter_gain_set && (device_info.features & HYDRASDR_CAP_FILTER_GAIN)) {
		filter_gain = (uint32_t)device_info.filter_gain.default_value;
	}

	/* Validate sample type against device supported types */
	if (sample_type && !(device_info.sample_types & (1 << sample_type_val))) {
		fprintf(stderr, "argument error: sample_type %d (%s) not supported by this device\n",
			sample_type_val, sample_type_name(sample_type_val));
		fprintf(stderr, "Supported sample types: ");
		for (uint8_t i = 0; i < HYDRASDR_SAMPLE_END; i++) {
			if (device_info.sample_types & (1 << i)) {
				fprintf(stderr, "%d=%s ", i, sample_type_name((enum hydrasdr_sample_type)i));
			}
		}
		fprintf(stderr, "\n");
		usage_dynamic(&device_info);
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}

	/* Set default RF port (first port) or validate user's choice */
	if (rf_port) {
		if ((device_info.features & HYDRASDR_CAP_RF_PORT_SELECT) && rf_port_val >= device_info.rf_port_count) {
			fprintf(stderr, "argument error: rf_port %u out of range [0-%d]\n",
				rf_port_val, device_info.rf_port_count - 1);
			usage_dynamic(&device_info);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
	} else {
		/* Default to first port (port 0) */
		rf_port_val = 0;
	}

	/* Generate WAV filename now that we have the frequency */
	if( receive_wav ) {
		time (&rawtime);
		timeinfo = localtime (&rawtime);
		strftime(date_time, DATE_TIME_MAX_LEN, "%Y%m%d_%H%M%S", timeinfo);
		snprintf(path_file, PATH_FILE_MAX_LEN, "HydraSDR_%uHz_%s_%s_%s.wav",
				 (uint32_t)freq_hz, date_time, sample_type_str, channels_str);
		path = path_file;
		fprintf(stderr, "Receive wav file: %s\n", path);
	}

	/* === Dynamic validation based on device capabilities === */

	/* Validate frequency against actual device limits */
	if( freq ) {
		if( (freq_hz > device_info.max_frequency) || (freq_hz < device_info.min_frequency) )
		{
			fprintf(stderr, "argument error: frequency_MHz=%.1f MHz out of device range [%.1f, %.1f] MHz\n",
							(double)freq_hz / 1e6,
							(double)device_info.min_frequency / 1e6,
							(double)device_info.max_frequency / 1e6);
			usage_dynamic(&device_info);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
	}

	/* Validate gains against actual device limits */
	if (vga_gain_set && (device_info.features & HYDRASDR_CAP_VGA_GAIN)) {
		if (vga_gain < device_info.vga_gain.min_value || vga_gain > device_info.vga_gain.max_value) {
			fprintf(stderr, "argument error: vga_gain %u out of range [%d-%d]\n",
				vga_gain, (int)device_info.vga_gain.min_value, (int)device_info.vga_gain.max_value);
			usage_dynamic(&device_info);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
	}
	if (lna_gain_set && (device_info.features & HYDRASDR_CAP_LNA_GAIN)) {
		if (lna_gain < device_info.lna_gain.min_value || lna_gain > device_info.lna_gain.max_value) {
			fprintf(stderr, "argument error: lna_gain %u out of range [%d-%d]\n",
				lna_gain, (int)device_info.lna_gain.min_value, (int)device_info.lna_gain.max_value);
			usage_dynamic(&device_info);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
	}
	if (mixer_gain_set && (device_info.features & HYDRASDR_CAP_MIXER_GAIN)) {
		if (mixer_gain < device_info.mixer_gain.min_value || mixer_gain > device_info.mixer_gain.max_value) {
			fprintf(stderr, "argument error: mixer_gain %u out of range [%d-%d]\n",
				mixer_gain, (int)device_info.mixer_gain.min_value, (int)device_info.mixer_gain.max_value);
			usage_dynamic(&device_info);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
	}
	if (rf_gain_set && (device_info.features & HYDRASDR_CAP_RF_GAIN)) {
		if (rf_gain < device_info.rf_gain.min_value || rf_gain > device_info.rf_gain.max_value) {
			fprintf(stderr, "argument error: rf_gain %u out of range [%d-%d]\n",
				rf_gain, (int)device_info.rf_gain.min_value, (int)device_info.rf_gain.max_value);
			usage_dynamic(&device_info);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
	}
	if (filter_gain_set && (device_info.features & HYDRASDR_CAP_FILTER_GAIN)) {
		if (filter_gain < device_info.filter_gain.min_value || filter_gain > device_info.filter_gain.max_value) {
			fprintf(stderr, "argument error: filter_gain %u out of range [%d-%d]\n",
				filter_gain, (int)device_info.filter_gain.min_value, (int)device_info.filter_gain.max_value);
			usage_dynamic(&device_info);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
	}
	if (linearity_gain && (device_info.features & HYDRASDR_CAP_LINEARITY_GAIN)) {
		if (linearity_gain_val < device_info.linearity_gain.min_value || linearity_gain_val > device_info.linearity_gain.max_value) {
			fprintf(stderr, "argument error: linearity_gain %u out of range [%d-%d]\n",
				linearity_gain_val, (int)device_info.linearity_gain.min_value, (int)device_info.linearity_gain.max_value);
			usage_dynamic(&device_info);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
	}
	if (sensitivity_gain && (device_info.features & HYDRASDR_CAP_SENSITIVITY_GAIN)) {
		if (sensitivity_gain_val < device_info.sensitivity_gain.min_value || sensitivity_gain_val > device_info.sensitivity_gain.max_value) {
			fprintf(stderr, "argument error: sensitivity_gain %u out of range [%d-%d]\n",
				sensitivity_gain_val, (int)device_info.sensitivity_gain.min_value, (int)device_info.sensitivity_gain.max_value);
			usage_dynamic(&device_info);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
	}

	/* Warn about gain options used on unsupported devices */
	if (vga_gain_set && !(device_info.features & HYDRASDR_CAP_VGA_GAIN)) {
		fprintf(stderr, "Warning: -v (VGA gain) not supported on this device, ignored\n");
	}
	if (lna_gain_set && !(device_info.features & HYDRASDR_CAP_LNA_GAIN)) {
		fprintf(stderr, "Warning: -l (LNA gain) not supported on this device, ignored\n");
	}
	if (mixer_gain_set && !(device_info.features & HYDRASDR_CAP_MIXER_GAIN)) {
		fprintf(stderr, "Warning: -m (Mixer gain) not supported on this device, ignored\n");
	}
	if (rf_gain_set && !(device_info.features & HYDRASDR_CAP_RF_GAIN)) {
		fprintf(stderr, "Warning: -R (RF gain) not supported on this device, ignored\n");
	}
	if (filter_gain_set && !(device_info.features & HYDRASDR_CAP_FILTER_GAIN)) {
		fprintf(stderr, "Warning: -F (Filter gain) not supported on this device, ignored\n");
	}

	if( call_set_packing == true )
	{
		result = hydrasdr_set_packing(device, packing_val);
		if( result != HYDRASDR_SUCCESS ) {
			fprintf(stderr, "hydrasdr_set_packing() failed: %s (%d)\n", hydrasdr_error_name(result), result);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
	}

	result = hydrasdr_set_rf_bias(device, biast_val);
	if( result != HYDRASDR_SUCCESS ) {
		fprintf(stderr, "hydrasdr_set_rf_bias() failed: %s (%d)\n", hydrasdr_error_name(result), result);
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}

	/* Set RF port if supported */
	if (device_info.features & HYDRASDR_CAP_RF_PORT_SELECT) {
		result = hydrasdr_set_rf_port(device, (hydrasdr_rf_port_t)rf_port_val);
		if( result != HYDRASDR_SUCCESS ) {
			fprintf(stderr, "hydrasdr_set_rf_port() failed: %s (%d)\n", hydrasdr_error_name(result), result);
			hydrasdr_close(device);
			return EXIT_FAILURE;
		}
		if (verbose) {
			fprintf(stderr, "RF port set to %d (%s)\n", rf_port_val,
				device_info.rf_port_info[rf_port_val].name);
		}
	}

	if (!strcmp(path,"-"))
		fd = stdout;
	else
		fd = fopen(path, "wb");
	if( fd == NULL ) {
		fprintf(stderr, "Failed to open file: %s\n", path);
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}
	/* Change fd buffer to have bigger one to store data to file */
	result = setvbuf(fd , NULL , _IOFBF , FD_BUFFER_SIZE);
	if( result != 0 ) {
		fprintf(stderr, "setvbuf() failed: %d\n", result);
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}
	
	/* Write Wav header */
	if( receive_wav ) 
	{
		fwrite(&wave_file_hdr, 1, sizeof(t_wav_file_hdr), fd);
	}
	
#ifdef _MSC_VER
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#else
	signal(SIGINT, &sigint_callback_handler);
	signal(SIGILL, &sigint_callback_handler);
	signal(SIGFPE, &sigint_callback_handler);
	signal(SIGSEGV, &sigint_callback_handler);
	signal(SIGTERM, &sigint_callback_handler);
	signal(SIGABRT, &sigint_callback_handler);
#endif

	if( (linearity_gain == false) && (sensitivity_gain == false) )
	{
		/* VGA gain */
		if (device_info.features & HYDRASDR_CAP_VGA_GAIN) {
			result = hydrasdr_set_gain(device, HYDRASDR_GAIN_TYPE_VGA, vga_gain);
			if( result != HYDRASDR_SUCCESS ) {
				fprintf(stderr, "hydrasdr_set_gain(VGA) failed: %s (%d)\n", hydrasdr_error_name(result), result);
			}
		}

		/* Mixer gain */
		if (device_info.features & HYDRASDR_CAP_MIXER_GAIN) {
			result = hydrasdr_set_gain(device, HYDRASDR_GAIN_TYPE_MIXER, mixer_gain);
			if( result != HYDRASDR_SUCCESS ) {
				fprintf(stderr, "hydrasdr_set_gain(MIXER) failed: %s (%d)\n", hydrasdr_error_name(result), result);
			}
		}

		/* LNA gain */
		if (device_info.features & HYDRASDR_CAP_LNA_GAIN) {
			result = hydrasdr_set_gain(device, HYDRASDR_GAIN_TYPE_LNA, lna_gain);
			if( result != HYDRASDR_SUCCESS ) {
				fprintf(stderr, "hydrasdr_set_gain(LNA) failed: %s (%d)\n", hydrasdr_error_name(result), result);
			}
		}

		/* RF gain (if supported - uses device default if not explicitly set) */
		if (device_info.features & HYDRASDR_CAP_RF_GAIN) {
			result = hydrasdr_set_gain(device, HYDRASDR_GAIN_TYPE_RF, rf_gain);
			if( result != HYDRASDR_SUCCESS ) {
				fprintf(stderr, "hydrasdr_set_gain(RF) failed: %s (%d)\n", hydrasdr_error_name(result), result);
			}
		}

		/* Filter gain (if supported - uses device default if not explicitly set) */
		if (device_info.features & HYDRASDR_CAP_FILTER_GAIN) {
			result = hydrasdr_set_gain(device, HYDRASDR_GAIN_TYPE_FILTER, filter_gain);
			if( result != HYDRASDR_SUCCESS ) {
				fprintf(stderr, "hydrasdr_set_gain(FILTER) failed: %s (%d)\n", hydrasdr_error_name(result), result);
			}
		}
	} else
	{
		if( linearity_gain == true )
		{
			if (device_info.features & HYDRASDR_CAP_LINEARITY_GAIN) {
				result = hydrasdr_set_gain(device, HYDRASDR_GAIN_TYPE_LINEARITY, linearity_gain_val);
				if( result != HYDRASDR_SUCCESS ) {
					fprintf(stderr, "hydrasdr_set_gain(LINEARITY) failed: %s (%d)\n", hydrasdr_error_name(result), result);
				}
			} else {
				fprintf(stderr, "Warning: Linearity gain not supported on this device\n");
			}
		}

		if( sensitivity_gain == true )
		{
			if (device_info.features & HYDRASDR_CAP_SENSITIVITY_GAIN) {
				result = hydrasdr_set_gain(device, HYDRASDR_GAIN_TYPE_SENSITIVITY, sensitivity_gain_val);
				if( result != HYDRASDR_SUCCESS ) {
					fprintf(stderr, "hydrasdr_set_gain(SENSITIVITY) failed: %s (%d)\n", hydrasdr_error_name(result), result);
				}
			} else {
				fprintf(stderr, "Warning: Sensitivity gain not supported on this device\n");
			}
		}
	}

	result = hydrasdr_start_rx(device, rx_callback, NULL);
	if( result != HYDRASDR_SUCCESS ) {
		fprintf(stderr, "hydrasdr_start_rx() failed: %s (%d)\n", hydrasdr_error_name(result), result);
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}

	result = hydrasdr_set_freq(device, freq_hz);
	if( result != HYDRASDR_SUCCESS ) {
		fprintf(stderr, "hydrasdr_set_freq() failed: %s (%d)\n", hydrasdr_error_name(result), result);
		hydrasdr_close(device);
		return EXIT_FAILURE;
	}

	fprintf(stderr, "Stop with Ctrl-C\n");

	average_rate = (float) wav_sample_per_sec;

	sleep(1);

	while( (hydrasdr_is_streaming(device) == HYDRASDR_TRUE) &&
		(do_exit == false) )
	{
		float average_rate_now = average_rate * 1e-6f;
		snprintf(str, sizeof(str), "%2.3f", average_rate_now);

		/* Read and display temperature if supported */
		if (device_info.features & HYDRASDR_CAP_TEMPERATURE_SENSOR) {
			hydrasdr_temperature_t temp;
			if (hydrasdr_get_temperature(device, &temp) == HYDRASDR_SUCCESS && temp.valid) {
				/* Update statistics with timestamps */
				if (temp.temperature_celsius < temp_min_celsius) {
					temp_min_celsius = temp.temperature_celsius;
					time(&temp_min_time);
				}
				if (temp.temperature_celsius > temp_max_celsius) {
					temp_max_celsius = temp.temperature_celsius;
					time(&temp_max_time);
				}
				temp_sample_count++;

				fprintf(stderr, "Streaming at %5s MSPS, Temp: %.2f DegC\n", str, temp.temperature_celsius);
			} else {
				fprintf(stderr, "Streaming at %5s MSPS\n", str);
			}
		} else {
			fprintf(stderr, "Streaming at %5s MSPS\n", str);
		}

		if ((limit_num_samples == true) && (bytes_to_xfer == 0))
			do_exit = true;
		else
			sleep(1);
	}
	
	result = hydrasdr_is_streaming(device);	
	if (do_exit)
	{
		fprintf(stderr, "\nUser cancel, exiting...\n");
	} else {
		fprintf(stderr, "\nExiting...\n");
	}
	
	gettimeofday(&t_end, NULL);
	time_diff = TimevalDiff(&t_end, &t_start);
	fprintf(stderr, "Total time: %5.4f s\n", time_diff);
	if (rate_samples > 0)
	{
		fprintf(stderr, "Average speed %2.4f MSPS %s\n", (global_average_rate * 1e-6f / rate_samples), (wav_nb_channels == 2 ? "IQ" : "Real"));
	}

	/* Report dropped samples (buffer overflows) */
	if (total_dropped_samples > 0)
	{
		fprintf(stderr, "WARNING: %s samples dropped (buffer overflow)\n",
			u64toa(total_dropped_samples, &ascii_u64_data1));
	}
	else
	{
		fprintf(stderr, "No samples dropped\n");
	}

	/* Report temperature statistics if supported */
	if ((device_info.features & HYDRASDR_CAP_TEMPERATURE_SENSOR) && temp_sample_count > 0)
	{
		float temp_min_f = (temp_min_celsius * 9.0f / 5.0f) + 32.0f;
		float temp_max_f = (temp_max_celsius * 9.0f / 5.0f) + 32.0f;
		char min_time_str[32], max_time_str[32];
		double duration_sec = difftime(temp_max_time, temp_min_time);

		strftime(min_time_str, sizeof(min_time_str), "%H:%M:%S", localtime(&temp_min_time));
		strftime(max_time_str, sizeof(max_time_str), "%H:%M:%S", localtime(&temp_max_time));

		fprintf(stderr, "Temperature: Min %.2f DegC (%.2f DegF) at %s, Max %.2f DegC (%.2f DegF) at %s (%.0fs)\n",
			temp_min_celsius, temp_min_f, min_time_str,
			temp_max_celsius, temp_max_f, max_time_str,
			duration_sec >= 0 ? duration_sec : -duration_sec);
	}

	/* Stop device and close */
	if(device != NULL)
	{
		result = hydrasdr_stop_rx(device);
		if( result != HYDRASDR_SUCCESS ) {
			fprintf(stderr, "hydrasdr_stop_rx() failed: %s (%d)\n", hydrasdr_error_name(result), result);
		}

		result = hydrasdr_close(device);
		if( result != HYDRASDR_SUCCESS )
		{
			fprintf(stderr, "hydrasdr_close() failed: %s (%d)\n", hydrasdr_error_name(result), result);
		}
	}

	/* Finalize WAV file header */
	if(fd != NULL)
	{
		if( receive_wav )
		{
			int seek_result;
			size_t write_result;

			/* Flush all pending writes before getting file position */
			fflush(fd);
			file_pos = ftell(fd);

			/* Update WAV header with final sizes */
			wave_file_hdr.hdr.size = file_pos - 8;
			wave_file_hdr.fmt_chunk.wFormatTag = wav_format_tag;
			wave_file_hdr.fmt_chunk.wChannels = wav_nb_channels;
			wave_file_hdr.fmt_chunk.dwSamplesPerSec = wav_sample_per_sec;
			wave_file_hdr.fmt_chunk.dwAvgBytesPerSec = wave_file_hdr.fmt_chunk.dwSamplesPerSec * wav_nb_byte_per_sample;
			wave_file_hdr.fmt_chunk.wBlockAlign = wav_nb_channels * (wav_nb_bits_per_sample / 8);
			wave_file_hdr.fmt_chunk.wBitsPerSample = wav_nb_bits_per_sample;
			wave_file_hdr.data_chunk.chunkSize = file_pos - sizeof(t_wav_file_hdr);

			/* Overwrite header with updated data */
			fflush(fd);
			seek_result = fseek(fd, 0, SEEK_SET);
			if (seek_result != 0) {
				fprintf(stderr, "ERROR: fseek to beginning failed: %d\n", seek_result);
			}
			write_result = fwrite(&wave_file_hdr, 1, sizeof(t_wav_file_hdr), fd);
			if (write_result != sizeof(t_wav_file_hdr)) {
				fprintf(stderr, "ERROR: WAV header write failed: wrote %zu of %zu bytes\n",
					write_result, sizeof(t_wav_file_hdr));
			}
			fflush(fd);
		}
		fclose(fd);
		fd = NULL;
	}
	fprintf(stderr, "done\n");
	return exit_code;
}
