# HydraSDR Command Line Tools

This collection provides command line utilities for controlling and managing HydraSDR devices.
Each tool serves a specific purpose for device configuration, calibration, data capture, and low-level hardware access.

## Build Requirements

- CMake 3.10 or higher
- C compiler (GCC, Clang, or Visual Studio 2022)
- libhydrasdr development libraries

### Build Instructions

#### Linux/MinGW64:
```bash
mkdir build
cd build
cmake ..
make
```

#### Visual Studio 2022 (x64):
```bash
mkdir build
cd build
cmake -G "Visual Studio 17 2022" -A x64 ..
cmake --build . --config Release
```

## HydraSDR Tools Documentation

### hydrasdr_calibrate
**Purpose**: Purpose: Read / Erase and Write calibration data to SPI flash memory.

**Usage**:
```bash
hydrasdr_calibrate -r                      # Read and display calibration data.
hydrasdr_calibrate -w <calibration in ppb> # Erase and Write calibration in ppb.
```

#### Frequency correction to ppb algorithm
* `calibration in ppb = ((actual_freq_MHz - observed_freq_MHz) / actual_freq_MHz) * 1e9`

### hydrasdr_gpio
**Purpose**: Control GPIO pins for hardware interfacing and LED control.

**Usage**:
```bash
hydrasdr_gpio -p <port> -n <pin> -r           # Read pin value
hydrasdr_gpio -p <port> -n <pin> -w <value>   # Write pin value
hydrasdr_gpio -r                              # Read all pins
```

**Parameters**:
- `-p`: Port number (0-7)
- `-n`: Pin number (0-31)
- `-w`: Value to write (0 or 1)
- `-s`: Serial number for multi-device setups

**Hardware Mappings**:
- LED1: Port 0, Pin 12 (0=OFF, 1=ON)
- Enable R828D: Port 1, Pin 7 (0=OFF, 1=ON)
- Enable BiasT: Port 1, Pin 13 (0=OFF, 1=ON)

### hydrasdr_gpiodir
**Purpose**: Configure GPIO pin directions (input/output).

**Usage**:
```bash
hydrasdr_gpiodir -p <port> -n <pin> -r        # Read pin direction
hydrasdr_gpiodir -p <port> -n <pin> -w <dir>  # Set pin direction
```

**Parameters**:
- `-w`: Direction (0=input, 1=output)

**Warning**: Improper GPIO direction configuration can damage hardware.

### hydrasdr_info
**Purpose**: Display comprehensive device information and capabilities.

**Usage**:
```bash
hydrasdr_info                    # Show info for first device
hydrasdr_info -s <serial_number> # Show info for specific device
```

**Information Displayed**:
- Library version
- Board ID and firmware version
- Part ID and serial number
- Supported sample rates

### hydrasdr_lib_version
**Purpose**: Display library version information.

**Usage**:
```bash
hydrasdr_lib_version
```

**Output**: Shows major.minor.revision version of the HydraSDR library.

### hydrasdr_r82x
**Purpose**: Direct register access for R82x tuner configuration.

**Usage**:
```bash
hydrasdr_r82x -n <register> -r        # Read register
hydrasdr_r82x -n <register> -w <val>  # Write register
hydrasdr_r82x -r                      # Read all registers
hydrasdr_r82x -c                      # Apply default configuration
```

**Parameters**:
- `-n`: Register number (0-31)
- `-w`: Value to write (0-255)
- `-c`: Configure with predefined values for testing

### hydrasdr_reset
**Purpose**: Reset HydraSDR devices.

**Usage**:
```bash
hydrasdr_reset                    # Reset all connected devices
hydrasdr_reset -s <serial_number> # Reset specific device
```

**Description**: Performs hardware reset on specified device or all connected devices.

### hydrasdr_rx
**Purpose**: Receive and capture RF data with comprehensive configuration options.

**Usage**:
```bash
hydrasdr_rx -r <filename> [options]  # Capture to file
hydrasdr_rx -w [options]             # Capture to WAV file (SDR++ format)
```

**Key Parameters**:
- `-f <freq_MHz>`: Set frequency (24-1850 MHz, default 900 MHz)
- `-a <rate>`: Sample rate
- `-t <type>`: Sample type (0=FLOAT32_IQ, 1=FLOAT32_REAL, 2=INT16_IQ, 3=INT16_REAL, 4=U16_REAL, 5=RAW)
- `-v <gain>`: VGA/IF gain (0-15, default 5)
- `-m <gain>`: Mixer gain (0-15, default 5)
- `-l <gain>`: LNA gain (0-14, default 1)
- `-g <gain>`: Linearity gain (0-21)
- `-h <gain>`: Sensitivity gain (0-21)
- `-b <val>`: Bias Tee (0=disabled, 1=enabled)
- `-p <val>`: Packing (0=16-bit, 1=12-bit packed, 2=12-bit packed with a chunk counter, 3=8-bit)
- `-H <val>`: Framed chunks with sample counters (1=enabled, default when the firmware supports it; 0=disabled)
- `-S <div>`: Tag the USB Start-Of-Frame of every frame (1 ms) whose number is a multiple of `div` with its sample index (0=off)
- `-U <baud>`: Baud rate of the auxiliary UART (e.g. a GNSS module); `-N 1` prints the bytes it receives
- `-C <ppb>`: Crystal correction in ppb, until the device resets
- `-W 1`: Feed the device watchdog every second and show its state
- `-n <samples>`: Limit number of samples
- `-d`: Verbose mode

**Sample Types**:
- 0: 32-bit float IQ
- 1: 32-bit float real
- 2: 16-bit integer IQ (default)
- 3: 16-bit integer real
- 4: 16-bit unsigned real
- 5: Raw ADC data

**WAV Mode**: Creates SDR++ compatible WAV files with automatic timestamped filenames in format: HydraSDR_<freq_Hz>_<YYYYMMDD>_<HHMMSS>_<sample_type>_<channels>.wav

With framing (firmware with the stream metadata commands), `hydrasdr_rx` also prints once per second the device ring counters, sample index and gap count, the last PPS edge and USB SOF tag.

### hydrasdr_debug
**Purpose**: Read and write device memory, and call device functions, for firmware debugging.

**Usage**:
```bash
hydrasdr_debug read <addr> [words]            # Read 32-bit words
hydrasdr_debug bytes <addr> [count]           # Read bytes
hydrasdr_debug write <addr> <word>...         # Write 32-bit words
hydrasdr_debug call <core> <addr> [r0..r3]    # Call a function on the M0 (0) or M4 (1)
```

### hydrasdr_set_rf_port
**Purpose**: Configure RF input port selection.

**Usage**:
```bash
hydrasdr_set_rf_port -p <port>        # Set RF port
hydrasdr_set_rf_port -p <port> -s <serial> # Set port on specific device
```

**Parameters**:
- `-p`: RF port number (0-2)

### hydrasdr_si5351c
**Purpose**: Direct access to Si5351C clock generator registers.

**Usage**:
```bash
hydrasdr_si5351c -n <reg> -r          # Read register
hydrasdr_si5351c -n <reg> -w <val>    # Write register
hydrasdr_si5351c -r                   # Read all registers
hydrasdr_si5351c -c                   # Show clock configuration
```

**Parameters**:
- `-n`: Register number (0-255)
- `-c`: Display multisynth configuration and output frequencies

### hydrasdr_spiflash
**Purpose**: Read / Write SPI flash memory for firmware and data storage.

**Usage**:
```bash
hydrasdr_spiflash -r <file> -a <addr> -l <len>  # Read from flash
hydrasdr_spiflash -w <file> -a <addr>           # Write to flash
```

**Parameters**:
- `-a <addr>`: Starting address (default 0)
- `-l <len>`: Number of bytes to read
- `-r <file>`: Read data to file
- `-w <file>`: Write data from file

**Memory Layout**:
- Flash size: 8 Mbit (1 MB)
- Firmware area: 0x00000-0x1FFFF (128 KB)
- Calibration area: 0x20000+ (sector 2)

## Common Parameters

Most tools support:
- `-s <serial_number>`: Target specific device by 64-bit serial number (hex format)
- `-d`: Enable verbose/debug output (where applicable)

## Examples

**Basic RF capture**:
```bash
hydrasdr_rx -f 100 -a 2000000 -r capture.bin
```

**WAV file capture with custom gains**:
```bash
hydrasdr_rx -w -f 103.4 -v 10 -l 5 -t 2  # Creates: HydraSDR_103400000Hz_20250613_194233_int16_IQ.wav
```

**Read device information**:
```bash
hydrasdr_info
```

**Control LED**:
```bash
hydrasdr_gpio -p 0 -n 12 -w 1  # Turn LED on
hydrasdr_gpio -p 0 -n 12 -w 0  # Turn LED off
```

**Enable Bias Tee**:
```bash
hydrasdr_gpio -p 1 -n 13 -w 1  # Enable
hydrasdr_gpio -p 1 -n 13 -w 0  # Disable
```

## Notes

- All frequency values are in Hz unless specified otherwise
- GPIO operations require careful attention to avoid hardware damage
- Flash operations can brick the device if firmware area is corrupted
- Use appropriate gain settings to avoid saturation or poor SNR
- WAV files are compatible with SDR++ and similar software
