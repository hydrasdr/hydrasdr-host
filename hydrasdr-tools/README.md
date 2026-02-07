# hydrasdr-tools

Command-line utilities for HydraSDR Software Defined Radio devices.

## Overview

hydrasdr-tools provides a comprehensive set of command-line utilities for
controlling and interacting with HydraSDR hardware.

## Tools

| Tool | Description |
|------|-------------|
| `hydrasdr_rx` | Receive and record RF samples to file |
| `hydrasdr_async_rx` | Asynchronous RF receiver with streaming |
| `hydrasdr_info` | Display device information and capabilities |
| `hydrasdr_list_devices` | List connected HydraSDR devices |
| `hydrasdr_lib_version` | Display library version |
| `hydrasdr_calibrate` | Frequency calibration utility |
| `hydrasdr_clockgen` | Clock generator configuration |
| `hydrasdr_gpio` | GPIO pin control |
| `hydrasdr_gpiodir` | GPIO direction configuration |
| `hydrasdr_reset` | Reset device |
| `hydrasdr_rf_frontend` | RF frontend configuration |
| `hydrasdr_set_rf_port` | RF port selection |
| `hydrasdr_spiflash` | SPI flash programming |
| `hydrasdr_r82x` | R82xx tuner configuration |
| `hydrasdr_si5351c` | Si5351C clock generator control |
| `hydrasdr_ddc_benchmark` | DDC algorithm benchmarking |
| `hydrasdr_api_benchmark` | API performance benchmarking |

## Building

See the parent directory README.md for complete build instructions.

### Quick Start (Linux/macOS)

```bash
mkdir build && cd build
cmake ..
make
```

### Quick Start (Windows with MSYS2/MinGW64)

```bash
mkdir build && cd build
cmake .. -G "Ninja"
ninja
```

## Documentation

- [Command Line Tools Reference](hydrasdr_tools_readme.md) - Complete usage guide
- [Frequency Calibration Procedure](hydrasdr_calibration_procedure.md) - PPB calibration
- [Troubleshooting Guide](hydrasdr_troubleshooting.md) - Performance optimization

## License

GNU General Public License v2.0 (GPL-2.0)

This software is based on the HackRF project. See [LICENSE.md](LICENSE.md) for details.

## Links

- HydraSDR: https://www.hydrasdr.com
- HackRF Project: https://greatscottgadgets.com/hackrf/
