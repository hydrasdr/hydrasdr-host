HydraSDR Software Defined Radio Host Tools + Shared Lib / DLL (Windows GNU/Linux MacOS) 
==============

A tiny, efficient and ultra evolutive software defined radio.

This repository contains host software (Linux/Windows/MacOS) for HydraSDR RFOne, a project to
produce a low cost and extensible open source software radio platform.

HydraSDR: https://www.hydrasdr.com

### Warnings about libusb v1.0.24 on Windows & GNU/Linux (use newer version libusb 1.0.25 or latest version)
It is not recommended to use libusb v1.0.24 especially on Windows as there is tons of known issues it is why we are using libusb v1.0.23
See all major issues affecting libusb v1.0.24:
* https://github.com/libusb/libusb/issues/850 Regression in libusb 1.0.24 under Linux -- buggy devices with configuration 0 #850 
* https://github.com/libusb/libusb/issues/858 Regression in 1.0.24: libusb_handle_events_timeout_completed not waiting on timeout #858 
* https://github.com/libusb/libusb/issues/913 Regression in 1.0.24: libusb 1.0.24 uses a lot of cpu in libusb_bulk_transfer under Linux #913 
* https://github.com/libusb/libusb/issues/994 1.0.24 - windows_exit hangs when exiting on with WinUSB #994 
* https://github.com/libusb/libusb/issues/982 libusb_set_interface_alt_setting(...) crashes in 1.0.24 in Win10. #982
For more details see https://github.com/libusb/libusb/issues & https://github.com/libusb/libusb/issues/777

## How to build hydrasdr-host software on Windows:

### Prerequisites for MSYS2/MinGW64:

* cmake-3.10 or more see http://www.cmake.org/cmake/resources/software.html
* libusb-1.0.23 see https://github.com/libusb/libusb/releases/download/v1.0.23/libusb-1.0.23.7z
* For Windows OS less than Vista Install Windows driver for HydraSDR hardware or use Zadig see http://sourceforge.net/projects/libwdi/files/zadig
  - If you want to use Zadig  select HydraSDR USB device and just install/replace it with WinUSB driver.

>**Note for Windows build:**
 You shall always execute hydrasdr_tools from Windows command shell and not from Cygwin or Mingw shell because on Cygwin/Mingw
 Ctrl C is not managed correctly and especially for hydrasdr_rx the Ctrl C(abort) will not stop correctly and will corrupt the file.

### For VisualStudio 2019 or later:

* `git clone https://github.com/hydrasdr/hydrasdr-host.git hydrasdr-host`
* Download https://github.com/libusb/libusb/releases/download/v1.0.23/libusb-1.0.23.7z
* Extract **libusb-1.0.23.7z** to hydrasdr-host directory
  * You should have **hydrasdr-host\libusb-1.0.23**
    * Inside this directory there is README.txt, libusb-1.0.def and some directories
* Download https://gcc.gnu.org/pub/pthreads-win32/pthreads-w32-2-9-1-release.zip
* Extract **pthreads-w32-2-9-1-release.zip** to hydrasdr-host directory **libpthread-2-9-1-win**
  * You should have **hydrasdr-host\libpthread-2-9-1-win**
    * Inside this directory there is lot of README files and some directories (dll, include, lib)
* Launch hydrasdr-host\libhydrasdr\vc\hydrasdr_2019.sln with VisualStudio 2019, choose **Release** and **Build Solution**

### For MSYS2/MinGW64:
Launch mingw64

`pacman -Syu`

`pacman -S cmake mingw-w64-x86_64-libusb mingw-w64-x86_64-ninja`

`git clone https://github.com/hydrasdr/hydrasdr-host.git hydrasdr-host`

`cd hydrasdr-host`

`mkdir build`

`cd build`

`cmake ../ -G "Ninja" -DLIBUSB_INCLUDE_DIR=/mingw64/include/libusb-1.0/`

`cmake --build . --config Release`

For a release build with install (need need administrative privileges):
`cmake --build . --config Release --target install`

For a specific build type (e.g., Debug):
`cmake --build . --config Debug`

## How to build the hydrasdr-host software on Linux:

### Prerequisites for Linux (Debian/Ubuntu/Raspbian):


`sudo apt-get install build-essential cmake libusb-1.0-0-dev pkg-config git`


### Build hydrasdr-host software on Linux:

`git clone https://github.com/hydrasdr/hydrasdr-host.git hydrasdr-host`

`cd hydrasdr-host`

`mkdir build`

`cd build`

`cmake ../ -DINSTALL_UDEV_RULES=ON`

`make`

`sudo make install`

`sudo ldconfig`

## Clean CMake temporary files/dirs:

`cd hydrasdr-host/build`

`rm -rf *`

## How to build hydrasdr-host software on FreeBSD.

### Get the prerequisites (from root)

`pkg install git cmake`

### Build

`git clone https:\\github.com\hydrasdr\hydrasdr-host.git`

`cd hydrasdr-host`

`mkdir build`

`cd build`

`cmake .. -DLIBUSB_LIBRARIES=/usr/lib/libusb.so`

`make`

(from root)
`make install`

### Add users to group plugdev

(from root)
`useradd -a -G plugdev <user>`

## How to build the hydrasdr-host software on MacOS

### Prerequisites

### Minimum macOS Version
- macOS 10.15 (Catalina) or newer, on x86-64
- macOS 11 (Big Sur) or newer, on arm64 (Apple Silicon)

### Install Homebrew (if not already installed)
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

### Install required dependencies
```bash
brew install cmake libusb pkg-config git
```

### Building from source

#### Clone the repository
```bash
git clone https://github.com/hydrasdr/hydrasdr-host.git hydrasdr-host
cd hydrasdr-host
mkdir build
cd build
```

#### Configure and build

#### For Intel Macs (x86_64):
```bash
cmake .. -DCMAKE_OSX_ARCHITECTURES=x86_64 -DINSTALL_UDEV_RULES=OFF
make
sudo make install
```

#### For Apple Silicon Macs (ARM64):
```bash
cmake .. -DCMAKE_OSX_ARCHITECTURES=arm64 -DINSTALL_UDEV_RULES=OFF
make
sudo make install
```

#### For Universal Binary (Intel + ARM64):
```bash
cmake .. -DCMAKE_OSX_ARCHITECTURES="x86_64;arm64" -DINSTALL_UDEV_RULES=OFF
make
sudo make install
```

#### Debug build (optional)
For development or debugging purposes:
```bash
cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_OSX_ARCHITECTURES=arm64 -DINSTALL_UDEV_RULES=OFF
make
sudo make install
```

#### Post-installation

#### Update dynamic linker cache
```bash
sudo update_dyld_shared_cache
```

#### Verify installation
Check that the tools are properly installed:
```bash
hydrasdr_info
```

#### Platform-specific notes

#### macOS Permissions
On modern macOS versions, you may need to grant permissions for USB device access:
1. Go to System Preferences → Security & Privacy → Privacy
2. Add your terminal application to "Input Monitoring" if prompted
3. For development, you may need to disable System Integrity Protection (SIP) temporarily

#### Universal Binary Considerations
- Building universal binaries requires both Intel and ARM64 versions of all dependencies
- Homebrew on Apple Silicon installs ARM64 versions by default
- For Intel versions on Apple Silicon, use: `arch -x86_64 brew install <package>`

#### Troubleshooting

#### If cmake fails to find libusb:
```bash
export PKG_CONFIG_PATH="/opt/homebrew/lib/pkgconfig:$PKG_CONFIG_PATH"
```
For Intel Macs, use `/usr/local/lib/pkgconfig` instead of `/opt/homebrew/lib/pkgconfig`.

#### If you encounter permission issues:
```bash
sudo chown -R $(whoami) /usr/local/lib/pkgconfig
```

#### For Apple Silicon specific issues:
Some older build scripts may not recognize the ARM64 architecture. If you encounter issues, try:
```bash
export ARCHFLAGS="-arch arm64"
```

https://www.hydrasdr.com

This file is part of HydraSDR™ RFOne Copyright © 2025 Benjamin VERNOUX. All rights reserved. (based on HackRF project see https://greatscottgadgets.com/hackrf/).
