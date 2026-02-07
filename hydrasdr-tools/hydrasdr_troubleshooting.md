This page provides some quick tips on troubleshooting problems.

Most common problem to use HydraSDR software-defined radio at full speed 10MSPS(320Mbit/s) (and even 5MSPS / 2.5MSPS) are mainly related to USB controller or/and CPU.

We created a set of command line utilities to help troubleshooting the performance problems.

## Windows:
- **Performance problems:**
  - Download the [latest release](https://github.com/hydrasdr/hydrasdr-host/releases) of the tools package
  - Open a console (cmd.exe) and run `hydrasdr_rx -r NUL -t 0`
  - Leave it running for 30 seconds, then Ctrl+C
  If the average throughput is bellow 10.0 MSPS then either your USB controller has problems or you CPU can’t process the data.
  - Possible solutions:
    - Use another USB port
    - Update your USB drivers (Prefer OEM drivers to generic ones)
      - See https://github.com/libusb/libusb/wiki/Windows for more details
    - Check your anti-virus or any other CPU heavy task
    - Use a PCIe USB2.0/3.0 controller

## Linux (Debian/Ubuntu):
Ubuntu users: 22.04 / 24.04 LTS is highly recommended.
- **Building HydraSDR Host, gr-osmosdr and gqrx:**
    1. Download the [hydrasdr-host_git](https://github.com/hydrasdr/hydrasdr-host) repository, compile it, install it
    1. Download the [gr-osmosdr hydrasdr fork](https://github.com/hydrasdr/gr-osmosdr) repository, compile it, install it
    1. Download the [gqrx](https://github.com/csete/gqrx.git) repository, compile it, install it
    1. Get pulseaudio using standard Arch way
    1. Configure pulseaudio (add user and group, such things)

- **Performance problems:**
  - Build the host tools following "How to build the host software on Linux:" https://github.com/hydrasdr/hydrasdr-host
  - Open a shell and run `hydrasdr_rx -r /dev/null -t 0`
  - Leave it running for 30 seconds, then Ctrl+C
  If the average throughput is bellow 10.0 MSPS then either your USB controller has problems or you CPU can’t process the data.
  - Possible solutions:
    - Use another USB port
    - Update your kernel
    - Use a PCIe USB2.0/3.0 controller

## OSX:
- For Gqrx, gr-osmosdr and HydraSDR software-defined radio see https://groups.google.com/forum/#!topic/gqrx/u4duL8iPva0

- **Performance problems:**
  - Open a shell and run `hydrasdr_rx -r /dev/null -t 0`
  - Leave it running for 30 seconds, then Ctrl+C
  If the average throughput is bellow 10.0 MSPS then either your USB controller has problems or you CPU can’t process the data.
  - Possible solutions:
    - Use another USB port
    - Update your kernel
    - Use a PCIe USB2.0/3.0 controller
    - See also [OS X: Identifying USB ports on your Mac](http://support.apple.com/en-gb/HT202875)
and test different USB port if you have speed issue at 10MSPS

# VMWare Linux
Unable to enumerate product, serial, and manufacturer when connected to VMWare VM (Linux)

Fix to apply
> usb.quirks.device0 = "0x1d50:0x60a1 skip-setconfig"

# Problematic Configurations
Post here only if minimum hardware requirements match:
- 3rd gen Intel i3 2.4 GHz CPU or equivalent
- 2GB of RAM (to run your own OS, HydraSDR software-defined radio barely needs 1MB of memory)
- High speed USB 2.0/3.0 controller

Cheap StarTech.com 4 Port PCI Express PCIe SuperSpeed USB 2.0/3.0 Controller (PEXUSB3S4V) which can be found on following links:
- https://www.amazon.com/StarTech-com-Express-SuperSpeed-Controller-Adapter/dp/B009AT5SVS
- https://www.startech.com/en-fr/cards-adapters/pexusb3s4v

More feedback are welcome about those cheap PCIe USB2.0/3.0 controller to ensure they work fine with HydraSDR software-defined radio + Host Tools (especially with Windows, Linux & MacOS).

# How to Check if the External Clock is Active

The external reference clock input (`CLKIN`) is fixed in firmware to **10 MHz**.
- It must be connected and stable **before powering on or resetting** the HydraSDR software-defined radio, since the device only checks for a valid external clock during startup.
  - If `CLKIN` is not detected, the internal TCXO will be used instead.
  - If needed, HydraSDR software-defined radio can also be reset via the dedicated API or the `hydrasdr_reset` tool.

To verify whether the external clock was active at power-on, run:
```
hydrasdr_si5351c -n 0 -r
```

* **If bit 4 is set** → external `CLKIN` is *not* used (TCXO is the active clock).
  Example: `[ 0] -> 0x11`
* **If bit 4 is cleared** → external `CLKIN` *is* used (external clock is active).
  Example: `[ 0] -> 0x01`

Note: The output format can be confusing the **first nibble** represents bits 7–4, and the **second nibble** represents bits 3–0.

For more details on the `CLKIN Loss Of Signal` register used in this check, see:
* *AN619: Manually Generating an Si5351 Register Map* (Skyworks)
  * Section **8.1 – Register Map Summary**
  * Section **9 – Register Descriptions**
  * [AN619 PDF](https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN619.pdf)


# Using multiple HydraSDR software-defined radio
Multiple HydraSDR software-defined radio use case is mainly for Coherent Receiver Array, Direction Finding, Passive Radar ... which requires to have multiple HydraSDR software-defined radio running at same time (on the same PC).

* StarTech PEXUSB3S44V with dedicated USB channels seems to be one of the best cards on the market, see for example some test here https://github.com/openstf/stf

* Solution to have independent USB3 & USB2 controller is to buy such PCIe to USB3 adapter:
- 4x Independent USB2/USB3 ports (PCIe to USB3)
  StarTech.com 4 Port USB 3.0 PCIe Card w/ 4 Dedicated 5Gbps Channels (USB 3.2 Gen 1) - UASP - SATA / LP4 Power - PCI Express Adapter Card (PEXUSB3S44V)
  Official StarTech.com link: https://www.startech.com/en-us/cards-adapters/pexusb3s44v price about 117USD
  Amazon.com link https://www.amazon.com/gp/product/B00HJZEA2S price about 90USD
- 2x Independent USB2/USB3 ports (PCIe to USB3)
  StarTech.com 4-Port USB 3.0 PCI Express Card Adapter - PCIe SuperSpeed USB 3.0 Expansion Card w/ 2 Dedicated 5Gbps Channels (PEXUSB3S42V) 
  Official StarTech.com link: https://www.startech.com/en-us/cards-adapters/pexusb3s42v price about 93USD
  Amazon.com link https://www.amazon.com/StarTech-com-Express-SuperSpeed-Dedicated-Channels/dp/B00HJZE9VK price about 80USD (4 dedicated USB3 ports is recommended as it is just 10USD more)

> It is heavily recommended to have a high end PC at least CoreI7 in order to use multiple HydraSDR software-defined radio at same time on the same PC.
> * For example with a CoreI7-3630QM @2.4GHz (4cores), SDR++ use about 10% CPU with 10MSPS with 32K FFT and FM stereo.

