# HydraSDR Calibration Procedure

## Overview

This procedure allows you to calibrate the frequency accuracy of your HydraSDR software-defined radio device 
by measuring and correcting frequency errors using parts-per-billion (PPB) corrections.

## Step-by-Step Calibration

1. **Clear existing calibration**: Reset any previous calibration by writing zero correction:
   ```bash
   hydrasdr_calibrate -w 0
   ```

2. **Restart the device**: Power cycle the HydraSDR software-defined radio or use `hydrasdr_reset` to ensure the new calibration is loaded during boot initialization

3. **Measure frequency error**: Use your measurement tool to compare actual frequency output against expected frequency, then calculate the correction needed using a PPM/PPB calculator (such as https://www.sitime.com/ppm-hz-calculator) - round the result to the nearest PPB value

4. **Apply the calculated correction**: Write the computed PPB value using `hydrasdr_calibrate -w <ppb_value>`:
   ```bash
   hydrasdr_calibrate -w -1942
   ```

5. **Restart the device again**: Power cycle the HydraSDR software-defined radio or use `hydrasdr_reset` to load and apply the new calibration settings

6. **Verify calibration**: Test with your measurement software to confirm the frequency correction is now accurate

## Important Notes

- Device restart is required after each calibration change because the calibration data is only read during the boot process
- Both positive and negative PPB correction values are supported
- The calibration data includes a timestamp and is stored in flash memory sector 2
- Use `hydrasdr_calibrate -r` to read current calibration values

## Example

If your measurement shows the device is running 1942 PPB slow, you would apply a positive correction:

```bash
# Clear existing calibration
hydrasdr_calibrate -w 0

# Restart device
hydrasdr_reset

# After measurement and calculation
hydrasdr_calibrate -w 1942

# Restart device to apply
hydrasdr_reset

# Verify with your measurement tool
```