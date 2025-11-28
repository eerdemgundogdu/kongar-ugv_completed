# Stand-Alone Bare-Metal Firmware (Alternative)

This folder contains a **bare-metal FreeRTOS** implementation for Teensy 4.1 as an alternative to the Arduino-based firmware.

## ⚠️ Status: Alternative Implementation

The main project uses **Arduino + FreeRTOS** (in `/firmware/` folder) which is:
- ✅ Easier to develop and debug
- ✅ Compatible with Arduino libraries
- ✅ Faster compilation
- ✅ **Currently used in competition**

This bare-metal version is:
- More low-level control
- Smaller binary size
- Requires ARM toolchain
- **Not currently integrated** (kept for reference)

## If You Want to Use Bare-Metal

### Requirements
```bash
# Install ARM toolchain
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi newlib-arm-none-eabi
```

### Build
```bash
cd Stand-Alone-Bare-Metal-main
make
```

### Flash
```bash
# Use Teensy Loader CLI
teensy_loader_cli --mcu=TEENSY41 -w -v build/firmware.hex
```

## Recommendation

**Stick with the Arduino-based firmware** in `/firmware/` folder unless you have specific bare-metal requirements. The Arduino version is:
- Already integrated with ROS 2
- Tested and working
- Easier to modify

## Integration Notes

If you want to migrate to bare-metal:
1. Port motor driver code from `/firmware/src/motor_driver.cpp`
2. Port PID controller from `/firmware/src/pid_controller.cpp`
3. Update serial protocol in `Teensy_UART.c`
4. Rebuild with `make`

For competition, **use the Arduino firmware** - it's production-ready.
