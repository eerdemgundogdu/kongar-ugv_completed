# Testing Bare-Metal FreeRTOS on Teensy 4.1

This folder contains a **hardware test** that runs FreeRTOS directly on the Teensy hardware without the Arduino framework.

## 🧪 What This Code Does
It verifies that:
1.  **FreeRTOS** is running (Task switching works).
2.  **UART (Serial)** communication works at the register level.

**Behavior:**
- **Serial1 (LPUART1)**: Sends "Hello from LPUART1!" every second and echoes back any character you send.
- **Serial2 (LPUART2)**: Sends "Hello from LPUART2!" every second and echoes back any character you send.

## 🛠️ Prerequisites (Windows)

You need to install these 3 tools to build and flash this code:

1.  **ARM GCC Toolchain**:
    -   Download: [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
    -   Install and add `bin` folder to your System PATH.
2.  **Make for Windows**:
    -   Download: [GnuWin32 Make](http://gnuwin32.sourceforge.net/packages/make.htm) or use Chocolatey (`choco install make`).
    -   Add to PATH.
3.  **Teensy Loader CLI**:
    -   Download: [Teensy Loader CLI](https://www.pjrc.com/teensy/loader_cli.html)
    -   Put `teensy_loader_cli.exe` in this folder or in your PATH.

## 🚀 How to Test

### Option 1: If you have the tools installed
1.  Open a terminal in this directory.
2.  Run:
    ```cmd
    make
    ```
3.  Connect your Teensy 4.1 via USB.
4.  Press the **Program Button** on the Teensy.
5.  Run:
    ```cmd
    teensy_loader_cli --mcu=TEENSY41 -w -v build/firmware.hex
    ```

### Option 2: Using the Helper Script
I have created a `build_and_flash.bat` script for you.
1.  Connect Teensy.
2.  Double-click `build_and_flash.bat`.
3.  Press the **Program Button** on the Teensy when prompted.

## 🔍 Verifying the Output
1.  Open a Serial Monitor (like PuTTY, TeraTerm, or Arduino Serial Monitor).
2.  Connect to the Teensy's COM port.
3.  Baudrate: **115200**.
4.  You should see:
    ```text
    Hello from LPUART1!
    Hello from LPUART1!
    ...
    ```
5.  Type something, and it should echo back.

---
**NOTE:** This is **NOT** the main robot firmware. The main robot firmware is in the `/firmware` folder and uses PlatformIO/Arduino, which is much easier to use. This is only for low-level hardware verification.
