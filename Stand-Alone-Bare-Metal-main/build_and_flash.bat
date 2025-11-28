@echo off
echo ==========================================
echo   Teensy 4.1 Bare-Metal Build & Flash
echo ==========================================

echo [1/3] Cleaning previous build...
if exist build rmdir /s /q build
mkdir build

echo [2/3] Building Firmware...
make
if %errorlevel% neq 0 (
    echo [ERROR] Build failed! Make sure you have 'make' and 'arm-none-eabi-gcc' installed and in your PATH.
    pause
    exit /b %errorlevel%
)

echo [3/3] Flashing Firmware...
echo.
echo *** PLEASE PRESS THE PROGRAM BUTTON ON YOUR TEENSY NOW ***
echo.
teensy_loader_cli --mcu=TEENSY41 -w -v build/firmware.hex

if %errorlevel% neq 0 (
    echo [ERROR] Flash failed! Make sure 'teensy_loader_cli.exe' is in this folder or PATH.
    pause
    exit /b %errorlevel%
)

echo.
echo [SUCCESS] Firmware flashed successfully!
pause
