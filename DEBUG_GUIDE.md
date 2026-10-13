# VS Code STM32 Development Guide

This document describes how to set up VS Code for STM32H7 development with CMake, debug, and JLink flashing.

## 1. Toolchain Locations

All required tools are installed in STM32CubeCLT:

| Tool | Path |
|------|------|
| ARM GCC Compiler | `C:/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe` |
| CMake | `C:/ST/STM32CubeCLT_1.21.0/CMake/bin/cmake.exe` |
| Ninja | `C:/ST/STM32CubeCLT_1.21.0/Ninja/bin/ninja.exe` |
| GDB | `C:/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-gdb.exe` |
| ST-Link GDB Server | `C:/ST/STM32CubeCLT_1.21.0/STLink-gdb-server/bin/ST-LINK_gdbserver.exe` |
| JLink GDB Server | `C:/Program Files (x86)/SEGGER/JLink/JLinkGDBServer.exe` |

## 2. VS Code IntelliSense Configuration

The `.vscode/c_cpp_properties.json` is already configured:

```json
{
    "configurations": [
        {
            "name": "STM32H7",
            "includePath": [
                "${workspaceFolder}/**",
                "${workspaceFolder}/Core/Inc",
                "${workspaceFolder}/User",
                "${workspaceFolder}/Drivers/CMSIS/Include",
                "${workspaceFolder}/Drivers/CMSIS/Device/ST/STM32H7xx/Include",
                "${workspaceFolder}/Drivers/STM32H7xx_HAL_Driver/Inc",
                "${workspaceFolder}/Middlewares/Third_Party/FreeRTOS/Source/include",
                "${workspaceFolder}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
                "${workspaceFolder}/Middlewares/Third_Party/lvgl/lvgl",
                "${workspaceFolder}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc",
                "${workspaceFolder}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc",
                "${workspaceFolder}/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc"
            ],
            "defines": [
                "USE_HAL_DRIVER",
                "STM32H7B0xx",
                "USE_FULL_LL_DRIVER",
                "ARM_MATH_CM7"
            ],
            "compilerPath": "C:/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe",
            "cStandard": "c11",
            "cppStandard": "c++17",
            "intelliSenseMode": "gcc-arm",
            "compileCommands": "${workspaceFolder}/build/compile_commands.json"
        }
    ],
    "version": 4
}
```

## 3. Build Commands

### Configure and Build (Debug)
```bash
cmake --preset=Debug
cmake --build --preset=Debug
```

### Configure and Build (Release)
```bash
cmake --preset=Release
cmake --build --preset=Release
```

### Clean Build
```bash
cmake --build --preset=Debug --clean-first
```

## 4. Debugging with VS Code

### Option A: Using ST-Link (Recommended)

Create `.vscode/launch.json`:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "ST-Link Debug",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/build/Debug/OPA2000_h7.elf",
            "args": [],
            "stopAtEntry": true,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "GDB",
            "miDebuggerPath": "C:/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-gdb.exe",
            "miDebuggerServerAddress": "localhost:3333",
            "preLaunchTask": "Start ST-Link GDB Server",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                },
                {
                    "description": "Set breakpoints",
                    "text": "-gdb-set breakpoint pending on",
                    "ignoreFailures": true
                }
            ]
        }
    ]
}
```

Create `.vscode/tasks.json` for starting GDB server:

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "Start ST-Link GDB Server",
            "type": "shell",
            "command": "C:/ST/STM32CubeCLT_1.21.0/STLink-gdb-server/bin/ST-LINK_gdbserver.exe",
            "args": [
                "-d",
                "-p", "3333",
                "-m", "hw",
                "--reset-mode", "SYSRESETREQ",
                "--serialno", "066DFF545352887267092043"
            ],
            "isBackground": true,
            "problemMatcher": {
                "owner": "custom",
                "pattern": {
                    "regexp": "^$"
                },
                "background": {
                    "activeOnStart": true,
                    "beginsPattern": ".",
                    "endsPattern": "."
                }
            },
            "presentation": {
                "reveal": "always",
                "panel": "new"
            }
        }
    ]
}
```

### Option B: Using JLink

Create `.vscode/launch.json` for JLink:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "JLink Debug",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/build/Debug/OPA2000_h7.elf",
            "args": [],
            "stopAtEntry": true,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "GDB",
            "miDebuggerPath": "C:/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-gdb.exe",
            "miDebuggerServerAddress": "localhost:2331",
            "preLaunchTask": "Start JLink GDB Server",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ]
        }
    ]
}
```

Create `.vscode/tasks.json` for JLink GDB server:

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "Start JLink GDB Server",
            "type": "shell",
            "command": "C:/Program Files (x86)/SEGGER/JLink/JLinkGDBServer.exe",
            "args": [
                "-port", "2331",
                "-device", "STM32H7B0VB",
                "-if", "SWD",
                "-speed", "4000"
            ],
            "isBackground": true,
            "problemMatcher": {
                "owner": "custom",
                "pattern": {
                    "regexp": "^$"
                },
                "background": {
                    "activeOnStart": true,
                    "beginsPattern": ".",
                    "endsPattern": "."
                }
            }
        }
    ]
}
```

## 5. Flash Programming

### Option A: Using STM32CubeProgrammer

Command line flash (replace with your device and file):

```bash
"C:/ST/STM32CubeCLT_1.21.0/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe" \
    -c port=SWD freq=4000 mode=Hotplug \
    -w "${workspaceFolder}/build/Debug/OPA2000_h7.hex" \
    -v \
    -rst
```

Or use the GUI:
```bash
"C:/ST/STM32CubeCLT_1.21.0/STM32CubeProgrammer/bin/STM32CubeProgrammer.exe"
```

### Option B: Using JLink

#### JLink Commander (Interactive)
```bash
"C:/Program Files (x86)/SEGGER/JLink/JLink.exe"
```

#### JLink Flash Lite (Simple GUI)
```bash
"C:/Program Files (x86)/SEGGER/JLink/JFlashLite.exe"
```

#### Command Line Flash with JLink
```bash
# Erase and program
"C:/Program Files (x86)/SEGGER/JLink/JLink.exe" \
    -device STM32H7B0VB \
    -if SWD \
    -speed 4000 \
    -CommanderScript "C:/path/to/flash.jlink"

# Or use JFlash
"C:/Program Files (x86)/SEGGER/JLink/JFlash.exe" \
    -openprj"STM32H7B0VB.jflash" \
    -open"build/Debug/OPA2000_h7.hex" \
    -auto \
    -exit
```

Create a JLink command script `flash.jlink`:

```jlink
r
h
loadfile "E:/Projects/RADIO/Projects/OPA2000/Code_2/OPA2000_h7/build/Debug/OPA2000_h7.hex"
r
q
```

### Option C: Using OpenOCD (Alternative)

If you have OpenOCD installed:

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
    -c "program build/Debug/OPA2000_h7.elf verify reset exit"
```

## 6. Quick Reference

| Action | Command |
|--------|---------|
| Build Debug | `cmake --build --preset=Debug` |
| Build Release | `cmake --build --preset=Release` |
| Start ST-Link GDB | `ST-LINK_gdbserver.exe -p 3333` |
| Start JLink GDB | `JLinkGDBServer.exe -port 2331 -device STM32H7B0VB` |
| Connect with GDB | `arm-none-eabi-gdb -ex "target remote localhost:3333" build/Debug/OPA2000_h7.elf` |

## 7. Troubleshooting

### IntelliSense Not Working
- Run `cmake --preset=Debug` first to generate `compile_commands.json`
- Reload VS Code window: `Ctrl+Shift+P` -> "Developer: Reload Window"

### GDB Connection Failed
- Check that no other GDB server is running on the port
- Verify the ST-Link/JLink is connected and recognized
- Try running GDB server manually to see error messages

### Flash Programming Failed
- Check SWD pins (SWDIO, SWCLK) are properly connected
- Verify target device is powered
- Try lowering SWD speed

## 8. Notes

- The STM32H7B0VBT6 device uses SWD (Serial Wire Debug) interface
- Default SWD speed: 4000 kHz (can be reduced if connection is unstable)
- For JLink, device name: `STM32H7B0VB` (check JLinkDevices.xml for exact name)