# OPA2000 H7 Firmware Implementation Documentation

## Overview

This document describes the implementation of the OPA2000 power amplifier controller firmware for STM32H7B0VBT6.

## File Structure

All user code is located in the `User/` folder (no nested directories):

```
User/
├── adc.h/c              - ADC data acquisition (8 channels with DMA)
├── app_tasks.h/c        - FreeRTOS task definitions
├── comm.h/c             - USB CDC, LPUART1, radio communication
├── config.h/c           - Configuration management (EEPROM)
├── eeprom.h/c           - AT24C02 I2C EEPROM driver
├── encoder.h/c          - Rotary encoder (LPTIM1 + debounce)
├── freq_detect.h/c      - Frequency detector (TIM4 ETR)
├── gui.h/c              - LVGL GUI with multi-language
├── language.h/c         - Chinese/English string tables
├── opa2000.h/c          - Global types and utilities
├── power_control.h/c    - Power, PTT, relays, band switching, fan
├── protection.h/c       - Protection system
├── touch.h/c            - Resistive/capacitive touchscreen
├── lv_conf.h            - LVGL configuration
├── usbd_compound_desc.h/c - USB Compound Device descriptor (CDC + MSC)
├── usbd_msc_if.h/c      - USB MSC storage interface (SD card)
├── usbd_cdc_msc_wrapper.h/c - CDC + MSC composite device
├── sdcard_storage.h/c   - SD card storage (legacy, integrated into usbd_msc_if)
└── (Middlewares)        - USB MSC class library copied from MSC_TEMPLATE
```

## Implemented Features

### 1. ADC Data Acquisition

**Channels:**
- TX Power 1
- Reflected Power 1
- Main Voltage
- Main Current
- TX Power 2
- Reflected Power 2
- NTC Temperature
- Detector Sample

**Implementation:**
- Continuous scan mode with DMA
- Simple moving average filter
- Real-time SWR calculation (show 1.0 when power < 10W)

### 2. Power Control

**GPIO Outputs:**
- PWR_ON_SIG - Power enable
- RF_BIAS_ON - RF bias enable
- RELAY1, RELAY2 - Transmit relays
- BAND_SEL1X ~ SEL6X - Band selection (160M to 20M)
- FAN1_PWM - Fan control (10KHz PWM)
- BUZZER - Buzzer output (PWM)
- LCD_BL - LCD backlight
- ALL_FAULT_OUT - Fault indicator

**Fan Control:**
- Temperature < 60°C: 50% speed
- Temperature 60-70°C: 80% speed
- Temperature > 70°C: 100% speed
- Hysteresis: 10°C for speed decrease

### 3. Protection System

**Protection Conditions:**
1. Reflected Power 2 > 400W
2. Reflected Power 2 > 200W AND SWR > 2.0
3. Reflected Power 1 > 600W
4. Main Current > 40A
5. HARDPWR_ERR external interrupt
6. COMP1 comparator trigger
7. COMP2 comparator trigger
8. Overdrive external interrupt

**Behavior:**
- External interrupts have higher priority than FreeRTOS
- Do NOT call FreeRTOS APIs from ISR handlers
- Directly disable PTT and set ALL_FAULT_OUT
- Use flag variables to communicate with GUI task

### 4. Communication

**PC Communication (USB CDC + LPUART1):**
- Shared RX/TX queue
- Default baud rate: 19200
- Commands: STATUS, POWER?, SWR?, VOLTAGE?, CURRENT?, TEMP?, BAND?, ALERT OFF, CONFIG?, SAVE CONFIG, etc.

**Radio Communication (USART1):**
- ICOM CIV format
- Yaesu CAT format
- Kenwood format
- Auto-detection of radio type

### 5. Touch Screen

**Supported Types:**
- Resistive: TCS2046 (SPI1)
- Capacitive: GT911 (I2C3)
- Auto-detection on startup

**Calibration:**
- 5-point method (corners + center)
- Linear regression for X and Y
- Residual check (< 10% = pass)
- Store in EEPROM config

### 6. Rotary Encoder

**Implementation:**
- LPTIM1 in encoder mode
- Encoder switch with software debounce (20ms)
- EXTI interrupt for switch with timer confirmation

### 7. Frequency Detection

**Implementation:**
- TIM4 ETR input counting
- Check every 100µs
- Frequency = count × 10000 Hz
- Compare with expected band from radio
- Trigger protection on mismatch

### 8. LVGL GUI

**Screens:**
- Main display: Power, Reflected, SWR, Voltage, Current, Temperature, Band
- Alert dialogs
- Calibration screen

**Features:**
- Multi-language support (Chinese/English)
- Real-time value updates
- Touch input
- Encoder navigation

### 9. Calibration

**Directional Coupler Calibration:**
- Two-stage procedure (5W and 20W)
- Measure forward and reflected power
- Linear regression with intercept
- Generate calibration parameters for all 4 power channels

**Touch Calibration:**
- 5-point method
- Display points at 10% from edges + center
- Check center residual

### 10. Configuration Storage

**Stored in EEPROM (AT24C02):**
- PC communication baud rate
- Radio type and baud rate
- Protection thresholds
- Calibration parameters
- Touch calibration data
- Language setting
- CRC32 validation

## Tasks Still to be Implemented Later

1. **USB Compound Device Integration**
   - MSC and compound USB code created but needs CubeMX configuration
   - Need to enable MSC class in CubeMX USB configuration
   - Need to update IOC file for compound device

2. **Complete Band Support**
   - Currently bands 160M-20M implemented
   - Need 17M, 15M, 12M, 10M (require decoder or more GPIO)

3. **Alert Sound Sequence**
   - 3-note alert (high, mid, low)
   - Each note 300ms, 3s interval
   - Implementation needs timer configuration

4. **More GUI Screens**
   - Settings menu
   - Calibration menu
   - Band display refinement

5. **SD Card Block Access**
   - Current MSC implementation needs refinement for direct SD card block access
   - Using FATFS file-based approach or direct SDMMC access

## Build Instructions

```bash
# Build Debug
cmake --preset=Debug
cmake --build --preset=Debug

# Build Release
cmake --preset=Release
cmake --build --preset=Release
```

## Known Limitations

- Some GPIO pin mappings may need verification against actual hardware
- Timer configurations need to match CubeMX settings
- USB MSC device not yet implemented
- Band selection limited to 6 bands without decoder

## Reference

- Design requirements: OPA2000_H7_固件设计要求.txt
- CubeMX configuration: OPA2000_h7.ioc