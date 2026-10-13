# OPA2000 Serial Command Reference

## Overview
The OPA2000 amplifier controller supports serial communication via USB CDC or LPUART1. Both interfaces share the same command set and RX queue.

## Default Settings
- Baud Rate: 19200 (configurable)
- Data Bits: 8
- Stop Bits: 1
- Parity: None

## Command Format
- Commands are case-insensitive
- Responses end with CRLF (`\r\n`)
- Use `?` for queries, no suffix for actions

---

## General Commands

### Status Query
```
STATUS?
```
Returns: `PWR=TX/BY/STBY,Band=160/80/...,Freq=14000000,Power=100.0,Reflect=5.0,SWR=1.2,Volt=13.5,Current=20.0,Temp=45.0,Alert=0`

### Power Query
```
POWER?
```
Returns: `POWER=100.0` (Watts)

### SWR Query
```
SWR?
```
Returns: `SWR=1.20`

### Voltage Query
```
VOLTAGE?
```
Returns: `VOLTAGE=13.5` (Volts)

### Current Query
```
CURRENT?
```
Returns: `CURRENT=20.0` (Amperes)

### Temperature Query
```
TEMP?
```
Returns: `TEMP=45.0` (Celsius)

### Band Query
```
BAND?
```
Returns: `BAND=20` (20 meters, 0 if None)

### Frequency Query
```
FREQ?
```
Returns: `FREQ=14000000` (Hz)

### Set Frequency (with auto band switch)
```
SET FREQ 14100
```
or
```
FREQ 14100
```
Sets frequency in **kHz** (must be within 1800-30000 kHz amateur bands). The system automatically switches to the corresponding band.

Returns: `FREQ=14100,BAND=20m,RANGE=14.000-18.168 MHz`

If frequency is invalid or outside amateur bands:
- `ERROR: Invalid frequency (1800-30000 kHz)`
- `ERROR: Frequency out of ham bands`

---

## Configuration Commands

### Set Language
```
LANG CHINESE
```
or
```
LANG ENGLISH
```
Returns: `LANG=CHINESE` or `LANG=ENGLISH`

### Get Language
```
LANG?
```
Returns: `LANG=ENGLISH`

### Get All Config
```
CONFIG?
```
Returns complete configuration string with all thresholds

### Set Configuration
```
SET <parameter>=<value>
```
Example: `SET PROT_REF_PWR1=600`

### Save Config
```
SAVE CONFIG
```
Returns: `CONFIG SAVED` or `ERROR: Save failed`

### Reset Config
```
RESET CONFIG
```
Returns: `CONFIG RESET`
Resets all config to defaults

---

## Alert Commands

### Dismiss Alert
```
ALERT OFF
```
Returns: `ALERT OFF OK`

---

## Calibration Commands

### Coupler Calibration

The amplifier has 2 directional couplers:
- **Coupler 1**: Located between PA output and LPF input
- **Coupler 2**: Located between LPF output and antenna

Each coupler measures forward and reflected power in both directions.

#### Start Calibration
```
CAL COUPLER 0
```
or
```
CAL COUPLER 1
```
Where `0` = Coupler 1 (PA→LPF), `1` = Coupler 2 (LPF→Antenna)

#### Calibration Procedure

When you start calibration, follow these steps:

**Stage 1 - 5W Reference:**
1. Set radio to AM mode at 5W output
2. Connect radio to amplifier input
3. Connect dummy load to antenna output
4. Press PTT, confirm forward power is displayed
5. Type `OK` and press Enter

The system will then prompt for reflected power measurement.

**Stage 2 - 20W Reference:**
1. Increase radio power to 20W
2. Press PTT, confirm forward power
3. Type `OK` and press Enter
4. Repeat for reflected power measurement

#### Calibration Complete
Once finished, the system displays:
- Calculated slope and offset
- Saves configuration automatically

#### Abort Calibration
```
CAL ABORT
```
or
```
ABORT
```
Cancels the current calibration session.

---

## Error Responses

```
ERROR: Unknown command
ERROR: Invalid coupler ID (0-1)
ERROR: Calibration already in progress
ERROR: Save failed
```

---

## Example Session

```
> STATUS?
PWR=STBY,Band=20,Freq=14000000,Power=0.0,Reflect=0.0,SWR=1.00,Volt=13.5,Current=0.0,Temp=25.0,Alert=0

> CAL COUPLER 1
========================================
CALIBRATION: Coupler 2 (LPF->Antenna)
========================================

Step 1: Connect radio to amp, set to AM mode
Step 2: Set radio power to 5W
Step 3: Press PTT and confirm forward power
Step 4: Type OK and press Enter when ready

> OK
>>> Recording 5W FORWARD power...
ADC=1234

Now DISCONNECT the antenna (or set RELAY2 to reflect)
Press PTT and confirm reflected power
Type OK and press Enter when ready

> OK
>>> Recording 5W REFLECTED power...
ADC=50

[... continues ...]

========================================
CALIBRATION COMPLETE!
========================================
Slope: 0.012345
Offset: -5.678901
Config saved!
```