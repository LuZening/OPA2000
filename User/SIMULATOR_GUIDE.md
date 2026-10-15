# OPA2000 GUI Simulator Runbook

This document explains, end-to-end, how to **build the LVGL v7 PC simulator
for the OPA2000 GUI and capture screenshots** of any popup, screen or
multi-step flow.  It is written for the GUI agent and any other agent who
needs to verify visual changes without a target board.

If something here disagrees with reality, the simulator itself is the
source of truth — open the project in Visual Studio 2022 and the truth is
what the IDE shows.

---

## 1. Where the simulator lives

| What | Path |
| --- | --- |
| Simulator solution | `E:\Projects\RADIO\Projects\LVGL_SIM_V7\lv_sim_visual_studio\LVGL.Simulator.sln` |
| Simulator project file | `E:\Projects\RADIO\Projects\LVGL_SIM_V7\lv_sim_visual_studio\LVGL.Simulator\LVGL.Simulator.vcxproj` |
| Simulator `main()` | `…\LVGL.Simulator\LVGL.Simulator.cpp` (calls `gui_init()` then runs `lv_task_handler()` in a loop) |
| Build output (Release/x86) | `…\Output\Binaries\Release\Win32\LVGL.Simulator.exe` |
| OPA2000 GUI source compiled into the sim | `e:\Projects\RADIO\Projects\OPA2000\Code_2\OPA2000_h7\User\Src\gui.c` and friends (paths declared in the vcxproj `<ClCompile>` and `<ClInclude>` items) |
| OPA2000 GUI header | `e:\Projects\RADIO\Projects\OPA2000\Code_2\OPA2000_h7\User\Inc\gui.h` |
| Screenshot script directory | `C:\Users\cpholzn\AppData\Local\Temp\opa_*.ps1` |
| Screenshot drop folder | `e:\Projects\RADIO\Projects\OPA2000\Code_2\OPA2000_h7\User\gui_shots\` |

> The platform name in MSBuild is **`x86`**, not `Win32`.  Using `Win32`
> triggers `MSB4126`.  Configuration is **`Release`**.

---

## 2. Building the simulator

### Toolchain

* Visual Studio 2022 (any edition — the BuildTools SKU works for headless
  builds, full IDE for interactive).
* MSBuild path used by the runbook:
  `C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe`
  (full IDE installs use `C:\Program Files\Microsoft Visual Studio\2022\…\MSBuild.exe`).

### Command

```powershell
# 1. kill any running instance first — it holds a write lock on the .exe
Get-Process LVGL.Simulator -ErrorAction SilentlyContinue | Stop-Process -Force

# 2. build (Release / x86)
& "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe" `
    "E:\Projects\RADIO\Projects\LVGL_SIM_V7\lv_sim_visual_studio\LVGL.Simulator.sln" `
    /p:Configuration=Release /p:Platform=x86 /v:minimal /nologo
```

For a clean rebuild add `/t:Rebuild`.  A normal incremental build is
enough after touching `gui.c` / `language.c` / `opa2000.c`; the vcxproj
recompiles them automatically.

### Expected errors and how to read them

* **`LNK1104: 无法打开文件 … LVGL.Simulator.exe`** — the simulator is
  still running.  Run the `Stop-Process` line above and retry.
* **`C2065 "MyCJK_12" 未声明的标识符`** — `User\Inc\gui.h` is missing the
  font declaration.  Add `LV_FONT_DECLARE(MyCJK_12);` next to
  `MyCJK_16` and rebuild.
* **`C1083: 无法打开包括文件: "stm32h7xx_hal.h"`** when building from
  `User\Inc\hw_pinmap.h` — the include is no longer guarded by
  `#ifndef LVGL_SIM`.  **The Control-agent branch owns that file**;
  restore the guard or, if you must build locally to unblock yourself,
  wrap the offending `#include` line in `#ifndef LVGL_SIM` … `#endif`.
  Do **not** commit that local change.
* **`warning C4244`** from `lv_obj_style_dec.h` — this is **normal noise
  from LVGL v7.11 itself**, not from our code.  Ignore.

### Build success criteria

* MSBuild prints `BUILD_DONE` (or no error lines after a `Select-String`
  filter) and exits 0.
* `…\Output\Binaries\Release\Win32\LVGL.Simulator.exe` exists and has a
  recent mtime.

---

## 3. What `#define LVGL_SIM` does

`gui.h` aliases the historical name:

```c
#if defined(LVGL_SIMULATE) && !defined(LVGL_SIM)
#  define LVGL_SIM
#endif
```

Inside `gui.c` (and friends) every `#ifndef LVGL_SIM` block uses the
simulator path:

* `lv_port_disp.c` is not built; `lv_win32_init()` in
  `LVGL.Simulator.cpp` provides the display.
* `user_adc.c` returns canned raw values via `rand()` instead of the
  real ADC DMA.
* Touch cal uses a clickable red dot in the simulator (see
  `cal_sim_touch_cb`); the production path uses
  `touch_resistive_read()` from the resistive touch driver.
* Coupler calibration reads `g_adc_data.adc1_raw[…]` (production) vs.
  `rand()`-based fake counts (simulator).

The `LVGL.Simulator.cpp` main loop also calls three GUI pumps on every
frame — **do not delete them** or the live values freeze:

```cpp
gui_update_display();
gui_update_sensor_inspect();
gui_process_coupler_cal();
gui_process_calibration();
```

If you add a new periodic GUI function, wire it here.

---

## 4. The screenshot & click toolchain

The runbook uses four small PowerShell scripts in
`C:\Users\cpholzn\AppData\Local\Temp\`.  Their roles:

| Script | Purpose | Key params |
| --- | --- | --- |
| `opa_shot.ps1` | Captures the LVGL.Simulator window to a PNG. | `-ShotPath <full png path>` |
| `opa_click.ps1` | Sends a single left-button click at an absolute screen coordinate. | `-DX <x> -DY <y>` |
| `opa_place.ps1` | Moves the window to `(X, Y)` and forces it to be 500x315. | `-X <x> -Y <y>` |
| `opa_flow.ps1` | Orchestrates the above to drive any Settings → sub-menu flow and screenshot the result. | `-Row`, `-SubX`, `-SubY`, `-Tag`, `-ExtraX`, `-ExtraY`, `-ExtraN` |

### Window geometry (constants the scripts assume)

* The script pins the window at screen `(100, 60)` with client size
  `480x272` (= `LV_HOR_RES_MAX × LV_VER_RES_MAX`).
* Therefore the **client area origin in screen coordinates is
  approximately `(110, 93)`** (8 px left border + 25 px title bar +
  2 px = 93; the left border accounts for the 10 px difference vs.
  100).  `opa_flow.ps1` resolves this dynamically via
  `ClientToScreen` and **you should pass all click coordinates in
  *client* (480x272) coordinates, not absolute screen coordinates**.

### Driving a one-level Settings menu click

```powershell
# Example: open "Sensors" (row index 1) and capture the popup.
& powershell -ExecutionPolicy Bypass -File C:\Users\cpholzn\AppData\Local\Temp\opa_flow.ps1 `
    -Row 1 -SubX -1 -SubY -1 -Tag menu_02_sensors
```

* `-Row` indexes the seven menu rows: `0` = Calibrate Sensors, `1` =
  Sensors, `2` = Fan, `3` = Band Communication, `4` = Protection,
  `5` = Screen, `6` = Language.
* The screenshot lands in `User\gui_shots\<Tag>.png`.

### Driving a two-level flow (menu → sub-button)

```powershell
# Open Calibrate Sensors, click "Coupler 1", capture.
# Coupler 1 in the 300x130 dialog sits at client (175, 148).
& powershell -ExecutionPolicy Bypass -File C:\Users\cpholzn\AppData\Local\Temp\opa_flow.ps1 `
    -Row 0 -SubX 175 -SubY 148 -Tag coupler_step1
```

### Driving a multi-step flow (e.g. 4×Start to reach calibration result)

```powershell
# Open coupler cal, click Start four times to walk steps 1→4,
# then screenshot the results page.
# Start button is at client (185, 244).
& powershell -ExecutionPolicy Bypass -File C:\Users\cpholzn\AppData\Local\Temp\opa_flow.ps1 `
    -Row 0 -SubX 175 -SubY 148 -Tag coupler_results `
    -ExtraX 185 -ExtraY 244 -ExtraN 4
```

### Why the orchestrator is the way it is

* `opa_flow.ps1` kills any running simulator, relaunches it, moves the
  window to a known position, and queries `ClientToScreen` once at the
  top.  All subsequent clicks are in *client coordinates* so the same
  numbers work after DPI changes or window moves.
* It also serialises clicks with a 1.5 s gap.  Anything shorter and
  LVGL drops events when the screen is busy rebuilding (this bit us
  on the four-step coupler calibration).
* The default `500x315` window size matches what
  `opa_place.ps1 -X 100 -Y 60` produces — keep the two in sync.

---

## 5. Common GUI pitfalls (recorded fixes)

* **Labels ignore the width you set on them.**  LVGL v7's
  `LV_LABEL_LONG_EXPAND` calls `lv_obj_set_size()` from
  `lv_label_refr_text()` and overrides whatever you passed.  Fix:
  `lv_label_set_long_mode(l, LV_LABEL_LONG_CROP)` *before* `set_size`.
  See `make_centered_label()` in `gui.c` for the canonical helper.
* **`lv_btnmatrix_set_map()` keeps the pointer, not a copy.**  Pass a
  `static const char *btns[]` so the array outlives the widget.
  `lang_btns[]` in `gui.c` is the canonical example.
* **The °C / °F row in the language popup.**  With three rows
  (`English / 中文 / °C / °F / Cancel`) the indices are
  `{0, 1, 2, 3, 4}` — `lang_btns[3]` is **°F**, not Cancel.  Cancel
  is `lang_btns[4]` (or `lang_btns[6]` when there are two `\n`
  rows).  The class-of-bugs `CLAUDE.md` lists.
* **Deleting LVGL objects from inside their own event callback is
  unsafe.**  `lv_obj_del()` frees memory that LVGL is still walking.
  Defer the rebuild with a `*_rebuild_pending` flag and do the work
  from the main-loop pump (the pattern used in `cc_start_cb` /
  `gui_process_coupler_cal`).

---

## 6. Touch calibration simulator quirk

The simulator's `cal_build_point_ui()` draws a red dot with a
`cal_sim_touch_cb` click handler, so the user can click the dot to
register a calibration point.  This is a sim-only branch guarded by
`#ifdef LVGL_SIM`.  In production the same red dot is drawn but the
point is recorded by `gui_process_calibration()` reading
`touch_resistive_read()`.

When testing touch calibration in the simulator you must click the dot
itself, not the area around it — the hit area is the 34x34 circle.

---

## 7. Where the captured PNGs go

Every screenshot ends up under:

```
e:\Projects\RADIO\Projects\OPA2000\Code_2\OPA2000_h7\User\gui_shots\<Tag>.png
```

File names follow the pattern `<stage>_<index>_<label>.png`, e.g.
`menu_02_sensors.png`, `coupler_step1.png`, `flow_touch_p3.png`.  The
folder is shared with the user — if a file is missing, check that the
`opa_flow.ps1` invocation completed without throwing, and that the
`<Tag>` argument doesn't collide with an existing file (PowerShell
silently overwrites).

---

## 8. The "what's blocking me" list

Anything outside of `User/Src/gui.c` and `User/Inc/gui.h` is owned by
the Control agent / CubeMX and **must not be edited by the GUI
agent**.  The current open items are:

1. **`User/Inc/hw_pinmap.h`** — needs the `#ifndef LVGL_SIM` guard
   around `#include "stm32h7xx_hal.h"` (line 26) so the simulator
   build doesn't try to pull in CMSIS.  The build currently fails
   here.
2. **`User/Inc/opa2000.h`** — `ConfigType` recently added `touch_cal_t
   touch;`, `coupler_t { gain_uW_per_code; offset_mW; }`, and changed
   `band_im_max_a` → `band_im_max_mA`.  The corresponding
   `User/Src/config.c` does not match this layout yet, so the ARM
   build fails.  The GUI uses the new layout, the firmware side has
   to catch up.
3. **CMake toolchain file** (`cmake/gcc-arm-none-eabi.cmake`) — the
   file lost all of its newlines.  Use a working toolchain file or
   restore from git.
4. **`Core/Src/ltdc.c`** — still 800x480.  Either regenerate from
   the `.ioc` or hand-edit the `LtdcHandle.Init` fields to 480x272.
5. **`CMakeLists.txt`** — references `eeprom.c` and `pca9539.c` that
   do not exist in `User/Src/`.

The full list with status lives in
`User\TODO-需要你手动修改-HIGHLY-IMPORTANT.txt`.

---

## 9. End-to-end smoke test (one-liner)

After every GUI change, run this to confirm the simulator still boots
and the main screen renders:

```powershell
Get-Process LVGL.Simulator -ErrorAction SilentlyContinue | Stop-Process -Force
& "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\MSBuild\Current\Bin\MSBuild.exe" `
    "E:\Projects\RADIO\Projects\LVGL_SIM_V7\lv_sim_visual_studio\LVGL.Simulator.sln" `
    /p:Configuration=Release /p:Platform=x86 /v:minimal /nologo
& powershell -ExecutionPolicy Bypass -File C:\Users\cpholzn\AppData\Local\Temp\opa_shot.ps1 `
    -ShotPath "e:\Projects\RADIO\Projects\OPA2000\Code_2\OPA2000_h7\User\gui_shots\smoke.png"
```

If the resulting `smoke.png` shows the main screen with three bars
(FWD / Reflected / SWR), a band label, a protocol label and a
`Settings` button in the bottom right, the GUI is healthy.  If
instead you see a blank white screen, the LVGL theme failed to
attach — re-check that `lvgl/src/lv_themes/lv_theme_template.c` is in
the vcxproj and `LV_USE_THEME_TEMPLATE 1` is in `lv_conf.h`.
