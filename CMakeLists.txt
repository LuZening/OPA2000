cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# LVGL Simulator mode option
option(LVGL_SIM "Build for LVGL simulator (no OPA2000 hardware dependencies)")
if(LVGL_SIM)
    add_compile_definitions(LVGL_SIM)
endif()

# Set the project name
set(CMAKE_PROJECT_NAME OPA2000_h7)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add USB MSC source files
file(GLOB USB_MSC_SOURCES
    "Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc.c"
    "Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_bot.c"
    "Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_data.c"
    "Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_scsi.c"
)

# Add LVGL core sources
file(GLOB_RECURSE LVGL_CORE_SOURCES
    "Middlewares/Third_Party/lvgl/src/lv_core/*.c"
)
file(GLOB_RECURSE LVGL_DRAW_SOURCES
    "Middlewares/Third_Party/lvgl/src/lv_draw/*.c"
)
file(GLOB_RECURSE LVGL_HAL_SOURCES
    "Middlewares/Third_Party/lvgl/src/lv_hal/*.c"
)
file(GLOB_RECURSE LVGL_MISC_SOURCES
    "Middlewares/Third_Party/lvgl/src/lv_misc/*.c"
)
file(GLOB_RECURSE LVGL_WIDGETS_SOURCES
    "Middlewares/Third_Party/lvgl/src/lv_widgets/*.c"
)
file(GLOB_RECURSE LVGL_FONT_SOURCES
    "Middlewares/Third_Party/lvgl/src/lv_font/lv_*.c"
)
file(GLOB_RECURSE LVGL_THEME_SOURCES
    "Middlewares/Third_Party/lvgl/src/lv_themes/lv_theme*.c"
)
file(GLOB_RECURSE LVGL_GPU_SOURCES
    "Middlewares/Third_Party/lvgl/src/lv_gpu/*.c"
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USB_MSC_SOURCES}
    ${LVGL_CORE_SOURCES}
    ${LVGL_DRAW_SOURCES}
    ${LVGL_HAL_SOURCES}
    ${LVGL_MISC_SOURCES}
    ${LVGL_WIDGETS_SOURCES}
    ${LVGL_FONT_SOURCES}
    ${LVGL_THEME_SOURCES}
    ${LVGL_GPU_SOURCES}
    User/Src/alc_pwm.c
    User/Src/app_tasks.c
    User/Src/boot.c
    User/Src/buzzer.c
    User/Src/comm.c
    User/Src/config.c
    User/Src/fan_tach.c
    User/Src/flash_eeprom.c
    User/Src/freq_detect.c
    User/Src/freertos_hooks.c
    User/Src/gui.c
    User/Src/lv_port_disp.c
    User/Src/key.c
    User/Src/language.c
    User/Src/opa2000.c
    User/Src/adc_calibration.c
    User/Src/power_control.c
    User/Src/protection.c
    User/Src/radio_band.c
    User/Src/radio_uart.c
    # Excluded until CubeMX generates the FatFs middleware: the current build
    # has no FatFs include directory at all, so sdcard_storage.h's
    # "#include <ff.h>" cannot resolve.  usbd_msc_if.c includes the same
    # header, so it goes with it.  Re-add both once FatFs (and USB_DEVICE,
    # for the MSC stack) are enabled in the .ioc and the code regenerated.
    # User/Src/sdcard_storage.c
    # User/Src/usbd_msc_if.c
    User/Src/touch.c
    User/Src/ns2009.c
    User/Src/user_adc.c
    User/Src/usbd_cdc_msc_wrapper.c
    User/Src/usbd_composite_desc.c
    User/Src/usb_dma.c
    User/fonts/MyCJK_12.c
    User/fonts/MyCJK_16.c
    User/fonts/MyAwesome_12.c
    User/fonts/MyAwesome_16.c
    User/fonts/MyAwesome_24.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc
    ${CMAKE_SOURCE_DIR}/User/Inc
    ${CMAKE_SOURCE_DIR}/User
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/lvgl
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/lvgl/src
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    LV_CONF_INCLUDE_SIMPLE
    # CDC endpoint overrides for CDC+MSC composite device
    CDC_IN_EP=0x82U
    CDC_OUT_EP=0x01U
    CDC_CMD_EP=0x81U
    # MSC endpoint overrides for CDC+MSC composite device
    MSC_EPIN_ADDR=0x83U
    MSC_EPOUT_ADDR=0x03U
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Generate HEX file after build
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND "C:/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-objcopy" -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generating HEX file..."
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    FreeRTOS
    STM32_Drivers
    USB_Device_Library

    # Add user defined libraries
    m
    nosys
    gcc
    c
)