# =============================================================================
#  OPA2000 H7 - host-side unit test harness
#
#  Builds a native x86/x64 executable that exercises the pure algorithms
#  and GPIO sequencing logic in User/Src/ against a stubbed HAL.
#
#  Usage (from the repository root):
#     cmake -S User/Tests -B build/host-tests -G "Visual Studio 17 2022" -A x64
#     cmake --build build/host-tests --config Release
#     ctest --test-dir build/host-tests -C Release --output-on-failure
#
#  Rules:
#   - Never links against Drivers/ or Middlewares/ - only the algorithm
#     .c files under test plus the HAL stub.
#   - Any new function must ship with at least one TEST_CASE() here
#     before it is merged (see docs/GLOBAL_RULES.md rule 0).
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(OPA2000_H7_HOST_TESTS C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/W3)
    add_compile_definitions(HOST_TEST_NATIVE)
    add_compile_options(/wd4996)   # target is C11; the CRT "secure" warnings are noise
    # Sources carry Chinese comments, same as the firmware.  Without this
    # MSVC decodes the UTF-8 bytes as CP936, which both warns (C4819) and
    # can corrupt a byte sequence that looks like a comment terminator,
    # producing a spurious syntax error far from the real cause.
    add_compile_options(/utf-8)
else()
    add_compile_options(-Wall -Wextra -Wno-unused-parameter)
    add_compile_definitions(HOST_TEST_NATIVE)
endif()

# -----------------------------------------------------------------------------
# Paths
# -----------------------------------------------------------------------------
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(USER_INC    "${REPO_ROOT}/User/Inc")
set(USER_SRC    "${REPO_ROOT}/User/Src")
set(STUB_DIR    "${CMAKE_CURRENT_SOURCE_DIR}/stubs")

# -----------------------------------------------------------------------------
# Include order matters: the stub dir must come FIRST so that
# <stm32h7xx_hal.h> and <main.h> resolve to our stubs instead of the
# real CMSIS / CubeMX headers.
# -----------------------------------------------------------------------------
include_directories(
    ${STUB_DIR}
    ${USER_INC}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# -----------------------------------------------------------------------------
# Test binary
# -----------------------------------------------------------------------------
add_executable(host_tests
    # harness
    ${CMAKE_CURRENT_SOURCE_DIR}/test_harness.c

    # HAL stub (also defines the extern TIM/DMA/ADC handles + fake ports)
    ${STUB_DIR}/stm32h7xx_hal_stub.c

    # algorithm under test
    ${USER_SRC}/opa2000.c
    ${USER_SRC}/config.c
    ${USER_SRC}/adc_calibration.c
    ${USER_SRC}/user_adc.c
    # boot.c is a real dependency now, not just something test_boot.c
    # mirrors: power_control.c's switch_band() calls
    # boot_apply_thresholds() so AWD1 tracks the band.  Without this line
    # the host link fails on the unresolved symbol.
    ${USER_SRC}/boot.c
    ${USER_SRC}/power_control.c
    ${USER_SRC}/protection.c
    ${USER_SRC}/buzzer.c
    ${USER_SRC}/fan_tach.c
    ${USER_SRC}/radio_band.c
    ${USER_SRC}/freq_detect.c
    ${USER_SRC}/flash_eeprom.c
    # comm.c is intentionally NOT linked into the host build: it depends
    # on LPUART1 HAL + USB CDC, neither of which has a host stub.  The
    # pure protocol layer (set/query/action dispatch) lives inside
    # comm.c as static functions for now; if/when we add a host-side
    # test for the protocol it should be lifted into a separate
    # comm_proto.c with the UART wrappers kept here.  See the TEST_DEBT
    # note in the user-facing task list.

    # test cases
    ${CMAKE_CURRENT_SOURCE_DIR}/test_opa2000.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_config_layout.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_adc_calibration.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_user_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_power_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_protection.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_fan_tach.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_radio_band.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_freq_detect.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_flash_eeprom.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_cmsis_os_shim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_config_mutex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_boot.c

    # Windows-backed CMSIS-RTOS2 mutex shim (host test only; firmware
    # uses the real FreeRTOS implementation in Middlewares/...).
    ${STUB_DIR}/cmsis_os_shim.c

    # flash_eeprom.c compiles its hardware half out of the host build, but
    # config.c calls eeprom_load / eeprom_save / eeprom_current_id
    # unconditionally - these stand-ins satisfy that link and give the host
    # run the documented "empty store" semantics config_init() expects.
    ${STUB_DIR}/flash_eeprom_host_stub.c

    # comm.c is not linked (LPUART1 HAL + USB CDC have no host stub), but
    # protection.c calls comm_post_alert_evt() to push alert transitions
    # into the CAT link.  This stand-in records them so test_protection.c
    # can assert one event per trigger / dismiss.
    ${STUB_DIR}/comm_host_stub.c
)

# Math library: only the test reference implementation uses log() to
# cross-check the integer NTC implementation.
include(CheckLibraryExists)
check_library_exists(m log "" HAVE_LIBM)
if(HAVE_LIBM)
    target_link_libraries(host_tests PRIVATE m)
endif()

# Windows kernel32 backs the CMSIS-RTOS2 mutex shim (CreateMutexW /
# WaitForSingleObject / _beginthreadex).  No-op on non-MSVC platforms
# because the shim itself is guarded for Windows builds only and is
# not part of the firmware image.
if(MSVC)
    target_link_libraries(host_tests PRIVATE kernel32)
    target_compile_definitions(host_tests PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# -----------------------------------------------------------------------------
# CTest registration
# -----------------------------------------------------------------------------
enable_testing()
add_test(NAME host_tests COMMAND host_tests)