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
    ${USER_SRC}/adc_calibration.c
    ${USER_SRC}/user_adc.c
    ${USER_SRC}/power_control.c
    ${USER_SRC}/protection.c
    ${USER_SRC}/buzzer.c
    ${USER_SRC}/radio_band.c
    ${USER_SRC}/flash_eeprom.c

    # test cases
    ${CMAKE_CURRENT_SOURCE_DIR}/test_opa2000.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_config_layout.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_adc_calibration.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_user_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_power_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_protection.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_radio_band.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_flash_eeprom.c
)

# Math library: only the test reference implementation uses log() to
# cross-check the integer NTC implementation.
include(CheckLibraryExists)
check_library_exists(m log "" HAVE_LIBM)
if(HAVE_LIBM)
    target_link_libraries(host_tests PRIVATE m)
endif()

# -----------------------------------------------------------------------------
# CTest registration
# -----------------------------------------------------------------------------
enable_testing()
add_test(NAME host_tests COMMAND host_tests)