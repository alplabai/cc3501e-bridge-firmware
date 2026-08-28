# SPDX-License-Identifier: Apache-2.0
#
# HOST unit tests for the bridge firmware.  Configured SEPARATELY from the
# top-level firmware build, which cross-compiles for the CC3501E's Cortex-M33
# and must not have a host toolchain spliced into it:
#
#   cmake -S tests/unit -B build-tests -DALP_SDK_ROOT=<path-to-alp-sdk>
#   cmake --build build-tests
#   ctest --test-dir build-tests --output-on-failure
#
# This suite moved here from alp-sdk when the firmware split into its own
# repository (alp-sdk ADR 0031).  It links the REAL firmware sources -- not
# mocks -- so the SPI framing and dispatcher seams under test are the same code
# paths the device runs.
cmake_minimum_required(VERSION 3.20)
project(cc3501e-bridge-unit-tests LANGUAGES C)

enable_testing()

set(FW ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# The wire contract lives in alp-sdk, and this firmware compiles alp-sdk's
# CANONICAL <alp/protocol/cc3501e.h> rather than a mirror -- the whole point of
# ADR 0031's single-sourcing.  The test does the same, so it cannot pass against
# a header this repo has drifted from.
set(ALP_SDK_ROOT "" CACHE PATH "Checkout of alplabai/alp-sdk (for <alp/protocol/cc3501e.h>)")
if(NOT ALP_SDK_ROOT OR NOT EXISTS "${ALP_SDK_ROOT}/include/alp/protocol/cc3501e.h")
    message(FATAL_ERROR
        "The transport suite compiles alp-sdk's canonical protocol header, which "
        "is not vendored here (ADR 0031).\n"
        "Clone https://github.com/alplabai/alp-sdk and configure with "
        "-DALP_SDK_ROOT=<path-to-that-checkout>.")
endif()

# Build this on LINUX (CI does; WSL works locally).  Two separate Windows
# problems, so that neither costs anyone an afternoon:
#
#   * MSVC: the shim registers cases with __attribute__((constructor)), which
#     cl.exe does not implement -- caught explicitly below rather than left to
#     fail later on an unrecognised /Wextra.
#   * MinGW: the firmware supplies WEAK default definitions
#     (transport_spi.c's bridge_transport_spi_polled() and friends) that the TI
#     HAL overrides on target.  ELF resolves those; PE does not, so a MinGW
#     build dies with a wall of "undefined reference to
#     bridge_transport_spi_*" / "worker_critical_*".  That is a PE limitation,
#     NOT a defect in the sources, and no CMake guard here can detect it.
if(NOT CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    message(FATAL_ERROR
        "The unit tests need GCC or Clang (got '${CMAKE_C_COMPILER_ID}'): the "
        "ztest shim registers cases via __attribute__((constructor)).\n"
        "On Windows, configure with -DCMAKE_C_COMPILER=gcc or run them in WSL.")
endif()

# Derive the GET_DIAG_INFO.fw_version marker from firmware-version.txt -- the
# single source of truth, exactly as the firmware CMake and the ti/ build do --
# so the test asserts against the DERIVED constant rather than a literal that
# drifts the moment someone bumps the version.
# Pre-1.0 packing: (MINOR << 8) | PATCH, so 0.4.1 -> 0x0401.
file(STRINGS ${FW}/firmware-version.txt CC3501E_BRIDGE_FW_VERSION LIMIT_COUNT 1)
if(NOT CC3501E_BRIDGE_FW_VERSION MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)")
    message(FATAL_ERROR "firmware-version.txt not SemVer: '${CC3501E_BRIDGE_FW_VERSION}'")
endif()
math(EXPR CC3501E_BRIDGE_FW_VERSION_U16
     "(${CMAKE_MATCH_2} << 8) | ${CMAKE_MATCH_3}" OUTPUT_FORMAT HEXADECIMAL)

add_library(ztest_shim STATIC ztest_shim.c)
target_include_directories(ztest_shim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Warnings are errors: this suite exists to catch framing and dispatch bugs, and
# a silent conversion warning in the firmware sources is exactly that class.
set(TEST_FLAGS -std=c11 -Wall -Wextra -Werror)

add_executable(test_transport_spi
    transport_spi/src/test_transport_spi.c
    ${FW}/src/transport_spi.c
    ${FW}/src/transport_sdio.c
    ${FW}/src/protocol.c
    ${FW}/src/protocol_meta.c
    ${FW}/src/protocol_gpio.c
    ${FW}/src/protocol_camera.c
    ${FW}/src/protocol_wifi.c
    ${FW}/src/protocol_sockets.c
    ${FW}/src/protocol_ble.c
    ${FW}/src/protocol_power.c
    ${FW}/src/protocol_diag.c
    ${FW}/src/protocol_ota.c
    ${FW}/src/worker.c
    ${FW}/src/event_ring.c
    ${FW}/hal/cc3501e_hw_stub.c
)
target_include_directories(test_transport_spi PRIVATE
    ${FW}/src
    ${FW}/hal
    ${ALP_SDK_ROOT}/include
)
target_compile_definitions(test_transport_spi PRIVATE
    CC3501E_BRIDGE_FW_VERSION_U16=${CC3501E_BRIDGE_FW_VERSION_U16})
target_compile_options(test_transport_spi PRIVATE ${TEST_FLAGS})
target_link_libraries(test_transport_spi PRIVATE ztest_shim)
add_test(NAME transport_spi COMMAND test_transport_spi)
