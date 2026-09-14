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

# Shared by both configurations below -- CC3501E_WIRE_CRC (CMakeLists.txt,
# customer-selectable) is a #if branch INSIDE these same sources
# (src/protocol.c, src/protocol_meta.c), not a different set of files, so
# both targets compile this identical list and differ only in which value
# they define it to and which test file drives them.
set(FW_SOURCES
    ${FW}/src/transport_spi.c
    ${FW}/src/transport_sdio.c
    ${FW}/src/protocol.c
    ${FW}/src/protocol_meta.c
    ${FW}/src/protocol_gpio.c
    ${FW}/src/protocol_spi.c
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

add_executable(test_transport_spi
    transport_spi/src/test_transport_spi.c
    ${FW_SOURCES}
)
target_include_directories(test_transport_spi PRIVATE
    ${FW}/src
    ${FW}/hal
    ${ALP_SDK_ROOT}/include
)
target_compile_definitions(test_transport_spi PRIVATE
    CC3501E_BRIDGE_FW_VERSION_U16=${CC3501E_BRIDGE_FW_VERSION_U16}
    # Explicit, not left to src/protocol.h's #ifndef fallback, so this
    # target keeps exercising the default (CRC-on) shape even if that
    # fallback's default is ever changed.
    CC3501E_WIRE_CRC=1)
target_compile_options(test_transport_spi PRIVATE ${TEST_FLAGS})
target_link_libraries(test_transport_spi PRIVATE ztest_shim)
add_test(NAME transport_spi COMMAND test_transport_spi)

# CC3501E_WIRE_CRC=OFF coverage: the byte-identical-to-3.1 shape.  A separate
# executable (ztest's shim registers cases as file-scope constructors --
# tests/unit/transport_spi/src/test_transport_spi.c's ~50 CRC-on-shaped
# cases cannot also run against CC3501E_WIRE_CRC=0 sources without a full
# rewrite of every transaction(), which is out of proportion to what this
# build option needs proving); this one is deliberately small and pins only
# the behaviour that is DIFFERENT under CC3501E_WIRE_CRC=0 -- see the test
# file's own top comment.
add_executable(test_transport_spi_no_crc
    transport_spi/src/test_transport_spi_no_crc.c
    ${FW_SOURCES}
)
target_include_directories(test_transport_spi_no_crc PRIVATE
    ${FW}/src
    ${FW}/hal
    ${ALP_SDK_ROOT}/include
)
target_compile_definitions(test_transport_spi_no_crc PRIVATE
    CC3501E_BRIDGE_FW_VERSION_U16=${CC3501E_BRIDGE_FW_VERSION_U16}
    CC3501E_WIRE_CRC=0)
target_compile_options(test_transport_spi_no_crc PRIVATE ${TEST_FLAGS})
target_link_libraries(test_transport_spi_no_crc PRIVATE ztest_shim)
add_test(NAME transport_spi_no_crc COMMAND test_transport_spi_no_crc)

# wifi_retry: the pure retry-delay + reason-restore decisions behind the
# reason-30 bounded retry (hal/ti/cc3501e_hw_ti_wifi.c).  Deliberately NOT
# built from FW_SOURCES: src/wifi_retry.c has no dependency on the rest of the
# firmware (no protocol.c, no worker.c, no HAL) by design -- see its own
# header for why -- and the real Wlan_Connect/event plumbing it was pulled out
# of needs the vendored TI SimpleLink SDK and is never linked into a host
# build at all, so this is the only piece of that fix host-testable full stop
# (mirrors src/sock_recv_commit.c's precedent above).
add_executable(test_wifi_retry
    wifi_retry/src/test_wifi_retry.c
    ${FW}/src/wifi_retry.c
)
target_include_directories(test_wifi_retry PRIVATE
    ${FW}/src
)
target_compile_options(test_wifi_retry PRIVATE ${TEST_FLAGS})
target_link_libraries(test_wifi_retry PRIVATE ztest_shim)
add_test(NAME wifi_retry COMMAND test_wifi_retry)
