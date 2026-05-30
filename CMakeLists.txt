cmake_minimum_required(VERSION 3.22)

set(NAME Hexapod_servo_driver)

include(pico_sdk_import.cmake)
include(pimoroni_pico_import.cmake)

project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Define the source files
set(SOURCES
    src/main.cpp
    src/proto/crc.cpp
    src/proto/cobs.cpp
    src/proto/frame.cpp
)

# Define the include directories
include_directories(src src/commands src/proto src/safety)

# Add source files
add_executable(${NAME} ${SOURCES})

# Include required libraries
# This assumes pimoroni-pico is stored alongside project
include(libraries/servo2040/servo2040)
include(drivers/analogmux/analogmux)
include(drivers/button/button)
include(drivers/analog/analog)

# link the libraries
target_link_libraries(${NAME}
    hardware_uart
    pico_stdlib
    servo2040
    analogmux
    analog
    button
)

# Enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)


# Create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# ============================================================================
# Phase 13 diagnostic — Pimoroni-only minimal test binary.
#
# Separate executable that uses ONLY Pimoroni's ServoCluster API directly.
# Build target name: test_pimoroni_direct.uf2
#
# Flash with:
#   sudo picotool load build/test_pimoroni_direct.uf2 && sudo picotool reboot
# ============================================================================
add_executable(test_pimoroni_direct src/test_pimoroni_direct.cpp)
target_link_libraries(test_pimoroni_direct
    pico_stdlib
    servo2040
)
pico_enable_stdio_usb(test_pimoroni_direct 1)
pico_enable_stdio_uart(test_pimoroni_direct 0)
pico_add_extra_outputs(test_pimoroni_direct)

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.uf2
    ${CMAKE_CURRENT_LIST_DIR}/README.md
    DESTINATION .
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)