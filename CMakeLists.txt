cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(hexapod_servo_driver)

pico_sdk_init()

add_executable(hexapod_servo_driver src/main.c)

target_link_libraries(hexapod_servo_driver pico_stdlib pico_stdio_usb)

pico_add_extra_outputs(hexapod_servo_driver)