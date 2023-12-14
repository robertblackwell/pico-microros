cmake_minimum_required(VERSION 3.27cmake)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_micro_ros_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()
set(TARGET pico-pubsub)
link_directories(libmicroros)
add_executable(${TARGET}
    src/uros_main.cpp
    src/uros_stdio_transport.h
    src/uros_stdio_transport.c
    src/uros_nonstdio_uart_transport.h
    src/uros_nonstdio_uart_transport.c
    src/common/trace.cpp
    src/common/trace.h
)

target_link_libraries(${TARGET}
    pico_stdlib
    microros
)

target_include_directories(${TARGET} PUBLIC
    libmicroros/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

pico_enable_stdio_usb(${TARGET} 1)
pico_enable_stdio_uart(${TARGET} 0)

# If you want the micro-ROS connection on uart0 change the above 2 lines to:
# pico_enable_stdio_usb(${TARGET} 0)
# pico_enable_stdio_uart(${TARGET} 1)

add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# 
# These next lines make uart0 work on pins 16 and 17
target_compile_definitions(${TARGET} PRIVATE
    PICO_DEFAULT_UART_RX_PIN=16
    PICO_DEFAULT_UART_TX_PIN=17
)
set(PICO_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
add_subdirectory(tests/test_trace)

# Generate UF2
pico_add_extra_outputs(${TARGET})
