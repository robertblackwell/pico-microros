
MESSAGE("Entered tests/test_trace  PICO_SRC_DIR    ${PICO_SRC_DIR}")

set(TargetStdio test_stdio_trace)

add_executable(${TargetStdio} 
    test_stdio_trace.cpp
    ${PICO_SRC_DIR}/common/trace.h
    ${PICO_SRC_DIR}/common/trace.cpp
    )

pico_enable_stdio_usb(${TargetStdio} 1)
pico_enable_stdio_uart(${TargetStdio} 0)
pico_add_extra_outputs(${TargetStdio})

# target_compile_definitions(${TargetStdio} PRIVATE
#     # PICO_DEFAULT_UART_RX_PIN=16
#     # PICO_DEFAULT_UART_TX_PIN=17
#     PICO_DEFAULT_UART=1
#     PICO_DEFAULT_UART_RX_PIN=9
#     PICO_DEFAULT_UART_TX_PIN=8
# )
target_include_directories(${TargetStdio} PUBLIC ${PICO_SRC_DIR} ${PICO_SRC_DIR}/common)
target_link_libraries(${TargetStdio} pico_stdlib pico_unique_id)

set(TargetUart test_uart_trace)

add_executable(${TargetUart} 
    test_uart_trace.cpp
    ${PICO_SRC_DIR}/common/trace.h 
    ${PICO_SRC_DIR}/common/trace.cpp)

pico_enable_stdio_usb(${TargetUart} 1)
pico_enable_stdio_uart(${TargetUart} 0)
pico_add_extra_outputs(${TargetUart})

target_compile_definitions(${TargetUart} PRIVATE
    # PICO_DEFAULT_UART_RX_PIN=16
    # PICO_DEFAULT_UART_TX_PIN=17
    # PICO_DEFAULT_UART=0
    PICO_DEFAULT_UART_RX_PIN=9
    PICO_DEFAULT_UART_TX_PIN=8
)

target_include_directories(${TargetUart} PUBLIC ${PICO_SRC_DIR} ${PICO_SRC_DIR}/common)
target_link_libraries(${TargetUart} pico_stdlib pico_unique_id)