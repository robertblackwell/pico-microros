#ifndef MICRO_ROS_PICOSDK
#define MICRO_ROS_PICOSDK
#ifdef __cplusplus
extern "C" 
{
#endif

#include <stdio.h>
#include <stdint.h>

// typedef int clockid_t;

#include <uxr/client/profile/transport/custom/custom_transport.h>
/**
 * This module operates the micro-ros to micro-ros agent link over stdio.
 * 
 * The actual data transmission can be over the usb connection of uart0.
 * 
 * The choise is made by the following selections in the CMakeLists.txt file
 * 
 * Use the usb connection
 *    pico_enable_stdio_usb(${TARGET} 1)
 *    pico_enable_stdio_uart(${TARGET} 0)
 *
 * Use the uart0 connection
 *    pico_enable_stdio_usb(${TARGET} 0)
 *    pico_enable_stdio_uart(${TARGET} 1)
 *
 * In the latter case the pins used for uart0 can be selected, again in the CmakeLists.txt
 * file by
 *   target_compile_definitions(${TARGET} PRIVATE
 *       PICO_DEFAULT_UART_RX_PIN=16
 *       PICO_DEFAULT_UART_TX_PIN=17
 *   )
 * The downside sending micro-ROS data over stdio is that CR/LF processing must be inhibited for
 * the micro-ros wire protocol to work AND stdio is not available for any kind of telemetry output.
 * Telemetry is still possible but without access to a working printf() function.
*/
bool uros_stdio_transport_open(struct uxrCustomTransport * transport);
bool uros_stdio_transport_close(struct uxrCustomTransport * transport);
/**
 * Modified by robert blackwell added const before uint8_t * buf to align with the .c file
*/
size_t uros_stdio_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t uros_stdio_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#ifdef __cplusplus
}
#endif


#endif //MICRO_ROS_PICOSDK
