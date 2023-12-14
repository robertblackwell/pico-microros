#ifndef H_uros_nonstdio_uart_transport_h
#define H_uros_nonstdio_uart_transport_h

#ifdef __cplusplus
extern "C" 
{
#endif

#include <stdio.h>
#include <stdint.h>

// typedef int clockid_t;

#include <uxr/client/profile/transport/custom/custom_transport.h>

bool uros_nonstdio_uart_transport_open(struct uxrCustomTransport * transport);
bool uros_nonstdio_uart_transport_close(struct uxrCustomTransport * transport);
/**
 * Modified by robert blackwell added const before uint8_t * buf to align with the .c file
*/
size_t uros_nonstdio_uart_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t uros_nonstdio_uart_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#ifdef __cplusplus
}
#endif


#endif //MICRO_ROS_PICOSDK
