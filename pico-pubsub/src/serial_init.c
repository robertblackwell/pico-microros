#include <serial_init.h>

void serial_init()
{
    #if defined   UROS_SERIAL_USB_STDIO_NOCRLF
    #elif defined UROS_SERIAL_UART0_STDIO_NOCRLF
    #elif defined UROS_SERIAL_UART1_STDIO_NOCRLF
    #elif defined UROS_SERIAL_UART1_NOSTDIO_TELEM_USB_STDIO
    #elif defined UROS_SERIAL_UART1_NOSTDIO_TELEM_UART0_STDIO
    #elif defined UROS_SERIAL_UROS_STDIO_NOCRLF_UART0
    #elif defined UROS_SERIAL_UROS_STDIO_NOCRLF_UART0
    #endif

}