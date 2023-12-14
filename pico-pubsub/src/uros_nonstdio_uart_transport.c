#include "uros_nonstdio_uart_transport.h" //added by robert blackwell
#include <stdio.h>
#include <sys/types.h> // added by robert blackwell for clockid_t
#include <time.h> // added by robert blackwell
#include <sys/time.h>
#include <pico/error.h> // added by robert blackwell
#include <pico/types.h>
#include "pico/stdlib.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>
#define FTRACE_ON
#include "common/trace.h"

// void usleep(uint64_t us)
// {
//     sleep_us(us);
// }

// int clock_gettime(clockid_t unused, struct timespec *tp)
// {
//     uint64_t m = time_us_64();
//     tp->tv_sec = m / 1000000;
//     tp->tv_nsec = (m % 1000000) * 1000;
//     return 0;
// }

#define UART_REF(transport) ((uart_inst_t*)(transport->args[0]))
uart_inst_t* uart_ref;
int first_time;
bool uros_nonstdio_uart_transport_open(struct uxrCustomTransport * transport)
{
    FTRACE("transport %p  uart_ref: %p \n", transport, transport->args);
    uart_ref = (uart_inst_t*)transport->args;
    // uart_init(uart1, 115200);
    first_time = true;
    return true;
}

bool uros_nonstdio_uart_transport_close(struct uxrCustomTransport * transport)
{
    uart_ref = (uart_inst_t*)transport->args;
    return true;
}

size_t uros_nonstdio_uart_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
    if(first_time) {
        sleep_ms(20);
        // FTRACE(" entered transport %p\n", transport);
        first_time = false;
    }
    sleep_us(100);
    FTRACE(" entered transport %p\n", transport);
    uart_ref = (uart_inst_t*)transport->args;
    for (size_t i = 0; i < len; i++)
    {
        if(uart_is_writable(uart_ref)) {
            uart_putc_raw(uart_ref, (char)buf[i]) ;
        } else {
        // if (buf[i] !=  putchar(buf[i]))
        // {
            *errcode = 1;
            FTRACE(" error transport %p\n", transport);
            return i;
        }
    }
    FTRACE(" exit normal transport %p\n", transport);
    return len;
}

size_t uros_nonstdio_uart_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    // FTRACE(" entered\n","");
    uart_ref = (uart_inst_t*)transport->args;

    uint64_t start_time_us = time_us_64();
    for (size_t i = 0; i < len; i++)
    {
        int64_t remaining_time_us = timeout * 1000 - (time_us_64() - start_time_us);
        if (remaining_time_us < 0)
        {
            *errcode = 1;
            return i;
        }
        
        if(uart_is_readable_within_us(uart_ref, remaining_time_us)) {//;// getchar_timeout_us(elapsed_time_us);
            char c = uart_getc(uart_ref);
            buf[i] = c;
        } else {
            *errcode = 1;
            // FTRACE(" error\n","");
            return i;
        }
    }
    // FTRACE(" exit\n","");
    return len;
}
