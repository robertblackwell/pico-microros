#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#define UART_ID uart0
#define UART_BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1
void  uart_printf(const char* fmt, ...);

#define FERROR(msg)             \
do{                             \
    uart_puts(UART_ID, "Fatal Error ");  \
    uart_puts(UART_ID, msg);             \
    uart_puts(UART_ID, "\n");            \
    sleep_ms(2000);             \
}while(1);

#define RETCHECK(ret, msg)      \
uart_printf("ret = %d msg: %s\n", ret, msg);
// if (ret != RCL_RET_OK)          \
// {                               \
//     FERROR(msg)                 \
// }

void  uart_printf(const char* fmt, ...)
{
    // return;
	static char str_buf[512];
    va_list(args);
    va_start(args, fmt);
    int len = vsnprintf(str_buf, 512, fmt, args);
    va_end(args);
    uart_puts(UART_ID, str_buf);

}

const char* publisher_topic = "pico_publisher_topic";

const uint LED_PIN = 25;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    uart_printf("Timer callback msg.data %d\n", msg.data);
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data++;
}

int main()
{
    stdio_init_all();
    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);
    sleep_ms(5000);
    uart_printf("Starting main\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;
    uart_printf("stop 1\n");
    allocator = rcl_get_default_allocator();
    uart_printf("stop 2\n");

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;
    rcl_ret_t ret;
    
    ret = rmw_uros_ping_agent(timeout_ms, attempts);
    RETCHECK(ret, "Ping Agent")
    uart_printf("stop 3\n");

    ret = rclc_support_init(&support, 0, NULL, &allocator);
    RETCHECK(ret, "support initt")

    ret = rclc_node_init_default(&node, "pico_node", "", &support);
    RETCHECK(ret, "node init")
    ret = rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        publisher_topic);
    RETCHECK(ret, "Publisher Init")

    ret = rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);
    RETCHECK(ret, "timer init")

    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    RETCHECK(ret, "executor_init")
    ret = rclc_executor_add_timer(&executor, &timer);
    RETCHECK(ret, "add timer")

    gpio_put(LED_PIN, 1);

    msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
