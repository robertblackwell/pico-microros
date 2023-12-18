#include <stdio.h>
#include <stdint.h>
#include <pico.h>
#include <pico/types.h>
#include "pico/stdlib.h"
// #include <pico/stdio_usb.h>

#include <hardware/structs/uart.h>
#include <hardware/regs/dreq.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <rmw_microros/rmw_microros.h>

#include "uros_stdio_transport.h"
#include "uros_nonstdio_uart_transport.h"
#define FTRACE_ON
#include "common/trace.h"

#if 0
#define UART_ID uart1
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
#endif
#define RETCHECK(ret, msg) print_fmt("ret = %d msg: %s\n", ret, msg);


const char* publisher_topic = "pico_publisher_topic";
const char* subscriber_topic = "pico_subscriber_topic";

const uint LED_PIN = 25;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

#define ENABLE_PUBLISHER

#if defined ENABLE_PUBLISHER
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    FTRACE("Timer callback msg.data %d\n", msg.data);
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data++;
}
#endif

rcl_subscription_t subscriber;
std_msgs__msg__String sub_msg;
char  sub_buffer[512];

void subscriber_callback(const void* msgin)
{
    if(msgin) {
        FTRACE("subscriber_callback Not NULL %s\n", ((std_msgs__msg__String*)msgin)->data.data);
    } else {
        FTRACE("subscriber_callback NULL\n", "");
    }

}
bool pingAgent(){
    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 100;
    const uint8_t attempts = 1;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK){
    	// gpio_put(LED_PIN, 0);
    	return false;
    } else {
    	// gpio_put(LED_PIN, 1);
        return true;
    }
}

int main()
{

    #if 0
    stdio_init_all();
    uart_init(uart1, 115200);
    gpio_set_function(8, GPIO_FUNC_UART);
    gpio_set_function(9, GPIO_FUNC_UART);

    sleep_ms(5000);
    printf("about to initialize transport uart0: %p uart1: %p\n", uart0, uart1);
    rmw_uros_set_custom_transport(
		true,
		uart1,
		uros_nonstdio_uart_transport_open,
		uros_nonstdio_uart_transport_close,
		uros_nonstdio_uart_transport_write,
		uros_nonstdio_uart_transport_read
	);    
    #else
    // init telemetry output
    // uart_init(uart1, 115200);
    // gpio_set_function(8, GPIO_FUNC_UART);
    // gpio_set_function(9, GPIO_FUNC_UART);
    // init micro-ros link
    stdio_init_all();
    trace_init_full(uart1, 115200, 8, 9);
    rmw_uros_set_custom_transport(
		true,
		NULL,
		uros_stdio_transport_open,
		uros_stdio_transport_close,
		uros_stdio_transport_write,
		uros_stdio_transport_read
	);
    #endif

    sleep_ms(5000);
    FTRACE("Starting main\n","");
    // This trick is thanks to Jon Durrant (youtube)[https://www.youtube.com/watch?v=IPxyBB2nrXE] and (github)[https://github.com/jondurrant/RPIPicoMicroRosServoExp]
    while(!pingAgent()) {
        sleep_ms(1000);
        FTRACE("waiting for agent\n","");
    }
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;
    rclc_executor_t executor_02;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;
    rcl_ret_t ret;
    
    ret = rmw_uros_ping_agent(timeout_ms, attempts);
    RETCHECK(ret, "Ping Agent")
    FTRACE("stop 3\n");

    ret = rclc_support_init(&support, 0, NULL, &allocator);
    RETCHECK(ret, "support initt")

    ret = rclc_node_init_default(&node, "pico_node", "", &support);
    RETCHECK(ret, "node init")
    size_t nodes_domain_id;
    ret = rcl_node_get_domain_id(&node, &nodes_domain_id);
    RETCHECK(ret, "Get domain id")
    FTRACE("Nodes domain id: %d \n", nodes_domain_id);


    sub_buffer[0] = '\0';
    sub_msg.data.data = &sub_buffer[0];
    sub_msg.data.capacity = 511;
    sub_msg.data.size = 0;
    ret = rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        subscriber_topic
    );
    RETCHECK(ret, "subscription init")

    #if defined ENABLE_PUBLISHER
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
    #endif
    
    ret = rclc_executor_init(&executor, &support.context, 2, &allocator);
    RETCHECK(ret, "executor_init")

    #if defined ENABLE_PUBLISHER
    ret = rclc_executor_add_timer(&executor, &timer);
    RETCHECK(ret, "add timer")
    #endif

    ret = rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &subscriber_callback, ON_NEW_DATA);
    RETCHECK(ret, "add subscriber")

    msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
    return 0;
}
