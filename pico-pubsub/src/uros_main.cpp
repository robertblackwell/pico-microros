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
#include "common/config.h"
#include "common/trace.h"
#include "common/argv.h"
#include "common/commands.h"
#include "common/dri0002.h"
#include "common/encoder.h"
#include "common/motion.h"
#include "common/reporter.h"
#include "common/robot.h"

#define RETCHECK(ret, msg) print_fmt("ret = %d msg: %s\n", ret, msg);


const char* publisher_topic = "pico_publisher_topic";
const char* subscriber_topic = "pico_subscriber_topic";

const uint LED_PIN = 25;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rcl_subscription_t subscriber;
std_msgs__msg__String sub_msg;
char  sub_buffer[512];
rcl_timer_t timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rclc_executor_t executor_02;

#if 0
DRI0002V1_4 dri0002{
		MOTOR_RIGHT_DRI0002_SIDE, 
		MOTOR_RIGHT_PWM_PIN, 				// E1
		MOTOR_RIGHT_DIRECTION_SELECT_PIN, 	// M1
		
		MOTOR_LEFT_DRI0002_SIDE, 
		MOTOR_LEFT_PWM_PIN, 				// E2
		MOTOR_LEFT_DIRECTION_SELECT_PIN	    // E2
};

/**
 * Encoders must funnel all their interrupts through a common isr handler but must pass a pointer to their
 * Encoder istance to that common isr. This is the least tricky way I can find of doing that
*/
void isr_apin_left();
void isr_bpin_left();

Encoder encoder_left{MOTOR_LEFT_ID, MOTOR_LEFT_NAME, MOTOR_LEFT_ENCODER_A_INT, MOTOR_LEFT_ENCODER_B_INT, isr_apin_left, isr_bpin_left};
void isr_apin_left(){encoder_common_isr(&encoder_left);}
void isr_bpin_left(){ /*encoder_common_isr(&encoder_left)*/;} // only want interrupts on the a-pin

void isr_apin_right();
void isr_bpin_right();
Encoder encoder_right{MOTOR_RIGHT_ID, MOTOR_RIGHT_NAME, MOTOR_RIGHT_ENCODER_A_INT, MOTOR_RIGHT_ENCODER_B_INT, isr_apin_right, isr_bpin_right};
void isr_apin_right(){encoder_common_isr(&encoder_right);}
void isr_bpin_right(){ /*encoder_common_isr(&encoder_right)*/;} // only want interrupts on the a-pin

Encoder* encoder_left_ptr = &encoder_left;
Encoder* encoder_right_ptr = &encoder_right;
MotionControl motion_controller{&dri0002, encoder_left_ptr, encoder_right_ptr};
Reporter reporter{encoder_left_ptr, encoder_right_ptr};
Task motion_task{2000, &motion_controller};
Task report_task{20, &reporter};
#endif
#define ENABLE_PUBLISHER

#if defined ENABLE_PUBLISHER
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    FTRACE("Timer callback msg.data %d\n", msg.data);
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data++;
}
#endif


void subscriber_callback(const void* msgin)
{
    if(msgin) {
        FTRACE("subscriber_callback Not NULL %s\n", ((std_msgs__msg__String*)msgin)->data.data);
        Argv            argv;
        CommandBuffer   cmd_buf;
        tokenize_line(((std_msgs__msg__String*)msgin)->data.data, argv);
        cmd_buf.fill_from_tokens(argv);
        FTRACE("subscriber command is %s\n", clicommand2string(cmd_buf.m_command_enum))
        if(cmd_buf.identity() == CliCommands::MotorsRpm) {
            auto left = cmd_buf.motors_rpm_command.m_left_rpm;
            auto right = cmd_buf.motors_rpm_command.m_right_rpm;
            robot_cmd_rpm(left, right);
            // motion_controller.pid_set_rpm(left, right);
            FTRACE("after moction_controller.pid_set_rpm left_rpm: %f right: %f\n", left, right);
        }

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
void init_transport_trace();
void init_robot_hw();
void init_ros_node();
int main()
{
    init_transport_trace();
    sleep_ms(5000);
    FTRACE("Starting main\n","");
    while(!pingAgent()) {
        sleep_ms(1000);
        FTRACE("waiting for agent\n","");
    }
    // init_robot_hw();
    robot_init();
    init_ros_node();

    msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        robot_tasks_run();
        // motion_task();
		// report_task();
    }
    return 0;
}
void init_transport_trace()
{
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
}
#if 0
void init_robot_hw()
{
	encoder_left_ptr->start_interrupts();
	encoder_right_ptr->start_interrupts();
}
#endif
void init_ros_node()
{
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
        RCL_MS_TO_NS(10000),
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
}
