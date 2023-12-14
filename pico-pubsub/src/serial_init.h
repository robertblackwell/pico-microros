#ifndef H_serial_h
#define H_serial_h

/**
 * This function contains the code the configures the serial input/output for a pico program.
 * 
 * It decides, based on compile time options set in the CMakeLists.txt file,
 * 
 * -    the micro-ROS line
 *      -   the port (usb, uart0, uart1) that will be used for the micro-ROS link,
 *      -   whether the micro-ROS link will be configured with stdio support, and 
 *      -   if so what special features it requires such as turning off CR/LF processing
 * 
 * -    a serial output port for telementry and printf() debugging
 *      -   if the micro-ROS port is running as stdio there will be NO telemetry port
 *          -   and use of printf() is prohibited
 *      -   this is the configuration provided in the github example [https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk] 
 * 
 * Our goal:
 *      -   is to have a micro-ROS port not running as stdio and a telemetry port running stdio
 *          thereby allowing micro-ROS comms and telemetry.
 * 
 * How  
 *      -   the micro-ROS port will be uart1 running without stdio support
*       -   stdio will use either the USB port or uart0 either of which can be configured for stdio stdio support.
*/

void serial_init();

#endif