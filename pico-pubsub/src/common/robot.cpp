
#define FTRACE_ON
#include "config.h"
#include "trace.h"
#include "argv.h"
#include "commands.h"
#include "dri0002.h"
#include "encoder.h"
#include "motion.h"
#include "reporter.h"


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

void robot_init()
{
	encoder_left_ptr->start_interrupts();
	encoder_right_ptr->start_interrupts();
}

void robot_tasks_run()
{
	motion_task();
	report_task();
}

void robot_cmd_rpm(double left, double right)
{
	motion_controller.pid_set_rpm(left, right);
}

void robot_cmd_stop()
{
	motion_controller.pid_set_rpm(0.0, 0.0);
}