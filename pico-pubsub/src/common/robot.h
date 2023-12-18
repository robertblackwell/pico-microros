#ifndef H_robot_h
#define H_robot_h

void robot_init();
void robot_tasks_run();
void robot_cmd_rpm(double left, double right);
void robot_cmd_stop();
#endif