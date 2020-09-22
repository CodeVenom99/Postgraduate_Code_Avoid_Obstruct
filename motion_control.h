#ifndef _MOTION_CONTROL_H_
#define _MOTION_CONTROL_H_
#include "variable.h"
#include "cfm.h"
#include "wfm.h"
typedef struct car_controll_struct
{

double left_speed;//预期左轮速度
double right_speed;//预期右轮速度
double speed;//线速度 = （左 + 右）/2
double compass_rad;//罗盘弧度
double compass_angle;
double position_rad;//合力预期弧度
double position_angle;
}car_controll_struct;
car_controll_struct car_controll;//小车控制结构体

void motion_init(void);
void Compute_angle_and_vecitory(Resultant_component_struct Resultant_Force,car_controll_struct * car_controll);//计算预期角度和速度
void fix_direction(double position_angle,const double *north ,car_controll_struct * car_controll);//到达关键点后达到姿态
void robot_set_speed(car_controll_struct car_controll,WbDeviceTag * wheels);//车轮控制
void status_switch(Attract_component_struct Attract_force,
                   Repulsion_component_struct Repulsion_Force,
                   Virtual_Resultant_component_Struct Virtual_Resultant_Force,
                   Resultant_component_struct * Resultant_Force);
#endif