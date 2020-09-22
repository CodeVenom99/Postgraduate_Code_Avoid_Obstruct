#ifndef _VARIABLE_H_
#define _VARIABLE_H_

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/compass.h>
#include <webots/device.h>
#include <webots/pen.h>
#include <webots/gps.h>
#include <webots/receiver.h>
#include <webots/emitter.h>

#define COMMUNICATION_CHANNEL 1
#define CAR_NUM_EXCEPT_SELF 1
#define TIME_STEP 4
#define PI (4*atan(1))

//打印车辆信息的预条件编译
#define car_info_printf 1

#define car_force_info_printf 1
//打印障碍信息的预条件编译
#define obstruct_info_printf 0

#define communication_info 0

#define TURN_COEFFICIENT 4
//TURN_COEFFICIENT=4.5,k=100在坑里的效果较好
#define Max_speed 20
#define Object_speed 10
#define WFM_speed 20
#define Sensor_Num 60
#define Sensor_Data_Num 90//是90不是60是因为方便找波谷
#define Virtual_Attract_Repulsion_Force_Angle 90

#define CFM_Status 0
#define WFM_Status 1
#define Keep_Going 2
#define STOP       3
#endif