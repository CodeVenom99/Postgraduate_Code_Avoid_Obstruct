#ifndef _VARIABLE_H_
#define _VARIABLE_H_

#define COMMUNICATION_CHANNEL 1
#define CAR_NUM_EXCEPT_SELF 2
#define TIME_STEP 4
#define PI (4*atan(1))
//打印车辆信息的预条件编译
#define car_info_printf 1
//打印障碍信息的预条件编译
#define obstruct_info_printf 1

#define PI (4*atan(1))
#define TURN_COEFFICIENT 5
//TURN_COEFFICIENT=4.5,k=100在坑里的效果较好
#define Max_speed 20
#define Object_speed 10
#define WFM_speed 20
#define Sensor_Data_Num 60//是90不是60是因为方便找波谷
#define Virtual_Attract_Repulsion_Force_Angle 90

#endif