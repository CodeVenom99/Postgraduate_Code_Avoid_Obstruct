#ifndef _DISTANCE_DETECT_H_
#define _DISTANCE_DETECT_H_
#include "variable.h"
typedef struct obstruct_data_struct
{
int angle;//角度
double rad;//弧度
double distance;//距离
int index;
int left_index;
int right_index;
}obstruct_data_struct;
obstruct_data_struct * obstruct_data;

typedef struct sensor_data_struct
{
double angle;//角度
double rad;//弧度
double distance;//距离
int VFH_data;//0或者1
}sensor_data_struct;//距离罗盘采集的数据
sensor_data_struct sensor_data[Sensor_Data_Num];


void distance_device_init(void);
void Get_Distance(WbDeviceTag * sensors,sensor_data_struct * sensor_data);//通过距离罗盘保存周围一圈的角度和距离数据
void findtrough(sensor_data_struct* sensor_data, int size,obstruct_data_struct ** obstruct_data);
void obstruct_outline(sensor_data_struct* sensor_data, int size,obstruct_data_struct ** obstruct_data);

#endif