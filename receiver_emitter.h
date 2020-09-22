#ifndef _RECEIVER_EMITTER_H_
#define _RECEIVER_EMITTER_H_

#include "variable.h"

WbDeviceTag receiver,emitter;
typedef struct communicate_struct
{
int id;//角度
int occlusion_id;
double dis;//弧度
double angle;//距离
double rad;
}communicate_struct;//距离罗盘采集的数据
communicate_struct communicate_data[CAR_NUM_EXCEPT_SELF];



void communicate_init(void);
void emitter_info(int *Occlusion_ID);
void receiver_info(communicate_struct *communicate_data);

#endif