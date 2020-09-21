#ifndef _RECEIVER_EMITTER_H_
#define _RECEIVER_EMITTER_H_
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/robot.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "variable.h"

WbDeviceTag receiver,emitter;
typedef struct communicate_struct
{
int id;//角度
double dis;//弧度
double angle;//距离
}communicate_struct;//距离罗盘采集的数据
communicate_struct communicate_data[CAR_NUM_EXCEPT_SELF];



void communicate_init(void);
void emitter_id(void);
void receiver_id(communicate_struct *communicate_data);

#endif