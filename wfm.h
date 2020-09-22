#ifndef _WFM_H_
#define _WFM_H_
#include "variable.h"
#include "cfm.h"
typedef struct Virtual_Resultant_component_Struct
{
double angle;//角度
double rad;//弧度
double Fatx;//引力在自身坐标下的x轴分量
double Fatz;//引力在自身坐标下的z轴分量
double Force;//引力=sqrt(pow(Fatx,2) + pow(Fatz,2));
}Virtual_Resultant_component_Struct;//存储目标点的对于车身的信息
Virtual_Resultant_component_Struct Virtual_Resultant_Force;

void WFM(Repulsion_component_struct Repulsion_Force,Virtual_Resultant_component_Struct * Virtual_Resultant_Force);

#endif