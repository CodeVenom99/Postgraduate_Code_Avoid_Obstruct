#ifndef _CFM_H_
#define _CFM_H_
#include "variable.h"
#include "distance_detect.h"

typedef struct Attract_component_struct
{
double angle;//角度
double rad;//弧度
double distance;//距离
double Fatx;//引力在自身坐标下的x轴分量
double Fatz;//引力在自身坐标下的z轴分量
double Force;//引力=sqrt(pow(Fatx,2) + pow(Fatz,2));
}Attract_component_struct;//存储目标点的对于车身的信息
Attract_component_struct Attract_force;

typedef struct Repulsion_component_struct
{
double Frerzz;//斥力在 斥力 方向上分解到z轴的力
double Frerxx;//斥力在 斥力 方向上分解到x轴的力
double Fatazz;//斥力在 引力 方向上分解到z轴的力
double Fataxx;//斥力在 引力 方向上分解到x轴的力
double Force;
double rad;
double angle;
}Repulsion_component_struct;//斥力结构体
Repulsion_component_struct Repulsion_Force;

typedef struct Resultant_component_struct
{
double Fsumz;//合力在z轴分量
double Fsumx;//合力在x轴分量
double Fsum;//合力
double angle;
double rad;
double speed;
}Resultant_component_struct;//合力结构体
Resultant_component_struct Resultant_Force;

void Compute_Target_Attract(const double *north , const double *gps_values,double k,Attract_component_struct * Attract_force);//只计算前方目标角度
void Compute_Repulsion(sensor_data_struct * sensor_data,int sensor_num,
                      Attract_component_struct Attract_force,
                      double Po,double m,double a,
                      Repulsion_component_struct * Repulsion_Force);//计算斥力
void CFM( Attract_component_struct  Attract_force,
                              Repulsion_component_struct Repulsion_Force,
                              Resultant_component_struct * Resultant_Force);//计算合力
double sum(double* example, int n);//数组加和






#endif