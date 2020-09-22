/*
在communication基础上
判断前方是否有阻碍车，根据优先级决定让阻碍车是停还是走
 */
#include "variable.h"
#include "receiver_emitter.h"
#include "cfm.h"
#include "wfm.h"
#include "distance_detect.h"
#include "motion_control.h"
#include "device.h"
#include "trm.h"
char Car_Name[3][6]={"car_1","car_2","car_3"};
char wheels_names[4][15] = {"wheel1_joint", "wheel2_joint", "wheel3_joint", "wheel4_joint"};
char sensors_names[Sensor_Num][4] = {"s1","s2","s3","s4","s5","s6","s7","s8","s9","s10",
                                "s11","s12","s13","s14","s15","s16","s17","s18","s19","s20",
                                "s21","s22","s23","s24","s25","s26","s27","s28","s29","s30",
                                "s31","s32","s33","s34","s35","s36","s37","s38","s39","s40",
                                "s41","s42","s43","s44","s45","s46","s47","s48","s49","s50",
                                "s51","s52","s53","s54","s55","s56","s57","s58","s59","s60",};
double Target_Point[1][2]={{4,4*sqrt(3)}};//假设目标点
WbDeviceTag wheels[4],sensors[Sensor_Num];
WbDeviceTag pen,gps,compass;
int CAR_Status = 0;
int obstruct_num=0;//障碍个数
int Occlusion_ID;//遮挡ID阿

double k = 30;//引力系数
double m = 0.2;//斥力系数
double Po = 0.4;//斥力影响范围,要结合传感器探测距离变化,单位米
double a = 0.5;//次方
                               
int switch_delay=0;
const double *gps_values; 
const double *north;

  
int main(int argc, char **argv) {

  wb_robot_init();
  distance_device_init();
  motion_init();
  communicate_init();
  device_init();  

  while (wb_robot_step(TIME_STEP)!=-1) {

 // wb_robot_step(10);
  wb_pen_write(pen, 1);
  gps_values = wb_gps_get_values(gps);
  north = wb_compass_get_values(compass);
  
  TRM(communicate_data,&Occlusion_ID);
  
  Compute_Target_Attract(north,gps_values,k,&Attract_force);//计算目标点引力
  Get_Distance(sensors,sensor_data);//采集一圈数据
  findtrough(sensor_data,Sensor_Data_Num,&obstruct_data);
  obstruct_outline(sensor_data,Sensor_Data_Num,&obstruct_data);
/**/
  Compute_Repulsion(sensor_data,Sensor_Num,Attract_force,Po,m,a,&Repulsion_Force);//斥力
  CFM(Attract_force,Repulsion_Force,&Resultant_Force);//合力
  WFM(Repulsion_Force,&Virtual_Resultant_Force);
  

  status_switch(Attract_force,Repulsion_Force,Virtual_Resultant_Force,&Resultant_Force);

  Compute_angle_and_vecitory(Resultant_Force,&car_controll);//预期速度，角度

/*  
  car_controll.left_speed=15;
  car_controll.right_speed=15; 
*/ 
  robot_set_speed(car_controll,wheels);//电机控制
  };
  wb_robot_cleanup();
  return 0;
}






  