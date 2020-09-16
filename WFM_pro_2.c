/*
在cfm的基础之上实现wfm
但是将wfm的速度设置为20或者更快会崩溃，这主要是因为在转弯后没能即使i调整方向，
再加上车速过快，导致墙超出了斥力范围导致
解决办法：
1.pid
2.低通滤波器  x
3.动态斥力范围
Resultant_Force.Fsum = 15*(1-fabs(cos(Repulsion_Force.rad)));
cfm斥力范围0.6
wfm斥力范围0.4
 */
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
#define TIME_STEP 4

//打印车辆信息的预条件编译
#define car_info_printf 1
//打印障碍信息的预条件编译
#define obstruct_info_printf 1

#define PI (4*atan(1))
#define TURN_COEFFICIENT 5.0

#define Max_speed 20
#define Object_speed 15
#define WFM_speed 15
#define Sensor_Data_Num 60//是90不是60是因为方便找波谷
#define Virtual_Attract_Repulsion_Force_Angle 90
WbDeviceTag wheels[4];

int obstruct_num=0;//障碍个数
double Target_Point[1][2]={{-4,-4*sqrt(3)}};//假设目标点

double k = 5;//引力系数
double m = 0.2;//斥力系数
double Po = 0.4;//斥力影响范围,要结合传感器探测距离变化,单位米
double a = 0.5;//次方

typedef struct sensor_data_struct
{
double angle;//角度
double rad;//弧度
double distance;//距离
int VFH_data;//0或者1
}sensor_data_struct;//距离罗盘采集的数据
sensor_data_struct sensor_data[Sensor_Data_Num];

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

typedef struct obstruct_data_struct
{
double angle;//障碍的角度
double rad;//障碍的弧度
double distance;//障碍的距离
}obstruct_data_struct;//障碍的数据
obstruct_data_struct * obstruct_data = NULL;


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
}Resultant_component_struct;//合力结构体
Resultant_component_struct Resultant_Force;

typedef struct Virtual_Resultant_component_Struct
{
double angle;//角度
double rad;//弧度
double Fatx;//引力在自身坐标下的x轴分量
double Fatz;//引力在自身坐标下的z轴分量
double Force;//引力=sqrt(pow(Fatx,2) + pow(Fatz,2));
}Virtual_Resultant_component_Struct;//存储目标点的对于车身的信息
Virtual_Resultant_component_Struct Virtual_Resultant_Force;

void Compute_Target_Attract(const double *north , const double *gps_values,double k,Attract_component_struct * Attract_force);//只计算前方目标角度
void Get_Distance(WbDeviceTag * sensors,sensor_data_struct * sensor_data);//通过距离罗盘保存周围一圈的角度和距离数据
void print_obstruct(void);//打印障碍信息
double sum(double* example, int n);//数组加和
void Compute_Repulsion(sensor_data_struct * sensor_data,int sensor_num,
                      Attract_component_struct Attract_force,
                      double Po,double m,double a,
                      Repulsion_component_struct * Repulsion_Force);//计算斥力
void CFM( Attract_component_struct  Attract_force,
                              Repulsion_component_struct Repulsion_Force,
                              Resultant_component_struct * Resultant_Force);//计算合力
void Compute_angle_and_vecitory(Resultant_component_struct Resultant_Force,car_controll_struct * car_controll);//计算预期角度和速度
void fix_direction(double position_angle,const double *north ,car_controll_struct * car_controll);//到达关键点后达到姿态
void robot_set_speed(car_controll_struct car_controll,WbDeviceTag * wheels);//车轮控制
void WFM(Repulsion_component_struct Repulsion_Force,Virtual_Resultant_component_Struct * Virtual_Resultant_Force);                                               
void State_switching(Attract_component_struct  Attract_force,Repulsion_component_struct Repulsion_Force);
int main(int argc, char **argv) {

  int i =0;
  int WFM_Status = 0;
  const double *gps_values; 
  const double *north;
  WbDeviceTag sensors[60],pen,gps,compass;
  wb_robot_init();

  char wheels_names[4][15] = {"wheel1_joint", "wheel2_joint", "wheel3_joint", "wheel4_joint"};
  char sensors_names[60][20] = {"s1","s2","s3","s4","s5","s6","s7","s8","s9","s10",
                                "s11","s12","s13","s14","s15","s16","s17","s18","s19","s20",
                                "s21","s22","s23","s24","s25","s26","s27","s28","s29","s30",
                                "s31","s32","s33","s34","s35","s36","s37","s38","s39","s40",
                                "s41","s42","s43","s44","s45","s46","s47","s48","s49","s50",
                                "s51","s52","s53","s54","s55","s56","s57","s58","s59","s60",};
  for (i = 0; i < 4 ; i++){
  wheels[i] = wb_robot_get_device(wheels_names[i]);
  wb_motor_set_position(wheels[i],INFINITY);
  wb_motor_set_velocity(wheels[i],0.0);
  }
  for (i = 0; i < 60 ; i++){
  sensors[i] = wb_robot_get_device(sensors_names[i]);
  wb_distance_sensor_enable(sensors[i],TIME_STEP);
  }    
  pen = wb_robot_get_device("pen");
  gps = wb_robot_get_device("gps");
  compass = wb_robot_get_device("compass");
  wb_gps_enable(gps, TIME_STEP);
  wb_compass_enable(compass, TIME_STEP);
  while (1) {
  wb_robot_step(10);
  wb_pen_write(pen, 1);
  gps_values = wb_gps_get_values(gps);
  north = wb_compass_get_values(compass);
  Compute_Target_Attract(north,gps_values,k,&Attract_force);//计算目标点引力
  Get_Distance(sensors,sensor_data);//采集一圈数据
 
  Compute_Repulsion(sensor_data,Sensor_Data_Num,Attract_force,Po,m,a,&Repulsion_Force);//斥力
  WFM(Repulsion_Force,&Virtual_Resultant_Force);
  CFM(Attract_force,Repulsion_Force,&Resultant_Force);//合力
/**/
  if(WFM_Status == 0)
  {
    if(fabs(Attract_force.angle)>90.0)
    {
      WFM_Status = 1;
      Po = 0.6;
      printf("进入wfm状态\r\n");
    }
  }
  else
  {
    if(fabs(Attract_force.angle)<=90.0)//||Repulsion_Force.Force==0
    {
      WFM_Status = 0;
      Po = 0.4;
      printf("进入cfm状态\r\n");
    }  
  }
  if(WFM_Status == 1)
  {
    Resultant_Force.Fsumz = Virtual_Resultant_Force.Fatz*5;
    Resultant_Force.Fsumx =Virtual_Resultant_Force.Fatx*5;
    //Resultant_Force.Fsum = WFM_speed;
    
    /**/
    if(Repulsion_Force.rad==0||Repulsion_Force.rad==180)//
      Resultant_Force.Fsum = 20;
    else
      Resultant_Force.Fsum = WFM_speed*(1-fabs(cos(Repulsion_Force.rad)));
    
    Resultant_Force.rad = Virtual_Resultant_Force.rad;
    Resultant_Force.angle = Virtual_Resultant_Force.angle;

  }

  Compute_angle_and_vecitory(Resultant_Force,&car_controll);//预期速度，角度

/*
  car_controll.left_speed=5;
  car_controll.right_speed=5; 
  */ 
  robot_set_speed(car_controll,wheels);//电机控制
  };
  wb_robot_cleanup();
  return 0;
}

void WFM(Repulsion_component_struct Repulsion_Force,Virtual_Resultant_component_Struct  * Virtual_Resultant_Force)
{
//double err;
if(Repulsion_Force.angle > 0)
  Virtual_Resultant_Force->angle = Repulsion_Force.angle - Virtual_Attract_Repulsion_Force_Angle;
else if(Repulsion_Force.angle < 0)
  Virtual_Resultant_Force->angle = Repulsion_Force.angle + Virtual_Attract_Repulsion_Force_Angle; 
else
  Virtual_Resultant_Force->angle =Attract_force.angle;
  
Virtual_Resultant_Force->Force = Repulsion_Force.Force;

Virtual_Resultant_Force->rad = Virtual_Resultant_Force->angle/180.0*PI;

Virtual_Resultant_Force->Fatz = sin(Virtual_Resultant_Force->rad) * Virtual_Resultant_Force->Force;
Virtual_Resultant_Force->Fatx = cos(Virtual_Resultant_Force->rad) * Virtual_Resultant_Force->Force;
#if car_info_printf
printf("虚拟力: Force = %.2f , angle = %.2f\r\n" ,Virtual_Resultant_Force->Force,Virtual_Resultant_Force->angle);
//printf("虚拟分量：Fatz = %.2f , Fatx = %.2f\r\n" ,Virtual_Resultant_Force->Fatz,Virtual_Resultant_Force->Fatx);
//printf("-------------------------------------------\r\n");
#endif
}
/*
通过距离罗盘保存周围一圈的角度和距离数据
规定车身正前方为0，左边是负角度，右边是正角度
*/
void Get_Distance(WbDeviceTag * sensors,sensor_data_struct * sensor_data)
{
  int i;
  for (i = 0; i < 60 ; i++)
  {
    sensor_data[i].distance = wb_distance_sensor_get_value(sensors[i]);//单位m
    sensor_data[i].angle=6*i;
    sensor_data[i].rad = sensor_data[i].angle/180*PI+PI;
  } 
  //printf("%f\r\n",sensor_data[15].distance);
}
/*
该函数计算目标引力
gps_values 传感器采集到的车身数据
k 引力系数
Attract_force 目标点信息
*/
void Compute_Target_Attract(const double *north , const double *gps_values,double k,Attract_component_struct * Attract_force)//只计算前方目标角度
{
double x,z,compass_angle,compass_rad,gps_angle,gps_rad; 
z=Target_Point[0][0] - gps_values[2];
x=Target_Point[0][1] - gps_values[0];

Attract_force->distance = sqrt(pow(x,2)+pow(z,2));//目标距离

compass_rad = atan2(north[0], north[2]);
compass_angle = (compass_rad - 1.5708) / PI * 180.0;
if (compass_angle < 0.0)
    compass_angle = compass_angle + 360.0;
compass_rad =compass_angle/180.0*PI;//指南针弧度,0～2pi

gps_rad = atan2(z, x);
if (gps_rad < 0.0)
    gps_rad = gps_rad + 2*PI;
gps_angle =gps_rad/PI*180.0;//目标在小车世界坐标系中角度,0～2pi
Attract_force->angle = gps_angle - compass_angle;//目标在车身坐标系下的夹角

if(Attract_force->angle < -180)
  Attract_force->angle = Attract_force->angle + 360;
if(Attract_force->angle > 180)
  Attract_force->angle = Attract_force->angle - 360;
/**/

  
Attract_force->rad = Attract_force->angle/180*PI;

Attract_force->Fatx = k *Attract_force->distance * cos(Attract_force->rad);
Attract_force->Fatz = k *Attract_force->distance * sin(Attract_force->rad);
Attract_force->Force = sqrt(pow(Attract_force->Fatx,2) + pow(Attract_force->Fatz,2));
#if car_info_printf
printf("目标坐标：z = %f  x = %f 小车坐标：z = %f  x = %f \r\n",Target_Point[0][0],Target_Point[0][1],gps_values[2],gps_values[0]);
printf("指南针 = %f  目标在小车世界坐标系中角度 = %f \r\n",compass_angle,gps_angle);
printf("在车身坐标系下：目标弧度 = %.2f , 目标角度 = %.2f , 目标距离 = %.2f\r\n" ,Attract_force->rad,Attract_force->angle,Attract_force->distance);
printf("引力：  Force = %.2f , angle = %.2f , 分量：Fatz = %.2f , Fatx = %.2f\r\n" ,Attract_force->Force,Attract_force->angle,Attract_force->Fatz,Attract_force->Fatx);
//printf("-------------------------------------------\r\n");
#endif

}
/*数组加和*/
double sum(double* example, int n)
{
	int i;
	double sum = 0;
	for (i = 0; i < n; i++)
	{
		sum  = sum + example[i];
	}
	return sum;
}
/*
该函数计算障碍斥力
obstruct_data 障碍信息
obstruct_num 障碍个数
Attract_force 目标点信息
Po 斥力影响范围
m 斥力系数
a 次方
Frerzz 斥力在 斥力 方向上分解到z轴的力
Frerxx 斥力在 斥力 方向上分解到x轴的力
Fatazz 斥力在 引力 方向上分解到z轴的力
Fataxx 斥力在 引力 方向上分解到x轴的力
*/
void Compute_Repulsion(sensor_data_struct * sensor_data,int sensor_num,
                      Attract_component_struct Attract_force,
                      double Po,double m,double a,
                      Repulsion_component_struct * Repulsion_Force
                      )
{
	
  int i = 0;
  double* Frer = (double*)malloc(sizeof(double) * sensor_num);
  double* Fata = (double*)malloc(sizeof(double) * sensor_num);
  double* Frerz = (double*)malloc(sizeof(double) * sensor_num);
  double* Frerx = (double*)malloc(sizeof(double) * sensor_num);
  double* Fataz = (double*)malloc(sizeof(double) * sensor_num);
  double* Fatax = (double*)malloc(sizeof(double) * sensor_num);
  if (Frer && Fata && Frerz && Frerx && Fataz && Fatax)
  {
  for(i = 0;i < sensor_num;i++)
  {
    if(sensor_data[i].distance > Po)
    {
      Frerz[i]=0;
      Frerx[i]=0;
      Fataz[i]=0;
      Fatax[i]=0;
    }
    else
    {
      if (sensor_data[i].distance < Po / 2.0)
      {
        Frer[i] = m * (1 / sensor_data[i].distance - 1 / Po) * (1 / pow(sensor_data[i].distance,2)) * pow(Attract_force.distance, a);
        Fata[i] = a * m * pow((1 / sensor_data[i].distance - 1 / Po), 2) * pow(Attract_force.distance, (a-1));
        Frerz[i] = Frer[i] * sin(sensor_data[i].rad);
        Frerx[i] = Frer[i] * cos(sensor_data[i].rad);
        Fataz[i] = Fata[i] * sin(Attract_force.rad);
        Fatax[i] = Fata[i] * cos(Attract_force.rad);
      } 
      else
      {
      Frer[i] = m * (1 / sensor_data[i].distance - 1 / Po) * (1 / pow(sensor_data[i].distance,2)) * pow(Attract_force.distance,2);
      Fata[i] = 2 * m * pow((1 / sensor_data[i].distance - 1 / Po), 2) * Attract_force.distance;
      Frerz[i] = Frer[i] * sin(sensor_data[i].rad);
      Frerx[i] = Frer[i] * cos(sensor_data[i].rad);
      Fataz[i] = Fata[i] * sin(Attract_force.rad);
      Fatax[i] = Fata[i] * cos(Attract_force.rad);
      }   
    }
   }
    Repulsion_Force->Frerzz = sum(Frerz, sensor_num);
    Repulsion_Force->Frerxx = sum(Frerx, sensor_num);
    Repulsion_Force->Fatazz = sum(Fataz, sensor_num);
    Repulsion_Force->Fataxx = sum(Fatax, sensor_num);
    Repulsion_Force->Force = sqrt(pow(Repulsion_Force->Frerzz + Repulsion_Force->Fatazz,2) + pow(Repulsion_Force->Frerxx + Repulsion_Force->Fataxx,2));
    Repulsion_Force->rad = atan2(Repulsion_Force->Frerzz + Repulsion_Force->Fatazz,Repulsion_Force->Frerxx + Repulsion_Force->Fataxx);
    Repulsion_Force->angle = Repulsion_Force->rad/PI*180;
  
#if car_info_printf
  printf("斥力：  Force = %.2f , angle = %.2f , 分量：", Repulsion_Force->Force, Repulsion_Force->angle);
  printf("Frerzz = %.2f , Frerxx = %.2f , ", Repulsion_Force->Frerzz, Repulsion_Force->Frerxx);
  printf("Fatazz = %.2f , Fataxx = %.2f\r\n", Repulsion_Force->Fatazz, Repulsion_Force->Fataxx); 
  //printf("-------------------------------------------\r\n");
#endif
   }

	free(Frer);
	free(Fata);
	free(Frerz);
	free(Frerx);
	free(Fataz);
	free(Fatax);
	Frer = Fata = Frerz = Frerx = Fataz = Fatax = NULL;
} 

/*
计算合力
Fsumz z轴上所有力的加和
Fsumx x轴上所有力的加和
*/
void CFM( Attract_component_struct  Attract_force,
                              Repulsion_component_struct Repulsion_Force,
                              Resultant_component_struct * Resultant_Force)
{
  if(Repulsion_Force. Force==0)
  {
    Resultant_Force->Fsum =Max_speed;//没有斥力就保持最大速度
    Resultant_Force->rad = Attract_force.rad;
   Resultant_Force->angle = Attract_force.angle;
  }
  else
  {
  Resultant_Force->Fsumz =  Attract_force.Fatz + Repulsion_Force.Frerzz + Repulsion_Force.Fatazz;
  Resultant_Force->Fsumx =  Attract_force.Fatx + Repulsion_Force.Frerxx + Repulsion_Force.Fataxx;
  //Resultant_Force->Fsum = 20*(1-fabs(cos(Repulsion_Force.rad)));
  Resultant_Force->Fsum =Object_speed;
  Resultant_Force->rad = atan2(Resultant_Force->Fsumz,Resultant_Force->Fsumx);
  Resultant_Force->angle = Resultant_Force->rad/PI*180;

  }

#if car_info_printf

  printf("合力：  Force= %.2f , angle = %.2f , rad = %.2f, ", Resultant_Force->Fsum,Resultant_Force->angle,Resultant_Force->rad);
  printf("分量：Fsumz = %.2f Fsumx = %.2f \r\n", Resultant_Force->Fsumz, Resultant_Force->Fsumx);
  //printf("-------------------------------------------\r\n");
#endif
}
/*
力转换成角度和速度
Position_Angle 预期角度
speed_controll 左右轮速
*/
void Compute_angle_and_vecitory(Resultant_component_struct Resultant_Force,car_controll_struct * car_controll)
{
  car_controll->position_rad = Resultant_Force.rad;
  car_controll->position_angle = Resultant_Force.angle;
  
  car_controll->speed = Resultant_Force.Fsum;
/*
  if(fabs(Resultant_Force.Fsum) < Max_speed )
    car_controll->speed = Resultant_Force.Fsum;
  else if(Resultant_Force.Fsum < -Max_speed)
    car_controll->speed = -Max_speed;
  else
    car_controll->speed = Max_speed;
*/
  //printf("angle = %d , rad = %f , min_distance = %f\r\n" ,sensor_data.angle , sensor_data.rad , sensor_data.min_distance);
  car_controll->left_speed = car_controll->speed + TURN_COEFFICIENT * car_controll->position_rad;
  car_controll->right_speed = car_controll->speed - TURN_COEFFICIENT * car_controll->position_rad; 
#if car_info_printf
  printf("车速：speed = %f , rad = %f , " ,car_controll->speed , car_controll->position_rad);
  printf("left_speed = %f , right_speed = %f\r\n" ,car_controll->left_speed , car_controll->right_speed);
  printf("-------------------------------------------\r\n");
#endif
}
/*控制电机转动*/
void robot_set_speed(car_controll_struct car_controll,WbDeviceTag * wheels) {
  //printf("left_speed = %f , right_speed = %f\r\n" ,speed_controll.left_speed , speed_controll.right_speed);
  wb_motor_set_velocity(wheels[0],car_controll.left_speed);
  wb_motor_set_velocity(wheels[1],car_controll.left_speed);
  wb_motor_set_velocity(wheels[2],car_controll.right_speed);
  wb_motor_set_velocity(wheels[3],car_controll.right_speed);
}
void fix_direction(double position_angle,const double *north ,car_controll_struct * car_controll)//position_rad是预期角度
{
  car_controll->compass_rad = atan2(north[0], north[2]);
  car_controll->compass_angle = (car_controll->compass_rad - 1.5708) / PI * 180.0;
  if (car_controll->compass_angle < 0.0)
    car_controll->compass_angle = car_controll->compass_angle + 360.0;
  car_controll->compass_rad = car_controll->compass_angle/180.0*PI;//指南针弧度,0～2pi
  car_controll->compass_rad = (position_angle - car_controll->compass_angle)/180.0*PI;
  
  car_controll->left_speed = 0 + TURN_COEFFICIENT * car_controll->compass_rad;
  car_controll->right_speed = 0 - TURN_COEFFICIENT * car_controll->compass_rad; 
}
  