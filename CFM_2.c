/*在CFM.c之上，
针对避障速度缓慢（特别是对于墙的避障）参考论文，
依据引力和斥力夹角，如果是钝角将斥力向引力方向旋转90度后计算合力，如果是锐角就不动。
论文中说的好处是解决局部最小值，其实还能解决避障速度缓慢
但是这样的处理过于粗暴，有一定的改善但是不够从好，不好在对于墙的避障，不能选择较好的转向
（主要因为没有考虑到墙的大小，不是根据障碍大小决定朝哪转，而是通过与引力的夹角决定朝哪转，这是错误的）
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
#define obstruct_info_printf 0

#define PI (4*atan(1))
#define Max_speed 15.0
#define TURN_COEFFICIENT 15.0
#define Sensor_Data_Num 90//是90不是60是因为方便找波谷

WbDeviceTag wheels[4];

int obstruct_num=0;//障碍个数
double Target_Point[1][2]={{-4,4*sqrt(3)}};//假设目标点

double k = 5;//引力系数
double m = 0.5;//斥力系数
double Po = 0.6;//斥力影响范围,要结合传感器探测距离变化,单位米
double a = 0.5;//次方

typedef struct sensor_data_struct
{
double angle;//角度
double rad;//弧度
double distance;//距离
}sensor_data_struct;//距离罗盘采集的数据
sensor_data_struct sensor_data[Sensor_Data_Num];

typedef struct car_controll_struct
{

double left_speed;//预期左轮速度
double right_speed;//预期右轮速度
double speed;//线速度 = （左 + 右）/2
double position_rad;//预期弧度
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
obstruct_data_struct * test_obstruct_data = NULL;

typedef struct Target_Point_info_struct
{
double angle;//角度
double rad;//弧度
double distance;//距离
double Fatx;//引力在自身坐标下的x轴分量
double Fatz;//引力在自身坐标下的z轴分量
}Target_Point_info_struct;//存储目标点的对于车身的信息
Target_Point_info_struct Target_Point_Attract;

typedef struct Repulsion_component_struct
{
double Frerzz;//斥力在 斥力 方向上分解到z轴的力
double Frerxx;//斥力在 斥力 方向上分解到x轴的力
double Fatazz;//斥力在 引力 方向上分解到z轴的力
double Fataxx;//斥力在 引力 方向上分解到x轴的力
}Repulsion_component_struct;//斥力结构体
Repulsion_component_struct Repulsion_component;

typedef struct Resultant_Force_struct
{
double Fsumz;//合力在z轴分量
double Fsumx;//合力在x轴分量
double Fsum;//合力
}Resultant_Force_struct;//合力结构体
Resultant_Force_struct Resultant_Force;

void Compute_Target_Attract(const double *north , const double *gps_values,double k,Target_Point_info_struct * Target_Point_Attract);//只计算前方目标角度
void Get_Distance(WbDeviceTag * sensors,sensor_data_struct * sensor_data);//通过距离罗盘保存周围一圈的角度和距离数据
void Find_Trough(sensor_data_struct* sensor_data, int size,int * obstruct_num,obstruct_data_struct ** obstruct_data);//障碍划分，运用波谷算法，因为波谷算法无法测算两侧临界边缘，所以扩大了范围
void print_obstruct(void);//打印障碍信息
double sum(double* example, int n);//数组加和
void Compute_Repulsion(obstruct_data_struct * obstruct_data,int obstruct_num,
                      Target_Point_info_struct Target_Point_Attract,
                      double Po,double m,double a,
                      Repulsion_component_struct * Repulsion_component);//计算斥力
void Compute_Resultant_Force( Target_Point_info_struct  Target_Point_Attract,
                              Repulsion_component_struct Repulsion_component,
                              Resultant_Force_struct * Resultant_Force);//计算合力
void Compute_angle_and_vecitory(Resultant_Force_struct Resultant_Force,car_controll_struct * car_controll);//计算预期角度和速度
void fix_direction(car_controll_struct * car_controll);//车身角度控制 
void robot_set_speed(car_controll_struct car_controll);//车轮控制
                                               

int main(int argc, char **argv) {

  int i =0;
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
  Compute_Target_Attract(north,gps_values,k,&Target_Point_Attract);//计算目标点引力
   
  Get_Distance(sensors,sensor_data);//采集一圈数据
  Find_Trough(sensor_data,Sensor_Data_Num,&obstruct_num,&obstruct_data);//障碍划分，存储障碍信息保存在obstruct_data，obstruct_data是全局变量

  Compute_Repulsion(obstruct_data,obstruct_num,Target_Point_Attract,Po,m,a,&Repulsion_component);//斥力
  Compute_Resultant_Force(Target_Point_Attract,Repulsion_component,&Resultant_Force);//合力
  Compute_angle_and_vecitory(Resultant_Force,&car_controll);//预期速度，角度
  fix_direction(&car_controll);//角度控制
/*
  car_controll.left_speed=5;
  car_controll.right_speed=5; 
  */ 
  robot_set_speed(car_controll);//电机控制
  };
  wb_robot_cleanup();
  return 0;
}


void Find_Trough(sensor_data_struct* sensor_data, int size,int * obstruct_num,obstruct_data_struct ** obstruct_data)//障碍划分，运用波谷算法，因为波谷算法无法测算两侧临界边缘，所以扩大了范围
{
	int i = 0;
	int* diff = (int*)malloc(sizeof(int) * size-1);
	if (diff)
	{
		for (i = 0; i < size-1; i++)
		{
			if (sensor_data[i + 1].distance - sensor_data[i] .distance> 0)
			{
				diff[i] = 1;
			}
			else if (sensor_data[i + 1].distance - sensor_data[i] .distance < 0)
			{
				diff[i] = -1;
			}
			else
			{
				diff[i] = 0;
			}
			//printf("%d", diff[i]);
		}
		for (i = size - 2; i >= 0; i--)
		{
			if (diff[i] == 0)
			{
				if (diff[i+1] >= 0)
					diff[i] = 1;
				else
					diff[i] = -1;			
			}

			//printf("%d ", diff[i]);
		}
		*obstruct_num = 0;
		for (i = 0; i < size - 2; i++)
		{

			if (diff[i + 1] - diff[i] == 2)
			{
                              if(i<=59)
                              {
                                 (*obstruct_num)++;//先循环一次，计算障碍个数，2个好处：1：避免创建链表，2：直接得出障碍个数
                                
                              }
    			}
		} 
		//printf("障碍个数：%d\r\n",obstruct_num);
		*obstruct_data = (obstruct_data_struct*)malloc(sizeof(obstruct_data_struct) * (*obstruct_num));
		if(*obstruct_data==NULL)printf("xxxxxxxxxxxxxxxx");
		*obstruct_num = 0;
		for (i = 0; i < size - 2; i++)//再循环一次存储据
		{

			if (diff[i + 1] - diff[i] == 2)
			{
                              if(i<=59)
                              {
                                if(i==59)
                                {
                                  (*obstruct_data)[*obstruct_num].distance = sensor_data[i+1].distance;
                                  (*obstruct_data)[*obstruct_num].angle = sensor_data[i+1].angle + 180;//斥力是反向的，所以角度加180
                                  (*obstruct_data)[*obstruct_num].rad = sensor_data[i+1].rad + PI;
                                  (*obstruct_num)++;
                                  //printf("波谷：%d 距离：%f cm 角度：%d\r\n", i+2-60,obstruct_data[i+1].distance,obstruct_data[i+1].angle);
                                }
                                  
                                else
                                {
                                  (*obstruct_data)[*obstruct_num].distance = sensor_data[i+1].distance;
                                  (*obstruct_data)[*obstruct_num].angle = sensor_data[i+1].angle + 180;//斥力是反向的，所以角度加180
                                  (*obstruct_data)[*obstruct_num].rad = sensor_data[i+1].rad + PI;
                                  (*obstruct_num)++;
                                  //printf("波谷：%d 距离：%f cm 角度：%d\r\n", i+2,obstruct_data[i+1].distance,obstruct_data[i+1].angle); 
                                }                                                    
                              }		
    			}
		} 
		#if obstruct_info_printf 
		for(i=0;i<(*obstruct_num);i++)
                 {
                 printf("障碍：%d 距离：%f cm 角度：%f (斥力方向+180)：%f\r\n", i+1,(*obstruct_data)[i].distance,(*obstruct_data)[i].angle-180,(*obstruct_data)[i].angle); 
                 }  
                 printf("-------------------------------------------\r\n");
                 #endif      	
	}

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
    sensor_data[i].rad = sensor_data[i].angle/180*PI;
  } 
  //printf("\r\n");
  for (i = 60; i < Sensor_Data_Num ; i++)
  {
    sensor_data[i].distance = wb_distance_sensor_get_value(sensors[i-60]);//单位m
    sensor_data[i].angle=6 *(i-60);
    sensor_data[i].rad = sensor_data[i-60].angle/180*PI;
  }    

}
/*
该函数计算目标引力
gps_values 传感器采集到的车身数据
k 引力系数
Target_Point_Attract 目标点信息
*/
void Compute_Target_Attract(const double *north , const double *gps_values,double k,Target_Point_info_struct * Target_Point_Attract)//只计算前方目标角度
{
double x,z,compass_angle,compass_rad,gps_angle,gps_rad; 
z=Target_Point[0][0] - gps_values[2];
x=Target_Point[0][1] - gps_values[0];

Target_Point_Attract->distance = sqrt(pow(x,2)+pow(z,2));//目标距离

compass_rad = atan2(north[0], north[2]);
compass_angle = (compass_rad - 1.5708) / PI * 180.0;
if (compass_angle < 0.0)
    compass_angle = compass_angle + 360.0;
compass_rad =compass_angle/180.0*PI;//指南针弧度

gps_rad = atan2(z, x);
if (gps_rad < 0.0)
    gps_rad = gps_rad + 2*PI;
gps_angle =gps_rad/PI*180.0;//目标在世界坐标系中角度

Target_Point_Attract->angle = gps_angle - compass_angle;//目标在车身坐标系下的夹角
Target_Point_Attract->rad = Target_Point_Attract->angle/180*PI;

Target_Point_Attract->Fatx = k *Target_Point_Attract->distance * cos(Target_Point_Attract->rad);
Target_Point_Attract->Fatz = k *Target_Point_Attract->distance * sin(Target_Point_Attract->rad);

#if car_info_printf
printf("目标坐标：z = %f  x = %f 小车坐标：z = %f  x = %f \r\n",Target_Point[0][0],Target_Point[0][1],gps_values[2],gps_values[0]);
printf("指南针 = %f  目标在世界坐标系中角度 = %f \r\n",compass_angle,gps_angle);
printf("在车身坐标系下：目标弧度 = %.2f , 目标角度 = %.2f , 目标距离 = %.2f\r\n" ,Target_Point_Attract->rad,Target_Point_Attract->angle,Target_Point_Attract->distance);
printf("引力：Fatz = %.2f , Fatx = %.2f\r\n" ,Target_Point_Attract->Fatz,Target_Point_Attract->Fatx);
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
Target_Point_Attract 目标点信息
Po 斥力影响范围
m 斥力系数
a 次方
Frerzz 斥力在 斥力 方向上分解到z轴的力
Frerxx 斥力在 斥力 方向上分解到x轴的力
Fatazz 斥力在 引力 方向上分解到z轴的力
Fataxx 斥力在 引力 方向上分解到x轴的力
*/
void Compute_Repulsion(obstruct_data_struct * obstruct_data,int obstruct_num,
                      Target_Point_info_struct Target_Point_Attract,
                      double Po,double m,double a,
                      Repulsion_component_struct * Repulsion_component
                      )
{
	
  int i = 0;
  double* Frer = (double*)malloc(sizeof(double) * obstruct_num);
  double* Fata = (double*)malloc(sizeof(double) * obstruct_num);
  double* Frerz = (double*)malloc(sizeof(double) * obstruct_num);
  double* Frerx = (double*)malloc(sizeof(double) * obstruct_num);
  double* Fataz = (double*)malloc(sizeof(double) * obstruct_num);
  double* Fatax = (double*)malloc(sizeof(double) * obstruct_num);
  if (Frer && Fata && Frerz && Frerx && Fataz && Fatax)
  {
  for(i = 0;i < obstruct_num;i++)
  {
    if(obstruct_data[i].distance > Po)
    {
      Frerz[i]=0;
      Frerx[i]=0;
      Fataz[i]=0;
      Fatax[i]=0;
    }
    else
    {
      if (obstruct_data[i].distance < Po / 2.0)
      {
        Frer[i] = m * (1 / obstruct_data[i].distance - 1 / Po) * (1 / pow(obstruct_data[i].distance,2)) * pow(Target_Point_Attract.distance, a);
        Fata[i] = a * m * pow((1 / obstruct_data[i].distance - 1 / Po), 2) * pow(Target_Point_Attract.distance, (a-1));
        Frerz[i] = Frer[i] * sin(obstruct_data[i].rad);
        Frerx[i] = Frer[i] * cos(obstruct_data[i].rad);
        Fataz[i] = Fata[i] * sin(Target_Point_Attract.rad);
        Fatax[i] = Fata[i] * cos(Target_Point_Attract.rad);
      } 
      else
      {
	Frer[i] = m * (1 / obstruct_data[i].distance - 1 / Po) * (1 / pow(obstruct_data[i].distance,2)) * pow(Target_Point_Attract.distance,2);
	Fata[i] = 2 * m * pow((1 / obstruct_data[i].distance - 1 / Po), 2) * Target_Point_Attract.distance;
        Frerz[i] = Frer[i] * sin(obstruct_data[i].rad);
        Frerx[i] = Frer[i] * cos(obstruct_data[i].rad);
        Fataz[i] = Fata[i] * sin(Target_Point_Attract.rad);
        Fatax[i] = Fata[i] * cos(Target_Point_Attract.rad);
      }   
    }
   }
    Repulsion_component->Frerzz = sum(Frerz, obstruct_num);
    Repulsion_component->Frerxx = sum(Frerx, obstruct_num);
    Repulsion_component->Fatazz = sum(Fataz, obstruct_num);
    Repulsion_component->Fataxx = sum(Fatax, obstruct_num);
#if car_info_printf
  printf("Frerzz = %f Frerxx = %f\r\n", Repulsion_component->Frerzz, Repulsion_component->Frerxx);
  printf("Fatazz = %f,Fataxx = %f\r\n", Repulsion_component->Fatazz, Repulsion_component->Fataxx); 
  //printf("-------------------------------------------\r\n");
#endif
   }

	free(Frer);
	free(Fata);
	free(Frerz);
	free(Frerx);
	free(Fataz);
	free(Fatax);
	Frer =  Fata = Frerz = Frerx = Fataz = Fatax = NULL;
} 

/*
计算合力
Fsumz z轴上所有力的加和
Fsumx x轴上所有力的加和
*/
void Compute_Resultant_Force( Target_Point_info_struct  Target_Point_Attract,
                              Repulsion_component_struct Repulsion_component,
                              Resultant_Force_struct * Resultant_Force)
{
  double attract_angle_1,attract_angle_2,attract_angle_3;
  
  double Frerzz,Frerxx;
  
  //double Frerzz = 10,Frerxx = -1;
  //double Frerzz=-1,Frerxx=10;
  double repulsion_angle = atan2(Repulsion_component.Frerzz,Repulsion_component.Frerxx)/PI*180;//斥力角  
  //double repulsion_angle = atan2(Frerzz,Frerxx)/PI * 180;//斥力角

  double attract_angle = atan2(Target_Point_Attract.Fatz,Target_Point_Attract.Fatx)/PI*180;//引力角
  //double attract_angle = 240;
  
  double repulsion = sqrt(pow(Repulsion_component.Frerzz,2) + pow(Repulsion_component.Frerxx,2));
  //double repulsion = sqrt(pow(Frerzz,2) + pow(Frerxx,2));
  repulsion_angle = ((int)repulsion_angle + 360) % 360 + repulsion_angle - (int)repulsion_angle;
  attract_angle = ((int)attract_angle + 360) % 360 + attract_angle - (int)attract_angle;  
  printf("斥力角 = %f 引力角 = %f\r\n",repulsion_angle,attract_angle);
  //printf("Frerzz = %f Frerxx = %f\r\n", Repulsion_component.Frerzz, Repulsion_component.Frerxx);
  //printf("Frerzz = %f Frerxx = %f\r\n", Frerzz, Frerxx);
    
  //attract_angle_0 = ((int)attract_angle + 0) % 360 + attract_angle - (int)attract_angle;
  attract_angle_1 = ((int)attract_angle + 90) % 360 + attract_angle - (int)attract_angle;
  attract_angle_2 = ((int)attract_angle + 180) % 360 + attract_angle - (int)attract_angle;
  attract_angle_3 = ((int)attract_angle + 270) % 360 + attract_angle - (int)attract_angle;
  
  //printf("%f %f %f %f\r\n",attract_angle_0,attract_angle_1,attract_angle_2,attract_angle_3);
  if(attract_angle >270 || attract_angle <90)
  {
    if(repulsion_angle < attract_angle_2 && repulsion_angle > attract_angle_1)
    {
      repulsion_angle = repulsion_angle - 90;
      // printf("四\r\n"); 
    }
      
    else if(repulsion_angle < attract_angle_3 && repulsion_angle > attract_angle_2)
    {
      repulsion_angle = repulsion_angle + 90;
       //printf("三\r\n");
    }  
  }
  else if(attract_angle > 90 && attract_angle <180)
  {
    if(repulsion_angle < attract_angle_2 && repulsion_angle > attract_angle_1)
    {
      repulsion_angle = repulsion_angle - 90;
       //printf("四\r\n"); 
    }
      
    else if(((repulsion_angle > 0)&&(repulsion_angle < attract_angle_3)) || ((repulsion_angle < 360)&&(repulsion_angle > attract_angle_2)))
    {
      repulsion_angle = repulsion_angle + 90;
      // printf("三\r\n");
    }   
  }
  else if(attract_angle < 270 && attract_angle > 180)
  {
    if(((repulsion_angle > 0)&&(repulsion_angle < attract_angle_2)) || ((repulsion_angle > attract_angle_1)&&(repulsion_angle < 360)))
    {
      repulsion_angle = repulsion_angle - 90;
      // printf("四\r\n"); 
    }
      
    else if(repulsion_angle < attract_angle_3 && repulsion_angle > attract_angle_2)
    {
      repulsion_angle = repulsion_angle + 90;
       //printf("三\r\n");
    }  
  }
  printf("斥力角 = %f\r\n",repulsion_angle);
  Frerzz = repulsion * sin(repulsion_angle/180.0*PI);
  Frerxx = repulsion * cos(repulsion_angle/180.0*PI);
  //printf("Frerzz = %f Frerxx = %f\r\n", Frerzz,Frerxx);  
  Resultant_Force->Fsumz = Target_Point_Attract.Fatz + Frerzz + Repulsion_component.Fatazz;
  Resultant_Force->Fsumx = Target_Point_Attract.Fatx + Frerxx + Repulsion_component.Fataxx;
  Resultant_Force->Fsum = sqrt(pow(Resultant_Force ->Fsumz,2) + pow(Resultant_Force ->Fsumx,2));
  printf("-------------------------------------------\r\n");
#if car_info_printf
  printf("Fsumz = %f Fsumx = %f 合力大小 = %f\r\n", Resultant_Force->Fsumz, Resultant_Force->Fsumx, Resultant_Force->Fsum);
  //printf("-------------------------------------------\r\n");
#endif
}
/*
力转换成角度和速度
Position_Angle 预期角度
speed_controll 左右轮速
*/
void Compute_angle_and_vecitory(Resultant_Force_struct Resultant_Force,car_controll_struct * car_controll)
{
  car_controll->position_rad = atan2(Resultant_Force.Fsumz,Resultant_Force.Fsumx);
  car_controll->position_angle = car_controll->position_rad/PI*180;
  
  if(Resultant_Force.Fsum < Max_speed && Resultant_Force.Fsum > -Max_speed)
    car_controll->speed = Resultant_Force.Fsum;
  else
  car_controll->speed = Max_speed;
}
/*控制电机转动*/
void robot_set_speed(car_controll_struct car_controll) {
  //printf("left_speed = %f , right_speed = %f\r\n" ,speed_controll.left_speed , speed_controll.right_speed);
  wb_motor_set_velocity(wheels[0],car_controll.left_speed);
  wb_motor_set_velocity(wheels[1],car_controll.left_speed);
  wb_motor_set_velocity(wheels[2],car_controll.right_speed);
  wb_motor_set_velocity(wheels[3],car_controll.right_speed);
}
void fix_direction(car_controll_struct * car_controll)//position_rad是预期角度
{
  //printf("angle = %d , rad = %f , min_distance = %f\r\n" ,sensor_data.angle , sensor_data.rad , sensor_data.min_distance);
  car_controll->left_speed = car_controll->speed + TURN_COEFFICIENT * car_controll->position_rad;
  car_controll->right_speed = car_controll->speed - TURN_COEFFICIENT * car_controll->position_rad; 
#if car_info_printf
  printf("合力角度 = %f 合力弧度 = %f\r\n",car_controll->position_angle,car_controll->position_rad);
  printf("left_speed = %f , right_speed = %f\r\n" ,car_controll->left_speed , car_controll->right_speed);
  printf("-------------------------------------------\r\n");
#endif
}
  