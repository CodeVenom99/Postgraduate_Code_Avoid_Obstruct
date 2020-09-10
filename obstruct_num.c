/*自动调节前视角范围，但是还是要根据情况加上固定值，
可能是因为角度控制的效果还不够好（主要表现在超调方面），
主要是因为距离罗盘的分辨率不够 */
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
#define TIME_STEP 16

#define run_code_2 1

#define Max_speed 0.0
//#define left_speed 0.0
//#define right_speed 0.0
#define TURN_COEFFICIENT 15.0
#define Target_angle 20
#define Target_Rad Target_angle/180.0*3.14
#define Target_Distance 0.4
#define KP 200.0
#define KI 1.0
#define KD 0.0
#define Sensor_Data_Num 90//是90不是60是因为方便找波谷
WbDeviceTag wheels[4];
typedef struct sensor_data_struct
{
int angle;//角度
double rad;//弧度
double distance;//距离
}sensor_data_struct;

typedef struct speed_controll_struct
{
double left_speed;
double right_speed;
}speed_controll_struct;

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
int obstruct_num=0;

void findtrough(sensor_data_struct* sensor_data, int size)//障碍划分，运用波谷算法，因为波谷算法无法测算两侧临界边缘，所以扩大了范围
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
		obstruct_num = 0;
		for (i = 0; i < size - 2; i++)
		{

			if (diff[i + 1] - diff[i] == 2)
			{
                              if(i<=59)
                              {
                                 obstruct_num++;//先循环一次，计算障碍个数，2个好处：1：避免创建链表，2：直接得出障碍个数
                                
                              }
    			}
		} 
		printf("障碍个数：%d\r\n",obstruct_num);
		obstruct_data = (obstruct_data_struct*)malloc(sizeof(obstruct_data_struct) * obstruct_num);
		if(obstruct_data==NULL)printf("xxxxxxxxxxxxxxxx");
		obstruct_num = 0;
		for (i = 0; i < size - 2; i++)//再循环一次存储据
		{

			if (diff[i + 1] - diff[i] == 2)
			{
                              if(i<=59)
                              {
                                /*if(i==59)
                                {
                                  obstruct_data[obstruct_num].index = i+1;
                                  obstruct_data[obstruct_num].distance = sensor_data[i+1].distance;
                                  obstruct_data[obstruct_num].angle = sensor_data[i+1].angle;
                                  obstruct_data[obstruct_num].rad = sensor_data[i+1].rad;
                                  obstruct_num++;
                                  //printf("波谷：%d 距离：%f cm 角度：%d\r\n", i+2-60,obstruct_data[i+1].distance,obstruct_data[i+1].angle);
                                }
                                  
                                else
                                {*/
                                  obstruct_data[obstruct_num].index = i+1;
                                  obstruct_data[obstruct_num].distance = sensor_data[i+1].distance;
                                  obstruct_data[obstruct_num].angle = sensor_data[i+1].angle;
                                  obstruct_data[obstruct_num].rad = sensor_data[i+1].rad;
                                  obstruct_num++;
                                  //printf("波谷：%d 距离：%f cm 角度：%d\r\n", i+2,obstruct_data[i+1].distance,obstruct_data[i+1].angle); 
                                //}
                                                    
                              }
		
    			}
		}
		
          	
	}

}

void obstruct_outline(sensor_data_struct* sensor_data, int size)
{
int i=0,index=0;
for(i=0;i<obstruct_num;i++)
  {
  
    for(index=obstruct_data[i].index;sensor_data[index].distance < 100;index--)
    {
    if(index-1==0)
      index=index+60;
    //printf("%d\r\n",index);
    obstruct_data[i].left_index = index;
    }
  
    for(index=obstruct_data[i].index;sensor_data[index].distance < 100;index++)
    {
      if(index>60)
        obstruct_data[i].right_index = index-60;
      else
        obstruct_data[i].right_index = index;
    }
  }
}
void print_obstruct(void)
{
int i=0;
for(i=0;i<obstruct_num;i++)
{
printf("障碍：%d 编号：%d 距离：%f cm 角度：%d  左 %d 右 %d\r\n", 
i+1,obstruct_data[i].index,obstruct_data[i].distance,obstruct_data[i].angle,
obstruct_data[i].left_index,obstruct_data[i].right_index); 
}
printf("------------------\r\n");
//obstruct_num = 0;
}
/*通过距离罗盘计算目标角度和距离*/
void get_distance(WbDeviceTag * sensors,sensor_data_struct * sensor_data)
{
  int i;
  for (i = 0; i < 60 ; i++)
  {
    sensor_data[i].distance = 100*wb_distance_sensor_get_value(sensors[i]);//单位cm

    if(i<=30&&i>=0)
    {
      sensor_data[i].angle=6*i;
      sensor_data[i].rad = sensor_data[i].angle/180*3.14;
    }
    else 
    {
      sensor_data[i].angle=-6*(60-i);
      sensor_data[i].rad = sensor_data[i].angle/180*3.14;
    }
  } 
  //printf("\r\n");
  for (i = 60; i < Sensor_Data_Num ; i++)
  {
    sensor_data[i].distance = 100*wb_distance_sensor_get_value(sensors[i-60]);//单位cm

    sensor_data[i].angle=6 *(i-60);
    sensor_data[i].rad = sensor_data[i-60].angle/180*3.14;
  }    

}
/*控制电机转动*/
void robot_set_speed(speed_controll_struct speed_controll) {
  //printf("left_speed = %f , right_speed = %f\r\n" ,speed_controll.left_speed , speed_controll.right_speed);
  wb_motor_set_velocity(wheels[0],speed_controll.left_speed);
  wb_motor_set_velocity(wheels[1],speed_controll.left_speed);
  wb_motor_set_velocity(wheels[2],speed_controll.right_speed);
  wb_motor_set_velocity(wheels[3],speed_controll.right_speed);
}


int main(int argc, char **argv) {

  int i; 
  sensor_data_struct sensor_data[Sensor_Data_Num]; 
  speed_controll_struct speed_controll;

  WbDeviceTag sensors[60],pen;
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
  while (1) {
  wb_robot_step(10);
  wb_pen_write(pen, 1);
  get_distance(sensors,sensor_data);
  speed_controll.left_speed=10;
  speed_controll.right_speed=10;
  findtrough(sensor_data,Sensor_Data_Num);
  obstruct_outline(sensor_data,Sensor_Data_Num);
  print_obstruct();
  robot_set_speed(speed_controll);
  };
  wb_robot_cleanup();
  return 0;
}

  