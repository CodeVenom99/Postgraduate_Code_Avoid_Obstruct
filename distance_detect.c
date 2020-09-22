#include "distance_detect.h"
void distance_device_init(void)
{
extern char sensors_names[Sensor_Num][4];
extern WbDeviceTag sensors[Sensor_Num];
  for (int i = 0; i < Sensor_Num ; i++){
  sensors[i] = wb_robot_get_device(sensors_names[i]);
  wb_distance_sensor_enable(sensors[i],TIME_STEP);
  }    
}
/*
通过距离罗盘保存周围一圈的角度和距离数据
规定车身正前方为0，左边是负角度，右边是正角度
*/
void Get_Distance(WbDeviceTag * sensors,sensor_data_struct * sensor_data)
{
/**/
  int i;
  for (i = 0; i < 60 ; i++)
  {
    sensor_data[i].distance = wb_distance_sensor_get_value(sensors[i]);//单位m

    if(i<=30&&i>=0)
    {
      sensor_data[i].angle=6*i;
      sensor_data[i].rad = sensor_data[i].angle/180*PI+PI;
    }
    else 
    {
      sensor_data[i].angle=-6*(60-i);
      sensor_data[i].rad = sensor_data[i].angle/180*PI+PI;
    }
  } 

  for (i = 60; i < Sensor_Data_Num ; i++)
  {
    sensor_data[i].distance = wb_distance_sensor_get_value(sensors[i-60]);//单位cm
    sensor_data[i].angle=6 *(i-60);
    sensor_data[i].rad = sensor_data[i-60].angle/180*PI+PI;
  } 
  /*  
  for (i = 0; i <= 5 ; i++)
  {
    printf("%f ",sensor_data[i].distance);
  }  
   printf("\r\n");
  for (i = 55; i <= 65 ; i++)
  {
    printf("%f ",sensor_data[i].distance);
  }  
   printf("\r\n");    
*/
}
void findtrough(sensor_data_struct* sensor_data, int size,obstruct_data_struct ** obstruct_data)//障碍划分，运用波谷算法，因为波谷算法无法测算两侧临界边缘，所以扩大了范围
{
	int i = 0;
	extern int obstruct_num;
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
                                 //printf("i：%d\r\n",i);
                              }
    			}
		} 
		 
		*obstruct_data = (obstruct_data_struct*)malloc(sizeof(obstruct_data_struct) * obstruct_num);
		if(obstruct_data)
		{
		obstruct_num = 0;
		for (i = 0; i < size - 2; i++)//再循环一次存储据
		{
			if (diff[i + 1] - diff[i] == 2)
			{
                              if(i<=59)
                              {
                                  (*obstruct_data)[obstruct_num].index = i+1;
                                  (*obstruct_data)[obstruct_num].distance = sensor_data[i+1].distance;
                                  (*obstruct_data)[obstruct_num].angle = sensor_data[i+1].angle;
                                  (*obstruct_data)[obstruct_num].rad = sensor_data[i+1].rad;
                                  obstruct_num++;
                                  //printf("波谷：%d 距离：%f cm 角度：%d\r\n", i+2,obstruct_data[i+1].distance,obstruct_data[i+1].angle);                  
                              }
    			}
		}		
		}
        	
	}
        free(diff);
}

void obstruct_outline(sensor_data_struct* sensor_data, int size,obstruct_data_struct ** obstruct_data)
{
int i=0,index=0;
extern int obstruct_num;
for(i=0;i<obstruct_num;i++)
  {
    for(index=(*obstruct_data)[i].index;sensor_data[index].distance < 1;index--)
    {
    if(index-1==0)
      index=index+60;
    (*obstruct_data)[i].left_index = index;
    }

    for(index=(*obstruct_data)[i].index;sensor_data[index].distance < 1;index++)
    {
      if(index>60)
        (*obstruct_data)[i].right_index = index-60;
      else
        (*obstruct_data)[i].right_index = index;
    }
  }
#if obstruct_info_printf
  printf("障碍个数：%d\r\n",obstruct_num);
  for(i=0;i<obstruct_num;i++)
  {
  printf("障碍：%d 编号：%d 距离：%f cm 角度：%d  左 %d 右 %d\r\n", 
  i+1, (*obstruct_data)[i].index, (*obstruct_data)[i].distance, (*obstruct_data)[i].angle,
   (*obstruct_data)[i].left_index, (*obstruct_data)[i].right_index); 
  }
  //printf("------------------\r\n");  
#endif
}
