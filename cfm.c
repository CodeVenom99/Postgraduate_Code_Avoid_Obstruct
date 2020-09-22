#include "cfm.h"
/*
该函数计算目标引力
gps_values 传感器采集到的车身数据
k 引力系数
Attract_force 目标点信息
*/
void Compute_Target_Attract(const double *north , const double *gps_values,double k,Attract_component_struct * Attract_force)//只计算前方目标角度
{
double x,z,compass_angle,compass_rad,gps_angle,gps_rad; 
extern double Target_Point[1][2];
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
#if car_force_info_printf
printf("目标坐标：z = %f  x = %f 小车坐标：z = %f  x = %f \r\n",Target_Point[0][0],Target_Point[0][1],gps_values[2],gps_values[0]);
printf("指南针 = %f  目标在小车世界坐标系中角度 = %f \r\n",compass_angle,gps_angle);
printf("在车身坐标系下：目标弧度 = %.2f , 目标角度 = %.2f , 目标距离 = %.2f\r\n" ,Attract_force->rad,Attract_force->angle,Attract_force->distance);
printf("引力：  Force = %.2f , angle = %.2f , 分量：Fatz = %.2f , Fatx = %.2f\r\n" ,Attract_force->Force,Attract_force->angle,Attract_force->Fatz,Attract_force->Fatx);
//printf("-------------------------------------------\r\n");
#endif
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
    Repulsion_Force->Fatazz = 0;
    Repulsion_Force->Fataxx = 0;
    Repulsion_Force->Force = sqrt(pow(Repulsion_Force->Frerzz + Repulsion_Force->Fatazz,2) + pow(Repulsion_Force->Frerxx + Repulsion_Force->Fataxx,2));
    Repulsion_Force->rad = atan2(Repulsion_Force->Frerzz + Repulsion_Force->Fatazz,Repulsion_Force->Frerxx + Repulsion_Force->Fataxx);
    Repulsion_Force->angle = Repulsion_Force->rad/PI*180;
  
#if car_force_info_printf
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
//  static double pre_angle=0;
  if(Repulsion_Force. Force==0)
  {
    Resultant_Force->speed =Max_speed;//没有斥力就保持最大速度
    Resultant_Force->rad = Attract_force.rad;
   Resultant_Force->angle = Attract_force.angle;
   
  }
  else
  {
  Resultant_Force->rad = atan2(Resultant_Force->Fsumz,Resultant_Force->Fsumx);
  Resultant_Force->angle = Resultant_Force->rad/PI*180;
/*
  printf("前: angle = %f ,pre_angle = %f\r\n " ,Resultant_Force->angle,pre_angle);

  Resultant_Force->angle =(1 * Resultant_Force->angle + (4-1)*pre_angle)/4;
  pre_angle=Resultant_Force->angle;

  Resultant_Force->rad = Resultant_Force->angle/180*PI;
  printf("后: angle = %f \r\n " ,Resultant_Force->angle);
*/  
  Resultant_Force->Fsumz =  Attract_force.Fatz + Repulsion_Force.Frerzz + Repulsion_Force.Fatazz;
  Resultant_Force->Fsumx =  Attract_force.Fatx + Repulsion_Force.Frerxx + Repulsion_Force.Fataxx;
  Resultant_Force->Fsum = sqrt(pow(Resultant_Force->Fsumz,2)+pow(Resultant_Force->Fsumx,2));
  if(fabs(Repulsion_Force.angle)<20||fabs(fabs(Repulsion_Force.angle)-180)<20)
    Resultant_Force->speed =Object_speed;
  else
    Resultant_Force->speed = Max_speed*(1-fabs(cos(Repulsion_Force.rad)));
  
  //Resultant_Force->Fsum =Object_speed;
  }

#if car_force_info_printf

  printf("合力：  Force= %.2f , angle = %.2f , rad = %.2f, ", Resultant_Force->Fsum,Resultant_Force->angle,Resultant_Force->rad);
  printf("分量：Fsumz = %.2f Fsumx = %.2f \r\n", Resultant_Force->Fsumz, Resultant_Force->Fsumx);
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