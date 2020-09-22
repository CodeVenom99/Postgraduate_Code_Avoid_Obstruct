#include "motion_control.h"
void motion_init(void)
{
  extern char wheels_names[4][15];
  extern WbDeviceTag wheels[4];
  for (int i = 0; i < 4 ; i++){
  wheels[i] = wb_robot_get_device(wheels_names[i]);
  wb_motor_set_position(wheels[i],INFINITY);
  wb_motor_set_velocity(wheels[i],0.0);
  }
}
/*
力转换成角度和速度
Position_Angle 预期角度
speed_controll 左右轮速
*/
void Compute_angle_and_vecitory(Resultant_component_struct Resultant_Force,car_controll_struct * car_controll)
{
#if car_info_printf
  extern char Car_Name[CAR_NUM_EXCEPT_SELF+1][6];
  extern int CAR_Status;  
#endif  
  car_controll->position_rad = Resultant_Force.rad;
  car_controll->position_angle = Resultant_Force.angle;
  car_controll->speed = Resultant_Force.speed;

  car_controll->left_speed = car_controll->speed + TURN_COEFFICIENT * car_controll->position_rad ;
  car_controll->right_speed = car_controll->speed - TURN_COEFFICIENT * car_controll->position_rad ; 
#if car_info_printf
  for(int i=0;i<CAR_NUM_EXCEPT_SELF+1;i++)
  {
    if(strncmp(wb_robot_get_name(), Car_Name[i], 5) == 0)
    {
      if(CAR_Status==1)
         printf("%s在wfm状态\r\n",Car_Name[i]);
      else
         printf("%s在cfm状态\r\n",Car_Name[i]);    
    }    
  }  
  printf("车速：speed = %f , angle = %f , " ,car_controll->speed , car_controll->position_angle);
  printf("left_speed = %f , right_speed = %f\r\n" ,car_controll->left_speed , car_controll->right_speed);

  printf("-------------------------------------------\r\n");
#endif
}
void status_switch(Attract_component_struct Attract_force,
                   Repulsion_component_struct Repulsion_Force,
                   Virtual_Resultant_component_Struct Virtual_Resultant_Force,
                   Resultant_component_struct * Resultant_Force)
{
extern int CAR_Status,switch_delay;
extern double Po;
if(CAR_Status == CFM_Status)
  {
    if(fabs(Attract_force.angle)>90.0)
    {
    switch_delay++;
    if(switch_delay > 100)
    {
      CAR_Status = WFM_Status;
      switch_delay = 0;
      Po = 0.6;
    }    
    }

  }
  else
  {
    if(fabs(Attract_force.angle)<=90.0)//||Repulsion_Force.Force==0
    {
    switch_delay++;
    if(switch_delay>100)
    {
      CAR_Status = CFM_Status;
      switch_delay = 0;
      Po = 0.4;
    }

    }  
  }
  switch (CAR_Status)
  {
    case CFM_Status:
      break;
    case WFM_Status:
    {
      if(fabs(Repulsion_Force.angle)<10||fabs(Repulsion_Force.angle-180)<10)//
      //if(fabs(Repulsion_Force.angle)-90<10)//||fabs(Repulsion_Force.angle-180)<10
        Resultant_Force->speed = 20;
      else
        Resultant_Force->speed = WFM_speed*(1-fabs(cos(Repulsion_Force.rad)));
      
      Resultant_Force->rad = Virtual_Resultant_Force.rad;
      Resultant_Force->angle = Virtual_Resultant_Force.angle;
      break;
    }
    case Keep_Going:
      break;
    case STOP:
    {
      Resultant_Force->speed = 0;
      Resultant_Force->rad = 0;
      Resultant_Force->angle = 0;  
      break;
    }  
  }


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
/*控制电机转动*/
void robot_set_speed(car_controll_struct car_controll,WbDeviceTag * wheels) {
  //printf("left_speed = %f , right_speed = %f\r\n" ,speed_controll.left_speed , speed_controll.right_speed);
  wb_motor_set_velocity(wheels[0],car_controll.left_speed);
  wb_motor_set_velocity(wheels[1],car_controll.left_speed);
  wb_motor_set_velocity(wheels[2],car_controll.right_speed);
  wb_motor_set_velocity(wheels[3],car_controll.right_speed);
}













