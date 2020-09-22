#include "trm.h"
//交通模式
void TRM(communicate_struct * communicate_data,int *Occlusion_ID)
{
  extern int CAR_Status;
  int i,self_id=0;
  extern char Car_Name[CAR_NUM_EXCEPT_SELF+1][6];
  for(i=0;i<CAR_NUM_EXCEPT_SELF+1;i++)
  {
    if(strncmp(wb_robot_get_name(), Car_Name[i], 5) == 0)
    {
      self_id =  atoi(&Car_Name[i][4]); 
    }  
  }
  receiver_info(communicate_data);

  for(i=0;i<CAR_NUM_EXCEPT_SELF;i++)
  {
  printf("%f %f \n",communicate_data[i].dis,fabs(communicate_data[i].angle));
    if(communicate_data[i].occlusion_id == self_id)
      CAR_Status = communicate_data[i].id > self_id ? Keep_Going : STOP;

    if(communicate_data[i].dis < 0.4 && fabs(communicate_data[i].angle)<45)//说明有车在前方
      *Occlusion_ID = communicate_data[i].id;
    else
      *Occlusion_ID = 0;    
  }
  emitter_info(Occlusion_ID);
}

























