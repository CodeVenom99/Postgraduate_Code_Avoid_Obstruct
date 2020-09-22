#include "receiver_emitter.h"

void communicate_init(void)
{
  receiver= wb_robot_get_device("receiver");
  emitter= wb_robot_get_device("emitter");

  wb_receiver_set_channel(receiver,COMMUNICATION_CHANNEL); 
  wb_emitter_set_channel(emitter, COMMUNICATION_CHANNEL);
  
  wb_receiver_enable(receiver, TIME_STEP);//采样设置为一样

}

void emitter_info(int *Occlusion_ID)
{

  int i;
  int id[2];
  extern char Car_Name[CAR_NUM_EXCEPT_SELF+1][6];
  for(i=0;i<CAR_NUM_EXCEPT_SELF+1;i++)
  {
    if(strncmp(wb_robot_get_name(), Car_Name[i], 5) == 0)
    {
      id[0] =  atoi(&Car_Name[i][4]);
      id[1] = *Occlusion_ID;
      break;
    }
  }
  wb_emitter_send(emitter, &id, 2 * sizeof(int));
#if communication_info
  for(i = 0 ; i < CAR_NUM_EXCEPT_SELF ; i++)
  {
    printf("发送数据：%d %d \n",id[0],id[1]);
  }
#endif  
}



void receiver_info(communicate_struct *communicate_data)
{
  int i;
  int *temp;
  const double *angle_xyz;
  for(i = 0 ; i < CAR_NUM_EXCEPT_SELF ; i++)
  {
    //printf("%d\r\n",wb_receiver_get_queue_length(receiver));
    if(wb_receiver_get_queue_length(receiver)>0)
    {
      temp = (int *)wb_receiver_get_data(receiver);
      communicate_data[i].id = *temp;
      communicate_data[i].occlusion_id = *(temp+1);
      communicate_data[i].dis=sqrt(1/wb_receiver_get_signal_strength(receiver));
      angle_xyz=wb_receiver_get_emitter_direction(receiver);
      communicate_data[i].rad=atan2(angle_xyz[2],angle_xyz[0]);
      communicate_data[i].angle=communicate_data[i].rad/PI*180;
      wb_receiver_next_packet(receiver);   
    }
 
  }
#if communication_info
  for(i = 0 ; i < CAR_NUM_EXCEPT_SELF ; i++)
  {
    printf("接收数据：%d %d %f %f\n",communicate_data[i].id,communicate_data[i].occlusion_id,communicate_data[i].dis,communicate_data[i].angle);
  }
#endif  
}









