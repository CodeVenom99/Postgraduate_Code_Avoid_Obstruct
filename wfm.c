#include "wfm.h"

void WFM(Repulsion_component_struct Repulsion_Force,Virtual_Resultant_component_Struct  * Virtual_Resultant_Force)
{
//double err;
if(Repulsion_Force.angle > 0)
  Virtual_Resultant_Force->angle = Repulsion_Force.angle - Virtual_Attract_Repulsion_Force_Angle;
else if(Repulsion_Force.angle < 0)
  Virtual_Resultant_Force->angle = Repulsion_Force.angle + Virtual_Attract_Repulsion_Force_Angle; 
else
  Virtual_Resultant_Force->angle =Attract_force.angle;
 Virtual_Resultant_Force->rad = Virtual_Resultant_Force->angle/180.0*PI; 

Virtual_Resultant_Force->Force = Repulsion_Force.Force;

Virtual_Resultant_Force->Fatz = sin(Virtual_Resultant_Force->rad) * Virtual_Resultant_Force->Force;
Virtual_Resultant_Force->Fatx = cos(Virtual_Resultant_Force->rad) * Virtual_Resultant_Force->Force;
#if car_force_info_printf
printf("虚拟力: Force = %.2f , angle = %.2f\r\n" ,Virtual_Resultant_Force->Force,Virtual_Resultant_Force->angle);
//printf("虚拟分量：Fatz = %.2f , Fatx = %.2f\r\n" ,Virtual_Resultant_Force->Fatz,Virtual_Resultant_Force->Fatx);
printf("-------------------------------------------\r\n");
#endif
}
