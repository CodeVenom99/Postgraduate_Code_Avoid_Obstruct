#include "device.h"
void device_init(void)
{
  extern WbDeviceTag pen,gps,compass;
  pen = wb_robot_get_device("pen");
  gps = wb_robot_get_device("gps");
  compass = wb_robot_get_device("compass");
  wb_gps_enable(gps, TIME_STEP);
  wb_compass_enable(compass, TIME_STEP);
}