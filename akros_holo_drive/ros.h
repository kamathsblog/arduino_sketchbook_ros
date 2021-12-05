#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "ArduinoHardware.h"

namespace ros
{
  // default is pubs: 25, sub: 25, in buf: 512, out buf: 512  
  typedef NodeHandle_<ArduinoHardware, 4, 4, 280, 280> NodeHandle;

  // This is legal too and will use the default 25, 25, 512, 512
  //typedef NodeHandle_<ArduinoHardware> NodeHandle;
}

#endif
