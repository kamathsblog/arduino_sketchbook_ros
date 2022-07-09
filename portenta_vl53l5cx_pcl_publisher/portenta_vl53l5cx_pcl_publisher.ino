/*
==========
Arduino Portenta code for Spakrfun Qwiic VL53L5CX ToF Imager
This code implements the following functionality:
  > Reads sensor measurements over I2C, synchronizes time with the ROS2 host PC, and populates the PointCloud2 message (publisher)
  > micro-ROS communication with Host PC using several transports (Raspberry Pi 4 with ROS2 Galactic)
Author: Aditya Kamath
adityakamath.github.io
github.com/adityakamath
==========
*/

//micro-ROS includes from micro_ros_arduino v2.0.5-galactic (https://github.com/micro-ROS/micro_ros_arduino)
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <sensor_msgs/msg/point_field.h>

//Other includes
#include <math.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <stdio.h>
#include <TimeLib.h>
#include <Wire.h>

//Defines
#if !defined(TARGET_PORTENTA_H7_M7)
#error This example is only avaible for Arduino Portenta H7 (M7 Core)
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define PROCESSING true //Flag for the Processing App to visualize sensor readings
#define HWSERIAL   Serial
#define OFFSET     4
#define NR_FIELDS  3
#define RESOLUTION 8
#define FOV        PI/2

//micro-ROS entities
rcl_publisher_t               publisher;
rclc_support_t                support;
rcl_allocator_t               allocator;
rcl_node_t                    node;
sensor_msgs__msg__PointCloud2 msg;

//VL53L5CX ToF sensor data
SparkFun_VL53L5CX    myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM
static int64_t time_ns;

//Union to convert between Float32 and UInt8 for x, y and z values
typedef union
{
  float   number;
  uint8_t bytes[4];
} FLOAT2UINT8_T;
FLOAT2UINT8_T x, y, z;

//Error loop
void error_loop()
{
  while(1)
  {
    digitalWrite(LEDR, !digitalRead(LEDR));
    delay(100);
  }
}

void setup()
{
  //SERIAL TRANSPORT (currently does not work for this example)
  //set_microros_transports();

  //UDP (WIFI) TRANSPORT (currently does not work for this example)
  //set_microros_wifi_transports("akwifi", "manipal2014", "192.168.0.107", 9999);

  //UDP (ETHERNET) TRANSPORT
  byte arduino_mac[] = {0x01, 0xAB, 0x23, 0xCD, 0x45, 0xEF}; //User assigned MAC address (different from existing addresses on network)
  IPAddress arduino_ip(192, 168, 1, 109); //User assigned IP address (same subnet as agent, but different IP from existing ones on network)
  IPAddress agent_ip(192, 168, 1, 100); //static IP defined on the ROS2 agent side
  set_microros_native_ethernet_udp_transports(arduino_mac, arduino_ip, agent_ip, 8888);

  bootM4(); //boot M4 core for dual core processing
  digitalWrite(LEDR, LOW);
  delay(2000);

  //initialize I2C and Serial communication
  if(PROCESSING){ HWSERIAL.begin(115200); }
  Wire.begin(); //This resets I2C bus to 100kHz
  Wire.setClock(1000000); //Sensor has max I2C freq of 1MHz

  //VL53L5CX: Configure and start the sensor
  myImager.setWireMaxPacketSize(128); //Increase default from 32 to 128 bytes
  if (myImager.begin() == false){ while (1); }
  myImager.setResolution(RESOLUTION * RESOLUTION); //Enable all 64 pads
  myImager.setRangingFrequency(15); //Using 8x8, min frequency is 1Hz and max is 15Hz
  myImager.startRanging();

  //micro-ROS: define allocator, create init_options, node and publisher
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_vl53l5cx_node", "", &support));
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2),
    "micro_ros_vl53l5cx_pcl_publisher")
  );

  //initialize message memory
  if(!micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2),
    &msg,
    (micro_ros_utilities_memory_conf_t) {})
    )
  {
    error_loop();
  }

  //Populate the static fields of the message
  msg.height       = RESOLUTION;
  msg.width        = RESOLUTION;
  msg.is_bigendian = false;
  msg.is_dense     = false;
  msg.point_step   = NR_FIELDS * OFFSET;
  msg.row_step     = msg.point_step * msg.width;
  msg.fields.size  = 3;
  msg.data.size    = msg.row_step * msg.height;

  msg.header.frame_id     = micro_ros_string_utilities_set(msg.header.frame_id, "tof_frame");
  msg.fields.data[0].name = micro_ros_string_utilities_set(msg.fields.data[0].name, "x");
  msg.fields.data[1].name = micro_ros_string_utilities_set(msg.fields.data[1].name, "y");
  msg.fields.data[2].name = micro_ros_string_utilities_set(msg.fields.data[2].name, "z");

  for(int i=0; i<msg.fields.size; i++)
  {
    msg.fields.data[i].offset   = i*OFFSET;
    msg.fields.data[i].datatype = 7;
    msg.fields.data[i].count    = 1;
  }
}

void loop()
{
  //Synchronize time
  RCCHECK(rmw_uros_sync_session(1000));
  time_ns = rmw_uros_epoch_nanos();
  msg.header.stamp.sec = time_ns / 1000000000;
  msg.header.stamp.nanosec = time_ns % 1000000000;

  //Poll sensor for new data
  int data_count = 0;
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    {
      //Loop over width and height of the depth image
      for(int w = 0; w < RESOLUTION; w++)
      {
        for(int h = 0; h < RESOLUTION; h++)
        {
          int depth = measurementData.distance_mm[w + h*RESOLUTION]; //in mm
          int depth_mm = (depth < 0) ? 0 : depth; //set invalid measurements to 0

          x.number = (cos(w * (FOV/RESOLUTION) - FOV/2.0 - PI/2) * depth_mm)/1000.0; //x in m
          y.number = (sin(h * (FOV/RESOLUTION) - FOV/2.0) * depth_mm)/1000.0; //y in m
          z.number = (depth_mm)/1000.0; //z in m

          //decompose Float32 into uint8 bytes and populate msg.data[]
          for(int i=0; i<OFFSET; i++){
            msg.data.data[data_count + i]            = uint8_t(x.bytes[i]);
            msg.data.data[data_count + i + OFFSET]   = uint8_t(y.bytes[i]);
            msg.data.data[data_count + i + OFFSET*2] = uint8_t(z.bytes[i]);
          }
          data_count += OFFSET*NR_FIELDS;

          //print row data as CSV to serial
          if(PROCESSING)
          {
            HWSERIAL.print(depth_mm);
            HWSERIAL.print(",");
          }
        }
      }
      if(PROCESSING){ HWSERIAL.println(""); } //print new line after each row
    }
  }
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  delay(20);
}
