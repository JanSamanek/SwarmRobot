#include "commands.h"
#include "buffer.h"
#include "instructions.h"
#include "command_factory.h"
#include "PING_sensor.h"

#include <CAN.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/range.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "error_check.h"

rcl_publisher_t frontSensorPublisher;
sensor_msgs__msg__Range * frontSensorMsg;

rcl_subscription_t instructionsSubscriber;
geometry_msgs__msg__Twist instructionMsg;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define BOOT_TIMEOUT 5000

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

Instructions instructions = {1024, 1024, 1024, 0, 0};

PINGSensorConfiguration frontSensorConfig = 
{
  .pingPin = 7,
  .minimumRange = 0.03f,
  .maximumRange = 4.0f,
  .fieldOfView = 15,
  .referenceFrameId = "PING_sensor_front_link"
};
PINGSensor ultraSonicSensorFront(frontSensorConfig);

void incomming_instructions_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  instructions = convertToInstructions(*msg);
}

void setup() 
{   
  set_microros_transports();

  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(ERROR_LED_PIN, HIGH);  
  
  delay(BOOT_TIMEOUT); 

  Serial.begin(115200);
  CanBus.begin(CAN_BAUD_1000K, CAN_STD_FORMAT);
  
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &instructionsSubscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "instructions"));

  RCCHECK(rclc_publisher_init_default(
    &frontSensorPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
   "/ping/front/measurement"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &instructionsSubscriber, &instructionMsg, &incomming_instructions_callback, ON_NEW_DATA));

  if(!micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      &frontSensorMsg,
      (micro_ros_utilities_memory_conf_t) {})
    )
  {
    error_loop();
  }

  frontSensorMsg->header.frame_id = micro_ros_string_utilities_set(frontSensorMsg->header.frame_id, "PING_sensor_front_link");
  frontSensorMsg->radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  frontSensorMsg->field_of_view = 15* (M_PI / 180);
  frontSensorMsg->min_range = 0.03f;
  frontSensorMsg->max_range = 4.0f;
  
}

void loop() 
{
  struct timespec tv = {0};
  clock_gettime(0, &tv);

  float distance = ultraSonicSensorFront.getMeasurement();
  frontSensorMsg->header.stamp.sec = tv.tv_sec;
  frontSensorMsg->header.stamp.nanosec = tv.tv_nsec;
  frontSensorMsg->range = distance;
  RCSOFTCHECK(rcl_publish(&frontSensorPublisher, frontSensorMsg, NULL));
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
