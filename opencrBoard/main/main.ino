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
rcl_publisher_t backSensorPublisher;

rcl_subscription_t instructionsSubscriber;
geometry_msgs__msg__Twist instructionMsg;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

CircularBuffer buffer(2048);
CommandFactory factory;

#define BOOT_TIMEOUT 5000

#define PERIOD_1000_MS 1000
#define PERIOD_100_MS 100
#define PERIOD_10_MS 10

uint32_t previousMillis10ms = 0;
uint32_t previousMillis100ms = 0;
uint32_t previousMillis1000ms = 0;

Instructions instructions = {1024, 1024, 1024, 0, 0};

PINGSensorConfiguration frontSensorConfig = 
{
  .pingPin = 7,
  .minimumRange = 0.03f,
  .maximumRange = 4.0f,
  .fieldOfView = 15,
  .referenceFrameId = "sensor_front"
};

PINGSensor ultraSonicSensorFront(frontSensorConfig);

PINGSensorConfiguration backSensorConfig = 
{
  .pingPin = 8,
  .minimumRange = 0.03f,
  .maximumRange = 4.0f,
  .fieldOfView = 15,
  .referenceFrameId = "sensor_back"
};

PINGSensor ultraSonicSensorBack(backSensorConfig);

sensor_msgs__msg__Range frontSensorMsg;
sensor_msgs__msg__Range backSensorMsg;

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

  CanBus.begin(CAN_BAUD_1000K, CAN_STD_FORMAT);
  
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "main_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &instructionsSubscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "instructions"));

  RCCHECK(rclc_publisher_init_default(
    &frontSensorPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
   "ping/front/measurement"));

  RCCHECK(rclc_publisher_init_default(
    &backSensorPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
   "ping/back/measurement"));

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

  frontSensorMsg.header.frame_id = micro_ros_string_utilities_set(frontSensorMsg.header.frame_id, ultraSonicSensorFront.configuration.referenceFrameId);
  frontSensorMsg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  frontSensorMsg.field_of_view = ultraSonicSensorFront.configuration.fieldOfView * (M_PI / 180);
  frontSensorMsg.min_range = ultraSonicSensorFront.configuration.minimumRange;
  frontSensorMsg.max_range = ultraSonicSensorFront.configuration.maximumRange;

  if(!micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      &backSensorMsg,
      (micro_ros_utilities_memory_conf_t) {})
    )
  {
    error_loop();
  }

  backSensorMsg.header.frame_id = micro_ros_string_utilities_set(backSensorMsg.header.frame_id, ultraSonicSensorBack.configuration.referenceFrameId);
  backSensorMsg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  backSensorMsg.field_of_view = ultraSonicSensorBack.configuration.fieldOfView * (M_PI / 180);
  backSensorMsg.min_range = ultraSonicSensorBack.configuration.minimumRange;
  backSensorMsg.max_range = ultraSonicSensorBack.configuration.maximumRange;

  buffer.push(factory.buildCommand(INIT_FREE_MODE_COMMAND));
  buffer.push(factory.buildCommand(INIT_CHASSIS_ACCELERATION_COMMAND));
  buffer.push(factory.buildCommand(INIT_COMMAND_1));
  buffer.push(factory.buildCommand(INIT_COMMAND_2));
  buffer.push(factory.buildCommand(INIT_COMMAND_3));
  buffer.push(factory.buildCommand(INIT_COMMAND_4));
  buffer.push(factory.buildCommand(INIT_COMMAND_5));
  
}

void loop() 
{
  uint32_t currentMillis = millis();

  if (currentMillis - previousMillis10ms >= PERIOD_10_MS) 
  {
    previousMillis10ms = currentMillis;
    
    buffer.push(factory.buildCommand(MOVE_COMMAND, instructions));
    buffer.push(factory.buildCommand(GIMBALL_COMMAND, instructions));
  }

  if (currentMillis - previousMillis100ms >= PERIOD_100_MS) 
  {
    previousMillis100ms = currentMillis;
     
    buffer.push(factory.buildCommand(COMMAND_1));
    buffer.push(factory.buildCommand(COMMAND_2));
    buffer.push(factory.buildCommand(COMMAND_3));
  }

  if (currentMillis - previousMillis1000ms >= PERIOD_1000_MS) 
  {
    previousMillis1000ms = currentMillis;
    
    buffer.push(factory.buildCommand(COMMAND_4));
    buffer.push(factory.buildCommand(COMMAND_5));
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

  fillMeasurementMessage(ultraSonicSensorFront, &frontSensorMsg);
  RCSOFTCHECK(rcl_publish(&frontSensorPublisher, &frontSensorMsg, NULL));

  fillMeasurementMessage(ultraSonicSensorBack, &backSensorMsg);
  RCSOFTCHECK(rcl_publish(&backSensorPublisher, &backSensorMsg, NULL));

  Command command;
  BufferStatus status = buffer.pop(command);
  if (status == BUFFER_EMPTY)
      return;
  
  sendCommand(command);
}
