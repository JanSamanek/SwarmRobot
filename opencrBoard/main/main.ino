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

#include "error_check.h"

rcl_publisher_t publisher;
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
  .referenceFrameId = "PING_sensor_front_link"
};

PINGSensor ultraSonicSensorFront(frontSensorConfig);

void incomming_instructions_callback(const void *msgin) {
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

  // RCCHECK(rclc_subscription_init_default(
  //   &instructionsSubscriber,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  //   "instructions"));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
   "PING/front/measurement"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 0, &allocator));
  
  // RCCHECK(rclc_executor_add_subscription(&executor, &instructionsSubscriber, &instructionMsg, &incomming_instructions_callback, ON_NEW_DATA));

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

  if (currentMillis - previousMillis10ms >= PERIOD_10_MS) {
    previousMillis10ms = currentMillis;
    callback_10_ms();
  }

  if (currentMillis - previousMillis100ms >= PERIOD_100_MS) {
    previousMillis100ms = currentMillis;
    callback_100_ms();
  }

  if (currentMillis - previousMillis1000ms >= PERIOD_1000_MS) {
    previousMillis1000ms = currentMillis;
    callback_1000_ms();
  }

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  sensor_msgs__msg__Range msg = generateMeasurementMessage(ultraSonicSensorFront);
  RCCHECK(rcl_publish(&publisher, &msg, NULL));

  Command command;
  BufferStatus status = buffer.pop(command);
  if (status == BUFFER_EMPTY)
      return;
  
  sendCommand(command);
}

void callback_1000_ms(void)
{
  buffer.push(factory.buildCommand(COMMAND_4));
  buffer.push(factory.buildCommand(COMMAND_5));
}

void callback_100_ms(void)
{
  buffer.push(factory.buildCommand(COMMAND_1));
  buffer.push(factory.buildCommand(COMMAND_2));
  buffer.push(factory.buildCommand(COMMAND_3));
}

void callback_10_ms(void)
{
  buffer.push(factory.buildCommand(MOVE_COMMAND, instructions));
  buffer.push(factory.buildCommand(GIMBALL_COMMAND, instructions));
}