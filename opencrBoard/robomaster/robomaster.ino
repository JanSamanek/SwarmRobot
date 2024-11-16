#include "commands.h"
#include "buffer.h"
#include "instruction.h"
#include "command_factory.h"

#include <CAN.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/pose2_d.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Pose2D msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 22

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


#define BOOT_TIMEOUT 5000

CircularBuffer buffer(2048);
CommandFactory factory;

Instruction instructions = 
{
  .speedX = 1024,
  .speedY = 1024,
  .rotation = 1024,
  .gimballYaw = 0,
  .gimballRoll = 0
};

#define PERIOD_1000_MS 1000000
#define PERIOD_100_MS 100000
#define PERIOD_10_MS 10000

HardwareTimer Timer1000ms(TIMER_CH1);
HardwareTimer Timer100ms(TIMER_CH2);
HardwareTimer Timer10ms(TIMER_CH3);


void setup() 
{   
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  delay(BOOT_TIMEOUT); 

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_node", "", &support));
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D), "microRos/moveInstructions"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  CanBus.begin(CAN_BAUD_1000K, CAN_STD_FORMAT);
  Serial.begin(115200);

  Timer1000ms.stop();
  Timer1000ms.setPeriod(PERIOD_1000_MS);        
  Timer1000ms.attachInterrupt(callback_1000_ms);

  Timer100ms.stop();
  Timer100ms.setPeriod(PERIOD_100_MS);        
  Timer100ms.attachInterrupt(callback_100_ms);

  Timer10ms.stop();
  Timer10ms.setPeriod(PERIOD_10_MS);        
  Timer10ms.attachInterrupt(callback_10_ms);

  buffer.push(factory.buildCommand(INIT_FREE_MODE_COMMAND, instructions));
  buffer.push(factory.buildCommand(INIT_CHASSIS_ACCELERATION_COMMAND, instructions));
  buffer.push(factory.buildCommand(INIT_COMMAND_1, instructions));
  buffer.push(factory.buildCommand(INIT_COMMAND_2, instructions));
  buffer.push(factory.buildCommand(INIT_COMMAND_3, instructions));
  buffer.push(factory.buildCommand(INIT_COMMAND_4, instructions));
  buffer.push(factory.buildCommand(INIT_COMMAND_5, instructions));

  Timer1000ms.start();
  Timer100ms.start();
  Timer10ms.start();
}

void loop() 
{
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

  Command command;

  BufferStatus status = buffer.pop(command);
  if (status == BUFFER_EMPTY)
  {
      return;
  }

  sendCommand(command);
}

void callback_1000_ms(void)
{
  buffer.push(factory.buildCommand(COMMAND_4, instructions));
  buffer.push(factory.buildCommand(COMMAND_5, instructions));
}

void callback_100_ms(void)
{
  buffer.push(factory.buildCommand(COMMAND_1, instructions));
  buffer.push(factory.buildCommand(COMMAND_2, instructions));
  buffer.push(factory.buildCommand(COMMAND_3, instructions));
}

void callback_10_ms(void)
{
  buffer.push(factory.buildCommand(MOVE_COMMAND, instructions));
  buffer.push(factory.buildCommand(GIMBALL_COMMAND, instructions));
}


void subscription_callback(const void * msgin)
{  
  const geometry_msgs__msg__Pose2D * msg = (const geometry_msgs__msg__Pose2D *)msgin;
  instructions.speedX = msg->x;
  instructions.speedY = msg->y;
  instructions.rotation = msg->theta;
}