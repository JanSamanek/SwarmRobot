#include "commands.h"
#include "buffer.h"
#include "instructions.h"
#include "command_factory.h"

#include <CAN.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "micro_ros_error_check.h"

#include "instructions_subscriber.h"

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

InstructionsSubscriber instructionsSubscriber;
CircularBuffer buffer(2048);
CommandFactory factory;

#define BOOT_TIMEOUT 5000
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

  instructionsSubscriber = InstructionsSubscriber(support);
  rcl_node_t instructionsNode = instructionsSubscriber.getNodeHandle();
  RCCHECK(rclc_node_init_default(&instructionsNode, "micro_ros_instructions_node", "", &support));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  rcl_subscription_t instructionsSubscription = instructionsSubscriber.getSubscriptionHandle();
  geometry_msgs__msg__Twist msg = instructionsSubscriber.msg;
  rclc_subscription_callback_t callback = InstructionsSubscriber::subscriptionCallback;
  RCCHECK(rclc_executor_add_subscription(&executor, &instructionsSubscription, &msg, callback, ON_NEW_DATA));

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

  Instructions instructions = instructionsSubscriber.getInstructions();
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
  Instructions instructions = instructionsSubscriber.getInstructions();
  buffer.push(factory.buildCommand(COMMAND_4, instructions));
  buffer.push(factory.buildCommand(COMMAND_5, instructions));
}

void callback_100_ms(void)
{
  Instructions instructions = instructionsSubscriber.getInstructions();
  buffer.push(factory.buildCommand(COMMAND_1, instructions));
  buffer.push(factory.buildCommand(COMMAND_2, instructions));
  buffer.push(factory.buildCommand(COMMAND_3, instructions));
}

void callback_10_ms(void)
{
  Instructions instructions = instructionsSubscriber.getInstructions();
  buffer.push(factory.buildCommand(MOVE_COMMAND, instructions));
  buffer.push(factory.buildCommand(GIMBALL_COMMAND, instructions));
}