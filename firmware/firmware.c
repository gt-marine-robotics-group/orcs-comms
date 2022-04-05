#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include <ServoInput.h>

/*
  GITMRG OrangeRX-based Remote Control System firmware

  v0.1 - FEB 2022
  Sean Fish, Nikolay Tranakiev, Jane Crowley

  Hardware
   - ORX R615X 6CH 2.4GHz 
   - TL50BLBGYRQ Light Tower

  Roadmap
   - Version 0: Arduino Nano - Serial Commuication
   - Version 1: Raspberry Pi Pico - micro-ROS

  Resources
  https://whyyesteam.wordpress.com/2019/01/09/using-an-rc-radio-controller-with-your-robot/
  https://info.bannerengineering.com/cs/groups/public/documents/literature/159857.pdf
  https://github.com/dmadison/ServoInput/blob/master/examples/RC_Receiver/RC_Receiver.ino
*/

// PINS -------------------------------------------------------------
const uint LED_PIN = 25;
const uint ORX_AUX1_PIN = 1; // checks if killed - need to figure out reset check
const uint ORX_GEAR_PIN = 2; // Auto vs Manual
const uint ORX_RUDD_PIN = 3; // yaw
const uint ORX_ELEV_PIN = 4; // WAM-V translate forward / backward
const uint ORX_AILE_PIN = 5; // WAM-V translate left / right
const uint ORX_THRO_PIN = 6;

// VARS -------------------------------------------------------------
int vehicleState;
bool killed;
int state;
int xTranslation;
int yTranslation;
int yaw;

// DEVICES ----------------------------------------------------------
ServoInputPin<ORX_AUX1_PIN> orxAux1; // 3 states
ServoInputPin<ORX_GEAR_PIN> orxGear; // 2 states
ServoInputPin<ORX_RUDD_PIN> orxRudd; // Continuous
ServoInputPin<ORX_ELEV_PIN> orxElev; // Continuous
ServoInputPin<ORX_AILE_PIN> orxAile; // Continuous
ServoInputPin<ORX_THRO_PIN> orxThro; // Continuous

// ROS --------------------------------------------------------------
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;


void setup() {
  Serial.begin(9600);
  Serial.println("ORCS COMMS INITIALIZING...");
  // Setup pins for reading ORX PWM signal
  int num = 0;
  while (!ServoInput.available()) {
    Serial.print("Waiting for signals... ");
    Serial.println(num);
  }
  Serial.println("==================================================");
  Serial.println("============ ORCS COMMS INIT COMPLETE ============");
  Serial.println("==================================================");
}

void loop() {
  // Check remote control state
    Serial.println(vehicleState);
  // Update and transmit motor output values
    killed = orxGear.getBoolean();
    state = orxAux1.map(0, 3);
    xTranslation = orxElev.map(-100, 100);
    yTranslation = orxAile.map(-100, 100);
    yaw = orxRudd.map(-100, 100);

  // Update vehicle state and light tower
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data++;
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);



    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }



    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);

    msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}

