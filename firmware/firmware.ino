#include <ServoInput.h>

/*
  GITMRG OrangeRX-based Remote Control System firmware

  v0.1 - FEB 2022
  Sean Fish, Nikolay Tranakiev

  Hardware
   - ORX R615X 6CH 2.4GHz 
   - TL50BLBGYRQ Light Tower

  Roadmap
   - Version 0: Arduino Nano - Serial Commuication
   - Version 1: Arduino Nano RP2040 - micro-ROS

  Resources
  https://whyyesteam.wordpress.com/2019/01/09/using-an-rc-radio-controller-with-your-robot/
  https://info.bannerengineering.com/cs/groups/public/documents/literature/159857.pdf
  https://github.com/dmadison/ServoInput/blob/master/examples/RC_Receiver/RC_Receiver.ino
*/

// PINS -------------------------------------------------------------
const int ORX_AUX1_PIN = 1; // checks if killed - need to figure out reset check
const int ORX_GEAR_PIN = 2; // Auto vs Manual
const int ORX_RUDD_PIN = 3; // yaw
const int ORX_ELEV_PIN = 4; // WAM-V translate forward / backward
const int ORX_AILE_PIN = 5; // WAM-V translate left / right
const int ORX_THRO_PIN = 6;

// VARS -------------------------------------------------------------
int vehicleState;

// DEVICES ----------------------------------------------------------
ServoInputPin<ORX_AUX1_PIN> orx_aux1;
ServoInputPin<ORX_GEAR_PIN> orx_gear;
ServoInputPin<ORX_RUDD_PIN> orx_rudd;
ServoInputPin<ORX_ELEV_PIN> orx_elev;
ServoInputPin<ORX_AILE_PIN> orx_aile;
ServoInputPin<ORX_THRO_PIN> orx_thro;

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
    orx_aux1 = ??
    orx_gear = 
    orx_rudd = 
    orx_elev = 
    orx_aile = 
    orx_thro = 
  // Update vehicle state and light tower
    vehicleState = 

}
