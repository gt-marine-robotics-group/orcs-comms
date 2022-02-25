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
//     vehicleState = 

}
