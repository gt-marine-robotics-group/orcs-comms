/*
  GITMRG OrangeRX-based Remote Control System firmware

  v0.1
  modified 20 FEB 2022
  by Sean Fish

  Hardware
   - ORX R615X 6CH 2.4GHz 
   - TL50BLBGYRQ Light Tower

  Roadmap
   - Version 0: Arduino Nano - Serial Commuication
   - Version 1: Arduino Nano RP2040 - micro-ROS

  Resources
  https://whyyesteam.wordpress.com/2019/01/09/using-an-rc-radio-controller-with-your-robot/
  https://info.bannerengineering.com/cs/groups/public/documents/literature/159857.pdf
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

void setup() {
  Serial.begin(9600);
  Serial.println("ORCS COMMS INIT");
  // Setup pins for reading ORX PWM signal
  attachInterrupt(digitalPinToInterrupt(ORX_AUX1_PIN), auxChange, CHANGE);
}

void loop() {
  // Check remote control state


}
