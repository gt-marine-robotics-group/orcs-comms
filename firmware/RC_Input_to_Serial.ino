
#include <ServoInput.h>
 
ServoInputPin<2> servo;;
ServoInputPin<3> servo3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  float angle = servo.getAngle();
  Serial.println(angle);
//  angle = servo3.getAngle();
//  Serial.println(angle);
  delay(100);
  
}
