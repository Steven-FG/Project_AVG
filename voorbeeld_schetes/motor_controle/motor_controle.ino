#include <ZumoShield.h>

// Motor defines
ZumoMotors motors;


void setup() {
  // put your setup code here, to run once:
  motors.setSpeeds(0,0);
}

void loop() {
  // put your main code here, to run repeatedly:
  motors.setSpeeds(100,100);

  delay(2000);

  motors.setSpeeds(-100,100);

  delay(4000);
}
