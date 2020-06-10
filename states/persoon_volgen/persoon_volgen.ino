#include <ZumoShield.h>

// Zet de standaard snelheden
#define HIGH_SPEED 300
#define AVRG_SPEED 200
#define SLOW_SPEED 150
#define STOP_SPEED 0
// of de motor nog op gaan moet komen
boolean start_up = true;

// Ultrasoon pins
// voor
#define TRIGGER_US_VOOR 11
#define ECHO_US_VOOR 5 
long timeperiod, cm;
// achter

// IR sensoren
#define IR_LINKS A1
#define IR_RECHTS A0
#define IR_POT 4

// State change pin
#define STATE_CHANGE_PIN 2

ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

void setup() {
  Serial.begin(9600);
  // Setup ultrasonen
  pinMode(TRIGGER_US_VOOR, OUTPUT); 
  pinMode(ECHO_US_VOOR, INPUT);
  // Setup IR sensoren
  pinMode(IR_LINKS, INPUT);
  pinMode(IR_RECHTS, INPUT);
  pinMode(IR_POT, INPUT);
  // setup state change pin4
  pinMode(STATE_CHANGE_PIN, INPUT);

  // wacht tot dat  
  button.waitForButton();
  
}

void loop() {
  boolean state = digitalRead(STATE_CHANGE_PIN);
  state = true;
  if (state) {
    boolean links = digitalRead(IR_LINKS);
    boolean rechts = digitalRead(IR_RECHTS);
  
    // als de arduino recht voor de persoon staat
    if (rechts == links) {
      Serial.println("Staat recht voor de persoon");
      digitalWrite(TRIGGER_US_VOOR, LOW);// sending 10 us pulse
      delayMicroseconds(2);
      digitalWrite(TRIGGER_US_VOOR, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGGER_US_VOOR, LOW);
  
      timeperiod = pulseIn(ECHO_US_VOOR, HIGH);
  
      cm = microsecondsToCentimeters(timeperiod);
  
      if (cm > 7 && cm < 50) {
        // te ver van af
        motors.setSpeeds(AVRG_SPEED, AVRG_SPEED);
        Serial.print("rijden bth");
      } else {
        Serial.print("freeze");
        motors.setSpeeds(STOP_SPEED, STOP_SPEED);
      }
      
    // als de persoon rechts van de AGV staat
    } else if (rechts) {
      Serial.println("ga naar rechts");
      motors.setLeftSpeed(AVRG_SPEED);
      motors.setRightSpeed(-AVRG_SPEED);
    // als de persoon links van de AGV staat
    } else if (links) {
      Serial.println("ga naar links");
      motors.setLeftSpeed(-AVRG_SPEED);
      motors.setRightSpeed(AVRG_SPEED);
    }
  } else {
    motors.setSpeeds(AVRG_SPEED, AVRG_SPEED);
    boolean pot = digitalRead(IR_POT);

    if (!pot) {
      motors.setSpeeds(STOP_SPEED, STOP_SPEED);
      delay(1000);
      motors.setSpeeds(AVRG_SPEED, AVRG_SPEED);
      delay(500);
    }
  }
}


long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}
