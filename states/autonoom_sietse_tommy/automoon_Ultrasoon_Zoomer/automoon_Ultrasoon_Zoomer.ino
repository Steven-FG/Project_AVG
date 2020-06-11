// voeg alle benodigde libraries toe
#include <Wire.h>
#include <ZumoShield.h>
#include <VL53L0X.h>

// Zet de standaard snelheden
#define HIGH_SPEED 200
#define BOOST_SPEED 150
#define AVRG_SPEED 115
#define SLOW_SPEED 100
#define STOP_SPEED 0
// Compass defines
#define KEREN_KALIBREREN    70    // hoeveel calibratie voorbelden er worden genomen
#define CRB_REG_M_2_5GAUSS  0x60  // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ     0x1C  // CRA_REG_M value for magnetometer 220 Hz update rate
// Toelaatbaar verschil (in graden) tussen hoedige hoek en target hoek
#define DEVIATION_THRESHOLD 5
// interupt voor time of flight
#define INT_TOF A0
// IR pin POT
#define IR_POT 4
#define TRIGGER_US_VOOR 11
#define ECHO_US_VOOR 5 
long timeperiod, cm;
boolean pot = 1;

ZumoMotors motors;
ZumoBuzzer buzzer;
LSM303 compass;
Pushbutton button(ZUMO_BUTTON);
VL53L0X randje;

int teller = 0;

// pc int setup functie
void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

// als er iets veranderd in de deze pcints
ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{
  pot = digitalRead(IR_POT);
} 

void setup () {
  // maak verbinding met de I2C bus als een master
  Wire.begin();
  
  //pinMode(IR_POT, INPUT);
  // de pcint fetup voor de potten detectie
  pciSetup(IR_POT);

  // Initiate LSM303
  compass.init();

  // Enables accelerometer and magnetometer
  compass.enableDefault();

  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate

  pinMode(INT_TOF,INPUT_PULLUP);
  digitalWrite(INT_TOF,LOW);

  randje.init();
  randje.setTimeout(500);
  
  // wacht tot dat  
  button.waitForButton();

  // kalibreer de gyroscoop
  kalibreer();

  // wacht tot dat  
  button.waitForButton();

  pinMode(TRIGGER_US_VOOR, OUTPUT); 
  pinMode(ECHO_US_VOOR, INPUT);

  
}

void loop() {
  // meet de afstand met de rand
  int afstand = aftand_tof(5);
  // als de agv in het pad staat
  if (afstand < 100) {
    // als de AGV naast een pot staat
    if (!pot) {
      motors.setSpeeds(STOP_SPEED, STOP_SPEED);
      buzzer.playNote(NOTE_G(5), 200, 15);
      delay(1000);
      motors.setSpeeds(AVRG_SPEED, AVRG_SPEED);
      delay(80);
    }
    // alse de AGV de juiste afstand van de rand heeft
    if (afstand >= 30 && afstand <= 40) {
      motors.setSpeeds(AVRG_SPEED, AVRG_SPEED);
      ultrasoon_object();
    // asl de GV te ver naar links staat
    } else if (afstand > 40) {
      motors.setLeftSpeed(BOOST_SPEED);
      motors.setRightSpeed(SLOW_SPEED);
      ultrasoon_object();
    // als de AGV te ver naar rechts staat
    } else if (afstand < 30) {
      motors.setLeftSpeed(SLOW_SPEED);
      motors.setRightSpeed(BOOST_SPEED);
      ultrasoon_object();
    }
  // als de AGV uit het pad is gereden
  } else {

    teller = teller + 1;

    if(teller == 1){
    // laat de AGV even recht door rijden
    motors.setSpeeds(AVRG_SPEED, AVRG_SPEED);
    delay(600);
    // sta stil
    motors.setLeftSpeed(STOP_SPEED);
    motors.setRightSpeed(STOP_SPEED);

    // draai met 180 graden
    draai(180, true);
    // rij weer terug het pad in
    motors.setSpeeds(AVRG_SPEED, AVRG_SPEED);
    delay(650);
    }

    if(teller == 2){
    // laat de AGV even recht door rijden
    motors.setSpeeds(AVRG_SPEED, AVRG_SPEED);
    delay(700);
    // sta stil
    motors.setLeftSpeed(STOP_SPEED);
    motors.setRightSpeed(STOP_SPEED);

      teller = 0;
    draai(85, true);
    // rijd naar volgende pad
    motors.setSpeeds(AVRG_SPEED, AVRG_SPEED);
    delay(1300);
    draai(85, true);
    // rijd pad in
    motors.setSpeeds(AVRG_SPEED, AVRG_SPEED);
    delay(750);
    }
  }

}


void kalibreer (void) {
  // The highest possible magnetic value to read in any direction is 2047
  // The lowest possible magnetic value to read in any direction is -2047
  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};

  // speel een waarschuwings toon om aan te geven dat de AGV gaat kalibreren
  // note, lengte in MS, volume (0 t/m 15)
  buzzer.playNote(NOTE_G(5), 200, 15);

  // wacht even voor dat de AGV gaat bewegen
  delay(500);

  // rechts om draaien om te kalibreren
  motors.setLeftSpeed(AVRG_SPEED);
  motors.setRightSpeed(-AVRG_SPEED);

  for(uint8_t i = 0; i < KEREN_KALIBREREN; i++)
  {
    // Take a reading of the magnetic vector and store it in compass.m
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);

    delay(50);
  }

  // stop de motoren
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  // Set calibrated values to compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;

  // note, lengte in MS, volume (0 t/m 15)
  buzzer.playNote(NOTE_G(4), 200, 15);
}

void draai (uint8_t graden, boolean rechts) {
  float start_richting = gemiddeldeRichting();
  float target_richting;
  int snelheid;
  boolean draaien = true;

  if (rechts)
    target_richting = start_richting + (float) graden;
  else
    target_richting = start_richting - (float) graden;

  target_richting = fmod(target_richting, 360);

  float abs_verschil;

  while (draaien) {
    
    float huidige_richting = snelleRichting();

    float relative_richting = relativeRichting(huidige_richting, target_richting);
    abs_verschil = abs(relative_richting);
    
    if (abs_verschil < 2) {
      motors.setLeftSpeed(STOP_SPEED);
      motors.setRightSpeed(STOP_SPEED);
      draaien = false;
      return;
    } else {

      snelheid = 85;

      if (abs_verschil > 10)
        snelheid += 50*abs_verschil/180;
      
      if (rechts) {
        if (relative_richting < 0)
          snelheid = 0 - snelheid;
        motors.setLeftSpeed(snelheid);
        motors.setRightSpeed(-snelheid);
      } else {
        if (relative_richting > 0)
          snelheid = 0 - snelheid;
        motors.setLeftSpeed(-snelheid);
        motors.setRightSpeed(snelheid);
      }
    }
  }
}

int aftand_tof(int c) {
  // Zet de time of flight aan
  digitalWrite(INT_TOF,HIGH);
  delay(20);

  int tmp;

  int distance = 0;

  for (int i = 0; i < c; i++) {
    // put your main code here, to run repeatedly:
    tmp = randje.readRangeSingleMillimeters();
    distance += tmp;
  }

  distance /= c;

  // Zet de time of flight uit
  digitalWrite(INT_TOF,LOW);

  return distance;
}

// neem het gemiddelde van de laatste 10 metingen voor de meest accurate meting
float gemiddeldeRichting()
{
  LSM303::vector<int32_t> avg = {0, 0, 0};

  for(int i = 0; i < 10; i ++)
  {
    compass.read();
    avg.x += compass.m.x;
    avg.y += compass.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}

// neem een snelle meting van de richting
float snelleRichting()
{
  LSM303::vector<int32_t> avg = {0, 0, 0};

  for(int i = 0; i < 5; i ++)
  {
    compass.read();
    avg.x += compass.m.x;
    avg.y += compass.m.y;
  }
  avg.x /= 5.0;
  avg.y /= 5.0;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}

float relativeRichting(float heading_from, float heading_to)
{
  float relative_heading = heading_to - heading_from;

  // constrain to -180 to 180 degree range
  if (relative_heading > 180)
    relative_heading -= 360;
  if (relative_heading < -180)
    relative_heading += 360;

  return relative_heading;
}

// vertaalt de x en y waarde naar een richting in graden
// LSM303::heading() neemt de ook de tilt in acht,
// maar kan hierdoor ook bedrogen worden door de acceleratie
// dit werkt beter op een water pas test lokatie.
// ik moet nog kijken wat beter is
template <typename T> float heading(LSM303::vector<T> v)
{
  float x_scaled =  2.0*(float)(v.x - compass.m_min.x) / ( compass.m_max.x - compass.m_min.x) - 1.0;
  float y_scaled =  2.0*(float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled)*180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}


long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}

void ultrasoon_object(){
 
  digitalWrite(TRIGGER_US_VOOR, LOW);// sending 10 us pulse
      delayMicroseconds(2);
      digitalWrite(TRIGGER_US_VOOR, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGGER_US_VOOR, LOW);
  
      timeperiod = pulseIn(ECHO_US_VOOR, HIGH);
  
      cm = microsecondsToCentimeters(timeperiod);
  
      if (cm < 3) {
        // te ver van af
        motors.setSpeeds(STOP_SPEED, STOP_SPEED);
        buzzer.playNote(NOTE_G(5), 200, 15);
        delay(1000);
        
      } 
}
