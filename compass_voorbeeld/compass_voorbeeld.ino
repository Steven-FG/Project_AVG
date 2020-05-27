#include <Wire.h>
#include <ZumoShield.h>
// Zet de standaard snelheden
#define HIGH_SPEED 300
#define AVRG_SPEED 200
#define SLOW_SPEED 100
#define STOP_SPEED 0
// Compass defines
#define KEREN_KALIBREREN    70    // hoeveel calibratie voorbelden er worden genomen
#define CRB_REG_M_2_5GAUSS  0x60  // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ     0x1C  // CRA_REG_M value for magnetometer 220 Hz update rate
// Toelaatbaar verschil (in graden) tussen hoedige hoek en target hoek
#define DEVIATION_THRESHOLD 5

ZumoMotors motors;
ZumoBuzzer buzzer;
LSM303 compass;
Pushbutton button(ZUMO_BUTTON);

void setup () {
  // maak een seriele verbinding
  Serial.begin(9600);
  // maak verbinding met de I2C bus als een master
  Wire.begin();

  // Initiate LSM303
  compass.init();

  // Enables accelerometer and magnetometer
  compass.enableDefault();

  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate
  
  // wacht tot dat  
  button.waitForButton();

  Serial.println("begin calibratie: ");
  // kalibreer de gyroscoop
  kalibreer();
  Serial.println("einde calibratie: ");

}

void loop () {
  motors.setSpeeds(AVRG_SPEED, AVRG_SPEED);
  delay(1000);
  // stop de motor om de magnetische verstorring met de gyroscoop zo stabiel mogenlijk te maken
  motors.setSpeeds(STOP_SPEED, STOP_SPEED);
  // wacht tot dat de AGV tot stil stand is gekomen
  delay(100);
  // draai 90 graden naar rechts
  draai (90, true);
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

      snelheid = 65;

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
