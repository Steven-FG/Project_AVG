// voeg alle benodigde libraries toe
#include <Wire.h>
#include <ZumoShield.h>
#include <VL53L0X.h>

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
// interupt voor time of flight
#define INT_TOF A0

ZumoMotors motors;
ZumoBuzzer buzzer;
LSM303 compass;
Pushbutton button(ZUMO_BUTTON);
VL53L0X randje;

void setup () {
  Serial.begin(9600);
  // maak verbinding met de I2C bus als een master
  Wire.begin();

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
  //button.waitForButton();
  
}

void loop() {
  digitalWrite(INT_TOF,HIGH);
  delay(20);
  // put your main code here, to run repeatedly:
  int distance = randje.readRangeSingleMillimeters();

  Serial.print("afstand: ");
  Serial.println(distance);

  digitalWrite(INT_TOF,LOW);
  delay(20);

  float richting = gemiddeldeRichting();

  Serial.print("richting: ");
  Serial.println(richting);

  delay(500);

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
