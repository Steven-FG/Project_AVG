
int trigPin = 12; 
int echoPin = 11; 
long timeperiod, cm, inches;
int Led1 = 10;

int sensorPin = A0;
int sensorPin2 = A1;
int sensorValue = 0;
int sensorValue2 = 0;
int Led3 = 9;
int Led4 = 8;


void setup() {
  // put your setup code here, to run once:
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(Led3, OUTPUT);
  pinMode(Led4, OUTPUT);
  Serial.begin(9600);  
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);  
  pinMode(Led1, OUTPUT);
  
}

void loop() 
  {
    // put your main code here, to run repeatedly:
    sensorValue = analogRead(sensorPin);
    sensorValue2 = analogRead(sensorPin2);
    Serial.println(sensorValue);
    Serial.println(sensorValue2);
  
    digitalWrite(trigPin, LOW);// sending 10 us pulse
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  
    timeperiod = pulseIn(echoPin, HIGH);
  
    inches = microsecondsToInches(timeperiod);
    cm = microsecondsToCentimeters(timeperiod);
    Serial.print("distcance in inches=");
    Serial.print(inches);
    Serial.print("   distance in centimeters=");
    Serial.print(cm);
    Serial.println();
  
    if(cm < 30)
      {
        digitalWrite(Led1, HIGH);
      }
        else 
        {
          digitalWrite(Led1, LOW);
        }
    
    delay(10);
    
    if(sensorValue < 20)
      {
        digitalWrite(2,HIGH);
        digitalWrite(Led3 , HIGH);
      }
        else 
        {
          digitalWrite(2,LOW);
          digitalWrite(Led3 ,LOW);
        }
    
    if(sensorValue2 < 20)
      {
        digitalWrite(3, HIGH);
        digitalWrite(Led4, HIGH);
      }
        else 
        {
          digitalWrite(3,LOW);
          digitalWrite(Led4, LOW);
        }
    delay(100);
}
    long microsecondsToInches(long microseconds)
    {
      return microseconds / 74 / 2;
    }
  
    long microsecondsToCentimeters(long microseconds)
    {
       return microseconds / 29 / 2;
    }
