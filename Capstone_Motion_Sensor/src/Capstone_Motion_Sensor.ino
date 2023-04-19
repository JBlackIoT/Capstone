/*
 * Project Capstone_Motion_Sensor
 * Description:Ultrasonic Motion Sensor
 * Author: JBlack
 * Date: 3-18-23
 */

void setup();
void loop();

const int trigpin= A5;
const int echopin= 7;
long duration;
int distance;
void setup()
{
  pinMode(trigpin,OUTPUT);
  pinMode(echopin,INPUT);
  Serial.begin(9600);
 
}
void loop()
{
  digitalWrite(trigpin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin,LOW);
  duration=pulseIn(echopin,HIGH);
  distance = duration*0.034/2;
  Serial.printf("distance");
}