/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/jamesblack/Documents/IoT/Capstone/BlueSafteyButton/src/BlueSafteyButton.ino"
/*
 * Project BlueSafteyButton
 * Description: Manual buttton to light up neo pixel strip blue
 * Author:JBlack
 * Date:03-18-23
 */


#include <neopixel.h>
#include "Adafruit_MQTT.h"
#include <credentials.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"

void setup();
void loop();
void MQTT_connect();
bool MQTT_ping();
#line 15 "/Users/jamesblack/Documents/IoT/Capstone/BlueSafteyButton/src/BlueSafteyButton.ino"
const int EMERGENCYBUTTON = D2;
const int PIXEL_COUNT = 60;
const int LEDPIN = D3;

bool button;
bool oldButton;
bool onOff = false;
bool status;
unsigned long currentTime;
unsigned long lastTime;


int pixelBri = 255;
Adafruit_NeoPixel pixel(PIXEL_COUNT, LEDPIN, WS2812B);
TCPClient TheClient;

Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish mqttEMERGENCYBUTTON = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/emergencybutton");



void setup() {
  Serial.begin(9600);
  pinMode(EMERGENCYBUTTON, INPUT);
  pinMode(LEDPIN, OUTPUT);
  pixel.begin();
  pixel.show();
}

void loop() {
  MQTT_connect ();
  MQTT_ping();
  button = digitalRead(EMERGENCYBUTTON);
  currentTime= millis();
  if(mqtt.Update()) { //if mqtt object (Adafruit.io) is available to receive data
Serial.printf("Publishing %0.2f to Adafruit.io feed emergencybutton \n",button);
mqttEMERGENCYBUTTON.publish(button);

  if (button != oldButton) {
    if (button == true) {
      onOff = !onOff;
    }
    oldButton = button;
  }
  //Serial.printf("onOff:%i\n", onOff);

  if (onOff == true) {
    for (int i = 0; i < PIXEL_COUNT; i++) {
      pixel.setPixelColor(i, 0, 0, 255);
    }
    pixel.setBrightness(pixelBri);
    pixel.show();
    Serial.printf("Emergency!\n", button);

    mqttEMERGENCYBUTTON.publish (button);
    
  }
  else {
    pixel.clear ();
    pixel.show ();
  }
  if ((currentTime-lastTime)>1000){
    mqttEMERGENCYBUTTON.publish (button);
    lastTime = millis(); 
  }
}
}
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}
