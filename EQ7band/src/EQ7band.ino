/*
 * Project EQ7band
 * Description: trying to figure out the EQ sound sensor and attach to neo pixel strip
 * Author:JBlack
 * Date:
 */
#include "kdsRainbows.h"
#include <neopixel.h>

int strobePin = D4;
int outPin= A3;
int resetPin= D5;
int level[7] = {0, 0, 0, 0, 0, 0, 0};
const int PIXELPIN = D3;
const int PIXELCOUNT = 120;
const int numBands = 7;
const int PIXPERBAND =8;
int i=0;
int soundInput;
const int anaPin= A3;
 int pixelFill (int startPixel, int endPixel, int hexColor);


Adafruit_NeoPixel pixel ( PIXELCOUNT , PIXELPIN , WS2812B ) ;
SYSTEM_MODE (SEMI_AUTOMATIC);
void setup() {
  pinMode(strobePin, OUTPUT);
  pinMode(resetPin, OUTPUT);   // Set reset pin as output
  pinMode(anaPin, INPUT);   // Set analog input pin as input
  pixel.begin();
  pixel.setBrightness (255);
  //pixel.clear();
  pixel.show();
  Serial.begin(9600);
}

void loop() {
  // Reset and start reading from the first spectrum band
  digitalWrite(resetPin, HIGH);   // Set reset pin high
  digitalWrite(resetPin,LOW);
  delayMicroseconds (75);
  
  // Cycle through each frequency band by pulsing the strobe.
  for (i = 0; i < 7; i++)  {
    digitalWrite(strobePin, LOW);
    delayMicroseconds(100);   
    level[i] = analogRead(outPin);
    digitalWrite(strobePin, HIGH);
    delayMicroseconds(100);
    soundInput=level[i];
    if (soundInput>=0 && soundInput<63) {
      
      pixelFill (PIXPERBAND*i,PIXPERBAND*i+PIXPERBAND,fullred);
      pixel.show ();
    }
   if (soundInput>=63 && soundInput<160) {
     
      pixelFill (PIXPERBAND*i,PIXPERBAND*i+PIXPERBAND,orange);
      pixel.show ();
    }
    if (soundInput>=160 && soundInput<400) {
      
      pixelFill (PIXPERBAND*i,PIXPERBAND*i+PIXPERBAND,fullyellow);
      pixel.show ();
    }
    if (soundInput>=400 && soundInput<1000) {
      
      pixelFill (PIXPERBAND*i,PIXPERBAND*i+PIXPERBAND,fullgreen);
      pixel.show ();
    }
    if (soundInput>=1000 && soundInput<2500) {
    
      pixelFill (PIXPERBAND*i,PIXPERBAND*i+PIXPERBAND,fullblue);
      pixel.show ();
    }
    if (soundInput>=2500 && soundInput<16000) {
      
      pixelFill (PIXPERBAND*i,PIXPERBAND*i+PIXPERBAND,purple);
      pixel.show ();
    }
  }
  // Print the sensor values to the serial monitor.
  Serial.print("Sensor Values: ");
   for (i = 0; i < 7; i++) {
    Serial.printf("Channel %i - Level %i\n",i,level[i]);
  }
  
  // Wait for a short period of time.
  delay(10);
}
int pixelFill (int startPixel, int endPixel, int hexColor){
  for (int i=startPixel; i<endPixel;i++){

pixel.setPixelColor (i,hexColor);
pixel.show();

  }
return (1);
}
