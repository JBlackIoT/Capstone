/*
 * Project L14_04_LoRaGPS
 * Description:  utilizing LoRa
 * Author: James
 * Date: 24-MAR-2023
 */

#include <Adafruit_GPS.h>
#include <IoTClassroom_CNM.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include <JsonParserGeneratorRK.h>
#include <credentials.h>

#define OLED_RESET D4

TCPClient TheClient; 

Adafruit_GPS GPS(&Wire);
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);
Adafruit_MQTT_Publish coordinates = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/GPSGenerator");

Adafruit_MQTT_Publish pubFeed = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME"/feeds/lat"); 
Adafruit_MQTT_Publish pubFeed2 = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME"/feeds/lon"); 
// Define User and Credentials
String password = "AA4104132968BA2224299079021594AB"; // AES128 password
String myName = "BLACK";
const int RADIOADDRESS = 0xCA; // Get address from Instructor, it will be a value between 0xC1 - 0xCF
const int TIMEZONE = -6;
const int LEDPIN = D7;

// Define Constants
const int RADIONETWORK = 7;    // range of 0-16
const int SENDADDRESS = 302;   // address of radio to be sent to

// Declare Variables
float lat, lon, alt;
int subValue;
int sat;
int lastTime;
int currentTime;

//IoTTimer myTimer(5000, []() { digitalWrite(LEDPIN, LOW); });  // turn off LED after 5 seconds
void MQTT_connect();
bool MQTT_ping();

//SYSTEM_MODE(AUTOMATIC);
void createEventPayLoad (float latValue, float longValue );

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 5000);
  Wire.begin();
  //Initialize GPS
  GPS.begin(0x10);  // The I2C address to use is 0x10
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPS.println(PMTK_Q_RELEASE);

  Serial1.begin(115200);
  reyaxSetup(password);
  //mqtt.publish(&pubFeed);
}

void loop() {
  GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }
   MQTT_connect();
  MQTT_ping();
    // this is our 'wait for incoming subscription packets' busy subloop 
 
  //Serial.printf("pubValue = %d\n", pubValue);
   
//digitalWrite(, latValue); 

  // Get data from GSP unit (best if you do this continuously)
  
    //String parse0 = Serial1.readStringUntil('=');  //+RCV
    //String parse1 = Serial1.readStringUntil(',');  // address received from
    //String parse2 = Serial1.readStringUntil(',');  // buffer length
    //String parse3 = Serial1.readStringUntil(',');  // fuseSound
    //String parse4 = Serial1.readStringUntil(',');  // fuseDust
    //String parse5 = Serial1.readStringUntil(',');  // rssi
    //String parse6 = Serial1.readStringUntil('\n'); // snr
    //String parse7 = Serial1.readString();          // extra

    //Serial.printf("parse0: %s\nparse1: %s\nparse2: %s\nparse3: %s\nparse4: %s\nparse5: %s\nparse6: %s\nparse7: %s\n", parse0.c_str(), parse1.c_str(), parse2.c_str(), parse3.c_str(), parse4.c_str(), parse5.c_str(), parse6.c_str(), parse7.c_str());
    //delay(100);
    //we want to every minute send lora data and publish to adafruit
    if (millis()-lastTime > 60000) { // Get the GPS data
      lastTime = millis();
      createEventPayLoad(lat,lon);
    getGPS(&lat,&lon,&alt,&sat);
    Serial.printf("\n=================================================================\n");
    Serial.printf("Lat: %0.6f, Lon: %0.6f, Alt: %0.6f, Satellites: %i\n",lat, lon, alt, sat);
    Serial.printf("=================================================================\n\n");
    sendData(myName, lat, lon, sat);  // Send the GPS data to the receiver
    //send_to_adafruit(gps.read); // Send the GPS data to Adafruit
}

 
  }


void getGPS(float *latitude, float *longitude, float *altitude, int *satellites) {
  int theHour;

  theHour = GPS.hour + TIMEZONE;
  if (theHour < 0) {
    theHour = theHour + 24;
  }

  Serial.printf("Time: %02i:%02i:%02i:%03i\n", theHour, GPS.minute, GPS.seconds, GPS.milliseconds);
  Serial.printf("Dates: %02i-%02i-%02i\n", GPS.month, GPS.day, GPS.year);
  Serial.printf("Fix: %i, Quality: %i", (int)GPS.fix, (int)GPS.fixquality);
  if (GPS.fix) {
    *latitude = GPS.latitudeDegrees;
    *longitude = GPS.longitudeDegrees;
    *altitude = GPS.altitude;
    *satellites = (int)GPS.satellites;
  }
}

void sendData(String name, float latitude, float longitude, int satelittes) {
  char buffer[60];
  sprintf(buffer, "AT+SEND=%i,60,%f,%f,%i,%s\r\n", SENDADDRESS, latitude, longitude, satelittes, name.c_str());
  Serial1.printf("%s",buffer);
  //Serial1.println(buffer); 
  delay(1000);
  if (Serial1.available() > 0)
  {
    Serial.printf("Awaiting Reply from send\n");
    String reply = Serial1.readStringUntil('\n');
    Serial.printf("Send reply: %s\n", reply.c_str());
  }
}

void reyaxSetup(String password) {
  // following four paramaters have most significant effect on range
  // recommended within 3 km: 10,7,1,7
  // recommended more than 3 km: 12,4,1,7
  const int SPREADINGFACTOR = 10;
  const int BANDWIDTH = 7;
  const int CODINGRATE = 1;
  const int PREAMBLE = 7;
  String reply; // string to store replies from module

  Serial1.printf("AT+ADDRESS=%i\r\n", RADIOADDRESS); // set the radio address
  delay(200);
  if (Serial1.available() > 0) {
    Serial.printf("Awaiting Reply from address\n");
    reply = Serial1.readStringUntil('\n');
    Serial.printf("Reply address: %s\n", reply.c_str());
  }

  Serial1.printf("AT+NETWORKID=%i\r\n", RADIONETWORK); // set the radio network
  delay(200);
  if (Serial1.available() > 0) {
    Serial.printf("Awaiting Reply from networkid\n");
    reply = Serial1.readStringUntil('\n');
    Serial.printf("Reply network: %s\n", reply.c_str());
  }

  Serial1.printf("AT+CPIN=%s\r\n", password.c_str());
  delay(200);
  if (Serial1.available() > 0) {
    Serial.printf("Awaiting Reply from password\n");
    reply = Serial1.readStringUntil('\n');
    Serial.printf("Reply: %s\n", reply.c_str());
  }

  Serial1.printf("AT+PARAMETER=%i,%i,%i,%i\r\n", SPREADINGFACTOR, BANDWIDTH, CODINGRATE, PREAMBLE);
  delay(200);
  if (Serial1.available() > 0) {
    reply = Serial1.readStringUntil('\n');
    Serial.printf("reply: %s\n", reply.c_str());
  }

  Serial1.printf("AT+ADDRESS?\r\n");
  delay(200);
  if (Serial1.available() > 0) {
    Serial.printf("Awaiting Reply\n");
    reply = Serial1.readStringUntil('\n');
    Serial.printf("Radio Address: %s\n", reply.c_str());
  }

  Serial1.printf("AT+NETWORKID?\r\n");
  delay(200);
  if (Serial1.available() > 0) {
    Serial.printf("Awaiting Reply\n");
    reply = Serial1.readStringUntil('\n');
    Serial.printf("Radio Network: %s\n", reply.c_str());
  }

  Serial1.printf("AT+PARAMETER?\r\n");
  delay(200);
  if (Serial1.available() > 0) {
    Serial.printf("Awaiting Reply\n");
    reply = Serial1.readStringUntil('\n');
    Serial.printf("RadioParameters: %s\n", reply.c_str());
  }

  Serial1.printf("AT+CPIN?\r\n");
  delay(200);
  if (Serial1.available() > 0) {
    Serial.printf("Awaiting Reply\n");
    reply = Serial1.readStringUntil('\n');
    Serial.printf("Radio Password: %s\n", reply.c_str());
  }
   display.begin(SSD1306_SWITCHCAPVCC,0x3C);
   display.clearDisplay();
  display.setCursor(0,0);
  display.setRotation(2);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  display.printf("lattitude is %f0.6\n",lat);
  display.printf("longitude is %f0.6\n",lon);
  display.printf("altitude is %f0.6\n",alt);
  display.printf("satelites are %i\n",sat);
  display.printf("Train Approaching!");
  display.display();
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


void createEventPayLoad(float latValue, float lonValue) {
  JsonWriterStatic<256> jw;
  {
    JsonWriterAutoObject obj(&jw);
    jw.insertKeyValue("lat", latValue);
    jw.insertKeyValue("lon", lonValue);
  }
  coordinates.publish(jw.getBuffer());
}
















