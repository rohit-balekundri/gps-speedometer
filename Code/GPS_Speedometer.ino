/**************************************************************************
 Final Project  ECE 508 Spring 2023
 
 TEAM 07
 Project Title: Arduino based GPS Speedometer
 
 Description: Interface using Arduino Nano Iot 33 with BN-220 GPS moudle.
 Capturing GPS locations and Calculate  Speed , Latitude and Longititude values.
 Sending this data  to  MQTT explorer and also display it via Oled Display
 
 Issues: No issues
 
 Code Reference:
 Code lecture 15 TaskScheduler
 https://github.com/arkhipenko/TaskScheduler
 **************************************************************************/

#include <Scheduler.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <stdio.h>
#include "modTinyGPS.h"
#include "myiot33_library.h"
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <Arduino_JSON.h>



/*------------------------------ WIFI SETUP-----------------------------------*/

const char teamnumber[15] = "Team 07";  
const char ssid[31] = "ONEPLUS7";    
const char pass[31] = "password";   
 
/*------------------------------------------------------------------------------*/



unsigned long currMillis, prevMillis;
char lcdBuffer[64];
String oledline[9];
const int RED_ledPin = 12;




/*------------------------------Wifi UDP Client and MQTT-------------------------*/

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
WiFiUDP ntpUDP;
NTPClient ntpClient(ntpUDP);
long epochNTP = 0;
char mqttClienId[31] = "client_1234_Team07";
long nmrMqttMesages = 0;
String mqttStringMessage;
char topicPub[61]  = "";
JSONVar myJsonDoc;  

TinyGPSPlus gps;
double latRnd, lonRnd, Speed;

/*--------------------------------------------------------------------------------*/



void setup() {
  Serial.begin(115200); delay(500);
  Serial1.begin(9600); delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RED_ledPin, OUTPUT);
  randomSeed(analogRead(A7));
     
  iot33StartOLED(1);
  oledline[1] = "GPS SPEEDOMETER";
  for (int jj=2; jj<=8; jj++){ oledline[jj]=""; }


  ntpClient.begin();

  Scheduler.startLoop(threadOLED);
  Scheduler.startLoop(threadWiFiNINA); delay(5000);
  Scheduler.startLoop(threadMQTT); delay(10000);
  Scheduler.startLoop(threadGPS);

}

void loop() {
  delay(33);
}

/*--------------------- WifiNINA Thread--------------------------------------------*/

void threadWiFiNINA() {
  
  if (WiFi.status() != WL_CONNECTED) {
    long prevWiFiMillis = millis();
    while (WiFi.status() != WL_CONNECTED) {    
      WiFi.begin(ssid, pass); delay(250);
    }
    sprintf(lcdBuffer, "Connected in %d sec", (millis()-prevWiFiMillis)/1000);
    digitalWrite(LED_BUILTIN, HIGH);   
    delay(1000);    
      
  } else {
     sprintf(lcdBuffer, "%lddBm %d.%d.%d.%d", WiFi.RSSI(), WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
  }
  
  delay(1000);
}

/*---------------------------------------------------------------------------------*/




/*------------------------- MQTT Thread---------------------------------------------*/

void threadMQTT() {
  const char mqttBroker[63] = "test.mosquitto.org";
  
  if (WiFi.status() == WL_CONNECTED){
    mqttClient.poll();
    sprintf(topicPub, "%s%s%s", "ece508/", teamnumber, "/nano33iot");  
  
    if (!mqttClient.connected()) {
      long prevMQTTMillis = millis();
               
      sprintf(mqttClienId, "client_%04d_%s", random(0, 9999), teamnumber);
      mqttClient.setId(mqttClienId);
      
      while (!mqttClient.connect(mqttBroker, 1883)) {
         delay(500);
      }
      sprintf(lcdBuffer, "MQTT conn in %d sec", (millis()-prevMQTTMillis)/1000);
     
      
    } else {
      
      mqttStringMessage = "";
      nmrMqttMesages++;
      
      myJsonDoc["msgnum"] = String(nmrMqttMesages);
      getMacWifiShieldMacRouterSS(lcdBuffer);
      myJsonDoc["mac"] = String(lcdBuffer);
      convDDHHMMSS(millis()/1000, lcdBuffer);
      myJsonDoc["uptime"] = String(lcdBuffer);
      myJsonDoc["analogA7"] = String(analogRead(A7));
      myJsonDoc["bn220"]["loc"]["latrnd"] = String(latRnd, 5);
      myJsonDoc["bn220"]["loc"]["lonrnd"] = String(lonRnd, 5);
      myJsonDoc["bn220"]["loc"]["Speed"] = String(Speed, 1);
      
      epochNTP = ntpClient.getEpochTime();
      myJsonDoc["epochNTP"] = String(epochNTP);
      mqttStringMessage = JSON.stringify(myJsonDoc);
      mqttClient.beginMessage(topicPub); mqttClient.print(mqttStringMessage); mqttClient.endMessage();    
    }
  
  }
  delay(3000);
}

/*----------------------------------------------------------------------------------------*/




/*--------------------------------- OLED Thread---------------------------------------------*/

void threadOLED() {
  ntpClient.update();
  
  convCurrentTimeUTC(ntpClient.getEpochTime(), lcdBuffer);
    
  convDDHHMMSS(millis()/1000, lcdBuffer); 
  oledline[8] = "Uptime: " + String(lcdBuffer);
  displayTextOLED(oledline, 1);
  double latRnd = (double) random(0, 1000)/1000;
  double lonRnd = (double) random(0, 1000)/1000;
  oledline[3] = "Speed(mph): " + String(gps.speed.mph(),2);
  oledline[4] = "Latitude:   " + String(gps.location.lat()+latRnd,5); 
  oledline[5] = "Longitude:  " + String(gps.location.lng()+lonRnd,5);
  
/*-------------------------------------------------------------------------------------------*/


/*-------------------------------------Over Speeding---------------------------------------------*/
 
  if (gps.speed.mph()>= 20){
    oledline[7] = "!!! OverSpeeding !!!";
    digitalWrite(RED_ledPin, HIGH);
  }   else {
   digitalWrite(RED_ledPin,LOW);
  }
  delay(1000);
}

/*---------------------------------------------------------------------------------------------------------*/




/*------------------------------ GPS Thread----------------------------------------------------------------*/

void threadGPS() {
  smartDelay(250);

  sprintf(lcdBuffer, "%04d-%02d-%02d %02d:%02d:%02d", gps.date.year(), gps.date.month(), gps.date.day(), 
                                                      gps.time.hour(), gps.time.minute(), gps.time.second());
  

  latRnd = (double) (gps.location.lat() + (double) random(0, 1000)/1000);
  lonRnd = (double) (gps.location.lng() + (double) random(0, 1000)/1000);
  Speed  = (double) gps.speed.kmph();
  
  delay(1000);
}

/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------GPS Deleay to push Data------------------------------------------------*/

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

/*--------------------------------------------- end of code ---------------------------------------------------*/
