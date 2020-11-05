
//#define DEBUG true
#include <Arduino.h>
#include <EV3UARTSensor.h>
// knjiznice SD
#include <FS.h>
#include <SD.h>
#include <SPI.h>
// knjizice wifi
#include <WiFi.h>              
#include <WiFiMulti.h>        
#include <ESP32WebServer.h>   
#include <ESPmDNS.h>
#include <WiFiClient.h>
// #include <Serial.h>
//#include <SoftwareSerial.h>

// SD card pin
#define SD_CS 5
#define BYTE_NACK 0x02
#define mode 4
 
//const char* ssid = "sarakevin";
//const char* password = "poiqwe123";
const char* ssid = "Avtomatika_AMS";
const char* password = "AvtonomniMobilniSistemi";
WiFiMulti wifiMulti;
ESP32WebServer server(80);

File tekst;

EV3UARTSensor sensor(16,17);


void setup() {
   Serial.begin(9600);
   Serial.println(MISO);
   pinMode(23,INPUT_PULLUP);
   pinMode(19,INPUT_PULLUP);
   pinMode(18,INPUT_PULLUP);
   pinMode(5,INPUT_PULLUP);


  while (!Serial) {
    ; // pocakaj na povezavo
  }
  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_CS)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");



  Serial.print("Test ce pride do sem");
  sensor.begin();
  Serial.println("\n Test");

}

unsigned long lastMessage = 0;

void loop() {
  sensor.check_for_data();
  if (sensor.get_status() == DATA_MODE && (millis() - lastMessage) > 1000 ) {
    Serial.println("\nI'm in!");
 
    Serial.println("nastavljam mode");
    sensor.set_mode(mode);
    Serial.println("mode nastavljen");
    delay(100);   

    Serial.print("Current mode is ");
    Serial.println(sensor.get_current_mode());
    Serial.print("Sample size is ");
    Serial.println(sensor.sample_size());
    Serial.print("Sensor type is ");
    Serial.println(sensor.get_type());
 
    float sample[sensor.sample_size()];
    sensor.fetch_sample(sample, 0);
    Serial.print("Sample is ");
	for(int i=0;i<sensor.sample_size();i++) {
	  Serial.print(sample[i]);
	  Serial.print(" ");
	}

  //zacetek testa
  Serial.println(" ");
  tekst = SD.open("/tekst.csv", FILE_WRITE);
  Serial.println("odpru tekst.csv");

 // if the file opened okay, write to it: Bla bla
  if (tekst) {
    Serial.println("zacetek testa");
    int frekvenca=millis();
    for(int u=0;u<=250;u++){
      //Serial.println("sem v testu");
      Serial2.write(BYTE_NACK);
      //Serial.println(" ");
      
      //Serial.println("napisal BYTE_NACK");
      while(Serial2.available()){
      sensor.check_for_data();
      }
      
      //Serial.println("senzor pogleda za data");
  
      float sample[sensor.sample_size()];
      sensor.fetch_sample(sample, 0);
      tekst.print(sample[0]);
      tekst.print(',');
      tekst.print(sample[1]);
      tekst.print(',');
      tekst.println(sample[2]);
      //tekst.println(';');
      //Serial.println("zapisal v tekst");
      while(millis()-frekvenca <= 20){

      }
      frekvenca=millis();
    }
    Serial.println("konec");
    delay(200);
    // close the file:
    tekst.close();
    Serial.println("done.");
    while(1){
      server.handleClient();

    };
   } else {
    // if the file didn't open, print an error:
    Serial.println("error opening tekst.csv");
   }

  
	  Serial.println();
    lastMessage = millis();
  }
}