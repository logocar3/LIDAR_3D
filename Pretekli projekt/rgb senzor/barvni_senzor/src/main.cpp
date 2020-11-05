
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

// #define DEBUG
//const char* ssid = "Grdina WiFi";
//const char* password = "parmezan";
//const char* ssid = "sarakevin";
//const char* password = "poiqwe123";
const char* ssid = "Innbox-Internet-29ebbd";
const char* password = "parmezan";
//const char* ssid = "Avtomatika_AMS";
//onst char* password = "AvtonomniMobilniSistemi";
WiFiMulti wifiMulti;
ESP32WebServer server(80);

File tekst;

EV3UARTSensor sensor(16,17);



void handleRoot() {
  /* server respond 200 with content "hello from ESP32!" */
  server.send(200, "text/plain", "hello from ESP32!");
}

void handleNotFound(){
  String message = "File Not Found\n\n";
  server.send(404, "text/plain", message);
}

void SD_file_download(String strText){
    #ifdef DEBUG
      Serial.println("sem v sd_file_download");
    #endif
    File download = SD.open("/" + strText); // Zakaj imaš argument strText če ga ne uporabiš?

    if (download) {
      Serial.println("pred server.stream");
      // mogoce "text/html" namesto application // Ali ste mogoče pomislili na ostale vrstice v primeru?
      server.sendHeader("Content-Type", "text/text");
      server.sendHeader("Content-Disposition", "attachment; filename="+strText);
      server.sendHeader("Connection", "close");
      size_t sent=server.streamFile(download, "application/octet-stream");
      //HTTPUpload& upload = server.upload();
      Serial.println("stream");
      download.close();  // Preden zapremo datoteko pustiti procesorju da dokonča zadeve
    }
    else if(!download) { // Ni potrebno še en if, ker je lahko le true/false
      Serial.println("ni uspel sd.open");
    }

}

void File_Download(){ // This gets called twice, the first pass selects the input, the second pass then processes the command line arguments
  //server.send(200, "text/plain", "prenos!");
  Serial.println("prenos");
  SD_file_download("tekst.csv");
  }


void setup() {
   Serial.begin(115200);
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

  //delay(100);

  //WiFi.config(ip, gateway, subnet);
  Serial.println("wifi begin0");
  delay(100);
  Serial.println("pred povezavo");
  delay(100);
  WiFi.begin(ssid, password);
  Serial.println("wifi begin1");
    /* Wait for connection */
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  /* we use mDNS here http://esp32.local */
  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }
  
  /* register callback function when user request root "/" */
  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);
  
  server.on("/File_Download", File_Download); 
  /* start web server */
  server.begin();
  Serial.println("HTTP server started");

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
    sensor.set_mode(mode);
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
    int d_test=millis();

    int frekvenca=millis();
    for(int u=0;u<=249;u++){
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
    d_test=millis()-d_test;


    Serial.println("konec");
    Serial.println("cas testa");
    Serial.println(d_test);
    delay(200);
    // close the file:
    tekst.close();
    Serial.println("done.");
    Serial.println(WiFi.localIP());
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