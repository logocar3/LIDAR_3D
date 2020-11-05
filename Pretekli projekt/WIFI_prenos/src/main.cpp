#include <Arduino.h>
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



//#include <Network.h>
//#include <Sys_Variables.h>
//#include <CSS.h>

// SD card pin
#define SD_CS 5

//const char* ssid = "sarakevin";
//const char* password = "poiqwe123";
const char* ssid = "Avtomatika_AMS";
const char* password = "AvtonomniMobilniSistemi";
WiFiMulti wifiMulti;
ESP32WebServer server(80);



File tekst;


void handleRoot() {
  /* server respond 200 with content "hello from ESP32!" */
  server.send(200, "text/plain", "hello from ESP32!");
}

void handleNotFound(){
  String message = "File Not Found\n\n";
  server.send(404, "text/plain", message);
}

void SD_file_download(String strText){
    Serial.println("sem v sd_file_download");
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
  SD_file_download("tekst.txt");
  }




void setup() {
  // put your setup code here, to run once:
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

  // PISANJE V DATOTEKO
  tekst = SD.open("/tekst.txt", FILE_WRITE);
  // if the file opened okay, write to it: Bla bla
  if (tekst) {
    Serial.print("Writing to tekst.txt...");
    tekst.println("test  22233");
    // close the file:
    tekst.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening tekst.txt");
  }

  //WiFi.config(ip, gateway, subnet);
  Serial.println("wifi begin");
  delay(1000);
  WiFi.begin(ssid, password);
  Serial.println("wifi begin");
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


}
void loop() {
  // put your main code here, to run repeatedly:
  server.handleClient();
}

