#include <Arduino.h>
// knjiznice SD
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// SD card pin
#define SD_CS 5

File tekst;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //pinMode(19,INPUT_PULLUP);
  //pinMode(SD_CS,INPUT_PULLUP);
  //pinMode(23,INPUT_PULLUP);
  //pinMode(18,INPUT_PULLUP);
 while (!Serial) {
    ; // pocakaj na povezavo
  }
  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_CS)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  tekst = SD.open("/tekst.txt", FILE_APPEND);

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


}

void loop() {
  // put your main code here, to run repeatedly:
}