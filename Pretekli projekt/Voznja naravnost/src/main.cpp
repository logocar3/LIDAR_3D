#include <Arduino.h>
#include <stdio.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include "driver/pcnt.h"

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

#define RELOAD_TMR      		1
#define PCNT_PULSE_GPIO1				32		// gpio for PCNT
#define PCNT_CONTROL_GPIO1			35
#define PCNT_PULSE_GPIO2				25		// gpio for PCNT
#define PCNT_CONTROL_GPIO2			33
//#define DIRECTION					25		// gpio for encoder direction input
// 
#define PCNT_H_LIM_VAL      32767 //int16
#define PCNT_L_LIM_VAL     -32767
//sd kartica
#define SD_CS 5
#define BYTE_NACK 0x02
#define mode 4
const char* ssid = "sarakevin";
const char* password = "poiqwe123";
//const char* ssid = "Avtomatika_AMS";
//const char* password = "AvtonomniMobilniSistemi";


//const char* ssid = "Innbox-internet-29ebbd";
//const char* password = "parmezan";



WiFiMulti wifiMulti;
ESP32WebServer server(80);

File tekst;
//EV3UARTSensor sensor(16,17); //preveri se enkrat!!!!

// desni motor
int MDP1 = 27; //motor desni pin 1 na h-bridge
int MDP2 = 26; //motor desni pin 2
int enableDESNI = 14; 

// levi motor
int MLP3 = 34; //motor levi pin 3
int MLP4 = 12; //motor levi pin 4
int enableLEVI = 13; 

// Setting PWM properties
const int freq = 30000;
const int pwmChannelL = 0;
const int pwmChannelD = 1;
const int resolution = 8;
float dutyCycleL = 150;
float dutyCycleD = 150;
int dutyCycle=150;
float up=0;

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
  SD_file_download("tekst.csv");
  
  
  }


void setup() {
  Serial.begin(9600);
        pcnt_config_t pcnt_config0 = {
		        .pulse_gpio_num = PCNT_PULSE_GPIO1,
		        .ctrl_gpio_num = PCNT_CONTROL_GPIO1,
            .lctrl_mode = PCNT_MODE_KEEP, 			// Reverse counting direction if low
		        .hctrl_mode = PCNT_MODE_REVERSE,    		            	// Keep the primary counter mode if high
            .pos_mode = PCNT_COUNT_INC,   			// Count up on the positive edge
		        .neg_mode = PCNT_COUNT_DEC,   			// Keep the counter value on the negative edge  
		        .counter_h_lim = PCNT_H_LIM_VAL,
		        .counter_l_lim = PCNT_L_LIM_VAL,
            .unit = PCNT_UNIT_0,
            .channel = PCNT_CHANNEL_0,
		    };
        pcnt_config_t pcnt_config1 = {
		    
            .pulse_gpio_num = PCNT_CONTROL_GPIO1,
		        .ctrl_gpio_num = PCNT_PULSE_GPIO1,
            .lctrl_mode = PCNT_MODE_REVERSE, 			// Reverse counting direction if low
		        .hctrl_mode = PCNT_MODE_KEEP,    		            	// Keep the primary counter mode if high
            .pos_mode = PCNT_COUNT_INC,   			// Count up on the positive edge
		        .neg_mode = PCNT_COUNT_DEC,   			// Keep the counter value on the negative edge  
		        .counter_h_lim = PCNT_H_LIM_VAL,
		        .counter_l_lim = PCNT_L_LIM_VAL,
            .unit = PCNT_UNIT_0,
            .channel = PCNT_CHANNEL_1, 
		    };
        pcnt_config_t pcnt_config2 = {
		        .pulse_gpio_num = PCNT_PULSE_GPIO2,
		        .ctrl_gpio_num = PCNT_CONTROL_GPIO2,
            .lctrl_mode = PCNT_MODE_KEEP, 			// Reverse counting direction if low
		        .hctrl_mode = PCNT_MODE_REVERSE,    		            	// Keep the primary counter mode if high
            .pos_mode = PCNT_COUNT_INC,   			// Count up on the positive edge
		        .neg_mode = PCNT_COUNT_DEC,   			// Keep the counter value on the negative edge  
		        .counter_h_lim = PCNT_H_LIM_VAL,
		        .counter_l_lim = PCNT_L_LIM_VAL,
            .unit = PCNT_UNIT_1,
            .channel = PCNT_CHANNEL_0,  
		    };
        pcnt_config_t pcnt_config3 = {
		    
            .pulse_gpio_num = PCNT_CONTROL_GPIO2,
		        .ctrl_gpio_num = PCNT_PULSE_GPIO2,
            .lctrl_mode = PCNT_MODE_REVERSE, 			// Reverse counting direction if low
		        .hctrl_mode = PCNT_MODE_KEEP,    		            	// Keep the primary counter mode if high
            .pos_mode = PCNT_COUNT_INC,   			// Count up on the positive edge
		        .neg_mode = PCNT_COUNT_DEC,   			// Keep the counter value on the negative edge  
		        .counter_h_lim = PCNT_H_LIM_VAL,
		        .counter_l_lim = PCNT_L_LIM_VAL,
            .unit = PCNT_UNIT_1,
            .channel = PCNT_CHANNEL_1,
		    };


        pcnt_unit_config(&pcnt_config0);
        pcnt_unit_config(&pcnt_config1);
        pcnt_set_filter_value(PCNT_UNIT_0, 100);
        pcnt_filter_enable(PCNT_UNIT_0);

        pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_ZERO);
        pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
        pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);

        /* Initialize PCNT's counter */
        pcnt_counter_pause(PCNT_UNIT_0);
        pcnt_counter_clear(PCNT_UNIT_0);

        pcnt_intr_enable(PCNT_UNIT_0);

        /* Everything is set up, now go to counting */
        pcnt_counter_resume(PCNT_UNIT_0);

        pcnt_unit_config(&pcnt_config2);
        pcnt_unit_config(&pcnt_config3);
        pcnt_set_filter_value(PCNT_UNIT_1, 100);
        pcnt_filter_enable(PCNT_UNIT_1);

        pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_ZERO);
        pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
        pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_L_LIM);

        /* Initialize PCNT's counter */
        pcnt_counter_pause(PCNT_UNIT_1);
        pcnt_counter_clear(PCNT_UNIT_1);

        pcnt_intr_enable(PCNT_UNIT_1);

        /* Everything is set up, now go to counting */
        pcnt_counter_resume(PCNT_UNIT_1);
        pinMode(MLP3, OUTPUT);
        pinMode(MLP4, OUTPUT);
        pinMode(enableLEVI, OUTPUT);
        ledcSetup(pwmChannelL, freq, resolution);
        ledcAttachPin(enableLEVI, pwmChannelL);

        pinMode(MDP1, OUTPUT);
        pinMode(MDP2, OUTPUT);
        pinMode(enableDESNI, OUTPUT);
        ledcSetup(pwmChannelD, freq, resolution);
        ledcAttachPin(enableDESNI, pwmChannelD);  
         
         //sd kartica
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
         
           //WiFi.config(ip, gateway, subnet);
         Serial.println("wifi begin0");
         delay(1000);
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
         /* we use mDNS here http://kevin.local */
         if (MDNS.begin("kevin")) {
            Serial.println("MDNS responder started");
         }
         
         /* register callback function when user request root "/" */
         server.on("/", handleRoot);
         server.onNotFound(handleNotFound);
         
         server.on("/File_Download", File_Download); 
         // 192.168.222.157/File_Download
         /* start web server */
         server.begin();
         Serial.println("HTTP server started");

         Serial.print("Test ce pride do sem");
         //sensor.begin();
         Serial.println("\n Test");

}

void loop() {
 int16_t enkoderL = 0;
 int16_t enkoderD = 0;
 float napaka0 = 0; //razilka med enkoderiji
 float napaka1 =0;
 float napaka2 =0;
 float napaka=0;
 int referenca=0;
 /*
 float Kp=dutyCycleD/10000; //ojacanje
 float Ki=0.2;
 float kd=1000;

 int napaka_integral=0;
 int napaka_odvod=0;
 int odvod=0;
 */





float q0= -2.43;
float q1=  3.24;
float q2= -0.81;



 Serial.println(" ");
 tekst = SD.open("/tekst.csv", FILE_WRITE);
 Serial.println("odpru tekst.csv"); 
 /*
         digitalWrite(MDP1, LOW);
         digitalWrite(MDP2, HIGH);
         ledcWrite(pwmChannelD, dutyCycleD);
         //levi motor
         digitalWrite(MLP3, LOW);
         digitalWrite(MLP4, HIGH);
         ledcWrite(pwmChannelL, dutyCycleL);
        */

if(tekst){
      int frekvenca=millis();
      while(enkoderD<10000){
         
         pcnt_get_counter_value(PCNT_UNIT_0, &enkoderL);
         pcnt_get_counter_value(PCNT_UNIT_1, &enkoderD);


        napaka = enkoderL-enkoderD ;//+ 0.2829; //delovna tocka y=-0.2829
        napaka0=referenca-napaka;

        up=up+ q0*napaka0 +q1*napaka1 +q2*napaka2;
        // preveri če mogoče boljse :
        // duttcycleD = dutycycleD+up
        dutyCycleD=dutyCycle+up;
        dutyCycleL=dutyCycle-up;

        napaka2=napaka1;
        napaka1=napaka0;
        

        /* 
         napaka_integral=napaka_integral+napaka;
         
         odvod=napaka-napaka_odvod;
         
         //printf("napaka: %.2f \n",napaka);
         dutyCycleL = dutyCycleL + napaka/Kp + napaka_integral*Ki + kd*odvod;

         napaka_odvod=napaka;
         */


         // 192.168.222.157/File_Download
         // 192.168.222.198
         if (dutyCycleL >= 180 ){
           dutyCycleL=180;
            
         }
         if (dutyCycleL <= 120 ){
           dutyCycleL=120;
            
         }


         if (dutyCycleD >= 180 ){
           dutyCycleD=180;
            
         }
         if (dutyCycleD <= 120){
           dutyCycleD=120;
            
         }
         //desni motor
         digitalWrite(MDP1, LOW);
         digitalWrite(MDP2, HIGH);
         ledcWrite(pwmChannelD, dutyCycleD);
         //levi motor
         digitalWrite(MLP3, LOW);
         digitalWrite(MLP4, HIGH);
         ledcWrite(pwmChannelL, dutyCycleL);
          
                     
                     tekst.print(enkoderL);
                     tekst.print(',');
                     tekst.print(enkoderD);
                     tekst.print(',');
                     tekst.print(dutyCycleL);
                     tekst.print(',');
                     tekst.print(dutyCycleD);
                     tekst.print(',');
                     tekst.print(up);
                     tekst.print(',');
                     tekst.println(napaka);
                     
                     
                     while(millis()-frekvenca <= 50){

                      }
                     frekvenca=millis();
          

      }
      delay(100);
      tekst.close();
      Serial.println("done.");
      //desni motor
         digitalWrite(MDP1, LOW);
         digitalWrite(MDP2, LOW);
         
         //levi motor
         digitalWrite(MLP3, LOW);
         digitalWrite(MLP4, LOW);
         
         delay(100); 
      while(1){
         
         server.handleClient();
         
                  
      }
  } else{
     Serial.println("error opening tekst.csv");
      while(1){
         
         
         
                  
      }
  }
}