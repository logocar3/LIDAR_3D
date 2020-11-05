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
EV3UARTSensor sensor(16,17); //preveri se enkrat!!!!

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
         /* we use mDNS here http://esp32.local */
         if (MDNS.begin("esp32")) {
            Serial.println("MDNS responder started");
         }
         delay(100);
         
         /* register callback function when user request root "/" */
         server.on("/", handleRoot);
         server.onNotFound(handleNotFound);
         
         server.on("/File_Download", File_Download); 
         // 192.168.222.157/File_Download
         /* start web server */
         server.begin();
         Serial.println("HTTP server started");

         Serial.print("Test ce pride do sem");
         sensor.begin();
         Serial.println("\n Test");
         delay(100);

}

unsigned long lastMessage = 0;

void loop() { 

sensor.check_for_data();

if (sensor.get_status() == DATA_MODE && (millis() - lastMessage) > 1000 ) {
    Serial.println("\n I'm in!");
    
    int16_t enkoderL = 0;
    int16_t enkoderD = 0;
    int napaka = 0; //razilka med enkoderiji
    /*
    float Kp=dutyCycleD/10000; //ojacanje
    float Ki=0.2;
    float kd=1000;

    int napaka_integral=0;
    int napaka_odvod=0;
    int odvod=0;
    */


    Serial.println("nastavljam mode");
    sensor.set_mode(mode);
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
      //valodltra je referenca 100 v lj 70
      int referecna=110;
      float r_prejsni=0;
      float g_prejsni=0;
      float b_prejsni=100;
      float kp =1.25;
      float ki=-1;
      int start =0;
      float cas=0;
      float napaka0 = 0; //razilka med 
      float napaka1 =0;
      //float napaka2=0;
      //float u=0; NESME BIT U KER JE ŽE LOOP
      float q0= -0.1228;
      float q1= -22.325*0.011;
      float uu=0;
      float nap=0;
      float nap1=0;
      

      start = millis();
      //valodltra 480
      for(int u=0;u<=400;u++){

        //pcnt_get_counter_value(PCNT_UNIT_0, &enkoderL);
        //pcnt_get_counter_value(PCNT_UNIT_1, &enkoderD);

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
        if(sample[0] > 255){
            sample[0]=r_prejsni;
          }
          if(sample[1] > 255){
            sample[1]=g_prejsni;
          }
          if(sample[2] > 255){
            sample[2]=b_prejsni;
          }
      

        //tekst.print(sample[0]);
        //tekst.print(',');
        //tekst.print(sample[1]);
        //tekst.print(',');

        napaka=referecna-sample[2];
        //napaka0=kp*napaka;
        

        //uu=q0*napaka0+q1*napaka1;
        //up=uu;
        up= kp*napaka;
        //nap1=nap;
        //napaka1=napaka0;
        
        if(up>50)
        {
          up=50;
        }

        if(up<-50)
        {
          up=-50;
        }

        dutyCycleD=dutyCycle+up;
        dutyCycleL=dutyCycle-up;

          digitalWrite(MLP3, LOW);
          digitalWrite(MLP4, HIGH);
          ledcWrite(pwmChannelL, dutyCycleL);
          
          digitalWrite(MDP1, LOW);
          digitalWrite(MDP2, HIGH);
          ledcWrite(pwmChannelD, dutyCycleD);
          
        cas = millis()-start;

        tekst.print(cas);
        tekst.print(',');
        tekst.print(up);
        tekst.print(',');
        tekst.print(uu);
        tekst.print(',');
        tekst.print(sample[2]);
        tekst.print(',');        
        tekst.println(napaka0);

        r_prejsni=sample[0];
        g_prejsni=sample[1];
        b_prejsni=sample[2];
        //tekst.println(';');
        //Serial.println("zapisal v tekst");
        //frekvenca 20 z 200 vzorci je bla prej, zdej probam 10 in 400
        while(millis()-frekvenca <= 10){

        }
        frekvenca=millis();
      }
      Serial.println("konec");
      Serial.println("cas testa:");
      d_test=millis()-d_test;
      Serial.println(d_test);



      delay(200);
      // close the file:
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

      };
    } else {
      // if the file didn't open, print an error:
      Serial.println("error opening tekst.csv");
    }

    
      Serial.println();
      lastMessage = millis();
  }
  
}