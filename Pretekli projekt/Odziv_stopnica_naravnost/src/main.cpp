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
//const char* ssid = "sarakevin";
//const char* password = "poiqwe123";
const char* ssid = "Avtomatika_AMS";
const char* password = "AvtonomniMobilniSistemi";
WiFiMulti wifiMulti;
ESP32WebServer server(80);
/*
int prbs[]={180,180,180,180,180,180,180,180,180,120,120,120,120,120,180,180,180,180,120,180,180,180,180,180,120,120,120,180,120,180,180,180,120,120,180,180,120,120,180,120,120,120,120,120,180,120,120,180,120,180,120,120,180,180,180,120,180,180,120,180,120,120,120,180,180,180,180,120,120,180,180,180,180,180,120,120,180,180,120,180,180,120,120,120,180,120,180,120,180,120,120,180,120,120,120,180,180,180,120,120,120,180,180,120,180,180,120,180,120,180,120,180,180,180,120,120,120,180,120,120,180,180,120,120,120,180,120,120,120,180,120,120,120,120,120,120,120,120,180,120,120,120,120,180,120,120,120,180,180,120,120,120,120,180,120,120,180,180,180,120,120,180,120,180,120,180,120,180,180,120,120,120,120,180,180,120,180,180,180,180,120,180,120,120,180,180,120,180,180,180,120,120,180,120,120,120,180,120,180,120,120,120,120,180,120,180,120,180,180,120,180,120,120,180,180,180,180,180,180,120,180,180,120,120,180,120,120,180,120,120,180,120,180,180,120,180,180,180,180,180,180,120,120,180,120,120,180,180,120,180,120,180,120,120,180,180,120,120,180,180,120,120,120,120,120,120,120,180,180,120,120,120,180,180,120,120,180,120,180,120,120,120,180,180,120,180,120,120,180,120,180,180,180,180,180,180,180,120,180,120,120,120,180,120,180,180,120,120,120,180,180,180,120,180,120,180,180,120,120,180,120,180,180,120,120,180,180,180,180,120,120,120,180,180,180,180,180,120,180,180,180,120,180,120,120,120,120,120,180,180,120,180,120,180,180,120,180,180,120,180,180,180,120,180,180,120,120,120,120,120,180,120,180,180,120,180,120,180,180,180,180,180,120,180,120,180,120,180,120,180,120,120,120,120,120,120,180,120,180,120,120,180,120,180,120,180,180,180,180,120,120,180,120,180,180,180,120,180,180,180,120,120,120,120,120,120,180,180,180,120,120,180,180,180,120,180,120,120,180,120,120,180,180,180,180,120,180,120,180,180,180,120,180,120,180,120,120,120,180,120,120,180,120,120,120,120,180,180,120,120,180,180,180,120,120,120,120,180,120,180,180,180,180,120,180,180,120,180,180,120,120,180,180,120,180,120,120,120,120,180,180,180,120,180,180,180,180,120,120,120,120
}; */

/*int prbs[]={30,30,30,30,30,30,30,30,30,-30,-30,-30,-30,-30,30,30,30,30,-30,30,30,30,30,30,-30,-30,-30,30,-30,30,30,30,-30,-30,30,30,-30,-30,30,-30,-30,-30,-30,-30,30,-30,-30,30,-30,30,-30,-30,30,30,30,-30,30,30,-30,30,-30,-30,-30,30,30,30,30,-30,-30,30,30,30,30,30,-30,-30,30,30,-30,30,30,-30,-30,-30,30,-30,30,-30,30,-30,-30,30,-30,-30,-30,30,30,30,-30,-30,-30,30,30,-30,30,30,-30,30,-30,30,-30,30,30,30,-30,-30,-30,30,-30,-30,30,30,-30,-30,-30,30,-30,-30,-30,30,-30,-30,-30,-30,-30,-30,-30,-30,30,-30,-30,-30,-30,30,-30,-30,-30,30,30,-30,-30,-30,-30,30,-30,-30,30,30,30,-30,-30,30,-30,30,-30,30,-30,30,30,-30,-30,-30,-30,30,30,-30,30,30,30,30,-30,30,-30,-30,30,30,-30,30,30,30,-30,-30,30,-30,-30,-30,30,-30,30,-30,-30,-30,-30,30,-30,30,-30,30,30,-30,30,-30,-30,30,30,30,30,30,30,-30,30,30,-30,-30,30,-30,-30,30,-30,-30,30,-30,30,30,-30,30,30,30,30,30,30,-30,-30,30,-30,-30,30,30,-30,30,-30,30,-30,-30,30,30,-30,-30,30,30,-30,-30,-30,-30,-30,-30,-30,30,30,-30,-30,-30,30,30,-30,-30,30,-30,30,-30,-30,-30,30,30,-30,30,-30,-30,30,-30,30,30,30,30,30,30,30,-30,30,-30,-30,-30,30,-30,30,30,-30,-30,-30,30,30,30,-30,30,-30,30,30,-30,-30,30,-30,30,30,-30,-30,30,30,30,30,-30,-30,-30,30,30,30,30,30,-30,30,30,30,-30,30,-30,-30,-30,-30,-30,30,30,-30,30,-30,30,30,-30,30,30,-30,30,30,30,-30,30,30,-30,-30,-30,-30,-30,30,-30,30,30,-30,30,-30,30,30,30,30,30,-30,30,-30,30,-30,30,-30,30,-30,-30,-30,-30,-30,-30,30,-30,30,-30,-30,30,-30,30,-30,30,30,30,30,-30,-30,30,-30,30,30,30,-30,30,30,30,-30,-30,-30,-30,-30,-30,30,30,30,-30,-30,30,30,30,-30,30,-30,-30,30,-30,-30,30,30,30,30,-30,30,-30,30,30,30,-30,30,-30,30,-30,-30,-30,30,-30,-30,30,-30,-30,-30,-30,30,30,-30,-30,30,30,30,-30,-30,-30,-30,30,-30,30,30,30,30,-30,30,30,-30,30,30,-30,-30,30,30,-30,30,-30,-30,-30,-30,30,30,30,-30,30,30,30,30,-30,-30,-30,-30
}; */

float prbs[]={12.886,12.886,12.886,12.886,12.886,12.886,-13.183,-13.183,-13.183,-13.183,28.919,28.919,28.919,28.919,-28.436,-28.436,-28.436,-28.436,-15.92,-15.92,-15.92,-15.92,-15.92,25.894,25.894,25.894,25.894,25.894,25.894,25.894,25.894,25.894,-13.888,-13.888,-13.888,-13.888,-13.888,-13.888,-13.888,-13.888,-13.888,5.9687,5.9687,5.9687,5.9687,5.9687,5.9687,25.827,25.827,25.827,25.827,25.827,25.827,25.827,25.827,1.7732,1.7732,1.7732,1.7732,1.7732,1.7732,1.7732,1.7732,1.7732,1.7732,13.139,13.139,13.139,13.139,13.139,13.139,13.139,13.139,-9.6184,-9.6184,-9.6184,-9.6184,-9.6184,-9.6184,-9.6184,-23.78,-19.467,-19.467,-19.467,-19.467,-13.257,-13.257,-13.257,-13.257,-13.257,-13.257,-13.257,-13.257,18.433,18.433,18.433,18.433,18.433,18.433,18.433,18.433,18.433,18.433,-3.1476,-3.1476,-3.1476,-3.1476,-3.1476,-3.1476,-7.9525,-7.9525,-7.9525,-7.9525,-7.9525,-7.9525,-7.9525,-7.9525,0.20155,0.20155,0.20155,0.20155,14.574,14.574,14.574,14.574,14.574,14.574,14.574,-7.0468,-7.0468,-7.0468,-7.0468,17.219,17.219,17.219,17.219,17.219,17.219,-28.317,-28.317,-28.317,-28.317,-28.317,-28.317,-28.317,-28.317,-28.317,-12.399,-12.399,-12.399,-12.399,-12.399,-12.399,12.171,12.171,20.064,20.064,20.064,20.064,20.064,20.064,20.064,20.064,27.846,27.846,27.846,18.643,18.643,18.643,18.643,18.643,18.643,18.643,18.643,18.643,18.643,-0.788,-0.788,-0.788,-0.788,-0.788,-0.788,-0.788,-0.788,-0.788,-0.788,19.117,19.117,19.117,19.117,19.117,19.117,19.117,19.117,25.406,1.3414,1.3414,1.3414,1.3414,1.3414,1.3414,1.3414,1.3414,16.662,16.662,16.662,16.662,3.1421,3.1421,3.1421,-15.625,-15.625,-15.625,-15.625,-15.625,2.5028,2.5028,2.5028,2.5028,2.5028,2.5028,2.5028,-17.665,-17.665,-17.665,22.684,22.684,22.684,22.684,22.684,22.684,22.684,-0.22778,-0.22778,-0.22778,-0.22778,-27.681,-27.681,-27.681,-27.681,-27.681,-27.681,-21.881,-21.881,-21.881,-21.881,-21.881,-21.881,-21.881,-21.881,-21.881,-21.881,26.662,26.662,26.662,26.662,26.662,26.662,26.662,26.662,29.442,-3.6121,-0.72325,-0.72325,-0.72325,-0.72325,-0.72325,-0.72325,-0.72325,-0.72325,26.807,26.807,26.807,26.807,26.807,-16.155,-16.155,24.524,24.524,-11.311,-11.311,-11.311,-11.311,-11.311,-11.311,15.741,15.741,-15.366,-29.218,-29.218,-29.218,-29.218,-29.218,-29.218,-29.218,-29.218,-29.218,-29.218,2.0932,2.3134,2.3134,2.3134,2.3134,2.3134,2.3134,2.3134,2.3134,2.3134,2.3134,-17.612,-3.2593,-11.314,-11.314,-11.314,-11.314,-11.314,-1.9584,-1.9584,-1.9584,-1.9584,17.885,17.885,17.885,-1.1373,-1.1373,26.207,-23.612,-23.612,-13.257,27.941,27.941,27.941,27.941,27.941,27.941,27.941,27.941,27.941,-24.828,-24.828,-24.828,-24.828,-24.828,-24.828,13.685,27.627,27.627,27.627,27.627,27.627,27.627,27.627,27.627,18.533,18.533,18.533,18.533,-3.2766,-3.2766,-3.2766,-3.2766,9.0861,9.0861,9.0861,9.0861,9.0861,9.0861,9.0861,9.0861,9.0861,3.19,3.19,3.19,3.19,-11.841,-11.841,-11.841,-11.841,-11.841,-11.841,-1.3615,-1.3615,-1.3615,-1.3615,-1.3615,-1.3615,-1.3615,-1.3615,-1.3615,27.749,27.749,27.749,27.749,27.749,27.749,27.749,27.749,27.749,-21.899,10.088,10.088,10.088,10.088,-15.993,-15.993,26.986,26.986,26.986,-29.25,28.1,28.1,28.1,28.1,28.1,4.9018,29.531,29.531,29.531,29.531,29.531,29.531,29.531,-22.718,-22.718,-22.718,-22.718,-22.718,-22.718,-22.718,-22.718,-22.718,-5.9967,-5.9967,-5.9967,-5.9967,-5.9967,-5.9967,-5.9967,-5.9967,-5.9967,-5.9967,25.901,25.901,25.901,25.901,25.901,28.224,28.224,28.224,28.224,-13.06,-13.06,-13.06,-13.06,-13.06,-13.06,-13.06,-13.06,-13.06,-15.772,-15.772,-15.772,-15.772,-25.357,-25.357,-27.819,-27.819,-27.819,-27.819,-27.819,-1.6233,-22.736,-22.736,-22.736,-22.736,28.073,28.073,28.073,28.073,28.073,28.073,28.073,16.051,16.051,-26.254,-26.254,-26.254,-26.254,-26.254,-26.254,-26.254,-26.254,-26.254,-26.254,3.7341,3.7341,3.7341,3.7341,3.7341,-24.532,-24.532,-24.532,-24.532,-24.532,-24.532,-24.532,-24.532,-24.532,-13.373,-13.373,-13.373,-13.373,-13.373,-13.373,-13.373,-13.373,-13.373,-13.373,-27.904,-27.904,-27.904,-27.904,-27.904,-27.904,-27.904,-27.904,-27.904,20.048,20.048,20.048,20.048,-16.954,-16.954,-16.954,-16.954,-16.954,-16.954,-16.954,-16.954,-16.954,-3.8406,-3.8406,-3.8406,4.7392,4.7392,4.7392,4.7392,29.991,29.991,29.991,10.671,10.671,10.671,10.671,10.671,10.671,26.718,26.718,26.718,12.059,12.059,12.059,12.059,12.059,2.1621,2.1621,2.1621,2.1621,-17.473,-17.473,-17.473,14.499,-10.525,-10.525,-10.525,-10.525,-10.525,-9.4764,-9.4764,-9.4764,-9.4764,-9.4764,-9.4764,-9.4764,7.6179,7.6179,7.6179,7.6179,7.6179,7.6179,7.6179,7.6179,24.355,24.355,24.355,24.355,24.355,24.355,24.355,24.355,24.355,24.355,2.7641,2.7641,2.7641,2.7641,2.7641,2.7641,-25.376,-25.376,-25.376,-25.376,-25.376,-25.376,26.164,26.164,26.164,26.164,26.164,-12.635,15.184,15.184,15.184,15.184,10.366,10.366,10.366,10.366,-21.306,-21.306,-21.306,-21.306,-29.094,-29.094,-29.094,-29.094,-29.094,-6.4458,-6.4458,-6.4458,-6.4458,-6.4458,-6.4458,-6.4458,1.6021,1.6021,1.6021,1.6021,1.6021,-27.727,-27.727,-6.9304,-6.9304,-6.9304,-6.9304,17.516,17.516,17.516,17.516,17.516,-12.491,-12.491,-12.491,-12.491,-12.491,-12.491,-12.491,-12.491,-12.491,11.761,21.987,21.987,21.987,21.987,21.987,21.987,21.987,21.987,15.647,15.647,15.647,15.647,15.647,15.647,9.111,9.111,9.111,9.111,9.111,9.111,9.111,9.111,25.415,25.415,25.415,25.415,25.415,25.415,25.415,25.415,25.415,25.415,19.668,19.668,19.668,19.668,19.668,19.668,19.668,19.668,19.668,19.668,11.66,11.66,11.66,11.66,11.66,11.66,11.66,11.66,11.66,11.66,13.122,13.122,13.122,13.122,13.122,13.122,13.122,13.122,13.122,7.9256,7.9256,7.9256,7.9256,-0.5157,-0.5157,-2.5601,-2.5601,-7.5402,-7.5402,-7.5402,-7.5402,6.7761,3.812,3.812,3.812,3.812,11.49,11.49,11.49,-13.448,-13.448,-13.448,-13.448,-13.448,-13.448,20.575,20.575,20.575,20.575,20.575,20.575,20.575,20.575,1.7397,1.7397,1.7397,1.7397,1.7397,1.7397,1.7397,1.7397,20.557,20.557,20.557,20.557,20.557,20.557,20.557,20.557,20.557,20.557,2.1981,2.1981,2.1981,2.1981,2.1981,2.1981,2.1981,22.927,10.602
};

File tekst;
//EV3UARTSensor sensor(14,12);

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
// int dutyCycleL = 150;
// int dutyCycleD = 150;
float dutyCycle = 150;

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
          /*
         Serial.println("wifi begin0");
         delay(1000);
         Serial.println("pred povezavo");
         delay(100);
         WiFi.begin(ssid, password);
         Serial.println("wifi begin1");
            //Wait for connection 
         while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
         }
         Serial.println("");
         Serial.print("Connected to ");
         Serial.println(ssid);
         Serial.print("IP address: ");
         Serial.println(WiFi.localIP());
         // we use mDNS here http://kevin.local //
         if (MDNS.begin("kevin")) {
            Serial.println("MDNS responder started");
         }
         
         // register callback function when user request root "/" //
         server.on("/", handleRoot);
         server.onNotFound(handleNotFound);
         
         server.on("/File_Download", File_Download); 
         // start web server //
         server.begin();
         Serial.println("HTTP server started");
         */
         Serial.print("Test ce pride do sem");
         //sensor.begin();
         Serial.println("\n Test");

}

void loop() {
 int16_t enkoderL = 0;
 int16_t enkoderD = 0;
 int napaka=0;
 
 int start=0;
 int frekvenca=0;
 int cas=0;
 float dutyL=0;
 float dutyD=0;
 Serial.println(" ");
 tekst = SD.open("/tekst.csv", FILE_WRITE);
 Serial.println("odpru tekst.csv");   

if(tekst){
      start = millis();
      for(int index=0;index<(sizeof(prbs)/sizeof(prbs[0]));index++){
         
         dutyD=dutyCycle + prbs[index];
         dutyL=dutyCycle - prbs[index];
         
         //desni motor
         digitalWrite(MDP1, LOW);
         digitalWrite(MDP2, HIGH);
         ledcWrite(pwmChannelD, dutyD);
         //levi motor
         digitalWrite(MLP3, LOW);
         digitalWrite(MLP4, HIGH);
         ledcWrite(pwmChannelL, dutyL);
         pcnt_get_counter_value(PCNT_UNIT_0, &enkoderL);
         pcnt_get_counter_value(PCNT_UNIT_1, &enkoderD);
         napaka= enkoderL - enkoderD;
         cas = millis()-start;
         
                     //printf("encoder levi:%d \t encoder desni:%d \n", enkoderL,enkoderD);
                     //zacetek testa
                     tekst.print(cas);
                     tekst.print(',');
                     tekst.print(prbs[index]);
                     tekst.print(',');
                     tekst.print(napaka);
                     tekst.print(',');
                     tekst.println(index);
                     while(millis()-frekvenca<50){

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
         
         //server.handleClient();
         
                  
      }
  } else{
     Serial.println("error opening tekst.csv");
  }
}