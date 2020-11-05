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

#define RELOAD_TMR      		1
#define PCNT_PULSE_GPIO1				32		// gpio for PCNT
#define PCNT_CONTROL_GPIO1			35
#define PCNT_PULSE_GPIO2				25		// gpio for PCNT
#define PCNT_CONTROL_GPIO2			33
//#define DIRECTION					25		// gpio for encoder direction input
// 
#define PCNT_H_LIM_VAL      32767 //int16
#define PCNT_L_LIM_VAL     -32767


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
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

}

void loop() {
  // put your main code here, to run repeatedly:
 
 int16_t count1 = 0;
 int16_t count2 = 0;

 while(1){

   pcnt_get_counter_value(PCNT_UNIT_0, &count1);
   pcnt_get_counter_value(PCNT_UNIT_1, &count2);
               printf("encoder levi:%d \t encoder desni:%d \n", count1,count2);
 }


}