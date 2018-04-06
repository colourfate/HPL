#include <api_hal_gpio.h>
#include <stdbool.h>
#include <api_debug.h>
#include <api_os.h>
#include <api_hal_pm.h>
#include <api_socket.h>
#include <stdio.h>
#include <api_hal_adc.h>

#include "led.h"
#include "integrated_nav.h"
#include "mymath.h"

extern struct location_position local_pos;
extern bool conn_status;

void led_testTask()
{
	GPIO_config_t led_gps = {
        .mode         = GPIO_MODE_OUTPUT,
		.pin = GPIO_PIN27,
        .defaultLevel = GPIO_LEVEL_LOW
    };
	GPIO_config_t led_net = {
        .mode         = GPIO_MODE_OUTPUT,
		.pin = GPIO_PIN28,
        .defaultLevel = GPIO_LEVEL_LOW
    };
	// open GPIO power
	PM_PowerEnable(POWER_TYPE_VPAD, true);
	// init GPIO27 and 28
	GPIO_Init(led_gps);
	GPIO_Init(led_net);
	// blink led when start up
	GPIO_SetLevel(led_gps, GPIO_LEVEL_HIGH);
	GPIO_SetLevel(led_net, GPIO_LEVEL_HIGH);
	OS_Sleep(1000);
	GPIO_SetLevel(led_gps, GPIO_LEVEL_LOW);
	GPIO_SetLevel(led_net, GPIO_LEVEL_LOW);

	// ADC
	uint16_t value = 0, mV = 0;
    ADC_Config_t config = {
        .channel = ADC_CHANNEL_0,
        .samplePeriod = ADC_SAMPLE_PERIOD_100MS
    };
    ADC_Init(config);
	bool low_power = false;

	while(1){
		
		if(!ADC_Read(ADC_CHANNEL_0, &value, &mV)){
			Trace(1, "ADC0 cannot read");
		}else{
			if(mV*3<3700){
				low_power = true;
			}else{
				low_power = false;
			}
		}
		if(low_power){
			GPIO_SetLevel(led_gps, GPIO_LEVEL_HIGH);
			GPIO_SetLevel(led_net, GPIO_LEVEL_HIGH);
			OS_Sleep(2000);
			GPIO_SetLevel(led_gps, GPIO_LEVEL_LOW);
			GPIO_SetLevel(led_net, GPIO_LEVEL_LOW);
		}else{
			if(local_pos.gps_valid){
				GPIO_SetLevel(led_gps, GPIO_LEVEL_HIGH);
				OS_Sleep(150);
				GPIO_SetLevel(led_gps, GPIO_LEVEL_LOW);
			}
			if(conn_status){
				GPIO_SetLevel(led_net, GPIO_LEVEL_HIGH);
			}else{
				GPIO_SetLevel(led_net, GPIO_LEVEL_LOW);
			}
		}
		OS_Sleep(2000);
	}
}

