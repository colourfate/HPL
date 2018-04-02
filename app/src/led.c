#include "api_hal_gpio.h"
#include "stdbool.h"
#include "api_debug.h"
#include "api_os.h"
#include "api_hal_pm.h"
#include <api_socket.h>
#include <stdio.h>

#include "led.h"
#include "integrated_nav.h"
#include "mymath.h"

extern int socketFd;
extern struct location_position local_pos;

void led_testTask()
{
	GPIO_config_t led_config = {
        .mode         = GPIO_MODE_OUTPUT,
        .defaultLevel = GPIO_LEVEL_LOW
    };
	// open GPIO power
	PM_PowerEnable(POWER_TYPE_VPAD, true);
	// init GPIO27 and 28
	led_config.pin = GPIO_PIN27;
	GPIO_Init(led_config);
	led_config.pin = GPIO_PIN28;
	GPIO_Init(led_config);
	char str[100] = {0};
	clock_t last_time = 0;

	while(1){
		led_config.pin = GPIO_PIN27;
		GPIO_SetLevel(led_config, GPIO_LEVEL_HIGH);
		OS_Sleep(500);
		GPIO_SetLevel(led_config, GPIO_LEVEL_LOW);
		OS_Sleep(500);

		if(socketFd != -1 && last_time != local_pos.timestamp){
			last_time = local_pos.timestamp;
			int8_t length = sprintf(str, "Latitude:%s, Longitude:%s, gps valid:%d", 
				my_ftoa(local_pos.lat), my_ftoa(local_pos.lon), (int)(local_pos.gps_valid));
			Socket_TcpipWrite(socketFd, str, length);
			Trace(3, "send gps data to server");
		}else{
			Trace(3, "can't get socket fd");
		}
	}
}

