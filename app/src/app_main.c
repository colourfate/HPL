#include <string.h>
#include <stdio.h>
#include <time.h>

#include <api_os.h>
#include <api_gps.h>
#include <api_event.h>
#include <api_hal_uart.h>
#include <api_debug.h>
#include <api_socket.h>
#include <api_network.h>
#include <api_hal_i2c.h>

#include "buffer.h"
#include "gps_parse.h"
#include "integrated_nav.h"
#include "JY901_i2c.h"
#include "led.h"
#include "socket.h"

/*********************************************************************/
// GPS
#define GPS_DATA_BUFFER_MAX_LENGTH 2048

Buffer_t gpsNmeaBuffer;
uint8_t  gpsDataBuffer[GPS_DATA_BUFFER_MAX_LENGTH];
uint8_t tmp[1024];

/****************************** APP ***********************************/
// APP
#define MAIN_TASK_STACK_SIZE    (2048 * 2)
#define MAIN_TASK_PRIORITY      1
#define MAIN_TASK_NAME          "Send GPS to server"

static HANDLE mainTaskHandle = NULL;

bool gps_ready = false;
bool conn_status = false;

/**********************************************************************/

void GpsUpdate()
{
    int32_t index = Buffer_Query(&gpsNmeaBuffer,"$GNVTG",strlen("$GNVTG"),Buffer_StartPostion(&gpsNmeaBuffer));
    if(index >= 0)
    {
        // Trace(1,"find $GNVTG");
        index = Buffer_Query(&gpsNmeaBuffer,"\r\n",strlen("\r\n"),index);
        if(index >= 0)
        {
            Trace(1,"find complete GPS frame");
            
            memset(tmp,0,sizeof(tmp));
            uint32_t len = Buffer_Size2(&gpsNmeaBuffer,index)+1;
            Trace(1,"frame len:%d",len);
            if(!Buffer_Gets(&gpsNmeaBuffer,tmp,len))
            {
                Trace(1,"get data from buffer fail");
                return;
            }
            GPS_Parse(tmp);
			gps_ready = true;
			Trace(1, "clock=%d", (int)(clock() / CLOCKS_PER_MSEC));
        }
    }
}


void EventDispatch(API_Event_t* pEvent)
{
	static uint8_t flag = 0; // GPS
    switch(pEvent->id)
    {	
		// socket event
        case API_EVENT_ID_NO_SIMCARD:
            Trace(10,"!!NO SIM CARD%d!!!!",pEvent->param1);
            break;

        case API_EVENT_ID_NETWORK_REGISTERED_HOME:
        case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
            Trace(2,"network register success");
			// the GPS may cause the two events
			if(socketFd == -1){
				Network_StartAttach();
			}
			flag = 1;
            break;

        case API_EVENT_ID_NETWORK_ATTACHED:
            Trace(2,"network attach success");
            Network_PDP_Context_t context = {
                .apn        ="cmnet",
                .userName   = ""    ,
                .userPasswd = ""
            };
            Network_StartActive(context);
            break;

        case API_EVENT_ID_NETWORK_ACTIVATED:
            Trace(2,"network activate success");
            sem = 1;
            break;

        case API_EVENT_ID_SOCKET_CONNECTED:
			conn_status = true;
            sem = 1;
            break;
        case API_EVENT_ID_SOCKET_SENT:
        {
            sem = 1;
            break;
        }
        case API_EVENT_ID_SOCKET_RECEIVED:
        {
            int fd = pEvent->param1;
            int length = pEvent->param2>RECEIVE_BUFFER_MAX_LENGTH?RECEIVE_BUFFER_MAX_LENGTH:pEvent->param2;
            memset(buffer,0,sizeof(buffer));
            length = Socket_TcpipRead(fd,buffer,length);
            Trace(2,"socket %d received %d bytes data:%s",fd,length,buffer);
            break;
        }
        case API_EVENT_ID_SOCKET_CLOSED:
        {
            int fd = pEvent->param1;
            Trace(2,"socket %d closed",fd);
            sem = 1;
			conn_status = false;
            break;
        }
        case API_EVENT_ID_SOCKET_ERROR:
        {
            int fd = pEvent->param1;
            Trace(2,"socket %d error occurred,cause:%d",fd,pEvent->param2);
            errorCode = pEvent->param2;
            sem = 1;
			conn_status = false;
            break;
        }
		// GPS event
		case API_EVENT_ID_GPS_UART_RECEIVED:
            // Trace(1,"received GPS data,length:%d, data:%s,flag:%d",pEvent->param1,pEvent->pParam1,flag);
            if(flag)
            {
                Buffer_Puts(&gpsNmeaBuffer,pEvent->pParam1,pEvent->param1);
                GpsUpdate();
            }
            break;
        case API_EVENT_ID_UART_RECEIVED:
            if(pEvent->param1 == UART1)
            {
                uint8_t data[pEvent->param2+1];
                data[pEvent->param2] = 0;
                memcpy(data,pEvent->pParam1,pEvent->param2);
                Trace(1,"uart received data,length:%d,data:%s",pEvent->param2,data);
            }
            break;
		
        default:
            break;
    }
}

/*
#define SLEEP_TASK_STACK_SIZE (1024*10)
#define SLEEP_TASK_PRIORITY	(0)
#define SLEEP_TASK_NAME	"sleep"

void sleep_testTask(void *pData)
{
	while(1){
		Trace(3, "10");
		OS_Sleep(50);
	}
}
*/

void app_MainTask(void *pData)
{
	API_Event_t* event=NULL;

	/**************************************************************/
	// init socket
	CreateSem(&sem);
    OS_CreateTask(socketTestTask,
        NULL, NULL, SOCKET_TASK_STACK_SIZE, SOCKET_TASK_PRIORITY, 0, 0, SOCKET_TASK_NAME);
	/**************************************************************/
	// init GPS
    GPS_Open(NULL);
    Buffer_Init(&gpsNmeaBuffer,gpsDataBuffer,GPS_DATA_BUFFER_MAX_LENGTH);
	/**************************************************************/
	// init i2c
	I2C_Config_t config;
    config.freq = I2C_FREQ_100K;
    I2C_Init(I2C_JY901, config);
	// start position estimation
	OS_CreateTask(position_estimator_testTask,
           NULL, NULL, INAV_TASK_STACK_SIZE, INAV_TASK_PRIORITY, 0, 0, INAV_TASK_NAME);
	//OS_CreateTask(sleep_testTask,
    //       NULL, NULL, SLEEP_TASK_STACK_SIZE, SLEEP_TASK_PRIORITY, 0, 0, SLEEP_TASK_NAME);
	/***************************************************************/
	// start led task
	OS_CreateTask(led_testTask,
            NULL, NULL, LED_TASK_STACK_SIZE, LED_TASK_PRIORITY, 0, 0, LED_TASK_NAME);
	// handle event
    while(1)
    {
        if(OS_WaitEvent(mainTaskHandle, (void**)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            EventDispatch(event);
            OS_Free(event->pParam1);
            OS_Free(event->pParam2);
            OS_Free(event);
        }
    }
}



// The name of entry fun must be 'xxx_MainTask', the 'xxx' is fold name
void app_Main(void)
{
	mainTaskHandle = OS_CreateTask(app_MainTask,
        NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
	// Listen the event of Task. A Main fun has only a Handle, named UserHandle
    OS_SetUserMainHandle(&mainTaskHandle);
}

