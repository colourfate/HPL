#include <string.h>
#include <stdio.h>
#include <api_os.h>
#include <api_gps.h>
#include <api_event.h>
#include <api_hal_uart.h>
#include <api_debug.h>
#include "buffer.h"
#include "gps_parse.h"
#include "integrated_nav.h"

#include <api_socket.h>
#include <api_network.h>

/*********************************************************************/
/////////////////////////socket configuration////////////////////////
// edit ip address and port
// (you can get ip and port from online tcp debug tool: http://tt.ai-thinker.com:8000/ttcloud)
#define SERVER_IP   "122.114.122.174"
#define SERVER_PORT 33780

#define DNS_DOMAIN  "www.neucrack.com"
#define RECEIVE_BUFFER_MAX_LENGTH 200
/*********************************************************************/
// socket
int socketFd = -1;
int socketFd2 = -1;
uint8_t buffer[RECEIVE_BUFFER_MAX_LENGTH];
int receivedDataCount = -1;
/*********************************************************************/
// GPS
#define GPS_TASK_NAME "read GPS data"

#define GPS_DATA_BUFFER_MAX_LENGTH 2048

Buffer_t gpsNmeaBuffer;
uint8_t  gpsDataBuffer[GPS_DATA_BUFFER_MAX_LENGTH];
uint8_t tmp[1024];

/****************************** APP ***********************************/
// APP
#define MAIN_TASK_STACK_SIZE    (2048 * 2)
#define MAIN_TASK_PRIORITY      0
#define MAIN_TASK_NAME          "Send GPS to server"

static HANDLE mainTaskHandle = NULL;

static char gps_data[100] = {0};

bool gps_ready = false;

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
            //Start DNS test
            memset(buffer,0,sizeof(buffer));
            DNS_Status_t dnsRet = DNS_GetHostByName(DNS_DOMAIN,buffer);
            if(dnsRet == DNS_STATUS_OK)
            {
                Trace(2,"DNS get ip address from domain success(return),domain:%s,ip:%s",DNS_DOMAIN,buffer);
            }
            else if(dnsRet == DNS_STATUS_ERROR)
            {
                Trace(2,"DNS get ip address error(return)!!!");
            }
            //Start connect tcp server
            socketFd = Socket_TcpipConnect(TCP,SERVER_IP,SERVER_PORT);
            Trace(2,"connect tcp server,socketFd:%d",socketFd);
            break;

        case API_EVENT_ID_SOCKET_CONNECTED:
            Socket_TcpipWrite(pEvent->param1,"hello...test string\n",strlen("hello...test string\n"));
            Trace(2,"socket %d send %d bytes data to server:%s",pEvent->param1, strlen("hello...test string\n"),"hello...test string\n");
			// set the socket status to ready
			//socket_status.ready = 1;
			break;

        case API_EVENT_ID_SOCKET_SENT:
        {
            int fd = pEvent->param1;
            Trace(2,"socket %d send data complete",fd);
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
			socketFd = -1;
            break;
        }
        case API_EVENT_ID_SOCKET_ERROR:
        {
            int fd = pEvent->param1;
            Trace(2,"socket %d error occurred,cause:%d",fd,pEvent->param2);
			socketFd = -1;
            break;
        }
        case API_EVENT_ID_DNS_SUCCESS:
            Trace(2,"DNS get ip address from domain success(event),domain:%s,ip:%s",pEvent->pParam1,pEvent->pParam2);
            break;

        case API_EVENT_ID_DNS_ERROR:
            Trace(2,"DNS get ip address error(event)!!!");
			socketFd = -1;
            break;

		// GPS event
		case API_EVENT_ID_GPS_UART_RECEIVED:
            // Trace(1,"received GPS data,length:%d, data:%s,flag:%d",pEvent->param1,pEvent->pParam1,flag);
            if(flag)
            {
                Buffer_Puts(&gpsNmeaBuffer,pEvent->pParam1,pEvent->param1);
                GpsUpdate();
				gps_ready = true;
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

// socket init
void Init()
{
    receivedDataCount = 0;
}

void gps_testTask(void *pData)
{
    GPS_Information_t* gpsInfo = Gps_GetInfo();
    uint8_t strTmp[100];
	uint8_t length = 0;

    while(1)
    {
        //UART_Write(UART1,tmp,strlen(tmp));
        memset(strTmp,0,sizeof(strTmp));
        Trace(1,"GPS fix:%d, BDS fix:%d, Latitude:%s, Longitude:%s",
			gpsInfo->fixGPS, gpsInfo->fixBDS, gpsInfo->latitude, gpsInfo->longitude);
		
		// socketFd is set in API_EVENT_ID_NETWORK_ACTIVATED
		if(socketFd != -1){
			length = sprintf(gps_data, "GPS fix:%d, BDS fix:%d, Latitude:%s, Longitude:%s",
				gpsInfo->fixGPS, gpsInfo->fixBDS, gpsInfo->latitude, gpsInfo->longitude);
			Socket_TcpipWrite(socketFd, gps_data, length);
			Trace(1, "send gps data to server\n");
		}
		OS_Sleep(5000);
    }
}

void app_MainTask(void *pData)
{
	API_Event_t* event=NULL;

	/**************************************************************/
	// init socket
	Init();
	// wait socket to connect
	while(socketFd == -1)
    {
        if(OS_WaitEvent(mainTaskHandle, (void**)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            EventDispatch(event);
            OS_Free(event->pParam1);
            OS_Free(event->pParam2);
            OS_Free(event);
        }
    }
	/**************************************************************/
	// init GPS
    GPS_Open(NULL);
	/*
    UART_Config_t config = {
        .baudRate = UART_BAUD_RATE_115200,
        .dataBits = UART_DATA_BITS_8,
        .stopBits = UART_STOP_BITS_1,
        .parity   = UART_PARITY_NONE,
        .rxCallback = NULL,
        .useEvent   = true
    };
    UART_Init(UART1,config);
    */
    Buffer_Init(&gpsNmeaBuffer,gpsDataBuffer,GPS_DATA_BUFFER_MAX_LENGTH);
	// send data
    OS_CreateTask(gps_testTask,
            NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, GPS_TASK_NAME);
	// init AHRS
	JY901_init();
	// start data fuse
	OS_CreateTask(position_estimator_testTask,
            NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, FUSE_TASK_NAME);
	
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

