#include <string.h>
#include <stdio.h>
#include <api_os.h>
#include <api_event.h>
#include <api_socket.h>
#include <api_network.h>
#include <api_debug.h>

#include "integrated_nav.h"
#include "mymath.h"
/*******************************************************************/
/////////////////////////socket configuration////////////////////////

#define DNS_DOMAIN  "120.78.167.211"
#define SERVER_PORT 10086
#define RECEIVE_BUFFER_MAX_LENGTH 200
/*******************************************************************/

static HANDLE socketTaskHandle = NULL;

int socketFd = -1;
uint8_t buffer[RECEIVE_BUFFER_MAX_LENGTH];
HANDLE sem = NULL;
int errorCode = 0;

void CreateSem(HANDLE* sem_)
{
    *sem_ = 0;
}

void WaitSem(HANDLE* sem_)
{
    while(*sem_ == 0)
        OS_Sleep(1);
    *sem_ = 0;
}

bool Connect()
{
    memset(buffer,0,sizeof(buffer));
    if(DNS_GetHostByName2(DNS_DOMAIN,(char*)buffer) != 0)
        return false;
    Trace(2,"DNS,domain:%s,ip:%s,strlen(ip):%d",DNS_DOMAIN,buffer,strlen(buffer));
    CreateSem(&sem);
    socketFd = Socket_TcpipConnect(TCP,buffer,SERVER_PORT);
    Trace(2,"connect tcp server,socketFd:%d",socketFd);
    WaitSem(&sem);
    Trace(2,"connect end");
    if(errorCode != 0)
    {
        errorCode = 0;
        Trace(2,"error ocurred");
        return false;
    }
    return true;
}
bool Write(uint8_t* data, uint16_t len)
{
    Trace(2,"Write");
    CreateSem(&sem);
    int ret = Socket_TcpipWrite(socketFd,data,len);
    if(ret <= 0)
    {
        Trace(2,"socket write fail:%d",ret);
        return false;
    }    
    Trace(2,"### socket %d send %d bytes data to server:%s,ret:%d",socketFd, len, data,ret);
    WaitSem(&sem);
    Trace(2,"### write end");
    if(errorCode != 0)
    {
        errorCode = 0;
        Trace(2,"error ocurred");
        return false;
    }
    return true;
}

bool Close()
{
    CreateSem(&sem);
    Socket_TcpipClose(socketFd);
    WaitSem(&sem);
    return true;
}

int data_packet(char *s, double lat, double lon, bool gps)
{
	char *lab1 = "lat:";
	char *lab2 = "lon:";
	char *lab3 = "gps:";
	char *p = s;
	memcpy(s, lab1, strlen(lab1));
	s += strlen(lab1);
	memcpy(s, my_ftoa(lat), 10);
	s += 10;
	*s++ = ',';
	memcpy(s, lab2, strlen(lab2));
	s += strlen(lab2);
	memcpy(s, my_ftoa(lon), 11);
	s += 11;
	*s++ = ',';
	memcpy(s, lab3, strlen(lab3));
	s += strlen(lab3);
	if(gps){
		*s++ = '1';
	}else{
		*s++ = '0';
	}
	*s++ = '\n';
	*s = '\0';
	return (int)(s - p);
}


void socketTestTask(void* param)
{
    int failCount = 0;
    int count = 0;
	char str[100];
	clock_t last_time = 0;
	
    WaitSem(&sem);
    Trace(2,"sem:%d,%p",(int)sem,(void*)sem);
    Trace(1,"start connect now");
    Connect();
    while(1)
    {
        if(failCount == 5)
        {
            Close();
        }
        if(failCount >= 5)
        {
            if(Connect())
                failCount = 0;
            else
                ++failCount;
        }
        else
        {
			// some bug in sprintf
			int8_t length = data_packet(str, local_pos.lat, local_pos.lon, local_pos.gps_valid);
			
			if(last_time != local_pos.timestamp){
				last_time = local_pos.timestamp;
				Trace(2, str);
				if(!Write(str, length))
	            {
	                ++failCount;
	                Trace(2,"write fail");
	            }
				Trace(2,"count:%d",count++);
			}
        }
        OS_Sleep(50);
    }
}

