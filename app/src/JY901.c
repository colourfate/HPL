#include <api_os.h>
#include <api_hal_uart.h>
#include <api_debug.h>

#include "JY901.h"

struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ       stcQ;

bool acc_ready = false, q_ready = false;

void move_buffer(uint8_t *buffer, uint8_t len)
{
	uint8_t i;

	for(i=0; i<len; i++){
		buffer[i] = buffer[i+len];
		buffer[i+len] = 0;
	}
}

// A 8 Bytes buffer is set in the SDK. When the buffer is full, the following function
// is called. If the data is more than 8 Bytes, the function will be called multiple times,
// handling 8 Bytes at a time.
void OnUart1ReceivedData(UART_Callback_Param_t param)
{
	static uint8_t ucRxBuffer[250];
	static uint8_t ucRxCnt = 0;
	uint8_t temp[20];
	uint8_t i = 0;

	// read the data, timeout 10 ms.
	memset(temp, 0, sizeof(temp));
	uint32_t len = UART_Read(UART1, temp, param.length, 10);
	// copy the data to buffer
	memcpy(ucRxBuffer+ucRxCnt, temp, len);
	ucRxCnt += len;
	// if the data header is not correct, discard current frame
	if(ucRxBuffer[0] != 0x55){
		do{
			// the header is not in buffer, count clean the buffer
			if(++i > ucRxCnt){
				memset(ucRxBuffer, 0, strlen(ucRxBuffer));
				ucRxCnt = 0;
				return;
			}
		} while(ucRxBuffer[i] != 0x55);
		// the current data count
		ucRxCnt -= i;
		move_buffer(ucRxBuffer, i);
	}
	// if buffer length is less than 11 Byte, return.
	if(ucRxCnt < 11){
		return;
	}else{
		switch(ucRxBuffer[1])
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;
			case 0x51:	
				memcpy(&stcAcc,&ucRxBuffer[2],8);
				acc_ready = true;
				break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
			case 0x59:	
				memcpy(&stcQ,&ucRxBuffer[2],8);
				q_ready = true;
				break;
		}
		ucRxCnt -= 11;
		// move the rest of data to the buffer header.
		move_buffer(ucRxBuffer, ucRxCnt);
	}
	
}


void JY901_init(void)
{
	UART_Config_t config = {
        .baudRate = UART_BAUD_RATE_9600,
        .dataBits = UART_DATA_BITS_8,
        .stopBits = UART_STOP_BITS_1,
        .parity   = UART_PARITY_NONE,
        .rxCallback = OnUart1ReceivedData,
    };
    UART_Init(UART1,config);
}
