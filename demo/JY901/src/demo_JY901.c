// The JY901 demo through i2c
#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"

#include "api_os.h"
#include "api_debug.h"
#include "api_event.h"
#include "api_hal_i2c.h"
#include "JY901_i2c.h"



#define MAIN_TASK_STACK_SIZE    (2048 * 2)
#define MAIN_TASK_PRIORITY      0
#define MAIN_TASK_NAME          "Main Test Task"

#define SECOND_TASK_STACK_SIZE    (2048 * 2)
#define SECOND_TASK_PRIORITY      1
#define SECOND_TASK_NAME          "Second Test Task"

static HANDLE mainTaskHandle = NULL;
static HANDLE secondTaskHandle = NULL;
#define I2C_ACC I2C2


void EventDispatch(API_Event_t* pEvent)
{
    switch(pEvent->id)
    {
        case API_EVENT_ID_NO_SIMCARD:
            Trace(10,"!!NO SIM CARD%d!!!!",pEvent->param1);
            break;

        case API_EVENT_ID_SYSTEM_READY:
            Trace(1,"system initialize complete");
            break;

        case API_EVENT_ID_NETWORK_REGISTERED_HOME:
        case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
            Trace(2,"network register success");
            break;

        default:
            break;
    }
}
void SecondTask(void *pData)
{
    uint8_t accId;
    I2C_Config_t config;

    config.freq = I2C_FREQ_100K;
    I2C_Init(I2C_ACC, config);

    while(1)
    {
		struct SAcc std_acc;
		memset(&std_acc, 0, sizeof(std_acc));
		JY901_read_acc(&std_acc);
		Trace(1, "ACC: %d %d %d",std_acc.a[0], std_acc.a[1], std_acc.a[2]);
		
		struct SGyro std_gyro;
		memset(&std_gyro, 0, sizeof(std_gyro));
		JY901_read_gyro(&std_gyro);
		Trace(1, "GYRO: %d %d %d",std_gyro.w[0], std_gyro.w[1], std_gyro.w[2]);
		struct SMag std_mag;
		
		memset(&std_mag, 0, sizeof(std_mag));
		JY901_read_mag(&std_mag);
		Trace(1, "MAG: %d %d %d",std_mag.h[0], std_mag.h[1], std_mag.h[2]);
		
		struct SAngle std_angle;
		memset(&std_angle, 0, sizeof(std_angle));
		JY901_read_angle(&std_angle);
		Trace(1, "ANGLE: %d %d %d",std_angle.Angle[0], std_angle.Angle[1], std_angle.Angle[2]);
		
		struct SQ std_q;
		memset(&std_q, 0, sizeof(std_q));
		JY901_read_q(&std_q);
		Trace(1, "Q: %d %d %d %d",std_q.q[0], std_q.q[1], std_q.q[2], std_q.q[3]);
        OS_Sleep(1000);
    }
}

void MainTask(void *pData)
{
    API_Event_t* event=NULL;

    secondTaskHandle = OS_CreateTask(SecondTask,
        NULL, NULL, SECOND_TASK_STACK_SIZE, SECOND_TASK_PRIORITY, 0, 0, SECOND_TASK_NAME);

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

void JY901_Main(void)
{
    mainTaskHandle = OS_CreateTask(MainTask,
        NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
    OS_SetUserMainHandle(&mainTaskHandle);
}

