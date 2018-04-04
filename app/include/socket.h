#ifndef _SOCKET_H_
#define _SOCKET_H_

#define SOCKET_TASK_STACK_SIZE    (2048 * 2)
#define SOCKET_TASK_PRIORITY      1
#define SOCKET_TASK_NAME          "Socket Test Task"
#define RECEIVE_BUFFER_MAX_LENGTH 200

extern HANDLE sem;
extern int errorCode;
extern int socketFd;
extern uint8_t buffer[RECEIVE_BUFFER_MAX_LENGTH];
void CreateSem(HANDLE* sem_);
void socketTestTask(void* param);
#endif