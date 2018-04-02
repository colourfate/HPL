#ifndef _LED_H_
#define _LED_H_

#define LED_TASK_STACK_SIZE    (1024 * 1)		// 1KB
#define LED_TASK_PRIORITY      1
#define LED_TASK_NAME         "GPIO Test Task"

void led_testTask();

#endif
