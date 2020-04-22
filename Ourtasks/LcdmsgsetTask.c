/******************************************************************************
* File Name          : LcdmsgsetTask.c
* Date First Issued  : 04/22/2020
* Description        : lcdi2c printf calling
*******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "LcdmsgsetTask.h"
#include "morse.h"

osMessageQId LcdmsgsetTaskQHandle;
TaskHandle_t LcdmsgsetTaskHandle;

/* *************************************************************************
 * osMessageQId LcdmsgsetTask_init(uint16_t qsize);
 *	@brief	: Setup the queue for pointers
 * @return	: NULL = failed; pointer to queue handle
 * *************************************************************************/
 osMessageQId LcdmsgsetTask_init(uint16_t qsize)
{
	LcdmsgsetTaskQHandle = xQueueCreate(qsize, sizeof(struct LCDMSGSET) );
	if (LcdmsgsetTaskQHandle == NULL) return NULL;
	return LcdmsgsetTaskQHandle;
}


/* *************************************************************************
 * void StartLcdmsgsetTask(void* argument);
 *	@brief	: Task startup
 * *************************************************************************/
static struct LCDMSGSET lsv;
void StartLcdmsgsetTask(void* argument)
{
	BaseType_t ret;

	ret = xQueueReceive(LcdmsgsetTaskQHandle,&lsv,0);
	if (ret == errQUEUE_EMPTY) return;

		if (lsv.ptr != NULL) // jic a NULL ptr got on the queue
		  (*lsv.ptr)(lsv.u);	// Go do something

	return;
}
 /* *************************************************************************
 * osThreadId xLcdmsgsTaskCreate(uint32_t taskpriority, uint16_t numbcb);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @param	: numbcb = number of message requests allowed in queue
 * @return	: LcdmsgsTaskHandle
 * *************************************************************************/
osThreadId xLcdmsgsetTaskCreate(uint32_t taskpriority, uint16_t numbcb)
{
	BaseType_t 	ret = xTaskCreate(&StartLcdmsgsetTask,"LcdmsgsetTask",\
		512,NULL,taskpriority,&LcdmsgsetTaskHandle);
	if (ret != pdPASS) morse_trap(401);//return NULL;

	LcdmsgsetTaskQHandle = xQueueCreate(numbcb, sizeof(struct LCDMSGSET) );
	if (LcdmsgsetTaskQHandle == NULL) morse_trap(37); //return NULL;


	return LcdmsgsetTaskHandle;
}
