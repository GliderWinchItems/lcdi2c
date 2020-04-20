/******************************************************************************
* File Name          : LcdmsgsTask.c
* Date First Issued  : 04/19/2020
* Description        : LCD I2C "printf" LCD msgs by number
*******************************************************************************/

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "LcdTask.h"
#include "LcdmsgsTask.h"
#include "morse.h"
//#include "yprintf.h

TaskHandle_t LcdmsgsTaskHandle = NULL;
QueueHandle_t LcdmsgsTaskQHandle;

/* *************************************************************************
 * void StartLcdTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartLcdmsgsTask(void* argument)
{
	BaseType_t Qret;	// queue receive return
	struct LCDMSGTASK_MSGREQ msgreq;
	uint8_t	unitnum;
	//uint8_t blink;
	uint8_t row;
	uint8_t col;


	/* Wait for LcdTask to complete the initialization and test display. */
	while(LcdTaskflag == 0);

/*
 * struct LCDTASK_LINEBUF* xLcdTaskintgetbuf(struct LCDI2C_UNIT* p);
*  @brief	: Get a buffer for a LCD unit
 * @param	: p = pointer to LCD unit struct (unit = I2C bus w address on bus)
 * @return	: NULL = fail, otherwise pointer to buffer struct
*/
 /* Get one line buffers for LCD units. */
	// Wait until LcdTask has completed instantiation of LCD unit.
	while (punitd4x20 == NULL) osDelay(2);
	// Four one line buffers for 4x20 unit 
	struct LCDTASK_LINEBUF* pu01 = xLcdTaskintgetbuf(punitd4x20,20);
	struct LCDTASK_LINEBUF* pu02 = xLcdTaskintgetbuf(punitd4x20,20);
	struct LCDTASK_LINEBUF* pu03 = xLcdTaskintgetbuf(punitd4x20,20);
	struct LCDTASK_LINEBUF* pu04 = xLcdTaskintgetbuf(punitd4x20,20);
	if (pu01 == NULL) morse_trap(751);
	if (pu02 == NULL) morse_trap(752);
	if (pu03 == NULL) morse_trap(753);
	if (pu04 == NULL) morse_trap(754);

	for ( ;; )
	{
		Qret = xQueueReceive(LcdmsgsTaskQHandle,&msgreq,portMAX_DELAY);
		if (Qret == pdPASS)
		{

			/* Unpack queued item. */
			unitnum = msgreq.row >> 4;
			row     = msgreq.row & 0x0f;
//			blink   = msgreq.col >> 4;
			col     = msgreq.col & 0x0f;

			switch (unitnum)
			{
				case 0: // 4x20 LCD #1
					switch (msgreq.msgnum)
					{
						case 0: // Unit #0 Message #1
HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14); // Red
							pu01->linereq = row;   pu01->colreq  = col;
							strncpy((char*)pu01->pbuf,"Happiness!          ",20);
							pu01->size = 20;
							xQueueSendToBack(LcdTaskQHandle, &pu01, 0);
						    break;

						case 1: // Unit #0 Message #2
							pu02->linereq = row;   pu02->colreq  = col;
							strncpy((char*)pu02->pbuf,"Is for this to work ",20);
							xQueueSendToBack(LcdTaskQHandle, &pu02, 0);
						    break;
						 

					}
			}

		}	
	}
}
 /* *************************************************************************
 * osThreadId xLcdmsgsTaskCreate(uint32_t taskpriority, uint16_t numbcb);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @param	: numbcb = number of message requests allowed in queue
 * @return	: LcdmsgsTaskHandle
 * *************************************************************************/
osThreadId xLcdmsgsTaskCreate(uint32_t taskpriority, uint16_t numbcb)
{
	BaseType_t ret = xTaskCreate(&StartLcdmsgsTask,"LcdI2CTask",256,NULL,taskpriority,&LcdmsgsTaskHandle);
	if (ret != pdPASS) morse_trap(35);//return NULL;

	LcdmsgsTaskQHandle = xQueueCreate(numbcb, sizeof(struct LCDMSGTASK_MSGREQ) );
	if (LcdmsgsTaskQHandle == NULL) morse_trap(37); //return NULL;


	return LcdmsgsTaskHandle;
}