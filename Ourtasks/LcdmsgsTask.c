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
volatile uint16_t LcdmsgsTaskflag; // 0 = routine not ready; 1 = ready

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
	while(LcdTaskflag == 0) osDelay(10);

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

	/* Let other's know msging is ready. */
	LcdmsgsTaskflag = 1;

//while(1==1) osDelay(10);

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
							lcdi2cprintf(&pu01,row,col,"1 loopct: %0.1f ",msgreq.var.f);
							xQueueSendToBack(LcdTaskQHandle, &pu01, 0);
						    break;

						case 1: // Unit #0 Message #2
							pu02->linereq = row;   pu02->colreq  = col;
							strncpy((char*)pu02->pbuf,"2 This is msg2$       ",20);
							pu02->size = 20;
							xQueueSendToBack(LcdTaskQHandle, &pu02, 0);
						    break;

						case 2: // Unit #0 Message #3
//							pu03->linereq = row;   pu03->colreq  = col;
//							pu03->size = 16;
//							strncpy((char*)pu03->pbuf,"3 Messages galore      ", 17);
							lcdi2cputs(&pu03,row,col,"3 Msgs via puts");
				
				//			strncpy((char*)pu03->pbuf,"SMOKE A LUCKY AND ? ",20);
				//			pu03->size = 20;
							xQueueSendToBack(LcdTaskQHandle, &pu03, 0);
						    break;


						default: // Message number not programmed
							morse_trap(758);
							break;
					}
					break;

				default: // Unit number not programmed
					morse_trap(759);
					break;
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
	BaseType_t ret = xTaskCreate(&StartLcdmsgsTask,"LcdI2CTask",512,NULL,taskpriority,&LcdmsgsTaskHandle);
	if (ret != pdPASS) morse_trap(35);//return NULL;

	LcdmsgsTaskQHandle = xQueueCreate(numbcb, sizeof(struct LCDMSGTASK_MSGREQ) );
	if (LcdmsgsTaskQHandle == NULL) morse_trap(37); //return NULL;


	return LcdmsgsTaskHandle;
}