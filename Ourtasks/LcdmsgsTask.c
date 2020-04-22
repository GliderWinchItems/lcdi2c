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
	// Wait until LcdTask has completed instantiation of LCD units
	while (punitd4x20 == NULL) osDelay(2);

	// Four one line buffers for 4x20 unit 
	struct LCDTASK_LINEBUF* pu01 = xLcdTaskintgetbuf(punitd4x20,20);
	struct LCDTASK_LINEBUF* pu02 = xLcdTaskintgetbuf(punitd4x20,20);
	struct LCDTASK_LINEBUF* pu03 = xLcdTaskintgetbuf(punitd4x20,20);
	struct LCDTASK_LINEBUF* pu04 = xLcdTaskintgetbuf(punitd4x20,20);
	if (pu01 == NULL) morse_trap(701);
	if (pu02 == NULL) morse_trap(702);
	if (pu03 == NULL) morse_trap(703);
	if (pu04 == NULL) morse_trap(704);

	// Four one line buffers for 4x16 unit 
	struct LCDTASK_LINEBUF* pu11 = xLcdTaskintgetbuf(punitd4x16,16);
	struct LCDTASK_LINEBUF* pu12 = xLcdTaskintgetbuf(punitd4x16,16);
	struct LCDTASK_LINEBUF* pu13 = xLcdTaskintgetbuf(punitd4x16,16);
	struct LCDTASK_LINEBUF* pu14 = xLcdTaskintgetbuf(punitd4x16,16);
	if (pu11 == NULL) morse_trap(711);
	if (pu12 == NULL) morse_trap(712);
	if (pu13 == NULL) morse_trap(713);
	if (pu14 == NULL) morse_trap(714);

	// Two one line buffers for 2x16 unit 
	struct LCDTASK_LINEBUF* pu21 = xLcdTaskintgetbuf(punitd2x16,16);
	struct LCDTASK_LINEBUF* pu22 = xLcdTaskintgetbuf(punitd2x16,16);
	struct LCDTASK_LINEBUF* pu23 = xLcdTaskintgetbuf(punitd2x16,16);
	struct LCDTASK_LINEBUF* pu24 = xLcdTaskintgetbuf(punitd2x16,16);
	if (pu21 == NULL) morse_trap(721);
	if (pu22 == NULL) morse_trap(722);
	if (pu23 == NULL) morse_trap(723);
	if (pu24 == NULL) morse_trap(724);


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
				/* ------------ 4x20 unit ---------------------------------- */

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
							lcdi2cputs(&pu03,row,col,"3 Msgs via puts");				
							xQueueSendToBack(LcdTaskQHandle, &pu03, 0);
						    break;

						default: // Message number not programmed
							morse_trap(758);
							break;
					}
					break;
				/* ------------ 4x16 unit ---------------------------------- */
				case 1: // 4x16 LCD
					switch (msgreq.msgnum)
					{
						case 0: // Unit #1 Message #1
							break;

						default: // Message number not programmed
							morse_trap(760);
							break;
					}
					break;
				/* ------------ 2x16 unit ---------------------------------- */
				case 2: // 2x16 LCD
					switch (msgreq.msgnum)
					{
						case 0: // Unit #2 Message #1
							break;

						default: // Message number not programmed
							morse_trap(761);
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
	BaseType_t ret = xTaskCreate(&StartLcdmsgsTask,"Lcd,sgsTask",512,NULL,taskpriority,&LcdmsgsTaskHandle);
	if (ret != pdPASS) morse_trap(35);//return NULL;

	LcdmsgsTaskQHandle = xQueueCreate(numbcb, sizeof(struct LCDMSGTASK_MSGREQ) );
	if (LcdmsgsTaskQHandle == NULL) morse_trap(37); //return NULL;


	return LcdmsgsTaskHandle;
}