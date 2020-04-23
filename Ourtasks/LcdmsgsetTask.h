/******************************************************************************
* File Name          : LcdmsgsetTask.h
* Date First Issued  : 04/22/2020
* Description        : lcdi2c printf calling
*******************************************************************************/

#ifndef __LCDMSGSETTASK
#define __LCDMSGSETTASK

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

union LCDSETVAR
{
	float f;
	uint32_t u32;
	 int32_t s32;
	uint16_t u16[2];
	 int16_t s16[2];
	uint8_t  u8[4];
	 int8_t  s8[4];
};

struct LCDMSGSET
{
	union LCDSETVAR u;
	void (*ptr)(union LCDSETVAR u);
};

 /* *************************************************************************/
 osThreadId xLcdmsgsetTaskCreate(uint32_t taskpriority, uint16_t numbcb);
 /* @brief	: Create task; task handle created is global for all to enjoy!
  * @param	: taskpriority = Task priority (just as it says!)
  * @param	: numbcb = number of message requests allowed in queue
  * @return	: LcdmsgsTaskHandle
  * *************************************************************************/

extern osMessageQId LcdmsgsetTaskQHandle;
extern TaskHandle_t LcdmsgsetTaskHandle;

#endif
