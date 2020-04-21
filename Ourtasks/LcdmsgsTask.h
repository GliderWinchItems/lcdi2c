/******************************************************************************
* File Name          : LcdmsgsTask.h
* Date First Issued  : 04/19/2020
* Description        : LCD I2C "printf" LCD msgs by number
*******************************************************************************/

#ifndef __LCDMSGSTASK
#define __LCDMSGSTASK

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "LcdTask.h"

union LCDVAR
{
	float f;
	uint32_t u32;
	 int32_t s32;
	uint16_t u16[2];
	 int16_t s16[2];
	uint8_t  u8[4];
	 int8_t  s8[4];
};

struct LCDMSGTASK_MSGREQ
{
	union LCDVAR var;
	uint8_t msgnum;  // Message number (0 -n)
	uint8_t row;     // Unit number (0 - m)|row (0 - n)
	uint8_t col;     // Blink (0-1)|column number to start (0-n)
	uint8_t spare;

};

/* *************************************************************************/
 osThreadId xLcdmsgsTaskCreate(uint32_t taskpriority, uint16_t numbcb);
 /* @brief	: Create task; task handle created is global for all to enjoy!
  * @param	: taskpriority = Task priority (just as it says!)
  * @param	: numbcb = number of message requests allowed in queue
  * @return	: LcdmsgsTaskHandle
  * *************************************************************************/

extern QueueHandle_t LcdmsgsTaskQHandle;
extern volatile uint16_t LcdmsgsTaskflag; // 0 = routine not ready; 1 = ready

#endif