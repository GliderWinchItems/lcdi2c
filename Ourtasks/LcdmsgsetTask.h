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
	void (*ptr)(union LCDSETVAR u);
	union LCDSETVAR u;
};


/* *************************************************************************/
osMessageQId LcdmsgsetTaskt_init(uint16_t qsize);
/*	@brief	: Setup the queue for pointers
 * @return	: NULL = failed; pointer to queue handle
 * *************************************************************************/
void lcdmsgset_poll(void);
/*	@brief	: 
 * *************************************************************************/

extern osMessageQId lcdmsgsetQHandle;

#endif
