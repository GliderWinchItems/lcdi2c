/******************************************************************************
* File Name          : LcdTask.h
* Date First Issued  : 04/05/2020
* Description        : LCD display 
*******************************************************************************/

#ifndef __LCDTASK
#define __LCDTASK

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "semphr.h"

/* I2C Line buffer */
struct LCDTASK_LINEBUF
{
	struct LCDI2C_UNIT* punit;  // Point to linked list entry for this I2C:address unit
	SemaphoreHandle_t semaphore;// Semaphore handle
	uint8_t* pbuf;              // Pointer to byte buffer to be sent
	uint8_t bufmax;             // Size of char buffer
	 int8_t size;               // Number of bytes to be sent
    uint8_t linereq;            // Line number requested(0 - n)
    uint8_t colreq;             // Column number requested(0 - n)
};

struct LCDPARAMS 
{
    I2C_HandleTypeDef * hi2c;  // I2C Struct
    uint8_t lines;             // Lines of the display
    uint8_t columns;           // Columns
    uint8_t address;           // I2C address =>shifted<= left by 1
    uint8_t backlight;         // Backlight
    uint8_t modeBits;          // Display on/off control bits
    uint8_t entryBits;         // Entry mode set bits
 	uint8_t lcdCommandBuffer[8];
    uint8_t numrows;  // Number of rows (lines) for this LCD unit
    uint8_t numcols;  // Number of columns for this LCD unit

};

/* Linked list of entries for each I2C:address (i.e. LCD units) */
struct LCDI2C_UNIT
{
	struct LCDI2C_UNIT* pnext; // Next bus unit on this I2C bus
	I2C_HandleTypeDef* phi2c;  // HAL I2C Handle for this bus
	struct LCDPARAMS lcdparams;
	TickType_t untiltickct; // Tickcount for the end of a delay
	uint8_t address;  // =>Not-shifted<= address
	uint8_t state;    // State machine
};

/* *************************************************************************/
 void vStartLcdTask(void);
/*	@brief	: Task startup
 * *************************************************************************/
osThreadId xLcdTaskCreate(uint32_t taskpriority, uint16_t numbcb);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @param	: numbcb = number of buffer control blocks to allocate for queue
 * @return	: LcdTaskHandle
 * *************************************************************************/
struct LCDI2C_UNIT* xLcdTaskcreateunit(I2C_HandleTypeDef* phi2c, 
    uint8_t address,
    uint8_t numrow,
    uint8_t numcol);
/*	@brief	: Instantiate a LCD unit on I2C peripheral and address
 * @param	: phi2c = pointer to I2C handle
 * @param	: address = I2C bus address
 * @param	: numrow = number of LCD rows
 * @param	: numcol = number of LCD columns
 * @return	: NULL = fail, otherwise pointer to unit on linked list
 * *************************************************************************/
 struct LCDTASK_LINEBUF* xLcdTaskintgetbuf(struct LCDI2C_UNIT* p, uint8_t bufmax);
 /* @brief	: Get a buffer for a LCD unit
  * @param	: p = pointer to LCD unit struct (unit = I2C bus w address on bus)
  * @param   : bufmax = size of buffer allocated
  * @return	: NULL = fail, otherwise pointer to buffer struct
  * *************************************************************************/
int lcdi2cprintf(struct LCDTASK_LINEBUF** pplb, int row, int col, const char *fmt, ...);
/* @brief	: 'printf' for uarts
 * @param	: pblb = pointer to pointer to line buff struct
 * @param   : row = row number (0-n) to position cursor
 * @param   : col = column number (0-n) to position cursor
 * @param	: format = usual printf format
 * @param	: ... = usual printf arguments
 * @return	: Number of chars "printed"
 * ************************************************************************************** */
int lcdi2cputs(struct LCDTASK_LINEBUF** pplb, int row, int col, char* pchr);
/* @brief	: Send zero terminated string to SerialTaskSend
 * @param	: pbcb = pointer to pointer to stuct with uart pointers and buffer parameters
 * @param   : row = row number (0-n) to position cursor
 * @param   : col = column number (0-n) to position cursor
 * @return	: Number of chars sent
 * ************************************************************************************** */

extern QueueHandle_t LcdTaskQHandle;
extern TaskHandle_t   LcdTaskHandle;

extern struct LCDI2C_UNIT* punitd4x20;
extern struct LCDI2C_UNIT* punitd4x16;
extern struct LCDI2C_UNIT* punitd2x16;

extern volatile uint8_t LcdTaskflag;

#endif
