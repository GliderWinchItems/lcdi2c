/******************************************************************************
* File Name          : LcdTask.c
* Date First Issued  : 04/05/2020
* Description        : LCD display 
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
#include "stm32f4xx_hal.h"
#include "LcdTask.h"
#include "lcd_hd44780_i2c.h"
#include "morse.h"
#include "LcdmsgsTask.h"
//#include "yprintf.h

struct LCDI2C_UNIT* punitd4x20 = NULL;
struct LCDI2C_UNIT* punitd4x16 = NULL;
struct LCDI2C_UNIT* punitd2x16 = NULL;

volatile uint8_t LcdTaskflag = 0;

extern I2C_HandleTypeDef hi2c1;

//osSemaphoreId vsnprintfSemaphoreHandle;
#define NOYPRINTFISPRESENT
#ifdef NOYPRINTFISPRESENT
SemaphoreHandle_t vsnprintfSemaphoreHandle;
#endif

enum LCD_STATE
{
	LCD_IDLE,
	LCD_SETRC,
	LCD_ROW,
	LCD_COL,
	LCD_CHR
};

#define DELAY_ROW 1; // Delay for row 
#define DELAY_COL 1; // Delay for col
#define DELAY_CHR 1; // Delay following each char
#define DELAY_SETRC 20; // Delay following set cursor row column

TaskHandle_t LcdTaskHandle = NULL;

/* Queue */
#define QUEUESIZE 32	// Total size of bcb's other tasks can queue up
QueueHandle_t LcdTaskQHandle;

/* Linked list head. */
static struct LCDI2C_UNIT* phdunit = NULL;   // Units instantiated; last = NULL
/* *************************************************************************
 * struct LCDI2C_UNIT* xLcdTaskcreateunit(I2C_HandleTypeDef* phi2c, 
 *    uint8_t address, 
 *    uint8_t numrow, 
 *    uint8_t numcol);
 *	@brief	: Instantiate a LCD unit on I2C peripheral and address
 * @param	: phi2c = pointer to I2C handle
 * @param	: address = I2C bus address
 * @param	: numrow = number of LCD rows
 * @param	: numcol = number of LCD columns
 * @return	: NULL = fail, otherwise pointer to unit on linked list
 * *************************************************************************/
struct LCDI2C_UNIT* xLcdTaskcreateunit(I2C_HandleTypeDef* phi2c, 
    uint8_t address, 
    uint8_t numrow, 
    uint8_t numcol)
{
	struct LCDI2C_UNIT* punit;
	struct LCDI2C_UNIT* ptmp;

taskENTER_CRITICAL();
	/* Check if this I2C bus & address (i.e. unit) is already present. */
	punit = phdunit; // Get pointer to first item on list
	if (punit == NULL)
	{ // Linked list is empty, so add first unit
		punit = (struct LCDI2C_UNIT*)calloc(1, sizeof(struct LCDI2C_UNIT));
		if (punit == NULL) morse_trap(230);
		phdunit = punit;  // Head points to first entry
//morse_trap(44);
	}
	else
	{ // Here, one or more on list.
		/* Search list for this I2C-address */
		while (punit != NULL)
		{
			if ((punit->phi2c == phi2c) && (punit->address == address))
			{ // Here this I2C-address is already on list.
				morse_trap(231); // ### ERROR: Duplicate ###
			}
			ptmp = punit;
			punit = punit->pnext;
		}
		/* Here, one or more is on list, but not this one. */
		punit = (struct LCDI2C_UNIT*)calloc(1, sizeof(struct LCDI2C_UNIT));
		if (punit == NULL) morse_trap(236);
		ptmp->pnext    = punit;  // Previous last entry points this new entry
	}

	/* Populate the control block for this unit. */
	punit->phi2c   = phi2c;   // HAL I2C Handle for this bus
	punit->address = address; // I2C bus address (not shifted)

	/* The remainder of LCDPARAMS will be initialized in the lcdInit() below. */
	punit->lcdparams.hi2c     = phi2c;
	punit->lcdparams.numrows  = numrow;  // number of LCD rows
	punit->lcdparams.numcols  = numcol;  // number of LCD columns
	punit->lcdparams.address  = address << 1; // Shifted address
   	punit->lcdparams.lines    = numrow;
   	punit->lcdparams.columns  = numcol;

	punit->state = LCD_IDLE; // Start off in idle (after final intialization)

taskEXIT_CRITICAL();

	/* Complete the initialization the LCD unit */ 
	struct LCDPARAMS* tmp = lcdInit(punit);
	if (tmp == NULL) morse_trap(237);

	return punit;
}
/* *************************************************************************
 * struct LCDTASK_LINEBUF* xLcdTaskintgetbuf(struct LCDI2C_UNIT* p, uint8_t bufmax);
 * @brief	: Get a buffer for a LCD unit
 * @param	: p = pointer to LCD unit struct (unit = I2C bus w address on bus)
 * @param   : bufmax = size of buffer allocated
 * @return	: NULL = fail, otherwise pointer to buffer struct
 * *************************************************************************/
struct LCDTASK_LINEBUF* xLcdTaskintgetbuf(struct LCDI2C_UNIT* p, uint8_t bufmax)
{
	struct LCDTASK_LINEBUF* plb = NULL;
	struct LCDI2C_UNIT* punit;
	
taskENTER_CRITICAL();
	if ( p == NULL) morse_trap(232); 

	/* Look up unit. */
	punit = phdunit;
	while (punit != NULL)
	{
		/* Check for I2C bus and address match. */
		if ((punit->phi2c == p->phi2c) && (punit->address == p->address))
		{ // Here found. Get struct for this LCD line buffer
			plb = (struct LCDTASK_LINEBUF*)calloc(1,sizeof(struct LCDTASK_LINEBUF));
			if (plb == NULL) morse_trap(233);

			// Pointer to lcd unit on linked list
			plb->punit = punit;

			/* Save max size of buffer allocated. */
			plb->bufmax = (punit->lcdparams.lines * punit->lcdparams.columns);

			// Get char buffer for up to max chars this unit displays. */
			plb->pbuf = (uint8_t*)calloc(plb->bufmax,sizeof(uint8_t));
			if (plb->pbuf == NULL) morse_trap(236);

			// Create buffer semaphore
			plb->semaphore = xSemaphoreCreateMutex(); // Semaphore for this buffer
			if (plb->semaphore == NULL) morse_trap(234);
			xSemaphoreGive(plb->semaphore); // Initialize

			taskEXIT_CRITICAL();
			return plb;
		}
		punit = punit->pnext;
	}
	/* Here: Likely this unit never got instantiated. */
	morse_trap(235);

taskEXIT_CRITICAL();
	return plb;
}
/* *************************************************************************
 * void StartLcdTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
static const uint8_t line1[20] = "1 123456789.........";
static const uint8_t line2[20] = "2 .........123456789";
static const uint8_t line3[20] = "3 abcdefgjhiklmnopqr";
static const uint8_t line4[20] = "4 ABCDEFGHIJKLMNOPQR";
void StartLcdTask(void* argument)
{
	BaseType_t Qret = 1;	// queue receive return
	struct LCDTASK_LINEBUF* plb; // Copied item from queue
	struct LCDPARAMS* p1;
	struct LCDI2C_UNIT* ptmp;

	/* Instantiate each LCD display unit. 
	   Arguments:
	     I2C bus handle
	     Address of unit on I2C bus
	     Number of rows (lines)
	     Number of columns
	*/
	punitd4x20 = xLcdTaskcreateunit(&hi2c1,0x27,4,20);
  	if (punitd4x20 == NULL) morse_trap(227);

  	punitd4x16 = xLcdTaskcreateunit(&hi2c1,0x26,4,16);
  	if (punitd4x16 == NULL) morse_trap(226);
  
  	punitd2x16 = xLcdTaskcreateunit(&hi2c1,0x25,2,16);
  	if (punitd2x16 == NULL) morse_trap(225);


  	/* Some test messages. */
#define DDD (pdMS_TO_TICKS(10)) // osDelay
  struct LCDPARAMS* pu1 = &punitd4x20->lcdparams;
    lcdSetCursorPosition(pu1,0, 0);
    lcdPrintStr(pu1,(uint8_t*)line1,20); osDelay(DDD);

    lcdSetCursorPosition(pu1,0, 1);
    lcdPrintStr(pu1,(uint8_t*)line2,20); osDelay(DDD);

    lcdSetCursorPosition(pu1,0, 2);
    lcdPrintStr(pu1,(uint8_t*)line3,20); osDelay(DDD);

    lcdSetCursorPosition(pu1,0, 3);
    lcdPrintStr(pu1,(uint8_t*)line4,20); osDelay(DDD);

    osDelay(pdMS_TO_TICKS(300));

  struct LCDPARAMS* pu2 = &punitd4x16->lcdparams;
    lcdSetCursorPosition(pu2,0, 0);
    lcdPrintStr(pu2,(uint8_t*)line1,16); osDelay(DDD);

    lcdSetCursorPosition(pu2,0, 1);
    lcdPrintStr(pu2,(uint8_t*)line2,16); osDelay(DDD);

    lcdSetCursorPosition(pu2,0, 2);
    lcdPrintStr(pu2,(uint8_t*)line3,16); osDelay(DDD);

    lcdSetCursorPosition(pu2,0, 3);
    lcdPrintStr(pu2,(uint8_t*)line4,16); osDelay(DDD);

    osDelay(pdMS_TO_TICKS(250));


  struct LCDPARAMS* pu3 = &punitd2x16->lcdparams;

    lcdSetCursorPosition(pu3,0, 0);
    lcdPrintStr(pu3,(uint8_t*)line1,16); osDelay(DDD);

    lcdSetCursorPosition(pu3,0, 1);
    lcdPrintStr(pu3,(uint8_t*)line2,16); osDelay(DDD);

    //osDelay(250);

 osDelay(pdMS_TO_TICKS(2000));
 
 // Clear all lines 
 uint32_t i;
 for (i = 0; i < 4; i++){ lcdSetCursorPosition(pu1,0, i);
  lcdPrintStr(pu1,(uint8_t*)"                    ",20);}

for (i = 0; i < 4; i++){ lcdSetCursorPosition(pu2,0, i);
  lcdPrintStr(pu2,(uint8_t*)"                ",16);}
   
for (i = 0; i < 2; i++){ lcdSetCursorPosition(pu3,0, i);
  lcdPrintStr(pu3,(uint8_t*)"                ",16);}

    /* Let LcdmsgsTask know that we are ready accept msgs. */
    LcdTaskflag = 1;

  /* Infinite loop */
  for(;;)
  {
 if (LcdTaskQHandle == NULL) morse_trap(229); // JIC debugging

	Qret = xQueueReceive( LcdTaskQHandle,&plb,portMAX_DELAY);
	if (Qret == pdPASS)
	{ // Here, OK item from queue
HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13); // Orange
		// Point to parameters for this unit
		ptmp = plb->punit;
		if (ptmp != NULL) // JIC
		{
			// Get LCDPARAMS pointer from lcd unit block
			p1 = &ptmp->lcdparams;
    
    		// Set cursor row/column for this hunit
			lcdSetCursorPosition(p1,plb->colreq,plb->linereq);

			// Send the glorious text of liberation! (Or, at least some ASCII)
  			lcdPrintStr(p1,plb->pbuf, plb->size);
		}
	}
  }
}
/* *************************************************************************
 * osThreadId xLcdTaskCreate(uint32_t taskpriority, uint16_t numbcb);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @param	: numbcb = number of buffer control blocks to allocate
 * @return	: LcdTaskHandle
 * *************************************************************************/
osThreadId xLcdTaskCreate(uint32_t taskpriority, uint16_t numbcb)
{
	/*
	BaseType_t xTaskCreate( TaskFunction_t pvTaskCode,
const char * const pcName,
unsigned short usStackDepth,
void *pvParameters,
UBaseType_t uxPriority,
TaskHandle_t *pxCreatedTask );
*/
	BaseType_t ret = xTaskCreate(&StartLcdTask,"LcdI2CTask",512,NULL,taskpriority,&LcdTaskHandle);
	if (ret != pdPASS) morse_trap(35);//return NULL;

	LcdTaskQHandle = xQueueCreate(numbcb, sizeof(struct LCDTASK_LINEBUF*) );
	if (LcdTaskQHandle == NULL) morse_trap(37);//return NULL;

#ifdef NOYPRINTFISPRESENT
	vsnprintfSemaphoreHandle = xSemaphoreCreateMutex(); // Semaphore for this buffer
	if (vsnprintfSemaphoreHandle == NULL) morse_trap(38);
#endif

	return LcdTaskHandle;
}
/* **************************************************************************************
 * int lcdprintf_init(void);
 * @brief	: 
 * @return	: 
 * ************************************************************************************** */
//{
//	yprintf_init();	// JIC not init'd
//	return;
//}
/* **************************************************************************************
 * int lcdi2cprintf(struct LCDTASK_LINEBUF** pplb, int row, int col, const char *fmt, ...);
 * @brief	: 'printf' for uarts
 * @param	: pblb = pointer to pointer to line buff struct
 * @param   : row = row number (0-n) to position cursor
 * @param   : col = column number (0-n) to position cursor
 * @param	: format = usual printf format
 * @param	: ... = usual printf arguments
 * @return	: Number of chars "printed"
 * ************************************************************************************** */
int lcdi2cprintf(struct LCDTASK_LINEBUF** pplb, int row, int col, const char *fmt, ...)
{
	struct LCDTASK_LINEBUF* plb = *pplb;
	va_list argp;

	/* Block if this buffer is not available. SerialSendTask will 'give' the semaphore 
      when the buffer has been sent. */
//	xSemaphoreTake(plb->semaphore, 5000);

	plb->linereq   = row;
	plb->colreq = col;

	/* Block if vsnprintf is being uses by someone else. */
	xSemaphoreTake( vsnprintfSemaphoreHandle, portMAX_DELAY );

	/* Construct line of data.  Stop filling buffer if it is full. */
	va_start(argp, fmt);
	plb->size = vsnprintf((char*)plb->pbuf,plb->bufmax, fmt, argp);
	va_end(argp);

	/* Limit byte count to be put on queue, from vsnprintf to max buffer sizes. */
	if (plb->size > plb->bufmax) 
			plb->size = plb->bufmax;

	/* Release semaphore controlling vsnprintf. */
	xSemaphoreGive( vsnprintfSemaphoreHandle );

	/* JIC */
	if (plb->size == 0) return 0;

	/* Place Buffer Control Block pointer on queue to LcdTask */
	xQueueSendToBack(LcdTaskQHandle, pplb, 0);

	return plb->size;
}
/* **************************************************************************************
 * int lcdi2cputs(struct LCDTASK_LINEBUF** pplb, int row, int col, char* pchr);
 * @brief	: Send zero terminated string to SerialTaskSend
 * @param	: pbcb = pointer to pointer to stuct with uart pointers and buffer parameters
 * @param   : row = row number (0-n) to position cursor
 * @param   : col = column number (0-n) to position cursor
 * @return	: Number of chars sent
 * ************************************************************************************** */
int lcdi2cputs(struct LCDTASK_LINEBUF** pplb, int row, int col, char* pchr)
{
	struct LCDTASK_LINEBUF* plb = *pplb;
	int sz = strlen(pchr); // Check length of input string
	if (sz == 0) return 0;

	/* Block (for a while) if this buffer is not yet available. 
      when the buffer has been sent. */
	xSemaphoreTake(plb->semaphore, 1024);

	// Save cursor position
	plb->linereq   = row; 	
	plb->colreq = col;

	strncpy((char*)plb->pbuf,pchr,plb->bufmax);	// Copy and limit size.

	/* Set size sent. */
	if (sz >= plb->bufmax)	// Did strcpy truncate?
		plb->size = plb->bufmax;	// Yes
	else
		plb->size = sz;	// No

	/* Place pointer to Buffer Control Block pointer on queue to LcdTask */
	xQueueSendToBack(LcdTaskQHandle, pplb, 0);
	return plb->size; 
}