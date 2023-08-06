/**********************************************************
filename: app.h
**********************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TASK_H__
#define __TASK_H__

#include "misc.h"

/* output variables for extern function --------------------------------------*/
void taskIrq(void);        //put in 1ms irq callback
void taskPolling(void);    //put in main loop

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
