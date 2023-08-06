/**********************************************************
filename: config.h
**********************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

// should be define in other file, such as board.h
#define USING_RTOS  0
#define USING_G0    1

#if USING_RTOS
#include "cmsis_os.h"
#endif

#if USING_F0
#include "stm32f0xx_hal.h"
#endif

#if USING_F1
#include "stm32f1xx_hal.h"
#endif

#if USING_G0
#include "stm32g0xx_hal.h"
#endif

#if USING_F3
#include "stm32f3xx_hal.h"
#endif

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
