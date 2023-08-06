/**********************************************************
filename: board.h
**********************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include "misc.h"
#include "rs485Dev.h"
#include "uartdev.h"
#include "app_timer.h"
#include "cmd_consumer.h"

/* output variables for extern function --------------------------------------*/

extern const char ABOUT[];
extern const char COMMON_HELP[];

// gloable var
extern char g_addrPre[4];
extern u8 g_boardAddr;
extern u8 g_baudHost;
extern u8 g_baud485;
extern u32 g_errorCode;
extern u8 g_initalDone;

extern UartDev_t console;
extern Rs485Dev_t rs485;

#define APP_TIMER_COUNT (8)
extern appTmrDev_t tmr[APP_TIMER_COUNT];
extern cmdConsumerDev_t cmdConsumer;

extern const PIN_T DE;
extern const PIN_T LAN_CS;
extern const PIN_T LAN_IRQ;
extern const PIN_T RUNNING;


// gloable method
void boardPreInit(void);
void boardInit(void);
u8 brdCmd(const char* CMD, void (*xprint)(const char* FORMAT_ORG, ...));

void print(const char* FORMAT_ORG, ...);
void printS(const char* MSG);
void print485(const char* FORMAT_ORG, ...);
void printS485(const char* STRING);
void printSUDP(const char* STRING);
void printUDP(const char* FORMAT_ORG, ...);

//s8 ioWrite(u16 addr, const u8 *pDat, u16 nBytes);
//s8 ioRead(u16 addr, u8 *pDat, u16 nBytes);
s8 ioWriteReg(u16 addr, s32 val);
s8 ioReadReg(u16 addr, s32 * val);
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
