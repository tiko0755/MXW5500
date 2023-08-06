/******************** (C) COPYRIGHT 2015 TIKO **********************************
* File Name          : task.c
* Author             : Tiko Zhong
* Date First Issued  : 11/18/2023
* Description        : 
*                      
********************************************************************************
* History:
* AGU05,2023: V0.0    
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/

#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include "task.h"
#include "board.h"
#include "led_flash.h"

/* Public variables ---------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static void forwardRes(void* rsrc, const char* MSG);
static void forwardReq(const char* MSG);
static u8 doCommand(char* buff, u16 len,
        void (*xprint)(const char* FORMAT_ORG, ...),
        void (*forward)(const char* MSG));
        
/* Private function prototypes -----------------------------------------------*/
static u16 task_tick;

void taskIrq(void){
    u8 i;
    for(i=0;i<APP_TIMER_COUNT;i++){
        tmr[i].isr(&tmr[i].rsrc, 1);
    }
    task_tick++;
}

/**
  * @brief polling task
  * @param none
  * @retval None
  */
void taskPolling(void){
    if(g_initalDone == 0)    return;
    
    // poll to send out 
    console.TxPolling(&console.rsrc);
    rs485.TxPolling(&rs485.rsrc);
    
    u8 i;
    for(i=0;i<APP_TIMER_COUNT;i++){
        tmr[i].polling(&tmr[i].rsrc);
    }

    if(task_tick >= 4){
        console.RxPolling(&console.rsrc);
        rs485.RxPolling(&rs485.rsrc);
        task_tick = 0;
    }
}

/**
  * @brief tack per 16ms
  * @param none
  * @retval None
  */
static void commandFormat(char* buff, u16 len){
    u16 i;
    for(i=0;i<len;i++){
        if(buff[i] == 0)    break;
        if(buff[i] == '(' || buff[i] == ')' || buff[i] == ',')    buff[i] = ' ';
        if(buff[i] >= 'A' && buff[i] <= 'Z')    buff[i] += 32;
    }
}

u8 doCommand(char* buff, u16 len,
        void (*xprint)(const char* FORMAT_ORG, ...),
        void (*forward)(const char* MSG)){
    s32 i;
    char *CMD;
    if(buff==NULL)    return 0;
    //message from uart
    if(sscanf(buff, "%d.", &i) <= 0){
        if(strncmp(buff, "help", strlen("help")) == 0){    
            xprint("%s +ok@%d.help()\r\n", COMMON_HELP, g_boardAddr);
        }
        else if(strncmp(buff, "about", strlen("about")) == 0){
            xprint("+ok@about(%d,\"%s\")\r\n", g_boardAddr, ABOUT);
            led_flash_answer(100, 30);
        }
//        else if(promise[0].Cmd(&promise[0].rsrc, buff)){}
//        else if(promise[1].Cmd(&promise[1].rsrc, buff)){}
//        else if(promise[2].Cmd(&promise[2].rsrc, buff)){}
//        else if(promise[3].Cmd(&promise[3].rsrc, buff)){}
        else{    xprint("+unknown@%s", buff);    }
    }
    else if(sscanf(buff, "%d.", &i)==1){
        if(i == g_boardAddr){
            commandFormat(buff, len);
            CMD = (char*)buff + strlen(g_addrPre);
            if(brdCmd(CMD, xprint)){    }
            else{        xprint("+unknown@%s", buff);    }
        }
        else{
//            if(promise[0].Cmd(&promise[0].rsrc, buff)){}
//            else if(promise[1].Cmd(&promise[1].rsrc, buff)){}
//            else if(promise[2].Cmd(&promise[2].rsrc, buff)){}
//            else if(promise[3].Cmd(&promise[3].rsrc, buff)){}
//            else if(forward){    forward((char*)buff);    }
        }
    }
    return 1;
}


static void forwardRes(void* rsrc, const char* MSG){
    if(strncmp(MSG, "+$timeout@req", strlen("+$timeout@req")) == 0){
        print("host: %s", MSG);
    }
    else{    printS(MSG);        }
}

static void forwardReq(const char* MSG){
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
