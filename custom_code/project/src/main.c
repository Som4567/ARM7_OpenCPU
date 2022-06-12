/*
 * main.c
 *
 */

#ifdef __CAN_PROJECT__
#include "ql_trace.h"
#include "ql_system.h"
#include "ql_gpio.h"
#include "ql_stdlib.h"
#include "ql_error.h"
#include "ql_uart.h"
#include "mcp2515.h"

#define DEBUG_ENABLE 1
#if DEBUG_ENABLE > 0
#define DEBUG_PORT  UART_PORT1
#define DBG_BUF_LEN   512
static char DBG_BUFFER[DBG_BUF_LEN];
#define APP_DEBUG(FORMAT,...) {\
    Ql_memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    Ql_sprintf(DBG_BUFFER,FORMAT,##__VA_ARGS__); \
    if (UART_PORT2 == (DEBUG_PORT)) \
    {\
        Ql_Debug_Trace(DBG_BUFFER);\
    } else {\
        Ql_UART_Write((Enum_SerialPort)(DEBUG_PORT), (u8*)(DBG_BUFFER), Ql_strlen((const char *)(DBG_BUFFER)));\
    }\
}
#else
#define APP_DEBUG(FORMAT,...)
#endif


static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara)
{

}

struct can_frame canMsg;
s32 ret;

/************************************************************************/
/* The entrance for this example application                            */
/************************************************************************/
void proc_main_task(s32 taskId)
{
    ST_MSG msg;

    // Register & open UART port
    ret = Ql_UART_Register(UART_PORT1, CallBack_UART_Hdlr, NULL);
    if (ret < QL_RET_OK)
    {
        Ql_Debug_Trace("Fail to register serial port[%d], ret=%d\r\n", UART_PORT1, ret);
    }
    ret = Ql_UART_Open(UART_PORT1, 115200, FC_NONE);
    if (ret < QL_RET_OK)
    {
        Ql_Debug_Trace("Fail to open serial port[%d], ret=%d\r\n", UART_PORT1, ret);
    }




    APP_DEBUG("\r\n<-- before 5 seconds sleep -->\r\n");
    Ql_GPIO_Init(PINNAME_PCM_SYNC,PINDIRECTION_OUT,PINLEVEL_HIGH,PINPULLSEL_PULLUP);   //CS high
//    Ql_GPIO_Init(PINNAME_PCM_IN,PINDIRECTION_OUT,PINLEVEL_HIGH,PINPULLSEL_PULLUP);   //CS high
//    Ql_GPIO_Init(PINNAME_PCM_SYNC,PINDIRECTION_OUT,PINLEVEL_HIGH,PINPULLSEL_PULLUP);   //CS high
//    Ql_GPIO_Init(PINNAME_PCM_OUT,PINDIRECTION_OUT,PINLEVEL_HIGH,PINPULLSEL_PULLUP);   //CS high

    Ql_Sleep(50000);

    APP_DEBUG("\r\n<-- after 5 seconds sleep -->\r\n");
    ret =-1;
    ret = init(0);
    if(ret <0)
           {
               APP_DEBUG("\r\n<-- In  main Failed!! Ql_SPI_Init fail, ret =%d-->\r\n",ret);
           }
           else
           {
               APP_DEBUG("\r\n<-- In main Ql_SPI_Init ret =%d -->\r\n",ret);
           }
    Ql_Sleep(100);

    APP_DEBUG("reset ret %d\n",reset());
    APP_DEBUG("setBitrate ret %d\n",setBitrate(CAN_500KBPS));
    APP_DEBUG("setListenOnlyMode ret %d\n",setListenOnlyMode());

    APP_DEBUG("------- CAN Read ----------\r\n");
    APP_DEBUG("ID  DLC   DATA\r\n");
    // Start message loop of this task
    while (TRUE)
    {
//        APP_DEBUG("------- CAN Read in while loop ----------\r\n");
//        Ql_Sleep(100);
//        APP_DEBUG(" readMessage ret %d \r\n",readMessage(&canMsg));
//        Ql_Sleep(100);
          ret = readMessage(&canMsg);
    	  if (ret == ERROR_OK) {
    		Ql_Debug_Trace("Inside If main loop \r\n");
    	    APP_DEBUG(" %X\t", canMsg.can_id); // print ID
    	    APP_DEBUG(" %X\t",canMsg.can_dlc); // print DLC

    	    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
    	      APP_DEBUG("%X\t",canMsg.data[i]);
    	    }

    	    APP_DEBUG("\r\n");
    	  }
    	  else
    	  {
            APP_DEBUG(" CAN Read Error ret %d \r\n",ret);
    	    APP_DEBUG(" %X\t", canMsg.can_id); // print ID
    	  }
       Ql_Sleep(100);

//        Ql_OS_GetMessage(&msg);
//        switch(msg.message)
//        {
//        case MSG_ID_USER_START:
//            break;
//        default:
//            break;
//        }
    }
}

#endif //  __CAN_PROJECT__

