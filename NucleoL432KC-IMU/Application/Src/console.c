#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include "appUSART.h"
#include "console.h"

static char lineBuffer[1024];
static uint8_t lineIndex = 0;

static void processLine(void);

static void processLine(void)
{
  switch(lineBuffer[0])
  {
    case 'B':
    	USART1TxStr(lineBuffer);
    	USART1TxStr("\r\n");
	    break;
    case 'E':
    	USART1TxStr(lineBuffer);
    	USART1TxStr("\r\n");
    	break;
    default:
    	break;
  }
}

void console(void)
{
  uint8_t data;

  for(;;)
  {
	USART1RxDataWait();
    while(USART1Rx(&data))
    {
      if((data=='\n')||(data=='\r'))
      {
        processLine();
        lineIndex=0;
        memset(lineBuffer,'\0', 1024);
      }
      else
      {
        lineBuffer[lineIndex]=data;
        lineIndex++;
      }
    }
    osDelay(1);
  }
}
