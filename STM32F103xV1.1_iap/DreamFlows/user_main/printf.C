#include "stm32f10x.h"
#include "integer.h"
#include "user_conf.h"
#include <stdio.h>
#include "printf.h"




PUTCHAR_PROTOTYPE  
{
  
  /* Place your implementation of fputc here */  
  /* e.g. write a character to the USART */  
  //-USART_SendData(COM, (uint8_t) ch);  
  //-EEP_Data++;
  /* Loop until the end of transmission */  
  //-while(USART_GetFlagStatus(COM, USART_FLAG_TC) == RESET)  
  //-{  
  //-}  

  return ch;  
} 


//-直接可以使用C库printf函数
//-printf("\r\nirq_flag is %d\n",irq_flag);

//-需要在 IAR的Options -> General Options ->Library Configuration里设置一下函数库，
//-不然printf函数不对，将Library Configuration 中的Library 设置由"Normal"改为"Full"就可以了
//-需要包含#include <stdio.h>头文件
//-对于STM32在IAR中有了上面的操作就可以使用库函数了.


