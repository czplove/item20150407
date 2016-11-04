#ifdef GLOBALS 
#define EXT
#else
#define EXT extern 
#endif

#include  <stdio.h>
#include  <string.h>
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_i2c.h"

#define Led_ON()   GPIO_SetBits(GPIOA, GPIO_Pin_8);  	    
#define Led_OFF()  GPIO_ResetBits(GPIOA, GPIO_Pin_8); 	      




EXT unsigned char fac_id,dev_id;
EXT unsigned char SST25_buffer[4096];
EXT uint32_t Mass_Memory_Size[2];
EXT uint32_t Mass_Block_Size[2];
EXT uint32_t Mass_Block_Count[2];



