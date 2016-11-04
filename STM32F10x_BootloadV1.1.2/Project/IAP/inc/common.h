/**
  ******************************************************************************
  * @file    IAP/inc/common.h
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    07/27/2009
  * @brief   This file provides all the headers of the common functions.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _COMMON_H
#define _COMMON_H

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "string.h"
#include "stm32f10x.h"
#include "ymodem.h"
#include "stm32_eval.h"

/* Exported types ------------------------------------------------------------*/
typedef  void (*pFunction)(void);

/* Exported constants --------------------------------------------------------*/
/* Constants used by Serial Command Line Mode */
#define CMD_STRING_SIZE       128

#define ApplicationAddress    0x8004000

#define FLASH_APP0_ADDR		0x08004000  	//第一个应用程序起始地址(存放在FLASH)
#define FLASH_APP1_ADDR		0x08024000  	//第一个应用程序起始地址(存放在FLASH)

#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址
#define STM32_FLASH_SIZE 512 	 		//所选STM32的FLASH容量大小(单位为K)



#define UINT8      unsigned char
#define UINT16     unsigned short
#define UINT32     unsigned long

#define u8      unsigned char
#define u16     unsigned short
#define u32     unsigned long

typedef char			CHAR;
typedef unsigned char	UCHAR;
typedef unsigned char	BYTE;

typedef unsigned short	WORD;

#define YES       0xaa
#define NO        0x55

#define LOBYTE(wValue)    ((BYTE)(wValue))
#define HIBYTE(wValue)    ((BYTE)((wValue) >> 8))


#ifdef STM32F10X_MD
 #define PAGE_SIZE                         (0x400)
 #define FLASH_SIZE                        (0x20000) /* 128K */
#elif defined STM32F10X_HD
 #define PAGE_SIZE                         (0x800)
 #define FLASH_SIZE                        (0x80000) /* 512K */
#elif defined STM32F10X_CL
 #define PAGE_SIZE                         (0x800)
 #define FLASH_SIZE                        (0x40000) /* 256K */
#else
 #error "Please select first the STM32 device to be used (in stm32f10x.h)"
#endif /* STM32F10X_MD */

/* Exported macro ------------------------------------------------------------*/
/* Common routines */
#define IS_AF(c)  ((c >= 'A') && (c <= 'F'))
#define IS_af(c)  ((c >= 'a') && (c <= 'f'))
#define IS_09(c)  ((c >= '0') && (c <= '9'))
#define ISVALIDHEX(c)  IS_AF(c) || IS_af(c) || IS_09(c)
#define ISVALIDDEC(c)  IS_09(c)
#define CONVERTDEC(c)  (c - '0')

#define CONVERTHEX_alpha(c)  (IS_AF(c) ? (c - 'A'+10) : (c - 'a'+10))
#define CONVERTHEX(c)   (IS_09(c) ? (c - '0') : CONVERTHEX_alpha(c))

#define SerialPutString(x) Serial_PutString((uint8_t*)(x))

/* Exported functions ------------------------------------------------------- */
void Int2Str(uint8_t* str,int32_t intnum);
uint32_t Str2Int(uint8_t *inputstr,int32_t *intnum);
uint32_t GetIntegerInput(int32_t * num);
uint32_t SerialKeyPressed(uint8_t *key);
uint8_t GetKey(void);
void SerialPutChar(uint8_t c);
void Serial_PutString(uint8_t *s);
void GetInputString(uint8_t * buffP);
uint32_t FLASH_PagesMask(__IO uint32_t Size);
void FLASH_DisableWriteProtectionPages(void);
void Main_Menu(void);
void SerialDownload(void);

#define BANK4_WRITE_START_ADDR  ((UINT32)0x0807E000)
#define BANK4_WRITE_END_ADDR    ((UINT32)0x0807E7FF)
#define BANK3_WRITE_START_ADDR  ((UINT32)0x0807E800)
#define BANK3_WRITE_END_ADDR    ((UINT32)0x0807EFFF)

#define APP_CONFIG_ADDR 		(BANK3_WRITE_START_ADDR + 0x000)	//配置地址,说明运行程序的选择
#define APP_CONFIG1_ADDR 		(BANK4_WRITE_START_ADDR + 0x000)	//配置地址,说明备份区的内容是否复制OK
#define APP_CONFIG2_ADDR 		(BANK4_WRITE_START_ADDR + 0x004)	//-记录备份区有效程序的大小,16位半字为单位

extern WORD    port_recv_pt[2];
extern WORD    port_recv_dl[2];
extern WORD    port_send_pt[2];
extern WORD    port_send_dl[2];
extern WORD    port_send_len[2];

extern unsigned char	port_recv[512];
extern unsigned char	port_send[512];
extern unsigned char	UART3_port_send[512];
extern BYTE	port_deal_buf[512];
extern BYTE	port_report[512];
extern BYTE	port_deal_flag[2];
extern BYTE	UART3_TO_UART2_FLAG;

extern WORD    recv_hex_lines;	//-记录当前接收到的hex文件的行号
extern WORD    expect_hex_lines;

#endif  /* _COMMON_H */

/*******************(C)COPYRIGHT 2009 STMicroelectronics *****END OF FILE******/
