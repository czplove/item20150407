/**
  ******************************************************************************
  * @file    stm32_eval.c
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    06/19/2009
  * @brief   This file provides firmware functions to manage Leds, push-buttons
  *          and COM ports available on STM32 Evaluation Boards from STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32_eval.h"

#include "common.h"

/** @addtogroup Utilities
  * @{
  */

/** @defgroup STM32_EVAL
  * @brief This file provides firmware functions to manage Leds, push-buttons
  *        and COM ports available on STM32 Evaluation Boards from STMicroelectronics.
  * @{
  */

/** @defgroup STM32_EVAL_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup STM32_EVAL_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup STM32_EVAL_Private_Macros
  * @{
  */
/**
  * @}
  */


/** @defgroup STM32_EVAL_Private_Variables
  * @{
  */

#define ADC1_DR_Address    ((u32)0x4001244C)
#define USART1_DR_Address    ((u32)0x40013804)
#define USART2_DR_Address    ((u32)0x40004404)
#define USART3_DR_Address    ((u32)0x40004804)


GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT, LED3_GPIO_PORT,
                                 LED4_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED1_GPIO_PIN, LED2_GPIO_PIN, LED3_GPIO_PIN,
                                 LED4_GPIO_PIN};
const uint32_t GPIO_CLK[LEDn] = {LED1_GPIO_CLK, LED2_GPIO_CLK, LED3_GPIO_CLK,
                                 LED4_GPIO_CLK};

#ifdef USE_STM3210C_EVAL
 GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {WAKEUP_BUTTON_PORT, TAMPER_BUTTON_PORT,
                                       KEY_BUTTON_PORT};

 const uint16_t BUTTON_PIN[BUTTONn] = {WAKEUP_BUTTON_PIN, TAMPER_BUTTON_PIN,
                                       KEY_BUTTON_PIN};

 const uint32_t BUTTON_CLK[BUTTONn] = {WAKEUP_BUTTON_CLK, TAMPER_BUTTON_CLK,
                                       KEY_BUTTON_CLK};

 const uint16_t BUTTON_EXTI_LINE[BUTTONn] = {WAKEUP_BUTTON_EXTI_LINE,
                                             TAMPER_BUTTON_EXTI_LINE,
                                             KEY_BUTTON_EXTI_LINE};

 const uint16_t BUTTON_PORT_SOURCE[BUTTONn] = {WAKEUP_BUTTON_PORT_SOURCE,
                                               TAMPER_BUTTON_PORT_SOURCE,
                                               KEY_BUTTON_PORT_SOURCE};

 const uint16_t BUTTON_PIN_SOURCE[BUTTONn] = {WAKEUP_BUTTON_PIN_SOURCE,
                                              TAMPER_BUTTON_PIN_SOURCE,
                                              KEY_BUTTON_PIN_SOURCE};
 const uint16_t BUTTON_IRQn[BUTTONn] = {WAKEUP_BUTTON_IRQn, TAMPER_BUTTON_IRQn,
                                        KEY_BUTTON_IRQn};

 USART_TypeDef* COM_USART[COMn] = {EVAL_COM1};

 GPIO_TypeDef* COM_PORT[COMn] = {EVAL_COM1_GPIO};

 const uint32_t COM_USART_CLK[COMn] = {EVAL_COM1_CLK};

 const uint32_t COM_POR_CLK[COMn] = {EVAL_COM1_GPIO_CLK};

 const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TxPin};

 const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RxPin};

#else
 GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {WAKEUP_BUTTON_PORT, TAMPER_BUTTON_PORT,
                                       KEY_BUTTON_PORT, RIGHT_BUTTON_PORT,
                                       LEFT_BUTTON_PORT, UP_BUTTON_PORT,
                                       DOWN_BUTTON_PORT, SEL_BUTTON_PORT};

 const uint16_t BUTTON_PIN[BUTTONn] = {WAKEUP_BUTTON_PIN, TAMPER_BUTTON_PIN,
                                       KEY_BUTTON_PIN, RIGHT_BUTTON_PIN,
                                       LEFT_BUTTON_PIN, UP_BUTTON_PIN,
                                       DOWN_BUTTON_PIN, SEL_BUTTON_PIN};

 const uint32_t BUTTON_CLK[BUTTONn] = {WAKEUP_BUTTON_CLK, TAMPER_BUTTON_CLK,
                                       KEY_BUTTON_CLK, RIGHT_BUTTON_CLK,
                                       LEFT_BUTTON_CLK, UP_BUTTON_CLK,
                                       DOWN_BUTTON_CLK, SEL_BUTTON_CLK};

 const uint16_t BUTTON_EXTI_LINE[BUTTONn] = {WAKEUP_BUTTON_EXTI_LINE,
                                             TAMPER_BUTTON_EXTI_LINE,
                                             KEY_BUTTON_EXTI_LINE,
                                             RIGHT_BUTTON_EXTI_LINE,
                                             LEFT_BUTTON_EXTI_LINE,
                                             UP_BUTTON_EXTI_LINE,
                                             DOWN_BUTTON_EXTI_LINE,
                                             SEL_BUTTON_EXTI_LINE};

 const uint16_t BUTTON_PORT_SOURCE[BUTTONn] = {WAKEUP_BUTTON_PORT_SOURCE,
                                               TAMPER_BUTTON_PORT_SOURCE,
                                               KEY_BUTTON_PORT_SOURCE,
                                               RIGHT_BUTTON_PORT_SOURCE,
                                               LEFT_BUTTON_PORT_SOURCE,
                                               UP_BUTTON_PORT_SOURCE,
                                               DOWN_BUTTON_PORT_SOURCE,
                                               SEL_BUTTON_PORT_SOURCE};

 const uint16_t BUTTON_PIN_SOURCE[BUTTONn] = {WAKEUP_BUTTON_PIN_SOURCE,
                                              TAMPER_BUTTON_PIN_SOURCE,
                                              KEY_BUTTON_PIN_SOURCE,
                                              RIGHT_BUTTON_PIN_SOURCE,
                                              LEFT_BUTTON_PIN_SOURCE,
                                              UP_BUTTON_PIN_SOURCE,
                                              DOWN_BUTTON_PIN_SOURCE,
                                              SEL_BUTTON_PIN_SOURCE};

 const uint16_t BUTTON_IRQn[BUTTONn] = {WAKEUP_BUTTON_IRQn, TAMPER_BUTTON_IRQn,
                                        KEY_BUTTON_IRQn, RIGHT_BUTTON_IRQn,
                                        LEFT_BUTTON_IRQn, UP_BUTTON_IRQn,
                                        DOWN_BUTTON_IRQn, SEL_BUTTON_IRQn};

 USART_TypeDef* COM_USART[COMn] = {EVAL_COM1, EVAL_COM2};

 GPIO_TypeDef* COM_PORT[COMn] = {EVAL_COM1_GPIO, EVAL_COM2_GPIO};

 const uint32_t COM_USART_CLK[COMn] = {EVAL_COM1_CLK, EVAL_COM2_CLK};

 const uint32_t COM_POR_CLK[COMn] = {EVAL_COM1_GPIO_CLK, EVAL_COM2_GPIO_CLK};

 const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TxPin, EVAL_COM2_TxPin};

 const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RxPin, EVAL_COM2_RxPin};

#endif
/**
  * @}
  */


/** @defgroup STM32_EVAL_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */


/** @defgroup STM32_EVAL_Private_Functions
  * @{
  */

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void STM_EVAL_LEDInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(GPIO_CLK[Led], ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void STM_EVAL_LEDOn(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BSRR = GPIO_PIN[Led];
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void STM_EVAL_LEDOff(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BRR = GPIO_PIN[Led];
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void STM_EVAL_LEDToggle(Led_TypeDef Led)
{
  GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];
}


#define FLASH_BASE            ((uint32_t)0x08000000) /*!< FLASH base address in the alias region */
#define VECT_TAB_OFFSET  0x0 /*!< Vector Table base offset field. This value must be a multiple of 0x200. */

/**
 * @brief In the following line adjust the External High Speed oscillator (HSE) Startup
   Timeout value
   */
#define HSE_STARTUP_TIMEOUT   ((uint16_t)0x0500) /*!< Time out for HSE start up */

/**
  * @brief  Sets System clock frequency to 72MHz and configure HCLK, PCLK2
  *         and PCLK1 prescalers.
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
static void SetSysClockTo72(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/
  /* Enable HSE */
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);

  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }

  if (HSEStatus == (uint32_t)0x01)
  {
    /* Enable Prefetch Buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    /* Flash 2 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;


    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;

    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;

    /* PCLK1 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;

#ifdef STM32F10X_CL
    /* Configure PLLs ------------------------------------------------------*/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    /* PREDIV1 configuration: PREDIV1CLK = PLL2 / 5 = 8 MHz */

    RCC->CFGR2 &= (uint32_t)~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL |
                              RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);
    RCC->CFGR2 |= (uint32_t)(RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 |
                             RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV5);

    /* Enable PLL2 */
    RCC->CR |= RCC_CR_PLL2ON;
    /* Wait till PLL2 is ready */
    while((RCC->CR & RCC_CR_PLL2RDY) == 0)
    {
    }


    /* PLL configuration: PLLCLK = PREDIV1 * 9 = 72 MHz */
    RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 |
                            RCC_CFGR_PLLMULL9);
#else
    /*  PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |
                                        RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);
#endif /* STM32F10X_CL */

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }

    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock
         configuration. User can add here some code to deal with this error */
  }
}

/**
  * @brief  Configures the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers.
  * @param  None
  * @retval None
  */
static void SetSysClock(void)
{

  //-SetSysClockTo56();

  SetSysClockTo72();


 /* If none of the define above is enabled, the HSI is used as System clock
    source (default after reset) */
}

/**
  * @brief  Setup the microcontroller system
  *         Initialize the Embedded Flash Interface, the PLL and update the
  *         SystemCoreClock variable.
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
void SystemInit (void)
{
  /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
#ifndef STM32F10X_CL
  RCC->CFGR &= (uint32_t)0xF8FF0000;
#else
  RCC->CFGR &= (uint32_t)0xF0FF0000;
#endif /* STM32F10X_CL */

  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
  RCC->CFGR &= (uint32_t)0xFF80FFFF;

#ifdef STM32F10X_CL
  /* Reset PLL2ON and PLL3ON bits */
  RCC->CR &= (uint32_t)0xEBFFFFFF;

  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x00FF0000;

  /* Reset CFGR2 register */
  RCC->CFGR2 = 0x00000000;
#elif defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || (defined STM32F10X_HD_VL)
  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x009F0000;

  /* Reset CFGR2 register */
  RCC->CFGR2 = 0x00000000;
#else
  /* Disable all interrupts and clear pending bits  */
  RCC->CIR = 0x009F0000;
#endif /* STM32F10X_CL */

#if defined (STM32F10X_HD) || (defined STM32F10X_XL) || (defined STM32F10X_HD_VL)
  #ifdef DATA_IN_ExtSRAM
    SystemInit_ExtMemCtl();
  #endif /* DATA_IN_ExtSRAM */
#endif

  /* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
  /* Configure the Flash Latency cycles and enable prefetch buffer */
  SetSysClock();

#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM. */
#else //-目前下面参与了编译
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH. */
#endif
}

void RCC_Configuration(void){

  SystemInit();


  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
  						| RCC_APB2Periph_GPIOD| RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO , ENABLE);

  //-测试用
  //-GPIO_InitTypeDef GPIO_InitStructure;
  //-GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-//-RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  //-GPIOC->BSRR = GPIO_Pin_6;
  //-GPIO_Init(GPIOC, &GPIO_InitStructure);
  //-end

  //-RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  //-RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
  /* Enable USART1, AFIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);
  /* Enable USART2, USART3 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3 | RCC_APB1Periph_UART4, ENABLE);

  /* TIM2 clocks enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//使能TIM1时钟

  /* Enable DMA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE );

  /* I2C1 clock enable */
  //-RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);   //-采用模拟I2C

#ifdef LSI_TIM_MEASURE
  /* Enable the LSI OSC */
  RCC_LSICmd(ENABLE);		//-the LSI OSC 是40KHz

  /* Wait till LSI is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {}

#endif
}


/*
STM32F103Rx     是64引脚的，引脚预定义如下：

--语音
PA9	  USART1_TX		ES704-UART-IN
PA10	USART1_RX		ES704-UART-OUT
--ES704
PC09	ES704-INTR-EVENT
PA08	ES704-RESET
--FLASH
PA04	FLASH-CS
PA05	FLASH-SLK
PA06	FLASH-SI
PA07	FLASH-SO
--i2c
PB6	I2C1_SCL	I2C1_SCL
PB7	I2C1_SDA	I2C1_SDA


--串口2和7620通讯
PA2	  USART2_TX
PA3 	USART2_RX


--Communication to 射灯板通讯  ,,这里需要注意和语音调试debug口共用需要整合
PB10	=	USART3_TX
PB11	=	USART3_RX


--PM2.5
--Sharp 5x PM2.5 (5VDC)	或者备用接CO2的
PC10	UART4_TX
PC11	UART4_RX

//-PC01	EN_CO2	输出控制串口读CO2值,如果不使用串口读CO2的话这个输出低电平即可
//-PC02	EN_PM 	输出控制串口读PM2.5值,如果串口4不共用的话,这个一直输出高电平保持导通就行


--VOC
PA12	I2C1_SCL	I2C1_SCL	,,使用模拟I2C
PA11	I2C1_SDA	I2C1_SDA



--噪声采集
PC5	ADC12_IN15	Noise Input


//-PC03	AD_NTC  ADC采集温度


--跑马灯
PB15	LD_OE_R
PC06	LD_LAT_R
PC07	LD_CLK_R
PC08	LD_SDI_R
PC09	LD_OE_G
PA08	LD_LAT_G
PA11	LD_CLK_G
PA12	LD_SDI_G
PD02	LD_OE_B
PB03	LD_LAT_B
PB04	LD_CLK_B
PB05	LD_SDI_B


--控制风扇
PA01  FAN-EN	输出控制


//---PWM输出呼吸灯
//-PC6	TIM3_CH1	PWM_G
//-PC7	TIM3_CH2	PWM_B
//-PC8	TIM3_CH3	PWM_R
*/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  //-为了UART设置参数
  /* Enable the USART2 Pins Software Remapping */
  //-GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
  //-USART2 Pins使用的就是PA2 和PA3没有重新定位
  //-Enable the USART3 Pins Software Remapping
  //-GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
  //-USART3 Pins使用的就是PC10 和PC11进行了部分重新映像

  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART2 Tx (PA.02) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART3 Tx (PB.10) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure USART4 Tx (PC.10) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART2 Rx (PA.03) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //-Configure USART3 Rx (PB.11) as input floating
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //-Configure USART4 Rx (PC.11) as input floating,进行PM2.5通讯接收
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

}

void uart_config(void)
{
	USART_InitTypeDef USART_InitStructure;

	//-配置可能的DMA
	DMA_InitTypeDef DMA_InitStructure;
#ifndef STM3210E_UP
  /*
     发送:
          把准备好的内容一次性让DMA发送出去,每次都需要修改长度.处理可以放在主循环中干,是否启动发送可以在中断中查询
     接收:
          开辟一个512的接收缓冲区,检查DMA传送的数量,然后读取数据.接收DMA是周期处理的,运用好指针的关系,我就可以模拟出
          一个512的FIFO.
  */
  // USARTy TX DMA1 Channel (triggered by USARTy Tx event) Config
  DMA_DeInit(DMA1_Channel7);  //-就是把所有的初始化为默认值
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_DR_Address;		//-外设地址(USART1):0x4001 3800 - 0x4001 3BFF + 0x04(寄存器偏移地址) = 外设绝对地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (UINT32)&port_send[0];   //-存储器地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;            //-数据传输方向,从外设读
  DMA_InitStructure.DMA_BufferSize = 8;             //-发送缓冲区尺寸,数据传输数量
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //-外设地址增量模式,不执行外设地址增量操作
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;				//-存储器增模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   //-外设数据宽度,8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;    //-存储器数据宽度,8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;     //-循环模式,执行循环操作,数据传输的数目变为0时，将会自动地被恢复成配置通道时设置的初值
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;   //-通道优先级,高?几个优先级怎么办:总共有4个优先级同一个模块上
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;   //-存储器到存储器模式,非存储器到存储器模式
  DMA_Init(DMA1_Channel7, &DMA_InitStructure);

  // USARTy RX DMA1 Channel (triggered by USARTy Rx event) Config
  DMA_DeInit(DMA1_Channel6);
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (UINT32)&port_recv[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 512;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//-需要周期循环,让DMA自动全部接收
  DMA_Init(DMA1_Channel6, &DMA_InitStructure);

	//-END

	//-对于下面的串口,同不同步在同一个端口上是可以设置的
  /* USART1 configuration ------------------------------------------------------*/
  /* USART and USART2 configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - Even parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 38400;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  //-USART_InitStructure.USART_Parity = USART_Parity_Even; //-偶校验
  USART_InitStructure.USART_Parity = USART_Parity_No;		//-无校验
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  //- Configure USART1
  USART_Init(USART2, &USART_InitStructure);
	//-如果使用中断方式,这里需要增加设置
  //- Enable the USART1
  USART_Cmd(USART2, ENABLE);  //-使能整个外设
  while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
	{ //-对于0说明发送缓冲区还有数据没有发送出去,所以需要等待直到发送空
	}

	//- Enable USARTy DMA TX request
  USART_DMACmd(USART2, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);

  //-DMA_Cmd(DMA1_Channel7, DISABLE);
  //-DMA_ClearFlag(DMA1_FLAG_TC7);
  //-通道使能 接收
  DMA_Cmd(DMA1_Channel6, ENABLE);
  //- Enable USARTy DMA TX Channel
  DMA_Cmd(DMA1_Channel7, ENABLE);

#else
  /*
     发送:
          把准备好的内容一次性让DMA发送出去,每次都需要修改长度.处理可以放在主循环中干,是否启动发送可以在中断中查询
     接收:
          开辟一个512的接收缓冲区,检查DMA传送的数量,然后读取数据.接收DMA是周期处理的,运用好指针的关系,我就可以模拟出
          一个512的FIFO.
  */
  /* USARTy TX DMA1 Channel (triggered by USARTy Tx event) Config */
  DMA_DeInit(DMA1_Channel4);  //-就是把所有的初始化为默认值
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Address;		//-外设地址(USART1):0x4001 3800 - 0x4001 3BFF + 0x04(寄存器偏移地址) = 外设绝对地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (UINT32)&port_send[0];   //-存储器地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;            //-数据传输方向,从外设读
  DMA_InitStructure.DMA_BufferSize = 1;             //-发送缓冲区尺寸,数据传输数量
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //-外设地址增量模式,不执行外设地址增量操作
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;				//-存储器增模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   //-外设数据宽度,8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;    //-存储器数据宽度,8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;     //-循环模式,执行循环操作,数据传输的数目变为0时，将会自动地被恢复成配置通道时设置的初值
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;   //-通道优先级,高?几个优先级怎么办:总共有4个优先级同一个模块上
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;   //-存储器到存储器模式,非存储器到存储器模式
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);

  /* USARTy RX DMA1 Channel (triggered by USARTy Rx event) Config */
  DMA_DeInit(DMA1_Channel5);
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (UINT32)&port_recv[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 512;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//-需要周期循环,让DMA自动全部接收
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);

	//-END

	//-对于下面的串口,同不同步在同一个端口上是可以设置的
  /* USART1 configuration ------------------------------------------------------*/
  /* USART and USART2 configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - Even parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 38400;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  //-USART_InitStructure.USART_Parity = USART_Parity_Even; //-偶校验
  USART_InitStructure.USART_Parity = USART_Parity_No;		//-无校验
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  /* Configure USART1 */
  USART_Init(USART1, &USART_InitStructure);
	//-如果使用中断方式,这里需要增加设置
  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);  //-使能整个外设
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{ //-对于0说明发送缓冲区还有数据没有发送出去,所以需要等待直到发送空
	}


	/* Enable USARTy DMA TX request */
  USART_DMACmd(USART1, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);

  //-通道使能 接收
  DMA_Cmd(DMA1_Channel5, ENABLE);
  /* Enable USARTy DMA TX Channel */
  DMA_Cmd(DMA1_Channel4, ENABLE);

#endif

}

//-就采用这样的形式,但是由于这个面对的板件各异我简单修整就可以了,不需要像他这样跳来跳去.
void STM_EVAL_bspInit(void)
{
  //-GPIO_InitTypeDef GPIO_InitStructure;
  //-EXTI_InitTypeDef EXTI_InitStructure;
  //-NVIC_InitTypeDef NVIC_InitStructure;
  //-初始化出一个debug口就行
  RCC_Configuration();
  GPIO_Configuration();
  //-NVIC_Configuration();  //-中断不需要设置目前
  uart_config();
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter can be one of following parameters:
  *     @arg Button_WAKEUP: Wakeup Push Button
  *     @arg Button_TAMPER: Tamper Push Button
  *     @arg Button_KEY: Key Push Button
  *     @arg Button_RIGHT: Joystick Right Push Button
  *     @arg Button_LEFT: Joystick Left Push Button
  *     @arg Button_UP: Joystick Up Push Button
  *     @arg Button_DOWN: Joystick Down Push Button
  *     @arg Button_SEL: Joystick Sel Push Button
  * @retval The Button GPIO pin value.
  */
uint32_t STM_EVAL_PBGetState(Button_TypeDef Button)
{
  return GPIO_ReadInputDataBit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}


/**
  * @brief  Configures COM port.
  * @param  COM: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:
  *     @arg COM1
  *     @arg COM2
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
/*
PA9	  USART1_TX
PA10	USART1_RX

--debug 口
PA2	  USART2_TX
PA3		USART2_RX
*/
void STM_EVAL_COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  /* Enable UART clock */
//-#if defined (USE_STM3210E_EVAL)
  if (COM == COM1)
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  }
  else
  {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  }
//-#elif defined (USE_STM3210B_EVAL)
//-  if (COM == COM1)
//-  {
//-    RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
//-  }
//-  else
//-  {
//-    /* Enable the USART2 Pins Software Remapping */
//-    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
//-    RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
//-  }
//-#elif defined (USE_STM3210C_EVAL)
//-  if (COM == COM1)
//-  {
//-    /* Enable the USART2 Pins Software Remapping */
//-    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
//-    RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
//-  }
//-#endif

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART2 Tx (PA.02) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART2 Rx (PA.03) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(COM_USART[COM], USART_InitStruct);

  /* Enable USART */
  USART_Cmd(COM_USART[COM], ENABLE);

  while(USART_GetFlagStatus(COM_USART[COM], USART_FLAG_TC) == RESET)
	{ //-对于0说明发送缓冲区还有数据没有发送出去,所以需要等待直到发送空
	}
}

/**
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
