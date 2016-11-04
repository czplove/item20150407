/*
	对底层的支持
*/
#include "stm32f10x.h"
#include "demo.h"
#include "integer.h"
#include "user_conf.h"



#define ADC1_DR_Address    ((u32)0x4001244C)
#define USART1_DR_Address    ((u32)0x40013804)
#define USART2_DR_Address    ((u32)0x40004404)
#define USART3_DR_Address    ((u32)0x40004804)


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

  //-Configure PA.01 as output push-pull
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIOA->BSRR = GPIO_Pin_1;		//-引脚输出高电平,这里风扇转
  GPIOA->BRR = GPIO_Pin_1;		//-引脚输出低电平,这里风扇就停止
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //- Configure PB.00, PB.01 as output push-pull
  //-GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOB, &GPIO_InitStructure);

  //- Configure PB.05, PB.12 as input push-pull ,手势中断1;手势2中断
  //-GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_12 ;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOB, &GPIO_InitStructure);

  //- Configure PA.00 as input push-pull ,触摸中断
  //-GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 ;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOA, &GPIO_InitStructure);

  //-Configure PB.13 -- PB.15 as Output push-pull,控制PWM输出呼吸灯
  //-TIM1_CH1N TIM1_CH2N TIM1_CH3N
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				//-设置为复用浮空输出
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOB, &GPIO_InitStructure);

  //-Configure PC.6 -- PC.8 as Output push-pull,控制PWM输出呼吸灯
  //-TIM3_CH1 TIM3_CH2 TIM3_CH3
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 |GPIO_Pin_8;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				//-设置为复用浮空输出
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOC, &GPIO_InitStructure);

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

  //-//-为了CAN外设的使用
  //-/* Configure CAN pin: RX */
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  //-GPIO_Init(GPIOD, &GPIO_InitStructure);

  //-/* Configure CAN pin: TX */
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  //-GPIO_Init(GPIOD, &GPIO_InitStructure);

  //-/* CAN pin remap to PD0/PD1 */
  //-GPIO_PinRemapConfig(GPIO_Remap2_CAN,ENABLE);  //-其实就是选择特殊功能引脚

  //-/* Configure PE.00 -- PE.15 as Output push-pull */
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOE, &GPIO_InitStructure);

  //-ADC
  //-Configure PC.5 (ADC12 Channel15) as analog input,噪声采集
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //-Configure PC.3 (ADC123 Channel13) as analog input,温度采集
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
#if 0
  /* I2C1 SDA and SCL configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
#else
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6  | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  I2C2_SCL_H;
  I2C2_SDA_H;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif
  //-VOC的模拟I2C
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11  | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  I2C1_SCL_H;
  I2C1_SDA_H;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //跑马灯配置
  //-//-RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO,ENABLE);
  //-GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//禁用JTAG,启用SW，释放PB3和PB4
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5| GPIO_Pin_15;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOB, &GPIO_InitStructure);

  //- Configure PC8, PC7, PC6 and PC9 as output push-pull
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIOC->BSRR |= GPIO_Pin_6;
  GPIOC->BRR |= GPIO_Pin_7;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //- Configure PA8, PA11 and PA12 as output push-pull
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIOB->BRR |= GPIO_Pin_13;
  GPIOB->BRR |= GPIO_Pin_15;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //-语音芯片配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				     //es704 reset pin
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				     //-语音唤醒中断
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //-测试用
	//PC04  浮动输入，外部有上拉电阻
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void GPIO_Configuration_out(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;


  //- Configure PD2 as output push-pull
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //- Configure PD2 as output push-pull
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void GPIO_Configuration_in(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void EXTI_Configuration(void)
{
  EXTI_InitTypeDef  EXTI_InitStructure;


//-EXTI_ClearITPendingBit(EXTI_Line12);
  /* Configure EXTI Line3 to generate an interrupt on falling edge */
  //-EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  //-EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  //-EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  //-EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  //-EXTI_Init(&EXTI_InitStructure);
  //-GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);

 //-振动中断 PC9	I2C2_SMBA	INT3
  EXTI_ClearITPendingBit(EXTI_Line9);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource9);
  //- Configure EXTI Line12 to generate an interrupt on falling edge
  EXTI_InitStructure.EXTI_Line = EXTI_Line9;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;		//-上升沿
  //-EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;		//-下降沿
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

}


void NVIC_Configuration(void)
{

  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  //-NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  //-NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  //-NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  //-NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //-NVIC_Init(&NVIC_InitStructure);

  //-NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
  //-NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  //-NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  //-NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //-NVIC_Init(&NVIC_InitStructure);

  //-NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  //-NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  //-NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  //-NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //-NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;			//-by zj 2015/9/7 11:04:20
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void init_system_tick(void)
{

   if (SysTick_Config(SystemCoreClock / 1000))  //-这个函数里面已经对滴答定时器进行了中断优先级的设置,属于第一级内
   {
     /* Capture error */
     while (1);
   }

}

///////////////////////////////////////////////////////////////////////////////


void uart_config(void)
{
	USART_InitTypeDef USART_InitStructure;

	//-配置可能的DMA
	DMA_InitTypeDef DMA_InitStructure;

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
  DMA_InitStructure.DMA_MemoryBaseAddr = (UINT32)&port_send[0][0];   //-存储器地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;            //-数据传输方向,从外设读
  DMA_InitStructure.DMA_BufferSize = 1;             //-发送缓冲区尺寸,数据传输数量
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
  DMA_InitStructure.DMA_MemoryBaseAddr = (UINT32)&port_recv[0][0];
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

  //-通道使能 接收
  DMA_Cmd(DMA1_Channel6, ENABLE);
  //- Enable USARTy DMA TX Channel
  DMA_Cmd(DMA1_Channel7, ENABLE);



	DMA_DeInit(DMA1_Channel2);  //-就是把所有的初始化为默认值
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART3_DR_Address;		//-外设地址(USART1):0x4001 3800 - 0x4001 3BFF + 0x04(寄存器偏移地址) = 外设绝对地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (UINT32)&UART3_port_send[0];   //-存储器地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;            //-数据传输方向,从外设读
  DMA_InitStructure.DMA_BufferSize = 1;             //-发送缓冲区尺寸,数据传输数量
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //-外设地址增量模式,不执行外设地址增量操作
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;				//-存储器增模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   //-外设数据宽度,8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;    //-存储器数据宽度,8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;     //-循环模式,执行循环操作,数据传输的数目变为0时，将会自动地被恢复成配置通道时设置的初值
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;   //-通道优先级,高?几个优先级怎么办:总共有4个优先级同一个模块上
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;   //-存储器到存储器模式,非存储器到存储器模式
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);

  //- USARTy RX DMA1 Channel (triggered by USARTy Rx event) Config
  DMA_DeInit(DMA1_Channel3);
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART3_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (UINT32)&port_recv[1][0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 512;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//-需要周期循环,让DMA自动全部接收
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	//-USART3 configuration
	USART_InitStructure.USART_BaudRate = 38400; //-38400 115200
 //USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  //USART_InitStructure.USART_Parity = USART_Parity_Even; //-偶校验
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);
  USART_Cmd(USART3, ENABLE);
  while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
	{ //-对于0说明发送缓冲区还有数据没有发送出去,所以需要等待直到发送空
	}

	//- Enable USARTy DMA TX request
  USART_DMACmd(USART3, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);

  //-通道使能 接收
  DMA_Cmd(DMA1_Channel3, ENABLE);
  //- Enable USARTy DMA TX Channel
  DMA_Cmd(DMA1_Channel2, ENABLE);

  //-Usart_Debug_Port_Init
  //-USART_InitStructure.USART_BaudRate = 115200;
  //-USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  //-USART_InitStructure.USART_StopBits = USART_StopBits_1;
  //-USART_InitStructure.USART_Parity = USART_Parity_No;
  //-USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  //-USART_InitStructure.USART_Mode =USART_Mode_Rx | USART_Mode_Tx;

  //Configure USART3
  //-USART_Init(USART3, &USART_InitStructure);
  // Enable the USART3
  //-USART_Cmd(USART3, ENABLE);
  //-USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);



  //////////////////////////////////////////////////////////////////////////////
  //-USART1 configuration


	//////////////////////////////////////////////////////////////////////////////
	//-USART4 configuration,用于PM2.5
  USART_InitStructure.USART_BaudRate = 2400;
 //USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  //-USART_InitStructure.USART_Parity = USART_Parity_Even; //-偶校验
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART4, &USART_InitStructure);
  USART_Cmd(UART4, ENABLE);
  while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET)
	{ //-对于0说明发送缓冲区还有数据没有发送出去,所以需要等待直到发送空
	}

}

void TIM_config(void)
{
	 /*TIM_TimeBaseInitTypeDef        TIM1_TimeBaseStructure;
   TIM_OCInitTypeDef                  TIM_OCInitStructure;
   TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
   //-----------------------------------定时器TIM1基本配置------------------------------------
   TIM_DeInit(TIM1);//重新将TIM1设为缺省值
   TIM_InternalClockConfig(TIM1);//采用内部时钟给TIM1提供时钟源 假如是8M
   TIM1_TimeBaseStructure.TIM_Prescaler=8;//7分频,TIM1频率为56MHz/7=8M
   TIM1_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
   TIM1_TimeBaseStructure.TIM_Period=PWM_Period_Value-1;//计数溢出大小,每计5000个数产生一个更新事件,即PWM输出频率1KHz
   TIM1_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//时钟分割TDTS=Tck_tim
   TIM1_TimeBaseStructure.TIM_RepetitionCounter=0x0;
   TIM_TimeBaseInit(TIM1,&TIM1_TimeBaseStructure);
   //-向上计数,没到值前参考电压为高,到了之后为低
   //-互补信号的输出是参考电压比较的,如果输出极性是好,那么主输出和参考电压一样,另一个相反,只是都有一个延时
   //- PWM1 Mode configuration: Channel1 		//-GREEN
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;		//-现在没有用这个引脚所以禁止输出
  TIM_OCInitStructure.TIM_Pulse = PWM_Period_Value;			//-可以先给个初值
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//-决定输出的极性:0： OC1高电平有效；1： OC1低电平有效。
  //下面几个参数是高级定时器才会用到，通用定时器不用配置
  TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;  //设置互补端输出极性
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;//使能互补端输出
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;  //死区后输出状态
	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCIdleState_Set;//死区后互补端输出状态

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  //第五步，死区和刹车功能配置，高级定时器才有的，通用定时器不用配置
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Disable;//运行模式下输出选择
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;//空闲模式下输出选择
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF; //锁定设置
	TIM_BDTRInitStructure.TIM_DeadTime = 0; //死区时间设置
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable; //刹车功能使能
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;//刹车输入极性
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;//自动输出使能
	TIM_BDTRConfig(TIM1,&TIM_BDTRInitStructure);

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  //- PWM1 Mode configuration: Channel2  //-BLUE
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = PWM_Period_Value;

  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  //- PWM1 Mode configuration: Channel3  //-RED
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  //-脉宽设置为PWM_Period_Value时,PWM引脚直接输出 1
  //-脉宽设置为0时,PWM引脚直接输出 0
  TIM_OCInitStructure.TIM_Pulse = PWM_Period_Value;		//-输出1 亮灯;输出0 灭灯

  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1, ENABLE);
  //- TIM1 enable counter
  TIM_Cmd(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);		//-TIMx->BDTR
  */

   //-独立看门狗
   /* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency dispersion) */
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  /* IWDG counter clock: LSI/32 */
  IWDG_SetPrescaler(IWDG_Prescaler_32);
  /* Set counter reload value to obtain 250ms IWDG TimeOut.
     Counter Reload Value = 250ms/IWDG counter clock period
                          = 250ms / (LSI/32)
                          = 0.25s / (LsiFreq/32)
                          = LsiFreq/(32 * 4)
                          = LsiFreq/128
   */
  IWDG_SetReload(1000);	//-这里的设置值决定了看门狗复位时间每次的时间是0.8mS,总次数将决定时长:100*0.8=80mS,那么在这个时间内必须喂狗
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  //-IWDG_Enable();

}

void adc_config(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;


	DMA_DeInit(DMA1_Channel1);  //-就是把所有需要的进行初始化赋值
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address; //-外设地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue[0];  //-存储器地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //-数据传输方向,从外设读
  //-DMA_InitStructure.DMA_BufferSize = 1; //-数据传输数量为1
  DMA_InitStructure.DMA_BufferSize = maxbuffer;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//-外设地址增量模式,不执行外设地址增量操作
  //-DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
  //-DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;//-存储器地址增量模式,不执行存储器地址增量操作
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//-外设数据宽度,16位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//-存储器数据宽度,16位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //-循环模式,执行循环操作
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//-通道优先级,高
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//-存储器到存储器模式,非存储器到存储器模式
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  /* Enable DMA channel1 */
//-通道开启,,我想通道开启之后就是源处有更新就自动复制到目的处
  /*ADC1和ADC2的区别可能就是两套配置便于切换*/
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  //-独立模式
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;  //-使用扫描模式
  //-ADC_InitStructure.ADC_ScanConvMode = DISABLE;  //
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  //-转换将连续进行直到该位被清除
 // ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; //-单次转换模式
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //-用软件控制位触发转换
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  //-数据右对齐
  //-ADC_InitStructure.ADC_NbrOfChannel = 1; //-定义在规则通道转换序列中的通道数目为1
  ADC_InitStructure.ADC_NbrOfChannel = 2;
  //ADC_InitStructure.ADC_NbrOfChannel = 7;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel10 configuration PIN15*///-通过下面的函数可以确定转换顺序和个数
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_28Cycles5); //-总转换时间TCONV= 1.5 + 12.5 = 14 周期 = 1 μ s

  /* ADC1 regular channel13 configuration PIN18*/
  //-ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 2, ADC_SampleTime_28Cycles5);		//-外部NTC数据读取
/* ADC1 regular channel13 configuration PIN18*/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 2, ADC_SampleTime_28Cycles5);  //-芯片内部温度


  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE); //-使用DMA模式,可能当完成之后就会自动发送请求处理信号
// ADC_DMACmd(ADC1, DISABLE);
  
  ADC_TempSensorVrefintCmd(ENABLE); //使能温度传感器和内部参考电压通道
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);  //-开启ADC

  /* Enable ADC1 reset calibaration register */
  ADC_ResetCalibration(ADC1); //-初始化校准寄存器
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1); //-软件设置以开始校准，并在校准结束时由硬件清除
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));

  DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);
  DMA_Cmd(DMA1_Channel1, ENABLE);

  /* Start ADC1 Software Conversion */
  //-ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

#if 0
void I2C1_Init(I2C_InitTypeDef* I2C_InitStruct)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);



    GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    I2C_Init(I2C1, I2C_InitStruct);
    I2C_Cmd(I2C1, ENABLE);




}

void I2C_init(void)
{
   I2C_InitTypeDef I2C_InitStructure;
    I2C_InitStructure.I2C_ClockSpeed    =  400000;
    I2C_InitStructure.I2C_Mode          =  I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle     =  I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1   =  0x7C;
    I2C_InitStructure.I2C_Ack           =  I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

   I2C1_Init(&I2C_InitStructure);

}
#endif





void Usart_es704_Init(void)
{
  //-GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  //-RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 , ENABLE);

  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  //-GPIO_Init(GPIOA, &GPIO_InitStructure);

  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  //-GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure USART1 */
  USART_Init(USART1, &USART_InitStructure);
   /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

}


