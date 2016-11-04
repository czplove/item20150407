/*
	�Եײ��֧��
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

  //-������
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

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//ʹ��TIM1ʱ��

  /* Enable DMA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE );

  /* I2C1 clock enable */
  //-RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);   //-����ģ��I2C

#ifdef LSI_TIM_MEASURE
  /* Enable the LSI OSC */
  RCC_LSICmd(ENABLE);		//-the LSI OSC ��40KHz

  /* Wait till LSI is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {}

#endif
}


/*
STM32F103Rx     ��64���ŵģ�����Ԥ�������£�

--����
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


--����2��7620ͨѶ
PA2	  USART2_TX
PA3 	USART2_RX


--Communication to ��ư�ͨѶ  ,,������Ҫע�����������debug�ڹ�����Ҫ����
PB10	=	USART3_TX
PB11	=	USART3_RX


--PM2.5
--Sharp 5x PM2.5 (5VDC)	���߱��ý�CO2��
PC10	UART4_TX
PC11	UART4_RX

//-PC01	EN_CO2	������ƴ��ڶ�CO2ֵ,�����ʹ�ô��ڶ�CO2�Ļ��������͵�ƽ����
//-PC02	EN_PM 	������ƴ��ڶ�PM2.5ֵ,�������4�����õĻ�,���һֱ����ߵ�ƽ���ֵ�ͨ����


--VOC
PA12	I2C1_SCL	I2C1_SCL	,,ʹ��ģ��I2C
PA11	I2C1_SDA	I2C1_SDA



--�����ɼ�
PC5	ADC12_IN15	Noise Input


//-PC03	AD_NTC  ADC�ɼ��¶�


--�����
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


--���Ʒ���
PA01  FAN-EN	�������


//---PWM���������
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
  //-GPIOA->BSRR = GPIO_Pin_1;		//-��������ߵ�ƽ,�������ת
  GPIOA->BRR = GPIO_Pin_1;		//-��������͵�ƽ,������Ⱦ�ֹͣ
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //- Configure PB.00, PB.01 as output push-pull
  //-GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOB, &GPIO_InitStructure);

  //- Configure PB.05, PB.12 as input push-pull ,�����ж�1;����2�ж�
  //-GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_12 ;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOB, &GPIO_InitStructure);

  //- Configure PA.00 as input push-pull ,�����ж�
  //-GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 ;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOA, &GPIO_InitStructure);

  //-Configure PB.13 -- PB.15 as Output push-pull,����PWM���������
  //-TIM1_CH1N TIM1_CH2N TIM1_CH3N
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				//-����Ϊ���ø������
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOB, &GPIO_InitStructure);

  //-Configure PC.6 -- PC.8 as Output push-pull,����PWM���������
  //-TIM3_CH1 TIM3_CH2 TIM3_CH3
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 |GPIO_Pin_8;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				//-����Ϊ���ø������
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOC, &GPIO_InitStructure);

  //-Ϊ��UART���ò���
  /* Enable the USART2 Pins Software Remapping */
  //-GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
  //-USART2 Pinsʹ�õľ���PA2 ��PA3û�����¶�λ
  //-Enable the USART3 Pins Software Remapping
  //-GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
  //-USART3 Pinsʹ�õľ���PC10 ��PC11�����˲�������ӳ��

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

  //-Configure USART4 Rx (PC.11) as input floating,����PM2.5ͨѶ����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //-//-Ϊ��CAN�����ʹ��
  //-/* Configure CAN pin: RX */
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  //-GPIO_Init(GPIOD, &GPIO_InitStructure);

  //-/* Configure CAN pin: TX */
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  //-GPIO_Init(GPIOD, &GPIO_InitStructure);

  //-/* CAN pin remap to PD0/PD1 */
  //-GPIO_PinRemapConfig(GPIO_Remap2_CAN,ENABLE);  //-��ʵ����ѡ�����⹦������

  //-/* Configure PE.00 -- PE.15 as Output push-pull */
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOE, &GPIO_InitStructure);

  //-ADC
  //-Configure PC.5 (ADC12 Channel15) as analog input,�����ɼ�
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //-Configure PC.3 (ADC123 Channel13) as analog input,�¶Ȳɼ�
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
  //-VOC��ģ��I2C
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11  | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  I2C1_SCL_H;
  I2C1_SDA_H;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //���������
  //-//-RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO,ENABLE);
  //-GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//����JTAG,����SW���ͷ�PB3��PB4
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

  //-����оƬ����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				     //es704 reset pin
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				     //-���������ж�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //-������
	//PC04  �������룬�ⲿ����������
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

 //-���ж� PC9	I2C2_SMBA	INT3
  EXTI_ClearITPendingBit(EXTI_Line9);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource9);
  //- Configure EXTI Line12 to generate an interrupt on falling edge
  EXTI_InitStructure.EXTI_Line = EXTI_Line9;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;		//-������
  //-EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;		//-�½���
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

   if (SysTick_Config(SystemCoreClock / 1000))  //-������������Ѿ��Եδ�ʱ���������ж����ȼ�������,���ڵ�һ����
   {
     /* Capture error */
     while (1);
   }

}

///////////////////////////////////////////////////////////////////////////////


void uart_config(void)
{
	USART_InitTypeDef USART_InitStructure;

	//-���ÿ��ܵ�DMA
	DMA_InitTypeDef DMA_InitStructure;

  /*
     ����:
          ��׼���õ�����һ������DMA���ͳ�ȥ,ÿ�ζ���Ҫ�޸ĳ���.������Է�����ѭ���и�,�Ƿ��������Ϳ������ж��в�ѯ
     ����:
          ����һ��512�Ľ��ջ�����,���DMA���͵�����,Ȼ���ȡ����.����DMA�����ڴ����,���ú�ָ��Ĺ�ϵ,�ҾͿ���ģ���
          һ��512��FIFO.
  */
  // USARTy TX DMA1 Channel (triggered by USARTy Tx event) Config
  DMA_DeInit(DMA1_Channel7);  //-���ǰ����еĳ�ʼ��ΪĬ��ֵ
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_DR_Address;		//-�����ַ(USART1):0x4001 3800 - 0x4001 3BFF + 0x04(�Ĵ���ƫ�Ƶ�ַ) = ������Ե�ַ
  DMA_InitStructure.DMA_MemoryBaseAddr = (UINT32)&port_send[0][0];   //-�洢����ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;            //-���ݴ��䷽��,�������
  DMA_InitStructure.DMA_BufferSize = 1;             //-���ͻ������ߴ�,���ݴ�������
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //-�����ַ����ģʽ,��ִ�������ַ��������
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;				//-�洢����ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   //-�������ݿ��,8λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;    //-�洢�����ݿ��,8λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;     //-ѭ��ģʽ,ִ��ѭ������,���ݴ������Ŀ��Ϊ0ʱ�������Զ��ر��ָ�������ͨ��ʱ���õĳ�ֵ
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;   //-ͨ�����ȼ�,��?�������ȼ���ô��:�ܹ���4�����ȼ�ͬһ��ģ����
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;   //-�洢�����洢��ģʽ,�Ǵ洢�����洢��ģʽ
  DMA_Init(DMA1_Channel7, &DMA_InitStructure);

  // USARTy RX DMA1 Channel (triggered by USARTy Rx event) Config
  DMA_DeInit(DMA1_Channel6);
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (UINT32)&port_recv[0][0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 512;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//-��Ҫ����ѭ��,��DMA�Զ�ȫ������
  DMA_Init(DMA1_Channel6, &DMA_InitStructure);

	//-END

	//-��������Ĵ���,ͬ��ͬ����ͬһ���˿����ǿ������õ�
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
  //-USART_InitStructure.USART_Parity = USART_Parity_Even; //-żУ��
  USART_InitStructure.USART_Parity = USART_Parity_No;		//-��У��
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  //- Configure USART1
  USART_Init(USART2, &USART_InitStructure);
	//-���ʹ���жϷ�ʽ,������Ҫ��������
  //- Enable the USART1
  USART_Cmd(USART2, ENABLE);  //-ʹ����������
  while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
	{ //-����0˵�����ͻ�������������û�з��ͳ�ȥ,������Ҫ�ȴ�ֱ�����Ϳ�
	}

	//- Enable USARTy DMA TX request
  USART_DMACmd(USART2, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);

  //-ͨ��ʹ�� ����
  DMA_Cmd(DMA1_Channel6, ENABLE);
  //- Enable USARTy DMA TX Channel
  DMA_Cmd(DMA1_Channel7, ENABLE);



	DMA_DeInit(DMA1_Channel2);  //-���ǰ����еĳ�ʼ��ΪĬ��ֵ
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART3_DR_Address;		//-�����ַ(USART1):0x4001 3800 - 0x4001 3BFF + 0x04(�Ĵ���ƫ�Ƶ�ַ) = ������Ե�ַ
  DMA_InitStructure.DMA_MemoryBaseAddr = (UINT32)&UART3_port_send[0];   //-�洢����ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;            //-���ݴ��䷽��,�������
  DMA_InitStructure.DMA_BufferSize = 1;             //-���ͻ������ߴ�,���ݴ�������
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //-�����ַ����ģʽ,��ִ�������ַ��������
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;				//-�洢����ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   //-�������ݿ��,8λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;    //-�洢�����ݿ��,8λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;     //-ѭ��ģʽ,ִ��ѭ������,���ݴ������Ŀ��Ϊ0ʱ�������Զ��ر��ָ�������ͨ��ʱ���õĳ�ֵ
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;   //-ͨ�����ȼ�,��?�������ȼ���ô��:�ܹ���4�����ȼ�ͬһ��ģ����
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;   //-�洢�����洢��ģʽ,�Ǵ洢�����洢��ģʽ
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);

  //- USARTy RX DMA1 Channel (triggered by USARTy Rx event) Config
  DMA_DeInit(DMA1_Channel3);
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART3_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (UINT32)&port_recv[1][0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 512;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//-��Ҫ����ѭ��,��DMA�Զ�ȫ������
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	//-USART3 configuration
	USART_InitStructure.USART_BaudRate = 38400; //-38400 115200
 //USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  //USART_InitStructure.USART_Parity = USART_Parity_Even; //-żУ��
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);
  USART_Cmd(USART3, ENABLE);
  while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
	{ //-����0˵�����ͻ�������������û�з��ͳ�ȥ,������Ҫ�ȴ�ֱ�����Ϳ�
	}

	//- Enable USARTy DMA TX request
  USART_DMACmd(USART3, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);

  //-ͨ��ʹ�� ����
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
	//-USART4 configuration,����PM2.5
  USART_InitStructure.USART_BaudRate = 2400;
 //USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  //-USART_InitStructure.USART_Parity = USART_Parity_Even; //-żУ��
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART4, &USART_InitStructure);
  USART_Cmd(UART4, ENABLE);
  while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET)
	{ //-����0˵�����ͻ�������������û�з��ͳ�ȥ,������Ҫ�ȴ�ֱ�����Ϳ�
	}

}

void TIM_config(void)
{
	 /*TIM_TimeBaseInitTypeDef        TIM1_TimeBaseStructure;
   TIM_OCInitTypeDef                  TIM_OCInitStructure;
   TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
   //-----------------------------------��ʱ��TIM1��������------------------------------------
   TIM_DeInit(TIM1);//���½�TIM1��Ϊȱʡֵ
   TIM_InternalClockConfig(TIM1);//�����ڲ�ʱ�Ӹ�TIM1�ṩʱ��Դ ������8M
   TIM1_TimeBaseStructure.TIM_Prescaler=8;//7��Ƶ,TIM1Ƶ��Ϊ56MHz/7=8M
   TIM1_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���ģʽ
   TIM1_TimeBaseStructure.TIM_Period=PWM_Period_Value-1;//���������С,ÿ��5000��������һ�������¼�,��PWM���Ƶ��1KHz
   TIM1_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//ʱ�ӷָ�TDTS=Tck_tim
   TIM1_TimeBaseStructure.TIM_RepetitionCounter=0x0;
   TIM_TimeBaseInit(TIM1,&TIM1_TimeBaseStructure);
   //-���ϼ���,û��ֵǰ�ο���ѹΪ��,����֮��Ϊ��
   //-�����źŵ�����ǲο���ѹ�Ƚϵ�,�����������Ǻ�,��ô������Ͳο���ѹһ��,��һ���෴,ֻ�Ƕ���һ����ʱ
   //- PWM1 Mode configuration: Channel1 		//-GREEN
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;		//-����û��������������Խ�ֹ���
  TIM_OCInitStructure.TIM_Pulse = PWM_Period_Value;			//-�����ȸ�����ֵ
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//-��������ļ���:0�� OC1�ߵ�ƽ��Ч��1�� OC1�͵�ƽ��Ч��
  //���漸�������Ǹ߼���ʱ���Ż��õ���ͨ�ö�ʱ����������
  TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;  //���û������������
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;//ʹ�ܻ��������
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;  //���������״̬
	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCIdleState_Set;//�����󻥲������״̬

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  //���岽��������ɲ���������ã��߼���ʱ�����еģ�ͨ�ö�ʱ����������
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Disable;//����ģʽ�����ѡ��
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;//����ģʽ�����ѡ��
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF; //��������
	TIM_BDTRInitStructure.TIM_DeadTime = 0; //����ʱ������
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable; //ɲ������ʹ��
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;//ɲ�����뼫��
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;//�Զ����ʹ��
	TIM_BDTRConfig(TIM1,&TIM_BDTRInitStructure);

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  //- PWM1 Mode configuration: Channel2  //-BLUE
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = PWM_Period_Value;

  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  //- PWM1 Mode configuration: Channel3  //-RED
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  //-��������ΪPWM_Period_Valueʱ,PWM����ֱ����� 1
  //-��������Ϊ0ʱ,PWM����ֱ����� 0
  TIM_OCInitStructure.TIM_Pulse = PWM_Period_Value;		//-���1 ����;���0 ���

  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1, ENABLE);
  //- TIM1 enable counter
  TIM_Cmd(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);		//-TIMx->BDTR
  */

   //-�������Ź�
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
  IWDG_SetReload(1000);	//-���������ֵ�����˿��Ź���λʱ��ÿ�ε�ʱ����0.8mS,�ܴ���������ʱ��:100*0.8=80mS,��ô�����ʱ���ڱ���ι��
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  //-IWDG_Enable();

}

void adc_config(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;


	DMA_DeInit(DMA1_Channel1);  //-���ǰ�������Ҫ�Ľ��г�ʼ����ֵ
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address; //-�����ַ
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue[0];  //-�洢����ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //-���ݴ��䷽��,�������
  //-DMA_InitStructure.DMA_BufferSize = 1; //-���ݴ�������Ϊ1
  DMA_InitStructure.DMA_BufferSize = maxbuffer;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//-�����ַ����ģʽ,��ִ�������ַ��������
  //-DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
  //-DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;//-�洢����ַ����ģʽ,��ִ�д洢����ַ��������
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//-�������ݿ��,16λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//-�洢�����ݿ��,16λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //-ѭ��ģʽ,ִ��ѭ������
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//-ͨ�����ȼ�,��
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//-�洢�����洢��ģʽ,�Ǵ洢�����洢��ģʽ
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  /* Enable DMA channel1 */
//-ͨ������,,����ͨ������֮�����Դ���и��¾��Զ����Ƶ�Ŀ�Ĵ�
  /*ADC1��ADC2��������ܾ����������ñ����л�*/
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  //-����ģʽ
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;  //-ʹ��ɨ��ģʽ
  //-ADC_InitStructure.ADC_ScanConvMode = DISABLE;  //
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  //-ת������������ֱ����λ�����
 // ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; //-����ת��ģʽ
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //-���������λ����ת��
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  //-�����Ҷ���
  //-ADC_InitStructure.ADC_NbrOfChannel = 1; //-�����ڹ���ͨ��ת�������е�ͨ����ĿΪ1
  ADC_InitStructure.ADC_NbrOfChannel = 2;
  //ADC_InitStructure.ADC_NbrOfChannel = 7;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel10 configuration PIN15*///-ͨ������ĺ�������ȷ��ת��˳��͸���
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_28Cycles5); //-��ת��ʱ��TCONV= 1.5 + 12.5 = 14 ���� = 1 �� s

  /* ADC1 regular channel13 configuration PIN18*/
  //-ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 2, ADC_SampleTime_28Cycles5);		//-�ⲿNTC���ݶ�ȡ
/* ADC1 regular channel13 configuration PIN18*/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 2, ADC_SampleTime_28Cycles5);  //-оƬ�ڲ��¶�


  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE); //-ʹ��DMAģʽ,���ܵ����֮��ͻ��Զ������������ź�
// ADC_DMACmd(ADC1, DISABLE);
  
  ADC_TempSensorVrefintCmd(ENABLE); //ʹ���¶ȴ��������ڲ��ο���ѹͨ��
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);  //-����ADC

  /* Enable ADC1 reset calibaration register */
  ADC_ResetCalibration(ADC1); //-��ʼ��У׼�Ĵ���
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1); //-��������Կ�ʼУ׼������У׼����ʱ��Ӳ�����
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


