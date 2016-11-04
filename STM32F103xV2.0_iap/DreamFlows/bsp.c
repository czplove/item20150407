/*
	�Եײ��֧��
*/
#include "user_conf.h"

//#include "stm32f10x_flash.h"


#define ADC1_DR_Address    ((u32)0x4001244C)
#define USART1_DR_Address    ((u32)0x40013804)




/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)  //-���������������п���ʹ�õ��Ĺ���ʱ��
{
  ErrorStatus HSEStartUpStatus;

  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);  //-ʹ���ⲿ���پ���

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  //-����Ĳ������ܳɹ�����ʧ��,���ʧ���˾Ͳ������������,����ֱ��ʹ�õ�Ĭ�ϵ�
  if(HSEStartUpStatus == SUCCESS)
  {
    /* HCLK = SYSCLK */ //- AHB Ԥ��Ƶ,AHBʱ�ӵ�Ԥ��Ƶϵ��Ϊ1
    RCC_HCLKConfig(RCC_SYSCLK_Div1);  //-��ϵͳʱ��ȷ��֮��,�����ܶ�ʱ�ӿ��Զ�������

    /* PCLK2 = HCLK */  //-����APBԤ��Ƶ(APB2),���Ƹ���APB2ʱ��(PCLK2) ��Ԥ��Ƶϵ��Ϊ1
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* PCLK1 = HCLK/2 */  //-����APBԤ��Ƶ(APB1),���Ƶ���APB1ʱ��(PCLK1) ��Ԥ��Ƶϵ��Ϊ 2��Ƶ
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* ADCCLK = PCLK2/4 */  //-ADCԤ��Ƶ,ADCʱ��Ƶ�ʷ�Ƶ����ΪADCʱ��
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); //-ADCʱ��Ϊ14 MHz

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);  //-number of wait states for a read operation programmed on-the-fly

    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* PLLCLK = 4MHz * 9 = 36 MHz */  //-HSEʱ����ΪPLL ����ʱ��,PLL 7��Ƶ���
    //-RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);  //-��ϵͳ��8*9= 72 MHz
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_7);  //-��ϵͳ��8*7= 56 MHz

    /* Enable PLL */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)  //-������ܳ�����ѭ��
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);  //-��������ѡ����PLL��Ϊϵͳʱ��

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)  //-��ʵ���Ƕ�ȡ״̬�Ĵ���ֵ
    {
    }

    //-���ӿ��Ź��Ĵ���
    /* Check if the system has resumed from IWDG reset */
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
    {
  	     /* Clear reset flags */
         RCC_ClearFlag();
    }
  }

  /* Enable USART1, AFIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);
  /* Enable USART2, USART3 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3, ENABLE);

  /* Enable GPIOA, GPIOB, GPIOC clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC, ENABLE);
  //-��ͬ��������ڲ�ͬ��������,���ڵĿ⺯���������ṩ�˱��˼·,���Ҷ���Ĳ���ͨ���Ժܺ�
  /* TIM2 clocks enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//ʹ��TIM1ʱ��

  //-/* CAN Periph clock enable */
  //-RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE);

  /* Enable DMA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE );

#if I2C_HARDWARE_FLAG
  /* I2C1 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  /* I2C2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
#else


#endif

#ifdef LSI_TIM_MEASURE
  /* Enable the LSI OSC */
  RCC_LSICmd(ENABLE);		//-the LSI OSC ��40KHz

  /* Wait till LSI is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {}

#endif

}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*
STM32F103Rx     ��64���ŵģ�����Ԥ�������£�

--������
PC0	=	LD_A0
PC1	=	LD_A1
PC2	=	LD_A2
PC3	=	LD_A3
PA4	=	LD_LAT
PA5	=	LD_CLK
PA6	=	LD_OE
PA7	=	LD_SDI
PB0		LD_LE


--���� ��
PB6	I2C1_SCL	I2C1_SCL
PB7	I2C1_SDA	I2C1_SDA
PB13	  I2C3_SMBA	INT1		--���ж�


--ADI����
PB10	I2C2_SCL	I2C2_SCL
PB11	I2C2_SDA	I2C2_SDA
PB12  I2C2_SMBA	INT2		--�����ж�2


--PWM���������
PC6	TIM3_CH1	PWM_R
PC7	TIM3_CH2	PWM_G
PC8	TIM3_CH3	PWM_B
PC9	        	LED_EN


--Communication to ������STM32�Ĵ���2ͨѶ
PA9	  USART1_TX
PA10	USART1_RX


--��������
PC12	I2C3	ԭ������Ϊģ��I2Cʹ�õ�������Ϊ������Ŷ��ⲿ״̬0 ����״̬;1 ����״̬

--debug ��
PA2	  USART2_TX
PA3		USART2_RX
*/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  //- Configure PC.00, PC.01, PC.02 and PC.03 as output push-pull
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  //- Configure PA.04, PA.05, PA.06 and PA.07 as output push-pull
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOA->BSRR = GPIO_Pin_6;		//-��֤GPIO�ĳ�ʼ״̬
  GPIOA->BRR = GPIO_Pin_7;		//-��֤GPIO�ĳ�ʼ״̬
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //- Configure PB.00 as output push-pull
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //- Configure PB.12, PB.13 as input push-pull ,ADI����;���ж�
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //- Configure PC.10 as input push-pull ,������ٶ��ж�
  //-GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 ;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  //-//-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//-��������û������,��������
  //-//-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//-������ŵ�ƽ״̬
  //-//-GPIOC->BSRR = GPIO_Pin_10;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOC, &GPIO_InitStructure);

  //- Configure PA.00 as input push-pull ,�����ж�
  //-GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 ;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOA, &GPIO_InitStructure);

  //- Configure PC.09 as output push-pull
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIOC->BSRR = GPIO_Pin_9;   //-�øߵ�ƽ����
  //-GPIOC->BRR = GPIO_Pin_9; //-�õ͵�ƽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  //-Configure PB.13 -- PB.15 as Output push-pull,����PWM���������
  //-TIM1_CH1N TIM1_CH2N TIM1_CH3N
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				//-����Ϊ���ø������
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOB, &GPIO_InitStructure);
  //-Configure PC.6 -- PC.8 as Output push-pull,����PWM���������
  //-TIM3_CH1 TIM3_CH2 TIM3_CH3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 |GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				//-����Ϊ���ø������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
  //- Ϊ�˲����ⲿ��·�Ƿ�������,������Ҫ����ָ����ƽ�����Ųʵ��Ƿ��ܿ�
  //-GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-//-���������ƽ1 ����.
  //-GPIOC->BRR = GPIO_Pin_6;		//-RED��֤GPIO�ĳ�ʼ״̬
  //-GPIOC->BSRR = GPIO_Pin_7;		//-GREEN��֤GPIO�ĳ�ʼ״̬
  //-GPIOC->BSRR = GPIO_Pin_8;   //-BLUE
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

  /* Configure USART3 Tx (PC.10) as alternate function push-pull */
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  //-GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART2 Rx (PA.03) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //-Configure USART3 Rx (PC.11) as input floating,����PM2.5ͨѶ����
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  //-GPIO_Init(GPIOC, &GPIO_InitStructure);

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
  //-Configure PC.4,5 (ADC Channel14,15) as analog input,����PM2.5ģ���źŵĲɼ�,���������ɼ�
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  //-GPIO_Init(GPIOC, &GPIO_InitStructure);

#if I2C_HARDWARE_FLAG
  /* I2C1 SDA and SCL configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure I2C2 pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
#else
  //-I2C1 PB6 SCK �������
  //-GPIO_Mode_Out_PP		����-���
  //-GPIO_Mode_Out_OD		��©-���
  //-��©������ǲ������ѹ���͵�ƽʱ�ӵأ��ߵ�ƽʱ���ӵء��������������裬��
  //-������ߵ�ƽʱ��ѹ��������������ĵ�Դ��ѹ�����ַ�ʽ�ʺ������ӵ������ѹ��
  //-��Ƭ����ѹ�͵�ʱ��ʹ��ʱҪ���������衣
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//-ͬһ��������,������������ʱ���ź�,���������������Ϳ�����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOB->BSRR = GPIO_Pin_6;		//-RED��֤GPIO�ĳ�ʼ״̬
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//-ͬһ��������,������������ʱ���ź�,���������������Ϳ�����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOB->BSRR = GPIO_Pin_7;		//-RED��֤GPIO�ĳ�ʼ״̬
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //-TWI_Delay();
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-I2C1_SDA_H;
  //-GPIO_Init(GPIOB, &GPIO_InitStructure);

  //-Configure I2C2 pins: SCL and SDA
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//-ͬһ��������,������������ʱ���ź�,���������������Ϳ�����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOB->BSRR = GPIO_Pin_10;		//-RED��֤GPIO�ĳ�ʼ״̬
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  I2C2_SDA_H;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
/*
  //-I2C3 A8 C9
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//-ͬһ��������,������������ʱ���ź�,���������������Ϳ�����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOA->BSRR = GPIO_Pin_8;		//-RED��֤GPIO�ĳ�ʼ״̬
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  I2C3_SDA_H;
  GPIO_Init(GPIOC, &GPIO_InitStructure);		//-LED_EN

  //-I2C4 C11 C12
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//-ͬһ��������,������������ʱ���ź�,���������������Ϳ�����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOC->BSRR = GPIO_Pin_11;		//-RED��֤GPIO�ĳ�ʼ״̬
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  //-ǿ�����߷�ֹǯλ?
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//-ͬһ��������,������������ʱ���ź�,���������������Ϳ�����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOC->BSRR = GPIO_Pin_12;		//-RED��֤GPIO�ĳ�ʼ״̬
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  TWI_Delay();
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  I2C4_SDA_H;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //-I2C5 B15 A12
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//-ͬһ��������,������������ʱ���ź�,���������������Ϳ�����
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOB->BSRR = GPIO_Pin_15;		//-RED��֤GPIO�ĳ�ʼ״̬
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  I2C5_SDA_H;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
*/
#endif

//-������
//PC12  �������룬�ⲿ����������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  //-GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void GPIO_Configuration_out(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;


  //- Configure PD2 as output push-pull
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 |GPIO_Pin_1 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //- Configure PD2 as output push-pull
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void GPIO_Configuration_in(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures the NVIC and Vector Table base address.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
  /* Set the Vector Table base location at 0x20000000 */
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */  //-��ϵͳ��Ԥ�����ﶨ�����������
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); //?
#endif

  /*
  	�쳣���ȼ�:
  	��оƬ֧��֧�����ȼ��������(ռ�����ȼ����ʹ����ȼ���).
  	���ǽ�ռ�����ȼ���Ϊ�����ȼ�������ж�������쳣������ͬ�������ȼ���
    ����ʹ�ô����ȼ���������ͬ���е��쳣�����ȼ��������ͬ���ڵĴ�����
    ���������ȼ��ʹ����ȼ��Ľ�Ͼ���ͨ����˵�����ȼ���
    ����ķ�����˼���Ƿ���ռ�ȵĿ����Ǽ���,�����ȼ������Ǽ���,������һ����
  M3��ͨ���ı�CPU�ĵ�ǰ���ȼ��������ֹ�ж�.PRIMASKλ:ֻ����NMI��hard faulty
  �쳣,�����ж�/�쳣��������(��ǰCPU���ȼ�=0).
  */
  /* Configure the Priority Group to 2 bits */
   //-0           7.1��ʾ 7 λ��ռʽ���ȼ���1 λ�����ȼ�
   //-1           6.2��ʾ 6 λ��ռʽ���ȼ���2 λ�����ȼ�
   //-2           5.3��ʾ 5 λ��ռʽ���ȼ���3 λ�����ȼ�
   //-3           4.4��ʾ 4 λ��ռʽ���ȼ���4 λ�����ȼ�
   //-4           3.5��ʾ 3 λ��ռʽ���ȼ���5 λ�����ȼ�
   //-5           2.6��ʾ 2 λ��ռʽ���ȼ���6 λ�����ȼ�
   //-6           1.7��ʾ 1 λ��ռʽ���ȼ���7 λ�����ȼ�
   //-7           0.8��ʾ 0 λ��ռʽ���ȼ���8 λ�����ȼ�
   //-�ж����ȼ��Ĵ�����8λ���,ÿ���ж�һ�����ȼ��Ĵ���,�����Ƕ���8λ�ֵ�
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //-Ϊ�˸���ռʽ���ȼ�����Ӧ���ȼ����ж����ȼ��Ĵ����ĸ���λ����������ȼ�������ռ��λ��
  //-������Ϊ���������ȼ�˵����λ,�����Ǹ���Щλ��ֵ
  /* enabling interrupt */
  NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //-�����ⲿ�ж�,˫���ȼ�����
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQChannel;          //ָ���ж�Դ
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //ָ����Ӧ���ȼ���1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        //ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQChannel;          //ָ���ж�Դ
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        //ָ����Ӧ���ȼ���1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        //ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);

  //-cheng
  /* Configure the Priority Group to 2 bits */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//-NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;
	//-NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =3;
	//-NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	//-NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//-NVIC_Init(&NVIC_InitStructure);
  //-USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  //-USART_Cmd(USART3, ENABLE);		//-�ж����ú���,��Ӧ������ʹ������,���кܶ����û������

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  //-DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);
  //-DMA_Cmd(DMA1_Channel1, ENABLE);

  /* Configure the SysTick handler priority */
  NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick, 1, 0);  //-���ȼ������˲���һ�������˱��ж�(SystemHandler_SysTick, 1, 0)
}


/*******************************************************************************
* Function Name  : SysTick_Config
* Description    : Configure a SysTick Base time to 1 ms.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Config(void)
{
  /* Configure HCLK clock as SysTick clock source */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);  //-����ϵͳʱ��core clock��Ϊ������λ

  /* SysTick interrupt each 1000 Hz with HCLK equal to 72MHz */
  //SysTick_SetReload(72000);  //-����ȫ����1S,��ô����1000����0.001S
  SysTick_SetReload(56000);  //-����ȫ����1S,��ô����1000����0.001S
  //-SysTick_SetReload(56000*20);  //-����ȫ����1S,��ô����1000����0.001S,*20 ����20mS

  /* Enable the SysTick Interrupt */
  SysTick_ITConfig(ENABLE); //-���Ƕ�ʹ��λ��һ

  /* Enable the SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Enable); //-��������ʼ����
}


//-�����жϵ�����
void commonIT_Config(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  //-TIM_OCInitTypeDef  TIM_OCInitStructure;

	//-�����ⲿ�ж�

  //-����2�ж� PB12	I2C2_SMBA	INT2
  //-EXTI_ClearITPendingBit(EXTI_Line12);
  //-GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
  //-//- Configure EXTI Line12 to generate an interrupt on falling edge
  //-EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  //-EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  //-//-EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;		//-������
  //-EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;		//-�½���
  //-EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  //-EXTI_Init(&EXTI_InitStructure);

  //-���ж� PB13	I2C2_SMBA	INT3
  EXTI_ClearITPendingBit(EXTI_Line13);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
  //- Configure EXTI Line12 to generate an interrupt on falling edge
  EXTI_InitStructure.EXTI_Line = EXTI_Line13;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  //-EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;		//-������
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;		//-�½���
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);


  //-����T2��������
  //-TIM2 Configuration: Output Compare Timing Mode:
  //-                 TIM2CLK = 36 *2 =72MHz, Prescaler = 17, TIM2 counter clock = 4 MHz
  //-                 TIM2 update Period = ARR / TIM2 counter Period = 2 ms
  //-                 CC1 OC period = 1ms
  //-ϵͳʱ����56MHz,TIM2���ڵ���ʱ��(APB1)����,Ȼ��������2��Ƶ���Ա�Ϊ28MHz,(���¿�֪ʵ��Ӳ��������56MHz)Ȼ��ʱ�Ӽ���
  //-����28��Ƶ�趨��,TIM2�ļ���ʱ��Ƶ�ʾͱ�Ϊ��1MHz,��ô����2000�ξ���1mS
  //-APB1���������Ƶ����36MHz
  //-��ʱ��ʱ��Ƶ�ʷ�����Ӳ��������2������Զ��趨:
  //-1 �����Ӧ��APBԤ��Ƶϵ����1,��ʱ����ʱ��Ƶ�������ڵ�APB����Ƶ��һ��
  //-2.����,��ʱ����ʱ��Ƶ�ʱ��趨Ϊ����������APB����Ƶ�ʵ�2��.
  //- Time base configuration
  TIM_TimeBaseStructure.TIM_Period = 1000;		//-2000*5����һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
  TIM_TimeBaseStructure.TIM_Prescaler = 55;		//-27������Ϊ TIMx ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ,,��������ʱ��Ƶ��CK_CNT����fCK_PSC/(PSC[15:0]+1)��
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;		//-������ʱ�ӷָ�,,00�� tDTS = tCK_INT
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;		//-ѡ���˼�����ģʽ,,00�����ض���ģʽ�����������ݷ���λ(DIR)���ϻ����¼�����
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  TIM_ARRPreloadConfig(TIM2,ENABLE); //-ÿ�������к����ɵ�����,������������,,ʹ�ܻ���ʧ�� TIMx �� ARR(�Զ�װ�ؼĴ���) �ϵ�Ԥװ�ؼĴ���
  // only counter overflow/underflow generate U interrupt
  TIM_UpdateRequestConfig(TIM2,TIM_UpdateSource_Global);	//-���� TIMx ��������Դ,1�����ʹ���˸����жϻ�DMA������ֻ�м��������/����Ų��������жϻ�DMA����

  // TIM IT enable
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //-���ж�ʹ��λ��������,���£��������������/�����������������ʼ��(ͨ����������ڲ�/�ⲿ����)

  // TIM2 enable counter
  TIM_Cmd(TIM2, ENABLE);  //-��ʼ����
  //-��ʱ��4����
  TIM_TimeBaseStructure.TIM_Period = 80;		//-����һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ130mS�����ж�һ��
  TIM_TimeBaseStructure.TIM_Prescaler = 55999;		//-������Ϊ TIMx ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ,,��������ʱ��Ƶ��CK_CNT����fCK_PSC/(PSC[15:0]+1)��
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;		//-������ʱ�ӷָ�,,00�� tDTS = tCK_INT
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;		//-ѡ���˼�����ģʽ,,00�����ض���ģʽ�����������ݷ���λ(DIR)���ϻ����¼�����
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  TIM_ARRPreloadConfig(TIM4,ENABLE); //-ÿ�������к����ɵ�����,������������,,ʹ�ܻ���ʧ�� TIMx �� ARR(�Զ�װ�ؼĴ���) �ϵ�Ԥװ�ؼĴ���
  // only counter overflow/underflow generate U interrupt
  TIM_UpdateRequestConfig(TIM4,TIM_UpdateSource_Global);	//-���� TIMx ��������Դ,1�����ʹ���˸����жϻ�DMA������ֻ�м��������/����Ų��������жϻ�DMA����

  // TIM IT enable
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //-���ж�ʹ��λ��������,���£��������������/�����������������ʼ��(ͨ����������ڲ�/�ⲿ����)

  // TIM4 enable counter
  TIM_Cmd(TIM4, ENABLE);  //-��ʼ����
}

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
  /* USARTy TX DMA1 Channel (triggered by USARTy Tx event) Config */
  DMA_DeInit(DMA1_Channel4);  //-���ǰ����еĳ�ʼ��ΪĬ��ֵ
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Address;		//-�����ַ(USART1):0x4001 3800 - 0x4001 3BFF + 0x04(�Ĵ���ƫ�Ƶ�ַ) = ������Ե�ַ
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
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);

  /* USARTy RX DMA1 Channel (triggered by USARTy Rx event) Config */
  DMA_DeInit(DMA1_Channel5);
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (UINT32)&port_recv[0][0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 512;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//-��Ҫ����ѭ��,��DMA�Զ�ȫ������
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);

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
  /* Configure USART1 */
  USART_Init(USART1, &USART_InitStructure);
	//-���ʹ���жϷ�ʽ,������Ҫ��������
  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);  //-ʹ����������
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{ //-����0˵�����ͻ�������������û�з��ͳ�ȥ,������Ҫ�ȴ�ֱ�����Ϳ�
	}


	/* Enable USARTy DMA TX request */
  USART_DMACmd(USART1, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);

  //-ͨ��ʹ�� ����
  DMA_Cmd(DMA1_Channel5, ENABLE);
  /* Enable USARTy DMA TX Channel */
  DMA_Cmd(DMA1_Channel4, ENABLE);


  /* USARTx configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure);
  /* Enable the USART2 */
  USART_Cmd(USART2, ENABLE);  //-ʹ����������
  while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
	{ //-����0˵�����ͻ�������������û�з��ͳ�ȥ,������Ҫ�ȴ�ֱ�����Ϳ�
	}

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
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
  /*�Ҳ�ADC1��ADC2��������ܾ����������ñ����л�*/
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  //-����ģʽ
 // ADC_InitStructure.ADC_ScanConvMode = ENABLE;  //-ʹ��ɨ��ģʽ
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;  //
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  //-ת������������ֱ����λ�����
 // ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; //-����ת��ģʽ
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //-���������λ����ת��
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  //-�����Ҷ���
  ADC_InitStructure.ADC_NbrOfChannel = 1; //-�����ڹ���ͨ��ת�������е�ͨ����ĿΪ1
  //ADC_InitStructure.ADC_NbrOfChannel = 7;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel10 configuration PIN15*///-ͨ������ĺ�������ȷ��ת��˳��͸���
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_1Cycles5); //-��ת��ʱ��TCONV= 1.5 + 12.5 = 14 ���� = 1 �� s


  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE); //-ʹ��DMAģʽ,���ܵ����֮��ͻ��Զ������������ź�
// ADC_DMACmd(ADC1, DISABLE);
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
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void TIM_config(void)
{
//	 TIM_TimeBaseInitTypeDef        TIM1_TimeBaseStructure;
//   TIM_OCInitTypeDef                  TIM_OCInitStructure;
//   TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
//   //-----------------------------------��ʱ��TIM1��������------------------------------------
//   TIM_DeInit(TIM1);//���½�TIM1��Ϊȱʡֵ
//   TIM_InternalClockConfig(TIM1);//�����ڲ�ʱ�Ӹ�TIM1�ṩʱ��Դ ������8M
//   TIM1_TimeBaseStructure.TIM_Prescaler=8;//7��Ƶ,TIM1Ƶ��Ϊ56MHz/7=8M
//   TIM1_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���ģʽ
//   TIM1_TimeBaseStructure.TIM_Period=PWM_Period_Value-1;//���������С,ÿ��5000��������һ�������¼�,��PWM���Ƶ��1KHz
//   TIM1_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//ʱ�ӷָ�TDTS=Tck_tim
//   TIM1_TimeBaseStructure.TIM_RepetitionCounter=0x0;
//   TIM_TimeBaseInit(TIM1,&TIM1_TimeBaseStructure);
//   //-���ϼ���,û��ֵǰ�ο���ѹΪ��,����֮��Ϊ��
//   //-�����źŵ�����ǲο���ѹ�Ƚϵ�,�����������Ǻ�,��ô������Ͳο���ѹһ��,��һ���෴,ֻ�Ƕ���һ����ʱ
//   /* PWM1 Mode configuration: Channel1 */		//-GREEN
//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;		//-����û��������������Խ�ֹ���
//  TIM_OCInitStructure.TIM_Pulse = PWM_Period_Value;			//-�����ȸ�����ֵ
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//-��������ļ���:0�� OC1�ߵ�ƽ��Ч��1�� OC1�͵�ƽ��Ч��
//  //���漸�������Ǹ߼���ʱ���Ż��õ���ͨ�ö�ʱ����������
//  TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;  //���û������������
//	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;//ʹ�ܻ��������
//	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;  //���������״̬
//	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCIdleState_Set;//�����󻥲������״̬
//
//  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
//  //���岽��������ɲ���������ã��߼���ʱ�����еģ�ͨ�ö�ʱ����������
//	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Disable;//����ģʽ�����ѡ��
//	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;//����ģʽ�����ѡ��
//	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF; //��������
//	TIM_BDTRInitStructure.TIM_DeadTime = 0; //����ʱ������
//	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable; //ɲ������ʹ��
//	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;//ɲ�����뼫��
//	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;//�Զ����ʹ��
//	TIM_BDTRConfig(TIM1,&TIM_BDTRInitStructure);
//
//  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
//
//  /* PWM1 Mode configuration: Channel2 */ //-BLUE
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
//  TIM_OCInitStructure.TIM_Pulse = PWM_Period_Value;
//
//  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
//  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
//
//  /* PWM1 Mode configuration: Channel3 */ //-RED
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
//  //-��������ΪPWM_Period_Valueʱ,PWM����ֱ����� 1
//  //-��������Ϊ0ʱ,PWM����ֱ����� 0
//  TIM_OCInitStructure.TIM_Pulse = PWM_Period_Value;		//-���1 ����;���0 ���
//
//  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
//  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
//
//  TIM_ARRPreloadConfig(TIM1, ENABLE);
//  /* TIM1 enable counter */
//  TIM_Cmd(TIM1, ENABLE);
//  TIM_CtrlPWMOutputs(TIM1, ENABLE);		//-TIMx->BDTR
   //-��ʱ��3 Ϊ�����PWM����ʼ��
   TIM_TimeBaseInitTypeDef        TIM3_TimeBaseStructure;
   TIM_OCInitTypeDef                  TIM_OCInitStructure;
   //-TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
   //-----------------------------------��ʱ��TIM3��������------------------------------------
   TIM_DeInit(TIM3);//���½�TIM1��Ϊȱʡֵ
   TIM_InternalClockConfig(TIM3);//�����ڲ�ʱ�Ӹ�TIM3�ṩʱ��Դ ��8M
   TIM3_TimeBaseStructure.TIM_Prescaler=8;//7��Ƶ,TIM1Ƶ��Ϊ56MHz/7=8M
   TIM3_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���ģʽ
   TIM3_TimeBaseStructure.TIM_Period=PWM_Period_Value-1;//���������С,ÿ��5000��������һ�������¼�,��PWM���Ƶ��1KHz
   TIM3_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//ʱ�ӷָ�TDTS=Tck_tim
   TIM3_TimeBaseStructure.TIM_RepetitionCounter=0x0;
   TIM_TimeBaseInit(TIM3,&TIM3_TimeBaseStructure);
   //-���ϼ���,û��ֵǰ�ο���ѹΪ��,����֮��Ϊ��
   //-�����źŵ�����ǲο���ѹ�Ƚϵ�,�����������Ǻ�,��ô������Ͳο���ѹһ��,��һ���෴,ֻ�Ƕ���һ����ʱ
   /* PWM1 Mode configuration: Channel1 */		//-RED
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//-����û��������������Խ�ֹ���
  TIM_OCInitStructure.TIM_Pulse = 0;			//-�����ȸ�����ֵ
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;		//-��������ļ���:0�� OC1�ߵ�ƽ��Ч��1�� OC1�͵�ƽ��Ч��
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */ //-GREEN
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */ //-BLUE
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  //-��������ΪPWM_Period_Valueʱ,PWM����ֱ����� 1
  //-��������Ϊ0ʱ,PWM����ֱ����� 0
  TIM_OCInitStructure.TIM_Pulse = 0;		//-PWM_Period_Value ���1 ���;0 ���0 ����
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);


  TIM_ARRPreloadConfig(TIM3, ENABLE);
  /* TIM1 enable counter */
  TIM_Cmd(TIM3, ENABLE);
  //-TIM_CtrlPWMOutputs(TIM3, ENABLE);		//-TIMx->BDTR ,,����Ǹ߼���ʱ�����еģ����pwm�����

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
  IWDG_SetReload(4000);	//-���������ֵ�����˿��Ź���λʱ��ÿ�ε�ʱ����0.8mS,�ܴ���������ʱ��:100*0.8=80mS,��ô�����ʱ���ڱ���ι��
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  //-IWDG_Enable();

}

