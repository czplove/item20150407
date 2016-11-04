/*
	对底层的支持
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
void RCC_Configuration(void)  //-本程序配置了所有可能使用到的功能时钟
{
  ErrorStatus HSEStartUpStatus;

  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);  //-使能外部高速晶振

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  //-上面的操作可能成功可能失败,如果失败了就不配置下面的了,而是直接使用的默认的
  if(HSEStartUpStatus == SUCCESS)
  {
    /* HCLK = SYSCLK */ //- AHB 预分频,AHB时钟的预分频系数为1
    RCC_HCLKConfig(RCC_SYSCLK_Div1);  //-当系统时钟确定之后,其它很多时钟可以独立配置

    /* PCLK2 = HCLK */  //-高速APB预分频(APB2),控制高速APB2时钟(PCLK2) 的预分频系数为1
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* PCLK1 = HCLK/2 */  //-低速APB预分频(APB1),控制低速APB1时钟(PCLK1) 的预分频系数为 2分频
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* ADCCLK = PCLK2/4 */  //-ADC预分频,ADC时钟频率分频后作为ADC时钟
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); //-ADC时钟为14 MHz

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);  //-number of wait states for a read operation programmed on-the-fly

    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* PLLCLK = 4MHz * 9 = 36 MHz */  //-HSE时钟作为PLL 输入时钟,PLL 7倍频输出
    //-RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);  //-本系统是8*9= 72 MHz
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_7);  //-本系统是8*7= 56 MHz

    /* Enable PLL */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)  //-这里可能出现死循环
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);  //-仅仅就是选择了PLL作为系统时钟

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)  //-其实就是读取状态寄存器值
    {
    }

    //-增加看门狗的处理
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
  //-不同的外设挂在不同的总线上,现在的库函数不仅仅提供了编程思路,而且定义的参数通用性很好
  /* TIM2 clocks enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//使能TIM1时钟

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
  RCC_LSICmd(ENABLE);		//-the LSI OSC 是40KHz

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
STM32F103Rx     是64引脚的，引脚预定义如下：

--点阵屏
PC0	=	LD_A0
PC1	=	LD_A1
PC2	=	LD_A2
PC3	=	LD_A3
PA4	=	LD_LAT
PA5	=	LD_CLK
PA6	=	LD_OE
PA7	=	LD_SDI
PB0		LD_LE


--光照 振动
PB6	I2C1_SCL	I2C1_SCL
PB7	I2C1_SDA	I2C1_SDA
PB13	  I2C3_SMBA	INT1		--振动中断


--ADI手势
PB10	I2C2_SCL	I2C2_SCL
PB11	I2C2_SDA	I2C2_SDA
PB12  I2C2_SMBA	INT2		--手势中断2


--PWM输出呼吸灯
PC6	TIM3_CH1	PWM_R
PC7	TIM3_CH2	PWM_G
PC8	TIM3_CH3	PWM_B
PC9	        	LED_EN


--Communication to 主板于STM32的串口2通讯
PA9	  USART1_TX
PA10	USART1_RX


--测试引脚
PC12	I2C3	原来是作为模拟I2C使用的现在作为检测引脚读外部状态0 测试状态;1 运行状态

--debug 口
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
  GPIOA->BSRR = GPIO_Pin_6;		//-保证GPIO的初始状态
  GPIOA->BRR = GPIO_Pin_7;		//-保证GPIO的初始状态
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //- Configure PB.00 as output push-pull
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //- Configure PB.12, PB.13 as input push-pull ,ADI手势;振动中断
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //- Configure PC.10 as input push-pull ,三轴加速度中断
  //-GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 ;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  //-//-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//-测试总线没有问题,可以拉高
  //-//-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//-检查引脚电平状态
  //-//-GPIOC->BSRR = GPIO_Pin_10;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOC, &GPIO_InitStructure);

  //- Configure PA.00 as input push-pull ,触摸中断
  //-GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 ;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOA, &GPIO_InitStructure);

  //- Configure PC.09 as output push-pull
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIOC->BSRR = GPIO_Pin_9;   //-置高电平亮灯
  //-GPIOC->BRR = GPIO_Pin_9; //-置低电平
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  //-Configure PB.13 -- PB.15 as Output push-pull,控制PWM输出呼吸灯
  //-TIM1_CH1N TIM1_CH2N TIM1_CH3N
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				//-设置为复用浮空输出
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-GPIO_Init(GPIOB, &GPIO_InitStructure);
  //-Configure PC.6 -- PC.8 as Output push-pull,控制PWM输出呼吸灯
  //-TIM3_CH1 TIM3_CH2 TIM3_CH3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 |GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				//-设置为复用浮空输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
  //- 为了测试外部电路是否有问题,现在需要来给指定电平看看炫彩灯是否受控
  //-GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-//-引脚输出电平1 灯灭.
  //-GPIOC->BRR = GPIO_Pin_6;		//-RED保证GPIO的初始状态
  //-GPIOC->BSRR = GPIO_Pin_7;		//-GREEN保证GPIO的初始状态
  //-GPIOC->BSRR = GPIO_Pin_8;   //-BLUE
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

  //-Configure USART3 Rx (PC.11) as input floating,进行PM2.5通讯接收
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  //-GPIO_Init(GPIOC, &GPIO_InitStructure);

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
  //-Configure PC.4,5 (ADC Channel14,15) as analog input,进行PM2.5模拟信号的采集,还有噪声采集
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
  //-I2C1 PB6 SCK 推挽输出
  //-GPIO_Mode_Out_PP		推挽-输出
  //-GPIO_Mode_Out_OD		开漏-输出
  //-开漏输出就是不输出电压，低电平时接地，高电平时不接地。如果外接上拉电阻，则
  //-在输出高电平时电压会拉到上拉电阻的电源电压。这种方式适合在连接的外设电压比
  //-单片机电压低的时候。使用时要加上拉电阻。
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//-同一个总线上,仅仅主机控制时钟信号,所以这里仅仅输出就可以了
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOB->BSRR = GPIO_Pin_6;		//-RED保证GPIO的初始状态
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//-同一个总线上,仅仅主机控制时钟信号,所以这里仅仅输出就可以了
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOB->BSRR = GPIO_Pin_7;		//-RED保证GPIO的初始状态
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //-TWI_Delay();
  //-GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  //-GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  //-GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //-I2C1_SDA_H;
  //-GPIO_Init(GPIOB, &GPIO_InitStructure);

  //-Configure I2C2 pins: SCL and SDA
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//-同一个总线上,仅仅主机控制时钟信号,所以这里仅仅输出就可以了
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOB->BSRR = GPIO_Pin_10;		//-RED保证GPIO的初始状态
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  I2C2_SDA_H;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
/*
  //-I2C3 A8 C9
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//-同一个总线上,仅仅主机控制时钟信号,所以这里仅仅输出就可以了
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOA->BSRR = GPIO_Pin_8;		//-RED保证GPIO的初始状态
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  I2C3_SDA_H;
  GPIO_Init(GPIOC, &GPIO_InitStructure);		//-LED_EN

  //-I2C4 C11 C12
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//-同一个总线上,仅仅主机控制时钟信号,所以这里仅仅输出就可以了
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOC->BSRR = GPIO_Pin_11;		//-RED保证GPIO的初始状态
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  //-强制拉高防止钳位?
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//-同一个总线上,仅仅主机控制时钟信号,所以这里仅仅输出就可以了
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOC->BSRR = GPIO_Pin_12;		//-RED保证GPIO的初始状态
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  TWI_Delay();
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  I2C4_SDA_H;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //-I2C5 B15 A12
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//-同一个总线上,仅仅主机控制时钟信号,所以这里仅仅输出就可以了
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIOB->BSRR = GPIO_Pin_15;		//-RED保证GPIO的初始状态
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  I2C5_SDA_H;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
*/
#endif

//-测试用
//PC12  浮动输入，外部有上拉电阻
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
#else  /* VECT_TAB_FLASH  */  //-本系统在预编译里定义了这个变量
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); //?
#endif

  /*
  	异常优先级:
  	此芯片支持支持优先级分组机制(占先优先级区和次优先级区).
  	我们将占先优先级称为组优先级。如果有多个挂起异常共用相同的组优先级，
    则需使用次优先级区来决定同组中的异常的优先级，这就是同组内的次优先
    级。组优先级和次优先级的结合就是通常所说的优先级。
    下面的分组意思就是分配占先的可以是几层,次优先级可以是几层,总数是一定的
  M3是通过改变CPU的当前优先级来允许或静止中断.PRIMASK位:只允许NMI和hard faulty
  异常,其它中断/异常都被屏蔽(当前CPU优先级=0).
  */
  /* Configure the Priority Group to 2 bits */
   //-0           7.1表示 7 位抢占式优先级，1 位子优先级
   //-1           6.2表示 6 位抢占式优先级，2 位子优先级
   //-2           5.3表示 5 位抢占式优先级，3 位子优先级
   //-3           4.4表示 4 位抢占式优先级，4 位子优先级
   //-4           3.5表示 3 位抢占式优先级，5 位子优先级
   //-5           2.6表示 2 位抢占式优先级，6 位子优先级
   //-6           1.7表示 1 位抢占式优先级，7 位子优先级
   //-7           0.8表示 0 位抢占式优先级，8 位子优先级
   //-中断优先级寄存器由8位组成,每个中断一个优先级寄存器,下面是对这8位分点
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //-为了给抢占式优先级和响应优先级在中断优先级寄存器的高四位分配各个优先级数字所占的位数
  //-上面是为了配置优先级说明的位,下面是给这些位赋值
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

  //-设置外部中断,双优先级设置
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQChannel;          //指定中断源
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //指定响应优先级别1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        //使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQChannel;          //指定中断源
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        //指定响应优先级别1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        //使能外部中断通道
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
  //-USART_Cmd(USART3, ENABLE);		//-中断配置好了,不应该立即使能外设,还有很多参数没有配置

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  //-DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);
  //-DMA_Cmd(DMA1_Channel1, ENABLE);

  /* Configure the SysTick handler priority */
  NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick, 1, 0);  //-优先级配置了并不一定开设了本中断(SystemHandler_SysTick, 1, 0)
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
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);  //-采用系统时钟core clock作为计数单位

  /* SysTick interrupt each 1000 Hz with HCLK equal to 72MHz */
  //SysTick_SetReload(72000);  //-数满全部是1S,那么除以1000就是0.001S
  SysTick_SetReload(56000);  //-数满全部是1S,那么除以1000就是0.001S
  //-SysTick_SetReload(56000*20);  //-数满全部是1S,那么除以1000就是0.001S,*20 就是20mS

  /* Enable the SysTick Interrupt */
  SysTick_ITConfig(ENABLE); //-就是对使能位置一

  /* Enable the SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Enable); //-计数器开始计数
}


//-开放中断的配置
void commonIT_Config(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  //-TIM_OCInitTypeDef  TIM_OCInitStructure;

	//-设置外部中断

  //-手势2中断 PB12	I2C2_SMBA	INT2
  //-EXTI_ClearITPendingBit(EXTI_Line12);
  //-GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
  //-//- Configure EXTI Line12 to generate an interrupt on falling edge
  //-EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  //-EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  //-//-EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;		//-上升沿
  //-EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;		//-下降沿
  //-EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  //-EXTI_Init(&EXTI_InitStructure);

  //-振动中断 PB13	I2C2_SMBA	INT3
  EXTI_ClearITPendingBit(EXTI_Line13);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
  //- Configure EXTI Line12 to generate an interrupt on falling edge
  EXTI_InitStructure.EXTI_Line = EXTI_Line13;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  //-EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;		//-上升沿
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;		//-下降沿
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);


  //-开放T2参数配置
  //-TIM2 Configuration: Output Compare Timing Mode:
  //-                 TIM2CLK = 36 *2 =72MHz, Prescaler = 17, TIM2 counter clock = 4 MHz
  //-                 TIM2 update Period = ARR / TIM2 counter Period = 2 ms
  //-                 CC1 OC period = 1ms
  //-系统时钟是56MHz,TIM2挂在低速时钟(APB1)线上,然后设置了2分频所以变为28MHz,(由下可知实际硬件分配了56MHz)然后时钟计数
  //-经过28分频设定后,TIM2的计算时钟频率就变为了1MHz,那么计数2000次就是1mS
  //-APB1的最大允许频率是36MHz
  //-定时器时钟频率分配由硬件按以下2钟情况自动设定:
  //-1 如果相应的APB预分频系数是1,定时器的时钟频率与所在的APB总线频率一致
  //-2.否则,定时器的时钟频率被设定为与其相连的APB总线频率的2倍.
  //- Time base configuration
  TIM_TimeBaseStructure.TIM_Period = 1000;		//-2000*5在下一个更新事件装入活动的自动重装载寄存器周期的值
  TIM_TimeBaseStructure.TIM_Prescaler = 55;		//-27用来作为 TIMx 时钟频率除数的预分频值,,计数器的时钟频率CK_CNT等于fCK_PSC/(PSC[15:0]+1)。
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;		//-设置了时钟分割,,00： tDTS = tCK_INT
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;		//-选择了计数器模式,,00：边沿对齐模式。计数器依据方向位(DIR)向上或向下计数。
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  TIM_ARRPreloadConfig(TIM2,ENABLE); //-每个都具有很自由的配置,而不是做死的,,使能或者失能 TIMx 在 ARR(自动装载寄存器) 上的预装载寄存器
  // only counter overflow/underflow generate U interrupt
  TIM_UpdateRequestConfig(TIM2,TIM_UpdateSource_Global);	//-设置 TIMx 更新请求源,1：如果使能了更新中断或DMA请求，则只有计数器溢出/下溢才产生更新中断或DMA请求

  // TIM IT enable
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //-对中断使能位进行设置,更新：计数器向上溢出/向下溢出，计数器初始化(通过软件或者内部/外部触发)

  // TIM2 enable counter
  TIM_Cmd(TIM2, ENABLE);  //-开始计数
  //-定时器4设置
  TIM_TimeBaseStructure.TIM_Period = 80;		//-在下一个更新事件装入活动的自动重装载寄存器周期的值130mS进入中断一次
  TIM_TimeBaseStructure.TIM_Prescaler = 55999;		//-用来作为 TIMx 时钟频率除数的预分频值,,计数器的时钟频率CK_CNT等于fCK_PSC/(PSC[15:0]+1)。
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;		//-设置了时钟分割,,00： tDTS = tCK_INT
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;		//-选择了计数器模式,,00：边沿对齐模式。计数器依据方向位(DIR)向上或向下计数。
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  TIM_ARRPreloadConfig(TIM4,ENABLE); //-每个都具有很自由的配置,而不是做死的,,使能或者失能 TIMx 在 ARR(自动装载寄存器) 上的预装载寄存器
  // only counter overflow/underflow generate U interrupt
  TIM_UpdateRequestConfig(TIM4,TIM_UpdateSource_Global);	//-设置 TIMx 更新请求源,1：如果使能了更新中断或DMA请求，则只有计数器溢出/下溢才产生更新中断或DMA请求

  // TIM IT enable
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //-对中断使能位进行设置,更新：计数器向上溢出/向下溢出，计数器初始化(通过软件或者内部/外部触发)

  // TIM4 enable counter
  TIM_Cmd(TIM4, ENABLE);  //-开始计数
}

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
  /* USARTy TX DMA1 Channel (triggered by USARTy Tx event) Config */
  DMA_DeInit(DMA1_Channel4);  //-就是把所有的初始化为默认值
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Address;		//-外设地址(USART1):0x4001 3800 - 0x4001 3BFF + 0x04(寄存器偏移地址) = 外设绝对地址
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
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);

  /* USARTy RX DMA1 Channel (triggered by USARTy Rx event) Config */
  DMA_DeInit(DMA1_Channel5);
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (UINT32)&port_recv[0][0];
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
  USART_Cmd(USART2, ENABLE);  //-使能整个外设
  while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
	{ //-对于0说明发送缓冲区还有数据没有发送出去,所以需要等待直到发送空
	}

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
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
  /*我猜ADC1和ADC2的区别可能就是两套配置便于切换*/
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  //-独立模式
 // ADC_InitStructure.ADC_ScanConvMode = ENABLE;  //-使用扫描模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;  //
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  //-转换将连续进行直到该位被清除
 // ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; //-单次转换模式
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //-用软件控制位触发转换
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  //-数据右对齐
  ADC_InitStructure.ADC_NbrOfChannel = 1; //-定义在规则通道转换序列中的通道数目为1
  //ADC_InitStructure.ADC_NbrOfChannel = 7;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel10 configuration PIN15*///-通过下面的函数可以确定转换顺序和个数
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_1Cycles5); //-总转换时间TCONV= 1.5 + 12.5 = 14 周期 = 1 μ s


  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE); //-使用DMA模式,可能当完成之后就会自动发送请求处理信号
// ADC_DMACmd(ADC1, DISABLE);
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
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void TIM_config(void)
{
//	 TIM_TimeBaseInitTypeDef        TIM1_TimeBaseStructure;
//   TIM_OCInitTypeDef                  TIM_OCInitStructure;
//   TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
//   //-----------------------------------定时器TIM1基本配置------------------------------------
//   TIM_DeInit(TIM1);//重新将TIM1设为缺省值
//   TIM_InternalClockConfig(TIM1);//采用内部时钟给TIM1提供时钟源 假如是8M
//   TIM1_TimeBaseStructure.TIM_Prescaler=8;//7分频,TIM1频率为56MHz/7=8M
//   TIM1_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
//   TIM1_TimeBaseStructure.TIM_Period=PWM_Period_Value-1;//计数溢出大小,每计5000个数产生一个更新事件,即PWM输出频率1KHz
//   TIM1_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//时钟分割TDTS=Tck_tim
//   TIM1_TimeBaseStructure.TIM_RepetitionCounter=0x0;
//   TIM_TimeBaseInit(TIM1,&TIM1_TimeBaseStructure);
//   //-向上计数,没到值前参考电压为高,到了之后为低
//   //-互补信号的输出是参考电压比较的,如果输出极性是好,那么主输出和参考电压一样,另一个相反,只是都有一个延时
//   /* PWM1 Mode configuration: Channel1 */		//-GREEN
//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;		//-现在没有用这个引脚所以禁止输出
//  TIM_OCInitStructure.TIM_Pulse = PWM_Period_Value;			//-可以先给个初值
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//-决定输出的极性:0： OC1高电平有效；1： OC1低电平有效。
//  //下面几个参数是高级定时器才会用到，通用定时器不用配置
//  TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;  //设置互补端输出极性
//	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;//使能互补端输出
//	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;  //死区后输出状态
//	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCIdleState_Set;//死区后互补端输出状态
//
//  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
//  //第五步，死区和刹车功能配置，高级定时器才有的，通用定时器不用配置
//	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Disable;//运行模式下输出选择
//	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;//空闲模式下输出选择
//	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF; //锁定设置
//	TIM_BDTRInitStructure.TIM_DeadTime = 0; //死区时间设置
//	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable; //刹车功能使能
//	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;//刹车输入极性
//	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;//自动输出使能
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
//  //-脉宽设置为PWM_Period_Value时,PWM引脚直接输出 1
//  //-脉宽设置为0时,PWM引脚直接输出 0
//  TIM_OCInitStructure.TIM_Pulse = PWM_Period_Value;		//-输出1 亮灯;输出0 灭灯
//
//  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
//  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
//
//  TIM_ARRPreloadConfig(TIM1, ENABLE);
//  /* TIM1 enable counter */
//  TIM_Cmd(TIM1, ENABLE);
//  TIM_CtrlPWMOutputs(TIM1, ENABLE);		//-TIMx->BDTR
   //-定时器3 为了输出PWM而初始化
   TIM_TimeBaseInitTypeDef        TIM3_TimeBaseStructure;
   TIM_OCInitTypeDef                  TIM_OCInitStructure;
   //-TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
   //-----------------------------------定时器TIM3基本配置------------------------------------
   TIM_DeInit(TIM3);//重新将TIM1设为缺省值
   TIM_InternalClockConfig(TIM3);//采用内部时钟给TIM3提供时钟源 是8M
   TIM3_TimeBaseStructure.TIM_Prescaler=8;//7分频,TIM1频率为56MHz/7=8M
   TIM3_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
   TIM3_TimeBaseStructure.TIM_Period=PWM_Period_Value-1;//计数溢出大小,每计5000个数产生一个更新事件,即PWM输出频率1KHz
   TIM3_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//时钟分割TDTS=Tck_tim
   TIM3_TimeBaseStructure.TIM_RepetitionCounter=0x0;
   TIM_TimeBaseInit(TIM3,&TIM3_TimeBaseStructure);
   //-向上计数,没到值前参考电压为高,到了之后为低
   //-互补信号的输出是参考电压比较的,如果输出极性是好,那么主输出和参考电压一样,另一个相反,只是都有一个延时
   /* PWM1 Mode configuration: Channel1 */		//-RED
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//-现在没有用这个引脚所以禁止输出
  TIM_OCInitStructure.TIM_Pulse = 0;			//-可以先给个初值
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;		//-决定输出的极性:0： OC1高电平有效；1： OC1低电平有效。
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */ //-GREEN
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */ //-BLUE
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  //-脉宽设置为PWM_Period_Value时,PWM引脚直接输出 1
  //-脉宽设置为0时,PWM引脚直接输出 0
  TIM_OCInitStructure.TIM_Pulse = 0;		//-PWM_Period_Value 输出1 灭灯;0 输出0 亮灯
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);


  TIM_ARRPreloadConfig(TIM3, ENABLE);
  /* TIM1 enable counter */
  TIM_Cmd(TIM3, ENABLE);
  //-TIM_CtrlPWMOutputs(TIM3, ENABLE);		//-TIMx->BDTR ,,这句是高级定时器才有的，输出pwm必须打开

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
  IWDG_SetReload(4000);	//-这里的设置值决定了看门狗复位时间每次的时间是0.8mS,总次数将决定时长:100*0.8=80mS,那么在这个时间内必须喂狗
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  //-IWDG_Enable();

}

