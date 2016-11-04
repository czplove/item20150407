/*
adxl345是三轴加速器,可以测量振动和加速度.
有中断方式和查询两种方式.
鉴于现在"梦想之花"对实时性要求不是很高,那么所有中断就是都可以打断的,那么就可以尽量
使用中断方式,这样会节约很多资源

现在先使用敲击模式来实现功能.

注:2015/7/23 18:49
读写寄存器才可以清除中断,如果长期在中断中如何避免错误,通过检查中断状态引脚的状态,如果
持续一段时间的话,就认为没有成功读写(处于不正常状态),如要恢复正常操作.
*/
#include "user_conf.h"


extern void delay_temp(int j);
extern int i2c_read_reg(struct i2c_client *client,u8 reg,u8 *data);
extern int i2c_write_reg(struct i2c_client *client,u8 reg,u8 value);

#define Adxl_SlaveAddress   0xA6   //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改
                              //ALT  ADDRESS引脚接地时地址为0xA6，接电源时地址为0x3A

//-B.	Marco Definition
#define I_AM_ADXL345      ((uint8_t)0xD4)


#define ADXL345_MODE_POWERDOWN       ((uint8_t)0x00)
#define ADXL345_MODE_ACTIVE          ((uint8_t)0x08)

#define ADXL345_OUTPUT_DATARATE_1    ((uint8_t)0x00)
#define ADXL345_OUTPUT_DATARATE_2    ((uint8_t)0x40)
#define ADXL345_OUTPUT_DATARATE_3    ((uint8_t)0x80)
#define ADXL345_OUTPUT_DATARATE_4    ((uint8_t)0xC0)

#define ADXL345_X_ENABLE            ((uint8_t)0x02)
#define ADXL345_Y_ENABLE            ((uint8_t)0x01)
#define ADXL345_Z_ENABLE            ((uint8_t)0x04)
#define ADXL345_AXES_ENABLE         ((uint8_t)0x07)
#define ADXL345_AXES_DISABLE        ((uint8_t)0x00)

#define ADXL345_BANDWIDTH_1         ((uint8_t)0x00)
#define ADXL345_BANDWIDTH_2         ((uint8_t)0x10)
#define ADXL345_BANDWIDTH_3         ((uint8_t)0x20)
#define ADXL345_BANDWIDTH_4         ((uint8_t)0x30)

#define ADXL345_FULLSCALE_250               ((uint8_t)0x00)
#define ADXL345_FULLSCALE_500               ((uint8_t)0x10)
#define ADXL345_FULLSCALE_2000              ((uint8_t)0x20)
 
#define ADXL345_BlockDataUpdate_Continous   ((uint8_t)0x00)
#define ADXL345_BlockDataUpdate_Single      ((uint8_t)0x80)
 
#define ADXL345_BLE_LSB                     ((uint8_t)0x00)
#define ADXL345_BLE_MSB                     ((uint8_t)0x40)
  
#define ADXL345_HIGHPASSFILTER_DISABLE      ((uint8_t)0x00)
#define ADXL345_HIGHPASSFILTER_ENABLE       ((uint8_t)0x10)
  
#define ADXL345_INT1INTERRUPT_DISABLE       ((uint8_t)0x00)
#define ADXL345_INT1INTERRUPT_ENABLE        ((uint8_t)0x80)
  
#define ADXL345_INT2INTERRUPT_DISABLE       ((uint8_t)0x00)
#define ADXL345_INT2INTERRUPT_ENABLE        ((uint8_t)0x08)

#define ADXL345_INT1INTERRUPT_LOW_EDGE      ((uint8_t)0x20)
#define ADXL345_INT1INTERRUPT_HIGH_EDGE     ((uint8_t)0x00)

#define ADXL345_BOOT_NORMALMODE             ((uint8_t)0x00)
#define ADXL345_BOOT_REBOOTMEMORY           ((uint8_t)0x80)
  
#define ADXL345_HPM_NORMAL_MODE_RES         ((uint8_t)0x00)
#define ADXL345_HPM_REF_SIGNAL              ((uint8_t)0x10)
#define ADXL345_HPM_NORMAL_MODE             ((uint8_t)0x20)
#define ADXL345_HPM_AUTORESET_INT           ((uint8_t)0x30)

#define ADXL345_HPFCF_0              0x00
#define ADXL345_HPFCF_1              0x01
#define ADXL345_HPFCF_2              0x02
#define ADXL345_HPFCF_3              0x03
#define ADXL345_HPFCF_4              0x04
#define ADXL345_HPFCF_5              0x05
#define ADXL345_HPFCF_6              0x06
#define ADXL345_HPFCF_7              0x07
#define ADXL345_HPFCF_8              0x08
#define ADXL345_HPFCF_9              0x09
//-寄存器地址偏移量
#define DEVICE_ID 									 0X00 //器件ID,0XE5
#define THRESH_TAP                   0X1D //敲击阀值
#define OFSX                         0X1E
#define OFSY                         0X1F
#define OFSZ                         0X20
#define DUR                          0X21
#define Latent                       0X22
#define Window                       0X23 
#define THRESH_ACT                   0X24
#define THRESH_INACT                 0X25 
#define TIME_INACT                   0X26
#define ACT_INACT_CTL                0X27 
#define THRESH_FF                    0X28 
#define TIME_FF                      0X29 
#define TAP_AXES                     0X2A 
#define ACT_TAP_STATUS               0X2B 
#define BW_RATE                      0X2C 
#define POWER_CTL                    0X2D 
#define INT_ENABLE                   0X2E
#define INT_MAP                      0X2F
#define INT_SOURCE                   0X30
#define DATA_FORMAT                  0X31
#define DATA_X0                      0X32
#define DATA_X1                      0X33
#define DATA_Y0                      0X34
#define DATA_Y1                      0X35
#define DATA_Z0                      0X36
#define DATA_Z1                      0X37
#define FIFO_CTL                     0X38
#define FIFO_STATUS                  0X39
#define ADXL_READ                    0X3B
#define ADXL_WRITE                   0X3A 



//-ADXL345初始化配置函数
void ADXL345_init(void)
{
	 struct i2c_client client;
	 u8 temp_data8;
	 int res = 1;
        
	 client.addr = Adxl_SlaveAddress;
	 client.num = 1;		//-表示I2C5口驱动
	 i2c_read_reg(&client,DEVICE_ID,&temp_data8);
	 
	 //-while(watch_data[1] != 0xE5);		//-测试为了直接看到现象,所以增加死循环,应用中慎用
  STM32_UP_selfT_flag1 |= 0x01;

  if(temp_data8 == 0xE5) //读取器件ID
  {
  	  res=i2c_write_reg(&client,INT_ENABLE,0x00); 		//0x80 DATA_READY 中断开启;0x40 单击中断 
			if(res <= 0)
				return;	//-实际中不能有,不能由于一次错误而系统崩溃,需要有容错能力				
			i2c_read_reg(&client,INT_ENABLE,&temp_data8);	
			if(temp_data8 != 0x00)
				return;
			
			 //=====初始化=====	
			res=i2c_write_reg(&client,DATA_FORMAT,0x2B);  //中断有效时输出低电平,13位全分辨率,输出数据右对齐,16g量程:位5为0 中断时变高,为1 中断时变低; 
			if(res <= 0)
				return;
			i2c_read_reg(&client,DATA_FORMAT,&temp_data8);	
			if(temp_data8 != 0x2B)
				return;
					
			res=i2c_write_reg(&client,BW_RATE,0x0A); 			//数据输出速度为100Hz 参考pdf13页,这里不一定指的就是I2C的
			if(res <= 0)
				return;
			i2c_read_reg(&client,BW_RATE,&temp_data8);	
			if(temp_data8 != 0x0A)
				return;	
			//-res=i2c_write_reg(&client,POWER_CTL,0x08); 		//链接使能,测量模式 选择电源模式   参考pdf24页
			//-if(res <= 0)
			//-	return;
			
			
			//-DATA_READY	数据寄存器中的三轴加速度数据已被更新
			//-SINGLE_TAP 单击
			//-DOUBLE_TAP 双击
			//-Activity 活动 任意一轴的值大于加速度值
			//-Inactivity
			//-FREE_FALL
			//-Watermark	水印
			//-Overrun	数据溢出中断
			
			//-敲击的中断配置,首先可以进入中断,然后通过读取INT_SOURCE清除中断信号,FIFO只能通过实际情况消失自动清除中断			
			res=i2c_write_reg(&client,THRESH_TAP,ADXL_THRESH_TAP);					//X 偏移量 根据测试传感器的状态写入pdf29页  0X00 (15.6mg/LSB)
			if(res <= 0)
				return;
			i2c_read_reg(&client,THRESH_TAP,&temp_data8);	
			if(temp_data8 != ADXL_THRESH_TAP)
				return;	
					
			res=i2c_write_reg(&client,DUR,ADXL_DUR);					//超过阀值的时间,这里就是防抖的效果,所以外边不需要了625 μs/LSB,时间小于这个值才行
			if(res <= 0)
				return;
			i2c_read_reg(&client,DUR,&temp_data8);	
			if(temp_data8 != ADXL_DUR)
				return;	
					
			res=i2c_write_reg(&client,Latent,0x00);					//值为0禁用双击功能
			if(res <= 0)
				return;	
			i2c_read_reg(&client,Latent,&temp_data8);	
			if(temp_data8 != 0x00)
				return;
					
			res=i2c_write_reg(&client,TAP_AXES,0x0F);					//检测的使能位各轴
			if(res <= 0)
				return;
			i2c_read_reg(&client,TAP_AXES,&temp_data8);	
			if(temp_data8 != 0x0F)
				return;		

			
			res=i2c_write_reg(&client,OFSX,0x00);					//X 偏移量 根据测试传感器的状态写入pdf29页  0X00 (15.6mg/LSB)
			if(res <= 0)
				return;
			i2c_read_reg(&client,OFSX,&temp_data8);	
			if(temp_data8 != 0x00)
				return;
					
			res=i2c_write_reg(&client,OFSY,0x00);
			if(res <= 0)
				return;
			i2c_read_reg(&client,OFSY,&temp_data8);	
			if(temp_data8 != 0x00)
				return;
					
			res=i2c_write_reg(&client,OFSZ,0x00); 
			if(res <= 0)
				return;
			i2c_read_reg(&client,OFSZ,&temp_data8);	
			if(temp_data8 != 0x00)
				return;	
			
			//-下面中断的配置有些讲究,可能需要先清除各种标志防止前期误触发
			res=i2c_write_reg(&client,INT_MAP,0x00); 			//配置到中断引脚1上
			if(res <= 0)
				return;
			i2c_read_reg(&client,INT_MAP,&temp_data8);	
			if(temp_data8 != 0x00)
				return;	
				
			res=i2c_write_reg(&client,POWER_CTL,0x08); 		//链接使能,测量模式 选择电源模式   参考pdf24页
			if(res <= 0)
				return;
			i2c_read_reg(&client,POWER_CTL,&temp_data8);	
			if(temp_data8 != 0x08)
				return;	
				
			res=i2c_write_reg(&client,INT_ENABLE,0x40); 		//0x80 DATA_READY 中断开启;0x40 单击中断 
			if(res <= 0)
				return;
			i2c_read_reg(&client,INT_ENABLE,&temp_data8);	
			if(temp_data8 != 0x40)
				return;	
			
			//GPIO_ResetBits(GPIOA,GPIO_Pin_3); //绿灯亮,提示校准中
			//-ADXL345_AUTO_Adjust((char*)&x,(char*)&y,(char*)&z); //自动校准
			//GPIO_SetBits(GPIOA,GPIO_Pin_3); //绿灯灭,提示校准完成 
			
			//-ADXL345_Read_Angle(); //读一次ADX345，防止校准后第一次读数据错误 
			STM32_UP_selfT_flag1 &= 0xfe;
			
			//-敲击变量初始化
      ADXL_TAP_off_flag = 0;
			return;
  }
  
}

//-ADXL345改变中断配置
int ADXL345_init_re(u8 data)
{
	 struct i2c_client client;
	 u16 temp_data16;
	 int res = 1;
        
	 client.addr = Adxl_SlaveAddress;
	 client.num = 1;		//-表示I2C5口驱动
	 
	 STM32_UP_selfT_flag1 |= 0x01;
	 
	 res=i2c_write_reg(&client,INT_ENABLE,0x00); 		//0x80 DATA_READY 中断开启;0x40 单击中断 
	 if(res <= 0)
			return 0;
  	  						
			temp_data16 = ADXL_THRESH_TAP+data;
			if((temp_data16 > 255) || (temp_data16 < ADXL_THRESH_TAP))		//?需要特殊处理
			   data = 255;
			else
				 data = ADXL_THRESH_TAP+data;   
			//-敲击的中断配置,首先可以进入中断,然后通过读取INT_SOURCE清除中断信号,FIFO只能通过实际情况消失自动清除中断			
			res=i2c_write_reg(&client,THRESH_TAP, data);					//X 偏移量 根据测试传感器的状态写入pdf29页  0X00 (15.6mg/LSB)
			if(res <= 0)
				return 0;
			//-res=i2c_write_reg(&client,DUR,ADXL_DUR);					//超过阀值的时间,这里就是防抖的效果,所以外边不需要了625 μs/LSB,时间小于这个值才行
			//-if(res <= 0)
			//-	return 0;			
						
			res=i2c_write_reg(&client,INT_ENABLE,0x40); 		//0x80 DATA_READY 中断开启;0x40 单击中断 
			if(res <= 0)
				return 0;
	
	STM32_UP_selfT_flag1 &= 0xfe;		
  //-敲击变量初始化
  ADXL_TAP_off_flag = 0;
  return 1;
}



//-为了提高人为反应速度,一但中断产生.立即认为敲击有效,仅仅是在后续一段时间内中断失效而已
void ADXL345_sub_it(void)	//-中断内处理函数,需要尽量少
{
	 //-struct i2c_client client;
	 //-u8 temp_data8;

	
	//-client.addr = Adxl_SlaveAddress;
	//-client.num = 1;		//-表示I2C1口驱动
	//-i2c_read_reg(&client,DEVICE_ID,&watch_data[0]);
	//-while(watch_data[0] != 0xE5);
		
  //-但是我觉得这里可能需要考虑防抖,一次主观敲击不该触发多次中断
  //-i2c_read_reg(&client,INT_SOURCE,&watch_data[29]);		//-中断源,读取数据可以清除中断信号
  if((ADXL_TAP_it_flag == 0) && (ADXL_TAP_off_flag == 0) && (led_display_flag != 0))
  {
	  //-敲击翻一页显示
	  //-if(led_display_start == 0x55)		//-设备ID号:CO2值
	  //-{
		//-   ps_flag_led = 1;
	  //-}	   
	  //-led_disp_L_init();
	  
	  HL_new_value_flag = 1;	//-切换了模式就需要更新数值
	  HL_flag++;
	  if(HL_flag > 5)
	  {
	  	 HL_RED_pwmval = 0;
		 	 HL_GREEN_pwmval = 0;
		 	 HL_BLUE_pwmval = 0;
		 	 HL_new_value_flag = 1;
	  	 HL_flag = 0;	  	 
	  }	
    
    UART1_transmit_flag = YES;		//-可以组织内容发送
	  UART1_transmit_control = 7;		//-允许主板跑马灯
	  	 
    HL_run_time = Time_2048ms_Counter;		//-每次敲击成功了,就重新计时关闭时间
	  //-ADI_channels_wait_time = Time_2048ms_Counter;
    //-ADI_channels_init_flag = 0x55;    //-需要重新校准
  }
  //-EEP_Data++;
  ADXL_TAP_it_flag = 1;	//-处于中断中
  ADXL_TAP_wait_time = cticks_ms;
}



























