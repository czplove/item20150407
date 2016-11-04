/*
adxl345�����������,���Բ����񶯺ͼ��ٶ�.
���жϷ�ʽ�Ͳ�ѯ���ַ�ʽ.
��������"����֮��"��ʵʱ��Ҫ���Ǻܸ�,��ô�����жϾ��Ƕ����Դ�ϵ�,��ô�Ϳ��Ծ���
ʹ���жϷ�ʽ,�������Լ�ܶ���Դ

������ʹ���û�ģʽ��ʵ�ֹ���.

ע:2015/7/23 18:49
��д�Ĵ����ſ�������ж�,����������ж�����α������,ͨ������ж�״̬���ŵ�״̬,���
����һ��ʱ��Ļ�,����Ϊû�гɹ���д(���ڲ�����״̬),��Ҫ�ָ���������.
*/
#include "user_conf.h"


extern void delay_temp(int j);
extern int i2c_read_reg(struct i2c_client *client,u8 reg,u8 *data);
extern int i2c_write_reg(struct i2c_client *client,u8 reg,u8 value);

#define Adxl_SlaveAddress   0xA6   //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�
                              //ALT  ADDRESS���Žӵ�ʱ��ַΪ0xA6���ӵ�Դʱ��ַΪ0x3A

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
//-�Ĵ�����ַƫ����
#define DEVICE_ID 									 0X00 //����ID,0XE5
#define THRESH_TAP                   0X1D //�û���ֵ
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



//-ADXL345��ʼ�����ú���
void ADXL345_init(void)
{
	 struct i2c_client client;
	 u8 temp_data8;
	 int res = 1;
        
	 client.addr = Adxl_SlaveAddress;
	 client.num = 1;		//-��ʾI2C5������
	 i2c_read_reg(&client,DEVICE_ID,&temp_data8);
	 
	 //-while(watch_data[1] != 0xE5);		//-����Ϊ��ֱ�ӿ�������,����������ѭ��,Ӧ��������
  STM32_UP_selfT_flag1 |= 0x01;

  if(temp_data8 == 0xE5) //��ȡ����ID
  {
  	  res=i2c_write_reg(&client,INT_ENABLE,0x00); 		//0x80 DATA_READY �жϿ���;0x40 �����ж� 
			if(res <= 0)
				return;	//-ʵ���в�����,��������һ�δ����ϵͳ����,��Ҫ���ݴ�����				
			i2c_read_reg(&client,INT_ENABLE,&temp_data8);	
			if(temp_data8 != 0x00)
				return;
			
			 //=====��ʼ��=====	
			res=i2c_write_reg(&client,DATA_FORMAT,0x2B);  //�ж���Чʱ����͵�ƽ,13λȫ�ֱ���,��������Ҷ���,16g����:λ5Ϊ0 �ж�ʱ���,Ϊ1 �ж�ʱ���; 
			if(res <= 0)
				return;
			i2c_read_reg(&client,DATA_FORMAT,&temp_data8);	
			if(temp_data8 != 0x2B)
				return;
					
			res=i2c_write_reg(&client,BW_RATE,0x0A); 			//��������ٶ�Ϊ100Hz �ο�pdf13ҳ,���ﲻһ��ָ�ľ���I2C��
			if(res <= 0)
				return;
			i2c_read_reg(&client,BW_RATE,&temp_data8);	
			if(temp_data8 != 0x0A)
				return;	
			//-res=i2c_write_reg(&client,POWER_CTL,0x08); 		//����ʹ��,����ģʽ ѡ���Դģʽ   �ο�pdf24ҳ
			//-if(res <= 0)
			//-	return;
			
			
			//-DATA_READY	���ݼĴ����е�������ٶ������ѱ�����
			//-SINGLE_TAP ����
			//-DOUBLE_TAP ˫��
			//-Activity � ����һ���ֵ���ڼ��ٶ�ֵ
			//-Inactivity
			//-FREE_FALL
			//-Watermark	ˮӡ
			//-Overrun	��������ж�
			
			//-�û����ж�����,���ȿ��Խ����ж�,Ȼ��ͨ����ȡINT_SOURCE����ж��ź�,FIFOֻ��ͨ��ʵ�������ʧ�Զ�����ж�			
			res=i2c_write_reg(&client,THRESH_TAP,ADXL_THRESH_TAP);					//X ƫ���� ���ݲ��Դ�������״̬д��pdf29ҳ  0X00 (15.6mg/LSB)
			if(res <= 0)
				return;
			i2c_read_reg(&client,THRESH_TAP,&temp_data8);	
			if(temp_data8 != ADXL_THRESH_TAP)
				return;	
					
			res=i2c_write_reg(&client,DUR,ADXL_DUR);					//������ֵ��ʱ��,������Ƿ�����Ч��,������߲���Ҫ��625 ��s/LSB,ʱ��С�����ֵ����
			if(res <= 0)
				return;
			i2c_read_reg(&client,DUR,&temp_data8);	
			if(temp_data8 != ADXL_DUR)
				return;	
					
			res=i2c_write_reg(&client,Latent,0x00);					//ֵΪ0����˫������
			if(res <= 0)
				return;	
			i2c_read_reg(&client,Latent,&temp_data8);	
			if(temp_data8 != 0x00)
				return;
					
			res=i2c_write_reg(&client,TAP_AXES,0x0F);					//����ʹ��λ����
			if(res <= 0)
				return;
			i2c_read_reg(&client,TAP_AXES,&temp_data8);	
			if(temp_data8 != 0x0F)
				return;		

			
			res=i2c_write_reg(&client,OFSX,0x00);					//X ƫ���� ���ݲ��Դ�������״̬д��pdf29ҳ  0X00 (15.6mg/LSB)
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
			
			//-�����жϵ�������Щ����,������Ҫ��������ֱ�־��ֹǰ���󴥷�
			res=i2c_write_reg(&client,INT_MAP,0x00); 			//���õ��ж�����1��
			if(res <= 0)
				return;
			i2c_read_reg(&client,INT_MAP,&temp_data8);	
			if(temp_data8 != 0x00)
				return;	
				
			res=i2c_write_reg(&client,POWER_CTL,0x08); 		//����ʹ��,����ģʽ ѡ���Դģʽ   �ο�pdf24ҳ
			if(res <= 0)
				return;
			i2c_read_reg(&client,POWER_CTL,&temp_data8);	
			if(temp_data8 != 0x08)
				return;	
				
			res=i2c_write_reg(&client,INT_ENABLE,0x40); 		//0x80 DATA_READY �жϿ���;0x40 �����ж� 
			if(res <= 0)
				return;
			i2c_read_reg(&client,INT_ENABLE,&temp_data8);	
			if(temp_data8 != 0x40)
				return;	
			
			//GPIO_ResetBits(GPIOA,GPIO_Pin_3); //�̵���,��ʾУ׼��
			//-ADXL345_AUTO_Adjust((char*)&x,(char*)&y,(char*)&z); //�Զ�У׼
			//GPIO_SetBits(GPIOA,GPIO_Pin_3); //�̵���,��ʾУ׼��� 
			
			//-ADXL345_Read_Angle(); //��һ��ADX345����ֹУ׼���һ�ζ����ݴ��� 
			STM32_UP_selfT_flag1 &= 0xfe;
			
			//-�û�������ʼ��
      ADXL_TAP_off_flag = 0;
			return;
  }
  
}

//-ADXL345�ı��ж�����
int ADXL345_init_re(u8 data)
{
	 struct i2c_client client;
	 u16 temp_data16;
	 int res = 1;
        
	 client.addr = Adxl_SlaveAddress;
	 client.num = 1;		//-��ʾI2C5������
	 
	 STM32_UP_selfT_flag1 |= 0x01;
	 
	 res=i2c_write_reg(&client,INT_ENABLE,0x00); 		//0x80 DATA_READY �жϿ���;0x40 �����ж� 
	 if(res <= 0)
			return 0;
  	  						
			temp_data16 = ADXL_THRESH_TAP+data;
			if((temp_data16 > 255) || (temp_data16 < ADXL_THRESH_TAP))		//?��Ҫ���⴦��
			   data = 255;
			else
				 data = ADXL_THRESH_TAP+data;   
			//-�û����ж�����,���ȿ��Խ����ж�,Ȼ��ͨ����ȡINT_SOURCE����ж��ź�,FIFOֻ��ͨ��ʵ�������ʧ�Զ�����ж�			
			res=i2c_write_reg(&client,THRESH_TAP, data);					//X ƫ���� ���ݲ��Դ�������״̬д��pdf29ҳ  0X00 (15.6mg/LSB)
			if(res <= 0)
				return 0;
			//-res=i2c_write_reg(&client,DUR,ADXL_DUR);					//������ֵ��ʱ��,������Ƿ�����Ч��,������߲���Ҫ��625 ��s/LSB,ʱ��С�����ֵ����
			//-if(res <= 0)
			//-	return 0;			
						
			res=i2c_write_reg(&client,INT_ENABLE,0x40); 		//0x80 DATA_READY �жϿ���;0x40 �����ж� 
			if(res <= 0)
				return 0;
	
	STM32_UP_selfT_flag1 &= 0xfe;		
  //-�û�������ʼ��
  ADXL_TAP_off_flag = 0;
  return 1;
}



//-Ϊ�������Ϊ��Ӧ�ٶ�,һ���жϲ���.������Ϊ�û���Ч,�������ں���һ��ʱ�����ж�ʧЧ����
void ADXL345_sub_it(void)	//-�ж��ڴ�����,��Ҫ������
{
	 //-struct i2c_client client;
	 //-u8 temp_data8;

	
	//-client.addr = Adxl_SlaveAddress;
	//-client.num = 1;		//-��ʾI2C1������
	//-i2c_read_reg(&client,DEVICE_ID,&watch_data[0]);
	//-while(watch_data[0] != 0xE5);
		
  //-�����Ҿ������������Ҫ���Ƿ���,һ�������û����ô�������ж�
  //-i2c_read_reg(&client,INT_SOURCE,&watch_data[29]);		//-�ж�Դ,��ȡ���ݿ�������ж��ź�
  if((ADXL_TAP_it_flag == 0) && (ADXL_TAP_off_flag == 0) && (led_display_flag != 0))
  {
	  //-�û���һҳ��ʾ
	  //-if(led_display_start == 0x55)		//-�豸ID��:CO2ֵ
	  //-{
		//-   ps_flag_led = 1;
	  //-}	   
	  //-led_disp_L_init();
	  
	  HL_new_value_flag = 1;	//-�л���ģʽ����Ҫ������ֵ
	  HL_flag++;
	  if(HL_flag > 5)
	  {
	  	 HL_RED_pwmval = 0;
		 	 HL_GREEN_pwmval = 0;
		 	 HL_BLUE_pwmval = 0;
		 	 HL_new_value_flag = 1;
	  	 HL_flag = 0;	  	 
	  }	
    
    UART1_transmit_flag = YES;		//-������֯���ݷ���
	  UART1_transmit_control = 7;		//-�������������
	  	 
    HL_run_time = Time_2048ms_Counter;		//-ÿ���û��ɹ���,�����¼�ʱ�ر�ʱ��
	  //-ADI_channels_wait_time = Time_2048ms_Counter;
    //-ADI_channels_init_flag = 0x55;    //-��Ҫ����У׼
  }
  //-EEP_Data++;
  ADXL_TAP_it_flag = 1;	//-�����ж���
  ADXL_TAP_wait_time = cticks_ms;
}



























