#include "user_conf.h"

#define  VOC_I2C_addr 		0xE0


u8  CMD_Set_ppmC02[3] = {0x08,0x01,0x90};
u8  CMD_Get_status[3] = {0x09,0x00,0x00};


UINT16 voc_value[16];
UINT16 voc_value_pt=0;

UINT16 co2_value[16];
UINT16 co2_value_pt=0;

int RS_value_rd;
UINT16 R0_value_rd;
u8  VOC_Status;
u8  VOC_HD_type=0;	//-不同VOC模块对应不同类型号,为查询到合适类型号时为0
u8  VOC_HD_type_y=0;  //-同一个地址下的子型号
u8  VOC_HD_type_cn=0;
/*
Like the VZ89 refresh its output once every seconds, please don't request VZ89 at a frequency > 1Hz.
*/


/******************************************************************************* * Function: _getCRC *
* *
* Description: *
* This function process and return the CRC *
* *
* Input parameters: *
* #1 Type of CRC {CCRC::Classic CRC; ECRC::Enhanced CRC} *
* #2 Data buffer pointer *
* #3 Data buffer size *
* *
* Returns: *
* CRC value *
*******************************************************************************/
UINT8 crc_getCrc(UINT8 *data, UINT8 size/*, byte crc_type*/) {
//--------------------------------------------------------------------
// Local variable
//--------------------------------------------------------------------
UINT8 crc = 0x00;
UINT8 i = 0x00;
UINT16 sum = 0x0000;
//--------------------------------------------------------------------
// Checking CRC type
//--------------------------------------------------------------------
//-if (crc_type == ECRC) crc = PID;
//--------------------------------------------------------------------
// Summation loop
//--------------------------------------------------------------------
for(i=0; i < size; i++) {
sum = crc + data[i];
crc = (UINT8)sum;
crc += (sum/0x100);
}// end loop
crc = 0xFF-crc; // complement
return(crc);
}//end Method


//-Set ppmC02  This command is used to send a ppmCO2 value from an analyser to the VZ89 in order to recalibrate its outputs.
//-0x08
//?给什么值,
/*
发送:
D1: Address [7bits] = 1110000 and R/W [1bit] = 0
D2: Command = 00001000
D3: Data MSB = 01010101
D4: Data MSB = 01010101
*/

u16 VZ89_Set_ppmC02(void)	//-作为一个初始条件,初始化VOC
{
	  struct i2c_client client;
	  int res;

	  client.addr = VOC_I2C_addr;                   // 将从机地址写入数据包pa12201001的地址就是0x3C
    client.flags = 0;
    client.num = 1;		//-硬件端口号1,I2C1

	  res = i2c_write_reg_Buffer(&client, CMD_Set_ppmC02, 3);
	  if(res <= 0)
	  {//-读值成功的话就继续判断
	 		return res;
		}

		return res;
}

//-Get VZ89 status This command is used to read the VZ89 status coded with 6 bytes
//-Note: the raw sensor resistor value [W] = 10*(D4 + (256*D5) + (65536*D6))
/*
发送:
D1: Address [7bits] = 1110000 and R/W [1bit] = 0
D2: Command = 00001001
D3: Data MSB = 01010101
D4: Data MSB = 01010101


接收:
D1: Address [7bits] = 1110000 and R/W [1bit] = 1

D1 (8bits) represent the CO2-equivalent signal value [13..242].
D2 (8bits) represent the VOC-short signal value [13..242].
D3 (8bits) represent the VOC-ppb signal value [13..242].
D4 (8bits) represent the 1st byte of raw sensor resistor value (LSB).
D5 (8bits) represent the 2nd byte of raw sensor resistor value.
D6 (8bits) represent the 3rd byte of raw sensor resistor value (MSB).
*/

#if 0
void VZ89_Read_VOC(void)
{
	  struct i2c_client client;
	  int res;
	  u8 value[8],i;
	  u16 temp_data;

	  client.addr = VOC_I2C_addr;                   // 将从机地址写入数据包pa12201001的地址就是0x3C
    client.flags = 0;
    client.num = 1;		//-硬件端口号1,I2C1

		//-读状态
		value[0] = 0x0c;
    value[1] = 0x00;
    value[2] = 0x00;
    value[3] = 0x00;
    value[4] = 0x00;
    //-value[5] = 0x00;
		value[5] = crc_getCrc(value,5);

		res = i2c_write_reg_Buffer(&client, value, 6);
	  if(res <= 0)
	  {//-读值成功的话就继续判断
	 		return;
		}

		res = i2c_read_data_Buffer(&client, 7, value);
    i = crc_getCrc(value,6);
	  if((res <= 0) || (i != value[6]))
	  {//-读值成功的话就继续判断
      //-EEP_Data = 10 * (value[3] + (value[4] *256) + (value[5] *65536));
	 		return;
		}

		//-计算CO2 ppm Monitoring Range 400-2000 ppm equivalent CO2
		temp_data = (UINT16)((value[1] - 13) * ((float)1600/229) + 400);

		co2_value[co2_value_pt++] = temp_data;
   	if(co2_value_pt > 15)
   	  co2_value_pt = 0;
   	temp_data = 0;
   	for(i=0;i < 16;i++)
   	  temp_data = temp_data + co2_value[i];
   	co2_data = temp_data / 16;

		//-计算tVOC ppb] Monitoring Range 0-1000 ppb isobutylene equivalent tVOCs
		temp_data = (UINT16)((value[0] - 13) * ((float)1000/229));
		if(temp_data > 1000)
		  temp_data = 1000;

		voc_value[voc_value_pt++] = temp_data;
   	if(voc_value_pt > 15)
   	  voc_value_pt = 0;
   	temp_data = 0;
   	for(i=0;i < 16;i++)
   	  temp_data = temp_data + voc_value[i];
   	VOC_data = temp_data / 16;
    //-VOC_data = temp_data / (16 *4);    //-临时测试用

    //-watch_data[0] = value[0];
    //-watch_data[1] = value[1];
    //-watch_data[2] = value[2];
    //-watch_data[3] = value[3];
    //-watch_data[4] = value[4];
    //-watch_data[5] = value[5];


		//-EEP_Data = 10 * (value[3] + (value[4] *256) + (value[5] *65536));
    RS_value_rd =  10 * (value[4] + (value[3] *256) + (value[2] *65536));
    VOC_Status = value[5];

    //-9. Reading VZ8(6)9 R0 (calibration value)
    value[0] = 0x10;
    value[1] = 0x00;
    value[2] = 0x00;
    value[3] = 0x00;
    value[4] = 0x00;
    //-value[5] = 0x00;
		value[5] = crc_getCrc(value,5);

		res = i2c_write_reg_Buffer(&client, value, 6);
	  if(res <= 0)
	  {//-读值成功的话就继续判断
	 		return;
		}

		res = i2c_read_data_Buffer(&client, 7, value);
    i = crc_getCrc(value,6);
	  if((res <= 0) || (i != value[6]))
	  {//-读值成功的话就继续判断
      //-EEP_Data = 10 * (value[3] + (value[4] *256) + (value[5] *65536));
	 		return;
		}

    R0_value_rd = value[0] + (value[1] << 8);

    //-8. Reading VZ8(6)9 Date code and revision
    value[0] = 0x0D;
    value[1] = 0x00;
    value[2] = 0x00;
    value[3] = 0x00;
    value[4] = 0x00;
		value[5] = crc_getCrc(value,5);

		res = i2c_write_reg_Buffer(&client, value, 6);
	  if(res <= 0)
	  {//-读值成功的话就继续判断
	 		return;
		}
		res = i2c_read_data_Buffer(&client, 7, value);
    i = crc_getCrc(value,6);
	  if((res <= 0) || (i != value[6]))
	  {//-读值成功的话就继续判断
      //-EEP_Data = 10 * (value[3] + (value[4] *256) + (value[5] *65536));
	 		return;
		}

    R0_value_rd = value[0] + (value[1] << 8);
///////////////////////////////////////////////////////////////////////////////
/*
    //-10. Setting VZ8(6)9 R0 with current RS value
    //-这里设置定值根据内部RS自己重新校准
    value[0] = 0x0E;
    value[1] = 0x00;
    value[2] = 0x00;
    value[3] = 0x00;
    value[4] = 0x00;
    //-value[5] = 0x00;
		value[5] = crc_getCrc(value,5);

		res = i2c_write_reg_Buffer(&client, value, 6);
	  if(res <= 0)
	  {//-读值成功的话就继续判断
	 		return;
		}

    //-11. Setting VZ8(6)9 R0 with I2C data
    //-人为设定基准值
    value[0] = 0x0F;
    value[1] = 0x2b;  //-R0 LSB
    value[2] = 0x03;  //-R0 MSB
    value[3] = 0x00;
    value[4] = 0x00;
    //-value[5] = 0x00;
		value[5] = crc_getCrc(value,5);

		res = i2c_write_reg_Buffer(&client, value, 6);
	  if(res <= 0)
	  {//-读值成功的话就继续判断
	 		return;
		}
*/




///////////////////////////////////////////////////////////////////////////////
    //-CO2的值后续通讯处理
		port_send_sense_data[1] = co2_data;

    //-经过一段时间再进行比较而不需要每次更新都比较
       		/*if((co2_data <= 750) && (co2_data >= 250))
       		{//-良好
       			 CO2_data_flag = 0;
       		}
       		else if((co2_data <= 1000) && (co2_data >= 751))
       		{//-轻微超标
       			 CO2_data_flag = 1;
       		}
       		else
       		{//-严重超标
       			 CO2_data_flag = 2;
       		}*/

		if(Judge_LongTime_In_MainLoop(co2_renew_wait_time,29)==YES)
	  {
	  		co2_renew_wait_time = Time_2048ms_Counter;

	  		if(co2_data_old >= co2_data)
	     {
	     	 temp_data = co2_data_old - co2_data;
	     }
	     else
	     	 temp_data = co2_data - co2_data_old;
	  		if(temp_data >= 200)
	  			port_send_sense_data[0] = 0x55;
	  		co2_data_old = co2_data;

	  }

	  //-VOC的值后续通讯处理
		port_send_sense_data[3] = VOC_data;

    //-经过一段时间再进行比较而不需要每次更新都比较
       		/*if((VOC_data <= 100) && (VOC_data >= 0))  //-这个目前还没有统一标准
       		{//-良好
       			 VOC_data_flag = 0;
       		}
       		else if((VOC_data <= 200) && (VOC_data >= 101))
       		{//-轻微超标
       			 VOC_data_flag = 1;
       		}
       		else
       		{//-严重超标
       			 VOC_data_flag = 2;
       		}*/

		if(Judge_LongTime_In_MainLoop(VOC_renew_wait_time,29)==YES)
	  {
	  		VOC_renew_wait_time = Time_2048ms_Counter;

	  		if(VOC_data_old >= VOC_data)
	     {
	     	 temp_data = VOC_data_old - VOC_data;
	     }
	     else
	     	 temp_data = VOC_data - VOC_data_old;
	  		if(temp_data >= 200)
	  			port_send_sense_data[0] = 0x55;
	  		VOC_data_old = VOC_data;

	  }
}
#endif



void VZ89_Read_VOC_test(void)
{
struct i2c_client client;
	  int res;
	  u8 value[9],i;
	  u16 temp_data;
    u32 rs,r0;
    float ppb_temp;


	  client.addr = VOC_HD_type;                   // 将从机地址写入数据包pa12201001的地址就是0x3C
    client.flags = 0;
    client.num = 1;		//-硬件端口号1,I2C1

		if(VOC_HD_type == 0)
		{
				//-第二种iAQ
				client.addr = 0xB4;                   // 将从机地址写入数据包pa12201001的地址就是0x3C
				res = i2c_read_data_Buffer(&client, 9, value);
			  if(res <= 0)
			  {//-读值成功的话就继续判断

			 		//-return;
				}
				else
				{
					VOC_HD_type = client.addr;
          return;
				}

        //-8. Reading VZ8(6)9 Date code and revision
      client.addr = 0xE0;
      value[0] = 0x0D;
      value[1] = 0x00;
      value[2] = 0x00;
      value[3] = 0x00;
      value[4] = 0x00;
      value[5] = crc_getCrc(value,5);

      res = i2c_write_reg_Buffer(&client, value, 6);
      if(res <= 0)
      {//-读值成功的话就继续判断
        //-return;
      }
      else
      {
        res = i2c_read_data_Buffer(&client, 7, value);
        i = crc_getCrc(value,6);
        if(res <= 0)
        {//-读值成功的话就继续判断
          //-return;
        }
        else
        {
          if(i != value[6])
          {
          	VOC_HD_type_cn++;
          	if(VOC_HD_type_cn > 10)
          	{
          		VOC_HD_type_cn = 0;
	            VOC_HD_type_y = 1;    //-检验不同过的默认为第一种情况
	            VOC_HD_type = client.addr;
          	}
          }
          else
          {
          	VOC_HD_type_cn = 0;
            VOC_HD_type_y = 2;
            VOC_HD_type = client.addr;
          }
          return;
        }
      }
    }
		else if(VOC_HD_type == 0xE0)
		{
      if(VOC_HD_type_y == 1)
      {
				res = i2c_write_reg_Buffer(&client, CMD_Get_status, 3);
			  if(res <= 0)
			  {//-读值成功的话就继续判断
          VOC_HD_type_y = 0;
          VOC_HD_type = 0;
			 		return;
				}

				res = i2c_read_data_Buffer(&client, 6, value);
			  if(res <= 0)
			  {//-读值成功的话就继续判断
		      VOC_HD_type_y = 0;
          VOC_HD_type = 0;
			 		return;
				}

				//-计算CO2 ppm Monitoring Range 400-2000 ppm equivalent CO2
				temp_data = (UINT16)((value[0] - 13) * ((float)1600/229) + 400);
				//-co2_value[co2_value_pt++] = temp_data;
		   	//-if(co2_value_pt > 15)
		   	//-  co2_value_pt = 0;
		   	//-temp_data = 0;
		   	//-for(i=0;i < 16;i++)
		   	//-  temp_data = temp_data + co2_value[i];
		   	//-co2_data = temp_data / 16;
        co2_data = temp_data;

				//-计算tVOC ppb] Monitoring Range 0-1000 ppb isobutylene equivalent tVOCs
				temp_data = (UINT16)((value[2] - 13) * ((float)1000/229));
				if(temp_data > 1000)
				  temp_data = 1000;
				//-voc_value[voc_value_pt++] = temp_data;
		   	//-if(voc_value_pt > 15)
		   	//-  voc_value_pt = 0;
		   	//-temp_data = 0;
		   	//-for(i=0;i < 16;i++)
		   	//-  temp_data = temp_data + voc_value[i];
		   	//-VOC_data = temp_data / 16;
		    VOC_data = temp_data;    //-临时测试用
      }
      else if(VOC_HD_type_y == 2)
      {
        value[0] = 0x0c;
        value[1] = 0x00;
        value[2] = 0x00;
        value[3] = 0x00;
        value[4] = 0x00;
        value[5] = crc_getCrc(value,5);

        res = i2c_write_reg_Buffer(&client, value, 6);
        if(res <= 0)
        {//-读值成功的话就继续判断
          VOC_HD_type_y = 0;
          VOC_HD_type = 0;
          return;
        }
        res = i2c_read_data_Buffer(&client, 7, value);
        i = crc_getCrc(value,6);
        if((res <= 0) || (i != value[6]))
        {//-读值成功的话就继续判断
          VOC_HD_type_y = 0;
          VOC_HD_type = 0;
          return;
        }

        //-计算CO2 ppm Monitoring Range 400-2000 ppm equivalent CO2
        temp_data = (UINT16)((value[1] - 13) * ((float)1600/229) + 400);
        //-co2_value[co2_value_pt++] = temp_data;
        //-if(co2_value_pt > 15)
        //-  co2_value_pt = 0;
        //-temp_data = 0;
        //-for(i=0;i < 16;i++)
        //-  temp_data = temp_data + co2_value[i];
        //-co2_data = temp_data / 16;
        co2_data = temp_data;

        //-计算tVOC ppb] Monitoring Range 0-1000 ppb isobutylene equivalent tVOCs
        temp_data = (UINT16)((value[0] - 13) * ((float)1000/229));
        if(temp_data > 1000)
          temp_data = 1000;
        //-voc_value[voc_value_pt++] = temp_data;
        //-if(voc_value_pt > 15)
        //-  voc_value_pt = 0;
        //-temp_data = 0;
        //-for(i=0;i < 16;i++)
        //-  temp_data = temp_data + voc_value[i];
        //-VOC_data = temp_data / 16;
        if(temp_data != 0)
          VOC_data = temp_data;    //-临时测试用
        else
        {
          //-
          rs = (value[2] << 16) + (value[3] << 8) + value[4];  //RS

          value[0] = 0x10;
          value[1] = 0x00;
          value[2] = 0x00;
          value[3] = 0x00;
          value[4] = 0x00;
          value[5] = crc_getCrc(value,5);

          res = i2c_write_reg_Buffer(&client, value, 6);
          if(res <= 0)
          {//-读值成功的话就继续判断
            VOC_HD_type_y = 0;
            VOC_HD_type = 0;
            return;
          }
          res = i2c_read_data_Buffer(&client, 7, value);
          i = crc_getCrc(value,6);
          if((res <= 0) || (i != value[6]))
          {//-读值成功的话就继续判断
            VOC_HD_type_y = 0;
            VOC_HD_type = 0;
            return;
          }

          r0 = (value[1] << 8) + value[0];  //R0
          if(rs>=r0) //RS 与R0 可通过IIC指令读取，命令指导书上有详细说明
          {
                   ppb_temp = rs - r0;
                   ppb_temp /= r0;
                   if(ppb_temp>0.3)
                   {
                          ppb_temp -= 0.3;
                   }
                   else if(ppb_temp<=0.3)
                   {
                          ppb_temp += 0.3;
                   }
                   ppb_temp *= 214;
                   VOC_data = (UINT16)ppb_temp;
          }

        }

        /*if(rs>=r0) //RS 与R0 可通过IIC指令读取，命令指导书上有详细说明
        {
                 //-ppb_temp = rs - r0;
                 ppb_temp /= r0;
                 if(ppb_temp>0.3)
                 {
                        ppb_temp -= 0.3;
                 }
                 else if(ppb_temp<=0.3)
                 {
                        ppb_temp += 0.3;
                 }
                 ppb_temp *= 214;
                 //-voc_ppb = ppb_temp;
        }*/
      }
      else
      {
          VOC_HD_type = 0;
          VOC_HD_type_y = 0;
      }
		}
		else if(VOC_HD_type == 0xB4)
		{
				res = i2c_read_data_Buffer(&client, 9, value);
			  if(res <= 0)
			  {//-读值成功的话就继续判断
		      VOC_HD_type = 0;
			 		return;
				}
		    temp_data = (unsigned int)((value[0])<<8) + value[1];
		    //-co2_value[co2_value_pt++] = temp_data;
		   	//-if(co2_value_pt > 15)
		   	//-  co2_value_pt = 0;
		   	//-temp_data = 0;
		   	//-for(i=0;i < 16;i++)
		   	//-  temp_data = temp_data + co2_value[i];
		   	//-co2_data = temp_data / 16;
        co2_data = temp_data;

		    temp_data = (unsigned int)((value[7])<<8) + value[8];
		    //-voc_value[voc_value_pt++] = temp_data;
		   	//-if(voc_value_pt > 15)
		   	//-  voc_value_pt = 0;
		   	//-temp_data = 0;
		   	//-for(i=0;i < 16;i++)
		   	//-  temp_data = temp_data + voc_value[i];
		   	//-VOC_data = temp_data / 16;
        VOC_data = temp_data;
		}
		else
		{
				VOC_data = 0;
				co2_data = 0;
				VOC_HD_type = 0;
		}


		port_send_sense_data[1] = co2_data;
    port_send_sense_data[3] = VOC_data;

    if(Judge_LongTime_In_MainLoop(co2_renew_wait_time,29)==YES)
	  {
	  		co2_renew_wait_time = Time_2048ms_Counter;

	  		if(co2_data_old >= co2_data)
	     {
	     	 temp_data = co2_data_old - co2_data;
	     }
	     else
	     	 temp_data = co2_data - co2_data_old;
	  		if(temp_data >= 200)
	  			port_send_sense_data[0] = 0x55;
	  		co2_data_old = co2_data;

	  }

		if(Judge_LongTime_In_MainLoop(VOC_renew_wait_time,29)==YES)
	  {
	  		VOC_renew_wait_time = Time_2048ms_Counter;

	  		if(VOC_data_old >= VOC_data)
	     {
	     	 temp_data = VOC_data_old - VOC_data;
	     }
	     else
	     	 temp_data = VOC_data - VOC_data_old;
	  		if(temp_data >= 200)
	  			port_send_sense_data[0] = 0x55;
	  		VOC_data_old = VOC_data;

	  }

}



void VZ89_Read_VOC(void)
{
struct i2c_client client;
	  int res;
	  u8 value[9],i;
	  u16 temp_data;
    u32 rs,r0;
    float ppb_temp;

	  client.addr = VOC_HD_type;                   // 将从机地址写入数据包pa12201001的地址就是0x3C
    client.flags = 0;
    client.num = 1;		//-硬件端口号1,I2C1

		if(VOC_HD_type == 0)
		{
				//-第二种iAQ
				client.addr = 0xB4;                   // 将从机地址写入数据包pa12201001的地址就是0x3C
				res = i2c_read_data_Buffer(&client, 9, value);
			  if(res <= 0)
			  {//-读值成功的话就继续判断

			 		//-return;
				}
				else
				{
					VOC_HD_type = client.addr;
          return;
				}

        //-8. Reading VZ8(6)9 Date code and revision
      client.addr = 0xE0;
      value[0] = 0x0D;
      value[1] = 0x00;
      value[2] = 0x00;
      value[3] = 0x00;
      value[4] = 0x00;
      value[5] = crc_getCrc(value,5);

      res = i2c_write_reg_Buffer(&client, value, 6);
      if(res <= 0)
      {//-读值成功的话就继续判断
        //-return;
      }
      else
      {
        res = i2c_read_data_Buffer(&client, 7, value);
        i = crc_getCrc(value,6);
        if(res <= 0)
        {//-读值成功的话就继续判断
          //-return;
        }
        else
        {
          if(i != value[6])
          {
          	VOC_HD_type_cn++;
          	if(VOC_HD_type_cn > 10)
          	{
          		VOC_HD_type_cn = 0;
            	VOC_HD_type_y = 1;    //-检验不同过的默认为第一种情况
            	VOC_HD_type = client.addr;
            }
          }
          else
          {
          	VOC_HD_type_cn = 0;
            VOC_HD_type_y = 2;
            VOC_HD_type = client.addr;
          }
          return;
        }
      }
    }
		else if(VOC_HD_type == 0xE0)
		{
      if(VOC_HD_type_y == 1)
      {
				res = i2c_write_reg_Buffer(&client, CMD_Get_status, 3);
			  if(res <= 0)
			  {//-读值成功的话就继续判断
          VOC_HD_type = 0;
          VOC_HD_type_y = 0;
			 		return;
				}

				res = i2c_read_data_Buffer(&client, 6, value);
			  if(res <= 0)
			  {//-读值成功的话就继续判断
		      VOC_HD_type = 0;
          VOC_HD_type_y = 0;
			 		return;
				}

				//-计算CO2 ppm Monitoring Range 400-2000 ppm equivalent CO2
				temp_data = (UINT16)((value[0] - 13) * ((float)1600/229) + 400);
				co2_value[co2_value_pt++] = temp_data;
		   	if(co2_value_pt > 15)
		   	  co2_value_pt = 0;
		   	temp_data = 0;
		   	for(i=0;i < 16;i++)
		   	  temp_data = temp_data + co2_value[i];
		   	co2_data = temp_data / 16;

				//-计算tVOC ppb] Monitoring Range 0-1000 ppb isobutylene equivalent tVOCs
				temp_data = (UINT16)((value[2] - 13) * ((float)1000/229));
				if(temp_data > 1000)
				  temp_data = 1000;
				//-传感器本身是有问题的这里特殊处理下
		    if(temp_data < 100)
		    {
		    	temp_data = temp_data;
		    }
		    else if(temp_data < 200)
		    {
		    	temp_data = temp_data/2;
		    }
		    else if(temp_data < 300)
		    {
		    	temp_data = temp_data/3;
		    }
		    else if(temp_data < 400)
		    {
		    	temp_data = temp_data/4;
		    }
		    else if(temp_data < 500)
		    {
		    	temp_data = temp_data/4;
		    }
		    else if(temp_data < 700)
		    {
		    	temp_data = temp_data/3;
		    }
		    else if(temp_data < 900)
		    {
		    	temp_data = temp_data/2;
		    }
		    else
		    {
		    	temp_data = temp_data;
		    }
		    //-end
				voc_value[voc_value_pt++] = temp_data;
		   	if(voc_value_pt > 15)
		   	  voc_value_pt = 0;
		   	temp_data = 0;
		   	for(i=0;i < 16;i++)
		   	  temp_data = temp_data + voc_value[i];
		   	VOC_data = temp_data / 16;
		    //-VOC_data = temp_data / (16 *4);    //-临时测试用
        if(VOC_data == 1000)
        {
          VOC_HD_type = 0;
          VOC_HD_type_y = 0;
        }
      }
      else if(VOC_HD_type_y == 2)
      {
        value[0] = 0x0c;
        value[1] = 0x00;
        value[2] = 0x00;
        value[3] = 0x00;
        value[4] = 0x00;
        value[5] = crc_getCrc(value,5);

        res = i2c_write_reg_Buffer(&client, value, 6);
        if(res <= 0)
        {//-读值成功的话就继续判断
          VOC_HD_type = 0;
          VOC_HD_type_y = 0;
          return;
        }
        res = i2c_read_data_Buffer(&client, 7, value);
        i = crc_getCrc(value,6);
        if((res <= 0) || (i != value[6]))
        {//-读值成功的话就继续判断
          VOC_HD_type = 0;
          VOC_HD_type_y = 0;
          return;
        }

        //-计算CO2 ppm Monitoring Range 400-2000 ppm equivalent CO2
        temp_data = (UINT16)((value[1] - 13) * ((float)1600/229) + 400);
        /*co2_value[co2_value_pt++] = temp_data;
        if(co2_value_pt > 15)
          co2_value_pt = 0;
        temp_data = 0;
        for(i=0;i < 16;i++)
          temp_data = temp_data + co2_value[i];
        co2_data = temp_data / 16;
*/
        co2_data = temp_data;

        //-计算tVOC ppb] Monitoring Range 0-1000 ppb isobutylene equivalent tVOCs
        temp_data = (UINT16)((value[0] - 13) * ((float)1000/229));
        if(temp_data > 1000)
          temp_data = 1000;
        if(temp_data != 0)
          VOC_data = temp_data;
        else
        {
          //-
          rs = ((value[2] << 16) + (value[3] << 8) + value[4]) * 10;  //RS

          value[0] = 0x10;
          value[1] = 0x00;
          value[2] = 0x00;
          value[3] = 0x00;
          value[4] = 0x00;
          value[5] = crc_getCrc(value,5);

          res = i2c_write_reg_Buffer(&client, value, 6);
          if(res <= 0)
          {//-读值成功的话就继续判断
            VOC_HD_type_y = 0;
            VOC_HD_type = 0;
            return;
          }
          res = i2c_read_data_Buffer(&client, 7, value);
          i = crc_getCrc(value,6);
          if((res <= 0) || (i != value[6]))
          {//-读值成功的话就继续判断
            VOC_HD_type_y = 0;
            VOC_HD_type = 0;
            return;
          }

          r0 = ((value[1] << 8) + value[0]) * 1000;  //R0
          if(rs>=r0) //RS 与R0 可通过IIC指令读取，命令指导书上有详细说明
          {
                   ppb_temp = rs - r0;
                   ppb_temp /= r0;
                   if(ppb_temp>0.3)
                   {
                          ppb_temp -= 0.3;
                   }
                   else if(ppb_temp<=0.3)
                   {
                          ppb_temp += 0.3;
                   }
                   ppb_temp *= 214;
                   VOC_data = (UINT16)ppb_temp;
          }
        }
      }
      else
      {
          VOC_HD_type = 0;
          VOC_HD_type_y = 0;
      }
	   if((NTC_data>32)&&(NTC_data<=35))
        {
          if((VOC_data > 400)&&(VOC_data <= 500))
          {
            VOC_data = VOC_data*90/100;
          }
          if((VOC_data > 500)&&(VOC_data <= 600))
          {
            VOC_data = VOC_data*86/100;
          }
          if((VOC_data > 600)&&(VOC_data <= 700))
          {
            VOC_data = VOC_data*82/100;
          }
          if((VOC_data > 700)&&(VOC_data <= 800))
          {
            VOC_data = VOC_data*86/100;
          }
          else
          {
            VOC_data = VOC_data;
          }
        }
        if((NTC_data>35)&&(NTC_data<=38))
        {
          if((VOC_data > 400)&&(VOC_data <= 500))
          {
            VOC_data = VOC_data*88/100;
          }
          if((VOC_data > 500)&&(VOC_data <= 600))
          {
            VOC_data = VOC_data*84/100;
          }
          if((VOC_data > 600)&&(VOC_data <= 700))
          {
            VOC_data = VOC_data*80/100;
          }
          if((VOC_data > 700)&&(VOC_data <= 800))
          {
            VOC_data = VOC_data*84/100;
          }
          else
          {
            VOC_data = VOC_data;
          }
        }
        if((NTC_data>38)&&(NTC_data<=41))
        {
          if((VOC_data > 400)&&(VOC_data <= 500))
          {
            VOC_data = VOC_data*86/100;
          }
          if((VOC_data > 500)&&(VOC_data <= 600))
          {
            VOC_data = VOC_data*82/100;
          }
          if((VOC_data > 600)&&(VOC_data <= 700))
          {
            VOC_data = VOC_data*78/100;
          }
          if((VOC_data > 600)&&(VOC_data <= 700))
          {
            VOC_data = VOC_data*82/100;
          }
          else
          {
            VOC_data = VOC_data;    
          }
        }
        if((NTC_data>41)&&(NTC_data<=45))
        {
          if((VOC_data > 400)&&(VOC_data <= 500))
          {
            VOC_data = VOC_data*84/100;
          }
          if((VOC_data > 500)&&(VOC_data <= 600))
          {
            VOC_data = VOC_data*80/100;
          }
          if((VOC_data > 600)&&(VOC_data <= 700))
          {
            VOC_data = VOC_data*76/100;
          }
          if((VOC_data > 700)&&(VOC_data <= 800))
          {
            VOC_data = VOC_data*80/100;
          }
          else
          {
            VOC_data = VOC_data;    
          }
        }
        if(NTC_data>45)
        {
          if((VOC_data > 400)&&(VOC_data <= 500))
          {
            VOC_data = VOC_data*82/100;
          }
          if((VOC_data > 500)&&(VOC_data <= 600))
          {
            VOC_data = VOC_data*78/100;
          }    
          if((VOC_data > 600)&&(VOC_data <= 700))
          {
            VOC_data = VOC_data*74/100;
          }
          if((VOC_data > 700)&&(VOC_data <= 800))
          {
            VOC_data = VOC_data*78/100;
          }
          else
          {
            VOC_data = VOC_data;    
          }
        }
		}
		else if(VOC_HD_type == 0xB4)
		{
				res = i2c_read_data_Buffer(&client, 9, value);
			  if(res <= 0)
			  {//-读值成功的话就继续判断
		      VOC_HD_type = 0;
			 		return;
				}
		    temp_data = (unsigned int)((value[0])<<8) + value[1];
		    co2_value[co2_value_pt++] = temp_data;
		   	if(co2_value_pt > 15)
		   	  co2_value_pt = 0;
		   	temp_data = 0;
		   	for(i=0;i < 16;i++)
		   	  temp_data = temp_data + co2_value[i];
		   	co2_data = temp_data / 16;

		    temp_data = (unsigned int)((value[7])<<8) + value[8];
		    voc_value[voc_value_pt++] = temp_data;
		   	if(voc_value_pt > 15)
		   	  voc_value_pt = 0;
		   	temp_data = 0;
		   	for(i=0;i < 16;i++)
		   	  temp_data = temp_data + voc_value[i];
		   	VOC_data = temp_data / 16;
		}
		else
		{
				VOC_data = 0;
				co2_data = 0;
				VOC_HD_type = 0;
		}


		port_send_sense_data[1] = co2_data;
    port_send_sense_data[3] = VOC_data;

    if(Judge_LongTime_In_MainLoop(co2_renew_wait_time,29)==YES)
	  {
	  		co2_renew_wait_time = Time_2048ms_Counter;

	  		if(co2_data_old >= co2_data)
	     {
	     	 temp_data = co2_data_old - co2_data;
	     }
	     else
	     	 temp_data = co2_data - co2_data_old;
	  		if(temp_data >= 200)
	  			port_send_sense_data[0] = 0x55;
	  		co2_data_old = co2_data;

	  }

		if(Judge_LongTime_In_MainLoop(VOC_renew_wait_time,29)==YES)
	  {
	  		VOC_renew_wait_time = Time_2048ms_Counter;

	  		if(VOC_data_old >= VOC_data)
	     {
	     	 temp_data = VOC_data_old - VOC_data;
	     }
	     else
	     	 temp_data = VOC_data - VOC_data_old;
	  		if(temp_data >= 200)
	  			port_send_sense_data[0] = 0x55;
	  		VOC_data_old = VOC_data;

	  }

}




///////////////////////////////////////////////////////////////////////////////
/*
STM32内部温度 转换采样值为温度
ADC转换结束以后，读取ADC_DR寄存器中的结果，转换温度值计算公式如下：
        V25 - VSENSE
T(℃) = ------------ + 25
          Avg_Slope
V25： 温度传感器在25℃时 的输出电压，典型值1.43 V。
VSENSE：温度传感器的当前输出电压，与ADC_DR 寄存器中的结果ADC_ConvertedValue之间的转换关系为：
          ADC_ConvertedValue * Vdd
VSENSE = --------------------------
          Vdd_convert_value(0xFFF)
Avg_Slope：温度传感器输出电压和温度的关联参数，典型值4.3 mV/℃。
//Converted Temperature
Vtemp_sensor = ADC_ConvertedValue * Vdd / Vdd_convert_value;
Current_Temp = (V25 - Vtemp_sensor)/Avg_Slope + 25;
*/
void get_ntc_adc_value(void) 	//-提取需要的点的ADC数值
{
   //-UINT16 TEMP;

   //-TEMP = samp_data_ram_base[1][2];
   //-NTC_data = NTC_CalcTemperature(TEMP);

   NTC_data=(UINT16)((1.43 - ADC_ConvertedValue[1]*3.35/4096)*1000/4.35 + 25);
   
   //-port_send_sense_data[5] = watch_data[0];
}





