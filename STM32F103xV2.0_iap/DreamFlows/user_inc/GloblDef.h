


#ifndef __GloblDef_H__
#define __GloblDef_H__

extern WORD    port_recv_pt[2];
extern WORD    port_recv_dl[2];
extern WORD    port_send_pt[2];
extern WORD    port_send_len[2];

extern BYTE	port_recv[2][512];
extern BYTE	port_send[2][512];
extern BYTE	port_deal_buf[2][512];
extern BYTE	port_report[512];
extern BYTE	port_deal_flag;

extern BYTE  rx2_cnt;
extern BYTE  uart2_rx_buff[100];

extern UL		samp_data_ch0_average;
extern UL		samp_data_ch1_average;
extern UL		samp_data_ch2_average;
extern UL		samp_data_ch3_average;
extern UL		samp_data_ch4_average;



/*extern struct L6_Ram
{								//start		len
	int16  samp_data_ram_base[4][128];			//0x0000	0x600  乘通道系数的结果。
	int16  samp_data_cf_ram_base[4][128];			//0x0600	0x600
	Uint16 samp_data_power_ram_base[128];			//0x0c00	0x080
	int16  samp_data_yc_ram_base[13][128];			//0x0c80	0x240  end 0x0fef
}L6Ram;
*/

extern BYTE	samp_data_pt;

extern WORD    port_send_sense_data[20];

/*extern struct  NET_PORT_VAR{
Uint16	net_port_recv_pt[UIP_CONNS];
Uint16   net_port_recv_dl[UIP_CONNS];
UINT8    net_port_recv[UIP_CONNS][2048];
Uint16	net_port_send_pt[UIP_CONNS];
Uint16   net_port_send_len[UIP_CONNS];
UINT8    net_port_send[UIP_CONNS][2048];
UINT8    net_port_deal_buf[UIP_CONNS][512];
		    }net_port_var;*/

//////////////////////////////////////////////////////////

#define NU_SUCCESS				0

//#define one_net
#define syn_clock		/*--IRIG-B's time from rs485 and net's date*/
//#define ex_syn_all 	/*only from IRIG-B */
//#define timeover_syn_e/*if timeover for syn_clock from net then auto sent clock to can*/

#define SERIAL_BUS_IDLE_TIME  4

/* Device Stste Definition */
extern UINT32 DeviceState;

#define indication_syn_clock_e     0x00000001   // Synchronize Clock Enable
#define indication_syn_clock_err   0x00000002   // Synchronize Clock Error
#define indication_net_run         0x00000004   // Net is running
#define indication_change_ip       0x00000008
#define indication_net_timeover    0x00000010
#define indication_syn_clock_ok    0x00000020	// Synchronized by IRIG-B
#define indication_sys_need_send   0x00000040
#define indication_version         0x00000080
#define indication_mon_serial0_rt  0x00000100	// Monitor UART0
#define indication_mon_serial1_rt  0x00000200	// Monitor UART1
#define indication_mon_tcp0_rt     0x00000400	// Monitor TCP0
#define indication_mon_tcp1_rt     0x00000800	// Monitor TCP1
#define indication_mon_232_rt      0x00001000	// Monitor 232
#define indication_stop_mon        0x00002000	// Stop Monitor

/*  */
extern BYTE  serial_bus_recv_state;

/* System Date&Time */
extern  BYTE     m_year;
extern  BYTE     m_month;
extern  BYTE     m_week;
extern  BYTE     m_date;
extern  BYTE     m_hour;
extern  BYTE     m_min;
extern  BYTE     m_sec;
extern  UINT16   m_msec;

/* User Monitor Buffer */
//-#define MAX_MON_BUF_SIZE  			1024		// Max Monitor Buffer Size
extern UINT8  monitor_buffer[MAX_MON_BUF_SIZE];
extern UINT16 monitor_in_pos,monitor_out_pos,monitor_out_pos_t;

/* UART TX/RX Buffer */
//-extern UART_BUFFER mUART_TX_BUFFER[NO_UART_CPU];
//extern UART_BUFFER mUART_RX_BUFFER[NO_UART_CPU];

/* UART Device Entry&Entry Pointer */
//extern UART_DEVICE_ENTRY  uart_device_task[NO_UART_CPU];
//extern UART_DEVICE_ENTRY  *fp_uart_device[NO_UART_CPU];

/* UART Channel x Initialize Function Pointer Array */
//extern int (*fp_uart_device_init[NO_UART_CPU])(UART_DEVICE_ENTRY *);

/* */
extern BYTE  serial_idle_counter[3];

/* 103 Protocol Device Self Address */
extern UINT16 net_u103_self_addr;

/* Index Protocol Type for TCP or UDP */
extern UINT8  protocal_type;



extern  UINT16  cticks_ms;    //-时间基准计数器
extern  UINT32  cticks_ms_32;
extern  UINT16  Time_2048ms_Counter;
extern  UINT32  cticks_5ms;    //-时间基准计数器
extern  UINT32  cticks_500ms;
extern  UINT32  cticks_s;
extern  UINT32  cticks_ms_pwm;
extern  UINT32  cticks_test;

extern  UINT16  cticks_SLEEP_serial;
extern  UINT16  TEST_Sensor_wait_time;
extern  UINT16  Sensor_data_wait_time;

#define SYS_TIME_OUT_NUM   0xfffffff0



//common function
extern void memcopy_to_udpbuf(BYTE *src,short len);


//-光照手势传感器使用的临时全局变量,测试用
extern UINT16	  i2c1_alsdata_16;
extern UINT16  	i2c1_psdata_16;
extern UINT8		i2c1_newdata_flag;		//-0 表示没有新数据;1 表示有新数据
extern UINT8		i2c1_psinit_flag;		  //-0 表示初始化成功;1 表示失败
extern UINT8		i2c1_psint_cn;		    //-记录中断持续时间
extern UINT8		i2c1_psint_flag;			//-0 表示中断信号无效;1 表示中断信号存在,开始进入中断后置1,直到软件清除为止
extern UINT8		i2c1_psint_flag_FX;
extern UINT8		i2c1_bus_error_cn;

extern UINT16	  i2c2_alsdata_16;
extern UINT16  	i2c2_psdata_16;
extern UINT8		i2c2_newdata_flag;	  //-0 表示没有新数据;1 表示有新数据
extern UINT8		i2c2_psinit_flag;		  //-0 表示初始化成功;1 表示失败
extern UINT8		i2c2_psint_cn;		    //-记录中断持续时间
extern UINT8		i2c2_psint_flag;			//-0 表示中断信号无效;1 表示中断信号存在,开始进入中断后置1,直到软件清除为止
extern UINT8		i2c2_psint_flag_FX;
extern UINT8		i2c2_bus_error_cn;
extern UINT8		i2c3_bus_error_cn;
extern UINT8		i2c4_bus_error_cn;
extern UINT8		i2c5_bus_error_cn;

extern UINT16  i2c3_psdata_16;
extern UINT16  i2c4_psdata_16;
extern UINT16  als_data;
extern UINT16  als_data2;
extern UINT16  als_data3;
extern UINT16  als_data4;
extern UINT8		ps_flag;
extern UINT32		ps1_int_time;
extern UINT32		ps2_int_time;
extern UINT16		VCNL_poll_wait_time;
//-ADI新手势
extern UINT16  ADI_CH1_OFFSET;
extern UINT16  ADI_CH2_OFFSET;
extern UINT16  ADI_CH3_OFFSET;
extern UINT16  ADI_CH4_OFFSET;
extern UINT16  ADI_OSC32K;
extern UINT16  ADI_OSC32M_TRIM;
extern UINT16  ADI_DATA_BUFFER_OUT;
extern UINT16  ADI_DSAMPLE_TIME;
extern UINT16  ADI_DEC_MODE;
extern UINT16  ADI_INT_STATUS;
extern UINT32 	ADI_X1;
extern UINT32 	ADI_X2;
extern UINT32 	ADI_Y1;
extern UINT32 	ADI_Y2;
extern UINT16	  ADI_channels_wait_time;
extern UINT8 	  ADI_channels_init_flag;
extern UINT16   ADI_PS_Value1;
extern UINT8		ADI_ps_flag;
extern UINT16	  ADI_ps_wait_time;
extern UINT16	  ADI_ps_H_time;

//-点阵屏显示临时变量
extern UINT8		led_display_deal_flag;
extern UINT8		led_display_flag;			//-切换显示页
extern UINT8		led_display_y;
extern UINT8		led_display_cn;        //-显示进入中断的次数
extern UINT8		led_display_cn_f;
extern UINT8		led_display_new;			//-显示是否有新数据需要准备0x55 表示现在准备新数据	0xaa 表示数据准备好了
extern UINT16  	led_display_data;			//-一页最多显示三位数据,所以最大是999
extern UINT8		led_display_num;				//-两个缓冲区轮流显示
extern UINT8		led_display_txpic_flag;				//-0x55 表示显示通讯下发的图片
extern UINT8		led_display_txpic_num;
extern UINT16	  cartoon_end_wait_time;
extern UINT32		led_display_data0[16];
extern UINT32		led_display_data1[16];	//-保存需要显示数据的内容
extern UINT32		led_display_data2[16];	//-保存需要显示数据的内容
extern UINT32		led_display_data3[16];	//-保存需要显示数据的内容
extern UINT32		led_display_data4[16];	//-保存需要显示数据的内容
extern UINT32		led_display_data5[16];	//-保存需要显示数据的内容
extern UINT32		led_display_data6[16];
extern UINT32	  led_display_data_temp[16];
extern UINT8		led_data_x;
extern UINT8		led_data_y;
extern UINT32		led_display_data_round[32];
extern UINT32		led_display_data_round1[32];
extern UINT8		led_display_Hx;			//-最终输出显示的行号
extern UINT8		led_display_Vbuffx;
extern UINT32		led_display_Vdata0[16];
extern UINT32		led_display_Vdata1[16];		//-最终输出显示的数据
extern UINT32		led_display_Vdata2[16];
extern UINT32	  *led_display_Vdata_pt;
extern UINT8		led_display_pageing;

extern UINT8		led_display_page;
extern UINT8		led_display_page_end;
extern UINT8		led_display_start;
extern UINT8		led_display_ye_ok;
extern UINT8    cticks_s_page;
extern UINT8		ps_flag_led;
extern UINT8		ps_flag_led_end;
extern UINT8		ps_flag_led_disp;
extern UINT8		led_display_long;
extern UINT16	  led_display_end_time;
extern UINT8    led_display_move_pt;
extern UINT8		led_display_data_flag;
extern UINT8		ps_flag_led_dis;

extern UINT16	  cartoon_page_change_time;
extern UINT16	  cartoon_start_time;

//-字库数组定义常数
extern  UINT8 ZIKU[][16];
extern  UINT8 ZIKU_CO2[][3];
extern  UINT8 ZIKU_PM25[][3];
extern  UINT8 ZIKU_TEMP[][3];
extern  UINT8 ZIKU_humidity[][3];
extern  UINT8 ZIKU_TIME[][3];
extern  UINT8 ZIKU_WIFI[][3];
extern  UINT8 ZIKU_cloudy[][3];
extern  UINT8 ZIKU_sunshine[][3];
extern  UINT8 ZIKU_lightning[][3];
extern  UINT8 ZIKU_rain[][3];
extern  UINT8 ZIKU_WIFIOFF[][3];
extern  UINT8 ZIKU_DROOM[][3];
extern  UINT8 ZIKU_HOOMIN[][3];
extern  UINT8 ZIKU_HOMEOUT[][3];
extern  UINT8 ZIKU_yawp[][3];
extern  UINT8 ZIKU_laugh[][3];
extern  UINT8 ZIKU_cry[][3];
extern  UINT8 ZIKU_00[][3];
extern  UINT8 ZIKU_01[][3];
extern  UINT8 ZIKU_02[][3];
extern  UINT8 ZIKU_03[][3];
extern  UINT8 ZIKU_04[][3];
extern  UINT8 ZIKU_user[][3];
extern  UINT8 ZIKU_FF[][3];
extern  UINT8 ZIKU_VOC[][3];
extern  UINT8 ZIKU_snow[][3];
extern  UINT8 ZIKU_shade[][3];

extern UINT8 ZIKU_SIN[][3];
extern UINT8 ZIKU_XIAO[][7];

extern UINT8 ZIKU_cartoon01[][3];
extern UINT8 ZIKU_cartoon02[][3];
extern UINT8 ZIKU_cartoon03[][3];
extern UINT8 ZIKU_cartoon04[][3];
extern UINT8 ZIKU_cartoon05[][3];
extern UINT8 ZIKU_cartoon06[][3];
extern UINT8 ZIKU_cartoon07[][3];
extern UINT8 ZIKU_cartoon08[][3];
extern UINT8 ZIKU_cartoon09[][3];
extern UINT8 ZIKU_cartoon10[][3];
extern UINT8 ZIKU_cartoon11[][3];
extern UINT8 ZIKU_cartoon12[][3];
extern UINT8 ZIKU_cartoon13[][3];
extern UINT8 ZIKU_cartoon14[][3];
extern UINT8 ZIKU_cartoon15[][3];
extern UINT8 ZIKU_cartoon16[][3];
extern UINT8 ZIKU_cartoon17[][3];
extern UINT8 ZIKU_cartoon18[][3];
extern UINT8 ZIKU_cartoon19[][3];
extern UINT8 ZIKU_cartoon20[][3];
extern UINT8 ZIKU_cartoon21[][3];
extern UINT8 ZIKU_cartoon22[][3];
extern UINT8 ZIKU_cartoon23[][3];
extern UINT8 ZIKU_cartoon24[][3];
extern UINT8 ZIKU_cartoon25[][3];
extern UINT8 ZIKU_cartoon26[][3];
extern UINT8 ZIKU_cartoon27[][3];
extern UINT8 ZIKU_cartoon28[][3];
extern UINT8 ZIKU_cartoon29[][3];

extern UINT8	temp_data01_8;
extern UINT8	temp_data02_8;
extern UINT8	temp_data03_8;
extern UINT8	temp_data04_8;

extern UINT16	temp_data01;
extern UINT16	temp_data02;
extern UINT16	temp_data03;
extern UINT16	temp_data04;

extern UINT16		watch_data[512];

extern UINT8		watch_cfgdata;
extern UINT16		watch_cfgdata_16;


//-触摸按键
/*
BIN5		BIN4		BIN3		BIN2		BIN1		GUARD
CS3			CS1			CS0			CS4			CS5			CS2
D5      D4      D0      D1      D3			保护
*/
#define KEY_BIN1         0x20
#define KEY_BIN2         0x10
#define KEY_BIN3         0x01
#define KEY_BIN4         0x02
#define KEY_BIN5         0x08
#define KEY_GUARD        0x04
#define KEY_MASK         0x3F

extern UINT8		CSKEY_cfgDATA;
extern UINT8		CSKEY_DATA;

//-by cheng CO2
extern UINT16 		Value_CO2_Buffer[10];
extern UINT8		  CO2_Counter;
extern char      ReadBuf_CO2[30];
extern int       Received_Over_time_co2,Received_Over_Flag_co2;
extern UINT8		  send_end_flag_co2;			//-值为1 表示发送结束了等待应答;值为0 已经应答不再等待
extern UINT8     Received_pt_co2;
extern UINT32    cticks_s_CO2;
extern int       co2_data;


//-by cheng pm2.5
extern char    ReadBuf_pm[30];
extern char 	 Received_Over_Flag_pm;
extern UINT16 		Value_PM_Buffer[10];
extern UINT8 		PM_Counter;
extern char    RcvStatus_pm;
extern UINT8     Received_pt_pm;
extern int pm_data;

//-by cheng 温湿度
extern UINT16 		temperature_data;
extern UINT16 		humidity_data;

//-by cheng PWM
extern UINT8    rgb_num;
extern UINT8		ps_flag_pwm;
extern UINT32   cticks_ms_pwm_loop;
extern UINT32  cticks_ms_pwm_R;
extern UINT32  cticks_ms_pwm_G;
extern UINT32  cticks_ms_pwm_B;
extern UINT32  cticks_ms_pwm_4;
extern UINT16    GREEN_pwmval;
extern UINT16    RED_pwmval;
extern UINT16    BLUE_pwmval;
extern UINT16    GREEN_pwmval_pt;
extern UINT16    RED_pwmval_pt;
extern UINT16    BLUE_pwmval_pt;
extern UINT16    white_pwmval_pt;
extern UINT16    GREEN_pwmval_num;		//-一个占空比占有的时长
extern UINT16    RED_pwmval_num;
extern UINT16    BLUE_pwmval_num;
extern UINT16    white_pwmval_num;
#define PWM_Period_Value        4000	//-更新事件的有效值
#define PWM_step_cn             (375*2)  	//-经过多少步由最小变为最大
//-运用在彩灯上时
extern UINT8     HL_flag;		//-彩灯标志,用于表示彩灯的运行模式:0 呼吸状态;1 彩灯状态
extern UINT16    HL_run_time;
extern UINT16    HL_run_time_delay;
extern UINT8     HL_new_value_flag;		//-0 表示占空比没有变化;1 有灯的占空比发生了变化
extern UINT16    HL_GREEN_pwmval;
extern UINT16    HL_RED_pwmval;
extern UINT16    HL_BLUE_pwmval;
extern UINT8     HL_step;
extern UINT8     HL_step_cn;
extern UINT8     HL_ld_R[6];		//-记录了每种颜色的亮度值(0~255)
extern UINT8     HL_ld_G[6];
extern UINT8     HL_ld_B[6];
extern UINT8     HL_ld_R_user[6];		//-用户记录了每种颜色的亮度值(0~255)
extern UINT8     HL_ld_G_user[6];
extern UINT8     HL_ld_B_user[6];
extern UINT8     HL_ld_brightness_flag;
extern UINT8     HL_ld_brightness[2];

//-噪声
#define maxbuffer 50
#define maxcounter 100
#define base 1723
extern UINT16 ADC_ConvertedValue[maxbuffer];
extern UINT16 ADC_ConvertedValue_TEMP[maxbuffer];
extern UINT8 Start_thansfer;
extern UINT8 ADC_Conv_flag;
extern UINT16 Noise_Value;

//-天气
extern UINT8 Weather_flag;

//-全局使用标志,暂时定义
extern UINT8     sys_err_flag;

//-测试用数组,存储点阵屏位置信息
extern UINT16    test_pt;
extern UINT16    test_cn[8];
extern UINT16    test_cn_wait_time;
extern UINT32    test_SIN_data[16];

//-模拟EEPROM 使用FLASH
extern UINT32 EEP_Data;
extern UINT8  EEP_Data_flag;


extern float float_data;

//-三轴加速度计
extern UINT16    ADXL_X_data_flag;	//-0表示数据为正数,1 表示数据为负数
extern UINT16    ADXL_Y_data_flag;
extern UINT16    ADXL_Z_data_flag;
extern UINT16    ADXL_X_data;		//-为绝对值
extern UINT16    ADXL_Y_data;
extern UINT16    ADXL_Z_data;
extern UINT8 		 ADXL_THRESH_TAP;	//-两个参数控制着敲击的灵敏度,有效阀值
extern UINT8 		 ADXL_DUR;

extern UINT8 		 ADXL_TAP_it_flag;
extern UINT8 		 ADXL_TAP_it_SOURCE;
extern UINT16		 ADXL_TAP_wait_time;
extern UINT8 		 ADXL_TAP_off_flag;		//-当需要的时候把敲击功能关闭
extern UINT16		 ADXL_TAP_off_time;


//-自检标识位
extern UINT8 		STM32_UP_selfT_flag1;
extern UINT8 		STM32_UP_error_flag1;
extern UINT8 		STM32_UP_selfT_cn1;
extern UINT8 		STM32_UP_selfT_cn2;
extern UINT8 		STM32_UP_selfT_cn3;

//-串口1
extern UINT8 		UART1_transmit_flag;		//-值为YES表示可以发送 值为NO表示不可以发送
extern UINT8 		UART1_transmit_control;	//-不同的值代表不同的发送内容
extern UINT8 		UART1_renew_flag;			//-0 表示可以通过串口更新数据库数据,0x55 表示不可以通过串口更新数据
extern UINT16 	UART1_renew_wait_time;


//-语音播报
extern UINT8 		voice_flag;
extern UINT8 		voice_keep_data_flag;
extern UINT16 	voice_keep_data_time;


//-VOC
extern UINT16 		VOC_data;

//-语音
extern UINT8 		es705_training_count;
extern UINT8 		es705_training_flag;
extern UINT16 	es705_training_wait_time;



#endif // __GloblDef_H__


