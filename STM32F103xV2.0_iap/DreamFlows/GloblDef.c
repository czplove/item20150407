/*
 不包含自己的头文件
 */

#include "user_conf.h"


WORD    port_recv_pt[2];
WORD    port_recv_dl[2];
WORD    port_send_pt[2];
WORD    port_send_len[2];

//-发现下面内存有出错的情况,为了防止出错,使用完之后进行冗余处理,比如清零
BYTE	port_recv[2][512];
BYTE	port_send[2][512]={1,2,3,4,5};
BYTE	port_deal_buf[2][512];
BYTE	port_report[512];
BYTE	port_deal_flag; 		//-值为0可以处理;值为0xaa表示内容还没有发送出去不可以处理

BYTE  rx2_cnt = 0;
BYTE  uart2_rx_buff[100];

UL		samp_data_ch0_average;
UL		samp_data_ch1_average;
UL		samp_data_ch2_average;
UL		samp_data_ch3_average;
UL		samp_data_ch4_average;



/*struct L6_Ram
{								//start		len
	int16  samp_data_ram_base[4][128];			//0x0000	0x600  乘通道系数的结果。
	int16  samp_data_cf_ram_base[4][128];			//0x0600	0x600
	Uint16 samp_data_power_ram_base[128];			//0x0c00	0x080
	int16  samp_data_yc_ram_base[13][128];			//0x0c80	0x240  end 0x0fef
}L6Ram;
*/

BYTE	samp_data_pt;

//-1	CO2
//-2  温度
//-3  湿度
//-4  PM2.5
//-5  噪声
WORD    port_send_sense_data[20];			//-里面是存储的所有可能的传感器数据,第一个元素作为标志位,为0x55时说明有数值更新了需要上送;为0时也是刚更新的数值,只是没有达到
																			//-上送要求


/*struct  NET_PORT_VAR{
Uint16	net_port_recv_pt[UIP_CONNS];
Uint16   net_port_recv_dl[UIP_CONNS];
UINT8    net_port_recv[UIP_CONNS][2048];
Uint16	net_port_send_pt[UIP_CONNS];
Uint16   net_port_send_len[UIP_CONNS];
UINT8    net_port_send[UIP_CONNS][2048];
UINT8    net_port_deal_buf[UIP_CONNS][512];
		    }net_port_var;*/


////////////////////////////////////////////////////////////
/* Monitor Buffer and Position Marker */
UINT8  monitor_buffer[MAX_MON_BUF_SIZE];
UINT16 monitor_in_pos = 0, monitor_out_pos = 0, monitor_out_pos_t = 0;

/* UART Channel x TX/RX Buffer Array */
//-UART_BUFFER mUART_TX_BUFFER[NO_UART_CPU];
//UART_BUFFER mUART_RX_BUFFER[NO_UART_CPU];

/* UART Device x Entry Array and Corresponding Pointer Array */
//UART_DEVICE_ENTRY  uart_device_task[NO_UART_CPU];
//UART_DEVICE_ENTRY   *fp_uart_device[NO_UART_CPU];

/* UART Channel x Initialize Function Pointer Array */
//-int (*fp_uart_device_init[NO_UART_CPU])(UART_DEVICE_ENTRY *);

/* Device Stste Definition */
UINT32 DeviceState =0;

/*
 * UART receive state, bit x for UART channel x
 * ->When byte was received to UART buffer, but not processed, set.
 * ->When byte was received to UART buffer, and processed, cleared.
 * ->When byte was not received to UART buffer, cleared.
*/
BYTE    serial_bus_recv_state=0;

/* Define Serial Port Idle Time->
 * ->When in receive state, clear serial_idle_counter when byte received
 * ->When in receive state, self-added serial_idle_counter when byte not received
 * ->When serial_idle_counter less than SERIAL_BUS_IDLE_TIME, one frame not ended
 */
BYTE    serial_idle_counter[3]={0,0,0};

/* Index Protocol Type for TCP or UDP */
BYTE  protocal_type;	// 1 for UDP, 0 for TCP

/* System date&time */
UINT8  m_year  =0;
UINT8  m_month =0;
UINT8  m_week  =0;
UINT8  m_date  =0;
UINT8  m_hour  =0;
UINT8  m_min   =0;
UINT8  m_sec   =0;
UINT16 m_msec  =0;


UINT16  cticks_ms; //-1ms时间基准计数器
UINT32  cticks_ms_32;
UINT16  Time_2048ms_Counter;
UINT32  cticks_5ms;    //-5ms时间基准计数器
UINT32  cticks_500ms;
UINT32  cticks_ms_pwm;
UINT32  cticks_test;		//-测试传感器时间

UINT32  cticks_s;

UINT16  cticks_SLEEP_serial;    //-伪任务休眠计数器	位15是标识位 0 数据无效	1 数据有效	低15位为计数值(mS级)
UINT16  TEST_Sensor_wait_time;
UINT16  Sensor_data_wait_time;

///const unsigned char version_number[80] ={"DSA LINE NETGATE  ver 1.0"};

//-光照手势传感器使用的临时全局变量,测试用
UINT16	i2c1_alsdata_16;
UINT16  i2c1_psdata_16;
UINT8		i2c1_newdata_flag;		//-0 表示没有新数据;1 表示有新数据
UINT8		i2c1_psinit_flag;		  //-0 表示初始化成功;1 表示失败
UINT8		i2c1_psint_cn;		    //-记录中断持续时间
UINT8		i2c1_psint_flag;			//-0 表示中断信号无效;1 表示中断信号存在,开始进入中断后置1,直到软件清除为止
UINT8		i2c1_psint_flag_FX;
UINT8		i2c1_bus_error_cn;		//-总线出错累加都一定数量之后重新初始化总线

UINT16	i2c2_alsdata_16;
UINT16  i2c2_psdata_16;
UINT8		i2c2_newdata_flag;	  //-0 表示没有新数据;1 表示有新数据
UINT8		i2c2_psinit_flag;		  //-0 表示初始化成功;1 表示失败
UINT8		i2c2_psint_cn;		    //-记录中断持续时间
UINT8		i2c2_psint_flag;			//-0 表示中断信号无效;1 表示中断信号存在,开始进入中断后置1,直到软件清除为止
UINT8		i2c2_psint_flag_FX;
UINT8		i2c2_bus_error_cn;
UINT8		i2c3_bus_error_cn;
UINT8		i2c4_bus_error_cn;
UINT8		i2c5_bus_error_cn;

UINT16  i2c3_psdata_16;
UINT16  i2c4_psdata_16;
UINT16  als_data;
UINT16  als_data2;
UINT16  als_data3;
UINT16  als_data4;
UINT8		ps_flag;		//-挥动手势的情况	0 没有挥动;1 由上到下;2 由下到上
UINT32		ps1_int_time;
UINT32		ps2_int_time;
UINT16		VCNL_poll_wait_time;
//-ADI新手势
UINT16  ADI_CH1_OFFSET;
UINT16  ADI_CH2_OFFSET;
UINT16  ADI_CH3_OFFSET;
UINT16  ADI_CH4_OFFSET;
UINT16  ADI_OSC32K;
UINT16  ADI_OSC32M_TRIM;
UINT16  ADI_DATA_BUFFER_OUT;
UINT16  ADI_DSAMPLE_TIME;
UINT16  ADI_DEC_MODE;
UINT16  ADI_INT_STATUS;
UINT32 	ADI_X1 = 0;
UINT32 	ADI_X2 = 0;
UINT32 	ADI_Y1 = 0;
UINT32 	ADI_Y2 = 0;
UINT16	ADI_channels_wait_time;		//-如果通道状态值偏移持续超过一段时间,就认为初始状态变化了,那么重新校准
UINT8 	ADI_channels_init_flag;
UINT16  ADI_PS_Value1;
UINT8		ADI_ps_flag;		//-值为0 说明处于空闲状态;0x55 说明已经识别到第一个有效手势,在一段时间内挥手失效
UINT16	ADI_ps_wait_time;
UINT16	ADI_ps_H_time;    //-记录手势中断高电平时间


//-点阵屏显示临时变量
UINT8		led_display_deal_flag;			//-是否准备菜单标志0x55处理准备,不需要显示的时候就不处理菜单
UINT8		led_display_flag;			//-作为一页内容,显示的方式标志
UINT8		led_display_y;        //-显示屏显示的行号最大值,就是在动画中的显示变量
UINT8		led_display_cn;        //-显示进入中断的次数
UINT8		led_display_cn_f;
UINT8		led_display_new;			//-显示是否有新数据需要准备 0x55表示现在准备新数据;	0xaa表示数据准备好了; 0xa5表示处于动画过程中.
UINT16  led_display_data;			//-一页最多显示三位数据,所以最大是999
UINT8		led_display_num;				//-两个缓冲区轮流显示:值1 2是两个常规静态显示区,5 6是通讯显示图片
UINT8		led_display_txpic_flag;				//-0x55 表示显示通讯下发的图片 0xaa 表示图片已经刷过了,主机可以继续刷新;手势结束刷图,恢复正常显示
UINT8		led_display_txpic_num;				//-刷固定图片的偏移量
UINT16	cartoon_end_wait_time;		//-记录一个动画开始的时间以便确定什么时候结束
UINT32	led_display_data0[16];	//-组织的数据放在这个里面
UINT32	led_display_data1[16];	//-保存需要显示数据的内容
UINT32	led_display_data2[16];	//-保存需要显示数据的内容
UINT32	led_display_data3[16];	//-保存需要显示数据的内容,专门显示动画效果的,最终数据区
UINT32	led_display_data4[16];	//-保存需要显示数据的内容,专门显示动画效果的,动态过程数据区
UINT32	led_display_data5[16];	//-通讯显示
UINT32	led_display_data6[16];
UINT32	led_display_data_temp[16];	//-临时保存数据的全局变量
UINT8		led_data_x;							//-表示在整个数据点中的坐标位置
UINT8		led_data_y;
UINT32	led_display_data_round[32];		//-一个全局变量用于旋转效果图
UINT32	led_display_data_round1[32];		//-作为旋转效果图的一个中间变量
UINT8		led_display_Hx;			//-最终输出显示的行号
UINT8		led_display_Vbuffx;
UINT32	led_display_Vdata0[16];		//-刚亮屏时状态未确定时的刷屏数据
UINT32	led_display_Vdata1[16];		//-最终输出显示的数据
UINT32	led_display_Vdata2[16];
UINT32	*led_display_Vdata_pt;
UINT8		led_display_pageing;		//-16行显示完了才可以切换,否则闪动

UINT8		led_display_page;		//-记录目前显示的是哪一页
UINT8		led_display_page_end;		//-记录目前系统可以显示的最大页号
UINT8		led_display_start;		//-值为0x55说明现在处于刷屏状态,0xaa 表示熄灭状态,0表示彻底熄灭
UINT8		led_display_ye_ok;		//-准备好了一页数据,保证刷完了才可以切换
UINT8   cticks_s_page;        //-没有触发条件延时一段时间后,熄灭屏幕
UINT8		led_display_long;			//-这是一个屏保时间,首先实现屏保计时,然后实现掉电保存
UINT16	led_display_end_time;	//-点阵屏熄灭之后的时长

UINT8		ps_flag_led;			//-挥动手势的情况	0 没有挥动;1 由上到下;2 由下到上,用于存储标志,供点阵屏使用,而不需要影响总ps_flag标志
UINT8		ps_flag_led_end;	//-记录最后一次挥动的手势
UINT8		ps_flag_led_disp;	//-用于完成显示过程
UINT8   led_display_move_pt;		//-左右移动时计数用的,相当于一个时间基准

UINT8		led_display_data_flag;		//-表示测量的数据是否是最新的:0 没有数据,需要等待更新;1 更新数据开始计时;2 数据有效;3 数据失效开始计时
UINT8		ps_flag_led_dis;		//-首次亮屏熄灭后

//-开机动画
UINT16	cartoon_page_change_time;
UINT16	cartoon_start_time;			//-记录开机动画运行的时间

//-测试观察用全局变量
UINT8	temp_data01_8;
UINT8	temp_data02_8;
UINT8	temp_data03_8;
UINT8	temp_data04_8;

UINT16	temp_data01;
UINT16	temp_data02;
UINT16	temp_data03;
UINT16	temp_data04;


UINT16		watch_data[512];		//-通用观察数据用数组

UINT8		watch_cfgdata;
UINT16		watch_cfgdata_16;

//-触摸按键
/*
BIN5		BIN4		BIN3		BIN2		BIN1		GUARD		Shield
CS3			CS1			CS0			CS4			CS5			CS2			CS15
D5      D4      D0      D1      D3			保护
*/
UINT8		CSKEY_cfgDATA;		//-读出的寄存器数据
UINT8		CSKEY_DATA;				//-形象的键值


//-by cheng CO2
UINT16 		Value_CO2_Buffer[10];
UINT8		  CO2_Counter=0;
char      ReadBuf_CO2[30];
int       Received_Over_time_co2,Received_Over_Flag_co2=0;
UINT8		  send_end_flag_co2;			//-值为1 表示发送结束了等待应答;值为0 已经应答不再等待
UINT8     Received_pt_co2=0;

UINT32    cticks_s_CO2;
int       co2_data;


//-by cheng pm2.5
char      ReadBuf_pm[30];
char 			Received_Over_Flag_pm;
UINT16 		Value_PM_Buffer[10];
UINT8 		PM_Counter=0; //-记录有几个有效数值了,待求平均数据
char      RcvStatus_pm;
UINT8     Received_pt_pm=0;

int       pm_data;


//-by cheng 温湿度
UINT16 		temperature_data;
UINT16 		humidity_data;

//-呼吸灯
//-char      interver;//pwm全局变量
UINT8     rgb_num;				//-位0 代表GREEN,位1 代表RED,位2 代表BLUE;各位1表示亮灯 0表示灭灯
UINT8		  ps_flag_pwm;		//-挥动手势的情况	0 没有挥动;1 由上到下;2 由下到上,用于存储标志,供PWM使用,而不需要影响总ps_flag标志
UINT32    cticks_ms_pwm_loop;	//-从灭到下一个灭,这样一个周期的长度
UINT32    cticks_ms_pwm_R;
UINT32    cticks_ms_pwm_G;
UINT32    cticks_ms_pwm_B;
UINT32    cticks_ms_pwm_4;
UINT16    GREEN_pwmval;
UINT16    RED_pwmval;
UINT16    BLUE_pwmval;
UINT16    GREEN_pwmval_pt;		//-偏移量
UINT16    RED_pwmval_pt;
UINT16    BLUE_pwmval_pt;
UINT16    white_pwmval_pt;
UINT16    GREEN_pwmval_num;		//-一个占空比占有的时长
UINT16    RED_pwmval_num;
UINT16    BLUE_pwmval_num;
UINT16    white_pwmval_num;
//-运用在彩灯上时
UINT8     HL_flag;		//-彩灯标志,用于表示彩灯的运行模式:0 关闭状态;1 彩灯状态
UINT16    HL_run_time;		//-彩灯运行时间计时
UINT16    HL_run_time_delay;
UINT8     HL_new_value_flag;		//-0 表示占空比没有变化;1 有灯的占空比发生了变化
UINT16    HL_GREEN_pwmval;
UINT16    HL_RED_pwmval;
UINT16    HL_BLUE_pwmval;
UINT8     HL_step;		//-控制炫彩灯运行的步奏
UINT8     HL_step_cn;
UINT8     HL_ld_R[6];		//-记录了每种颜色的亮度值(0~255)
UINT8     HL_ld_G[6];
UINT8     HL_ld_B[6];
UINT8     HL_ld_R_user[6];		//-用户记录了每种颜色的亮度值(0~255)
UINT8     HL_ld_G_user[6];
UINT8     HL_ld_B_user[6];
UINT8     HL_ld_brightness_flag;    //-0x55处于调光中 0没有
UINT8     HL_ld_brightness[2]={100,100};

//-噪声
#define maxbuffer 50
#define maxcounter 100
#define base 1723
UINT16 ADC_ConvertedValue[maxbuffer];
UINT16 ADC_ConvertedValue_TEMP[maxbuffer];
UINT8 Start_thansfer=0;
UINT8 ADC_Conv_flag;
UINT16 Noise_Value;

//-天气
UINT8 Weather_flag;		//-0 表示没有天气数据;1晴，2阴，3多云，4雨，5雪

//-全局使用标志,暂时定义
UINT8     sys_err_flag;		//-数据代表错误类型,多个一起错误现在不考虑
//-1 触摸按键总线出错
//-2 上手势传感器出错
//-3 下手势传感器出错

//-测试用数组,存储点阵屏位置信息
UINT16    test_pt;
UINT16    test_cn[8];
UINT16    test_cn_wait_time;
UINT32    test_SIN_data[16];


//-模拟EEPROM 使用FLASH
UINT32 EEP_Data;
UINT8  EEP_Data_flag;		//-0x55说明数据有修改,需要写入FLASH



float float_data;

//-三轴加速度计
UINT16    ADXL_X_data_flag;	//-0表示数据为正数,1 表示数据为负数
UINT16    ADXL_Y_data_flag;
UINT16    ADXL_Z_data_flag;
UINT16    ADXL_X_data;		//-为绝对值
UINT16    ADXL_Y_data;
UINT16    ADXL_Z_data;
UINT8 		ADXL_THRESH_TAP;	//-两个参数控制着敲击的灵敏度,有效阀值
UINT8 		ADXL_DUR;					//-阀值持续的时间

UINT8 		ADXL_TAP_it_flag;			//-值为0 没有中断;值为1 处于中断中
UINT8 		ADXL_TAP_it_SOURCE;		//-记录中断源值
UINT16		ADXL_TAP_wait_time;		//-这个时间用于防抖动
UINT8 		ADXL_TAP_off_flag;		//-当需要的时候把敲击功能关闭
UINT16		ADXL_TAP_off_time;		//-防止通讯出错永久关闭,加个最大延时

//-自检标识位
//-位0 三轴加速度计;位1 光敏;位2 手势
UINT8 		STM32_UP_selfT_flag1;		//-每一位代表一个器件的好坏:0 正常,1 异常.位0 三轴加速度
UINT8 		STM32_UP_error_flag1;		//-说明外设永久损坏需要重新上电或者更换器件
UINT8 		STM32_UP_selfT_cn1;
UINT8 		STM32_UP_selfT_cn2;
UINT8 		STM32_UP_selfT_cn3;


//-串口1
UINT8 		UART1_transmit_flag;		//-值为YES表示可以发送 值为NO表示不可以发送
UINT8 		UART1_transmit_control;	//-不同的值代表不同的发送内容

UINT8 		UART1_renew_flag;			//-0 表示可以通过串口更新数据库数据,0x55 表示不可以通过串口更新数据
UINT16 		UART1_renew_wait_time;



//-语音播报
UINT8 		voice_flag;				//-0 表示不播报;0x55 播报
UINT8 		voice_keep_data_flag;		//-显示数据是否被冻结,0 没有冻结;0x55 被冻结
UINT16 		voice_keep_data_time;		//-为了实现同步需要冻结一段时间直到播报结束


//-VOC
UINT16 		VOC_data;

//-语音
UINT8 		es705_training_count;   //-学习模式时成功学习的次数
UINT8 		es705_training_flag;		//-说明了语音的学习状态,可以控制其它流程
UINT16 		es705_training_wait_time;



