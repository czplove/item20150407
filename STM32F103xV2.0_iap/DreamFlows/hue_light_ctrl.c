#include "user_conf.h"

/*
T3初始化 1ms---定时		中断处理函数 用于指示灯的调光 
T4初始化 1ms---定时                用于应用的调光

上面可能采用的是人家的模式现在根据实际效果自己来调
现在有两种方案:
1.RGB 定值可以随意定
由于调值范围是0~255,所以只要把PWM这么多等分,三个色元可以随意修改就行.
2.自由呼吸

现在决定很多东西放在1mS中断内处理,那么设置的定时器周期必须大于1mS,但是又不能大太多
这样会导致更新的数据有丢失,那么PWM周期也设为1mS


 */


////////////////////////////////////////////////////////////////////////////////


void TIM1_Configuration_PWM_R(void)
{         
   TIM_SetCompare1(TIM3, RED_pwmval);
   TIM_SetCompare2(TIM3, BLUE_pwmval);
   TIM_SetCompare3(TIM3, GREEN_pwmval);
}

void TIM1_Configuration_PWM_G(void)
{
   TIM_SetCompare1(TIM3, RED_pwmval);
   TIM_SetCompare2(TIM3, BLUE_pwmval);
   TIM_SetCompare3(TIM3, GREEN_pwmval);
}

void TIM1_Configuration_PWM_B(void)
{
   TIM_SetCompare1(TIM3, RED_pwmval);
   TIM_SetCompare2(TIM3, BLUE_pwmval);
   TIM_SetCompare3(TIM3, GREEN_pwmval);
}

void TIM1_Conf_HL_PWM_RGB(void)	//-把全量程分为255等份,然后接收任何数值参数
{
	 TIM_SetCompare1(TIM3, HL_RED_pwmval);
   TIM_SetCompare2(TIM3, HL_BLUE_pwmval);
   TIM_SetCompare3(TIM3, HL_GREEN_pwmval);
}

//-2015/5/19 13:31:00
//-这里是整个彩灯的调度入口程序处,灯的呼吸,也就是亮灭没有问题了现在需要可控,比如说调色.
void huelight_sub(void)
{
	 //-UINT16 interver;
	  
	  //-printf("\r\ntest printf is %d\n",0);
	 	
	 	//TIM1_Configuration_PWM_RGB();		
}

/*
关闭的时候可以停留在任何一种颜色的任何一个亮度上.
其他情况下都在做呼吸
*/
void huelight_sub_it(void)
{
	float temp_float_data;
	UINT32    temp_cticks_ms_pwm_loop;
	
	 if(HL_flag == 0)
	 {//-0 关闭状态,熄灭所有的灯
	 	   if(HL_new_value_flag == 0)
			 {//-0 表示占空比没有变化,,那么保持原有的占空比无需调整
			 			
			 }
			 else
			 {//-1 有灯的占空比发生了变化
			 	   //-HL_RED_pwmval = 0;
		 	  	 //-HL_GREEN_pwmval = 0;
		 	  	 //-HL_BLUE_pwmval = 0;
			 	   TIM1_Conf_HL_PWM_RGB();
			 	   HL_new_value_flag = 0;
			 }	 	
	 }
	 else if(HL_flag == 1)
	 {//-2 用户状态,可以随意用户设置,只能设置一种颜色
	 	   if(HL_new_value_flag == 0)
			 {//-0 表示占空比没有变化,,那么保持原有的占空比无需调整
			 			
			 }
			 else
			 {//-1 有灯的占空比发生了变化
          if(HL_ld_brightness_flag == 0)
          {
             temp_float_data = ((float)HL_ld_brightness[0] / (float)100);
             HL_RED_pwmval = (UINT16)(PWM_Period_Value * ((float)HL_ld_R_user[0] / (float)255) * temp_float_data);
             HL_GREEN_pwmval = (UINT16)(PWM_Period_Value * ((float)HL_ld_G_user[0] / (float)255) * temp_float_data);
             HL_BLUE_pwmval = (UINT16)(PWM_Period_Value * ((float)HL_ld_B_user[0] / (float)255) * temp_float_data);
             TIM1_Conf_HL_PWM_RGB();
             HL_new_value_flag = 0;
          }
          else
          {
              if(cticks_ms_pwm_R >= 15)
              {
                 if(HL_ld_brightness[0] != HL_ld_brightness[1])
                 {
                    if(HL_ld_brightness[0] >= HL_ld_brightness[1])
                    {
                        HL_ld_brightness[0]--;
                    }
                    else
                    {
                        HL_ld_brightness[0]++;
                    }
                    
                    temp_float_data = ((float)HL_ld_brightness[0] / (float)100);
                     HL_RED_pwmval = (UINT16)(PWM_Period_Value * ((float)HL_ld_R_user[0] / (float)255) * temp_float_data);
                     HL_GREEN_pwmval = (UINT16)(PWM_Period_Value * ((float)HL_ld_G_user[0] / (float)255) * temp_float_data);
                     HL_BLUE_pwmval = (UINT16)(PWM_Period_Value * ((float)HL_ld_B_user[0] / (float)255) * temp_float_data);
                     TIM1_Conf_HL_PWM_RGB();
                     //-HL_new_value_flag = 0;
                 }
                 else
                 {
                     temp_float_data = ((float)HL_ld_brightness[0] / (float)100);
                     HL_RED_pwmval = (UINT16)(PWM_Period_Value * ((float)HL_ld_R_user[0] / (float)255) * temp_float_data);
                     HL_GREEN_pwmval = (UINT16)(PWM_Period_Value * ((float)HL_ld_G_user[0] / (float)255) * temp_float_data);
                     HL_BLUE_pwmval = (UINT16)(PWM_Period_Value * ((float)HL_ld_B_user[0] / (float)255) * temp_float_data);
                     TIM1_Conf_HL_PWM_RGB();
                     HL_new_value_flag = 0;
                     HL_ld_brightness_flag = 0;
                 }
                  cticks_ms_pwm_R = 0;
              }
          }
			 }
	 	  	 	  	 	  	 	 	 	
	 }	
	 else if(HL_flag == 2)	//-烛光晚餐
	 {
	 	  if(HL_step == 0)	//-每一步固定一个频率的呼吸,不同步之间可以修改
	 	  {
	 	  	 if((cticks_ms_pwm_R >= RED_pwmval_num) && ((rgb_num & 0x02) != 0))
		 	   {
		 	   	 RED_pwmval_pt++;
		 	   	 temp_cticks_ms_pwm_loop = 2000;
		 	  	 if((RED_pwmval_pt * RED_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 RED_pwmval_pt = 0;
		 	  	 	 HL_step = 1;
		 	  	 } 	 
		 	  	 temp_float_data = sin((((float)RED_pwmval_pt * (float)RED_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 //-float_data = (float)PWM_Period_Value / (cticks_ms_pwm_loop / RED_pwmval_num);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)196 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)116 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)6 / (float)255));
		 	  	 //-GREEN_pwmval = PWM_Period_Value;		//-亮灯
		 	  	 //-BLUE_pwmval = PWM_Period_Value;
		 	  	 TIM1_Configuration_PWM_R();
		 	  	 cticks_ms_pwm_R = 0;
		 	   }
	 	  }
	 	  else if(HL_step == 1)
	 	  {
	 	  	 if((cticks_ms_pwm_G > GREEN_pwmval_num) && ((rgb_num & 0x01) != 0))
		 	   {
		 	  	 GREEN_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 300;
		 	  	 if((GREEN_pwmval_pt * GREEN_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 GREEN_pwmval_pt = 0;
		 	  	 	 HL_step = 2;
		 	  	 } 
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)GREEN_pwmval_pt * (float)GREEN_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)255 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)192 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)0 / (float)255));
		 	  	 TIM1_Configuration_PWM_G();
		 	  	 cticks_ms_pwm_G = 0;
		 	   }
	 	  }
	 	  else if(HL_step == 2)
	 	  {
	 	  	 if((cticks_ms_pwm_B > BLUE_pwmval_num) && ((rgb_num & 0x04) != 0))
		 	   {
		 	  	 BLUE_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 300;
		 	  	 if((BLUE_pwmval_pt * BLUE_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 BLUE_pwmval_pt = 0;
		 	  	 	 HL_step = 3;
		 	  	 }
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)BLUE_pwmval_pt * (float)BLUE_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)255 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)204 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)10 / (float)255));
		 	  	 TIM1_Configuration_PWM_B();
		 	  	 cticks_ms_pwm_B = 0;
		 	   }
	 	  }
	 	  else if(HL_step == 3)
	 	  {
	 	  	 if((cticks_ms_pwm_B > BLUE_pwmval_num) && ((rgb_num & 0x04) != 0))
		 	   {
		 	  	 BLUE_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 300;
		 	  	 if((BLUE_pwmval_pt * BLUE_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 BLUE_pwmval_pt = 0;
		 	  	 	 HL_step = 4;
		 	  	 }
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)BLUE_pwmval_pt * (float)BLUE_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)254 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)255 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)10/ (float)255));
		 	  	 TIM1_Configuration_PWM_B();
		 	  	 cticks_ms_pwm_B = 0;
		 	   }
	 	  }
	 	  else if(HL_step == 4)
	 	  {
	 	  	 if((cticks_ms_pwm_B > BLUE_pwmval_num) && ((rgb_num & 0x04) != 0))
		 	   {
		 	  	 BLUE_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 300;
		 	  	 if((BLUE_pwmval_pt * BLUE_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 BLUE_pwmval_pt = 0;
		 	  	 	 HL_step = 5;
		 	  	 }
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)BLUE_pwmval_pt * (float)BLUE_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)255 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)204 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)10 / (float)255));
		 	  	 TIM1_Configuration_PWM_B();
		 	  	 cticks_ms_pwm_B = 0;
		 	   }
	 	  }
	 	  else
	 	  {
	 	  	 if((cticks_ms_pwm_4 > white_pwmval_num) && ((rgb_num & 0x04) != 0))
		 	   {
		 	  	 white_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 300;
		 	  	 if((white_pwmval_pt * white_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 white_pwmval_pt = 0;
		 	  	 	 HL_step_cn++;
		 	  	 	 if(HL_step_cn >= 3)
		 	  	 	 {
		 	  	 	 	  HL_step_cn = 0;
		 	  	 	 	  HL_step = 0;
		 	  	 	 }
		 	  	 	 else
		 	  	 	 {		
		 	  	 	 		HL_step = 1;
		 	  	 	 }
		 	  	 }
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)white_pwmval_pt * (float)white_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)255 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)192 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)0 / (float)255));
		 	  	 TIM1_Configuration_PWM_B();
		 	  	 cticks_ms_pwm_4 = 0;
		 	   }
	 	  }
	 	  	 	  	 	  	 	 	 	
	 }
	 else if(HL_flag == 3)	//-月色迷人
	 {
	 	  if(HL_step == 0)	//-每一步固定一个频率的呼吸,不同步之间可以修改
	 	  {
	 	  	 if((cticks_ms_pwm_R >= RED_pwmval_num) && ((rgb_num & 0x02) != 0))
		 	   {
		 	   	 RED_pwmval_pt++;
		 	   	 temp_cticks_ms_pwm_loop = 3000;
		 	  	 if((RED_pwmval_pt * RED_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 RED_pwmval_pt = 0;
		 	  	 	 HL_step = 1;
		 	  	 } 	 
		 	  	 temp_float_data = sin((((float)RED_pwmval_pt * (float)RED_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 //-float_data = (float)PWM_Period_Value / (cticks_ms_pwm_loop / RED_pwmval_num);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)112 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)48 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)160 / (float)255));
		 	  	 //-GREEN_pwmval = PWM_Period_Value;		//-亮灯
		 	  	 //-BLUE_pwmval = PWM_Period_Value;
		 	  	 TIM1_Configuration_PWM_R();
		 	  	 cticks_ms_pwm_R = 0;
		 	   }
	 	  }
	 	  else if(HL_step == 1)
	 	  {
	 	  	 if((cticks_ms_pwm_G > GREEN_pwmval_num) && ((rgb_num & 0x01) != 0))
		 	   {
		 	  	 GREEN_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 3000;
		 	  	 if((GREEN_pwmval_pt * GREEN_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 GREEN_pwmval_pt = 0;
		 	  	 	 HL_step = 2;
		 	  	 } 
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)GREEN_pwmval_pt * (float)GREEN_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)145 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)72 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)200 / (float)255));
		 	  	 TIM1_Configuration_PWM_G();
		 	  	 cticks_ms_pwm_G = 0;
		 	   }
	 	  }
	 	  else if(HL_step == 2)
	 	  {
	 	  	 if((cticks_ms_pwm_B > BLUE_pwmval_num) && ((rgb_num & 0x04) != 0))
		 	   {
		 	  	 BLUE_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 3000;
		 	  	 if((BLUE_pwmval_pt * BLUE_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 BLUE_pwmval_pt = 0;
		 	  	 	 HL_step = 3;
		 	  	 }
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)BLUE_pwmval_pt * (float)BLUE_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)168 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)110 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)212 / (float)255));
		 	  	 TIM1_Configuration_PWM_B();
		 	  	 cticks_ms_pwm_B = 0;
		 	   }
	 	  }
	 	  else if(HL_step == 3)
	 	  {
	 	  	 if((cticks_ms_pwm_B > BLUE_pwmval_num) && ((rgb_num & 0x04) != 0))
		 	   {
		 	  	 BLUE_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 3000;
		 	  	 if((BLUE_pwmval_pt * BLUE_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 BLUE_pwmval_pt = 0;
		 	  	 	 HL_step = 4;
		 	  	 }
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)BLUE_pwmval_pt * (float)BLUE_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)255 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)153 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)255 / (float)255));
		 	  	 TIM1_Configuration_PWM_B();
		 	  	 cticks_ms_pwm_B = 0;
		 	   }
	 	  }
	 	  else if(HL_step == 4)
	 	  {
	 	  	 if((cticks_ms_pwm_B > BLUE_pwmval_num) && ((rgb_num & 0x04) != 0))
		 	   {
		 	  	 BLUE_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 3000;
		 	  	 if((BLUE_pwmval_pt * BLUE_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 BLUE_pwmval_pt = 0;
		 	  	 	 HL_step = 5;
		 	  	 }
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)BLUE_pwmval_pt * (float)BLUE_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)255 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)102 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)204 / (float)255));
		 	  	 TIM1_Configuration_PWM_B();
		 	  	 cticks_ms_pwm_B = 0;
		 	   }
	 	  }
	 	  else
	 	  {
	 	  	 if((cticks_ms_pwm_4 > white_pwmval_num) && ((rgb_num & 0x04) != 0))
		 	   {
		 	  	 white_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 3000;
		 	  	 if((white_pwmval_pt * white_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 white_pwmval_pt = 0;
		 	  	 	 HL_step = 0;
		 	  	 }
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)white_pwmval_pt * (float)white_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)255 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)51 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)204 / (float)255));
		 	  	 TIM1_Configuration_PWM_B();
		 	  	 cticks_ms_pwm_4 = 0;
		 	   }
	 	  }
	 	  	 	  	 	  	 	 	 	
	 }
	 else if(HL_flag == 4)	//-蓝调小夜曲
	 {
	 	  if(HL_step == 0)	//-每一步固定一个频率的呼吸,不同步之间可以修改
	 	  {
	 	  	 if((cticks_ms_pwm_R >= RED_pwmval_num) && ((rgb_num & 0x02) != 0))
		 	   {
		 	   	 RED_pwmval_pt++;
		 	   	 temp_cticks_ms_pwm_loop = 3000;
		 	  	 if((RED_pwmval_pt * RED_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 RED_pwmval_pt = 0;
		 	  	 	 HL_step = 1;
		 	  	 } 	 
		 	  	 temp_float_data = sin((((float)RED_pwmval_pt * (float)RED_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 //-float_data = (float)PWM_Period_Value / (cticks_ms_pwm_loop / RED_pwmval_num);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)0 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)24 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)255 / (float)255));
		 	  	 //-GREEN_pwmval = PWM_Period_Value;		//-亮灯
		 	  	 //-BLUE_pwmval = PWM_Period_Value;
		 	  	 TIM1_Configuration_PWM_R();
		 	  	 cticks_ms_pwm_R = 0;
		 	   }
	 	  }
	 	  else if(HL_step == 1)
	 	  {
	 	  	 if((cticks_ms_pwm_G > GREEN_pwmval_num) && ((rgb_num & 0x01) != 0))
		 	   {
		 	  	 GREEN_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 2000;
		 	  	 if((GREEN_pwmval_pt * GREEN_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 GREEN_pwmval_pt = 0;
		 	  	 	 HL_step = 2;
		 	  	 } 
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)GREEN_pwmval_pt * (float)GREEN_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)2 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)85 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)254 / (float)255));
		 	  	 TIM1_Configuration_PWM_G();
		 	  	 cticks_ms_pwm_G = 0;
		 	   }
	 	  }
	 	  else if(HL_step == 2)
	 	  {
	 	  	 if((cticks_ms_pwm_B > BLUE_pwmval_num) && ((rgb_num & 0x04) != 0))
		 	   {
		 	  	 BLUE_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 1000;
		 	  	 if((BLUE_pwmval_pt * BLUE_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 BLUE_pwmval_pt = 0;
		 	  	 	 HL_step = 3;
		 	  	 }
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)BLUE_pwmval_pt * (float)BLUE_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)11 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)139 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)226 / (float)255));
		 	  	 TIM1_Configuration_PWM_B();
		 	  	 cticks_ms_pwm_B = 0;
		 	   }
	 	  }
	 	  else if(HL_step == 3)
	 	  {
	 	  	 if((cticks_ms_pwm_B > BLUE_pwmval_num) && ((rgb_num & 0x04) != 0))
		 	   {
		 	  	 BLUE_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 3000;
		 	  	 if((BLUE_pwmval_pt * BLUE_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 BLUE_pwmval_pt = 0;
		 	  	 	 HL_step = 4;
		 	  	 }
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)BLUE_pwmval_pt * (float)BLUE_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)0 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)112 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)192 / (float)255));
		 	  	 TIM1_Configuration_PWM_B();
		 	  	 cticks_ms_pwm_B = 0;
		 	   }
	 	  }
	 	  else if(HL_step == 4)
	 	  {
	 	  	 if((cticks_ms_pwm_B > BLUE_pwmval_num) && ((rgb_num & 0x04) != 0))
		 	   {
		 	  	 BLUE_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 2000;
		 	  	 if((BLUE_pwmval_pt * BLUE_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 BLUE_pwmval_pt = 0;
		 	  	 	 HL_step = 5;
		 	  	 }
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)BLUE_pwmval_pt * (float)BLUE_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)2 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)85 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)254 / (float)255));
		 	  	 TIM1_Configuration_PWM_B();
		 	  	 cticks_ms_pwm_B = 0;
		 	   }
	 	  }
	 	  else
	 	  {
	 	  	 if((cticks_ms_pwm_4 > white_pwmval_num) && ((rgb_num & 0x04) != 0))
		 	   {
		 	  	 white_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 1000;
		 	  	 if((white_pwmval_pt * white_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 white_pwmval_pt = 0;
		 	  	 	 HL_step = 0;
		 	  	 }
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)white_pwmval_pt * (float)white_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)11 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)139 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)226 / (float)255));
		 	  	 TIM1_Configuration_PWM_B();
		 	  	 cticks_ms_pwm_4 = 0;
		 	   }
	 	  }
	 	  	 	  	 	  	 	 	 	
	 }
	 else if(HL_flag == 5)	//-绿色心情
	 {
	 	  if(HL_step == 0)	//-每一步固定一个频率的呼吸,不同步之间可以修改
	 	  {
	 	  	 if((cticks_ms_pwm_R >= RED_pwmval_num) && ((rgb_num & 0x02) != 0))
		 	   {
		 	   	 RED_pwmval_pt++;
		 	   	 temp_cticks_ms_pwm_loop = 3000;
		 	  	 if((RED_pwmval_pt * RED_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 RED_pwmval_pt = 0;
		 	  	 	 HL_step = 1;
		 	  	 } 	 
		 	  	 temp_float_data = sin((((float)RED_pwmval_pt * (float)RED_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 //-float_data = (float)PWM_Period_Value / (cticks_ms_pwm_loop / RED_pwmval_num);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)0 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)255 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)0 / (float)255));
		 	  	 //-GREEN_pwmval = PWM_Period_Value;		//-亮灯
		 	  	 //-BLUE_pwmval = PWM_Period_Value;
		 	  	 TIM1_Configuration_PWM_R();
		 	  	 cticks_ms_pwm_R = 0;
		 	   }
	 	  }
	 	  else if(HL_step == 1)
	 	  {
	 	  	 if((cticks_ms_pwm_G > GREEN_pwmval_num) && ((rgb_num & 0x01) != 0))
		 	   {
		 	  	 GREEN_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 1000;
		 	  	 if((GREEN_pwmval_pt * GREEN_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 GREEN_pwmval_pt = 0;
		 	  	 	 HL_step = 2;
		 	  	 } 
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)GREEN_pwmval_pt * (float)GREEN_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)51 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)242 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)7 / (float)255));
		 	  	 TIM1_Configuration_PWM_G();
		 	  	 cticks_ms_pwm_G = 0;
		 	   }
	 	  }
	 	  else if(HL_step == 2)
	 	  {
	 	  	 if((cticks_ms_pwm_B > BLUE_pwmval_num) && ((rgb_num & 0x04) != 0))
		 	   {
		 	  	 BLUE_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 1000;
		 	  	 if((BLUE_pwmval_pt * BLUE_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 BLUE_pwmval_pt = 0;
		 	  	 	 HL_step = 3;
		 	  	 }
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)BLUE_pwmval_pt * (float)BLUE_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)30 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)233 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)5 / (float)255));
		 	  	 TIM1_Configuration_PWM_B();
		 	  	 cticks_ms_pwm_B = 0;
		 	   }
	 	  }
	 	  else if(HL_step == 3)
	 	  {
	 	  	 if((cticks_ms_pwm_B > BLUE_pwmval_num) && ((rgb_num & 0x04) != 0))
		 	   {
		 	  	 BLUE_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 1000;
		 	  	 if((BLUE_pwmval_pt * BLUE_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 BLUE_pwmval_pt = 0;
		 	  	 	 HL_step = 0;
		 	  	 }
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)BLUE_pwmval_pt * (float)BLUE_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)20 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)240 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)20 / (float)255));
		 	  	 TIM1_Configuration_PWM_B();
		 	  	 cticks_ms_pwm_B = 0;
		 	   }
	 	  }
	 	  else if(HL_step == 4)
	 	  {
	 	  	 if((cticks_ms_pwm_B > BLUE_pwmval_num) && ((rgb_num & 0x04) != 0))
		 	   {
		 	  	 BLUE_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 3000;
		 	  	 if((BLUE_pwmval_pt * BLUE_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 BLUE_pwmval_pt = 0;
		 	  	 	 HL_step = 0;
		 	  	 }
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)BLUE_pwmval_pt * (float)BLUE_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)153 / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)255 / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)51 / (float)255));
		 	  	 TIM1_Configuration_PWM_B();
		 	  	 cticks_ms_pwm_B = 0;
		 	   }
	 	  }
	 	  else
	 	  {
	 	  	 if((cticks_ms_pwm_4 > white_pwmval_num) && ((rgb_num & 0x04) != 0))
		 	   {
		 	  	 white_pwmval_pt++;
		 	  	 temp_cticks_ms_pwm_loop = 3000;
		 	  	 if((white_pwmval_pt * white_pwmval_num) >= temp_cticks_ms_pwm_loop)
		 	  	 {
		 	  	 	 white_pwmval_pt = 0;
		 	  	 	 HL_step = 0;
		 	  	 }
		 	  	 	 	 
		 	  	 temp_float_data = sin((((float)white_pwmval_pt * (float)white_pwmval_num) / (float)temp_cticks_ms_pwm_loop) * pi);
		 	  	 RED_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)HL_ld_R_user[HL_step] / (float)255));
		 	  	 GREEN_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)HL_ld_G_user[HL_step] / (float)255));
		 	  	 BLUE_pwmval = (UINT16)(((float)PWM_Period_Value * temp_float_data) * ((float)HL_ld_B_user[HL_step] / (float)255));
		 	  	 TIM1_Configuration_PWM_B();
		 	  	 cticks_ms_pwm_4 = 0;
		 	   }
	 	  }
	 	  	 	  	 	  	 	 	 	
	 }
	
	   
  
  //-测试全亮
  //-HL_RED_pwmval = (PWM_Period_Value * 255) / 255;
	//-HL_GREEN_pwmval = (PWM_Period_Value * 255) / 255;
	//-HL_BLUE_pwmval = (PWM_Period_Value * 255) / 255;
	//-TIM1_Conf_HL_PWM_RGB();
}

