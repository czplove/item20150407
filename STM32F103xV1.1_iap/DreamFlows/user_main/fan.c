/*

*/
#include "user_conf.h"


#define FAN_RUN(BitVal)  		GPIO_WriteBit(GPIOA,GPIO_Pin_1,BitVal)


void FAN_sub(void)
{
	 //-FAN_RUN((BitAction)1);   //-��ʱ������
	 if(FAN_RUN_flag == 1)
	 {	
	 	   FAN_RUN((BitAction)1);
			 if((FAN_RUN_wait_flag == 1) && (Judge_LongTime_In_MainLoop(FAN_RUN_wait_time,58)==YES))		//-146Լ5����
		   {
		      FAN_RUN_wait_time = Time_2048ms_Counter;	
		      FAN_RUN_wait_flag = 0;
		      FAN_RUN_flag = 0;	
		      FAN_RUN((BitAction)0); 													
		   }
	 }
}