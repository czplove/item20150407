#ifndef _comuse_h
#define _comuse_h

/*
//Function
extern void Initial_CPUCOM(void);
extern void Initial_CAN(BYTE PortNo);
extern void init_port();
extern void init_all_port();
extern void port_send_begin_no_monitor(BYTE the_port_no);
extern void port_send_begin();

//extern void Ex_transmit_yk_cmd_unit();
//extern void Ex_transmit_yk_reply_unit();
extern void Ex_pipe_unit();
extern void Ex_pipe_port();

//应用 core  前提已释放 temp_loop,temp_loop1,temp_int,temp_lp_int 
extern void core_update_DC();
extern void core_update_YC();
extern void core_update_YX();
extern void core_update_YM();
extern void core_insert_SOE();
extern void core_insert_SOECOS();
extern void core_get_yx_set_unit();
extern void core_get_yc_set_unit();
extern void core_get_bh_bank_report(BYTE the_port);
// by x.zhao
extern BYTE Ex_Produce_Transmit_Info(void);
extern BYTE Ex_YK_CDTBB_ObjectNo_To_UnitAddr(void);



extern void Read_Time_From_Dallas(void);
extern void  Write_Time_To_Dallas(void);

extern void Store_Rcd_Info_Myself(void);
extern void Store_Rcd_Info_System(void);

extern void  Judge_P554_CAN_Reset(void);



extern void BCH_Calculate(void);
*/
extern BYTE     Judge_Time_In_MainLoop(WORD start_time,WORD judge_value);
extern BYTE     Judge_Time_In_OtherInt(WORD start_time,WORD judge_value);
extern BYTE Judge_LongTime_In_MainLoop(WORD start_time,WORD judge_value);
extern BYTE Judge_LongTime_In_OtherInt(WORD start_time,WORD judge_value);
extern void Host_LowLevelDelay(UINT32 milliseconds);
extern void TWI_Delay(void);
extern void NOP_Delay(UINT16 i);

extern void Clock_Process(void);

#endif /* _comuse_h */
