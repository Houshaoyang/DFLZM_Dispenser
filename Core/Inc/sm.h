#ifndef _SM_H_
#define _SM_H_


#include <stdint.h>

typedef enum
{
	STATE_IDLE,
	STATE_CHILD_LOCK,
	STATE_WATER_OUT,
	STATE_PRE_HEAT,
	STATE_DISINFECT,	
}State;

enum
{
	CHILD_LOCK_PRESS_EVT,
	TEMPER_CHG_EVT,
	WATER_OUT_PRESS_EVT,
	PRE_HEAT_PRESS_EVT,
	DISINFECTION_PRESS_EVT,
	DRY_BURNING_EVT,
	EVT_NUM,
};

typedef struct
{
	State CurrentState;					//饮水机当前状态
	uint8_t temp_setting;				//目标温度
	uint8_t disinfect_finish_flag; 		//加热消毒结束 1，未结束 0
	uint8_t disinfect_clr_water_flag;	//清空消毒，开始清空 1，未开启清空0
	uint8_t temper_index;
}WaterDispenser;

void WaterDispenser_Init(WaterDispenser *Dispenser);

void WaterDispenser_Eventhandler(WaterDispenser *Dispenser,int event);

void do_lock(WaterDispenser *Dispenser);
void lock_release(WaterDispenser *Dispenser);
void pre_heat(WaterDispenser *Dispenser);
void stop_pre_heat(WaterDispenser *Dispenser);
void water_out(WaterDispenser *Dispenser);
void stop_water_out(WaterDispenser *Dispenser);
void do_disinfection(WaterDispenser *Dispenser);
void stop_disinfection(WaterDispenser *Dispenser);
void change_target_temper(WaterDispenser *Dispenser);

#endif
