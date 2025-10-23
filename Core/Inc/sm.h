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

typedef enum{
	NO_FAULT,
	ERR_DRY_BURNING,				//
	ERR_WATER_SHORTAGE,
	ERR_WATER_OUTLET_FOLD,	//WATER_OUTLET fold but bump working
}Err_code;

typedef struct
{
	State CurrentState;					//饮水机当前状态
	uint8_t temp_setting;				//目标温度
	uint8_t disinfect_finish_flag; 		//加热消毒结束 1，未结束 0
	uint8_t disinfect_clr_water_flag;	//清空消毒，开始清空 1，未开启清空0
	uint8_t temper_index;
	Err_code fault_code;
	uint8_t heating_enabled;        // 
	uint8_t heating_pwr;        // 
	uint8_t pump_speed;             // 
	uint8_t need_clear_container;             // 
}WaterDispenser;

void enter_lock(WaterDispenser *Dispenser);
void WaterDispenser_Init(WaterDispenser *Dispenser);
void WaterDispenser_Eventhandler(WaterDispenser *Dispenser,int event);

#endif
