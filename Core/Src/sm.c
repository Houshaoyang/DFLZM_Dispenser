#include "sm.h"
#include "app.h"
#include "gpio.h"
//#include <stdio.h>


#define SACTIONS 2
#define SNEXT_STATE 2
#define NUM_COLS 3

enum
{
	ENTER_LOCK,  
	EXIT_LOCK,
	CHG_TARGET_TEMPER,
	ENTER_WATER_OUT,
	EXIT_WATER_OUT,
	CLEAR_WATER,
	ENTER_PRE_HEAT,
	EXIT_PRE_HEAT,
	ENTER_DISINFECT, 
	EXIT_DISINFECT,
	ERROR_HANDLE,
	ENTER_IDLE,
	EXIT_IDLE,
	SIGNORE
};

static const uint8_t state_table_idle[][NUM_COLS]=
{
/* Event                            Action 1               Action 2               Next state */
/* CHILD_LOCK_PRESS_EVT */          {EXIT_IDLE,							ENTER_LOCK,         			STATE_CHILD_LOCK },
/* TEMPER_CHG_EVT */                {EXIT_IDLE,   					ENTER_IDLE,       	  		STATE_IDLE },
/* WATER_OUT_PRESS_EVT */ 		   		{EXIT_IDLE,							ENTER_WATER_OUT,          STATE_WATER_OUT },
/* PRE_HEAT_PRESS_EVT */            {EXIT_IDLE,							ENTER_PRE_HEAT,            STATE_PRE_HEAT },
/* DISINFECTION_PRESS_EVT */        {EXIT_IDLE,							ENTER_DISINFECT,          STATE_DISINFECT },
/* ERROR_EVT			 */      					{SIGNORE,              SIGNORE,        			  STATE_CHILD_LOCK }
};

static const uint8_t state_table_child_lock[][NUM_COLS]=
{
/* Event                            Action 1               Action 2               Next state */
/* CHILD_LOCK_PRESS_EVT */          {EXIT_LOCK, 					 ENTER_IDLE,            STATE_IDLE },
/* TEMPER_CHG_EVT */                {CHG_TARGET_TEMPER,	   SIGNORE,       	 STATE_CHILD_LOCK },
/* WATER_OUT_PRESS_EVT */ 		  		{SIGNORE,     		   	 SIGNORE,        	 STATE_CHILD_LOCK },
/* PRE_HEAT_PRESS_EVT */            {SIGNORE,              SIGNORE,        	 STATE_CHILD_LOCK },
/* DISINFECTION_PRESS_EVT */        {SIGNORE,              SIGNORE,          STATE_CHILD_LOCK },
/* ERROR_EVT			 */      					{SIGNORE,              SIGNORE,        	 STATE_CHILD_LOCK }
};

static const uint8_t state_table_water_out[][NUM_COLS]=
{
/* Event                            Action 1               Action 2               Next state */
/* CHILD_LOCK_PRESS_EVT */          {SIGNORE,         	   SIGNORE,          STATE_WATER_OUT },
/* TEMPER_CHG_EVT */                {SIGNORE,	           SIGNORE,       	 STATE_WATER_OUT },
/* WATER_OUT_PRESS_EVT */ 		  	  {EXIT_WATER_OUT,       ENTER_LOCK,        	STATE_CHILD_LOCK },
/* PRE_HEAT_PRESS_EVT */            {SIGNORE,              SIGNORE,        	 STATE_WATER_OUT },
/* DISINFECTION_PRESS_EVT */        {SIGNORE,              SIGNORE,          STATE_WATER_OUT },
/* ERROR_EVT			 */      					{SIGNORE,              ENTER_LOCK,        	STATE_CHILD_LOCK }
};

static const uint8_t state_table_pre_heatting[][NUM_COLS]=
{
/* Event                            Action 1               Action 2               Next state */
/* CHILD_LOCK_PRESS_EVT */          {SIGNORE,         	   SIGNORE,          STATE_PRE_HEAT },
/* TEMPER_CHG_EVT */                {SIGNORE,       	  	 SIGNORE,       	 STATE_PRE_HEAT },
/* WATER_OUT_PRESS_EVT */ 				 	{SIGNORE,        	 		 SIGNORE,          STATE_PRE_HEAT },
/* PRE_HEAT_PRESS_EVT */            {EXIT_PRE_HEAT,        ENTER_LOCK,        STATE_CHILD_LOCK },
/* DISINFECTION_PRESS_EVT */        {SIGNORE,              SIGNORE,          STATE_PRE_HEAT },
/* ERROR_EVT			 */      					{SIGNORE,              ENTER_LOCK,        STATE_CHILD_LOCK }
};

static const uint8_t state_table_disinfection[][NUM_COLS]=
{
/* Event                            Action 1               Action 2               Next state */
/* CHILD_LOCK_PRESS_EVT */          {SIGNORE,         	   SIGNORE,          STATE_DISINFECT },
/* TEMPER_CHG_EVT */                {SIGNORE, 	    	 		 SIGNORE,       	 STATE_DISINFECT },
/* WATER_OUT_PRESS_EVT */ 					{CLEAR_WATER,      	   SIGNORE,          STATE_DISINFECT },
/* PRE_HEAT_PRESS_EVT */            {SIGNORE,              SIGNORE,        	 STATE_DISINFECT },
/* DISINFECTION_PRESS_EVT */        {EXIT_DISINFECT,       ENTER_LOCK,         STATE_CHILD_LOCK },
/* ERROR_EVT			 */      					{SIGNORE,              ENTER_LOCK,        	STATE_CHILD_LOCK }
};

typedef const uint8_t (*ST_TBL)[NUM_COLS];
static const ST_TBL st_tbl[] =
{
	state_table_idle,
	state_table_child_lock,
	state_table_water_out,
	state_table_pre_heatting,
	state_table_disinfection
};

void enter_lock(WaterDispenser *Dispenser)
{
	set_all_leds_status(LED_ON,LED_OFF,LED_OFF,LED_OFF,LED_OFF);

	mDispenser.heating_pwr = 0;                                    //Set heating power to xx%
	mDispenser.pump_speed = 0;                                     //Set pump speed to xx%
	mDispenser.heating_enabled = FALSE;                                 //Disable heating                               //Disable heating
	mDispenser.disinfect_finish_flag = FALSE;
	mDispenser.disinfect_clr_water_flag = FALSE;
	mDispenser.need_clear_container = FALSE;
	
	TW_Valve_IN;        //set three way valve state,GPIO_PIN_RESET: water out, GPIO_PIN_SET: water loop back
	PUMP_OFF;
	HEATER_OFF;
//	Alarm_Start(&mAlarm,0,0,10,LOCK_ALARM);
}

void exit_lock(WaterDispenser *Dispenser)
{
//	Alarm_Cancel(&mAlarm);	
}

void change_target_temper(WaterDispenser *Dispenser)
{
	mDispenser.temper_index ++;
	mDispenser.temper_index = mDispenser.temper_index % 5;
	Dispenser->temp_setting = target_temper_tbl[mDispenser.temper_index];
}

void enter_water_out(WaterDispenser *Dispenser)
{
	set_all_leds_status(LED_OFF,LED_OFF,LED_BLINK,LED_OFF,LED_OFF);
	if(mDispenser.temp_setting == 25)	{
		mDispenser.heating_enabled = FALSE; //Disable heating
		TW_Valve_OUT;	
	}else{
		mDispenser.heating_enabled = TURE;
		TW_Valve_IN;
	}

	switch(mDispenser.temp_setting)
	{
		case 25:
			mDispenser.pump_speed = PUMP_SPEED_25;	
			mDispenser.heating_pwr = HEATING_PWR_25;			//Set heating power to xx%
			break;
		case 45:
			mDispenser.pump_speed = PUMP_SPEED_45;
			mDispenser.heating_pwr = HEATING_PWR_45;		
			break;
		case 55:
			mDispenser.pump_speed = PUMP_SPEED_55;
			mDispenser.heating_pwr = HEATING_PWR_55;		
			break;
		case 85:
			mDispenser.pump_speed = PUMP_SPEED_85;	
			mDispenser.heating_pwr = HEATING_PWR_85;
			break;
		case 95:
			mDispenser.pump_speed = PUMP_SPEED_95;	
			mDispenser.heating_pwr = HEATING_PWR_95;
			break;
		defualt:break;
	}
}

void clear_water(WaterDispenser *Dispenser)
{
	if(Dispenser->disinfect_finish_flag == TURE)        //消毒结束后进入排空状态
	{
		if(Dispenser->disinfect_clr_water_flag == FALSE)  	
		{
			Dispenser->disinfect_clr_water_flag = TURE;	//排空未开启，开始排水
			mDispenser.pump_speed = 100;
		} else{
			Dispenser->disinfect_clr_water_flag = FALSE;	//排空已开启，停止排水
			mDispenser.pump_speed = 0;
		}
	}
}

void exit_water_out(WaterDispenser *Dispenser)
{                                 
}

void enter_preheat(WaterDispenser *Dispenser)
{
	set_all_leds_status(LED_OFF,LED_OFF,LED_OFF,LED_BLINK,LED_OFF);
	mDispenser.temper_index = 1;	//set target water temper 45℃
	Dispenser->temp_setting = target_temper_tbl[mDispenser.temper_index];
	
	mDispenser.heating_enabled = 1;                                 //Enable heating
	TW_Valve_IN;
	mDispenser.heating_pwr = HEATING_PWR_45;                                    //Set heating power to xx%
	mDispenser.pump_speed = PUMP_SPEED_45;                                    //Set pump speed to xx%
}

void exit_preheat(WaterDispenser *Dispenser)
{
	Alarm_Cancel(&mAlarm);
}

void enter_disinfection(WaterDispenser *Dispenser)
{
	set_all_leds_status(LED_OFF,LED_OFF,LED_OFF,LED_OFF,LED_BLINK);
	Alarm_Start(&mAlarm,0,0,10,DISINFECT_ALARM);
	mDispenser.temper_index = 3;  //set target water temper 85℃
	Dispenser->temp_setting = target_temper_tbl[mDispenser.temper_index];
	
	mDispenser.heating_enabled = TURE;                                 //Enable heating
	TW_Valve_IN;
	mDispenser.heating_pwr = HEATING_PWR_85;                                    //Set heating power to xx%
	mDispenser.pump_speed = PUMP_SPEED_85;                                    //Set pump speed to xx%
}

void exit_disinfection(WaterDispenser *Dispenser)
{
	if(mDispenser.need_clear_container == TURE){ //clear containter befor eixt disinfect state
		mDispenser.CurrentState = STATE_DISINFECT;
		mDispenser.disinfect_finish_flag = TURE;
		set_led_status(LED_ID_WATER_OUT,LED_BLINK);
		set_led_status(LED_ID_DISINFECT,LED_ON);
		return ;
	}

	mDispenser.heating_pwr = 0;                       		          //Set heating power to xx%
	mDispenser.pump_speed = 0;                                     //Set pump speed to xx%
	
	Alarm_Cancel(&mAlarm);

}

void error_handle(WaterDispenser *Dispenser)
{
//	mDispenser.heating_enabled = 0;                                 //Disable heating
//	HAL_GPIO_WritePin(TW_Valve, TW_Valve_IN);
//	mDispenser.heating_pwr = 0;                       		          //Set heating power to xx%
//	mDispenser.pump_speed = 0;  
}

void enter_idle(WaterDispenser *Dispenser)
{
	set_all_leds_status(LED_OFF,LED_OFF,LED_OFF,LED_OFF,LED_OFF);
	Alarm_Start(&mAlarm,0,0,10,IDLE_ALARM);
	if(mDispenser.fault_code != ERR_220V){ 	//if not ~220V fault ,cancel fault code
		mDispenser.fault_code = NO_FAULT;
	}
}

void exit_idle(WaterDispenser *Dispenser)
{
	Alarm_Cancel(&mAlarm);
}

typedef void (*WATER_DISPENSER_ACT)(WaterDispenser *Dispenser);
const WATER_DISPENSER_ACT WaterDispenser_action[] =
{
	enter_lock,    		 			
	exit_lock,         			
	change_target_temper,		
	enter_water_out, 					
	exit_water_out,   
	clear_water,
	enter_preheat,         
	exit_preheat,    				
	enter_disinfection,      
	exit_disinfection,    
	error_handle,
	enter_idle,    				
	exit_idle,
	NULL
};

void WaterDispenser_Eventhandler(WaterDispenser *Dispenser,int event)
{
	ST_TBL state_table;
	uint8_t action;
	int i;
	
	state_table = st_tbl[Dispenser->CurrentState];	
	Dispenser->CurrentState = state_table[event][SNEXT_STATE];
	for(i=0;i<SACTIONS;i++)
	{
		if((action = state_table[event][i]) != SIGNORE)
		{
			WaterDispenser_action[action](Dispenser);
		}
	}
}
