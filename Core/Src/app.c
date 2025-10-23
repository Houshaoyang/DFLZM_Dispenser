#include "app.h"
//#include <stdio.h>
#include <stdlib.h>
#include "adc.h"
//===========================================================
//----------Temperature comparison table (ADC values corresponding to 0 to 100℃)
static const unsigned int temptab[]={  //0 to 100℃ 
  2809, 2764, 2718, 2671, 2625, 2577, 2530, 2482, 2434, 2386, //0-9
  2337, 2289, 2241, 2193, 2145, 2097, 2049, 2002, 1955, 1908,
  1862, 1816, 1771, 1726, 1682, 1638, 1595, 1553, 1512, 1471,
  1431, 1391, 1353, 1315, 1278, 1241, 1206, 1171, 1137, 1104,
  1072, 1040, 1009, 979, 950, 922, 894, 867, 841, 815,
  791, 767, 743, 721, 699, 677, 657, 636, 617, 598,
  580, 562, 545, 528, 512, 496, 481, 467, 452, 439,
  425, 412, 400, 388, 376, 365, 354, 343, 333, 323,
  313, 304, 295, 286, 278, 270, 262, 254, 247, 240,
  233, 226, 220, 213, 207, 201, 196, 190, 185, 180	//90-99
}; 

#ifdef ENABLE_DEBUG_DISPLAY
//Segment code display related enumeration definitions
enum{
	A1=0,
	B1,
	C1,
	D1,
	E1,
	F1,
	G1,
	H1,
	A2,
	B2,
	C2,
	D2,
	E2,
	F2,
	G2
};

//Dot matrix decoding table for configuring GPIO output modes and states
const uint8_t DotDecode[][10] = {
//        Dr1_dir        Dr1        
/*A1*/ {GPIO_MODE_OUTPUT_PP, GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT, GPIO_MODE_INPUT, GPIO_MODE_INPUT,  1,0,0,0,0},
/*B1*/ {GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT, GPIO_MODE_INPUT,  1,0,0,0,0},
/*C1*/ {GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT, GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT,  1,0,0,0,0},
/*D1*/ {GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT, GPIO_MODE_INPUT, GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP,  1,0,0,0,0},
/*E1*/ {GPIO_MODE_OUTPUT_PP, GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT, GPIO_MODE_INPUT, GPIO_MODE_INPUT,  0,1,0,0,0},
/*F1*/ {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT, GPIO_MODE_INPUT,  0,1,0,0,0},
/*G1*/ {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT, GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP,  0,1,0,0,0},  //DuFeiPeng design board
/*H1*/ {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT,  0,1,0,0,0},   //DuFeiPeng design board
/*A2*/ {GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT, GPIO_MODE_INPUT,  0,0,1,0,0},
/*B2*/ {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT, GPIO_MODE_INPUT,  0,0,1,0,0},
/*C2*/ {GPIO_MODE_INPUT, GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT,  0,0,1,0,0},
/*D2*/ {GPIO_MODE_INPUT, GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP,  0,0,1,0,0},
/*F2*/ {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT,  0,0,0,1,0},
/*E2*/ {GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT, GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT,  0,0,0,1,0},
/*G2*/ {GPIO_MODE_INPUT, GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_OUTPUT_PP, GPIO_MODE_INPUT,  0,0,0,1,0}
};

//Segment code table for 7-segment display of 0-9 and some characters
const uint8_t Segment_Code[]={
//Xx   		 HGFEDCBA
0x3F, // 0B00111111 (0)   1:LED on 0:LED off
0x06, // 0B00000110 (1)
0x5B, // 0B01011011 (2)
0x4F, // 0B01001111 (3)
0x66, // 0B01100110 (4)
0x6D, // 0B01101101 (5)
0x7D, // 0B01111101 (6)
0x07, // 0B00000111 (7)
0x7F, // 0B01111111 (8)
0x6F, // 0B01101111 (9)
0x79, // 0B01111001 (E)
};
#endif

//Key structure instance definitions
key key_child_lock = {
	KEY_CHILD_LOCK,
	KEY_UP,
	EVT_NUM,
	CHILD_LOCK_PRESS_EVT,
    0,
    0
	} ,	\
	key_temper_chg = {
	KEY_TEMPER_CHG,
	KEY_UP,
	TEMPER_CHG_EVT,
	TEMPER_CHG_EVT,
    0,
    0
	} , \
	key_water_out= {
	KEY_WATER_OUT,
	KEY_UP,
	EVT_NUM,
	WATER_OUT_PRESS_EVT,
    0,
    0
	}  , \
	key_pre_heat = {
	KEY_PRE_HEAT,
	KEY_UP,
	PRE_HEAT_PRESS_EVT,
	PRE_HEAT_PRESS_EVT,
    0,
    0
	} , \
	key_disinfect= {
	 KEY_DISINFECT,
	 KEY_UP,
   DISINFECTION_PRESS_EVT,
	 DISINFECTION_PRESS_EVT,
    0,
    0
	} ;

//LED status table
led led_tbl[] = {
	{LED_ID_CHILD_LOCK,  LED_OFF},   //Child lock LED initial state is on
	{LED_ID_TEMPER_CHG,  LED_OFF},   //Temperature adjustment LED initial state is off
	{LED_ID_WATER_OUT,   LED_OFF},   //Water outlet LED initial state is off
	{LED_ID_PRE_HEAT,    LED_OFF},   //Preheat LED initial state is off
	{LED_ID_DISINFECT,   LED_OFF}};  //Disinfection LED initial state is off

//Alarm structure instance
alarm mAlarm;
//Water dispenser structure instance
WaterDispenser mDispenser;
	
#ifdef ENABLE_DEBUG_PTC
//PTC (heating element) related structure
ptc	ptc_in,ptc_out;
#define NUM_ADC_CHANNEL 2
//uint32_t adc_value[2];  //ADC sampling value array
#endif
	
//Flow pulse counter
pulse_counter iFlow,PassZero_Detect;

//Flow value
volatile uint16_t Flow_Value = 0;

// PID control parameters
#define INTEGRAL_ENABLE_THRESHOLD 5
#define PID_KP 2.0       //Proportional gain
#define PID_KI 0.1       //Integral gain
#define PID_KD 0.5       //Derivative gain
volatile int pid_error = 0;         //Current error
volatile int pid_last_error = 0;    //Previous error
volatile int pid_integral = 0;      //Integral term
int pid_output = 0;                 //PID output

volatile unsigned char heating_cnt = 0;            //Duty cycle count (0-10)
volatile unsigned char pump_cnt = 0;               //Pump duty cycle count (0-5)

uint8_t AlarmTimeBase_1s = 0;       //1 second alarm time base flag
uint8_t KeyTimeBase_20ms = 0;       //20 millisecond key time base flag
uint8_t LedTimeBase_500ms = 0;      //500 millisecond LED time base flag
uint16_t time_cnt_1s = 0;           //1 second counter
uint16_t time_cnt_500ms = 0;        //500 millisecond counter
uint16_t time_cnt_10ms = 0;         //10 millisecond counter

alarm_xx IntZero_timer_ms = {TIMERSTOP,FALSE,0,13},DryBurn_Timer_s = {TIMERSTOP,FALSE,0,2};

//Target temperature table
uint8_t target_temper_tbl[] = {25,45,55,85,95};

/**
 * @brief System initialization function
 * @note Initializes water dispenser state, PTC, heating and pump parameters, LED states, etc.
 */
void System_Init(void)
{
	key_child_lock.status = get_key_io_level(key_child_lock.id);
	key_temper_chg.status = get_key_io_level(key_temper_chg.id);
	key_water_out.status = get_key_io_level(key_water_out.id);
	key_pre_heat.status = get_key_io_level(key_pre_heat.id);
	key_disinfect.status = get_key_io_level(key_disinfect.id);
	
	mDispenser.temper_index =0;                          //Initialize temperature index to 0
	mDispenser.CurrentState = STATE_CHILD_LOCK;          //Initial state is child lock state
	mDispenser.temp_setting = target_temper_tbl[mDispenser.temper_index];  //Set initial target temperature
	mDispenser.fault_code = NO_FAULT;
	enter_lock(&mDispenser);

}

/**
 * @brief Water outlet processing function
 * @note Enables heating, turns off heater, sets pump speed to 100%, calculates PID if debug is enabled
 */
void waterout_process(void)
{
	
#ifdef ENABLE_DEBUG_PID	
	if(mDispenser.heating_enabled == 1)
	{
		calculate_pid();
		if((ptc_out.temper <= mDispenser.temp_setting+3)\
			&&(ptc_out.temper >= mDispenser.temp_setting-3))//Zero cold water
		{
			TW_Valve_OUT;	// water out
		}
	}
#endif
}

/**
 * @brief Preheat processing function
 * @note Enables heating, sets pump speed to 100%, calculates PID if debug is enabled
 */
void preheat_process(void)
{	
	if(ptc_in.temper >= 45 && mDispenser.heating_enabled == 1){		//preheat finished
		mDispenser.heating_enabled = 0; //stop heating
		mDispenser.heating_pwr = 0;
		mDispenser.pump_speed = 0;
		Alarm_Start(&mAlarm,0,3,0,PREHEAT_ALARM);
		set_led_status(LED_ID_PRE_HEAT,LED_ON);
	}else{
		#ifdef ENABLE_DEBUG_PID
		if(mDispenser.heating_enabled == 1) calculate_pid();
		#endif
	}
}

/**
 * @brief Disinfection processing function
 * @note Controls heating and pump status based on disinfection completion flag and clear water flag,
 *       calculates PID if debug is enabled
 */
void disinfect_process(void)
{
	if(mDispenser.disinfect_finish_flag == 0)	{
		#ifdef ENABLE_DEBUG_PID
    calculate_pid();
		if(mAlarm.mm < 27) mDispenser.need_clear_container =1;
		#endif
	}else{
		mDispenser.heating_enabled = 0;	//disinfect finish,stop pump,Three way valve dir :waterout
		mDispenser.pump_speed = 0;
		TW_Valve_OUT;
		mDispenser.heating_pwr = 0;
  }

//	if(mDispenser.disinfect_clr_water_flag == 1){  //Clear water after disinfection  impliment in clear_water()
//		if(mDispenser.fault_code == ERR_WATER_OUTLET_FOLD) return;
//		mDispenser.pump_speed = 100;
//    }else  mDispenser.pump_speed = 0;						
}

/**
 * @brief Main loop processing function
 * @note Executes corresponding processing logic according to the current state of the water dispenser
 */
void loop_fun(void)
 {
	switch(mDispenser.CurrentState)
    {
		case STATE_IDLE:                  //Idle state

				break;

		case STATE_CHILD_LOCK:            //Child lock state

				break;
		
		case STATE_WATER_OUT:             //Water outlet state
				 waterout_process();            //Call water outlet processing function
				break;
		
		case STATE_PRE_HEAT:              //Preheat state
				 preheat_process();             //Call preheat processing function
				break;
		
		case STATE_DISINFECT:             //Disinfection state
				 disinfect_process();           //Call disinfection processing function
				break;
		
    default :
        	break;
    }
 }

void lock_alarm_timeout(void)
{
	//goto sleep
}

void idle_alarm_timeout(void)
{
	WaterDispenser_Eventhandler(&mDispenser,CHILD_LOCK_PRESS_EVT);//triger virtual lock key press,gong to lock state
}

void preheat_alarm_timeout(void)
{
	mDispenser.heating_enabled = FALSE; //stop heating
	set_led_status(LED_ID_PRE_HEAT,LED_BLINK);
}
/**
 * @brief Disinfection alarm timeout processing function
 * @note Sets disinfection completion flag, sets water outlet LED to blink, sets disinfection LED to on
 */
void disinfect_alarm_timeout(void)
{
	mDispenser.heating_pwr = 0;
	mDispenser.pump_speed = 0;
	mDispenser.disinfect_finish_flag = 1;
	set_led_status(LED_ID_WATER_OUT,LED_BLINK);
	set_led_status(LED_ID_DISINFECT,LED_ON);
}

//Alarm callback function table
const alarm_cb alarm_cb_tbl[]=
{
	lock_alarm_timeout,
	idle_alarm_timeout,
	preheat_alarm_timeout,
	disinfect_alarm_timeout
};

/**
 * @brief Start alarm function
 * @param alarm Pointer to alarm structure
 * @param hh Hours
 * @param mm Minutes
 * @param ss Seconds
 * @param type_flag Alarm type flag
 * @note Sets alarm parameters and starts alarm if time is not zero
 */
void Alarm_Start(alarm *alarm,uint8_t hh,uint8_t mm,uint8_t ss,uint8_t type_flag)
 {
	if(!((hh ==0) && ( mm ==0) && (ss == 0)))
    {
		alarm->hh = hh;
		alarm->mm = mm;
		alarm->ss = ss;
		alarm->type_flag = type_flag;
		alarm->state = ON;
		alarm->alarm_timeout_cb = alarm_cb_tbl[type_flag];
	}
 }
 
/**
 * @brief Cancel alarm function
 * @param alarm Pointer to alarm structure
 * @note Resets alarm parameters and turns off alarm if alarm is active
 */
void Alarm_Cancel(alarm *alarm)
{
	if(alarm->state != OFF){
		alarm->hh = 0;
		alarm->mm = 0;
		alarm->ss = 0;
		alarm->type_flag = 0;
		alarm->state = OFF;
//		printf("cancel timer \n");
	}
//	else printf("Alarm_Cancel:alarm don't exist \n");
}
 
/**
 * @brief Alarm processing function
 * @note Updates alarm time every second, triggers callback function and turns off alarm when time reaches zero
 */
void Alarm_Process(void)
 {

	if((AlarmTimeBase_1s==1)&&(mAlarm.state == ON)&&(mAlarm.hh > 0 || mAlarm.mm > 0 || mAlarm.ss > 0))
    {
		if(mAlarm.ss > 0) {
			mAlarm.ss--;
		} else {
			if(mAlarm.mm > 0) {
				mAlarm.mm--;
				mAlarm.ss = 59;
			} else {
				if(mAlarm.hh > 0) {
					mAlarm.hh--;
					mAlarm.mm = 59;
					mAlarm.ss = 59;
				}                     	 
			}
		}
		
		if((mAlarm.hh == 0 && mAlarm.mm == 0 && mAlarm.ss == 0))
		{
			if(mAlarm.alarm_timeout_cb) mAlarm.alarm_timeout_cb();
			 mAlarm.state = OFF;
			 mAlarm.alarm_timeout_cb = NULL;
		}

		if(mAlarm.state == ON)
		{
//			printf("alarm time remain hh:mm:ss  %d:%d:%d\n",mAlarm.hh,mAlarm.mm,mAlarm.ss);
		}

//		printf("alarm state : %d \n",mAlarm.state);
        AlarmTimeBase_1s = 0;    
    }
 }

/**
 * @brief Alarm callback example function
 * @note Prints alarm type flag
 */
void alarm_cb_demo(void)
{
//	printf("alarm timeout :mAlarm.type_flag: %d \n",mAlarm.type_flag);
}

/**
 * @brief Get key IO level
 * @param key_index Key index
 * @return Key level state (1 for high level, 0 for low level)
 * @note Reads the corresponding GPIO level according to the key index
 */
GPIO_PinState get_key_io_level(uint8_t key_index)
{
	GPIO_PinState ret = 1;
	switch (key_index)
	{
		case KEY_CHILD_LOCK:      
				 ret = HAL_GPIO_ReadPin(LOCK_KEY);
		break;

    case KEY_TEMPER_CHG:        
				 ret = HAL_GPIO_ReadPin(TEMP_CHG_KEY);
		break;

    case KEY_WATER_OUT:        
				 ret = HAL_GPIO_ReadPin(WATEROUT_KEY);
		break;

    case KEY_PRE_HEAT:         
				 ret = HAL_GPIO_ReadPin(PREHEAT_KEY);
		break;

	case KEY_DISINFECT:    		
		ret = HAL_GPIO_ReadPin(DISINFECT_KEY);
		break;
	
	default:
		break;
	}

	return ret;
}

/**
 * @brief Single key scan function
 * @param _key Pointer to key structure
 * @note Detects key state (pressed, short press, long press) and updates key information
 */
uint8_t test_id;
void single_key_Scan(key* _key)  
 {
	 GPIO_PinState PinState;
	 test_id = _key->id;
#if TTP224C_OUTPUT_MODE == TTP224C_DIRECT_MODE
	if(get_key_io_level(_key->id) == 0 ) //Detects key press
	{
		_key->status = KEY_DOWN;
    _key->press_time++;         //Increase press time count
  }
  else
  {
		if(_key->press_time >= LONG_PRESS_TIME/10)  //Long press judgment
    {
			_key->status = KEY_LONG_PRESS;
    }else if(_key->press_time >= SHORT_PRESS_TIME/10)  //Short press judgment
    {
			_key->status = KEY_SHORT_PRESS;
    }
    _key->press_time=0;  //Reset press time count
  }  
#elif	TTP224C_OUTPUT_MODE == TTP224C_TOGGLE_MODE
	if((PinState=get_key_io_level(_key->id)) != _key->status ) //Detects key press
	{
		_key->status = PinState;
		_key->pressed_flag =1;
  }
#endif
 }

/**
 * @brief Key scan function
 * @note Scans all key states
 */
void Keys_Scan(void)
{
	single_key_Scan(&key_child_lock);
	single_key_Scan(&key_temper_chg);
	single_key_Scan(&key_water_out);
	single_key_Scan(&key_pre_heat);
	single_key_Scan(&key_disinfect);
}

/**
 * @brief Key status processing function
 * @param _key Pointer to key structure
 * @note Triggers corresponding events based on key state (short press, long press)
 */
void Key_status_handler(key* _key)
{
	#if TTP224C_OUTPUT_MODE == TTP224C_DIRECT_MODE
	switch(_key->status){
    case KEY_SHORT_PRESS:
		if(_key->short_press_event != EVT_NUM)
		{
			WaterDispenser_Eventhandler(&mDispenser,_key->short_press_event);
		}
		break;
        
    case KEY_LONG_PRESS:
		if(_key->long_press_event != EVT_NUM)
		{
			WaterDispenser_Eventhandler(&mDispenser,_key->long_press_event);
		}
		break;
    default :break;
    }
    _key->status = KEY_UP;  //Reset key state to not pressed
	#elif	TTP224C_OUTPUT_MODE == TTP224C_TOGGLE_MODE
	if(_key->pressed_flag ==1)
	{
		_key->pressed_flag =0;
		WaterDispenser_Eventhandler(&mDispenser,_key->long_press_event);
	}
	#endif
}

/**
 * @brief Key processing function
 * @note Processes all key states
 */
void Keys_handler(void)
{
	Key_status_handler(&key_child_lock);
	Key_status_handler(&key_temper_chg);
	Key_status_handler(&key_water_out);
	Key_status_handler(&key_pre_heat);
	Key_status_handler(&key_disinfect);
}

/************    LED control related functions    ****************/
/**
 * @brief Set LED status
 * @param led_index LED index
 * @param status LED status (on, off, blink)
 * @note Sets the corresponding LED's status according to the LED index
 */
void set_led_status(uint8_t led_index,uint8_t status)
{
	led_tbl[led_index].status = status;
	switch (led_tbl[led_index].id)
	{
		case LED_ID_CHILD_LOCK:     // Child lock LED
				 HAL_GPIO_WritePin(LOCK_LED,status);
			break;
		
		case LED_ID_TEMPER_CHG:        // Temperature adjustment LED
				HAL_GPIO_WritePin(TEMP_CHG_LED,status);
			break;
		
		case LED_ID_WATER_OUT:         // Water outlet LED
				 HAL_GPIO_WritePin(WATEROUT_LED,status);
			break;
		
		case LED_ID_PRE_HEAT:          // Preheat LED
			   HAL_GPIO_WritePin(PREHEAT_LED,status);
			break;
		
		case LED_ID_DISINFECT:    		//Disinfection LED:
				HAL_GPIO_WritePin(DISINFECT_LED,status);
			break;
		
		default:
			break;
	}
}

/**
 * @brief Set all LEDs status
 * @param led1_sta Child lock LED status
 * @param led2_sta Temperature adjustment LED status
 * @param led3_sta Water outlet LED status
 * @param led4_sta Preheat LED status
 * @param led5_sta Disinfection LED status
 * @note Batch sets the status of all LEDs
 */
void set_all_leds_status(uint8_t led1_sta,uint8_t led2_sta,uint8_t led3_sta,uint8_t led4_sta,uint8_t led5_sta)
{
		set_led_status(LED_ID_CHILD_LOCK,led1_sta);
    set_led_status(LED_ID_TEMPER_CHG,led2_sta);
    set_led_status(LED_ID_WATER_OUT,led3_sta);
    set_led_status(LED_ID_PRE_HEAT,led4_sta);
    set_led_status(LED_ID_DISINFECT,led5_sta);
}

/**
 * @brief LED blinking processing function
 * @note Processes LED blinking status every 500ms, toggles the level of blinking LEDs
 */
void led_blink(void)
{
	uint8_t LED_index = 0;
	if(LedTimeBase_500ms == 1 )
	{	
		LedTimeBase_500ms = 0;
//		printf("LED_id %d  status %d\n",led_tbl[LED_CHILD_LOCK].id,led_tbl[LED_CHILD_LOCK].status);
		for(LED_index = 0 ; LED_index < LED_ID_NUM ; LED_index++)
		{
			if(led_tbl[LED_index].status == LED_BLINK)  //If LED status is blinking
			{
				switch (LED_index)
				{
				case LED_ID_CHILD_LOCK:     // Child lock LED blinking
						 HAL_GPIO_TogglePin(LOCK_LED);  //Toggle LED level
						 break;
				
    			case LED_ID_TEMPER_CHG:        // Temperature adjustment LED blinking
						 HAL_GPIO_TogglePin(TEMP_CHG_LED);
					break;
				
 				case LED_ID_WATER_OUT:         //Water outlet LED blinking
					   HAL_GPIO_TogglePin(WATEROUT_LED);
					break;
				
				  case LED_ID_PRE_HEAT:          // Preheat LED blinking
					   HAL_GPIO_TogglePin(PREHEAT_LED);
					break;
					
				case LED_ID_DISINFECT:    		//Disinfection LED blinking
						 HAL_GPIO_TogglePin(DISINFECT_LED);
					break;
				
				default:
					break;
				}
			}
		}
	}
}

/**
 * @brief Microsecond delay function
 * @param us Number of microseconds to delay
 * @note Implements specified microsecond-level delay
 */
void DelayUs(uint32_t us) 
{
	if (us >= 1000) {
		HAL_Delay(us / 1000);  //Millisecond level delay
		us %= 1000; 
	}
	for (uint32_t i = 0; i < us * 8; i++) {  //Microsecond level delay loop
		__NOP();  //No operation
	}
}

#ifdef ENABLE_DEBUG_DISPLAY
/**
 * @brief Display function
 * @note Controls 7-segment display to show temperature setting and temperature indication
 */
void display(void)
{
	uint8_t i,j,tens_num,units_num;
	uint32_t time=50;
	#ifdef SEGMENTCODE_TEST
		
	for(i=A1;i<=G2;i++)
	{
			GPIO_CONFIG(Dr1_group,Dr1,DotDecode[i][0],DotDecode[i][5]); 
			GPIO_CONFIG(Dr2_group,Dr2,DotDecode[i][1],DotDecode[i][6]);
			GPIO_CONFIG(Dr3_group,Dr3,DotDecode[i][2],DotDecode[i][7]);
			GPIO_CONFIG(Dr4_group,Dr4,DotDecode[i][3],DotDecode[i][8]);
			GPIO_CONFIG(Dr5_group,Dr5,DotDecode[i][4],DotDecode[i][9]);
			DelayUs(time);
			HAL_Delay(1000);
			GPIO_CONFIG(Dr1_group,Dr1,GPIO_MODE_INPUT,GPIO_PIN_RESET); //set all LEDs input to prevent accidental lighting
			GPIO_CONFIG(Dr2_group,Dr2,GPIO_MODE_INPUT,GPIO_PIN_RESET);
			GPIO_CONFIG(Dr3_group,Dr3,GPIO_MODE_INPUT,GPIO_PIN_RESET);
			GPIO_CONFIG(Dr4_group,Dr4,GPIO_MODE_INPUT,GPIO_PIN_RESET);
			GPIO_CONFIG(Dr5_group,Dr5,GPIO_MODE_INPUT,GPIO_PIN_RESET);   
//			HAL_Delay(1000);
	}	
	#else

	if(mDispenser.fault_code == NO_FAULT){
		tens_num = mDispenser.temp_setting/10;
		units_num =mDispenser.temp_setting%10; 
	}else
	{
		tens_num = 0x0a;
		units_num =mDispenser.fault_code; 
	}
	//Display tens digit
	for(i=A1;i<=H1;i++)
	{
		if(((Segment_Code[tens_num]>>i)&0x01) == 1)   //If corresponding segment code bit is 1, light up the segment
		{
			GPIO_CONFIG(Dr1_group,Dr1,DotDecode[i][0],DotDecode[i][5]); 
			GPIO_CONFIG(Dr2_group,Dr2,DotDecode[i][1],DotDecode[i][6]);
			GPIO_CONFIG(Dr3_group,Dr3,DotDecode[i][2],DotDecode[i][7]);
			GPIO_CONFIG(Dr4_group,Dr4,DotDecode[i][3],DotDecode[i][8]);
			GPIO_CONFIG(Dr5_group,Dr5,DotDecode[i][4],DotDecode[i][9]);
			DelayUs(time);
			GPIO_CONFIG(Dr1_group,Dr1,GPIO_MODE_INPUT,GPIO_PIN_RESET); //set all LEDs input to prevent accidental lighting
			GPIO_CONFIG(Dr2_group,Dr2,GPIO_MODE_INPUT,GPIO_PIN_RESET);
			GPIO_CONFIG(Dr3_group,Dr3,GPIO_MODE_INPUT,GPIO_PIN_RESET);
			GPIO_CONFIG(Dr4_group,Dr4,GPIO_MODE_INPUT,GPIO_PIN_RESET);
			GPIO_CONFIG(Dr5_group,Dr5,GPIO_MODE_INPUT,GPIO_PIN_RESET);        
		}      
	}
	//Display units digit
	for(i=A1;i<=G1;i++)
	{
		if(((Segment_Code[units_num]>>i)&0x01) == 1)
		{
			j=i+8;
			GPIO_CONFIG(Dr1_group,Dr1,DotDecode[j][0],DotDecode[j][5]); 
			GPIO_CONFIG(Dr2_group,Dr2,DotDecode[j][1],DotDecode[j][6]);
			GPIO_CONFIG(Dr3_group,Dr3,DotDecode[j][2],DotDecode[j][7]);
			GPIO_CONFIG(Dr4_group,Dr4,DotDecode[j][3],DotDecode[j][8]);
			GPIO_CONFIG(Dr5_group,Dr5,DotDecode[j][4],DotDecode[j][9]);
			DelayUs(time);
			GPIO_CONFIG(Dr1_group,Dr1,GPIO_MODE_INPUT,GPIO_PIN_RESET); //set all LEDs input to prevent accidental lighting
			GPIO_CONFIG(Dr2_group,Dr2,GPIO_MODE_INPUT,GPIO_PIN_RESET);
			GPIO_CONFIG(Dr3_group,Dr3,GPIO_MODE_INPUT,GPIO_PIN_RESET);
			GPIO_CONFIG(Dr4_group,Dr4,GPIO_MODE_INPUT,GPIO_PIN_RESET);
			GPIO_CONFIG(Dr5_group,Dr5,GPIO_MODE_INPUT,GPIO_PIN_RESET);   
			}
	}
	//Display temperature indication
	if(mDispenser.fault_code == NO_FAULT){
		GPIO_CONFIG(Dr1_group,Dr1,DotDecode[H1][0],DotDecode[H1][5]); 
		GPIO_CONFIG(Dr2_group,Dr2,DotDecode[H1][1],DotDecode[H1][6]);
		GPIO_CONFIG(Dr3_group,Dr3,DotDecode[H1][2],DotDecode[H1][7]);
		GPIO_CONFIG(Dr4_group,Dr4,DotDecode[H1][3],DotDecode[H1][8]);
		GPIO_CONFIG(Dr5_group,Dr5,DotDecode[H1][4],DotDecode[H1][9]);
		DelayUs(time);
		GPIO_CONFIG(Dr1_group,Dr1,GPIO_MODE_INPUT,GPIO_PIN_RESET); //set all LEDs input to prevent accidental lighting
		GPIO_CONFIG(Dr2_group,Dr2,GPIO_MODE_INPUT,GPIO_PIN_RESET);
		GPIO_CONFIG(Dr3_group,Dr3,GPIO_MODE_INPUT,GPIO_PIN_RESET);
		GPIO_CONFIG(Dr4_group,Dr4,GPIO_MODE_INPUT,GPIO_PIN_RESET);
		GPIO_CONFIG(Dr5_group,Dr5,GPIO_MODE_INPUT,GPIO_PIN_RESET); 
	}
	#endif
}
#endif

#ifdef ENABLE_DEBUG_PTC
/**
 * @brief Get PTC temperature
 * @param _ptc PTC structure
 * @note Converts ADC sampling to temperature value using sliding window filtering
 */

void get_ptc_temper(ptc* _ptc)
{ 
	float sum = 0;
	int i;
 uint16_t adc_value,temper;
	/*******Get ADC temperature value*******/   
	HAL_ADC_Start(&hadc);//Start ADC1
	HAL_ADC_PollForConversion(&hadc,10);//Wait for conversion completion
	adc_value = HAL_ADC_GetValue(&hadc);//Get ADC value
	for(temper=0;temper<100;temper++)
	{	
		if(temper>=99)temper = 99;			//Temperature upper limit 100
		if(adc_value >= temptab[temper]){	    //When ADC conversion value is greater than or equal to temperature table value, get current temperature and exit
			break;				//Get temperature value, exit loop
		}
	}     
	/*******Sliding window filtering to calculate average temperature*******/
	_ptc->buffer[_ptc->buffer_id]=temper;
	_ptc->buffer_id = (++_ptc->buffer_id) % WINDOW_SIZE;
	for(i=0;i<WINDOW_SIZE;i++) sum += _ptc->buffer[i];
	_ptc->temper = sum/WINDOW_SIZE;
//	_ptc->temper = temper;
}

/**
 * @brief ADC get value function
 * @note Gets temperature values of PTC input and output, stops ADC
 */
void ADC_Get_Value(void)
{
	get_ptc_temper(&ptc_in);
	get_ptc_temper(&ptc_out);
	HAL_ADC_Stop(&hadc);
}

#endif

#ifdef ENABLE_DEBUG_PID
/**
 * @brief PID control calculation function
 * @note Calculates PID output based on target temperature and actual temperature for controlling heating power
 */
void calculate_pid(void)
{
    // Calculate current error
    pid_error = mDispenser.temp_setting - ptc_out.temper;
    
    // Calculate integral term
//    pid_integral += pid_error;

	if(abs(pid_error) < INTEGRAL_ENABLE_THRESHOLD)
	{
		pid_integral += pid_error;  // 误差小时才累积积分
	}else{
		pid_integral = 0;  // 误差大时清零积分，避免饱和
	}
    
    // Integral limit to prevent integral saturation
    if (pid_integral > 100) pid_integral = 100;
    if (pid_integral < -100) pid_integral = -100;
    
    // Calculate derivative term
    int derivative = pid_error - pid_last_error;
    
    // Calculate PID output
    pid_output = (PID_KP * pid_error) + (PID_KI * pid_integral) + (PID_KD * derivative);
    
    // Output limit
    if (pid_output > 100) pid_output = 100;
    if (pid_output < 0) pid_output = 0;
    mDispenser.heating_pwr = pid_output;
    // Save current error as last error
    pid_last_error = pid_error;
}
#endif
/**
 * @brief Safety check function
 * @note Checks for excessive temperature and water shortage, triggers corresponding safety protection measures
 */
void safety_check(void)
{
	if(mDispenser.pump_speed > 0 && iFlow.HZ < FLOW_0_HZ){  //start DryBurn and WaterShortage detect 2s
		if(DryBurn_Timer_s.start_flag == TIMERSTOP){
			DryBurn_Timer_s.start_flag = TIMERSTART;
			DryBurn_Timer_s.cnt = 0;
		}	
	}else{
		DryBurn_Timer_s.start_flag = TIMERSTOP;
	}	
	
	if(DryBurn_Timer_s.timersup == TURE){			//handle DryBurn and WaterShortage
		DryBurn_Timer_s.timersup = FALSE;
		if(mDispenser.heating_pwr > 0){
			mDispenser.fault_code = ERR_DRY_BURNING;
		}else{
			mDispenser.fault_code = ERR_WATER_SHORTAGE;
		}
		WaterDispenser_Eventhandler(&mDispenser,DRY_BURNING_EVT);			
	}
	
//	if(mDispenser.pump_speed > 0 && HAL_GPIO_ReadPin(TW_Valve) == OUT && HAL_GPIO_ReadPin(Micro_SW)== OFF)	//check if Micro_SW off 
//	{
//		mDispenser.fault_code = ERR_WATER_OUTLET_FOLD;
//	}
//	
//	if(mDispenser.fault_code == ERR_WATER_OUTLET_FOLD \
//		&& (HAL_GPIO_ReadPin(TW_Valve) == OUT && HAL_GPIO_ReadPin(Micro_SW)== ON))
//	{
//		mDispenser.fault_code = NO_FAULT;
//	}
}

