#ifndef _APP_H_
#define _APP_H_

#include <stdint.h>
#include "sm.h"
#include "gpio.h"
#include "GpioCtrlMacros.h"
#include "flash.h"

/**************define struct*********/
typedef struct{
uint8_t hh;
uint8_t mm;
uint8_t ss;
uint8_t type_flag;
uint8_t timeout_flag;
uint8_t state;
void (*alarm_timeout_cb)(void);
}alarm ;

typedef void (*alarm_cb)(void);

typedef struct {
    uint8_t HZ;
    uint16_t pulse_cnt;
}pulse_counter;

typedef enum {
    KEY_UP = 0,      // 按键释放
    KEY_DOWN,        // 按键按下
    KEY_SHORT_PRESS, // 短按
    KEY_LONG_PRESS   // 长按
} key_status;

typedef enum {
    KEY_CHILD_LOCK = 0,    // 童锁按键
    KEY_TEMPER_CHG,        // 调温按键
    KEY_WATER_OUT,         // 出水按键
    KEY_PRE_HEAT,          // 预加热按键
	KEY_DISINFECT    	   //消毒按键
}key_id;

typedef struct{
	key_id id;
	key_status status;
	uint8_t short_press_event;
	uint8_t long_press_event;
    uint32_t press_time;
}key;

typedef enum {
    LED_ID_CHILD_LOCK = 0,      // 童锁
    LED_ID_TEMPER_CHG,        // 调温
    LED_ID_WATER_OUT,         // 出水
    LED_ID_PRE_HEAT,          // 预加热
	LED_ID_DISINFECT,    		//消毒
    LED_ID_NUM
}led_id;

typedef enum {
    LED_ON = 0,      // 亮
    LED_OFF,        // 灭
    LED_BLINK,      // 闪烁
}led_status;

typedef struct {
    uint8_t id;
    led_status status;
}led;


typedef enum {
    LOCK_ALARM = 0,      //10S no action in LOCK state,goto sleep
		IDLE_ALARM,					 //10S no action in IDLE state,goto LOCK state
		PREHEAT_ALARM,			 //wait 3min after preheat finish, loop preheat
		DISINFECT_ALARM,		 //20min disinfection
}alarm_type;

#define WINDOW_SIZE 10
typedef struct {
	unsigned char channel;
    unsigned char buffer_id;
    float buffer[10];
    float temper;
}ptc;

/**************define global variable*********/
extern alarm mAlarm;
extern WaterDispenser mDispenser;
extern pulse_counter iFlow;

#ifdef ENABLE_DEBUG_PTC
extern ptc ptc_in,ptc_out;
extern uint32_t adc_value[2];
#endif

extern volatile uint16_t Flow_Value;
extern int pid_output;
extern volatile unsigned char heating_cnt;               // ռ�ձȼ���ֵ��0-10��
extern volatile unsigned char pump_cnt;               // ռ�ձȼ���ֵ��0-5��

extern uint8_t AlarmTimeBase_1s; //1s alarm base time  flag
extern uint8_t KeyTimeBase_20ms;  //20ms key base time  flag
extern uint8_t LedTimeBase_500ms;  //20ms key base time  flag
extern uint16_t time_cnt_1s;
extern uint16_t time_cnt_500ms;
extern uint16_t time_cnt_10ms;

extern uint8_t target_temper_tbl[];


//volatile WaterDispenser mWaterDispenser = {STATE_CHILD_LOCK,50,CHILD_LOCK_DOWN_EVT};

void System_Init(void);
int get_key_io_level(uint8_t key_index);
void Board_Init(void);	    	//
void Adc_Init(void);		    //	
void Timer0_Init(void);	
void Timer2_Init (void);
void Int_Init(void);

void loop_fun(void);

void Alarm_Start(alarm *alarm,uint8_t hh,uint8_t mm,uint8_t ss,uint8_t type_flag); 
void Alarm_Cancel(alarm *alarm);
void Alarm_Process(void);

void Keys_Scan(void);
void Keys_handler(void);

void set_led_status(uint8_t led_index,uint8_t status);
void set_all_leds_status(uint8_t led1_sta,uint8_t led2_sta,uint8_t led3_sta,uint8_t led4_sta,uint8_t led5_sta);
void led_blink(void);

#ifdef ENABLE_DEBUG_DISPLAY
void display(void);
#endif

#ifdef ENABLE_DEBUG_PTC
void ADC_Get_Value(void);
void get_ptc_temper(ptc _ptc);
uint16_t Get_Adc_Ddata(unsigned char adcChannel);
#endif

#ifdef ENABLE_DEBUG_PID
void calculate_pid(void); 
void safety_check(void);
#endif
#endif
