#ifndef GPIO_MACROS_H
#define GPIO_MACROS_H

#include "stm32f0xx_hal.h"
/*****************debug ctrl define*********************/
#define ENABLE_DEBUG_PTC 1
#define ENABLE_DEBUG_DISPLAY 1
#define ENABLE_DEBUG_PID 1
//#define SEGMENTCODE_TEST 1

/**************define macro*********/
#define ON  1
#define OFF 0

/*****************timer para define*********************/
#define LONG_PRESS_TIME 1000 //ms
#define SHORT_PRESS_TIME  200   //ms  
#define LED_BLINK_FREQ_MS 500   //ms

/*****************flow HZ table*********************/
#define FLOW_0_HZ 12 //0 flow HZ

/*****************GPIO port define*********************/
#define LOCK_LED       GPIOB,GPIO_PIN_15
#define TEMP_CHG_LED   GPIOA,GPIO_PIN_5
#define WATEROUT_LED   GPIOC,GPIO_PIN_15
#define PREHEAT_LED    GPIOC,GPIO_PIN_14
#define DISINFECT_LED  GPIOC,GPIO_PIN_13

#define LOCK_KEY       GPIOB,GPIO_PIN_10
#define TEMP_CHG_KEY   GPIOB,GPIO_PIN_6
#define WATEROUT_KEY   GPIOB,GPIO_PIN_5
#define PREHEAT_KEY    GPIOB,GPIO_PIN_4
#define DISINFECT_KEY  GPIOB,GPIO_PIN_3

#define PUMP           GPIOB,GPIO_PIN_1
#define BUZZER         GPIOA,GPIO_PIN_15
#define HEATER         GPIOA,GPIO_PIN_4
#define Micro_SW			 GPIOA,GPIO_PIN_2
#define TW_Valve			 GPIOB,GPIO_PIN_11
#define TW_Valve_OUT	 GPIO_PIN_RESET
#define TW_Valve_IN		 GPIO_PIN_SET
#define LED1					 GPIOB,GPIO_PIN_2

#define Dr1         	 GPIO_PIN_6
#define Dr1_group    	 GPIOA
#define Dr2         	 GPIO_PIN_7
#define Dr2_group    	 GPIOA
#define Dr3        		 GPIO_PIN_12
#define Dr3_group    	 GPIOB
#define Dr4       	   GPIO_PIN_13
#define Dr4_group    	 GPIOB
#define Dr5      		   GPIO_PIN_14
#define Dr5_group    	 GPIOB

/*****************GPIO ctrl define*********************/
// Macro definition: Initialize GPIO as output mode (push-pull output)
#define GPIO_INIT_OUTPUT(group, pin) do { \
    GPIO_InitTypeDef GPIO_InitStruct = {0}; \
    __HAL_RCC_##group##_CLK_ENABLE(); \
    GPIO_InitStruct.Pin = pin; \
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; \
    GPIO_InitStruct.Pull = GPIO_NOPULL; \
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; \
    HAL_GPIO_Init(group, &GPIO_InitStruct); \
} while(0)

// Macro definition: Switch GPIO to high-impedance state (input mode)
#define GPIO_SET_HIGHZ(group, pin) do { \
    GPIO_InitTypeDef GPIO_InitStruct = {0}; \
    GPIO_InitStruct.Pin = pin; \
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; \
    GPIO_InitStruct.Pull = GPIO_NOPULL; \
    HAL_GPIO_Init(group, &GPIO_InitStruct); \
} while(0)

// Macro definition: Switch GPIO to output mode (based on initialized clock)
#define GPIO_SET_OUTPUT(group, pin) do { \
    GPIO_InitTypeDef GPIO_InitStruct = {0}; \
    GPIO_InitStruct.Pin = pin; \
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; \
    GPIO_InitStruct.Pull = GPIO_NOPULL; \
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; \
    HAL_GPIO_Init(group, &GPIO_InitStruct); \
} while(0)

// Macro definition: General GPIO configuration (supports input/output modes, sets initial level when in output mode)
#define GPIO_CONFIG(group, pin, mode, level) do { \
    GPIO_InitTypeDef GPIO_InitStruct = {0}; \
    GPIO_InitStruct.Pin = pin; \
    GPIO_InitStruct.Mode = mode; \
    GPIO_InitStruct.Pull = GPIO_NOPULL; \
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; \
    HAL_GPIO_Init(group, &GPIO_InitStruct); \
    /* Set initial level if in output mode */ \
    if((mode == GPIO_MODE_OUTPUT_PP) || (mode == GPIO_MODE_OUTPUT_OD)) { \
        HAL_GPIO_WritePin(group, pin, level); \
    } \
} while(0)


// Macro definition: Set GPIO output to high level
#define GPIO_SET_HIGH(group, pin) HAL_GPIO_WritePin(group, pin, GPIO_PIN_SET)

// Macro definition: Set GPIO output to low level
#define GPIO_SET_LOW(group, pin) HAL_GPIO_WritePin(group, pin, GPIO_PIN_RESET)

// Macro definition: Toggle GPIO output level
#define GPIO_TOGGLE(group, pin) HAL_GPIO_TogglePin(group, pin)

#endif /* GPIO_MACROS_H */
