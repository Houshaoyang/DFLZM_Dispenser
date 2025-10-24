/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */	

//
//                 STM32F030C6T6  SOP48(C)
//                 ----------------
//  TEMP_1_AD--------|(PA0)    (PB0)|-----------PASSZERO     
//  TEMP_2_AD--------|(PA1)    (PB1)|-----------PUMP
//  Micro_SW---------|(PA2)    (PB2)|-----------LED1
//  FLX_DETECT-------|(PA3)    (PB3)|-----------DISINFECT_KEY
//  HEATER-----------|(PA4)    (PB4)|-----------PREHEAT_KEY
//  TEMP_CHG_LED-----|(PA5)    (PB5)|-----------WATEROUT_KEY
//  Dr1--------------|(PA6)    (PB6)|-----------TEMP_CHG_KEY
//  Dr2--------------|(PA7)    (PB7)|-----------     
//  -----------------|(PA8)    (PB8)|-----------
//  -----------------|(PA9)    (PB9)|------------
// 	-----------------|(PA10)   (PB10)|------------LOCK_KEY
//  -----------------|(PA11)   (PB11)|------------TW_Valve
//  -----------------|(PA12)   (PB12)|------------Dr3
//  -----------------|(PA13)   (PB13)|------------Dr4
//  -----------------|(PA14)   (PB14)|------------Dr5
//  BUZZER-----------|(PA15)   (PB15)|------------LOCK_LED
//  DISINFECT_LED------|(PC13)   
//  PREHEAT_LED--------|(PC14)   
//  WATEROUT_LED-------|(PC15) 

///**************define output ports*********/
//#define LOCK_LED       GPIOB,GPIO_PIN_15
//#define TEMP_CHG_LED   GPIOA,GPIO_PIN_5
//#define WATEROUT_LED   GPIOC,GPIO_PIN_15
//#define PREHEAT_LED    GPIOC,GPIO_PIN_14
//#define DISINFECT_LED  GPIOC,GPIO_PIN_13

//#define LOCK_KEY       GPIOB,GPIO_PIN_10
//#define TEMP_CHG_KEY   GPIOB,GPIO_PIN_6
//#define WATEROUT_KEY   GPIOB,GPIO_PIN_5
//#define PREHEAT_KEY    GPIOB,GPIO_PIN_4
//#define DISINFECT_KEY  GPIOB,GPIO_PIN_3

//#define PUMP           GPIOB,GPIO_PIN_1
//#define BUZZER         GPIOA,GPIO_PIN_15
//#define HEATER         GPIOA,GPIO_PIN_4
//#define Micro_SW			 GPIOA,GPIO_PIN_2
//#define TW_Valve			 GPIOB,GPIO_PIN_11

//#define Dr1         	 GPIOA,GPIO_PIN_6
//#define Dr2         	 GPIOA,GPIO_PIN_7
//#define Dr3        		 GPIOB,GPIO_PIN_12
//#define Dr4       	   GPIOB,GPIO_PIN_13
//#define Dr5      		   GPIOB,GPIO_PIN_14
//#define LED1					 GPIOB,GPIO_PIN_2
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sm.h"
#include "app.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim16);
//			HAL_TIM_Base_Start_IT(&htim17);	
	HAL_ADCEx_Calibration_Start(&hadc);
	System_Init();

//	save_flash_data();
//	read_flash_data();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		display();
//		loop_fun();
		Keys_handler();
		led_blink();
		Alarm_Process();
		ADC_Get_Value();
//		safety_check();
//		HAL_GPIO_TogglePin(BUZZER);
//		DelayUs(20000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
	if(GPIO_Pin == GPIO_PIN_0)		//Zero_detect_pin
	{
		mDispenser.heating_enabled = FALSE;
		if(mDispenser.heating_enabled == TURE){
			++PassZero_Detect.pulse_cnt;
			heating_cnt = (heating_cnt + 1) % 10;		//pump power control pwm frequence 1000/5 HZ 
			if(heating_cnt < (mDispenser.heating_pwr/10)){
			HEATER_ON;
//			HAL_TIM_Base_Start_IT(&htim17);		
			IntZero_timer_ms.start_flag = TIMERSTART ;	//	start conduction angle timer 
		}	
		}

	}
	
	if(GPIO_Pin == GPIO_PIN_3)		//flow_detect_pin count pulse
	{
		++iFlow.pulse_cnt;
	}
  /* NOTE: This function should not be modified, when the callback is needed,
            the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */ 
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	  if (htim->Instance == TIM16) {
				if(time_cnt_1s < 1000)  //1s timer for alarm
        {
					time_cnt_1s++;
				}else{
					AlarmTimeBase_1s = 1;
					time_cnt_1s = 0;
					if(DryBurn_Timer_s.start_flag == TIMERSTART){
						if(DryBurn_Timer_s.cnt < DryBurn_Timer_s.time_setting){
							DryBurn_Timer_s.cnt++;
						}else{
								DryBurn_Timer_s.timersup = TURE;
								DryBurn_Timer_s.cnt =0;
								DryBurn_Timer_s.start_flag = TIMERSTOP;
						}
					}
				}
				
        if(time_cnt_10ms < 10) //scan key state in every 10ms
        {
          time_cnt_10ms++;
        }else 
				{
					time_cnt_10ms = 0;
					Keys_Scan();
//					HAL_GPIO_TogglePin(TW_Valve);
        }
 
        if(time_cnt_500ms < LED_BLINK_FREQ_MS){  //500ms for led blink
            time_cnt_500ms++;
        }else{
            LedTimeBase_500ms = 1;
            time_cnt_500ms = 0;
						iFlow.HZ = 2*iFlow.pulse_cnt;//???????
            iFlow.pulse_cnt =0;  //????????????		
						PassZero_Detect.HZ = PassZero_Detect.pulse_cnt;
						PassZero_Detect.pulse_cnt = 0;
        }

        pump_cnt = (pump_cnt + 1) % 10;		//pump power control pwm frequence 1000/5 HZ 
        if(pump_cnt < (mDispenser.pump_speed/10))
				{
					PUMP_ON; //GPIO_PIN_RESET: pump ON  GPIO_PIN_SET:pump OFF
				}
				else PUMP_OFF;  // GPIO_PIN_RESET: pump ON  GPIO_PIN_SET:pump OFF
				
				if(IntZero_timer_ms.start_flag == TIMERSTART){	//	conduction angle count
					if(IntZero_timer_ms.cnt < IntZero_timer_ms.time_setting){
						IntZero_timer_ms.cnt++;
					}else{
						HEATER_OFF;
						IntZero_timer_ms.cnt =0;
						IntZero_timer_ms.start_flag = TIMERSTOP;
					}
				}
		}
		
		if (htim->Instance == TIM17) {
			HAL_TIM_Base_Stop_IT(&htim17);
		}
		
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
