/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OPEN_RED_LED								(HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1))
#define CLOSE_RED_LED								(HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0))
#define OPEN_YELLOW_LED								(HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1))
#define CLOSE_YELLOW_LED							(HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0))
#define OPEN_GREEN_LED								(HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1))
#define CLOSE_GREEN_LED								(HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0))
#define Button_pressed								(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0)
#define GREEN_LED_ON								(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == 1)
#define YELLOW_LED_ON								(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == 1)
#define GREEN_LED_OFF								(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == 0)
#define YELLOW_LED_OFF								(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == 0)
#define RED_LED_OFF									(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 0)
#define GREEN_LED_MAX_CLOSE_TIME					(Initial_value + RED_LED_ON_DURATION + YELLOW_LED_ON_DURATION + GREEN_LED_ON_DURATION - 1000)
#define Red_led										(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1))
#define	Yellow_Led									(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2))
#define	Green_Led									(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define Initial_value						10
#define RED_LED_ON_DURATION					4000
#define YELLOW_LED_ON_DURATION				2000
#define GREEN_LED_ON_DURATION				8000
#define TIME_FOR_FILTER						50
#define FORWARD_DIRECTION					1
#define	BACKWARD_DIRECTION					2

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t Timer_counter = 0;
uint32_t Pressing_time = 0;
uint32_t Pressing_time_old = 0;
uint32_t Pressing_time_old_old = 0;
uint8_t Led_Updated_Flag = 0;
uint8_t button_state = 0;
uint32_t Timer_counter_old = 0;
uint8_t LED_Reading_Flag = 0;
uint32_t timer_when_button_is_pressed = 0;
uint8_t condition1_button_pressed = 0;
uint8_t condition2_button_pressed = 0;
uint8_t Green_led_flag_for_condition2 = 0;
uint8_t Distinguish_Yellow_Led = 0;
uint8_t Red = 0;
uint8_t Yellow = 0;
uint8_t Green = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  if (button_state == 0)
  {
	  if (Button_pressed)
	   {
		  Pressing_time++;
		  Pressing_time_old = 0;
	   }
	   else
	   {
		  if (Pressing_time != 0)
		  {
			  Pressing_time_old = Pressing_time;
			  Pressing_time = 0;
		  }

	   }
	   if (Pressing_time_old > TIME_FOR_FILTER)
	   {
		  button_state = 1;

	   }
	   else
	   {
		   button_state = 0;
	   }
  }

	Red = Red_led;
	Yellow = Yellow_Led;
	Green = Green_Led;

  //Normal Cycle of Traffic Light//
  if (button_state == 0 || button_state == 2 )
  {
	  Timer_counter++;
	  if (Timer_counter == Initial_value)
	  {
		  OPEN_RED_LED;
	  }
	  if (Timer_counter == Initial_value + RED_LED_ON_DURATION)
	  {
		  CLOSE_RED_LED;
		  OPEN_YELLOW_LED;
		  Distinguish_Yellow_Led = FORWARD_DIRECTION;
	  }
	  if (Timer_counter == Initial_value + RED_LED_ON_DURATION + YELLOW_LED_ON_DURATION)
	  {
		  CLOSE_YELLOW_LED;
		  Distinguish_Yellow_Led = 0;
		  OPEN_GREEN_LED;
	  }
	  if (Timer_counter == Initial_value + RED_LED_ON_DURATION + YELLOW_LED_ON_DURATION + GREEN_LED_ON_DURATION)
	   {
		  CLOSE_GREEN_LED;
		  OPEN_YELLOW_LED;
		  Distinguish_Yellow_Led = BACKWARD_DIRECTION;
	   }
	  if (Timer_counter == Initial_value + RED_LED_ON_DURATION + YELLOW_LED_ON_DURATION + GREEN_LED_ON_DURATION + YELLOW_LED_ON_DURATION)
	   {
		  CLOSE_YELLOW_LED;
		  Distinguish_Yellow_Led = 0;
		  Timer_counter = 0;
	   }
  }


  //*****************************************//

  //Butona basıldıgında gerçekleşicek koşulu seçme



  if (button_state == 1)  // This expression does not read the led status when the button is pressed, it reads the led status when we take our hand off the button.
  {

	  timer_when_button_is_pressed ++;
	  if (Timer_counter_old != Timer_counter)
	    {
	  	  LED_Reading_Flag = 1;
	  	  Timer_counter_old = Timer_counter;

	    }
	  if (GREEN_LED_ON &&  (LED_Reading_Flag == 1))
	  {
		  if ((Timer_counter_old < GREEN_LED_MAX_CLOSE_TIME))
		  {
			  LED_Reading_Flag = 0;
			  condition1_button_pressed = 1;
		  }
		  else
		  {
			button_state = 0;
			Pressing_time_old =0;
			timer_when_button_is_pressed = 0;
			LED_Reading_Flag = 0;
		  }

	  }


	  else if (YELLOW_LED_ON && LED_Reading_Flag == 1 && Distinguish_Yellow_Led == FORWARD_DIRECTION)
	  {
		  LED_Reading_Flag = 0;
		  timer_when_button_is_pressed = 0;
		  button_state = 2;
		  condition2_button_pressed = 1;
		  Green_led_flag_for_condition2 = 1;
	  }
	  else if (LED_Reading_Flag == 1 && YELLOW_LED_OFF && GREEN_LED_OFF && RED_LED_OFF)
	  {
		  button_state = 0;
		  Pressing_time_old =0;
		  Timer_counter = 0;
		  timer_when_button_is_pressed = 0;
		  LED_Reading_Flag = 0;
	  }
	  else if (LED_Reading_Flag == 1)
	  {
		  button_state = 0;
		  Pressing_time_old =0;
		  timer_when_button_is_pressed = 0;
		  LED_Reading_Flag = 0;
	  }

  }

  //***************************************************************************************************************//
  //Butona basıldıgında oluşacak 2 condition

  if (condition1_button_pressed)
  {

	  if (timer_when_button_is_pressed == 1000)
	  {
		  CLOSE_GREEN_LED;
		  OPEN_YELLOW_LED;
	  }
	  if (timer_when_button_is_pressed == 3000)
	  {
		  CLOSE_YELLOW_LED;
		  condition1_button_pressed = 0;
		  timer_when_button_is_pressed = 0;
		  button_state = 0;
		  Pressing_time_old =0;
		  Timer_counter = 0;


	  }

  }
  else if (condition2_button_pressed)
  {
	  if (GREEN_LED_ON && Green_led_flag_for_condition2)
	  {
		  Distinguish_Yellow_Led = 0;
		  condition2_button_pressed = 0;
		  Green_led_flag_for_condition2 = 0;
		  condition1_button_pressed = 1;
		  button_state = 1;
		  timer_when_button_is_pressed = 0;
	  }
  }




  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
