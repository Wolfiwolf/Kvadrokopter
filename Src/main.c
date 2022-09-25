/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "usbd_cdc_if.h"
// #include "device_drivers/lsm303agr.h"
// #include "device_drivers/l3g4200d.h"
// #include "device_drivers/bno055.h"
#include "device_drivers/mpu_9250.h"
// #include "device_drivers/imu.h"
// #include "device_drivers/mpu_6050.h"
#include "communications/controller_listener.h"
#include "vehicle_operations/vehicle_controller.h"
#include "motors_controls/motors_controls.h"

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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;

/* Definitions for motorControl */
osThreadId_t motorControlHandle;
const osThreadAttr_t motorControl_attributes = {
  .name = "motorControl",
  .stack_size = 230 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tilt_control */
osThreadId_t tilt_controlHandle;
const osThreadAttr_t tilt_control_attributes = {
  .name = "tilt_control",
  .stack_size = 400 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for angle_mutex */
osMutexId_t angle_mutexHandle;
const osMutexAttr_t angle_mutex_attributes = {
  .name = "angle_mutex"
};
/* Definitions for motors_power_mutex */
osMutexId_t motors_power_mutexHandle;
const osMutexAttr_t motors_power_mutex_attributes = {
  .name = "motors_power_mutex"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C2_Init(void);
void MotorControl(void *argument);
void TiltControl(void *argument);

/* USER CODE BEGIN PFP */

float angle_x, angle_y, angle_z;
int motors_power;
struct MotorSpeeds prev_speeds;
uint8_t kill;

void micro_delay(uint16_t duration) {
	/*
	 __HAL_TIM_SET_COUNTER(&htim3, 0);
	 while (__HAL_TIM_GET_COUNTER(&htim3) < duration)
	 ;
	 */
}

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance = 0;

#define TRIG_PIN GPIO_PIN_3
#define TRIG_PORT GPIOE

// Let's write the callback function

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	/*
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // if the interrupt source is channel1
			{
		if (Is_First_Captured == 0) // if the first value is not captured
				{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured == 1)   // if the first is already captured
				{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1) {
				Difference = IC_Val2 - IC_Val1;
			}

			else if (IC_Val1 > IC_Val2) {
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034 / 2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
		}
	}
	 */
}

void HCSR04_Read(void) {
	/*
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
	micro_delay(10);
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
	 */
}

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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

	kill = 0;

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of angle_mutex */
  angle_mutexHandle = osMutexNew(&angle_mutex_attributes);

  /* creation of motors_power_mutex */
  motors_power_mutexHandle = osMutexNew(&motors_power_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of motorControl */
  motorControlHandle = osThreadNew(MotorControl, NULL, &motorControl_attributes);

  /* creation of tilt_control */
  tilt_controlHandle = osThreadNew(TiltControl, NULL, &tilt_control_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

// DISTANCE CODE
	/*
	 HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	 */

// TIME CODE
	while (1) {
		// DISTANCE SENSOR CODE
		/*
		 HCSR04_Read();
		 HAL_Delay(10);
		 if (Distance < 50) {
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 1);
		 } else {
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 0);
		 }
		 */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 144;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L_MOTOR_CW_GPIO_Port, L_MOTOR_CW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE8 PE9 PE10 PE11
                           PE12 PE13 PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : L_MOTOR_CW_Pin */
  GPIO_InitStruct.Pin = L_MOTOR_CW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L_MOTOR_CW_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_MotorControl */
int power_offset = 0;
float x_tilt_set_point = 0.0f;
float x_tilt_set_point_target = 0.0f;
float y_tilt_set_point = 0.0f;
float y_tilt_set_point_target = 0.0f;

uint16_t starting_motor_power = 210;

uint8_t start_flight = 0;
uint8_t testing_motors = 0;
uint8_t uart_data[7];

void motors_test() {
	set_FL_motor_speed(&htim2, 10);
	set_FR_motor_speed(&htim2, 10);
	set_BL_motor_speed(&htim2, 10);
	set_BR_motor_speed(&htim2, 10);
	HAL_Delay(200);
	set_FL_motor_speed(&htim2, 20);
	set_FR_motor_speed(&htim2, 20);
	set_BL_motor_speed(&htim2, 20);
	set_BR_motor_speed(&htim2, 20);
	HAL_Delay(200);
	set_FL_motor_speed(&htim2, 0);
	set_FR_motor_speed(&htim2, 0);
	set_BL_motor_speed(&htim2, 0);
	set_BR_motor_speed(&htim2, 0);
	HAL_Delay(200);
	set_FL_motor_speed(&htim2, 20);
	set_FR_motor_speed(&htim2, 20);
	set_BL_motor_speed(&htim2, 20);
	set_BR_motor_speed(&htim2, 20);
	HAL_Delay(200);
	set_FL_motor_speed(&htim2, 0);
	set_FR_motor_speed(&htim2, 0);
	set_BL_motor_speed(&htim2, 0);
	set_BR_motor_speed(&htim2, 0);
	testing_motors = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
	if (start_flight != 0 || testing_motors != 0) {

		if (uart_data[0] == '0') {
			if (uart_data[2] != '1') {
				motors_power = 0;
				start_flight = 0;
			}
		}

		HAL_UART_Receive_IT(&huart4, uart_data, 7);
		return;
	}

	// 0-0-000
	if (uart_data[0] == '0') {
		if (uart_data[2] == '1') {
			if (motors_power == 0) {
				starting_motor_power = (uart_data[4] - 48) * 100
						+ (uart_data[5] - 48) * 10 + (uart_data[6] - 48);
				start_flight = 1;
			}
		} else {
			motors_power = 0;
		}
	} else if (uart_data[0] == '1') {
		if (uart_data[2] == '0') {
			x_tilt_set_point_target = 10.0f;
		} else if (uart_data[2] == '1') {
			x_tilt_set_point_target = 0.0f;
		} else {
			x_tilt_set_point_target = -10.0f;
		}
	} else if (uart_data[0] == '2') {
		if (uart_data[2] == '0') {
			y_tilt_set_point_target = -10.0f;
		} else if (uart_data[2] == '1') {
			y_tilt_set_point_target = 0.0f;
		} else {
			y_tilt_set_point_target = 10.0f;
		}
	} else if (uart_data[0] == '3') {
		if (uart_data[2] == '0') {
			power_offset = -30;
		} else if (uart_data[2] == '1') {
			power_offset = 0;
		} else {
			power_offset = 30;
		}
	} else if (uart_data[0] == '4') {
		if (motors_power != 0) {
			motors_power = (uart_data[4] - 48) * 100 + (uart_data[5] - 48) * 10
					+ (uart_data[6] - 48);
		}
	} else if (uart_data[0] == '5') {
		if (motors_power == 0) {
			testing_motors = 1;
		}
	}

	HAL_UART_Receive_IT(&huart4, uart_data, 7);
}

void calibrate_motors() {
	set_FL_motor_speed(&htim2, 500);
	set_FR_motor_speed(&htim2, 500);
	set_BL_motor_speed(&htim2, 500);
	set_BR_motor_speed(&htim2, 500);
	HAL_Delay(3000);
	set_FL_motor_speed(&htim2, 0);
	set_FR_motor_speed(&htim2, 0);
	set_BL_motor_speed(&htim2, 0);
	set_BR_motor_speed(&htim2, 0);
	HAL_Delay(8000);
	motors_test();

}

void update_motor_speeds(float ang_x, float ang_y, float ang_z,
		struct MotorSpeeds *prev_speeds) {
	if (motors_power > 50)
		*prev_speeds = correct_motors_for_tilt(&htim2,
				motors_power + power_offset, ang_x, ang_y, ang_z, prev_speeds);
	else
		*prev_speeds = correct_motors_for_tilt(&htim2,
				motors_power + power_offset, 0.0f, 0.0f, 0.0f, prev_speeds);
}

/**
 * @brief  Function implementing the motorControl thread.
 * @param  argument: Not used
 * @retval None
 */

#define LOG_USB
// #define LOG_USB_TIME 25
#define LOG_USB_TIME 5

/* USER CODE END Header_MotorControl */
void MotorControl(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */


	// HEIGHT SENSOR


	// MOTORS CONTROL CODE
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	// osMutexAcquire(motors_power_mutexHandle, osWaitForever);
	motors_power = 0;
	// osMutexRelease(motors_power_mutexHandle);
	calibrate_motors();

	uint32_t currentMillies, last_transmit;
	currentMillies = HAL_GetTick();
	last_transmit = currentMillies;

	prev_speeds.m1 = 0.0f;
	prev_speeds.m2 = 0.0f;
	prev_speeds.m3 = 0.0f;
	prev_speeds.m4 = 0.0f;
	/*
	 */

	motor_controls_init();
	HAL_UART_Receive_IT(&huart4, uart_data, 7);

	/* Infinite loop */
	for (;;) {

		currentMillies = HAL_GetTick();

		// MOTORS CONTROL CODE

		osMutexAcquire(angle_mutexHandle, osWaitForever);
		float ang_x = angle_x;
		float ang_y = angle_y;
		float ang_z = angle_z;
		osMutexRelease(angle_mutexHandle);

		update_motor_speeds(ang_x, ang_y, ang_z, &prev_speeds);

		if (fabsf(x_tilt_set_point - x_tilt_set_point_target) > 0.0005f)
			x_tilt_set_point +=
					0.001f
							* ((x_tilt_set_point_target - x_tilt_set_point)
									/ fabsf(
											x_tilt_set_point_target
													- x_tilt_set_point));
		if (fabsf(y_tilt_set_point - y_tilt_set_point_target) > 0.0005f)
			y_tilt_set_point +=
					0.001f
							* ((y_tilt_set_point_target - y_tilt_set_point)
									/ fabsf(
											y_tilt_set_point_target
													- y_tilt_set_point));

		// MOTORS TEST
		if (testing_motors == 1) {
			motors_test();
		}

		// POWER UP CODE (NO CONTROL)
		if (start_flight == 1) {
			uint32_t current = HAL_GetTick();
			uint32_t prev = current;
			while (start_flight == 1) {
				osMutexAcquire(angle_mutexHandle, osWaitForever);
				ang_x = angle_x;
				ang_y = angle_y;
				ang_z = angle_z;
				osMutexRelease(angle_mutexHandle);
				current = HAL_GetTick();
				update_motor_speeds(ang_x, ang_y, ang_z, &prev_speeds);

				if (current - prev > 150) {
					prev = current;
					motors_power += 5;
					if (motors_power >= starting_motor_power) {
						start_flight = 0;
					}
				}

#ifdef LOG_USB
				if (current - last_transmit > LOG_USB_TIME) {
					last_transmit = current;
					uint8_t buffer[6 * 7];
					for (int i = 0; i < 6 * 7; ++i)
						buffer[i] = ' ';

					// Start comment
					float tempx = angle_x * 100.0f;
					float tempy = angle_y * 100.0f;
					int angX = roundf(tempx);
					int angY = roundf(tempy);
					itoa(angX, (char*) buffer + 6 * 1, 10);
					itoa(angY, (char*) buffer + 6 * 2, 10);
					int m1 = prev_speeds.m1;
					itoa(m1, (char*) buffer + 6 * 3, 10);
					int m2 = prev_speeds.m2;
					itoa(m2, (char*) buffer + 6 * 4, 10);
					int m3 = prev_speeds.m3;
					itoa(m3, (char*) buffer + 6 * 5, 10);
					int m4 = prev_speeds.m4;
					itoa(m4, (char*) buffer + 6 * 6, 10);
					// End comment
					buffer[0] = '#';
					buffer[1] = '#';
					buffer[2] = '#';
					buffer[3] = '#';
					buffer[4] = '#';
					buffer[6 * 1 - 1] = '\n';
					buffer[6 * 2 - 1] = '\n';
					buffer[6 * 3 - 1] = '\n';
					buffer[6 * 4 - 1] = '\n';
					buffer[6 * 5 - 1] = '\n';
					buffer[6 * 6 - 1] = '\n';
					buffer[6 * 7 - 1] = '\n';

					CDC_Transmit_FS(buffer, 6 * 7);
				}
#endif
			}
		}

#ifdef LOG_USB
		// DEBUG USB TRANSMIT
		if (currentMillies - last_transmit > LOG_USB_TIME) {
			last_transmit = currentMillies;
			uint8_t buffer[6 * 7];
			for (int i = 0; i < 6 * 7; ++i)
				buffer[i] = ' ';

			// Start comment
			float tempx = angle_x * 100.0f;
			float tempy = angle_y * 100.0f;
			int angX = roundf(tempx);
			int angY = roundf(tempy);
			itoa(angX, (char*) buffer + 6 * 1, 10);
			itoa(angY, (char*) buffer + 6 * 2, 10);
			int m1 = prev_speeds.m1;
			itoa(m1, (char*) buffer + 6 * 3, 10);
			int m2 = prev_speeds.m2;
			itoa(m2, (char*) buffer + 6 * 4, 10);
			int m3 = prev_speeds.m3;
			itoa(m3, (char*) buffer + 6 * 5, 10);
			int m4 = prev_speeds.m4;
			itoa(m4, (char*) buffer + 6 * 6, 10);
			// End comment
			buffer[0] = '#';
			buffer[1] = '#';
			buffer[2] = '#';
			buffer[3] = '#';
			buffer[4] = '#';
			buffer[6 * 1 - 1] = '\n';
			buffer[6 * 2 - 1] = '\n';
			buffer[6 * 3 - 1] = '\n';
			buffer[6 * 4 - 1] = '\n';
			buffer[6 * 5 - 1] = '\n';
			buffer[6 * 6 - 1] = '\n';
			buffer[6 * 7 - 1] = '\n';

			CDC_Transmit_FS(buffer, 6 * 7);
		}
#endif

	}
	/*
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_TiltControl */

void get_tilt(struct Angles *output) {
	// return IMU_get_axis_data(output);
	mpu9250_get_axis_data_quaternion(output);
	// return mpu6050_get_axis_data_quaternion();
}

void calibrate_tilt() {
	HAL_Delay(3000);
	// IMU_calibrate();
	mpu9250_calibrate();
	// mpu6050_calibrate();
}

/**
 * @brief Function implementing the tilt_control thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TiltControl */
void TiltControl(void *argument)
{
  /* USER CODE BEGIN TiltControl */
	/* Infinite loop */

	const float radian_to_degrees = 57.295779513;

	uint32_t currentMillies, prevMillies;
	currentMillies = HAL_GetTick();
	prevMillies = currentMillies;

	// mpu6050_init();
	mpu9250_init();
	// BNO055_Init();
	// IMU_init();

	osMutexAcquire(angle_mutexHandle, osWaitForever);
	angle_x = 0.0f;
	angle_y = 0.0f;
	angle_z = 0.0f;
	osMutexRelease(angle_mutexHandle);

	// LOGGING CODE
	/*
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, 1);
	 HAL_Delay(1000);
	 Mount_SD("/");
	 Create_File("drone_log.txt");
	 Create_Dir("drone_logs");
	 Unmount_SD("/");
	 */

	struct Angles angles;
	calibrate_tilt();
	for (;;) {
		get_tilt(&angles);

		osMutexAcquire(angle_mutexHandle, osWaitForever);
		angle_x = angles.x_angle;
		angle_y = angles.y_angle;
		angle_z = angles.z_angle;
		osMutexRelease(angle_mutexHandle);

	}
  /* USER CODE END TiltControl */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
	while (1) {
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
