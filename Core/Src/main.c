/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "rng.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "focus.h"
#include "ssd1306.h"

//#define USE_INTERNAL_TEMP_SENSOR

#ifdef USE_INTERNAL_TEMP_SENSOR
#include "internal_temp_sensor.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_MSG_REPEAT_PERIOD 100
#define CAN_MSG_TIME_INTERVAL 5
#define RANDOM_MSG_TIME_INTERVAL 2500

#define COOLANT_TEMPERATURE 90 // degC
#define FUEL_LEVEL 75 // %

#define DELAYED_START_INTERVAL 4000

#define TOGGLE_TIME_INTERVAL 1000

#define STOP_RANDOM_MSG
#define RANDOM_MSG_ID 0x100

//#define SEND_ALL_POSSIBLE_IDS
#define PATTERN_TO_BE_SEND  0xAA // 0b00011000 // 0xAA or 0x55

#define STARTING_FRAME 0x00

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData8[8];
uint8_t TxData8Random[8];
uint8_t Random8Bytes[8];
uint32_t TxMailbox;

uint16_t vehicle_speed = 0;

uint16_t engine_speed = 0;

uint32_t CanMsgSoftTimer;
uint32_t RandomMsgSoftTimer;
uint32_t DelayedStartSoftTimer;
uint32_t ToggleSoftTimer;
uint32_t OdometerSoftTimer;
uint32_t OledSoftTimer;

volatile uint8_t rng_number_flag = 0;
uint8_t rng_number_count = 0;
uint8_t rng_message_flag = 0;
uint32_t rng_number;

uint8_t toggle_variable = 0;
uint8_t flip_me_variable = 0xAA;  // odometer

volatile uint8_t green_button_flag = 0;

uint16_t msg_id = STARTING_FRAME;

char lcd_line[128];

#ifdef USE_INTERNAL_TEMP_SENSOR
typedef struct AdcValues
{

	uint16_t Raw[2]; /* Raw values from ADC */
	int32_t IntSensTmp; /* Temperature */

} adcval_t;

adcval_t Adc;

typedef struct Flags
{
	uint8_t ADCCMPLT;

} flag_t;

flag_t Flg =
{ 0, };
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MarkCanFrame(void);
void ComposeRandomMessage(void);
void DisplayUnits(void);
void EmulateIncreasingMileage(void);
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
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_FDCAN1_Init();
	MX_I2C3_Init();
	MX_RNG_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

#ifdef USE_INTERNAL_TEMP_SENSOR
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) Adc.Raw, 2);
	HAL_TIM_Base_Start(&htim3); /* This timer starts ADC conversion */
#endif

	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_SetCursor(20, 0);
	ssd1306_WriteString("ufnalski.edu.pl", Font_6x8, White);
	ssd1306_SetCursor(2, 11);
	ssd1306_WriteString("Ford Focus MK3 2013", Font_6x8, White);
	ssd1306_SetCursor(10, 22);
	ssd1306_WriteString("Instrument cluster", Font_6x8, White);
	ssd1306_SetCursor(2, 33);
	ssd1306_WriteString("CAN messages hacking", Font_6x8, White);
	ssd1306_UpdateScreen();

	HAL_RNG_GenerateRandomNumber_IT(&hrng);

	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
	FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			0) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}

	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	CanMsgSoftTimer = HAL_GetTick();
	RandomMsgSoftTimer = HAL_GetTick();
	DelayedStartSoftTimer = HAL_GetTick();
	ToggleSoftTimer = HAL_GetTick();
	OdometerSoftTimer = HAL_GetTick();
	OledSoftTimer = HAL_GetTick();
	memset(RxData, 0xFF, sizeof(RxData));

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		if (HAL_GetTick() - ToggleSoftTimer > TOGGLE_TIME_INTERVAL)
		{
			ToggleSoftTimer = HAL_GetTick();
			toggle_variable ^= 1;
		}

#ifdef SEND_ALL_POSSIBLE_IDS
		if ((HAL_GetTick() - CanMsgSoftTimer > CAN_MSG_REPEAT_PERIOD)
				&& (msg_id <= 0x7FF))
		{
			CanMsgSoftTimer = HAL_GetTick();
			ssd1306_SetCursor(2, 44);
						sprintf(lcd_line, "Msg ID: %d of %d", msg_id, 0x7FF);
						ssd1306_WriteString(lcd_line, Font_6x8, White);
						ssd1306_UpdateScreen();

						PrintCanFrame0xToUart(&TxHeader, TxData8);
						MarkCanFrame();

			send_ignition_on(CAN_MSG_TIME_INTERVAL);

			HAL_Delay(CAN_MSG_TIME_INTERVAL);
			TxHeader.DataLength = FDCAN_DLC_BYTES_8;
			TxHeader.Identifier = msg_id;
			memset(TxData8, PATTERN_TO_BE_SEND, sizeof(TxData8));

			if (msg_id != 0x80) // something in this frame switches off the cluster permanently
			{
				if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8)
						!= HAL_OK)
				{
					Error_Handler();
				}
			}
			msg_id++;

			HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port,
			BLUE_LED_Pin);
		}

#else

		ComposeRandomMessage();

		if (HAL_GetTick() - CanMsgSoftTimer > CAN_MSG_TIME_INTERVAL)
		{
			CanMsgSoftTimer = HAL_GetTick();

			if (HAL_GetTick() - DelayedStartSoftTimer > DELAYED_START_INTERVAL)
			{
				CalculateRpmSpeed(&engine_speed, &vehicle_speed);
			}

			HAL_Delay(CAN_MSG_TIME_INTERVAL);
			TxHeader.DataLength = FDCAN_DLC_BYTES_8;
			TxHeader.Identifier = RANDOM_MSG_ID;
			if ((HAL_GetTick() - RandomMsgSoftTimer > RANDOM_MSG_TIME_INTERVAL)
					&& (rng_message_flag == 1))
			{
				RandomMsgSoftTimer = HAL_GetTick();
				rng_message_flag = 0;
				HAL_RNG_GenerateRandomNumber_IT(&hrng);
				memcpy(TxData8Random, Random8Bytes, sizeof(TxData8Random));
				TxData8Random[0] = 0;
				TxData8Random[1] = 0xff;
				TxData8Random[2] = 0;
				TxData8Random[3] = 0;
				TxData8Random[4] = 0xff;
				TxData8Random[5] = 0;
				TxData8Random[6] = 0;
				TxData8Random[7] = 0;
				PrintCanFrame0bToUart(&TxHeader, TxData8Random);
				PrintCanFrame0xToUart(&TxHeader, TxData8Random);
			}
			MarkCanFrame();
			memcpy(TxData8, TxData8Random, sizeof(TxData8));
#ifndef STOP_RANDOM_MSG
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8)
					!= HAL_OK)
			{
				Error_Handler();
			}
#endif

			send_ignition_on(CAN_MSG_TIME_INTERVAL);
			send_engine_status(CAN_MSG_TIME_INTERVAL);
			send_battery_status(g_batt_fail, CAN_MSG_TIME_INTERVAL);

			EmulateIncreasingMileage();
			send_airbag_status(flip_me_variable, CAN_MSG_TIME_INTERVAL);

			send_dimming_mode(CAN_MSG_TIME_INTERVAL);
			send_dimm_level(CAN_MSG_TIME_INTERVAL);
			send_turn_indicator((g_turn_l && toggle_variable),
					(g_turn_r && toggle_variable),
					g_cruise_speed, CAN_MSG_TIME_INTERVAL);

			send_parking_brake(0, CAN_MSG_TIME_INTERVAL);

			send_rpm_and_speed(engine_speed, vehicle_speed, 0,
			CAN_MSG_TIME_INTERVAL);
			send_fuel_level(FUEL_LEVEL, CAN_MSG_TIME_INTERVAL);
			send_engine_temp(COOLANT_TEMPERATURE, CAN_MSG_TIME_INTERVAL);

#ifdef USE_INTERNAL_TEMP_SENSOR
			if (Flg.ADCCMPLT) /* Conversion completed, do calculations */
			{
				/* Temperature Sensor ADC-value, Reference Voltage ADC-value (if use) */
				Adc.IntSensTmp = TMPSENSOR_getTemperature(Adc.Raw[0],
						Adc.Raw[1]);

				Flg.ADCCMPLT = 0; /* Nullify flag */

			}
			ssd1306_SetCursor(2, 44);
			sprintf(lcd_line, "Core temp.: %ld deg. C", Adc.IntSensTmp);
			ssd1306_WriteString(lcd_line, Font_6x8, White);
			ssd1306_UpdateScreen();
			send_outside_temp((int16_t) Adc.IntSensTmp, CAN_MSG_TIME_INTERVAL);
#else
			send_outside_temp(-31, CAN_MSG_TIME_INTERVAL);
#endif
			send_instant_fuel(g_high_beam, g_rear_fog, 127,
			CAN_MSG_TIME_INTERVAL);
			send_adaptive_cruise_control(g_acc_distance, g_acc_distance2,
			g_cruise && g_acc_on, g_cruise_standby, CAN_MSG_TIME_INTERVAL);

//			if (green_button_flag == 1)
//			{
//				green_button_flag = 0;
//				send_play_alarm(CAN_MSG_TIME_INTERVAL);
//			}

			DisplayUnits();
			HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port,
			BLUE_LED_Pin);
		}
#endif
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
#ifdef USE_INTERNAL_TEMP_SENSOR
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1) /* Check if the interrupt comes from ACD1 */
	{
		/* Set flag to true */
		Flg.ADCCMPLT = 255;
	}
}
#endif

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		/* Retreive Rx messages from RX FIFO0 */
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData)
				!= HAL_OK)
		{
			/* Reception Error */
			Error_Handler();
		}

		if (HAL_FDCAN_ActivateNotification(hfdcan,
		FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
		{
			/* Notification Error */
			Error_Handler();
		}
		else
		{
			HAL_GPIO_TogglePin(YELLOW_LED_GPIO_Port,
			YELLOW_LED_Pin);
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GREEN_BUTTON_Pin)
	{
		green_button_flag = 1;
	}
}

void HAL_RNG_ReadyDataCallback(RNG_HandleTypeDef *hrng, uint32_t random32bit)
{
	rng_number_flag = 1;
}

void MarkCanFrame(void)
{
	if (green_button_flag == 1)
	{
		green_button_flag = 0;
		uint8_t uart_message[128];
		uint16_t message_size = sprintf((char*) uart_message,
				"--> green check mark\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_message, message_size, 200);
	}
}

void ComposeRandomMessage(void)
{
	typedef union
	{
		uint32_t uint32;
		uint8_t uint8[4];
	} uint32to8_t;

	uint32to8_t uint32to8_converter;

	if ((rng_number_flag == 1) && (rng_message_flag == 0)
			&& (rng_number_count < 2))
	{
		uint32to8_converter.uint32 = HAL_RNG_ReadLastRandomNumber(&hrng);
		rng_number_flag = 0;
		for (uint8_t i = 0; i < 4; i++)
		{
			Random8Bytes[i + 4 * rng_number_count] =
					uint32to8_converter.uint8[i];
		}
		rng_number_count++;
		HAL_RNG_GenerateRandomNumber_IT(&hrng);
		if (rng_number_count == 2)
		{
			rng_message_flag = 1;
			rng_number_count = 0;
		}
	}
}

void DisplayUnits(void)
{
	if (HAL_GetTick() - OledSoftTimer > 1000)
	{
		OledSoftTimer = HAL_GetTick();

		if ((RxData[2] & 0x0F) == 0x02)
		{
			sprintf(lcd_line, "Unt: meters + deg. C  ");
		}
		else if ((RxData[2] & 0x0F) == 0x03)
		{
			sprintf(lcd_line, "Unt: meters + deg. F  ");
		}
		else if ((RxData[2] & 0x0F) == 0x00)
		{
			sprintf(lcd_line, "Unt: miles + deg. C   ");
		}
		else if ((RxData[2] & 0x0F) == 0x01)
		{
			sprintf(lcd_line, "Unt: miles + deg. F   ");
		}
		else
		{
			sprintf(lcd_line, "Sth went wrong!   ");
		}
		ssd1306_SetCursor(2, 55);
		ssd1306_WriteString(lcd_line, Font_6x8, White);
		ssd1306_UpdateScreen();
	}
}

void EmulateIncreasingMileage(void)
{
	if (HAL_GetTick() - OdometerSoftTimer > 20000) // 50 km/h
	{
		OdometerSoftTimer = HAL_GetTick();
		flip_me_variable ^= 0xFF;
	}
}

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
