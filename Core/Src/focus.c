/*
 * focus.c
 *
 *  Created on: Jan 9, 2024
 *      Author: user
 *
 * Sources:
 *   https://github.com/bigunclemax/FocusIPCCtrl
 *   https://github.com/gizmo87898/FordFocusMK3_Cluster_BeamNG
 *
 */

#include "focus.h"
#include "fdcan.h"
#include "usart.h"
#include "stdio.h"

extern volatile uint8_t up_button_flag;
extern volatile uint8_t down_button_flag;
extern volatile uint8_t left_button_flag;
extern volatile uint8_t right_button_flag;
extern volatile uint8_t mid_button_flag;
extern volatile uint8_t set_button_flag;
extern volatile uint8_t reset_button_flag;

extern FDCAN_TxHeaderTypeDef TxHeader;
extern uint8_t TxData8[8];

// -------------------------------------------------------------------- 0x3A (58) ----------------------
void send_turn_indicator(uint8_t left, uint8_t right, uint16_t cruise_speed,
		uint32_t delay_before_send)
{
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x3A;  // 58 DEC
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0x82;
	TxData8[1] = (0x83u | (right << 3u) | (left << 2u));
	TxData8[2] = 0;
	TxData8[3] = 0x02;
	TxData8[4] = (0x80u | (cruise_speed > 133));
	TxData8[5] = (uint8_t) (cruise_speed * 0x200 / 268);
	TxData8[6] = 0;
	TxData8[7] = 0;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// -------------------------------------------------------------------- 0x40 (64) ----------------------
void send_airbag_status(uint8_t add_250m_to_odometer,
		uint32_t delay_before_send)
{

	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x40;  // 64 DEC
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0x66;
	TxData8[1] = 0x07;
	TxData8[2] = 0x40;
	TxData8[3] = 0xff;
	TxData8[4] = 0xbe;
	TxData8[5] = add_250m_to_odometer;
	TxData8[6] = 0x38;
	TxData8[7] = 0x00;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}

}

// -------------------------------------------------------------------- 0x070 (112) ----------------------
void send_adaptive_cruise_control(uint8_t accDistance, uint8_t accDistance2,
		uint8_t accStatus, uint8_t accStandby, uint32_t delay_before_send)
{
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x070;  // 112 DEC
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0x00;
	TxData8[1] = 0x9C;
	TxData8[2] = 0x04;
	TxData8[3] = 0x80;
	TxData8[4] = 0x00;
	TxData8[5] = 0xF4;
	TxData8[6] = 0xE8;
	TxData8[7] = 0x54;

	if (accStatus && accStandby)
	{
		accDistance = accDistance * 0x10;
		TxData8[2] = accDistance;
		TxData8[4] = 0x11;
	}
	else if (accStatus)
	{
		accDistance2 = 0xEF + (accDistance2 * 2);
		TxData8[2] = 0x22;
		TxData8[3] = 0x22;
		TxData8[4] = accDistance2;
	}

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// -------------------------------------------------------------------- 0x80 (128) ----------------------
void send_ignition_on(uint32_t delay_before_send)
{

	uint8_t acc = (g_cruise_status && !g_acc_on) ? 0xa7 : 0x07;
	if (g_cruise_status && g_cruise_standby)
		acc = 0x67;
	uint8_t door = (!g_drv_door) | (!g_psg_door << 1u) | (!g_rdrv_door << 2u)
			| (!g_rpsg_door << 3u) | (!g_boot << 4u) | (!g_hood << 5u);
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x80;  // 128 DEC
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0x77; // ignition
	TxData8[1] = (g_head_lights << 7) | 0x03;
	TxData8[2] = 0x07;
	TxData8[3] = door;
	TxData8[4] = 0xD9;
	TxData8[5] = acc;
	TxData8[6] = 0x03;
	TxData8[7] = 0x82;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}

}

// -------------------------------------------------------------------- 0x110 (272) ----------------------
void send_rpm_and_speed(uint16_t rpm, uint16_t speed, uint8_t speed_warning,
		uint32_t delay_before_send)
{

	uint8_t warning = (speed_warning) ? 0xdd : 0xc0;
	rpm /= 2;
	speed = speed * 10000 / 105; //km\h

	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x110;  // 272 DEC
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = warning; // airbag
	TxData8[1] = 0xcf;
	TxData8[2] = 0;
	TxData8[3] = 0;
	TxData8[4] = (uint8_t) (rpm >> 8);
	TxData8[5] = (uint8_t) (rpm & 0xFF);
	TxData8[6] = (uint8_t) (speed >> 8);
	TxData8[7] = (uint8_t) (speed & 0xFF);
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// -------------------------------------------------------------------- 0x1A4 (420) ----------------------
void send_outside_temp(int16_t temp, uint32_t delay_before_send)
{

	temp = 0x0160 + (temp + 40) * 4;
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x1a4;  // 420 DEC

	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0;
	TxData8[1] = 0;
	TxData8[2] = 0;
	TxData8[3] = (uint8_t) ((temp >> 8u) & 0xFu);
	TxData8[4] = (uint8_t) (temp & 0xFFu);
	TxData8[5] = 0x80;
	TxData8[6] = 0;
	TxData8[7] = 0x01;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x2a0;  // 672 DEC

	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0x06;
	TxData8[1] = 0;
	TxData8[2] = 0;
	TxData8[3] = 0;
	TxData8[4] = 0x1e;
	TxData8[5] = 0x36;
	TxData8[6] = 0x9e;
	TxData8[7] = 0x4c;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// -------------------------------------------------------------------- 0x1A8 (424) ----------------------
void send_instant_fuel(uint8_t highBeam, uint8_t rearFog, uint8_t instantFuel,
		uint32_t delay_before_send)
{
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x1A8;  // 424 DEC
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0x2e;
	TxData8[1] = (0xc0u | highBeam);
	TxData8[2] = (0x2cu | rearFog << 6);
	TxData8[3] = 0x0f;
	TxData8[4] = 0x8e;
	TxData8[5] = instantFuel;
	TxData8[6] = 0xe1;
	TxData8[7] = 0xba;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// -------------------------------------------------------------------- 0x1E0 (480) ----------------------
void send_dimming_mode(uint32_t delay_before_send)
{
	HAL_Delay(delay_before_send);

	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x1e0;  // 480 DEC
	TxData8[0] = 0x42;
	TxData8[1] = 0x00;  // 0x00 - day, 0x80 - night
	TxData8[2] = 0x1c;
	TxData8[3] = 0x00;
	TxData8[4] = 0x12;
	TxData8[5] = 0x03;
	TxData8[6] = 0x91;
	TxData8[7] = 0xe3;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// -------------------------------------------------------------------- 0x240 (576) ----------------------
void send_parking_brake(uint8_t brakeApplied, uint32_t delay_before_send)
{
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x240;  // 576 DEC
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0x00;
	TxData8[1] = 0x02;
	TxData8[2] = 0x00;
	TxData8[3] = (0x40 | (brakeApplied << 7));
	TxData8[4] = 0x00; // 0x80 - with alert
	TxData8[5] = 0;
	TxData8[6] = 0;
	TxData8[7] = 0;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// -------------------------------------------------------------------- 0x250 (592) ----------------------
void send_engine_status(uint32_t delay_before_send)
{
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x250;  // 592 DEC
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0x20;
	TxData8[1] = 0xd5;
	TxData8[2] = 0x14;
	TxData8[3] = 0x0b;
	TxData8[4] = 0x08;
	TxData8[5] = 0x13;
	TxData8[6] = 0x16;
	TxData8[7] = 0x35;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// -------------------------------------------------------------------- 0x290 (656) ----------------------
void send_dimm_level(uint32_t delay_before_send)
{
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x290;  // 656 DEC
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0x98; // 0x98 airbag
	TxData8[1] = 0x00;
	TxData8[2] = 0x01; // 0x01
	TxData8[3] = 0x00;
	TxData8[4] = 0x05; //  0x05 or 0x06 dimming?
	TxData8[5] = 0x00;
	TxData8[6] = 0x00;
	TxData8[7] = 0x00;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// -------------------------------------------------------------------- 0x300 (768) ----------------------
void send_play_alarm(uint32_t delay_before_send)
{
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x300;  // 768 DEC
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0; // airbag
	TxData8[1] = 0x00; //0xC0;
	TxData8[2] = 0;
	TxData8[3] = 0;
	TxData8[4] = 0;
	TxData8[5] = 0x80;
	TxData8[6] = 0;
	TxData8[7] = 0;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// -------------------------------------------------------------------- 0x320 (800) ----------------------
void send_fuel_level(uint16_t fuel, uint32_t delay_before_send)
{

	uint16_t _fuel = 0xf00 - (0x2a + fuel * 235 / 50);
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x320;  // 800 DEC
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0;
	TxData8[1] = 0;
	TxData8[2] = (uint8_t) (0x10 | ((_fuel >> 8) & 0x0F));
	TxData8[3] = (uint8_t) (_fuel & 0xFF);
	TxData8[4] = 0;
	TxData8[5] = 0;
	TxData8[6] = 0;
	TxData8[7] = 0;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// -------------------------------------------------------------------- 0x360 (864) ----------------------
void send_engine_temp(uint8_t temp, uint32_t delay_before_send)
{
	temp += 60;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x360;  // 864 DEC
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = 0xe0;
	TxData8[1] = 0;
	TxData8[2] = 0x38;
	TxData8[3] = 0x40;
	TxData8[4] = 0;
	TxData8[5] = 0xe0;
	TxData8[6] = 0x69;
	TxData8[7] = temp;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

// -------------------------------------------------------------------- 0x508 (1288) ----------------------
void send_battery_status(uint8_t batteryStatus, uint32_t delay_before_send)
{
	//byte[1] if - 0x12 day, if 0x00 - night
	HAL_Delay(delay_before_send);
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.Identifier = 0x508;  // 1288 DEC
	memset(TxData8, 0, sizeof(TxData8));
	TxData8[0] = (0x10 | !batteryStatus);
	TxData8[1] = 0;
	TxData8[2] = 0;
	TxData8[3] = 0;
	TxData8[4] = 0x08;
	TxData8[5] = 0;
	TxData8[6] = 0;
	TxData8[7] = 0;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData8) != HAL_OK)
	{
		Error_Handler();
	}
}

void CalculateRpmSpeed(uint16_t *engine_speed, uint16_t *vehicle_speed)
{
	static uint8_t gear_number = 0;
	static uint8_t engine_speed_down = 0;

	if (!engine_speed_down)
	{
		if (*engine_speed <= ENGINE_SPEED_MAX - ENGINE_SPEED_INCREMENT)
		{
			*engine_speed = *engine_speed + ENGINE_SPEED_INCREMENT;
		}
		else if (gear_number < 5)
		{
			engine_speed_down = 1;
			gear_number++;
		}
		else
		{
			;
		}
	}
	else
	{
		if (*engine_speed > ENGINE_SPEED_GEAR_SHIFT)
		{
			*engine_speed = *engine_speed - ENGINE_SPEED_DECREMENT;
		}
		else
		{
			engine_speed_down = 0;
		}
	}

	if (*vehicle_speed <= VEHICLE_SPEED_MAX - VEHICLE_SPEED_INCREMENT)
	{
		*vehicle_speed = *vehicle_speed + VEHICLE_SPEED_INCREMENT;
	}
}

void PrintCanFrame0bToUart(FDCAN_TxHeaderTypeDef*msgHeader, uint8_t* msgData)
{
	uint8_t uart_message[128];
	uint16_t message_size =
			sprintf((char*) uart_message,
					"[0x%02X] "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN"\r\n",
					(unsigned int) msgHeader->Identifier,
					BYTE_TO_BINARY(msgData[0]),
					BYTE_TO_BINARY(msgData[1]),
					BYTE_TO_BINARY(msgData[2]),
					BYTE_TO_BINARY(msgData[3]),
					BYTE_TO_BINARY(msgData[4]),
					BYTE_TO_BINARY(msgData[5]),
					BYTE_TO_BINARY(msgData[6]),
					BYTE_TO_BINARY(msgData[7]));
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_message, message_size, 200);
}

void PrintCanFrame0xToUart(FDCAN_TxHeaderTypeDef*msgHeader, uint8_t* msgData)
{
	uint8_t uart_message[128];

	 uint16_t message_size =
			sprintf((char*) uart_message,
					"[0x%02X] 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n",
					(unsigned int) msgHeader->Identifier, msgData[0], msgData[1],
					msgData[2], msgData[3], msgData[4], msgData[5], msgData[6],
					msgData[7]);
	HAL_UART_Transmit(&huart2, uart_message, message_size, 200);
}
