/*
 * mustang.h
 *
 *  Created on: Jan 9, 2024
 *      Author: user
 */

// Sources:
// https://github.com/bigunclemax/FocusIPCCtrl
// https://github.com/gizmo87898/FordFocusMK3_Cluster_BeamNG
#ifndef INC_FOCUS_H_
#define INC_FOCUS_H_

#include "main.h"
#include <string.h>

#define ENGINE_SPEED_MAX 5750 // rpm
#define ENGINE_SPEED_INCREMENT 150
#define ENGINE_SPEED_DECREMENT 300
#define VEHICLE_SPEED_MAX 185 // km/h
#define VEHICLE_SPEED_INCREMENT 1
#define ENGINE_SPEED_GEAR_SHIFT 3000 // rpm

#define BYTE_TO_BINARY_PATTERN "0b%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

#define g_batt_fail 1

/* turns */
#define g_turn_l         0
#define g_turn_r         0

/* doors */
#define g_drv_door       0
#define g_psg_door       0
#define g_rdrv_door      0
#define g_rpsg_door      0
#define g_hood           0
#define g_boot           0
/* limit and cruise */
#define g_cruise         1
#define g_cruise_standby 1
#define g_acc_on         1
#define g_cruise_on      1
#define g_limit_on       0
#define g_cruise_speed   0
#define g_acc_distance   7
#define g_acc_distance2 10
#define g_cruise_status  0

/* lights */
#define g_rear_fog       1
#define g_high_beam      1
#define g_head_lights    1

void send_outside_temp(int16_t temp, uint32_t delay_before_send);
void send_engine_temp(uint8_t temp, uint32_t delay_before_send);
void send_fuel_level(uint16_t fuel, uint32_t delay_before_send);
void send_ignition_on(uint32_t delay_before_send);
void send_engine_status(uint32_t delay_before_send);
void send_airbag_status(uint8_t add_250m_to_odometer,
		uint32_t delay_before_send);
void send_play_alarm(uint32_t delay_before_send);
void send_parking_brake(uint8_t brakeApplied, uint32_t delay_before_send);
void send_turn_indicator(uint8_t left, uint8_t right, uint16_t cruise_speed,
		uint32_t delay_before_send);
void send_battery_status(uint8_t batteryStatus, uint32_t delay_before_send);
void send_rpm_and_speed(uint16_t rpm, uint16_t speed, uint8_t speed_warning,
		uint32_t delay_before_send);
void send_dimming_mode(uint32_t delay_before_send);
void send_dimm_level(uint32_t delay_before_send);
void send_instant_fuel(uint8_t highBeam, uint8_t rearFog, uint8_t instantFuel,
		uint32_t delay_before_send);
void send_adaptive_cruise_control(uint8_t accDistance, uint8_t accDistance2,
		uint8_t accStatus, uint8_t accStandby, uint32_t delay_before_send);
void CalculateRpmSpeed(uint16_t *engine_speed, uint16_t *vehicle_speed);
void PrintCanFrame0bToUart(FDCAN_TxHeaderTypeDef *msgHeader, uint8_t *msgData);
void PrintCanFrame0xToUart(FDCAN_TxHeaderTypeDef *msgHeader, uint8_t *msgData);

#endif /* INC_FOCUS_H_ */
