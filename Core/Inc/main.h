/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Modbus.h"
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern RTC_TimeTypeDef gTime;
extern RTC_DateTypeDef gDate;
extern RTC_HandleTypeDef hrtc;
extern I2C_HandleTypeDef hi2c1;

extern modbusHandler_t ModbusHandler; // Modbus handler structure
extern uint16_t ModbusDATARX[50];
extern bool ModbusCoil[40];

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

extern uint32_t startupAction;

extern volatile bool proximityDetect;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Error_Handler_with_info(const char *file, int line);
void MIDI_NoteOn(uint8_t channel, uint8_t note, uint8_t velocity);
void MIDI_NoteOff(uint8_t channel, uint8_t note, uint8_t velocity);
void MIDI_ProgramChange(uint8_t channel, uint8_t program);
uint16_t* GetModbusDataPointer(uint16_t startAddress);
void ReadModbusData(uint8_t functionCode, uint16_t startAddress, uint16_t numRegisters);
void ReadCoilsInChunks();
void ReadInputRegistersInChunks();
void ReadHoldingRegistersInChunks();
void WriteSingleRegister(uint16_t startAddress, uint16_t value);
void WriteMultipleRegisters(uint16_t startAddress, uint16_t* values, uint16_t numRegisters);
void dispenseLEDState(bool enabled);
void printMemoryUsage();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FRAMEBUFFER 0x20000000
#define PROX_OUT_Pin GPIO_PIN_15
#define PROX_OUT_GPIO_Port GPIOG
#define PROX_OUT_EXTI_IRQn EXTI15_IRQn
#define TEST_PROX_OUT_Pin GPIO_PIN_11
#define TEST_PROX_OUT_GPIO_Port GPIOC
#define TEST_PROX_OUT_EXTI_IRQn EXTI11_IRQn
#define TS_INT_Pin GPIO_PIN_12
#define TS_INT_GPIO_Port GPIOA
#define LCD_BCKLT_Pin GPIO_PIN_9
#define LCD_BCKLT_GPIO_Port GPIOH
#define LCD_STBY_Pin GPIO_PIN_3
#define LCD_STBY_GPIO_Port GPIOC
#define EN_LCD_N_Pin GPIO_PIN_2
#define EN_LCD_N_GPIO_Port GPIOC
#define BTN_EXT_Pin GPIO_PIN_4
#define BTN_EXT_GPIO_Port GPIOC
#define BTN_EXT_EXTI_IRQn EXTI4_IRQn
#define LED_EXT_Pin GPIO_PIN_2
#define LED_EXT_GPIO_Port GPIOB
#define LED_STATUS_Pin GPIO_PIN_15
#define LED_STATUS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define ERROR_HANDLER() Error_Handler_with_info(__FILE__, __LINE__)

#define BIBOMODE_MEMO_ADDRESS 		0x083EE000 //8Kbytes to play with
#define PRESET_MEMO_ADDRESS 		0x083F0000 //8Kbytes to play with
#define CONTROL_MEMO_ADDRESS 		0x083F2000 //8Kbytes to play with
#define POWER_MEMO_ADDRESS 			0x083F4000 //8Kbytes to play with
#define MAINTAIN_MEMO_ADDRESS 		0x083F6000 //8Kbytes to play with
#define DATETIME_MEMO_ADDRESS 		0x083F8000 //8Kbytes to play with
#define ACCESS_MEMO_ADDRESS 		0x083FA000 //8Kbytes to play with
#define ENGINEERING_MEMO_ADDRESS 	0x083FC000 //8Kbytes to play with
#define LOGS_MEMO_ADDRESS 			0x083FE000 //8Kbytes to play with

// DRV2605L I2C address
#define DRV2605L_I2C_ADDR  0x5A  // 7-bit address (shifted by 1)
// DRV2605L Registers
#define DRV2605L_REG_MODE            0x01
#define DRV2605L_REG_RTPIN           0x02
#define DRV2605L_REG_WAVESEQ1        0x04
#define DRV2605L_REG_WAVESEQ2        0x05
#define DRV2605L_REG_OVERDRIVE       0x0D
#define DRV2605L_REG_SUSTAINPOS      0x0E
#define DRV2605L_REG_SUSTAINNEG      0x0F
#define DRV2605L_REG_BREAK           0x10
#define DRV2605L_REG_AUDIOMAX        0x13
#define DRV2605L_REG_FEEDBACK        0x1A
#define DRV2605L_REG_CONTROL3        0x1D
#define DRV2605L_REG_GO              0x0C

#define HOT_WATER_RESET 0
#define COLD_WATER_RESET 1
#define BIBO_DISPENSE_HOT_WATER 2
#define BIBO_DISPENSE_COLD_WATER 3
#define BIBO_DRY_BURN_DETECTION_ENABLE 4
#define BIBO_HEATER_FAULT_DETECTION_ENABLE 5
#define HEATING_SWITCH 6
#define HOT_WATER_TANK_INLET_SOLENOID_VALVE 7
#define HOT_WATER_OUTLET_SOLENOID_VALVE 8
#define COLD_WATER_TANK_INLET_SOLENOID_VALVE 9
#define COLD_WATER_OUTLET_SOLENOID_VALVE 10
#define MAIN_INLET_SOLENOID_VALVE 11
#define UV_LIGHT 12
#define WATER_FLOW_SWITCH 13
#define ENERGY_SAVING_STATUS 14
#define FAN_OUTPUT 15
#define COMPRESSOR_OUTPUT 16
#define HEATING_OUTPUT 17
#define INSTANT_HEATING_OUTPUT 18
#define CUP_DETECTION_1 19
#define CUP_DETECTION_2 20
#define HOT_WATER_TEMPERATURE_SENSOR_FAILURE 21
#define COLD_WATER_TEMPERATURE_SENSOR_FAILURE 22
#define INSTANT_HEATING_SENSOR_FAILURE 23
#define REFRIGERATION_FAILURE 24
#define UV_ERROR 25
#define DRY_BURNING 26
#define HEATING_ELEMENT_FAILURE 27
#define NO_FILTER 28
#define INVALID_CARD 29
#define FILTER_MICROSWITCH 30
#define CARD_NUMBERBYTE_1_BYTE_2 0
#define CARD_NUMBERBYTE_3_BYTE_4 1
#define FILTER_SNBYTE_1_BYTE_2 2
#define FILTER_SNBYTE_3_BYTE_4 3
#define FILTER_SNBYTE_5_BYTE_6 4
#define FILTER_SNBYTE_7_BYTE_8 5
#define FILTER_SNBYTE_9_BYTE_10 6
#define FILTER_SNBYTE_11_BYTE_12 7
#define FILTER_SNBYTE_13_BYTE_14 8
#define REMAINED_FILTER_CAPACITYL 9
#define REMAINED_FILTER_LIFETIMEHOURS 10
#define TOTAL_FILTER_CAPACITYL 11
#define TOTAL_FILTER_LIFETIMEHOURS 12
#define HOT_WATER_TEMPERATURE 13
#define COLD_WATER_TEMPERATURE 14
#define INSTANT_HEATING_TEMPERATURE 15
#define REMAINING_FLOW_RATE_ML 16
#define TOTAL_WATER_OUTPUT_BYTE_1_BYTE_2_L 17
#define TOTAL_WATER_OUTPUT_BYTE_3_BYTE_4_L 18
#define BIT_ADDRESS_DATA_00001_00016 19
#define BIT_ADDRESS_DATA_00017_00032 20
#define BIT_ADDRESS_DATA_00033_00048 21
#define CURRENT_FLOW_RATE 22
#define DISPENSE_HOT_WATER 23
#define HOT_WATER_OUTPUT_CAPACITY 24
#define PAUSE_RESUME_DISPENSE 25
#define DISPENSE_COLD_WATER 26
#define COLD_WATER_OUTPUT_CAPACITY 27
#define DISPENSE_MIXED_WATER 28
#define MIXED_WATER_OUTPUT_CAPACITY 29
#define SET_MIXED_WATER_TEMPERATURE 30
#define CLEANING 31
#define SET_HOT_WATER_TEMPERATURE 32
#define SET_COLD_WATER_TEMPERATURE 33
#define LOW_LIMITATION_OF_ENERGY_SAVING_TEMPERATURE 34
#define UP_LIMITATION_OF_ENERGY_SAVING_TEMPERATURE 35
#define UV_ON_INTERVAL_TIME_MINUTES_DURING_STANDBY 36
#define CUP_DETECTION_SENSITIVITY 37
#define DRY_BURN_DETECTION_ENABLE 38
#define HEATER_FAULT_DETECTION_ENABLE 39
#define SLEEP 40
#define FILTER_REFRESH 41
#define REMAINED_DECREASING_FILTER_CAPACITYL 42
#define REMAINED_DECREASING_FILTER_LIFE_HOUR 43
#define CLEANING_TIME 44

#define ADD_HOT_WATER_RESET 1
#define ADD_COLD_WATER_RESET 2
#define ADD_BIBO_DISPENSE_HOT_WATER 3
#define ADD_BIBO_DISPENSE_COLD_WATER 4
#define ADD_BIBO_DRY_BURN_DETECTION_ENABLE 5
#define ADD_BIBO_HEATER_FAULT_DETECTION_ENABLE 6
#define ADD_HEATING_SWITCH 7
#define ADD_HOT_WATER_TANK_INLET_SOLENOID_VALVE 17
#define ADD_HOT_WATER_OUTLET_SOLENOID_VALVE 18
#define ADD_COLD_WATER_TANK_INLET_SOLENOID_VALVE 19
#define ADD_COLD_WATER_OUTLET_SOLENOID_VALVE 20
#define ADD_MAIN_INLET_SOLENOID_VALVE 21
#define ADD_UV_LIGHT 22
#define ADD_WATER_FLOW_SWITCH 23
#define ADD_ENERGY_SAVING_STATUS 24
#define ADD_FAN_OUTPUT 25
#define ADD_COMPRESSOR_OUTPUT 26
#define ADD_HEATING_OUTPUT 27
#define ADD_INSTANT_HEATING_OUTPUT 28
#define ADD_CUP_DETECTION_1 31
#define ADD_CUP_DETECTION_2 32
#define ADD_HOT_WATER_TEMPERATURE_SENSOR_FAILURE 33
#define ADD_COLD_WATER_TEMPERATURE_SENSOR_FAILURE 34
#define ADD_INSTANT_HEATING_SENSOR_FAILURE 35
#define ADD_REFRIGERATION_FAILURE 37
#define ADD_UV_ERROR 38
#define ADD_DRY_BURNING 39
#define ADD_HEATING_ELEMENT_FAILURE 40
#define ADD_NO_FILTER 41
#define ADD_INVALID_CARD 42
#define ADD_FILTER_MICROSWITCH 43
#define ADD_CARD_NUMBERBYTE_1_BYTE_2 30001
#define ADD_CARD_NUMBERBYTE_3_BYTE_4 30002
#define ADD_FILTER_SNBYTE_1_BYTE_2 30003
#define ADD_FILTER_SNBYTE_3_BYTE_4 30004
#define ADD_FILTER_SNBYTE_5_BYTE_6 30005
#define ADD_FILTER_SNBYTE_7_BYTE_8 30006
#define ADD_FILTER_SNBYTE_9_BYTE_10 30007
#define ADD_FILTER_SNBYTE_11_BYTE_12 30008
#define ADD_FILTER_SNBYTE_13_BYTE_14 30009
#define ADD_REMAINED_FILTER_CAPACITYL 30010
#define ADD_REMAINED_FILTER_LIFETIMEHOURS 30011
#define ADD_TOTAL_FILTER_CAPACITYL 30012
#define ADD_TOTAL_FILTER_LIFETIMEHOURS 30013
#define ADD_HOT_WATER_TEMPERATURE 30014
#define ADD_COLD_WATER_TEMPERATURE 30015
#define ADD_INSTANT_HEATING_TEMPERATURE 30016
#define ADD_REMAINING_FLOW_RATE_ML 30017
#define ADD_TOTAL_WATER_OUTPUT_BYTE_1_BYTE_2_L 30018
#define ADD_TOTAL_WATER_OUTPUT_BYTE_3_BYTE_4_L 30019
#define ADD_BIT_ADDRESS_DATA_00001_00016 30020
#define ADD_BIT_ADDRESS_DATA_00017_00032 30021
#define ADD_BIT_ADDRESS_DATA_00033_00048 30022
#define ADD_CURRENT_FLOW_RATE 30023
#define ADD_DISPENSE_HOT_WATER 40001
#define ADD_HOT_WATER_OUTPUT_CAPACITY 40002
#define ADD_PAUSE_RESUME_DISPENSE 40003
#define ADD_DISPENSE_COLD_WATER 40004
#define ADD_COLD_WATER_OUTPUT_CAPACITY 40005
#define ADD_DISPENSE_MIXED_WATER 40006
#define ADD_MIXED_WATER_OUTPUT_CAPACITY 40007
#define ADD_SET_MIXED_WATER_TEMPERATURE 40008
#define ADD_CLEANING 40009
#define ADD_SET_HOT_WATER_TEMPERATURE 40010
#define ADD_SET_COLD_WATER_TEMPERATURE 40011
#define ADD_LOW_LIMITATION_OF_ENERGY_SAVING_TEMPERATURE 40012
#define ADD_UP_LIMITATION_OF_ENERGY_SAVING_TEMPERATURE 40013
#define ADD_UV_ON_INTERVAL_TIME_MINUTES_DURING_STANDBY 40014
#define ADD_CUP_DETECTION_SENSITIVITY 40015
#define ADD_DRY_BURN_DETECTION_ENABLE 40016
#define ADD_HEATER_FAULT_DETECTION_ENABLE 40017
#define ADD_SLEEP 40018
#define ADD_FILTER_REFRESH 40019
#define ADD_REMAINED_DECREASING_FILTER_CAPACITYL 40020
#define ADD_REMAINED_DECREASING_FILTER_LIFE_HOUR 40021
#define ADD_CLEANING_TIME 40022

#define BUTTON_IDLE 0x00
#define BUTTON_PRESSED 0x01
#define BUTTON_RELEASED_WITHIN_5S 0x02
#define BUTTON_RELEASED_WITHIN_30S 0x03
#define BUTTON_HELD_5S 0x06 // New flag for 5-second hold
#define BUTTON_HELD_30S 0x04
#define BUTTON_RELEASED_AFTER_30S 0x05

#define NO_MOVEMENT false
#define MOVEMENT true

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
