/*
 * ModbusDataManager.h
 *
 *  Created on: Feb 27, 2025
 *      Author: root
 */

#ifndef INC_MODBUSDATAMANAGER_H_
#define INC_MODBUSDATAMANAGER_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include "stdbool.h"
#include "stdint.h"
#include "event_groups.h"

/* Define the maximum sizes for data arrays */
#define MAX_MODBUS_DATA_SIZE   45  // Limited to exactly the registers we need
#define MAX_MODBUS_COIL_SIZE   31  // Limited to exactly the coils we need

/* Define event bits for data change notifications */
#define MODBUS_DATA_CHANGED_BIT  (1 << 0)
#define MODBUS_COIL_CHANGED_BIT  (1 << 1)

/* Data change callback function type */
typedef void (*DataChangeCallback)(uint16_t address, uint16_t value);
typedef void (*CoilChangeCallback)(uint16_t address, bool value);

/* Initialization function - must be called before using any other functions */
void ModbusDataManager_Init(void);

/* Data access functions */
uint16_t ModbusDataManager_GetData(uint16_t address);
bool ModbusDataManager_GetCoil(uint16_t address);
void ModbusDataManager_SetData(uint16_t address, uint16_t value);
void ModbusDataManager_SetCoil(uint16_t address, bool value);

/* Bulk data operations for efficient updates */
void ModbusDataManager_UpdateDataBulk(uint16_t startAddress, uint16_t* values, uint16_t count);
void ModbusDataManager_UpdateCoilsBulk(uint16_t startAddress, bool* values, uint16_t count);

/* Get direct pointers to data (use with caution and only when necessary) */
/* These functions take a semaphore that must be released with ReleaseLock() */
uint16_t* ModbusDataManager_LockDataAccess(void);
bool* ModbusDataManager_LockCoilAccess(void);
void ModbusDataManager_ReleaseLock(void);

/* Notification system */
void ModbusDataManager_RegisterDataChangeCallback(DataChangeCallback callback);
void ModbusDataManager_RegisterCoilChangeCallback(CoilChangeCallback callback);
EventGroupHandle_t ModbusDataManager_GetEventGroup(void);

/* Memory usage reporting */
void ModbusDataManager_ReportMemoryUsage(void);

#endif /* INC_MODBUSDATAMANAGER_H_ */
