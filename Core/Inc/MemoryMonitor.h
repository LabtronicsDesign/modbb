/*
 * MemoryMonitor.h
 *
 *  Created on: Feb 27, 2025
 *      Author: root
 */

#ifndef INC_MEMORYMONITOR_H_
#define INC_MEMORYMONITOR_H_

#include "FreeRTOS.h"
#include <stdint.h>
#include "stdbool.h"

/* Initialize memory monitor */
void MemoryMonitor_Init(void);

/* Get current heap and stack usage statistics */
uint32_t MemoryMonitor_GetFreeHeapSize(void);
uint32_t MemoryMonitor_GetMinEverFreeHeapSize(void);
void MemoryMonitor_PrintTaskStackUsage(void);

/* Advanced memory tracking */
void MemoryMonitor_StartTracking(const char* label);
void MemoryMonitor_StopTracking(const char* label);
void MemoryMonitor_PrintAllocationSummary(void);

/* Memory watch functions - can be set to trigger a callback when memory drops below threshold */
typedef void (*MemoryWarningCallback)(uint32_t currentFree);
void MemoryMonitor_SetWarningThreshold(uint32_t thresholdBytes, MemoryWarningCallback callback);
void MemoryMonitor_Check(void); // Call periodically to check memory

/* Stack usage monitor */
void MemoryMonitor_CheckAllTaskStacks(void);

#endif /* INC_MEMORYMONITOR_H_ */
