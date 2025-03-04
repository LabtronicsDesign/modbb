/* ModbusDataManager.c */
#include "ModbusDataManager.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>

/* Private data structures */
static uint16_t modbus_data[MAX_MODBUS_DATA_SIZE];
static bool modbus_coil[MAX_MODBUS_COIL_SIZE];
static SemaphoreHandle_t data_mutex;
static EventGroupHandle_t data_event_group;

/* Callback management */
#define MAX_CALLBACKS 5
static DataChangeCallback data_callbacks[MAX_CALLBACKS];
static CoilChangeCallback coil_callbacks[MAX_CALLBACKS];
static uint8_t num_data_callbacks = 0;
static uint8_t num_coil_callbacks = 0;

/* Initialization function */
void ModbusDataManager_Init(void) {
    // Create mutex for thread-safe access
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        // Failed to create mutex
        // In production code, this should trigger an error handler
        for(;;);
    }

    // Create event group for notifications
    data_event_group = xEventGroupCreate();
    if (data_event_group == NULL) {
        // Failed to create event group
        vSemaphoreDelete(data_mutex);
        for(;;);
    }

    // Initialize data arrays
    memset(modbus_data, 0, sizeof(modbus_data));
    memset(modbus_coil, 0, sizeof(modbus_coil));

    // Initialize callback arrays
    memset(data_callbacks, 0, sizeof(data_callbacks));
    memset(coil_callbacks, 0, sizeof(coil_callbacks));
}

/* Data access functions */
uint16_t ModbusDataManager_GetData(uint16_t address) {
    uint16_t value = 0;

    if (address < MAX_MODBUS_DATA_SIZE) {
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            value = modbus_data[address];
            xSemaphoreGive(data_mutex);
        }
    }

    return value;
}

bool ModbusDataManager_GetCoil(uint16_t address) {
    bool value = false;

    if (address < MAX_MODBUS_COIL_SIZE) {
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            value = modbus_coil[address];
            xSemaphoreGive(data_mutex);
        }
    }

    return value;
}

void ModbusDataManager_SetData(uint16_t address, uint16_t value) {
    if (address < MAX_MODBUS_DATA_SIZE) {
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            // Only update if the value has changed
            if (modbus_data[address] != value) {
                modbus_data[address] = value;
                xSemaphoreGive(data_mutex);

                // Notify any callbacks
                for (uint8_t i = 0; i < num_data_callbacks; i++) {
                    if (data_callbacks[i] != NULL) {
                        data_callbacks[i](address, value);
                    }
                }

                // Set event bit
                xEventGroupSetBits(data_event_group, MODBUS_DATA_CHANGED_BIT);
            } else {
                xSemaphoreGive(data_mutex);
            }
        }
    }
}

void ModbusDataManager_SetCoil(uint16_t address, bool value) {
    if (address < MAX_MODBUS_COIL_SIZE) {
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            // Only update if the value has changed
            if (modbus_coil[address] != value) {
                modbus_coil[address] = value;
                xSemaphoreGive(data_mutex);

                // Notify any callbacks
                for (uint8_t i = 0; i < num_coil_callbacks; i++) {
                    if (coil_callbacks[i] != NULL) {
                        coil_callbacks[i](address, value);
                    }
                }

                // Set event bit
                xEventGroupSetBits(data_event_group, MODBUS_COIL_CHANGED_BIT);
            } else {
                xSemaphoreGive(data_mutex);
            }
        }
    }
}

/* Bulk data operations */
void ModbusDataManager_UpdateDataBulk(uint16_t startAddress, uint16_t* values, uint16_t count) {
    if (startAddress + count <= MAX_MODBUS_DATA_SIZE && values != NULL) {
        bool dataChanged = false;

        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            for (uint16_t i = 0; i < count; i++) {
                if (modbus_data[startAddress + i] != values[i]) {
                    modbus_data[startAddress + i] = values[i];
                    dataChanged = true;

                    // Notify callbacks for this specific address
                    for (uint8_t j = 0; j < num_data_callbacks; j++) {
                        if (data_callbacks[j] != NULL) {
                            data_callbacks[j](startAddress + i, values[i]);
                        }
                    }
                }
            }
            xSemaphoreGive(data_mutex);

            // Set event bit if any data changed
            if (dataChanged) {
                xEventGroupSetBits(data_event_group, MODBUS_DATA_CHANGED_BIT);
            }
        }
    }
}

void ModbusDataManager_UpdateCoilsBulk(uint16_t startAddress, bool* values, uint16_t count) {
    if (startAddress + count <= MAX_MODBUS_COIL_SIZE && values != NULL) {
        bool coilChanged = false;

        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            for (uint16_t i = 0; i < count; i++) {
                if (modbus_coil[startAddress + i] != values[i]) {
                    modbus_coil[startAddress + i] = values[i];
                    coilChanged = true;

                    // Notify callbacks for this specific address
                    for (uint8_t j = 0; j < num_coil_callbacks; j++) {
                        if (coil_callbacks[j] != NULL) {
                            coil_callbacks[j](startAddress + i, values[i]);
                        }
                    }
                }
            }
            xSemaphoreGive(data_mutex);

            // Set event bit if any coil changed
            if (coilChanged) {
                xEventGroupSetBits(data_event_group, MODBUS_COIL_CHANGED_BIT);
            }
        }
    }
}

/* Direct access functions - use with caution */
uint16_t* ModbusDataManager_LockDataAccess(void) {
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        return modbus_data;
    }

    return NULL;
}

bool* ModbusDataManager_LockCoilAccess(void) {
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        return modbus_coil;
    }

    return NULL;
}

void ModbusDataManager_ReleaseLock(void) {
    xSemaphoreGive(data_mutex);
}

/* Notification system */
void ModbusDataManager_RegisterDataChangeCallback(DataChangeCallback callback) {
    if (callback != NULL && num_data_callbacks < MAX_CALLBACKS) {
        data_callbacks[num_data_callbacks++] = callback;
    }
}

void ModbusDataManager_RegisterCoilChangeCallback(CoilChangeCallback callback) {
    if (callback != NULL && num_coil_callbacks < MAX_CALLBACKS) {
        coil_callbacks[num_coil_callbacks++] = callback;
    }
}

EventGroupHandle_t ModbusDataManager_GetEventGroup(void) {
    return data_event_group;
}

/* Memory usage reporting */
void ModbusDataManager_ReportMemoryUsage(void) {
    HeapStats_t heapStats;

    // Get heap statistics
    vPortGetHeapStats(&heapStats);

    printf("---- Memory Usage Report ----\n");
    printf("Current free bytes: %u\n", heapStats.xAvailableHeapSpaceInBytes);
    printf("Minimum ever free: %u\n", heapStats.xMinimumEverFreeBytesRemaining);
    printf("Largest free block: %u\n", heapStats.xSizeOfLargestFreeBlockInBytes);
    printf("Number of free blocks: %u\n", heapStats.xNumberOfFreeBlocks);
    printf("Allocation count: %u\n", heapStats.xNumberOfSuccessfulAllocations);
    printf("Free count: %u\n", heapStats.xNumberOfSuccessfulFrees);
    printf("----------------------------\n");
}
