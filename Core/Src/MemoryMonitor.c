/* MemoryMonitor.c */
#include "MemoryMonitor.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

/* Structure to track memory allocations by label */
#define MAX_TRACKING_LABELS 10
#define MAX_LABEL_LENGTH 20

typedef struct {
    char label[MAX_LABEL_LENGTH];
    uint32_t startHeapSize;
    uint32_t endHeapSize;
    uint32_t bytesUsed;
    bool active;
} MemoryTracker_t;

/* Private variables */
static MemoryTracker_t memoryTrackers[MAX_TRACKING_LABELS];
static uint8_t numTrackers = 0;

/* Memory warning tracking */
static uint32_t warningThreshold = 0;
static MemoryWarningCallback warningCallback = NULL;

/* Initialize memory monitor */
void MemoryMonitor_Init(void) {
    memset(memoryTrackers, 0, sizeof(memoryTrackers));
}

/* Get current heap usage statistics */
uint32_t MemoryMonitor_GetFreeHeapSize(void) {
    return xPortGetFreeHeapSize();
}

uint32_t MemoryMonitor_GetMinEverFreeHeapSize(void) {
    return xPortGetMinimumEverFreeHeapSize();
}

void MemoryMonitor_PrintTaskStackUsage(void) {
    TaskStatus_t *pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    uint32_t ulTotalRunTime;

    /* Get number of tasks */
    uxArraySize = uxTaskGetNumberOfTasks();

    /* Allocate memory for task status array */
    pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

    if (pxTaskStatusArray != NULL) {
        /* Generate task status info */
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);

        printf("---- Task Stack Usage ----\n");

        /* Print stack high water mark for each task */
        for (x = 0; x < uxArraySize; x++) {
            printf("Task: %-16s Stack HWM: %u\n",
                   pxTaskStatusArray[x].pcTaskName,
                   pxTaskStatusArray[x].usStackHighWaterMark);
        }

        printf("-------------------------\n");

        /* Free the memory allocated to hold the task status */
        vPortFree(pxTaskStatusArray);
    } else {
        printf("Failed to allocate memory for task status\n");
    }
}

/* Advanced memory tracking */
void MemoryMonitor_StartTracking(const char* label) {
    if (numTrackers < MAX_TRACKING_LABELS && label != NULL) {
        strncpy(memoryTrackers[numTrackers].label, label, MAX_LABEL_LENGTH - 1);
        memoryTrackers[numTrackers].label[MAX_LABEL_LENGTH - 1] = '\0';
        memoryTrackers[numTrackers].startHeapSize = xPortGetFreeHeapSize();
        memoryTrackers[numTrackers].active = true;
        numTrackers++;
    }
}

void MemoryMonitor_StopTracking(const char* label) {
    for (uint8_t i = 0; i < numTrackers; i++) {
        if (memoryTrackers[i].active && strcmp(memoryTrackers[i].label, label) == 0) {
            memoryTrackers[i].endHeapSize = xPortGetFreeHeapSize();
            memoryTrackers[i].bytesUsed = memoryTrackers[i].startHeapSize - memoryTrackers[i].endHeapSize;
            memoryTrackers[i].active = false;
            return;
        }
    }
}

void MemoryMonitor_PrintAllocationSummary(void) {
    printf("---- Memory Allocations ----\n");

    for (uint8_t i = 0; i < numTrackers; i++) {
        if (!memoryTrackers[i].active) {
            printf("%-20s: %u bytes\n", memoryTrackers[i].label, memoryTrackers[i].bytesUsed);
        }
    }

    printf("--------------------------\n");
    printf("Total Free Heap: %u bytes\n", xPortGetFreeHeapSize());
    printf("Min Ever Free:   %u bytes\n", xPortGetMinimumEverFreeHeapSize());
    printf("--------------------------\n");
}

/* Memory watch functions */
void MemoryMonitor_SetWarningThreshold(uint32_t thresholdBytes, MemoryWarningCallback callback) {
    warningThreshold = thresholdBytes;
    warningCallback = callback;
}

void MemoryMonitor_Check(void) {
    uint32_t currentFree = xPortGetFreeHeapSize();

    if (warningCallback != NULL && currentFree < warningThreshold) {
        warningCallback(currentFree);
    }
}

/* Stack usage monitor */
void MemoryMonitor_CheckAllTaskStacks(void) {
    TaskStatus_t *pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    uint32_t ulTotalRunTime;

    /* Get number of tasks */
    uxArraySize = uxTaskGetNumberOfTasks();

    /* Allocate memory for task status array */
    pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

    if (pxTaskStatusArray != NULL) {
        /* Generate task status info */
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);

        /* Check each task's stack usage */
        for (x = 0; x < uxArraySize; x++) {
            /* Check if stack high water mark is very low (potentially dangerous) */
            if (pxTaskStatusArray[x].usStackHighWaterMark < 50) {  // Consider 50 words a critical threshold
                printf("WARNING: Task '%s' has very low stack space remaining. HWM: %u words\n",
                       pxTaskStatusArray[x].pcTaskName,
                       pxTaskStatusArray[x].usStackHighWaterMark);
            }
        }

        /* Free the memory allocated to hold the task status */
        vPortFree(pxTaskStatusArray);
    }
}
