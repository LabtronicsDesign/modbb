/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "data_struct.h"
#include "stm32u5xx_hal.h" // for NVIC_reset
#include <string.h> // For memset
#include <stdlib.h> // For abs()
#include "ModbusDataManager.h"
#include "MemoryMonitor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t User_ButtonState;
extern uint32_t buttonPressTime;
extern bool isButtonHeld;

// Static buffer for Modbus library internal use only - not accessed directly by our code
static uint16_t modbus_lib_buffer[MAX_MODBUS_DATA_SIZE];

osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = { .name = "myQueue01" };

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask", .priority = (osPriority_t) osPriorityNormal, .stack_size = 384 * 4 };
/* Definitions for buttonTask */
osThreadId_t buttonTaskHandle;
const osThreadAttr_t buttonTask_attributes = { .name = "buttonTask", .priority = (osPriority_t) osPriorityLow, .stack_size = 64 * 4 };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void sendCheckSleep(dispenseAddressValuePair *command);
void sendCheckDispense(dispenseAddressValuePair *command);
void sendCheckSetTemp(dispenseAddressValuePair *command);
void lowMemoryWarningCallback(uint32_t currentFree);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartButtonTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName) {
	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	 called if a stack overflow is detected. */
	printf("STACK OVERFLOW in task: %s\n", pcTaskName);
	while (1)
		; // Hang for debug
}
/* USER CODE END 4 */

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void) {

}

__weak unsigned long getRunTimeCounterValue(void) {
	return 0;
}
/* USER CODE END 1 */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

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
	myQueue01Handle = osMessageQueueNew(8, sizeof(dispenseAddressValuePair), &myQueue01_attributes);
	/* USER CODE END RTOS_QUEUES */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of buttonTask */
	buttonTaskHandle = osThreadNew(StartButtonTask, NULL, &buttonTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief Function implementing the defaultTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN defaultTask */
	/* Initialize data manager and memory monitor */
		ModbusDataManager_Init();
		MemoryMonitor_Init();

		/* Set a memory warning threshold */
		MemoryMonitor_SetWarningThreshold(2048, lowMemoryWarningCallback);

		/* Track memory usage for initialization */
		MemoryMonitor_StartTracking("DefaultTaskInit");

		//remember time set
		HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
		HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);

		/* Modbus Master initialization */
		ModbusHandler.uModbusType = MB_MASTER;
		ModbusHandler.port = &huart3;
		ModbusHandler.u8id = 0; // Master ID
		ModbusHandler.u16timeOut = 1000;
		ModbusHandler.EN_Port = NULL; // No RS485

		// Use the static buffer for Modbus library's internal use
		ModbusHandler.u16regs = modbus_lib_buffer;
		ModbusHandler.u16regsize = MAX_MODBUS_DATA_SIZE;

		ModbusHandler.xTypeHW = USART_HW;
		ModbusInit(&ModbusHandler);
		ModbusStart(&ModbusHandler);

		dispenseLEDState(true);
		// Set instrument to Acoustic Grand Piano (Program 0)
		MIDI_ProgramChange(0, 4);
		MIDI_NoteOn(0, 110, 127); //playing

		//SET DEFAULT PARAMETERS
		WriteSingleRegister(ADD_CUP_DETECTION_SENSITIVITY, 2);
		WriteSingleRegister(ADD_LOW_LIMITATION_OF_ENERGY_SAVING_TEMPERATURE, 60);
		WriteSingleRegister(ADD_UP_LIMITATION_OF_ENERGY_SAVING_TEMPERATURE, 70);
		if(startupAction != 1){
			WriteSingleRegister(ADD_SET_HOT_WATER_TEMPERATURE, 92);
			WriteSingleRegister(ADD_SET_COLD_WATER_TEMPERATURE, 5);
		}
		osDelay(100);

		/* Stop tracking memory for initialization */
		MemoryMonitor_StopTracking("DefaultTaskInit");

		/* Print memory allocation summary */
		MemoryMonitor_PrintAllocationSummary();

		/* Infinite loop */
		for (;;) {
			/* Check if we're dispensing */
			bool isDispensingHot = ModbusDataManager_GetCoil(BIBO_DISPENSE_HOT_WATER);
			bool isDispensingCold = ModbusDataManager_GetCoil(BIBO_DISPENSE_COLD_WATER);

			if(!isDispensingHot && !isDispensingCold){
				dispenseLEDState(false);
				ReadCoilsInChunks();  // Read function code 1 data
				ReadInputRegistersInChunks();  // Read function code 4 data
				ReadHoldingRegistersInChunks();  // Read function code 3 data
			}
			else{ //DISPENSING
				dispenseLEDState(true);
				/* Track memory for dispense operations */
				MemoryMonitor_StartTracking("DispensingOperation");

				ReadModbusData(1, ADD_CUP_DETECTION_1, 6);  // Read 13 coils starting from address 31
				bool tempCoils[6];
				for (int i = 0; i < 6; i++)
				{
					tempCoils[i] = bitRead(ModbusDATARX[48], i);
				}
				/* Update coils in the data manager */
				ModbusDataManager_UpdateCoilsBulk(CUP_DETECTION_1, tempCoils, 6);

				ReadModbusData(4, ADD_REMAINING_FLOW_RATE_ML, 1);  // Reading addresses 30010-30018
				/* Update data in the data manager */
				ModbusDataManager_SetData(REMAINING_FLOW_RATE_ML, ModbusDATARX[REMAINING_FLOW_RATE_ML]);

				static uint8_t counter = 0;
				counter++;
				if(counter == 5) {
					counter = 0;
					ReadModbusData(1, ADD_HOT_WATER_RESET, 7);
					bool tempResetCoils[7];
					for (int i = 0; i < 7; i++) {
						if ((i + HOT_WATER_RESET) < sizeof(ModbusCoil)/sizeof(ModbusCoil[0])) {
							tempResetCoils[i] = bitRead(ModbusDATARX[45], i);
						}
					}
					/* Update coils in the data manager */
					ModbusDataManager_UpdateCoilsBulk(HOT_WATER_RESET, tempResetCoils, 7);
				}

				MemoryMonitor_StopTracking("DispensingOperation");
			}

			dispenseAddressValuePair newCommand = {
				.address = ADD_DISPENSE_HOT_WATER,
				.buffer = {0, 0, 0, 0, 0, 0, 0, 0},
				.size = 8
			};
			if (osMessageQueueGet(myQueue01Handle, &newCommand, NULL, 0) != osOK) { // Failed to receive data from the queue
					__NOP();
			}
			else { // Data received successfully
				if(newCommand.size == 1){ //single command register request
					if(newCommand.address==ADD_SLEEP){
						sendCheckSleep(&newCommand);
					}
					else{ //any other single register command
						WriteSingleRegister(newCommand.address, newCommand.buffer[0]);
					}
				}
				else{ //normal multiple register command
					if(newCommand.address==ADD_DISPENSE_HOT_WATER || newCommand.address==ADD_DISPENSE_COLD_WATER || newCommand.address==ADD_DISPENSE_MIXED_WATER){
						sendCheckDispense(&newCommand);
					}
					else if(newCommand.address == ADD_SET_HOT_WATER_TEMPERATURE){
						sendCheckSetTemp(&newCommand);
					}
					else{ //any other multiple register command
						WriteMultipleRegisters(newCommand.address, newCommand.buffer, newCommand.size);
					}
				}
			}

			/* Periodically check stack usage */
			static uint32_t stackCheckCounter = 0;
			if (++stackCheckCounter >= 100) { // Check every ~10 seconds
				stackCheckCounter = 0;
				MemoryMonitor_PrintTaskStackUsage();
			}

			osDelay(100);
		}
	/* USER CODE END defaultTask */
}

/* USER CODE BEGIN Header_StartButtonTask */
/**
 * @brief Function implementing the buttonTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void *argument) {
	/* USER CODE BEGIN buttonTask */
	// Track if we need to send a stop command next
	bool needStopCommand = false;

	/* Track memory usage of this task */
	MemoryMonitor_StartTracking("ButtonTaskInit");

	/* Infinite loop */
	for (;;) {
		if (isButtonHeld) {
			uint32_t currentTime = HAL_GetTick();
			uint32_t heldDuration = currentTime - buttonPressTime;

			if (heldDuration >= 5000 && heldDuration < 30000) {
				// Set flag for 5-second hold if not already set
				if (User_ButtonState != BUTTON_HELD_5S) {
					User_ButtonState = BUTTON_HELD_5S;
					HAL_GPIO_WritePin(LCD_STBY_GPIO_Port, LCD_STBY_Pin, GPIO_PIN_RESET);
				}
			}
			if (heldDuration >= 30000) {
				User_ButtonState = BUTTON_HELD_30S;
				isButtonHeld = false; // Reset to avoid repeated setting
				MIDI_NoteOn(0, 110, 127); //playing
			}
		}

		// Check for button press (assuming you have a function or variable to detect single press)
		// For example, if you have a buttonPressed flag that's set elsewhere when button is pressed and released
		if (User_ButtonState == BUTTON_PRESSED && !needStopCommand) {
			// Button pressed, send a random dispense command
			dispenseAddressValuePair command;

			// Generate a random number between 0 and 2 to select which command to send
			static uint32_t randomSeed = 0;
			randomSeed++;
			uint32_t randomCmd = randomSeed % 3;

			switch (randomCmd) {
				case 0:
					// Hot water command
					command.address = 40001;
					command.buffer[0] = 0xff00;
					command.buffer[1] = 200;
					command.size = 2;
					break;

				case 1:
					// Cold water command
					command.address = 40004;
					command.buffer[0] = 0xff00;
					command.buffer[1] = 300;
					command.size = 2;
					break;

				case 2:
					// Mixed water command
					command.address = 40006;
					command.buffer[0] = 0xff00;
					command.buffer[1] = 500;
					command.buffer[2] = 35;
					command.size = 3;
					break;
			}

			// Send command to queue
			osMessageQueuePut(myQueue01Handle, &command, 0, 0);

			// Set flag to send stop command next
			needStopCommand = true;

			// Clear button state
			User_ButtonState = BUTTON_IDLE;
		} else if (User_ButtonState == BUTTON_PRESSED && needStopCommand) {
			// Need to send stop command
			dispenseAddressValuePair stopCommand = { .address = 40001, .buffer = { 0, 0, 0, 0, 0, 0, 0, 0 }, .size = 8 };

			// Send stop command to queue
			osMessageQueuePut(myQueue01Handle, &stopCommand, 0, 0);

			// Reset flag
			needStopCommand = false;

			// Clear button state
			User_ButtonState = BUTTON_IDLE;
		}

		// Periodically check memory
		MemoryMonitor_Check();
		osDelay(100);
	}
	/* USER CODE END buttonTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void sendCheckSleep(dispenseAddressValuePair *command) {
	if (command->buffer[0] == 0xFF00) {
		//energy saving mode
		do {
			WriteSingleRegister(command->address, command->buffer[0]);
			osDelay(2);
			ReadModbusData(1, ADD_WATER_FLOW_SWITCH, 6);  // Read water flow switch coils
            // Continue until energy saving mode is active
		} while (!ModbusDataManager_GetCoil(ENERGY_SAVING_STATUS)); //repeat if still normal mode
	} else {
		//0x0 normal mode
		do {
			WriteSingleRegister(command->address, command->buffer[0]);
			osDelay(2);
			ReadModbusData(1, ADD_WATER_FLOW_SWITCH, 6);  // Read water flow switch coils
            // Continue until normal mode is active
		} while (ModbusDataManager_GetCoil(ENERGY_SAVING_STATUS)); //repeat if still in energy saving mode
	}
}

void sendCheckDispense(dispenseAddressValuePair *command) {
	if (command->buffer[0] == 0xFF00) {
		//dispense command and checker
		do {
			WriteMultipleRegisters(command->address, command->buffer, command->size);
			osDelay(10);
			ReadModbusData(3, ADD_DISPENSE_HOT_WATER, 8); //confirms dispense has been sent
			ReadModbusData(1, ADD_HOT_WATER_TANK_INLET_SOLENOID_VALVE, 6);  // Read valve status
		} while (!ModbusDataManager_GetCoil(MAIN_INLET_SOLENOID_VALVE)); // repeats if dispense was not initiated

		ReadModbusData(1, ADD_CUP_DETECTION_1, 6); // Read cup detection coils
		ReadModbusData(4, ADD_REMAINING_FLOW_RATE_ML, 1); //checks dispense has been set
	} else if (command->buffer[0] == 0) {
		//stop dispense command and checker
		do {
			WriteMultipleRegisters(command->address, command->buffer, command->size);
			osDelay(10);
			ReadModbusData(3, ADD_DISPENSE_HOT_WATER, 8); //confirms dispense has been sent
			ReadModbusData(1, ADD_HOT_WATER_TANK_INLET_SOLENOID_VALVE, 6);  // Read valve status
		} while (ModbusDataManager_GetCoil(MAIN_INLET_SOLENOID_VALVE)); // repeats if still dispensing
	}
}

void sendCheckSetTemp(dispenseAddressValuePair *command) {
	int tempAddress;
	do {
		WriteSingleRegister(command->address, command->buffer[0]);
		osDelay(2);
		ReadModbusData(3, ADD_SET_HOT_WATER_TEMPERATURE, 2); //confirms changes

		/* Update data in the data manager */
		ModbusDataManager_SetData(SET_HOT_WATER_TEMPERATURE, ModbusDATARX[SET_HOT_WATER_TEMPERATURE]);
		ModbusDataManager_SetData(SET_COLD_WATER_TEMPERATURE, ModbusDATARX[SET_COLD_WATER_TEMPERATURE]);

		if(command->address == ADD_SET_HOT_WATER_TEMPERATURE) tempAddress = SET_HOT_WATER_TEMPERATURE;
		else tempAddress = SET_COLD_WATER_TEMPERATURE;
	} while (ModbusDATARX[tempAddress] != command->buffer[0]); // repeats if change not propagated
}

/* Memory warning callback */
void lowMemoryWarningCallback(uint32_t currentFree) {
    printf("WARNING: Memory is low! Only %lu bytes free\n", currentFree);
}

/*
 * MIDI AUDIO SOUNDS
 */
void MIDI_NoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {
	uint8_t msg[3];
	msg[0] = 0x90 | (channel & 0x0F); // Note On message
	msg[1] = note & 0x7F;             // Note number (0-127)
	msg[2] = velocity & 0x7F;         // Velocity (0-127)
	HAL_UART_Transmit(&huart1, msg, 3, HAL_MAX_DELAY);
}

void MIDI_NoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {
	uint8_t msg[3];
	msg[0] = 0x80 | (channel & 0x0F); // Note Off message
	msg[1] = note & 0x7F;             // Note number (0-127)
	msg[2] = velocity & 0x7F;         // Velocity (usually 0)
	HAL_UART_Transmit(&huart1, msg, 3, HAL_MAX_DELAY);
}

void MIDI_ProgramChange(uint8_t channel, uint8_t program) {
	uint8_t msg[2];
	msg[0] = 0xC0 | (channel & 0x0F); // Program Change message
	msg[1] = program & 0x7F;          // Program number (0-127)
	HAL_UART_Transmit(&huart1, msg, 2, HAL_MAX_DELAY);
}

/*
 * MODBUS FUNCTIONS
 */

// Function to read Modbus coils (Function Code 1)
void ReadModbusCoils(uint16_t startAddress, uint16_t numCoils) {
    modbus_t telegram;
    static uint16_t tempRegBuffer[16]; // Buffer for raw coil data (16 bit registers)

    // Validate parameters
    if (numCoils > 256) { // Max limit for Modbus coils in one request
        printf("Error: Requested coil count exceeds maximum allowed\n");
        return;
    }

    // Fill the telegram structure for coil reading
    telegram.u8id = 1;                // Modbus slave ID
    telegram.u8fct = 1;               // Function code 1 (Read Coils)
    telegram.u16RegAdd = startAddress; // Starting address
    telegram.u16CoilsNo = numCoils;    // Number of coils to read
    telegram.u16reg = tempRegBuffer;   // Use our temporary buffer

    // Send query to read coils
    ModbusQuery(&ModbusHandler, telegram);

    // Wait for response
    uint32_t result = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Check if result is an error code (not OP_OK_QUERY which is 20)
    if (result != 0 && result != 20) {
        // Error occurred
        printf("Modbus coil read error for address %u, error code: %lu\n", startAddress, result);
        return;
    }

    // Determine which coil index to start updating in the data manager
    uint16_t coilStartIndex = 0;

    if (startAddress == ADD_HOT_WATER_RESET) {
        coilStartIndex = HOT_WATER_RESET;
//        printf("Reading HOT_WATER_RESET coils, data: 0x%04X\n", tempRegBuffer[0]);
    }
    else if (startAddress == ADD_HOT_WATER_TANK_INLET_SOLENOID_VALVE) {
        coilStartIndex = HOT_WATER_TANK_INLET_SOLENOID_VALVE;
//        printf("Reading HOT_WATER_TANK_INLET_SOLENOID_VALVE coils, data: 0x%04X\n", tempRegBuffer[0]);
    }
    else if (startAddress == ADD_WATER_FLOW_SWITCH) {
        coilStartIndex = WATER_FLOW_SWITCH;
//        printf("Reading WATER_FLOW_SWITCH coils, data: 0x%04X\n", tempRegBuffer[0]);
    }
    else if (startAddress == ADD_CUP_DETECTION_1) {
        coilStartIndex = CUP_DETECTION_1;
//        printf("Reading CUP_DETECTION_1 coils, data: 0x%04X\n", tempRegBuffer[0]);
    }
    else if (startAddress == ADD_UV_ERROR) {
        coilStartIndex = UV_ERROR;
//        printf("Reading UV_ERROR coils, data: 0x%04X\n", tempRegBuffer[0]);
    }
    else {
//        printf("Unknown coil address: %u\n", startAddress);
        return;
    }

    // Extract individual coil values (bits) from the response
    bool coilValues[16]; // Maximum 16 coils per register
    uint16_t numCoilsToProcess = (numCoils > 16) ? 16 : numCoils;

    // Debug the raw value before bit extraction
//    printf("Raw coil register value: 0x%04X\n", tempRegBuffer[0]);

    // Extract each bit
    for (int i = 0; i < numCoilsToProcess; i++) {
        // For debugging, print each bit being extracted
        bool bitValue = bitRead(tempRegBuffer[0], i);
//        printf("  Bit %d: %d\n", i, bitValue);
        coilValues[i] = bitValue;
    }

    // Update data manager
    ModbusDataManager_UpdateCoilsBulk(coilStartIndex, coilValues, numCoilsToProcess);

    osDelay(10);
}

// Function to read Modbus registers (Function Code 3 or 4)
void ReadModbusRegisters(uint8_t functionCode, uint16_t startAddress, uint16_t numRegisters) {
    modbus_t telegram;
    static uint16_t tempRegBuffer[MAX_MODBUS_DATA_SIZE]; // Temporary buffer for communication

    // Validate parameters
    if (numRegisters > MAX_MODBUS_DATA_SIZE) {
        printf("Error: Requested register count exceeds maximum buffer size\n");
        return;
    }

    // Ensure we're only handling register read functions
    if (functionCode != 3 && functionCode != 4) {
        printf("Error: Invalid function code for register reading: %d\n", functionCode);
        return;
    }

    // Fill the telegram structure
    telegram.u8id = 1;                // Modbus slave ID
    telegram.u8fct = functionCode;    // Function code (3 or 4)
    telegram.u16RegAdd = startAddress; // Starting address
    telegram.u16CoilsNo = numRegisters; // Number of registers to read
    telegram.u16reg = tempRegBuffer;  // Use our temporary buffer

    // Send query to read registers
    ModbusQuery(&ModbusHandler, telegram);

    // Wait for response
    uint32_t result = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Check if result is an error code (not OP_OK_QUERY which is 20)
    if (result != 0 && result != 20) {
        // Error occurred
        printf("Modbus register read error for function code %d, error code: %lu\n", functionCode, result);
        return;
    }

    // For registers (function code 3 or 4)
    uint16_t regStartIndex = 0;

    if (startAddress >= 30001 && startAddress <= 30023) {
        // Input registers
        regStartIndex = startAddress - 30001;
    }
    else if (startAddress >= 40001 && startAddress <= 40022) {
        // Holding registers
        regStartIndex = startAddress - 40001 + 23; // Offset to after input registers
    }
    else {
        printf("Unknown register address range: %u\n", startAddress);
        return;
    }

    // For debugging, print the registers we received
//    printf("Read %d registers starting at address %u:\n", numRegisters, startAddress);
//    for (int i = 0; i < numRegisters; i++) {
//        printf("  Register[%d] = 0x%04X\n", i, tempRegBuffer[i]);
//    }

    // Update data manager with register values
    ModbusDataManager_UpdateDataBulk(regStartIndex, tempRegBuffer, numRegisters);

    osDelay(10);
}

// Main wrapper function that directs the call to the appropriate specialized function
void ReadModbusData(uint8_t functionCode, uint16_t startAddress, uint16_t numItems) {
    // Route the call to the appropriate function based on function code
    if (functionCode == 1) {
        // For coils (function code 1)
        ReadModbusCoils(startAddress, numItems);
    }
    else if (functionCode == 3 || functionCode == 4) {
        // For holding registers (3) or input registers (4)
        ReadModbusRegisters(functionCode, startAddress, numItems);
    }
    else {
        printf("Error: Unsupported function code: %d\n", functionCode);
    }
}

void ReadCoilsInChunks() {
    uint16_t count;

    // Read chunk 1: Coils from address 1 to 7
    count = abs(ADD_HOT_WATER_RESET - ADD_HEATING_SWITCH) + 1;
    ReadModbusData(1, ADD_HOT_WATER_RESET, count);  // Read 7 coils starting from address 1

    // Read chunk 2: Coils from address 17 to 22
    count = abs(ADD_HOT_WATER_TANK_INLET_SOLENOID_VALVE - ADD_UV_LIGHT) + 1;
    ReadModbusData(1, ADD_HOT_WATER_TANK_INLET_SOLENOID_VALVE, count);

    // Read chunk 3: Coils from address 23 to 28
    count = abs(ADD_WATER_FLOW_SWITCH - ADD_INSTANT_HEATING_OUTPUT) + 1;
    ReadModbusData(1, ADD_WATER_FLOW_SWITCH, count);

    // Read chunk 4: Coils from address 31 to 35
    count = 6;
    ReadModbusData(1, ADD_CUP_DETECTION_1, count);

    // Read chunk 5: Coils from address 37 to 43
    count = 6;
    ReadModbusData(1, ADD_UV_ERROR, count);
}

// Read input registers (function code 4)
void ReadInputRegistersInChunks() {
    uint16_t count;

    // Read chunk 1: registers from address 0 to 9
    count = abs(ADD_CARD_NUMBERBYTE_1_BYTE_2 - ADD_FILTER_SNBYTE_13_BYTE_14) + 1;
    ReadModbusData(4, ADD_CARD_NUMBERBYTE_1_BYTE_2, count);

    // Read chunk 2: registers from address 10 to 16
    count = abs(ADD_REMAINED_FILTER_CAPACITYL - ADD_REMAINING_FLOW_RATE_ML) + 1;
    ReadModbusData(4, ADD_REMAINED_FILTER_CAPACITYL, count);

    // Read chunk 3: registers from address 17 to 22
    count = abs(ADD_TOTAL_WATER_OUTPUT_BYTE_1_BYTE_2_L - ADD_CURRENT_FLOW_RATE) + 1;
    ReadModbusData(4, ADD_TOTAL_WATER_OUTPUT_BYTE_1_BYTE_2_L, count);
}

// Read holding registers (function code 3)
void ReadHoldingRegistersInChunks() {
    uint16_t count;

    // Read chunk 1: registers from address 23 to 30
    count = abs(ADD_DISPENSE_HOT_WATER - ADD_SET_MIXED_WATER_TEMPERATURE) + 1;
    ReadModbusData(3, ADD_DISPENSE_HOT_WATER, count);

    // Read chunk 2: registers from address 31 to 37
    count = abs(ADD_CLEANING - ADD_CUP_DETECTION_SENSITIVITY) + 1;
    ReadModbusData(3, ADD_CLEANING, count);

    // Read chunk 3: registers from address 38 to 44
    count = abs(ADD_DRY_BURN_DETECTION_ENABLE - ADD_FILTER_REFRESH) + 1;
    ReadModbusData(3, ADD_DRY_BURN_DETECTION_ENABLE, count);

    //Last 40020+ are not read as 03 function code not present
}

// Function to write to a single register on the slave (Function Code 6)
void WriteSingleRegister(uint16_t startAddress, uint16_t value) {
    modbus_t telegram;
    // Ensure the address is within the writable range (40001-40022)
    if (startAddress < 40001 || startAddress > 40022) {
        printf("Error: Invalid register address %u\n", startAddress);
        return;
    }

    // Fill the telegram structure
    telegram.u8id = 1;            // Modbus slave ID
    telegram.u8fct = MB_FC_WRITE_REGISTER;         // Function code 6 (write single register)
    telegram.u16RegAdd = startAddress;             // Starting address (e.g., 40001)
    telegram.u16CoilsNo = 1;                       // Number of registers to write (1 for this function)
    telegram.u16reg = &value;                      // Pointer to the value to be written

    // Send the query to the Modbus slave
    ModbusQueryInject(&ModbusHandler, telegram);

    // Wait for response
    uint32_t result = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait indefinitely for task notification
    if (result != 0 && result != 20) {
       printf("Modbus write error for register address %u, error code: %lu\n", startAddress, result);
       return;
    }

    // If successful, update data manager
    if (startAddress >= 40001 && startAddress <= 40022) {
        uint16_t dataIndex = startAddress - 40001 + DISPENSE_HOT_WATER;
        ModbusDataManager_SetData(dataIndex, value);
    }

    osDelay(5);
}

// Function to write to multiple registers on the slave (Function Code 16)
void WriteMultipleRegisters(uint16_t startAddress, uint16_t* values, uint16_t numRegisters) {
    modbus_t telegram;
    // Ensure the address and number of registers are within the writable range
    if (startAddress < 40001 || (startAddress + numRegisters - 1) > 40022) {
    	printf("Error: Invalid register address or range (start: %u, count: %u)\n", startAddress, numRegisters);
        return;
    }

    // Fill the telegram structure
    telegram.u8id = 1;            // Modbus slave ID
    telegram.u8fct = MB_FC_WRITE_MULTIPLE_REGISTERS; // Function code 16 (write multiple registers)
    telegram.u16RegAdd = startAddress;             // Starting address (e.g., 40001)
    telegram.u16CoilsNo = numRegisters;            // Number of registers to write
    telegram.u16reg = values;                      // Pointer to the values to be written

    // Send the query to the Modbus slave
    ModbusQueryInject(&ModbusHandler, telegram);

    // Wait for response
    uint32_t result = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait indefinitely for task notification
    if (result != 0 && result != 20) {
    	printf("Modbus write error for register range starting at address %u, error code: %lu\n", startAddress, result);
    	return;
    }

    // If successful, update data manager
    if (startAddress >= 40001 && startAddress <= 40022) {
        uint16_t dataStartIndex = startAddress - 40001 + DISPENSE_HOT_WATER;
        for (uint16_t i = 0; i < numRegisters; i++) {
            if (dataStartIndex + i < MAX_MODBUS_DATA_SIZE) {
                ModbusDataManager_SetData(dataStartIndex + i, values[i]);
            }
        }
    }

    osDelay(5);
}

void dispenseLEDState(bool enabled) {
	bool dispenseLEDOn = enabled;
	static bool previousDispenseLEDOn = false;

	// Holds the last state of dispenseLEDOn
	if (dispenseLEDOn != previousDispenseLEDOn){
		// Update the LED state
		if (dispenseLEDOn) {
			HAL_GPIO_WritePin(LED_EXT_GPIO_Port, LED_EXT_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(LED_EXT_GPIO_Port, LED_EXT_Pin, GPIO_PIN_SET);
		}
		// Update the previous state
		previousDispenseLEDOn = dispenseLEDOn;
	}
}

/* USER CODE END Application */
