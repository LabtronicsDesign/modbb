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

osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};
/* Definitions for buttonTask */
osThreadId_t buttonTaskHandle;
const osThreadAttr_t buttonTask_attributes = {
  .name = "buttonTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void sendCheckSleep(dispenseAddressValuePair *command);
void sendCheckDispense(dispenseAddressValuePair *command);
void sendCheckSetTemp(dispenseAddressValuePair *command);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartButtonTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
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
	 myQueue01Handle = osMessageQueueNew (8, sizeof(dispenseAddressValuePair), &myQueue01_attributes);
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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
	/* USER CODE BEGIN defaultTask */
		//remember time set
		HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
		HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);

		//int cycleCounter = 0;

		/* Modbus Slave initialization */
		ModbusHandler.uModbusType = MB_MASTER;
		ModbusHandler.port = &huart3;
		ModbusHandler.u8id = 0; //Master ID
		ModbusHandler.u16timeOut = 1000;
		ModbusHandler.EN_Port = NULL; // No RS485
		ModbusHandler.u16regs = ModbusDATARX;
		ModbusHandler.u16regsize = sizeof(ModbusDATARX) / sizeof(ModbusDATARX[0]);
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

		/* Infinite loop */
		for (;;) {
			if(!ModbusCoil[BIBO_DISPENSE_HOT_WATER] && !ModbusCoil[BIBO_DISPENSE_COLD_WATER]){
				dispenseLEDState(false);
				ReadCoilsInChunks();  // Read function code 1 data
				ReadInputRegistersInChunks();  // Read function code 4 data
				ReadHoldingRegistersInChunks();  // Read function code 3 data
			}
			else{ //DISPENSING
				dispenseLEDState(true);
				ReadModbusData(1, ADD_CUP_DETECTION_1, 6);  // Read 13 coils starting from address 31
				for (int i = 0; i < 6; i++)
				{
					ModbusCoil[i+CUP_DETECTION_1] = bitRead(ModbusDATARX[48], i);  // Extract each coil state
				}
				ReadModbusData(4, ADD_REMAINING_FLOW_RATE_ML, 1);  // Reading addresses 30010-30018
				static uint8_t counter = 0;
				counter++;
				if(counter == 5) {
				    counter = 0;
				    ReadModbusData(1, ADD_HOT_WATER_RESET, 7);
				    for (int i = 0; i < 7; i++) {
				        if ((i + HOT_WATER_RESET) < sizeof(ModbusCoil)/sizeof(ModbusCoil[0])) {
				            ModbusCoil[i + HOT_WATER_RESET] = bitRead(ModbusDATARX[45], i);
				        }
				    }
				}
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

			UBaseType_t uxHighWaterMark;
			if(defaultTaskHandle != NULL)
			{
				uxHighWaterMark = uxTaskGetStackHighWaterMark(defaultTaskHandle);
				printf("Task: %s, Stack HWM: %lu\n", pcTaskGetName(defaultTaskHandle), uxHighWaterMark);
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
void StartButtonTask(void *argument)
{
  /* USER CODE BEGIN buttonTask */
	// Track if we need to send a stop command next
	  bool needStopCommand = false;
	/* Infinite loop */
	  for(;;)
	  {
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
		      }
		      else if (User_ButtonState == BUTTON_PRESSED && needStopCommand) {
		        // Need to send stop command
		        dispenseAddressValuePair stopCommand = {
		          .address = 40001,
		          .buffer = {0, 0, 0, 0, 0, 0, 0, 0},
		          .size = 8
		        };

		        // Send stop command to queue
		        osMessageQueuePut(myQueue01Handle, &stopCommand, 0, 0);

		        // Reset flag
		        needStopCommand = false;

		        // Clear button state
		        User_ButtonState = BUTTON_IDLE;
		      }

		      UBaseType_t uxHighWaterMark;
				if(buttonTaskHandle != NULL)
				{
					uxHighWaterMark = uxTaskGetStackHighWaterMark(buttonTaskHandle);
					printf("Task: %s, Stack HWM: %lu\n", pcTaskGetName(buttonTaskHandle), uxHighWaterMark);
				}
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
			ReadModbusData(1, ADD_WATER_FLOW_SWITCH, 6);  // Read 12 coils starting from address 17
			for (int i = 0; i < 6; i++)
			{
				ModbusCoil[i+WATER_FLOW_SWITCH] = bitRead(ModbusDATARX[47], i);  // Extract each coil state
			}
		} while (!ModbusCoil[ENERGY_SAVING_STATUS]); //repeat if still normal mode
	} else {
		//0x0 normal mode
		do {
			WriteSingleRegister(command->address, command->buffer[0]);
			osDelay(2);
			ReadModbusData(1, ADD_WATER_FLOW_SWITCH, 6);  // Read 12 coils starting from address 17
			for (int i = 0; i < 6; i++)
			{
				ModbusCoil[i+WATER_FLOW_SWITCH] = bitRead(ModbusDATARX[47], i);  // Extract each coil state
			}
		} while (ModbusCoil[ENERGY_SAVING_STATUS]); //repeat if still in energy saving mode
	}
}

void sendCheckDispense(dispenseAddressValuePair *command) {
	if (command->buffer[0] == 0xFF00) {
		//dispense command and checker
		do {
			WriteMultipleRegisters(command->address, command->buffer, command->size);
			osDelay(10);
			ReadModbusData(3, ADD_DISPENSE_HOT_WATER, 8); //confirms dispense has been sent for debugging checks
			ReadModbusData(1, ADD_HOT_WATER_TANK_INLET_SOLENOID_VALVE, 6);  // Read 12 coils starting from address 17
			for (int i = 0; i < 6; i++)
			{
				ModbusCoil[i+HOT_WATER_TANK_INLET_SOLENOID_VALVE] = bitRead(ModbusDATARX[46], i);  // Extract each coil state
			}
		} while (!ModbusCoil[MAIN_INLET_SOLENOID_VALVE]); // repeats if dispense was not initiated
		ReadModbusData(1, ADD_CUP_DETECTION_1, 6); // Read 13 coils starting from address 31
		for (int i = 0; i < 6; i++) {
			ModbusCoil[i + CUP_DETECTION_1] = bitRead(ModbusDATARX[48], i);
		}
		ReadModbusData(4, ADD_REMAINING_FLOW_RATE_ML, 1); //checks dispense has been set
	} else if (command->buffer[0] == 0) {
		//stop dispense command and checker
		do {
			WriteMultipleRegisters(command->address, command->buffer, command->size);
			osDelay(10);
			ReadModbusData(3, ADD_DISPENSE_HOT_WATER, 8); //confirms dispense has been sent for debugging checks
			ReadModbusData(1, ADD_HOT_WATER_TANK_INLET_SOLENOID_VALVE, 6);  // Read 12 coils starting from address 17
			for (int i = 0; i < 6; i++)
			{
				ModbusCoil[i+HOT_WATER_TANK_INLET_SOLENOID_VALVE] = bitRead(ModbusDATARX[46], i);  // Extract each coil state
			}
		} while (ModbusCoil[MAIN_INLET_SOLENOID_VALVE]); // repeats if still dispensing
	}
}

void sendCheckSetTemp(dispenseAddressValuePair *command) {
	int tempAddress;
	do {
		WriteSingleRegister(command->address, command->buffer[0]);
		osDelay(2);
		ReadModbusData(3, ADD_SET_HOT_WATER_TEMPERATURE, 2); //confirms changes
		if(command->address == ADD_SET_HOT_WATER_TEMPERATURE) tempAddress = SET_HOT_WATER_TEMPERATURE;
		else tempAddress = SET_COLD_WATER_TEMPERATURE;
	} while (ModbusDATARX[tempAddress] != command->buffer[0]); // repeats if change not propagated
}

/*
 * MIDI AUDIO SOUNDS
 */
void MIDI_NoteOn(uint8_t channel, uint8_t note, uint8_t velocity)
{
    uint8_t msg[3];
    msg[0] = 0x90 | (channel & 0x0F); // Note On message
    msg[1] = note & 0x7F;             // Note number (0-127)
    msg[2] = velocity & 0x7F;         // Velocity (0-127)
    HAL_UART_Transmit(&huart1, msg, 3, HAL_MAX_DELAY);
}

void MIDI_NoteOff(uint8_t channel, uint8_t note, uint8_t velocity)
{
    uint8_t msg[3];
    msg[0] = 0x80 | (channel & 0x0F); // Note Off message
    msg[1] = note & 0x7F;             // Note number (0-127)
    msg[2] = velocity & 0x7F;         // Velocity (usually 0)
    HAL_UART_Transmit(&huart1, msg, 3, HAL_MAX_DELAY);
}

void MIDI_ProgramChange(uint8_t channel, uint8_t program)
{
    uint8_t msg[2];
    msg[0] = 0xC0 | (channel & 0x0F); // Program Change message
    msg[1] = program & 0x7F;          // Program number (0-127)
    HAL_UART_Transmit(&huart1, msg, 2, HAL_MAX_DELAY);
}

/*
 * MODBUS FUNCTIONS
 */
// Mapping function to return the index for the active Modbus addresses
uint16_t* GetModbusDataPointer(uint16_t startAddress) {
    // Map relevant Coil addresses
    if (startAddress == ADD_HOT_WATER_RESET) {
        return &ModbusDATARX[45];  // to be bitread later
    }
    else if (startAddress == ADD_HOT_WATER_TANK_INLET_SOLENOID_VALVE) {
        return &ModbusDATARX[46];  // to be bitread later
    }
    else if (startAddress == ADD_WATER_FLOW_SWITCH) {
        return &ModbusDATARX[47];  // to be bitread later
    }
    else if (startAddress == ADD_CUP_DETECTION_1) {
		return &ModbusDATARX[48];  // to be bitread later
	}
	else if (startAddress == ADD_UV_ERROR) {
		return &ModbusDATARX[49];  // to be bitread later
	}
    // Map Input Registers (30001-30023)
    else if (startAddress >= 30001 && startAddress <= 30023) {
        return &ModbusDATARX[startAddress - 30001 + 0];  // Map to indices 0-22
    }
    // For holding registers (addresses 40001–40022)
	else if (startAddress >= 40001 && startAddress <= 40022) {
		return &ModbusDATARX[startAddress - 40001 + 23];  // Offset to fit after input registers (index 23+)
	}
    // If address is not in the range, return NULL or handle error
    return NULL;  // Error: address not supported
}

// Function to read Modbus data based on function code, start address, and number of registers or coils
void ReadModbusData(uint8_t functionCode, uint16_t startAddress, uint16_t numRegisters) {
    modbus_t telegram;

    // Get pointer to the appropriate location in ModbusDATARX[] for this start address
	uint16_t* dataPtr = GetModbusDataPointer(startAddress);

	if (dataPtr == NULL) {
		// Handle error: address is not supported
		__NOP(); //printf("Invalid Modbus start address: %d\n", startAddress);
		return;
	}

    // Fill the telegram structure based on the function code
    telegram.u8id = 1;             					// Modbus slave ID
    telegram.u8fct = functionCode;                  // Function code (1, 3, or 4)
    telegram.u16RegAdd = startAddress;              // Starting address
    telegram.u16CoilsNo = numRegisters;             // Number of coils/registers to read
    telegram.u16reg = dataPtr;  					// Pointer to the relevant section of ModbusDATARX

    // Send query to read coils, holding registers, or input registers
    ModbusQuery(&ModbusHandler, telegram);

    // Wait for response (or handle asynchronously in your system)
    uint32_t result = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait indefinitely for the task notification

    if (result) {
        // Error occurred
        __NOP(); //printf("Modbus read error for function code %d, error code: %d\n", functionCode, result);
    }
    osDelay(10);
}

void ReadCoilsInChunks() {
	uint16_t count;
    // Read chunk 1: Coils from address 1 to 7
	count = abs(ADD_HOT_WATER_RESET - ADD_HEATING_SWITCH) + 1;
    ReadModbusData(1, ADD_HOT_WATER_RESET, count);  // Read 7 coils starting from address 1
    for (int i = 0; i < count; i++)
	{
		ModbusCoil[i+HOT_WATER_RESET] = bitRead(ModbusDATARX[45], i);  // Extract each coil state
	}
    // Read chunk 2: Coils from address 17 to 22
    count = abs(ADD_HOT_WATER_TANK_INLET_SOLENOID_VALVE - ADD_UV_LIGHT) + 1;
    ReadModbusData(1, ADD_HOT_WATER_TANK_INLET_SOLENOID_VALVE, count);  // Read 12 coils starting from address 17
    for (int i = 0; i < count; i++)
	{
		ModbusCoil[i+HOT_WATER_TANK_INLET_SOLENOID_VALVE] = bitRead(ModbusDATARX[46], i);  // Extract each coil state
	}
    // Read chunk 3: Coils from address 23 to 28
    count = abs(ADD_WATER_FLOW_SWITCH - ADD_INSTANT_HEATING_OUTPUT) + 1;
    ReadModbusData(1, ADD_WATER_FLOW_SWITCH, count);  // Read 12 coils starting from address 17
	for (int i = 0; i < count; i++)
	{
		ModbusCoil[i+WATER_FLOW_SWITCH] = bitRead(ModbusDATARX[47], i);  // Extract each coil state
	}
    // Read chunk 4: Coils from address 31 to 35
	count = 6;
    ReadModbusData(1, ADD_CUP_DETECTION_1, count);  // Read 13 coils starting from address 31
    for (int i = 0; i < count; i++)
	{
		ModbusCoil[i+CUP_DETECTION_1] = bitRead(ModbusDATARX[48], i);  // Extract each coil state
	}
    // Read chunk 5: Coils from address 37 to 43
    count =  6;
    ReadModbusData(1, ADD_UV_ERROR, count);  // Read 13 coils starting from address 31
	for (int i = 0; i < count; i++)
	{
		ModbusCoil[i+UV_ERROR] = bitRead(ModbusDATARX[49], i);  // Extract each coil state
	}
}

// Read input registers (function code 4)
void ReadInputRegistersInChunks() {
	uint16_t count;
	// Read chunk 1: registers from address 0 to 9
	count = abs(ADD_CARD_NUMBERBYTE_1_BYTE_2 - ADD_FILTER_SNBYTE_13_BYTE_14) + 1;
    ReadModbusData(4, ADD_CARD_NUMBERBYTE_1_BYTE_2, count);  // Reading addresses 30001-30009
    // Read chunk 2: registers from address 10 to 16
    count = abs(ADD_REMAINED_FILTER_CAPACITYL - ADD_REMAINING_FLOW_RATE_ML) + 1;
    ReadModbusData(4, ADD_REMAINED_FILTER_CAPACITYL, count);  // Reading addresses 30010-30018
    // Read chunk 3: registers from address 17 to 22
    count = abs(ADD_TOTAL_WATER_OUTPUT_BYTE_1_BYTE_2_L - ADD_CURRENT_FLOW_RATE) + 1;
	ReadModbusData(4, ADD_TOTAL_WATER_OUTPUT_BYTE_1_BYTE_2_L, count);  // Reading addresses 30018-30023
}

// Read holding registers (function code 3)
void ReadHoldingRegistersInChunks() {
	uint16_t count;
	// Read chunk 1: registers from address 23 to 30
	count = abs(ADD_DISPENSE_HOT_WATER - ADD_SET_MIXED_WATER_TEMPERATURE) + 1;
    ReadModbusData(3, ADD_DISPENSE_HOT_WATER, count);  // Reading addresses 40001-40008
	// Read chunk 2: registers from address 31 to 37
    count = abs(ADD_CLEANING - ADD_CUP_DETECTION_SENSITIVITY) + 1;
	ReadModbusData(3, ADD_CLEANING, count);  // Reading addresses 40009-40015
	// Read chunk 3: registers from address 38 to 44
	count = abs(ADD_DRY_BURN_DETECTION_ENABLE - ADD_FILTER_REFRESH) + 1;
	ReadModbusData(3, ADD_DRY_BURN_DETECTION_ENABLE, count);  // Reading addresses 40016-40019
}

// Function to write to a single register on the slave (Function Code 6)
void WriteSingleRegister(uint16_t startAddress, uint16_t value) {
    modbus_t telegram;
    // Ensure the address is within the writable range (40001-40022)
    if (startAddress < 40001 || startAddress > 40022) {
        __NOP(); //printf("Error: Invalid register address %d\n", startAddress);
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
    if (result) {
       __NOP();//printf("Modbus write error for register address %d, error code: %d\n", startAddress, result);
    }
    osDelay(5);
}

// Function to write to multiple registers on the slave (Function Code 16)
void WriteMultipleRegisters(uint16_t startAddress, uint16_t* values, uint16_t numRegisters) {
    modbus_t telegram;
    // Ensure the address and number of registers are within the writable range
    if (startAddress < 40001 || (startAddress + numRegisters - 1) > 40022) {
    	__NOP(); //printf("Error: Invalid register address or range (start: %d, count: %d)\n", startAddress, numRegisters);
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
    if (result) {
    	__NOP(); //printf("Modbus write error for register range starting at address %d, error code: %d\n", startAddress, result);
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

