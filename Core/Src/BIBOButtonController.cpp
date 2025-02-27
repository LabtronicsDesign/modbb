/*
 * ButtonController.cpp
 *
 *  Created on: Nov 1, 2024
 *      Author: labibl
 */

#include <BIBOButtonController.hpp>
#include <main.h>
//#include <touchgfx/hal/HAL.hpp>

//void BIBOButtonController::init()
//{
////	previousState = 0xFF;
//
//}
//
//bool BIBOButtonController::sample(uint8_t &key){
//	if(User_ButtonState){
//		User_ButtonState = 0x00;
//		key = 0;
//		return true;
//	}
//	return false;
//}

/*
 * HAPTIC Drive
 */
// Function to set a waveform effect
void DRV2605L_SetWaveform(uint8_t slot, uint8_t effect)
{
    // Each slot holds a specific waveform (max 8 slots), we use 2 slots here (WAVESEQ1, WAVESEQ2)
    uint8_t reg = DRV2605L_REG_WAVESEQ1 + slot;
    uint8_t go_data[2] = {reg, effect};
    HAL_I2C_Master_Transmit(&hi2c1, DRV2605L_I2C_ADDR << 1, go_data, 2, HAL_MAX_DELAY);
}

// Function to start haptic effect
void DRV2605L_Start(void)
{
    uint8_t go_data[2] = {DRV2605L_REG_GO, 1}; // Set GO bit to 1
    HAL_I2C_Master_Transmit(&hi2c1, DRV2605L_I2C_ADDR << 1, go_data, 2, HAL_MAX_DELAY);
}

// Function to stop haptic effect
void DRV2605L_Stop(void)
{
    uint8_t go_data[2] = {DRV2605L_REG_GO, 0}; // Set GO bit to 0
    HAL_I2C_Master_Transmit(&hi2c1, DRV2605L_I2C_ADDR << 1, go_data, 2, HAL_MAX_DELAY);
}

void hapticResponse(){
	// haptic action when the screen is touched
//	DRV2605L_SetWaveform(0, effects[5]); // Set the first slot to the desired effect
	DRV2605L_SetWaveform(0, effects[5]); // Set the first slot to the desired effect
	DRV2605L_Start();
}

void hapticScrollResponse(){
	// haptic action when the screen is touched
	DRV2605L_SetWaveform(0, effects[1]); // Set the first slot to the desired effect
	DRV2605L_Start();
}

/*
 * MIDI AUDIO SOUNDS
 */
void NoteOn(uint8_t channel, uint8_t note, uint8_t velocity)
{
    uint8_t msg[3];
    msg[0] = 0x90 | (channel & 0x0F); // Note On message
    msg[1] = note & 0x7F;             // Note number (0-127)
    msg[2] = velocity & 0x7F;         // Velocity (0-127)
    HAL_UART_Transmit(&huart1, msg, 3, HAL_MAX_DELAY);
}

void NoteOff(uint8_t channel, uint8_t note, uint8_t velocity)
{
    uint8_t msg[3];
    msg[0] = 0x80 | (channel & 0x0F); // Note Off message
    msg[1] = note & 0x7F;             // Note number (0-127)
    msg[2] = velocity & 0x7F;         // Velocity (usually 0)
    HAL_UART_Transmit(&huart1, msg, 3, HAL_MAX_DELAY);
}

void ProgramChange(uint8_t channel, uint8_t program)
{
    uint8_t msg[2];
    msg[0] = 0xC0 | (channel & 0x0F); // Program Change message
    msg[1] = program & 0x7F;          // Program number (0-127)
    HAL_UART_Transmit(&huart1, msg, 2, HAL_MAX_DELAY);
}

void audioClickResponse(){
	ProgramChange(0, 4);
	NoteOn(0, 110, 127); //playing
}

/*
#include <BIBOButtonController.hpp>
hapticResponse();
hapticScrollResponse();
audioClickResponse();
audioSequence1Response();
audioSequence2Response();

 */
