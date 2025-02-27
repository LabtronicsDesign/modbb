#ifndef BIBOBUTTONCONTROLLER_HPP_
#define BIBOBUTTONCONTROLLER_HPP_


//#include <platform/driver/button/ButtonController.hpp>
#include "main.h"

 extern "C" {
 	extern uint8_t User_ButtonState;
 }

const uint8_t effects[] = {1,4,7,10,12,14,15,16,17,21,24,27,31,34,37,41,44,47,52,54,56,58,64,118};


void DRV2605L_SetWaveform(uint8_t slot, uint8_t effect);
void DRV2605L_Start(void);
void DRV2605L_Stop(void);
void hapticResponse();
void hapticScrollResponse();
void NoteOn(uint8_t channel, uint8_t note, uint8_t velocity);
void NoteOff(uint8_t channel, uint8_t note, uint8_t velocity);
void ProgramChange(uint8_t channel, uint8_t program);
void audioClickResponse();


//class BIBOButtonController: public touchgfx:: ButtonController
//{
//	public:
//		virtual void init();
//		virtual bool sample(uint8_t& key);
//	private:
//		uint8_t previousState;
//
//};

#endif /* BIBOBUTTONCONTROLLER_HPP_*/
