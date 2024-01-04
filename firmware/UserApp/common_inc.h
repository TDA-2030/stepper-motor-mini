#ifndef COMMON_INC_H_
#define COMMON_INC_H_

#ifdef __cplusplus
extern "C" {
#endif
/*---------------------------- C Scope ---------------------------*/
#include "stdint-gcc.h"

void Main();
void OnUartCmd(uint8_t* _data, uint16_t _len);
void OnCanCmd(uint8_t _cmd, uint8_t* _data, uint32_t _len);

#ifdef __cplusplus
}
/*---------------------------- C++ Scope ---------------------------*/

#include <cstdio>
#include "Motor/motor.h"

#ifdef GD32F130_150
#include "mt6816_gd32.h"
#include "tb67h450_gd32.h"
#include "encoder_calibrator_gd32.h"
#include "button_gd32.h"
#include "led_gd32.h"
#elif defined(STM32F103xB)
#include "mt6816_stm32.h"
#include "tb67h450_stm32.h"
#include "encoder_calibrator_stm32.h"
#include "button_stm32.h"
#include "led_stm32.h"
#endif

#include "adc.h"

#endif

#endif // end of COMMON_INC_H_
