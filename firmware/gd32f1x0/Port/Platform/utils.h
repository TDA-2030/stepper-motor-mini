#ifndef CTRL_STEP_FW_ST_HARDWARE_H
#define CTRL_STEP_FW_ST_HARDWARE_H

#include <stdint-gcc.h>

#ifdef __cplusplus
extern "C" {
#endif

uint64_t GetSerialNumber();

#define HAL_NVIC_SystemReset NVIC_SystemReset
#define HAL_Delay delay_1ms

#ifdef __cplusplus
}


#endif
#endif
