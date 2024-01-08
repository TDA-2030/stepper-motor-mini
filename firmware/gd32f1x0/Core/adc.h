
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

void adc_config(void);
float AdcGetChipTemperature();

extern uint16_t whole_adc_data[2][12];


#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
