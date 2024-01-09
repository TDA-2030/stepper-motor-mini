
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

void adc_config(void);
float AdcGetChipTemperature();


#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
