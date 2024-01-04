/**************************************************************************//**
  \file     timer.h
  \brief    this is the header file of timer.c.
  \author   Lao·Zhu
  \version  V1.0.1
  \date     10. October 2021
 ******************************************************************************/

#ifndef MINIFOC_HARDWARE_TIMER_H_
#define MINIFOC_HARDWARE_TIMER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void timer13_config(void);
void timer13_disable(void);
void timer13_start(void);

void timer1_config(void);
void timer1_start(void);
void timer1_disable(void);

void pwm_timer2_config(void);
void pwm_timer2_set_dutycycle(uint32_t cha, uint32_t chb);

#ifdef __cplusplus
}
#endif

#endif //MINIFOC_HARDWARE_TIMER_H_
