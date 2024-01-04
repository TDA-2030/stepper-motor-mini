/**************************************************************************//**
  \file     timer.c
  \brief    this file contains the code implementation of timer initialization
            and disable timer function and setting timer comparison value.
  \author   Lao·Zhu
  \version  V1.0.2
  \date     29. October 2021
 ******************************************************************************/

#include "tim.h"
#include "gd32f1x0.h"


/*!
    \brief disable timer1 periph and timer1 interrupt
*/
void timer1_disable(void) {
    /* stop TIMER1 and deinit */
    timer_disable(TIMER1);
    timer_deinit(TIMER1);

    /* disable TIMER1 update interrupt and clock */
    timer_interrupt_disable(TIMER1, TIMER_INT_UP);
    nvic_irq_disable(TIMER1_IRQn);
    rcu_periph_clock_disable(RCU_TIMER1);
}

/*!
    \brief configure timer1 periph for timing interrupt
*/
void timer1_config(void) {
    timer_parameter_struct timer_initpara;

    /* enable TIMER1 clock*/
    rcu_periph_clock_enable(RCU_TIMER1);
    timer_deinit(TIMER1);

    /* TIMER1CLK = SystemCoreClock / 72 = 1MHz */
    timer_initpara.prescaler = 71;

    /* timer edge alignment up count mode */
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;

    /* Period = 1000 / TIM2_FREQUENCY (kHz) */
    timer_initpara.period = 9999;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    
}

void timer1_start()
{
    /* enable TIMER1 update interrupt */
    timer_interrupt_enable(TIMER1, TIMER_INT_UP);
    timer_enable(TIMER1);
    nvic_irq_enable(TIMER1_IRQn, 4, 0);
}

/*!
    \brief configure timer13 periph for timing interrupt
*/
void timer13_config(void) {
    timer_parameter_struct timer_initpara;

    /* enable TIMER13 clock*/
    rcu_periph_clock_enable(RCU_TIMER13);
    timer_deinit(TIMER13);

    /* TIMER1CLK = SystemCoreClock / 72 = 1MHz */
    timer_initpara.prescaler = 71;

    /* timer edge alignment up count mode */
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;

    /* Period = 1000000 / TIM13_FREQUENCY (kHz) */
    timer_initpara.period = 49;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER13, &timer_initpara);
}

void timer13_start()
{
    /* enable TIMER13 update interrupt */
    timer_interrupt_enable(TIMER13, TIMER_INT_UP);
    timer_enable(TIMER13);
    nvic_irq_enable(TIMER13_IRQn, 0, 0);
}

/*!
    \brief disable timer13 periph and timer2 interrupt
*/
void timer13_disable(void) {
    /* stop TIMER13 and deinit */
    timer_disable(TIMER13);
    timer_deinit(TIMER13);

    /* disable TIMER13 update interrupt and clock*/
    timer_interrupt_disable(TIMER13, TIMER_INT_UP);
    nvic_irq_disable(TIMER13_IRQn);
    rcu_periph_clock_disable(RCU_TIMER13);
}

/*!
    \brief configure timer2 periph and its gpios
*/
void pwm_timer2_config(void) {
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    /* enable GPIO clock and TIMER2 clock*/
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_TIMER2);
    timer_deinit(TIMER2);

    /* configure TIMER2_CH2 as alternate function push-pull */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);

    /* configure TIMER2_CH3 as alternate function push-pull */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

    /* connect port to TIMER2_CH2 TIMER2_CH3 */
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_0);
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_1);

    /* TIMER2CLK = SystemCoreClock / 1 = 36MHz */
    timer_initpara.prescaler = 0;

    /* timer center alignment up count mode */
    timer_initpara.alignedmode = TIMER_COUNTER_CENTER_UP;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;

    /* Period = 36000 / PWM_FREQUENCY */
    timer_initpara.period = 1023;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2, &timer_initpara);

    /* CH1, CH2 and CH3 configuration in PWM0 mode */
    timer_ocintpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_channel_output_config(TIMER2, TIMER_CH_2, &timer_ocintpara);
    timer_channel_output_config(TIMER2, TIMER_CH_3, &timer_ocintpara);

    /* CH2 configuration in PWM mode1,duty cycle 0% */
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_2, 0);
    timer_channel_output_mode_config(TIMER2, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

    /* CH3 configuration in PWM mode1,duty cycle 0% */
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_3, 0);
    timer_channel_output_mode_config(TIMER2, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);

    /* auto-reload preload and timer enable */
    timer_auto_reload_shadow_enable(TIMER2);
    timer_enable(TIMER2);
}

/*!
    \brief     update timer2 duty-cycle
    \param[in] cha: duty-cycle of channel0, 0 ~ 1023
    \param[in] chb: duty-cycle of channel1, 0 ~ 1023
*/
void pwm_timer2_set_dutycycle(uint32_t cha, uint32_t chb) {
    /* update the comparison register of timer1 */
    TIMER_CH2CV(TIMER2) = cha;
    TIMER_CH3CV(TIMER2) = chb;
}