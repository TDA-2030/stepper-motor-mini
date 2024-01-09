#include "main.h"
#include "tb67h450_gd32.h"
#include "tim.h"

void TB67H450::InitGpio()
{
    /* enable GPIO clock and TIMER2 clock*/
    rcu_periph_clock_enable(RCU_GPIOA);

    gpio_mode_set(HW_ELEC_BM_GPIO_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HW_ELEC_BM_Pin | HW_ELEC_BP_Pin | HW_ELEC_AM_Pin | HW_ELEC_AP_Pin);
    gpio_output_options_set(HW_ELEC_BM_GPIO_Port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, HW_ELEC_BM_Pin | HW_ELEC_BP_Pin | HW_ELEC_AM_Pin | HW_ELEC_AP_Pin);
    
    /*Configure Signal pin Output Level */
    gpio_bit_reset(HW_ELEC_BM_GPIO_Port, HW_ELEC_BM_Pin | HW_ELEC_BP_Pin | HW_ELEC_AM_Pin | HW_ELEC_AP_Pin);
}


void TB67H450::InitPwm()
{
    
}


void TB67H450::DacOutputVoltage(uint16_t _voltageA_3300mVIn12bits, uint16_t _voltageB_3300mVIn12bits)
{
    pwm_timer2_set_dutycycle(_voltageB_3300mVIn12bits >> 2, _voltageA_3300mVIn12bits >> 2);
}


void TB67H450::SetInputA(bool _statusAp, bool _statusAm)
{
    gpio_bit_write(HW_ELEC_AP_GPIO_Port, HW_ELEC_AP_Pin, (bit_status)_statusAp);
    gpio_bit_write(HW_ELEC_AM_GPIO_Port, HW_ELEC_AM_Pin, (bit_status)_statusAm);
}


void TB67H450::SetInputB(bool _statusBp, bool _statusBm)
{
    gpio_bit_write(HW_ELEC_BP_GPIO_Port, HW_ELEC_BP_Pin, (bit_status)_statusBp);
    gpio_bit_write(HW_ELEC_BM_GPIO_Port, HW_ELEC_BM_Pin, (bit_status)_statusBm);
}
