#include "tb67h450_stm32.h"
#include "tim.h"

void TB67H450::InitGpio()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Signal Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure Signal pin Output Level */
    HAL_GPIO_WritePin(HW_ELEC_BM_GPIO_Port, HW_ELEC_BM_Pin | HW_ELEC_BP_Pin | HW_ELEC_AM_Pin | HW_ELEC_AP_Pin,
                      GPIO_PIN_RESET);

    /*Configure Signal pins : PAPin PAPin PAPin PAPin */
    GPIO_InitStruct.Pin = HW_ELEC_BM_Pin | HW_ELEC_BP_Pin | HW_ELEC_AM_Pin | HW_ELEC_AP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(HW_ELEC_BM_GPIO_Port, &GPIO_InitStruct);
}


void TB67H450::InitPwm()
{
    MX_TIM2_Init();
}


void TB67H450::DacOutputVoltage(uint16_t _voltageA_3300mVIn12bits, uint16_t _voltageB_3300mVIn12bits)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, _voltageA_3300mVIn12bits >> 2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, _voltageB_3300mVIn12bits >> 2);
}


void TB67H450::SetInputA(bool _statusAp, bool _statusAm)
{
    _statusAp ? (GPIOA->BSRR = HW_ELEC_AP_Pin) : (GPIOA->BRR = HW_ELEC_AP_Pin);
    _statusAm ? (GPIOA->BSRR = HW_ELEC_AM_Pin) : (GPIOA->BRR = HW_ELEC_AM_Pin);
}


void TB67H450::SetInputB(bool _statusBp, bool _statusBm)
{
    _statusBp ? (GPIOA->BSRR = HW_ELEC_BP_Pin) : (GPIOA->BRR = HW_ELEC_BP_Pin);
    _statusBm ? (GPIOA->BSRR = HW_ELEC_BM_Pin) : (GPIOA->BRR = HW_ELEC_BM_Pin);
}
