/**************************************************************************//**
  \file     gd32f1x0_it.c
  \brief    gd32f1x0 interrupt handler function source file
  \author   Lao·Zhu
  \version  V1.0.2
  \date     29. October 2021
 ******************************************************************************/
#include <string.h>
#include "gd32f1x0_it.h"
#include "main.h"
#include "systick.h"
#include "usart.h"


extern void GpioPin7InterruptCallback();
extern void Tim1Callback100Hz();
extern void Tim3CaptureCallback();
extern void Tim4Callback20kHz();
extern void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

/*!
    \brief this function handles NMI exception
*/
void NMI_Handler(void) {
}

/*!
    \brief this function handles HardFault exception
*/
void HardFault_Handler(void) {
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief this function handles MemManage exception
*/
void MemManage_Handler(void) {
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief this function handles BusFault exception
*/
void BusFault_Handler(void) {
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief this function handles UsageFault exception
*/
void UsageFault_Handler(void) {
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief this function handles SVC exception
*/
void SVC_Handler(void) {
}

/*!
    \brief this function handles DebugMon exception
*/
void DebugMon_Handler(void) {
}

/*!
    \brief this function handles PendSV exception
*/
void PendSV_Handler(void) {
}

/*!
    \brief this function handles SysTick exception
*/
void SysTick_Handler(void)
{
    delay_decrement();
}

/*!
    \brief this function handles USART RBNE interrupt request
*/
void USART0_IRQHandler(void)
{
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE)){
        /* clear IDLE flag */
        usart_interrupt_flag_clear(USART0, USART_INT_FLAG_IDLE);
        
        /* number of data received */
        volatile uint8_t rxLen = UART_TR_BUFFER_SIZE - (dma_transfer_number_get(DMA_CH2));
        OnRecvEnd(g_rx_buffer, rxLen);
        memset(g_rx_buffer, 0, rxLen);
        
        /* disable DMA and reconfigure */
        dma_channel_disable(DMA_CH2);
        dma_transfer_number_config(DMA_CH2, UART_TR_BUFFER_SIZE);
        dma_channel_enable(DMA_CH2);
    }
}

void DMA_Channel1_2_IRQHandler(void)
{
    if(dma_interrupt_flag_get(DMA_CH1, DMA_INT_FLAG_FTF)){
        HAL_UART_TxCpltCallback(0);
        dma_interrupt_flag_clear(DMA_CH1, DMA_INT_FLAG_G);
        dma_channel_disable(DMA_CH1);
    }
}

/*!
    \brief this function handles TIMER1 TIMER_INT_UP interrupt request
*/
void TIMER1_IRQHandler(void) {
    /* judge whether a timer update interrupt is generated, clear timer interrupt flag bit */
    if (SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_UP)) {
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_UP);

        Tim1Callback100Hz();
        
    }
}

/*!
    \brief this function handles TIMER13 TIMER_INT_UP interrupt request
*/
void TIMER13_IRQHandler(void) {
    /* judge whether a timer update interrupt is generated, clear timer interrupt flag bit */
    if (SET == timer_interrupt_flag_get(TIMER13, TIMER_INT_UP)) {
        timer_interrupt_flag_clear(TIMER13, TIMER_INT_UP);

        Tim4Callback20kHz();
       
    }
}
