/**************************************************************************//**
  \file     uart.c
  \brief    this file contains the code implementation of UART interface
            initialization function and medium capacity transmission protocol
            transceiver function.
  \author   Lao·Zhu
  \version  V1.0.2
  \date     29. October 2021
 ******************************************************************************/

#include "main.h"
#include "gd32f1x0.h"
#include "common_inc.h"
#include "Platform/retarget.h"
#include "usart.h"
// #define USART0_TDATA_ADDRESS      ((uint32_t)0x40013828)
// #define USART0_RDATA_ADDRESS      ((uint32_t)0x40013824)

uint8_t g_rx_buffer[UART_TR_BUFFER_SIZE] = {0};
void (* OnRecvEnd)(uint8_t* data, uint16_t len);

/*!
    \brief configure uart0 periph and its gpios
*/
void uart_config(void) {
    /* enable GPIO clock and UART clock*/
    rcu_periph_clock_enable(RCU_DMA);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to UARTx */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);

    /* configure UART Tx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_9);

    /* configure UART Rx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_10);

    /* UART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);

    /* enable UART RBNE interrupt */
    usart_flag_clear(USART0, USART_FLAG_IDLE);
    usart_interrupt_enable(USART0, USART_INT_IDLE);

    /* deinitialize DMA channel1 */
    dma_deinit(DMA_CH1);
    dma_parameter_struct dma_init_struct;
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = (uint32_t)0;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = UART_TR_BUFFER_SIZE;
    dma_init_struct.periph_addr = (uint32_t)(&USART_TDATA(USART0));
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_init(DMA_CH1, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA_CH1);
    dma_memory_to_memory_disable(DMA_CH1);
    /* enable DMA transfer complete interrupt */
    dma_interrupt_enable(DMA_CH1, DMA_CHXCTL_FTFIE);
    /* enable DMA channel1 */
    // dma_channel_enable(DMA_CH1);

    /* deinitialize DMA channel2 */
    dma_deinit(DMA_CH2);
    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)g_rx_buffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = UART_TR_BUFFER_SIZE;
    dma_init_struct.periph_addr = (uint32_t)(&USART_RDATA(USART0));
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_init(DMA_CH2, &dma_init_struct);
    dma_circulation_disable(DMA_CH2);
    dma_memory_to_memory_disable(DMA_CH2);
    dma_channel_enable(DMA_CH2);

    /* USART DMA enable for transmission and reception */
    usart_dma_receive_config(USART0, USART_DENR_ENABLE);
    usart_dma_transmit_config(USART0, USART_DENT_ENABLE);
     /* UART interrupt configuration */
    nvic_irq_enable(USART0_IRQn, 3, 0);

    nvic_irq_enable(DMA_Channel1_2_IRQn,2,0);

    usart_enable(USART0);

    Uart_SetRxCpltCallBack(OnUartCmd);
}

void Uart_SetRxCpltCallBack(void(* xerc)(uint8_t*, uint16_t))
{
  OnRecvEnd = xerc;
}

void uart_send_dma(const uint8_t *data, uint32_t len)
{
    dma_transfer_number_config(DMA_CH1, len);
    dma_memory_address_config(DMA_CH1, (uint32_t)data);
    dma_channel_enable(DMA_CH1);
}