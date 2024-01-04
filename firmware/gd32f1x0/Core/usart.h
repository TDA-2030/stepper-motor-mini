/**************************************************************************//**
  \file     uart.h
  \brief    this is the header file of uart.c.
  \author   Lao·Zhu
  \version  V1.0.1
  \date     10. October 2021
 ******************************************************************************/

#ifndef MINIFOC_HARDWARE_UART_H_
#define MINIFOC_HARDWARE_UART_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UART_TR_BUFFER_SIZE 32

typedef struct
{
  /* data */
}UART_HandleTypeDef;


extern uint8_t g_rx_buffer[UART_TR_BUFFER_SIZE];
extern void (*OnRecvEnd)(uint8_t *data, uint16_t len);
void Uart_SetRxCpltCallBack(void(*xerc)(uint8_t *, uint16_t));
void uart_send_dma(const uint8_t *data, uint32_t len);


#ifdef __cplusplus
}
#endif

#endif // MINIFOC_HARDWARE_UART_H_
