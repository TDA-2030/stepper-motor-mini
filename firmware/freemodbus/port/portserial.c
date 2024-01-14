/*
 * FreeModbus Libary: Atmel AT91SAM3S Demo Application
 * Copyright (C) 2010 Christian Walter <cwalter@embedded-solutions.at>
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id$
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>


/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "port.h"
#include "mbport.h"
#include "usart.h"
#include "gpio.h"
#include "main.h"

void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable)
{
#ifdef GD32F130_150
    if ( xRxEnable ) {
        gpio_rs485_enable_send(FALSE);
        usart_interrupt_flag_clear(USART0, USART_INT_FLAG_RBNE);
        usart_interrupt_enable(USART0, USART_INT_RBNE);
    } else {
        usart_interrupt_disable(USART0, USART_INT_RBNE);
    }

    if ( xTxEnable ) {
        gpio_rs485_enable_send(TRUE);
        usart_interrupt_flag_clear(USART0, USART_INT_FLAG_TC);
        usart_interrupt_enable(USART0, USART_INT_TC);
    } else {
        gpio_rs485_enable_send(FALSE);
        usart_interrupt_disable(USART0, USART_INT_TC);
    }
#elif defined(STM32F103xB)

    if ( xRxEnable ) {
        gpio_rs485_enable_send(FALSE);
        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

    } else {
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    }

    if ( xTxEnable ) {
        gpio_rs485_enable_send(TRUE);
        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
    } else {
        gpio_rs485_enable_send(FALSE);
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_TC);
    }
#endif
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{

    return TRUE;
}

void
vMBPortSerialClose( void )
{

}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
#ifdef GD32F130_150
    usart_data_transmit(USART0, (uint8_t)ucByte);
#elif defined(STM32F103xB)
    huart1.Instance->DR = ucByte;
#endif

    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR *pucByte )
{
#ifdef GD32F130_150
    *pucByte = (CHAR)usart_data_receive(USART0);
#elif defined(STM32F103xB)
    *pucByte = (CHAR)huart1.Instance->DR;
#endif

    return TRUE;
}


