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


/* ----------------------- Modbus includes ----------------------------------*/
#include "port.h"
#include "mb.h"
#include "mbport.h"

typedef struct {
    uint32_t counter_50us;
    uint32_t period;
    uint8_t enable;
} virtimer_t;

static virtimer_t virtimer = {0};

static void virtual_timer_init(void)
{
    virtimer.counter_50us = 0;
    virtimer.period = 0;
    virtimer.enable = 0;
}

void virtual_timer_tick_50us(void)
{
    if (!virtimer.enable) {
        return;
    }

    virtimer.counter_50us += 1;
    if (virtimer.counter_50us > virtimer.period) {
        ( void )pxMBPortCBTimerExpired(  );
        virtimer.enable = 0;
    }

}


/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
    virtual_timer_init();
    virtimer.period = usTim1Timerout50us;
    return TRUE;
}

void
vMBPortTimerClose( void )
{
    virtimer.enable = 0;
}

void
vMBPortTimersEnable(  )
{
    virtimer.counter_50us = 0;
    virtimer.enable = 1;
}

void
vMBPortTimersDisable(  )
{
    virtimer.enable = 0;
}

void
vMBPortTimersDelay( USHORT usTimeOutMS )
{
    
}

