/*
 * FreeModbus Libary: ARM7 Demo Application
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */


/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"

/* ----------------------- Defines ------------------------------------------*/
#define REG_HOLDING_START           0x1000
#define REG_HOLDING_NREGS           130

/* ----------------------- Static variables ---------------------------------*/
static uint16_t   usRegHoldingStart = REG_HOLDING_START;
static uint16_t   usRegHoldingBuf[REG_HOLDING_NREGS];

void modbus_init(uint8_t id)
{
    id = 0x0a;
    /* Select either ASCII or RTU Mode. */
    ( void )eMBInit( MB_RTU, id, 0, 115200, MB_PAR_NONE );

    /* Initialize the holding register values before starting the
     * Modbus stack
     */
    for (int i = 0; i < REG_HOLDING_NREGS; i++ ) {
        usRegHoldingBuf[i] = ( unsigned short )i;
    }

    /* Enable the Modbus Protocol Stack. */
    ( void )eMBEnable(  );
}

void modbus_poll(void)
{
    /* Call the main polling loop of the Modbus protocol stack. */
        ( void )eMBPoll(  );
}


eMBErrorCode
eMBRegHoldingCB( UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
    if ( ( usAddress >= REG_HOLDING_START ) &&
            ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) ) {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode ) {
        /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
            while ( usNRegs > 0 ) {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

        /* Update current register values with new values from the
         * protocol stack. */
        case MB_REG_WRITE:
            while ( usNRegs > 0 ) {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
        }
    } else {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

