/*
 * FreeModbus Libary: ARM7/AT91SAM7X Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

#ifndef _MB_PORT_H__
#define _MB_PORT_H__

#include <stdint.h>
#include <assert.h>


#ifdef _cplusplus
extern          "C"
{
#endif

/* ----------------------- Defines ------------------------------------------*/

#define INLINE                         inline
#define STATIC                         static

#define PR_BEGIN_EXTERN_C              extern "C" {
#define	PR_END_EXTERN_C                }

#define MB_PORT_HAS_CLOSE              0
#define ENTER_CRITICAL_SECTION( )     
#define EXIT_CRITICAL_SECTION( )       

#ifndef TRUE
#define TRUE                           ( BOOL )1
#endif

#ifndef FALSE
#define FALSE                          ( BOOL )0
#endif

/* ----------------------- Type definitions ---------------------------------*/
    typedef char    BOOL;

    typedef signed char BYTE;
    typedef unsigned char UBYTE;

    typedef unsigned char UCHAR;
    typedef char    CHAR;

    typedef unsigned short USHORT;
    typedef short   SHORT;

    typedef unsigned long ULONG;
    typedef long    LONG;

/* ----------------------- Function prototypes ------------------------------*/
    void            vMBPInit( void );

#ifdef _cplusplus
}
#endif

#endif
