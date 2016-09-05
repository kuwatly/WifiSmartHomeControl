/*********************************************************************
 *
 *	Hardware specific definitions for the WiFi Comm Demo Board
 *
 *********************************************************************
 * FileName:        HardwareProfile.h
 * Dependencies:    None
 * Processor:       PIC32
 * Compiler:        Microchip C32 v1.10 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright (C) 2002-2012 Microchip Technology Inc.  All rights
 * reserved.
 *
 * Microchip licenses to you the right to use, modify, copy, and
 * distribute:
 * (i)  the Software when embedded on a Microchip microcontroller or
 *      digital signal controller product ("Device") which is
 *      integrated into Licensee's product; or
 * (ii) ONLY the Software driver source files ENC28J60.c, ENC28J60.h,
 *		ENCX24J600.c and ENCX24J600.h ported to a non-Microchip device
 *		used in conjunction with a Microchip ethernet controller for
 *		the sole purpose of interfacing with the ethernet controller.
 *
 * You should refer to the license agreement accompanying this
 * Software for additional information regarding your rights and
 * obligations.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 *
 * Author               Date		Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Chris Smith		09/30/11	Original
 ********************************************************************/
#ifndef __HARDWARE_PROFILE_H
#define __HARDWARE_PROFILE_H

#include "GenericTypeDefs.h"
#include "Compiler.h"

// PIC32MX processor
#define GetSystemClock()		(40000000ul)      // Hz
#define GetInstructionClock()	(GetSystemClock()/1)
#define GetPeripheralClock()	(GetInstructionClock()/1)	// Set your divider according to your Peripheral Bus Frequency configuration fuse setting


// Hardware mappings
//----------------------------
// LED and Button I/O pins
//----------------------------
#define LED0_TRIS			(TRISEbits.TRISE0)	// Ref D10 Green
#define LED0_IO				(LATEbits.LATE0)
#define LED1_TRIS			(TRISFbits.TRISF1)	// Ref D9 Yellow
#define LED1_IO				(LATFbits.LATF1)
#define LED2_TRIS			(TRISFbits.TRISF0)	// Ref D8 Red
#define LED2_IO				(LATFbits.LATF0)

#define LEDS_ON()           {LED0_ON(); LED1_ON(); LED2_ON();}
#define LEDS_OFF()          {LED0_OFF(); LED1_OFF(); LED2_OFF();}
#define LED0_ON()           LATESET = BIT_0;
#define LED0_OFF()          LATECLR = BIT_0;
#define LED0_INV()          LATEINV = BIT_0;

#define LED1_ON()           LATFSET = BIT_1;
#define LED1_OFF()          LATFCLR = BIT_1;
#define LED1_INV()          LATFINV = BIT_1;

#define LED2_ON()           LATFSET = BIT_0;
#define LED2_OFF()          LATFCLR = BIT_0;
#define LED2_INV()          LATFINV = BIT_0;

#define SW0_TRIS            (TRISDbits.TRISD9)
#define	SW0_IO              (PORTDbits.RD9)

#define VBAT_TRIS           (TRISBbits.TRISB0)


//----------------------------
// MRF24WB0M WiFi I/O pins
//----------------------------
#define WF_CS_TRIS			(TRISGbits.TRISG9)
#define WF_CS_IO			(LATGbits.LATG9)
#define WF_SDI_TRIS			(TRISGbits.TRISG7)
#define WF_SCK_TRIS			(TRISGbits.TRISG6)
#define WF_SDO_TRIS			(TRISGbits.TRISG8)
#define WF_RESET_TRIS		(TRISDbits.TRISD1)
#define WF_RESET_IO			(LATDbits.LATD1)
        
// NOTE: Must Ensure WiFi Interrupt Handler definition WF_INT_VECTOR is set to:
// _EXTERNAL_1_VECTOR in file WF_Config.h
#define WF_INT_TRIS	        (TRISDbits.TRISD8)  // INT1
#define WF_INT_IO		    (PORTDbits.RD8)

#define WF_HIBERNATE_TRIS	(TRISEbits.TRISE4)
#define	WF_HIBERNATE_IO		(PORTEbits.RE4)

#define WF_INT_EDGE		    (INTCONbits.INT1EP)
#define WF_INT_IE			(IEC0bits.INT1IE)
#define WF_INT_IF			(IFS0bits.INT1IF)
#define WF_INT_IE_CLEAR     IEC0CLR
#define WF_INT_IF_CLEAR     IFS0CLR
#define WF_INT_IE_SET       IEC0SET
#define WF_INT_IF_SET       IFS0SET
#define WF_INT_BIT          0x00000080
#define WF_INT_IPCSET       IPC1SET
#define WF_INT_IPCCLR       IPC1CLR
#define WF_INT_IPC_MASK     0xFF000000
#define WF_INT_IPC_VALUE    0x0C000000

#define WF_SSPBUF			(SPI2BUF)
#define WF_SPISTAT			(SPI2STAT)
#define WF_SPISTATbits		(SPI2STATbits)

#define WF_SPICON1			(SPI2CON)
#define WF_SPICON1bits		(SPI2CONbits)
#define WF_SPI_IE_CLEAR     IEC1CLR
#define WF_SPI_IF_CLEAR     IFS1CLR
#define WF_SPI_INT_BITS     0x000000e0
#define WF_SPI_BRG		    (SPI2BRG)
#define WF_MAX_SPI_FREQ     (10000000ul)	// Hz

//----------------------------
// UART to Telnet Mapping
//----------------------------
#define BusyUART()          (TelnetOutFree() ? 0 : (StackTask(), 1))
#define putcUART            putchar
#define putrsUART(a)        fputs((const char*)a,(FILE *)stdout)
#define putsUART(a)         fputs((const char*)a,(FILE *)stdout)
#define DataRdyUART()       TelnetInChars()
#define ReadUART()          TelnetGet()

//----------------------------
// Sensor Port Mapping
//----------------------------
#define SENSOR_UART         UART2

#endif
