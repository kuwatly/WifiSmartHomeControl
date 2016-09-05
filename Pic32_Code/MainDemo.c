/*********************************************************************
 *
 *  Main Application Entry Point
 *   -Demonstrates how to call and use the Microchip WiFi Module and
 *    TCP/IP stack
 *   -Reference: Microchip TCP/IP Stack Help (TCPIP Stack Help.chm)
 *
 *********************************************************************
 * FileName:        MainDemo.c
 * Dependencies:    TCPIP.h
 * Processor:       PIC32
 * Compiler:        Microchip C32 v1.11b or higher
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
 *	ENCX24J600.c and ENCX24J600.h ported to a non-Microchip device
 *	used in conjunction with a Microchip ethernet controller for
 *	the sole purpose of interfacing with the ethernet controller.
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
 * File Description:
 * Change History:
 * Date         Comment
 * ----------   -----------------------------------------
 * 2/21/2012    Initial release
 ********************************************************************/

#include "MainDemo.h"
#if defined( WF_CONSOLE )
#include "TCPIP Stack/WFConsole.h"
#include "IperfApp.h"
#endif 

#include <p32xxxx.h>		// Include PIC32 specifics header file
#include <plib.h>		// Include the PIC32 Peripheral Library

// Set configuration fuses
#pragma config FNOSC = FRCPLL, FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPLLODIV = DIV_2, FPBDIV = DIV_1, FWDTEN = OFF, POSCMOD = OFF, FSOSCEN = OFF, CP = OFF

//--------------------------------------------------------------------------------
// These defines use the C preprocessor to create the AppConfig structure elements
//--------------------------------------------------------------------------------
// MDEF(R,N) macro concatenates tokens to form "MY_DEFAULT_<R>BYTE<N>"
// For example MDEF(IP_ADDR_,4) expands to "MY_DEFAULT_IP_ADDR_BYTE4"

#define MDEF(R,N) MY_DEFAULT_ ## R ## BYTE ## N

// BUILD_MYDEFAULT32 invokes the MDEF(R,N) macro 4 times to create a 32 bit
// value from 4 byte values.
//
// For example BUILD_MYDEFAULT32(IP_ADDR_) expands to
// (MY_DEFAULT_IP_ADDR_BYTE4<<24|MY_DEFAULT_IP_ADDR_BYTE3<<16
//  |MY_DEFAULT_IP_ADDR_BYTE2<<8|MY_DEFAULT_IP_ADDR_BYTE1)

#define BUILD_MYDEFAULT32(R) (MDEF(R,4)<<24|MDEF(R,3)<<16|MDEF(R,2)<<8|MDEF(R,1))

// MACD(N) expands to MY_DEFAULT_MAC_BYTE<N>
// For example MACD(1) expands to MY_DEFAULT_MAC_BYTE1

#define MACD(N) MY_DEFAULT_MAC_BYTE ## N


#define SYSCLK (80000000L) 	// Give the system's clock frequency
#define NUM_OF_LEDS (10)	// Defines the number of LEDs on the LED bar, can be expanded

#define CONFIG1 (ADC_MODULE_ON | ADC_FORMAT_INTG32 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON)
#define CONFIG2 (ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF)
#define CONFIG3 (ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15)
#define CONFIGPORT (ENABLE_AN15_ANA)
#define CONFIGSCAN (SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 | SKIP_SCAN_AN12 |  SKIP_SCAN_AN13 |  SKIP_SCAN_AN14)

// Application-dependent structure used to contain IP address information
APP_CONFIG AppConfig =
{
    {BUILD_MYDEFAULT32(IP_ADDR_)},
    {BUILD_MYDEFAULT32(MASK_)},
    {BUILD_MYDEFAULT32(GATE_)},
    {BUILD_MYDEFAULT32(PRIMARY_DNS_)},
    {BUILD_MYDEFAULT32(SECONDARY_DNS_)},
    {BUILD_MYDEFAULT32(IP_ADDR_)},
    {BUILD_MYDEFAULT32(MASK_)},
    MY_DEFAULT_HOST_NAME,
	{1,1},	// Flags
    {{MACD(1),MACD(2),MACD(3),MACD(4),MACD(5),MACD(6)}}
};

// Globals
BOOL gRFModuleVer1209orLater = FALSE;


extern unsigned char TelnetPut(unsigned char c);

// Private helper functions.
static void InitializeBoard(void);
static void SelfTest(void);
void UARTTxBuffer(char *buffer, UINT32 size);


void _general_exception_handler(unsigned cause, unsigned status)
{
    Nop();
    Nop();
}


// Used for re-directing printf and UART statements to the Telnet daemon
void _mon_putc(char c)
{
    #ifdef STACK_USE_TELNET_SERVER
	TelnetPut(c);
    #endif
}
void initializeADC(){
	CloseADC10();		// Generally, you should disable the ADC before setup.

	// Use ground as negative reference for channel A instead of pin AN1 (RB1)
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF);
	OpenADC10( CONFIG1, CONFIG2, CONFIG3, CONFIGPORT, CONFIGSCAN);
				// Setup for the ADC10.
	EnableADC10();		// Enables the ADC10.
	while(!mAD1GetIntFlag() ) {}; // mAD1GetIntFlag() checks the interrupt flag for the AD10.
				      // Waits till a conversion is finished so that there's
				      // values in the ADC result registers.
}

void initializeLEDBar(){
	TRISDCLR = 0x03FF;	// Sets pins RD0 to RD9 as digital outputs.
}

// ************************************************************
// Main application entry point.
// ************************************************************
int main(void)
{
    static DWORD t = 0;
    tWFDeviceInfo deviceInfo;

    // Initialize application specific hardware
    InitializeBoard();

    // Initialize TCP/IP stack timer
    TickInit();

    // Initialize the MPFS File System
    MPFSInit();

    // Initialize core stack layers (MAC, ARP, TCP, UDP) and
    // application modules (HTTP, SNMP, etc.)
    StackInit();

    // Needed for modules with firware version 1209 or later
    WF_GetDeviceInfo(&deviceInfo);
    if (deviceInfo.romVersion == 18 && deviceInfo.patchVersion >= 9)
		gRFModuleVer1209orLater = TRUE;

    // Run Self Test if SW0 pressed
    if(SW0_IO == 0)
        SelfTest();

    #ifdef STACK_USE_TELNET_SERVER
    // Initialize Telnet and
    // Put Remote client in Remote Character Echo Mode
    TelnetInit();
 	putc(0xff, stdout);     // IAC = Interpret as Command
	putc(0xfe, stdout);     // Type of Operation = DONT
	putc(0x22, stdout);     // Option = linemode
	putc(0xff, stdout);     // IAC = Interpret as Command
	putc(0xfb, stdout);     // Type of Operation = DO
	putc(0x01, stdout);     // Option = echo
    #endif

    // Initialize WiFi Scan State Machine NV variables
    WFInitScan();

    // Establish WiFi Connection
    WiFiConnect();

    // Initialize Zeroconf Link-Local state-machine.
    ZeroconfLLInitialize();

    // Initialize DNS Host-Name from TCPIPConfig.h.
    mDNSInitialize(MY_DEFAULT_HOST_NAME);
    mDNSServiceRegister(
            (const char *) AppConfig.NetBIOSName,	// base name of the service
            "_http._tcp.local",			    // type of the service
            80,				                // TCP or UDP port, at which this service is available
            ((const BYTE *)"path=/index.htm"),	// TXT info
            1,								    // auto rename the service when if needed
            NULL,							    // no callback function
            NULL							    // no application context
            );

    mDNSMulticastFilterRegister();			

    #if defined(WF_CONSOLE)
    // Initialize the WiFi Console App
	WFConsoleInit();
    #endif

    // Now that all items are initialized, begin the co-operative
    // multitasking loop.  This infinite loop will continuously
    // execute all stack-related tasks, as well as your own
    // application's functions.  Custom functions should be added
    // at the end of this loop.
    // Note that this is a "co-operative mult-tasking" mechanism
    // where every task performs its tasks (whether all in one shot
    // or part of it) and returns so that other tasks can do their
    // job.
    // If a task needs very long time to do its job, it must be broken
    // down into smaller pieces so that other tasks can have CPU time.
    SYSTEMConfigPerformance(SYSCLK);
	initializeADC();	// Initialize the ADC10
//	initializeLEDBar();	// Initialize pins RD0 - RD9 as digital outputs

    while(1)
    {

        // Blink LED0 twice per sec when unconfigured, once per sec after config
        /*if((TickGet() - t >= TICK_SECOND/(4ul - (CFGCXT.isWifiDoneConfigure*2ul))))
        {
            t = TickGet();
           if ((ReadADC10(0) / 102 )<5 )
                LED0_INV();
            //LATD = (0x3FF >> (NUM_OF_LEDS - ( ReadADC10(0) / 102 ) ) );
        }*/

        // This task performs normal stack task including checking
        // for incoming packet, type of packet and calling
        // appropriate stack entity to process it.
        StackTask();

        // This task invokes each of the core stack application tasks
        StackApplications();

        // This task handles Zeroconf Link-Local events and takes the actions
        // accoding to current-state and event-notifications from ARP-Module.
        ZeroconfLLProcess();

        // This routine calls the mDNS state-machine
        mDNSProcess();

        // Process application specific tasks here.
        // Any custom modules or processing you need to do should
        // go here.
        #if defined(WF_CONSOLE)
		WFConsoleProcess();
		WFConsoleProcessEpilogue();
		#endif

    }
}

/****************************************************************************
  Function:
    static void InitializeBoard(void)

  Description:
    This routine initializes the hardware.

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
static void InitializeBoard(void)
{
    // Note: WiFi Module hardware Initialization handled by StackInit() Library Routine

    // Enable multi-vectored interrupts
    INTEnableSystemMultiVectoredInt();

    // Enable optimal performance
    SYSTEMConfigPerformance(GetSystemClock());

    // Use 1:1 CPU Core:Peripheral clocks
    mOSCSetPBDIV(OSC_PB_DIV_1);     

    // Disable JTAG port so we get our I/O pins back, but first
    // wait 50ms so if you want to reprogram the part with
    // JTAG, you'll still have a tiny window before JTAG goes away.
    // The PIC32 Starter Kit debuggers use JTAG and therefore must not
    // disable JTAG.
    DelayMs(50);
    DDPCONbits.JTAGEN = 0;

    // LEDs
    LEDS_OFF();
    LED0_TRIS = 0;
    LED1_TRIS = 0;
    LED2_TRIS = 0;

    // Push Button
    SW0_TRIS = 1;
    
}

/****************************************************************************
  Function:
    void SelfTest()

  Description:
    This routine performs a self test of the hardware.

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
void SelfTest()
{
    char value = 0;
    char* buf[32];

    // Configure Sensor Serial Port
    UARTConfigure(SENSOR_UART, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(SENSOR_UART, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(SENSOR_UART, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(SENSOR_UART, GetPeripheralClock(), 9600);
    UARTEnable(SENSOR_UART, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Verify MRF24WB0MA MAC Address
    if(AppConfig.MyMACAddr.v[0] == 0x00 && AppConfig.MyMACAddr.v[1] == 0x1E)
    {
        //********************************************************************
        // Prints a label using ESC/P commands to a Brother PT-9800PCN printer
        //********************************************************************
        // Send ESC/P Commands to setup printer
        UARTTxBuffer("\033ia\000\033@\033X\002",9); // ESC i a 0 = Put Printer in ESC/P Mode
                                                    // ESC @ = Reset Printer to Default settings
                                                    // ESC X 2 = Specify Character Size
        // Send the Info to Print for the MAC Address label
        UARTTxBuffer("MRF24WB0MA\r",11);
        sprintf((char *)buf,"MAC: %02X%02X%02X%02X%02X%02X",AppConfig.MyMACAddr.v[0],AppConfig.MyMACAddr.v[1],AppConfig.MyMACAddr.v[2],AppConfig.MyMACAddr.v[3],AppConfig.MyMACAddr.v[4], AppConfig.MyMACAddr.v[5]);
        UARTTxBuffer((char *)buf, strlen((const char *)buf));

        // Print the label
        UARTTxBuffer("\f",1);
        
        // Toggle LED's
        while(1)
        {
            LED0_IO = value;
            LED1_IO = value >> 1;
            LED2_IO = value >> 2;

            DelayMs(400);

            if(value == 8)
                value = 0;
            else
                value++;

        }
    }
    else    // MRF24WB0MA Failure
    {
        while(1)
        {
            LEDS_ON();
            DelayMs(700);
            LEDS_OFF();
            DelayMs(700);
        }
    }
}

/****************************************************************************
  Function:
    void UARTTxBuffer(char *buffer, UINT32 size)

  Description:
    This routine sends data out the Sensor UART port.

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
void UARTTxBuffer(char *buffer, UINT32 size)
{
    while(size)
    {
        while(!UARTTransmitterIsReady(SENSOR_UART))
            ;

        UARTSendDataByte(SENSOR_UART, *buffer);

        buffer++;
        size--;
    }

    while(!UARTTransmissionHasCompleted(SENSOR_UART))
        ;
}

/****************************************************************************
  Function:
    void DisplayIPValue(IP_ADDR IPVal)

  Description:
    This routine formats and prints the current IP Address.

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
void DisplayIPValue(IP_ADDR IPVal)
{
	printf("%u.%u.%u.%u", IPVal.v[0], IPVal.v[1], IPVal.v[2], IPVal.v[3]);
}
