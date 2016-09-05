/**************************************************************
 * HTTPPrint.h
 * Provides callback headers and resolution for user's custom
 * HTTP Application.
 * 
 * This file is automatically generated by the MPFS Utility
 * ALL MODIFICATIONS WILL BE OVERWRITTEN BY THE MPFS GENERATOR
 **************************************************************/

#ifndef __HTTPPRINT_H
#define __HTTPPRINT_H

#include "TCPIP Stack/TCPIP.h"

#if defined(STACK_USE_HTTP2_SERVER)

extern HTTP_STUB httpStubs[MAX_HTTP_CONNECTIONS];
extern BYTE curHTTPID;

void HTTPPrint(DWORD callbackID);
void HTTPPrint_led(WORD);
void HTTPPrint_btn(WORD);
void HTTPPrint_scan(void);
void HTTPPrint_bssCount(void);
void HTTPPrint_wlan(void);
void HTTPPrint_ssid(void);
void HTTPPrint_prevSSID(void);
void HTTPPrint_prevWLAN(void);
void HTTPPrint_curPrivacy(void);
void HTTPPrint_Demoversion(void);
void HTTPPrint_aplist(void);
void HTTPPrint_ipaddr(void);

void HTTPPrint(DWORD callbackID)
{
	switch(callbackID)
	{
        case 0x0000000b:
			HTTPPrint_led(2);
			break;
        case 0x0000000c:
			HTTPPrint_led(1);
			break;
        case 0x00000017:
			HTTPPrint_led(0);
			break;
        case 0x00000018:
			HTTPPrint_btn(0);
			break;
        case 0x0000004a:
			HTTPPrint_scan();
			break;
        case 0x0000004c:
			HTTPPrint_bssCount();
			break;
        case 0x00000056:
			HTTPPrint_wlan();
			break;
        case 0x0000005b:
			HTTPPrint_ssid();
			break;
        case 0x0000005d:
			HTTPPrint_prevSSID();
			break;
        case 0x0000005e:
			HTTPPrint_prevWLAN();
			break;
        case 0x00000067:
			HTTPPrint_curPrivacy();
			break;
        case 0x00000068:
			HTTPPrint_Demoversion();
			break;
        case 0x00000069:
			HTTPPrint_aplist();
			break;
        case 0x0000006a:
			HTTPPrint_ipaddr();
			break;
		default:
			// Output notification for undefined values
			TCPPutROMArray(sktHTTP, (ROM BYTE*)"!DEF", 4);
	}

	return;
}

void HTTPPrint_(void)
{
	TCPPut(sktHTTP, '~');
	return;
}

#endif

#endif