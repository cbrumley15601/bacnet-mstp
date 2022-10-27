/*-----------------------------------------------------------------------------
Copyright 2022 Coleman Brumley

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in 
the Software without restriction, including without limitation the rights to 
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies 
of the Software, and to permit persons to whom the Software is furnished to do 
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all 
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-----------------------------------------------------------------------------*/ 
#ifndef __MSTP_H_INCLUDED
#define __MSTP_H_INCLUDED

#include "mstp_ioctl.h"

#define MSTP_MSG "mstp: "

#define MSTP_PACKET_LEN    8 /* no way out, unless you change the code */

extern unsigned char mstp_magic;

/* Timer defined, based on HZ = 200 in /usr/src/asm/param.h */
#define MSTP_1SEC   (HZ)     /* 1000 ms */
#define MSTP_500MS  (HZ/2)   /* 500 ms */
#define MSTP_100MS  (HZ/10)  /* 200 ms */
#define MSTP_10MS   (HZ/100) /* 10 ms */
#define MSTP_5MS    (HZ/200)  /* 5 ms */

/* Boolean Defines */
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

/* These are the baud rate hash definitions */
#define USERBAUD9600 '9'+'6' //57 + 54 = 111
#define USERBAUD1920 '1'+'9' //49 + 57 = 106
#define USERBAUD3840 '3'+'8' //51 + 56 = 107
#define USERBAUD5760 '5'+'7' //53 + 55 = 108
#define USERBAUD7680 '7'+'6' //55 + 54 = 109
#define USERBAUD1152 '1'+'1' //49 + 49 = 98

typedef unsigned char octet;
typedef unsigned short word;
#ifndef _dword
typedef unsigned long dword;
#define _dword
#endif
typedef unsigned char UINT8;

#define byte octet
#define mstpbool octet

#define _octet	1
#define _word	1

//max apdu  = 480
//max NL hdr = 21 (worst case)

//max data	= 501, includes NPDU (21 bytes)
//mstp hdr	= 8
//datacrc	= 2
//0xFF pad	= 1
//Total		= 512
#define		maxrx				512				//max chars in rx buffer (NPDU size)
#define		maxtx				512				//max chars transmitted
#define		INPUT_BUFFER_SIZE	maxrx
#define maxrcvqsize			(INPUT_BUFFER_SIZE*4)		//circular receive queue size

//status flags
#define		receivedPFM		1
#define		receivedToken	2
#define		receivedDER		4
#define		receivedDNER	8
#define		solemanager		16
#define		CRCErr			32

#define MSTP_BROADCAST_ADDRESS 		0xFF

typedef struct _mstpFrame {
	struct _mstpFrame  *next;
	word	plen;						//transmit length
	octet	req;						//0=no reply expected, 1=reply expected
	octet	pre[2];						//preamble 55 FF
	octet	type;						//frame type
	octet	da;							//dest addr
	octet	sa;							//source addr
	octet	dlenhi;						//big endian data length
	octet	dlenlo;
	octet	hcrc;
	octet	data[maxtx+3];				//data+2CRC+pad
	} mstpFrame;

#define mstpBroadcast					0xFF

//																						***206 End
//MSTP frame types
#define	mftToken						0x00
#define	mftPollForManager				0x01
#define	mftReplyToPollForManager		0x02
#define	mftTestRequest					0x03
#define	mftTestResponse					0x04
#define	mftBACnetDataExpectingReply		0x05
#define	mftBACnetDataNotExpectingReply	0x06
#define	mftReplyPostponed				0x07
#define mftUnknown						0x08

//Low Level Receive State Machine states
#define rfsmIdle						0
#define rfsmPreamble					1
#define	rfsmHeader						2
#define	rfsmData						3
#define	rfsmSkipData					4

//Manager Node State Machine states
#define	mnsmInitialize					0
#define mnsmIdle						1
#define mnsmUseToken					2
#define mnsmWaitForReply				3
#define mnsmDoneWithToken				4
#define mnsmPassToken					5
#define mnsmNoToken						6
#define mnsmPollForManager				7
#define mnsmAnswerDataRequest			8

#ifdef __cplusplus
extern "C" {            /* Assume C declarations for C++ */
#endif /* __cplusplus */

//------------------------------------------------------------------------
//Functions provided by MSTP.C:
void mstpVarInit(byte port,int turnaround);							//				***206 Begin
void mstpReset(byte);
void mstpTimerCallback(void);

//------------------------------------------------------------------------
//Functions BACnet Stack must provide for MSTP.C to call:

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //__MSTP_H_INCLUDED
