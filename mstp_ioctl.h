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

#ifndef __MSTP_IOCTL_H__
#define __MSTP_IOCTL_H__

#include <linux/ioctl.h>

/* ioctl defines
 *
 * From /usr/src/linux/Documentation/ioctl-number.txt:
 * 0xBA isn't used, so we'll grab it for our driver
 * I'm not sure how to register this range for real, but this should do
 *
 * Code    Seq#    Include File            Comments
 * =========================================================================
 * 0xBA    C0-FF   mstp.h                  coleman.brumley@gmail.com
 */

#define MSTP_IOC_MAGIC 0xBA

#define MSTP_IOC_SETMAXMANAGER		_IOW(MSTP_IOC_MAGIC,0xC0,unsigned)
#define MSTP_IOC_SETMAXINFOFRAMES	_IOW(MSTP_IOC_MAGIC,0xC1,unsigned)
#define MSTP_IOC_SETMACADDRESS		_IOW(MSTP_IOC_MAGIC,0xC2,unsigned char)
#define MSTP_IOC_SETTUSAGE			_IOW(MSTP_IOC_MAGIC,0xC6,unsigned)
#define MSTP_IOC_GETMAXMANAGER		_IOR(MSTP_IOC_MAGIC,0xC3,unsigned)
#define MSTP_IOC_GETMAXINFOFRAMES	_IOR(MSTP_IOC_MAGIC,0xC4,unsigned)
#define MSTP_IOC_GETMACADDRESS		_IOR(MSTP_IOC_MAGIC,0xC5,unsigned char)
#define MSTP_IOC_GETTUSAGE			_IOR(MSTP_IOC_MAGIC,0xC7,unsigned)
#define MSTP_IOC_GETVER				_IOR(MSTP_IOC_MAGIC,0xC8,unsigned)
#define MSTP_IOC_GETONLINE			_IOR(MSTP_IOC_MAGIC,0xC9,unsigned)

#define MSTP_MIN_NR 0xC0
#define MSTP_MAX_NR 0xC9

#define N_MSTP N_MOUSE

//#define N_MSTP  (NR_LDISCS-5) 

/***************************************************************************** 
 * The following is needed for kernels greater than 2.6.22 
 * and a GLIBC less than ??? because the kernel and GLIBC weren't synced 
 * at this point and this is the easiest way to get to 76800 baud
 *****************************************************************************/
#ifndef TIOCSSSERIAL
#define TIOCSSERIAL  0x541F		
#endif

#endif

