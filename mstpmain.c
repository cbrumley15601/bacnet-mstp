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
/*----------------------------------------------------------------------------
Description:
                        BACnet MS/TP Driver for a Linux Kernel module
                        Uses hrtimer API and incorporates all published changes
                                through 135-2020 including addenda and errata
                        Language based on Add 135-2020ce, which removes all
divisive language
----------------------------------------------------------------------------*/
#ifndef MODULE
#define MODULE
#endif

#ifndef __KERNEL__
#define __KERNEL__
#endif

#include "mstp.h"
#include "queue.h"
#include <asm/delay.h> //udelay, busywait
#include <asm/ioctls.h>
#include <asm/termios.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/tty.h>

#define EXPORT_SYMTAB

#define MSTPMODULE_VERSION "5.30"
#define MSTPMODULE_NAME "BACnet MS/TP Driver"
#define ver 530      // version*100
#define DEV_MSTP 345 /* just random */
#define HW_LEN INPUT_BUFFER_SIZE
//#define USE_PAD_BYTE 1
//#define EXTRA_DEBUG 1
#define IO_TEST_PIN 48

/* /proc/BACnet/mstpstatus entry*/
static struct proc_dir_entry *bacnet_dir;

/* module timer states */
unsigned char mod_state;
#define STATE_Ready 'R'
#define STATE_Done 'D'

/* hrtimer settings */
#define NSEC_PER_MSEC 1000000L
static struct hrtimer hr_timer;
enum hrtimer_restart enHRTimer = HRTIMER_NORESTART;
s64 i64TimeInNsec = 1 * NSEC_PER_MSEC;

///////////////////////////////////////////////////////////////////////
//	MS/TP constant values

#define Npoll                                                                  \
  50 // number of tokens received or used before Poll for Manager (fixed)
#define Nretrytoken 1     // number of retries on sending the token (fixed)
#define Nminoctets 4      // number of "events (octets) for active line (fixed)
#define Tframeabort 100   //(60 bit times) 100 ms max (fixed)
#define Tframegap 20      //(20 bit times) (fixed)
#define Tnotoken 500      // silence time for loss of token (fixed)
#define Tpostdrive 15     //(15 bit times) (fixed)
#define Treplydelay 200   // 200 ms (fixed) the lower the better!		***223
#define Treplytimeout 300 // 300 ms (fixed)
#define Tslot 10          // 10 ms (fixed)

/* this structure stores what goes to the upper layers */
struct mstp_data_t {
  unsigned char SourceAddress;
  unsigned char DestinationAddress;
  unsigned char FrameType;
  unsigned char data[INPUT_BUFFER_SIZE]; /* Here's the data! 		*/
  int count;                             /* how much data?			*/
  void *next;
};

///////////////////////////////////////////////////////////////////////
//	MS/TP variable values

static byte This_Station = 0xFE;
static unsigned int Nmax_info_frames = 10;
static unsigned int Nmax_manager = 127;
static word Index = 0;
static byte HeaderCRC = 0xFF;
static word DataCRC = 0xFFFF;
static bool ReceivedValidFrame = false;
static bool ReceivedInvalidFrame = false;
static byte FrameType;
static byte SourceAddress;
static byte DestinationAddress;
static word DataLength = 0;
static byte SoleManager;
static byte ns, ps;
static bool DataAvailable;
static byte tokencount = 0;
static byte framecount = 0;
static byte retrycount = 0;
static byte RFSMstate = rfsmIdle;
static byte mnstate = mnsmInitialize;
static int rx_errors = 0;
static int Tturnaround = (40 / 38400);
static byte InputBuffer[maxrx];
static byte OutputBuffer[maxtx];
static u_long ReplyTimer = 0;
static int eventcount = 0;
static int Tusage_timeout = 35;
static int Tusage_timeoutTP = 85;
static queue receive_queue = {0};
static queue send_queue = {0};
static unsigned int headercrccnt = 0;
static unsigned int datacrcerrcnt = 0;
static unsigned int rxinvalidframe = 0;
static int baud = 38400;
struct tty_struct *mstp_tty = NULL;
static int num_rx_errors = 0;
static int num_fe = 0;
static int num_pe = 0;
static int num_oe = 0;
static int num_unkerr = 0;
static int frame_abort_errors = 0;
static unsigned long num_rx_bytes = 0;
static unsigned long true_delay = 0;
static unsigned long num_tx_bytes = 0;
static unsigned long TX_PFM_Count = 0;
static unsigned long RX_PFM_Count = 0;
static unsigned long TX_Token_Count = 0;
static unsigned long RX_Token_Count = 0;
static unsigned long Num_Invalid_Large_Frames = 0;
static unsigned long numHdrCRCErrs = 0;
static unsigned long numDataCRCErrs = 0;
static unsigned char errb[maxrx + 8];
static bool online = false;
static unsigned long SentPacketCounter = 0;
static unsigned long RecdPacketCounter = 0;
static int joined_state = 0;
/* Spinlock to protect the mstp_tty */
DEFINE_SPINLOCK(mstp_tty_lock);

///////////////////////////////////////////////////////////////////////
//	MS/TP constant values

#define Nmaxmanager Nmax_manager
#define Tusagetimeout Tusage_timeout
#define Tframe_abort Tframeabort
#define Tno_token Tnotoken
#define Nmin_octets Nminoctets
#define Nretry_token Nretrytoken
#define Treply_timeout Treplytimeout

#define nMSTPports 1

static char *mnsm_strings[] = {
    "00 Initialize",   "01 Idle",           "02 UseToken",
    "03 WaitForReply", "04 DoneWithToken",  "05 PassToken",
    "06 NoToken",      "07 PollForManager", "08 AnswerDataRequest"};

static char *rfsm_strings[] = {"00 Idle", "01 Preamble", "02 Header", "03 Data",
                               "04 SkipData"};

#ifdef __cplusplus
extern "C" { /* Assume C declarations for C++ */
#endif       /* __cplusplus */

///////////////////////////////////////////////////////////////////////
//	function prototypes

static word CalcDataCRC(byte dv, word cv);
static octet CalcHeaderCRC(byte dv, byte cv);
int CalcTXTime(word);
static void SendFrame(byte SendFrameType, byte destination, byte src,
                      byte *data, unsigned data_len);
struct mstp_data_t *ChkTxQ(byte);
struct mstp_data_t *GetTxQ(byte);
static u_char RFSM(u_char ch);
static bool ManagerNodeStateMachine(void);
static ssize_t mstp_read(struct tty_struct *tty, struct file *file,
                         unsigned char __user *buf, size_t nr, void **cookie, unsigned long offset);
static ssize_t mstp_write(struct tty_struct *tty, struct file *file,
                          const unsigned char *buf, size_t nr);
static unsigned int mstp_poll(struct tty_struct *tty, struct file *filp,
                              poll_table *wait);

// NOTE: these functions are in the low level uart driver
extern void mstpSetToMSTP(struct tty_struct *tty);
extern int mstpReadSilenceTimer(void);
extern void mstpResetSilenceTimer(void);
extern void mstpSetSilenceTimer(int val);
extern int mstpTransmitComplete(struct tty_struct *tty);
extern unsigned long mstpShutdownLock(struct tty_struct *tty);
extern void mstpShutdownUnlock(struct tty_struct *tty, unsigned long);

#ifdef __cplusplus
}
#endif /* __cplusplus */

///////////////////////////////////////////////////////////////////////
//	initialize mstp subsystem

void mstpVarInit(byte port, int turnaround) {
  if (port > nMSTPports)
    return;
  headercrccnt = 0;
  datacrcerrcnt = 0;
  rxinvalidframe = 0;
}

///////////////////////////////////////////////////////////////////////
//	Calculate the Transmit Time of a buffered frame
//
// in:	n			the number of octets to send
//		baudrate	the baudrate
//
// out:	calculated transmit time in msec

int CalcTXTime(word n) {
  int tt;

  switch (baud) // we must calculate how long it will take to transmit
  {
  case 9600:
    tt = (int)n; // 9600 is approximately 1 msec per octet
    break;
  case 19200:
    tt = (int)(n >> 1); // 19200 is approximately 0.5 msec per octet
    break;
  case 38400:
    tt = (int)(n >> 2); // 38400 is approximately 0.25 msec per octet
    break;
  case 76800:
    tt = (int)(n >> 4); // 76800 is approximately 0.125 msec per octet
    break;
  case 57600:
    tt = (int)((n >> 2) +
               (n >> 1)); // 57600 is approximately 0.166 msec per octet
    break;
  case 115200:
    tt = (int)((n >> 2) +
               (n >> 1)); // 115200 is approximately 0.083 msec per octet
    tt >>= 2;
    break;
  default:
    tt = (int)n; // default is approximately 1 msec per octet
    break;
  }
  return tt;
}

///////////////////////////////////////////////////////////////////////
//	Reset
//
//	Resets the state machines (goes back to startup)
//
// in:	port	the port number to reset

void mstpReset(byte port) {
  mnstate = mnsmInitialize;             // we've been starved of time, restart
  mstpSetSilenceTimer(Tframeabort + 1); // to reset the RFSM
  RFSMstate = rfsmIdle;
}

///////////////////////////////////////////////////////////////////////
//	Work function for servicing MNSM
//

void mstpTimerCallback(void) {
  bool transitionnow = false;
  unsigned long flags;
  if (This_Station > 127)
    return; // not yet inited
  spin_lock_irqsave(&mstp_tty_lock, flags);
  if (!mstp_tty)
    goto end;
  if (!mstpTransmitComplete(mstp_tty))
    goto end;
  if (mstpReadSilenceTimer() > 0) {
    transitionnow = ManagerNodeStateMachine();
    while ((transitionnow == true) && (mstpReadSilenceTimer() > 0) &&
           (mstpTransmitComplete(mstp_tty))) {
      transitionnow = ManagerNodeStateMachine();
    }
  }
end:
  spin_unlock_irqrestore(&mstp_tty_lock, flags);
  return;
}

///////////////////////////////////////////////////////////////////////
//	Receive Frame State Machine
//

static u_char RFSM(u_char ch) {
  struct mstp_data_t *mstp_receive_ptr;
  static unsigned long hbpos = 0;
  switch (RFSMstate) {
  case rfsmIdle:
    if (rx_errors != 0) // EatAnError
    {
      rx_errors = 0;
      mstpResetSilenceTimer();
      eventcount++;
      RFSMstate = rfsmIdle;
      break;
    } else if (rx_errors == 0) // EatAnOctet
    {
      if (DataAvailable == true) {
        if (ch != 0x55) {
          DataAvailable = false;
          mstpResetSilenceTimer();
          eventcount++;
          RFSMstate = rfsmIdle;
          break;
        } else if (ch == 0x55) // Preamble1
        {
          DataAvailable = false;
          mstpResetSilenceTimer();
          eventcount++;
          hbpos = 0;
          errb[hbpos++] = ch;
          RFSMstate = rfsmPreamble;
          break;
        }
      }
    }
    break;
  case rfsmPreamble:
    if (mstpReadSilenceTimer() > Tframe_abort) // Timeout
    {
      frame_abort_errors++;
      RFSMstate = rfsmIdle;
      break;
    }
    if (rx_errors != 0) // Error
    {
      mstpResetSilenceTimer();
      eventcount++;
      RFSMstate = rfsmIdle;
      break;
    } else if (rx_errors == 0) {
      if (DataAvailable == true) {
        if (ch == 0xFF) // Preamble2
        {
          DataAvailable = false;
          mstpResetSilenceTimer();
          eventcount++;
          errb[hbpos++] = ch;
          Index = 0;
          HeaderCRC = 0xFF;
          RFSMstate = rfsmHeader;
          break;
        } else if (ch == 0x55) // RepeatedPreamble1
        {
          DataAvailable = false;
          mstpResetSilenceTimer();
          eventcount++;
          errb[hbpos++] = ch;
          RFSMstate = rfsmPreamble;
          break;
        } else // if((ch!=0x55) && (ch!=0xFF))				//Not
               // Preamble
        {
          DataAvailable = false;
          mstpResetSilenceTimer();
          eventcount++;
          errb[hbpos++] = ch;
          RFSMstate = rfsmIdle;
          break;
        }
      }
    }
    break;
  case rfsmHeader:
    if (mstpReadSilenceTimer() > Tframe_abort) // Timeout
    {
      frame_abort_errors++;
      ReceivedInvalidFrame = true;
      RFSMstate = rfsmIdle;
      break;
    }
    if (rx_errors != 0) // Error
    {
      rx_errors = 0;
      mstpResetSilenceTimer();
      eventcount++;
      errb[hbpos++] = ch;
      ReceivedInvalidFrame = true;
      RFSMstate = rfsmIdle;
      break;
    }
    if ((rx_errors == 0) && (DataAvailable == true)) {
      if (Index == 0) // FrameType
      {
        DataAvailable = false;
        mstpResetSilenceTimer();
        eventcount++;
        errb[hbpos++] = ch;
        HeaderCRC = CalcHeaderCRC(ch, HeaderCRC);
        FrameType = ch;
        Index = 1;
        RFSMstate = rfsmHeader;
        break;
      }
      if (Index == 1) // DestinationAddress
      {
        DataAvailable = false;
        mstpResetSilenceTimer();
        eventcount++;
        errb[hbpos++] = ch;
        HeaderCRC = CalcHeaderCRC(ch, HeaderCRC);
        DestinationAddress = ch;
        Index = 2;
        RFSMstate = rfsmHeader;
        break;
      }
      if (Index == 2) // SourceAddress
      {
        DataAvailable = false;
        mstpResetSilenceTimer();
        eventcount++;
        errb[hbpos++] = ch;
        HeaderCRC = CalcHeaderCRC(ch, HeaderCRC);
        SourceAddress = ch;
        Index = 3;
        RFSMstate = rfsmHeader;
        break;
      }
      if (Index == 3) // Length1
      {
        DataAvailable = false;
        mstpResetSilenceTimer();
        eventcount++;
        errb[hbpos++] = ch;
        HeaderCRC = CalcHeaderCRC(ch, HeaderCRC);
        DataLength = ch * 256;
        Index = 4;
        RFSMstate = rfsmHeader;
        break;
      }
      if (Index == 4) // Length2
      {
        DataAvailable = false;
        mstpResetSilenceTimer();
        eventcount++;
        errb[hbpos++] = ch;
        HeaderCRC = CalcHeaderCRC(ch, HeaderCRC);
        DataLength += ch;
        Index = 5;
        RFSMstate = rfsmHeader;
        break;
      }
      if (Index == 5) // HeaderCRC
      {
        DataAvailable = false;
        mstpResetSilenceTimer();
        eventcount++;
        errb[hbpos++] = ch;
        HeaderCRC = CalcHeaderCRC(ch, HeaderCRC);
        // printk(MSTP_MSG "Header CRC=%02X -->
        // %s,Index=%d\n",HeaderCRC,rfsm_strings[RFSMstate],Index);
        if (HeaderCRC != 0x55) // BadCRC
        {
          // printk(MSTP_MSG "*****Header CRC Error!******\n");
          ReceivedInvalidFrame = true;
          RFSMstate = rfsmIdle;
          numHdrCRCErrs++;
          // memcpy(errb,hb,hbpos-1);
          errb[++hbpos] = HeaderCRC;
          break;
        } else if (HeaderCRC ==
                   0x55) // HeaderCRC state follows, not a state per se though
        {
          memset(errb, 0x00, sizeof(errb));
          if ((DestinationAddress != This_Station) && // NotForUs
              (DestinationAddress != 0xFF) && (DataLength == 0)) {
            RFSMstate = rfsmIdle;
            hbpos = 0;
            break;
          } else {
            if (DestinationAddress == This_Station) {
              if (FrameType == mftToken) {
                RX_Token_Count++;
              }
              if (FrameType == mftPollForManager) {
                RX_PFM_Count++;
              }
            }
            if (DataLength == 0) // No Data
            {
              ReceivedValidFrame = true;
              RFSMstate = rfsmIdle;
              break;
            } else if ((DataLength != 0) && // Data
                       (DataLength <= maxrx)) {
              if ((DestinationAddress !=
                   This_Station) && // DataNotForUs (Addendum 135-2008z-3)
                  (DestinationAddress != 0xFF)) {
                Index = 0;
                RFSMstate = rfsmSkipData;
                break;
              } else {
                Index = 0;
                DataCRC = 0xFFFF;
                RFSMstate = rfsmData;
                break;
              }
            } else // FrameTooLong
            {
              if (DataLength <= (maxrx * 2)) {
                RFSMstate = rfsmSkipData; // reasonable length
              } else // This is an invalid length, don't try to consume it
              {
                Num_Invalid_Large_Frames++;
                DataAvailable = false;
                mstpResetSilenceTimer();
                ReceivedInvalidFrame = true;
                RFSMstate = rfsmIdle;
                hbpos = 0;
              }
              break;
            }
          }
        }
      }
    }
    break;
  case rfsmSkipData: // SkipData (Addendum 135-2008z-3)
    if (mstpReadSilenceTimer() > Tframe_abort) // Timeout
    {
      frame_abort_errors++;
      ReceivedInvalidFrame = true;
      RFSMstate = rfsmIdle;
      break;
    }
    if (rx_errors != 0) // Error
    {
      rx_errors = 0;
      mstpResetSilenceTimer();
      ReceivedInvalidFrame = true;
      RFSMstate = rfsmIdle;
      break;
    }
    if (rx_errors == 0) {
      if (DataAvailable) {
        if (Index < (DataLength + 1)) // DataOctet
        {
          DataAvailable = false;
          mstpResetSilenceTimer();
          Index++;
          RFSMstate = rfsmSkipData;
          break;
        } else if (Index == (DataLength + 1)) // Done
        {
          DataAvailable = false;
          mstpResetSilenceTimer();
          RFSMstate = rfsmIdle;
          break;
        }
      }
    }
    break;
  case rfsmData:
    if (mstpReadSilenceTimer() > Tframe_abort) // Timeout
    {
      frame_abort_errors++;
      ReceivedInvalidFrame = true;
      RFSMstate = rfsmIdle;
      break;
    }
    if (rx_errors != 0) // Error
    {
      rx_errors = 0;
      mstpResetSilenceTimer();
      ReceivedInvalidFrame = true;
      RFSMstate = rfsmIdle;
      break;
    }
    if (rx_errors == 0) {
      if (DataAvailable == true) {
        if (Index <= DataLength) // Data
        {
          DataAvailable = false;
          mstpResetSilenceTimer();
          DataCRC = CalcDataCRC(ch, DataCRC);
          InputBuffer[Index++] = ch;
          RFSMstate = rfsmData;
          break;
        } else if (Index == (DataLength + 1)) {
          DataAvailable = false;
          mstpResetSilenceTimer();
          DataCRC = CalcDataCRC(ch, DataCRC);
          if (DataCRC != 0xF0B8) {
            ReceivedInvalidFrame = true;
            RFSMstate = rfsmIdle;
            numDataCRCErrs++;
            break;
          } else if (DataCRC == 0xF0B8) {
            ReceivedValidFrame = true;
            // the following is "outside" the standard
            // as soon as we get any data that's broadcast or for TS
            // then we hand it off to the RxQ for processing
            if (FrameType == mftBACnetDataExpectingReply ||
                FrameType ==
                    mftBACnetDataNotExpectingReply) // queue only these types
            {
              mstp_receive_ptr =
                  (struct mstp_data_t *)alloc_entry(sizeof(struct mstp_data_t));
              if (!mstp_receive_ptr) {
                ReceivedValidFrame = false;
                RFSMstate = rfsmIdle;
                break;
              }
              mstp_receive_ptr->SourceAddress = SourceAddress;
              mstp_receive_ptr->DestinationAddress = DestinationAddress;
              mstp_receive_ptr->FrameType = FrameType;
              memmove(mstp_receive_ptr->data, InputBuffer, DataLength);
              mstp_receive_ptr->count = DataLength;
              Q_PushHead(&receive_queue, mstp_receive_ptr);
              // printk(MSTP_MSG "Put %d into the receive
              // queue\n",mstp_receive_ptr->count);
            }
            RFSMstate = rfsmIdle;
            break;
          }
        }
      }
    }
    break;
  default:
    break;
  }
  return RFSMstate;
}

//////////////////////////////////////////////////////////////////////
// The Manager Node State Machine
//	out: true if we need to immediately transition
//		 false otherwise

static bool ManagerNodeStateMachine(void) {
  bool transitionnow = false;
  static struct mstp_data_t *mstp_send_ptr;
  switch (mnstate) {
  case mnsmInitialize:
    ns = This_Station;
    ps = This_Station;
    tokencount = Npoll;
    SoleManager = false;
    ReceivedValidFrame = false;
    ReceivedInvalidFrame = false;
    mnstate = mnsmIdle;
    mstpResetSilenceTimer();
    break;
  case mnsmIdle:
    if (mstpReadSilenceTimer() >= Tno_token) // LostToken
    {
      eventcount = 0; // Addendum 135-2004d-8
      mnstate = mnsmNoToken;
      break;
    }
    if (ReceivedInvalidFrame == true) // ReceivedInvalidFrame
    {
      ReceivedInvalidFrame = false;
      mnstate = mnsmIdle;
      break;
    }
    if (ReceivedValidFrame == true) {
      if ((DestinationAddress != This_Station) && // ReceivedUnwantedFrame (a)
          (DestinationAddress != 0xFF)) {
        ReceivedValidFrame = false;
        mnstate = mnsmIdle;
        break;
      }
      if ((DestinationAddress ==
           0xFF) && //(b) -- such frames may not be broadcast
          ((FrameType == mftToken) || (FrameType == mftTestRequest) ||
           (FrameType >= mftUnknown))) {
        ReceivedValidFrame = false;
        mnstate = mnsmIdle;
        break;
      }
      if (FrameType >= mftUnknown) //(c) -- proprietary not know to us
      {
        ReceivedValidFrame = false;
        mnstate = mnsmIdle;
        break;
      }
      if (DestinationAddress == This_Station) {
        if (FrameType == mftToken) // ReceivedToken
        {
          ReceivedValidFrame = false;
          framecount = 0;
          SoleManager = false;
          if (joined_state == 0) {
#ifdef EXTRA_DEBUG
            printk(MSTP_MSG "Joined the MS/TP network\n");
#endif
            joined_state = 1;
            online = true;
          }
          mnstate = mnsmUseToken;
          transitionnow = true;
          return transitionnow;
          // break;
        }
        if (FrameType == mftPollForManager) {
          SendFrame(mftReplyToPollForManager, SourceAddress, This_Station, NULL,
                    0);
          if (joined_state == 1) {
#ifdef EXTRA_DEBUG
            printk(MSTP_MSG "Rec'd a PFM after joining\n");
#endif
            joined_state = 0;
          }
          ReceivedValidFrame = false;
          mnstate = mnsmIdle;
          break;
        }
        if (FrameType ==
            mftBACnetDataExpectingReply) // ReceivedDataNeedingReply
        {
          ReplyTimer = 0;
          ReceivedValidFrame = false;
          mnstate = mnsmAnswerDataRequest;
          break;
        }
        if (FrameType == mftTestRequest) // ReceivedTestRequest
        {
          if (DataLength <= (maxtx - 21)) // we have space to echo data
            SendFrame(
                mftTestResponse, SourceAddress, This_Station, InputBuffer,
                DataLength); // handle this here w/o application's involvement
          else               // it's ok to respond with no data
            SendFrame(mftTestResponse, SourceAddress, This_Station, InputBuffer,
                      0); // handle this here w/o application's involvement
          ReceivedValidFrame = false;
          mnstate = mnsmIdle;
          break;
        }
        if (FrameType == mftTestResponse) // ReceivedTestResponse
        {
          ReceivedValidFrame = false; // just drop it
          mnstate = mnsmIdle;
          break;
        }
        if ((FrameType ==
             mftReplyToPollForManager) || // Addendum 135-2016bm-3 (case d in
                                          // ReceivedUnwantedFrame)
            (FrameType == mftReplyPostponed)) {
          ReceivedValidFrame = false; // just drop it
          mnstate = mnsmIdle;
          break;
        }
      }
      if ((DestinationAddress == This_Station) || // ReceivedDataNoReply
          (DestinationAddress == 0xFF)) {
        if (FrameType == mftBACnetDataNotExpectingReply) {
          ReceivedValidFrame = false;
          mnstate = mnsmIdle;
          break;
        }
      }
      if ((DestinationAddress == 0xFF) &&
          (FrameType == mftBACnetDataExpectingReply)) // BroadcastDataNeedingReply
                                                      // (Addendum 135-2004b-9)
      {
        ReceivedValidFrame = false;
        mnstate = mnsmIdle;
        break;
      }
    }
    break;
  case mnsmUseToken:
    if (!Q_Size(&send_queue)) // NothingToSend
    {
      framecount = Nmax_info_frames;
      mnstate = mnsmDoneWithToken;
      transitionnow = true;
      return transitionnow;
    } else {
      mstp_send_ptr = (struct mstp_data_t *)Q_PopTail(&send_queue);
      if (mstp_send_ptr == NULL) {
#ifdef EXTRA_DEBUG
        printk(MSTP_MSG "Got NULL in pop queue in mnsmUseToken\n");
#endif
        Q_Empty(&send_queue, Nmax_info_frames);
        framecount = Nmax_info_frames;
        mnstate = mnsmDoneWithToken;
        transitionnow = true;
        return transitionnow;
      } else // if(mstp_send_ptr!=NULL)
      {
        if ((mstp_send_ptr->FrameType == mftTestResponse) ||
            (mstp_send_ptr->FrameType == mftBACnetDataNotExpectingReply) ||
            ((mstp_send_ptr->FrameType ==
              mftBACnetDataExpectingReply) && // Addendum 135-2004b-9, allow DER
                                              // to be broadcast
             (mstp_send_ptr->DestinationAddress == 0xFF))) {
          transitionnow = true;
          SendFrame(mstp_send_ptr->FrameType, mstp_send_ptr->DestinationAddress,
                    mstp_send_ptr->SourceAddress, mstp_send_ptr->data,
                    mstp_send_ptr->count);
          framecount++;
          mnstate = mnsmDoneWithToken; // SendNoWait, send the next frame asap
        } else if ((mstp_send_ptr->FrameType == mftTestRequest) ||
                   ((mstp_send_ptr->FrameType == mftBACnetDataExpectingReply) &&
                    (mstp_send_ptr->DestinationAddress != 0xFF))) {
          mnstate = mnsmWaitForReply; // SendAndWait, ok to exit and enter later
          SendFrame(mstp_send_ptr->FrameType, mstp_send_ptr->DestinationAddress,
                    mstp_send_ptr->SourceAddress, mstp_send_ptr->data,
                    mstp_send_ptr->count);
          framecount++;
        } else // UnknownFrameType, drop it, drop it like it's hot
        {
#ifdef EXTRA_DEBUG
          printk(MSTP_MSG "Unknown Frame type in output queue\n");
#endif
          framecount = Nmax_info_frames;
          mnstate = mnsmDoneWithToken;
          transitionnow = true;
          free_entry(mstp_send_ptr);
          return transitionnow;
        }
        free_entry(mstp_send_ptr);
      }
      break;
    }
    break;
  case mnsmWaitForReply:
    if (mstpReadSilenceTimer() >= Treply_timeout) // ReplyTimeout
    {
      framecount = Nmax_info_frames;
      mnstate = mnsmDoneWithToken;
      transitionnow = true;
      return transitionnow;
      // break;
    } else // SilenceTimer < Treply_timeout
    {
      if (ReceivedInvalidFrame == true) // InvalidFrame
      {
        ReceivedInvalidFrame = false;
        mnstate = mnsmDoneWithToken;
        transitionnow = true;
        return transitionnow;
        // break;
      }
      if (ReceivedValidFrame == true) {
        if (DestinationAddress == This_Station) {
          if ((FrameType == mftBACnetDataNotExpectingReply) ||
              (FrameType == mftTestResponse) || // ReceivedReply
              (FrameType == mftReplyPostponed))
          // || or FrameType is an NER proprietary frame
          {
            ReceivedValidFrame = false;
            mnstate = mnsmDoneWithToken;
            transitionnow = true;
            return transitionnow;
            // break;
          }
        } else // UnexpectedFrame
        {
          if (SoleManager == (byte) true) {
#ifdef EXTRA_DEBUG
            printk(MSTP_MSG "ReceivedUnexpectedFrame in PFM state 1096\n");
#endif
            SoleManager = false;
          }
          ReceivedValidFrame = false;
          mnstate = mnsmIdle; // drop token on purpose
          return false;
          // break;
        }
      }
    }
    break;
  case mnsmDoneWithToken:
    if (framecount < Nmax_info_frames) {
      mnstate = mnsmUseToken;
      transitionnow = true;
      return transitionnow;
      // break;
    } else {
      if (tokencount < (Npoll)) // errata, compare with Npoll
      {
        if ((SoleManager == false) &&
            (ns == This_Station)) // NextStationUnknown (Addendum 135-2008v-1)
        {
          ps = (This_Station + 1) % (Nmaxmanager + 1);
          SendFrame(mftPollForManager, ps, This_Station, NULL, 0);
          retrycount = 0;
          mnstate = mnsmPollForManager;
          break;
        }
        if (SoleManager == (byte) true) // SoleManager
        {
          // this first check is to force the next PFM
          // without it, a node can wait up to 300ms at the end
          // of the PFM cycle, and we don't want that since it's
          // a useless wait
          if (!Q_Size(
                  &send_queue)) // correct 300 ms gap after poll for max manager
          {
            framecount = Nmax_info_frames;
            tokencount =
                Npoll; // force the next PFM...now instead of waiting 50 tokens
            return true;
          } else {
            framecount = 0;
            tokencount++;
            mnstate = mnsmUseToken;
            return true;
          }
          // no need to break; here, as we return directly from each the above
          // cases
        }
        // the comparison with NS was removed in the 2008 standard
        if ((SoleManager == false)) // || (ns==((TS+1)%(Nmaxmanager+1))))
                                    // //SendToken
        {
          tokencount++;
          SendFrame(mftToken, ns, This_Station, NULL, 0);
          retrycount = 0;
          eventcount = 0;
          mnstate = mnsmPassToken;
          break;
        }
      } else if ((tokencount >= (Npoll))) // errata, compare with Npoll
      {
        if (ns != (ps + 1) % (Nmaxmanager + 1)) // SendMaintenancePFM
        {
          ps = (ps + 1) % (Nmaxmanager + 1);
          SendFrame(mftPollForManager, ps, This_Station, NULL, 0);
          retrycount = 0;
          mnstate = mnsmPollForManager;
          break;
        } else {
          if (SoleManager == false) // ResetMaintenancePFM
          {
            ps = This_Station;
            SendFrame(mftToken, ns, This_Station, NULL, 0);
            retrycount = 0;
            eventcount = 0;
            tokencount = 1;
            mnstate = mnsmPassToken;
            break;
          } else // SoleManagerRestartMaintenancePFM
          {
            ps = (ns + 1) % (Nmaxmanager + 1);
            SendFrame(mftPollForManager, ps, This_Station, NULL, 0);
            ns = This_Station;
            retrycount = 0;
            tokencount = 0; // Addendum 135-2004d-8
            // eventcount=0;								//Addendum
            // 135-2004d-8
            mnstate = mnsmPollForManager;
            break;
          }
        }
      }
    }
    break;
  case mnsmPassToken:
    if ((mstpReadSilenceTimer() < Tusage_timeoutTP) && // SawTokenUser
        (eventcount > Nmin_octets)) {
      mnstate = mnsmIdle;
      break;
    }
    if ((mstpReadSilenceTimer() >= Tusage_timeoutTP) && // RetrySendToken
        (retrycount < Nretry_token)) {
      retrycount++;
      SendFrame(mftToken, ns, This_Station, NULL, 0);
      eventcount = 0;
      mnstate = mnsmPassToken;
      break;
    }
    if ((mstpReadSilenceTimer() >= Tusage_timeout) &&
        (retrycount >= Nretry_token)) {
      if (This_Station ==
          ((ns + 1) %
           (Nmaxmanager + 1))) // FindNewSuccessorUnknown - Add 135-2012bg-9,
                               // stop node from sending PFM to itself
      {
        ps = ((This_Station + 1) % (Nmaxmanager + 1));
        SendFrame(mftPollForManager, ps, This_Station, NULL, 0);
        ns = This_Station;
        retrycount = 0;
        tokencount = 0;
        mnstate = mnsmPollForManager;
        break;
      } else // FindNewSuccessor
      {
        ps = ((ns + 1) % (Nmaxmanager + 1));
        SendFrame(mftPollForManager, ps, This_Station, NULL, 0);
        ns = This_Station;
        retrycount = 0;
        tokencount = 0;
        mnstate = mnsmPollForManager;
        break;
      }
    }
    break;
  case mnsmNoToken:
    if ((mstpReadSilenceTimer() <
         (Tno_token + (Tslot * This_Station))) && // SawFrame
        (eventcount > Nmin_octets)) {
      mnstate = mnsmIdle;
      break;
    }
    if ((mstpReadSilenceTimer() >=
         ((Tno_token + (Tslot * This_Station)))) && // why this,not in standard
        (eventcount < Nmin_octets) &&
        (ReceivedInvalidFrame == true)) {
      ReceivedInvalidFrame = false;
      mnstate = mnsmIdle;
      break;
    }
    if ((mstpReadSilenceTimer() >=
         ((Tno_token + (Tslot * This_Station)))) && // GenerateToken
        (mstpReadSilenceTimer() <=
         (Tno_token + (Tslot * (This_Station + 1))))) {
      ps = ((This_Station + 1) % (Nmaxmanager + 1));
      SendFrame(mftPollForManager, ps, This_Station, NULL, 0);
      ns = This_Station;
      retrycount = 0;
      tokencount = 0;
      // eventcount=0;
      // //Addendum 135-2004d-8
      mnstate = mnsmPollForManager;
      break;
    }
    if (eventcount > Nmin_octets) {
      mnstate = mnsmIdle; // we missed our slot and another manager is preent
      break;
    }
    break;
  case mnsmPollForManager:
    if (ReceivedValidFrame == true) {
      if ((DestinationAddress == This_Station) && // ReceivedReplyToPFM
          (FrameType == mftReplyToPollForManager)) {
        SoleManager = false;
        ns = SourceAddress;
        eventcount = 0;
        SendFrame(mftToken, ns, This_Station, NULL,
                  0); // pass token to node that replied
        ps = This_Station;
        tokencount = 0;
        retrycount = 0;
        ReceivedValidFrame = false;
        mnstate = mnsmPassToken;
        break;
      }
      if ((DestinationAddress != This_Station) || // ReceivedUnexpectedFrame
          (FrameType != mftReplyToPollForManager)) {
        if (SoleManager == (byte) true) {
#ifdef EXTRA_DEBUG
          printk(MSTP_MSG "ReceivedUnexpectedFrame in PFM state 1290\n");
#endif
          SoleManager = false;
        }
        ReceivedValidFrame = false;
        mnstate = mnsmIdle; // drop token on purpose
        return false;
      }
      break;
    }
    if ((SoleManager == (byte) true) && // SoleManager
        ((mstpReadSilenceTimer() >= Tusage_timeout) ||
         (ReceivedInvalidFrame ==
          true))) // there was no valid reply, use the token
    {
      framecount = 0;
      ReceivedInvalidFrame = false;
      mnstate = mnsmUseToken;
      transitionnow = true;
      return transitionnow;
    }
    // this transition isn't in the standard, at all (no addendum, no errata)
    // the concept here is that we've previously declared SoleManager
    // yet, we've encountered at Nmin_octets of traffic, so therefore
    // there must be another manager on the network. Exit SoleManager and
    // just wait for the network to "naturally" re-sync.
    if ((SoleManager == (byte) true) &&
        (eventcount >
         Nmin_octets)) // SawOtherTransmitter (rejoin network from SoleManager)
    {
#ifdef EXTRA_DEBUG
      printk(MSTP_MSG "Previously SoleManager, but detected other traffic\n");
#endif
      SoleManager =
          false; // we were SoleManager, but other traffic means be silent
      ns = This_Station;
      ps = This_Station;
      framecount = 0;
      eventcount = 0;
      retrycount = 0;
      tokencount = 0;
      mnstate = mnsmIdle; // return to IDLE and wait for a poll to us
      return false;
    }
    if (SoleManager == false) {
      if (((mstpReadSilenceTimer() >= Tusage_timeout) ||
           (ReceivedInvalidFrame == true))) {
        if (ns !=
            This_Station) // DoneWithPFM -- there was no valid reply to the
                          // maintenance poll for a manager at address PS
        {
          eventcount = 0;
          SendFrame(mftToken, ns, This_Station, NULL,
                    0); // pass the token to the next station
          retrycount = 0;
          ReceivedInvalidFrame = false;
          mnstate = mnsmPassToken;
          break;
        }
        if (ns == This_Station) {
          if (This_Station != (ps + 1) % (Nmaxmanager + 1)) // SendNextPFM
          {
            ps = (ps + 1) % (Nmaxmanager + 1);
            SendFrame(mftPollForManager, ps, This_Station, NULL, 0);
            retrycount = 0;
            ReceivedInvalidFrame = false;
            mnstate = mnsmPollForManager;
            break;
          }
          if (This_Station ==
              (ps + 1) % (Nmaxmanager + 1)) // Declare SoleManager
          {
#ifdef EXTRA_DEBUG
            printk(MSTP_MSG "Declared SoleManager\n");
#endif
            SoleManager = true;
            joined_state = 0;
            online = true;
            eventcount = 0;
            framecount = 0;
            ReceivedInvalidFrame = false;
            mnstate = mnsmUseToken;
            transitionnow = true;
            return transitionnow;
          }
        }
      }
    }
    break;
  case mnsmAnswerDataRequest:
    SendFrame(mftReplyPostponed, SourceAddress, This_Station, NULL, 0);
    mnstate = mnsmIdle;
    break;
  default:
    break;
  }
  return transitionnow;
}

/**************************************************************
 * SendFrame
 **************************************************************
 * UINT8 SendFrameType  --> type of frame to send - see defines
 * UINT8 destination  	--> destination address
 * UINT8 source		    --> source address
 * UINT8 *data          --> any data to be sent - may be null
 * unsigned data_len    --> number of bytes of data (up to 501)
 **************************************************************/
static void SendFrame(byte SendFrameType, byte destination, byte src,
                      byte *data, unsigned data_len) {
  byte HeaderCRC; // used for running CRC calculation
  union {
    unsigned short dw;
    byte db[2];
  } u;
  unsigned int i;
  int bytes_written = 0;
  int OutputBufferSize;
  int x;
  if (!mstp_tty || !mstp_tty->ops->write)
    return; /* no backend */
  if (destination == This_Station)
    return; // never send to ourselves

  // anything less than 38400 will have a
  // true delay greater than 2000us which
  // an invalid value for udelay

  if (tty_get_baud_rate(mstp_tty) < 38400) {
    if (mstpReadSilenceTimer() < Tturnaround) {
      x = (Tturnaround * USEC_PER_MSEC) -
          (mstpReadSilenceTimer() * USEC_PER_MSEC);
      while (x > 0) {
        udelay(1);
        x--;
      }
    }
  } else {
    udelay(true_delay);
  }

  // Transmit the preamble octets X'55', X'FF'.
  // As each octet is transmitted, set SilenceTimer to zero.
  OutputBuffer[0] = (UINT8)0x55;
  OutputBuffer[1] = (UINT8)0xFF;
  HeaderCRC = (UINT8)0xFF;
  HeaderCRC = CalcHeaderCRC(SendFrameType, HeaderCRC);
  // Transmit the Frame Type, Destination Address, Source Address,
  // and Data Length octets. Accumulate each octet into HeaderCRC.
  // As each octet is transmitted, set SilenceTimer to zero.
  OutputBuffer[2] = SendFrameType;
  if (SendFrameType == mftToken)
    TX_Token_Count++;
  if (SendFrameType == mftPollForManager)
    TX_PFM_Count++;
  HeaderCRC = CalcHeaderCRC(destination, HeaderCRC);
  OutputBuffer[3] = destination;
  HeaderCRC = CalcHeaderCRC(src, HeaderCRC);
  OutputBuffer[4] = src;

  // use a union here to get the MSB and LSB of DataLen
  u.dw = data_len;
  OutputBuffer[5] = u.db[1];
  HeaderCRC = CalcHeaderCRC(u.db[1], HeaderCRC);
  OutputBuffer[6] = u.db[0];
  HeaderCRC = CalcHeaderCRC(u.db[0], HeaderCRC);
  // Transmit the ones-complement of HeaderCRC. Set SilenceTimer to zero.
  OutputBuffer[7] = ~HeaderCRC;
  OutputBufferSize = 8;

  // If there are data octets, initialize DataCRC to X'FFFF'.
  if (data_len > 0) {
    u.dw = 0xFFFF;
    // Transmit any data octets. Accumulate each octet into DataCRC.
    // As each octet is transmitted, set SilenceTimer to zero.
    memmove(&OutputBuffer[8], data, data_len);
    OutputBufferSize += data_len;
    for (i = 0; i < data_len; i++) {
      u.dw = CalcDataCRC(data[i], u.dw);
    }
    // Transmit the ones-complement of DataCRC, least significant octet first.
    // As each octet is transmitted, set SilenceTimer to zero.
    OutputBuffer[8 + data_len] = ~u.db[0];
    OutputBufferSize++;
    OutputBuffer[9 + data_len] = ~u.db[1];
    OutputBufferSize++;
  }
#ifdef USE_PAD_BYTE
  OutputBuffer[OutputBufferSize] = 0xFF; // pad
  OutputBufferSize++;
#endif
  mstpResetSilenceTimer();
  bytes_written =
      mstp_tty->ops->write(mstp_tty, OutputBuffer, OutputBufferSize);
  // mstp_tty->ops->wait_until_sent(mstp_tty,CalcTXTime((word)OutputBufferSize));
  num_tx_bytes += bytes_written;
  mstpResetSilenceTimer();
  mstpSetSilenceTimer(CalcTXTime((word)bytes_written + 3) * -1);
  return;
}

///////////////////////////////////////////////////////////////////////
//	calculate Header CRC (from BACnet Appendix G)
// in:	dv	data value to accumulate
//		cv	current crc value
// out:	new crc

static byte CalcHeaderCRC(byte dv, byte cv) {
  uint16_t crc;

  crc = cv ^ dv;
  crc = crc ^ (crc << 1) ^ (crc << 2) ^ (crc << 3) ^ (crc << 4) ^ (crc << 5) ^
        (crc << 6) ^ (crc << 7);
  return (octet)((crc & 0xFE) ^ ((crc >> 8) & 1));
}

///////////////////////////////////////////////////////////////////////
//	calculate Data CRC (from BACnet Appendix G)
// in:	dv	data value to accumulate
//		cv	current crc value
// out:	new crc

static word CalcDataCRC(byte dv, word cv) {
  uint16_t crcLow;
  crcLow = (cv & 0xFF) ^ dv;
  return (cv >> 8) ^ (crcLow << 8) ^ (crcLow << 3) ^ (crcLow << 12) ^
         (crcLow >> 4) ^ (crcLow & 0x0F) ^ ((crcLow & 0x0F) << 7);
}

//////////////////////////////////////////////////////////////////////
// Linux Kernel module stuff follows
//
// Timer function, in charge of mstpTimerCallback and administering SilenceTimer
//	and ReplyTimer

static enum hrtimer_restart mstp_timer_function(struct hrtimer *timer) {
  unsigned long t2, diff, msec;
  static unsigned long t1 = 0;
  int st = 0;

  if (t1 == 0)
    t1 = jiffies;
  if (mod_state != STATE_Done) {
    t2 = jiffies;
    mstpTimerCallback();
    diff = (long)t2 - (long)t1;
    msec = jiffies_to_msecs(diff);
    st = mstpReadSilenceTimer();
    st += msec;
    mstpSetSilenceTimer(st);
    ReplyTimer += msec;
    t1 = jiffies; // timestamp of when we left the MNSM, including wait for
                  // transmit
  }
  hrtimer_forward(timer, hrtimer_cb_get_time(timer),
                  ktime_set(0, i64TimeInNsec));
  return enHRTimer;
}

/*
 * Line discipline methods
 */

/*
 * Handle the 'receiver data ready' interrupt.
 * This function is called by the 'tty_io' module in the kernel when
 * a block of MSTP data has been received, which can now be decapsulated
 * and sent on to the receive frame state machine for further processing.
 ************************************************************************
 * Notes:
 * This function is called by the low-level tty driver to send
 * characters received by the hardware to the line discipline for
 * processing.
 * <cp> (char buffer) is a pointer to the buffer of input character received by
 *the device. <fp> (flag buffer) is a pointer to a pointer of flag bytes which
 *indicate whether a character was received with a parity error, etc. Possible
 *fp values:  TTY_NORMAL, TTY_BREAK, TTY_PARITY, TTY_FRAME and TTY_OVERRUN
 *
 */
static int mstp_receive(struct tty_struct *tty, const unsigned char *cp,
                        char *fp, int count) {
  int c = count, i = 0;
  if (!mstp_tty || !mstp_tty->ops->write) {
    count = 0;
    return c; /* no backend */
  }
  if (enHRTimer == HRTIMER_NORESTART) {
    count = 0;
    return c;
  }
  num_rx_bytes += count;
  for (i = 0; i < c; i++) {
    if (fp) {
      switch (fp[i]) {
      case TTY_FRAME:
        num_rx_errors++;
        rx_errors = 1;
        num_fe++;
        break;
      case TTY_PARITY:
        num_rx_errors++;
        rx_errors = 1;
        num_pe++;
        break;
      case TTY_OVERRUN:
        num_rx_errors++;
        rx_errors = 1;
        num_oe++;
        break;
      default:
        break;
      }
    }
    DataAvailable = true;
    RFSM(cp[i]);
    mstpResetSilenceTimer();
  }
  return c;
}

/* Open and close keep track of the tty involved */

static int mstp_open(struct tty_struct *tty) {
  unsigned long flags;
  spin_lock_init(&mstp_tty_lock);
  spin_lock_irqsave(&mstp_tty_lock, flags);
  mstp_tty = tty;

  printk(MSTP_MSG "tty index is %d\n", tty->index);
  mstp_tty->receive_room = maxrcvqsize;
  mstp_tty->ops->flush_buffer(mstp_tty);

  // printk(MSTP_MSG "receive_room=%d\n",tty->receive_room);
  baud = tty_get_baud_rate(mstp_tty);
  // Tturnaround = (40/baud)*1000;
  switch (baud) {
  case 9600:
    Tturnaround = 5;
    true_delay = 4167; // 4.167 ms
    break;
  case 19200:
    Tturnaround = 3;
    true_delay = 2083; // 2.083 ms
    break;
  case 38400:
    Tturnaround = 2;
    true_delay = 1042; // 1.042 ms
    break;
  case 57600:
    true_delay = 694; // 694 us
    Tturnaround = 1;
    break;
  case 115200:
    true_delay = 347; // 347 us
    Tturnaround = 1;
    break;
  default:
    // Tturnaround=1;
    // must be 76800
    baud = 76800;
    Tturnaround = 1;
    true_delay = 521; // 521 us
    break;
  }
  mstpVarInit(0, Tturnaround);
  mstpSetToMSTP(mstp_tty);
  printk(MSTP_MSG "Device %s set to MS/TP @ %d\n", mstp_tty->name, baud);
  spin_unlock_irqrestore(&mstp_tty_lock, flags);
  return 0;
}

static void mstp_close(struct tty_struct *tty) {
  unsigned long flags;
  mod_state = STATE_Done; /* indicate we're done                              */
  hrtimer_cancel(&hr_timer); /* stop the timers after next execution */
  enHRTimer = HRTIMER_NORESTART;
  spin_lock_irqsave(&mstp_tty_lock, flags);
  mstp_tty = NULL;
  spin_unlock_irqrestore(&mstp_tty_lock, flags);
}

static int mstp_custom_ioctl(unsigned int cmd, unsigned long arg) {
  int retVal = 0;
  ktime_t kt;

  // printk(KERN_ERR MSTP_MSG "IOCTL cmd = %d arg = %ld\n",cmd,arg);

  /* First, make sure the command is valid */
  if (_IOC_TYPE(cmd) != MSTP_IOC_MAGIC)
    retVal = -ENOTTY;
  if (_IOC_NR(cmd) > MSTP_MAX_NR)
    retVal = -ENOTTY;
  if (_IOC_NR(cmd) < MSTP_MIN_NR)
    retVal = -ENOTTY;

  // printk(KERN_ERR MSTP_MSG "Custom IOCTL for us\n");

  /* Next, handle the command */
  switch (cmd) {
  case MSTP_IOC_GETONLINE:
    if ((joined_state != 0) || (SoleManager == true))
      retVal = 1;
    break;
  case MSTP_IOC_SETMAXMANAGER: // we should shutdown here and restart for each
                               // of these
    Nmax_manager = arg;
    // printk(MSTP_MSG "Setting Nmax_manager to %d\n",Nmax_manager);
    if (Nmax_manager > 127)
      Nmax_manager = 127;
    retVal = 0;
    break;
  case MSTP_IOC_SETMAXINFOFRAMES:
    Nmax_info_frames = arg;
    // if(Nmax_info_frames > 20) Nmax_info_frames = 20;
    // printk(MSTP_MSG "Setting Nmax_info_frames to %d\n",Nmax_info_frames);
    retVal = 0;
    break;
  case MSTP_IOC_SETMACADDRESS:
    This_Station = (octet)arg;
    // printk(MSTP_MSG "Setting This_Station to %d\n",This_Station);
    if (This_Station > 127)
      This_Station = 127;
    ns = This_Station;
    ps = This_Station;
    retVal = 0;
    mnstate = mnsmInitialize;             // we've been starved of time, restart
    mstpSetSilenceTimer(Tframeabort + 1); // to reset the RFSM
    RFSMstate = rfsmIdle;
    if (enHRTimer == HRTIMER_NORESTART) {
      enHRTimer = HRTIMER_RESTART;
      kt = ktime_set(0, i64TimeInNsec);
      hrtimer_init(&hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
      hrtimer_set_expires(&hr_timer, kt);
      hr_timer.function = &mstp_timer_function;
      hrtimer_start(&hr_timer, kt, HRTIMER_MODE_ABS);
      mod_state = STATE_Ready;
    }
    break;
  case MSTP_IOC_SETTUSAGE:
    Tusage_timeout = arg; // 5 ms is our RX latency from the OS
    if (Tusage_timeout > 35)
      Tusage_timeout =
          35; // Tusage_timeout range is 20-35 (Addendum 135-2016bm-1)
    if (Tusage_timeout < 20)
      Tusage_timeout = 20;
    retVal = 0;
    break;
  case MSTP_IOC_GETMAXMANAGER:
    // return put_user(Nmax_manager,(char*)arg);
    retVal = Nmax_manager;
    break;
  case MSTP_IOC_GETMAXINFOFRAMES:
    // return put_user(Nmax_info_frames,(char*)arg);
    retVal = Nmax_info_frames;
    break;
  case MSTP_IOC_GETMACADDRESS:
    // return put_user(This_Station,(char*)arg);
    retVal = This_Station;
    break;
  case MSTP_IOC_GETTUSAGE:
    retVal = Tusage_timeout - 5 - Tturnaround;
    break;
  case MSTP_IOC_GETVER:
    retVal = ver;
    break;
  default:
    retVal = -ENOIOCTLCMD;
    break;
  }
  if ((retVal == 0) &&
      (cmd !=
       MSTP_IOC_GETONLINE)) // only if the values were set somehow, not got
  {
    mstpVarInit(0, Tturnaround);
  }
  return retVal;
}

/* Handles the incoming tty ioctls and send them to N_TTY */
/* If it's one of our IOCTLs, call mstp_custom_ioctl	  */
static int mstp_ioctl(struct tty_struct *tty, struct file *file,
                      unsigned int cmd, unsigned long arg) {
  int retVal = 0;
  int qcount = 0;
  unsigned long flags;
  flags = mstpShutdownLock(tty);
  /* First, make sure the command is valid */
  if (_IOC_TYPE(cmd) != MSTP_IOC_MAGIC)
    retVal = -ENOTTY;
  if (_IOC_NR(cmd) > MSTP_MAX_NR)
    retVal = -ENOTTY;
  if (_IOC_NR(cmd) < MSTP_MIN_NR)
    retVal = -ENOTTY;

  // printk(KERN_ERR MSTP_MSG "IOCTL for us\n");

  switch (cmd) {
    /* Allow user space to work with the tty port */
    /* Here, we'd define our custom ioctl calls   */
  case TCSETS:
  case TCSETA:
  case TCGETS:
  case TCGETA:
  case TIOCSSERIAL:
  case TIOCSETD:
    retVal =
        n_tty_ioctl_helper(tty, (struct file *)file, cmd, (unsigned long)arg);
    break;
  case FIONREAD: // return the packet count available to user space
    qcount = Q_Size(&receive_queue);
    *(int *)arg = qcount;
    break;
  case MSTP_IOC_SETMAXMANAGER:
  case MSTP_IOC_SETMAXINFOFRAMES:
  case MSTP_IOC_SETMACADDRESS:
  case MSTP_IOC_SETTUSAGE:
  case MSTP_IOC_GETMAXMANAGER:
  case MSTP_IOC_GETMAXINFOFRAMES:
  case MSTP_IOC_GETMACADDRESS:
  case MSTP_IOC_GETTUSAGE:
  case MSTP_IOC_GETVER:
    retVal = mstp_custom_ioctl(cmd, (unsigned long)arg);
    break;
  default:
    retVal =
        n_tty_ioctl_helper(tty, (struct file *)file, cmd, (unsigned long)arg);
    break;
  }

  mstpShutdownUnlock(tty, flags);
  return retVal;
}

/*note that it is the responsibility of the line discpline to provide
 * both the routine for accepting characters from the underlying device
 * driver (via the receive method),and the routine for handling read
 * requests from the read() system call (via the read_chan method).  If you
 * didn't provide the read_chan routine, then a userspace program won't be
 * able to read the data collected by the line discpline, at least not via
 * the read system call.
 */

// TODO -- This needs to do something else, probably reset the state machines
//	 but this is a start...
void mstp_default(void) {
  // Empty queues, reset state machines, etc.
}

/* mstp_read()
 *      Called to retreive one frame of data (if available)
 * Arguments:
 *      tty             pointer to tty instance data
 *      file            pointer to open file object
 *      buf             pointer to returned data buffer
 *      nr              size of returned data buffer
 *      cookie          if non-NULL, this is a continuation read
 *      offset          where to continue reading from (unused in n_tty)
 * Return Value:
 *      Number of bytes returned or error code
 */
static ssize_t mstp_read(struct tty_struct *tty, struct file *file,
                         unsigned char __user *buf, size_t nr, 
                         void **cookie, unsigned long offset) {
  int error = 0;
  ssize_t ret = 0;
  // unsigned long flags;
  static struct mstp_data_t *mstp_receive_ptr;
  unsigned char data[INPUT_BUFFER_SIZE +
                     1]; /* Here's the data! account for source address  */
  if (!mstp_tty)
    return -EIO;
  if (!buf)
    return -EIO;

  if (Q_Size(&receive_queue) > 0) {
    mstp_receive_ptr = Q_PopTail(&receive_queue);
    if (!mstp_receive_ptr) // ==NULL
    {
      printk(MSTP_MSG "Pop Q NULL pointer in mstp_read\n");
      Q_Empty(&receive_queue, Nmax_info_frames);
      return 0;
    }
    if (mstp_receive_ptr->count > nr) {
      // printk(MSTP_MSG "Buffer is too small (%d >
      // %d)\n",mstp_receive_ptr->count,nr);
      error = (ssize_t)-EOVERFLOW;
    } else {
      data[0] = mstp_receive_ptr
                    ->SourceAddress; /* Where did it come from?  The destination
                                        is either broadcast or us */
      memcpy(&data[1], &mstp_receive_ptr->data, mstp_receive_ptr->count);
      error = copy_to_user(buf, data, mstp_receive_ptr->count + 1);
      // printk(MSTP_MSG "Put %d into user space,
      // error=%d\n",mstp_receive_ptr->count+1,error);
      ret = (ssize_t)mstp_receive_ptr->count + 1;
      free_entry(mstp_receive_ptr);
      RecdPacketCounter++;
    }
  } else {
    if (file->f_flags &
        O_NONBLOCK) // if this is set, return, otherwise sit and spin
      return -EAGAIN;
  }
  if (error)
    ret = (ssize_t)error;

  return ret;
}

/* mstp_write()
 *
 * 	Called to write one frame of data (if available)
 *
 * Arguments:
 *
 *      tty             pointer to tty instance data
 *      file            pointer to open file object
 *      buf             pointer to returned data buffer
 *      nr              size of returned data buffer
 *
 * Return Value:
 *
 * 	Number of bytes sent or error code
 *
 * Notes:
 * 	This function expects the following packet format:
 * 	[0] = FrameType, important for the state machines to send
 * 		FrameType is either
 * 		mftBACnetDataExpectingReply -or-
 *		mftBACnetDataNotExpectingReply
 * 	[1] = DestinationAddress -- 1 byte MAC Address
 *   [2] = SourceAddress	-- 1 byte MAC Address 0xFF means use our assigned
 *address [3] = DataLength Byte 1 [4] = DataLength Byte 2 [5] = Data for n bytes
 *
 */
static ssize_t mstp_write(struct tty_struct *tty, struct file *file,
                          const unsigned char *buf, size_t nr) {
  unsigned char data[INPUT_BUFFER_SIZE];
  struct mstp_data_t *mstp_data_ptr;
  size_t i = 0;

  if (!mstp_tty) {
    printk(MSTP_MSG "mstp_write: mstp_tty is NULL\n");
    return -EIO;
  }
  if (nr > INPUT_BUFFER_SIZE) { // too big
    printk(MSTP_MSG "mstp_write: size_t too big\n");
    return -ENOMEM;
  }
  if (nr < 5) {
    printk(MSTP_MSG "mstp_write: size_t too small\n");
    return -ENOMEM; // the received data is too small!
  }
  if ((joined_state == 0) || (SoleManager == true)) {
    // #ifdef EXTRA_DEBUG
    // 		printk(MSTP_MSG "mstp_write: not online yet, fake it\n");
    // #endif
    return nr; // pretend we did it
  }
  for (i = 0; i < nr; i++) {
    data[i] = buf[i];
  }
  if (Q_Size(&send_queue) < Nmax_info_frames) // is there room?
  {
    mstp_data_ptr = alloc_entry(sizeof(struct mstp_data_t));
    if (!mstp_data_ptr) {
      printk(
          MSTP_MSG
          "mstp_write: NULL mstp_data_ptr returned from alloc in mstp_send\n");
      Q_Empty(&send_queue, Nmax_info_frames);
      return -EFAULT;
    }
    mstp_data_ptr->FrameType = data[0];
    // printk(MSTP_MSG "FrameType = %2X\n",mstp_data_ptr->FrameType);
    mstp_data_ptr->DestinationAddress = data[1];
    if (data[2] == 0xFF)
      mstp_data_ptr->SourceAddress = This_Station;
    else
      mstp_data_ptr->SourceAddress = data[2];
    // printk(MSTP_MSG "da = %2X
    // sa=%2X\n",mstp_data_ptr->DestinationAddress,mstp_data_ptr->SourceAddress);
    mstp_data_ptr->count = ((data[3] * 256) + data[4]);
    // printk(MSTP_MSG "dlen = %2X\n",mstp_data_ptr->count);
    if (mstp_data_ptr->count < INPUT_BUFFER_SIZE) {
      memcpy(&mstp_data_ptr->data, &data[5], mstp_data_ptr->count);
      SentPacketCounter++;
      Q_PushHead(&send_queue, mstp_data_ptr); // Add it to our send queue
      return nr;
    } else {
      printk(MSTP_MSG "mstp_write: no room in send queue\n");
      return -ENOMEM;
    }
  } else {
    // printk(MSTP_MSG "mstp_write: max frames exceeded\n");
    return -ENOMEM;
  }
}

static unsigned int mstp_poll(struct tty_struct *tty, struct file *filp,
                              poll_table *wait) {
  return 0;
}

/* mstp_wakeup()
 *
 *    Callback for transmit wakeup. Called when low level
 *    device driver can accept more send data.
 *
 * Arguments:        tty    pointer to associated tty instance data
 * Return Value:    None
 */
static void mstp_wakeup(struct tty_struct *tty) {
  clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
}

/* proc_read - proc_read_mstp
 * proc_read_mstp is the callback function that the kernel calls when
 * there's a read file operation on the /proc file (for example,
 * cat /proc/mstp). The file's data pointer (&mstp_data_out) is
 * passed in the data parameter. You first cast it to the mstp_data_out_t
 * structure. This proc_read function then uses the sprintf function to
 * create a string that is pointed to by the page pointer. The function then
 * returns the length of page. Because mstp_data_out->value is set to
 * "Default", the command cat /proc/BACnet/mstpstatus should return
 * mstp Default
 */

static int proc_show_mstpstatus(struct seq_file *m, void *v) {
  int i = 0;
  unsigned long flags;
  seq_printf(m, "\n%s %s\n", MSTPMODULE_NAME, MSTPMODULE_VERSION);
  seq_printf(m,
             "============================================================\n");
  seq_printf(m, "MS/TP MAC Address:          %d\n", This_Station);
  spin_lock_irqsave(&mstp_tty_lock, flags);
  if (mstp_tty) {
    seq_printf(m, "Baud Rate:                  %d\n",
               tty_get_baud_rate(mstp_tty));
  }
  spin_unlock_irqrestore(&mstp_tty_lock, flags);
  seq_printf(m, "SilenceTimer:               %d\n", mstpReadSilenceTimer());
  seq_printf(m, "Max Manager:                 %d\n", Nmax_manager);
  seq_printf(m, "Max Info Frames:            %d\n", Nmax_info_frames);
  seq_printf(m, "Next Station:               %d\n", ns);
  seq_printf(m, "Poll Station:               %d\n", ps);
  if (RFSMstate == rfsmHeader) {
    seq_printf(m, "RFSM State:                 %s, Index=%d\n",
               rfsm_strings[RFSMstate], Index);
  } else if (RFSMstate == rfsmSkipData)
    seq_printf(m, " RFSM Skipping Data: Index=%d,DataLength=%d\n", Index,
               DataLength);
  else
    seq_printf(m, "RFSM State:                 %s\n", rfsm_strings[RFSMstate]);
  seq_printf(m, "MNSM State:                 %s\n", mnsm_strings[mnstate]);
  seq_printf(m, "Tturnaround:                %d\n", (int)(Tturnaround));
  seq_printf(m, "PFM Timeout:                %d\n", Tusage_timeout);
  seq_printf(m, "TX PFM Count:               %ld\n", TX_PFM_Count);
  seq_printf(m, "RX PFM Count:               %ld\n", RX_PFM_Count);
  seq_printf(m, "Token Usage Timeout:        %d\n", Tusage_timeoutTP);
  seq_printf(m, "TX Token Count:             %ld\n", TX_Token_Count);
  seq_printf(m, "RX Token Count:             %ld\n", RX_Token_Count);
  seq_printf(m, "tokencount/Npoll            %d/%d\n", tokencount, Npoll);
  seq_printf(m, "Event Count:                %d\n", eventcount);
  seq_printf(m, "Total Bytes Received:       %ld\n", num_rx_bytes);
  seq_printf(m, "Total Bytes Sent:           %ld\n", num_tx_bytes);
  seq_printf(m, "Error Count:                %d\n", num_rx_errors);
  seq_printf(m, "Framing Errors:             %d\n", num_fe);
  seq_printf(m, "Parity Errors:              %d\n", num_pe);
  seq_printf(m, "Overrun Errors:             %d\n", num_oe);
  seq_printf(m, "Unknown Errors:             %d\n", num_unkerr);
  seq_printf(m, "Tframe_abort violations:    %d\n", frame_abort_errors);
  seq_printf(m, "Invalid Large Frames:       %ld\n", Num_Invalid_Large_Frames);
  seq_printf(m, "Data CRC Error Count:       %ld\n", numDataCRCErrs);
  seq_printf(m, "Header CRC Error Count:     %ld\n", numHdrCRCErrs);
  if (numHdrCRCErrs > 0) {
    seq_printf(m, "RX Pkt: ");
    for (i = 0; i < 8; i++) {
      seq_printf(m, "%02X ", errb[i]);
    }
    seq_printf(m, "(%02X)\n", errb[8]);
  }
  seq_printf(m, "RX Queue Size:              %d\n", Q_Size(&receive_queue));
  seq_printf(m, "TX Queue Size:              %d\n", Q_Size(&send_queue));
  seq_printf(m, "RX Packets:                 %ld\n", RecdPacketCounter);
  seq_printf(m, "TX Packets:                 %ld\n", SentPacketCounter);
  seq_printf(m, "\n");
  return 0;
}

static int mstp_proc_open(struct inode *inode, struct file *file) {
  return single_open(file, proc_show_mstpstatus, NULL);
}

//static const struct file_operations mstp_fops = {
static const struct proc_ops mstp_fops = {
    .proc_open = mstp_proc_open,
    .proc_read = seq_read,
    .proc_lseek = seq_lseek,
    .proc_release = seq_release,
};

/*
 * Module management
 */

static struct tty_ldisc_ops mstp_ldisc = {
    .owner = THIS_MODULE,
    .name = "mstp",
    .open = mstp_open,
    .close = mstp_close,
    .read = mstp_read,
    .write = mstp_write,
    .ioctl = mstp_ioctl,
    .poll = mstp_poll,
    .receive_buf2 = mstp_receive,
    .write_wakeup = mstp_wakeup,
};

static int __init mstp_init(void) {
  int err;
  /*
   * At module load time, we must register our mouse and line discipline
   */
  err = tty_register_ldisc(N_MSTP, &mstp_ldisc);
  if (err) {
    printk(KERN_ERR MSTP_MSG "can't register line discipline\n");
    return err;
  }

  bacnet_dir = proc_mkdir("BACnet", NULL); /* create BACnet /proc directory */
  if (bacnet_dir == NULL) {
    printk(KERN_ERR MSTP_MSG "can't make /proc/BACnet!\n");
    goto no_bacnet_dir; // unregister the tty if this fails
    return -ENOMEM;
  }
  /* Create the proc entry and make it readable and writeable by all - 0666 */
  if (proc_create("mstpstatus", 0666, bacnet_dir, &mstp_fops) == NULL) {
    printk(KERN_ERR MSTP_MSG "can't make /proc/BACnet/mstpstatus\n");
    goto no_mstp_status; // remove bacnet_dir, unregister the tty
    return -ENOMEM;
  }

  /* init queue */
  Q_Init(&receive_queue);
  Q_Init(&send_queue);
  mod_state = STATE_Ready; // so mstp_open can start the timer!

  /* everything initialized */
  printk(KERN_INFO "%s %s initialized\n", MSTPMODULE_NAME, MSTPMODULE_VERSION);
  return 0;

  // clean up /proc directory if we get a serious error along the way
  // order of clean up is important, note we don't remove mstpdata here because
  // it's the last thing to get created if it failed it didn't get created
no_mstp_status:
  remove_proc_entry(
      "BACnet", NULL); /* remove the proc entry to avoid Bad Things 	*/
no_bacnet_dir:         /* the bacnet proc dir entry failed 			*/
  tty_unregister_ldisc(
      N_MSTP); /* unregister ourselves */

  return -EFAULT;
}

static void __exit mstp_unload(void) {
  static struct mstp_data_t *mstp_tmp_ptr;
  mod_state = STATE_Done;    /* indicate we're done
                              */
  hrtimer_cancel(&hr_timer); /* stop the timers after next execution */
  enHRTimer = HRTIMER_NORESTART;

  // empty our queues
  if (Q_Size(&receive_queue)) /* Empty the receive queue	*/
  {
    mstp_tmp_ptr = Q_PopTail(&receive_queue);
    while (mstp_tmp_ptr) {
      free_entry(mstp_tmp_ptr);
      mstp_tmp_ptr = Q_PopTail(&receive_queue);
    }
  }
  if (Q_Size(&send_queue)) /* Empty the send queue	*/
  {
    mstp_tmp_ptr = Q_PopTail(&send_queue);
    while (mstp_tmp_ptr) {
      free_entry(mstp_tmp_ptr);
      mstp_tmp_ptr = Q_PopTail(&send_queue);
    }
  }

  remove_proc_entry(
      "mstpstatus",
      bacnet_dir); /* remove the proc entry to avoid Bad Things 	*/
  remove_proc_entry(
      "BACnet", NULL); /* remove the proc entry to avoid Bad Things 	*/
  tty_unregister_ldisc(
      N_MSTP); /* unregister ourselves 				*/
  printk(KERN_INFO "%s %s unloaded\n", MSTPMODULE_NAME, MSTPMODULE_VERSION);
}

module_init(mstp_init);
module_exit(mstp_unload);

MODULE_AUTHOR("Coleman Brumley");
MODULE_DESCRIPTION("BACnet MS/TP Serial Line Discipline");
MODULE_LICENSE("GPL");
