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

#include "queue.h"
#include "mstp.h"
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

/* this strcture stores what goes to the upper layers */
struct mstp_data_t {
  unsigned char SourceAddress;
  unsigned char DestinationAddress;
  unsigned char FrameType;
  unsigned char data[INPUT_BUFFER_SIZE]; /* Here's the data! 		*/
  int count;                             /* how much data?			*/
  void *next;
};

///////////////////////////////////////////////////////////////////////
//	Initialize a semaphore
//
// in:	q		the semaphore

void semaClear(queue *q) { rwlock_init(&q->q_mutex); }

///////////////////////////////////////////////////////////////////////
//	Capture a semaphore
//
// in:	q		the semaphore

void semaCapture(queue *q) { read_lock(&q->q_mutex); }

///////////////////////////////////////////////////////////////////////
//	Release a semaphore
//
// in:	q		the semaphore

void semaRelease(queue *q) { read_unlock(&q->q_mutex); }

///////////////////////////////////////////////////////////////////////
//	Initialize a frfifo
//
// in:	q		points to the frfifo to initialize

int Q_Init(queue *q) {
  q->front = q->rear = NULL;
  q->count = 0;
  semaClear(q);

  return True_;
}

///////////////////////////////////////////////////////////////////////
//	Empty a frfifo - suspect it's corrupted
//
// in:	q		points to the frfifo to initialize
//		depth	typically max info frames

void Q_Empty(queue *q, unsigned int depth) {
  static struct mstp_data_t *mstp_tmp_ptr;

  semaCapture(q);

  if (Q_Size(q)) /* Empty the receive queue	*/
  {
    mstp_tmp_ptr = Q_PopTail(q);
    while (depth != 0) {
      depth--;
      if (mstp_tmp_ptr != NULL) {
        free_entry(mstp_tmp_ptr);
      }
      mstp_tmp_ptr = Q_PopTail(q);
    }
  }

  q->front = q->rear = NULL;
  q->count = 0;
  semaRelease(q);
}

///////////////////////////////////////////////////////////////////////
//	Add a packet to a frfifo
//
// in:	q		points to the frfifo to insert into
//		p		points to the packet to be inserted

void PutQ(queue *q, struct mstp_data_t *p) //	***255
{
  struct mstp_data_t *lg; // lg is the last guy in present queue

  semaCapture(q);
  lg = (struct mstp_data_t *)q->rear;
  if (lg == NULL) // p is the first and only guy
    q->front = p; // so front also points to him
  else            // in all other cases, rear guy must point to new guy
    lg->next = p;
  q->rear = p;    // new guy is now the last one
  p->next = NULL; // tail next is NULL
  q->count++;
  semaRelease(q);
}

///////////////////////////////////////////////////////////////////////
//	remove a packet from a frfifo
//
// in:	q		points to the frfifo to remove from
// out:	NULL	it's empty
//		else	pointer to a packet

void *GetQ(queue *q) {
  struct mstp_data_t *fg;              // fg is the first guy in present queue
  semaCapture(q);                      // capture queue semaphore
  fg = (struct mstp_data_t *)q->front; // get a lock first
  if (fg == NULL) {
    semaRelease(q);
    return NULL; // empty
  }
  if (--q->count == 0)
    q->rear = NULL;
  q->front = fg->next;
  semaRelease(q);
  return fg; // return the first one
}

void Q_PushHead(queue *q, void *d) { PutQ(q, d); }

void Q_PushTail(queue *q, void *d) { PutQ(q, d); }

void *Q_PopTail(queue *q) { return GetQ(q); }

int Q_Size(queue *q) { return q->count; };

void *alloc_entry(int size) {
  void *e;
  // the flags are IMPORTANT!!!
  // GFP_ATOMIC
  // Used to allocate memory from interrupt handlers and
  //	other code outside of a process context. Never sleeps.
  // GFP_KERNEL
  // Normal allocation of kernel memory. May sleep.
  // e = kmalloc(size, GFP_KERNEL);
  e = kmalloc(size, GFP_ATOMIC);
  if (e)
    memset(e, 0, size);
  return e;
}

void free_entry(void *e) { kfree(e); }
