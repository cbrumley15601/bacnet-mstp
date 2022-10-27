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

#ifndef QUEUE__H
#define QUEUE__H

#include <linux/spinlock.h>

#ifndef True_
#define True_ 1
#endif

#ifndef False_
#define False_ 0
#endif

typedef struct _queue { 					//a generic fifo
	void	*front;
	void	*rear;	
	void	*next;
	int		count;
	rwlock_t q_mutex;
} queue;

void *alloc_entry(int size);
void free_entry(void *e);

void Q_Empty(queue  *q, unsigned int depth);

int    Q_Init(queue  *q);
int    Q_Size(queue *q);
void  *Q_PopTail(queue *q);
void    Q_PushHead(queue *q, void *d);
void    Q_PushTail(queue *q, void *d);
#endif
