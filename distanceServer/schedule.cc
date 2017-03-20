// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/schedule.cc,v $
//
//  Created by:      $Author: rstone $
//
//  Revision:        $Revision: 1.1 $
//
//  Date of revision $Date: 2002/09/14 16:52:31 $
//
//  This file is part of the Robotic Telelabor Project.
//
// ============================================================================
//
//                 ------------- HISTORY -----------------
//
//  $Log: schedule.cc,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.2  1999/11/10 16:59:30  schulz
//  changed to operators from friends to simple static functions
//
//  Revision 1.1  1999/05/25 14:43:50  schulz
//  Moved pthreads and locking stuff from RTLClientServer into libstore.a
//  Added C-functions for locking the database from within C-stuff.
//
//  Revision 1.3  1998/10/01 11:33:29  schulz
//  Inserted some ifdefs to handle differences between the GNU and the SUN compilers
//
//  Revision 1.2  1998/09/30 15:07:19  schulz
//  Several changes to make things compile under gcc 2.7.2
//
//  Revision 1.1  1998/06/19 13:58:00  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.4  1997/10/25 08:30:09  schulz
//  added initialize method to some classes
//  modified matchTest to estimate position and door state seperately
//
//  Revision 1.3  1997/03/04 17:31:48  schulz
//  Removed old declarations for block_wait, which is no longer used
//
//  Revision 1.2  1997/03/04 17:15:25  schulz
//  Added file header and revision history
//
//
// ----------------------------------------------------------------------------

#include <unistd.h>
#include <sys/time.h>
#ifdef __GNUG__
#include <typeinfo>
#endif
#include <iostream.h>
#include <stdio.h>
#include "schedule.hh"


// ---------------------------------------------------------------------------

int
timeval_to_usec(struct timeval* t)
{
    return t->tv_sec * 1000000 + t->tv_usec;
}

void
time_subtract(struct timeval *res, struct timeval *t1, struct timeval *t2)
{
    res->tv_sec = t1->tv_sec - t2->tv_sec;
    res->tv_usec = t1->tv_usec - t2->tv_usec;
    if(res->tv_usec < 0) {
	res->tv_usec += 1000000;
	res->tv_sec -= 1;
    }
    if(res->tv_sec < 0) {
      res->tv_sec = 0;
      res->tv_usec = 0;
    }
}

static int 
operator<(struct timeval first, struct timeval second)
{
    int sec;
    int usec;
    sec = first.tv_sec - second.tv_sec;
    usec = first.tv_usec - second.tv_usec;
    if(usec < 0) 
	sec -= 1;
    return sec < 0;
}

//---------------------------------------------------------------------------


t_scheduler::t_scheduler()
{
    queue.first = queue.last = NULL;
}

t_event*
t_scheduler::schedule(int id, int type, void (*job)(), int priority, int delay)
{
  t_event *new_event = new t_event(type, job, priority, delay);
  queue.insert(new_event);
  return new_event;
}

void
t_scheduler::triggerNextEvent(struct timeval *timeout)
{
    int sec, delay;
    int usec,delta;
    struct timeval now, before, waiting_time,late;
    t_event *next, *current;
    while(1) {
	current = queue.pop();
	while(current && current->due()) {
	    gettimeofday(&before,NULL);
	    current->job();
	    gettimeofday(&now,NULL);
	    time_subtract(&waiting_time, &now, &before);
	    time_subtract(&late, &now, &current->start_time);
	    current->duration.tv_sec = waiting_time.tv_sec;
	    current->duration.tv_usec = waiting_time.tv_usec;	    
	    current->delta_start.tv_sec = late.tv_sec/2;
	    current->delta_start.tv_usec =
		late.tv_usec/2+500000*(late.tv_sec%2);
	    if(current->type == REGULAR) {
	      current->new_start_time();
	    }
	    else {
	      delete current;
	      current = queue.pop();
	    }
	}
	if(current) queue.insert(current);
	next = queue.next();
	if(next) {
	    gettimeofday(&now,NULL);	
	    time_subtract(&waiting_time, &next->start_time, &now);
	    time_subtract(timeout, &waiting_time, &next->delta_start);
	}
	else {
	    timeout->tv_sec = 1;
	    timeout->tv_usec = 0;
	}
	if(timeval_to_usec(timeout) > 0) {
	    return;
	}
    }
}


//---------------------------------------------------------------------------

t_event::t_event(int typ, void (*myjob)(), int mypriority, int mydelay)
{
    type = typ;
    job = myjob;
    next_start_delay = mydelay;
    priority = mypriority;
    new_start_time();
    start_time.tv_usec %= 1000000;
    duration.tv_sec = 0;
    duration.tv_usec = 10000;
    delta_start.tv_sec = 0;
    delta_start.tv_usec = 0;
}

void
t_event::adjust_delay(int newdelay)
{
  next_start_delay = newdelay;
}

void
t_event::new_start_time()
{
  gettimeofday( &insertion_time, NULL);
  delay = next_start_delay;
  start_time.tv_usec = insertion_time.tv_usec + delay; 
  start_time.tv_sec = insertion_time.tv_sec;
  start_time.tv_sec += start_time.tv_usec / 1000000;
  start_time.tv_usec %= 1000000;
}

static int
operator<(t_event first, t_event second)
{
    struct timeval t1, t2;
    time_subtract(&t1, &first.start_time, &first.delta_start);
    time_subtract(&t2, &second.start_time, &second.delta_start);    
    return t1 < t2;
}

//  sooner returns true, if second should start earlier then first finishs  
int
t_event::sooner(struct timeval *now, t_event *first)
{
    int sec;
    int usec;
    struct timeval timediff, timediff2;
    time_subtract(&timediff2, &start_time, now);
    time_subtract(&timediff, &timediff2, &delta_start);
    timediff2.tv_usec = first->duration.tv_usec + 500;
    timediff2.tv_sec = first->duration.tv_sec +
	               timediff2.tv_usec / 1000000;    
    timediff2.tv_usec %= 1000000;
    return timediff < timediff2;
}

// check if event is due

int
t_event::due()
{
    int sec;
    int usec;
    struct timeval now,timediff, timediff2;
    gettimeofday(&now, NULL);
    time_subtract(&timediff2, &start_time, &now);
    time_subtract(&timediff, &timediff2, &delta_start);
    return (timediff.tv_sec < 0 ||
	    timediff.tv_sec == 0 && timediff.tv_usec < 5000);
}

// for debugging purposes

void
t_eventlist::print_list()
{
    t_event* current = first;
    struct timeval now;
    gettimeofday(&now,NULL);
    printf("current queue: %d %d\n", now.tv_sec, now.tv_usec);
    while(current) {
	printf("\t%x: %d, %d %d \n",
	       current,
	       current->priority,
	       current->start_time.tv_sec,
	       current->start_time.tv_usec);
	current = current->next;
    }
}


// insert into ordered list sort on time

void
t_eventlist::insert(t_event* new_event)
{
    t_event* current;
    if(!first) {
	first = last = new_event;
	new_event->prev = new_event->next = NULL;
    }
    else {
	current = first;
	while( current && ( *current < *new_event) ) {
	    if( new_event->type != REGULAR &&
	        current->type == new_event->type) {
	      fprintf(stderr, "scheduler: WARNING, dropped one event\n");
	      delete new_event;
	      return;
	    }
	    current = current->next;
	}
	if(current) {
	    if(current->prev)
		current->prev->next = new_event;
	    else
		first = new_event;
	    new_event->prev = current->prev;
	    current->prev = new_event;
	    new_event->next = current;
	}
	else {
	    last->next = new_event;
	    new_event->next = NULL;
	    new_event->prev = last;
	    last = new_event;
	}
    }
//    print_list();
}


// which event is triggered next

t_event*
t_eventlist::next()
{
    struct timeval now;
    t_event *current = first;
    gettimeofday(&now,NULL);
    while( current && current->next &&
	   current->priority > current->next->priority &&
	   current->next->sooner( &now, current) ) {
	current = current->next;
    }
    return current;
}

void
t_eventlist::remove(t_event* current)
{
    if(current->prev)
	current->prev->next = current->next;
    else
	first = current->next;
    if(current->next)
	current->next->prev = current->prev;
    else 
	last = current->prev;
}


// pop the event which is to be triggered next,
// drop unimportant event, which can not be handled in time.

t_event*
t_eventlist::pop()
{
  struct timeval now;
  t_event *to_remove, *current = first;
  //    print_list();
  if(!first) return NULL;
  gettimeofday(&now,NULL);
  while( current->next &&
	 current->priority > current->next->priority &&
	 current->next->sooner( &now, current) ) {
    remove(current);
    to_remove = current;
    current = current->next;
    //	printf("dropping one event\n");
    if(to_remove->type == REGULAR) {
      
      int delta = timeval_to_usec(&current->duration);
      to_remove->start_time.tv_usec =
	to_remove->insertion_time.tv_usec + delta; 
      to_remove->start_time.tv_sec =
	to_remove->insertion_time.tv_sec;
      to_remove->start_time.tv_sec +=
	to_remove->start_time.tv_usec / 1000000;
      to_remove->start_time.tv_usec %= 1000000;
      if(to_remove->priority > 0) to_remove->priority-=1;
      to_remove->delta_start.tv_sec = 0;
      to_remove->delta_start.tv_usec = 0;	    
      insert(to_remove);
    }
    else {
      delete to_remove;
    }
  }
  remove(current);
  return current;
}

/*
struct timeval test_dur;

t_scheduler *test_scheduler;

void
proc1()
{
    struct timeval now;
    gettimeofday(&now,NULL);
    printf("proc1: time %d, duration (%d %d)\n", now.tv_sec,
	   test_dur.tv_sec, test_dur.tv_usec); 
    test_scheduler->schedule(REGULAR, proc1, 1, 200000, &test_dur);
}

void
proc2()
{
    struct timeval now;
    gettimeofday(&now,NULL);
    printf("proc2: time %d, duration (%d %d)\n", now.tv_sec,
	   test_dur.tv_sec, test_dur.tv_usec); 
    test_scheduler->schedule(REGULAR, proc2, 1, 100000, &test_dur);
}

void
proc3()
{
    struct timeval now;
    gettimeofday(&now,NULL);
    printf("proc3: time %d, duration (%d %d)\n", now.tv_sec,
	   test_dur.tv_sec, test_dur.tv_usec); 
    test_scheduler->schedule(REGULAR, proc3, 1, 128509, &test_dur);
}


main()
{
    test_dur.tv_sec = 0;
    test_dur.tv_usec = 600;
    test_scheduler = new t_scheduler();
    test_scheduler->schedule(REGULAR, proc1, 2, 200000, &test_dur);
    test_scheduler->schedule(REGULAR, proc2, 1, 100000, &test_dur);
    test_scheduler->schedule(REGULAR, proc3, 1, 128509, &test_dur);        
    test_scheduler->triggerNextEvent();
}

*/

