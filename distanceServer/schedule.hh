// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/schedule.hh,v $
//
//  Created by:      $Author: rstone $
//
//  Revision:        $Revision: 1.1 $
//
//  Date of revision $Date: 2002/09/14 16:52:31 $
//
// ============================================================================
//
//                 ------------- HISTORY -----------------
//
//  $Log: schedule.hh,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.2  1999/11/10 16:59:30  schulz
//  changed to operators from friends to simple static functions
//
//  Revision 1.1  1998/10/12 13:32:07  schulz
//  This is a complete new version
//
//  Revision 1.1  1998/06/19 13:58:00  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.2  1997/03/04 17:28:43  schulz
//  Added file header to *.hh files
//  Removed some old "Zombie" files
//
//
//
// ----------------------------------------------------------------------------

#ifndef _SCHEDULE_HH
#define _SCHEDULE_HH

#include <sys/time.h>
#define REGULAR 0

class t_event {
public:
  int delay;
  int next_start_delay;
  struct timeval duration;
  struct timeval insertion_time;    
  struct timeval start_time;
  struct timeval delta_start;
  int priority;
  void (*job)();
  int due();
  int sooner(struct timeval*, t_event*);
  void new_start_time();
  t_event(int typ, void (*)(), int priority, int delay);
  t_event *next;
  t_event *prev;
  int type;
  //  friend int operator<(t_event, t_event);
  //  friend int operator<(struct timeval, struct timeval);
  void adjust_delay(int delay);
};

class t_eventlist {
public:
    void insert(t_event*);
    t_event* pop();
    void remove(t_event*);
    t_event* next();
    void print_list();
    t_event* first;
    t_event* last;    
private:
    friend int timeval_to_usec(struct timeval*);
};


class t_scheduler {
public:
    t_scheduler();
    t_event* schedule(int id, int type, void (*)(),
		      int priority, int delay);
    void triggerNextEvent(struct timeval*);    
private:
  t_eventlist queue;
};

int timeval_to_usec(struct timeval* t);
void time_subtract(struct timeval *res, struct timeval *t1, struct timeval *t2);

#ifdef USE_PTHREADS
 DO NOT USE THIS FILE WHEN MULTITHREADING IS ENABLED !!!
#endif

#define MAX_NUMBER_OF_THREADS 1
inline int myThread() {return 0;};

#endif
