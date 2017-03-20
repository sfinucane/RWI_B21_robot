// ---------------------------------------------------------------------------
//
//  Source file:
//    $Source: /usr/local/cvs/bee/src/distanceServer/pthreads_schedule.hh,v $
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
//  $Log: pthreads_schedule.hh,v $
//  Revision 1.1  2002/09/14 16:52:31  rstone
//  *** empty log message ***
//
//  Revision 1.1  1999/05/25 14:43:50  schulz
//  Moved pthreads and locking stuff from RTLClientServer into libstore.a
//  Added C-functions for locking the database from within C-stuff.
//
//  Revision 1.1  1998/06/19 13:57:55  schulz
//  I finally added the RTLClientServer stuff to the Bee-Repository.
//
//  Revision 1.2  1997/03/04 17:28:43  schulz
//  Added file header to *.hh files
//  Removed some old "Zombie" files
//
//
//
// ----------------------------------------------------------------------------

#ifndef _PTHREADS_SCHEDULE_HH 
#define _PTHREADS_SCHEDULE_HH 

#include <pthread.h>
#include <sys/time.h>
#define REGULAR 0

#ifndef USE_PTHREADS
#define USE_PTHREADS
#endif

#define MAX_NUMBER_OF_THREADS 20
int myThread(void);


class t_event {
public:
  t_event(int myId, int mytype, void (*start)(), int mydelay)
    {
      id = myId;
      type = mytype;
      job = start;
      delay = mydelay;
    };
  void adjust_delay(int newdelay) {
    delay = newdelay;
  };

  int id;
  int type;
  void (*job)();
  int delay;
};

class t_scheduler {
public:
  t_scheduler();
  t_event* schedule( int id, int type, void (*)(),
		 int priority, int delay);
  void triggerNextEvent(struct timeval* timeout);
private:
  int num_threads;
};

int timeval_to_usec(struct timeval* t);
void time_subtract(struct timeval *res, struct timeval *t1, struct timeval *t2);


#endif
