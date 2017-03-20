#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <iostream.h>

#include "RTL/pthreads_schedule.hh"


static pthread_key_t thread_event;


// next two to be compatible with schedule.cc

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

void
myUsleep(int msec)
{
  struct timespec delay;
  delay.tv_sec = msec / 1000000;
  delay.tv_nsec = 1000 * (msec % 1000000);
  //  cout << "sleeping for " << delay.tv_sec << " " << delay.tv_nsec << endl;
  nanosleep(&delay, &delay);
}

/*
void
myUsleep(int delay)
{
  pthread_cond_t cvp;
  pthread_mutex_t mp;
  struct timespec to;
  struct timeval now;
  int err;
  err = pthread_mutex_init(&mp, NULL);
  err = pthread_cond_init(&cvp, NULL);
  gettimeofday(&now, NULL);
  now.tv_usec += delay;
  now.tv_sec += now.tv_usec / 1000000;
  now.tv_usec %= 1000000;
  to.tv_nsec = now.tv_usec * 1000;
  to.tv_sec = now.tv_sec;
  err = pthread_mutex_lock(&mp);
  while(1) {
    err = pthread_cond_timedwait(&cvp, &mp, &to);
    if(err == ETIMEDOUT) {
      break;
    }
    else
      fprintf(stderr, "cond_timedwait returned for unknown reason (%d)\n", err);
    //exit(0);
  }
  err = pthread_mutex_unlock(&mp);
  err = pthread_cond_destroy(&cvp);
  err = pthread_mutex_destroy(&mp);
}
*/

static void
deleteEvent(void* e)
{
  delete ((t_event*)e);
}


t_scheduler::t_scheduler()
{
  int myPolicy;
  struct sched_param schedparams;
  pthread_getschedparam(pthread_self(), &myPolicy, &schedparams); 
  schedparams.sched_priority = 21;
  pthread_setschedparam(pthread_self(), myPolicy, &schedparams);
  pthread_key_create(&thread_event, deleteEvent);
  t_event *e = new t_event(0, 0, NULL, 0); 
  pthread_setspecific(thread_event, e);
  cout << "Main thread: " << (int) pthread_self() << endl;
}

void *
startJob( void* e)
{
  pthread_setspecific(thread_event, e); 
  if(((t_event*)e)->type == REGULAR) {
    while(1) {
      ((t_event*)e)->job();
      myUsleep(((t_event*)e)->delay);
    }
  }
  else {
    myUsleep(((t_event*)e)->delay);
    ((t_event*)e)->job();
  }
  return NULL;
}

t_event*
t_scheduler::schedule(int id, int type, void (*job)(), int priority, int delay)
{
  pthread_t pthread_id;
  pthread_attr_t thread_attrs;
  struct sched_param schedparams;

  t_event* myevent = new t_event(id, type, job, delay);
  pthread_attr_init(&thread_attrs);
  pthread_attr_getschedparam(&thread_attrs, &schedparams);
  pthread_attr_setdetachstate(&thread_attrs, PTHREAD_CREATE_DETACHED);
  pthread_attr_setscope(&thread_attrs, PTHREAD_SCOPE_PROCESS);

  pthread_create(&pthread_id,
		 &thread_attrs,
		 startJob, myevent);
  return myevent;
}


void
t_scheduler::triggerNextEvent(struct timeval* timeout)
{	
  timeout->tv_usec = 100000;
  timeout->tv_sec = 0;
}


int 
myThread(void)
{
  t_event *e = (t_event*) pthread_getspecific(thread_event);
  return e->id;
}


