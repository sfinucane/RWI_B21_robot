
#ifndef _LOCKING_HH
#define _LOCKING_HH

#include <pthread.h>
#include <stdio.h>

#ifdef __cplusplus

class t_RWlock
{
public:
  t_RWlock() {
    pthread_mutex_init(&monitor, NULL);
    pthread_cond_init(&readers_ok, NULL);
    pthread_cond_init(&writer_ok, NULL);
    rwlock = 0;
    waiting_writers = 0;
    callCnt = 0;
  };
  ~t_RWlock() {
    pthread_mutex_destroy(&monitor);
    pthread_cond_destroy(&readers_ok);
    pthread_cond_destroy(&writer_ok);
  };
  inline void readlock() {
    pthread_mutex_lock(&monitor);

    while(rwlock < 0 || waiting_writers) {
      pthread_cond_wait(&readers_ok, &monitor);
    }
    rwlock++;
    //    fprintf(stderr, "0x%x readlocked by %d\n", this, pthread_self());
    pthread_mutex_unlock(&monitor);
  };
  inline void writelock() {
    pthread_mutex_lock(&monitor);

    if(rwlock < 0 && lockedBy == pthread_self()) {
      callCnt++;
    }
    else {
      // if(rwlock != 0)
      //      fprintf(stderr, "%d waiting for lock 0x%x\n", pthread_self(), this);
      while(rwlock != 0) {
	waiting_writers++;
	pthread_cond_wait(&writer_ok, &monitor);
	waiting_writers--;
      }
      rwlock = -1;
      callCnt = 1;
      lockedBy = pthread_self();
      //      fprintf(stderr, "0x%x locked by %d\n", this, pthread_self());
    }
    pthread_mutex_unlock(&monitor);
  };
  inline void unlock() {
    pthread_mutex_lock(&monitor);
    if(rwlock < 0) {
      callCnt--;
      if(callCnt == 0) {
	rwlock = 0;
	//	fprintf(stderr, "0x%x unlocked by %d\n", this, pthread_self());
      }
    }
    else
      rwlock--;

    int ww = (waiting_writers && rwlock == 0);
    int wr = (waiting_writers == 0 && rwlock == 0);
    pthread_mutex_unlock(&monitor);
    if(ww)
      pthread_cond_signal(&writer_ok);
    else if (wr)
      pthread_cond_broadcast(&readers_ok);
  };
private:
  pthread_mutex_t monitor;
  int rwlock;
  int callCnt;
  pthread_cond_t readers_ok;
  unsigned int waiting_writers;
  pthread_cond_t writer_ok;
  pthread_t lockedBy;
};

class t_mutex_guard
{
public:
  t_mutex_guard(t_RWlock& l) : rwlock(l) {
    rwlock.writelock();
  };
  ~t_mutex_guard() {
    rwlock.unlock();
  };
private:
  t_RWlock& rwlock;
};

#endif

#endif
