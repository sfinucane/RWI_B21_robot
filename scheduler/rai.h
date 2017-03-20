
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** Welcome!
 *****
 ***** This file is part of the robot control software provided
 ***** by Real World Interface Inc.
 *****
 ***** All copyrights are by Real World Interface Inc., Carnegie
 ***** Mellon University, and the University of Bonn, Germany.
 ***** Removing the copyright information is illegal. Please read
 ***** and make sure you understand the disclaimer below.
 *****
 ***** Contact tyson@rwii.com if you have questions or would
 ***** like to obtain further information.
 *****
 ***** We hope you enjoy working with this software package.
 *****
 *****                      Tyson D. Sawyer and Sebastian Thrun
 *****
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED
 ***** BY APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING
 ***** THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM
 ***** "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR
 ***** IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 ***** OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE
 ***** ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM
 ***** IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME
 ***** THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO
 ***** LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES
 ***** SUSTAINED BY YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM
 ***** TO OPERATE WITH ANY OTHER PROGRAMS OR FAILURE TO CONTROL A
 ***** PHYSICAL DEVICE OF ANY TYPE), EVEN IF SUCH HOLDER OR OTHER
 ***** PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Source file:     $Source: /usr/local/cvs/bee/src/scheduler/rai.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:37:37 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: rai.h,v $
 * Revision 1.1  2002/09/14 15:37:37  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1997/03/14 20:42:20  tyson
 * bug fixes and cleanup
 *
 * Revision 1.1.1.1  1996/09/22 16:46:21  rhino
 * General reorganization of the directories/repository, fusion with the
 * RWI software.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


/*
 * Copyright 1994, Brown University, Providence, RI
 *
 * Permission to use and modify this software and its documentation for
 * any purpose other than its incorporation into a commercial product is
 * hereby granted without fee.  Permission to copy and distribute this
 * software and its documentation only for non-commercial use is also
 * granted without fee, provided, however, that the above copyright notice
 * appear in all copies, that both that copyright notice and this permission
 * notice appear in supporting documentation, that the name of Brown
 * University not be used in advertising or publicity pertaining to
 * distribution of the software without specific, written prior permission,
 * and that the person doing the distribution notify Brown University of
 * such distributions outside of his or her organization. Brown University
 * makes no representations about the suitability of this software for
 * any purpose.  It is provided "as is" without express or implied warranty.
 * Brown University requests notification of any modifications to this
 * software or its documentation.
 *
 * Send the following redistribution information:
 *
 *	Name:
 *	Organization:
 *	Address (postal and/or electronic):
 *
 * To:
 *	Software Librarian
 *	Computer Science Department, Box 1910
 *	Brown University
 *	Providence, RI 02912
 *
 *		or
 *
 *	brusd@cs.brown.edu
 *
 * We will acknowledge all electronic notifications.
 */

/******************************************************/
/*                                                    */
/* rai.h                                              */
/*                                                    */
/* Include file for the module scheduler              */
/* version 0.8                                        */
/* James Kurien   jmk@cs.brown.edu                    */
/*                                                    */
/*                                                    */
/*                                                    */
/* revision history:                                  */
/*  7/15/94 jmk       Changed API to work with msec   */
/*		      rather than sec,usec.  Added    */
/*                    msec functions like rai_time    */
/*                                                    */
/*  6/23/94 jmk       Basic functionality working     */
/*                                                    */
/*                                                    */
/******************************************************/

#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>
#include <fcntl.h> 
#include <limits.h>

#ifndef RAI_H
#define RAI_H
/* Debugging */

/* set these via -DVERBOSE on compiler command line see the make file*/ 
/*
#undef VERBOSE
#undef PROFILE
#undef TIME_CHECK
*/

#define DEFAULT_REPORT_INTERVAL 10

/* Limits */

#define MAX_MODULES 100			/*Max # modules scheduler handles*/

#define TRUE 1
#ifdef FALSE
#undef FALSE
#endif
#define FALSE !TRUE	


#define ERROR -1			/*Error val returned by open,read   */
#define END_OF_TIME LONG_MAX		/*For defining a maximum time event */
#define USEC_PER_SEC 1000000		/*For carry, borrow in macros below */
#define IOFLAGS O_RDONLY 		/*For opening file for select module*/


/* the file where errors go.  Set to stderr by defualt */
extern FILE * RaiError;




/******************************************************/
/*                                                    */
/*  type: RaiModule                                   */
/*                                                    */
/*  See the README file for an overview.              */
/*                                                    */
/*  These should be created via makeModule below,     */
/*  only after RaiInit has been called.               */
/*                                                    */
/*                                                    */
/******************************************************/


typedef struct timeval timeval;
typedef struct RaiModule_struct RaiModule; 

struct RaiModule_struct
{
  char* name;	    /* for tracing and debugging output*/
  
  /* callbacks*/
  void (*select) (RaiModule * );
  void (*timeout) (RaiModule *);
  void (*poll) (RaiModule *);
  void (*shutdown) (RaiModule *);

  int poll_on;
  timeval poll_interval;   /* time between poll events      */
  timeval next_poll;	   /* clock time of next poll event */

  int timeout_on;
  timeval timeout_limit;  /* how far into future to set timeout*/
  timeval next_timeout;	  /* clock time of next timeout event */

  int select_on;
  int  fd;		 /* file descriptor for selects       */

  /* currently not used, but could be set after module is init*/
  int up;


  char trace_on;       /* should we report on this module */

  /* extra time values for profiling & debugging*/
  timeval select_time_limit;	/* max time select callback should use */ 
  timeval poll_time_limit;	/* max time polling callback should use */ 
  timeval timeout_time_limit;	/* max time timeout callback should use */ 

  /* time value for checking polling interval compliance */
  timeval poll_error_limit;	/* time actual,desired poll interval may differ*/

  timeval timeout_time;
  timeval select_time;
  timeval poll_time;

  int timeout_occurrences;
  int select_occurrences;
  int poll_occurrences;

};


/******************************************************/
/*                                                    */
/* See the README for how each member of the API works*/
/*                                                    */
/******************************************************/

#ifdef __cplusplus
extern "C" {
#endif
/* controlling the scheduler itself*/

void RaiInit(void);
void RaiShutdown(void);
void RaiStart(void);

/* cause ^C and kill to call RaiShutdown */
void catchInterrupts(void);

void setErrorStream(FILE * file);

/* manipulating modules to schedule */

RaiModule* makeModule(char* name,void (*shutdown) (RaiModule *));

void addPolling(RaiModule* mod,void (*poll) (RaiModule *),unsigned long msecs);

void addSelect(RaiModule* mod, void (*select) (RaiModule *), int fd);

void addTimeout(RaiModule* mod, void (*timeout) (RaiModule *),unsigned long msecs);



/* reset the timeout alarm offset from the current time */
void resetTimeout(RaiModule* mod);

void setPollingErrorLimit(RaiModule * mod, unsigned long msecs);
void setTimeLimit(RaiModule* mod,timeval* limit,unsigned long msecs);

#define setPollTimeLimit(mod,msecs) \
setTimeLimit(mod,&(mod->poll_time_limit),msecs);

#define setTimeoutTimeLimit(mod,msecs) \
setTimeLimit(mod,&(mod->timeout_time_limit),msecs);

#define setSelectTimeLimit(mod,msecs) \
setTimeLimit(mod,&(mod->select_time_limit),msecs);

unsigned long getRaiTime(void);
void setRaiTime(void);

#define disableTimeout(mod) (mod)->timeout_on = FALSE
#define enableTimeout(mod) (mod)->timeout_on = TRUE
#define disablePolling(mod) (mod)->poll_on = FALSE
#define enablePolling(mod) (mod)->poll_on = TRUE
#define disableSelect(mod) (mod)->select_on = FALSE
#define enableSelect(mod) (mod)->select_on = TRUE



/******************************************************

 Debugging stuff

******************************************************/

void raiTrace(RaiModule*);
void raiUntrace(RaiModule*);

void profileOn();
void profileOff();

void setReportInterval(int);

void timeCheckOn();
void timeCheckOff();



/*********************************************************/
/*                                                       */
/* timeval macros                                        */
/*                                                       */
/*  The timeval structure in sys/times.h, and used by    */
/*  gettimeofday() has a seconds member and a usec member*/
/*  tv_sec and tv_usec.  Therefore, adding two timevals, */
/*  comparing them, etc is a pain.                       */
/*                                                       */
/*  Note timevalToMsecs returns the number of msecs      */
/*  represented by a timeval.  IT WILL OVERFLOW if you   */
/*  send it too large a timeval (a time of about 48 days)*/
/*  For printing intervals, this is fine.  However, for  */
/*  absolute time, we overcome the 48 day limit by using */
/*  init_rai_time to set a "zero" timeval and subtracting*/
/*  this zero from any timeval we use in this macro.	 */
/*                                                       */
/*  These all do the obvious thing except for            */
/*  add_interval.  This is used in debugging to add the  */
/*  amount of time between two timevals to a third       */
/*  timeval.  For example, we take time before and after */
/*  a call, then add the elapsed time to the amount of   */
/*  we have spent on the call to date.                   */
/*                                                       */
/*  So you pass the beginning and ends of ther interval  */
/*  and the difference gets added to the accumulator     */
/*                                                       */
/*********************************************************/


#define msecsToUsecs(msecs,secs,usecs) \
{secs = msecs / 1000;  usecs = (msecs % 1000) * 1000;}

/* Here we add 499 to the usec to get round up.  Otherwise 800 usec  */
/*  contributes 0 msec to the msecs value			     */
#define timevalToMsecs(time) \
                        ((time.tv_sec * 1000) + ((time.tv_usec + 499) / 1000))

#define timeGreater(t1,t2) ((t1.tv_sec > t2.tv_sec) || \
			     ((t1.tv_sec==t2.tv_sec)&&(t1.tv_usec>=t2.tv_usec)))

#define addTime(t1,t2,t3) {t3.tv_usec = t1.tv_usec + t2.tv_usec; 	\
			    t3.tv_sec= t1.tv_sec+t2.tv_sec;		\
			    if (t3.tv_usec >= USEC_PER_SEC)		\
			      {	t3.tv_usec -= USEC_PER_SEC;		\
				t3.tv_sec++;}}

#define subtractTime(t1,t2,t3) {t3.tv_usec =t1.tv_usec - t2.tv_usec; 	\
				 t3.tv_sec=t1.tv_sec - t2.tv_sec;	\
				 if (t3.tv_usec < 0)			\
				   {t3.tv_usec+=USEC_PER_SEC;		\
				    t3.tv_sec--;}}

#define printTime(fle,t) fprintf(fle,"%ld secs, %ld usecs\n",t.tv_sec,t.tv_usec)
#define printTimeAsMsecs(fle,t) \
                          fprintf(fle,"%ld msecs\n",timevalToMsecs(t))
#define resetTime(t) {t.tv_usec=0;t.tv_sec=0;}
#define nullTime(t) (t.tv_usec==0 && t.tv_sec==0)

#define addInterval(begin,end,accum) { subtractTime(accum,begin,accum);	\
					  addTime(accum,end,accum); }	

#ifdef __cplusplus
}
#endif

#endif   /* ifdef RAI_H */
