
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/robot/devUtils.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:55:27 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: devUtils.h,v $
 * Revision 1.1  2002/09/14 15:55:27  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:14  rhino
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







/*****************************************************************************
 * PROJECT: Rhino
 *
 * FILE: devUtils.h
 *
 * ABSTRACT:
 * 
 * This file provides a set for routines for use with device interfaces.
 * A device interface is a set of routines that provides high level access
 * to a device through its device driver.  All device interfaces are required
 * to be able to connect to a socket rather than the device driver. The routine
 * provided in the file are to help with connecting to a simulator.
 *
 * Note : ANSI C headers are used.
 *
 *****************************************************************************/

#ifndef DEVUTIL_LOADED
#define DEVUTIL_LOADED

#include <stdio.h>
#include <sys/types.h>
#include <sys/time.h>
#include "Common.h"
#include "devNames.h"
#include <limits.h>
#define USE_OLD_TTY
/*#include <sys/ttydev.h>*/


/* declare some constants that should be in a system header file. */
#define stdin_fd  0
#define stdout_fd 1
#define stderr_fd 2

/* define any device constants */

/* defaults for communication */
/*#include <sys/ttydev.h>*/

#define DEFAULT_BAUD    B9600
#define DEFAULT_PORT    1621

/* Type declarations */

/*****************************************************************************
 * TYPE : DEV_INTERFACE_TYPE, DEV_INTERFACE_PTR
 * 
 * Usage : this structure is used to pass information to open a socket to
 * a simulated device.  The BOOLEAN field simulator will be non-null if the
 * simulator is to be used.  In such a case, the machine field gives the 
 * name of the machine running the simulator and the portNumber gives the number
 * of the socket.
 *****************************************************************************/

/* A device output handler receives the socket descriptor and the number
 * of bytes to be read.
 */

typedef struct _dev *DEV_PTR;

typedef void (*DEVICE_OUTPUT_HND)(int, long );
typedef void (*DEVICE_SET_TIMEOUT)(DEV_PTR, int);
typedef void (*DEVICE_CANCEL_TIMEOUT)(DEV_PTR);

typedef struct _dev {
  BOOLEAN use_simulator;
  struct {
    char *machine;
    int portNumber;
  } sim_dev;
  struct {
    char *ttyPort;
    int baudCode;   /* one of the codes B0 .. B9600 in ttydev.h */
  } ttydev;
  char *devName;
  int fd;
  BOOLEAN listen;
  BOOLEAN debug;
  FILE *debug_file;
  fd_set *readMask;
  DEVICE_OUTPUT_HND outputHnd;
  void (* timeoutHnd)(void);  
  DEVICE_SET_TIMEOUT  setTimeout;  
  DEVICE_CANCEL_TIMEOUT cancelTimeout;
  void (* pollHnd)(void);
  struct timeval pollInterval;
  struct timeval pollTime;
  struct timeval timeOutTime;
  void (* sigHnd)(void);
  BOOLEAN use_rwi_server;
}  DEV_TYPE;

/* forward declaration for default stdin handler */

void stdin_defaultInputHnd(int fd, long chars_available);

#ifdef DECLARE_DEVUTILS_VARS

DEV_PTR devices[FD_SETSIZE];
fd_set devConnections;

DEV_TYPE    stdin_device = 
{ FALSE,
    { "", DEFAULT_PORT},
    { "Standard in", DEFAULT_BAUD},
    "Standard in",
    stdin_fd,
    TRUE,
    FALSE,
    (FILE *) NULL,
    &devConnections,
    (DEVICE_OUTPUT_HND) stdin_defaultInputHnd,
    (void (*)(void)) NULL,  
    (DEVICE_SET_TIMEOUT)  NULL,  
    (DEVICE_CANCEL_TIMEOUT) NULL,
    (void (*)(void)) NULL,  
    {0, 0},
    {LONG_MAX, 0},
    {LONG_MAX, 0},
    (void (*)(void)) NULL
    };

#else

extern DEV_PTR devices[FD_SETSIZE];
extern fd_set devConnections;

extern DEV_TYPE  stdin_device;

#endif
/* Utility routine */

int connectToSocket(DEV_PTR);

int connectSimulator(DEV_PTR);

int connectTotty(DEV_PTR dev);

void connectDev(DEV_PTR dev);

void disconnectDev(DEV_PTR dev);

void flushChars(DEV_PTR);

BOOLEAN writeN(DEV_PTR, char *, int);

int readN(DEV_PTR, char *, int);

long numChars(int);

void cancelTimeout (DEV_PTR);
void setTimeout (DEV_PTR dev, int seconds);

void devMainLoop(void);

void ProcessDevices(void);

void ProcessSingleDevice(DEV_PTR dev);

void devInit();

void devStartPolling(DEV_PTR dev,
		     struct timeval *interval,
		     void (* handler)(void));

typedef struct { char *buffer;
		 int length;
		 int nextChar;
		 char delimChar;
		 void (*processRoutine)(char *);
	       } LINE_BUFFER_TYPE, *LINE_BUFFER_PTR;

LINE_BUFFER_PTR createLineBuffer(int lineLength, char delimChar,
				 void (*processRoutine)(char *));

void processOutput (DEV_PTR device, LINE_BUFFER_PTR lineBuffer, 
		    int chars_available);

BOOLEAN WaitForResponse(DEV_PTR dev, int *doneFlag, long timeout);

void ExitFromWait(void);

BOOLEAN noinput(void);

/* Enhanced setting possibilities for the devices. Necessary for the
 * laser range finders. */
void m_setrts( int fd);
void m_setparms( int fd,
		char *baudr,
		char *par,
		char *bits,
		int hwf,
		int swf);

#endif
