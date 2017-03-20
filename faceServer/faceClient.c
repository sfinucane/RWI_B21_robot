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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/faceServer/faceClient.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 21:06:30 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: faceClient.c,v $
 * Revision 1.1  2002/09/14 21:06:30  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1999/12/28 05:51:04  thrun
 * new head control module. by Mike Montemerlo
 *
 * Revision 1.8  1998/01/14 19:38:35  swa
 * Changed the client library so that it re-connects to the server once
 * the pt server dies. Changed the server so that it stays alive once a
 * client dies and allow the client to reconnect.
 *
 * Revision 1.7  1997/07/29 22:44:41  thrun
 * Improved face interface
 *
 * Revision 1.6  1997/07/17 17:31:50  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.5  1997/06/26 23:08:16  tyson
 * I dont remember what
 *
 * Revision 1.4  1997/05/10 14:01:33  tyson
 * Mutiple armServer clients, fixed faceClient.c, added face operation to wander.
 *
 * Revision 1.3  1997/04/11 18:56:54  tyson
 * minor fixes and chasing TCX segv
 *
 * Revision 1.2  1997/03/11 17:16:40  tyson
 * added IR simulation and other work
 *
 * Revision 1.1  1997/02/25 18:12:43  tyson
 * client lib for FACE and lots of little stuff
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#ifndef lint
static char rcsid[] =
"$Id: faceClient.c,v 1.1 2002/09/14 21:06:30 rstone Exp $";
static void *const use_rcsid = ((void)&use_rcsid, (void)&rcsid, 0);
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <ctype.h>
#include <sys/time.h>

#include <tcx.h>
#include <rai.h>
#include <raiClient.h>
#include <bUtils.h>

#define TCX_define_variables
#define DEFINE_REPLY_HANDLERS
#define FACE_STATIC_HANDLERS
#include "FACE-messages.h"
#include "faceClient.h"

extern void tcxRegisterCloseHnd(void (*closeHnd)());

/*
 * This should include hostname and
 * PID or something like that
 */

#define FACE_CLIENT_NAME "faceclient"

static int   faceHaveLock        = 1;
int faceConnected = 0;

void faceSetEyes(int pos) {
#ifdef TCX_debug
  fprintf(stderr, "%s:%s(): %d\n", __FILE__, __FUNCTION__, pos);
#endif

  if (!FACE||faceConnected==0) {
    fprintf(stderr,
	    "%s(%s): faceServer is not connected. \n", __FILE__, __FUNCTION__);
    faceConnect( 0 );

  } else {

    if (FACE && faceHaveLock) {
      tcxSendMsg(FACE, "FACE_set_eyes", &pos);
    }
  }
}

void faceSetBrows(int pos) {
#ifdef TCX_debug
  fprintf(stderr, "%s:%s(): %d\n", __FILE__, __FUNCTION__, pos);
#endif

  if (!FACE||faceConnected==0) {
    fprintf(stderr,
	    "%s(%s): faceServer is not connected. \n", __FILE__, __FUNCTION__);
    faceConnect( 0 );

  } else {

    if (FACE && faceHaveLock) {
      tcxSendMsg(FACE, "FACE_set_brows", &pos);
    }
  }
}

void faceSetMouth(int pos) {
#ifdef TCX_debug
  fprintf(stderr, "%s:%s(): %d\n", __FILE__, __FUNCTION__, pos);
#endif

  if (!FACE||faceConnected==0) {
    fprintf(stderr,
	    "%s(%s): faceServer is not connected. \n", __FILE__, __FUNCTION__);
    faceConnect( 0 );

  } else {

    if (FACE && faceHaveLock) {
      tcxSendMsg(FACE, "FACE_set_mouth", &pos);
    }
  }
}


void faceSetState(int eyes, int brows, int mouth) {
  FACE_state state;

#ifdef TCX_debug
  fprintf(stderr, "%s:%s(): %d %d %d\n", __FILE__, __FUNCTION__, eyes, brows, mouth);
#endif

  if (!FACE||faceConnected==0) {
    fprintf(stderr,
	    "%s(%s): faceServer is not connected. \n", __FILE__, __FUNCTION__);
    faceConnect( 0 );

  } else {
    state.eyes = eyes;
    state.brows = brows;
    state.mouth = mouth;

    if (FACE && faceHaveLock) {
      tcxSendMsg(FACE, "FACE_set_state", &state);
    }
  }
}

void faceAdjustMood(int pos) {
#ifdef TCX_debug
  fprintf(stderr, "%s:%s(): %d\n", __FILE__, __FUNCTION__, pos);
#endif

  if (!FACE||faceConnected==0) {
    fprintf(stderr,
	    "%s(%s): faceServer is not connected. \n", __FILE__, __FUNCTION__);
    faceConnect( 0 );

  } else {

    if (FACE && faceHaveLock) {
      tcxSendMsg(FACE, "FACE_adjust_mood", &pos);
    }
  }
}


void faceAdjustEyes(int pos) {
#ifdef TCX_debug
  fprintf(stderr, "%s:%s(): %d\n", __FILE__, __FUNCTION__, pos);
#endif

  if (!FACE||faceConnected==0) {
    fprintf(stderr,
	    "%s(%s): faceServer is not connected. \n", __FILE__, __FUNCTION__);
    faceConnect( 0 );

  } else {

    if (FACE && faceHaveLock) {
      tcxSendMsg(FACE, "FACE_adjust_eyes", &pos);
    }
  }
}


void faceReset() {
#ifdef TCX_debug
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);
#endif

  if (!FACE||faceConnected==0) {
    fprintf(stderr,
	    "%s(%s): faceServer is not connected. \n", __FILE__, __FUNCTION__);
    faceConnect( 0 );

  } else {

    if (FACE && faceHaveLock) {
      tcxSendMsg(FACE, "FACE_reset", NULL);
    }
  }
}

void
faceRegister(void)
{
  int numberOfMessages;
  int numberOfHandlers;

  TCX_REG_MSG_TYPE messages[] = { FACE_messages }; 
  
  numberOfMessages = sizeof(messages)/sizeof(TCX_REG_MSG_TYPE); 
  numberOfHandlers = 
    sizeof(FACE_reply_handler_array)/sizeof(TCX_REG_HND_TYPE); 

  registerInterface("", numberOfMessages, messages,
		    numberOfHandlers, FACE_reply_handler_array);
}

/*------------------------------------------------------------*/

/* ---------------------------------------------------------
 *
 * hook up to ptServer. if ptServer dies, try reconnecting 
 * every three seconds
 *
 * --------------------------------------------------------*/
int
faceConnect( int wait_till_established )
{
  static struct timeval last_time = {0, 0};
  struct timeval this_time;
  float  time_difference;

  int auto_reply_position = 1;

#if ( defined(G_DEBUG_TCX) )
  fprintf(stderr, "%s: %s\n", __FILE__, __FUNCTION__);
#endif

  gettimeofday(&this_time, NULL);
  time_difference = 
    ((float) (this_time.tv_sec - last_time.tv_sec))
    + (((float) (this_time.tv_usec - last_time.tv_usec))
       /  1000000.0);

  if (time_difference < 3.0)
    return -1;

  if ( wait_till_established ) { /* 1 */

    fprintf(stderr, "FaceClient: Connecting to Face server...\n");
    FACE = tcxConnectModule(TCX_FACE_MODULE_NAME);
    faceConnected = 1;
    fprintf(stderr, "FaceClient: Connected.\n");

  } else {			/* 0 */

    if ( faceConnected == 0 || !FACE ) { /* doppelt haelt besser */
      fprintf(stderr, "FaceClient: Connecting to Face server...\n");
      FACE  = tcxConnectOptional(TCX_FACE_MODULE_NAME);
      if (FACE && faceHaveLock) {
	faceConnected = 1;
	fprintf(stderr, "FaceClient: Connected.\n");
	tcxSendMsg(FACE, "FACE_init_query", &auto_reply_position);
      } else {
	faceConnected = 0;
      }
    }

  }
  
  last_time.tv_sec  = this_time.tv_sec;
  last_time.tv_usec = this_time.tv_usec;
  
  return 0;
}

/*------------------------------------------------------------*/

void
faceDisconnected(void)
{
#ifdef TCX_debug
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);
#endif

  /* need a ptConnected-check here??? */
  FACE = NULL;
  faceConnected = 0;

}

/***************************************************************************
 ***************************************************************************
 ***************************************************************************/


static void FACE_state_reply_handler(TCX_REF_PTR ref, FACE_state_ptr data) {
#ifdef TCX_debug
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);
#endif
  tcxFree("FACE_state_reply", data);

  printf("%d %d %d\n", data->eyes, data->brows, data->mouth);
}

static void FACE_init_reply_handler(TCX_REF_PTR ref, int *data) {
#ifdef TCX_debug
  fprintf(stderr, "%s:%s()\n", __FILE__, __FUNCTION__);
#endif
  tcxFree("FACE_init_reply", data);
}

static void FACE_status_update_handler(TCX_REF_PTR ref, FACE_state_ptr data) {
  printf("%d %d %d\n", data->eyes, data->brows, data->mouth);
}
