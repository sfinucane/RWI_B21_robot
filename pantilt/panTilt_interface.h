
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/pantilt/panTilt_interface.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:26:54 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: panTilt_interface.h,v $
 * Revision 1.1  2002/09/14 15:26:54  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1998/08/23 01:56:21  thrun
 * ?
 *
 * Revision 1.2  1997/02/25 18:12:43  tyson
 * client lib for PANTILT and lots of little stuff
 *
 * Revision 1.1.1.1  1996/09/22 16:46:29  rhino
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


/*****************************************************************************/

#ifndef PANTILT_INTERFACE_LOADED
#define PANTILT_INTERFACE_LOADED

#define USE_OLD_TTY
#include "devUtils.h"

/************************************************************************/
/*  Simplest panTilt commands                                            */
/************************************************************************/
/* PANTILT_init must be called prior to any of the PANTILT commands being 
   utilised. 
   */

typedef struct {
  DEV_TYPE dev;
  /* put in here whatever is needed for starting up the real device.*/
}  PANTILT_TYPE, *PANTILT_PTR;


int PANTILT_init(void);
void PANTILT_outputHnd(int fd, long chars_available);
void PANTILT_terminate(void);

#ifdef DECLARE_PANTILT_VARS

/*
 * the port and baud rates are now overridden by the beeSoft.ini file
 */

PANTILT_TYPE  panTilt_device = 
{ 
  { FALSE,
      { "", DEFAULT_PORT},
      { "/dev/cur3", 13}, 
      PANTILT_DEV_NAME,
      -1,
      TRUE,
      FALSE,			/* debug! */
      (FILE *) NULL,
      (fd_set *) NULL,
      (DEVICE_OUTPUT_HND) PANTILT_outputHnd,
      (void (*)(void)) NULL,  
      (DEVICE_SET_TIMEOUT)  setTimeout,  
      (DEVICE_CANCEL_TIMEOUT) cancelTimeout,
      (void (*)(void)) NULL,  
      {0, 0},
      {LONG_MAX, 0},
      {LONG_MAX, 0},
      (void (*)(void)) NULL
      }
};

#else

extern PANTILT_TYPE  panTilt_device;
extern char failureString[80];

BOOLEAN PANTILT_init(void);
void PANTILT_outputHnd (int fd, long chars_available);
void PANTILT_terminate(void);
BOOLEAN PANTILT_setVelocity (float panVelocity, float tiltVelocity);
BOOLEAN PANTILT_setAcceleration (float panAcceleration, float tiltAcceleration);
BOOLEAN PANTILT_position (DEGREES *panAngle, DEGREES *tiltAngle);
BOOLEAN PANTILT_pan (DEGREES panAngle);
BOOLEAN PANTILT_panRelative (DEGREES panAngle);
BOOLEAN PANTILT_tilt (DEGREES tiltAngle);
BOOLEAN PANTILT_tiltRelative (DEGREES tiltAngle);
BOOLEAN PANTILT_move (DEGREES panAngle, DEGREES tiltAngle);
BOOLEAN PANTILT_moveRelative (DEGREES panAngle, DEGREES tiltAngle);
BOOLEAN PANTILT_reset(void);
void PANTILT_limits (DEGREES *minPanAngle, DEGREES *maxPanAngle,
		     DEGREES *minTiltAngle, DEGREES *maxTiltAngle,
		     float *maxPanVelocity, float *maxTiltVelocity);
#endif

#endif
