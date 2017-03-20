
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/LASER-messages.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:25:05 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: LASER-messages.h,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.8  1999/12/06 13:19:05  fox
 * Added time stamps to base and laser report.
 *
 * Revision 1.7  1999/04/18 19:00:05  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
 *
 * Revision 1.6  1999/03/12 23:17:25  fox
 * Adapted to c++ style.
 *
 * Revision 1.5  1998/01/14 00:37:24  thrun
 * New option "-laserserver" lets the base receive laser date from
 * the laserServer. This is the recommended option. It seems to be
 * much more reliable than reading in the data directly.
 *
 * Revision 1.4  1998/01/13 20:56:20  thrun
 * new option "-keyboard" switches off stdin.
 *
 * Revision 1.3  1997/06/25 14:37:13  fox
 * Changed the laser messages.
 *
 * Revision 1.2  1997/03/14 17:48:58  fox
 * Changed laser messages.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:05  rhino
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



#ifndef LASER_messages_defined
#define LASER_messages_defined





#include "tcx.h"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* 
 *  Notice: this file is part of BASE-messages.
 */


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** TCX module name - this is a unique identifier for TCX *******/

#define TCX_LASER_MODULE_NAME "LASER"


#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR LASER;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR LASER;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** LASER data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/

#define FRONT_LASER 0
#define BACK_LASER  1

typedef struct {
  int   f_numberOfReadings;
  int*  f_reading;
  float f_startAngle;
  float f_angleResolution;

  int   r_numberOfReadings;
  int*  r_reading;
  float r_startAngle;
  float r_angleResolution;
  float xPos;
  float yPos;
  float rotPos;
  struct timeval f_timeStamp;
  struct timeval r_timeStamp;
} LASER_laser_reply_type, *LASER_laser_reply_ptr;

#define LASER_laser_reply_format "{int, <{int} : 1>, float, float, int, <{int} : 5>, float, float, float, float, float, {long, long}, {long, long}}"


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** LASER commands - these are the commands/queries understood by LASER ****/


#ifdef TCX_define_variables		/* do this exactly once! */

#define LASER_messages \
  {"LASER_laser_reply",   LASER_laser_reply_format}


#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/




#ifdef DEFINE_REPLY_HANDLERS


/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with LASER ******/




/******* (a) Procedure headers ******/




void LASER_laser_reply_handler(TCX_REF_PTR                ref,
			       LASER_laser_reply_ptr      data);




/******* (b) Handler array ******/



TCX_REG_HND_TYPE LASER_reply_handler_array[] = {
  {"LASER_laser_reply", "LASER_laser_reply_handler",
   (void (*)()) LASER_laser_reply_handler, TCX_RECV_ALL, NULL}

};


#endif

#endif
