
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection/DETECTION-messages.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:44:56 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: DETECTION-messages.h,v $
 * Revision 1.1  2002/09/14 20:44:56  rstone
 * *** empty log message ***
 *
 * Revision 1.9  1998/08/29 21:44:41  fox
 * Changed the detection report and improved stuck detection.
 *
 * Revision 1.8  1998/08/23 22:57:38  fox
 * First version of building maps of humans.
 *
 * Revision 1.7  1997/05/28 09:01:28  wolfram
 * added motion-only mode
 *
 * Revision 1.6  1997/05/09 16:28:38  fox
 * Works quiet fine.
 *
 * Revision 1.5  1997/05/06 14:50:18  fox
 * *** empty log message ***
 *
 * Revision 1.4  1997/05/06 14:22:56  fox
 * Nothing special.
 *
 * Revision 1.3  1997/05/06 08:19:51  fox
 * Added several messages.
 *
 * Revision 1.2  1997/05/05 16:54:03  fox
 * Incorporated graphics from people.c.
 *
 * Revision 1.1.1.1  1997/04/30 16:17:09  fox
 * Module to detect unexpected obstacles.
 *
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/



#ifndef DETECTION_messages_defined
#define DETECTION_messages_defined



#include "tcx.h"

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/* 
   Notice: when including this file, you need to have the flag

   TCX_define_variables

   be defined exactly once. This will allocate memory for the
   module pointer and the message arrays.
*/

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** TCX module name - this is a unique identifier for TCX *******/

#define TCX_DETECTION_MODULE_NAME "DETECTION"


#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR DETECTION;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR DETECTION;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**********************************************************************
 *
 * This message allows to subscribe for regular updates 
 *
 */

#define SEND_EVERY_REPORT 0
#define SEND_ONLY_AT_FIRST_DETECTION 1
typedef struct {
  int subscribe;		     /* n>0=subscribe, 0=unsubscribe */
} DETECTION_register_auto_update_type, *DETECTION_register_auto_update_ptr;

#define DETECTION_register_auto_update_format "{int}"


#define DETECTION_look_for_unexpected_format NULL
#define DETECTION_look_for_motion_format NULL
#define DETECTION_stop_format NULL

/* Modes of the detection. */
#define LOOK_FOR_MOTION     0
#define LOOK_FOR_UNEXPECTED 1
#define STOP_DETECTION 2
#define BUILD_HUMAN_MAP 5

/* Different things to be found. */
#define FOUND_NOTHING   0 
#define FOUND_SOMETHING 1 
#define FOUND_STUCK     2
 
typedef struct {
  int mode;
  int numberOfObstacles;
  float* angles;
  float* distances;
} DETECTION_update_status_reply_type, *DETECTION_update_status_reply_ptr;

#define DETECTION_update_status_reply_format "{int, int, <{float}:2>, <{float}:2>}"



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** DETECTION commands - these are the commands/queries understood by DETECTION ****/


#ifdef TCX_define_variables		/* do this exactly once! */


#define DETECTION_messages \
  {"DETECTION_register_auto_update",        DETECTION_register_auto_update_format},\
  {"DETECTION_look_for_motion",             DETECTION_look_for_motion_format},\
  {"DETECTION_look_for_unexpected",         DETECTION_look_for_unexpected_format},\
  {"DETECTION_stop",                        DETECTION_stop_format},\
  {"DETECTION_update_status_reply",         DETECTION_update_status_reply_format}
#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_REPLY_HANDLERS


/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with DETECTION ******/



/******* (a) Procedure headers ******/


void
DETECTION_update_status_reply_handler( TCX_REF_PTR                 ref,
				       DETECTION_update_status_reply_ptr status);


/******* (b) Handler array ******/



TCX_REG_HND_TYPE DETECTION_reply_handler_array[] = {
  {"DETECTION_update_status_reply", "DETECTION_update_status_reply_handler",
   DETECTION_update_status_reply_handler, TCX_RECV_ALL, NULL}
};


#endif

#endif
