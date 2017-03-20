
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/robotDetect/ROBOT_DETECTION-messages.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 16:40:52 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: ROBOT_DETECTION-messages.h,v $
 * Revision 1.1  2002/09/14 16:40:52  rstone
 * *** empty log message ***
 *
 * Revision 1.4  1999/09/08 22:06:43  fox
 * *** empty log message ***
 *
 * Revision 1.3  1999/09/03 23:02:54  fox
 * Added time stamp.
 *
 * Revision 1.2  1999/08/27 15:24:31  fox
 * Added name of observer robot.
 *
 * Revision 1.1.1.1  1999/04/19 15:07:24  fox
 * New module allowing different robots to detect each other using cameras.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#ifndef ROBOT_DETECTION_messages_defined
#define ROBOT_DETECTION_messages_defined


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

#define TCX_ROBOT_DETECTION_MODULE_NAME "ROBOT_DETECTION"



#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR ROBOT_DETECTION;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR ROBOT_DETECTION;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**********************************************************************
 *
 * This message allows to subscribe for regular updates 
 *
 */

typedef struct {
  int subscribe;		     /* n>0=subscribe, 0=unsubscribe */
} ROBOT_DETECTION_register_auto_update_type, *ROBOT_DETECTION_register_auto_update_ptr;

#define ROBOT_DETECTION_register_auto_update_format "{int}"


typedef struct {
  int   robotDetected;
  float distance;
  float distanceUncertainty;
  float angle;
  float angleUncertainty;
  char* nameOfDetectedRobot;
  char* nameOfObserverRobot;
  struct timeval timeStamp;
  float  currentRotSpeed;
} ROBOT_DETECTION_status_reply_type, *ROBOT_DETECTION_status_reply_ptr;

#define ROBOT_DETECTION_status_reply_format "{int, float, float, float, float, string, string, {long, long}, float}"



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** ROBOT_DETECTION commands - these are the commands/queries understood by ROBOT_DETECTION ****/


#ifdef TCX_define_variables		/* do this exactly once! */


#define ROBOT_DETECTION_messages \
{"ROBOT_DETECTION_register_auto_update", ROBOT_DETECTION_register_auto_update_format},\
{"ROBOT_DETECTION_status_reply",        ROBOT_DETECTION_status_reply_format}

#endif

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_REPLY_HANDLERS


/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with ROBOT_DETECTION ******/



/******* (a) Procedure headers ******/


void
ROBOT_DETECTION_status_reply_handler( TCX_REF_PTR                 ref,
				      ROBOT_DETECTION_status_reply_ptr status);

/******* (b) Handler array ******/

TCX_REG_HND_TYPE ROBOT_DETECTION_reply_handler_array[] = {
  {"ROBOT_DETECTION_status_reply", "ROBOT_DETECTION_status_reply_handler",
   (void (*)()) ROBOT_DETECTION_status_reply_handler, TCX_RECV_ALL, NULL}
};


#endif

#endif
