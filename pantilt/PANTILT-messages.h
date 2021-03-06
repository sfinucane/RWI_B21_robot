
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/pantilt/PANTILT-messages.h,v $
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
 * $Log: PANTILT-messages.h,v $
 * Revision 1.1  2002/09/14 15:26:54  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1997/02/25 18:12:42  tyson
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



#ifndef PANTILT_messages_defined
#define PANTILT_messages_defined





#include "tcx.h"
#include "pantiltClient.h"  /* get TCX_PANTILT_MODULE_NAME */

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


#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR PANTILT = NULL;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR PANTILT;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** PANTILT data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/



#define PANTILT_init_query_format "int"

#define PANTILT_init_reply_format "int"

#define PANTILT_terminate_format NULL



typedef struct {
  float pan_velocity;
  float tilt_velocity;
} PANTILT_set_velocity_type, *PANTILT_set_velocity_ptr;

#define PANTILT_set_velocity_format "{float, float}"



typedef struct {
  float pan_acceleration;
  float tilt_acceleration;
} PANTILT_set_acceleration_type, *PANTILT_set_acceleration_ptr;

#define PANTILT_set_acceleration_format "{float, float}"





#define PANTILT_position_query_format NULL




typedef struct {
  float pan_pos;
  float tilt_pos;
} PANTILT_position_reply_type, *PANTILT_position_reply_ptr;

#define PANTILT_position_reply_format "{float, float}"





#define PANTILT_pan_format "float"

#define PANTILT_pan_relative_format "float"

#define PANTILT_tilt_format "float"

#define PANTILT_tilt_relative_format "float"




typedef struct {
  float pan_target;
  float tilt_target;
} PANTILT_move_type, *PANTILT_move_ptr;

#define PANTILT_move_format "{float, float}"




typedef struct {
  float pan_delta_target;
  float tilt_delta_target;
} PANTILT_moveRelative_type, *PANTILT_moveRelative_ptr;

#define PANTILT_moveRelative_format "{float, float}"




#define PANTILT_reset_format NULL

#define PANTILT_limits_query_format NULL

#define PANTILT_disconnect_format NULL


typedef struct {
  float pan_min_angle;
  float pan_max_angle;
  float tilt_min_angle;
  float tilt_max_angle;
  float pan_max_velocity;
  float tilt_max_velocity;
} PANTILT_limits_reply_type, *PANTILT_limits_reply_ptr;

#define PANTILT_limits_reply_format "{float, float, float, float, float, float}"


typedef struct {
  float pan_pos;
  float tilt_pos;
  float pan_velocity;
  float tilt_velocity;
  float pan_acceleration;
  float tilt_acceleration;
  int   error;
} PANTILT_status_update_type, *PANTILT_status_update_ptr;

#define PANTILT_status_update_format "{float, float, float, float, float, float, int}"



typedef struct {
  float x;
  float y;
  float height;
} PANTILT_track_point_type, *PANTILT_track_point_ptr;

#define PANTILT_track_point_format "{float, float, float}"

#define PANTILT_stop_tracking_format NULL

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** PANTILT commands - these are the commands/queries understood by PANTILT ****/


#ifdef TCX_define_variables		/* do this exactly once! */


#define PANTILT_messages \
  {"PANTILT_init_query",       PANTILT_init_query_format},\
  {"PANTILT_init_reply",       PANTILT_init_reply_format},\
  {"PANTILT_terminate",        PANTILT_terminate_format},\
  {"PANTILT_set_velocity",     PANTILT_set_velocity_format},\
  {"PANTILT_set_acceleration", PANTILT_set_acceleration_format},\
  {"PANTILT_position_query",   PANTILT_position_query_format},\
  {"PANTILT_position_reply",   PANTILT_position_reply_format},\
  {"PANTILT_pan",              PANTILT_pan_format},\
  {"PANTILT_pan_relative",     PANTILT_pan_relative_format},\
  {"PANTILT_tilt",             PANTILT_tilt_format},\
  {"PANTILT_tilt_relative",    PANTILT_tilt_relative_format},\
  {"PANTILT_move",             PANTILT_move_format},\
  {"PANTILT_moveRelative",     PANTILT_moveRelative_format},\
  {"PANTILT_reset",            PANTILT_reset_format},\
  {"PANTILT_limits_query",     PANTILT_limits_query_format},\
  {"PANTILT_limits_reply",     PANTILT_limits_reply_format},\
  {"PANTILT_status_update",    PANTILT_status_update_format},\
  {"PANTILT_disconnect",       PANTILT_disconnect_format},\
  {"PANTILT_track_point",      PANTILT_track_point_format},\
  {"PANTILT_stop_tracking",    PANTILT_stop_tracking_format}


#endif



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


#ifdef DEFINE_REPLY_HANDLERS


/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with PANTILT ******/




/******* (a) Procedure headers ******/
#ifdef PANTILT_STATIC_HANDLERS
static void
PANTILT_position_reply_handler(TCX_REF_PTR                ref,
			       PANTILT_position_reply_ptr data);

static void
PANTILT_init_reply_handler(TCX_REF_PTR             ref,
			   int                    *data);

static void
PANTILT_limits_reply_handler(TCX_REF_PTR              ref,
			     PANTILT_limits_reply_ptr data);

static void
PANTILT_status_update_handler(TCX_REF_PTR              ref,
			      PANTILT_status_update_ptr data);
#else
void PANTILT_position_reply_handler(TCX_REF_PTR                ref,
				    PANTILT_position_reply_ptr data);

void PANTILT_init_reply_handler(TCX_REF_PTR             ref,
				int                    *data);

void PANTILT_limits_reply_handler(TCX_REF_PTR              ref,
				  PANTILT_limits_reply_ptr data);


void PANTILT_status_update_handler(TCX_REF_PTR              ref,
				  PANTILT_status_update_ptr data);
#endif

/******* (b) Handler array ******/



TCX_REG_HND_TYPE PANTILT_reply_handler_array[] = {
  {"PANTILT_init_reply", "PANTILT_init_reply_handler",
    (void (*)()) PANTILT_init_reply_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_position_reply", "PANTILT_position_reply_handler",
    (void (*)()) PANTILT_position_reply_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_limits_reply", "PANTILT_limits_reply_handler",
    (void (*)()) PANTILT_limits_reply_handler, TCX_RECV_ALL, NULL},
  {"PANTILT_status_update", "PANTILT_status_update_handler",
    (void (*)()) PANTILT_status_update_handler, TCX_RECV_ALL, NULL}
};


#endif

#endif
