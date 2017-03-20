
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/plan/PLAN-messages.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:47:04 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: PLAN-messages.h,v $
 * Revision 1.1  2002/09/14 15:47:04  rstone
 * *** empty log message ***
 *
 * Revision 1.7  1999/07/23 10:04:32  haehnel
 * *** empty log message ***
 *
 * Revision 1.6  1999/05/28 20:30:45  thrun
 * necessary modifications for BeeSoft light
 *
 * Revision 1.5  1999/02/07 20:52:11  thrun
 * inroduced subgoals, modified PLAN_action_reply to return entire plan.
 *
 * Revision 1.4  1998/01/19 04:58:56  swa
 * "{int, int}" was too much for PLAN_register_auto_update_format. It should
 * be "{int}"
 *
 * Revision 1.3  1997/01/10 12:38:50  fox
 * Added another parameter to PLAN_parameter_message.
 *
 * Revision 1.2  1997/01/08 15:54:27  fox
 * Added a message PLAN_parameter_message to set some parameters with other
 * modules.
 * IMPORTANT: maps are updated even if busy > 3.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:30  rhino
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



#ifndef PLAN_messages_defined
#define PLAN_messages_defined




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

#define TCX_PLAN_MODULE_NAME "PLAN"



#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR PLAN;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR PLAN;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** PLAN data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/





typedef struct {
  int subscribe_status;		/* 1=subscribe, 0=unsubscribe */
} PLAN_register_auto_update_type, *PLAN_register_auto_update_ptr;

#define PLAN_register_auto_update_format "{int}"



typedef struct{
  float x;
  float y;
  float max_radius;
  float reward;
  int   name;			/* "name" of the goal (->marker) */
  int   add;			/* 0=delete, 1=create */
} PLAN_goal_message_type, *PLAN_goal_message_ptr;

#define PLAN_goal_message_format "{float, float, float, float, int, int}"





typedef struct{
  float x, y, orientation;
} PLAN_new_robot_pos_message_type, *PLAN_new_robot_pos_message_ptr;

#define PLAN_new_robot_pos_message_format "{float, float, float}"





typedef struct{
  float x, y, orientation;
  int   stuck;
} PLAN_action_query_type, *PLAN_action_query_ptr;
#define PLAN_action_query_format "{float, float, float, int}"


typedef struct{
  float turn;
  float base;
  float goal_x;
  float goal_y;
  float goal_dist;
  int   speed;
  int   final_action;	   /* set 1 iff action will achieve goal pos. */
  int   active_goal_name;  /* nane (number) of the goal selected, -1 if none*/

  int   num_subgoals;		/* new: we will also return subgoals */
  float *subgoals_x;		/* along with their coordinates */
  float *subgoals_y;
  float plan_dist;
} PLAN_action_reply_type, *PLAN_action_reply_ptr;

#define PLAN_action_reply_format "{float, float, float, float, float, int, int, int, int, <float : 9>, <float : 9>, float}"






typedef struct{
  int   type;			/* 0=reset all, 1=new pos, 2=new neg */
  float from_x, from_y, to_x, to_y;
} PLAN_constraints_message_type, *PLAN_constraints_message_ptr;

#define PLAN_constraints_message_format "{int, float, float, float, float}"

#define PLAN_start_autonomous_message_format "int" /* 1=exploration mode */

#define PLAN_stop_autonomous_message_format "int" /* 1=stop the robot,
						   * 0=let go*/

#define PLAN_quit_message_format NULL


#define PLAN_status_query_format NULL
  

typedef struct{
  int   num_goals;

  float *goal_x;
  float *goal_y;
  float *goal_distance;
  int   *goal_name;
  int   *goal_visible;
  
  int   goal_mode;
  int   autonomous;

  float robot_x;
  float robot_y;
  float robot_orientation;
} PLAN_status_reply_type, *PLAN_status_reply_ptr;

#define PLAN_status_reply_format "{int, <float : 1>, <float : 1>, <float : 1>, <int : 1>, <int : 1>, int, int, float, float, float}"


typedef struct{
  float max_security_dist;
  float max_adjust_angle ;
  float collision_threshold;
  float max_goal_distance;
  float max_final_approach_distance;
  float max_approach_distance;
} PLAN_parameter_message_type, *PLAN_parameter_message_ptr;

#define PLAN_parameter_message_format "{float, float, float, float, float, float}"


#define PLAN_remove_all_goals_format NULL


#define PLAN_reset_exploration_table_format NULL

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** PLAN commands - these are the commands/queries understood by PLAN ****/


#ifdef TCX_define_variables		/* do this exactly once! */


#define PLAN_messages \
  {"PLAN_register_auto_update",     PLAN_register_auto_update_format},\
  {"PLAN_goal_message",             PLAN_goal_message_format},\
  {"PLAN_new_robot_pos_message",    PLAN_new_robot_pos_message_format},\
  {"PLAN_action_query",             PLAN_action_query_format},\
  {"PLAN_action_reply",             PLAN_action_reply_format},\
  {"PLAN_constraints_message",      PLAN_constraints_message_format},\
  {"PLAN_start_autonomous_message", PLAN_start_autonomous_message_format},\
  {"PLAN_stop_autonomous_message",  PLAN_stop_autonomous_message_format},\
  {"PLAN_quit_message",             PLAN_quit_message_format},\
  {"PLAN_status_query",             PLAN_status_query_format},\
  {"PLAN_status_reply",             PLAN_status_reply_format},\
  {"PLAN_remove_all_goals",         PLAN_remove_all_goals_format},\
  {"PLAN_reset_exploration_table",  PLAN_reset_exploration_table_format},\
  {"PLAN_parameter_message",        PLAN_parameter_message_format}
#endif



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_REPLY_HANDLERS



/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with PLAN ******/




/******* (a) Procedure headers ******/



void PLAN_action_reply_handler(TCX_REF_PTR              ref,
			       PLAN_action_reply_ptr    action);

void PLAN_status_reply_handler(TCX_REF_PTR              ref,
			       PLAN_status_reply_ptr    status);


/******* (b) Handler array ******/



TCX_REG_HND_TYPE PLAN_reply_handler_array[] = {

  {"PLAN_action_reply", "PLAN_action_reply_handler",
     (void *) PLAN_action_reply_handler, TCX_RECV_ALL, NULL},
  {"PLAN_status_reply", "PLAN_status_reply_handler",
     (void *) PLAN_status_reply_handler, TCX_RECV_ALL, NULL}
};


#endif

#endif
