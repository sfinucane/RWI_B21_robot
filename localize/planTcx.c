
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/planTcx.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:40 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: planTcx.c,v $
 * Revision 1.1  2002/09/14 20:45:40  rstone
 * *** empty log message ***
 *
 * Revision 1.23  2000/03/06 20:00:45  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.22  1999/12/16 16:14:00  fox
 * Several preparation changes for angles.
 *
 * Revision 1.21  1999/08/27 22:22:33  fox
 * Better communication for multi robot localization.
 *
 * Revision 1.20  1999/01/11 19:47:53  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.18  1998/11/23 21:19:27  fox
 * Fixed some minor bugs.
 *
 * Revision 1.17  1998/11/17 23:26:24  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.16  1998/10/29 03:45:03  fox
 * Nothing special.
 *
 * Revision 1.15  1998/08/31 22:29:22  wolfram
 * Several changes
 *
 * Revision 1.14  1998/08/24 07:39:50  wolfram
 * final version for Washington
 *
 * Revision 1.13  1998/08/11 23:05:39  wolfram
 * Changes for the Smithsonian museum.
 *
 * WARNING: LOCALIZE now uses only one stddev for Lasers. Laser beams have 25 m
 * Maxrange now.
 * Please use ~wolfram/bee/data/localize/params/laser-0.1stdev with this version
 * of LOCALIZE
 *
 * Revision 1.12  1998/06/12 10:16:34  fox
 * Implemented virutal sensor.
 *
 * Revision 1.11  1998/02/12 15:47:21  derr
 * compiler warnings (unused variable ... ) removed
 *
 * Revision 1.10  1997/10/22 08:50:07  fox
 * Fixed a bug.
 *
 * Revision 1.9  1997/08/16 22:59:51  fox
 * Last version before I change selsection.
 *
 * Revision 1.8  1997/06/03 11:49:22  fox
 * Museum version.
 *
 * Revision 1.7  1997/05/27 07:42:33  fox
 * Nothing special.
 *
 * Revision 1.6  1997/05/26 09:34:16  fox
 * Replaced entropy by information.
 *
 * Revision 1.5  1997/05/09 16:28:18  fox
 * Nothing special.
 *
 * Revision 1.4  1997/04/30 12:25:41  fox
 * Some minor changes.
 *
 * Revision 1.3  1997/03/13 17:36:22  fox
 * Temporary version. Don't use!
 *
 * Revision 1.2  1997/01/30 13:34:13  fox
 * Minor changes.
 *
 * Revision 1.1  1997/01/29 12:36:23  fox
 * New version.
 *
 * Revision 1.10  1997/01/14 16:53:24  fox
 * Removed a very very nasty bug for sensor selection.
 *
 * Revision 1.9  1997/01/10 15:19:23  fox
 * Improved several methods.
 *
 * Revision 1.8  1997/01/08 18:28:20  fox
 * Fixed a bug in setCellList.
 *
 * Revision 1.7  1997/01/08 15:52:57  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.6  1997/01/06 17:38:41  fox
 * Improved version.
 *
 * Revision 1.5  1997/01/03 18:07:47  fox
 * Successfully localized the robot and moved it into several rooms without
 * loosing the position again.
 *
 * Revision 1.4  1997/01/03 10:09:47  fox
 * First version with exploration.
 *
 * Revision 1.3  1996/12/31 11:43:55  fox
 * First version using RANDOM_MODE of COLLI.
 *
 * Revision 1.2  1996/12/31 09:19:24  fox
 * Last version before I switched exploration mode in PLAN to random
 * mode in COLLI.
 *
 * Revision 1.1  1996/12/20 15:47:07  fox
 * Commnication with the plan module.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include "general.h"
#include "script.h"
#include "function.h"
#include "movement.h"
#include "correction.h"
#include "localTcx.h"
#include "planTcx.h"
#include "file.h"
#include "proximity.h"

#include "PLAN-messages.h"
#include "MAP-messages.h"


struct {
  float goalDistance;
  bool goalPointGiven;
  bool active;
} planStatus = { 0.0, 0, 0};


/***********************************************************************
 * Initializes the tcx connection with PLAN and sends an initial map.
 ***********************************************************************/
void
initializePlan( actionInformation* actionInfo)
{
  bool planWasConnected = (PLAN != NULL);
  static bool firstTime = TRUE;

  if ( ! planWasConnected || firstTime) {

    if ( timeExpired( PLAN_CONNECTION_TIMER) < 5.0)
      return;
    else
      resetTimer( PLAN_CONNECTION_TIMER);
    
    /* Send this map to the planning module. */
    if ( connectionEstablished( &PLAN, moduleName[PLAN_MODULE])) {
      
      if ( ! planWasConnected || firstTime) {
	
	firstTime = FALSE;

	/* Send the map to PLAN. */
	sendMapToPlan( &(actionInfo->planMap), INVERTED, 0);
	if (actionInfo->entropyMap.initialized)
	  sendMapToPlan( &(actionInfo->entropyMap), INVERTED, 1);
	if (actionInfo->potentialMap.initialized)
	  sendMapToPlan( &(actionInfo->potentialMap), INVERTED, 2);
	
	/* Inform the planning module about the position of the robot. */
	if ( ! actionInfo->useProbGrid || actionInfo->localMaxima.numberOfCells > 0)
	  sendCorrectionParams( actionInfo->correctionParam,
				PLAN, moduleName[PLAN_MODULE]);
	
	subscribeToPlanStatus();
      }
    }
  }
}

/***********************************************************************
 * Initializes the tcx connection with MAP and sends an initial map.
 ***********************************************************************/
void
initializeMap( actionInformation* actionInfo)
{
  static bool waitForTimer = FALSE;
  bool mapWasConnected = (MAP != NULL);

  if ( ! mapWasConnected) {
    
    if ( timeExpired( MAP_CONNECTION_TIMER) < 5.0)
      return;
    else
      resetTimer( MAP_CONNECTION_TIMER);
    
    if ( actionInfo->onlineMapping) {
      
      /* Get map updates from map. */
      if ( connectionEstablished( &MAP, moduleName[MAP_MODULE])) {
	
	if ( ! mapWasConnected) {
	  MAP_register_auto_update_type reg = {1,0};
	  tcxSendMsg( MAP, "MAP_register_auto_update", &reg);
	}
      }
      return;
    }
    else {
      
      /* Send this map to the planning module. */
      if ( connectionEstablished( &MAP, moduleName[MAP_MODULE])) {
	
	if ( ! mapWasConnected) {
	  waitForTimer = TRUE;
	  setTimer( MAP_TIMER);
	}
	else if ( waitForTimer && (timeExpired( MAP_TIMER) > 1.0)) {
	  /* Send the map to MAP. */
	  fprintf( stderr, "Send map to map.\n");
	  sendMap( &(actionInfo->planMap), INVERTED, MAP, moduleName[MAP_MODULE]);
	  waitForTimer = FALSE;
	}
      }
    }
  }
}


/***********************************************************************
 * Dummy
 ***********************************************************************/
void
PLAN_action_reply_handler( TCX_REF_PTR              ref,
			   PLAN_action_reply_ptr    action)
{;ref=ref;action=action;}



/***********************************************************************
 * Subscribes to PLAN status updates.
 ***********************************************************************/
bool
subscribeToPlanStatus()
{
  if ( connectionEstablished( &PLAN, moduleName[PLAN_MODULE])) {
    PLAN_register_auto_update_type registerType = {1};

    writeLog( "TCX message to PLAN: subscribe.\n");

    tcxSendMsg( PLAN, "PLAN_register_auto_update", &registerType);
    return TRUE;
  }
  else
    return FALSE;
}

/***********************************************************************
 * Handles a PLAN status.
 ***********************************************************************/
void
PLAN_status_reply_handler( TCX_REF_PTR              ref,
			   PLAN_status_reply_ptr    status)
{
  writeLog( "Received status from PLAN.\n");

  planStatus.active         = status->autonomous;
  planStatus.goalPointGiven = status->goal_mode;

  if ( status->num_goals > 0) {
    planStatus.goalDistance   = *(status->goal_distance);
    writeLog( "goal dist: %.1f\n", *(status->goal_distance));
  }
  writeLog( "robot  %.1f %.1f %.1f\n",
	   status->robot_x, status->robot_y, status->robot_orientation);
  writeLog( "mode   %d %d\n",
	   status->goal_mode, status->autonomous);

  tcxFree("PLAN_status_reply", status);
  ref=ref;
}


/***********************************************************************
 * Returns wether the plan module controls the robot.
 ***********************************************************************/
bool
planActive()
{
  return PLAN != NULL && planStatus.active && planStatus.goalPointGiven;
}


/*****************************************************************************
 *****************************************************************************
 * Send the map to the planning module.
 *****************************************************************************
 *****************************************************************************/
bool
sendMapToPlan( probabilityGrid* map, bool inverted, int type)
{
  if ( MAP == NULL) {
    fprintf( stderr, "Send map to plan.\n");
    return sendMapReplyType( map, inverted, PLAN, moduleName[PLAN_MODULE], type);
  }
  else
    fprintf( stderr, "MAP connected: don't send map to plan.\n");
  return FALSE;
}


/*****************************************************************************
 * Removes all goals in the planning module.
 *****************************************************************************/
bool
removeAllGoalPointsInPlan()
{
  if ( connectionEstablished( &PLAN, moduleName[PLAN_MODULE])) {

    writeLog( "TCX message to PLAN: remove all goal points.\n");

    tcxSendMsg ( PLAN, "PLAN_remove_all_goals", NULL);
    return TRUE;
  }

  return FALSE;
}


/*****************************************************************************
 * Send the map to the planning module.
 *****************************************************************************/
bool
sendGoalPointToPlan( realPosition goal)
{
  if ( connectionEstablished( &PLAN, moduleName[PLAN_MODULE])) {

    PLAN_goal_message_type goal_message;

    goal_message.x = goal.x;
    goal_message.y = goal.y;
    goal_message.max_radius = 0.0;        /* dummy */
    goal_message.reward     = 0.0;        /* dummy */
    goal_message.name       = 0;          /* dummy */
    goal_message.add        = 1;          /* dummy */

    writeLog( "TCX message to PLAN: goal point.\n");

    tcxSendMsg ( PLAN, "PLAN_goal_message", &goal_message );
    return TRUE;
  }
  else
    return FALSE;
}




/*****************************************************************************
 * Send the planning module the command to start to move to the goal points.
 *****************************************************************************/
bool
planStartAutonomous()
{
  if ( connectionEstablished( &PLAN, moduleName[PLAN_MODULE])) {

    int autonomousMode = 1;


    writeLog( "TCX message to PLAN: start autonomous.\n");

    tcxSendMsg ( PLAN, "PLAN_start_autonomous_message", &autonomousMode);
    return TRUE;
  }
  else
    return FALSE;
}


/*****************************************************************************
 * Send the planning module the command to start to move to the goal points.
 *****************************************************************************/
bool
planStopAutonomous()
{
  if ( connectionEstablished( &PLAN, moduleName[PLAN_MODULE])) {

    int autonomousMode = 0;

    writeLog( "TCX message to PLAN: stop autonomous.\n");

    tcxSendMsg ( PLAN, "PLAN_stop_autonomous_message", &autonomousMode);
    return TRUE;
  }
  else
    return FALSE;
}


bool
sendParametersToPlan( int type)
{
  if ( connectionEstablished( &PLAN, moduleName[PLAN_MODULE])) {

    PLAN_parameter_message_type parameter;

    /* This is the normal case for PLAN. Values taken from plan.ini. */
    if ( type ==  UNIQUE_POSITION_PARAMETER) {

      /* To adjust plan. */
      parameter.collision_threshold         = 0.65;
      parameter.max_security_dist           = 50.0;
      parameter.max_adjust_angle            = 45.0;

      /* To check  wether goal is reached. */
      parameter.max_goal_distance           = 50.0;

      /* Distance to final target. */
      parameter.max_final_approach_distance = 50.0;

      /* Distance to intermediate target. */
      parameter.max_approach_distance = 135.0;
    }

    /* The map is an overlay of different maps. Let PLAN be less conservative. */
    else {

      /* To adjust plan. */
      parameter.collision_threshold         = 0.4;
      parameter.max_security_dist           = 50.0;
      parameter.max_adjust_angle            = 30.0;

      /* To check  wether goal is reached. */
      parameter.max_goal_distance           = 50.0;

      /* Distance to final target. */
      parameter.max_final_approach_distance = 50.0;

      /* Distance to intermediate target. */
      parameter.max_approach_distance = 50.0;
    }

    writeLog( "TCX message to PLAN: parameter type %d.\n", type);

    tcxSendMsg( PLAN, "PLAN_parameter_message", &parameter);

    return TRUE;
  }
  else
    return FALSE;
}

