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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/activeAutomat.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 17:06:29 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: activeAutomat.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.13  1998/10/29 03:44:57  fox
 * Nothing special.
 *
 * Revision 1.12  1998/08/31 22:29:18  wolfram
 * Several changes
 *
 * Revision 1.11  1998/07/01 10:45:55  fox
 * Final update of question sensor.
 *
 * Revision 1.10  1998/06/30 13:55:02  fox
 * Updated question sensor.
 *
 * Revision 1.9  1998/02/12 15:47:18  derr
 * compiler warnings (unused variable ... ) removed
 *
 * Revision 1.8  1997/09/09 19:45:10  fox
 * Fixed a bug in finding local maxima and did several changes to active
 * localization.
 *
 * Revision 1.7  1997/06/20 07:36:07  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.6  1997/06/03 11:49:20  fox
 * Museum version.
 *
 * Revision 1.5  1997/05/26 08:47:41  fox
 * Last version before major changes.
 *
 * Revision 1.4  1997/03/20 14:26:48  fox
 * Should be a good version.
 *
 * Revision 1.3  1997/03/13 17:36:19  fox
 * Temporary version. Don't use!
 *
 * Revision 1.2  1997/01/31 17:11:00  fox
 * Integrated laser reply.
 *
 * Revision 1.1  1997/01/29 12:22:59  fox
 * First version of restructured LOCALIZE.
 *
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include "general.h"
#include "activeInternal.h"
#include "activeLocalize.h"

#include "movement.h"
#include "file.h"
#include "question.h"

/* Coordination of COLLI and PLAN. */
#include "localTcx.h"
#include "planTcx.h"
#include "colliTcx.h"
#include "BASE-messages.h"
#include "PLAN-messages.h"

/* Different states. */
#define Dummy_State          -1
#define Initial_State         0
#define Random_Movement_State 1
#define Move_To_Target_State  2
#define Unique_Position_State 3

/* Different actions of the states. */
#define START    0
#define CONTINUE 1
#define QUIT     2

static int currentState = Initial_State;

/* Functions acting on the different states. */
static int initialState(        actionInformation* info, sensingActionMask* mask,
				realCellList* maxPositions, int action);
static int randomMovementState( actionInformation* info, sensingActionMask* mask,
				realCellList* maxPositions, int action);
static int moveToTargetState(   actionInformation* info, sensingActionMask* mask,
				realCellList* maxPositions, int action);
static int uniquePositionState( actionInformation* info, sensingActionMask* mask,
				realCellList* maxPositions, int action);

/* Check wether selection is possible / necessary. */
static bool actionPossible( actionInformation* info, sensingActionMask* mask,
			    realCellList* maxPositions);

static bool
actionNecessary( actionInformation* info, realCellList* maxPositions);

/**************************************************************************
 * Clean up the current state and return to initial state.
 **************************************************************************/
void
resetAutomat()
{
  switch ( currentState) {

  case Initial_State :
    /* Nothing to be done. */
    break;
  case Unique_Position_State:
    /* Already passive. Nothing to be done. */
    break;
  case Random_Movement_State:
    /* Reset colli mode and stop the robot. */
    setColliMode( DEFAULT_MODE);
    stopColli();
    break;
  case Move_To_Target_State:
    /* Reset plan and stop the robot. */
    planStopAutonomous();
    removeAllGoalPointsInPlan();
    stopColli();
    break;
  }

  sendMapToPlan( globalActiveParameters.planMap, INVERTED, 0);
  sendParametersToPlan( UNIQUE_POSITION_PARAMETER);
  currentState = Initial_State;
  return;
}

/**************************************************************************
 * Main update: finite state automat.
 **************************************************************************/
void
computeNextState( actionInformation* info, sensingActionMask* mask)
{
  int nextState;

  /* We only use local maxima above the threshold. The maxima are normalized
   * such that they sum up to 1.0. We have to undo this by mulitplication with
   * originalSumOfProbs. */
  realCellList *localMaxima = &(info->localMaxima);
  realCellList maxPositions;
  int cellCnt;

  maxPositions.numberOfCells = 0;
  maxPositions.originalSumOfProbs = 0.0;
  for ( cellCnt = 0; cellCnt < localMaxima->numberOfCells; cellCnt++)
    if ( localMaxima->cell[cellCnt].prob * localMaxima->originalSumOfProbs
	 >= globalActiveParameters.minProbOfLocalMax) {
      maxPositions.numberOfCells++;
      maxPositions.originalSumOfProbs +=
	localMaxima->cell[cellCnt].prob * localMaxima->originalSumOfProbs;
      maxPositions.cell[cellCnt] = localMaxima->cell[cellCnt];
    }
  normalizeRealCellList( &maxPositions);

  /**************************************************************************
   * Start the state transitions.
   **************************************************************************/
  switch ( currentState) {

    /* Start the robot in a random motion. */
  case Initial_State :

    nextState = initialState( info, mask, &maxPositions, START);

    /* Quit this state and start the next one if necessary. */
    if ( nextState != currentState) {

      initialState( info, mask, &maxPositions, QUIT);

      switch ( nextState) {

      case Random_Movement_State :
	randomMovementState( info, mask, &maxPositions, START);
	break;

      case Move_To_Target_State :
	moveToTargetState( info, mask, &maxPositions, START);
	break;

      case Unique_Position_State :
	uniquePositionState( info, mask, &maxPositions, START);
	break;
      }
      currentState = nextState;
    }
    break;


    /* The position is determined uniquely. Just send the correction parameters
     * to the other modules. */
  case Unique_Position_State :

    nextState = uniquePositionState( info, mask, &maxPositions, CONTINUE);

    /* Quit this state and start the next one if necessary. */
    if ( nextState != currentState) {

      uniquePositionState( info, mask, &maxPositions, QUIT);

      switch ( nextState) {

      case Move_To_Target_State :
	moveToTargetState( info, mask, &maxPositions, START);
	break;

      case Random_Movement_State :
	randomMovementState( info, mask, &maxPositions, START);
	break;
      }
      currentState = nextState;
    }
    break;

    /* No action selected yet. Move the robot randomly. */
  case Random_Movement_State :

    nextState = randomMovementState( info, mask, &maxPositions, CONTINUE);

    /* Quit this state and start the next one if necessary. */
    if ( nextState != currentState) {


      randomMovementState(  info, mask, &maxPositions, QUIT);

      switch ( nextState) {

      case Move_To_Target_State :
	moveToTargetState( info, mask, &maxPositions, START);
	break;

      case Unique_Position_State :
	uniquePositionState( info, mask, &maxPositions, START);
	break;
      }
      currentState = nextState;
    }
    break;

    /* A desired action is selected and PLAN tries to control
     * the robot to this position. */
  case Move_To_Target_State :

    nextState = moveToTargetState( info, mask, &maxPositions, CONTINUE);
    
#ifdef CONSIDER_QUESTION
    setQuestion( info);
#endif

    /* Quit this state and start the next one if necessary. */
    if ( nextState != currentState) {
      
      moveToTargetState( info, mask, &maxPositions, QUIT);

      switch ( nextState) {

      case Random_Movement_State :
	randomMovementState( info, mask, &maxPositions, START);
	break;

      case Unique_Position_State :
	uniquePositionState( info, mask, &maxPositions, START);
	break;
      }
      currentState = nextState;
    }
  }
  broadcastStatusReport( mask, DONT_CONSIDER_PLAN);
}


/***********************************************************************
 ***********************************************************************
 * Different functions for the states of action selection.
 ***********************************************************************
 ***********************************************************************/

/***********************************************************************
 *
 ***********************************************************************/
static int
initialState( actionInformation* info,
	      sensingActionMask* mask,
	      realCellList* maxPositions, int action)
{
  switch ( action) {

  case START :

    fprintf( stderr, "-- Start exploration.\n");
    planStopAutonomous();
    removeAllGoalPointsInPlan();

    if ( ! actionNecessary( info, maxPositions))
      return Unique_Position_State;
    else if ( actionPossible( info, mask, maxPositions))
      return Move_To_Target_State;
    else
      return Random_Movement_State;

  case CONTINUE :

    fprintf( stderr, "Action %d not implemented.\n", action);
    return Dummy_State;

  case QUIT :
    return Dummy_State;
  }
  return Dummy_State;
}


/***********************************************************************
 * No action selected. Robot moves randomly.
 ***********************************************************************/
static int
randomMovementState( actionInformation* info,
		     sensingActionMask* mask,
		     realCellList* maxPositions, int action)
{
  switch ( action) {
  case START :

    fprintf( stderr, "-- Start random movement.\n");

    setColliMode( RANDOM_MODE);
    sendGoalPointToColli( 1000.0, 1000.0);
    return Random_Movement_State;

  case CONTINUE :

    fprintf( stderr, "-- Continue random movement.\n");

    /* Is the action necessary ? */
    if (! actionNecessary( info, maxPositions))
      return Unique_Position_State;

    /* Can we select an action? */
    else if ( actionPossible( info, mask, maxPositions)) {

      return Move_To_Target_State;
    }
    else
      return Random_Movement_State;

  case QUIT :

    fprintf( stderr, "-- Quit random movement.\n");
    setColliMode( DEFAULT_MODE);
    stopColli();
    return Dummy_State;
  }
  return Dummy_State;
}


/***********************************************************************
 * Action selected and PLAN controls robot to the target.
 ***********************************************************************/
static int
moveToTargetState( actionInformation* info,
		   sensingActionMask* mask,
		   realCellList* maxPositions, int action)
{
#define minWaitUntilPlanActive 5

  static int waitUntilPlanActiveCnt;

  switch ( action) {

  case START :

    fprintf( stderr, "-- Start movement to target.\n");

    sendParametersToPlan( UNKNOWN_POSITION_PARAMETER);

    /* Try to find the best action for the localization and start PLAN. */
    computeBestTarget( &(globalActiveParameters.abstractSensors),
		       maxPositions,
		       &(info->positionProbs),
		       &(info->map));

    waitUntilPlanActiveCnt = 0;
    return Move_To_Target_State;

  case CONTINUE :

    fprintf( stderr, "-- Continue movement to target.\n");

    /* Is the action necessary ? */
    if (! actionNecessary( info, maxPositions))
      return Unique_Position_State;

    /* No better action, just keep on if possible. */
    else if ( planActive()
	      || ( ! planActive() && (waitUntilPlanActiveCnt++ < minWaitUntilPlanActive))) {

      if ( connectionEstablished( &PLAN, moduleName[PLAN_MODULE]))
	tcxSendMsg( PLAN, "PLAN_status_query", NULL);

      return Move_To_Target_State;
    }

    /* The robot has reached the target point. Start with random movement. */
    else
      return Random_Movement_State;

  case QUIT :
    fprintf( stderr, "-- Quit movement to target.\n");

    planStopAutonomous();
    removeAllGoalPointsInPlan();
    stopColli();

    return Dummy_State;
  }
  mask=mask;
  return Dummy_State;
}


/***********************************************************************
 * Position of the robot is uniquely determined.
 ***********************************************************************/
static int
uniquePositionState( actionInformation* info,
		     sensingActionMask* mask,
		     realCellList* maxPositions, int action)
{
  switch ( action) {
  case START :

    fprintf( stderr, "-- Start unique position.\n");
    if ( connectionEstablished( &PLAN, moduleName[PLAN_MODULE]))
      sendMapToPlan( globalActiveParameters.planMap, INVERTED, 0);
    return Unique_Position_State;

  case CONTINUE :

    fprintf( stderr, "-- Continue unique position.\n");

    if ( actionNecessary( info, maxPositions)) {

      if ( actionPossible( info, mask, maxPositions))
	return Move_To_Target_State;
      else
	return Random_Movement_State;
    }
    else
      return Unique_Position_State;

  case QUIT :

    fprintf( stderr, "-- Quit unique position.\n");

    setColliMode( DEFAULT_MODE);
    stopColli();
    planStopAutonomous();
    removeAllGoalPointsInPlan();

    return Dummy_State;
  }
  return Dummy_State;
}



/***********************************************************************
 * Action only makes sense if we have at least 2 local maxima and if
 * the summed probability of the maxima is high enough. *
 ***********************************************************************/
static bool
actionPossible( actionInformation* info, sensingActionMask* mask,
		realCellList* maxPositions)
{
  static bool firstTime = TRUE;

  if ( firstTime ||
       (mask->normalizeGrid
       && mask->consider[MOVEMENT]
       && mask->perform[MOVEMENT][INTEGRATE_MOVEMENT])) {

    firstTime = FALSE;

    if ( maxPositions->numberOfCells > 1
	 && maxPositions->numberOfCells <= globalActiveParameters.maxNumberOfMaxima
	 && maxPositions->originalSumOfProbs >= globalActiveParameters.minSumOfProbs) {

      fprintf( stderr, "# ACTION: %d local maxima with sum %f above threshold.\n",
	       maxPositions->numberOfCells,
	       maxPositions->originalSumOfProbs);

      writeLog( "# ACTION: %d local maxima with sum %f above threshold.\n",
	       maxPositions->numberOfCells,
	       maxPositions->originalSumOfProbs);

      return TRUE;
    }
    else
      return FALSE;
  }
  else {

    if ( maxPositions->numberOfCells > 0 && mask->normalizeGrid)
      fprintf( stderr, "# %d local maxima with sum %f (threshold: %f).\n",
	       maxPositions->numberOfCells,
	       maxPositions->originalSumOfProbs,
	       globalActiveParameters.minSumOfProbs);

    return FALSE;
  }
  info=info;
}

/***********************************************************************
 * If there is only one local maximum and its probability is high enough
 * action is not necessary.
 ***********************************************************************/
static bool
actionNecessary( actionInformation* info, realCellList* maxPositions)
{
  if ( maxPositions->numberOfCells == 1
       && maxPositions->originalSumOfProbs >= globalActiveParameters.minSumOfProbs)
    return FALSE;
  else
    return TRUE;
  info=info;
}


