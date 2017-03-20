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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/activeLocalize.c,v $
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
 * $Log: activeLocalize.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.39  1999/12/16 16:13:58  fox
 * Several preparation changes for angles.
 *
 * Revision 1.38  1999/11/02 18:12:32  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.37  1999/04/29 00:58:27  fox
 * Some minor changes for multi localize.
 *
 * Revision 1.36  1998/11/17 23:26:16  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.35  1998/10/29 03:44:57  fox
 * Nothing special.
 *
 * Revision 1.34  1998/08/31 22:29:18  wolfram
 * Several changes
 *
 * Revision 1.33  1998/08/19 16:33:53  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.32  1998/08/04 19:35:47  wolfram
 * Fixed a bug in activeLocalize.c
 *
 * Revision 1.31  1998/07/01 10:45:57  fox
 * Final update of question sensor.
 *
 * Revision 1.30  1998/06/30 13:55:03  fox
 * Updated question sensor.
 *
 * Revision 1.29  1998/06/12 10:16:22  fox
 * Implemented virutal sensor.
 *
 * Revision 1.28  1997/12/11 17:06:28  fox
 * Added some parameters.
 *
 * Revision 1.27  1997/12/02 15:20:34  fox
 * Nothing remarkable.
 *
 * Revision 1.26  1997/11/25 17:12:42  fox
 * Should work.
 *
 * Revision 1.25  1997/11/21 15:36:02  fox
 * Modifications in graphic
 *
 * Revision 1.24  1997/11/20 12:58:06  fox
 * Version with good sensor selection.
 *
 * Revision 1.23  1997/10/31 15:22:03  fox
 * Fixed a bug.
 *
 * Revision 1.22  1997/10/31 13:11:39  fox
 * Version for active sensing.
 *
 * Revision 1.21  1997/09/26 17:02:07  wolfram
 * Intermediate version, do not use
 *
 * Revision 1.20  1997/09/09 19:45:10  fox
 * Fixed a bug in finding local maxima and did several changes to active
 * localization.
 *
 * Revision 1.19  1997/08/22 04:16:37  fox
 * Final version before IJCAI.
 *
 * Revision 1.18  1997/08/16 22:59:49  fox
 * Last version before I change selsection.
 *
 * Revision 1.17  1997/08/16 21:52:02  wolfram
 * Grid maps can now be cad maps.  Obstacles are enlarged accordingly
 *
 * Revision 1.16  1997/08/09 18:23:30  wolfram
 * Improved computation of angles (aligned readings are weighted
 * according to their distance to the robot.
 *
 * Revision 1.15  1997/08/02 16:51:00  wolfram
 * 1. Changed the order of indexes of the grid (and only of the grid):
 * The order now is grid->prob[rot][x][y].  This results in a significant
 * speed-up for different operations such as normalization and
 * integration of new sensory data.  Reimplemented the ConvolveThirdDim
 * procedure for convolving over rot.
 *
 * 2. Changed the algorithm to detect linear alignments of readings.  Now
 * we use the approach of Lu.
 *
 * 3. Linear alignments of readings is also checked for laser readings.
 *
 * 4. Expected distances are now computed given the simulator map if
 * available.  For that purpose the library libGetDistance is included.
 *
 * 5. Graphic output now displays the simulator map (if available). This
 * concernes the map overlay as well as the robot window.
 *
 * 6. Fixed some minor bugs.
 *
 * 7. Added different parameters to the ini-file (see example.ini).
 *
 * Revision 1.14  1997/07/04 17:29:12  fox
 * Final version before holiday!!!
 *
 * Revision 1.13  1997/06/25 14:16:38  fox
 * Changed laser incorporation.
 *
 * Revision 1.12  1997/06/20 07:36:07  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.11  1997/05/27 07:42:29  fox
 * Nothing special.
 *
 * Revision 1.10  1997/05/26 08:47:42  fox
 * Last version before major changes.
 *
 * Revision 1.9  1997/03/20 14:53:25  fox
 * Removed display of active fields.
 *
 * Revision 1.8  1997/03/19 17:52:40  fox
 * New laser parameters.
 *
 * Revision 1.7  1997/03/17 18:41:12  fox
 * First version with sensor selection in cluttered environments.
 *
 * Revision 1.6  1997/03/14 17:58:15  fox
 * This version should run quite stable now.
 *
 * Revision 1.5  1997/03/13 17:36:19  fox
 * Temporary version. Don't use!
 *
 * Revision 1.4  1997/01/31 17:11:01  fox
 * Integrated laser reply.
 *
 * Revision 1.3  1997/01/30 17:17:23  fox
 * New version with integrated laser.
 *
 * Revision 1.2  1997/01/30 13:34:11  fox
 * Minor changes.
 *
 * Revision 1.1  1997/01/29 12:22:59  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.40  1997/01/23 14:07:20  fox
 * Version of ijcai-submission.
 *
 * Revision 1.39  1997/01/19 23:35:14  fox
 * Fixed a bug in convolve.
 *
 * Revision 1.38  1997/01/19 19:31:15  fox
 * yeah
 *
 * Revision 1.37  1997/01/18 19:41:02  fox
 * Improved action selection.
 *
 * Revision 1.36  1997/01/16 12:42:47  fox
 * Fixed a bug in scan masks.
 *
 * Revision 1.35  1997/01/14 16:53:21  fox
 * Removed a very very nasty bug for sensor selection.
 *
 * Revision 1.34  1997/01/13 16:54:12  fox
 * Nothing special.
 *
 * Revision 1.33  1997/01/10 15:19:22  fox
 * Improved several methods.
 *
 * Revision 1.32  1997/01/08 18:28:19  fox
 * Fixed a bug in setCellList.
 *
 * Revision 1.31  1997/01/08 15:52:54  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.30  1997/01/07 13:14:27  wolfram
 * Movement decisions are made on the basis of a list of real positions
 *
 * Revision 1.29  1997/01/07 09:50:36  fox
 * Added a new parameter.
 *
 * Revision 1.28  1997/01/07 09:38:05  fox
 * Changed the loading of the ini file such that all parameters have to be
 * given only if the corresponding feature has to be used.
 *
 * Revision 1.27  1997/01/06 17:38:40  fox
 * Improved version.
 *
 * Revision 1.26  1997/01/03 18:07:46  fox
 * Successfully localized the robot and moved it into several rooms without
 * loosing the position again.
 *
 * Revision 1.25  1997/01/03 10:09:44  fox
 * First version with exploration.
 *
 * Revision 1.24  1996/12/31 11:43:54  fox
 * First version using RANDOM_MODE of COLLI.
 *
 * Revision 1.23  1996/12/31 09:19:22  fox
 * Last version before I switched exploration mode in PLAN to random
 * mode in COLLI.
 *
 * Revision 1.22  1996/12/20 15:29:36  fox
 * Added four parameters.
 *
 * Revision 1.21  1996/12/13 13:55:36  fox
 * Implemented connection to PLAN.
 *
 * Revision 1.20  1996/12/05 17:18:07  fox
 * First attempt to plan a path to interesting places.
 *
 * Revision 1.19  1996/12/04 14:29:59  fox
 * ok
 *
 * Revision 1.18  1996/12/03 15:40:24  fox
 * ok
 *
 * Revision 1.17  1996/12/03 12:27:39  fox
 * Seems to work with the thin corridor.
 *
 * Revision 1.16  1996/12/02 18:46:25  fox
 * First version with the new expected distances.
 *
 * Revision 1.15  1996/12/02 10:32:03  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.14  1996/11/29 15:33:36  fox
 * ok
 *
 * Revision 1.13  1996/11/29 11:03:11  fox
 * change movement field.
 *
 * Revision 1.12  1996/11/29 09:31:39  fox
 * ok
 *
 * Revision 1.11  1996/11/28 17:56:21  fox
 * *** empty log message ***
 *
 * Revision 1.10  1996/11/28 12:40:29  wolfram
 * Wolframs version.
 *
 * Revision 1.9  1996/11/28 09:05:49  fox
 * nicks spezielles.
 *
 * Revision 1.8  1996/11/27 15:51:16  fox
 * For Woflram.
 *
 * Revision 1.7  1996/11/27 12:18:17  fox
 * Nothing special.
 *
 * Revision 1.6  1996/11/26 16:08:25  fox
 * Nothing special.
 *
 * Revision 1.5  1996/11/26 11:08:11  fox
 * Improved version.
 *
 * Revision 1.4  1996/11/25 19:35:40  fox
 * Test version for decisions of movements.
 *
 * Revision 1.3  1996/11/25 09:47:23  wolfram
 * Added graphic support for movment errors
 * Improved procedure for computing the entropy of a grid
 *
 * Revision 1.2  1996/11/22 16:33:46  fox
 * First version.
 *
 * Revision 1.1  1996/11/21 14:28:50  fox
 * Decides which action to perform next.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include "general.h"
#include "activeLocalize.h"
#include "activeInternal.h"
#include "selection.h"
#include "file.h"
#include "graphic.h"
#include "allocate.h"
#include "sonar.h"
#include "laser.h"

#include "PLAN-messages.h"
#include "localTcx.h"
#include "planTcx.h"
#include "dynamic.h"

#define UTILITY_WEIGHT 1.0
#define COST_WEIGHT 1.0

#define MAX_NUMBER_OF_MAXIMA_TOKEN          0
#define MIN_SUM_OF_PROBS_TOKEN              1
#define MIN_PROB_OF_LOCAL_MAX_TOKEN         2
#define MIN_DIST_FROM_CURRENT_POS_TOKEN     3
#define SHRINK_MAXIMA_SIZE_TOKEN            4
#define MAX_COST_OF_POSITION_TOKEN          5
#define QUOTA_OF_AREA_TOKEN                 6
#define GO_TO_GOAL_TOKEN                    7
#define GOAL_X_TOKEN                        8
#define GOAL_Y_TOKEN                        9
#define SHOW_FIELDS_TOKEN                   10
#define MIN_NUMBER_OF_SELECTED_SENSORS_TOKEN 11
#define MAX_NUMBER_OF_SELECTED_SENSORS_TOKEN 12
#define USE_MEASURED_FEATURES_TOKEN          13
#define CONSIDER_INACTIVE_CELLS_TOKEN        14
#define SELECTION_MODE_TOKEN                 15
#define SELECTION_THRESHOLD_TOKEN            16

activeParameters globalActiveParameters;

static bool activeLocalizationDesired = FALSE;

bool verbose = TRUE;
	       
/**************************************************************************
 **************************************************************************
 * Forward declarations.
 **************************************************************************
 **************************************************************************/
static void
sendOccupancyProbsAndCorrectionToPlan( probabilityGrid* occupancyProbs,
				       realPosition* basePos);

static void
sendMaxPositionAsGoalToPlan( probabilityGrid* netPayoff,
			     int numberOfStepsInEachDirection);

static void
displayFields( realCellList* cells,
	       probabilityGrid* utilities,
	       probabilityGrid* costs,
	       probabilityGrid* netPayoffs,
	       probabilityGrid* occupancyProbs);

static void
initializeAbstractSensors( abstractSensorVector* allSensors,
			   abstractSensorVector* sensors,
			   int useQuestion, int useSonar, int useLaser);

static void
initializeMovementFields( probabilityGrid* utilities,
			  probabilityGrid* costs,
			  probabilityGrid* netPayoffs,
			  probabilityGrid* occupancyProbs,
			  positionProbabilityGrid* grid,
			  int numberOfStepsInEachDirection);

static void
computeInitialOccupancyProbabilities( probabilityGrid* map,
				      probabilityGrid* initialOccupancyProbs);


static void
computeWeightedOccupancyProbabilities( probabilityGrid* initialOccupancyProbs,
				       realCellList* maxPositions,
				       int numberOfStepsInEachDirection,
				       float cmPerStep,
				       positionProbabilityGrid* grid,
				       probabilityGrid* weightedOccupancyProbs);

static void
computeUtilities( realCellList* maxPositions,
		  abstractSensorVector* abstractSensors,
		  probabilityGrid* costs,
		  int numberOfStepsInEachDirection,
		  float cmPerStep,
		  positionProbabilityGrid* grid,
		  probabilityGrid* initialOccupancyProbs,
		  probabilityGrid* utilities);
		  
		 
static void
computeNetPayoff( probabilityGrid* utilities, 
		  probabilityGrid* costs,
		  probabilityGrid* netPayOff,
		  probability utilityWeight,
		  probability costWeight);

/**************************************************************************
 **************************************************************************
 * Mandatory functions.
 **************************************************************************
 **************************************************************************/
void
initialize_ACTIVE( char* fileName, actionInformation* actionInfo,
		   sensingActionMask* actionMask)
{
  /**********************************************************
   * Get the parameters from the file. 
   **********************************************************/
  token tok[NUMBER_OF_ACTION_PARAMETERS];

  /**********************************************************
   * Initialize with default values
   **********************************************************/
  globalActiveParameters.maxNumberOfMaxima = -1;
  globalActiveParameters.shrinkMaximaSize = 10;
  globalActiveParameters.minSumOfProbs = 0.3;
  globalActiveParameters.minProbOfLocalMax = 0.04;
  globalActiveParameters.minDistFromCurrentPos = 100.0;
  globalActiveParameters.maxCostOfPosition = 0.8;
  globalActiveParameters.quotaOfArea = 0.5;
  globalActiveParameters.goToGoal = 0;
  globalActiveParameters.goalPoint.x = 0.0;
  globalActiveParameters.goalPoint.y = 0.0;
  globalActiveParameters.showFields = 0;
  globalActiveParameters.minNumberOfSelectedSensors = 10;
  globalActiveParameters.maxNumberOfSelectedSensors = 180;
  globalActiveParameters.useMeasuredFeatures = 1;
  globalActiveParameters.considerInactiveCells = 1;
  globalActiveParameters.selectionMode = UNEXPECTED_SELECTION;
  globalActiveParameters.selectionThreshold = 0.99;
  
  setTokensInitialized(tok, NUMBER_OF_ACTION_PARAMETERS);
  
  /*********************
    now get the values
    *******************/

  tok[MAX_NUMBER_OF_MAXIMA_TOKEN].format   = INT_FORMAT;
  tok[MAX_NUMBER_OF_MAXIMA_TOKEN].variable = &(globalActiveParameters.maxNumberOfMaxima);
  tok[MAX_NUMBER_OF_MAXIMA_TOKEN].keyWord  = MAX_NUMBER_OF_MAXIMA_FOR_ACTION_KEYWORD;
  
  tok[SHRINK_MAXIMA_SIZE_TOKEN].format   = INT_FORMAT;
  tok[SHRINK_MAXIMA_SIZE_TOKEN].variable = &(globalActiveParameters.shrinkMaximaSize);
  tok[SHRINK_MAXIMA_SIZE_TOKEN].keyWord  = SHRINK_MAXIMA_SIZE_KEYWORD;
  
  tok[MIN_SUM_OF_PROBS_TOKEN].format   = FLOAT_FORMAT;
  tok[MIN_SUM_OF_PROBS_TOKEN].variable = &(globalActiveParameters.minSumOfProbs);
  tok[MIN_SUM_OF_PROBS_TOKEN].keyWord  = MIN_SUM_OF_PROBS_KEYWORD;
  
  tok[MIN_PROB_OF_LOCAL_MAX_TOKEN].format   = FLOAT_FORMAT;
  tok[MIN_PROB_OF_LOCAL_MAX_TOKEN].variable = &(globalActiveParameters.minProbOfLocalMax);
  tok[MIN_PROB_OF_LOCAL_MAX_TOKEN].keyWord  = MIN_PROB_OF_LOCAL_MAX_KEYWORD;
  
  tok[MIN_DIST_FROM_CURRENT_POS_TOKEN].format   = FLOAT_FORMAT;
  tok[MIN_DIST_FROM_CURRENT_POS_TOKEN].variable =
    &(globalActiveParameters.minDistFromCurrentPos);
  tok[MIN_DIST_FROM_CURRENT_POS_TOKEN].keyWord  = MIN_DIST_FROM_CURRENT_POS_KEYWORD;
  
  tok[MAX_COST_OF_POSITION_TOKEN].format   = FLOAT_FORMAT;
  tok[MAX_COST_OF_POSITION_TOKEN].variable = &(globalActiveParameters.maxCostOfPosition);
  tok[MAX_COST_OF_POSITION_TOKEN].keyWord  = MAX_COST_OF_POSITION_KEYWORD;

  tok[QUOTA_OF_AREA_TOKEN].format   = FLOAT_FORMAT;
  tok[QUOTA_OF_AREA_TOKEN].variable = &(globalActiveParameters.quotaOfArea);
  tok[QUOTA_OF_AREA_TOKEN].keyWord  = QUOTA_OF_AREA_KEYWORD;

  tok[GO_TO_GOAL_TOKEN].format   = FLOAT_FORMAT;
  tok[GO_TO_GOAL_TOKEN].variable = &(globalActiveParameters.goToGoal);
  tok[GO_TO_GOAL_TOKEN].keyWord  = GO_TO_GOAL_KEYWORD;

  tok[GOAL_X_TOKEN].format   = FLOAT_FORMAT;
  tok[GOAL_X_TOKEN].variable = &(globalActiveParameters.goalPoint.x);
  tok[GOAL_X_TOKEN].keyWord  = GOAL_X_KEYWORD;

  tok[GOAL_Y_TOKEN].format   = FLOAT_FORMAT;
  tok[GOAL_Y_TOKEN].variable = &(globalActiveParameters.goalPoint.y);
  tok[GOAL_Y_TOKEN].keyWord  = GOAL_Y_KEYWORD;

  tok[SHOW_FIELDS_TOKEN].format   = INT_FORMAT;
  tok[SHOW_FIELDS_TOKEN].variable = &(globalActiveParameters.showFields);
  tok[SHOW_FIELDS_TOKEN].keyWord  = SHOW_FIELDS_KEYWORD;

  tok[MIN_NUMBER_OF_SELECTED_SENSORS_TOKEN].format   = INT_FORMAT;
  tok[MIN_NUMBER_OF_SELECTED_SENSORS_TOKEN].variable =
    &(globalActiveParameters.minNumberOfSelectedSensors);
  tok[MIN_NUMBER_OF_SELECTED_SENSORS_TOKEN].keyWord  = MIN_NUMBER_OF_SELECTED_SENSORS_KEYWORD;

  tok[MAX_NUMBER_OF_SELECTED_SENSORS_TOKEN].format   = INT_FORMAT;
  tok[MAX_NUMBER_OF_SELECTED_SENSORS_TOKEN].variable =
    &(globalActiveParameters.maxNumberOfSelectedSensors);
  tok[MAX_NUMBER_OF_SELECTED_SENSORS_TOKEN].keyWord  = MAX_NUMBER_OF_SELECTED_SENSORS_KEYWORD;

  tok[USE_MEASURED_FEATURES_TOKEN].format   = INT_FORMAT;
  tok[USE_MEASURED_FEATURES_TOKEN].variable =
    &(globalActiveParameters.useMeasuredFeatures);
  tok[USE_MEASURED_FEATURES_TOKEN].keyWord  = USE_MEASURED_FEATURES_KEYWORD;
  
  tok[CONSIDER_INACTIVE_CELLS_TOKEN].format   = INT_FORMAT;
  tok[CONSIDER_INACTIVE_CELLS_TOKEN].variable =
    &(globalActiveParameters.considerInactiveCells);
  tok[CONSIDER_INACTIVE_CELLS_TOKEN].keyWord  = CONSIDER_INACTIVE_CELLS_KEYWORD;
  
  tok[SELECTION_MODE_TOKEN].format   = INT_FORMAT;
  tok[SELECTION_MODE_TOKEN].variable = &(globalActiveParameters.selectionMode);
  tok[SELECTION_MODE_TOKEN].keyWord  = SELECTION_MODE_KEYWORD;

  tok[SELECTION_THRESHOLD_TOKEN].format   = FLOAT_FORMAT;
  tok[SELECTION_THRESHOLD_TOKEN].variable = &(globalActiveParameters.selectionThreshold);
  tok[SELECTION_THRESHOLD_TOKEN].keyWord  = SELECTION_THRESHOLD_KEYWORD;

  /* First try to read the tokens. Not all have to be given in the ini file. */
  readTokens( fileName, tok, NUMBER_OF_ACTION_PARAMETERS, FALSE);

  if ( globalActiveParameters.selectionMode == ENTROPY_SELECTION)
    writeLog( "# Use entropy selection.\n");
  else if ( globalActiveParameters.selectionMode == UNEXPECTED_SELECTION)
    writeLog( "# Use unexpected selection with threshold %f.\n",
	      globalActiveParameters.selectionThreshold);
  else
    writeLog( "# Use human selection with threshold %f.\n",
	      globalActiveParameters.selectionThreshold);
  
  /* This map is sent to PLAN. */
  globalActiveParameters.initialPositionProbs = &(actionInfo->initialPositionProbs);
  globalActiveParameters.planMap = &(actionInfo->planMap);
  globalActiveParameters.basePosition = &(actionInfo->actualSensings.basePosition);

  initializeAbstractSensors( actionInfo->abstractSensors,
			     &(globalActiveParameters.abstractSensors),
			     actionMask->use[QUESTION],
			     actionMask->use[SONAR],
			     actionMask->use[LASER]);
}


/**************************************************************************
 * Starts the active localization.
 **************************************************************************/
void
startActiveLocalization()
{
  activeLocalizationDesired = TRUE;
}

/**************************************************************************
 * Stops the active localization.
 **************************************************************************/
void
stopActiveLocalization()
{
  if ( activeLocalizationDesired) {    
    activeLocalizationDesired = FALSE;
    resetAutomat();
  }
}

/**************************************************************************
 * Main update: finite state automat.
 **************************************************************************/
void
performActions_ACTIVE( actionInformation* info, sensingActionMask* mask)
{
  if ( activeLocalizationDesired)
    
    /* Check out what to do next. */
    computeNextState( info, mask);
  
  else {
    
    /*--------------------------------------------------------------------
     * Locazliation is supposed to be passive.
     * If PLAN is newly connected send it the map and finally send
     * the status report to all connected modules.
     *--------------------------------------------------------------------*/
    
    /* Send the map to MAP. This function can be found in planTcx.c. */
    
    initializeMap( info);
    initializePlan( info);

    broadcastStatusReport( mask, CONSIDER_PLAN);
  }
}

/***********************************************************************
 * Computes the relative target which gets the minimal a posteriori entropy
 * for a given probability and a given set of sensors.
 ***********************************************************************/
void
computeBestTarget( abstractSensorVector* abstractSensors,
		   realCellList* maxPositions,
		   positionProbabilityGrid* grid,
		   probabilityGrid* map)
{
  int numberOfStepsInEachDirection;
  float cmPerStep = grid->positionResolution;
  float usedTime;
  
  /*-------------------------------------------------------------------------
   * Utilities and costs of the different relative positions.
   *-------------------------------------------------------------------------*/
  static probabilityGrid utilities;
  static probabilityGrid costs;
  static probabilityGrid netPayoffs;
  static probabilityGrid initialOccupancyProbs;
  static probabilityGrid weightedOccupancyProbs;
  
  setTimer(0);
  fprintf( stderr, "Start looking for best action ... \n");
  writeLog( "Start looking for best action ... \n");
	   
  /* How many steps do we want to try in each direction? */
  numberOfStepsInEachDirection = iMax( grid->sizeX, grid->sizeY)
    * grid->positionResolution * globalActiveParameters.quotaOfArea / cmPerStep;

  /* Allocate memory and set default values. */
  initializeMovementFields( &utilities, &costs, &netPayoffs,
			    &weightedOccupancyProbs, grid,
			    numberOfStepsInEachDirection);

  /* For each step the weighted occupancy probability relative to the maxpositions
   * is computed and used for dynamic programming. */
  computeInitialOccupancyProbabilities( map, &initialOccupancyProbs);
  
  computeWeightedOccupancyProbabilities( &initialOccupancyProbs, maxPositions,
					 numberOfStepsInEachDirection,
					 cmPerStep, grid,
					 &weightedOccupancyProbs);
    
  /* Send the occupancy probs and the corresponding correction parameters
   * to the planning module. */
  sendOccupancyProbsAndCorrectionToPlan( &weightedOccupancyProbs,
					 globalActiveParameters.basePosition);
  
  /* Spread the utility of the robot position over the cost field. */
  spreadCosts( &weightedOccupancyProbs, &costs, -1);

  /* Display the whole stuff. */
  if ( globalActiveParameters.showFields)
    displayFields( maxPositions,
		   NULL, &costs, NULL, &weightedOccupancyProbs);

  /* Step through all moves and compute the a posteriori utility of the
   * new positions (only for position with costs below threshold). */
  computeUtilities( maxPositions, abstractSensors,
		    &costs, numberOfStepsInEachDirection,
		    cmPerStep, grid, &initialOccupancyProbs, &utilities);

  /* Now we got the utilities and the costs of each relative movement and
   * combine these values into the netpayoff of the movements. */
  computeNetPayoff( &utilities, &costs, &netPayoffs,
		    UTILITY_WEIGHT, COST_WEIGHT);
  
  
  /* Send the position with highest net payoff as a target point
   * to PLAN. */
  sendMaxPositionAsGoalToPlan( &netPayoffs, numberOfStepsInEachDirection);
  
  /* Display the whole stuff. */
  if ( globalActiveParameters.showFields)
    displayFields( maxPositions,
		   &utilities, 
		   NULL, &netPayoffs, NULL);

  usedTime = timeExpired(0);
  fprintf( stderr, "done in %f seconds.\n", usedTime);
  writeLog( "done in %f seconds.\n", usedTime);
}


/***********************************************************************
 * Initializes the sensors to be used for the aposteriori entropy.
 ***********************************************************************/
static void
initializeAbstractSensors( abstractSensorVector* allSensors,
			   abstractSensorVector* sensors,
			   int useQuestion, int useSonar, int useLaser)
{
#ifdef CONSIDER_QUESTION

#define NUM_ABSTRACT 5
  
  int sensorNumber;
  int numberOfAbstractSensors = NUM_ABSTRACT;
  
  sensors->numberOfSensors = numberOfAbstractSensors;
  sensors->sensor = (abstractSensorType*)
    malloc ( numberOfAbstractSensors * sizeof( abstractSensorType));

  if (useQuestion)
    sensors->sensor[0] = allSensors[ABSTRACT_QUESTION].sensor[2]; 
  
  if (useSonar){
    sensors->sensor[1] = allSensors[ABSTRACT_SONAR].sensor[0]; 
    sensors->sensor[2] = allSensors[ABSTRACT_SONAR].sensor[6]; 
    sensors->sensor[3] = allSensors[ABSTRACT_SONAR].sensor[12]; 
    sensors->sensor[4] = allSensors[ABSTRACT_SONAR].sensor[18];
  }
#else
  
#define NUM_ABSTRACT 18
  
  int sensorNumber;
  int numberOfAbstractSensors = NUM_ABSTRACT;
  
  sensors->numberOfSensors = numberOfAbstractSensors;
  sensors->sensor = (abstractSensorType*)
    malloc ( numberOfAbstractSensors * sizeof( abstractSensorType));

  if (useSonar){
   sensors->sensor[1] = allSensors[ABSTRACT_SONAR].sensor[0]; 
   sensors->sensor[2] = allSensors[ABSTRACT_SONAR].sensor[6]; 
   sensors->sensor[3] = allSensors[ABSTRACT_SONAR].sensor[12]; 
   sensors->sensor[4] = allSensors[ABSTRACT_SONAR].sensor[18];
  }

  if (useLaser){

/*   sensors->sensor[0] = allSensors[ABSTRACT_REAR_LASER].sensor[0];  */
/*   sensors->sensor[1] = allSensors[ABSTRACT_REAR_LASER].sensor[179]; */
/*   sensors->sensor[2] = allSensors[ABSTRACT_FRONT_LASER].sensor[90]; */
/*   sensors->sensor[3] = allSensors[ABSTRACT_FRONT_LASER].sensor[135]; */
/*   sensors->sensor[4] = allSensors[ABSTRACT_FRONT_LASER].sensor[179]; */
/*   sensors->sensor[5] = allSensors[ABSTRACT_REAR_LASER].sensor[45]; */
/*   sensors->sensor[6] = allSensors[ABSTRACT_REAR_LASER].sensor[90]; */
/*   sensors->sensor[7] = allSensors[ABSTRACT_REAR_LASER].sensor[135]; */
/*   return;  */
  }

    
  if ( useSonar) {
    
    for ( sensorNumber = 0; sensorNumber < numberOfAbstractSensors; sensorNumber++) {
      sensors->sensor[sensorNumber] =
	allSensors[ABSTRACT_SONAR].sensor[allSensors[ABSTRACT_SONAR].numberOfSensors
					 / NUM_ABSTRACT * sensorNumber];
    }
  }
  else if ( useLaser) {

     numberOfAbstractSensors = numberOfAbstractSensors / 2; 
    
     for ( sensorNumber = 0; sensorNumber < numberOfAbstractSensors; sensorNumber++) {
       sensors->sensor[sensorNumber] =
	 allSensors[ABSTRACT_FRONT_LASER].
	 sensor[allSensors[ABSTRACT_FRONT_LASER].numberOfSensors
	       / numberOfAbstractSensors * sensorNumber];
     }
     
     for ( sensorNumber = 0; sensorNumber < numberOfAbstractSensors; sensorNumber++) {
      sensors->sensor[sensorNumber+numberOfAbstractSensors] =
	allSensors[ABSTRACT_REAR_LASER].
	sensor[allSensors[ABSTRACT_REAR_LASER].numberOfSensors
	      / numberOfAbstractSensors * sensorNumber];
    }
  }
#endif
}


/***********************************************************************
 * Initializes the two fields which contain the utilities and costs
 * of performing movements. 
 ***********************************************************************/
static void
displayFields( realCellList* cells,
	       probabilityGrid* utilities,
	       probabilityGrid* costs,
	       probabilityGrid* netPayoffs,
	       probabilityGrid* occupancyProbs)
{
  static gridWindow* utilityWin;
  static gridWindow* costWin;
  static gridWindow* netPayoffWin;
  static gridWindow* occupancyWin;
  
  static bool firstUtility = TRUE;
  static bool firstCost = TRUE;
  static bool firstNetPayoff = TRUE;
  static bool firstOccupancyProb = TRUE;

  if ( firstUtility || firstCost || firstNetPayoff || firstOccupancyProb)
    putc( 7, stderr);
    
  if ( firstUtility && utilities != NULL) {
    utilityWin   = createMapWindow( utilities, "Utilities", 850, 600, 2);
    firstUtility = FALSE;
  }
  if ( firstCost && costs != NULL) {
    costWin   = createMapWindow( costs, "Costs", 850, 600, 2);
    firstCost = FALSE;
  }
  if ( firstNetPayoff && netPayoffs != NULL) {
    netPayoffWin   = createMapWindow( netPayoffs, "NetPayoffs", 850, 600, 2);
    firstNetPayoff = FALSE;
  }
  if ( firstOccupancyProb && occupancyProbs != NULL) {
    occupancyWin   = createMapWindow( occupancyProbs, "OccupancyProbs", 850, 600, 2);
    firstOccupancyProb = FALSE;
  }

  if ( utilities != NULL)
    displayMovementField( utilityWin, utilities, cells);
  if ( costs != NULL)
    displayMovementField( costWin, costs, cells);
  if ( netPayoffs != NULL)
    displayMovementField( netPayoffWin, netPayoffs, cells);
  if ( occupancyProbs != NULL)
    displayMovementField( occupancyWin, occupancyProbs, cells);
}


/***********************************************************************
 * Initializes the two fields which contain the utilities and costs
 * of performing movements. 
 ***********************************************************************/
static void
initializeMovementFields( probabilityGrid* utilities,
			  probabilityGrid* costs,
			  probabilityGrid* netPayoffs,
			  probabilityGrid* occupancyProbs,
			  positionProbabilityGrid* grid,
			  int numberOfStepsInEachDirection)
{
#define INITIAL_UTILITY 1e10
  int x, y;

  static int firstTime = TRUE;

  if (verbose)
    fprintf( stderr, "Initialize movement fields (%d) ...", numberOfStepsInEachDirection);
	     
  /* This whole stuff has to be done only once. */
  if ( firstTime) {
    
    /* Set the different parameters of the field. */
    utilities->sizeX = 2 * numberOfStepsInEachDirection + 1;
    utilities->sizeY = 2 * numberOfStepsInEachDirection + 1;
    
    utilities->origsizeX = utilities->sizeX;
    utilities->origsizeY = utilities->sizeY;
    
    utilities->offsetX = 0;
    utilities->offsetY = 0;
    
    utilities->resolution = grid->positionResolution;
    utilities->unknown = INITIAL_UTILITY;
    utilities->initialized = TRUE;

    /* Copy into the cost field. */
    *occupancyProbs = *utilities;
    *netPayoffs = *utilities;
    *costs = *utilities;
    
    /* Allocate memory. */
    utilities->prob = (mapProbability**)
      allocate2D( utilities->sizeX, utilities->sizeY, MAP_PROBABILITY);
    occupancyProbs->prob = (mapProbability**)
      allocate2D( occupancyProbs->sizeX, occupancyProbs->sizeY, MAP_PROBABILITY);
    netPayoffs->prob = (mapProbability**)
      allocate2D( netPayoffs->sizeX, netPayoffs->sizeY, MAP_PROBABILITY);
    costs->prob = (mapProbability**)
      allocate2D( costs->sizeX, costs->sizeY, MAP_PROBABILITY);

    firstTime = FALSE;
  }

  for ( x = 0; x < utilities->sizeX; x++)
    for ( y = 0; y < utilities->sizeY; y++) {
      utilities->prob[x][y] 
	= occupancyProbs->prob[x][y] = netPayoffs->prob[x][y]
	= INITIAL_UTILITY;
    }

  /* Initialize the center position of the robot with highest utility. */
  for ( x = 0; x < utilities->sizeX; x++)
    for ( y = 0; y < utilities->sizeY; y++) 
      costs->prob[x][y] = MAX_COST;

  costs->prob[costs->sizeX / 2][costs->sizeY / 2] = MIN_COST;
  costs->unknown = MAX_COST;

  if (verbose)
    fprintf( stderr, "done.\n");
}


/***********************************************************************
 * The map is assumed to be the same at each call so we use it only
 * at the first call.
 ***********************************************************************/
static void
computeInitialOccupancyProbabilities( probabilityGrid* map,
				      probabilityGrid* initialOccupancyProbs)
{
  int x, y;
  static bool firstTime = TRUE;

  /* The map is assumed to be the same at each call so we use it only to
   * initialize the probs once. */
  if ( firstTime) {
    
    *initialOccupancyProbs = *map;
    
    initialOccupancyProbs->prob = (mapProbability**)
      allocate2D( initialOccupancyProbs->sizeX,
		  initialOccupancyProbs->sizeY,
		  MAP_PROBABILITY);
    
    for ( x = 0; x < initialOccupancyProbs->sizeX; x++)
      for ( y = 0; y < initialOccupancyProbs->sizeY; y++) 
	initialOccupancyProbs->prob[x][y] = 1.0 - positionProbabilityMap( x, y, *map, 0.0);
    firstTime = FALSE;
  }
}


/***********************************************************************
 * For each step the weighted occupancy probability relative to the maxpositions
 * is computed and used for dynamic programming. 
 ***********************************************************************/
static void
computeWeightedOccupancyProbabilities( probabilityGrid* initialOccupancyProbs,
				       realCellList* maxPositions,
				       int numberOfStepsInEachDirection,
				       float cmPerStep,
				       positionProbabilityGrid* grid,
				       probabilityGrid* weightedOccupancyProbs)
{
  int forwardStep, sidewardStep;
  movement move = {0.0, 0.0, 0.0};
  realCellList destinationCells;

  if (verbose)
    fprintf( stderr, "Compute weighted occupancy probabilities ... ");
  
  /*-------------------------------------------------------------------------
   * Step through all moves and compute the occupancy probability of the
   * corresponding positions.
   *-------------------------------------------------------------------------*/
  for ( forwardStep = -numberOfStepsInEachDirection;
	forwardStep <= numberOfStepsInEachDirection;
	forwardStep++) {
    
    int posInFieldY = numberOfStepsInEachDirection + forwardStep;
    move.forward = cmPerStep * forwardStep;
    
    for ( sidewardStep = -numberOfStepsInEachDirection;
	  sidewardStep <= numberOfStepsInEachDirection;
	  sidewardStep++) {
      
      int posInFieldX = numberOfStepsInEachDirection + sidewardStep;
      move.sideward = cmPerStep * sidewardStep;
      
      /* Apply the movement to the cells in the list.
       * The value of a movement is only important if at least one of the
       * endpoints is in the map. */
      applyMovement( maxPositions, move, grid, &destinationCells);
      
      if ( destinationCells.numberOfCellsInMap > 0) 
	
	/* This "weighted map" is used later to compute the real costs
	 * to reach one of the relative positions. */
	
	weightedOccupancyProbs->prob[posInFieldX][posInFieldY] =
	  weightedOccupancyProbability( &destinationCells, initialOccupancyProbs);
    }
  }
  if (verbose)
    fprintf( stderr, "done.\n");
}


/***********************************************************************
 * Step through all moves and compute the a posteriori utility of the
 * new positions (only for position with costs below threshold).
 ***********************************************************************/
static void
computeUtilities( realCellList* maxPositions,
		  abstractSensorVector* abstractSensors,
		  probabilityGrid* costs,
		  int numberOfStepsInEachDirection,
		  float cmPerStep,
		  positionProbabilityGrid* grid,
		  probabilityGrid* initialOccupancyProbs,
		  probabilityGrid* utilities)
{
  int forwardStep, sidewardStep;
  realCellList destinationCells;
  movement move = {0.0, 0.0, 0.0};
  double entropies[MAX_NUMBER_OF_SENSORS];

  /* We search for the movement with the best utility. */
  float maxUtility = -1e20, utility;


  for ( forwardStep = -numberOfStepsInEachDirection;
	forwardStep <= numberOfStepsInEachDirection;
	forwardStep++) {
    
    int posInFieldY = numberOfStepsInEachDirection + forwardStep;
    move.forward = cmPerStep * forwardStep;

    if ( forwardStep % 10 == 0) {
      swallowStatusReports(DONT_WAIT);

      fprintf( stderr, "\rComputing utilities ... %.1f%%",
	       (float) ( numberOfStepsInEachDirection + forwardStep)
	       / (float) ( 2 * numberOfStepsInEachDirection + 1) * 100.0);
    }

#define STEP_SKIP 1

    if ( forwardStep % STEP_SKIP == 0) {
	   
      for ( sidewardStep = -numberOfStepsInEachDirection;
	    sidewardStep <= numberOfStepsInEachDirection;
	    sidewardStep++) {
	
	if ( sidewardStep % STEP_SKIP == 0) {
	  
	  int posInFieldX = numberOfStepsInEachDirection + sidewardStep;
	  move.sideward = cmPerStep * sidewardStep;
	
	  /* Apply the movement to the cells in the list.
	 * The value of a movement is only important if at least one of the
	 * endpoints is in the map. */
	  applyMovement( maxPositions, move, grid, &destinationCells);
	
	  if ( destinationCells.numberOfCellsInMap > 0) 
	
	    if ( costs->prob[posInFieldX][posInFieldY]
		 < globalActiveParameters.maxCostOfPosition) {
	  
	      /* Compute the expected entropy after having fired the senors. */
	      double information = computeExpectedEntropyDiff( &destinationCells,
							       abstractSensors,
							       grid,
							       initialOccupancyProbs,
							       entropies,
#ifdef CONSIDER_QUESTION
							       DONT_CONSIDER_INACTIVE);
#else
							       CONSIDER_INACTIVE);
#endif
      

	      if ( information != 0.0) {
	    
		utility = information;
	    
		/* Is it good? */
		if ( utility > maxUtility) 
		  maxUtility = utility;
	    
		/* Store the utilities. */
		utilities->prob[posInFieldX][posInFieldY] = utility;
	      }
	    }
	}
      }
    }
  }
  
  shrinkMaxima( utilities, globalActiveParameters.shrinkMaximaSize);
  normalizeMap( utilities);

  fprintf( stderr, "done.\n");
}


static void
computeNetPayoff( probabilityGrid* utilities, 
		  probabilityGrid* costs,
		  probabilityGrid* netPayOff,
		  probability utilityWeight,
		  probability costWeight)
{
  int x, y;

  for ( x = 0; x < netPayOff->sizeX; x++)
    for ( y = 0; y < netPayOff->sizeY; y++)
      if ( utilities->prob[x][y] != utilities->unknown)
	netPayOff->prob[x][y] = utilityWeight * utilities->prob[x][y]
	- costWeight * costs->prob[x][y];

  normalizeMap( netPayOff);
}

/* Send the occupancy probs and the corresponding correction parameters
 * to the planning module. */
static void
sendOccupancyProbsAndCorrectionToPlan( probabilityGrid* occupancyProbs,
				       realPosition* basePos)
{
  realPosition mapPos;
  correctionParameter corr;
    
  mapPos.x = occupancyProbs->sizeX * occupancyProbs->resolution / 2.0;
  mapPos.y = occupancyProbs->sizeY * occupancyProbs->resolution / 2.0;
  mapPos.rot = DEG_90;

  sendMapToPlan( occupancyProbs, INVERTED, 0);

  computeCorrectionParam( *basePos, mapPos, &corr);
  sendCorrectionParams( corr,
			PLAN,
			moduleName[PLAN_MODULE]);
}

/* Send the position with highest net payoff as a target point
 * to PLAN. */
static void
sendMaxPositionAsGoalToPlan( probabilityGrid* netPayoffs,
			     int numberOfStepsInEachDirection)
{
  int x, y;
  int maxX = 0, maxY = 0;
  int currentX = numberOfStepsInEachDirection;
  int currentY = numberOfStepsInEachDirection;
  float distInGridCells = globalActiveParameters.minDistFromCurrentPos /
    netPayoffs->resolution;
  realPosition maxPos;
  probability max = 0.0;
  
  
  /* Search for the position with highest net payoff. The position must
   * be at least minDistFromCurrentPos cm away from the current position of
   * the robot. */
  for ( x = 0; x < 2 * numberOfStepsInEachDirection + 1; x++)
    for ( y = 0; y < 2 * numberOfStepsInEachDirection  + 1; y++) {
      if ( sqrt( iSqr( x - currentX) + iSqr( y - currentY)) > distInGridCells) {
	if ( netPayoffs->prob[x][y] != netPayoffs->unknown &&
	     netPayoffs->prob[x][y] > max) {
	  max = netPayoffs->prob[x][y];
	  maxX = x;
	  maxY = y;
	}
      }
    }
    
  maxPos.x = netPayoffs->resolution * maxX;
  maxPos.y = netPayoffs->resolution * maxY;
  sendGoalPointToPlan( maxPos);
  planStartAutonomous();
}


