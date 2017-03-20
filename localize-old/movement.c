
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/movement.c,v $
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
 * $Log: movement.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.61  2000/03/06 20:00:45  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.60  2000/01/26 22:56:44  fox
 * Might work. If anything weird happens, contact me.
 *
 * Revision 1.59  2000/01/19 16:21:47  fox
 * Version after I created density sample sets. Removed all defines so
 * that it should work properly. If you want to use zlib, set USE_ZLIB
 * in the Makefile.
 *
 * Revision 1.58  2000/01/10 19:04:21  fox
 * DON'T USE!
 *
 * Revision 1.57  2000/01/02 15:33:16  fox
 * Should work.
 *
 * Revision 1.56  1999/12/27 09:52:51  fox
 * *** empty log message ***
 *
 * Revision 1.55  1999/11/02 18:12:35  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.54  1999/09/01 21:26:55  fox
 * Works almost perfectly :-)
 *
 * Revision 1.53  1999/01/22 17:48:07  fox
 * Final version of sampling paper and denver demo.
 *
 * Revision 1.52  1998/11/17 23:26:23  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.51  1998/11/03 21:02:20  fox
 * Lasers will not be integrated if the robot rotates faster than
 * MAX_ROT_VEL_FOR_LASERS deg/sec.
 *
 * Revision 1.50  1998/10/19 18:29:56  fox
 * *** empty log message ***
 *
 * Revision 1.49  1998/10/02 15:16:39  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.48  1998/09/25 04:02:56  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.47  1998/08/19 16:33:56  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.46  1998/01/22 13:06:17  fox
 * First version after selection-submission.
 *
 * Revision 1.45  1997/12/19 10:30:31  wolfram
 * Changed MINIMUM_UPDATE_PROBABILITY_QUOTA and resetNewPlanes
 *
 * Revision 1.44  1997/12/18 12:57:25  wolfram
 * Fixed exceptions in shiftAndMultiplyPlane
 *
 * Revision 1.43  1997/12/17 16:48:30  wolfram
 * Added NEIGHBOR_PLANES_TO_ADD keyword for normalize
 *
 * Revision 1.42  1997/12/11 17:06:30  fox
 * Added some parameters.
 *
 * Revision 1.41  1997/12/09 12:03:23  wolfram
 * Added support for Marker in scripts
 *
 * Revision 1.40  1997/12/03 09:09:08  fox
 * Renamde USE_MOVEMENT to USE_POSITION.
 *
 * Revision 1.39  1997/12/02 15:20:39  fox
 * Nothing remarkable.
 *
 * Revision 1.38  1997/11/27 18:11:19  fox
 * Several changes to make angles work better.
 *
 * Revision 1.37  1997/11/20 17:10:14  wolfram
 * Fixed some accesses to not initialized memory using purify
 *
 * Revision 1.36  1997/11/20 12:58:12  fox
 * Version with good sensor selection.
 *
 * Revision 1.35  1997/11/07 12:39:41  fox
 * Added some graphic features.
 *
 * Revision 1.34  1997/10/02 09:20:28  wolfram
 * Better initialization for angles
 *
 * Revision 1.33  1997/09/26 17:02:10  wolfram
 * Intermediate version, do not use
 *
 * Revision 1.32  1997/08/19 22:19:08  wolfram
 * Fixed a bug in display of robot window and in planeOfRotation
 *
 * Revision 1.31  1997/08/02 16:51:04  wolfram
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
 * Revision 1.30  1997/07/04 17:29:15  fox
 * Final version before holiday!!!
 *
 * Revision 1.29  1997/06/20 07:36:11  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.28  1997/06/03 11:49:22  fox
 * Museum version.
 *
 * Revision 1.27  1997/05/26 08:47:52  fox
 * Last version before major changes.
 *
 * Revision 1.26  1997/03/14 17:58:21  fox
 * This version should run quite stable now.
 *
 * Revision 1.25  1997/03/13 17:36:22  fox
 * Temporary version. Don't use!
 *
 * Revision 1.24  1997/01/29 12:23:10  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.23  1997/01/19 23:35:15  fox
 * Fixed a bug in convolve.
 *
 * Revision 1.22  1997/01/19 19:31:17  fox
 * yeah
 *
 * Revision 1.21  1997/01/19 18:56:57  wolfram
 * Again a bug ...
 *
 * Revision 1.20  1997/01/18 14:07:55  fox
 * Test version.
 *
 * Revision 1.19  1997/01/16 12:42:50  fox
 * Fixed a bug in scan masks.
 *
 * Revision 1.18  1997/01/10 15:19:22  fox
 * Improved several methods.
 *
 * Revision 1.17  1997/01/08 15:52:57  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.16  1997/01/07 13:14:29  wolfram
 * Movement decisions are made on the basis of a list of real positions
 *
 * Revision 1.15  1997/01/07 12:52:00  wolfram
 * applyMovement now takes a list of localMaxima as input
 *
 * Revision 1.14  1997/01/07 09:38:06  fox
 * Changed the loading of the ini file such that all parameters have to be
 * given only if the corresponding feature has to be used.
 *
 * Revision 1.13  1997/01/03 10:09:46  fox
 * First version with exploration.
 *
 * Revision 1.12  1996/12/20 15:29:40  fox
 * Added four parameters.
 *
 * Revision 1.11  1996/12/02 10:32:09  wolfram
 * Expected distance file now includes distance + variances
 *
 * Revision 1.10  1996/11/29 15:33:39  fox
 * ok
 *
 * Revision 1.9  1996/11/26 11:08:13  fox
 * Improved version.
 *
 * Revision 1.8  1996/11/25 19:35:41  fox
 * Test version for decisions of movements.
 *
 * Revision 1.7  1996/11/25 09:47:24  wolfram
 * Added graphic support for movment errors
 * Improved procedure for computing the entropy of a grid
 *
 * Revision 1.6  1996/11/21 14:37:14  wolfram
 * Movement Support for Exploration
 *
 * Revision 1.5  1996/11/21 14:08:05  wolfram
 * First steps toward action selection
 *
 * Revision 1.4  1996/11/18 09:58:31  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.3  1996/10/24 12:07:11  fox
 * Fixed a bug.
 *
 * Revision 1.2  1996/10/24 09:56:54  fox
 * LOCALIZE also works in a write protected directory.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:31  rhino
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


#include "movement.h"
#include "allocate.h"
#include "general.h"
#include "map.h"
#include "file.h"
#include "communication.h"
#include "condensation.h"

#define USE_POSITION_TOKEN        0
#define INTEGRATE_THRESHOLD_TOKEN 1
#define CONVOLVE_THRESHOLD_TOKEN  2
#define MAX_QUOTA_OF_PLANES_TOKEN 3
#define XY_KERNEL_SIZE_TOKEN      4
#define Z_KERNEL_SIZE_TOKEN       5
#define XY_KERNEL_VALUES_TOKEN    6
#define Z_KERNEL_VALUES_TOKEN     7


typedef struct {
  float integrateThreshold;
  float convolveThreshold;
  float maxQuotaOfPlanes;
  int xyKernelSize;
  int zKernelSize;
  int usePosition;
} movementParameters;

movementParameters globalMovementParameters;
extern informationsFor_COMMUNICATION communicationInfo;

/********************************************************************
 ********************************************************************
 * Forward declarations of internal functions.
 ********************************************************************
 ********************************************************************/

static void
integrateMovement( informationsFor_MOVEMENT* info);

static void
convolveGrid( informationsFor_MOVEMENT* info);

static void
setKernelValues( float* kern, int size);

/**************************************************************************
 **************************************************************************
 * Mandatory functions.
 **************************************************************************
 **************************************************************************/

void
initialize_MOVEMENT( char* fileName,
		     actionInformation* actionInfo,
		     sensingActionMask* actionMask,
		     sensingFunctions* handlers)
{
  char xyKernelString[MAX_STRING_LENGTH],zKernelString[MAX_STRING_LENGTH];

  /* This struct contains all relevant information for later integration
   * of movement informations. */
  informationsFor_MOVEMENT* info = 
    (informationsFor_MOVEMENT*) malloc( sizeof( informationsFor_MOVEMENT));
  
  token tok[NUMBER_OF_MOVEMENT_PARAMETERS];
  
  /*-------------------------------------------------------------------------
   * Initialize parameters
   *------------------------------------------------------------------------*/

  globalMovementParameters.integrateThreshold = 20;
  globalMovementParameters.convolveThreshold = 200;
  globalMovementParameters.maxQuotaOfPlanes = 0.4;
  globalMovementParameters.usePosition = TRUE;
  info->xyKernel.size = 2;
  info->zKernel.size = 2;
  strcpy(xyKernelString, "0.96 0.02");
  strcpy(zKernelString, "0.94 0.03");

  setTokensInitialized(tok, NUMBER_OF_MOVEMENT_PARAMETERS);

  /*-------------------------------------------------------------------------
   * Get the parameters from the file. 
   *------------------------------------------------------------------------*/
  tok[USE_POSITION_TOKEN].format   = INT_FORMAT;
  tok[USE_POSITION_TOKEN].variable = &(globalMovementParameters.usePosition);
  tok[USE_POSITION_TOKEN].keyWord  = USE_POSITION_KEYWORD;
  
  tok[INTEGRATE_THRESHOLD_TOKEN].format   = FLOAT_FORMAT;
  tok[INTEGRATE_THRESHOLD_TOKEN].variable = &(globalMovementParameters.integrateThreshold);
  tok[INTEGRATE_THRESHOLD_TOKEN].keyWord  = INTEGRATE_MOVEMENT_THRESHOLD_KEYWORD;
  
  tok[CONVOLVE_THRESHOLD_TOKEN].format   = FLOAT_FORMAT;
  tok[CONVOLVE_THRESHOLD_TOKEN].variable = &(globalMovementParameters.convolveThreshold);
  tok[CONVOLVE_THRESHOLD_TOKEN].keyWord  = CONVOLVE_MOVEMENT_THRESHOLD_KEYWORD;

  tok[MAX_QUOTA_OF_PLANES_TOKEN].format   = FLOAT_FORMAT;
  tok[MAX_QUOTA_OF_PLANES_TOKEN].variable = &(globalMovementParameters.maxQuotaOfPlanes);
  tok[MAX_QUOTA_OF_PLANES_TOKEN].keyWord  = MAX_QUOTA_OF_PLANES_FOR_MOVEMENT_KEYWORD;

  tok[XY_KERNEL_SIZE_TOKEN].format   = INT_FORMAT;
  tok[XY_KERNEL_SIZE_TOKEN].variable = &(info->xyKernel.size);
  tok[XY_KERNEL_SIZE_TOKEN].keyWord  = XY_KERNEL_SIZE_KEYWORD;
  
  tok[Z_KERNEL_SIZE_TOKEN].format   = INT_FORMAT;
  tok[Z_KERNEL_SIZE_TOKEN].variable = &(info->zKernel.size);
  tok[Z_KERNEL_SIZE_TOKEN].keyWord  = Z_KERNEL_SIZE_KEYWORD;
   
  tok[XY_KERNEL_VALUES_TOKEN].format   = MULTI_VALUE_FORMAT;
  tok[XY_KERNEL_VALUES_TOKEN].variable = xyKernelString;
  tok[XY_KERNEL_VALUES_TOKEN].keyWord  = XY_KERNEL_VALUES_KEYWORD;
  
  tok[Z_KERNEL_VALUES_TOKEN].format   = MULTI_VALUE_FORMAT;
  tok[Z_KERNEL_VALUES_TOKEN].variable = zKernelString;
  tok[Z_KERNEL_VALUES_TOKEN].keyWord  = Z_KERNEL_VALUES_KEYWORD;

  readTokens( fileName, tok, NUMBER_OF_MOVEMENT_PARAMETERS, FALSE); 

  /*-------------------------------------------------------------------------
   * done.
   *------------------------------------------------------------------------*/

  /*-------------------------------------------------------------------------
   * Get the position probabilities,
   *------------------------------------------------------------------------*/
  if ( ! actionInfo->onlineMapping && ! actionInfo->initialPositionProbs.initialized) {
    fprintf( stderr,
	     "Error: position probabilities not initialized for the movements.\n");
    closeLogAndExit(1);
  }

  /*-------------------------------------------------------------------------
   * Set the pointers to the structures provided in actionInfo.
   *------------------------------------------------------------------------*/
  info->aPrioriProbs = &(actionInfo->initialPositionProbs);
  info->grid         = &(actionInfo->positionProbs);
  info->delta        = &(actionInfo->actualSensings.delta);
  info->useProbGrid  = &(actionInfo->useProbGrid);
  info->samples      = &(actionInfo->samples);
  info->tcxSamples   = &(actionInfo->tcxSamples);
  
  /*-------------------------------------------------------------------------
   * Initialze the movement structure.
   *------------------------------------------------------------------------*/
  info->delta->forward  = 0.0;
  info->delta->sideward = 0.0;
  info->delta->rotation = 0.0;
  info->delta->isNew    = FALSE;
  
  actionInfo->actualSensings.noMoveCnt = 0;
  actionInfo->actualSensings.distanceTraveled = 0.0;
  
  /*-------------------------------------------------------------------------
   * Initialze the motion kernel.
   *------------------------------------------------------------------------*/
  info->xyKernel.element = allocate1D( info->xyKernel.size, FLOAT);
  getInitValuesFloat( xyKernelString, info->xyKernel.element,
		      info->xyKernel.size);
  info->zKernel.element  = allocate1D( info->zKernel.size, FLOAT);
  getInitValuesFloat( zKernelString, info->zKernel.element,
		      info->zKernel.size);
  /* Now place the struct in the global information struct. */
  actionInfo->info[MOVEMENT] = info;
  
  /*-------------------------------------------------------------------------
   * Initialze the mask.
   *------------------------------------------------------------------------*/
  actionMask->numberOfActions[MOVEMENT] = NUMBER_OF_ACTIONS_MOVEMENT;

  /* Movement must always be used!!!! */
  actionMask->use[MOVEMENT] = TRUE;

  /*-------------------------------------------------------------------------
   * Initialze the handlers.
   *------------------------------------------------------------------------*/
  handlers->checkIfConsider[MOVEMENT]            = checkIfConsider_MOVEMENT;
  handlers->checkWhichActionsToPerform[MOVEMENT] = checkWhichActionsToPerform_MOVEMENT;
  handlers->performActions[MOVEMENT]             = performActions_MOVEMENT;
}

void
checkIfConsider_MOVEMENT( actionInformation* info,
			  sensingActionMask* mask)
{
  /* Just check for the threshold. */
  mask->consider[MOVEMENT] = mask->use[MOVEMENT]
    && ( !mask->consider[ANGLE] || info->positionProbs.quotaOfPlanesToBeUpdated
	 < globalMovementParameters.maxQuotaOfPlanes);
}


void
checkWhichActionsToPerform_MOVEMENT( actionInformation* info,
				     sensingActionMask* mask)
{
  if ( mask->consider[MOVEMENT]) {
    
    /* Check whether the grid or the samples have to be shifted.
     * This has to be done after enough movement. */
    if ( info->summedMovements[MOVEMENT][INTEGRATE_MOVEMENT]
	 > globalMovementParameters.integrateThreshold) {
      mask->perform[MOVEMENT][INTEGRATE_MOVEMENT] = TRUE;
      info->summedMovements[MOVEMENT][INTEGRATE_MOVEMENT] = 0.0;
    }
    else
      mask->perform[MOVEMENT][INTEGRATE_MOVEMENT] = FALSE;
    

    /* Check whether the grid has to be convolved. */
    if ( info->useProbGrid &&
	 (info->summedMovements[MOVEMENT][CONVOLVE_POS_GRID]
	  > globalMovementParameters.convolveThreshold)) {          
      mask->perform[MOVEMENT][CONVOLVE_POS_GRID] = TRUE;
      info->summedMovements[MOVEMENT][CONVOLVE_POS_GRID] = 0.0;
    }
    else
      mask->perform[MOVEMENT][CONVOLVE_POS_GRID] = FALSE;
    
  }
  else {
      mask->perform[MOVEMENT][INTEGRATE_MOVEMENT] = FALSE;
      mask->perform[MOVEMENT][CONVOLVE_POS_GRID] = FALSE;
  }

  /* If the robot has stopped we update the current position. */
  if ( ( info->actualSensings.noMoveCnt == 10
	 && info->summedMovements[MOVEMENT][INTEGRATE_MOVEMENT] > 10.0)) {
    fprintf(stderr, "# No motion. Integrate last centimeters.\n");
    writeLog( "# No motion. Integrate last centimeters.\n");
    mask->perform[MOVEMENT][INTEGRATE_MOVEMENT] = TRUE;
    info->summedMovements[MOVEMENT][INTEGRATE_MOVEMENT] = 0.0;
  }
  
  if ( communicationInfo.scr->newMarker) {    
    fprintf(stderr, "#Marker --> integrate movement.\n");
    writeLog( "#Marker --> integrate movement.\n");
    mask->perform[MOVEMENT][INTEGRATE_MOVEMENT] = TRUE;
    info->summedMovements[MOVEMENT][INTEGRATE_MOVEMENT] = 0.0;
  }
}


void
performActions_MOVEMENT( actionInformation* actionInfo,
			 sensingActionMask* mask)
{
  /* This structure contains all relevant information for the actions. */
  informationsFor_MOVEMENT* info =
    (informationsFor_MOVEMENT*) actionInfo->info[MOVEMENT];

  /* #define DDD */
#ifdef DDD
  return; 
#endif

  if ( mask->perform[MOVEMENT][INTEGRATE_MOVEMENT]) {

    integrateMovement( info);
    
    /* Because the shift of the grid has changed we must check whether
     * all local maxima are still in the grid. */
    if ( actionInfo->useProbGrid)
      removeCellsNotInGrid( &(actionInfo->localMaxima),
			    &(actionInfo->positionProbs));    
  }

  /* Convolve as often as desired. */
  if (mask->perform[MOVEMENT][CONVOLVE_POS_GRID]) {
    convolveGrid( info);
  }
}

/********************************************************************
 ********************************************************************
 * Global functions.
 ********************************************************************
 ********************************************************************/

/**************************************************************************
 * Computes the distancee between two points.
 **************************************************************************/
float
distanceBetweenPoints( realPosition p1, realPosition p2)
{
  return sqrt( fSqr( p1.x - p2.x) + fSqr( p1.y - p2.y));
}


/**************************************************************************
 * Computes the movement between two points in the robot's coordinate system.
 **************************************************************************/
movement
movementBetweenRobotPoints( realPosition start, realPosition end)
{
  return( movementBetweenPoints( robotPosition2CorrectPosition( start),
				 robotPosition2CorrectPosition( end)));
}

/**************************************************************************
 * Computes the movement between two points in the correct coordinate system.
 **************************************************************************/
movement
movementBetweenPoints( realPosition start, realPosition end)
{
  sensing_MOVEMENT delta;

  /* compute forward and sideward sensing_MOVEMENT */
  delta.forward =
    + (end.y - start.y) * sin(start.rot)
    + (end.x - start.x) * cos(start.rot);
  
  delta.sideward =
    - (end.y - start.y) * cos(start.rot)
    + (end.x - start.x) * sin(start.rot);
  
  delta.rotation = end.rot - start.rot;

  return delta;
}

/**************************************************************************
 * Converts a position in the robot's coordinates into a
 * correct position.
 **************************************************************************/
realPosition
robotPosition2CorrectPosition( realPosition robPos)
{
  realPosition pos;

  pos.x   = robPos.x;
  pos.y   = robPos.y;
  pos.rot = normalizedAngle( deg2Rad( - robPos.rot + 90.0));

  return pos;
}


/**************************************************************************
 * Converts a correct position into the robot's coordinates.
 **************************************************************************/
realPosition
correctPosition2RobotPosition( realPosition pos)
{
  realPosition robPos;

  robPos.x   = pos.x;
  robPos.y   = pos.y;
  robPos.rot = - rad2Deg( pos.rot) + 90.0;

  return robPos;
}


/**************************************************************************
 * Converts a position from the sonarint script into the robot's coordinates.
 **************************************************************************/
realPosition
scriptPosition2RobotPosition( realPosition scriptPos)
{
  realPosition pos;

  pos.x   = scriptPos.x;
  pos.y   = scriptPos.y;
  pos.rot = 90.0 - scriptPos.rot;

  return pos;
}


/**************************************************************************
 * Computes the end point given a start point and a sensing_MOVEMENT.
 **************************************************************************/
realPosition
endPoint( realPosition start, movement move)
{
  realPosition end;
  float cosRot;
  float sinRot;

  cosRot =  cos(start.rot);
  sinRot =  sin(start.rot);
  if ( move.forward == 0.0 && move.sideward == 0.0 && move.rotation == 0.0)
    return start;
  
  /* This is the correct formula. */
  /*   end.x = start.x + */
  /*     cos( start.rot) * move.forward + cos( start.rot - DEG_90) * move.sideward; */
  
  /*   end.y = start.y + */
  /*     sin( start.rot) * move.forward + sin( start.rot - DEG_90) * move.sideward; */
  
  
  /* Replaced cos( r - 90) by sin( r) and sin( r - 90) by -cos( r) */
  end.x = start.x + cosRot * move.forward + sinRot * move.sideward;
  end.y = start.y + sinRot * move.forward - cosRot * move.sideward;
  end.rot = normalizedAngle( start.rot + move.rotation);

  return end;
}

/**************************************************************************
 * Computes the end point given a start point and a sensing_MOVEMENT.
 **************************************************************************/
realPosition
endPointOfPolar( realPosition start, polarMovement move)
{
  realPosition end;
  float absoluteAngle = start.rot + move.angle;
  
  end.x = start.x + cos(absoluteAngle) * move.distance;
  end.y = start.y + sin(absoluteAngle) * move.distance;
  end.rot = normalizedAngle( start.rot + move.rotation);

  return end;
}


/**************************************************************************
 * Computes the end grid position given a start position
 **************************************************************************/
gridPosition
endPosition( gridPosition gridStart, positionProbabilityGrid *grid,
	     movement delta)
{
  return
    gridPositionOfRealPosition(
			       endPoint(
					realPositionOfGridPosition(gridStart,
								   grid),
					delta),
			       grid);
}


/********************************************************************
 ********************************************************************
 * The local functions.
 ********************************************************************
 ********************************************************************/

    

/*****************************************************************************
 * Shifts the values by <delta>.
 * The probabilities of the new map positions are also integrated.
 *****************************************************************************/
static void
integrateMovement( informationsFor_MOVEMENT* info)
{
  int plane, deltaX, deltaY;
  static int shiftCnt = 0;
  bool firstTime = TRUE;

  setTimer( 0);
    
  if ( *(info->useProbGrid)) {
    
    /* Just for easier access. */
    
    float resolution = info->grid->positionResolution;
    
    /* Shift the data according to the sensing_MOVEMENT and multiply each position with
     * its probability. This has to be done for each angle. */
    
    for ( plane = 0; plane < info->grid->sizeZ; plane++) {
      
      if ( info->grid->updatePlane[plane]) {
	
	if ( firstTime) {
	  
	  firstTime = FALSE;
	  
	  writeLog("#        Shifting no.%d ( %.0f %.0f %.1f) ... ",
		   ++shiftCnt,
		   info->grid->summedMovementOfPlane[plane].x,
		   info->grid->summedMovementOfPlane[plane].y,
		   rad2Deg(info->grid->summedMovementOfPlane[plane].rot));
	  
	  fprintf( stderr, "# +++++++++++++++++++++++++++++++++++++++++++++++++\n");
	  fprintf( stderr,"#        Shifting no.%d ( %.0f %.0f %.1f) ... ",
		   shiftCnt,
		   info->grid->summedMovementOfPlane[plane].x,
		   info->grid->summedMovementOfPlane[plane].y,
		   rad2Deg(info->grid->summedMovementOfPlane[plane].rot));
	}
	
	deltaX = round( info->grid->summedMovementOfPlane[plane].x / resolution);
	deltaY = round( info->grid->summedMovementOfPlane[plane].y / resolution);
	
	if ( deltaX != 0 || deltaY != 0) {
	  
	  shiftAndMultiplyProbabilities( info->grid,
					 info->aPrioriProbs,
					 plane,
					 info->grid->sizeX, 
					 info->grid->sizeY,
					 deltaX,
					 deltaY,
					 globalMovementParameters.usePosition);
	  
	  info->grid->summedMovementOfPlane[plane].x -= deltaX * resolution;
	  info->grid->summedMovementOfPlane[plane].y -= deltaY * resolution;
	}
      }
    }
    
  }

  /* Shift the sample. */
  else {
    writeLog("#        Shifting no.%d ( %.0f %.0f %.1f) ... ",
	     ++shiftCnt,
	     info->samples->summedMovement.x,
	     info->samples->summedMovement.y,
	     rad2Deg(info->samples->summedMovement.rot));
    
    fprintf( stderr, "# +++++++++++++++++++++++++++++++++++++++++++++++++\n");
    fprintf( stderr,"#        Shifting no.%d ( %.0f %.0f %.1f) ... ",
	     shiftCnt,
	     info->samples->summedMovement.x,
	     info->samples->summedMovement.y,
	     rad2Deg(info->samples->summedMovement.rot));
    
    shiftSamples( info->samples, info->aPrioriProbs);

    /* Copy the updated samples into the tcx set. */
    copySamples( info->samples, info->tcxSamples);
  }

  writeLog( "done in %.2f secs.\n", timeExpired(0));
  fprintf( stderr, "done in %.2f secs.\n", timeExpired(0));
  fprintf( stderr, "# +++++++++++++++++++++++++++++++++++++++++++++++++\n");
}

/*****************************************************************************
 * smoothes the grid to model the uncertainty of the movements. 
 *****************************************************************************/
static void
convolveGrid( informationsFor_MOVEMENT* info)
{
  float time1, time2;
  writeLog( "# Convolving ... ");
  fprintf( stderr, "# Convolving ... ");
  setTimer(0);
  convolveXYPlane( info->grid->prob,
		   info->grid->minimumProbability,
		   info->grid->updatePlane,
		   info->grid->sizeX,
		   info->grid->sizeY,
		   info->grid->sizeZ,
		   info->xyKernel,
		   info->xyKernel);
  time1 = timeExpired(0);
  setTimer(0);

  swallowStatusReports(DONT_WAIT);

  convolveZDimension( info->grid->prob,
		      info->grid->minimumProbability,
		      info->grid->updatePlane,
		      info->grid->sizeX,
		      info->grid->sizeY,
		      info->grid->sizeZ,
		      info->zKernel);
  time2 = timeExpired(0);
  
  writeLog( "done (%.2f+%.2f = %.2fsecs).\n", time1, time2, time1+time2);
  fprintf( stderr, "done (%.2f+%.2f = %.2fsecs).\n",
	   time1, time2, time1+time2);
}




/***********************************************************************
 * Applies a movement action to a list of grid cells
 ***********************************************************************/

void
applyMovement( realCellList *startPositions,
	       movement delta,
	       positionProbabilityGrid *grid,
	       realCellList *endPositions)
{
  register int i;
  
  endPositions->numberOfCellsInMap = 0;
  endPositions->numberOfCells = startPositions->numberOfCells;
  endPositions->originalSumOfProbs = startPositions->originalSumOfProbs;
  
  for (i = 0; i < startPositions->numberOfCells; i++){

    endPositions->cell[i].pos =
      endPoint( startPositions->cell[i].pos, delta);
    endPositions->cell[i].prob = startPositions->cell[i].prob;
  }

  for (i = 0; i < endPositions->numberOfCells; i++) 
    if ( ( endPositions->inMap[i] =
	   realPositionInGrid( endPositions->cell[i].pos.x,
			       endPositions->cell[i].pos.y,
			       grid)))
      endPositions->numberOfCellsInMap++;
}




/***********************************************************************
 * Returns a kernel for convolution of movements.
 ***********************************************************************/
static void
setKernelValues( float* kern, int size)
{
  switch (size) {

  case 1:
    kern[0] = 1.0;
    break;
  case 2:
    kern[0] = 0.94;
    kern[1] = 0.03;
    break;
  case 3:
    kern[0] = 0.8;
    kern[1] = 0.07;
    kern[2] = 0.03;
    break;
  case 4:
    kern[0] = 0.78;
    kern[1] = 0.07;
    kern[2] = 0.03;
    kern[3] = 0.01;
    break;
  default:
    putc( 7, stderr);
    fprintf( stderr, "Error: Kernel size %d not supported (max size 4).\n", size);
    closeLogAndExit(-1);
    return;
  }
}

