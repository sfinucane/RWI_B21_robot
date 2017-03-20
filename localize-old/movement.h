
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/movement.h,v $
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
 * $Log: movement.h,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.14  1999/12/28 16:54:45  fox
 * Intermediate version. Might not work.
 *
 * Revision 1.13  1999/09/01 21:26:55  fox
 * Works almost perfectly :-)
 *
 * Revision 1.12  1998/10/02 15:16:40  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.11  1998/09/25 04:02:57  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.10  1997/06/20 07:36:11  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.9  1997/03/13 17:36:38  fox
 * Temporary version. Don't use!
 *
 * Revision 1.8  1997/01/16 12:42:50  fox
 * Fixed a bug in scan masks.
 *
 * Revision 1.7  1997/01/07 12:52:00  wolfram
 * applyMovement now takes a list of localMaxima as input
 *
 * Revision 1.6  1996/12/02 10:32:09  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.5  1996/11/26 11:08:13  fox
 * Improved version.
 *
 * Revision 1.4  1996/11/25 09:47:25  wolfram
 * Added graphic support for movment errors
 * Improved procedure for computing the entropy of a grid
 *
 * Revision 1.3  1996/11/21 14:37:14  wolfram
 * Movement Support for Exploration
 *
 * Revision 1.2  1996/11/21 14:08:05  wolfram
 * First steps toward action selection
 *
 * Revision 1.1.1.1  1996/09/22 16:46:33  rhino
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


#ifndef MOVEMENT_INCLUDE
#define MOVEMENT_INCLUDE

#include "general.h"
#include "sensings.h"
#include "probGrid.h"
#include "function.h"

/*********************************************************
 *********************************************************
 * The following functions and structures are mandatory for
 * each sensor.
 *********************************************************
 *********************************************************/

/* Actions to be performed for the sensing_MOVEMENT. */
#define INTEGRATE_MOVEMENT      0
#define CONVOLVE_POS_GRID       1

#define NUMBER_OF_ACTIONS_MOVEMENT 2

void
initialize_MOVEMENT( char* fileName,
		     actionInformation* actionInfo,
		     sensingActionMask* actionMask,
		     sensingFunctions* handlers);

void
checkIfConsider_MOVEMENT( actionInformation* actionInfo,
			  sensingActionMask* mask);
void
checkWhichActionsToPerform_MOVEMENT( actionInformation* actionInfo,
				     sensingActionMask* mask);

void
performActions_MOVEMENT( actionInformation* actionInfo,
			 sensingActionMask* mask);


/* This struct stores information needed to integrate the movement information. */
typedef struct {
  probabilityGrid*         aPrioriProbs;
  positionProbabilityGrid* grid;
  sampleSet*               samples;
  sampleSet*               tcxSamples;
  sensing_MOVEMENT*        delta;
  int*                     useProbGrid;
  kernel                   xyKernel;
  kernel                   zKernel;
} informationsFor_MOVEMENT;

/*********************************************************
 *********************************************************
 * Structures.
 *********************************************************
 *********************************************************/

/*********************************************************
 *********************************************************
 * Other functions.
 *********************************************************
 *********************************************************/

movement
movementBetweenRobotPoints( realPosition start, realPosition end);

movement
movementBetweenPoints( realPosition start, realPosition end);

float
distanceBetweenPoints( realPosition p1, realPosition p2);

realPosition
endPoint( realPosition start, movement move);

realPosition
endPointOfPolar( realPosition start, polarMovement move);

realPosition
robotPosition2CorrectPosition( realPosition robPos);

realPosition
correctPosition2RobotPosition( realPosition pos);

realPosition
scriptPosition2RobotPosition( realPosition scriptPos);

gridPosition
endPosition( gridPosition gridStart, positionProbabilityGrid *grid,
	     movement delta);

void
applyMovement( realCellList *startPositions,
	       movement delta,
	       positionProbabilityGrid *grid,
	       realCellList *endPositions);

#endif









