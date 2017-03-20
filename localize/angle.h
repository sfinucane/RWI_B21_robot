
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/angle.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:39 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: angle.h,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.6  1999/12/15 16:16:39  fox
 * First attempt to extract p(l | o).
 *
 * Revision 1.5  1997/08/02 16:51:00  wolfram
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
 * Revision 1.4  1997/03/13 17:36:34  fox
 * Temporary version. Don't use!
 *
 * Revision 1.3  1996/12/02 10:32:00  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.2  1996/11/18 09:58:29  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.1.1.1  1996/09/22 16:46:32  rhino
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


#ifndef ANGLE_INCLUDE
#define ANGLE_INCLUDE

#include "general.h"
#include "proximity.h"
#include "probGrid.h"
#include "sensings.h"
#include "function.h"

/*********************************************************
 *********************************************************
 * The following functions and structures are mandatory for
 * each sensor.
 *********************************************************
 *********************************************************/

/* Actions to be performed for the angles. */
#define INTEGRATE_ANGLE_GRID    0
#define UPDATE_ANGLE_GRID       1
#define CONVOLVE_ANGLE_GRID     2

#define NUMBER_OF_ACTIONS_ANGLE 3

typedef struct {
  wall   wall[MAX_SIZE_OF_SCAN];
  wall   bestWall;
  int    bestWallIndex;
  int    numberOfWalls;
  bool   isNew;
} sensing_ANGLE;


void
initialize_ANGLE( char* fileName,
		  actionInformation* actionInfo,
		  sensingActionMask* actionMask,
		  sensingFunctions* handlers);

void
checkIfConsider_ANGLE( actionInformation* actionInfo,
		       sensingActionMask* mask);

void
checkWhichActionsToPerform_ANGLE( actionInformation* actionInfo,
				  sensingActionMask* mask);

void
performActions_ANGLE( actionInformation* actionInfo,
		      sensingActionMask* mask);

/*********************************************************
 *********************************************************
 * Structures.
 *********************************************************
 *********************************************************/

typedef struct {
  float angleResolution;
  int numberOfAngles;
  bool isNew;
  probability* prob;
  probability minProb;
  probability integrationThreshold;
} angleProbTable;

/* This struct stores information needed to integrate the angle information. */
typedef struct {
  sensing_PROXIMITY*       sonarScan;
  bool useSonar;
  sensing_PROXIMITY*       frontLaserScan;
  sensing_PROXIMITY*       rearLaserScan;
  bool useLaser;
  positionProbabilityGrid* positionProbs;
  sensing_ANGLE            angles;
  angleProbTable           aPrioriAngleProbs;
  angleProbTable           angleProbs;
  kernel                   angleKernel;
  sensingActionMask*       actionMask;
} informationsFor_ANGLE;

/*********************************************************
 *********************************************************
 * Other functions.
 *********************************************************
 *********************************************************/
#define NUMBER_OF_MINIMA_TILL_THRESHOLD 2

#endif


















