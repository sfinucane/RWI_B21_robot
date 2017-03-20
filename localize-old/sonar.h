
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/sonar.h,v $
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
 * $Log: sonar.h,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.11  1999/06/25 19:48:14  fox
 * Minor changs for the urbie.
 *
 * Revision 1.10  1998/10/29 03:45:06  fox
 * Nothing special.
 *
 * Revision 1.9  1998/03/02 06:35:25  wolfram
 * Added output of sensor errror values. Small change in graphic.c
 *
 * Revision 1.8  1997/09/29 16:29:42  wolfram
 * Angles now handle the situation where no sensor is given
 *
 * Revision 1.7  1997/08/02 16:51:09  wolfram
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
 * Revision 1.6  1997/03/13 17:36:39  fox
 * Temporary version. Don't use!
 *
 * Revision 1.5  1997/01/30 17:17:27  fox
 * New version with integrated laser.
 *
 * Revision 1.4  1997/01/29 12:23:15  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.3  1996/12/09 10:12:02  wolfram
 * Fixed a bug in angle computation
 *
 * Revision 1.2  1996/12/02 10:32:15  wolfram
 * Expected distance file now includes distances + variances
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


#ifndef SONAR_INCLUDE
#define SONAR_INCLUDE

#include "general.h"
#include "sensings.h"
#include "probGrid.h"
#include "proximity.h"
#include "activeLocalize.h"

/*********************************************************
 *********************************************************
 * The following functions and structures are mandatory for
 * each sensor.
 *********************************************************
 *********************************************************/

/* Action to be performed for the sonars. */
#define INTEGRATE_SONAR         0

#define NUMBER_OF_ACTIONS_SONAR 1

#define SONAR_MAX_EXPECTED_DISTANCE 630
#define SONAR_HEIGHT                100

typedef struct {
  bool aaaiSonars;
  float integrateThreshold;
  float maxQuotaOfPlanes;
  sensorParameters parameters;
  bool useSonar;
  int robotType;
  int numberOfScansToBeCombined;
} sonarParameters;


typedef struct {
  int numberOfScans;
  int sizeOfScans;
  int currentScanNumber; 
  abstractSensorVector* scan;
  realPosition* position;
  distanceScan* distances;
} combinedAbstractScan;


void
initialize_SONAR( char* fileName,
		  actionInformation* actionInfo,
		  sensingActionMask* actionMask,
		  sensingFunctions* handlers);

void
checkIfConsider_SONAR( actionInformation* actionInfo,
		       sensingActionMask* mask);

void
checkWhichActionsToPerform_SONAR( actionInformation* actionInfo,
				  sensingActionMask* mask);

void
performActions_SONAR( actionInformation* actionInfo,
		      sensingActionMask* mask);

/*********************************************************
 *********************************************************
 * Structures.
 *********************************************************
 *********************************************************/


/* This struct stores information needed to integrate the sonar information. */
typedef struct {
  informationsFor_PROXIMITY  general;
} informationsFor_SONAR;


extern int numberOfSonars;

/*********************************************************
 *********************************************************
 * Functions.
 *********************************************************
 *********************************************************/

float
sonarRot(int i);

realPosition
sonarPosition( realPosition pos, int num);

distance
obstacleDistanceReal_SONAR( probabilityGrid m, realPosition pos,
			   distance maxRange);

distance
obstacleDistance_SONAR( probabilityGrid m,
		       int x, int y, float rot,
		       distance maxRange);

#endif




