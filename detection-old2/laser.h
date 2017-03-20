
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection-old2/laser.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:04 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: laser.h,v $
 * Revision 1.1  2002/09/14 20:45:04  rstone
 * *** empty log message ***
 *
 * Revision 1.8  1998/09/05 00:25:28  fox
 * Works fine to detect people in the museum.
 *
 * Revision 1.7  1998/08/23 22:57:41  fox
 * First version of building maps of humans.
 *
 * Revision 1.6  1997/05/28 14:04:11  fox
 * Fixed a bug.
 *
 * Revision 1.5  1997/05/09 16:28:39  fox
 * Works quiet fine.
 *
 * Revision 1.4  1997/05/06 14:22:59  fox
 * Nothing special.
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/



#ifndef LASER_INCLUDE
#define LASER_INCLUDE

#include "general.h"

/*********************************************************
 *********************************************************
 * The following functions and structures are mandatory for
 * each sensor.
 *********************************************************
 *********************************************************/

#define FRONT_START_INDEX 270
#define REAR_START_INDEX 90


/* Laser parameter */
/* the product of the latter two must be 256 */
#define MAX_SIZE_OF_LASER_SCAN 360

#define LASER_MAX_EXPECTED_DISTANCE 500
#define NUMBER_OF_EXPECTED_DISTANCES 128
#define NUMBER_OF_STDDEVS 2

/*********************************************************
 *********************************************************
 * Structures.
 *********************************************************
 *********************************************************/

#define MAX_SIZE_OF_SCAN 400

typedef unsigned char expectedDistance;

typedef struct {
  int numberOfStdDevs;
  int numberOfExpectedDistances;
  int numberOfMeasuredDistances;
  float sensorMaxRange;
  float openingAngle;
  float sensorHeight;
  float *stdDevThreshold;
  float *stdDevRangeClose;
  float *stdDevRangeFar;
  float *probFactorClose;
  float *probFactorFar;
  float *minimumProbClose;
  float *minimumProbFar;
  float *additionalMaxRangeProb;
} sensorParameters;

/* This struct is used to get the probability of a measured distance
 * given the expected distance.
 */
typedef struct {
  distance distanceResolution;
  int numberOfMeasuredDistances;
  int numberOfExpectedDistances;
  int numberOfStdDevs;
  float** prob;
} distProbTable;

/* This struct is used to get the probability of a measured distance
 * given the expected distance.
 */
typedef struct {
  int sizeX;
  int sizeY;
  int sizeZ;
  float angleResolution;
  distance distanceResolution;
  int numberOfExpectedDistances;
  int numberOfStdDevs;
  expectedDistance*** dist;
} expectedDistTable;


/*********************************************************
 *********************************************************
 * Functions.
 *********************************************************
 *********************************************************/

/* extern realPosition mapPosition; */
extern gridPosition frontGrid;
extern gridPosition rearGrid;

extern distanceScan laserScan;
extern float laserMaxRange;
extern int sizeOfEnvironmentX, sizeOfEnvironmentY;

void
initializeLaser( char* file, int readDistances);


void
initLaserScan( distanceScan* laserScan);

void
updateLaserMapPositions();

int
indexIsOfFrontLaser( int index);

void
updateLaserFeatures();

float
probOfFeature( int measuredFeature,
	       float sensorRot,
	       gridPosition pos);
		     
float
expectedDist( gridPosition pos, float sensorRot);
		    
int
expectedFeature( gridPosition pos, float sensorRot);

#endif
