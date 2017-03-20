
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection-old/laser.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:01 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: laser.c,v $
 * Revision 1.1  2002/09/14 20:45:01  rstone
 * *** empty log message ***
 *
 * Revision 1.10  1999/01/27 16:33:43  fox
 * Nothing special.
 *
 * Revision 1.9  1998/09/05 00:25:27  fox
 * Works fine to detect people in the museum.
 *
 * Revision 1.8  1998/08/23 22:57:41  fox
 * First version of building maps of humans.
 *
 * Revision 1.7  1997/05/28 14:04:10  fox
 * Fixed a bug.
 *
 * Revision 1.6  1997/05/28 09:01:29  wolfram
 * added motion-only mode
 *
 * Revision 1.5  1997/05/09 16:28:39  fox
 * Works quiet fine.
 *
 * Revision 1.4  1997/05/06 14:22:58  fox
 * Nothing special.
 *
 * Revision 1.3  1997/05/06 08:19:53  fox
 * Added several messages.
 *
 * Revision 1.2  1997/05/05 16:54:07  fox
 * Incorporated graphics from people.c.
 *
 * Revision 1.1.1.1  1997/04/30 16:17:09  fox
 * Module to detect unexpected obstacles.
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
#include "allocate.h"
#include "laser.h"
#include "function.h"
#include "localTcx.h"
#include "DETECTION-messages.h"

#define FRONT_LASER 0
#define REAR_LASER 1

#define PIONEER_ATRV
#ifdef B18_ROBOT
#define FRONT_LASER_OFFSET 35.5
#define REAR_LASER_OFFSET -10.0
#else
#ifdef PIONEER_ATRV
#define FRONT_LASER_OFFSET 8.0
#define REAR_LASER_OFFSET 0.0
#else
#define FRONT_LASER_OFFSET 11.5
#define REAR_LASER_OFFSET -11.5
#endif
#endif


#define LASER_OPENING_ANGLE                      M_PI / 60;
#define LASER_HEIGHT                                40
#define LASER_MAX_EXPECTED_DISTANCE 500
#define LASER_NUMBER_OF_EXPECTED_DISTANCES 128
#define LASER_NUMBER_OF_STDDEVS 2
/* the product of the latter two must be 256 */

gridPosition frontGrid, rearGrid;

extern int selectedDetectionMode;

static float
stdDevThreshold_LASER[LASER_NUMBER_OF_STDDEVS] = {20.0, MAXFLOAT},
stdDevRangeClose_LASER[LASER_NUMBER_OF_STDDEVS] = {4.0,100.0},
stdDevRangeFar_LASER[LASER_NUMBER_OF_STDDEVS] = {4.0,100.0},
probFactorClose_LASER[LASER_NUMBER_OF_STDDEVS] = {0.1, 0.1},
probFactorFar_LASER[LASER_NUMBER_OF_STDDEVS] = {0.1, 10.0},
minimumProbClose_LASER[NUMBER_OF_STDDEVS] = {0.9, 0.1},
minimumProbFar_LASER[NUMBER_OF_STDDEVS] = {0.0001, 0.1},
additionalMaxRangeProb_LASER[NUMBER_OF_STDDEVS] = {0.0, 0.0};

distProbTable normedDistProbFuncTab;
expectedDistTable expectedDistances;
distanceScan laserScan;
float laserMaxRange = LASER_MAX_EXPECTED_DISTANCE;
int sizeOfEnvironmentX, sizeOfEnvironmentY;

/********************************************************************
 ********************************************************************
 * Forward declarations of internal functions.
 ********************************************************************
 ********************************************************************/

static void
computeDistProbFunctionParameter_LASER( sensorParameters *params);

static expectedDistTable
readDistTab( char* fileName);

static distProbTable
probFunctionTable( sensorParameters params,
		   expectedDistTable expDist);

/* A value of the distance probability function. */
static float 
distProbFunction( sensorParameters params,
		  int stdDevIndex,
                  int expectedDistIndex,
                  int measuredDistIndex);

/********************************************************************
 ********************************************************************
 * Global functions.
 ********************************************************************
 ********************************************************************/

void
initializeLaser( char* file, int readDistances)
{
  sensorParameters params;
  
  if ( readDistances) {
    expectedDistances = readDistTab( file);
    
    computeDistProbFunctionParameter_LASER( &params);
    
    normedDistProbFuncTab = probFunctionTable( params,
					       expectedDistances);
  }
  
  initLaserScan( &laserScan);
}


void
updateLaserMapPositions()
{
  realPosition tmp;
  float robRot = robotPosition.rot;
  
  laserScan.frontRealMap = robotPosition;
  laserScan.rearRealMap = robotPosition;

  robotCoordinates2MapCoordinates( robotPosition.x,
				   robotPosition.y,
				   uncorrectedRobotRot,
				   correctionX,
				   correctionY,
				   correctionRot,
				   correctionType,
				   &(laserScan.frontRealMap.x),
				   &(laserScan.frontRealMap.y),
				   &(laserScan.frontRealMap.rot));  
  
  laserScan.frontRealMap.x += FRONT_LASER_OFFSET * cos( robRot);
  laserScan.frontRealMap.y += FRONT_LASER_OFFSET * sin( robRot);

  /* Set the grid position. */
  frontGrid = gridPositionOfRealPosition( laserScan.frontRealMap, mapResolution);

  robotCoordinates2MapCoordinates( robotPosition.x,
				   robotPosition.y,
				   uncorrectedRobotRot,
				   correctionX,
				   correctionY,
				   correctionRot,
				   correctionType,
				   &(laserScan.rearRealMap.x),
				   &(laserScan.rearRealMap.y),
				   &(laserScan.rearRealMap.rot));  


  laserScan.rearRealMap.x += REAR_LASER_OFFSET * cos( robRot);
  laserScan.rearRealMap.y += REAR_LASER_OFFSET * sin( robRot);
  
  rearGrid = gridPositionOfRealPosition( laserScan.rearRealMap, mapResolution);
}

int
indexIsOfFrontLaser( int index)
{
  return index < REAR_START_INDEX || index >= FRONT_START_INDEX;
}


float
probOfFeature( int feature,
	       float sensorRot,
	       gridPosition pos)
{
  int distTabPlane =
    round( (Deg2Rad(pos.rot) + sensorRot) / expectedDistances.angleResolution)
    % expectedDistances.sizeZ;

  int expectedDist = expectedDistances.dist[pos.x][pos.y][distTabPlane];

#ifdef TEST
  fprintf( stderr, "%d %d --> %f\n", feature, expectedDist, normedDistProbFuncTab.prob[expectedDist][feature]);
#endif
  return normedDistProbFuncTab.prob[expectedDist][feature];
}

float
expectedDist( gridPosition pos, float sensorRot)
{
  int distTabPlane =
    round( (Deg2Rad(pos.rot) + sensorRot) / expectedDistances.angleResolution)
    % expectedDistances.sizeZ;

  return expectedDistances.dist[pos.x][pos.y][distTabPlane]
    / expectedDistances.numberOfStdDevs * expectedDistances.distanceResolution;
}

		    
int
expectedFeature( gridPosition pos, float sensorRot)
{
  int distTabPlane =
    round( (Deg2Rad(pos.rot) + sensorRot) / expectedDistances.angleResolution)
    % expectedDistances.sizeZ;
  
  return expectedDistances.dist[pos.x][pos.y][distTabPlane]
    / expectedDistances.numberOfStdDevs;
}

		    

void
updateLaserFeatures()
{
  int i;

  for ( i = 0; i < laserScan.numberOfReadings; i++)
    laserScan.reading[i].feature =
      iMin( laserScan.numberOfFeatures,
	    (int) round ( laserScan.reading[i].dist / laserScan.distanceResolution));

}

/********************************************************************
 ********************************************************************
 * The local functions.
 ********************************************************************
 ********************************************************************/



static void
computeDistProbFunctionParameter_LASER( sensorParameters *params)
{
  params->numberOfStdDevs = LASER_NUMBER_OF_STDDEVS;
  params->numberOfExpectedDistances =
    params->numberOfMeasuredDistances =
    LASER_NUMBER_OF_EXPECTED_DISTANCES;
  params->sensorMaxRange = LASER_MAX_EXPECTED_DISTANCE;
  params->openingAngle = LASER_OPENING_ANGLE;
  params->sensorHeight = LASER_HEIGHT;
  params->stdDevThreshold = stdDevThreshold_LASER;
  params->stdDevRangeClose = stdDevRangeClose_LASER;
  params->stdDevRangeFar = stdDevRangeFar_LASER;
  params->probFactorClose = probFactorClose_LASER;
  params->probFactorFar = probFactorFar_LASER;
  params->minimumProbClose = minimumProbClose_LASER;
  params->minimumProbFar = minimumProbFar_LASER;
  params->additionalMaxRangeProb = additionalMaxRangeProb_LASER;
}


/***********************************************************************
 * Initializes the structures containing the raw laser data as well
 * as the laser data in abstract sensor structures.
 **********************************************************************/
void
initLaserScan( distanceScan* laserScan)
{
  int i;

  /*------------------------------------------------------------------------
   * Initialze the global structure of raw sensors.
   *-----------------------------------------------------------------------*/
  laserScan->numberOfReadings = MAX_SIZE_OF_LASER_SCAN;
  laserScan->reading          = (distanceReading*)
    malloc( MAX_SIZE_OF_LASER_SCAN * sizeof( distanceReading));
  laserScan->maxDistance      = LASER_MAX_EXPECTED_DISTANCE;
  laserScan->isNew            = FALSE;

  laserScan->numberOfFeatures = normedDistProbFuncTab.numberOfMeasuredDistances;
  laserScan->distanceResolution = normedDistProbFuncTab.distanceResolution;

  for ( i = 0; i < MAX_SIZE_OF_LASER_SCAN; i++)
    laserScan->reading[i].rot = (float) i * DEG_360 / MAX_SIZE_OF_LASER_SCAN;
}




/*****************************************************************************
 * Reads the distance table from <fileName>.
 *****************************************************************************/
static expectedDistTable
readDistTab( char* fileName)
{
  FILE* file;
  int x, y, z;
  expectedDistance dummy;
  expectedDistTable tab;

  if ( (file = fopen( fileName, "r")) == NULL) {
    fprintf( stderr, "# Error: cannot open %s.\n", fileName);
    tab.dist = NULL;
    return tab;
  }
  
  fscanf( file, "%d %d %d", &(tab.sizeX), &(tab.sizeY), &(tab.sizeZ));
  fprintf( stderr,
	   "# Reading expected dists from %s (size is %d X %d X %d) ... ",
	   fileName, tab.sizeX, tab.sizeY, tab.sizeZ);

  /* Allocate memory. */
  tab.dist = (expectedDistance***)
    allocate3D( tab.sizeX, tab.sizeY, tab.sizeZ, EXPECTED_DISTANCE);
  
  fscanf( file, "%f %f %d %d%c",
	  &(tab.angleResolution),
	  &(tab.distanceResolution),
	  &(tab.numberOfExpectedDistances),
	  &(tab.numberOfStdDevs),
	  &dummy);

  /* Set the global values. */
  sizeOfEnvironmentX = tab.sizeX;
  sizeOfEnvironmentY = tab.sizeY;

  for ( x = 0; x < tab.sizeX; x++) 
    for ( y = 0; y < tab.sizeY; y++)
      for ( z = 0; z < tab.sizeZ; z++) 
	tab.dist[x][y][z] = (expectedDistance) fgetc( file);
  fclose( file);

  fprintf( stderr, "done\n");

  return tab;
}
  

/*****************************************************************************
 * Computes the probabilities of sensor measurements for given expected
 * measurements.
 *****************************************************************************/
static distProbTable
probFunctionTable( sensorParameters params,
		   expectedDistTable expDist)
{
   int measuredCnt, expectedCnt, stdDevCnt, expectedIndex;
   distProbTable table;
   
   table.distanceResolution        = expDist.distanceResolution;
   table.numberOfExpectedDistances =
      table.numberOfMeasuredDistances = expDist.numberOfExpectedDistances;
   table.numberOfStdDevs = expDist.numberOfStdDevs;
   
   
  /***********************************************************************
   * Allocate memory.
   ***********************************************************************/
   table.prob = (float**)
      allocate2D( table.numberOfExpectedDistances * table.numberOfStdDevs,
		 table.numberOfMeasuredDistances,
		 FLOAT);
   
   /***********************************************************************
    * Fill in the probabilities.
    ***********************************************************************/
   for ( expectedCnt = 0;
	 expectedCnt < table.numberOfExpectedDistances; expectedCnt++) 
     for (stdDevCnt = 0; stdDevCnt < table.numberOfStdDevs; stdDevCnt++){
       expectedIndex = expectedCnt*table.numberOfStdDevs + stdDevCnt;
       for ( measuredCnt = 0;
	     measuredCnt < table.numberOfMeasuredDistances; measuredCnt++){
	 table.prob[expectedIndex][measuredCnt] =
	   distProbFunction( params,
			     expectedCnt,
			     stdDevCnt,
			     measuredCnt);
       }
     }
   
   if (1) {
     char fileName[80];
     static int tmpcount = 0;
     FILE *fp;
     for (stdDevCnt = 0; stdDevCnt < table.numberOfStdDevs; stdDevCnt++) {
	sprintf(fileName, "dist%d-%d.gnu", tmpcount, stdDevCnt);
	fp = fopen(fileName, "wt");
	fprintf(fp, "# %d %d\n", table.numberOfExpectedDistances,
		table.numberOfMeasuredDistances);
	for (measuredCnt = 0;
	     measuredCnt < table.numberOfMeasuredDistances; measuredCnt++) {
	  for (expectedCnt = 0;
	       expectedCnt < table.numberOfExpectedDistances; expectedCnt++){
	    expectedIndex = expectedCnt*table.numberOfStdDevs + stdDevCnt;
	    fprintf(fp, "%.3e\n", table.prob[expectedIndex][measuredCnt]);
	  }
	  fprintf(fp,"\n");
	}
	fclose(fp);
     }
     tmpcount++;
   }
   
   return table;
}


static float 
distProbFunction( sensorParameters params,
		  int expectedDistIndex,
		  int stdDevCnt,
		  int measuredDistIndex)
{
  float sigmaSlope, sigmaOffset, factorSlope, factorOffset, sigma, factor;
  
  sigmaSlope = (params.stdDevRangeFar[stdDevCnt] -
    params.stdDevRangeClose[stdDevCnt]) / params.numberOfMeasuredDistances;
  sigmaOffset = params.stdDevRangeClose[stdDevCnt];
  
  factorSlope = (params.probFactorFar[stdDevCnt]
		 - params.probFactorClose[stdDevCnt])
    / params.numberOfMeasuredDistances;
  factorOffset = params.probFactorClose[stdDevCnt];
  
  sigma = linFunction( sigmaSlope, sigmaOffset, expectedDistIndex);
  
  factor = ( params.minimumProbClose[stdDevCnt] -
	     params.minimumProbFar[stdDevCnt]) /
    gauss( 0.0, sigma, 0.0);
  
  if ( measuredDistIndex <= expectedDistIndex)    
    return fMin( params.minimumProbClose[stdDevCnt],
		 params.minimumProbClose[stdDevCnt] -
		 factor * gauss( measuredDistIndex, sigma, expectedDistIndex));
  else
    return params.minimumProbFar[stdDevCnt];
}




