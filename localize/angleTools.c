
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/angleTools.c,v $
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
 * $Log: angleTools.c,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.36  2000/03/06 20:00:42  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.35  1999/12/17 18:26:23  fox
 * Second version of precompuations.
 *
 * Revision 1.34  1999/12/16 16:13:58  fox
 * Several preparation changes for angles.
 *
 * Revision 1.33  1999/01/14 00:32:59  wolfram
 * Changes for vision
 *
 * Revision 1.32  1999/01/11 19:47:46  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.30  1998/12/16 14:59:01  wolfram
 * First version without libGetDistance. Use with caution.
 *
 * Revision 1.29  1998/10/02 15:16:36  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.28  1997/11/27 18:11:17  fox
 * Several changes to make angles work better.
 *
 * Revision 1.27  1997/11/20 17:10:13  wolfram
 * Fixed some accesses to not initialized memory using purify
 *
 * Revision 1.26  1997/11/20 12:58:08  fox
 * Version with good sensor selection.
 *
 * Revision 1.25  1997/10/29 09:11:41  wolfram
 * Simulator map is now installed only once
 *
 * Revision 1.24  1997/10/03 12:25:12  wolfram
 * slight changes
 *
 * Revision 1.23  1997/10/01 11:29:56  fox
 * Minor changes.
 *
 * Revision 1.22  1997/09/29 22:08:39  wolfram
 * threshold for computing local maxima now depends on the size of the grid.
 * angles are now approximated by a gaussian.
 *
 * Revision 1.21  1997/09/16 10:10:49  wolfram
 * Angles now have the same amplitude
 *
 * Revision 1.20  1997/09/14 17:33:33  wolfram
 * Direction of walls is now [0:360], angle probabilities are computed
 * using the simulator map if it is available.
 *
 * Revision 1.19  1997/08/09 18:23:31  wolfram
 * Improved computation of angles (aligned readings are weighted
 * according to their distance to the robot.
 *
 * Revision 1.18  1997/08/02 16:51:01  wolfram
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
 * Revision 1.17  1997/03/19 17:52:41  fox
 * New laser parameters.
 *
 * Revision 1.16  1997/03/14 17:58:16  fox
 * This version should run quite stable now.
 *
 * Revision 1.15  1997/03/13 17:36:19  fox
 * Temporary version. Don't use!
 *
 * Revision 1.14  1997/01/31 16:19:16  wolfram
 * Added start position to each reading of a scan
 *
 * Revision 1.13  1997/01/29 12:23:00  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.12  1997/01/10 15:19:20  fox
 * Improved several methods.
 *
 * Revision 1.11  1997/01/08 15:52:53  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.10  1996/12/19 14:33:27  wolfram
 * More sensitive computation of expected distance tables
 *
 * Revision 1.9  1996/12/09 10:11:59  wolfram
 * Fixed a bug in angle computation
 *
 * Revision 1.8  1996/12/04 14:29:58  fox
 * ok
 *
 * Revision 1.7  1996/12/02 10:32:00  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.6  1996/11/29 09:31:38  fox
 * ok
 *
 * Revision 1.5  1996/11/22 16:33:45  fox
 * First version.
 *
 * Revision 1.4  1996/11/18 09:58:29  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.3  1996/10/24 12:07:08  fox
 * Fixed a bug.
 *
 * Revision 1.2  1996/10/24 09:56:51  fox
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


#include <stdio.h>
#include "general.h"
#include "file.h"
#include "angle.h"
#include "angleTools.h"
#include "sonar.h"
#include "laser.h"
#include "allocate.h"
#include "function.h"
#include "graphic.h"
#include "math.h"
#include "movement.h"
#include "proximityTools.h"
#include "graphic.h"

#define NUMBER_OF_POINTS MAX_SIZE_OF_SCAN
#define MAXVARIANCE 10.0
#define ANGLE_MIN_PROB 1e-20

/********************************************************************
 ********************************************************************
 * Forward declarations of internal functions.
 ********************************************************************
 ********************************************************************/


/**************************************************************************
 **************************************************************************
 * Global functions.
 **************************************************************************
 **************************************************************************/

void
normalizeAngleGrid( probability *prob, int numberOfValues){

  normalize1D( prob, numberOfValues,  ANGLE_MIN_PROB);
}


angleProbTable
readAngleProbTable( char* fileName)
{
  
  angleProbTable tab;
  
  FILE* fp;
  int angleCnt;
  probability tmp;
  double sum = 0.0;
  
  if ( fileName == NULL || (fp = fopen( fileName, "r")) == NULL) {
    fprintf( stderr, "# Error: cannot open angle probability file.\n");
    writeLog( "# Error: cannot open angle probability file.\n");
    tab.prob = NULL;
    return tab;
  }
  
  writeLog( "# Reading a priori angles from %s ... ", fileName);
  fprintf( stderr, "# Reading a priori angles from %s ... ", fileName);

  if ( fscanf( fp, "# angles %d",
	       &(tab.numberOfAngles)) < 1) {
    fprintf( stderr, "# Error when reading %s.\n", fileName);
    fprintf( stderr, "%d\n", tab.numberOfAngles);
    tab.prob = NULL;
    return tab;
  }
  
  tab.angleResolution = DEG_360 / ( tab.numberOfAngles);
  tab.isNew = FALSE;
  if ((tab.prob = (probability*)  allocate1D( tab.numberOfAngles, PROBABILITY))
      == NULL){
    char *error = "Error: not enough memory for allocating angle probabilities\n";
    fprintf(stderr, error);
    writeLog(error);
    closeLogAndExit(1);
  }
  
  /* Get the data from the file. */
  for ( angleCnt = 0; angleCnt < tab.numberOfAngles; angleCnt++) {
    if ( fscanf( fp, "%f", &tmp) != 1) {
      fprintf( stderr, "# Wrong data in angle file.\n");
      closeLogAndExit(1);
    }
    sum += tab.prob[angleCnt] = tmp;
  }
  fclose( fp);

  normalizeAngleGrid(tab.prob, tab.numberOfAngles);
  
  writeLog( "done\n");
  fprintf( stderr, "done\n");
  
  return tab;
}

angleProbTable
initializedAngleProbabilityGrid( angleProbTable* angleProbs)
{
  int angleCnt;
  angleProbTable grid;

  grid = *angleProbs;
  grid.prob = (probability*) malloc( grid.numberOfAngles * sizeof( probability));
  
  for ( angleCnt = 0; angleCnt < grid.numberOfAngles; angleCnt++) 
    grid.prob[angleCnt] = 1.0;

  return grid;
}




void
updateWallInformationsScan( sensing_PROXIMITY *scan,
			    sensing_ANGLE* angleSensing,
			    int numberOfAlignedReadings,
			    int minAlignedReadings,
			    bool cyclicScan)
{
  int i,j,k;
  int numberOfReadings;
  realPosition sensingEnd[NUMBER_OF_POINTS];
  bool useSensing[NUMBER_OF_POINTS];
  int numberOfUsedSensors;
  realPosition sensingStart;
  movement tmpReading;
  int wallCnt = angleSensing->numberOfWalls;
  
  float averageX, averageY, sumX2, sumY2, sumXY, averageDist;
  float alpha, variance;

  int bestWallIndex;
  float maxCertainty;

  if ( ! scan->isNew) 
    return;

  if (wallCnt == 0){
    bestWallIndex = 0;
    maxCertainty = 0.0;
  }
  else{
    maxCertainty = angleSensing->bestWall.certainty;
    bestWallIndex = angleSensing->bestWallIndex;
  }
      
  if (cyclicScan)
    numberOfReadings = scan->numberOfReadings;
  else
    numberOfReadings = scan->numberOfReadings - numberOfAlignedReadings;
      
  sensingStart.x = sensingStart.y = sensingStart.rot = 
    tmpReading.rotation = tmpReading.sideward = 0.0;
  sensingStart = endPoint( sensingStart, scan->offset);
  
  
  /* mark sensings to be used */
  for (i = 0; i < scan->numberOfReadings; i++)
    if (scan->reading[i].dist < scan->maxDistance)
      useSensing[i] = TRUE;
    else
      useSensing[i] = FALSE;
  
  
  /* compute the endpoints of the readings */
  for (i = 0; i < scan->numberOfReadings; i++)
    if (useSensing[i]){
      sensingStart.rot = scan->reading[i].rot;
      tmpReading.forward = scan->reading[i].dist;
      sensingEnd[i] = endPoint(sensingStart, tmpReading);
    }
  
  /*
   * Now fit lines for all n-tupels of adjacent values (points)
   */
  for (i = 0; i < numberOfReadings; i++){ /* i=first value */
    
    /* compute number of used sensigs */
    numberOfUsedSensors = 0;
    for (j = 0, k = i; j < numberOfAlignedReadings; j++, k++){
      /* j: over all values */
      if (k >= scan->numberOfReadings) k = 0;
      if (useSensing[k])
	numberOfUsedSensors++;
    }
    
    if (numberOfUsedSensors >= minAlignedReadings){
      /* Compute averages */
      averageX = averageY = averageDist = 0.0;
      for (j = 0, k = i; j < numberOfAlignedReadings; j++, k++){
	if (k >= scan->numberOfReadings) k = 0;
	if (useSensing[k]){
	  averageX += sensingEnd[k].x;
	  averageY += sensingEnd[k].y;
	  averageDist += scan->reading[k].dist;
	}
      }
      averageX /= numberOfUsedSensors;
      averageY /= numberOfUsedSensors;
      averageDist /= numberOfUsedSensors;
      
      /* Compute sumX2, sumY2, and sumXY2 */
      sumX2 = sumY2 = sumXY = 0.0;
      for (j = 0, k = i; j < numberOfAlignedReadings; j++, k++){
	if (k >= scan->numberOfReadings) k = 0;
	if (useSensing[k]){
	  sumX2 += fSqr(sensingEnd[k].x - averageX);
	  sumY2 += fSqr(sensingEnd[k].y - averageY);
	  sumXY += (sensingEnd[k].y - averageY) * (sensingEnd[k].x - averageX);
	}
      }

      if (fAbs(sumX2 - sumY2) > 1e-20 && averageDist >= 100.0){
	variance = (sumX2 + sumY2
		    - sqrt( 4*sumXY*sumXY + fSqr(sumX2 - sumY2))) /
	  ( 2 * numberOfUsedSensors);
	
	/* weight the variance according do the distance of the wall */
	variance *= scan->maxDistance / (2 * averageDist) ;

	if (variance < MAXVARIANCE) {
	  float wallAngle, sinWallAngle;
	  angleSensing->wall[wallCnt].pos.x = averageX;
	  angleSensing->wall[wallCnt].pos.y = averageY;
	  alpha = 0.5 * atan2(-2*sumXY, sumY2 - sumX2);
	  /* alpha is the normal to the linear function. Therefore,
	     the wallAngle is alpha + DEG_90 */
	  wallAngle = normalizedAngle(alpha + DEG_90);
	  sinWallAngle = sin(wallAngle);
	  if (sinWallAngle == 0){
	    if (averageY > 0)
	      wallAngle = DEG_180;
	    else
	      wallAngle = 0.0;
	  }
	  else {
	    /* we compute the intersection of the linear function with
	       the x-axis. If it is right from the origin, the wallAngle
	       is in [0, DEG_180] else it is in [DEG_180, DEG_360] */
	    float lambda = -averageY / sinWallAngle;
	    float x = averageX + lambda * cos(wallAngle);
	    if (x >= 0){
	      if (wallAngle >= DEG_180)
		wallAngle -= DEG_180;
	    }
	    else
	      if (wallAngle < DEG_180)
		wallAngle += DEG_180;
	  }
	  angleSensing->wall[wallCnt].pos.rot = wallAngle;
	  angleSensing->wall[wallCnt].isNew = TRUE;
	  angleSensing->isNew = TRUE; /* at least one wall found */
	  angleSensing->wall[wallCnt].certainty = fSqr(1 - variance/MAXVARIANCE);
	  if (angleSensing->wall[wallCnt].certainty > maxCertainty){
	    if (0) fprintf(stderr, "i: %d, alpha: %f, x: %f, y: %f, cert %f,  dist %f\n", wallCnt, rad2Deg(alpha), averageX, averageY, angleSensing->wall[wallCnt].certainty, averageDist);
	    bestWallIndex = wallCnt;
	    maxCertainty = angleSensing->wall[wallCnt].certainty;
	    if (0) fprintf(stderr, "x %f, y %f\n", averageX, averageY);
	  }
	  wallCnt++;
	}

      }
    }
  }
  /* update the best wall */
  angleSensing->bestWallIndex = bestWallIndex;
  angleSensing->bestWall = angleSensing->wall[bestWallIndex];

  angleSensing->numberOfWalls = wallCnt;

}




void
updateWallInformations( informationsFor_ANGLE* info){

  info->angles.isNew = FALSE;
  info->angles.numberOfWalls = 0;

  if (info->useSonar)
    updateWallInformationsScan( info->sonarScan,
				&(info->angles),
				globalAngleParameters.alignedSonarReadings,
				globalAngleParameters.minAlignedSonarReadings,
				TRUE);
  if (info->useLaser){
    updateWallInformationsScan( info->frontLaserScan,
				&(info->angles),
				globalAngleParameters.alignedLaserReadings,
				globalAngleParameters.minAlignedLaserReadings,
				FALSE);
    updateWallInformationsScan( info->rearLaserScan,
				&(info->angles),
				globalAngleParameters.alignedLaserReadings,
				globalAngleParameters.minAlignedLaserReadings,
				FALSE);
  }
}



probability
probabilityOfAngle( float angle, angleProbTable* angleProbabilities)
{
  int index = round( angle / angleProbabilities->angleResolution)
    % angleProbabilities->numberOfAngles;
  
  return angleProbabilities->prob[index];
}



#define newAngles
#ifdef newAngles

angleProbTable
preprocessedAngleProbabilitiesORG( probabilityGrid* map,
				simulatorMap* simMap,
			        probabilityGrid* aPrioriPositionProbs,
				expectedDistTable *distTab,
				char* fileName)
{
  int x, y, rot;
  int angleIndex;
  int read;
  int wallNumber;
  probability positionProb;
  realPosition realPos;
  FILE *fp;
  bool useSimulatorMap = FALSE;
  char measuredFileName[80];
  
  sensing_PROXIMITY expectedScan;
  static sensing_ANGLE angleSensing;
  distanceReading reading[MAX_SIZE_OF_SCAN];

#define NUMBER_OF_DIRECTIONS 360
#define NUMBER_OF_BEAMS 180
#define OCCUPANCY_THRESHOLD 0.1

  float angleCnt[NUMBER_OF_DIRECTIONS];
  float angleCertainty[NUMBER_OF_DIRECTIONS];
  int   beamAngle[NUMBER_OF_DIRECTIONS];
  
  int robotOrientation = 0;
  int alignedReadings = iMin(6, NUMBER_OF_BEAMS / 6);
  int minAlignedReadings = iMin(4, NUMBER_OF_BEAMS / 6);
  alignedReadings = 30;
  minAlignedReadings = 26;
  
  if (alignedReadings <= minAlignedReadings)
    alignedReadings = minAlignedReadings + 1;
    
  writeLog( "# Computing a priori angle probabilities ...\n"); 

  if (NUMBER_OF_BEAMS > MAX_SIZE_OF_SCAN){
    fprintf(stderr, "Number of beams exceeds maximum size of scan in!\n");
    closeLogAndExit(-1);
  }
  expectedScan.numberOfReadings = NUMBER_OF_BEAMS;
  expectedScan.maxDistance      = distTab->numberOfExpectedDistances * distTab->distanceResolution;
  expectedScan.isNew            = TRUE;
  expectedScan.reading = reading;
  expectedScan.offset.forward = expectedScan.offset.sideward =
    expectedScan.offset.rotation = 0.0;


  if (simMap->initialized)
    useSimulatorMap = TRUE;
  
  for ( read = 0; read < NUMBER_OF_BEAMS; read++) {
    expectedScan.reading[read].rot =
      normalizedAngle(DEG_270 + (DEG_180 * read) / NUMBER_OF_BEAMS);
    beamAngle[read] = deg2Rad( expectedScan.reading[read].rot);
  }
  
  
  for ( rot = 0; rot < NUMBER_OF_DIRECTIONS; rot++) {
    angleCnt[rot] = angleCertainty[rot] = 0.0;
  }
  

  /* We compute the wall only for each position in the map. */
  realPos.rot = 0;
  for ( x = 0; x < map->sizeX; x++) {
    
    if ( x % iMax( 1, (map->sizeX / 10)) == 0)
      fprintf( stderr, "\r# Computing a priori angle probabilities %.2f%%",
	       (float) x / (float) map->sizeX * 100.0);
    
    for ( y = 0; y < map->sizeY; y++) {
      positionProb = aPrioriPositionProbs->prob[x][y];
      if ( map->prob[x][y] != map->unknown && positionProb > 0.9) {
	
	/* Fill in the expected distances at the given position. */
	for ( read = 0; read < NUMBER_OF_BEAMS; read++) {
	  if (useSimulatorMap){
	    expectedScan.reading[read].dist =
	      simulatorObjectDistance( simMap,
				       realPos.x,
				       realPos.y,
				       30,
				       cos(realPos.rot),
				       sin(realPos.rot),
				       SONAR_MAX_EXPECTED_DISTANCE);
	  }
	  else
	    expectedScan.reading[read].dist =
	      /* obstacleDistanceReal( *map, realPos, SONAR_MAX_EXPECTED_DISTANCE); */
	      expectedDistanceGridMap( map, x, y, robotOrientation + beamAngle[read],
				       distTab->distanceResolution,
				       distTab->numberOfExpectedDistances);
	  
	  if (0 && expectedScan.reading[read].dist == 0.0 ||
	      expectedScan.reading[read].dist >= SONAR_MAX_EXPECTED_DISTANCE)
	    expectedScan.reading[read].dist = SONAR_MAX_EXPECTED_DISTANCE;
	}
	angleSensing.numberOfWalls = 0;
	angleSensing.isNew = FALSE;
	/* Now compute the angle of these readings. */
	updateWallInformationsScan( &expectedScan,
				    &angleSensing,
				    alignedReadings,
				    minAlignedReadings,
				    TRUE);
	
	if ( angleSensing.isNew)
	  for (wallNumber = 0; wallNumber < angleSensing.numberOfWalls; wallNumber++){
	    angleIndex = round((rad2Deg(angleSensing.wall[wallNumber].pos.rot)))
	      % NUMBER_OF_DIRECTIONS;
	    while (angleIndex < 0) angleIndex += NUMBER_OF_DIRECTIONS;
	    angleCnt[angleIndex] += positionProb;
	    angleCertainty[angleIndex] += 
	      angleSensing.wall[wallNumber].certainty * positionProb;
	  }
      }
    }
  }
  
  fprintf( stderr, "\r# Computing a priori angle probabilities ... done.\n");

  normalize1D( angleCertainty, NUMBER_OF_DIRECTIONS, 1e-4);
  
  /* now smooth the grid */ 
  {
    float element[2] = {0.8, 0.1};
    kernel kern;
    int i;
    
    kern.size = 2;
    kern.element = element;
    
    for (i= 0; i < 20; i++)
      convolve1DTorus(angleCertainty, NUMBER_OF_DIRECTIONS, kern);
  }
  
  normalize1D( angleCertainty, NUMBER_OF_DIRECTIONS, 1e-6);

  sprintf(measuredFileName, "%s.measured", fileName);
  
  writeLog( " done.\n# Dumping file %s\n", measuredFileName);
  fprintf(stderr, "# Dumping file %s\n", measuredFileName);
  
  if ( (fp = fopen( measuredFileName, "wt")) == NULL) {
    char *error = "# Error: cannot open %s.\n";
    fprintf( stderr, error, measuredFileName);
    writeLog( error, measuredFileName);
    for ( rot = 0; rot < NUMBER_OF_DIRECTIONS; rot++) 
      fprintf( stderr, "%g\n", angleCertainty[rot]);
    closeLogAndExit(1);
  }
  
  fprintf( fp, "# angles %d\n", NUMBER_OF_DIRECTIONS);
  
  for ( rot = 0; rot < NUMBER_OF_DIRECTIONS; rot++) 
    fprintf( fp, "%g\n", angleCertainty[rot]);
  
  fprintf( fp, "# counts\n");
  for ( rot = 0; rot < NUMBER_OF_DIRECTIONS; rot++) 
    fprintf( fp, "# %f\n", angleCnt[rot]);
  
  fclose( fp);

  /* now compute a balanced distribution */

  {
    float balanced[NUMBER_OF_DIRECTIONS];
    int maximum[NUMBER_OF_DIRECTIONS];
    int numberOfMaxima = 0, i;
    
#define MIN_ANGLE_PROB 1e-5;
#define MIN_MAX_ANGLE_PROB 0.005
#define ANGLE_SIGMA 5.0
    for (rot = 0; rot < NUMBER_OF_DIRECTIONS; rot++)
      balanced[rot] = MIN_ANGLE_PROB;;

    for (rot = 0; rot < NUMBER_OF_DIRECTIONS; rot++)
      if (angleCertainty[rot] > MIN_MAX_ANGLE_PROB){
	bool isMaximum;
	
	isMaximum = TRUE;
	for (i = rot - 10; i < rot + 10 && isMaximum; i++){
	  int index;
	  
	  if (i < 0)
	    index = i + NUMBER_OF_DIRECTIONS;
	  else if (i >= NUMBER_OF_DIRECTIONS)
	    index = i - NUMBER_OF_DIRECTIONS;
	  else
	    index = i;
	  
	  if (i != rot)
	    isMaximum = (angleCertainty[rot] > angleCertainty[index]);
	}

	if (isMaximum){
	  maximum[numberOfMaxima++] = rot;
	  fprintf(stderr,"%d\n", rot);
	}
      }

    for (i = 0; i < numberOfMaxima; i++)
      for (rot = 0; rot < NUMBER_OF_DIRECTIONS; rot++)
	balanced[rot] = fMax(balanced[rot],
			     fMax(gauss( (float) (rot - NUMBER_OF_DIRECTIONS),
					 ANGLE_SIGMA,
					 (float) maximum[i]),
				  fMax(gauss((float)(rot+NUMBER_OF_DIRECTIONS),
					     ANGLE_SIGMA,
					     (float) maximum[i]),
				       gauss( (float) rot, ANGLE_SIGMA,
					      (float) maximum[i]))));

    normalize1D( balanced, NUMBER_OF_DIRECTIONS, 1e-4);

    /* now dump the grid */
    
    if ( (fp = fopen( fileName, "wt")) == NULL) {
      char *error = "# Error: cannot open %s.\n";
      fprintf( stderr, error, fileName);
      writeLog( error, fileName);
      for ( rot = 0; rot < NUMBER_OF_DIRECTIONS; rot++) 
	fprintf( stderr, "%g\n", balanced[rot]);
      closeLogAndExit(1);
    }
    else {
      fprintf( fp, "# angles %d\n", NUMBER_OF_DIRECTIONS);
      for ( rot = 0; rot < NUMBER_OF_DIRECTIONS; rot++) 
	fprintf( fp, "%g\n", balanced[rot]);
      fclose(fp);
    }
  }
  
  /* Now get the values into the structure. */
  return readAngleProbTable( fileName);
}

#else


angleProbTable
preprocessedAngleProbabilities( probabilityGrid* map,
				simulatorMap* simMap,
			        probabilityGrid* aPrioriPositionProbs,
				char* fileName)
{

#define NUMBER_OF_WALLS 4
#define NUMBER_OF_DIRECTIONS 360
#define WALL_ANGLE 0.0
#define STANDARD_DEVIATION_OF_ANGLES 6.0
#define DUMMY_FILE_NAME "angleFile"
#define MIN_PROB 0.005
  
  int rotCnt, wallCnt;
  float sum = 0.0;
  float angleCertainty[NUMBER_OF_DIRECTIONS];

  for ( rotCnt = 0; rotCnt < NUMBER_OF_DIRECTIONS; rotCnt++) 
    angleCertainty[rotCnt] = 0.0;

  /* There are four different directions of the walls. */
  for ( wallCnt = 0; wallCnt < NUMBER_OF_WALLS; wallCnt++) {
    
    float wallAngle = WALL_ANGLE + (float) wallCnt * DEG_90;

    /*  Which is the actual angle of the wall? */
    for ( rotCnt = 0; rotCnt < NUMBER_OF_DIRECTIONS; rotCnt++) {
      
      float angleDistance = fAbs( deg2Rad( (float) rotCnt) - wallAngle);

      if ( angleDistance > DEG_180)
	angleDistance = DEG_360 - angleDistance;

      angleCertainty[rotCnt] += gauss( rad2Deg( angleDistance),
				   STANDARD_DEVIATION_OF_ANGLES,
				   0.0);
    }
  }
  
  /* Normalization */
  normalize1D(angleCertainty, NUMBER_OF_DIRECTIONS, 1e-8);

  if ( fileName == NULL)
    fileName = DUMMY_FILE_NAME;

  writeLog( " done.\nDump into %s\n", fileName);
  
  {
      FILE* file;
      
      if ( (file = fopen( fileName, "w")) == NULL) {
	fprintf( stderr, "Error: cannot open %s.\n", fileName);
	for ( rotCnt = 0; rotCnt < NUMBER_OF_DIRECTIONS; rotCnt++) 
	  fprintf( stderr, "%f\n", angleCertainty[rotCnt]);
	closeLogAndExit(1);
      }
      
      fprintf( file, "# angles %d\n", NUMBER_OF_DIRECTIONS);
      
      for ( rotCnt = 0; rotCnt < NUMBER_OF_DIRECTIONS; rotCnt++) 
	fprintf( file, "%g\n", angleCertainty[rotCnt]);
      
      fclose( file);
    }
  
  /* Now get the values into the structure. */
  return readAngleProbTable( fileName);
}

#endif

angleProbTable
preprocessedAngleProbabilities( probabilityGrid* map,
				simulatorMap* simMap,
				probabilityGrid* aPrioriPositionProbs,
				expectedDistTable *distTab,
				char* fileName)
{
  int x, y, rot;
  int angleIndex;
  int read;
  int wallNumber;
  probability positionProb;
  FILE *fp;
  bool useSimulatorMap = FALSE;
  char measuredFileName[80];
  
  sensing_PROXIMITY expected360Scan;
  sensing_PROXIMITY expectedScan;
  static sensing_ANGLE angleSensing;

#define NUMBER_OF_DIRECTIONS 360
#define NUMBER_OF_BEAMS 180
#define OCCUPANCY_THRESHOLD 0.1

  distanceReading reading360[NUMBER_OF_DIRECTIONS];
  distanceReading reading[NUMBER_OF_BEAMS];

  float angleCnt[NUMBER_OF_DIRECTIONS];
  float angleCertainty[NUMBER_OF_DIRECTIONS];
  int   beamAngle[NUMBER_OF_DIRECTIONS];

  point endPoint[NUMBER_OF_DIRECTIONS];
  float sinus[NUMBER_OF_DIRECTIONS];
  float cosinus[NUMBER_OF_DIRECTIONS];
  
  int robotOrientation = 0;
  float oneDegree = DEG_360 / 360.0;
  int alignedReadings = iMin(6, NUMBER_OF_DIRECTIONS / 6);
  int minAlignedReadings = iMin(4, NUMBER_OF_DIRECTIONS / 6);
  alignedReadings = 30;
  minAlignedReadings = 26;
  
  if (alignedReadings <= minAlignedReadings)
    alignedReadings = minAlignedReadings + 1;
    
  writeLog( "# Computing a priori angle probabilities ...\n"); 

  if (NUMBER_OF_DIRECTIONS > MAX_SIZE_OF_SCAN){
    fprintf(stderr, "Number of beams exceeds maximum size of scan in!\n");
    closeLogAndExit(-1);
  }

  expected360Scan.numberOfReadings = NUMBER_OF_DIRECTIONS;
  expected360Scan.maxDistance      = distTab->numberOfExpectedDistances * distTab->distanceResolution;
  expected360Scan.isNew            = TRUE;
  expected360Scan.reading          = reading360;
  expected360Scan.offset.forward   = expected360Scan.offset.sideward =
    expected360Scan.offset.rotation = 0.0;

  expectedScan.numberOfReadings = NUMBER_OF_BEAMS;
  expectedScan.maxDistance      = distTab->numberOfExpectedDistances * distTab->distanceResolution;
  expectedScan.isNew            = TRUE;
  expectedScan.reading          = reading;
  expectedScan.offset.forward = expectedScan.offset.sideward =
    expectedScan.offset.rotation = 0.0;

  for ( read = 0; read < NUMBER_OF_DIRECTIONS; read++) {
    expected360Scan.reading[read].rot =
      normalizedAngle(DEG_270 + oneDegree * read);
    beamAngle[read] = rad2Deg( expected360Scan.reading[read].rot);
    cosinus[read] = cos( expected360Scan.reading[read].rot);
    sinus[read] = sin( expected360Scan.reading[read].rot);
  }
  
  for ( read = 0; read < NUMBER_OF_BEAMS; read++) {
    expectedScan.reading[read].rot =
      normalizedAngle(DEG_270 + oneDegree * read);
  }
  
  for ( rot = 0; rot < NUMBER_OF_DIRECTIONS; rot++) {
    angleCnt[rot] = angleCertainty[rot] = 0.0;
  }
  
  /* We compute the wall only for each position in the map. */
  for ( x = 0; x < map->sizeX; x+=5) {
    
    if ( x % iMax( 1, (map->sizeX / 10)) == 0)
      fprintf( stderr, "\r# Computing a priori angle probabilities %.2f%%",
	       (float) x / (float) map->sizeX * 100.0);
    
    for ( y = 0; y < map->sizeY; y+=5) {
      
      positionProb = aPrioriPositionProbs->prob[x][y];
      
      if ( map->prob[x][y] != map->unknown && positionProb > 0.9) {
	
	/* Fill in the expected scan (360 degs). */
	for ( read = 0; read < NUMBER_OF_DIRECTIONS; read++) {
	  float dist = 
	    /*    	    expectedDistanceGridMap( map, x, y, beamAngle[read],  */
	    /*    				     distTab->distanceResolution,  */
	    /*    				     distTab->numberOfExpectedDistances);  */
	    distTab->dist[x][y][beamAngle[read]] * distTab->distanceResolution;
	  
	  expected360Scan.reading[read].dist = dist;
	  
	  /* Now compute the corresponding end points of the beams. */
/*  	  endPoint[read].x = cosinus[read] * dist; */
/*  	  endPoint[read].y = sinus[read] * dist; */
	}
	
	/* Look at the different robot positions. */
  	for ( robotOrientation = 0; robotOrientation < 360; robotOrientation += 5) { 
	  
	  float centerX = 0.0;
	  float centerY = 0.0;
	  float angle, distance;
	  
	  /* Now compute the center of gravity of the scan. */
	  for ( read = 0; read < NUMBER_OF_BEAMS; read++) {

	    float distIn360Scan =
	      expected360Scan.reading[(read + robotOrientation) % NUMBER_OF_DIRECTIONS].dist;

	    centerX += cosinus[read] * distIn360Scan;
	    centerY += sinus[read] * distIn360Scan;
	  }

	  centerX /= NUMBER_OF_BEAMS;
	  centerY /= NUMBER_OF_BEAMS;
	  
	  distance = sqrt( fSqr( centerX) + fSqr( centerY));
	  angle = normalizedAngle( atan2( centerY, centerX));
	  
	  writeLog( "%f %f %f %f #c\n", centerX, centerY, distance, angle);
	}
      }
    }
    
  }
}








