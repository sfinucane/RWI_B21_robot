
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection-old2/detection.c,v $
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
 * $Log: detection.c,v $
 * Revision 1.1  2002/09/14 20:45:04  rstone
 * *** empty log message ***
 *
 * Revision 1.17  1998/09/05 00:25:25  fox
 * Works fine to detect people in the museum.
 *
 * Revision 1.16  1998/08/29 21:44:42  fox
 * Changed the detection report and improved stuck detection.
 *
 * Revision 1.15  1998/08/23 22:57:39  fox
 * First version of building maps of humans.
 *
 * Revision 1.14  1997/06/03 11:49:14  fox
 * Museum version.
 *
 * Revision 1.13  1997/05/28 14:04:09  fox
 * Fixed a bug.
 *
 * Revision 1.12  1997/05/28 12:47:32  wolfram
 * new version
 *
 * Revision 1.11  1997/05/28 11:11:34  wolfram
 * parameter tuning
 *
 * Revision 1.10  1997/05/28 09:01:28  wolfram
 * added motion-only mode
 *
 * Revision 1.9  1997/05/28 07:17:56  fox
 * is egal.
 *
 * Revision 1.8  1997/05/25 10:46:40  fox
 * Fixed a bug.
 *
 * Revision 1.7  1997/05/25 10:40:50  fox
 * Nothing special.
 *
 * Revision 1.6  1997/05/25 10:39:10  thrun
 * test.
 *
 * Revision 1.5  1997/05/09 16:28:38  fox
 * Works quiet fine.
 *
 * Revision 1.4  1997/05/06 14:22:56  fox
 * Nothing special.
 *
 * Revision 1.3  1997/05/06 08:19:52  fox
 * Added several messages.
 *
 * Revision 1.2  1997/05/05 16:54:05  fox
 * Incorporated graphics from people.c.
 *
 * Revision 1.1.1.1  1997/04/30 16:17:09  fox
 * Module to detect unexpected obstacles.
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include "general.h"
#include "PANTILT-messages.h"
#include "detection.h"
#include "allocate.h"
#include "function.h"
#include "laser.h"
#include "localTcx.h"
#include "graphics.h"
#include "o-graphics.h"
#include "pantilt.h"

#include "DETECTION-messages.h"


/* int detectionMode = STOP_DETECTION; */
int prevMode = STOP_DETECTION;
int detectionMode = LOOK_FOR_UNEXPECTED;

#define NOTHING_SEEN 0
#define SEEN_SOMETHING_ONCE 1
#define WAIT 2
#define SEEN_A_PERSON 3

static bool verbose = FALSE;
static int detectionState = NOTHING_SEEN; 

#define UNEXPECTED_THRESHOLD 0.7
#define UNEXPECTED_EXPIRE_THRESHOLD 3.0

#define ROTATIONAL_SPEED_THRESHOLD 10.0

/********************************************************************
 ********************************************************************
 * Local functions.
 ********************************************************************
 ********************************************************************/
static void
computeLikelihoodsOfReadings( gridPosition currentPos,
			      int start,
			      int number,
			      float* likelihoods);

static void
extractObstacles( float* evaluations,
		  float threshold,
		  detectionStruct* obstacles);

static void
checkForPeople( detectionStruct* obstacles,
		float expiredThreshold);


/********************************************************************
 * Detects unexpected readings and puts the information into the
 * detection struct.
 ********************************************************************/
void
detectUnexpectedObstacles( detectionStruct* obstacles)
{
  float likelihoods[MAX_SIZE_OF_SCAN];
  
  setTimer(0);
  
  if ( rotationalSpeed > ROTATIONAL_SPEED_THRESHOLD) {
    obstacles->foundSomething = FOUND_NOTHING;
    return;
  }

  if ( prevMode != obstacles->detectionMode) 
    detectionState = NOTHING_SEEN;
    
  /* Compute the likelihood of each reading given the current position. */
  if ( frontGrid.x != rearGrid.x ||
       frontGrid.y != rearGrid.y) {

    /* Front laser. */
    computeLikelihoodsOfReadings( frontGrid,
				  FRONT_START_INDEX,
				  laserScan.numberOfReadings / 2,
				  likelihoods);
    /* Rear laser. */
    computeLikelihoodsOfReadings( rearGrid,
				  REAR_START_INDEX,
				  laserScan.numberOfReadings / 2,
				  likelihoods);
  }
  else
    /* Both lasers at the same position. */
    computeLikelihoodsOfReadings( frontGrid,
				  0,
				  laserScan.numberOfReadings,
				  likelihoods);
    
  /* Extract the relevant information for the angles. */
  extractObstacles( likelihoods,
		    UNEXPECTED_THRESHOLD,
		    obstacles);


  checkForPeople( obstacles,
		  UNEXPECTED_EXPIRE_THRESHOLD);

  displayDetection( obstacles);

  prevMode = obstacles->detectionMode;
}



/********************************************************************
 * Detects motion and puts the information into the
 * detection struct.
 ********************************************************************/
void
detectMotion( detectionStruct* obstacles)
{
  static int firstTime = TRUE;
  static distanceScan prevScan;

  float distances[MAX_SIZE_OF_SCAN];
  int i;

  if ( firstTime) {
    initLaserScan( &prevScan);
    firstTime = FALSE;
  }

  /* No detection of motion possible if the robot moves. */
  if ( robotInMotion) {
    obstacles->foundSomething = FOUND_NOTHING;
    stopPanTilt();
    displayDetection( obstacles);
    return;
  }

  /* First call after change of mode. Just  copy the current scan
   * and wait until next call. */
  if ( prevMode != obstacles->detectionMode) {
    
    /* Initialize the scan. Nothing else to be done at the first call. */
    for ( i = 0; i < prevScan.numberOfReadings; i++)
      prevScan.reading[i].dist = laserScan.reading[i].dist;

    obstacles->foundSomething = FOUND_NOTHING;
    prevMode = obstacles->detectionMode;
    detectionState = NOTHING_SEEN;
    displayDetection( obstacles);
    return;
  }
  
  /* Compute the distances. */
  
  for ( i = 0; i < prevScan.numberOfReadings; i++) {
    distances[i] = fMin( THRESHOLD,
			 fAbs( prevScan.reading[i].dist - laserScan.reading[i].dist));
  }
  
  /* Extract the relevant information for the angles. */
  extractObstacles( distances,
		    MOTION_THRESHOLD,
		    obstacles);

  checkForPeople( obstacles,
		  MOTION_EXPIRE_THRESHOLD);

  displayDetection( obstacles);

  if (0) {
    static int first = TRUE;
    
    if ( first && (obstacles->foundSomething != FOUND_NOTHING)) {
      sayHello();
      first = FALSE;
    }
    else
      first = TRUE;
  }
  
  /* Copy the current scan for the next call. */
  for ( i = 0; i < prevScan.numberOfReadings; i++)
    prevScan.reading[i].dist = laserScan.reading[i].dist;
  prevMode = obstacles->detectionMode;
}



/**********************************************************************
 ********************************************************************/
static void
computeLikelihoodsOfReadings( gridPosition currentPos,
			      int first,
			      int number,
			      float* likelihoods)
{
  /* We search around the current position and treat the other cells as local
   * maxima. */
  gridCell cell;
  int deltaRot = 2;
  int deltaPosX = 2;
  int deltaPosY = 2;
  int rot;
  int currentX = currentPos.x;
  int currentY = currentPos.y;
  int currentRot = currentPos.rot;

  int i, j;

  static int firstTime = TRUE;
  static gridCellList cellList;

  /* Initialize the cube around the current position. */
  if ( firstTime) {
    int x,y,rot;
    int cellCnt = 0;
    float prob, probSum = 0.0;
    
#define STD_DEV 2.5

    for ( x = - deltaPosX; x <= deltaPosX; x++)
      for ( y = - deltaPosY; y <= deltaPosY; y++)
	for ( rot = - deltaRot; rot <= deltaRot; rot++) {
	  float dist = sqrt( fSqr( x) + fSqr( y) + fSqr( rot));
	  prob = gauss( dist, STD_DEV, 0.0);
	  cellList.cell[cellCnt++].prob = prob;
	  probSum += prob;
	}
    cellCnt = 0;prob=0;
    for ( x = - deltaPosX; x <= deltaPosX; x++)
      for ( y = - deltaPosY; y <= deltaPosY; y++)
	for ( rot = - deltaRot; rot <= deltaRot; rot++) 
	  cellList.cell[cellCnt++].prob /= probSum;
    
    firstTime = FALSE;
  }
	  
  cellList.numberOfCells = 0;
  

  for ( rot = currentRot - deltaRot;
	rot <= currentRot + deltaRot;
	rot+=1) {
    
    if ( rot > 359)
      cell.pos.rot = rot - 360;
    else if ( rot < 0)
      cell.pos.rot = rot + 360;
    else
      cell.pos.rot = rot;
    
    for ( cell.pos.x = currentX - deltaPosX;
	  cell.pos.x <= currentX + deltaPosX;
	  cell.pos.x+=1) 
      
      if (cell.pos.x >= 0 && cell.pos.x < sizeOfEnvironmentX) {
	
	for ( cell.pos.y = currentY - deltaPosY;
	      cell.pos.y <= currentY + deltaPosY;
	      cell.pos.y+=1)
	  
	  if (cell.pos.y >= 0 && cell.pos.y < sizeOfEnvironmentY) {	    
	    cellList.cell[cellList.numberOfCells].pos = cell.pos;
	    cellList.numberOfCells++;
	}
      }
  } 

  /*--------------------------------------------------------------
   *--------------------------------------------------------------*/
  for ( i = 0; i < number; i++) {

    int sensorNumber = i + first; 
    
    if ( sensorNumber >= laserScan.numberOfReadings)
      sensorNumber -= laserScan.numberOfReadings;
    
    likelihoods[sensorNumber] = 0.0;
    
    /*--------------------------------------------------------------
     * Now compute the likelihood.
     *--------------------------------------------------------------*/
    
    for ( j = 0; j < cellList.numberOfCells; j++) {

      likelihoods[sensorNumber] += cellList.cell[j].prob *
	probOfFeature( laserScan.reading[sensorNumber].feature,
		       laserScan.reading[sensorNumber].rot,
		       cellList.cell[j].pos);
    }
#ifdef TEST
fprintf(stderr, "beam %d --> %f\n", sensorNumber, likelihoods[sensorNumber]);
#endif

  }
}




static void
extractObstacles( float* evaluations,
		  float threshold,
		  detectionStruct* obstacles)
{
  
  int i;
  int sensorsToBeClustered = 360 / NUMBER_OF_DETECTION_ANGLES;
  int numberOfAngles = NUMBER_OF_DETECTION_ANGLES;

  float clusterOffset = Deg2Rad( (float) sensorsToBeClustered / 2.0);

  if ( obstacles->numberOfAngles < numberOfAngles) {

    if ( obstacles->distances != NULL)
      free( obstacles->distances);
    obstacles->distances = (float*) allocate1D( numberOfAngles, FLOAT);
    obstacles->prev_distances = (float*) allocate1D( numberOfAngles, FLOAT);
    obstacles->measuredDistances = (float*) allocate1D( numberOfAngles, FLOAT);
    obstacles->angles = (float*) allocate1D( numberOfAngles, FLOAT);
    obstacles->numberOfAngles = numberOfAngles;
    
    for ( i = 0; i < numberOfAngles; i++) {
      obstacles->prev_distances[i] = 0.0;
      obstacles->distances[i] = -1.0;
      obstacles->measuredDistances[i] = -1.0;
      obstacles->angles[i] = 
	laserScan.reading[i * sensorsToBeClustered - sensorsToBeClustered / 2].rot;
    }
  }
  
  for ( i = 0; i < laserScan.numberOfReadings; i++) 
    
    /* Check for the angle cluster. */
    if ( i> 0 && i % sensorsToBeClustered == 0) {
      
      int angleNumber = i / sensorsToBeClustered;
      int clusterIndex;
      float minDist = 1e5;
      float minAngle = 0.0;
      float averageOfCluster = 0.0;
      for ( clusterIndex = i; clusterIndex < i + sensorsToBeClustered;
	    clusterIndex++) {
	averageOfCluster += evaluations[clusterIndex];
	if ( laserScan.reading[i].dist < minDist) {
	  minDist = laserScan.reading[i].dist;
	  minAngle = laserScan.reading[i].rot;
	}
      }
      
      averageOfCluster /= sensorsToBeClustered;

      /* Store the raw distances. */
      obstacles->measuredDistances[angleNumber] = minDist;      

      /* Found something unexpected. */
      if ( averageOfCluster > threshold && minDist < MAX_DETECTION_RANGE) {
	
	float relativeAngle = clusterOffset + laserScan.reading[i].rot;

	obstacles->distances[angleNumber] = minDist;
	obstacles->angles[angleNumber] = minAngle;
	
	if ( verbose) {
	  for ( clusterIndex = i; clusterIndex < i + sensorsToBeClustered;
		clusterIndex++) {
	    if (0) fprintf(stderr, "%f  %f  %f %f\n",
		    expectedDist( rearGrid, laserScan.reading[clusterIndex].rot),
		    evaluations[clusterIndex],
		    laserScan.reading[clusterIndex].dist,
		    laserScan.reading[clusterIndex].rot);
	  }
	  
	  fprintf( stderr, "%f --> %f  (%f)\n",
		   (float) Rad2Deg( relativeAngle),
		   obstacles->distances[angleNumber],
		   averageOfCluster);
	}
      }
      else {
	obstacles->distances[angleNumber] = 0.0;
      }
    }
}


/* Checks wether an object has been found. This can be used
 * for unexpected objects and for moving objects.  Any kind
 * of object has to be confirmed once before it is inserted into
 * the detectionStruct. */
static void
checkForPeople( detectionStruct* obstacles,
		float expiredThreshold)
{
  int i, minIndex = 0;
  int foundSomething = FALSE;
  float minDist = 1e5;
  float minAngle = 0.0;
  realPosition currentPerson;
  
  static realPosition lastPerson;

  obstacles->foundSomething = FOUND_NOTHING;

  /* Just for display. */
  if ( useGraphics)
    G_clear_markers(PERSON);

  /* Search for the closest detected obstacle. */
  for (i = 0; i < obstacles->numberOfAngles; i++)
    if (obstacles->distances[i] > 0.0 && obstacles->distances[i] < minDist) {
      foundSomething = TRUE;
      minDist = obstacles->distances[i];
      minAngle = obstacles->angles[i] + robotPosition.rot;
      minIndex = i;
    }
  
  /* Found something interesting. Compute the position. */
  if ( foundSomething) {
    if ( indexIsOfFrontLaser(minIndex)) { 
      currentPerson.x = laserScan.frontRealRob.x + minDist * cos( minAngle);
      currentPerson.y = laserScan.frontRealRob.y + minDist * sin( minAngle);
    }
    else {
      currentPerson.x = laserScan.rearRealRob.x + minDist * cos( minAngle);
      currentPerson.y = laserScan.rearRealRob.y + minDist * sin( minAngle);
    }      
  }
  
  switch (detectionState) {
    
    /* Nothing seen yet. */
  case NOTHING_SEEN:

    if ( foundSomething) {
#ifdef CHILDREN
      detectionState = SEEN_A_PERSON;
      /* Object confirmed. Put it into the struct. */
      obstacles->foundSomething = FOUND_SOMETHING;
      obstacles->index = minIndex;
      obstacles->detected.x = currentPerson.x;
      obstacles->detected.y = currentPerson.y;	
#else
      detectionState = SEEN_SOMETHING_ONCE;
#endif
    }
    break;

    /* Seen something thing for the first time. Still need confirmation. */
  case SEEN_SOMETHING_ONCE:
    
#define MAX_CONFIRMATION_DIST 50.0

    if ( foundSomething) {
      /* Is it the same object? */
      if ( positionDist( currentPerson.x, currentPerson.y,
			 lastPerson.x, lastPerson.y) < MAX_CONFIRMATION_DIST) {

	/* Object confirmed. Put it into the struct. */
	obstacles->foundSomething = FOUND_SOMETHING;
	obstacles->index = minIndex;
	obstacles->detected.x = currentPerson.x;
	obstacles->detected.y = currentPerson.y;	
	
	/* Just graphics. */
	if ( useGraphics) {
	  EZX_bell();
	  G_add_marker( PERSON,
			minDist * cos( DEG_90 + minAngle - robotPosition.rot),
			minDist * sin( DEG_90 + minAngle - robotPosition.rot),
			0);
	}

	detectionState = SEEN_A_PERSON;
      }
    }
    else
      /* Not confirmed. "Wait for new object. */
      detectionState = NOTHING_SEEN;
    break;
    
  case SEEN_A_PERSON:

    if ( ! foundSomething) {
      /* Lost object. Wait until timer expired. */
      setTimer( DETECTION_TIMER);
      detectionState = WAIT;
    }
    else {
      /* Found object again. Sca this one. */
      obstacles->foundSomething = FOUND_SOMETHING;
      obstacles->index = minIndex;
      obstacles->detected.x = currentPerson.x;
      obstacles->detected.y = currentPerson.y;	
      
      putc(7, stderr);

      /* Display it. */
      if (useGraphics)
	G_add_marker( PERSON,
		      minDist * cos( DEG_90 + minAngle - robotPosition.rot),
		      minDist * sin( DEG_90 + minAngle - robotPosition.rot),
		      0);
    }
    
    break;

  case WAIT:
    
    if ( foundSomething) {
      /* Return to confirmation state. */
      detectionState = SEEN_SOMETHING_ONCE;
    }
    else if ( timeExpired( DETECTION_TIMER) > expiredThreshold) {
      /* OK! Nothing new, so stop the pantilt. */
      stopPanTilt();
      detectionState = NOTHING_SEEN;
    }
    break;
  }

  {
    static int previosDetectionState = -1;

    /* Show the state. */
    if ( detectionState != previosDetectionState) {
      if ( useGraphics)
	G_display_switch(PERSON_BUTTON, detectionState);
      previosDetectionState = detectionState;
    }
  }

  lastPerson = currentPerson;
}



