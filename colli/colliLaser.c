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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliLaser.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:25:05 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: colliLaser.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.21  2000/07/12 16:53:29  fox
 * Allows to send rear laser sweeps even when they are not used for collision avoidance.
 *
 * Revision 1.20  1999/12/06 13:19:05  fox
 * Added time stamps to base and laser report.
 *
 * Revision 1.19  1999/04/18 19:00:05  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
 *
 * Revision 1.18  1998/12/10 17:45:06  fox
 * Changed some settings for the pioneer at.
 *
 * Revision 1.17  1998/11/22 23:54:20  fox
 * Fixed a bug for rectangular robots. If tvel == 0, then the distance
 * to the next obstacle is set to zero. This is done to avoid too much
 * rotation.
 *
 * Revision 1.16  1998/11/19 03:14:31  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.15  1998/11/12 18:36:59  fox
 * Version running on the pioneers.
 *
 * Revision 1.14  1998/11/08 20:27:49  fox
 * FIXED A BUG FOR RECTANGULAR ROBOTS (DON'T ASK).
 *
 * Revision 1.13  1998/10/23 20:50:29  fox
 * *** empty log message ***
 *
 * Revision 1.12  1998/09/05 00:20:00  fox
 * Version for internet night in Washington.
 *
 * Revision 1.11  1998/08/29 21:50:02  fox
 * Adapted exception handling in order to deal with people around the robot.
 *
 * Revision 1.10  1998/08/26 23:23:40  fox
 * Final version before I changed exception handling.
 *
 * Revision 1.9  1998/08/18 16:24:23  fox
 * Added support for b18 robot.
 *
 * Revision 1.8  1998/01/14 00:37:25  thrun
 * New option "-laserserver" lets the base receive laser date from
 * the laserServer. This is the recommended option. It seems to be
 * much more reliable than reading in the data directly.
 *
 * Revision 1.7  1997/06/25 14:37:14  fox
 * Changed the laser messages.
 *
 * Revision 1.6  1997/04/17 09:19:38  fox
 * Minor changes.
 *
 * Revision 1.5  1997/04/17 09:16:19  fox
 * Added timeout for laser devices --> colliServer only needs 30% cpu.
 *
 * Revision 1.4  1997/03/14 18:40:52  fox
 * Fixed a bug.
 *
 * Revision 1.3  1997/02/04 18:00:32  fox
 * Added paramters to BASE_setMode.
 *
 * Revision 1.2  1997/01/07 13:47:14  fox
 * Improved rotate_away.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:04  rhino
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


#include "collisionIntern.h"
#include "laser_interface.h"

LaserPoints Laser_CollPoints;

#define MAX_REMEMBER_TIME 15.0

#define NUMBER_OF_COLLPOINTS_PER_LASER 45

float LASER_UPDATE_TIME = 1.0;
float LASER_PING_TIME = 0.2;

static int laserVerbose =  FALSE;

/***************************************************************************
  * forward declarations.
 ***************************************************************************/
static Point
laserCenter( Point rpos, float rRot,
	     float laserOffset, float laserAngle);

static void
setLaserPoints( float laserOffset, float laserAngle,
		LASER_reading readings);

/***************************************************************************
  * Initializes all values necessary in the structure for the laser points. 
 ***************************************************************************/
void
initLaserPointsStructure()
{
  int i;
  struct timeval now;

  gettimeofday( &now, 0);
    
  Laser_CollPoints.maxNumberOfPoints = MAX_NUMBER_OF_LASER_POINTS;
  Laser_CollPoints.numberOfPoints    = 0;
  Laser_CollPoints.maxRememberTime = MAX_REMEMBER_TIME;
  
  Laser_CollPoints.points = (struct Point*)
    malloc( MAX_NUMBER_OF_LASER_POINTS * sizeof(struct Point));
  
  Laser_CollPoints.times = (struct timeval*)
    malloc( MAX_NUMBER_OF_LASER_POINTS * sizeof(struct timeval));
  
  for ( i = 0; i < MAX_NUMBER_OF_LASER_POINTS; i++) {
    Laser_CollPoints.points[i].x = F_ERROR;
    Laser_CollPoints.times[i] = now;
  }
}


#define GAP_BETWEEN_RESTARTS 0.25

void
checkLaserPing( struct timeval* now)
{
  if ( ! use_laser_server && ! use_simulator) {
    
    static struct timeval lastFrontRestart;
    static struct timeval lastRearRestart;
    
    float frontTDiff = timeDiff( now, &Last_FrontLaserPoint_Update);
    float rearTDiff = timeDiff( now, &Last_RearLaserPoint_Update);

    /* Check front laser and send request if necessary. */
    if ( USE_FRONT_LASER && (frontTDiff > LASER_PING_TIME)) {
      if ( timeDiff( now, &lastFrontRestart) > GAP_BETWEEN_RESTARTS) {
	gettimeofday( &lastFrontRestart, 0);
	requestNextLaserScan( FRONT_LASER);
	if (0) fprintf(stderr, "Ping front laser.");
      }
    }
    
    /* Check rear laser and send request if necessary. */
    if ( USE_REAR_LASER && (rearTDiff > LASER_PING_TIME)) {
      if ( timeDiff( now, &lastRearRestart) > GAP_BETWEEN_RESTARTS) {
	gettimeofday( &lastRearRestart, 0);
	requestNextLaserScan( REAR_LASER);
	if (0) fprintf(stderr, "Ping rear laser.");
      }
    }
  }
}

/**********************************************************************
 * Updates the structure containing the points of the obstacles as detected
 * by the laser range finders. 
 **********************************************************************/
void
update_LaserPoints( Pointer callback_data, Pointer client_data)
{
  int i;

  if ( frontLaserReading.new) {

    if ( laserVerbose) {
      struct timeval now;
      static struct timeval last;
      
      gettimeofday( &now, 0);
      fprintf(stderr, "td %f\n", timeDiff( &now, &last));
      gettimeofday( &last, 0);
    }

    setLaserPoints( FRONT_LASER_OFFSET, FRONT_LASER_ANGLE,
		    frontLaserReading);

    frontLaserReading.new = FALSE;
    
    Last_FrontLaserPoint_Update = frontLaserReading.timeStamp;
  }
  
  if ( rearLaserReading.new) {
	
    setLaserPoints( REAR_LASER_OFFSET, REAR_LASER_ANGLE,
		    rearLaserReading);

    rearLaserReading.new = FALSE;
    
    Last_RearLaserPoint_Update = rearLaserReading.timeStamp;
  }

  /* Copy the laser points into the global obstacle point structure. */
  combinedObstaclePoints[LASER_POINTS].no_of_points =
    Laser_CollPoints.numberOfPoints;
  
  for ( i = 0; i < Laser_CollPoints.numberOfPoints; i++)
    combinedObstaclePoints[LASER_POINTS].points[i] =
      Laser_CollPoints.points[i];

  COLLI_update_tcx_LaserPoints();
}

/***************************************************************************
 * The center points of the two laser scanners given the robot's position.
 ***************************************************************************/
static Point
laserCenter( Point rPos, float rRot,
	     float laserOffset, float laserAngle)
{
  Point center;

  center.x = rPos.x + (fcos( rRot + laserAngle) * laserOffset);
  center.y = rPos.y + (fsin( rRot + laserAngle) * laserOffset);

  return center;
}


/******************************************************************
 * Converts the distances in <readings> into absolute points, given
 * the robot position and the position of the laser.
 ******************************************************************/
static void
setLaserPoints( float laserOffset, float laserAngle,
		LASER_reading readings)
{
  int actualCnt, combineCnt, minIndex = -1;
  int lastDefined;
  struct timeval now;
  BOOLEAN verbose = FALSE;
  Point lCenter;
  float lRot, minReading = LASER_MAX_RANGE;
  int pointsToCombine = readings.numberOfReadings / NUMBER_OF_COLLPOINTS_PER_LASER;

  /* No time check in the first call. */
  static BOOLEAN firstTime = TRUE;

  /* Delete all points in the actual scan area. This area is in front of
   * the laser center position. We substract one cm because of rounding
   * errors. */
  Point scanAreaCenter   = laserCenter( readings.rPos, readings.rRot,
					laserOffset + 0.1, laserAngle);
  
  float scanAreaRotation = readings.rRot + laserAngle;

  gettimeofday( &now, 0);

  
  /* First check which points to delete:
   * Delete all points in the actual scan area
   * Delete all points which are too old.
   */
  for ( actualCnt = 0, lastDefined = 0;
	actualCnt < Laser_CollPoints.numberOfPoints;
	actualCnt++) {

    /* Too old? */
    if ( (firstTime
	  || timeDiff( &now, &(Laser_CollPoints.times[actualCnt])) 
	  <= Laser_CollPoints.maxRememberTime)) {

      /* All points behind the laser center have to be remembered. */
      if ( behindPoint( scanAreaCenter, scanAreaRotation,
			Laser_CollPoints.points[actualCnt])) {
	Laser_CollPoints.points[lastDefined] = Laser_CollPoints.points[actualCnt];
	Laser_CollPoints.times[lastDefined]  = Laser_CollPoints.times[actualCnt];
	lastDefined++;
	if (verbose) fprintf( stderr, "keep   : %d\n", actualCnt);
      }
      else
	if (verbose) fprintf( stderr, "Remove : %d\n", actualCnt);
    }
    else {
      if (verbose) fprintf( stderr, "Too old: %d\n", actualCnt);
    }
  }

  /* Now insert the actual readings. */
  lCenter = laserCenter( readings.rPos, readings.rRot,
			     laserOffset, laserAngle);
  
  lRot    = readings.rRot;
  
   /* actualCnt points to the actual readings.
   * lastDefined points to the laser points. */   
  for ( actualCnt = 0, combineCnt = 0;
	actualCnt < readings.numberOfReadings;
	actualCnt++, combineCnt++) {

    if ( (combineCnt > 0) && combineCnt % pointsToCombine == 0) {
      
      /* We found a minimal reading in the last combineCnt readings. */
      if ( minIndex > 0) {
	
	float beamRot = 
	  lRot + readings.startAngle + readings.angleResolution * minIndex;
	
	if ( lastDefined < Laser_CollPoints.maxNumberOfPoints) {
	  Laser_CollPoints.points[lastDefined].x =
	    lCenter.x + (fcos( beamRot) * readings.reading[minIndex]);
	  Laser_CollPoints.points[lastDefined].y =
	    lCenter.y + (fsin( beamRot) * readings.reading[minIndex]);
	  Laser_CollPoints.times[lastDefined] = now;
	  lastDefined++;
	}
	else {
	  fprintf( stderr, "Too many readings (max: %d).\n",
		   Laser_CollPoints.maxNumberOfPoints);
	  break;
	}
      }
      minIndex = -1;
      minReading = LASER_MAX_RANGE;
    }
    
    if ( readings.reading[actualCnt] > 0.0
	 && readings.reading[actualCnt] <= minReading) {
      minIndex = actualCnt;
      minReading = readings.reading[actualCnt];
    }
  }

  Laser_CollPoints.numberOfPoints = lastDefined ;

  if (verbose) fprintf( stderr, "remembered points %d\n", lastDefined);

  firstTime = FALSE;
}

