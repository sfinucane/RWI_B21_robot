
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/rectangularRobot.c,v $
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
 * $Log: rectangularRobot.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.9  1999/06/25 19:48:09  fox
 * Minor changs for the urbie.
 *
 * Revision 1.8  1999/03/09 15:48:43  wolfram
 * Added geometry of PIONEER_II
 *
 * Revision 1.7  1998/11/22 23:54:22  fox
 * Fixed a bug for rectangular robots. If tvel == 0, then the distance
 * to the next obstacle is set to zero. This is done to avoid too much
 * rotation.
 *
 * Revision 1.6  1998/11/19 03:14:33  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.5  1998/11/12 18:37:01  fox
 * Version running on the pioneers.
 *
 * Revision 1.4  1998/11/08 20:27:52  fox
 * FIXED A BUG FOR RECTANGULAR ROBOTS (DON'T ASK).
 *
 * Revision 1.3  1998/10/30 18:20:51  fox
 * Added support for pioniers.
 *
 * Revision 1.2  1998/10/23 20:50:34  fox
 * *** empty log message ***
 *
 * Revision 1.1  1998/09/09 08:01:34  wolfram
 * Adding missing files
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include "collisionIntern.h"
#include "colliTrajectories.h"

/* #define STRAIGHT_TRAJECTORY_THROUGH_CENTER  */
/* #define STRAIGHT_TRAJECTORY_THROUGH_WHOLE_ROBOT */

/* After all values are determined we assume that pt1 is the one closer to the
 * center of the rotation. */
typedef struct rotatingLine {
  Point center;       /* Center of rotation. */
  Point pt1;          
  Point pt2;
  float innerRadius;  
  float outerRadius;
  float innerAngle;
  float outerAngle;
} rotatingLine;

rectangularRobot b18 = {27.0, 27.0, 51.0, 18.0};

rectangularRobot urban = {30.0, 30.0, 35.0, 35.0};

#define SMALL_WHEELS
#ifdef SMALL_WHEELS
rectangularRobot pioneer = {25.0, 25.0, 21.0, 33.0};
#else
rectangularRobot pioneer = {26.0, 26.0, 21.0, 33.0};
#endif
rectangularRobot rectRob;

rectangularRobot pioneer_II = {18.5, 18.5, 16.0, 27.0};

void
setRectangularRobot( int robotType)
{
  if ( robotType == B18_ROBOT)
    rectRob = b18;
  else if ( robotType == PIONEER_ATRV)
    rectRob = pioneer;
  else if ( robotType == PIONEER_II)
    rectRob = pioneer_II;
  else if ( robotType == URBAN_ROBOT)
    rectRob = urban;
  else {
    fprintf(stderr, "Only know b18, pioneer, and urban as rectangular robots.\n");
    exit(0);
  }
}

/* Determines the absolute positions of the lines describing the robot.
 * The lines start at the front left corner and procede counterclockwise. */
void
determineRectangularRobotLines( Point center, float rrot,
				float rvel, float securitySize,
				float addSpeedSize, LineSeg* lines)
{
  Point frontLeft, frontRight, rearLeft, rearRight;
  float cosRot = cos(rrot);
  float sinRot = sin(rrot);

  float leftWidth = rectRob.centerToLeft + securitySize;
  float rightWidth = rectRob.centerToRight + securitySize;

  float centerToFront = rectRob.centerToFront + securitySize;
  float centerToRear = rectRob.centerToRear + securitySize;

  /* Add the speed dependent security distance to the width of the robot. */
  if ( rvel > 0) {
    /* Right turn. */
    leftWidth += (1.0 - ACTUAL_MODE->edge_portion) * addSpeedSize;
    rightWidth += ACTUAL_MODE->edge_portion * addSpeedSize;
  }
  else {
    /* Left turn. */
    leftWidth += ACTUAL_MODE->edge_portion * addSpeedSize;
    rightWidth += (1.0 - ACTUAL_MODE->edge_portion) * addSpeedSize;
  }

  frontLeft.x = center.x +
    cosRot * centerToFront + cos(rrot + DEG_90) * leftWidth;
  frontLeft.y = center.y +
    sinRot * centerToFront + sin(rrot + DEG_90) * leftWidth;
  
  frontRight.x = center.x +
    cosRot * centerToFront + cos(rrot - DEG_90) * rightWidth;
  frontRight.y = center.y +
    sinRot * centerToFront + sin(rrot - DEG_90) * rightWidth;

  rearLeft.x = center.x -
    cosRot * centerToRear - cos(rrot - DEG_90) * leftWidth;
  rearLeft.y = center.y -
    sinRot * centerToRear - sin(rrot - DEG_90) * leftWidth;

  rearRight.x = center.x -
    cosRot * centerToRear - cos(rrot + DEG_90) * rightWidth;
  rearRight.y = center.y -
    sinRot * centerToRear - sin(rrot + DEG_90) * rightWidth;

  lines[LEFT_LINE].pt1 = frontLeft;
  lines[LEFT_LINE].pt2 = rearLeft;
  
  lines[REAR_LINE].pt1 = rearLeft;
  lines[REAR_LINE].pt2 = rearRight;
  
  lines[RIGHT_LINE].pt1 = rearRight;
  lines[RIGHT_LINE].pt2 = frontRight;
  
  lines[FRONT_LINE].pt1 = frontRight;
  lines[FRONT_LINE].pt2 = frontLeft;

  /* Now add some centimeters in order to cover the whole area. */
  if (1) {
#define ADD_FOR_SAFETY 2.0
    float addToFrontX = cosRot * ADD_FOR_SAFETY;
    float addToFrontY = sinRot * ADD_FOR_SAFETY;
    
    float addToSideX  = cos(rrot - DEG_90) * ADD_FOR_SAFETY;
    float addToSideY  = sin(rrot - DEG_90) * ADD_FOR_SAFETY;
    
    lines[LEFT_LINE].pt1.x += addToFrontX;
    lines[LEFT_LINE].pt1.y += addToFrontY;
    lines[LEFT_LINE].pt2.x -= addToFrontX;
    lines[LEFT_LINE].pt2.y -= addToFrontY;
    
    lines[RIGHT_LINE].pt1.x -= addToFrontX;
    lines[RIGHT_LINE].pt1.y -= addToFrontY;
    lines[RIGHT_LINE].pt2.x += addToFrontX;
    lines[RIGHT_LINE].pt2.y += addToFrontY;
    
    lines[REAR_LINE].pt1.x -= addToSideX;
    lines[REAR_LINE].pt1.y -= addToSideY;
    lines[REAR_LINE].pt2.x += addToSideX;
    lines[REAR_LINE].pt2.y += addToSideY;

    lines[FRONT_LINE].pt1.x += addToSideX;
    lines[FRONT_LINE].pt1.y += addToSideY;
    lines[FRONT_LINE].pt2.x -= addToSideX;
    lines[FRONT_LINE].pt2.y -= addToSideY;
  }
}


/* Determines the rings corresponding to the rotating lines. May also change the
 * ordering of the two points . */
static void
setRotatingLines( rotatingLine* rotLines)
{
  int i;
  
  for ( i = 0; i < NUMBER_OF_LINES_IN_ROBOT; i++) {
    if ( rotLines[i].pt1.x == F_ERROR)
      rotLines[i].innerRadius = rotLines[i].outerRadius = 0.0;
    else {
      float dist1 = compute_distance( rotLines[i].center, rotLines[i].pt1);
      float dist2 = compute_distance( rotLines[i].center, rotLines[i].pt2);

      if (dist1 < dist2) {
	rotLines[i].innerRadius = dist1;
	rotLines[i].outerRadius = dist2;
      }
      else {
	/* Swap the points so that the first point is always the inner point. */
	Point tmp = rotLines[i].pt1;
	rotLines[i].pt1 = rotLines[i].pt2;
	rotLines[i].pt2 = tmp;
	
	rotLines[i].innerRadius = dist2;
	rotLines[i].outerRadius = dist1;
      }
    }
  }
}
  


/*--------------------------------------------------------------------
 * Determines the four lines representing the sweep areas around the center.
 * Each line determines a ring. For each ring we can determine the distance
 * to the next obstacle on this ring.
 * --------------------------------------------------------------------*/
static void
determineRotatingLines(Point rpos, float rrot, float tvel,
		       float rvel, float securitySize, float addSpeedSize,
		       rotatingLine* rotLines, LineSeg* robotLines)
{
  Circle_trajectory tmp;
  int line;
  int nothingCanHappenAtRear = FALSE;
  int rightTurn = rvel > 0.0;

  LineSeg lineFacingObstacles, lineAtRear;
  LineSeg left,right,front,rear;
  float dist;
  Point meetPoint;
  int positionOfMeetPoint;
  
  if (rvel != 0.0) {

    float rad_rot, rdir, tdir, radius;
    Point center;

    /* Convert the current location of the robot into the locations of
     * the lines. */
    determineRectangularRobotLines( rpos, rrot, rvel,
				    securitySize, addSpeedSize, robotLines);
	
    /* Determine the center of the circular trajectory. */
    rdir = (rvel >= 0.0) ? 1.0 : -1.0;
    tdir = (tvel >= 0.0) ? 1.0 : -1.0;
    rad_rot = rrot - (rdir*tdir) * DEG_90;
    radius = fabs (tvel / rvel);
    
    center.x = rpos.x + fcos( rad_rot) * radius;
    center.y = rpos.y + fsin( rad_rot) * radius;

    for ( line = 0; line < NUMBER_OF_LINES_IN_ROBOT; line++)
      rotLines[line].center = center;
    
    /* Check for each line of the robot the ring covered by this line when
     * rotating around the center. Just because I'm too stupid to find a general
     * solution and because of lack of time to find someone more clever, I treat
     * each line seperately. */
    
    /* The meetPoint is given by the closest point of the center of the trajectory and
     * the line defined by the corresponding robot line. The lines which sweep over
     * the collision area are denoted by left, right,front, and rear, respectively. */

    /* First deal with the left and right line. */
    
    /* -------- Left line. -------- */
    dist = minDistanceToLine( center, robotLines[LEFT_LINE],
			      &meetPoint, &positionOfMeetPoint);
    
    /* The collision part of the line is given by the meetPoint
     * and the rear or front corner. */
    rotLines[LEFT_LINE].pt1 = meetPoint;
    rotLines[LEFT_LINE].pt2 = rightTurn ?
      robotLines[LEFT_LINE].pt2 : robotLines[LEFT_LINE].pt1;
    
    /* -------- Right line. -------- */
    dist = minDistanceToLine( center, robotLines[RIGHT_LINE],
			      &meetPoint, &positionOfMeetPoint);
    
    /* The collision part of the line is given by the meetPoint
     * and the rear or front corner. */
    rotLines[RIGHT_LINE].pt1 = meetPoint;
    rotLines[RIGHT_LINE].pt2 = rightTurn ?
      robotLines[RIGHT_LINE].pt2 : robotLines[RIGHT_LINE].pt1;
    
    /* -------- Front line and rear line depend on motion direction. -------- */
    if ( tvel > 0.0) {
      lineFacingObstacles = robotLines[FRONT_LINE];
      lineAtRear = robotLines[REAR_LINE];
    }
    else {
      lineFacingObstacles = robotLines[REAR_LINE];
      lineAtRear = robotLines[FRONT_LINE];
    }
    
    dist = minDistanceToLine( center, lineFacingObstacles,
			      &meetPoint, &positionOfMeetPoint);
    
    /* Front line. */
    if (rightTurn) {
      /* The collision part of the line is given by the left front
       * corner and the closer next point. */
      rotLines[FRONT_LINE].pt1 = lineFacingObstacles.pt2;
      if ( positionOfMeetPoint == BETWEEN_POINTS)
	rotLines[FRONT_LINE].pt2 = meetPoint;
      else
	rotLines[FRONT_LINE].pt2 = lineFacingObstacles.pt1;
    }
    else {
      /* The collision part of the line is given by the right front
       * corner and the closer next point. */
      rotLines[FRONT_LINE].pt1 = lineFacingObstacles.pt1;
      if ( positionOfMeetPoint == BETWEEN_POINTS)
	rotLines[FRONT_LINE].pt2 = meetPoint;
      else
	rotLines[FRONT_LINE].pt2 = lineFacingObstacles.pt2;
    }
    
    dist = minDistanceToLine( center, lineAtRear,
			      &meetPoint, &positionOfMeetPoint);
    
    /* ---- Rear line. ---- */
    /* Nothing can happen if the position of the meet point is outside
     * the robot. */
    if ( positionOfMeetPoint != BETWEEN_POINTS) {
      rotLines[REAR_LINE].pt1.x = F_ERROR;
      nothingCanHappenAtRear = TRUE;
    }
    else {
      
      if (rightTurn) {
	/* The collision part of the line is given by the right rear
	 * corner and the meetPoint. */
	rotLines[REAR_LINE].pt1 = lineAtRear.pt2;
	rotLines[REAR_LINE].pt2 = meetPoint;
      }
      else {
	/* The collision part of the line is given by the left rear
	 * corner and the closer next point. */
	rotLines[REAR_LINE].pt1 = lineAtRear.pt1;
	rotLines[REAR_LINE].pt2 = meetPoint;
      }

      if (0) {
	int i;
	for ( i = 0; i < NUMBER_OF_LINES_IN_ROBOT; i++)
	  fprintf( stderr, "rad %f --- %f %f\n",
		   radius,
		   compute_distance( rotLines[i].center, rotLines[i].pt1),
		   compute_distance( rotLines[i].center, rotLines[i].pt2));
      }
    }
    
    /* Now we have defined the two points of the rotating lines and it
     * remains to determine the corresponding rings. */
    setRotatingLines( rotLines);

    /* For testing display the relevant parts of the lines. */
    {
      int i;
      rotatingLine* line;
      
      for ( i = 0; i < NUMBER_OF_LINES_IN_ROBOT; i++) {

	line = &(rotLines[i]);
	
	if ( (i == REAR_LINE) && nothingCanHappenAtRear)
	  colli_tcx_status.testLines[i].pt1.x = I_ERROR;
	else {
	  colli_tcx_status.testLines[i].pt1.x = line->pt1.x;
	  colli_tcx_status.testLines[i].pt1.y = line->pt1.y;
	  colli_tcx_status.testLines[i].pt2.x = line->pt2.x;
	  colli_tcx_status.testLines[i].pt2.y = line->pt2.y;
	}
      }
    }
  }
}


/* Computes the angle until the robot collides with the given collPoint.
 * May also be used to find out the distance to the target. */
static float 
computeAngleToPointOnRotLines( Point rpos, float rrot, float rvel,
			       Point collPoint,
			       rotatingLine* rotLines)
{
  float minAngle = DEG_360;
  int line;

  float distanceFromCenter = compute_distance( rotLines[0].center, collPoint);
  
  /* --------------------------------------------------------------------
   * Check for all the cases which need special treatment.
   * -------------------------------------------------------------------- */
  for ( line = 0; line < NUMBER_OF_LINES_IN_ROBOT; line++) {

    rotatingLine rotLine = rotLines[line];

    /* Is the point in the ring? */
    if ( distanceFromCenter < rotLine.outerRadius &&
	 distanceFromCenter > rotLine.innerRadius) {
      
      /* Compute the point on the line which is going to hit this obstacle. */
      Circle collisionCircle;
      LineSeg fixedLine;
      Point contactPoint[2];
      int numberOfCuts;
      float angle = DEG_360;
      
      collisionCircle.M = rotLine.center;
      collisionCircle.rad = distanceFromCenter;
      
      fixedLine.pt1 = rotLine.pt1;
      fixedLine.pt2 = rotLine.pt2;
      
      numberOfCuts = cut_circle_and_line( collisionCircle, fixedLine, contactPoint);
      
      if (numberOfCuts != 2) {
	fprintf( stderr, "oops, something's wrong.\n");
      }
      else if ( point_on_LineSeg( contactPoint[0], fixedLine)) {

	angle = angleToCutPoint( contactPoint[0], rvel,
				 collPoint, rotLine.center);
	
	if (0) fprintf(stderr, "%d in %f %f %f  out %f %f %f  dist %f  cont %f %f %f\n",
		       numberOfCuts,
		       rotLine.innerRadius, fixedLine.pt1.x, fixedLine.pt1.y,
		       rotLine.outerRadius, fixedLine.pt2.x, fixedLine.pt2.y,
		       compute_distance( rotLine.center, collPoint),
		       contactPoint[0].x, contactPoint[0].y, compute_distance( rotLine.center, contactPoint[0]));
	
      }
      
      else if ( point_on_LineSeg( contactPoint[1], fixedLine)) {
	
	angle = angleToCutPoint( contactPoint[1], rvel,
				 collPoint, rotLine.center);
	
      }
      else {
	if (1) fprintf(stderr, "%d in %f \n%f %f  out %f \n%f %f  dist %f  cont \n%f %f %f  \n%f %f %f\n",
		       numberOfCuts,
		       rotLine.innerRadius, fixedLine.pt1.x, fixedLine.pt1.y,
		       rotLine.outerRadius, fixedLine.pt2.x, fixedLine.pt2.y,
		       compute_distance( rotLine.center, collPoint),
		       contactPoint[0].x, contactPoint[0].y,
		       compute_distance( rotLine.center, contactPoint[0]),
		       contactPoint[1].x, contactPoint[1].y,
		       compute_distance( rotLine.center, contactPoint[1]));
	fprintf(stderr, "betwe %d %d   -  %d %d \n",
		inBetween( contactPoint[0].x, fixedLine.pt1.x, fixedLine.pt2.x),
		inBetween( contactPoint[0].y, fixedLine.pt1.y, fixedLine.pt2.y),
		inBetween( contactPoint[1].x, fixedLine.pt1.x, fixedLine.pt2.x),
		inBetween( contactPoint[1].y, fixedLine.pt1.y, fixedLine.pt2.y));
      }
      if ( angle < minAngle)
	minAngle = angle;
    }
  }
  return minAngle;
}


/* Checks whether one of the points is within the robot. */
static BOOLEAN
pointInsideRobot( Point *points, int numberOfPoints,
		  LineSeg *robLines)
{
  int pt;
  LineSeg leftLine = robLines[LEFT_LINE];
  LineSeg rightLine = robLines[RIGHT_LINE];

  /* We must get the right order for the points. */
  swap(&leftLine);
  
  for ( pt = 0; pt < numberOfPoints; pt++) {
    if ( is_point_in_line_area( &leftLine, &rightLine, points[pt])) {
      if (0) {
	fprintf(stderr, "Point inside\n");
	fprintf(stderr, "%d (%f %f  %f %f) --  (%f %f  %f %f)  -- %f %f\n",
		pt, leftLine.pt1.x, leftLine.pt1.y, 
		leftLine.pt2.x, leftLine.pt2.y, 
		rightLine.pt1.x, rightLine.pt1.y, 
		rightLine.pt2.x, rightLine.pt2.y,
		points[pt].x, points[pt].y);
      }
      return TRUE;
    }
  }
  
  return FALSE;
}


/**********************************************************************
 * The lines do always lead from the robot to MAX_RANGE              *
 **********************************************************************/
void
straightTrajectory( Point rpos, float rrot, float tvel, float securitySize,
		    LineSeg *lline, LineSeg *rline)
{
  float lineLength = MAX_RANGE + rectRob.centerToFront;

  float cosRot = cos(rrot);
  float sinRot = sin(rrot);

  /* Add the size of the robot to get the real size. */
  float leftWidth = rectRob.centerToLeft + securitySize;
  float rightWidth = rectRob.centerToRight + securitySize;

  float dir = (tvel >= 0.0) ? 1.0 : -1.0;

  float centerToFront = (dir > 0.0) ? rectRob.centerToFront : rectRob.centerToRear;

  lline->pt1.x = rpos.x + dir * cos(rrot + DEG_90) * leftWidth;
  lline->pt1.y = rpos.y + dir * sin(rrot + DEG_90) * leftWidth;
  
  lline->pt2.x = lline->pt1.x + dir * cosRot * lineLength;
  lline->pt2.y = lline->pt1.y + dir * sinRot * lineLength;
  
  rline->pt1.x = rpos.x + dir * cos(rrot - DEG_90) * rightWidth;
  rline->pt1.y = rpos.y + dir * sin(rrot - DEG_90) * rightWidth;
  
  rline->pt2.x = rline->pt1.x + dir * cosRot * lineLength;
  rline->pt2.y = rline->pt1.y + dir * sinRot * lineLength;

  
#ifndef STRAIGHT_TRAJECTORY_THROUGH_CENTER

#define ROBOT_SECURITY_SIZE (6.0)

  /* In this case the line goes through the front of the robot. */
  lline->pt1.x += dir * cosRot * (centerToFront - ROBOT_SECURITY_SIZE);
  lline->pt1.y += dir * sinRot * (centerToFront - ROBOT_SECURITY_SIZE);
  
  rline->pt1.x += dir * cosRot * (centerToFront - ROBOT_SECURITY_SIZE);
  rline->pt1.y += dir * sinRot * (centerToFront - ROBOT_SECURITY_SIZE);

#endif
}


/**********************************************************************
 * The next two functions return the distance / angle to the closest collision
 * point. 
 **********************************************************************/
static float
minAngleToCollisionPoint( Point rpos, float rrot, float rvel,float tvel, 
			  Point *points, int numberOfPoints, 
			  rotatingLine* rotLines)
{
  int i;
  float angle, min_angle=DEG_360; 
  Point actualPoint;
  
  for (i=0; i < numberOfPoints; i++) {
    
    if (points[i].x != F_ERROR) {
      
      /* Now compute the angle to this point within one of the rings. */
      angle = computeAngleToPointOnRotLines( rpos, rrot, rvel,
					     points[i], rotLines);

      if ( angle < min_angle) {
	min_angle = angle;
      }
    }  
  } /* End for i. */

  return(min_angle);
} 




static float
minDistToCollisionPointOnTrajectory( Point rpos, float rrot, float tvel,
				     float securitySize,
				     Point *points, int numberOfPoints,
				     LineSeg *lline, LineSeg *rline)
{
  int i;
  Point actualPoint;
  float dist, min_dist=1000.0;
  
  for (i = 0; i < numberOfPoints; i++) {
    
    if ( points[i].x != F_ERROR) {
      
      actualPoint = points[i];
      
      if ( is_point_in_line_area( lline, rline, actualPoint)) {
	
	LineSeg robotLine;
	
	/* Generate a line going through the robot. */
	robotLine.pt1 = lline->pt1;
	robotLine.pt2 = rline->pt1;
	
	dist = minDistanceToLine( actualPoint, robotLine,
				  NULL, NULL);
	if (dist < min_dist) 
	  min_dist = dist;
      }
    }
  }

#ifdef STRAIGHT_TRAJECTORY_THROUGH_CENTER
  /* We still need to substract the length of the robot. */
  if ( tvel > 0.0)
    min_dist = MAX( 0.0, min_dist - rectRob.centerToFront - securitySize);
  else
    min_dist = MAX( 0.0, min_dist - rectRob.centerToRear - securitySize);
#else
  if ( tvel > 0.0)
    min_dist = MAX( 0.0, min_dist - securitySize);
  else
    min_dist = MAX( 0.0, min_dist - securitySize);
#endif
  
  return( min_dist);
}



float
computeCollisionDistanceRectangularRobot( Point rpos, float rrot, float tvel, float rvel,
					  float securitySize, float addSpeedSize, 
					  float *target_dist)
{
  float colldist;               /* Distance till next collision */
  LineSeg lline, rline;         /* Collision area (line)    */
  float min_angle=DEG_360;
  float min_dist=MAX_RANGE;
  float tmp, trajectoryRadius = 0.0;
  int sensor;
  BOOLEAN line_flag=FALSE;
  rotatingLine rotatingRobot[NUMBER_OF_LINES_IN_ROBOT];
  
  /* Check for the kind of the collision area. */
  if (fabs(rvel) < EPSILON) {
    rvel = 0.0; 
    line_flag = TRUE;
  }
  else if (fabs(tvel/rvel) > MAX_CURVE_RADIUS) {
    rvel = 0.0; 
    line_flag = TRUE;
  }
  else
    trajectoryRadius = fabs( tvel/rvel);

  if (line_flag) {
    
    /* Compute the collision area if rvel too small (i.e. we get two lines) */
    straightTrajectory( rpos, rrot, tvel, securitySize+0.5*addSpeedSize,
			&lline, &rline);
    
    /*--------------------------------------------------
     * All Sensors
     *--------------------------------------------------*/
    for ( sensor = 0; sensor < NUMBER_OF_SENSORS; sensor++) {
      if ( useSensorPoints[sensor]) {
	tmp = minDistToCollisionPointOnTrajectory( rpos, rrot, tvel,
						   securitySize,
						   combinedObstaclePoints[sensor].points,
						   combinedObstaclePoints[sensor].no_of_points,
						   &lline, &rline);

	if ( tmp < min_dist) {
	  min_dist = tmp;
	}
      }
    }
    
    colldist = MIN(MAX_RANGE, min_dist); 
  }
  else {

    LineSeg robotLines[NUMBER_OF_LINES_IN_ROBOT];

    /* The collision area is given by a ring for each line of the rectangle */
    determineRotatingLines( rpos,
			    rrot,
			    tvel, rvel,
			    securitySize, addSpeedSize,
			    rotatingRobot, robotLines);

    /*--------------------------------------------------
     * All Sensors.
     *--------------------------------------------------*/
    for ( sensor = 0; sensor < NUMBER_OF_SENSORS; sensor++) {
      
      if ( useSensorPoints[sensor] && min_angle > 0.0) {
	
	/* Check whether any point is within the robot. */
	if ( pointInsideRobot( combinedObstaclePoints[sensor].points,
			       combinedObstaclePoints[sensor].no_of_points,
			       robotLines)) {
	  min_angle = 0.0;
	}	
	else {	  
	  tmp =
	    minAngleToCollisionPoint(  rpos, rrot, rvel, tvel,
				       combinedObstaclePoints[sensor].points,
				       combinedObstaclePoints[sensor].no_of_points,
				       rotatingRobot);
	  
	  if ( tmp < min_angle) {
	    min_angle = tmp;
	  }
	}
      }
    }

    
    if ( tvel == 0.0) {
      colldist = RAD_TO_DEG( min_angle);
    }
    else if ( min_angle >= DEG_180) {      
      colldist =  MAX( DEG_180 * trajectoryRadius, 2.0 * ACTUAL_MODE->min_dist);
    }
    else
      colldist = min_angle * trajectoryRadius;
    
    /* Now cut collDist. */
    colldist = MIN( colldist, MAX_RANGE);
  }

  if (target_flag) {
    
    if (line_flag) {

      *target_dist =
	MIN( minDistToCollisionPointOnTrajectory( rpos, rrot, tvel,
						  securitySize,
						  &target,
						  1,
						  &lline, &rline),
	     MAX_RANGE);
    }
    else {
      if ( tvel == 0.0) {
	*target_dist = MAX_RANGE;
      }
      else
	*target_dist =
	  MIN( trajectoryRadius * minAngleToCollisionPoint(  rpos, rrot, rvel, tvel,
							     &target,
							     1,
							     rotatingRobot),
	       MAX_RANGE);
      
    }
  }
  
  return( colldist);
}

