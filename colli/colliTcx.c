
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliTcx.c,v $
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
 * $Log: colliTcx.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.16  1999/09/08 21:11:50  fox
 * *** empty log message ***
 *
 * Revision 1.15  1999/06/25 19:48:08  fox
 * Minor changs for the urbie.
 *
 * Revision 1.14  1999/03/09 16:43:59  wolfram
 * Fixed a bug
 *
 * Revision 1.13  1999/03/09 15:48:39  wolfram
 * Added geometry of PIONEER_II
 *
 * Revision 1.12  1998/10/30 18:20:50  fox
 * Added support for pioniers.
 *
 * Revision 1.11  1998/10/23 20:50:30  fox
 * *** empty log message ***
 *
 * Revision 1.10  1998/08/26 23:23:41  fox
 * Final version before I changed exception handling.
 *
 * Revision 1.9  1998/08/18 16:24:24  fox
 * Added support for b18 robot.
 *
 * Revision 1.8  1997/07/17 17:31:46  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.7  1997/05/06 18:03:30  fox
 * Infrareds and bumper should work. Take care of the INDEX!
 *
 * Revision 1.6  1997/05/06 17:05:03  fox
 * Nothing special.
 *
 * Revision 1.5  1997/04/10 15:22:09  fox
 * Fixed a bug so that the rotation is not set to zero at start up.
 *
 * Revision 1.4  1997/04/10 12:08:21  fox
 * Incorporated support for the tactiles.
 *
 * Revision 1.3  1997/03/26 18:42:02  fox
 * Incorporated external obstacle information into the collision avoidance.
 *
 * Revision 1.2  1997/02/11 17:59:27  fox
 * Don't use sonar in FIND_DOOR_MODE.
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
#include "colliTrajectories.h"


/**********************************************************************
 **********************************************************************
 *               Functions for data transfer via TCX                  *
 **********************************************************************
 **********************************************************************/

void
initTcxStructures()
{
    
  /* These values are only set once and must no be changed. */
  colli_tcx_status.rememberInterval = REMEMBER_INTERVAL;
  
  /* Initialize the vision lines and points */
  colli_tcx_status.no_of_external_points = 0;
  colli_tcx_status.external_points = NULL;
  
  colli_tcx_status.no_of_bumper_points = 0;
  colli_tcx_status.bumper_points = NULL;
  
  colli_tcx_status.no_of_ir_points = 0;
  colli_tcx_status.ir_points = NULL;
  
  /* Initialize the laser points */
  colli_tcx_status.no_of_laser_points = 0;
  colli_tcx_status.laser_points = NULL;
}

/**********************************************************************
 * Updates the values in colli_tcx_status. These values are shown in the 
 * trajectory of the robot in the collision graphics.
**********************************************************************/
void
COLLI_update_tcx_trajectory(Point rpos, float rrot,
			    float tvel, float rvel,
			    float dist)
{

  if ( n_auto_colli_update_modules > 0) {

    BOOLEAN lineFlag = FALSE;
    float addSpeedSize = speed_dependent_security_dist(tvel);
    float size;
    
    int realTvel = (int) rwi_base.trans_current_speed;
    
    if (target_flag) {
      colli_tcx_status.targetpoint.x = casted_float(target.x); 
      colli_tcx_status.targetpoint.y = casted_float(target.y); 
    }  
    else
      colli_tcx_status.targetpoint.x = I_ERROR;
    
    /* Check for the kind of the trajectory and store the values
     * in the corresponding integers. */
    
    if (fabs(rvel) < EPSILON) 
      lineFlag = TRUE;
    else if (fabs(tvel/rvel) > MAX_CURVE_RADIUS) 
      lineFlag = TRUE;
    
    colli_tcx_status.rpos.x = casted_float(rpos.x);
    colli_tcx_status.rpos.y = casted_float(rpos.y);
    colli_tcx_status.rrot   = casted_float(RAD_TO_DEG(rrot));
    colli_tcx_status.rvel   = casted_float(RAD_TO_DEG(rvel));
    colli_tcx_status.dist   = casted_float(dist);
      
    colli_tcx_status.tvel   = realTvel;
    
    /*    colli_tcx_status.tvel   = casted_float(tvel); */

    if ( robotShape == ROUND_ROBOT) {
      
      float armsize;
      Point tmp;
      
      size = ROB_RADIUS + ACTUAL_MODE->security_dist;
      
      if ( armState != INSIDE)
	armsize = computeArmOffset( tvel, rvel);
      else 
	armsize = 0.0;
      
      
      if ( lineFlag) {
	LineSeg lline, rline;                    /* Collision area (line)    */
	position_to_lines(rpos, rrot, tvel, size+0.5*addSpeedSize, &lline, &rline); 
	
	colli_tcx_status.leftLine.pt1.x = casted_float(lline.pt1.x); 
	colli_tcx_status.leftLine.pt1.y = casted_float(lline.pt1.y); 
	colli_tcx_status.leftLine.pt2.x = casted_float(lline.pt2.x); 
	colli_tcx_status.leftLine.pt2.y = casted_float(lline.pt2.y); 
	
	colli_tcx_status.rightLine.pt1.x = casted_float(rline.pt1.x); 
	colli_tcx_status.rightLine.pt1.y = casted_float(rline.pt1.y); 
	colli_tcx_status.rightLine.pt2.x = casted_float(rline.pt2.x); 
	colli_tcx_status.rightLine.pt2.y = casted_float(rline.pt2.y); 
	
	colli_tcx_status.innerCircle.M.x = I_ERROR;
	colli_tcx_status.outerCircle.M.x = I_ERROR;
      }
      else {
	Circle_trajectory circ =
	  velocities_to_circle_trajectory( rpos, rrot, tvel, rvel,
					   size, addSpeedSize,armsize);
	
	colli_tcx_status.outerCircle.rad = casted_float( circ.big_rad);
	colli_tcx_status.outerCircle.M.x = casted_float( circ.M.x);
	colli_tcx_status.outerCircle.M.y = casted_float( circ.M.y);
	
	colli_tcx_status.innerCircle.rad = casted_float( circ.small_rad);
	colli_tcx_status.innerCircle.M.x = casted_float( circ.M.x);
	colli_tcx_status.innerCircle.M.y = casted_float( circ.M.y);
	
	if ( armState != INSIDE) {
	  colli_tcx_status.armCircle.rad = casted_float( circ.arm_rad);
	  colli_tcx_status.armCircle.M.x = casted_float( circ.M.x);
	  colli_tcx_status.armCircle.M.y = casted_float( circ.M.y);
	  tmp = computeInnerArmPoint(rpos,rrot,rvel,tvel);
	  colli_tcx_status.innerArmPoint.x = casted_float(tmp.x);
	  colli_tcx_status.innerArmPoint.y = casted_float(tmp.y);
	  tmp = computeOuterArmPoint(rpos,rrot,rvel,tvel);
	  colli_tcx_status.outerArmPoint.x = casted_float(tmp.x);
	  colli_tcx_status.outerArmPoint.y = casted_float(tmp.y);
	}
      }
    }
    else if ( robotType == B18_ROBOT ||
	      robotType == PIONEER_ATRV || robotType == PIONEER_II ||
	      robotType == URBAN_ROBOT) {
      
      int l, i;
      iPoint fr,fl,rr,rl;
      LineSeg lines[NUMBER_OF_LINES_IN_ROBOT];

      /* Create robot without additional sizes */
      determineRectangularRobotLines( rpos, rrot,
				      rvel, 0.0, 0.0, lines);
      
      for ( l = 0; l < NUMBER_OF_LINES_IN_ROBOT; l++) {
	colli_tcx_status.robotLines[l].pt1.x = casted_float(lines[l].pt1.x);
	colli_tcx_status.robotLines[l].pt1.y = casted_float(lines[l].pt1.y);
	colli_tcx_status.robotLines[l].pt2.x = casted_float(lines[l].pt2.x);
	colli_tcx_status.robotLines[l].pt2.y = casted_float(lines[l].pt2.y);
      }

      /* Now the trajectory. */
      if ( lineFlag) {
	LineSeg lline, rline;                    /* Collision area (line)    */

	straightTrajectory(rpos, rrot, tvel,
			   ACTUAL_MODE->security_dist+0.5*addSpeedSize,
			   &lline, &rline); 

	colli_tcx_status.leftLine.pt1.x = casted_float(lline.pt1.x); 
	colli_tcx_status.leftLine.pt1.y = casted_float(lline.pt1.y); 
	colli_tcx_status.leftLine.pt2.x = casted_float(lline.pt2.x); 
	colli_tcx_status.leftLine.pt2.y = casted_float(lline.pt2.y); 
	
	colli_tcx_status.rightLine.pt1.x = casted_float(rline.pt1.x); 
	colli_tcx_status.rightLine.pt1.y = casted_float(rline.pt1.y); 
	colli_tcx_status.rightLine.pt2.x = casted_float(rline.pt2.x); 
	colli_tcx_status.rightLine.pt2.y = casted_float(rline.pt2.y); 

	colli_tcx_status.testLines[0].pt1.x = I_ERROR;
	
	colli_tcx_status.innerCircle.M.x = I_ERROR;
	colli_tcx_status.outerCircle.M.x = I_ERROR;
      }
      else {

	float rad_rot, rdir, tdir, radius;
	float maxRad = 0.0, minRad = MAX_CURVE_RADIUS;	
	Point center;
	LineSeg lines[NUMBER_OF_LINES_IN_ROBOT];

	/* Determine the center of the circular trajectory. */
	rdir = (rvel >= 0.0) ? 1.0 : -1.0;
	tdir = (tvel >= 0.0) ? 1.0 : -1.0;
	rad_rot = rrot - (rdir*tdir) * DEG_90;
	radius = fabs (tvel / rvel);

	center.x = rpos.x + fcos( rad_rot) * radius;
	center.y = rpos.y + fsin( rad_rot) * radius;

	colli_tcx_status.outerCircle.M.x =
	  colli_tcx_status.innerCircle.M.x = casted_float(center.x);
	colli_tcx_status.outerCircle.M.y =
	  colli_tcx_status.innerCircle.M.y = casted_float(center.y);
	
	determineRectangularRobotLines( rpos, rrot,
					rvel,  ACTUAL_MODE->security_dist,
					addSpeedSize, lines);
	
	
	/* Extract the most extrem corners of the robot. */
	for ( i = 0; i < NUMBER_OF_LINES_IN_ROBOT; i++) {
	  float dist = compute_distance( center, lines[i].pt1);
	  if ( dist > maxRad)
	    maxRad = dist;
	  if ( dist < minRad)
	    minRad = dist;
	}
	
	colli_tcx_status.outerCircle.rad = maxRad;
	if ( radius < rectRob.centerToLeft && radius < rectRob.centerToRight)
	  colli_tcx_status.innerCircle.rad = 0.0;
	else
	  colli_tcx_status.innerCircle.rad = minRad;
      }
    }
    else {
      fprintf( stderr, "Sorry. Only B21, B18, Pioneer, and Urban supported in COLLI_update_tcx_trajectory.\n");
      exit(0);
    }
  }
}



/**********************************************************************
 * Updates the values in colli_tcx_status. These lines are shown in the 
 * collision graphics in red.
**********************************************************************/
void
COLLI_update_tcx_CollLines(void)
{
  int i;
  LineSeg tmp;

  if ( n_auto_colli_update_modules > 0) {
    for (i = 0; i < bRobot.sonar_cols[0]; i++) {
      if ( use_sonar) {
	tmp = CollLines[i][next_CollLine_reading]; 
	(colli_tcx_status.sonar_lines)[i].pt1.x = casted_float(tmp.pt1.x); 
	(colli_tcx_status.sonar_lines)[i].pt1.y = casted_float(tmp.pt1.y); 
	(colli_tcx_status.sonar_lines)[i].pt2.x = casted_float(tmp.pt2.x); 
	(colli_tcx_status.sonar_lines)[i].pt2.y = casted_float(tmp.pt2.y); 
      }
      else
	colli_tcx_status.sonar_lines[i].pt1.x = I_ERROR;
    }
  }
}

/**********************************************************************
 * Updates the values in colli_tcx_status. These lines are shown in the 
 * collision graphics in red.
**********************************************************************/
void
COLLI_update_tcx_LaserPoints(void)
{
  int i;

  if ( n_auto_colli_update_modules > 0) {
    
    /* If the number of points has changed we allocate new memory. */
    if ( colli_tcx_status.no_of_laser_points !=
	 combinedObstaclePoints[LASER_POINTS].no_of_points) {

      if ( colli_tcx_status.no_of_laser_points > 0)
	free (colli_tcx_status.laser_points);
      
      colli_tcx_status.no_of_laser_points =
	combinedObstaclePoints[LASER_POINTS].no_of_points;
      
      if (colli_tcx_status.no_of_laser_points > 0) 
	colli_tcx_status.laser_points = (iPoint *) 
	  malloc( colli_tcx_status.no_of_laser_points * sizeof(struct iPoint));
      else
	colli_tcx_status.laser_points = (iPoint *) NULL; 
    }

    for (i=0; i < colli_tcx_status.no_of_laser_points; i++) {
      (colli_tcx_status.laser_points)[i].x =
	casted_float(combinedObstaclePoints[LASER_POINTS].points[i].x); 
      (colli_tcx_status.laser_points)[i].y =
	casted_float(combinedObstaclePoints[LASER_POINTS].points[i].y); 
    }
  }
}


/**********************************************************************
 * Updates the values in colli_tcx_status. These points are shown in the 
 * collision graphics in blue.
**********************************************************************/
void
COLLI_update_tcx_ExternalObstaclePoints(void)
{
  int i;
  Point tmp;

  if (n_auto_colli_update_modules > 0) {

    if (colli_tcx_status.no_of_external_points > 0) 
      free (colli_tcx_status.external_points);
    
    colli_tcx_status.no_of_external_points =
      combinedObstaclePoints[EXTERNAL_POINTS].no_of_points;
    
    if (colli_tcx_status.no_of_external_points > 0) {
      
      colli_tcx_status.external_points = (iPoint *) 
	malloc( colli_tcx_status.no_of_external_points * sizeof(struct iPoint));
      
      for (i=0; i<colli_tcx_status.no_of_external_points; i++) {
 	tmp = combinedObstaclePoints[EXTERNAL_POINTS].points[i];
	(colli_tcx_status.external_points)[i].x = casted_float(tmp.x); 
	(colli_tcx_status.external_points)[i].y = casted_float(tmp.y); 
      }
    }
    else 
      colli_tcx_status.external_points = NULL;
  }
  else {
    if (colli_tcx_status.no_of_external_points > 0) 
      free (colli_tcx_status.external_points);
    colli_tcx_status.no_of_external_points = 0;
  }
}


/**********************************************************************
 * Updates the values in colli_tcx_status. These points are shown in the 
 * collision graphics in blue.
**********************************************************************/
void
COLLI_update_tcx_BumperPoints(void)
{
  int i;
  Point tmp;

  if (n_auto_colli_update_modules > 0) {
    if (colli_tcx_status.no_of_bumper_points > 0) 
      free (colli_tcx_status.bumper_points);
    
    colli_tcx_status.no_of_bumper_points = combinedObstaclePoints[BUMPER_POINTS].no_of_points;

    if (colli_tcx_status.no_of_bumper_points > 0) {
     
      colli_tcx_status.bumper_points = (iPoint *) 
	malloc( colli_tcx_status.no_of_bumper_points * sizeof(struct iPoint));
      
      for (i=0; i<colli_tcx_status.no_of_bumper_points; i++) {
 	tmp = combinedObstaclePoints[BUMPER_POINTS].points[i];
	(colli_tcx_status.bumper_points)[i].x = casted_float(tmp.x); 
	(colli_tcx_status.bumper_points)[i].y = casted_float(tmp.y); 
      }
    }
    else 
      colli_tcx_status.bumper_points = NULL;
  }
  else {
    colli_tcx_status.no_of_bumper_points = 0; 
    colli_tcx_status.bumper_points = NULL;
  }
}


/* Updates the values in colli_tcx_status. These points are shown in the 
 * collision graphics in green. */
void
COLLI_update_tcx_IrPoints(void)
{
  int i;
  Point tmp;

  if (n_auto_colli_update_modules > 0) {

    if (colli_tcx_status.no_of_ir_points > 0) 
      free (colli_tcx_status.ir_points);
    
    colli_tcx_status.no_of_ir_points = combinedObstaclePoints[IR_POINTS].no_of_points;

    if (colli_tcx_status.no_of_ir_points > 0) {
      
      colli_tcx_status.ir_points = (iPoint *) 
	malloc( colli_tcx_status.no_of_ir_points * sizeof(struct iPoint));
      
      for (i=0; i<colli_tcx_status.no_of_ir_points; i++) {
 	tmp = combinedObstaclePoints[IR_POINTS].points[i];
	(colli_tcx_status.ir_points)[i].x = casted_float(tmp.x); 
	(colli_tcx_status.ir_points)[i].y = casted_float(tmp.y); 
      }
    }
    else 
      colli_tcx_status.ir_points = NULL;
  }
  else {
    colli_tcx_status.no_of_ir_points = 0; 
    colli_tcx_status.ir_points = NULL;
  }
}

