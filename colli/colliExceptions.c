
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliExceptions.c,v $
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
 * $Log: colliExceptions.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.34  1999/11/15 13:28:05  fox
 * *** empty log message ***
 *
 * Revision 1.33  1999/10/14 19:56:39  fox
 * IMPORTANT change: if the orientation of the robot is off more than 60 deg.
 * it will prefer rotation a lot.
 *
 * Revision 1.32  1999/09/08 21:11:50  fox
 * *** empty log message ***
 *
 * Revision 1.31  1999/07/23 19:46:36  fox
 * Added ability to read grid maps for the obstacle server.
 *
 * Revision 1.29  1999/06/25 19:48:08  fox
 * Minor changs for the urbie.
 *
 * Revision 1.28  1999/03/09 16:43:59  wolfram
 * Fixed a bug
 *
 * Revision 1.27  1999/03/09 15:48:37  wolfram
 * Added geometry of PIONEER_II
 *
 * Revision 1.26  1998/12/10 17:45:06  fox
 * Changed some settings for the pioneer at.
 *
 * Revision 1.25  1998/11/23 21:19:24  fox
 * Fixed some minor bugs.
 *
 * Revision 1.24  1998/11/22 23:54:20  fox
 * Fixed a bug for rectangular robots. If tvel == 0, then the distance
 * to the next obstacle is set to zero. This is done to avoid too much
 * rotation.
 *
 * Revision 1.23  1998/11/19 03:14:31  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.22  1998/11/12 18:36:59  fox
 * Version running on the pioneers.
 *
 * Revision 1.21  1998/10/30 18:20:49  fox
 * Added support for pioniers.
 *
 * Revision 1.20  1998/10/23 20:50:29  fox
 * *** empty log message ***
 *
 * Revision 1.19  1998/09/12 21:26:45  fox
 * Final version of the museum.
 *
 * Revision 1.18  1998/09/05 00:24:37  fox
 * Shoudl work with B21 and B18.
 *
 * Revision 1.17  1998/09/05 00:20:00  fox
 * Version for internet night in Washington.
 *
 * Revision 1.16  1998/08/29 21:50:01  fox
 * Adapted exception handling in order to deal with people around the robot.
 *
 * Revision 1.15  1998/08/26 23:23:39  fox
 * Final version before I changed exception handling.
 *
 * Revision 1.14  1998/08/18 16:24:23  fox
 * Added support for b18 robot.
 *
 * Revision 1.13  1997/10/01 11:53:17  fox
 * Fixed a bug.
 *
 * Revision 1.12  1997/06/17 09:39:30  fox
 * Changed rotate away.
 *
 * Revision 1.11  1997/06/11 16:46:14  fox
 * Fixed a bug in rotate away.
 *
 * Revision 1.10  1997/06/03 11:49:11  fox
 * Museum version.
 *
 * Revision 1.9  1997/05/14 08:10:56  fox
 * Fixed a bug when setting random mode.
 *
 * Revision 1.8  1997/04/17 09:19:38  fox
 * Minor changes.
 *
 * Revision 1.7  1997/04/10 12:08:21  fox
 * Incorporated support for the tactiles.
 *
 * Revision 1.6  1997/04/09 12:57:49  fox
 * Minor changes.
 *
 * Revision 1.5  1997/01/07 13:47:13  fox
 * Improved rotate_away.
 *
 * Revision 1.4  1997/01/03 18:08:57  fox
 * Made some minor changes.
 *
 * Revision 1.3  1996/12/23 15:50:35  fox
 * Added minor support for the bumpers. COLLI stops whenever a bumper has
 * triggered a signal.
 *
 * Revision 1.2  1996/12/23 12:48:53  fox
 * First attempt to incorporate the tactiles.
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

/* These values determine the maximum velocities during 
   exceptions (rotate_away and achieve_distance).
   They are allowed to be above the MAX_... values. */
float EXCEPTION_TRANS_ACCELERATION = 60.0;
float EXCEPTION_TRANS_VELOCITY = 20.0;
float EXCEPTION_ROT_ACCELERATION = DEG_TO_RAD(60.0);
float EXCEPTION_ROT_VELOCITY = DEG_TO_RAD(30.0);

BOOLEAN alreadyTriedRotation = FALSE;
BOOLEAN alreadyTriedTranslation = FALSE;

BOOLEAN achieveDistanceFlag = FALSE; 
BOOLEAN rotateAwayFlag = FALSE;
BOOLEAN robotStuck = FALSE;

#define MAX_NUMBER_OF_EXCEPTIONS 3

/**********************************************************************
 **********************************************************************
 *                     Collision functions                            *
 **********************************************************************
 **********************************************************************/

/* This routine is called when the robot is too close to an obstacle
 * to translate forward.
 * In this case the robot just turns in the free direction that is closer 
 * to the target. */
static void
rotateAwayTrajectory(Point rpos, float rrot, int numberOfExceptions);

/* This routine is callled when the robot is too close to an obstacle.
 * In this case the robot just turns in the direction that is closer 
 * to free space than the other.*/
static void
achieveDistanceTrajectory(Point rpos, float rrot, int numberOfExceptions);


/**********************************************************************
 * In the random mode we prefer straight motion. 
 **********************************************************************/
BOOLEAN
straightMotionIsPossible( Point rpos, float rrot)
{
    float cDist, tDist;

#ifdef STRAIGHT_OR_RANDOM_DIRECTION
#define MIN_DISTANCE_FOR_STRAIGHT_MOTION 200.0
#else
#define MIN_DISTANCE_FOR_STRAIGHT_MOTION 120.0
#endif
#define MIN_DISTANCE_FOR_STRAIGHT_MOTION_WITH_ARM 150.0
    
    /* Is straight motion possible? */
    if ( actual_velocities.min_rvel > 0.001 || actual_velocities.max_rvel < 0.001)
	return FALSE;
    
    /* Is the motion allowed? */
    if ( admissible( rpos,
		     rrot, 
		     ACTUAL_MODE->target_max_trans_speed, 
		     0.0,
		     0, 0,
		     &cDist,
		     &tDist)) {
	/* Is enough space to the next obstacle? */
	if ( mode_number == RANDOM_MODE &&
	     cDist > MIN_DISTANCE_FOR_STRAIGHT_MOTION)
	    return TRUE;
	if ( mode_number == ARM_OUT_RANDOM_MODE &&
	     cDist > MIN_DISTANCE_FOR_STRAIGHT_MOTION_WITH_ARM)
	    return TRUE;
    } 
    return FALSE;
}



/**********************************************************************
 * Starts special modes to deal with exceptions. If rotation is allowed
 * we do this.
 **********************************************************************/
void
startExceptionHandling( Point rpos,
			float rrot,
			VelocityCombination bestCombination,
			int numberOfExceptions)
{
  alreadyTriedTranslation = FALSE;
  alreadyTriedRotation = FALSE;

  
  numberOfExceptions = 0;
  
  if ( robotShape == ROUND_ROBOT) {
    
#define PREFER_ROTATION    
#ifdef PREFER_ROTATION

    /* Rotate away only when arm is not outside. */
    if ( bestCombination.rvel != 0.0 && armState == INSIDE) {
      
      /* There is at least one admissible trajectory. But the
       * robot is only allowed to rotate. */
      fprintf(stderr, "No translation allowed.\n");
      
      rotateAwayFlag = TRUE;
      newTargetPoint = FALSE;
      rotateAwayTrajectory(rpos, rrot, numberOfExceptions);
    }
    else {
      fprintf( stderr, "No trajectory admissible.\n");
      if ( armState != INSIDE)
	fprintf( stderr, "Just got in translate backwards because of arm outside.\n");
      /* No trajectory is admissible. */
      achieveDistanceFlag = TRUE;
      newTargetPoint = FALSE;
      achieveDistanceTrajectory( rpos, rrot, numberOfExceptions);
    }
#else
    /* First try to back up some centimeters. */
    if ( numberOfExceptions >= MAX_NUMBER_OF_EXCEPTIONS) {
      
      if ( numberOfExceptions == MAX_NUMBER_OF_EXCEPTIONS) 
	fprintf(stderr, "That's too much.\n");
      else
	fprintf(stderr, "%d\n", numberOfExceptions);
      achieveDistanceFlag = TRUE;
      return;
    }
    
    if ( bestCombination.rvel != 0.0) 
      fprintf( stderr, "Could rotate but try to back up.\n");
    
    /* No trajectory is admissible. */
    achieveDistanceFlag = TRUE;
    newTargetPoint = FALSE;
    achieveDistanceTrajectory( rpos, rrot, numberOfExceptions);
#endif
  }
  else {

    if ( robotType == PIONEER_ATRV || robotType == PIONEER_II || robotType == URBAN_ROBOT || mode_number == ESCAPE_MODE || mode_number == RANDOM_MODE) {
      fprintf(stderr, "BEST %f %f\n", bestCombination.rvel, bestCombination.tvel);
      if ( bestCombination.rvel != 0.0) {
	
	/* There is at least one admissible trajectory. But the
	 * robot is only allowed to rotate. */
	fprintf(stderr, "No translation allowed.\n");
	rotateAwayFlag = TRUE;
	newTargetPoint = FALSE;
	rotateAwayTrajectory(rpos, rrot, numberOfExceptions);
      }
      else {
	/* No trajectory is admissible. */
	fprintf( stderr, "No trajectory admissible.\n");
	achieveDistanceFlag = TRUE;
	newTargetPoint = FALSE;
	achieveDistanceTrajectory( rpos, rrot, numberOfExceptions);
      }
    }
    /* Just wait and do nothing. */
    else {
      /* Rotation is not allowed as well. */
      fprintf(stderr, "Can't do anything.\n");
      BASE_TranslateHalt();
      BASE_RotateHalt();
      target_flag = TRUE; 
      robotStuck = TRUE;
      return;
    }
  }
}

/**********************************************************************
 * The rotation or translation has to be finished.
 **********************************************************************/
BOOLEAN
keepOnWithExceptionHandling( Point rpos,
			     float rrot, int numberOfExceptions)
{
  numberOfExceptions++;
  
  if ( !achieveDistanceFlag && !rotateAwayFlag) 
    return FALSE;
  
  if ( achieveDistanceFlag) {
    /* The robot has to translate backward and there has no more collision
     * avoidance to be done.
     */
    achieveDistanceTrajectory(rpos, rrot, numberOfExceptions);
    COLLI_update_tcx_trajectory(rpos, rrot, -30.0, 0.0, 0.0);
  }
  else if (rotateAwayFlag){ 
    /* The robot has to turn and there has no more collision avoidance
     * to be done.
     */
    rotateAwayTrajectory(rpos, rrot, numberOfExceptions);
    COLLI_update_tcx_trajectory(rpos, rrot, 0.0, 1.0, 0.0);
  }
  COLLI_send_colli_update();
  return TRUE;
}
    


/**********************************************************************
 * This routine is called when the robot is too close to an obstacle
 * to translate forward.
 * In this case the robot just turns in the free direction that is closer 
 * to the target.
 **********************************************************************/
void
rotateAwayTrajectory( Point rpos, float rrot, int numberOfExceptions)
{

  int i;
  float dist=0.0, targetAngle, targetDist, angle = DEG_360;
  float leftDist, rightDist, securityDist;
  static BOOLEAN first_time=TRUE;
  static int cnt = 0;

#define ANGLE_SKIP 10

  if (0) fprintf(stderr, "rotate counter %d\n", numberOfExceptions);

  if ( robotShape == ROUND_ROBOT) {
    
    if ( armState != INSIDE) {
      fprintf( stderr, "Can't rotate away when arm is out.\n");
      return;
    }
    
    if (first_time) {
      
      if (dumpInfo)
	fprintf( dumpFile, "Rotate away");
      fprintf(stderr, "Rotate away");
      
      securityDist = 2.0 * ACTUAL_MODE->min_dist;
      
      targetAngle = compute_angle_2p(rpos, target);

      /* changed random mode. */
      if ( mode_number ==  RANDOM_MODE) {
	securityDist = MIN_DISTANCE_FOR_STRAIGHT_MOTION + 50;
	targetAngle =  (float) rand() / RAND_MAX * DEG_360;
      }
      
      dist = computeCollisionDistance( rpos, targetAngle,
				       ACTUAL_MODE->target_max_trans_speed,
				       0.0, 
				       ROB_RADIUS + ACTUAL_MODE->security_dist,
				       0.0,
				       0.0,
				       &targetDist);
      
      if (dist < securityDist) {
	
	/* The direct way to the target is not free so we have to look
	 * for the best direction. 
	 */
	
	for (i=10; i<180 && dist<securityDist; i+=ANGLE_SKIP) {
	  
	  leftDist = computeCollisionDistance(
					      rpos, 
					      targetAngle + DEG_TO_RAD((float)i),
					      ACTUAL_MODE->target_max_trans_speed,
					      0.0, 
					      ROB_RADIUS+ACTUAL_MODE->security_dist,
					      0.0,
					      0.0,
					      &targetDist);
	  
	  rightDist = computeCollisionDistance(
					       rpos,
					       targetAngle - DEG_TO_RAD((float)i),
					       ACTUAL_MODE->target_max_trans_speed,
					       0.0, 
					       ROB_RADIUS+ACTUAL_MODE->security_dist,
					       0.0,
					       0.0,
					       &targetDist);
	  
	  if ( leftDist > securityDist || rightDist > securityDist) {
	    if ( leftDist > rightDist) {
	      dist = leftDist;
	      angle = targetAngle + DEG_TO_RAD((float) i) + DEG_TO_RAD(ANGLE_SKIP);
	      if (0) fprintf(stderr, "\nlefttar %f  --> ", RAD_TO_DEG( rrot - targetAngle));
	    }
	    else {
	      dist = rightDist;
	      angle = targetAngle - DEG_TO_RAD((float) i) - DEG_TO_RAD(ANGLE_SKIP);
	      if (0) fprintf(stderr, "\nrighttar %f  --> ", RAD_TO_DEG( rrot - targetAngle));
	    }
	    angle = normed_angle( rrot - angle);
	    if (0) fprintf(stderr, "%f\n", RAD_TO_DEG(angle));
	  }
	}    
	if ( angle == DEG_360)
	  fprintf( stderr, "nothing found\n");
      }
      else {
	angle = normed_angle( rrot - targetAngle);
      }
      /* Choose the shortest direction. */    
      if (angle > DEG_180) 
	angle -= DEG_360;
      
      BASE_TranslateCollisionAcceleration(ACTUAL_MODE->exception_trans_acceleration);
      BASE_RotateCollisionAcceleration( RAD_TO_DEG( ACTUAL_MODE->exception_rot_acceleration));
      BASE_TranslateHalt();
      BASE_RotateHalt();
      
      /* In the stop commands the target flag is set to FALSE. 
       * We still want to be in the target mode.
       */
      target_flag = TRUE;
      BASE_TranslateVelocity (0.0);
      BASE_RotateVelocity ( RAD_TO_DEG( ACTUAL_MODE->exception_rot_velocity));
      BASE_TranslateCollisionVelocity (0.0);
      BASE_RotateCollisionVelocity ( RAD_TO_DEG( ACTUAL_MODE->exception_rot_velocity));
      
      BASE_Rotate((double)RAD_TO_DEG(angle));
      if (dumpInfo)
	fprintf( dumpFile, " (%f) ", RAD_TO_DEG(angle));
      fprintf(stderr, " (%f) ", RAD_TO_DEG(angle));
      first_time = FALSE;
      cnt = 0;
    }      
    
    /* We have to wait until the rotation is finished. */
    else if ( ! stillInRotation()) {
      if (dumpInfo)
	fprintf( dumpFile, "done.\n");
      fprintf(stderr, "done.\n");
      rotateAwayFlag = FALSE;
      first_time = TRUE;
      BASE_TranslateCollisionAcceleration(ACTUAL_MODE->target_trans_acceleration);
      BASE_RotateCollisionAcceleration(RAD_TO_DEG(ACTUAL_MODE->target_rot_acceleration));
      BASE_TranslateCollisionVelocity (0.0);
      BASE_RotateCollisionVelocity (0.0);
    }
  }
  else if ( robotType == B18_ROBOT || robotType == PIONEER_ATRV
	    || robotType == PIONEER_II || robotType == URBAN_ROBOT) {

    /* IMPORTANT: this part is rewritten in radiants!!!! */
    if (first_time) {
      
#define NUMBER_OF_ANGLES_TO_BE_TRIED 40
#define ADDITIONAL_ROTATION (0.0)
      
      int angleCnt;
      float absoluteAngle, angleToRotate = 0.0;
      float angleSkip = DEG_360 / NUMBER_OF_ANGLES_TO_BE_TRIED;
      float relAngle, rightAngle, leftAngle;
      float angleFromTarget = DEG_360, closestAngle = DEG_360;
      float leftDistances[NUMBER_OF_ANGLES_TO_BE_TRIED];
      float rightDistances[NUMBER_OF_ANGLES_TO_BE_TRIED];

      securityDist = 2.0 * ACTUAL_MODE->min_dist;
      
      /* Relative direction to the target. */
      targetAngle = compute_angle_2p(rpos, target);
#ifdef ROTATE_DEBUG
      printf( "tangle %f\n", RAD_TO_DEG( targetAngle));
#endif
      /* Determine how far the robot is allowed to rotate. */
      rightAngle = DEG_TO_RAD( 
			      computeCollisionDistance( rpos, rrot,
							0.0,
							ACTUAL_MODE->exception_rot_velocity, 
							ROB_RADIUS + 0.8 * ACTUAL_MODE->security_dist,
							0.0,
							0.0,
							&targetDist));
      
      leftAngle = DEG_TO_RAD( 
			     computeCollisionDistance( rpos, rrot,
						       0.0,
						       -ACTUAL_MODE->exception_rot_velocity, 
						       ROB_RADIUS + 0.8 * ACTUAL_MODE->security_dist,
						       0.0,
						       0.0,
						       &targetDist));


#ifdef ROTATE_DEBUG
      printf( "l %f  r %f\n", RAD_TO_DEG(leftAngle), RAD_TO_DEG(rightAngle));      
#endif      
      
      /* Sanity check. */
      if ( leftAngle >= DEG_360 && rightAngle >= DEG_360)
	leftAngle = rightAngle = DEG_180;
      else if ( leftAngle + rightAngle >= DEG_360)
	fprintf(stderr, "l %f r %f\n", RAD_TO_DEG(leftAngle), RAD_TO_DEG(rightAngle));

      /* Check for the best direction relative to the target direction. */
      for ( angleCnt = 0; angleCnt < NUMBER_OF_ANGLES_TO_BE_TRIED; angleCnt++) {
	
	relAngle = angleCnt * angleSkip;

	/* i counts over directions starting at the direction to the target location.
	 * To check whether the angle can safely be reached, we must convert it into
	 * direction relative to the robot heading. */

	/* Can the absolute angle be reached by rotation? */
	if ( relAngle >= leftAngle - ADDITIONAL_ROTATION)
	  leftDistances[angleCnt] = 0.0;
	else {
	  leftDistances[angleCnt] = 
	    computeCollisionDistance(
				     rpos, 
				     rrot + relAngle,
				     ACTUAL_MODE->target_max_trans_speed,
				     0.0, 
				     ROB_RADIUS+ACTUAL_MODE->security_dist,
				     0.0,
				     0.0,
				     &targetDist);
	}

	if ( relAngle >= rightAngle - ADDITIONAL_ROTATION)
	  rightDistances[angleCnt] = 0.0;
	else {
	  rightDistances[angleCnt] = 
	    computeCollisionDistance(
				     rpos, 
				     rrot - relAngle,
				     ACTUAL_MODE->target_max_trans_speed,
				     0.0, 
				     ROB_RADIUS+ACTUAL_MODE->security_dist,
				     0.0,
				     0.0,
				     &targetDist);
	}
	
#ifdef ROTATE_DEBUG
	printf( "robrot %f %f -> %f %f\n", RAD_TO_DEG(rrot), 
		RAD_TO_DEG(relAngle), 
		leftDistances[angleCnt], rightDistances[angleCnt]);
#endif
      }
      
      /* Now we know which angles can be reached and the free space 
       * in these directions.  Check which one is closest to the target direction. */
      for ( angleCnt = 0; angleCnt < NUMBER_OF_ANGLES_TO_BE_TRIED; angleCnt++) {
	
	relAngle = angleCnt * angleSkip;
	
	/* Check left turn. */
	if ( leftDistances[angleCnt] > securityDist) {

	  absoluteAngle = rrot + relAngle + angleSkip;

	  angleFromTarget = normed_angle( targetAngle - absoluteAngle);
	  if ( angleFromTarget > DEG_180)
	    angleFromTarget = DEG_360 - angleFromTarget;
	  
	  if ( angleFromTarget < closestAngle) {
	    closestAngle = angleFromTarget;
	    angleToRotate = - RAD_TO_DEG(normed_angle( absoluteAngle - rrot)) - 10.0;
	    printf( "left close -- %f\n", angleToRotate);
	  }
#ifdef ROTATE_DEBUG
	    printf( "left %f %f %f\n", leftDistances[angleCnt], RAD_TO_DEG(absoluteAngle),
		    RAD_TO_DEG(angleFromTarget));
#endif
	}
	/* Check right turn. */
	if ( rightDistances[angleCnt] > securityDist) {

	  absoluteAngle = rrot - relAngle - angleSkip;

	  angleFromTarget = normed_angle( absoluteAngle - targetAngle);
	  if ( angleFromTarget > DEG_180)
	    angleFromTarget = DEG_360 - angleFromTarget;

	  if ( angleFromTarget < closestAngle) {
	    closestAngle = angleFromTarget;
	    angleToRotate = RAD_TO_DEG(normed_angle( rrot - absoluteAngle)) + 10.0;
	    printf( "right close -- %f\n", angleToRotate);
	  }
#ifdef ROTATE_DEBUG
	  printf( "right %f %f %f\n", leftDistances[angleCnt], RAD_TO_DEG(absoluteAngle),
		  RAD_TO_DEG(angleFromTarget));
#endif
	}
      }
      
      /* If the robot rotates too far away or if the angle after rotation is
       * not much closer than before. */
      if ( fabs(closestAngle) > DEG_90) {
	fprintf( stderr, "\t\t\t -- no rotation found\n");
	if ( angleToRotate != 0.0)
	  fprintf( stderr, "\t\t\t --- close %f  rob %f\n", RAD_TO_DEG(closestAngle),
		   RAD_TO_DEG(angleFromTarget));
      	first_time = TRUE;
	
	rotateAwayFlag = FALSE;
	alreadyTriedRotation = TRUE;
	
	/* We'd like to translate but that's not possible as well. Just stop. */
	if ( alreadyTriedTranslation) {
	  fprintf(stderr, "\t\t\t --- Can't do anything.\n");
	  BASE_TranslateHalt();
	  BASE_RotateHalt();
	  target_flag = TRUE;
	  return;
	}
	else {
	  fprintf( stderr, "\t\t\t --- Try to translate.\n",
		   rightAngle, leftAngle);
	  achieveDistanceFlag = TRUE;
	  achieveDistanceTrajectory(rpos, rrot, numberOfExceptions);
	  return;
	}
      }
      else {
	
	BASE_TranslateCollisionAcceleration(ACTUAL_MODE->exception_trans_acceleration);
	BASE_RotateCollisionAcceleration( RAD_TO_DEG( ACTUAL_MODE->exception_rot_acceleration));
	BASE_TranslateHalt();
	BASE_RotateHalt();
	
	/* In the stop commands the target flag is set to FALSE. 
	 * We still want to be in the target mode.
	 */
	target_flag = TRUE;
	BASE_TranslateVelocity (0.0);
	BASE_RotateVelocity ( RAD_TO_DEG( ACTUAL_MODE->exception_rot_velocity));
	BASE_TranslateCollisionVelocity (0.0);
	BASE_RotateCollisionVelocity ( RAD_TO_DEG( ACTUAL_MODE->exception_rot_velocity));
	
	BASE_Rotate((double) angleToRotate);
	if (dumpInfo)
	  fprintf( dumpFile, " (%f) ", angleToRotate);
	fprintf(stderr, "rotate by (%f) degrees ... ", angleToRotate);
	first_time = FALSE;
	cnt = 0;
      }      
    }
    /* We have to wait until the rotation is finished. */
    else if ( ! stillInRotation()) {
      if (dumpInfo)
	fprintf( dumpFile, "done.\n");
      fprintf(stderr, "done.\n");
      rotateAwayFlag = FALSE;
      first_time = TRUE;
      BASE_TranslateCollisionAcceleration(ACTUAL_MODE->target_trans_acceleration);
      BASE_RotateCollisionAcceleration(RAD_TO_DEG(ACTUAL_MODE->target_rot_acceleration));
      BASE_TranslateCollisionVelocity (0.0);
      BASE_RotateCollisionVelocity (0.0);
    }
  }
  else
    fprintf( stderr, "Sorry. Only B21, B18, Pioneer, and Urban supported in rotateAwayTrajectory.\n");
}

/**********************************************************************
 * This routine is callled when the robot is too close to an obstacle.
 * In this case the robot just turns in the direction that is closer 
 * to free space than the other.
 **********************************************************************/
void
achieveDistanceTrajectory(Point rpos, float rrot, int numberOfExceptions)
{
  float backupDist, moveOnDist, target_dist;
  static BOOLEAN first_time = TRUE;
  static Point start_pos;
  static int cnt;
  
  if (0) fprintf(stderr, "achieve counter %d\n", numberOfExceptions);
  
  /* Maybe this was a wrong reading and the way is free again? */
  moveOnDist = computeCollisionDistance(rpos, rrot,
					ACTUAL_MODE->target_max_trans_speed, 0.0, 
					ROB_RADIUS+ACTUAL_MODE->security_dist,
					0.0,0.0,
					&target_dist);
  
  if ( moveOnDist > 3.0 * ACTUAL_MODE->min_dist) {
    if (dumpInfo)
      fprintf( dumpFile, "free again.\n");
    fprintf(stderr, "free again.\n");
    BASE_TranslateCollisionAcceleration(ACTUAL_MODE->target_trans_acceleration);
    BASE_RotateCollisionAcceleration(RAD_TO_DEG(ACTUAL_MODE->target_rot_acceleration));
    achieveDistanceFlag = FALSE;
    first_time = TRUE;
    return;
  }    

  /* Well there is still something in the way and we have to back up. */
  else {
    
    if ( numberOfExceptions > MAX_NUMBER_OF_EXCEPTIONS) {
      if ( numberOfExceptions > 10) {
	numberOfExceptions = 0;
      }
      else {
	return;
      }
    }
    
    /* Can the robot drive backward? */
    backupDist = computeCollisionDistance(rpos, rrot+DEG_180,
					  ACTUAL_MODE->target_max_trans_speed, 0.0, 
					  ROB_RADIUS + 0.5 * ACTUAL_MODE->security_dist,
					  0.0,0.0,
					  &target_dist);
    
    
#define BACKUP 20.0

    if (backupDist < BACKUP) {
      
      if ( armState != INSIDE) {
	fprintf( stderr, "Normally would try to rotate but better try to translate");
	fprintf( stderr, "when arm is out.\n");
      }
      else {
	fprintf(stderr, "Backup dist: %f\n", backupDist);
	first_time = TRUE;
	achieveDistanceFlag = FALSE;
	alreadyTriedTranslation = TRUE;
	
	if ( alreadyTriedRotation) {
	  /* Rotation is not allowed as well. */
	  fprintf(stderr, "Can't do anything.\n");
	  BASE_TranslateHalt();
	  BASE_RotateHalt();
	  target_flag = TRUE;
	  return;
	}
	else {
	  /* Rotation is the only thing we can do. */
	  if (first_time) {
	    if (dumpInfo)
	      fprintf( dumpFile, "Can't translate backward. Try to rotate.\n");
	    fprintf(stderr, "Can't translate backward. Try to rotate.\n");
	  }
	  else {
	    if (dumpInfo)
	      fprintf( dumpFile, "Can't translate backward any more. Try to rotate.\n");
	    fprintf(stderr, "Can't translate backward any more. Try to rotate.\n");
	    BASE_TranslateVelocity (0.0);
	    BASE_TranslateHalt();
	  
	    /* In the stop commands the target flag is set to FALSE. 
	     * We still want to be in the target mode.
	     */
	    target_flag = TRUE;
	  }
	  
	  rotateAwayFlag = TRUE;
	  rotateAwayTrajectory(rpos, rrot, numberOfExceptions);
	  return;
	}
      }
    }
    else {
      if (first_time) {
	if (dumpInfo)
	  fprintf( dumpFile, "Translate backward ");
	fprintf(stderr, "Translate backward ");
	start_pos = rpos;
	
	BASE_TranslateCollisionAcceleration( ACTUAL_MODE->exception_trans_acceleration);
	BASE_RotateCollisionAcceleration( RAD_TO_DEG( ACTUAL_MODE->exception_rot_acceleration));
	BASE_TranslateHalt();
	BASE_RotateHalt();
	
	/* In the stop commands the target flag is set to FALSE. 
	 * We still want to be in the target mode.
	 */
	target_flag = TRUE;
	BASE_TranslateVelocity (ACTUAL_MODE->exception_trans_velocity);
	BASE_RotateVelocity (0.0);
	BASE_TranslateCollisionVelocity (ACTUAL_MODE->exception_trans_velocity);
	BASE_RotateCollisionVelocity (0.0);
	
#define BACKUP_WITH_ARM_OUT 40.0
	
	if ( ! colli_go_backward) {
	  if ( armState != INSIDE)
	    BASE_Translate( - BACKUP_WITH_ARM_OUT);
	  else
	    BASE_Translate(- BACKUP);
	}
	else {
	  if ( armState != INSIDE)
	    BASE_Translate(  BACKUP);
	  else
	    BASE_Translate(  BACKUP);
	}
	first_time = FALSE;
	cnt = 0;
      }
      
      /* We have to wait until the translation is finished. */
      else {
	cnt++;
	if ( cnt > 15 || ! stillInTranslation()) {
	  if (dumpInfo)
	    fprintf( dumpFile, "done.\n");
	  fprintf(stderr, "done.\n");
	  BASE_TranslateCollisionAcceleration(ACTUAL_MODE->target_trans_acceleration);
	  BASE_RotateCollisionAcceleration(RAD_TO_DEG(ACTUAL_MODE->target_rot_acceleration));
	  achieveDistanceFlag = FALSE;
	  first_time = TRUE;
	  cnt = 0;
	}
	else
	  fprintf(stderr, "cnt %d\n", cnt);
      }
    }
  }
}







