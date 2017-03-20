
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliBase.c,v $
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
 * $Log: colliBase.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.13  1999/09/27 23:17:39  fox
 * Minor changes.
 *
 * Revision 1.12  1999/09/24 14:29:26  fox
 * Added support for scout robot.
 *
 * Revision 1.11  1999/09/08 21:11:50  fox
 * *** empty log message ***
 *
 * Revision 1.10  1999/07/23 19:46:36  fox
 * Added ability to read grid maps for the obstacle server.
 *
 * Revision 1.8  1999/06/25 19:48:08  fox
 * Minor changs for the urbie.
 *
 * Revision 1.7  1998/11/19 03:14:31  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.6  1998/09/18 15:44:22  fox
 * First attempt to get a running system after museum.
 *
 * Revision 1.5  1998/09/12 21:26:45  fox
 * Final version of the museum.
 *
 * Revision 1.4  1998/08/29 21:50:01  fox
 * Adapted exception handling in order to deal with people around the robot.
 *
 * Revision 1.3  1998/08/26 23:23:39  fox
 * Final version before I changed exception handling.
 *
 * Revision 1.2  1998/08/18 16:24:22  fox
 * Added support for b18 robot.
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

float transVelocitySetInPreviousInterval = 0.0;
float rotVelocitySetInPreviousInterval = 0.0;
BOOLEAN tryToAccelerate = FALSE;

velocities actual_velocities;
BOOLEAN rot_wanted = FALSE;

/**********************************************************************
 **********************************************************************
 *       Functions especially for collision avoidance                 *
 **********************************************************************
 **********************************************************************/

/**********************************************************************
 * The actual values are set. The rotation is set such that the
 * common mathematical functions can be applied to it.
 * At the same time the velocities are updated and stored in
 * the global structure "actual_velocities".  
 **********************************************************************/
void
updateActualPosition( Point* rpos, float* rrot,
		      BOOLEAN considerDirection)
{
  rpos->x = (float) rwi_base.pos_x;
  rpos->y = (float) rwi_base.pos_y;
  
   /* If the robot should move backwards we just turn it by 180.0
    * degrees. The rest of the computation stays the same.
    */
  if ( !considerDirection || !colli_go_backward)
    *rrot   = normed_angle( (float) DEG_TO_RAD(90.0 -  rwi_base.rot_position)); 
  else
    *rrot   = normed_angle( (float) DEG_TO_RAD(270.0 -  rwi_base.rot_position));
}


/**********************************************************************
 * The actual values are set. The rotation is set such that the
 * common mathematical functions can be applied to it.
 * At the same time the velocities are updated and stored in
 * the global structure "actual_velocities".  
 **********************************************************************/
void
updateActualConfiguration( Point* rpos, float* rrot)
{
  updateActualPosition( rpos, rrot, CONSIDER_DIRECTION);
  
  compute_current_velocities();
  compute_possible_velocities( fabs( rwi_base.trans_acceleration),
			       DEG_TO_RAD( fabs( rwi_base.rot_acceleration)));
  compute_possible_trajectories();
}



/**********************************************************************
 * If the robot should move backward we have to swap the velocities.
 **********************************************************************/
void
compute_current_velocities(void)
{
  actual_velocities.current_tvel = (float) rwi_base.trans_current_speed; 
  actual_velocities.current_rvel = (float) DEG_TO_RAD(rwi_base.rot_current_speed);

  if ( actual_velocities.current_tvel < -5.0) {
    fprintf(stderr, "neg %f\n", actual_velocities.current_tvel);
    actual_velocities.current_tvel = 0.0;
  }

  /* #define TEST */
#ifdef TEST
  if (target_flag)
    printf( "%f %f #setrev\n", actual_velocities.current_tvel, 
	    transVelocitySetInPreviousInterval);
  if ( 0 && (1 || tryToAccelerate)) {
    actual_velocities.current_tvel = transVelocitySetInPreviousInterval;
    actual_velocities.current_rvel = rotVelocitySetInPreviousInterval;
    /*      0.5 * (actual_velocities.current_tvel + velocitySetInPreviousInterval); */
  }
#endif  

  if ( robotShape != ROUND_ROBOT && tryToAccelerate) {
    actual_velocities.current_tvel = 
      0.5 * (actual_velocities.current_tvel + transVelocitySetInPreviousInterval);
  }

  if (actual_velocities.current_tvel < MIN_TRANS_SPEED) {
    if (rwi_base.trans_set_direction == NEGATIVE)
      actual_velocities.current_tvel *= -1.0;
  }
  else
    if (rwi_base.trans_direction == NEGATIVE)
      actual_velocities.current_tvel *= -1.0;
  
  if (actual_velocities.current_rvel < MIN_ROT_SPEED) {
    if (rwi_base.rot_set_direction == NEGATIVE)
      actual_velocities.current_rvel *= -1.0;
  }
  else
    if (rwi_base.rot_direction == NEGATIVE)
      actual_velocities.current_rvel *= -1.0;

  /* If the robot should go backward, we change the sign of the velocity.
   * Now we can do the same computations as for the forward direction.
   */
  if ( colli_go_backward) 
      actual_velocities.current_tvel *= -1.0;
  
  return;
}


/**********************************************************************
 * Compute maximal allowed velocities.
 * The maximal velocity is set in the desired_trajectory. If the coll_dist
 * is too small to brake desired_trajectory.admissible is set to FALSE.
 **********************************************************************/
void
setVelocities(Point rpos, float coll_dist, trajectory desired_trajectory)
{
  float tvel, rvel, tmp_tvel=0.0;
  int directionChanged = FALSE;

  compute_max_velocity(coll_dist, &desired_trajectory);

  if ( !desired_trajectory.admissible) {
    /* trajectory not allowed ---> emergencyStop */
    tvel = 0.0;  
    rvel = 0.0;  
    if ( desired_trajectory.tvel < 0.0)
      directionChanged = TRUE;

    if (!emergencyStop) {
      BASE_TranslateCollisionAcceleration(50.0);
      emergencyStop = TRUE;
      if (dumpInfo)
	fprintf( dumpFile, "Oops!\n");
      fprintf( stderr, "Oops! Obstacle too close.\n");
    }
    rwi_base.emergency = TRUE;
  }
  
  else {
    if (target_flag) {

      /* Only positive tvel. 0.8 because the robot cannot stop immediately. */
      tmp_tvel = 0.8 * fsqrt( SQR( targetArriveSpeed) +
			      2.0 * ACTUAL_MODE->target_trans_acceleration *
			      compute_distance(rpos, target));
      
      /* The targetArriveSpeed depends on the position of the nextTarget. */
      desired_trajectory.max_tvel = MIN(desired_trajectory.max_tvel, tmp_tvel);
    }
    
    if ( emergencyStop && desired_trajectory.tvel != 0.0) {
      BASE_TranslateCollisionAcceleration(ACTUAL_MODE->target_trans_acceleration);
      emergencyStop = FALSE;
      if (dumpInfo)
	fprintf( dumpFile, "ok\n");
      if (0) fprintf(stderr, "ok %f %f\n", desired_trajectory.tvel, RAD_TO_DEG(desired_trajectory.rvel));
    }
    
    if ( desired_trajectory.tvel > 0.0) 
      tvel = MIN(desired_trajectory.tvel, desired_trajectory.max_tvel);
    else 
      tvel = MAX(desired_trajectory.tvel, desired_trajectory.min_tvel);
    
    
    if ( fabs(desired_trajectory.tvel) > EPSILON){
      rvel = RAD_TO_DEG(desired_trajectory.rvel * tvel / desired_trajectory.tvel); 
    }
    else {
      rvel = RAD_TO_DEG(desired_trajectory.rvel);
    }
  }
  

  /* We have to change the direction of the translational movement
   * according to colli_go_backward.
   */
  if ( !colli_go_backward) 
    rwi_base.trans_set_direction = (tvel >= 0.0) ? 
	  POSITIVE : NEGATIVE;
  else
    rwi_base.trans_set_direction = (tvel < 0.0) ? 
      POSITIVE : NEGATIVE;
  
  if ( directionChanged) {
    if (rwi_base.trans_set_direction == POSITIVE)
      rwi_base.trans_set_direction = NEGATIVE;
    else
      rwi_base.trans_set_direction = POSITIVE;
  }

  rwi_base.rot_set_direction = (rvel >= 0.0) ? 
      POSITIVE : NEGATIVE;

  if ( tvel > actual_velocities.current_tvel) {
    tryToAccelerate = TRUE;
  }
  else
    tryToAccelerate = FALSE;

  {
    static float prevX = 0.0;
    static float prevY = 0.0;
    static float prevDist = 0.0;

    float c_dist = MAX( 0.0, coll_dist - 
		     fabs( COLLISION_UPDATE_INTERVAL *
			   actual_velocities.current_tvel));
    float m_tvel_for_required_space =
      maxVelForEmergencyStop( c_dist, ACTUAL_MODE->target_trans_acceleration);

    float max_tvel_for_required_space =
      maxVelForEmergencyStop( coll_dist, ACTUAL_MODE->target_trans_acceleration);


    float movement = sqrt( SQR(rwi_base.pos_x - prevX) + SQR( rwi_base.pos_y - prevY));

/* #define TEST */
#ifdef TEST
    if ( target_flag && 
	 (desired_trajectory.tvel != tvel || 
	 RAD_TO_DEG(desired_trajectory.rvel) != rvel))
      printf( "%f %f %f %f %f %f %f %f #rti\n", coll_dist, c_dist,
	      tvel, m_tvel_for_required_space, max_tvel_for_required_space, 
	      tmp_tvel, desired_trajectory.tvel);
    else 
      printf( "%f %f %f %f %f %f %f %f #rt\n", coll_dist, c_dist,
	      tvel, m_tvel_for_required_space, max_tvel_for_required_space, 
	      tmp_tvel, desired_trajectory.tvel);
    
#endif
    prevX = rwi_base.pos_x;
    prevY = rwi_base.pos_y;
    prevDist = coll_dist;
  }
  
  if (0) fprintf(stderr, "%f %f %f\n", tvel, rvel, coll_dist);

  transVelocitySetInPreviousInterval = tvel;
  rotVelocitySetInPreviousInterval = rvel;

  BASE_TranslateCollisionVelocity ((double) fabs(tvel));
  BASE_RotateCollisionVelocity ((double) fabs(rvel));
}	




/**********************************************************************
 * We have to change the direction of the translational movement
 * according to colli_go_backward.
 **********************************************************************/
void
setDirections( trajectory desiredTrajectory)
{
  if ( colli_go_backward) {
    if (desiredTrajectory.tvel < 0.0)
      BASE_TranslateForward();
    else
      BASE_TranslateBackward();
  }
  else {
    if (desiredTrajectory.tvel < 0.0)
      BASE_TranslateBackward();
    else
      BASE_TranslateForward();
  }
  if (desiredTrajectory.rvel > 0.0)
    BASE_RotateClockwise();
  else
    BASE_RotateAnticlockwise();
}




