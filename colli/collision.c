
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/collision.c,v $
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
 * $Log: collision.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.41  1999/12/06 13:19:05  fox
 * Added time stamps to base and laser report.
 *
 * Revision 1.40  1999/11/15 13:28:06  fox
 * *** empty log message ***
 *
 * Revision 1.39  1999/10/14 19:56:40  fox
 * IMPORTANT change: if the orientation of the robot is off more than 60 deg.
 * it will prefer rotation a lot.
 *
 * Revision 1.38  1999/09/27 23:17:40  fox
 * Minor changes.
 *
 * Revision 1.37  1999/09/24 14:29:26  fox
 * Added support for scout robot.
 *
 * Revision 1.36  1999/09/08 22:21:12  fox
 * Removed a debug info.
 *
 * Revision 1.35  1999/09/08 21:11:51  fox
 * *** empty log message ***
 *
 * Revision 1.34  1999/07/23 19:46:37  fox
 * Added ability to read grid maps for the obstacle server.
 *
 * Revision 1.32  1999/04/18 19:00:06  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
 *
 * Revision 1.31  1999/03/09 15:48:41  wolfram
 * Added geometry of PIONEER_II
 *
 * Revision 1.30  1998/12/10 17:45:07  fox
 * Changed some settings for the pioneer at.
 *
 * Revision 1.29  1998/11/22 23:54:21  fox
 * Fixed a bug for rectangular robots. If tvel == 0, then the distance
 * to the next obstacle is set to zero. This is done to avoid too much
 * rotation.
 *
 * Revision 1.28  1998/11/19 03:14:32  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.27  1998/11/12 18:37:01  fox
 * Version running on the pioneers.
 *
 * Revision 1.26  1998/10/23 20:50:32  fox
 * *** empty log message ***
 *
 * Revision 1.25  1998/09/18 15:44:22  fox
 * First attempt to get a running system after museum.
 *
 * Revision 1.24  1998/09/12 21:26:46  fox
 * Final version of the museum.
 *
 * Revision 1.23  1998/09/05 00:20:01  fox
 * Version for internet night in Washington.
 *
 * Revision 1.22  1998/08/29 21:50:04  fox
 * Adapted exception handling in order to deal with people around the robot.
 *
 * Revision 1.21  1998/08/26 23:23:43  fox
 * Final version before I changed exception handling.
 *
 * Revision 1.20  1998/08/18 16:24:26  fox
 * Added support for b18 robot.
 *
 * Revision 1.19  1998/05/13 07:19:41  fox
 * Minor changes.
 *
 * Revision 1.18  1998/05/13 07:07:41  fox
 * Fixed some bugs I found due to graphical output.
 *
 * Revision 1.17  1998/01/22 13:04:23  fox
 * Minor changes.
 *
 * Revision 1.16  1997/11/12 17:07:30  fox
 * Removed some old arm stuff.
 *
 * Revision 1.15  1997/06/17 09:39:31  fox
 * Changed rotate away.
 *
 * Revision 1.14  1997/05/14 08:10:57  fox
 * Fixed a bug when setting random mode.
 *
 * Revision 1.13  1997/04/10 12:08:22  fox
 * Incorporated support for the tactiles.
 *
 * Revision 1.12  1997/03/14 17:48:59  fox
 * Changed laser messages.
 *
 * Revision 1.11  1997/02/28 12:57:10  fox
 * Minor changes.
 *
 * Revision 1.10  1997/02/11 17:59:27  fox
 * Don't use sonar in FIND_DOOR_MODE.
 *
 * Revision 1.9  1997/01/10 17:50:13  fox
 * *** empty log message ***
 *
 * Revision 1.8  1997/01/07 13:52:39  fox
 * Removed debug output.
 *
 * Revision 1.7  1997/01/07 13:47:15  fox
 * Improved rotate_away.
 *
 * Revision 1.6  1997/01/03 18:08:58  fox
 * Made some minor changes.
 *
 * Revision 1.5  1996/12/23 15:50:36  fox
 * Added minor support for the bumpers. COLLI stops whenever a bumper has
 * triggered a signal.
 *
 * Revision 1.4  1996/12/23 12:48:54  fox
 * First attempt to incorporate the tactiles.
 *
 * Revision 1.3  1996/10/14 17:30:55  fox
 * Fixed a minor bug to prefer straight motion.
 *
 * Revision 1.2  1996/10/09 13:59:55  fox
 * Changed the directory where to find the parameter file colli_modes.ini.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:05  rhino
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

#ifdef CLEANUP
#include "colliCleanUp.h"
#endif

#include "colliGnuPlot.h"

/* The module that sent the approach command. */
TCX_REF_PTR TCX_sender_of_action = NULL;

#define WATCHDOG_INTERVAL 1.5

/* The following values must only be changed within a norm file */
/* See "collision.hlp" for further explanation.  */

float MAX_RANGE = 800.0;

float MIN_ROT_SPEED = DEG_TO_RAD(1.0); 
float MIN_TRANS_SPEED = 0.1;

float MAX_CURVE_RADIUS = 20000.0;
float MIN_DIST = 15.0;
float MAX_COLLISION_LINE_LENGTH = 20.0;

/* These values influence the target trajectories */
float MIN_DIST_FOR_TARGET_WAY_FREE = 300.0;

float TARGET_TRANS_ACCELERATION = 50.0;
float TARGET_ROT_ACCELERATION = DEG_TO_RAD(50.0);
float TARGET_MAX_ROT_SPEED = DEG_TO_RAD(60.0);
float TARGET_MAX_TRANS_SPEED = 50.0;

int REMEMBER_INTERVAL = 2;

int NUMBER_OF_RVELS = 11;
int NUMBER_OF_TVELS = 10;

/* The cycle time of the collision avoidance. */
float COLLISION_UPDATE_INTERVAL = 0.3;
float ACCELERATION_ASSERTION_TIME = 0.3;

float DISTANCE_FACTOR = 0.5;
float VELOCITY_FACTOR = 0.8;
float ANGLE_FACTOR = 2.0;
float MIN_SPEED_FOR_NARROW_BEAM = 30.0;
float DOOR_ANGLE = DEG_TO_RAD(60.0);
LineSeg **TargetLines;
int SMOOTH_WIDTH = 2;
float SECURITY_DIST = 8.0;
float EDGE_PORTION = 0.5;
float *histogram;

/* With these values the speed dependend security_dist is computed. */
float MAX_SECURITY_DIST  =  50.0;
float MIN_SECURITY_SPEED =  5.0;
float MAX_SECURITY_SPEED =  120.0;


/* Values for collision avoidance with arm outside. */
int armState = INSIDE;

int STACK_SIZE = 100;
float SECURITY_ANGLE = 25.0;
float ROLLBACK_DISTANCE = 30.0;
float ROLLBACK_MIN_TRANSLATION = 15.0;
float ROLLBACK_MIN_ROTATION = 15.0;
int   ROTATE_AWAY_PERSISTENCE = 3;

ObstaclePoints combinedObstaclePoints[NUMBER_OF_SENSORS];
BOOLEAN useSensorPoints[NUMBER_OF_SENSORS];

int exceptionCnt = 0;

/**********************************************************************
 **********************************************************************
 *                     Collision functions                            *
 **********************************************************************
 **********************************************************************/

void update_CollisionStatus(Pointer, Pointer);

/**********************************************************************
 * Checks whether the direct way to the target is free and changes
 * the velocity factor.
 * targetWayFree:    
 * FALSE --> nothing special
 * TRUE --> direct way is free and for the evaluation of the trajectories 
 * the minimization of the angle to the target is more important. 
 * So we reduce VELOCITY_FACTOR in the evaluation function.
 * If ANGLE_FACTOR == 0.0 we only want to maximize the velocity and targetWayFree
 * is not necessary.
 **********************************************************************/
BOOLEAN
checkIfTargetWayFree( Point rpos,
		      float targetDist,
		      float targetRot,
		      BOOLEAN targetWayIsAlreadyFree)
{
  float collDist;
  
  if (ACTUAL_MODE->angle_factor == 0.0) 
    /* Nothing to be done. */
      return FALSE;
  else {
    
    /* We test if the direct way to the target is free. If the angle to the target
     * is bigger than DOOR_ANGLE  we use a very narrow beam.
     */
    if ( fabs( targetRot) < DOOR_ANGLE ||
	actual_velocities.current_tvel < MIN_SPEED_FOR_NARROW_BEAM)
      /* We use MIN_TRANS_SPEED because we don't want to add speed dependend
       * security dist. 
       */
      
      collDist =
	collDistInTargetDirection( rpos,
				MIN_TRANS_SPEED,
				target,
				ROB_RADIUS + ACTUAL_MODE->security_dist);
    else
      collDist =
	collDistInTargetDirection( rpos,
				  MIN_TRANS_SPEED,
				  target,
				  ROB_RADIUS);
    
    if ( /* The target is closer than the obstacle. */
	(collDist > targetDist) ||
	/* Enough free space, we don't want to look further. */
	(collDist > ACTUAL_MODE->min_dist_for_target_way_free) ||
	/* Enough space to get close to the target. */
	(mode_number == APPROACH_OBJECT_MODE &&
	 collDist > (targetDist - fabs(approachDist) + ROB_RADIUS))) {

	/* The first time. */
	if ( ! targetWayIsAlreadyFree) {
	    if (dumpInfo)
		fprintf( dumpFile, "Free!\n");
/*  	    fprintf(stderr, "Free!\n"); */
	}
	return TRUE;
    }
    else if ( targetWayIsAlreadyFree) {
	if (dumpInfo)
	    fprintf( dumpFile, "Not free any longer!\n");
/*  	fprintf(stderr, "Not free any longer!\n");  */
    }
    return FALSE;
  }
  return FALSE; 
}


/* Constants used to norm the values. */
#define MAX_VEL 90.0
#define MAX_ANG DEG_180
#define MAX_DIST 800.0
   
/**********************************************************************
 * Evaluation funciton. If the direct way to the target is free the angle
 * becomes very important.
 **********************************************************************/
float
evaluationFunction( float* velocity,
		    float* angle,
		    float* distance,
		    BOOLEAN targetWayFree,
		    BOOLEAN rotateToTargetPoint)
{
  if ( mode_number == RANDOM_MODE) {
    return VELOCITY_FACTOR * *velocity + DISTANCE_FACTOR * *distance;
  }
  
  /* The robot's heading differs more than 60 degrees from the taget direction. */
  if ( *angle < 0.3) {
    *velocity *= -0.5;
    *distance *= 0.1;
  }
  else if ( targetWayFree) {
    *velocity *= 0.1;
    *distance *= 0.1;
  }
  else if ( rotateToTargetPoint) {
    *velocity *= 0.1;
    *distance *= 0.1;
  }

  return 
    VELOCITY_FACTOR * *velocity + ANGLE_FACTOR * *angle + DISTANCE_FACTOR * *distance;
}


/**********************************************************************
 * Checks whether a velocity on a curvature is admissible. 
 **********************************************************************/
BOOLEAN
admissible( Point rpos, float rrot,
	    float tvel, float rvel,
	    int onlyTranslation, int onlyRotation,
	    float *collDist,
	    float* targetDist)
{
  float distance;
  float rotationDist,translationDist;
  
  if ( onlyTranslation && rvel != 0.0) {
    return FALSE;
  }

  if ( onlyRotation && tvel != 0.0)
    return FALSE;
       
#ifdef TEST
  if ( tvel == 0) {
    distance = MIN( MAX_DIST,
		    collisionDistance( rpos,
				       rrot, 
				       tvel, 
				       rvel,
				       targetDist));
    fprintf(stderr, "rve %f  -> %f\n", RAD_TO_DEG(rvel), distance);
  }
  else
    distance = 0;
#endif    
  distance = MIN( MAX_DIST,
		  collisionDistance( rpos,
				     rrot, 
				     tvel, 
				     rvel,
				     targetDist));

  /* For security we substract the distance which will be reached after
   * the next time interval.
   */
  
  if (fabs(tvel)<EPSILON)
    {
      if ( robotShape == ROUND_ROBOT)
	rotationDist = MAX( 0.0,
			    ( distance*DEG_360/MAX_RANGE) -
			    fabs( COLLISION_UPDATE_INTERVAL *
				  actual_velocities.current_rvel));
      else
	rotationDist = MAX( 0.0,
			    ( distance) -
			    fabs( COLLISION_UPDATE_INTERVAL *
				  actual_velocities.current_rvel));
    }
  else
    {
      if (fabs(rvel)<EPSILON)
	{
	  rotationDist = DEG_360;
	}
      else
	{
	  /* Compute the angle from the distance. */
	  if ( robotShape == ROUND_ROBOT)
	    rotationDist = MAX( 0.0,
				( distance / (fabs(tvel/rvel))) -
				fabs( COLLISION_UPDATE_INTERVAL *
				      actual_velocities.current_rvel));
	  else
	    rotationDist = MAX( 0.0,
				( distance) -
				fabs( COLLISION_UPDATE_INTERVAL *
				      actual_velocities.current_rvel));
	}
    }

  translationDist = MAX( 0.0,
			 distance -
			 fabs( 2.0 * COLLISION_UPDATE_INTERVAL *
			       actual_velocities.current_tvel));
  
  if (0) fprintf(stderr, "td %f -> %f  --> %f\n", distance, translationDist, 
	  maxVelForEmergencyStop( translationDist,
				  ACTUAL_MODE->target_trans_acceleration));


  *collDist = distance;
  
  if (dumpInfo) {
    fprintf( dumpFile, "tv: %f  rv: %f\n", tvel, rvel);
    fprintf( dumpFile, "coll distance: %f\n", distance);
  }
  
  /* We first check whether the distance to the obstacle is big enough. */
  if ((translationDist < ACTUAL_MODE->min_dist) ||
      ((rotationDist < DEG_TO_RAD( ACTUAL_MODE->security_angle)) &&
       (armState != INSIDE) && (fabs(tvel) < EPSILON)))
    {
      return FALSE; 
    }
  else
    {
      /* If the arm is not inside we also have to check whether rotation is possible. */
      if (armState != INSIDE)
	{  
	  if ((rvel >
	       maxVelForEmergencyStop( rotationDist,
				       ACTUAL_MODE->target_rot_acceleration))
	      || (tvel >
		  maxVelForEmergencyStop( translationDist,
					  ACTUAL_MODE->target_trans_acceleration)))
	    {
	      return FALSE;
	    }
	  else 
	    {
	      return TRUE;
	    }
	}
      else
	{
	  if ( tvel >
	       maxVelForEmergencyStop( translationDist,
				       ACTUAL_MODE->target_trans_acceleration)) {
	    return FALSE;
	  }
	  else {
	    return TRUE;
	  }
      }
   }
}




/**********************************************************************
 * Computes the evaluation of the distance to the next obstacle.
 **********************************************************************/
float
distanceEvaluation( float tvel, float rvel,
		    float collDist, float targetDist)
{
  /* If the target is on the trajectory and closer than the
   * obstacles we set the evaluation to maximum. */
  if ( targetDist < collDist)
    return MAX_RANGE;
   
  /* To evaluate the distance we don't want to use the maximum
    * value for very small circular trajectories. That's why
    * we take the maximum from the collDist and the circumference of
    * the circle.
    */

  if ( rvel != 0.0) {
    float circumference = fabs( DEG_360 * ( tvel / rvel ));
    return MIN( MAX_RANGE, MIN( collDist, circumference)); 
  }
  else      
    /* Straight trajectory. Use the original distance. */
    return MIN( MAX_RANGE, collDist);
}


/**********************************************************************
 * Computes the evaluation of the translational velocity.
 **********************************************************************/
float
velocityEvaluation( float tvel)
{
   return tvel;
}


/**********************************************************************
 * Computes the evaluation of the heading towards the target.
 **********************************************************************/
float
angleEvaluation( Point rpos, float rrot,
		float tvel, float rvel)
{
  /* We compute the angle to the target with respect to the velocities
   * and accelerations of the robot.
   * First we compute the angle to the target on the lookahead point.
   */  
  float remainderAngle =
    fabs(
	 lookaheadTargetAngle(target, rpos, rrot, 
			      tvel, rvel,
			      ACTUAL_MODE->target_trans_acceleration,
			      ACTUAL_MODE->target_rot_acceleration,
			      4.0 * COLLISION_UPDATE_INTERVAL));

  /* The value has to be maximal if the remainderAngle is minimal. */
  return DEG_180 - remainderAngle;
}
   

/**********************************************************************
 * Evaluates the different combinations of tvel and rvel.
 * Stores the values in the three matrices.
 * Return value shows whether there is any combination admissible with
 * tvel != 0.0;
 **********************************************************************/
BOOLEAN
evaluateCombinations( Point rpos, float rrot,
		      VelocityCombination** combinations,
		      float** velocities,
		      float** distances,
		      float** angles)
{
   int i, j;
   int onlyTranslation, onlyRotation;
   float collDist, targetDist;
   BOOLEAN noTranslationAllowed = TRUE;

   /* We don't want the robot to move very slow. Thats why we set some combinations
    * to undefined. To determine the slow combinations we set the fraction of
    * velocities to be deleted. */
#define VELOCITY_DELETE_FRACTION 0.5

   float minTransVelocity =
     MIN( actual_velocities.max_tvel,
	  ACTUAL_MODE->target_trans_acceleration *
	  ACCELERATION_ASSERTION_TIME)
     * VELOCITY_DELETE_FRACTION;

   float maxNegativeRotVelocity =
      MAX( actual_velocities.min_rvel,
	  -ACTUAL_MODE->target_rot_acceleration *
	  ACCELERATION_ASSERTION_TIME)
	 * VELOCITY_DELETE_FRACTION;
   
   float minPositiveRotVelocity =
      MIN( actual_velocities.max_rvel,
	  ACTUAL_MODE->target_rot_acceleration *
	  ACCELERATION_ASSERTION_TIME)
	 * VELOCITY_DELETE_FRACTION;

   /* Check for the current situation whether only translation or only
    * rotation is allowed. */
   checkWhetherOnlyRotationOrOnlyTranslation( rpos,
					     rrot, 
					     &onlyTranslation,
					     &onlyRotation);

   if ( onlyRotation)
     fprintf(stderr, "\t\tROTATION\n");
   if ( onlyTranslation)
     fprintf(stderr, "\t\tTRANSLATION\n");
     
   for ( i = 0; i < ACTUAL_MODE->number_of_rvels; i++) {
      
      for ( j = 0; j < ACTUAL_MODE->number_of_tvels; j++) {
	 
	float tvel = combinations[i][j].tvel;
	float rvel = combinations[i][j].rvel;
	
	/* We skip all trajectories for which the value of
	 * tvel and rvel is less than the corresponding MIN_VELOCITY. */
	if ( ( fabs(tvel) < minTransVelocity) &&
	      (( rvel <= 0.0 && rvel > maxNegativeRotVelocity) || 
	       ( rvel >= 0.0 && rvel < minPositiveRotVelocity))) {
	   if (0) printf("%f #tvsk\n", tvel); 
	   velocities[i][j] = UNDEFINED;
	   angles[i][j]     = UNDEFINED;
	   distances[i][j]  = UNDEFINED;
	 }
	 else if (! admissible( rpos,
				rrot, 
				tvel, 
				rvel,
				onlyTranslation,
				onlyRotation,
				&collDist,
				&targetDist)) {
	   if (0) printf("%f %f #tvna\n", collDist, tvel); 
	   
	   velocities[i][j]  = UNDEFINED;
	   angles[i][j]      = UNDEFINED;
	   distances[i][j]   = UNDEFINED;
	 }
	 else {  /* Everyting is fine. Evaluate the velocities. */
	   noTranslationAllowed = noTranslationAllowed && tvel < EPSILON;
	   if (0) printf("%f %f #tvad\n", collDist, tvel); 
	   
	   velocities[i][j]  = velocityEvaluation( tvel);
	   
	   angles[i][j]      = angleEvaluation( rpos,
						rrot,
						tvel,
						rvel);
	   distances[i][j]   = distanceEvaluation( tvel,
						   rvel,
						   collDist,
						   targetDist);
	 }
      }
   }

   return noTranslationAllowed;
}


/**********************************************************************
 * Norms the values of each matrix into [0;1]. Returns the index with
 * the best evaluation.
 **********************************************************************/
VelocityCombination
findBestEvaluation( BOOLEAN targetWayFree,	/* Determines the evaluation function */
		    float targetRot,      /* Influences rotateToTargetPoint */
		    VelocityCombination** combinations, /* Velocities */
		    float** velocities,   /* Evaluation of velocity. */
		    float** distances,    /* Evaluation of distance. */
		    float** angles,       /* Evaluation of angle. */
		    float** values)       /* Weighted sum of evaluations. */
{
   int i, j;
   float bestValue = 1e-10;
   VelocityCombination bestCombination = {0.0, 0.0};

  /* If this flag is TRUE the robot will use a special
   * evaluation function which emphasizes the importance of
   * the target angle.
   */
   BOOLEAN rotateToTargetPoint = checkIfRotateToTargetPoint(targetRot);
   
   for ( i = 0; i < ACTUAL_MODE->number_of_rvels; i++) {
     
     for ( j = 0; j < ACTUAL_MODE->number_of_tvels; j++) {
       
       if ( velocities[i][j] != UNDEFINED &&
	    distances[i][j] != UNDEFINED &&
	    angles[i][j] != UNDEFINED)  {

	 /* Norm the values. */
	 velocities[i][j] /= MAX_VEL;
	 angles[i][j] /= MAX_ANG;
	 distances[i][j] /= MAX_DIST;
	 
	 values[i][j] = evaluationFunction(
					   &(velocities[i][j]),
					   &(angles[i][j]),
					   &(distances[i][j]),
					   targetWayFree,
					   rotateToTargetPoint);

       }
       else
	 values[i][j] = UNDEFINED;
     }
   }
   
   /* Now let's smooth the histogram of the best values. */
   smoothGrid( values,
	       ACTUAL_MODE->number_of_rvels,
	       ACTUAL_MODE->number_of_tvels,
	       ACTUAL_MODE->smooth_width);
   
   for ( i = 0; i < ACTUAL_MODE->number_of_rvels; i++) 
     for ( j = 0; j < ACTUAL_MODE->number_of_tvels; j++) {
       
       /* If its better --> take it */
       if ( values[i][j] > bestValue) {
	 bestValue = values[i][j];
	 bestCombination = combinations[i][j];  
       }
       /* If its as good --> we prefer straight motion, otherwise throw the dice */
       else if ( values[i][j] == bestValue)
	 /* We prefer straight motion. */
	 if ( ( fabs( combinations[i][j].rvel) < bestCombination.rvel) || (rand() % 2 == 0)) {
	   bestValue = values[i][j];
	   bestCombination = combinations[i][j];
	 }
     }
   
   return bestCombination;
}


/**********************************************************************
 * Computes the best traectory to the target point. 
 **********************************************************************/
trajectory
computeTargetTrajectory(Point rpos, float rrot)
{
   
   /* Those two values will be needed frequently in the remainder of the function. */
   float targetDist = compute_distance(rpos, target);
   float targetRot = compute_robot_angle_to_point(rpos, rrot, target);
   
   /* If no translation is allowed we use the an exceptional mode. */
   BOOLEAN noTranslationAllowed;
      
   /* The next matrizes contain for each trajectory the rvel / tvel
    * combination. */
   VelocityCombination** combinations;
   VelocityCombination bestCombination;
   
   /* The next matrizes are used to store the evaluation for the rvels
    * and tvels described above.
    */
   float** velocities;
   float** distances;
   float** angles;
   float** values;
   
   /* If the direct way to the target is free we reduce the velocity factor. */
   static BOOLEAN targetWayFree;

   /* For random movement we check whether straight motion is ok. */
   if ( mode_number == RANDOM_MODE) {
     if ( straightMotionIsPossible( rpos, rrot)) {
       exceptionCnt = 0;
       fprintf(stderr, "|");
       return closest_possible_trajectory( ACTUAL_MODE->target_max_trans_speed, 0.0);
     }
#ifdef STRAIGHT_OR_RANDOM_DIRECTION
     else {
       trajectory dummy = {0,0,0};
       
       bestCombination.tvel = 0.0;
       bestCombination.rvel = 1.0;
       
       exceptionCnt++;
       startExceptionHandling( rpos,
			       rrot,
			       bestCombination,
			       exceptionCnt);
       return dummy;
     }
#endif
   }

   if ( robotType != B18_ROBOT)
     targetWayFree = checkIfTargetWayFree( rpos,
					   targetDist,
					   targetRot,
					   targetWayFree);
   else
     targetWayFree = FALSE;
   
   
   /* Allocates memory for the matrizes. */
   allocateEvaluations( &velocities, &distances, &angles, &values,
			ACTUAL_MODE->number_of_rvels,
			ACTUAL_MODE-> number_of_tvels); 
   
   /* Selects points from the dynamic window. */
   generateVelocityCombinations( &combinations,
				 ACTUAL_MODE->number_of_rvels,
				 ACTUAL_MODE-> number_of_tvels); 
   
   /* We check whether the robot has reached the target and do
    * what is necessary considering the actual mode.  If the
    * function returns TRUE we also quit this function.
    */
   if ( handleArrivalAtTarget( rpos, rrot, &targetDist, &targetRot)) {
      trajectory dummy;
      dummy.tvel = 0.0;
      dummy.rvel = 0.0;
      dummy.admissible = FALSE;
      return dummy;
   }
   
   
   /* Uses the evaluation function for each of the combinations.
    * Also checks whether there is one combination admissible
    * which includes translation. */
   noTranslationAllowed = evaluateCombinations( rpos, rrot,
						combinations,
						velocities,
						distances,
						angles);
   
   /* Looks for the combination with the best value.
    * Also stores the best values in COLLGRAPH structures.
    */
   bestCombination = findBestEvaluation( targetWayFree,
					 targetRot,
					 combinations,
					 velocities,
					 distances,  
					 angles,     
					 values);

   COLLI_DumpEvaluationFunction( combinations,
				 velocities,
				 distances,
				 angles,
				 NULL,
				 values,
				 ACTUAL_MODE->velocity_factor,
				 ACTUAL_MODE->distance_factor,
				 ACTUAL_MODE->angle_factor,
				 ACTUAL_MODE->smooth_width,
				 ACTUAL_MODE->number_of_rvels,
				 ACTUAL_MODE->number_of_tvels);
   
   if ( noTranslationAllowed) {
     /* This might have two reasons.
      * 1. The robot is only allowed to rotate.
      * 2. There is no admissible trajectory at all.
      */
     trajectory dummy;
     dummy.tvel = 0.0;
     dummy.rvel = 0.0;
     dummy.admissible = FALSE;
     
     /* in dependence of the armState we decide to execute the corresponding
	exception-handling */
     if (armState != INSIDE) {

	 startHalting( rpos, rrot);
      }
     else {
       exceptionCnt++;

       startExceptionHandling( rpos,
			       rrot,
			       bestCombination,
			       exceptionCnt);
     }
     return dummy;
   }
   else {
     exceptionCnt = 0;
     return closest_possible_trajectory( bestCombination.tvel, bestCombination.rvel);
   }
}


/**********************************************************************
 * This routine is called by the base. It checks the current velocities
 * of the base and tries to get as close as possible to the desired
 * velocities. The maximum translational velocity is determined by the
 * distance to the next obstacle represented by collision lines fro
 * the sonar or vision. 
 * If the target flag is set best trajectory to the target point is 
 * computed with respect to the obstacles.
 * Because this routine is computational expensive we often check the 
 * sonar device to send new sonar commands.
 **********************************************************************/
void update_CollisionStatus(Pointer callback_data, Pointer client_data)
{
  struct timeval begint;
  float rrot;			/* values used for collision avoidance */ 
  Point rpos;
  float collDist, targetDist=MAX_RANGE;
  trajectory desired_trajectory; /* This trajectory is set by BASE commands or
				  * computed in computeTargetTrajectory(). */

  static int firstTime = TRUE;

  gettimeofday( &begint, 0);

    /* If the position is set for the first time, then reset the collision structures. */
  if ( firstTime) {
    COLLI_reset_obstacle_structs();
    firstTime = FALSE;
  }

  /* Status reports come too frequently, so we reject most of them. */
  if ( ! alreadyInNextUpdateInterval()) {
    return;
  }

  robotStuck = FALSE;

  if ( use_collision) {
    
    BASE_WatchDogTimer((int)( WATCHDOG_INTERVAL * 1000.0)); 
    outputCollisionStatus();
    
#ifdef GNUPLOT
    rwi_base.trans_current_speed = 40.0;
    rwi_base.rot_current_speed = DEG_TO_RAD(0.0);
#else
    /* The data is too old and there is nothing to do here. */
    if ( !update_check_ok()) return;
#endif
    
    /* The actual velocities are stored in the global structure "actual_velocities" */
    updateActualConfiguration( &rpos, &rrot);     
      
#ifdef CLEANUP
    /* e.g. the robot tries to pick up an object. */
    if ( inCleanUpSequence( rpos, rrot))
      return;
#endif
      
    /* If a target is given we try to find the best trajectory. This trajectory
     * is stored in desired_trajectory.
     */
    if ( target_flag) {
#ifdef GNUPLOT
      computeGnuPlotTrajectory( rpos, rrot);
#endif
      
#ifndef GNUPLOT
      if ( haltingFlag || rotatingAwayFlag) {
	/* There is an exception with arm outside.  */
	exceptionHandling(rpos, rrot);
	COLLI_send_colli_update();
	return;
      }
      if ( keepOnWithExceptionHandling( rpos, rrot, exceptionCnt)) {
	/* There is an exception with arm inside.  */
	COLLI_send_colli_update();
	return;
      }
#endif

      /* In the normal target mode only positive tvel is allowed.
       * achieveDistanceFlag and rotateAwayFlag are set when the
       * robot is too close to an object.
       */
      if ( robotType != B18_ROBOT && actual_velocities.current_tvel < -5.0)  {
	if (dumpInfo) {
	  fprintf( dumpFile, "Negative velocity (%f) with target ",
		   actual_velocities.current_tvel);
	  fprintf( dumpFile, "not allowed. Try to brake.\n");
	}
	
	BASE_TranslateCollisionVelocity (0.0);
	BASE_RotateCollisionVelocity (0.0);
	
	return;
      }
      
      /***************************************************************
       ***************************************************************
       * Everything seems to be normal. Let's compute the best
       * trajectory.
       ***************************************************************
	 ****************************************************************/
      else {
	actual_velocities.current_tvel =
	  MAX(0.0, actual_velocities.current_tvel);
	setStartTime( EVERYBODIES_TIMER);
	desired_trajectory = computeTargetTrajectory(rpos, rrot);
	if (0) fprintf(stderr, "\t\t%f\n", timeExpired(EVERYBODIES_TIMER));
      }
    }
    /* Maybe in computeTargetTrajectory() an exception was detected.
     * If so we quit this function.
     */
    
    /* In this case the robot is in a local minimum and has to escape. */
    if ( haltingFlag || rotatingAwayFlag || achieveDistanceFlag || rotateAwayFlag) {
      COLLI_send_colli_update();
      return;
    }
    
#ifdef CLEANUP
    /* The robot just started to turn to the basket or to pick up an object. */
    if ( inCleanUpSequence(rpos, rrot))
      return;
#endif
    /* If no target is given we check whether the desired velocities are
     * admissible. 
     */
    if ( ! target_flag) {
      if (!rot_wanted){
	desired_trajectory =
	  closest_possible_trajectory( desired_trans_velocity, 0.0);
      }
      else{
	desired_trajectory =
	  closest_possible_trajectory( desired_trans_velocity,
				      DEG_TO_RAD(desired_rot_velocity));
      }
    }

    /* The desired trajectory whether computed in computeTargetTrajectory() or
     * determined by the user via BASE commands has to be checked for
     * admissibility.
     */

    collDist = MIN( MAX_DIST,
		    collisionDistance( rpos,
				       rrot,
				       desired_trajectory.tvel,
				       desired_trajectory.rvel,
				       &targetDist));
    SONAR_look_for_sonar_device();

    /* We want to stop when we reach the target.
     * We add MIN_DIST for faster approach. */
    if ( (mode_number == APPROACH_OBJECT_MODE
	  || mode_number == APPROACH_TRASH_BIN_MODE )
	&& target_flag
	&& (targetDist - fabs( approachDist) + ROB_RADIUS) < collDist)
      collDist = targetDist + ACTUAL_MODE->min_dist; 
    
    /* Let's send the trajectory to the COLLGRAPH module. */
    COLLI_update_tcx_trajectory( rpos, rrot,
				 desired_trajectory.tvel,
				 desired_trajectory.rvel,
				 collDist);

    COLLI_send_colli_update();

#ifdef GNUPLOT
    return;
#endif
    /* Set the velocities according to the distance to the next obstacle. */
    setVelocities(rpos, collDist, desired_trajectory); 
    
    /* We have to change the direction of the translational movement
     * according to colli_go_backward.
     */
    if (target_flag) 
      setDirections( desired_trajectory);
    outputTimeInfo( &begint);
  }
}


