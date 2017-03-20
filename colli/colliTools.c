
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliTools.c,v $
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
 * $Log: colliTools.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.27  1999/11/15 13:28:06  fox
 * *** empty log message ***
 *
 * Revision 1.26  1999/10/14 19:56:40  fox
 * IMPORTANT change: if the orientation of the robot is off more than 60 deg.
 * it will prefer rotation a lot.
 *
 * Revision 1.25  1999/09/24 14:29:26  fox
 * Added support for scout robot.
 *
 * Revision 1.24  1999/09/08 21:11:51  fox
 * *** empty log message ***
 *
 * Revision 1.23  1998/11/22 23:54:20  fox
 * Fixed a bug for rectangular robots. If tvel == 0, then the distance
 * to the next obstacle is set to zero. This is done to avoid too much
 * rotation.
 *
 * Revision 1.22  1998/11/19 03:14:32  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.21  1998/10/23 20:50:30  fox
 * *** empty log message ***
 *
 * Revision 1.20  1998/09/12 21:26:45  fox
 * Final version of the museum.
 *
 * Revision 1.19  1998/09/05 00:20:00  fox
 * Version for internet night in Washington.
 *
 * Revision 1.18  1998/08/29 21:50:03  fox
 * Adapted exception handling in order to deal with people around the robot.
 *
 * Revision 1.17  1998/08/26 23:23:42  fox
 * Final version before I changed exception handling.
 *
 * Revision 1.16  1998/08/18 16:24:25  fox
 * Added support for b18 robot.
 *
 * Revision 1.15  1998/05/13 07:07:41  fox
 * Fixed some bugs I found due to graphical output.
 *
 * Revision 1.14  1998/01/16 11:11:16  schulz
 * Added a message BASE_reset_obstacle_line_field, which allows to clear
 * the obstacle line field via TCX.
 *
 * Revision 1.13  1998/01/14 00:37:25  thrun
 * New option "-laserserver" lets the base receive laser date from
 * the laserServer. This is the recommended option. It seems to be
 * much more reliable than reading in the data directly.
 *
 * Revision 1.12  1997/11/12 17:07:29  fox
 * Removed some old arm stuff.
 *
 * Revision 1.11  1997/10/23 10:56:04  rhino
 * Added -tactile flag.
 *
 * Revision 1.10  1997/07/17 17:31:46  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.9  1997/05/23 15:40:26  fox
 * Rotation is finished after timeout of colli.
 *
 * Revision 1.8  1997/05/06 17:05:04  fox
 * Nothing special.
 *
 * Revision 1.7  1997/04/17 09:16:19  fox
 * Added timeout for laser devices --> colliServer only needs 30% cpu.
 *
 * Revision 1.6  1997/04/10 12:08:22  fox
 * Incorporated support for the tactiles.
 *
 * Revision 1.5  1997/03/26 18:42:02  fox
 * Incorporated external obstacle information into the collision avoidance.
 *
 * Revision 1.4  1997/02/05 15:41:03  fox
 * Implemented support for both laser scanners.
 *
 * Revision 1.3  1996/12/23 15:50:35  fox
 * Added minor support for the bumpers. COLLI stops whenever a bumper has
 * triggered a signal.
 *
 * Revision 1.2  1996/10/14 17:30:54  fox
 * Fixed a minor bug to prefer straight motion.
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
#include "LASER_SERVER-messages.h"


/* These times are used to check wether the data is actual enough. */
struct timeval Last_CollLine_Update;
struct timeval Last_VisionLine_Update;
struct timeval Last_VisionPoint_Update;
struct timeval Last_FrontLaserPoint_Update;
struct timeval Last_RearLaserPoint_Update;


extern float DEFAULT_TRANSLATION_SPEED;
extern float DEFAULT_ROTATION_SPEED;

/* These values are used to check for time intervals. */
int timeInterval;
struct timeval beginTime;

int next_CollLine_reading = 0;
LineSeg **CollLines;

ObstacleLines External_Obstacle_Lines;

#define MAX_RECOVER_TIME 20.0

/**********************************************************************
 **********************************************************************
 *                         Init functions                            *
 **********************************************************************
 **********************************************************************/



/**********************************************************************/
void

COLLI_init_obstacle_structs(void)
{
  int i,j;
  void COLLI_reset_obstacle_structs(void);

  /* Allocate memory for CollLines*/
  CollLines = (struct LineSeg **) malloc(bRobot.sonar_cols[0] * sizeof(struct LineSeg *));
  for (i = 0; i < bRobot.sonar_cols[0]; i++)
    CollLines[i] = (struct LineSeg *) malloc( REMEMBER_INTERVAL * sizeof(struct LineSeg));

  /* Bumper. */
  combinedObstaclePoints[BUMPER_POINTS].points = (struct Point*)
    malloc( NUMBER_OF_POINTS_PER_BUMPER * MAX_NUMBER_OF_BUMPERS * sizeof(struct Point));
  
  /* Ir. */
  combinedObstaclePoints[IR_POINTS].points = (struct Point*)
    malloc( MAX_NUMBER_OF_IRS * sizeof(struct Point));

  /* Sonar. */
  combinedObstaclePoints[SONAR_POINTS].no_of_points = 
    bRobot.sonar_cols[0] * REMEMBER_INTERVAL * POINTS_PER_SONAR;
  
  combinedObstaclePoints[SONAR_POINTS].points = (struct Point*)
    malloc( bRobot.sonar_cols[0] * REMEMBER_INTERVAL * POINTS_PER_SONAR
	    * sizeof(struct Point));

  /* Laser. */
  combinedObstaclePoints[LASER_POINTS].points = (struct Point*)
    malloc( MAX_NUMBER_OF_LASER_POINTS * sizeof(struct Point));

  COLLI_reset_obstacle_structs();

  /* Check which sensors to use. */		
  useSensorPoints[SONAR_POINTS] = use_sonar;
  useSensorPoints[IR_POINTS] = use_ir;
  useSensorPoints[LASER_POINTS] = use_laser;
  useSensorPoints[BUMPER_POINTS] = use_bumper;
  useSensorPoints[EXTERNAL_POINTS] = TRUE;
}

/**********************************************************************/
/* The following function is used by the BASE_reset_obstacle_line_field    */
/* TCX message (ds 14/01/98)
/**********************************************************************/

void
COLLI_reset_obstacle_structs(void)
{
  int i,j;
  extern LaserPoints Laser_CollPoints;

  next_CollLine_reading = 0;
  
  External_Obstacle_Lines.no_of_lines = 0;

  combinedObstaclePoints[EXTERNAL_POINTS].no_of_points = 0;
  combinedObstaclePoints[BUMPER_POINTS].no_of_points = 0;
  combinedObstaclePoints[IR_POINTS].no_of_points = 0;
  combinedObstaclePoints[LASER_POINTS].no_of_points = 0;

  for (i=0; i < bRobot.sonar_cols[0]; i++) {
    for (j=0; j<REMEMBER_INTERVAL; j++)
      CollLines[i][j].pt1.x = F_ERROR;
  }

  for (i=0; i < combinedObstaclePoints[SONAR_POINTS].no_of_points; i++) {
    combinedObstaclePoints[SONAR_POINTS].points[i].x = F_ERROR;
  }

  /* Reset lasers. */
  Laser_CollPoints.numberOfPoints    = 0;
  for (i=0; i < MAX_NUMBER_OF_LASER_POINTS; i++) {
    combinedObstaclePoints[LASER_POINTS].points[i].x = F_ERROR;
  }
}



/******************************************************************************
 * We set some default values and allocate memory for global variables        *
 ******************************************************************************/

void
init_collision_avoidance( char* iniFile)
{
  int i;

  /* We use lookup tables for sin and cos for faster computation. */
  init_fast_sin();
  init_fast_cos();

  /* First we install an array of modes. All values in the modes are set to default. 
   * So every non specified value in colli_modes.ini will be ok.
   */
  mode_structure_array = (struct mode_structure **) 
    malloc( NUMBER_OF_MODES * sizeof(struct mode_structure *)); 
  
  /* Now we read the values for the differnt modes. */
  load_parameters( iniFile);

  /* If no mode is set we use the default mode. */
  ACTUAL_MODE = mode_structure_array[DEFAULT_MODE];
  mode_number = DEFAULT_MODE;

  BASE_SetIntervalUpdates((long) ( COLLISION_UPDATE_INTERVAL * 1000.0)); 
  
  BASE_RotateCollisionAcceleration(RAD_TO_DEG(ACTUAL_MODE->target_rot_acceleration));
  BASE_TranslateCollisionAcceleration(ACTUAL_MODE->target_trans_acceleration);
  BASE_RotateVelocity( DEFAULT_ROTATION_SPEED);
  BASE_TranslateVelocity( DEFAULT_TRANSLATION_SPEED);

  /* No target specified */
  target_flag = FALSE;

  /* No target reached */
  rwi_base.collision_state.target_reached = FALSE;
  
  /* For recovery from sonar errors */
  gettimeofday(&Last_CollLine_Update, 0);
  
  /* For recovery from laser errors */
  gettimeofday(&Last_FrontLaserPoint_Update, 0);
  gettimeofday(&Last_RearLaserPoint_Update, 0);
  
  /* We want to delete old vision lines and points */
  gettimeofday(&Last_VisionLine_Update, 0);
  gettimeofday(&Last_VisionPoint_Update, 0);

  /* Allocate memory for the sonar angles and consider mask. */
  SonarAngle = (float *) malloc( bRobot.sonar_cols[0] * sizeof(float));
  considerSonar = (int *) malloc( bRobot.sonar_cols[0] * sizeof(int));

  initTcxStructures();
  init_SonarStructs();
  initLaserPointsStructure();
  COLLI_init_obstacle_structs();
}


/**********************************************************************/
void
COLLI_start_collision_avoidance( char* iniFile)
{
  
  init_collision_avoidance( iniFile);

  BASE_InstallHandler(update_CollisionStatus, STATUS_REPORT, NULL);  
  
  if (use_bumper) 
    BASE_InstallHandler(COLLI_BumpHandler, BUMP, NULL);  

  if ( use_sonar) {
    SONAR_InstallHandler(update_CollLines, SONAR_RT_COMPLETE, NULL);   
    SONAR_InstallHandler(Inc_next_reading, SONAR_REPORT, NULL);
    SONAR_LoopStart(sonar_mask_array[0]);
  }


  /* Convert the distances into points after each scan. */
  if ( use_laser) {
    LASER_InstallHandler( update_LaserPoints, SINGLE_LASER_REPORT, NULL);
  }
}



/**********************************************************************/
void
Inc_next_reading(Pointer callback_data, Pointer client_data)
{
  next_CollLine_reading = (next_CollLine_reading+1) % REMEMBER_INTERVAL;
}


BOOLEAN 
alreadyInNextUpdateInterval()
{
  if (robotShape == ROUND_ROBOT)
    return TRUE;
  else {

    static struct timeval Last_Collision_Avoidance;
    static float intervalBoarder;
    struct timeval now;
    float tDiff;
    
    static BOOLEAN firstTime = TRUE;
    
    if (firstTime) {
      intervalBoarder = - 0.1;
      gettimeofday(&Last_Collision_Avoidance,0);
      firstTime = FALSE;
      return TRUE;
    }
    
    gettimeofday(&now,0);
    
    /* Let's check whether the lasers have to be pinged. */
    checkLaserPing( &now);
    
    /* Time longer than expected interval. */
    tDiff = timeDiff( &now, &Last_Collision_Avoidance) 
      - COLLISION_UPDATE_INTERVAL;
    
    if ( tDiff > intervalBoarder) {
      Last_Collision_Avoidance = now;
      return TRUE;
    }
    else
      return FALSE;
  }
}


/* We compute the time since the last call to this routine. 
 * When has the collision avoidance been updated the last time? */
static BOOLEAN
checkBaseUpdate( struct timeval* now)
{
  static struct timeval Last_Collision_Avoidance;
  float tDiff;

  static BOOLEAN firstTime = TRUE;
  if (firstTime) {
    gettimeofday(&Last_Collision_Avoidance,0);
    firstTime = FALSE;
  }

  tDiff = timeDiff( now, &Last_Collision_Avoidance);

  if (0) printf("%f #b\n", tDiff);

  Last_Collision_Avoidance = *now;

  if ( tDiff > 0.8) {
    
    BOOLEAN targetFlagSave = target_flag;
    
    if ( dumpInfo) {
      fprintf(dumpFile, "No collision avoidance for (%f) seconds!\n", tDiff);
      fprintf(stderr, "Stop the robot\n");
    }
    
    fprintf(stderr, "\n\nNo collision avoidance for (%f) seconds!\n\n", tDiff);
    fprintf(stderr, "Stop the robot\n");
    BASE_TranslateHalt();
    BASE_RotateHalt();
    target_flag = targetFlagSave;
    return FALSE;
  }
  else
    return TRUE;
}  


static BOOLEAN
checkSonarUpdate( struct timeval* now)
{
    
  float tDiff = timeDiff( now, &Last_CollLine_Update);
  
  if ( tDiff > SONAR_UPDATE_TIME) {
    
    if ( dumpInfo) {
      fprintf(dumpFile, "No sonar for %f seconds. ", tDiff);
    }
    fprintf( stderr, "No sonar for %f seconds. ", tDiff);

    /* Try to start the loop again. */
    if (base_device.dev.use_simulator) {
      if (dumpInfo)
	fprintf( dumpFile, "Try to start sonar simulation loop again.\n");
      SONAR_LoopStart(sonar_act_mask);
    }
    return FALSE;	 /* Sonar too old. */
  }
  else
    return TRUE;    /* Sonar up to date. */
}


static BOOLEAN
checkLaserUpdate( struct timeval* now)
{
  float frontTDiff = timeDiff( now, &Last_FrontLaserPoint_Update);
  float rearTDiff = timeDiff( now, &Last_RearLaserPoint_Update);
  
  int laserUpToDate = TRUE;

  if (0) printf("%f %f #l\n", frontTDiff, rearTDiff);
  
  if ( use_laser && use_laser_server) {
    if ( LASER_SERVER == NULL) {
      LASER_SERVER = tcxConnectOptional(moduleName[LASER_SERVER_MODULE]); 
      if ( LASER_SERVER != NULL) {
	LASER_SERVER_register_auto_update_type subscribe;
    
	fprintf(stderr, "Connected.\n");
	subscribe.sweep0        = 1;
	subscribe.sweep1        = 1;
	tcxSendMsg(LASER_SERVER, "LASER_SERVER_register_auto_update", &subscribe);
      }
    }
  }

  /* Check front laser and send request if necessary. */
  if ( USE_FRONT_LASER && (frontTDiff > LASER_UPDATE_TIME)) {
    fprintf(stderr, "No FRONT laser for %f seconds. ", frontTDiff); 
    laserUpToDate = FALSE;  
  }

  /* Check rear laser and send request if necessary. */
  if ( USE_REAR_LASER && (rearTDiff > LASER_UPDATE_TIME)) {
    fprintf(stderr, "No REAR laser for %f seconds. ", rearTDiff);
    laserUpToDate = FALSE;
  }
  
  return laserUpToDate;
}



/**********************************************************************
 * Checks wether the collision information (sonar lines, vision lines and
 * vision points) are up to date and wether the 
 * time since last collision avoidance is not too big.
 **********************************************************************/
BOOLEAN
update_check_ok(void)
{
  static BOOLEAN robotStoppedAlready = FALSE;
  static BOOLEAN targetFlagAtStop;
  static struct timeval timeOfStop; 
  BOOLEAN stopRobot = FALSE;
  
  struct timeval now;
  gettimeofday( &now, 0);

  /* We compute the time since the last call to this routine. */
  /* When has the collision avoidance been updated the last time? */
  if ( ! checkBaseUpdate( &now))
    return FALSE;
  
  updateBumperTimer();
  
  /*----------------------------------------------------------------
   * Check for proximity sensors.
   *----------------------------------------------------------------*/

  /* We compute the time since last data received from sonar. *
   * If the data is too old, we stop the robot.               */
  stopRobot = use_sonar && ! checkSonarUpdate( &now);
  
  /* We compute the time since last data received from laser. *
   * If the data is too old, we stop the robot.               */
  stopRobot = stopRobot || ( use_laser && ! checkLaserUpdate( &now));
  
  /* Stop the robot if the proximity information is too old. */
  if ( stopRobot) {

    /* Is it the first time to stop the robot?
     * Save the target flag and the current time */
    if ( ! robotStoppedAlready) {
      targetFlagAtStop    = target_flag;
      target_flag         = FALSE;
      robotStoppedAlready = TRUE;
      gettimeofday( &timeOfStop, 0);

      fprintf( stderr, "Stop the robot\n");
      BASE_TranslateHalt();
      BASE_RotateHalt();
    }
  }
  else {

    if ( robotStoppedAlready) {
      
      /* Recover from the stop if it is recent enough. */
      if ( timeDiff( &now, &timeOfStop) < MAX_RECOVER_TIME) {
	rwi_base.stopped_by_colli = TRUE;
	fprintf( stderr, "Activate target again.\n");
	target_flag = targetFlagAtStop;
      }
    }

    robotStoppedAlready = FALSE;
  }

  return ! stopRobot;
}



/**********************************************************************
 **********************************************************************
 *            Functions to check the state of the robot and
 *               the collision avoidance.
 **********************************************************************
 **********************************************************************/




/**********************************************************************
 * This routine is called to check wether the robot is still in rotation.
 **********************************************************************/
BOOLEAN 
stillInRotation()
{
  static int rotationCnt = 0; 
  static int noRotationCnt = 0; 

#define MIN_ROT_WAIT_CNT 2
#define MIN_NO_ROTATION_CNT 1


  /* We want to wait at least two cycles at the beginning. */ 
  if (rotationCnt++ > MIN_ROT_WAIT_CNT &&
      fabs(actual_velocities.current_rvel) < MIN_ROT_SPEED) { 
    if ( ++noRotationCnt > MIN_NO_ROTATION_CNT) {
      noRotationCnt = 0; 
      rotationCnt = 0; 
      return FALSE;
    }
    else {
      if (dumpInfo) {
	fprintf( dumpFile, "No rotation, but still wait.\n");
      }
      return TRUE;
    }
  } 
  else { 
    if (dumpInfo) {
      fprintf ( dumpFile, "rv %d  %f\n", rotationCnt, 
	       RAD_TO_DEG(actual_velocities.current_rvel)); 
    }
    return TRUE; 
  }
}

/**********************************************************************
 * This routine is called to check wether the robot is still in rotation.
 **********************************************************************/
BOOLEAN 
stillInTranslation()
{
  static int translationCnt = 0;
  static int noTranslationCnt = 0;

#define MIN_TRANS_WAIT_CNT 3
#define MIN_NO_TRANSLATION_CNT 0

  /* We want to wait at least two cycles at the beginning. */
  if (translationCnt++ > MIN_TRANS_WAIT_CNT &&
      fabs(actual_velocities.current_tvel) < MIN_TRANS_SPEED) {
    if ( ++noTranslationCnt > MIN_NO_TRANSLATION_CNT) {
      noTranslationCnt = 0; 
      translationCnt = 0; 
      return FALSE;
    }
    else {
      if (dumpInfo) {
	fprintf( stderr, "No translation, but still wait.\n");
	fprintf( dumpFile, "No translation, but still wait.\n");
      }
      return TRUE;
    }
  }
  else {
    if (dumpInfo) {
      fprintf ( dumpFile, "tv %d  %f\n", translationCnt,
	       actual_velocities.current_tvel);
      fprintf (stderr, "tv %d  %f\n", translationCnt,
	       actual_velocities.current_tvel);
    }
    return TRUE;
  }
}



/**********************************************************************
 * The next two functions are used to check for a time interval.
 **********************************************************************/

void
setTimer( int secs)
{
  fprintf( stderr, "Set timer to %d secs.\n", secs);
  timeInterval = secs;
  gettimeofday( &beginTime, 0);
}


BOOLEAN
stillInTimeInterval()
{
  struct timeval now;

  gettimeofday( &now, 0);
  now.tv_usec -= beginTime.tv_usec;
  now.tv_sec -= beginTime.tv_sec;
  if (now.tv_usec < 0) {
    now.tv_usec += 1000000;
    now.tv_sec -= 1;
  }
  
  if ( dumpInfo) {
    if ( (int) now.tv_sec % 10 == 0 && (int) now.tv_sec != 0) {
      fprintf( dumpFile, "Sec (%.4f)\n",
	      (float)now.tv_sec + (float)now.tv_usec / 1000000.0);
      fprintf( stderr, "Sec (%.4f)\n",
	      (float)now.tv_sec + (float)now.tv_usec / 1000000.0);
    }
  }
  if (now.tv_sec > timeInterval)
    return FALSE;
  else return TRUE;
}




/**********************************************************************
 **********************************************************************
 *            Functions to determine admissible velocities.
 **********************************************************************
 **********************************************************************/

     

/**********************************************************************
 *  Tests wether a point is behind the robot.
 **********************************************************************/
BOOLEAN
behindPoint( Point rpos, float rrot, Point pt)
{
  return ( fabs( compute_robot_angle_to_point( rpos, rrot, pt)) >=  DEG_90);
}


/**********************************************************************
 *   Computes the angle distance from the robot at rpos to pt.        *
 **********************************************************************/
float
compute_robot_angle_to_point(Point rpos, float rrot, Point target)
{
  float angle; 

  angle = rrot - compute_angle_2p(rpos, target);

  if (angle < -DEG_180)
    angle = DEG_360 + angle;
  else if (angle > DEG_180)
    angle -= DEG_360;

  return(angle);
}


/**********************************************************************
 * Computes the maximal velocity such that the robot can stop within the 
 * distance. 
 * The formula is the solution to:
 * coll_dist = -0.5 * a * SQR(t) + tvel * t      and
 * tvel = -a * t
 ***********************************************************************/
float
maxVelForEmergencyStop(float distance, float acceleration)
{
  return 0.8 * fsqrt( 2.0 * distance * acceleration);
}


/**********************************************************************
 * Computes the distance covered by the robot for a given velocity
 * and acceleration until it stops.
 ***********************************************************************/
float
brakeDistance( float velocity, float acceleration)
{
  if (fabs(acceleration) > EPSILON) {
    float time = fabs(velocity / acceleration);
    float dist = fabs(velocity) * time + 0.5 * acceleration * SQR(time);
    return velocity > 0.0 ? dist : -dist;
  }
  else
    return(MAXFLOAT);
}

/**********************************************************************
 * Computes the time needed to stop.
 ***********************************************************************/
float
brakeTime( float velocity, float acceleration)
{
   if (fabs(acceleration) > EPSILON) 
      return fabs(velocity / acceleration);
  else
    return(MAXFLOAT);
}


/**********************************************************************
 **********************************************************************
 *            Functions to determine admissible velocities.
 **********************************************************************
 **********************************************************************/


/* Checks wether direct way to the target is free. */
BOOLEAN
checkForTargetWayFree( Point rpos, float rrot, float tvel,
		       float radius, Point target)
{
  float targetDist, collDist;
  
  /* The distance to the target. */
  targetDist = compute_distance(rpos, target);
  
  /* The distance to the next obstacle. */
  collDist = collDistInTargetDirection( rpos,
				       tvel,
				       target,
				       radius);
  
  if (collDist > targetDist) 
    return TRUE;
  else
    return FALSE;
}





/**********************************************************************
 * The security dist should depend on the velocity. 
 **********************************************************************/
float
speed_dependent_security_dist(float tvel)
{
  float tmp;

  /* If tvel is MIN_SECURITY_SPEED the additional dist should be 0.0.
   * If tvel is MAX_SECURITY_SPEED the additional dist should be MAX_SECURITY_DIST
   */

#ifdef QUADRATIC_GROWTH  
  tmp = fnorm( fabs(tvel),
	       ACTUAL_MODE->min_security_speed,
	       ACTUAL_MODE->max_security_speed, 0.0, 1.0);
  tmp = SQR( MAX(0.0, tmp));

  return( tmp * ACTUAL_MODE->max_security_dist);
#else
  tmp = fnorm( fabs(tvel),
	       ACTUAL_MODE->min_security_speed,
	       ACTUAL_MODE->max_security_speed, 0.0, ACTUAL_MODE->max_security_dist);
#endif

  return( MAX( 0.0, tmp));
}



/**********************************************************************
 * The maximal velocity is stored in traj->max_tvel if traj->tvel positive
 * (otherwise in traj->min_tvel).
 **********************************************************************/
void
compute_max_velocity( float coll_dist, trajectory *traj)
{
  
  float max_tvel_for_required_space;
  float min_tvel, max_tvel, tmp;

  traj->admissible = TRUE;

  /* This gives us the security that the robot has one more cycle
   * to stop.
   */
  if (fabs(traj->tvel) < EPSILON)
  {
    if ( robotShape == ROUND_ROBOT)
      coll_dist = MAX( 0.0, (coll_dist*DEG_360/MAX_RANGE) - 
		       fabs( COLLISION_UPDATE_INTERVAL *
			     actual_velocities.current_rvel));
    else
      coll_dist = MAX( 0.0,
		       ( coll_dist) -
		       fabs( COLLISION_UPDATE_INTERVAL *
			     actual_velocities.current_rvel));
    
  }
  else
    {
      coll_dist = MAX( 0.0, coll_dist - 
		       fabs( COLLISION_UPDATE_INTERVAL *
			     actual_velocities.current_tvel));
    }

  /* If the robot is in the emergencyStop we are more conservative about
   * starting the robot again.
   */
  if ( 0 && emergencyStop) coll_dist *= 0.5;

  if( fabs(traj->tvel) < EPSILON)
  {
    if (coll_dist < DEG_TO_RAD(ACTUAL_MODE->security_angle))
      {
	traj->admissible = FALSE;
	return;
      }
  }
  else
  {
    if( coll_dist < ACTUAL_MODE->min_dist)
      { 
	traj->admissible = FALSE;
	return;
      }
  }
  /* Which velocity is relevant? If tvel is negative the most safe velocity in this
   * direction is max_tvel, otherwise it is min_tvel. 
   */

  if (traj->tvel == 0.0) {
    /* No collision possible so we don't have to change any velocity. */
    traj->max_tvel = traj->min_tvel = 0.0;
    return;
  }
 
  if (traj->tvel < 0.0) {
    min_tvel = MAX(0.0, -traj->max_tvel);
    max_tvel = -traj->min_tvel;
  }
  else {
    min_tvel = MAX(0.0, traj->min_tvel);
    max_tvel = traj->max_tvel;
  }
    
    
  max_tvel_for_required_space =
    maxVelForEmergencyStop( coll_dist, ACTUAL_MODE->target_trans_acceleration);
  
  if (min_tvel >= max_tvel_for_required_space) {
    /* Trajectory not allowed. Emergency_stop not possible */
    traj->admissible = FALSE;
    return;
  }
  
  else  
    /* We use the biggest velocity without speed reduction */
    tmp = MIN(max_tvel, max_tvel_for_required_space);
  
  if (traj->tvel < 0.0) 
     traj->min_tvel = -tmp;
  else if (traj->tvel > 0.0) 
     traj->max_tvel = tmp;
}



/**********************************************************************
 * Given the actual velocities and accelerations this function computes
 * the possible velocities within the next ACCELERATION_ASSERTION_TIME
 * seconds. The values are stored in the structure actual_velocities.
 **********************************************************************/
void
compute_possible_velocities( float tacc, float racc)
{
  float delta_tv, delta_rv;
  float max_tvel, max_rvel;
  
  max_tvel = target_flag ? ACTUAL_MODE->target_max_trans_speed : fabs(desired_trans_velocity);
  max_rvel = target_flag ? ACTUAL_MODE->target_max_rot_speed : fabs(DEG_TO_RAD(desired_rot_velocity));

  if (tacc < 0.0) {
    if ( dumpInfo)
      fprintf(dumpFile, "Error. Negative trans acceleration.\n");
    fprintf(stderr, "Error. Negative trans acceleration.\n");
    tacc = -tacc;
  }

  if (racc < 0.0) {
    if (dumpInfo)
      fprintf(dumpFile, "Error. Negative rot acceleration.\n");
    fprintf(stderr, "Error. Negative rot acceleration.\n");
    racc = -racc;
  }

  delta_tv = tacc * ACCELERATION_ASSERTION_TIME;
  
  actual_velocities.max_tvel = MIN( max_tvel, actual_velocities.current_tvel + delta_tv);

  
  if (target_flag  && !rotatingAwayFlag && !haltingFlag && !achieveDistanceFlag && !rotateAwayFlag)
    actual_velocities.min_tvel = MAX(0.0, actual_velocities.current_tvel - delta_tv);
  else
    actual_velocities.min_tvel = MAX(-max_tvel, actual_velocities.current_tvel - delta_tv);

  
  delta_rv = racc * ACCELERATION_ASSERTION_TIME;
  
  actual_velocities.max_rvel = MIN( max_rvel, actual_velocities.current_rvel + delta_rv);
  actual_velocities.min_rvel = MAX( -max_rvel, actual_velocities.current_rvel - delta_rv);

  if (actual_velocities.max_tvel < actual_velocities.min_tvel){
    if (target_flag) {
      if (dumpInfo)
	fprintf(dumpFile, "Error, must increase max_tvel from %f to %f\n",
		actual_velocities.max_tvel,actual_velocities.min_tvel);
    }
    actual_velocities.max_tvel = actual_velocities.min_tvel;
  }
  if (actual_velocities.max_rvel < actual_velocities.min_rvel){ 
    if (target_flag) {
      if (dumpInfo)
	fprintf(dumpFile, "Error, must increase max_rvel from %f to %f\n",
		actual_velocities.max_rvel,actual_velocities.min_rvel);
    }
    actual_velocities.max_rvel = actual_velocities.min_rvel;
  }
}





/**********************************************************************
 * The matrizes contain for each trajectory the rvel / tvel
 * combination for DIFFERENT_VELOCITIES different tvels.
 **********************************************************************/
void
generateVelocityCombinations( VelocityCombination*** combinations,
			      int iDim, int jDim)
{
    int i,j;
    float tvelStep, rvelStep;
    float minTvel, minRvel;
    float rvelIterations = 0.0;
    float previousRvel;
    
    int sign;
    BOOLEAN foundStraightTrajectory = FALSE;
    
    static int iDimension = 0;
    static int jDimension = 0;
    static VelocityCombination** combi = NULL;
    
    /* Let's check wether the dimensions have changed. */
    if ( iDimension != iDim || jDimension != jDim) {
      
	if ( combi != NULL)
	   free( combi);
	
	combi = (VelocityCombination **)
	    malloc( iDim * sizeof(VelocityCombination *));
	for (i = 0; i < iDim; i++) 
	    combi[i] = (VelocityCombination *)
		malloc( jDim * sizeof(VelocityCombination));

	iDimension = iDim;
	jDimension = jDim;
    }
    
    minTvel = actual_velocities.min_tvel;
    minRvel = actual_velocities.min_rvel;
    
    tvelStep = (actual_velocities.max_tvel - minTvel) /
	(float) (ACTUAL_MODE->number_of_tvels -1);
    rvelStep = (actual_velocities.max_rvel - minRvel) /
	(float) (ACTUAL_MODE->number_of_rvels -1);

    for ( i = 0, sign = SGN( minRvel), previousRvel = minRvel; i < iDimension; i++) {
      
      float rvel = minRvel + i * rvelStep;
      
      /* If the sign of the rotation changes we insert straight translation. */
      if ( ! foundStraightTrajectory && (SGN( rvel) != sign)) {
	foundStraightTrajectory = TRUE;
	
	/* Set the rotational velocity, which is closer to zero, to zero. */
	if ( fabs( rvel) < fabs( previousRvel)) 
	  rvel = 0.0;
	else
	  /* The previous velocity was smaller. */
	  /* This can't happen for i == 0. */
	  for ( j = 0; j < jDimension; j++)  {
	    combi[i-1][j].rvel = 0.0;
	  }
      }
      for ( j = 0; j < jDimension; j++)  {
	combi[i][j].rvel = rvel;
	combi[i][j].tvel = minTvel + j * tvelStep;
      }
    }
    
    /* Now set the values. */
    *combinations = combi;
}


/**********************************************************************
 * The next matrizes are used to store the evaluation for the rvels
 * and tvels described above.
 **********************************************************************/
void
allocateEvaluations( float*** velocities, float*** distances,
		     float*** angles, float*** values,
		     int iDim, int jDim)
{
   static int iDimension = 0;
   static int jDimension = 0;
   
   static float** vels = NULL;
   static float** dists = NULL;
   static float** angs = NULL;
   static float** vals = NULL;

   /* Let' check wether the dimensions have changed. */
   if ( iDimension != iDim || jDimension != jDim) {

      int i;

      /* Velocities. */
      if ( vels != NULL)
	 free( vels);
      vels = (float **) malloc( iDim * sizeof(float *));
      for (i = 0; i < iDim; i++) 
	 vels[i] = (float *) malloc( jDim * sizeof(float));
      
    
      /* Distances. */
      if ( dists != NULL)
	 free( dists);
      
      dists = (float **) malloc( iDim * sizeof(float *));
      for (i = 0; i < iDim; i++) 
	 dists[i] = (float *) malloc( jDim * sizeof(float));
      
      
      /* Angles. */
      if ( angs != NULL) 
	 free( angs);
      
      angs = (float **) malloc( iDim * sizeof(float *));
      for (i = 0; i < iDim; i++) 
	 angs[i] = (float *) malloc( jDim * sizeof(float));


      /* Values. */
      if ( vals != NULL)
	 free( vals);
      
      vals = (float **) malloc( iDim * sizeof(float *));
      for (i = 0; i < iDim; i++) 
	 vals[i] = (float *) malloc( jDim * sizeof(float));
      
      iDimension = iDim;
      jDimension = jDim;
   }
   
   *velocities = vels;
   *distances = dists;
   *angles = angs;
   *values = vals;
}

   

