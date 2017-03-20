
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/collisionIntern.h,v $
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
 * $Log: collisionIntern.h,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.19  1999/11/15 13:28:06  fox
 * *** empty log message ***
 *
 * Revision 1.18  1999/10/14 19:56:40  fox
 * IMPORTANT change: if the orientation of the robot is off more than 60 deg.
 * it will prefer rotation a lot.
 *
 * Revision 1.17  1999/01/05 22:37:33  fox
 * Some minor changes.
 *
 * Revision 1.16  1998/10/23 20:50:32  fox
 * *** empty log message ***
 *
 * Revision 1.15  1998/08/29 21:50:04  fox
 * Adapted exception handling in order to deal with people around the robot.
 *
 * Revision 1.14  1998/08/26 23:23:44  fox
 * Final version before I changed exception handling.
 *
 * Revision 1.13  1998/08/18 16:24:27  fox
 * Added support for b18 robot.
 *
 * Revision 1.12  1997/07/17 17:31:47  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.11  1997/06/17 09:39:31  fox
 * Changed rotate away.
 *
 * Revision 1.10  1997/05/14 08:10:57  fox
 * Fixed a bug when setting random mode.
 *
 * Revision 1.9  1997/05/06 18:03:31  fox
 * Infrareds and bumper should work. Take care of the INDEX!
 *
 * Revision 1.8  1997/05/06 17:05:04  fox
 * Nothing special.
 *
 * Revision 1.7  1997/04/26 13:56:18  fox
 * Added targetDefined, targetX, and targetY to status report.
 *
 * Revision 1.6  1997/04/10 12:08:23  fox
 * Incorporated support for the tactiles.
 *
 * Revision 1.5  1997/04/09 12:57:50  fox
 * Minor changes.
 *
 * Revision 1.4  1997/03/26 18:42:05  fox
 * Incorporated external obstacle information into the collision avoidance.
 *
 * Revision 1.3  1997/02/04 18:00:34  fox
 * Added paramters to BASE_setMode.
 *
 * Revision 1.2  1997/01/31 17:10:59  fox
 * New laser version.
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




#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <values.h>
#include "tcx.h"
#include "tcxP.h"
#include "Common.h"
#include "rwibase_interface.h"
#include "sonar_interface.h" 
/* #include "laser_interface.h"  */
#include "BASE-messages.h"
#include "SONAR-messages.h"
#include "LASER-messages.h"
#include "IR-messages.h"
#include "COLLI-messages.h"
#include "colli-handlers.h"
#include "base-handlers.h"
#include "collision.h"

#ifdef UNIBONN
#include "SUNVIS-messages.h"
#endif /* UNI-BONN */
#ifdef CLEANUP
#include "ARM-messages.h"
#endif


/* Variables shared by the different modules of the collision avoidance. */

#define MAX_NUMBER_OF_BUMPERS 100
#define NUMBER_OF_POINTS_PER_BUMPER 2

typedef struct tactileInfo {
  int numberOfActivatedTactiles;
  float angle[B_MAX_SENSOR_ROWS*B_MAX_SENSOR_COLS];
} tactileInfo;


#define MAX_NUMBER_OF_IRS B_MAX_SENSOR_COLS
typedef struct irInfo {
  int numberOfActivatedIrs;
  float value[B_MAX_SENSOR_COLS];
  float angle[B_MAX_SENSOR_COLS];
} irInfo;


/* Information about trajectories. */
typedef struct trajectory {
  float rvel;
  float tvel;
  float max_tvel;
  float min_tvel;
  BOOLEAN admissible;
} trajectory;

/* To store the actual velocities and the velocities reachabel within
 * the next time interval. */
typedef struct velocities {
  float current_tvel;
  float current_rvel;
  float min_tvel;
  float max_tvel;
  float min_rvel;
  float max_rvel;
  float min_ratio;
  float max_ratio;
} velocities;


/* The robot moves on middle_rad. The other two radiuses show the
 * collision area. */
typedef struct Circle_trajectory {
  float arm_rad;
  float big_rad;
  float small_rad;
  float middle_rad;
  float innerRobotRadius;
  float outerRobotRadius;
  Point M;
} Circle_trajectory;

#define UNDEFINED MAXFLOAT

/***************************************************************************
 * Different states of the arm. 
 ***************************************************************************/

extern int armState;

#define INSIDE 1
#define OUTSIDE 2
#define READY_TO_GRIP 3
#define OBJECT_LIFTED 4
#define OBJECT_DROPPED 5


/***************************************************************************
***************************************************************************
*    FILE:     collision.c
*
*    FUNCTION: variables of collision.c not needed by external modules.
***************************************************************************
***************************************************************************/

extern float MAX_RANGE;

extern float MIN_ROT_SPEED;
extern float MIN_TRANS_SPEED;

extern float MAX_CURVE_RADIUS;
extern float MIN_DIST;
extern float MAX_COLLISION_LINE_LENGTH;
extern float COLLISION_UPDATE_INTERVAL;
extern float ACCELERATION_ASSERTION_TIME;

/* These values influence the target trajectories */
extern float TARGET_TRANS_ACCELERATION;
extern float TARGET_ROT_ACCELERATION;
extern float TARGET_MAX_ROT_SPEED;
extern float TARGET_MAX_TRANS_SPEED;
extern float MIN_DIST_FOR_TARGET_WAY_FREE;
extern float MAX_TRANS_ANGLE;
extern float VELOCITY_FACTOR;
extern float ANGLE_FACTOR;
extern float DISTANCE_FACTOR;
extern float DOOR_ANGLE;
extern int SMOOTH_WIDTH;

/* These values determine the speed dependend security distances. */
extern float MAX_SECURITY_DIST;
extern float MIN_SECURITY_SPEED;
extern float MAX_SECURITY_SPEED;

/* These values determine our new Armcollision */
extern int STACK_SIZE;
extern float SECURITY_ANGLE;
extern float ROLLBACK_DISTANCE;
extern float ROLLBACK_MIN_TRANSLATION;
extern float ROLLBACK_MIN_ROTATION;
extern int   ROTATE_AWAY_PERSISTENCE;

/* How many different velocities do we try for each trajectory in 
 * compute_target_trajectory()? */
#define DIFFERENT_VELOCITIES 20

extern int NUMBER_OF_RVELS;
extern int NUMBER_OF_TVELS;

void
update_CollisionStatus(Pointer callback_data, Pointer client_data);

float
angleEvaluation( Point rpos, float rrot,
		float tvel, float rvel);

float
distanceEvaluation( float tvel, float rvel,
		   float collDist, float targetDist);

float
velocityEvaluation( float tvel);

float
evaluationFunction( float* velocity,
		    float* angle,
		    float* distance,
		    BOOLEAN targetWayFree,
		    BOOLEAN rotateToTargetPoint);

BOOLEAN
admissible( Point rpos, float rrot,
	    float tvel, float rvel,
	    int onlyTranslation, int onlyRotation,
	    float *collDist,
	    float* targetDist);

/***************************************************************************
***************************************************************************
*    FILE:     colliTcx.c
*
*    FUNCTION: functions for sending information to COLLGRAPH
***************************************************************************
***************************************************************************/

/* Allocates memory and initializes values. */
void
initTcxStructures();

/* Updates the values in colli_tcx_status. These values are shown in the 
 * trajectory of the robot in the collision graphics. */
void
COLLI_update_tcx_trajectory(Point rpos, float rrot,
			    float tvel, float rvel,
			    float dist);

/* Updates the values in colli_tcx_status. These lines are shown in the 
 * collision graphics in red. */
void
COLLI_update_tcx_CollLines(void);

/* Updates the values in colli_tcx_status. These points are shown in the 
 * collision graphics in green. */
void
COLLI_update_tcx_LaserPoints(void);

/* Updates the values in colli_tcx_status. These points are shown in the 
 * collision graphics in green. */
void
COLLI_update_tcx_IrPoints(void);

/* Updates the values in colli_tcx_status. These points are shown in the 
 * collision graphics in blue. */
void
COLLI_update_tcx_ExternalObstaclePoints(void);

void 
pantiltTrackPoint( float robotX, float robotY, 
		   float x, float y);

void 
stopTargetTracking();

/***************************************************************************
***************************************************************************
*    FILE:     colliBase.c
*
*    FUNCTION: functions to get and set velocities of the base
***************************************************************************
***************************************************************************/

extern velocities actual_velocities;

/*  The actual values are set. The rotation is set such that the
 * commonn mathematical functions can be applied to it.
 * At the same time the velocities are updated and stored in
 * the global structure "actual_velocities". */
#define CONSIDER_DIRECTION 1
#define DONT_CONSIDER_DIRECTION 0
void
updateActualPosition( Point* rpos, float* rrot,
		      BOOLEAN considerDirection);

void
updateActualConfiguration( Point* rpos, float* rrot);

/* If the robot should move backward we have to swap the velocities. */
void
compute_current_velocities(void);

/* Compute maximal allowed velocities.
 * The maximal velocity is set in the desired_trajectory. If the coll_dist
 * is too small to brake desired_trajectory.admissible is set to FALSE. */
void
setVelocities(Point rpos, float coll_dist, trajectory desired_trajectory);

/* We have to change the direction of the translational movement
 * according to colli_go_backward. */
void
setDirections( trajectory desiredTrajectory);


/***************************************************************************
***************************************************************************
*    FILE:     colliSonar.c
*
*    FUNCTION: functions to integrate information from the sonar sensors
***************************************************************************
***************************************************************************/

extern float *SonarAngle;
extern int *considerSonar;
extern float SONAR_UPDATE_TIME;

void
init_SonarStructs(void);

/* Given the position of the robot and the distance of a particular 
 * sonar the collision line representing this reading is given back. */
LineSeg
sonar_to_lineseg(Point rpos, float rrot, 
		 int sonar_no, float dist);

void 
dontConsiderSonar(int number);

void 
reconsiderSonar(int number);

/* In the case that the robot has to move backward we just swap the
 * sonar information such that the rear sonars seem to be at the front
 * of the robot (the base then only has to change the sign of the
 * velocities. */
void
update_CollLines(Pointer callback_data, Pointer client_data);


/***************************************************************************
***************************************************************************
*    FILE:     colliLaser.c
*
*    FUNCTION: functions to integrate information from the laser scanners
***************************************************************************
***************************************************************************/

extern float LASER_UPDATE_TIME;

void
initLaserPointsStructure();

void
checkLaserPing( struct timeval* now);

void
update_LaserPoints( Pointer callback_data, Pointer client_data);


/***************************************************************************
***************************************************************************
*    FILE:     colliTrajectories.c
*
*    FUNCTION: dealing with trajectories and collision distances
***************************************************************************
***************************************************************************/



Circle_trajectory
velocities_to_circle_trajectory(Point rpos, float rrot, float tvel,
				float rvel, float size, float addsize,float armsize);

void
position_to_lines(Point rpos, float rrot, float tvel, float size,
		  LineSeg *lline, LineSeg *rline);

/*   Computes the time until the robot reaches the closest obstacle   *
 *   which is in the tajectory area given through the velocities and  *
 *   the size of the area.                                            *
 *   If the target_flag is true the distance to the target point is   *
 *   also computed and set in *target_dist.
 *   This function uses all different sonar, vision lines and points.
 *   Instead of using lines we only consider three points of a line. */
float
computeCollisionDistance(Point rpos, float rrot, float tvel, float rvel,
			      float size, float addsize,float armsize,
			      float *target_dist);


/* Check for the current situation whether only translation or only
 * rotation is allowed. */
void
checkWhetherOnlyRotationOrOnlyTranslation(  Point rpos, float rrot,
					   int* onlyTranslation,
					   int* onlyRotation);

/* Computes the distance to the next obstacle on the direct way to the target. */
float
collDistInTargetDirection( Point rpos, float tvel,
			   Point target, float radius);

/* Computes the angle to the target point for a given trajectory.
 * If the robot passes the target angle within the time the 
 * returned angle is negative. */
float
lookaheadTargetAngle(Point target, Point rpos, float rrot, 
		     float tvel, float rvel,
		     float tacc, float racc,
		     float time);
    
/* Given the minimal and maximal velocities we compute the extreme traj
 * and store them as angles in the rvel - tvel space. */
void
compute_possible_trajectories(void);

/* Given the values for the desired velocities the function returns a trajectory
 * with the minimal and maximal possible velocities on this traj with respect to
 * the actual velocities. If the traj is not possible the function returns a 
 * traj with the closest velocities. */
trajectory
closest_possible_trajectory( float tvel, float rvel);


/***************************************************************************
***************************************************************************
*    FILE:     colliTools.c
*
*    FUNCTION: some help functions
***************************************************************************
***************************************************************************/

/* Used for the check of the actuality of the data. */
extern struct timeval Last_CollLine_Update;
extern struct timeval Last_ExternalObstaclePoint_Update;
extern struct timeval Last_FrontLaserPoint_Update;
extern struct timeval Last_RearLaserPoint_Update;

void
COLLI_init_Lines(void);

/* We set some default values and allocate memory for global variables */
void
init_collision_avoidance();

#define DEFAULT_INITIALIZATION_FILE "colli_modes.ini"

void
COLLI_start_collision_avoidance( char* iniFile);

/* Sets the pointer for the collision lines to the oldest lines. */
void
Inc_next_reading(Pointer callback_data, Pointer client_data);


/* Checks whether the collision information (sonar lines, vision lines and
 * vision points) are up to date and whether the 
 * time since last collision avoidance is not too big. */
BOOLEAN
update_check_ok(void);

/* This routine is called to check whether the robot is still in rotation. */
BOOLEAN 
stillInRotation();

 /* This routine is called to check whether the robot is still in rotation. */
BOOLEAN 
stillInTranslation();

/* The next two functions are used to check whether a set time interval
 * has expired. */
void
setTimer( int secs);
BOOLEAN
stillInTimeInterval();

/*  Tests whether a point is behind the robot. */
BOOLEAN
behindPoint( Point rpos, float rrot, Point pt);

float
compute_robot_angle_to_point(Point rpos, float rrot, Point target);

/* Computes the maximal velocity such that the robot can stop within the 
 * distance. 
 * The formula is the solution to:
 * coll_dist = -0.5 * a * SQR(t) + tvel * t      and
 * tvel = -a * t */
float
maxVelForEmergencyStop(float distance, float acceleration);

/* Computes the distance covered by the robot for a given velocity
 * and acceleration until it stops. */
float
brakeDistance( float velocity, float acceleration);

/* Returns the time needed to stop. */
float
brakeTime( float velocity, float acceleration);
 
/* Checks whether direct way to the target is free. */
BOOLEAN
checkForTargetWayFree( Point rpos, float rrot, float tvel,
		       float radius, Point target);

/* The security dist should depend on the velocity.  */
float
speed_dependent_security_dist(float tvel);



/* The maximal velocity is stored in traj->max_tvel if traj->tvel positive
 * (otherwise in traj->min_tvel). */
void
compute_max_velocity(float coll_dist, trajectory *traj);

/* Given the actual velocities and accelerations this function computes 
 * the possible velocities within the next COLLISION_UPDATE_INTERVAL
 * seconds. The values are stored in the structure actual_velocities. */
void
compute_possible_velocities( float tacc, float racc);

/* The histogram containing the trajectories is generated. These
 * trajectories will be evaluated within the next sections of
 * compute_target_trajectory(). */
trajectory*
generatedTrajectories();

void
generateVelocityCombinations( VelocityCombination*** combinations,
			      int iDim, int jDim);

void
allocateEvaluations( float*** velocities, float*** distances,
		     float*** angles, float*** values,
		     int iDim, int jDim);


/* Just calls computeCollisionDistance(). */
float
collisionDistance( Point rpos,
		   float rrot,
		   float tvel,
		   float rvel,
		   float* targetDist);

/***************************************************************************
***************************************************************************
*    FILE:     mathTools.c
*
*    FUNCTION: for geometrical computations
***************************************************************************
***************************************************************************/

#define EPSILON 0.00001


float 
fsin( float x);
float 
fcos( float x);
void 
init_fast_cos(void);
void 
init_fast_sin(void);

float 
fsqrt( float x);

float 
fatan2( float x, float y);

/* The next three functions can be called to check for time intervals.
 * The index is used to differentiate between the intervals. It must be
 * less than MAX_NUMBER_OF_TIMERS-1! (see mathTools.c) */
#define MAX_NUMBER_OF_TIMERS 10

/* the following numbers are already in use:
   0: FRONT_LASER
   1: READ_LASER
   */
#define IR_TIMER 7
#define BUMPER_TIMER 8
#define EVERYBODIES_TIMER 9

void
setStartTime( int i);
float
timeExpired( int i);
void
printExpiredTime( int i, BOOLEAN longFormat);

float
timeDiff( struct timeval* t1, struct timeval* t2);

/*   float-Normierung: b = norm (a,...)                                      */
float 
fnorm( float a, float amin, float amax, float bmin, float bmax);

int 
casted_float(float f);

/* Norms the angle into [0,2 PI]. */
float 
normed_angle(float angle);

void 
norm_angle(float *angle);

BOOLEAN 
smaller(Point pt1, Point pt2);

BOOLEAN 
smallerOrEqual(Point pt1, Point pt2);

/* Swaps the two points of the line. */
void 
swap(LineSeg *line);

/* Given a one dimensional histogram the function smoothes the values.
 * SMOOTH_WIDTH gives the distance of influence of the different values
 * on each other. */
void 
smoothHistogram(float* hist, int size, int smoothWidth);

/* Given a two dimensional grid the function smoothes the values.
 * SMOOTH_WIDTH gives the distance of influence of the different values
 * on each other. */
void
smoothGrid( float** grid, int iDim, int jDim, int smoothWidth);

float 
max_norm_distance(Point pt1, Point pt2);

float 
compute_distance(Point pt1, Point pt2);

/*   Computes the angle from pt1 to pt2. */
float 
compute_angle_2p(Point pt1, Point pt2);

/*   Computes the angle from pt1 to pt2 (counterclockwise) given the  *
 *   midpoint M of the circle. The angle is in [0.0 : 2 pi]           */
float 
compute_angle_3p(Point M, Point pt1, Point pt2);

/*  Computes a circle given two points and the angle of the tangent      *
 * through the first point .                                             *
 * If the solution is a line we return a circle with midpoint.x = F_ERROR.*/
Circle
compute_circle(Point pt1, float pt1_angle, Point pt2);

/*  If the lines are parallel the coordinates of the point are F_ERROR */
Point 
cut_line_and_line(LineSeg line1, LineSeg line2);

/*  If the two points of the line are too close to each other, the    *
 *  function returns -1. */
int
cut_circle_and_line(Circle circle, LineSeg line, Point S[2]);

/* Reutrns number of intersection points. */
int 
cut_circle_and_circle(Circle *c1, Circle *c2, Point *pt);

/* Tests whether a point on a line specified by two points is in between
 * the two points. This works only if the point is on the line!!!! */
BOOLEAN 
point_on_LineSeg(Point pt, LineSeg line);

int get_collpoints_circle(Circle circle_big,Circle circle_small,LineSeg line,Point coll_pt[]);

#define BETWEEN_POINTS 0
#define CLOSER_TO_FIRST_POINT 1
#define CLOSER_TO_SECOND_POINT 2
float 
minDistanceToLine(Point pt, LineSeg line,
		  Point* meetPoint, int* relativePosition);

/***************************************************************************
***************************************************************************
*    FILE:     colliFiles.c
*
*    FUNCTION: input/output  from/to files
***************************************************************************
***************************************************************************/

/* Do we want to dump information? */
extern BOOLEAN dumpInfo;

/* If this function is called relevant values will be written to
 * stderr. This is just for debugging. */
void
COLLI_DumpInfo();

 /* Sets a mark in the dumpfile. */
void
COLLI_MarkInfo();

/* Loads parameters for the different modes from a file.
 * The default values are replaced by these values. */
BOOLEAN
load_parameters(char *filename);

/* Writes the status of some variables in the dumpfile. */
void
outputCollisionStatus();

/* Writes time spent sind begint in the dumpfile. */
void
outputTimeInfo( struct timeval* begint);


/***************************************************************************
***************************************************************************
*    FILE:     colliModes.c
*
*    FUNCTION: sets the different modes.
***************************************************************************
***************************************************************************/

/* The strcuctures determine the behaviour in the
 * different modes.
 */
extern mode_structure **mode_structure_array;
extern int mode_number;

/* If this flag is set the collision avoidance moves backward. */
extern BOOLEAN colli_go_backward;

/* Sets the direction to move to the next target point. */
void
COLLI_GoForward();

/* Sets the direction to move to the next target point. */
void
COLLI_GoBackward();

/* Sets the actual mode and the corresponding values. */
void
COLLI_SetMode( int mode);

/* Sets all modes to the default value. */
void
setAllModesToDefault();

/* Copies the value of the default mode into all other modes. To do this
 * the DEFAULT_MODE must be allocated. Memory for the other modes will
 * be allocated in this function. */
void
copyDefaultMode();

/* With this function the values of the actual mode can be changed. */
void
COLLI_get_parameters(COLLI_parameter_ptr parameters);

mode_structure*
default_mode_structure(void);

    
/***************************************************************************
***************************************************************************
*    FILE:     colliTargets.c
*
*    FUNCTION: interface to set target points.
***************************************************************************
***************************************************************************/

extern float targetArriveSpeed;
extern float approachDist;
extern BOOLEAN newTargetPoint;
extern Point nextTarget;

/* Sets the approach distance used for the next call of
 * COLLI_TranslateRelative(). */
void
COLLI_SetApproachDist( double dist);

/* Set the next target point. */
void
COLLI_TranslateRelative(double forward, double side);

/* Set the next target point. */
void
COLLI_TranslateRelative_TwoPoints(double forward1, double side1,
				       double forward2, double side2);

void
COLLI_GotoAbsolute( float x, float y, BOOLEAN new_target);

void
COLLI_GotoAbsoluteWithDist( float x, float y, float dist, BOOLEAN new_target);

     /* Sets the target point on x, y. The mode determines the behaviour of 
 * the robot on it's way to this point. If the distance to the target
 * is less than dist the robot stops and sets target_reached to TRUE. */
void
COLLI_ApproachAbsolute( float x, float y, float dist, BOOLEAN new_target, int mode);
    
/* Same as COLLI_ApproachAbsolute but when the first point is reached the
 * robot tries to get to the next point. */
void
COLLI_ApproachAbsolute_TwoPoints( float x1, float y1, float x2, float y2,
				  float dist, BOOLEAN new_target ,
				  TCX_REF_PTR TCX_sender);

/* If <new_target_point> is TRUE the robot will try to rotate to
 * the target until the conditions are not met any longer. */
BOOLEAN
checkIfRotateToTargetPoint( float targetRot);

/* What to do when the robot has reached a target depends on the
 * current mode.
 * The function returns TRUE if this cycle of the collision avoidance
 * should be ended.
 * If we use a new target we adapt <targetDist> and <targetRot>. */
BOOLEAN
handleArrivalAtTarget( Point rpos,
		      float rrot,
		      float* targetDist,
		      float* targetRot);

/***************************************************************************
***************************************************************************
*    FILE:     colliExceptions.c
*
*    FUNCTION: exceptional modes of the col.av. if obstacles are too close.
***************************************************************************
***************************************************************************/

/* These values determine the maximum velocities during 
   exceptions (rotate_away and achieve_distance).
   They are allowed to be above the MAX_... values. */
extern float EXCEPTION_TRANS_ACCELERATION;
extern float EXCEPTION_TRANS_VELOCITY;
extern float EXCEPTION_ROT_ACCELERATION;
extern float EXCEPTION_ROT_VELOCITY;

extern BOOLEAN emergencyStop;
extern BOOLEAN achieveDistanceFlag;
extern BOOLEAN rotateAwayFlag;
extern BOOLEAN robotStuck;

/* In the random mode we prefer straight motion. */
BOOLEAN straightMotionIsPossible( Point rpos, float rrot);

/* Starts special modes to deal with exceptions. */
void
startExceptionHandling( Point rpos,
			float rrot,
			VelocityCombination bestCombination,
			int numberOfExceptions);

/* Exception handling has started and the rotation or translation
 * has to be finished. */
BOOLEAN
keepOnWithExceptionHandling( Point rpos,
			     float rrot,
			     int numberOfExceptions);


/***************************************************************************
***************************************************************************
*    FILE:     colliArmException.c
*
*    FUNCTION: handles exceptions with arm outside
***************************************************************************
***************************************************************************/
extern BOOLEAN rotatingAwayFlag;
extern BOOLEAN haltingFlag;
extern BOOLEAN rollingBackFlag;
extern int rotateAwayNumber;


/* Starts special modes to deal with exceptions. */
void
startHalting( Point rpos,
	     float rrot);
	     

/* Exception handling has started and the rotation or translation
 * has to be finished. */
void
exceptionHandling( Point rpos,
		  float rrot);

void startRollingBack( float distance );


/***************************************************************************
***************************************************************************
*    FILE:     colliArm.c
*
*    FUNCTION: Helpfunctions for colliTrajectories.c
***************************************************************************
***************************************************************************/

#define ARMLENGTH 48.08
#define ADDANGLE 0.4386671


Point computeInnerArmPoint(Point rpos,float rrot,float rvel,float tvel);
Point computeOuterArmPoint(Point rpos,float rrot,float rvel,float tvel);

/* If the arm is outside then the security dist depends on the radius of
 * the trajectory. */
float
computeArmOffset( float tvel, float rvel);


/***************************************************************************
****************************************************************************
*     FILE:     colliStack.c
*
*     FUNCTION: 
****************************************************************************
***************************************************************************/

struct itemType
{
   Point pos;
   float rot;
};

struct node
{
    struct itemType key;
    struct node *prev;
    struct node *next;
};

extern struct node *head;
extern struct node *bottom;
extern int actualStackSize;

void initStack();
void push(struct itemType v);
struct itemType pop();
int isempty();
void deleteBottom();
void printStack();



/***************************************************************************
****************************************************************************
*     FILE:     colliBumper.c
*
*     FUNCTION: 
****************************************************************************
***************************************************************************/

void
updateBumperTimer();

/***************************************************************************
****************************************************************************
*     FILE:     colliIr.c
*
*     FUNCTION: 
****************************************************************************
***************************************************************************/

void
COLLI_IrHandler( irInfo* irs);

/***************************************************************************
****************************************************************************
*     FILE:     colliLaser.c
*
*     FUNCTION: 
****************************************************************************
***************************************************************************/

#define MAX_NUMBER_OF_LASER_POINTS 500

extern BOOLEAN newStatusReport;

extern int foxDebug;


/***************************************************************************
****************************************************************************
*     FILE:     rectangularRobot.c
*
*     FUNCTION: 
****************************************************************************
***************************************************************************/

void
setRectangularRobot( int robotType);
