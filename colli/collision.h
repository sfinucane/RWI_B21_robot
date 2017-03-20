
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/collision.h,v $
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
 * $Log: collision.h,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.25  1999/10/02 09:06:42  thrun
 * New robot type: XR4000.
 *
 * Revision 1.24  1999/09/08 21:11:51  fox
 * *** empty log message ***
 *
 * Revision 1.23  1999/06/25 19:48:09  fox
 * Minor changs for the urbie.
 *
 * Revision 1.22  1999/03/09 15:48:42  wolfram
 * Added geometry of PIONEER_II
 *
 * Revision 1.21  1999/02/23 09:39:16  schulz
 * Added support for the Pioneer II robot. Use option -pioneer2
 *
 * Revision 1.20  1999/01/05 22:37:32  fox
 * Some minor changes.
 *
 * Revision 1.19  1998/10/30 18:20:51  fox
 * Added support for pioniers.
 *
 * Revision 1.18  1998/10/23 20:50:32  fox
 * *** empty log message ***
 *
 * Revision 1.17  1998/08/18 16:24:27  fox
 * Added support for b18 robot.
 *
 * Revision 1.16  1998/01/14 00:37:25  thrun
 * New option "-laserserver" lets the base receive laser date from
 * the laserServer. This is the recommended option. It seems to be
 * much more reliable than reading in the data directly.
 *
 * Revision 1.15  1997/11/12 17:07:30  fox
 * Removed some old arm stuff.
 *
 * Revision 1.14  1997/07/17 17:31:47  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.13  1997/05/06 18:03:31  fox
 * Infrareds and bumper should work. Take care of the INDEX!
 *
 * Revision 1.12  1997/05/06 17:05:04  fox
 * Nothing special.
 *
 * Revision 1.11  1997/04/26 13:56:18  fox
 * Added targetDefined, targetX, and targetY to status report.
 *
 * Revision 1.10  1997/04/10 12:08:23  fox
 * Incorporated support for the tactiles.
 *
 * Revision 1.9  1997/03/26 18:42:04  fox
 * Incorporated external obstacle information into the collision avoidance.
 *
 * Revision 1.8  1997/03/25 21:44:42  tyson
 * Many bug fixes.
 *
 * Revision 1.7  1997/03/14 17:48:59  fox
 * Changed laser messages.
 *
 * Revision 1.6  1997/03/11 18:56:33  tyson
 * minor compile cleanups
 *
 * Revision 1.5  1997/02/04 18:00:33  fox
 * Added paramters to BASE_setMode.
 *
 * Revision 1.4  1997/02/02 22:32:29  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.3  1997/01/07 13:47:15  fox
 * Improved rotate_away.
 *
 * Revision 1.2  1996/12/23 15:50:36  fox
 * Added minor support for the bumpers. COLLI stops whenever a bumper has
 * triggered a signal.
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



#ifndef COLLISION_LOADED
#define COLLISION_LOADED

#include "bUtils.h"
#include "COLLI-messages.h"

#define F_ERROR MAXFLOAT
#define I_ERROR (MAXINT)

#define ROB_RADIUS (bRobot.base_radius) 

#define COLLI_SONAR_RADIUS 2.48

#define SONAR_OPEN_ANGLE (DEG_TO_RAD(7.5))

#define MAX_TRANS_SPEED 90.0
#define MAX_ROT_SPEED (DEG_TO_RAD(60.0))
#define NO_OF_SONARS bRobot.sonar_cols[0]

#define DEG_90 (1.5707963705)
#define DEG_180 (3.1415927410)
#define DEG_270 (4.7123891115)
#define DEG_360 (6.2831854820)

#define NUMBER_OF_B18_SONARS 25
#define NUMBER_OF_URBAN_SONARS 10
#define NUMBER_OF_PIONEER_AT_SONARS 7
#define NUMBER_OF_PIONEER_II_SONARS 16
      

#define SGN(x)    ((x) >= 0.0 ? (1) : (-1))

/**********************************************************************/

typedef struct Point {
  float x;
  float y;
} Point;

typedef struct VelocityCombination {
  float tvel;
  float rvel;
} VelocityCombination;

typedef struct Circle {
  float rad;
  Point M;
} Circle;

typedef struct LineSeg {
  Point pt1;
  Point pt2;
} LineSeg;

typedef struct ObstacleLines {
  int no_of_lines;
  LineSeg *lines;
} ObstacleLines;

typedef struct ObstaclePoints {
  int no_of_points;
  Point *points;
} ObstaclePoints;


typedef struct LaserPoints {
  int maxNumberOfPoints;
  int numberOfPoints;
  float maxRememberTime;
  Point* points;
  struct timeval* times;
} LaserPoints;


typedef struct mode_structure {
    float velocity_factor;
    float angle_factor;
    float distance_factor;
    
    int number_of_rvels;
    int number_of_tvels;
    
    float target_trans_acceleration;
    float target_rot_acceleration;
    float target_max_rot_speed;
    float target_max_trans_speed;

    float exception_trans_acceleration;
    float exception_trans_velocity;
    float exception_rot_acceleration;
    float exception_rot_velocity;

    float min_dist;
    int smooth_width;
    float security_dist;
    float min_dist_for_target_way_free;
    float max_collision_line_length;
    float max_range;

    float edge_portion;
    float max_security_speed;
    float min_security_speed;
    float max_security_dist;

    /* Parameter for exception handling with arm outisde. */
    
    float security_angle;
    int   rotate_away_persistence;
 } mode_structure;


void COLLI_DumpInfo();
void COLLI_MarkInfo();
void COLLI_init_Lines(void);
void COLLI_start_collision_avoidance();
void COLLI_update_tcx_status(LineSeg, LineSeg, Circle, BOOLEAN, Point, float, int*, int*);
void COLLI_SetApproachDist( double dist);
void COLLI_StartExploration();
void COLLI_PickupObject( double forward, double side);
void COLLI_TranslateRelative(double xdir, double ydir);
void COLLI_TranslateRelative_TwoPoints(double x1, double y1, double x2, double y2);
void COLLI_GotoAbsolute(float x, float y, BOOLEAN new_target );
void COLLI_GotoAbsoluteWithDist( float x, float y, float dist, BOOLEAN new_target);
void COLLI_ApproachAbsolute(float, float, float, BOOLEAN, int );
void COLLI_ApproachAbsolute_TwoPoints(float, float, float,
				      float, float, BOOLEAN, TCX_REF_PTR);
void COLLI_get_parameters(COLLI_parameter_ptr);
void COLLI_set_wall_orientation(float orient);
void COLLI_GoForward();
void COLLI_GoBackward();
void COLLI_BumpHandler( Pointer callback_data, Pointer client_data);

/* These two functions start to write the evaluation function into
 * files. */
void COLLI_StartToDumpGnuPlot( double step);
void COLLI_StartToDumpMathematica( double step);

#ifdef UNIBONN

void
COLLI_get_CollPoints_from_vision(COLLI_vision_point_ptr);

void
COLLI_get_CollLines_from_vision(COLLI_vision_line_ptr vision_lines);

#endif

/* For communication with the arm. These functions send commands to the arm. */

#ifdef CLEANUP
void ARM_moveToGround(void);
void ARM_liftObject(void);
void ARM_dropObject(void);
void ARM_moveIn(void);

/* These functions get information about the successfull executions of commands. */

void ARM_moveToGroundReady(int success);
void ARM_liftObjectReady(int success);
void ARM_dropObjectReady(int success);
void ARM_moveInReady(int success);


/* Just for testing. The next functions simulate the vision to send objects. */
void
COLLI_SimulateCloseObject( double forward, double side);

void
COLLI_SimulateFarObject( double forward, double side);

void
COLLI_SimulateCloseTrashBin( double forward, double side);

void
COLLI_SimulateFarTrashBin( double forward, double side);

#endif

/*****************************************************************************
 * External variables
 *****************************************************************************/

extern BOOLEAN use_sonar;
extern BOOLEAN use_laser;
extern BOOLEAN use_laser_server;
extern BOOLEAN msp_sonar;

extern BOOLEAN use_arm;
extern BOOLEAN use_bumper;
extern BOOLEAN use_ir;
extern BOOLEAN use_vision;
extern BOOLEAN use_collision;

#define B21_ROBOT    0
#define B18_ROBOT    1
#define PIONEER_ATRV 2
#define PIONEER_II   3
#define URBAN_ROBOT  4
#define B14_ROBOT    5
#define SCOUT_ROBOT  6
#define XR4000_ROBOT 7

#define ROUND_ROBOT 0
#define RECTANGULAR_ROBOT 1

extern int robotType;
extern int robotShape;

extern BOOLEAN colli_go_backward;

/* This is used to write information relevant for collision
 * avoidance into a dump file.
 */
extern BOOLEAN dumpInfo;
extern FILE* dumpFile;

extern int mode_number;
extern int REMEMBER_INTERVAL;
extern float SECURITY_DIST;
extern float EDGE_PORTION;
extern LineSeg **CollLines;
extern int next_CollLine_reading;
extern float *histogram;
extern BOOLEAN target_flag;
extern BOOLEAN rot_wanted;
extern Point target;
extern mode_structure *ACTUAL_MODE;
extern mode_structure **mode_structure_array;

/* New struct containing points for all kinds of sensors. */
#define NUMBER_OF_SENSORS 5
#define SONAR_POINTS 0
#define LASER_POINTS 1
#define IR_POINTS 2
#define BUMPER_POINTS 3
#define EXTERNAL_POINTS 4
#define POINTS_PER_SONAR 2

extern ObstaclePoints combinedObstaclePoints[NUMBER_OF_SENSORS];
extern BOOLEAN useSensorPoints[NUMBER_OF_SENSORS];

#endif /* COLLISION_LOADED */



