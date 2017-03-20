
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/COLLI-messages.h,v $
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
 * $Log: COLLI-messages.h,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.9  1998/08/18 16:24:20  fox
 * Added support for b18 robot.
 *
 * Revision 1.8  1998/05/13 07:07:39  fox
 * Fixed some bugs I found due to graphical output.
 *
 * Revision 1.7  1997/05/06 17:15:36  fox
 * Fixed a bug.
 *
 * Revision 1.6  1997/05/06 17:05:03  fox
 * Nothing special.
 *
 * Revision 1.5  1997/05/06 15:41:34  fox
 * Incorporated irs.
 *
 * Revision 1.4  1997/04/10 12:08:20  fox
 * Incorporated support for the tactiles.
 *
 * Revision 1.3  1997/03/26 18:42:01  fox
 * Incorporated external obstacle information into the collision avoidance.
 *
 * Revision 1.2  1997/01/07 13:47:12  fox
 * Improved rotate_away.
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



#ifndef COLLI_messages_defined
#define COLLI_messages_defined



#include "tcx.h"
/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_COLLI_MODES

/* Different modes for the collision avoidance. */
#define DEFAULT_MODE 0        
#define FAST_TRAVEL_MODE 1
#define FIND_DOOR_MODE 2    
#define SERVO_MODE 3
#define ARM_OUT_MODE 4

/* The next modes are necessary for demonstrations. */
#define RANDOM_MODE 5
#define APPROACH_OBJECT_MODE 6
#define APPROACH_TRASH_BIN_MODE 7
#define ARM_OUT_RANDOM_MODE 8
#define NUMBER_OF_MODES 9

#endif /*DEFINE_COLLI_MODES */

/**** TCX module name - this is a unique identifier for TCX *******/

#define TCX_COLLI_MODULE_NAME "BASE"


#ifdef TCX_define_variables		/* do this exactly once! */

TCX_MODULE_PTR COLLI;	/* needs to be allocate in a user program */

#else

extern TCX_MODULE_PTR COLLI;	/* otherwise: reference */

#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** COLLI data types (for the commands listed below)
 **** Make sure the struct defs and the format defs are consistent! ****/


typedef struct iPoint {
  int x;
  int y;
} iPoint;


typedef struct iLine {
  iPoint pt1;
  iPoint pt2;
} iLine;

typedef struct iCircle {
  iPoint M;
  int rad;
} iCircle;

typedef struct COLLI_colli_reply_type {

  iPoint rpos;		/* Robot position */
  int rrot;			/* Rotation */
  int tvel;
  int rvel;
  int dist;
  
  iPoint targetpoint;		/* The target */

  /* Two points of the arm. */
  iPoint innerArmPoint;
  iPoint outerArmPoint;		

  /* The trajectories */
  iLine leftLine;
  iLine rightLine;

  iCircle innerCircle;
  iCircle outerCircle;
  iCircle armCircle;

  /* The collision line field. */
  int rememberInterval;	
  iLine sonar_lines[25];

  /* For usage of external information. */
  int no_of_external_points;
  iPoint *external_points;

  /* The laser points. */
  int no_of_laser_points;
  iPoint *laser_points;

  /* The bumper and ir points. */
  int no_of_ir_points;
  iPoint *ir_points;

  int no_of_bumper_points;
  iPoint *bumper_points;

  int rectangularRobot;
  iLine robotLines[4];
  iLine testLines[4];
} COLLI_colli_reply_type, *COLLI_colli_reply_ptr;


typedef struct COLLI_vision_line_type {
  int no_of_lines;
  iLine *lines;
} COLLI_vision_line_type, *COLLI_vision_line_ptr;

typedef struct COLLI_vision_point_type {
  int no_of_points;
  iPoint *points;
} COLLI_vision_point_type, *COLLI_vision_point_ptr;


typedef struct COLLI_parameter_type {
  float velocity_factor;
  float angle_factor;
  float distance_factor;
  float target_max_trans_speed;
  float target_max_rot_speed;
  float target_trans_acceleration;
  float target_rot_acceleration;
} COLLI_parameter_type, *COLLI_parameter_ptr;



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/



#define COLLI_vision_line_format "{int, <{int,int,int,int}:1>}"

#define COLLI_vision_point_format "{int, <{int,int}:1>}"

#define COLLI_parameter_format "{float, float, float, float, float, float, float}"

/* #define COLLI_colli_reply_format "{{int,int}, int, int, int, int, {int,int}, {int,int}, {int,int}, {int,int,int,int}, {int,int,int,int}, {int,int,int}, {int,int,int}, {int,int,int}, int, [{int,int,int,int} : 24], int, <{int,int}:16>, int, <{int,int}:18>, int, <{int,int}:20>, int, <{int,int}:22>}"  */

#define COLLI_colli_reply_format "{{int,int}, int, int, int, int, {int,int}, {int,int}, {int,int}, {int,int,int,int}, {int,int,int,int}, {int,int,int}, {int,int,int}, {int,int,int}, int, [{int,int,int,int} : 25], int, <{int,int}:16>, int, <{int,int}:18>, int, <{int,int}:20>, int, <{int,int}:22>, int, [{int,int,int,int} : 4], [{int,int,int,int} : 4]}"

#define COLLI_disconnect_format NULL

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


#ifdef TCX_define_variables		/* do this exactly once! */

#define COLLI_messages \
  {"COLLI_colli_reply",         COLLI_colli_reply_format},\
  {"COLLI_vision_line",         COLLI_vision_line_format},\
  {"COLLI_vision_point",        COLLI_vision_point_format},\
  {"COLLI_parameter",           COLLI_parameter_format},\
  {"COLLI_disconnect",          COLLI_disconnect_format}
#endif


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#ifdef DEFINE_REPLY_HANDLERS


/***** reply handlers -- these handlers must be defined within the
 ***** program that communicates with COLLI ******/

/******* (a) Procedure headers ******/

void COLLI_colli_reply_handler(TCX_REF_PTR                ref,
			       COLLI_colli_reply_ptr      data);

/******* (b) Handler array ******/

TCX_REG_HND_TYPE COLLI_reply_handler_array[] = {
  {"COLLI_colli_reply", "COLLI_colli_reply_handler",
     COLLI_colli_reply_handler, TCX_RECV_ALL, NULL}
};


#endif









#endif
