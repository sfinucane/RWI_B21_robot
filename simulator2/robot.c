/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File:                        robot.c
 *****
 ***** Part of:                     RHINO SOFTWARE
 *****
 ***** Creator:                     Peter Wallosek, University of Bonn
 *****
 ***** Date of creation:            Jun 1994
 *****
 *****
 *****
 *****
 ***** $Source: /usr/local/cvs/bee/src/simulator2/robot.c,v $
 *****
 ***** $Revision: 1.1 $
 *****
 ***** $Date: 2002/09/14 16:11:07 $
 *****
 ***** $Author: rstone $
 *****
 *****
 *****
 ***** Contact thrun@carbon.cs.bonn.edu or thrun@cs.cmu.edu.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 ***** 
 ***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY
 ***** APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING THE
 ***** COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM "AS IS"
 ***** WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
 ***** INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 ***** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE ENTIRE
 ***** RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU.
 ***** SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL
 ***** NECESSARY SERVICING, REPAIR OR CORRECTION.
 ***** 
 ***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
 ***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
 ***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
 ***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
 ***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
 ***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF
 ***** DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU
 ***** OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY
 ***** OTHER PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN
 ***** ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: robot.c,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.28  2000/03/09 09:30:09  schulz
 * Added Saschas multi-robot extension
 *
 * Revision 1.27  1999/08/26 21:17:29  fox
 * First changes for simulated multi robot localization.
 *
 * Revision 1.26  1997/07/17 17:31:52  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.25  1997/03/27 11:10:06  schulz
 * - Disabled tactile support for UNIBONN at the moment.
 * The robot does not seem to recover after a hit
 * - one more change in base_coord_{x,y}
 *
 * Revision 1.24  1997/03/25 21:44:47  tyson
 * Many bug fixes.
 *
 * Revision 1.23  1997/02/26 09:21:36  schulz
 * Fixed BeamRobot3()
 *
 * Revision 1.22  1997/02/25 18:12:45  tyson
 * client lib for PANTILT and lots of little stuff
 *
 * Revision 1.21  1997/02/07 09:15:24  schulz
 * SIMULATOR_set_robot_position message no longer changes the base position.
 *
 * Revision 1.20  1997/01/30 13:33:06  schulz
 * minor fixes
 *
 * Revision 1.19  1997/01/28 13:58:27  schulz
 * Added a TCX message port to move the robot from a program.
 * Not yet tested!
 *
 * Revision 1.18  1997/01/06 18:10:57  schulz
 * more coord system related fixes
 *
 * Revision 1.17  1997/01/06 15:05:50  schulz
 * *** empty log message ***
 *
 * Revision 1.16  1997/01/06 10:51:51  schulz
 * -- (0,0) is really in the lower left corner now!
 * -- sonar simulation works again.
 *
 * Revision 1.15  1996/12/19 15:37:44  schulz
 * -- adjusted default sonar_range to 650
 * -- introduced sonar_infinity as 3610.777832
 *
 * Revision 1.14  1996/12/13 15:24:24  schulz
 * robots initial position is 1213 1213 0 in base coordinate system now.
 *
 * Revision 1.13  1996/12/10 13:43:59  schulz
 * -- added -tcx option
 * -- loading of maps now possible
 * -- file requester has bigger width now.
 *    The translation for the return key has still to be changed!
 *
 * Revision 1.12  1996/12/02 17:01:57  schulz
 * new user interface for obstacle insertion
 * some fixes
 *
 * Revision 1.11  1996/11/18 14:43:12  ws
 * default ROBOT_RADIUS is taken from rai/B21/Base.h now.
 * Played around with job priorities a little bit.          DS
 *
 * Revision 1.10  1996/11/15 12:21:23  ws
 * Changed InsideObstacle to use the obstacle grid.          DS
 *
 * Revision 1.9  1996/11/14 13:43:33  ws
 * Obstacles are solid now!             DS
 *
 * Revision 1.8  1996/11/11 09:28:30  schulz
 * stopped the simulator from producing expose events while
 * drawing the sonar beams.
 *
 * Revision 1.7  1996/10/25 14:27:12  ws
 * some improvements
 * - better options handling
 * - added an initialization file, many parameters are modifiable now
 * - Modified some timings to reduce CPU load
 *   (TCX is polled after every 50 ms now)
 *
 * Revision 1.6  1996/10/23 08:57:17  ws
 * eliminated cross file references to struct robot
 *
 * Revision 1.5  1996/10/22 08:39:40  ws
 * fixed rotation initialization in base.c
 *
 * Revision 1.4  1996/10/15 13:58:54  ws
 * - new features: deletion of obstacles (Shift+ right mouse button)
 *               usage information (-h or -help option)
 * - added -laser option
 * - Now the simulator waits for BASE to come up before it starts
 *   sending laserReports.
 *
 * Revision 1.3  1996/10/09 13:27:42  schulz
 * reorganized obstacle store and fixed some more bugs.
 *
 * Revision 1.2  1996/10/07  14:08:10  schulz
 * robots initial position is read from map file now.
 *
 * Revision 1.1.1.1  1996/09/30 16:17:45  schulz
 * reimport on new source tree
 *
 * Revision 1.4  1996/08/27 15:15:20  schulz
 * Many bug fixes...
 * Installed File Headers, changed c++ file names to .cc/.hh file suffix
 * There is still a great performance problem!
 *
 * Revision 1.3  1996/08/27 08:12:03  schulz
 * Changed Filename suffix to .cc for c++ files
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <math.h>
#include <stdio.h>
#include <bUtils.h>
#include "simxw.h"
#include "playground.hh"
#include "robot.h"
#include "base.h"
#include "sonar.h"
#include "trigofkt.h"

extern Boolean no_tcx;

struct robot_struct robot;  
extern float sonar_dists[];

void robot_HandleMove()
{
    HandleMove();
}

void DrawSonarBeams(float *dists)
{
  
    if(!sonarOn) return;
    else {
	int i;
	float f;
	float cos_f; 
	float sin_f;
	float r = robot.RobotRadius;
	float xr = playground_x(robot.map_robot_x);
	float yr = playground_y(robot.map_robot_y);
	float dist;
	f = robot.robot_deg - sonar_offset;
	f *= M_PI/180;
	automatic_redraw(FALSE);
	for(i = 0; i < bRobot.sonar_cols[0]; i++) {
	    cos_f = myCOS(f);
	    sin_f = mySIN(f);
	    dist = dists[i]+r;
	    if(dist >= sonar_infinity) expose_beam(i,xr,yr,xr,yr);
	    else {
		expose_beam(i, xr+r*cos_f, yr+r*sin_f,
			    xr+dist*cos_f, yr+dist*sin_f);

	    }
	    f -= 2*M_PI/bRobot.sonar_cols[0];
	    while(f > 2*M_PI) f -= 2*M_PI;
	    while(f < 0) f += 2*M_PI; 	    
	}
	automatic_redraw(TRUE);
    }
}

static float playground_old_x;
static float playground_old_y;
void DrawRobot()
{
  Boolean alarm;
  if(traceOn)  {
    drawTraceLine(playground_old_x,
		  playground_old_y,
		  playground_x(robot.map_robot_x),
		  playground_y(robot.map_robot_y));
    playground_old_x = playground_x(robot.map_robot_x);
    playground_old_y = playground_y(robot.map_robot_y);
  }
  alarm = ( obstacles_min_distance(playground_x(robot.map_robot_x),
				  playground_y(robot.map_robot_y))
	   <= robot.RobotRadius); 
  DrawRobotFigure(alarm,
		  playground_x(robot.map_robot_x),
		  playground_y(robot.map_robot_y),
		  robot.RobotRadius, 360-robot.robot_deg);
}

void MoveRobot()
{
  DrawSonarBeams(sonar_dists);
  DrawLasers();
  if(robot.map_robot_x == robot.map_old_robot_x &&  
     robot.map_robot_y == robot.map_old_robot_y &&
     robot.robot_deg == robot.old_robot_deg)
    return;  
  robot.map_old_robot_y = robot.map_robot_y;
  robot.map_old_robot_x = robot.map_robot_x;
  robot.old_robot_deg = robot.robot_deg;
  DrawRobot();
}

void BeamRobot(float x, float y)
{
  robot.map_robot_x = base_x(x);
  robot.map_robot_y = base_y(y);
  robot.changePos = 1;
  robot.placed = TRUE;
  MoveRobot();  
}

extern float base_init_rot;
extern float base_correct_x;
extern float base_correct_y;

void BeamRobot3(float x, float y, float rot)
{
  float oldx = base_coord_x();
  float oldy = base_coord_y();  
  robot.map_robot_start_x += base_x(x) - robot.map_robot_x;
  robot.map_robot_start_y += base_y(y) - robot.map_robot_y;
  robot.map_robot_x = base_x(x);
  robot.map_robot_y = base_y(y);
  robot.robot_start_deg += 180.0 / M_PI * rot - robot.robot_deg;  
  robot.robot_deg = 180.0 / M_PI * rot;
  base_init_rot = (ROBOT_INIT_ROT - robot.robot_start_deg);
  BaseVariables.RotateWhere = robot.robot_deg;
  base_correct_x += oldx - base_coord_x();
  base_correct_y += oldy - base_coord_y();
  robot.changePos = 1;
  robot.placed = TRUE;
  MoveRobot();  
}

void InitRobot()
{
  robot.placed = FALSE;
  robot.runflag = 1;                  	/*** is robot running ? ***/
  robot.map_robot_start_x = robot.map_robot_x;
  robot.map_robot_start_y = robot.map_robot_y;
  robot.robot_start_deg = robot.robot_deg;
  robot.map_old_robot_x = robot.map_robot_x;
  robot.map_old_robot_y = robot.map_robot_y;    

  robot.RobotRadius = bRobot.base_radius;               /*** cm ***/
  robot.placed = TRUE;
  robot.changePos = 1;                	/*** true, when robot position was changed manually ***/
  robot.base_BRH = 0.0;
  InitRobotFigure(playground_x(robot.map_robot_x),
		  playground_y(robot.map_robot_y),
		  robot.RobotRadius,robot.robot_deg);
}

void setRobotPosition(float x, float y, float a)
{
  robot.map_old_robot_x = robot.map_robot_x;
  robot.map_old_robot_y = robot.map_robot_y;  
  robot.map_robot_x = x;
  robot.map_robot_y = y;
  robot.robot_deg = a;
}

void getRobotPosition(float *x, float *y, float *a)
{
  *x = robot.map_robot_x;
  *y = robot.map_robot_y;
  *a = robot.robot_deg;
}

void robot_setTarget(float x, float y)
{
  if(!no_tcx)  
      base_sendTarget(base_x(x), base_y(y));
}










