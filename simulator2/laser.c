

/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 *****
 ***** File:                        laser.c
 *****
 ***** Part of:                     RHINO SOFTWARE
 *****
 ***** Creator:                     Wolfram Burgard, University of Bonn
 *****
 ***** Date of creation:            Aug 1996
 *****
 *****
 *****
 *****
 ***** $Source: /usr/local/cvs/bee/src/simulator2/laser.c,v $
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
 * $Log: laser.c,v $
 * Revision 1.1  2002/09/14 16:11:07  rstone
 * *** empty log message ***
 *
 * Revision 1.21  2000/03/09 09:30:07  schulz
 * Added Saschas multi-robot extension
 *
 * Revision 1.20  1999/11/23 12:50:07  schulz
 * Fixed upper bounds in insert_grid
 *
 * Revision 1.19  1997/01/29 17:47:16  schulz
 * fixed a bug fill_ws_ray()
 *
 * Revision 1.18  1997/01/27 15:13:28  schulz
 * added added multiple sensor planes to the sensor simulation stuff
 * I hope this does not introduce to many new bugs.
 *
 * Revision 1.17  1997/01/10 15:19:32  schulz
 * minor bug fixes
 *
 * Revision 1.16  1997/01/10 13:10:06  fox
 * Removed some comments.
 *
 * Revision 1.15  1997/01/07 10:43:47  schulz
 * cleaned up changes in laser.c
 * reoved out commented code from playground.cc
 *
 * Revision 1.14  1997/01/06 18:10:57  schulz
 * more coord system related fixes
 *
 * Revision 1.13  1997/01/06 10:51:50  schulz
 * -- (0,0) is really in the lower left corner now!
 * -- sonar simulation works again.
 *
 * Revision 1.12  1996/12/04 05:42:30  tyson
 * simulator2 <-> baseServer partially tested.  Cleaning up bumble and sensor tests as demos.  Added frameWork module.
 *
 * Revision 1.11  1996/11/28 13:16:39  schulz
 * minor bugfix
 *
 * Revision 1.10  1996/11/18 14:43:11  ws
 * default ROBOT_RADIUS is taken from rai/B21/Base.h now.
 * Played around with job priorities a little bit.          DS
 *
 * Revision 1.9  1996/11/14 13:43:32  ws
 * Obstacles are solid now!             DS
 *
 * Revision 1.8  1996/10/30 12:47:06  schulz
 * changed inifile handling a little bit
 * changed sonar min. distance to 5cm
 *
 * Revision 1.7  1996/10/25 14:27:11  ws
 * some improvements
 * - better options handling
 * - added an initialization file, many parameters are modifiable now
 * - Modified some timings to reduce CPU load
 *   (TCX is polled after every 50 ms now)
 *
 * Revision 1.6  1996/10/23 08:57:16  ws
 * eliminated cross file references to struct robot
 *
 * Revision 1.5  1996/10/14 12:19:09  ws
 * added support for different surfaces (not yet complete) and sonar error
 *
 * Revision 1.4  1996/10/09 15:23:50  fox
 * Laser report is sent independently from the sonar report.
 *
 * Revision 1.3  1996/10/09 14:26:04  ws
 * removed some printfs fixed even more bugs
 *
 * Revision 1.2  1996/10/04 13:53:14  fox
 * Fixed a bug in laser interface
 *
 * Revision 1.1.1.1  1996/09/30 16:17:44  schulz
 * reimport on new source tree
 *
 * Revision 1.5  1996/09/11 14:03:49  schulz
 * - Speed up sonar and laser simulation by a factor of 20.
 *   We precalculate the set of obstacles which can be hit by a
 *   laser/sonar beam and check only for these few obstacles, if they are hit.
 *   Switched to S.Thruns version of line and circle intersection routines,
 *   these are a little bit faster then the old ones.
 * - Included laser visualisation  (to slow an some machines)
 *
 * Revision 1.4  1996/08/27 15:15:20  schulz
 * Many bug fixes...
 * Installed File Headers, changed c++ file names to .cc/.hh file suffix
 * There is still a great performance problem!
 *
 * Revision 1.3  1996/08/27 08:12:02  schulz
 * Changed Filename suffix to .cc for c++ files
 *
 * Revision 1.2  1996/08/26 14:38:07  schulz
 * *** empty log message ***
 *
 * Revision 1.1  1996/08/26 11:12:57  schulz
 * *** empty log message ***
 *
 * Revision 1.1  1996/08/16 21:09:39  fox
 * Laser simulation.
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/



#include <signal.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <values.h>
#include <math.h>
#include <stdio.h>

#include "laser.h"
#include "trigofkt.h"
#include "robot.h"
#include "simxw.h"
#include "playground.hh"
#include "surface.hh"


#define LASER_DEBUG 1

#include "LASER-messages.h"
#include "SIMULATOR-messages.h"

/* TCX_MODULE_PTR MODULE_LASER = NULL; */

static SIMULATOR_message_to_laser_type laserMsg;

int no_of_lasers = NUMBER_OF_LASERS;
int no_of_readings = NUMBER_OF_LASER_READINGS;
float laser_resolution = LASER_ANGLE_RESOLUTION;
float laser_range = LASER_RANGE;
float laser_offset = LASER_OFFSET;
float laser_zpos = 40.0;

static laserRangeFinder frontLaser;
static laserRangeFinder rearLaser;
static firstTime = 1;

float 
normedAngle(float angle);

extern Boolean use_baseServer;

int Float2Int(float x)
{
  if (x > MAXINT)
    x = MAXINT;
  else if (x < 0.0)
    x = -1;
  return (int) x;
}  

void
initLasers()
{
    int i;
    switch(2) {                             /* we initialize both because
					       we always send both */
    case 2: initLaser(&rearLaser, 90.0);
    case 1: initLaser(&frontLaser, 270.0);
    default:
    }
    laserMsg.r_reading = (int *) malloc(NUMBER_OF_LASER_READINGS
					 * sizeof(int));
    laserMsg.f_reading = (int *) malloc(NUMBER_OF_LASER_READINGS
					* sizeof(int));    
    laserMsg.r_numberOfReadings = NUMBER_OF_LASER_READINGS;
    laserMsg.f_numberOfReadings = NUMBER_OF_LASER_READINGS;
    laserMsg.r_angleResolution = LASER_ANGLE_RESOLUTION;
    laserMsg.f_angleResolution = LASER_ANGLE_RESOLUTION;
    laserMsg.r_startAngle = 0;
    laserMsg.f_startAngle = 0;
    for(i = 0; i < NUMBER_OF_LASER_READINGS; i++) {
	laserMsg.r_reading[i] = LASER_INFINITY_RANGE;
	laserMsg.f_reading[i] = LASER_INFINITY_RANGE;
    }
}

float
Deg2Rad(float angle)
{
  return angle * M_PI / 180.0;
}


void
initLaser(laserRangeFinder *laser, float startAngle)
{

  int i;
  
  laser->numberOfReadings = no_of_readings;

  for (i = 0; i < laser->numberOfReadings; i++)
	laser->reading[i] = 0.0;

  laser->angleResolution = laser_resolution;

  laser->startAngle = startAngle;

  laser->distanceFromCenter = laser_offset;
  
  return;

} 

float
laserAngle(int beam, float robotRot, laserRangeFinder *laser)
{
  return robotRot + laser->startAngle + beam * laser->angleResolution;
}

position
laserPosition(float robotX, float robotY, float robotRot,
		   laserRangeFinder *laser)
{
  position pos;
  
  pos.x = robotX +
    laser->distanceFromCenter*
      cos(Deg2Rad(normedAngle( robotRot + laser->startAngle + 90)));
  pos.y = robotY + laser->distanceFromCenter*sin( Deg2Rad(normedAngle(robotRot
						   + laser->startAngle + 90)));
  return pos;
}





/*************************************************************************/
/* PROCEDURE :	GetDistance(LaserRangFinder *laser)      		**/
/* Parameter :			No. of cell				**/
/* 									**/
/* Pass through list of obstacles and find the closest one inside range **/
/* Check angle and surface and check whether if obstacle is hit		**/
/* 									**/
/*************************************************************************/

void
getLaserReadings(laserRangeFinder *laser)
{
  float angle, dist;
  float rx,ry,rori;
  int beam;
  position pos, endPos;

  getRobotPosition(&rx,&ry,&rori);
  pos = laserPosition(rx, ry, rori, laser);
  laser->robotPos.x = rx;
  laser->robotPos.y = ry;
  
  laser->robotRot = rori;

  for (beam = 0; beam < laser->numberOfReadings; beam++)
  {
    angle = normedAngle( laserAngle(beam, rori, laser));

    endPos.x = pos.x + laser_range * cos( Deg2Rad( angle));
    endPos.y = pos.y + laser_range * sin( Deg2Rad(angle));    
    if( get_distance(LASER_SENSOR, pos.x, pos.y, laser_zpos,
		     0.001,
		     endPos.x, endPos.y, &dist))
      laser->reading[beam] = dist;
    else
      laser->reading[beam] = laser_range;
  }
}

void DrawLaserReadings(int offset, laserRangeFinder *laser)
{
    float angle1, dist;
    float rx,ry,rori;
    int beam;
    position startPos, endPos, pos;

    if(!laserOn) return;
    getRobotPosition(&rx,&ry,&rori);
    
    for (beam = 0; beam < laser->numberOfReadings; beam+=3) {
	angle1 = Deg2Rad(normedAngle(laserAngle(beam, rori, laser)));
	pos = laserPosition(rx, ry, rori, laser);
	startPos.x = playground_x(pos.x);
	startPos.y = playground_y(pos.y);  
	endPos.x = startPos.x+laser->reading[beam]*cos(angle1);
	endPos.y = startPos.y+laser->reading[beam]*sin(angle1);
	expose_laser_beam(offset + beam,
			  startPos.x, startPos.y, endPos.x, endPos.y);
    }
}

void DrawLasers()
{
    switch(no_of_lasers) {
    case 2: DrawLaserReadings(frontLaser.numberOfReadings, &rearLaser);    
    case 1: DrawLaserReadings(0, &frontLaser);
    default:
    }
}

void
getLasers()
{
  if (firstTime){
    fprintf(stderr,"# Initializing Lasers ...");
    initLasers();
    fprintf(stderr," done\n");
    firstTime = 0;
  }
  switch(no_of_lasers) {
  case 2:
    getLaserReadings(&rearLaser);
  case 1:
    getLaserReadings(&frontLaser);
  default:
  }
}


/*************************************************************************/
/* PROCEDURE :			laserReport()      			**/
/* Parameter :			none					**/
/* 									**/
/* Sends laser report via TCX						**/
/* 									**/
/*************************************************************************/

void
laserReport()
{
  int i=0;
  int time_lost;

  
  getLasers();

#ifdef LASERDEBUG
  fprintf(stderr, "SIMULATOR :   SENDING  LASER Report\n");
  fprintf(stderr, "# Copying laser values in message format ...");
#endif
  switch(no_of_lasers) {
  case 2:
      laserMsg.r_numberOfReadings = rearLaser.numberOfReadings;
      for (i=0; i<rearLaser.numberOfReadings; i++){
	  laserMsg.r_reading[i] =
	      Float2Int( rearLaser.reading[i]);
      }
      laserMsg.r_startAngle = Deg2Rad(normedAngle(rearLaser.startAngle
						  + 180.0));
      laserMsg.r_angleResolution = Deg2Rad(rearLaser.angleResolution);
  case 1:
      laserMsg.f_numberOfReadings = frontLaser.numberOfReadings;
      for (i=0; i<frontLaser.numberOfReadings; i++){
	  laserMsg.f_reading[i] =
	      Float2Int( frontLaser.reading[i]);
      }
      laserMsg.f_startAngle = Deg2Rad( normedAngle(frontLaser.startAngle));
      laserMsg.f_angleResolution = Deg2Rad( frontLaser.angleResolution);
  default:
  }
#ifdef LASERDEBUG
  fprintf(stderr, " done\n");
  fprintf(stderr, "# Sending message ...");
#endif
  if (use_baseServer == FALSE) {
    tcxSendMsg( MODULE_BASE, "SIMULATOR_message_to_laser", &laserMsg);
  }
#ifdef LASERDEBUG
  fprintf(stderr, " done\n");
#endif
/*
  free(laserMsg.r_reading);
  free(laserMsg.f_reading);
*/
  return;

} /* laserReport() */

