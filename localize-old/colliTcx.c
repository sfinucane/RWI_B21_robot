
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/colliTcx.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 17:06:29 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: colliTcx.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.8  2000/03/06 20:00:42  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.7  1998/10/29 03:44:58  fox
 * Nothing special.
 *
 * Revision 1.6  1998/06/12 10:16:24  fox
 * Implemented virutal sensor.
 *
 * Revision 1.5  1998/02/12 15:47:19  derr
 * compiler warnings (unused variable ... ) removed
 *
 * Revision 1.4  1997/06/03 11:49:20  fox
 * Museum version.
 *
 * Revision 1.3  1997/02/12 15:08:35  fox
 * Integrated laser support.
 *
 * Revision 1.2  1997/02/05 16:02:39  fox
 * Changed BASE_setmode message.
 *
 * Revision 1.1  1997/01/29 12:36:21  fox
 * New version.
 *
 * Revision 1.3  1997/01/06 17:38:39  fox
 * Improved version.
 *
 * Revision 1.2  1996/12/31 11:43:54  fox
 * First version using RANDOM_MODE of COLLI.
 *
 * Revision 1.1  1996/12/31 09:19:45  fox
 * Communication with COLLI.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include "general.h"
#include "localTcx.h"
#include "colliTcx.h"
#include "file.h"
#include "function.h"
#include "laser.h"

#include "BASE-messages.h"


/***********************************************************************
 ***********************************************************************/
void
initializeColli( actionInformation* actionInfo)
{
  static bool alreadyInitialized = FALSE;

  if ( ! alreadyInitialized) {

    /* Send this map to the planning module. */
    if ( connectionEstablished( &BASE, moduleName[BASE_MODULE])) {

      fprintf( stderr, "Initialize BASE.\n");
      writeLog( "Initialize BASE.\n");
      alreadyInitialized = TRUE;

    }
  }
  actionInfo=actionInfo;
}


/***********************************************************************
 *
 ***********************************************************************/
bool
setColliMode( int mode)
{
  BASE_setmode_type modeInfo;
  modeInfo.useSonar = -1;
  modeInfo.useLaser = -1;
  modeInfo.modeNumber = mode;

  if ( mode == RANDOM_MODE)
    modeInfo.useSonar = 1;
  else
    modeInfo.useSonar = 1;

  if ( connectionEstablished( &BASE, moduleName[BASE_MODULE])) {

#ifdef USER_debug
    fprintf( stderr, "TCX message to COLLI: set mode %d.\n", mode);
#endif
    fprintf( stderr, "TCX message to COLLI: set mode %d.\n", mode);
    tcxSendMsg ( BASE, "BASE_setmode", &modeInfo);

    return TRUE;
  }
  else
    return FALSE;
}


/***********************************************************************
 *
 ***********************************************************************/
bool
stopColli()
{
  if ( connectionEstablished( &BASE, moduleName[BASE_MODULE])) {

    writeLog( "TCX message to COLLI: stop robot.\n");

    tcxSendMsg ( BASE, "BASE_stop_robot", NULL);

    return TRUE;
  }
  else
    return FALSE;
}


/***********************************************************************
 *
 ***********************************************************************/
bool
sendGoalPointToColli( float forward, float sideward)
{
  if ( connectionEstablished( &BASE, moduleName[BASE_MODULE])) {

    BASE_goto_relative_type target;

    target.rel_target_x = forward;
    target.rel_target_y = sideward;

    writeLog( "TCX message to COLLI: goal point (%f, %f).\n", forward, sideward);

    tcxSendMsg ( BASE, "BASE_goto_relative", &target);

    return TRUE;
  }
  else
    return FALSE;
}


/***********************************************************************
 *
 ***********************************************************************/
#define MAX_VIRTUAL_DISTANCE 400.0
#define MAX_NUMBER_OF_VIRTUAL_READINGS 180

void
sendVirtualReadingsToColli( actionInformation* info, int numberOfReadings,
			    float* distances)
{
  if ( connectionEstablished( &BASE, moduleName[BASE_MODULE])) {

    int sens, currentPoint = 0;
    BASE_obstacle_points_type obstaclePoints;
    obstaclePoint points[MAX_NUMBER_OF_VIRTUAL_READINGS];
    float angleStep = DEG_360 / numberOfReadings;
    
    /* Consider offset of lasers. */
    float robRot =
      normalizedAngle( deg2Rad( - info->actualSensings.basePosition.rot + 90.0)) - DEG_90;
    float robX = info->actualSensings.basePosition.x +
      cos( robRot + DEG_90) * frontLaserOffset;
    float robY = info->actualSensings.basePosition.y +
      sin( robRot + DEG_90) * frontLaserOffset;
    
    for ( sens = 0; sens < numberOfReadings; sens++) {
      
      /* Skip offset to rear laser. */
      if ( sens == numberOfReadings / 2) {
	robX += cos( robRot + DEG_90) * (-frontLaserOffset + rearLaserOffset);
	robY -= sin( robRot + DEG_90) * (-frontLaserOffset + rearLaserOffset);
      }
      
      if ( distances[sens] < MAX_VIRTUAL_DISTANCE) {
	
	float absoluteAngle = robRot + sens* angleStep;
	
	points[currentPoint].x = robX + cos( absoluteAngle) * distances[sens];
	points[currentPoint].y = robY + sin( absoluteAngle) * distances[sens];
	
	if (0) fprintf( stderr, "%d %f %f --> %f %f --> %d %d\n", sens, rad2Deg(absoluteAngle),
			distances[sens], robX, robY, points[currentPoint].x,points[currentPoint].y);
	currentPoint++;
      }
    }
    
    obstaclePoints.points = points;
    obstaclePoints.no_of_points = currentPoint;

    fprintf(stderr, "Send %d readings\n", currentPoint);
    tcxSendMsg( BASE, "BASE_obstacle_points", &obstaclePoints);
  }
}


