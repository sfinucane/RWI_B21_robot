
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliBumper.c,v $
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
 * $Log: colliBumper.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.5  1998/08/18 16:24:22  fox
 * Added support for b18 robot.
 *
 * Revision 1.4  1997/05/06 18:03:30  fox
 * Infrareds and bumper should work. Take care of the INDEX!
 *
 * Revision 1.3  1997/04/26 13:56:17  fox
 * Added targetDefined, targetX, and targetY to status report.
 *
 * Revision 1.2  1997/04/17 09:16:18  fox
 * Added timeout for laser devices --> colliServer only needs 30% cpu.
 *
 * Revision 1.1  1997/04/10 12:32:31  fox
 * Support for bumpers.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/



#include "collisionIntern.h"

#define BUMPER_REMEMBER_TIME 10.0
static float bumperTimeOut;


static void
update_BumperPoints( tactileInfo* tactiles);


void
updateBumperTimer()
{
  if ( combinedObstaclePoints[BUMPER_POINTS].no_of_points > 0)
    if ( timeExpired( BUMPER_TIMER) > BUMPER_REMEMBER_TIME) {
      combinedObstaclePoints[BUMPER_POINTS].no_of_points = 0;
      fprintf(stderr, "Bumper too old (remove it).\n");
      COLLI_update_tcx_BumperPoints();
    }
}


/**********************************************************************
 * Handles tacitle callback.
 **********************************************************************/
void
COLLI_BumpHandler( Pointer callback_data, Pointer client_data)
{
  int angleCnt = 0;
  int tmpTarget = target_flag;
  tactileInfo* tactiles = (tactileInfo*) callback_data;

  if ( (combinedObstaclePoints[BUMPER_POINTS].no_of_points +
	NUMBER_OF_POINTS_PER_BUMPER * tactiles->numberOfActivatedTactiles)
       > MAX_NUMBER_OF_BUMPERS) {
    fprintf(stderr, "Too many bumpers (%d, %d). Resize MAX_NUMBER_OF_BUMPERS.\n",
	    tactiles->numberOfActivatedTactiles, MAX_NUMBER_OF_BUMPERS);
    return;
  }
  
  if ( tactiles->numberOfActivatedTactiles < 1) {
    if ( 0) fprintf( stderr, "Bumper released.\n");
    return;
  }
  
  update_BumperPoints( tactiles);
  setStartTime( BUMPER_TIMER);

  BASE_TranslateCollisionAcceleration( ACTUAL_MODE->exception_trans_acceleration);
  BASE_RotateCollisionAcceleration( RAD_TO_DEG( ACTUAL_MODE->exception_rot_acceleration));

  BASE_TranslateHalt();
  BASE_RotateHalt();

  target_flag = tmpTarget;
}


/**********************************************************************
 * Updates the structure containing the points of the obstacles as detected
 * by the bumpers.
 **********************************************************************/
#define BUMPER_ANGLE (DEG_TO_RAD( 10.0))

static void
update_BumperPoints( tactileInfo* tactiles)
{
  int cnt, pointCnt; 
  int activeBumper = combinedObstaclePoints[BUMPER_POINTS].no_of_points;
  Point rpos;
  float rrot;

  float angle;
  float angleStep;

  /* We insert several points per bumper. */
  if ( NUMBER_OF_POINTS_PER_BUMPER == 1) {
    angle = 0.0;
    angleStep = 0.0;
  }
  else {
    angle     = - BUMPER_ANGLE / 2.0;
    angleStep = BUMPER_ANGLE / (float) (NUMBER_OF_POINTS_PER_BUMPER - 1);
  }
  
  updateActualPosition( &rpos, &rrot, DONT_CONSIDER_DIRECTION);

  for ( cnt = 0; cnt < tactiles->numberOfActivatedTactiles; cnt++) {
    
    for ( pointCnt = 0; pointCnt < NUMBER_OF_POINTS_PER_BUMPER; pointCnt++) {
      
      float pointAngle = rrot + tactiles->angle[cnt] + angle + pointCnt * angleStep;
      int index = activeBumper + NUMBER_OF_POINTS_PER_BUMPER * cnt + pointCnt;
      float translationTillStop;
      float distanceToObstacle;

      /* If it is a rear bumper we substract the distance till the robot
       * has stopped. Otherwise we add this distance. */
      translationTillStop = brakeDistance( rwi_base.trans_current_speed,
					   -ACTUAL_MODE->exception_trans_acceleration);
      if (tactiles->angle[cnt] > DEG_90 && tactiles->angle[cnt] < DEG_270)
	translationTillStop = - translationTillStop;
      
      distanceToObstacle = translationTillStop + ROB_RADIUS;
      
      combinedObstaclePoints[BUMPER_POINTS].points[index].x =
	rpos.x + (fcos( pointAngle) * distanceToObstacle);
      combinedObstaclePoints[BUMPER_POINTS].points[index].y =
	rpos.y + (fsin( pointAngle) * distanceToObstacle);
    }
  }

  combinedObstaclePoints[BUMPER_POINTS].no_of_points = activeBumper
    + NUMBER_OF_POINTS_PER_BUMPER * tactiles->numberOfActivatedTactiles;
  
  COLLI_update_tcx_BumperPoints();
}
