
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliIr.c,v $
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
 * $Log: colliIr.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.5  1998/08/18 16:24:23  fox
 * Added support for b18 robot.
 *
 * Revision 1.4  1997/06/17 09:39:30  fox
 * Changed rotate away.
 *
 * Revision 1.3  1997/05/06 18:03:30  fox
 * Infrareds and bumper should work. Take care of the INDEX!
 *
 * Revision 1.2  1997/05/06 17:05:03  fox
 * Nothing special.
 *
 * Revision 1.2  1997/04/17 09:16:18  fox
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/



#include "collisionIntern.h"

#define IR_THRESHOLD_DIST 10.0
#define IR_REMEMBER_TIME 3.0

static void
update_IrPoints( irInfo* irs);

void
COLLI_IrHandler( irInfo* irs)
{
  if ( 0 && irs->numberOfActivatedIrs > 0)
    fprintf( stderr, "%d irs\n", irs->numberOfActivatedIrs);

  if ( irs->numberOfActivatedIrs > 0
       ||
       ( irs->numberOfActivatedIrs == 0
	&&
	combinedObstaclePoints[IR_POINTS].no_of_points > 0
	 &&
	 ( timeExpired( IR_TIMER) > IR_REMEMBER_TIME))
       )
    update_IrPoints( irs);
  
  COLLI_update_tcx_IrPoints();
}

static void
update_IrPoints( irInfo* irs)
{
  int cnt;
  Point rpos;
  float rrot;

  float distanceToObstacle = ROB_RADIUS + IR_THRESHOLD_DIST;
  
  updateActualPosition( &rpos, &rrot, DONT_CONSIDER_DIRECTION);

  for ( cnt = 0; cnt < irs->numberOfActivatedIrs; cnt++) {
    
    float angle = rrot + irs->angle[cnt];
    
    combinedObstaclePoints[IR_POINTS].points[cnt].x =
      rpos.x + (fcos( angle) * distanceToObstacle);
    combinedObstaclePoints[IR_POINTS].points[cnt].y =
      rpos.y + (fsin( angle) * distanceToObstacle);
  }

  if ( irs->numberOfActivatedIrs > 0)
    setStartTime( IR_TIMER);
  
  combinedObstaclePoints[IR_POINTS].no_of_points = irs->numberOfActivatedIrs;
}

