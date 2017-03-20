
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliVision.c,v $
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
 * $Log: colliVision.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.5  1998/08/29 21:50:03  fox
 * Adapted exception handling in order to deal with people around the robot.
 *
 * Revision 1.4  1998/08/26 23:23:43  fox
 * Final version before I changed exception handling.
 *
 * Revision 1.3  1998/08/18 16:24:26  fox
 * Added support for b18 robot.
 *
 * Revision 1.2  1997/03/26 18:42:04  fox
 * Incorporated external obstacle information into the collision avoidance.
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


/**********************************************************************
* Receives collision lines from the vision module and uses them 
* like normal CollLines. 
**********************************************************************/
void COLLI_get_CollLines_from_vision(COLLI_vision_line_ptr vision_lines)
{
  int i;
  float x, y;

  if (vision_lines->no_of_lines > 0) {
    
    if (dumpInfo)
      fprintf( dumpFile, "Received vision line(s) at\n");
    fprintf(stderr, "Received vision line(s) at\n");
    if (External_Obstacle_Lines.no_of_lines > 0) 
      free (External_Obstacle_Lines.lines);
    
    External_Obstacle_Lines.no_of_lines = vision_lines->no_of_lines;
    External_Obstacle_Lines.lines = (LineSeg *) 
      malloc( External_Obstacle_Lines.no_of_lines * sizeof(struct LineSeg));
    
    for (i=0; i<External_Obstacle_Lines.no_of_lines; i++) {
      fprintf(stderr, " P1: %d %d  P2: %d %d\n", 
	      vision_lines->lines[i].pt1.x,
	      vision_lines->lines[i].pt1.y, 
	      vision_lines->lines[i].pt2.x,
	      vision_lines->lines[i].pt2.y);

      External_Obstacle_Lines.lines[i].pt1.x = (float) vision_lines->lines[i].pt1.x;
      External_Obstacle_Lines.lines[i].pt1.y = (float) vision_lines->lines[i].pt1.y;
      External_Obstacle_Lines.lines[i].pt2.x = (float) vision_lines->lines[i].pt2.x;
      External_Obstacle_Lines.lines[i].pt2.y = (float) vision_lines->lines[i].pt2.y;
    }
    
    gettimeofday(&Last_VisionLine_Update, 0);
    COLLI_update_tcx_External_Obstacle_Lines();

    x = 0.5 * (External_Obstacle_Lines.lines[0].pt1.x + External_Obstacle_Lines.lines[0].pt2.x);
    y = 0.5 * (External_Obstacle_Lines.lines[0].pt1.y + External_Obstacle_Lines.lines[0].pt2.y);
    
 }
  else
    fprintf( stderr, "Received 0 lines from vision.\n");
}

/**********************************************************************/
void COLLI_get_CollPoints_from_vision(COLLI_vision_point_ptr vision_points)
{
  int i;

  if (vision_points->no_of_points > 0) {
    
    fprintf(stderr, "Received %d vision point(s).\n",vision_points->no_of_points );
    if (combinedObstaclePoints[EXTERNAL_POINTS].no_of_points > 0) 
      free (combinedObstaclePoints[EXTERNAL_POINTS].points);
    
    combinedObstaclePoints[EXTERNAL_POINTS].no_of_points = vision_points->no_of_points;
    combinedObstaclePoints[EXTERNAL_POINTS].points = (Point *) 
      malloc( combinedObstaclePoints[EXTERNAL_POINTS].no_of_points * sizeof(struct Point));
    
    for (i=0; i<combinedObstaclePoints[EXTERNAL_POINTS].no_of_points; i++) {
            fprintf(stderr, " P: %d %d\n", 
	      vision_points->points[i].x,
	      vision_points->points[i].y);
	
      combinedObstaclePoints[EXTERNAL_POINTS].points[i].x = (float) vision_points->points[i].x;
      combinedObstaclePoints[EXTERNAL_POINTS].points[i].y = (float) vision_points->points[i].y;
    }

    gettimeofday(&Last_VisionPoint_Update, 0);
    COLLI_update_tcx_External_Obstacle_Points();
  }
}

