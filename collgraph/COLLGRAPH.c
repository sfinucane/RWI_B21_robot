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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/collgraph/COLLGRAPH.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 16:46:27 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: COLLGRAPH.c,v $
 * Revision 1.1  2002/09/14 16:46:27  rstone
 * *** empty log message ***
 *
 * Revision 1.25  1999/06/23 12:07:52  haehnel
 * I don´t need always dump-information
 *
 * Revision 1.24  1999/04/18 16:56:13  fox
 * Multiple COLLGRAPHS can connect to the same robot.
 *
 * Revision 1.23  1998/11/12 18:32:33  fox
 * Nothing special.
 *
 * Revision 1.22  1998/10/30 18:53:04  fox
 * Changed support for multiple robots.
 *
 * Revision 1.21  1998/09/12 21:26:47  fox
 * Final version of the museum.
 *
 * Revision 1.20  1998/06/12 10:14:10  fox
 * Enhanced functions for dumping figures.
 *
 * Revision 1.19  1998/04/22 15:24:08  wolfram
 * Added the -robot option for multi-robot support
 *
 * Revision 1.18  1998/01/12 16:22:18  fox
 * Removed dump file.
 *
 * Revision 1.17  1997/10/30 16:38:02  fox
 * Changed window size.
 *
 * Revision 1.16  1997/10/25 00:23:22  tyson
 * Still getting the X stuff right
 *
 * Revision 1.15  1997/07/17 17:31:45  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.14  1997/06/03 11:49:10  fox
 * Museum version.
 *
 * Revision 1.13  1997/05/06 18:04:12  fox
 * Changed a color.
 *
 * Revision 1.12  1997/04/10 12:11:12  fox
 * Added bumper display.
 *
 * Revision 1.11  1997/04/01 11:35:57  fox
 * Adapted names to COLLI-messages.h
 *
 * Revision 1.10  1997/03/25 21:44:41  tyson
 * Many bug fixes.
 *
 * Revision 1.9  1997/02/22 05:16:30  thrun
 * Version numbers are now also checked for libraries.
 *
 * Revision 1.8  1997/02/22 00:59:01  thrun
 * Introduced version number support
 *
 * Revision 1.7  1997/02/05 16:23:50  fox
 * Cleaned up the bug fixes created by tyson and sebastian when cleaning up
 * previous bugs.
 *
 * Revision 1.6  1997/02/05 16:02:37  fox
 * Changed BASE_setmode message.
 *
 * Revision 1.5  1997/02/02 22:32:27  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.4  1996/12/03 05:35:25  thrun
 * If TCXHOST is not se, the software will now assume "localhost" and
 * won't terminate.
 *
 * Revision 1.3  1996/10/24 06:55:34  fox
 * COLLGRAPH works even without write permissions in the directory.
 *
 * Revision 1.2  1996/10/14 17:31:07  fox
 * Changed the output.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:26  rhino
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


#include <ctype.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <values.h>
#include <signal.h>
#include "EZX11.h"
#include "tcx.h"
#include "tcxP.h"
#include "Common.h"
#include "bUtils.h"

#define DEFINE_REPLY_HANDLERS /* this makes sure we'll get the handler array */
#define TCX_define_variables  /* this makes sure variables are installed */ 
#include "BASE-messages.h"
#include "SONAR-messages.h"
#include "LASER-messages.h"
#include "IR-messages.h"
#include "collision.h" 

#include "beeSoftVersion.h"
#include "librobot.h"
#include "libezx.h"


#define TCX_USER_MODULE_NAME "COLLGRAPH"
/* #define USER_debug */

#define NUMBER_OF_MODULES 2
#define MODULE_NAME_LENGTH 80
#define COLLGRAPH_MODULE 0
#define BASE_MODULE 1

EZXW_p robwin;

static  BASE_register_auto_update_type data = {0, 0, 0, 0, 1};

int block_wait(struct timeval *timeout, int tcx_initialized,
	       int X_initialized);

int COLLISION_REMEMBER_INTERVAL;

FILE* dumpFile;

BOOLEAN replayMode = FALSE;
BOOLEAN waitMode = TRUE;
BOOLEAN dumpMode = FALSE;
BOOLEAN onlyMotionMode = TRUE;
BOOLEAN displayGraphics = TRUE;
int colliSubscribe = 1;

iPoint rpos;
float rrot; 
iLine **iCollLines;

/* #define PAPER_OUTPUT */
/* #define TALK_OUTPUT */
/* #define BW_OUTPUT  */

#ifdef TALK_OUTPUT
#define THICK 5
#define DOT_SIZE 5
#else
#define THICK 3
#define DOT_SIZE 3
#endif

#ifdef BW_OUTPUT
#define ROBOT_COL         C_GREY80
#define TARGET_COL        C_GREY60
#define TRAJECTORY_COL    C_GREY60
#define SONAR_COL         C_GREY40
#define LASER_COL         C_GREY40
#define SERVER_COL        C_BLACK
#define BACKGROUND_COL    C_BLACK
#define INFRARED_COL      C_GREY40
#define BUMPER_COL        C_GREY40
int W_SIZE = 1000;
#else

#define ROBOT_COL         C_BLUE
#define TARGET_COL        C_YELLOW
#define TRAJECTORY_COL    C_LAWNGREEN
#define SONAR_COL         C_RED
#define LASER_COL         C_CYAN
#define SERVER_COL        C_BLUE
#define BACKGROUND_COL    C_WHITE
#define INFRARED_COL      C_GREY70
#define BUMPER_COL        C_DEEPPINK
int W_SIZE = 400;
#endif

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/**** the following handlers won't be used! *****/

void
BASE_robot_position_reply_handler(TCX_REF_PTR                   ref,
                                  BASE_robot_position_reply_ptr pos)
{
  tcxFree("BASE_robot_position_reply", pos);
}

void
BASE_update_status_reply_handler(TCX_REF_PTR                   ref,
                                 BASE_update_status_reply_ptr status)
{
  tcxFree("BASE_update_status_reply", status);
}

void
BASE_action_executed_reply_handler(TCX_REF_PTR                   ref,
                                        BASE_action_executed_reply_ptr data)
{
  tcxFree("BASE_action_executed_reply", data);
}

void
SONAR_sonar_reply_handler(TCX_REF_PTR           ref,
                          SONAR_sonar_reply_ptr sonar)
{
  tcxFree("SONAR_sonar_reply", sonar);
}

void
SONAR_ir_reply_handler(TCX_REF_PTR           ref,
                       SONAR_ir_reply_ptr ir)
{
  tcxFree("SONAR_ir_reply", ir);
}


void
IR_ir_reply_handler(TCX_REF_PTR           ref,
                    IR_ir_reply_ptr ir)
{
  tcxFree("IR_ir_reply", ir);
}


void
LASER_laser_reply_handler(TCX_REF_PTR           ref,
                          LASER_laser_reply_ptr laser)
{
  int i;

  fprintf( stderr, "Front: ");
  for ( i = 0; i < laser->f_numberOfReadings; i++)
    fprintf(stderr, "%d ", laser->f_reading[i]);

  fprintf( stderr, "Rear: ");
  for ( i = 0; i < laser->r_numberOfReadings; i++)
    fprintf(stderr, "%d ", laser->r_reading[i]);


  tcxFree("LASER_laser_reply", laser);
}




/**********************************************************************
 *                          Help functions                            *
 **********************************************************************/
     
/**********************************************************************/
float fcos( float x)
{
  return((float) cos((double) x));
}


/**********************************************************************/
float fsin( float x)
{
  return((float) sin((double) x));
}


/**********************************************************************
 *                          EZX routines                              *
 **********************************************************************/

void DrawLine( iPoint rpos, iLine line, int col)
{
  int col_sve;
  
  col_sve = EZX_SetColor(col);

  if (line.pt1.x != I_ERROR)
    EZX_DrawLine(robwin,
                 W_SIZE/2 + (line.pt1.x-rpos.x),
                 W_SIZE/2 - (line.pt1.y-rpos.y), 
                 W_SIZE/2 + (line.pt2.x-rpos.x), 
                 W_SIZE/2 - (line.pt2.y-rpos.y)); 

  EZX_Flush();
  
  EZX_SetColor(col_sve);
}


/**********************************************************************/
void DrawCircle( iPoint rpos, iCircle circ, int col)
{
  int col_sve;
  
  col_sve = EZX_SetColor(col); 
  
  if (circ.rad > 0)
    EZX_DrawCircle(robwin, W_SIZE/2 +  (circ.M.x-rpos.x), 
                   W_SIZE/2 -  (circ.M.y-rpos.y), 
                   circ.rad);
  EZX_Flush();
  
  EZX_SetColor(col_sve);
}
  

/**********************************************************************/
void DrawArc( iPoint rpos, iCircle circ,
              float startAngle, float endAngle,
              int col)
{
  int col_sve;
  float angleToEZXCount = 64.0;
  col_sve = EZX_SetColor(col); 

  if (circ.rad > 0)
    EZX_DrawArc( robwin,
                 W_SIZE/2 + (circ.M.x-rpos.x), 
                 W_SIZE/2 - (circ.M.y-rpos.y), 
                 circ.rad, circ.rad,
                 (int) (startAngle * angleToEZXCount),
                 (int) (endAngle * angleToEZXCount));
  EZX_Flush();
  
  EZX_SetColor(col_sve);
}
  

/**********************************************************************/
void DrawMark( iPoint rpos, iPoint markpos, int col)

{  int col_sve;
  int width_sve;
  
  col_sve = EZX_SetColor(col);
  width_sve = EZX_SetLineWidth(THICK);
  
  EZX_DrawCircle( robwin, W_SIZE/2 +  (markpos.x-rpos.x), W_SIZE/2 - 
                  (markpos.y-rpos.y), 7); 
  EZX_Flush();
  
  EZX_SetColor(col_sve);
  EZX_SetLineWidth(width_sve);
}
/**********************************************************************/
void DrawInfoLine(iPoint rpos, float tvel, float dist)
{
  int width_sve;
  char s[80];

  width_sve = EZX_SetLineWidth(THICK);

  sprintf(s, "Tvel %.1f", tvel);
  EZX_DrawTextAt(robwin, W_SIZE - 10, W_SIZE, s, 'R'); 

  sprintf(s, "Distance %.1f", dist);
  EZX_DrawTextAt(robwin, 10, W_SIZE, s, 'L'); 

  EZX_Flush();
  EZX_SetLineWidth(width_sve);
}
  

/**********************************************************************/
void DrawRobot( iPoint rpos, float rrot)
{
  iCircle rob;
  iLine rob_line, tmp;
  int width_sve;

  width_sve = EZX_SetLineWidth(THICK);
  
  rob.rad = ROB_RADIUS;
  rob.M = rpos;
  rob_line.pt1 = rpos;
  rob_line.pt2.x = rpos.x + (int) (cos( (double) rrot) * ROB_RADIUS);
  rob_line.pt2.y = rpos.y + (int) (sin( (double) rrot) * ROB_RADIUS);
  DrawLine( rpos, rob_line, ROBOT_COL);
  DrawCircle( rpos, rob, ROBOT_COL);

  EZX_Flush();

  EZX_SetLineWidth(width_sve);
}


/**********************************************************************/
void
DrawArmInformation( iPoint rpos, iCircle armCircle,
                   iPoint innerArmPoint, iPoint outerArmPoint)
{
   if ( armCircle.M.x != I_ERROR)
      DrawCircle(rpos, armCircle, C_RED);

   if ( innerArmPoint.x != I_ERROR)
      DrawMark( rpos, innerArmPoint, C_RED);
   if ( outerArmPoint.x != I_ERROR)
      DrawMark( rpos, outerArmPoint, C_CYAN);
}

/**********************************************************************/
void
DrawCircleTrajectory( iPoint rpos,
                      iCircle inner, iCircle outer,
                      iPoint target)
{
#ifdef PAPER_OUTPUT
  float dx, dy;
  float start = 0.0;
  float end = 360.0;

  if ( target.x != I_ERROR && (inner.M.x != rpos.x || inner.M.y != rpos.y)) {

    dx = rpos.x - inner.M.x;
    dy = rpos.y - inner.M.y;
    start = RAD_TO_DEG( atan2((double) dy, (double) dx));
    if ( start < 0) start += 360.0;

    dx = target.x - inner.M.x;
    dy = target.y - inner.M.y;
    end = RAD_TO_DEG( atan2((double) dy, (double) dx));
    if ( end < 0) end += 360.0;

    end -= start;
    if ( end > 180)
      end = 360 - end;
    else if ( end < -180)
      end += 360;
    
    if ( end < 0) 
      end -= 10; 
    else 
      end += 10; 
  }

  EZX_SetLineStyle(1); 
  DrawArc(rpos, inner, start, end, TRAJECTORY_COL);
  DrawArc(rpos, outer, start, end, TRAJECTORY_COL);
  EZX_SetLineStyle(0); 
#else
  DrawCircle(rpos, inner, TRAJECTORY_COL);
  DrawCircle(rpos, outer, TRAJECTORY_COL);
#endif
}

/**********************************************************************/
void
DrawStraightTrajectory(iPoint rpos, iLine left, iLine right)
{
#ifdef PAPER_OUTPUT
  EZX_SetLineStyle(1); 
#endif
  DrawLine( rpos, left, TRAJECTORY_COL);
  DrawLine( rpos, right, TRAJECTORY_COL);
#ifdef PAPER_OUTPUT
  EZX_SetLineStyle(0); 
#endif
}


/**********************************************************************/
void DrawTarget( iPoint rpos, iPoint target)
{
    iLine showline;

    if ( target.x != I_ERROR) {
        showline.pt1 = rpos;
        showline.pt2 = target;
        DrawLine(rpos, showline, TARGET_COL);
        DrawMark(rpos, target, TARGET_COL);
    }
}

/**********************************************************************/
void
DrawCollLines(void)
{
  int i, j;
  
  for (i=0; i<bRobot.sonar_cols[0]; i++) 
    for (j=0; j < COLLISION_REMEMBER_INTERVAL; j++)  
      if (iCollLines[i][j].pt1.x != I_ERROR) 
        DrawLine( rpos, iCollLines[i][j], SONAR_COL);
}


/**********************************************************************/
void
DrawLaserCollPoints( int no, iPoint *points)
{
  int i;
  int col_sve;

  col_sve = EZX_SetColor(LASER_COL);
  
  for (i=0; i < no; i++) 
    if (points[i].x != I_ERROR) {
#ifdef PAPER_OUTPUT
      /* In paper no max range. */
      if ( sqrt( ( rpos.x - points[i].x) * ( rpos.x - points[i].x)
		 + (rpos.y - points[i].y) * (rpos.y - points[i].y)) < 490)
#endif
	EZX_FillCircle( robwin, W_SIZE/2 + (points[i].x-rpos.x),
			W_SIZE/2 - (points[i].y-rpos.y), DOT_SIZE);
    }

  EZX_SetColor(col_sve);
  EZX_Flush();
}

/**********************************************************************/
void
DrawExternalCollLines(int no,iLine *lines)
{
  int i;
  
  for (i=0; i<no; i++) 
    if (lines[i].pt1.x != I_ERROR) 
      DrawLine( rpos, lines[i], SERVER_COL);
}


/**********************************************************************/
void
DrawExternalCollPoints(int no, iPoint *points)
{
  int i;
  int col_sve;
  
  col_sve = EZX_SetColor(SERVER_COL);

  for (i=0; i<no; i++) {
    if (points[i].x != I_ERROR) 
      EZX_FillCircle( robwin, W_SIZE/2 + (points[i].x-rpos.x),
                     W_SIZE/2 - (points[i].y-rpos.y), DOT_SIZE); 
  }
  
  EZX_SetColor(col_sve);
  EZX_Flush();
}


/**********************************************************************/
void
DrawBumperCollPoints(int no, iPoint *points)
{
  int i;
  int col_sve;
  
  col_sve = EZX_SetColor(BUMPER_COL);

  for (i=0; i<no; i++) {
    if (points[i].x != I_ERROR) 
      EZX_FillCircle( robwin, W_SIZE/2 + (points[i].x-rpos.x),
                     W_SIZE/2 - (points[i].y-rpos.y), 5); 
  }
  
  EZX_SetColor(col_sve);
  EZX_Flush();
}


/**********************************************************************/
void
DrawIrCollPoints(int no, iPoint *points)
{
  int i;
  int col_sve;
  
  col_sve = EZX_SetColor(INFRARED_COL);

  for (i=0; i<no; i++) {
    if (points[i].x != I_ERROR) 
      EZX_FillCircle( robwin, W_SIZE/2 + (points[i].x-rpos.x),
                     W_SIZE/2 - (points[i].y-rpos.y), 5); 
  }
  
  EZX_SetColor(col_sve);
  EZX_Flush();
}

#define RECT_ROBOT_FRONT_LENGTH 30.0

/**********************************************************************/
void
DrawRectangularRobot( iPoint rpos, iLine* robot, iLine* testLines)
{
  int i;
  iLine robLine;
  
  for (i=0; i<4; i++) 
    DrawLine( rpos, robot[i], ROBOT_COL);

  if ( testLines[0].pt1.x != I_ERROR) 
    for (i=0; i<4; i++)
      DrawLine( rpos, testLines[i], INFRARED_COL);
  
  robLine.pt1 = rpos;
  
  robLine.pt2.x = rpos.x + (int) (cos( (double) rrot) * RECT_ROBOT_FRONT_LENGTH);
  robLine.pt2.y = rpos.y + (int) (sin( (double) rrot) * RECT_ROBOT_FRONT_LENGTH);
  DrawLine( rpos, robLine, ROBOT_COL);
}



void
dumpInformation( COLLI_colli_reply_ptr colli, char* fileName)
{
  int i;
  static BOOLEAN firstTime = TRUE;
  
  if ( firstTime) {
    
    firstTime = FALSE;
    if ((dumpFile = fopen( fileName, "w")) == NULL) {
      fprintf( stderr, "WARNING: cannot open dumpFile %s.\n", fileName);
      return;
    }
  }
  else if ( dumpFile == NULL)
    return;
  
  /* Robot position. */
  fprintf( dumpFile, "%i %i %i\n",
             ((colli->rpos).x), ((colli->rpos).y), (colli->rrot));
  
  /* Velocities and target. */
  fprintf( dumpFile, "%i %i %i %i %i\n",
	   (colli->tvel), (colli->rvel), (colli->dist),
	   ((colli->targetpoint).x), ((colli->targetpoint).y));
  
  /* Remember interval. */
  fprintf( dumpFile, "%i\n", (colli->rememberInterval));
  
  /* Sonar lines */
  for ( i=0; i < bRobot.sonar_cols[0]; i++)
    fprintf( dumpFile, "%i %i %i %i\n",
	     ((colli->sonar_lines)[i].pt1.x), 
	     ((colli->sonar_lines)[i].pt1.y), 
	     ((colli->sonar_lines)[i].pt2.x), 
	     ((colli->sonar_lines)[i].pt2.y));
  
  /* Laser points. */
  fprintf( dumpFile, "%i\n", colli->no_of_laser_points);
  for ( i=0; i < colli->no_of_laser_points; i++)
    fprintf( dumpFile, "%i %i\n",
	     ((colli->laser_points)[i].x), 
	     ((colli->laser_points)[i].y));
  
  /* Ir points. */
  fprintf( dumpFile, "%i\n", colli->no_of_ir_points);
  for ( i=0; i < colli->no_of_ir_points; i++)
    fprintf( dumpFile, "%i %i\n",
	     ((colli->ir_points)[i].x), 
	     ((colli->ir_points)[i].y));
  
  /* Bumper points. */
  fprintf( dumpFile, "%i\n", colli->no_of_bumper_points);
  for ( i=0; i < colli->no_of_bumper_points; i++)
    fprintf( dumpFile, "%i %i\n",
	     ((colli->bumper_points)[i].x), 
	     ((colli->bumper_points)[i].y));
  
  /* External points. */
  fprintf( dumpFile, "%i\n", colli->no_of_external_points);
  for ( i=0; i < colli->no_of_external_points; i++)
    fprintf( dumpFile, "%i %i\n",
	     ((colli->external_points)[i].x), 
	     ((colli->external_points)[i].y));
  
  /* Trajectory. */
  fprintf( dumpFile, "%i %i %i %i\n",
	   ((colli->leftLine).pt1.x), 
	   ((colli->leftLine).pt1.y), 
	   ((colli->leftLine).pt2.x), 
	   ((colli->leftLine).pt2.y));
  
  fprintf( dumpFile, "%i %i %i %i\n",
	   ((colli->rightLine).pt1.x), 
	   ((colli->rightLine).pt1.y), 
	   ((colli->rightLine).pt2.x), 
	   ((colli->rightLine).pt2.y));
  
  fprintf( dumpFile, "%i %i %i %i %i %i\n",
	   ((colli->innerCircle).M.x),
	   ((colli->innerCircle).M.y),
	   ((colli->innerCircle).rad),
	   ((colli->outerCircle).M.x),
	   ((colli->outerCircle).M.y),
	   ((colli->outerCircle).rad));
    
  
  fflush( dumpFile);
}


BOOLEAN
readInformation( COLLI_colli_reply_ptr colli, char* fileName)
{
    int i;
    static BOOLEAN firstTime = TRUE;
    static BOOLEAN oldNumberOfLaserPoints = 0;
    static BOOLEAN oldNumberOfIrPoints = 0;
    static BOOLEAN oldNumberOfBumperPoints = 0;
    static BOOLEAN oldNumberOfExternalPoints = 0;

    static int prevX, prevY, prevRot;
    int positionHasChanged = FALSE;
    
    if ( firstTime) {
      
      if ((dumpFile = fopen( fileName, "r")) == NULL) {
	fprintf( stderr, "WARNING: cannot open dumpFile %s.\n", fileName);
	return 0;
      }
      firstTime = FALSE;
    }

    if (feof (dumpFile))
	return FALSE;

    do {
      fscanf( dumpFile, "%i %i %i",
	      &((colli->rpos).x), &((colli->rpos).y), &(colli->rrot));

      positionHasChanged =
	(prevX != colli->rpos.x) ||
	(prevY != colli->rpos.y) ||
	(prevRot != colli->rrot);

      fscanf( dumpFile, "%i %i %i %i %i",
	      &(colli->tvel), &(colli->rvel), &(colli->dist),
	      &((colli->targetpoint).x), &((colli->targetpoint).y));
      
      prevX = colli->rpos.x;
      prevY = colli->rpos.y;
      prevRot = colli->rrot;

      /* Sonar lines. */
      fscanf( dumpFile, "%i", &(colli->rememberInterval));
      
      for ( i=0; i < bRobot.sonar_cols[0]; i++)
	fscanf( dumpFile, "%i %i %i %i",
		&((colli->sonar_lines)[i].pt1.x), 
		&((colli->sonar_lines)[i].pt1.y), 
		&((colli->sonar_lines)[i].pt2.x), 
		&((colli->sonar_lines)[i].pt2.y));
      
      /* Laser points. */
      fscanf( dumpFile, "%i", &(colli->no_of_laser_points));
      
      if ( colli->no_of_laser_points != oldNumberOfLaserPoints) {
	if ( oldNumberOfLaserPoints != 0)
	  free( colli->laser_points);
	colli->laser_points = ( iPoint*)
	  malloc ( colli->no_of_laser_points * sizeof (struct iPoint));
	oldNumberOfLaserPoints = colli->no_of_laser_points;
      }
      
      for ( i=0; i < colli->no_of_laser_points; i++)
	fscanf( dumpFile, "%i %i",
		&((colli->laser_points)[i].x), 
		&((colli->laser_points)[i].y));
      
      /* Ir points. */
      fscanf( dumpFile, "%i", &(colli->no_of_ir_points));
      
      if ( colli->no_of_ir_points != oldNumberOfIrPoints) {
	if ( oldNumberOfIrPoints != 0)
	  free( colli->ir_points);
	colli->ir_points = ( iPoint*)
	  malloc ( colli->no_of_ir_points * sizeof (struct iPoint));
	oldNumberOfIrPoints = colli->no_of_ir_points;
      }
      
      for ( i=0; i < colli->no_of_ir_points; i++)
	fscanf( dumpFile, "%i %i",
		&((colli->ir_points)[i].x), 
		&((colli->ir_points)[i].y));
      
      /* Bumper points. */
      fscanf( dumpFile, "%i", &(colli->no_of_bumper_points));
      
      if ( colli->no_of_bumper_points != oldNumberOfBumperPoints) {
	if ( oldNumberOfBumperPoints != 0)
	  free( colli->bumper_points);
	colli->bumper_points = ( iPoint*)
	  malloc ( colli->no_of_bumper_points * sizeof (struct iPoint));
	oldNumberOfBumperPoints = colli->no_of_bumper_points;
      }
      
      for ( i=0; i < colli->no_of_bumper_points; i++)
	fscanf( dumpFile, "%i %i",
		&((colli->bumper_points)[i].x), 
		&((colli->bumper_points)[i].y));
      
      /* External points. */
      fscanf( dumpFile, "%i", &(colli->no_of_external_points));
      
      if ( colli->no_of_external_points != oldNumberOfExternalPoints) {
	if ( oldNumberOfExternalPoints != 0)
	  free( colli->external_points);
	colli->external_points = ( iPoint*)
	  malloc ( colli->no_of_external_points * sizeof (struct iPoint));
	oldNumberOfExternalPoints = colli->no_of_external_points;
      }
      
      for ( i=0; i < colli->no_of_external_points; i++)
	fscanf( dumpFile, "%i %i",
		&((colli->external_points)[i].x), 
		&((colli->external_points)[i].y));
      
      /* Trajectory. */
      fscanf( dumpFile, "%i %i %i %i",
	      &((colli->leftLine).pt1.x), 
	      &((colli->leftLine).pt1.y), 
	      &((colli->leftLine).pt2.x), 
	      &((colli->leftLine).pt2.y));
      
      fscanf( dumpFile, "%i %i %i %i",
	      &((colli->rightLine).pt1.x), 
	      &((colli->rightLine).pt1.y), 
	      &((colli->rightLine).pt2.x), 
	      &((colli->rightLine).pt2.y));
      
      fscanf( dumpFile, "%i %i %i %i %i %i",
	      &((colli->innerCircle).M.x),
	      &((colli->innerCircle).M.y),
	      &((colli->innerCircle).rad),
	      &((colli->outerCircle).M.x),
	      &((colli->outerCircle).M.y),
	      &((colli->outerCircle).rad));
      fprintf(stderr, "%d %d\n", onlyMotionMode, positionHasChanged);
    }
    while ( ! feof( dumpFile) && ( onlyMotionMode && ! positionHasChanged) );
    
#ifdef PAPER_OUTPUT    
    {
      static int cnt = 0;

      if ( cnt++ < 2) {
        float factor = -0.8;
        float dx = factor * (colli->outerCircle.M.x - colli->rpos.x);
        float dy = factor * (colli->outerCircle.M.y - colli->rpos.y);

        fprintf( stderr, "%f %f --> %f, %d\n", dx, dy,
                 RAD_TO_DEG(atan2(dy, dx)),
                 colli->rrot);
        colli->outerCircle.M.x = colli->rpos.x + dx;
        colli->outerCircle.M.y = colli->rpos.y + dy;
        colli->outerCircle.rad = (int) sqrt( dx*dx + dy*dy) + 39;
        
        dx = factor * (colli->innerCircle.M.x - colli->rpos.x);
        dy = factor * (colli->innerCircle.M.y - colli->rpos.y);
        
        colli->innerCircle.M.x = colli->rpos.x + dx;
        colli->innerCircle.M.y = colli->rpos.y + dy;
        colli->innerCircle.rad = (int) sqrt( dx*dx + dy*dy) - 43;
      }
    }
#endif
    return !onlyMotionMode || positionHasChanged;
}



/* Reads information from a dumpFile and calls the ordinary
 * reply handler.
 */
void
replay( char* fileName)
{
    TCX_REF_PTR           ref = NULL;
    COLLI_colli_reply_type colli;

    int frameNumber = 0;
    
    /* No support for vision and arm information. */
    colli.no_of_external_points = 0;
    colli.innerArmPoint.x     = I_ERROR;
    colli.outerArmPoint.x     = I_ERROR;
    colli.armCircle.M.x       = I_ERROR;
    
    while ( readInformation( &colli, fileName)) {
      COLLI_colli_reply_handler( ref, &colli);
      if ( waitMode) {
	fprintf( stderr, "Frame no. %d (return for next frame).", frameNumber++);
	getchar();
      }
      else if (0) 
	usleep( 500000);
    }
    
    fprintf( stderr, "Last frame reached.\n");
    getchar();
    fclose( dumpFile);
    exit(1);
}
    
/************************************************************************
 *
 *                 *** TCX HANDLER ***
 *
 *   NAME:         COLLI_colli_reply
 *                 
 *   FUNCTION:     handles a message of that type
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void
COLLI_colli_reply_handler( TCX_REF_PTR           ref,
                          COLLI_colli_reply_ptr colli)
{
  int i, j;
  static BOOLEAN first_time = TRUE;
  static int next_CollLine_reading;

  if (first_time) {
    first_time = FALSE;

    next_CollLine_reading = 0;
    COLLISION_REMEMBER_INTERVAL = colli->rememberInterval;

    iCollLines = (struct iLine **) malloc(bRobot.sonar_cols[0] *
                                          sizeof(struct iLine *));
    for (i=0; i<bRobot.sonar_cols[0]; i++) {
      iCollLines[i] = (struct iLine *) 
        malloc( COLLISION_REMEMBER_INTERVAL * sizeof(struct iLine));
      
      for (j=0; j<COLLISION_REMEMBER_INTERVAL; j++)
        iCollLines[i][j].pt1.x= iCollLines[i][j].pt1.y = 
          iCollLines[i][j].pt2.x= iCollLines[i][j].pt2.y = 0.0;
    }
  }
  
  if ( (!replayMode) && (dumpMode) )
    dumpInformation( colli, "dumpFile");
  
  rrot = (float) DEG_TO_RAD( (float)colli->rrot);
  rpos.x = colli->rpos.x;
  rpos.y = colli->rpos.y;
  
  for (i = 0; i < bRobot.sonar_cols[0]; i++)
    iCollLines[i][next_CollLine_reading] = (colli->sonar_lines)[i];
    
  if (displayGraphics) {
    EZX_SetLineWidth(THICK);
    EZX_ClearWindow(robwin);
    EZX_Flush();
    
    if (colli->rectangularRobot)
      DrawRectangularRobot( rpos, colli->robotLines, colli->testLines);
    else 
      DrawRobot( rpos, rrot);
    
#ifndef PAPER_OUTPUT
    DrawInfoLine( rpos, colli->tvel, colli->dist);
#endif
    
    DrawTarget( rpos, colli->targetpoint);

    /* Draw the trajectory. */

    if ( colli->innerCircle.M.x != I_ERROR)
      DrawCircleTrajectory(rpos, colli->innerCircle, colli->outerCircle,
			   colli->targetpoint);
    else if ( colli->leftLine.pt1.x != I_ERROR)
      DrawStraightTrajectory( rpos, colli->leftLine, colli->rightLine);
    
    /* Draw information about the arm. */
    DrawArmInformation( rpos,
                        colli->armCircle,
                        colli->innerArmPoint, colli->outerArmPoint);
    
    /* Draw the collision lines. */
    DrawLaserCollPoints( colli->no_of_laser_points, colli->laser_points);
    DrawCollLines();
    DrawExternalCollPoints(colli->no_of_external_points, colli->external_points);
    DrawBumperCollPoints(colli->no_of_bumper_points, colli->bumper_points);
    DrawIrCollPoints(colli->no_of_ir_points, colli->ir_points);

  }
  
  next_CollLine_reading = (next_CollLine_reading+1) % COLLISION_REMEMBER_INTERVAL;
  
  if ( ! replayMode)
      tcxFree("COLLI_colli_reply", colli);
}







/************************************************************************
 *
 *   NAME:         USER_close_handler
 *                 
 *   FUNCTION:     handles a close message (special case)
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void COLLGRAPH_close_handler(char *name, TCX_MODULE_PTR module)
{
#ifdef USER_debug
  fprintf(stderr, "COLLGRAPH: closed connection detected: %s\n", name);
#endif

  if ( dumpFile != NULL)
    fflush( dumpFile);

  if (COLLI != NULL) {
    if (!strcmp(name, tcxModuleName(COLLI))) {
      COLLI = NULL;
    }
  }

  if (!strcmp(name, "TCX Server")){ /* TCX shut down */
    exit(0);
  }
  else
    fprintf( stderr, "Closed connection detected: %s\n", name);
}


/************************************************************************
 *
 *   NAME:         init_graphics()
 *                 
 *   FUNCTION:     initializes a simple graphics window
 *                 
 *   PARAMETERS:   standard TCX handler parameters
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/

void init_graphics( char* robName)
{
  if ( robName != NULL)
    robwin = EZX_MakeWindow(robName, W_SIZE, W_SIZE, "+0+0"); 
  else
    robwin = EZX_MakeWindow("collision", W_SIZE, W_SIZE, "+0+0"); 
  
  EZX_InitDefaultColors();
  EZX_UseFont(theGC, "lucidasans-18");
  EZX_SetWindowBackground( robwin, C_WHITE);
  EZX_Flush();
}




/************************************************************************
 *
 *   NAME:         init_tcx( char *robot_name)
 *                 
 *   FUNCTION:     initializes TCX communication and connects
 *                 to the BASE.
 *                 
 *   PARAMETERS:   standard TCX handler parameters, name of the robot
 *                 
 *   RETURN-VALUE: 
 *                 
 ************************************************************************/


void init_tcx(char *robot_name)
{
  char tcxName[128], hostname[80], pidname[80];
  char *tcxMachine = NULL;
  
  TCX_REG_MSG_TYPE TCX_message_array[] = {
    BASE_messages,
    SONAR_messages,
    LASER_messages,
    IR_messages,
    COLLI_messages
  };

  if ( robot_name != NULL)
    tcxSetModuleNameExtension( robot_name);

  tcxMachine = getenv("TCXHOST");   

  if (!tcxMachine) {
    fprintf(stderr, "TCXHOST is not set.  Assuming 'localhost'\n");
    tcxMachine="localhost";
  }
  
  strcpy(tcxName, TCX_USER_MODULE_NAME);
  sprintf(pidname, "_%d_", getpid());
  strcat(tcxName, pidname);
  gethostname(hostname, 32);
  strcat(tcxName, hostname);
      
  tcxInitialize( tcxName, (void *) tcxMachine);
  check_version_number(BEESOFT_VERSION_MAJOR, BEESOFT_VERSION_MINOR,
		       BEESOFT_VERSION_ROBOT_TYPE, BEESOFT_VERSION_DATE,
		       NULL, 0);
  check_version_number(libezx_major, libezx_minor,
		       libezx_robot_type, libezx_date,
		       "libezx", 0);
  check_version_number(librobot_major, librobot_minor,
		       librobot_robot_type, librobot_date,
		       "librobot", 1);
  
  fprintf(stderr, "done.\n");

  tcxRegisterMessages( TCX_message_array, sizeof(TCX_message_array)
		      / sizeof(TCX_REG_MSG_TYPE));

  tcxRegisterHandlers( BASE_reply_handler_array,
		      sizeof(BASE_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers( SONAR_reply_handler_array,
		      sizeof(SONAR_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers( LASER_reply_handler_array,
		      sizeof(LASER_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers( IR_reply_handler_array,
		      sizeof(IR_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterHandlers( COLLI_reply_handler_array,
		      sizeof(COLLI_reply_handler_array)
		      / sizeof(TCX_REG_HND_TYPE));

  tcxRegisterCloseHnd(COLLGRAPH_close_handler);

  fprintf(stderr, "Connecting to %s...", TCX_BASE_MODULE_NAME);
  COLLI = tcxConnectModule( TCX_BASE_MODULE_NAME);
  fprintf(stderr, "done.\n");
  
  data.subscribe_status_report = 0;
  data.subscribe_sonar_report  = 0;
  data.subscribe_laser_report  = 0;
  data.subscribe_colli_report  = colliSubscribe;

  tcxSendMsg(COLLI, "BASE_register_auto_update", &data);
}

/************************************************************************
 *
 *   NAME:         interrupt_handler()
 *                 
 *   FUNCTION:     some ^C signal or kill command arrived
 *                 
 *   PARAMETERS:   int sig           signal
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void interrupt_handler(int sig)
{
    fprintf(stderr, "Thanks for making a stupid program mildly content.\n");
    fflush(stderr);
    exit(0);
}




/************************************************************************
 *
 *   NAME:         main 
 *                 
 *   FUNCTION:     main loop - checks for tcx events
 *                 
 *   PARAMETERS:   none
 *                 
 *   RETURN-VALUE: none
 *                 
 ************************************************************************/

void main(int argc, char *argv[])
{
  struct timeval TCX_waiting_time = {1, 0};
  int auto_reply_position =  1;
  int i;
  char *robot_name = NULL;
  
  struct bParamList * params = NULL;

  /* add the parameter file */
  params = bParametersAddFile(params, "etc/beeSoft.ini");

  /* add some enviroment variables */
  params = bParametersAddEnv(params, "", "TCXHOST");

  /* add command line arguements */
  params = bParametersAddArray(params, "", argc, argv);

  /* This puts the information in the global bRobot struct. */
  bParametersFillParams(params);

  for (i=1; i<argc; i++) {
    if ((strcmp(argv[i],"-replay")==0))
      replayMode = TRUE;
    else if ((strcmp(argv[i],"-nowait")==0))
      waitMode = FALSE;
    else if ((strcmp(argv[i],"-dump")==0))
      dumpMode = TRUE;
    else if ((strcmp(argv[i],"-nomotion")==0))
      onlyMotionMode = FALSE;
    else if ((strcmp(argv[i],"-subscribe")==0))
      if ( argc < i + 2)
	fprintf ( stderr, "Need a number following keyword \"subscribe\"\n");
      else
	colliSubscribe = atoi( argv[++i]);
    else if ((strcmp(argv[i],"-nodisplay")==0))
      displayGraphics = FALSE;
    else if ((strcmp(argv[i],"-robot")==0))
      if ( argc < i + 2)
	fprintf ( stderr, "Need a name following keyword \"robot\"\n");
      else
	robot_name = argv[++i];
    else if ((strcmp(argv[i],"-size")==0))
      if ( argc < i + 2)
	fprintf ( stderr, "Need an integer following keyword \"size\"\n");
      else
	W_SIZE = atoi(argv[++i]);
    else {
       fprintf(stderr,
	      "\nusage: COLLGRAPH [-replay -nowait -nodisplay -subscribe -robot <robName> -size <winSize> <n>]\n");
      exit(-1);
     }
  }

  if ( displayGraphics)
    init_graphics( robot_name);

  /* In this mode the data will be read from a file. */
  if ( replayMode)
    replay( "dumpFile");
  
  init_tcx(robot_name);	

  signal(SIGTERM, &interrupt_handler); /* kill interupt handler */
  signal(SIGINT,  &interrupt_handler); /* control-C interupt handler */

  for (;;){
    if (COLLI == NULL){
      fprintf(stderr, "Connecting to %s...", TCX_BASE_MODULE_NAME);
      COLLI = tcxConnectModule(TCX_BASE_MODULE_NAME);
      fprintf(stderr, "done.\n");
      tcxSendMsg(COLLI, "BASE_register_auto_update", &data);
    }
    else 
      block_wait( NULL, 1, displayGraphics);
    
    tcxRecvLoop((void *) &TCX_waiting_time);
  }

  exit(0);			/* should never reach here! */
}

/* Some ugliness to get ezx to work properly */
void check_redraw_screen() {}
