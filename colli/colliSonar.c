
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliSonar.c,v $
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
 * $Log: colliSonar.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.23  1999/10/02 09:06:42  thrun
 * New robot type: XR4000.
 *
 * Revision 1.22  1999/09/24 14:29:26  fox
 * Added support for scout robot.
 *
 * Revision 1.21  1999/09/08 21:31:57  fox
 * *** empty log message ***
 *
 * Revision 1.20  1999/09/08 21:11:50  fox
 * *** empty log message ***
 *
 * Revision 1.19  1999/06/25 19:48:08  fox
 * Minor changs for the urbie.
 *
 * Revision 1.18  1999/03/09 16:43:59  wolfram
 * Fixed a bug
 *
 * Revision 1.17  1999/03/09 15:48:38  wolfram
 * Added geometry of PIONEER_II
 *
 * Revision 1.16  1999/02/23 09:39:16  schulz
 * Added support for the Pioneer II robot. Use option -pioneer2
 *
 * Revision 1.15  1998/11/08 20:27:50  fox
 * FIXED A BUG FOR RECTANGULAR ROBOTS (DON'T ASK).
 *
 * Revision 1.14  1998/10/30 18:20:49  fox
 * Added support for pioniers.
 *
 * Revision 1.13  1998/10/23 20:50:30  fox
 * *** empty log message ***
 *
 * Revision 1.12  1998/08/29 21:50:02  fox
 * Adapted exception handling in order to deal with people around the robot.
 *
 * Revision 1.11  1998/08/26 23:23:41  fox
 * Final version before I changed exception handling.
 *
 * Revision 1.10  1998/08/18 16:24:24  fox
 * Added support for b18 robot.
 *
 * Revision 1.9  1998/05/13 07:07:40  fox
 * Fixed some bugs I found due to graphical output.
 *
 * Revision 1.8  1997/07/17 17:31:46  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.7  1997/04/26 13:56:18  fox
 * Added targetDefined, targetX, and targetY to status report.
 *
 * Revision 1.6  1997/03/25 21:44:42  tyson
 * Many bug fixes.
 *
 * Revision 1.5  1997/02/28 12:57:09  fox
 * Minor changes.
 *
 * Revision 1.4  1997/02/11 17:59:26  fox
 * Don't use sonar in FIND_DOOR_MODE.
 *
 * Revision 1.3  1997/02/04 18:00:33  fox
 * Added paramters to BASE_setMode.
 *
 * Revision 1.2  1997/02/02 22:32:28  tyson
 * added colli client library, B14 support, misc cleanup and bug fixes.  Sebastian and Tyson
 *
 * Revision 1.1.1.1  1996/09/22 16:46:04  rhino
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


#include <bUtils.h>
#include "collisionIntern.h"

#define SONAR_RANGE_IN_ARM_DIRECTION 100.0

float SONAR_UPDATE_TIME = 1.1;

float *SonarAngle;
int* considerSonar;

float B18SonarOffsetForward[] = {-0.1223, 0.2915, 0.3645, 0.4269, 0.4509, 0.4639,
				 0.4743, 0.4639, 0.4509, 0.4269, 0.3645, 0.2915,
				 -0.1223, -0.0662, -0.0331, 0.2053, 0.2834, 0.3382,
				 0.3665, 0.3665, 0.3382, 0.2834, 0.2053, -0.0331, -0.0662};

float B18SonarOffsetSideward[] = { 0.1990, 0.2387, 0.2301, 0.2046, 0.1408, 0.0696,
				   0.0, -0.0696, -0.1408, -0.2046, -0.2301, -0.2387,
				   -0.1990,0.1460,0.1988, 0.2055, 0.1867, 0.1273,
				   0.0498, -0.0498, -0.1273, -0.1867, -0.2055,-0.1988,-0.1460};

float B18SonarAngle[] = { 180.0, 90.0, 72.0, 54.0, 36.0, 18.0,
			  0.0,-18.0,-36.0,-54.0,-72.0,-90.0,
			  -180.0, 180.0, 95.0, 85.0, 60.0, 35.0,
			  10.0, -10.0, -35.0, -60.0, -85.0, -95.0, -180.0};

/* Added 10cm to values of hand book. */
float PioneerATSonarOffsetForward[] = {20.0, 22.0, 23.0, 23.0, 23.0, 22.0, 20.0};
float PioneerATSonarOffsetSideward[] = { 10.0, 8.0, 4.0, 0.0, -4.0, -8.0, -10.0};
float PioneerATSonarAngle[] = { 90.0, 30.0, 15.0, 0.0, -15.0, -30.0, -90.0};

float PioneerIISonarOffsetForward[] = {11.5, 15.5, 19.0, 21.0, 21.0, 19.0, 15.5, 11.5,
                                     -11.5, -15.5, -19.0, -21.0, -21.0, -19.0, -15.5, -11.5}; 

float PioneerIISonarOffsetSideward[] = { 13.0, 11.5, 8.0, 2.5, -2.5, -8.0, -11.5, -13.0,
					 -13.0, -11.5, -8.0, -2.5, 2.5, 8.0, 11.5, 13.0};

float PioneerIISonarAngle[] = { 90.0, 50.0, 30.0, 10.0, -10.0, -30.0, -50.0, -90.0,
                              -90.0, -130.0, -150.0, -170.0, 170.0, 150.0, 130.0, 90};

float UrbanSonarOffsetForward[] = { 22.07, 21.52, 6.27, -9.73, -17.65, -17.65, -9.73, 6.27, 21.52, 22.07};

float UrbanSonarOffsetSideward[] = { 4.39, 9.34, 17.71, 17.71, 17.71, -17.71, -17.71, -17.71, -9.34, -4.39};

float UrbanSonarAngle[] = { 3.0, 11.0, 81.0, 90.0, 99.0, -99.0, -90.0, -81.0, -11.0, -3.0};


float *SonarOffsetForward;
float *SonarOffsetSideward;
float *SonarAngle;

void 
dontConsiderSonar(int number)
{
  int i;

  if ( number >= 0 && number < bRobot.sonar_cols[0])
    considerSonar[number] = FALSE;
  else if (use_laser) {
    for (i = 0; i < bRobot.sonar_cols[0]; i++)
      considerSonar[i] = FALSE;
  }
}

void 
reconsiderSonar(int number)
{
  int i;

  if ( number > 0 && number < bRobot.sonar_cols[0])
    considerSonar[number] = TRUE;
  else {
    for (i = 0; i < bRobot.sonar_cols[0]; i++)
      considerSonar[i] = TRUE;
  }
}

/**********************************************************************/
void
init_SonarStructs(void)
{
  float tmp;
  int i;

  for ( i = 0; i < bRobot.sonar_cols[0]; i++) 
    considerSonar[i] = TRUE;

  /* The angle of the first sonar sensor has changed. 
     SonarAngle[0] = DEG_TO_RAD(360.0 - 1.0); */
  if ( robotShape == ROUND_ROBOT) {

    if (use_simulator || robotType == B21_ROBOT || robotType == B14_ROBOT) {
      SonarAngle[0] = DEG_TO_RAD(360.0 - 7.5);
      tmp = DEG_TO_RAD(360.0 / ((float) bRobot.sonar_cols[0]));
      for (i=1; i<NO_OF_SONARS; i++)
	SonarAngle[i] = (SonarAngle[i-1] - tmp);
    }
    else if (robotType == SCOUT_ROBOT) {
      fprintf( stderr, "Hmm, this could be either a B14 or a scout.\n");
      SonarAngle[0] = 0.0;
      tmp = DEG_TO_RAD(360.0 / ((float) bRobot.sonar_cols[0]));
      for (i=1; i<NO_OF_SONARS; i++)
	SonarAngle[i] = (SonarAngle[i-1] + tmp);
      if ( use_laser) {
	int son = 0;
	fprintf( stderr, "Use the laser range-finder. Switch off some off the sonars.\n");
	for ( son = 0; son < bRobot.sonar_cols[0]; son++)
	  if ( son < 4 || son > 12)  
	    /* 	  if ( son > 4 && son < 12)  */
	    dontConsiderSonar(son);
      }
    }
    else if (robotType == XR4000_ROBOT) {
      fprintf(stderr, "Hey, since when do we run BeeSoft on those robots?.\n");
      SonarAngle[0] = 0.0;
      /* currently doesn't deal correctly with 48 sonars */
      fprintf(stderr, "### NUMBER OF SONARS: %d ###\n",
	      bRobot.sonar_cols[0]);	      
      tmp = DEG_TO_RAD(360.0 / 24.0);
      for (i=1; i<NO_OF_SONARS; i++){
	SonarAngle[i] = (SonarAngle[i-1] + tmp);
	for (; SonarAngle[i] >= 2.0 * M_PI;) SonarAngle[i] -= 2.0 * M_PI;
      }
    }
  }
  else if ( robotType == B18_ROBOT) {
    fprintf( stderr, "Congratulations! You're the happy owner of a B18!!\n");
    SonarAngle = B18SonarAngle;
    for ( i = 0; i < bRobot.sonar_cols[0]; i++) {
      SonarAngle[i] = DEG_TO_RAD( SonarAngle[i]);
      B18SonarOffsetSideward[i] *= 100.0;
      B18SonarOffsetForward[i] *= 100.0;
    }
    considerSonar[12] = considerSonar[13] = FALSE;
    considerSonar[9] = considerSonar[13] = FALSE;
    considerSonar[7] = considerSonar[6] = FALSE;

    SonarOffsetSideward = B18SonarOffsetSideward;
    SonarOffsetForward = B18SonarOffsetForward;
  }  
  else if ( robotType == PIONEER_ATRV || robotType == PIONEER_II) {
    fprintf( stderr, "Congratulations! You're the happy owner of a Pioneer!!\n");
    if(robotType == PIONEER_ATRV) {
      SonarOffsetForward = PioneerATSonarOffsetForward;
      SonarOffsetSideward = PioneerATSonarOffsetSideward;
      SonarAngle = PioneerATSonarAngle;
    }
    else {
      SonarOffsetForward = PioneerIISonarOffsetForward;
      SonarOffsetSideward = PioneerIISonarOffsetSideward;
      SonarAngle = PioneerIISonarAngle;      
    }
    for ( i = 0; i < bRobot.sonar_cols[0]; i++) {
      SonarAngle[i] = DEG_TO_RAD( SonarAngle[i]);
    }
  }
  else if ( robotType == URBAN_ROBOT) {
    fprintf( stderr, "Congratulations! You're the happy owner of an Urbie!!\n");
    SonarAngle = UrbanSonarAngle;
    for ( i = 0; i < bRobot.sonar_cols[0]; i++) {
      SonarAngle[i] = normed_angle(DEG_TO_RAD( SonarAngle[i]));
    }
    SonarOffsetForward = UrbanSonarOffsetForward;
    SonarOffsetSideward = UrbanSonarOffsetSideward;
  }  
  else {
    fprintf(stderr, "COLLI is not configured for a robot with %d sonars\n",
	    bRobot.sonar_cols[0]);
    exit(1);
  }
}



/**********************************************************************
 * Given the position of the robot and the distance of a particular 
 * sonar the collision line representing this reading is given back.
 **********************************************************************/
LineSeg
sonar_to_lineseg( Point rpos, float rrot, 
		  int sonar_no, float dist)
{
  Point sonar_pos,  sonar_left_edge, sonar_right_edge;
  LineSeg line;
  float sonar_rot, sonar_plain, open_angle, tmp, start_angle;

  open_angle = SONAR_OPEN_ANGLE;

  /* If the line is too far away we don't consider it. */ 
  if (dist > 0.0 && dist <= MAX_RANGE) {
    
    /* We want to limit the length of the lineseg to MAX_COLLISION_LINE_LENGTH */
    if (dist > EPSILON && ACTUAL_MODE->max_collision_line_length > 0.0)
	if ((tmp = 0.5 * atan(ACTUAL_MODE->max_collision_line_length / dist))
	    < open_angle)
	  open_angle = tmp;
    
    dist /= fcos(open_angle);
    
    if ( robotShape == ROUND_ROBOT) {
      
      if ( armState == INSIDE) {
	
	/* If the robot is very fast we don't consider the lines behind it. 
	 * If the current tvel is 100.0 we only use lines starting at 90.0 
	 * degrees. This is only allowed if the arm is inside.
	 */
	tmp = (float) rwi_base.trans_current_speed * 0.01;
	
	start_angle = DEG_180 - tmp * DEG_90; 
	
	if ( SonarAngle[sonar_no] > start_angle && 
	     SonarAngle[sonar_no] < DEG_360 - start_angle) {
	  line.pt1.x = F_ERROR;
	  return(line);      
	}
      }
      
      sonar_rot = rrot + SonarAngle[sonar_no];
      norm_angle(&sonar_rot);
      
      sonar_plain = sonar_rot + DEG_90;
      
      sonar_pos.x = rpos.x + (fcos( sonar_rot) * ROB_RADIUS);
      sonar_pos.y = rpos.y + (fsin( sonar_rot) * ROB_RADIUS);
      
      sonar_left_edge.x = sonar_pos.x + (fcos( sonar_plain) * COLLI_SONAR_RADIUS);
      sonar_left_edge.y = sonar_pos.y + (fsin( sonar_plain) * COLLI_SONAR_RADIUS);
      sonar_right_edge.x = sonar_pos.x - (fcos( sonar_plain) * COLLI_SONAR_RADIUS);
      sonar_right_edge.y = sonar_pos.y - (fsin( sonar_plain) * COLLI_SONAR_RADIUS);
      
      line.pt1.x = sonar_left_edge.x + fcos( sonar_rot + open_angle) * dist;
      line.pt1.y = sonar_left_edge.y + fsin( sonar_rot + open_angle) * dist;
      line.pt2.x = sonar_right_edge.x + fcos( sonar_rot - open_angle) * dist;
      line.pt2.y = sonar_right_edge.y + fsin( sonar_rot - open_angle) * dist;
    }
    
    /* It's a rectangular robot. */
    else if (robotType == B18_ROBOT ||
	     robotType == PIONEER_ATRV || robotType == PIONEER_II ||
	     robotType == URBAN_ROBOT) {
      
      sonar_rot = rrot + SonarAngle[sonar_no];
      norm_angle(&sonar_rot);
      
      sonar_plain = sonar_rot + DEG_90;
      
      sonar_pos.x =
	rpos.x + fcos(rrot) * SonarOffsetForward[sonar_no] +
	( fcos( rrot + DEG_90) * SonarOffsetSideward[sonar_no]);
      
      sonar_pos.y =
	rpos.y + fsin( rrot) * SonarOffsetForward[sonar_no] +
	( fsin( rrot + DEG_90) * SonarOffsetSideward[sonar_no]);

      
/*       fprintf( stderr, "%d: %f %f %f %f\n", sonar_no, dist, SonarOffsetForward[sonar_no],  */
/* 	       SonarOffsetSideward[sonar_no], RAD_TO_DEG(sonar_plain));  */
      
      sonar_left_edge.x = sonar_pos.x + (fcos( sonar_plain) * COLLI_SONAR_RADIUS);
      sonar_left_edge.y = sonar_pos.y + (fsin( sonar_plain) * COLLI_SONAR_RADIUS);
      sonar_right_edge.x = sonar_pos.x - (fcos( sonar_plain) * COLLI_SONAR_RADIUS);
      sonar_right_edge.y = sonar_pos.y - (fsin( sonar_plain) * COLLI_SONAR_RADIUS);
      
      line.pt1.x = sonar_left_edge.x + fcos( sonar_rot + open_angle) * dist;
      line.pt1.y = sonar_left_edge.y + fsin( sonar_rot + open_angle) * dist;
      line.pt2.x = sonar_right_edge.x + fcos( sonar_rot - open_angle) * dist;
      line.pt2.y = sonar_right_edge.y + fsin( sonar_rot - open_angle) * dist;
    }
    else {
      fprintf( stderr, "Sorry. Only B21, B18, Pioneer, and Urban supported in sonar_to_lineseg .\n");
      exit(0);
    }
    
    /* We want the point with the smaller x value (if possible) to be pt1 */
    if (smaller( line.pt2, line.pt1))
      swap(&line);
    
    return(line);
  }
  else {
    line.pt1.x = F_ERROR;
  }
    
  return(line);
}




/**********************************************************************
 * In the case that the robot has to move backward we just swap the
 * sonar information such that the rear sonars seem to be at the front
 * of the robot (the base then only has to change the sign of the
 * velocities.
 **********************************************************************/
void 
update_CollLines(Pointer callback_data, Pointer client_data)
{
  int i, tmp;
  int *sonar_no;
  float rrot;
  Point rpos;
  int offsetForSonarPoints =
    next_CollLine_reading * POINTS_PER_SONAR * bRobot.sonar_cols[0];
  int currentPoint;

  if ( use_sonar) {

    if (use_simulator || robotType == B21_ROBOT) {

#ifdef B21
      /* If the arm is outside we can't use these sonars. */
      if (armState != INSIDE) {
	sonar_readings[0] = sonar_readings[23] = SONAR_RANGE_IN_ARM_DIRECTION;
	sonar_readings[1] = sonar_readings[22] = SONAR_RANGE_IN_ARM_DIRECTION;
      }
#endif

      rpos.x = ((float) rwi_base.pos_x + sonar_start_pos_x) * 0.5;
      rpos.y = ((float) rwi_base.pos_y + sonar_start_pos_y) * 0.5;
      
      if ((sonar_start_rot < 90.0 && rwi_base.rot_position > 270.0) ||
	  (sonar_start_rot > 270.0 && rwi_base.rot_position < 90.0))
	rrot = (rwi_base.rot_position + 360.0 + sonar_start_rot) * 0.5;
      else
	rrot = (rwi_base.rot_position + sonar_start_rot) * 0.5;
      
      rrot   = normed_angle( (float) DEG_TO_RAD(90.0 -  rrot));
      
      sonar_no = (int *) callback_data;
      
      for (i=0; i<4; i++) {
	if(( tmp = sonar_no[i]) >= 0) {
	  CollLines[tmp][next_CollLine_reading] = 
	    sonar_to_lineseg(rpos, rrot, tmp, sonar_readings[tmp]);
	  /* Update the point description of these readings as well. */
	  currentPoint = offsetForSonarPoints + tmp * POINTS_PER_SONAR;
	  combinedObstaclePoints[SONAR_POINTS].points[currentPoint] =
	    CollLines[tmp][next_CollLine_reading].pt1;
	  combinedObstaclePoints[SONAR_POINTS].points[currentPoint+1] =
	    CollLines[tmp][next_CollLine_reading].pt2;
	}
	else {
	  fprintf(stderr, "sonar reading not ok.\n");
	}
      }
    }
    
    /* It's a B18. */
    else if (robotType == B18_ROBOT ||
	     robotType == PIONEER_ATRV || robotType == PIONEER_II ||
	     robotType == URBAN_ROBOT || robotType == SCOUT_ROBOT ||
	     robotType == XR4000_ROBOT) {

#define MIN_DIST_OF_SONAR 10.0
#define MAX_DIST_OF_SONAR 500.0
#define REPLACEMENT_MIN_DIST 5.0

      
      updateActualPosition( &rpos, &rrot, DONT_CONSIDER_DIRECTION);

      for (i=0; i<bRobot.sonar_cols[0]; i++) {

 	if ( sonar_readings[i] > MAX_DIST_OF_SONAR || ! considerSonar[i]) {
	  CollLines[i][next_CollLine_reading].pt1.x = F_ERROR;
	  CollLines[i][next_CollLine_reading].pt2.x = F_ERROR;
	}
	else {

	  if ( sonar_readings[i] <= MIN_DIST_OF_SONAR) {
	    sonar_readings[i] = REPLACEMENT_MIN_DIST;
	  }

	  CollLines[i][next_CollLine_reading] = 
	    sonar_to_lineseg(rpos, rrot, i, sonar_readings[i]);
	}

	/* Update the point description of these readings as well. */
	currentPoint = offsetForSonarPoints + i * POINTS_PER_SONAR;
	combinedObstaclePoints[SONAR_POINTS].points[currentPoint] =
	  CollLines[i][next_CollLine_reading].pt1;
	combinedObstaclePoints[SONAR_POINTS].points[currentPoint+1] =
	  CollLines[i][next_CollLine_reading].pt2;
      }
    }
    else {
      fprintf( stderr, "Sorry. Only B21, B18, Pioneer, Urban,  Scout, and XR4000 supported in update_CollLines.\n");
      exit(0);
    }
    
    gettimeofday(&Last_CollLine_Update, 0);
  }
  
  COLLI_update_tcx_CollLines();
  /* COLLI_update_tcx_ExternalObstaclePoints();  */
}





