
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/robot/corr.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:55:27 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: corr.c,v $
 * Revision 1.1  2002/09/14 15:55:27  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1998/08/11 23:05:40  thrun
 * .
 *
 * Revision 1.1  1998/08/11 23:03:48  thrun
 * corr is now a general library
 *
 * Revision 1.2  1998/08/11 22:05:26  thrun
 * New correction parameters - experimental version.
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
#include <math.h>
#include <sys/time.h>

#define MAX_RANDOM 1073741823.5
#define RAND() ((((float) random())/MAX_RANDOM)-1.0)
#define RAND_POS() ((((float) random())/MAX_RANDOM)/2.0)





/************************************************************************
 *
 *   NAME:         compute_forward_correction
 *                 
 *   FUNCTION:     Computes a correction to robot position estimate
 *                 
 *   PARAMETERS:   robot_x, robot_y, robot_orientation  non-corrected
 *                                                      robot pos
 *                 corr_x, corr_y, corr_angle           corretion parameters
 *                 corr_type                            ditto
 *                 *corr_robot_x, *corr_robot_y, *corr_orientation
 *                                                      corrected position
 *
 *   RETURN-VALUE: nonw
 *                 
 ************************************************************************/

void 
compute_forward_correction(float robot_x, 
			   float robot_y, 
			   float robot_orientation,
			   float corr_x, 
			   float corr_y, 
			   float corr_angle, /* in deg. */
			   int   corr_type,
			   float *corr_robot_x,
			   float *corr_robot_y, 
			   float *corr_robot_orientation)
{
  float corr_angle_2pi;

  *corr_robot_orientation = robot_orientation - corr_angle;
  for (; *corr_robot_orientation <= -180.0;) *corr_robot_orientation += 360.0;
  for (; *corr_robot_orientation >   180.0;) *corr_robot_orientation -= 360.0;

  corr_angle_2pi = corr_angle * M_PI / 180.0;

  *corr_robot_x = ((robot_x - corr_x) * cos(corr_angle_2pi))
    + ((robot_y - corr_y) * sin(corr_angle_2pi));

  *corr_robot_y = ((robot_y - corr_y) * cos(corr_angle_2pi))
    - ((robot_x - corr_x) * sin(corr_angle_2pi));
}


/************************************************************************
 *
 *   NAME:         compute_backward_correction
 *                 
 *   FUNCTION:     Computes a BASE robot position estimate
 *                 based on a corrected position
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: nonw
 *                 
 ************************************************************************/



void 
compute_backward_correction(float corr_robot_x, 
			    float corr_robot_y, 
			    float corr_robot_orientation,
			    float corr_x, 
			    float corr_y, 
			    float corr_angle, /* in deg. */
			    int   corr_type,
			    float *robot_x,
			    float *robot_y, 
			    float *robot_orientation)
{
  float corr_angle_2pi;

  *robot_orientation  = corr_angle + corr_robot_orientation;
  for (; *robot_orientation <= -180.0;) *robot_orientation += 360.0;
  for (; *robot_orientation >   180.0;) *robot_orientation -= 360.0;

  corr_angle_2pi = corr_angle * M_PI / 180.0;

  *robot_x = corr_x + (corr_robot_x * cos(corr_angle_2pi))
    - (corr_robot_y * sin(corr_angle_2pi));

  *robot_y = corr_y + (corr_robot_y * cos(corr_angle_2pi))
    + (corr_robot_x * sin(corr_angle_2pi));
}  



/************************************************************************
 *
 *   NAME:         compute_correction_parameters
 *                 
 *   FUNCTION:     Computes a set of correction parameters
 *                 from an uncorrected and a corrected position
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: nonw
 *                 
 ************************************************************************/



void 
compute_correction_parameters(float robot_x, 
			      float robot_y, 
			      float robot_orientation,
			      float corr_robot_x,
			      float corr_robot_y, 
			      float corr_robot_orientation,
			      float *corr_x, 
			      float *corr_y, 
			      float *corr_angle,
			      int   *corr_type)
{
  float corr_angle_2pi;

  *corr_angle = robot_orientation - corr_robot_orientation;
  for (; *corr_angle <= -180.0;) *corr_angle += 360.0;
  for (; *corr_angle >   180.0;) *corr_angle -= 360.0;

  corr_angle_2pi = *corr_angle * M_PI / 180.0;

  *corr_x     = robot_x - (corr_robot_x * cos(corr_angle_2pi))
    + (corr_robot_y * sin(corr_angle_2pi));

  *corr_y     = robot_y - (corr_robot_y * cos(corr_angle_2pi))
    - (corr_robot_x * sin(corr_angle_2pi));
}


/************************************************************************
 *
 *   NAME:         update_correction_parameters
 *                 
 *   FUNCTION:     Updates a set of correction parameters
 *                 based on a previous set, and a local change
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: nonw
 *                 
 ************************************************************************/




void 
update_correction_parameters(float corr_robot_x, 
			     float corr_robot_y, 
			     float corr_robot_orientation,
			     float delta_x,
			     float delta_y,
			     float delta_orientation,
			     float *corr_x, 
			     float *corr_y, 
			     float *corr_angle,
			     int   *corr_type)
{
  float robot_x, robot_y, robot_orientation;
  float corr_robot_x2, corr_robot_y2, corr_robot_orientation2;
  

  compute_backward_correction(corr_robot_x, corr_robot_y, 
			      corr_robot_orientation,
			      *corr_x, *corr_y, *corr_angle, *corr_type,
			      &robot_x, &robot_y,  &robot_orientation);
  
  corr_robot_x2 = corr_robot_x + delta_x;
  corr_robot_y2 = corr_robot_y + delta_y;
  corr_robot_orientation2 = corr_robot_orientation + delta_orientation;

  
  compute_correction_parameters(robot_x, robot_y, robot_orientation,
				corr_robot_x2, corr_robot_y2, 
				corr_robot_orientation2,
				corr_x, corr_y, corr_angle, corr_type);
}




/************************************************************************
 *
 *   NAME:         test_correction()
 *                 
 *   FUNCTION:     Just an internal test to check the consistency
 *                 of above procedures. Don't use, until you have
 *                 serious doubts.
 *                 
 *   PARAMETERS:   
 *
 *   RETURN-VALUE: nonw
 *                 
 ************************************************************************/



void test_correction()
{
  int i;

  float robot_x, robot_y, robot_orientation, diff_orientation;
  float corr_robot_x, corr_robot_y, corr_robot_orientation;
  float test_robot_x, test_robot_y, test_robot_orientation;
  float corr_x, corr_y, corr_angle;
  int   corr_type;
  float change_x, change_y, change_orientation;
  float error, deviation;

  fprintf(stderr, "Testing Correction Parameters...\n");

  for (i = 0; i < 100000; i++){
    robot_x =                (float) ((int) (RAND() * 500.0));
    robot_y =                (float) ((int) (RAND() * 500.0));
    robot_orientation =      (float) ((int) (RAND() * 180.0));
    corr_robot_x =           (float) ((int) (RAND() * 500.0));
    corr_robot_y =           (float) ((int) (RAND() * 500.0));
    corr_robot_orientation = (float) ((int) (RAND() * 180.0));

    if (i == 0){
      robot_x =                10.0;
      robot_y =                10.0;
      robot_orientation =      30.0;
      corr_robot_x =           -10.0;
      corr_robot_y =           10.0;
      corr_robot_orientation = 90.0;
    }
    

    compute_correction_parameters(robot_x, robot_y, robot_orientation,
				  corr_robot_x, corr_robot_y, 
				  corr_robot_orientation,
				  &corr_x, &corr_y, &corr_angle, &corr_type);

    compute_forward_correction(robot_x, robot_y, robot_orientation,
			       corr_x, corr_y, corr_angle, corr_type,
			       &test_robot_x, &test_robot_y, 
			       &test_robot_orientation);


    diff_orientation = test_robot_orientation - corr_robot_orientation;
    for (; diff_orientation < -180.0;) diff_orientation += 360.0;
    for (; diff_orientation >= 180.0;) diff_orientation -= 360.0;

    deviation = fabs(test_robot_x - corr_robot_x) 
      + fabs(test_robot_y - corr_robot_y) 
	+ fabs(diff_orientation);

    if (deviation > 0.001)

      fprintf(stderr, "(1)\trobot_x=%g\n\trobot_y=%g\n\trobot_orientation=%g\n\tcorr_robot_x=%g\n\tcorr_robot_y=%g\n\tcorr_robot_orientation=%g\n\ttest_robot_x=%g\n\ttest_robot_y=%g\n\ttest_robot_orientation=%g\n\tcorr_x=%g\n\tcorr_y=%g\n\tcorr_angle=%g\n\tcorr_type=%d\nError: %g\n\n", robot_x, robot_y, robot_orientation, corr_robot_x, corr_robot_y, corr_robot_orientation, test_robot_x, test_robot_y, test_robot_orientation, corr_x, corr_y, corr_angle, corr_type, deviation);


  }




  for (i = 0; i < 100000; i++){
    corr_robot_x =           (float) ((int) (RAND() * 500.0));
    corr_robot_y =           (float) ((int) (RAND() * 500.0));
    corr_robot_orientation = (float) ((int) (RAND() * 180.0));
    corr_x =                 (float) ((int) (RAND() * 500.0));
    corr_y =                 (float) ((int) (RAND() * 500.0));
    corr_angle =             (float) ((int) (RAND() * 180.0));
    corr_type =              (RAND() >= 0.0);

    compute_backward_correction(corr_robot_x, corr_robot_y, 
				corr_robot_orientation,
				corr_x, corr_y, corr_angle, corr_type,
				&robot_x, &robot_y,  &robot_orientation);
    
    compute_forward_correction(robot_x, robot_y, robot_orientation,
			       corr_x, corr_y, corr_angle, corr_type,
			       &test_robot_x, &test_robot_y, 
			       &test_robot_orientation);



    deviation = fabs(test_robot_x - corr_robot_x) 
      + fabs(test_robot_y - corr_robot_y) 
	+ fabs(test_robot_orientation - corr_robot_orientation);

    if (deviation > 0.001)

      fprintf(stderr, "(2)\trobot_x=%g\n\trobot_y=%g\n\trobot_orientation=%g\n\tcorr_robot_x=%g\n\tcorr_robot_y=%g\n\tcorr_robot_orientation=%g\n\ttest_robot_x=%g\n\ttest_robot_y=%g\n\ttest_robot_orientation=%g\n\tcorr_x=%g\n\tcorr_y=%g\n\tcorr_angle=%g\n\tcorr_type=%d\nError: %g\n\n", robot_x, robot_y, robot_orientation, corr_robot_x, corr_robot_y, corr_robot_orientation, test_robot_x, test_robot_y, test_robot_orientation, corr_x, corr_y, corr_angle, corr_type, deviation);




  }





  for (i = 0; i < 2; i++){
    corr_robot_x =           (float) ((int) (RAND() * 500.0));
    corr_robot_y =           (float) ((int) (RAND() * 500.0));
    corr_robot_orientation = (float) ((int) (RAND() * 180.0));
    corr_x =                 (float) ((int) (RAND() * 500.0));
    corr_y =                 (float) ((int) (RAND() * 500.0));
    corr_angle =             (float) ((int) (RAND() * 180.0));
    corr_type =              (RAND() >= 0.0);
    change_x               = (float) ((int) (RAND() * 100.0));
    change_y               = (float) ((int) (RAND() * 100.0));
    change_orientation     = (float) ((int) (RAND() * 100.0));


    if (i < 100000){
      corr_robot_x           = 0.0;
      corr_robot_y           = 0.0;
      corr_robot_orientation = 0.0;
      corr_x                 = 10.0;
      corr_y                 = 0.0;
      corr_angle             = 0.0;
      corr_type              = i;
      change_x               = 0.0;
      change_y               = 0.0;
      change_orientation     = 0.0;
    }

    compute_backward_correction(corr_robot_x, corr_robot_y, 
				corr_robot_orientation,
				corr_x, corr_y, corr_angle, corr_type,
				&robot_x, &robot_y,  &robot_orientation);
    

    update_correction_parameters(robot_x, robot_y, robot_orientation,
				 change_x, change_y, change_orientation,
				 &corr_x, &corr_y, &corr_angle, &corr_type);


    compute_forward_correction(robot_x, robot_y, robot_orientation,
			       corr_x, corr_y, corr_angle, corr_type,
			       &test_robot_x, &test_robot_y, 
			       &test_robot_orientation);


    corr_robot_x += change_x;
    corr_robot_y += change_y;
    corr_robot_orientation += change_orientation;
    for (;corr_robot_orientation >  180.0;) corr_robot_orientation -= 360.0;
    for (;corr_robot_orientation < -180.0;) corr_robot_orientation += 360.0;



    deviation = fabs(test_robot_x - corr_robot_x) 
      + fabs(test_robot_y - corr_robot_y) 
	+ fabs(test_robot_orientation - corr_robot_orientation);

    if (deviation > 0.001)



      fprintf(stderr, "(3)\tchange_x=%g\n\tchange_y=%g\n\tchange_orientation=%g\n\trobot_x=%g\n\trobot_y=%g\n\trobot_orientation=%g\n\tcorr_robot_x=%g\n\tcorr_robot_y=%g\n\tcorr_robot_orientation=%g\n\ttest_robot_x=%g\n\ttest_robot_y=%g\n\ttest_robot_orientation=%g\n\tcorr_x=%g\n\tcorr_y=%g\n\tcorr_angle=%g\n\tcorr_type=%d\nError: %g\n\n", change_x, change_y, change_orientation,  robot_x, robot_y, robot_orientation, corr_robot_x, corr_robot_y, corr_robot_orientation, test_robot_x, test_robot_y, test_robot_orientation, corr_x, corr_y, corr_angle, corr_type, deviation);


  }
  fprintf(stderr, "Done.\n");
}







static float
Deg2Rad(float x)
{
  return x * M_PI / 180.0;
}


static float
Rad2Deg(float x)
{
  return x * 180.0 / M_PI;
}

void
robotCoordinates2MapCoordinates( float robX, float robY, float robRot,
				 float corrX, float corrY, float corrRot, int corrType,
				 float* mapX, float* mapY, float* mapRot)
{
  float robRotDeg = 90.0 - robRot;
  
  compute_forward_correction( robX, robY, robRotDeg,
			      corrX, corrY, corrRot, corrType,
			      mapX, mapY, mapRot);
  *mapRot = Deg2Rad( *mapRot);
}


void
mapCoordinates2RobotCoordinates( float mapX, float mapY, float mapRot,
				 float corrX, float corrY, float corrRot, int corrType,
				 float* robX, float* robY, float* robRot)
{
  float mapRotDeg = Rad2Deg( mapRot);

  compute_backward_correction( mapX, mapY, mapRotDeg,
			       corrX, corrY, corrRot, corrType,
			       robX, robY, robRot);
  *robRot = 90.0 - *robRot;
}


void
computeCorrectionParameter( float robX, float robY, float robRot,
			    float mapX, float mapY, float mapRot,
			    float* corrX, float* corrY, float* corrRot, int* corrType)
{
  /* Equalize rotations to deg. */
  float mapRotDeg = Rad2Deg( mapRot);
  float robRotDeg = 90.0 - robRot;
  compute_correction_parameters( robX, robY, robRotDeg,
				 mapX, mapY, mapRotDeg,
				 corrX, corrY, corrRot, corrType);

/* #define TEST */
#ifdef TEST
  {
    fprintf(stderr, "BEGIN\nmap %f %f %f\n", mapX, mapY, mapRot);
    fprintf(stderr, "rob %f %f %f\n", robX, robY, robRot);

    robotCoordinates2MapCoordinates( robX, robY, robRot,
				     *corrX, *corrY, *corrRot, *corrType,
				     &mapX, &mapY, &mapRot);
    
    fprintf(stderr, "map %f %f %f\n", mapX, mapY, mapRot);
    fprintf(stderr, "rob %f %f %f\n", robX, robY, robRot);
    
    mapCoordinates2RobotCoordinates( mapX, mapY, mapRot, 
				     *corrX, *corrY, *corrRot, *corrType,
				     &robX, &robY, &robRot);
    fprintf(stderr, "map %f %f %f\n", mapX, mapY, mapRot);
    fprintf(stderr, "rob %f %f %f\n", robX, robY, robRot);
  }
#endif
}

