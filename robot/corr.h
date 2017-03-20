
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/robot/corr.h,v $
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
 * $Log: corr.h,v $
 * Revision 1.1  2002/09/14 15:55:27  rstone
 * *** empty log message ***
 *
 * Revision 1.2  2000/01/27 14:11:28  schulz
 * Added an extern "C" wrapper for C++ compliance
 *
 * Revision 1.1  1998/08/11 23:03:49  thrun
 * corr is now a general library
 *
 * Revision 1.2  1998/08/11 22:05:26  thrun
 * New correction parameters - experimental version.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:27  rhino
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


#ifdef __cplusplus
extern "C" {
#endif





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


void compute_forward_correction(float robot_x, 
				float robot_y, 
				float robot_orientation,
				float corr_x, 
				float corr_y, 
				float corr_angle, /* in deg. */
				int   corr_type,
				float *corr_robot_x,
				float *corr_robot_y, 
				float *corr_orientation);



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



void compute_backward_correction(float corr_robot_x, 
				 float corr_robot_y, 
				 float corr_robot_orientation,
				 float corr_x, 
				 float corr_y, 
				 float corr_angle, /* in deg. */
				 int   corr_type,
				 float *robot_x,
				 float *robot_y, 
				 float *orientation);


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



void compute_correction_parameters(float robot_x, 
				   float robot_y, 
				   float robot_orientation,
				   float corr_robot_x,
				   float corr_robot_y, 
				   float corr_orientation,
				   float *corr_x, 
				   float *corr_y, 
				   float *corr_angle,
				   int   *corr_type);



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




void update_correction_parameters(float corr_robot_x, 
				  float corr_robot_y, 
				  float corr_robot_orientation,
				  float delta_x,
				  float delta_y,
				  float delta_orientation,
				  float *corr_x, 
				  float *corr_y, 
				  float *corr_angle,
				  int   *corr_type);

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



void test_correction();


void
robotCoordinates2MapCoordinates( float robX, float robY, float robRot,
				 float corrX, float corrY, float corrRot, int corrType,
				 float* mapX, float* mapY, float* mapRot);



void
mapCoordinates2RobotCoordinates( float mapX, float mapY, float mapRot,
				 float corrX, float corrY, float corrRot, int corrType,
				 float* robX, float* robY, float* robRot);


void
computeCorrectionParameter( float robX, float robY, float robRot,
			    float mapX, float mapY, float mapRot,
			    float* corrX, float* corrY, float* corrRot, int* corrType);

#ifdef __cplusplus
}
#endif
