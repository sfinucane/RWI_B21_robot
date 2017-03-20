
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/robot/robot_specifications.h,v $
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
 * $Log: robot_specifications.h,v $
 * Revision 1.1  2002/09/14 15:55:27  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1997/07/17 17:31:51  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.2  1997/06/25 23:52:49  thrun
 * Various changes. Makde display in o-graphics much faster. changed
 * some of the parameter files to more sensible values. improves the
 * display in "learn". the commander now displays high-resolution
 * images.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:12  rhino
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









/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

#define dRESH		  640		/* H resolution */
#define dRESV		  480		/* V resolution */

#define X_SIZE dRESH
#define Y_SIZE (dRESV+12)

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


/******** PAN/TILT CONSTANTS ******************/


#define MIN_TILT_ANGLE -46.95
#define MAX_TILT_ANGLE  31.26
#define MIN_PAN_ANGLE -158.86
#define MAX_PAN_ANGLE  158.91
#define INITIAL_TILT -10.0	/* in degree */
#define INITIAL_PAN    0.0	/* in degree */
#define PAN_LOW_SPEED    40.0
#define PAN_NORMAL_SPEED 80.0
#define PAN_HIGH_SPEED   130.0
#define TILT_LOW_SPEED    40.0
#define TILT_NORMAL_SPEED 80.0
#define TILT_HIGH_SPEED   130.0


#define PSEUDO_SUBSAMPLING_FACTOR    8
#define PSEUDO_PICTURE_SIZE_HORIZONTAL 320
#define PSEUDO_PICTURE_SIZE_VERTICAL   240
#define PSEUDO_PICTURE_OFFSET_HORIZONTAL 0 
#define PSEUDO_PICTURE_OFFSET_VERTICAL   2

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/******** BASE CONSTANTS ******************/


#define MAX_TRANS_VELOCITY     100.0
#define MAX_ROT_VELOCITY       100.0
#define MAX_TRANS_ACCELERATION 100.0
#define MAX_ROT_ACCELERATION   100.0


#define INITIAL_TRANS_VELOCITY      4.0
#define INITIAL_ROT_VELOCITY        20.0

#define INITIAL_TRANS_ACCELERATION  30.0
#define INITIAL_ROT_ACCELERATION    40.0

#define TELE_MAX_ROT_VELOCITY   40.0 /* for teleoperation */
#define TELE_MAX_TRANS_VELOCITY 50.0 /* for teleoperation */
#define TELE_MAX_TRANS_DIST 200.0 /* for teleoperation */


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/


