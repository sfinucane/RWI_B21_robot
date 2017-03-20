
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection-old/detection.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:00 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: detection.h,v $
 * Revision 1.1  2002/09/14 20:45:00  rstone
 * *** empty log message ***
 *
 * Revision 1.16  1998/09/05 00:25:26  fox
 * Works fine to detect people in the museum.
 *
 * Revision 1.15  1998/08/29 21:44:42  fox
 * Changed the detection report and improved stuck detection.
 *
 * Revision 1.14  1998/08/23 22:57:39  fox
 * First version of building maps of humans.
 *
 * Revision 1.13  1997/06/03 11:49:14  fox
 * Museum version.
 *
 * Revision 1.12  1997/05/28 14:04:10  fox
 * Fixed a bug.
 *
 * Revision 1.11  1997/05/28 12:47:33  wolfram
 * new version
 *
 * Revision 1.10  1997/05/28 11:11:34  wolfram
 * parameter tuning
 *
 * Revision 1.9  1997/05/28 09:09:36  thrun
 * .
 *
 * Revision 1.8  1997/05/28 09:01:28  wolfram
 * added motion-only mode
 *
 * Revision 1.7  1997/05/25 10:40:51  fox
 * Nothing special.
 *
 * Revision 1.6  1997/05/25 10:39:11  thrun
 * test.
 *
 * Revision 1.5  1997/05/09 16:28:38  fox
 * Works quiet fine.
 *
 * Revision 1.4  1997/05/06 14:22:57  fox
 * Nothing special.
 *
 * Revision 1.3  1997/05/06 08:19:52  fox
 * Added several messages.
 *
 * Revision 1.2  1997/05/05 16:54:05  fox
 * Incorporated graphics from people.c.
 *
 * Revision 1.1.1.1  1997/04/30 16:17:09  fox
 * Module to detect unexpected obstacles.
 *
 * Revision 1.3  1997/04/03 13:17:49  fox
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#ifndef DETECTION_INCLUDE
#define DETECTION_INCLUDE

#define PEOPLE

/* #define CHILDREN */

#ifdef PEOPLE
#define TRACK_HEIGHT 10.0
#define NUMBER_OF_DETECTION_ANGLES 72
#define MAX_DETECTION_RANGE 250.0
#define MOTION_THRESHOLD 25.0
#define THRESHOLD 10.0
#define MOTION_EXPIRE_THRESHOLD 5.0
#endif
#ifdef CHILDREN
#define TRACK_HEIGHT -20.0
#define NUMBER_OF_DETECTION_ANGLES 180
#define MAX_DETECTION_RANGE 150.0
#define MOTION_THRESHOLD 7.0
#define THRESHOLD 9.0
#define MOTION_EXPIRE_THRESHOLD     1.0
#endif

extern int detectionMode;

typedef struct {
  int numberOfAngles;
  float* distances;
  float* prev_distances;
  float* angles;
  float* measuredDistances;
  int foundSomething;
  int index;
  int detectionMode;
  point detected;
} detectionStruct;

void
detectUnexpectedObstacles( detectionStruct* obstacles);

void
detectMotion( detectionStruct* obstacles);

void
stuckDetection( detectionStruct* obstacles);

void
emergencyHandling( realPosition robPos, int emergency);

extern detectionStruct obstacles;

#endif


