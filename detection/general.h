
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection/general.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:44:56 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: general.h,v $
 * Revision 1.1  2002/09/14 20:44:56  rstone
 * *** empty log message ***
 *
 * Revision 1.9  2000/12/04 20:25:49  thrun
 * Problem with compilation.
 *
 * Revision 1.8  1998/09/05 00:25:26  fox
 * Works fine to detect people in the museum.
 *
 * Revision 1.7  1998/08/29 21:44:42  fox
 * Changed the detection report and improved stuck detection.
 *
 * Revision 1.6  1998/08/23 22:57:40  fox
 * First version of building maps of humans.
 *
 * Revision 1.5  1997/06/03 11:49:14  fox
 * Museum version.
 *
 * Revision 1.4  1997/05/06 14:22:57  fox
 * Nothing special.
 *
 * Revision 1.3  1997/05/06 08:19:52  fox
 * Added several messages.
 *
 * Revision 1.2  1997/05/05 16:54:06  fox
 * Incorporated graphics from people.c.
 *
 * Revision 1.1.1.1  1997/04/30 16:17:09  fox
 * Module to detect unexpected obstacles.
 *
 * Revision 1.20  1997/03/13 17:36:35  fox
 * Temporary version. Don't use!
 *
 * Revision 1.19  1997/02/13 18:30:13  thrun
 * Dieter and Wolfram: Do you know htat MAXFLOAT is not defined under SUN
 * OS 4.1.3?
 *
 * Revision 1.18  1997/02/12 15:08:36  fox
 * Integrated laser support.
 *
 * Revision 1.17  1997/02/11 11:04:08  wolfram
 * Faster Computation of expected distances, height of sensors is considered
 *
 * Revision 1.16  1997/01/31 16:19:16  wolfram
 * Added start position to each reading of a scan
 *
 * Revision 1.15  1997/01/29 12:23:07  fox
 * First version of restructured DETECTION.
 *
 * Revision 1.14  1997/01/18 18:19:22  wolfram
 * *** empty log message ***
 *
 * Revision 1.13  1997/01/07 08:52:08  wolfram
 * Added time to movements and struct realCellList
 *
 * Revision 1.12  1997/01/06 16:30:47  wolfram
 * Added time stamp to proximity sensings
 *
 * Revision 1.11  1997/01/03 10:09:46  fox
 * First version with exploration.
 *
 * Revision 1.10  1996/12/19 14:33:28  wolfram
 * More sensitive computation of expected distance tables
 *
 * Revision 1.9  1996/12/02 10:32:05  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.8  1996/11/28 17:56:22  fox
 * *** empty log message ***
 *
 * Revision 1.7  1996/11/26 11:08:11  fox
 * Improved version.
 *
 * Revision 1.6  1996/11/21 16:03:54  wolfram
 * Added extern c for c++
 *
 * Revision 1.5  1996/11/21 14:55:49  wolfram
 * added gridCellList to general.h
 *
 * Revision 1.4  1996/11/21 12:40:25  fox
 * Tools for bayesian reasoning on the grids.
 *
 * Revision 1.3  1996/11/18 09:58:29  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.2  1996/10/24 12:07:10  fox
 * Fixed a bug.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:32  rhino
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


#ifndef GENERAL_INCLUDE
#define GENERAL_INCLUDE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <sys/time.h>

#include "EZX11.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TRUE 1
#define FALSE 0

#define MAX_STRING_LENGTH 255
#define MAX_NUMBER_OF_CELLS 255

#ifndef MAXFLOAT
#define   MAXFLOAT        ((float)3.40282346638528860e+38)
#endif

extern int useGraphics;
extern float mapResolution;

typedef unsigned int bool;
typedef float distance;


typedef struct {
    float x;
    float y;
} point;

typedef struct {
    float x;
    float y;
    float rot;
} realPosition;

typedef struct {
    int x;
    int y;
    int rot;
} gridPosition;



typedef struct{
  gridPosition pos;
  float prob;
} gridCell;

typedef struct{
  gridCell cell[MAX_NUMBER_OF_CELLS];
  bool inMap[MAX_NUMBER_OF_CELLS];
  int numberOfCells;
  int numberOfCellsInMap;
  float originalSumOfProbs;
} gridCellList;

typedef struct {
  int numberOfDistances;
  float*** probs;
  float deltaDist;
} humanProbTable;

typedef struct {
  bool initialized;
  realPosition refPos;
  int sizeX;
  int sizeY;
  float resolution;
  float** probs;
} probabilityGrid;


/*--------------------------------------------------------------------------
 * Structs for proximity sensors.
 *------------------------------------------------------------------------*/

/* This struct contains a single distance information. */
typedef struct {
  distance dist;
  int feature;
  float rot;
} distanceReading;


/* This struct contains a collection of distance readings. */
typedef struct {
  bool             isNew;
  distance         maxDistance;
  int              numberOfReadings;
  int              numberOfFeatures;
  float            distanceResolution;
  distanceReading* reading;
  realPosition     frontRealRob;
  realPosition     rearRealRob;
  realPosition     frontRealMap;
  realPosition     rearRealMap;
} distanceScan;


/*********************************************************************\
|*********************************************************************|
|*********************************************************************|
*  Stuff taken from PEOPLE.
|*********************************************************************|
|*********************************************************************|
\*********************************************************************/

typedef struct{
  int   tcx_initialized;
  int   graphics_initialized;
  int   use_graphics;
  int   use_tcx;
  int   base_connected;
  int   localize_connected;
  int   pantilt_connected;
  int   sound_connected;
  int   mouth_connected;
  int   quit;
} PROGRAM_STATE, *PROGRAM_STATE_PTR;


typedef struct{
  float robot_size;		/* size of the robot in cm */
  int   num_detections;		/* number of detection angles */
  float first_detection_angle;	/* offset of the first detection */
  float *detection_angles;	/* detection angle values (vector) */
  float max_detection_range;	/* maximum sensor range in cm */
  float min_detection_range;	/* minimum sensor range in cm */
} ROBOT_SPECIFICATIONS, *ROBOT_SPECIFICATIONS_PTR;



typedef struct{
  float x;
  float y;
  float orientation;
  float translational_speed;
  float rotational_speed;
  float *sensor_values;
  float *detection_values;
  int   known;

  int person_found;
  float local_person_x;
  float local_person_y;
  float global_person_x;
  float global_person_y;
} ROBOT_STATE, *ROBOT_STATE_PTR;



/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern PROGRAM_STATE            program_state_data;
extern ROBOT_STATE              robot_state_data;
extern ROBOT_SPECIFICATIONS     robot_specifications_data;
extern PROGRAM_STATE_PTR        program_state;
extern ROBOT_STATE_PTR          robot_state;
extern ROBOT_SPECIFICATIONS_PTR robot_specifications;


/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

extern struct bParamList * bParamList;


#ifdef __cplusplus
}
#endif


#endif

