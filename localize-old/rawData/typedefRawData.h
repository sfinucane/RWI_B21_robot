/* Time-stamp: <98/02/16 15:01:28 wolfram>
***********************************************************************
***********************************************************************
***********************************************************************
*****
***** File name:                   rawData.hh
*****
***** Part of:                     RHINO SOFTWARE
*****
***** Creator:                     Andreas Derr, University of Bonn
*****
***** Date of creation:            Oct 1997
*****
*****
*****
*****
***** $Source: /usr/local/cvs/bee/src/localize/rawData/typedefRawData.h,v $
*****
***** $Revision: 1.1 $
*****
***** $Date: 2002/09/14 17:06:31 $
*****
***** $Author: rstone $
*****
*****
*****
***** Contact derr@informatik.uni-bonn.de
*****
***********************************************************************
***********************************************************************
***********************************************************************
*****
***** THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY
***** APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING THE
***** COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM "AS IS"
***** WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
***** INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
***** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE ENTIRE
***** RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU.
***** SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL
***** NECESSARY SERVICING, REPAIR OR CORRECTION.
*****
***** IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN
***** WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY
***** MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE
***** LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL,
***** INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR
***** INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF
***** DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU
***** OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY
***** OTHER PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN
***** ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
*****
***********************************************************************
***********************************************************************
***********************************************************************
**
**
**                  ----- REVISION HISTORY -----
**
** $Log: typedefRawData.h,v $
** Revision 1.1  2002/09/14 17:06:31  rstone
** *** empty log message ***
**
** Revision 1.2  1998/02/16 14:25:17  wolfram
** Can now be compiled
**
** Revision 1.1  1998/02/12 18:14:03  derr
** Added library for reading laserint- sonarint- and 'new'-scripts.
**
** Revision 1.1  1998/02/12 15:59:52  derr
** New library librawData for reading laserint-, sonarint and 'new'-script.
** Will replace bee/src/localize/script.c.
**
**
**
***********************************************************************
***********************************************************************
**********************************************************************/

#ifndef TYPEDEF_RAWDATA_INCLUDE
#define TYPEDEF_RAWDATA_INCLUDE

/* load some general definitions */
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include "../general.h"

#define NO_LOG_FILE NULL

typedef enum {
  SONAR_DATA        =   1,
  LASER_DATA        =   2,
  MOVEMENT_DATA     =   4,
  BASEPOSITION_DATA =   8,
  MARKER_DATA       =  16,
  TACTILE_DATA      =  32,
  INFRARED_DATA     =  64,
  BUTTON_DATA       = 128,
  ALL_DATA          = 255
} t_WantedData; /* change rawData::setWantedDate, too */

typedef enum {
  UNKNOWN_TYPE       =  0,
  ORIG_SCRIPT_TYPE,
  SECOND_SCRIPT_TYPE,
  TCX_TYPE
} t_RawDataType;

#define WRONG_SCRIPT UNKNOWN_TYPE
#define ORIG_SCRIPT ORIG_SCRIPT_TYPE
#define CORR_SCRIPT UNKNOWN_TYPE
#define SECOND_SCRIPT SECOND_SCRIPT_TYPE

typedef enum {
  CONTINUE_ON_ERROR  =  0,
  STOP_ON_ERROR,
  CORE_ON_ERROR
} t_ErrorAction;

typedef enum {
  ERROR              = -1,
  NO_NEW_DATA,
  NEW_DATA,
  END_REACHED
} t_Result;

typedef distanceReading  t_DistanceReading;

typedef struct {
  unsigned int        maxNumberOfReadings;
  unsigned int        actualNumberOfReadings;
  t_DistanceReading  *reading;
} t_Reading;

typedef realPosition t_RealPosition;

#ifndef SENSINGS_INCLUDE_SPECIAL_SCRIPT_DATA
#define SENSINGS_INCLUDE_SPECIAL_SCRIPT_DATA
typedef distanceScan sensing_PROXIMITY;
typedef distanceScan sensing_SONAR;
typedef movement sensing_MOVEMENT;
typedef answer sensing_QUESTION;

/* The actual sensings. */
typedef struct {
  realPosition                basePosition;
  sensing_MOVEMENT            delta;
  sensing_PROXIMITY           sonar;
  sensing_PROXIMITY           frontLaser;
  sensing_PROXIMITY           rearLaser;
  sensing_QUESTION            answer;
  float                       distanceTraveled;
  int                         noMoveCnt;
} rawSensings;
#endif

typedef struct {
  int dummy;
} t_Tactile;
typedef struct {
  int dummy;
} t_Button;
typedef struct {
  int dummy;
} t_Infrared;

/* const unsigned int NUMBER_OF_SONARS          =   24;
   const unsigned int NUMBER_OF_LASERS          =    2; */
/* const unsigned int MAX_VALUESPERSCAN         = 1024; */
#define NUMBER_OF_SCANS_PER_LASER 180
#define MAX_VALUESPERSCAN 1024
/* This struct contains the script file information */

/* Correction used for museum scripts. */
#define CORRECTION_FACTOR 1.024609

typedef struct {
   /* set this once for openScript() */
  char *fileName;
  int odometryCorrection;
  float timeFactor;

  /* set this every time for readScript() */
  float nonRelevantTime; /* will be substracted in readScript() */

  /* get this with readScript() */
  realPosition start;
  realPosition currentPos;
  bool newMarker;
  int sonarCount;
  int positionCount;
  int laserCount;
  int markerCount;
  float distanceOffset; /* in cm */
  int bumpOccured;
  float bumpForward;
  float bumpSideward;
  float bumpRot;
  int noMoveCount;
} script;

#endif

/* End of TYPEDEF_RAWDATA_HEADER */

