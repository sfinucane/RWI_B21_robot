
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/laser_interface.h,v $
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
 * $Log: laser_interface.h,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.17  1999/12/06 13:19:06  fox
 * Added time stamps to base and laser report.
 *
 * Revision 1.16  1999/09/24 14:29:27  fox
 * Added support for scout robot.
 *
 * Revision 1.15  1998/11/08 20:27:51  fox
 * FIXED A BUG FOR RECTANGULAR ROBOTS (DON'T ASK).
 *
 * Revision 1.14  1998/10/23 20:50:33  fox
 * *** empty log message ***
 *
 * Revision 1.13  1998/08/26 23:23:44  fox
 * Final version before I changed exception handling.
 *
 * Revision 1.12  1998/08/18 16:24:28  fox
 * Added support for b18 robot.
 *
 * Revision 1.11  1998/05/13 07:07:42  fox
 * Fixed some bugs I found due to graphical output.
 *
 * Revision 1.10  1998/01/13 20:56:21  thrun
 * new option "-keyboard" switches off stdin.
 *
 * Revision 1.9  1997/10/23 10:56:05  rhino
 * Added -tactile flag.
 *
 * Revision 1.8  1997/04/17 09:16:19  fox
 * Added timeout for laser devices --> colliServer only needs 30% cpu.
 *
 * Revision 1.7  1997/04/07 14:51:29  thrun
 * Modified for only _one_ laser (using UNI_BONN)   (swa) (thrun)
 *
 * Revision 1.6  1997/02/05 15:41:03  fox
 * Implemented support for both laser scanners.
 *
 * Revision 1.5  1997/02/04 18:00:34  fox
 * Added paramters to BASE_setMode.
 *
 * Revision 1.4  1997/01/31 10:22:58  fox
 * First attempt to updated laser routines.
 *
 * Revision 1.3  1996/10/14 17:30:56  fox
 * Fixed a minor bug to prefer straight motion.
 *
 * Revision 1.2  1996/10/09 13:59:55  fox
 * Changed the directory where to find the parameter file colli_modes.ini.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:06  rhino
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





/*****************************************************************************
 * PROJECT: rhino
 *****************************************************************************/

#ifndef LASER_INTERFACE_LOADED
#define LASER_INTERFACE_LOADED

#include "devUtils.h"
#include "handlers.h"

#include "collision.h"

/************************************************************************
 *  Specifications of the range finders. 
 ************************************************************************/

#define FRONT_LASER 0
#define REAR_LASER  1

/* 14 is for a baud rate of 19200 */
#define LASER_BAUD_RATE 13
#define LASER_MAX_RANGE 5000

/* Angle relative to the robot */
extern float FRONT_LASER_ANGLE;
extern float REAR_LASER_ANGLE;

/* All these values used to be defines. Changed them so that we can set them
 * arbitrarily. */
extern int NUMBER_OF_LASERS;
extern int USE_FRONT_LASER;
extern int USE_REAR_LASER;

extern float FRONT_LASER_OFFSET; /* Offset in cm from the center of the robot */
extern float REAR_LASER_OFFSET; /* Offset in cm from the center of the robot */

/* Will be set in laser_interface.c. */
#define DUMMY_LASER_DEVICE "DUMMY_DEV"


/************************************************************************
 *  Type for the laser readings.
 ************************************************************************/

typedef struct LASER_reading {
 /* Have the values been converted to obstacle points? */
  int new;

  /* Values specific to the mode of the scans. */
  int numberOfReadings;
  float* reading;
  int*  blendung;
  int*  wfv;
  int*  sfv;
  float startAngle;
  float angleResolution;
  Point rPos;
  float rRot;

  /* Time of the scan. */
  struct timeval timeStamp;
} LASER_reading;

/* Readings of the two laser range finders. */
extern LASER_reading frontLaserReading;
extern LASER_reading rearLaserReading;

typedef struct {
  int laserNumber;
  LASER_reading* scan;
  DEV_TYPE dev;
}  LASER_TYPE, *LASER_PTR;

#define LASER_POLLSECONDS          1


/************************************************************************
 *  Simple laser commands
 *  NOTE : LASER_init must be called prior to any other LASER command.
 ************************************************************************/

BOOLEAN start_laser();
void
requestNextLaserScan( int laserNumber);
void stop_laser(void);
void LASER_look_for_laser_device(void);

/* Events of the lasers and the corresponding handler functions. */
#define SINGLE_LASER_REPORT     0  /* laser_readings of one laser complete */
#define COMPLETE_LASER_REPORT   1  /* laser_readings of all lasers complete */
#define LASER_REPORT_MISSED     2  /* not used */
#define LASER_NUMBER_EVENTS     3

void  LASER_InstallHandler(Handler handler, int event, Pointer client_data);
void  LASER_RemoveHandler(Handler handler, int event);
void  LASER_RemoveAllHandlers(int event);

/* Functions called by the device. */
void FRONT_LASER_outputHnd( int fd, long chars_available);
void REAR_LASER_outputHnd( int fd, long chars_available);
void LASER_timeoutHnd(void);


#ifdef DECLARE_LASER_VARS

/*********************************************************************\
|*********************************************************************|
\*********************************************************************/

/************************************************************************
 *  Laser device type.
 ************************************************************************/

LASER_TYPE    frontLaserDevice = 
{
  FRONT_LASER,
  &frontLaserReading,
  { FALSE,
    { "", DEFAULT_PORT},
    /* 14 is for a baud rate of 19200 */
    { DUMMY_LASER_DEVICE, LASER_BAUD_RATE},
    LASER_DEV_NAME,
    -1,
    TRUE,
    FALSE,
    (FILE *) NULL,
    (fd_set *) NULL,
    (DEVICE_OUTPUT_HND) FRONT_LASER_outputHnd,
    LASER_timeoutHnd,
    (DEVICE_SET_TIMEOUT)  setTimeout,  
    (DEVICE_CANCEL_TIMEOUT) cancelTimeout,
    (void (*)(void)) NULL,  
    {0, 0},
    {LONG_MAX, 0},
    {0x2FFFFFFF, 0},
    (void (*)(void)) NULL,
    FALSE
  }
};


LASER_TYPE    rearLaserDevice = 
{
  REAR_LASER,
  &rearLaserReading,
  { FALSE,
    { "", DEFAULT_PORT},
    /* 14 is for a baud rate of 19200 */
    { DUMMY_LASER_DEVICE, LASER_BAUD_RATE},
    LASER_DEV_NAME,
    -1,
    TRUE,
    FALSE,
    (FILE *) NULL,
    (fd_set *) NULL,
    (DEVICE_OUTPUT_HND) REAR_LASER_outputHnd,
    LASER_timeoutHnd,
    (DEVICE_SET_TIMEOUT)  setTimeout,  
    (DEVICE_CANCEL_TIMEOUT) cancelTimeout,
    (void (*)(void)) NULL,  
    {0, 0},
    {LONG_MAX, 0},
    {0x2FFFFFFF, 0},
    (void (*)(void)) NULL,
    FALSE
  }
};

#else

extern LASER_TYPE   frontLaserDevice;
extern LASER_TYPE   rearLaserDevice;

#endif

#endif

