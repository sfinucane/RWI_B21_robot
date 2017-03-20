
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
a *****
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/script.h,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 17:06:29 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: script.h,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.19  1999/10/21 17:30:44  fox
 * Fixed some bugs in real time scripts.
 *
 * Revision 1.18  1999/09/03 22:22:40  fox
 * Changed hadnling of real time script. This version contains both.
 *
 * Revision 1.17  1999/09/01 00:02:58  fox
 * Getting closer. But still not close enough.
 *
 * Revision 1.16  1999/08/27 22:22:34  fox
 * Better communication for multi robot localization.
 *
 * Revision 1.15  1999/06/24 00:21:53  fox
 * Some changes for the urbies.
 *
 * Revision 1.14  1999/04/29 13:35:21  fox
 * Further adaptation to make multi localize run.
 *
 * Revision 1.13  1999/03/12 00:41:50  fox
 * Minor changes.
 *
 * Revision 1.12  1998/08/23 00:01:05  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.11  1998/04/19 10:40:38  wolfram
 * Added graphical display to multi localization
 *
 * Revision 1.10  1998/02/12 15:49:11  derr
 * librawData support implemented.
 * see Makefile for further information.
 *
 * Revision 1.9  1998/01/22 13:06:23  fox
 * First version after selection-submission.
 *
 * Revision 1.8  1998/01/05 10:37:16  fox
 * Several changes especially in selection parts.
 *
 * Revision 1.7  1997/12/09 12:03:24  wolfram
 * Added support for Marker in scripts
 *
 * Revision 1.6  1997/12/02 15:20:43  fox
 * Nothing remarkable.
 *
 * Revision 1.5  1997/04/27 15:48:22  wolfram
 * Changes in script.c
 *
 * Revision 1.4  1997/03/13 17:36:39  fox
 * Temporary version. Don't use!
 *
 * Revision 1.3  1997/01/17 13:21:08  fox
 * Added timer for non relevant functions.
 *
 * Revision 1.2  1996/12/02 10:32:13  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.1.1.1  1996/09/22 16:46:33  rhino
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


#ifndef SCRIPT_INCLUDE
#define SCRIPT_INCLUDE

#ifndef DO_NOT_USE_NEW_SCRIPT_LIBRARY
#warning new script support
#include "channelData.h"
#else
#include "general.h"
#include "sensings.h"
#include "map.h"
#include "sonar.h"

#define WRONG_SCRIPT 0
#define ORIG_SCRIPT 1
#define CORR_SCRIPT 2

#define BUFFLEN 250000

extern float nonRelevantTime;
extern float elapsedScriptTime;


typedef struct {
  int hour;
  int minute;
  float second;
} scriptTime;


/* This struct contains the script file information */
typedef struct {
  char *fileName;
  realPosition start;
  realPosition currentPos;
  FILE *fp;
  bool newMarker;
  int scriptType;
  int sonarCount;
  int positionCount;
  int laserCount;
  int markerCount;
  int imageCount;
  float distanceOffset;
  scriptTime timeOfScript;
  scriptTime startTimeOfScript;
  struct timeval time;
  struct timeval startTimeVal;
  float timeFactor;
  actionInformation *actionInfo;
  int odometryCorrection;
  int bumpOccured;
  float bumpForward;
  float bumpSideward;
  float bumpRot;
  int realTime;
  realPosition mapPosition;
  int newMapPosition;
  float timeToBeSkipped;
  float runTime;
} script;

/* Correction used for museum scripts. */
#define CORRECTION_FACTOR 1.024609

int
openScript(char *fileName, script *s);

int
closeScript(script *s);

bool
readScript(script* s,
	   rawSensings* actualSensings);

bool
getReadings(char *line, float *sensor, int numberOfReadings);

#endif
#endif

