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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/script.c,v $
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
 * $Log: script.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.76  2000/03/06 20:00:46  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.75  2000/01/26 22:56:44  fox
 * Might work. If anything weird happens, contact me.
 *
 * Revision 1.74  1999/12/15 16:16:41  fox
 * First attempt to extract p(l | o).
 *
 * Revision 1.73  1999/11/15 13:28:04  fox
 * *** empty log message ***
 *
 * Revision 1.72  1999/11/02 18:12:37  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.71  1999/10/21 17:30:44  fox
 * Fixed some bugs in real time scripts.
 *
 * Revision 1.70  1999/09/29 16:06:11  fox
 * Should work.
 *
 * Revision 1.69  1999/09/09 02:48:38  fox
 * Final version before germany.
 *
 * Revision 1.68  1999/09/06 16:36:04  fox
 * Many changes.
 *
 * Revision 1.67  1999/09/03 22:58:35  fox
 * Doesn't work correctly.
 *
 * Revision 1.66  1999/09/03 22:25:57  fox
 * Removed old version of realtime handling.
 *
 * Revision 1.65  1999/09/03 22:22:40  fox
 * Changed hadnling of real time script. This version contains both.
 *
 * Revision 1.64  1999/09/01 00:02:58  fox
 * Getting closer. But still not close enough.
 *
 * Revision 1.63  1999/08/30 05:48:43  fox
 * Doesn't work!!
 *
 * Revision 1.62  1999/08/27 22:22:34  fox
 * Better communication for multi robot localization.
 *
 * Revision 1.61  1999/06/25 19:48:13  fox
 * Minor changs for the urbie.
 *
 * Revision 1.60  1999/06/24 00:21:53  fox
 * Some changes for the urbies.
 *
 * Revision 1.59  1999/06/23 16:22:03  fox
 * Added robot type urban.
 *
 * Revision 1.58  1999/04/29 13:35:21  fox
 * Further adaptation to make multi localize run.
 *
 * Revision 1.57  1999/04/28 22:16:00  fox
 * The script can now contain robot detection information. If such a detection
 * is found, then it is sent to the module MULTI_LOCALIZE (not perfect though).
 *
 * Revision 1.56  1999/04/21 22:58:01  fox
 * First attempt to get samples from multi back.
 *
 * Revision 1.55  1999/04/21 14:06:01  fox
 * Just an intermediate version.
 *
 * Revision 1.54  1999/04/18 19:00:11  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
 *
 * Revision 1.53  1999/03/19 16:04:41  wolfram
 * Added REAL_TIME simulation in script.c
 *
 * Revision 1.52  1999/03/12 00:41:50  fox
 * Minor changes.
 *
 * Revision 1.51  1999/02/17 19:42:25  fox
 * Enhanced gif utilities.
 *
 * Revision 1.50  1999/01/22 18:10:42  fox
 * Removed some hacks done for denver and the sampling paper.
 *
 * Revision 1.49  1999/01/22 17:48:10  fox
 * Final version of sampling paper and denver demo.
 *
 * Revision 1.48  1999/01/11 19:47:56  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.46  1998/11/03 21:02:22  fox
 * Lasers will not be integrated if the robot rotates faster than
 * MAX_ROT_VEL_FOR_LASERS deg/sec.
 *
 * Revision 1.45  1998/10/29 03:45:05  fox
 * Nothing special.
 *
 * Revision 1.44  1998/10/26 22:16:17  wolfram
 * Added logScale option for plotting
 *
 * Revision 1.43  1998/09/18 15:44:29  fox
 * First attempt to get a running system after museum.
 *
 * Revision 1.42  1998/08/26 15:34:04  wolfram
 * Finished integration of vision
 *
 * Revision 1.41  1998/08/23 00:01:04  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.40  1998/08/19 16:33:58  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.39  1998/06/12 10:16:43  fox
 * Implemented virutal sensor.
 *
 * Revision 1.38  1998/04/19 10:40:38  wolfram
 * Added graphical display to multi localization
 *
 * Revision 1.37  1998/02/12 15:49:10  derr
 * librawData support implemented.
 * see Makefile for further information.
 *
 * Revision 1.36  1998/02/10 13:04:49  wolfram
 * LOCALIZE reads new scripts now! First alpha 0.0001-version
 *
 * Revision 1.35  1998/01/22 13:06:22  fox
 * First version after selection-submission.
 *
 * Revision 1.34  1998/01/05 10:37:15  fox
 * Several changes especially in selection parts.
 *
 * Revision 1.33  1997/12/19 11:30:14  fox
 * FIXED a bug I added.
 *
 * Revision 1.32  1997/12/09 12:03:23  wolfram
 * Added support for Marker in scripts
 *
 * Revision 1.31  1997/12/02 15:20:42  fox
 * Nothing remarkable.
 *
 * Revision 1.30  1997/11/27 18:11:21  fox
 * Several changes to make angles work better.
 *
 * Revision 1.29  1997/11/20 12:58:15  fox
 * Version with good sensor selection.
 *
 * Revision 1.28  1997/10/01 11:30:01  fox
 * Minor changes.
 *
 * Revision 1.27  1997/09/11 20:39:09  fox
 * Added possibility to read script from stdin.
 *
 * Revision 1.26  1997/08/22 04:16:41  fox
 * Final version before IJCAI.
 *
 * Revision 1.25  1997/08/12 21:42:57  wolfram
 * Time factor <= 1 results in using every reading in the script
 *
 * Revision 1.24  1997/08/02 16:51:08  wolfram
 * 1. Changed the order of indexes of the grid (and only of the grid):
 * The order now is grid->prob[rot][x][y].  This results in a significant
 * speed-up for different operations such as normalization and
 * integration of new sensory data.  Reimplemented the ConvolveThirdDim
 * procedure for convolving over rot.
 *
 * 2. Changed the algorithm to detect linear alignments of readings.  Now
 * we use the approach of Lu.
 *
 * 3. Linear alignments of readings is also checked for laser readings.
 *
 * 4. Expected distances are now computed given the simulator map if
 * available.  For that purpose the library libGetDistance is included.
 *
 * 5. Graphic output now displays the simulator map (if available). This
 * concernes the map overlay as well as the robot window.
 *
 * 6. Fixed some minor bugs.
 *
 * 7. Added different parameters to the ini-file (see example.ini).
 *
 * Revision 1.23  1997/06/27 16:26:31  fox
 * New model of the proximity sensors.
 *
 * Revision 1.22  1997/05/09 16:28:18  fox
 * Nothing special.
 *
 * Revision 1.21  1997/04/27 16:39:00  wolfram
 * Corrected script is no longer written
 *
 * Revision 1.20  1997/04/27 15:48:21  wolfram
 * Changes in script.c
 *
 * Revision 1.19  1997/03/24 06:55:29  wolfram
 * Cleaned up graphic.c and added a graphic window as a global variable
 * in graphic.c
 *
 * Revision 1.18  1997/03/13 17:36:24  fox
 * Temporary version. Don't use!
 *
 * Revision 1.17  1997/01/31 16:19:18  wolfram
 * Added start position to each reading of a scan
 *
 * Revision 1.16  1997/01/30 17:17:27  fox
 * New version with integrated laser.
 *
 * Revision 1.15  1997/01/29 12:23:14  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.14  1997/01/20 13:09:39  fox
 * Unbelievable, that this positino estimation ever worked ..........
 *
 * Revision 1.13  1997/01/19 15:42:26  wolfram
 * Script time difference is compted correctly even if the time in the script
 * does not increase
 *
 * Revision 1.12  1997/01/18 14:07:57  fox
 * Test version.
 *
 * Revision 1.11  1997/01/17 13:21:07  fox
 * Added timer for non relevant functions.
 *
 * Revision 1.10  1997/01/16 19:43:24  fox
 * And another bug ...
 *
 * Revision 1.9  1997/01/07 08:51:24  wolfram
 * Added time to movements
 *
 * Revision 1.8  1997/01/06 17:38:03  wolfram
 * Added time stamp for readings
 *
 * Revision 1.7  1996/12/05 14:31:35  wolfram
 * *** empty log message ***
 *
 * Revision 1.6  1996/12/02 10:32:13  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.5  1996/11/18 09:58:32  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.4  1996/11/15 17:44:09  ws
 * *** empty log message ***
 *
 * Revision 1.3  1996/10/24 12:07:13  fox
 * Fixed a bug.
 *
 * Revision 1.2  1996/10/24 09:56:56  fox
 * LOCALIZE also works in a write protected directory.
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

#ifdef DO_NOT_USE_NEW_SCRIPT_LIBRARY
#include <sys/time.h>
#include <ctype.h>
#include <unistd.h>

#include "stdio.h"
#include "script.h"
#include "file.h"
#include "map.h"
#include "function.h"
#include "sonar.h"
#include "laser.h"
#include "movement.h"
#include "graphic.h"
#include "localize.h"
#include "vision.h"
#include "condensation.h"
#include "communication.h"

#define WRONG_SCRIPT 0
#define ORIG_SCRIPT 1
#define NEW_SCRIPT 2

#define ORIG_SCRIPTPOSMARK "#ROBOT"
#define SCRIPTBUMPERMARK "#BUMP"
#define SCRIPT_DETECTION_MARK "#DETECTION"
#define SCRIPT_MAP_POSITION_MARK "#MAP_POSITION"

#define ORIG_SCRIPTSONARMARK "#SONAR 24:"
#define ORIG_SCRIPTLASERMARK "#LASER"
/* #define ORIG_SCRIPTLASERMARK "lasers" */
#define ORIG_SCRIPTMARKERMARK "#MARKER:"
#define ORIG_TIMEMARK "@SENS"
#define ORIG_TIMEOFFSET 9


#define NEW_SCRIPTPOSMARK "position:"
#define NEW_SCRIPTSONARMARK "sonars 24 :"
#define NEW_SCRIPTLASERMARK "lasers"
#define NEW_TIMEMARK "time:"
#define NEW_TIMEOFFSET 0
#define NEW_IMAGE_MARK "image"
#define MAX_VALUESPERSCAN 1024

float nonRelevantTime;
float elapsedScriptTime;

static bool
timeReadingLine(char* line)
{
  return (strncmp(line, ORIG_TIMEMARK, strlen(ORIG_TIMEMARK)) == 0);
}


static bool
newImageReadingLine(char* line)
{
  return (strncmp(line, NEW_IMAGE_MARK, strlen(NEW_IMAGE_MARK)) == 0);
}


static bool
newTimeReadingLine(char* line)
{
  return (strncmp(line, NEW_TIMEMARK, strlen(NEW_TIMEMARK)) == 0);
}


static bool
markerReadingLine(char* line)
{
  return (strncmp(line,ORIG_SCRIPTMARKERMARK,
		  strlen(ORIG_SCRIPTMARKERMARK)) == 0);
}

static bool
bumperReadingLine(char* line)
{
  return (strncmp(line,SCRIPTBUMPERMARK,
		  strlen(SCRIPTBUMPERMARK)) == 0);
}

static bool
detectionReadingLine(char* line)
{
  return (strncmp( line, SCRIPT_DETECTION_MARK,
		  strlen(SCRIPT_DETECTION_MARK)) == 0);
}

static bool
mapPositionReadingLine(char* line)
{
  return (strncmp( line, SCRIPT_MAP_POSITION_MARK,
		  strlen(SCRIPT_MAP_POSITION_MARK)) == 0);
}

static bool
positionReadingLine(char* line)
{
  return (strncmp(line,ORIG_SCRIPTPOSMARK,
		  strlen(ORIG_SCRIPTPOSMARK)) == 0);
}


static bool
newPositionReadingLine(char* line)
{
  return (strncmp(line,NEW_SCRIPTPOSMARK,
		  strlen(NEW_SCRIPTPOSMARK)) == 0);
}

static bool
sonarReadingLine(char* line)
{
  return (strncmp(line,ORIG_SCRIPTSONARMARK,
		  strlen(ORIG_SCRIPTSONARMARK)) == 0);
}

static bool
newSonarReadingLine(char* line)
{
  return (strncmp(line,NEW_SCRIPTSONARMARK,
		  strlen(NEW_SCRIPTSONARMARK)) == 0);
}

static bool
laserReadingLine(char* line)
{
  return (strncmp(line,ORIG_SCRIPTLASERMARK,
		  strlen(ORIG_SCRIPTLASERMARK)) == 0);
}

static bool
newLaserReadingLine(char* line)
{
  return (strncmp(line,NEW_SCRIPTLASERMARK,
		  strlen(NEW_SCRIPTLASERMARK)) == 0);
}


float
scriptTimeDiff(scriptTime newTime, scriptTime oldTime){

    float newSeconds, oldSeconds;

    newSeconds = newTime.hour * 3600 + newTime.minute * 60 + newTime.second;

    oldSeconds = oldTime.hour * 3600 + oldTime.minute * 60 + oldTime.second;

    if (newTime.hour < oldTime.hour)
      return (86400 + newSeconds - oldSeconds);
    else
      return (newSeconds - oldSeconds);
}

realPosition
measuredPosition = {0.0, 0.0, 0.0};

int
openScript(char *fileName, script *s) {

  FILE *fp;

  bool eof, timeRead, timeSkipped = FALSE, positionRead;

  char line[BUFFLEN];

  float startX, startY, startRot;

  /* try to open file */
  if ( strncmp( fileName, STDIN_NAME, 5) != 0) {
    if ((fp = fopen( fileName,"rt")) == NULL) {
      fprintf(stderr,"ERROR: Could not open file '%s'!\n",fileName);
      return(0);
    }
  }
  else
    fp=stdin;

  if ( s->realTime) {
    writeLog( "# Script will be processed in real time simulation with factor %f.\n",
	      s->timeFactor);
    fprintf( stderr, "# Script will be processed in real time simulation with factor %f.\n",
	      s->timeFactor);
  
  }

  s->scriptType = WRONG_SCRIPT;
  eof = timeRead = positionRead = FALSE;
  s->distanceOffset = 0.0;
  elapsedScriptTime = 0.0;
  
  writeLog( "# _OLD_ script support enabled.\n");

  /* check file, load start data */
  while (!eof && !( timeSkipped && positionRead)){

    if (!(eof = (fgets(line,BUFFLEN,fp) == NULL))){
      if (positionReadingLine(line)){
	s->scriptType = ORIG_SCRIPT;
	if (sscanf(&line[strlen(ORIG_SCRIPTPOSMARK)],"%f %f %f",
		   &startX, &startY, &startRot) == 3)
	  positionRead = TRUE;
      }
      else if (newPositionReadingLine(line)){
	s->scriptType = NEW_SCRIPT;
	if (sscanf(&line[strlen(NEW_SCRIPTPOSMARK)], "%f %f %f",
		   &startX, &startY, &startRot) == 3)
	  positionRead = TRUE;
      }
      
      if (timeReadingLine(line)) {

	if (sscanf(&line[strlen(ORIG_TIMEMARK)+ORIG_TIMEOFFSET], "%d:%d:%f",
		   &(s->timeOfScript.hour),
		   &(s->timeOfScript.minute),
		   &(s->timeOfScript.second)) == 3) {
	
	  /* The first time item. Set the start time and check whether
	   * skipping is desired. */
	  if ( ! timeRead) {
	    
	    s->startTimeOfScript.hour = s->timeOfScript.hour;
	    s->startTimeOfScript.minute = s->timeOfScript.minute;
	    s->startTimeOfScript.second = s->timeOfScript.second;
	    timeRead = TRUE;
	    if ( s->timeToBeSkipped <= 0.0)
	      timeSkipped = TRUE;
	    else
	      writeLog( "# Skip first %f seconds.\n", s->timeToBeSkipped);
	  }
	  else {
	    /* Skipped enough. Set the new start time of the script. */
	    if ( scriptTimeDiff(s->timeOfScript, s->startTimeOfScript) > s->timeToBeSkipped) {
	      timeSkipped = TRUE;
	      s->startTimeOfScript.hour = s->timeOfScript.hour;
	      s->startTimeOfScript.minute = s->timeOfScript.minute;
	      s->startTimeOfScript.second = s->timeOfScript.second;
	    }
	  }
	}
      }
      else if (newTimeReadingLine(line)) {
	long secs, usecs;
	if (sscanf(&line[strlen(NEW_TIMEMARK)+NEW_TIMEOFFSET], "%ld %ld",
		   &secs, &usecs) == 2){
	  s->timeOfScript.hour = (secs % 86400) / 3600;
	  s->timeOfScript.minute = (secs % 3600) / 60;
	  s->timeOfScript.second = (secs % 60) + usecs/1000000.0;

	  /* The first time item. Set the start time and check whether
	   * skipping is desired. */
	  if ( ! timeRead) {
	    
	    s->startTimeOfScript.hour = s->timeOfScript.hour;
	    s->startTimeOfScript.minute = s->timeOfScript.minute;
	    s->startTimeOfScript.second = s->timeOfScript.second;
	    timeRead = TRUE;

	    if ( s->timeToBeSkipped <= 0.0)
	      timeSkipped = TRUE;
	    else
	      writeLog( "# Skip first %f seconds.\n", s->timeToBeSkipped);
	  }
	  else {
	    /* Skipped enough. Set the new start time of the script. */
	    if ( scriptTimeDiff(s->timeOfScript, s->startTimeOfScript) > s->timeToBeSkipped) {
	      timeSkipped = TRUE;
	      s->startTimeOfScript.hour = s->timeOfScript.hour;
	      s->startTimeOfScript.minute = s->timeOfScript.minute;
	      s->startTimeOfScript.second = s->timeOfScript.second;
	    }
	  }
	}
      }
    }
  }

  if (timeRead && positionRead){

    if (s->scriptType == ORIG_SCRIPT)
      writeLog( "# File type of %s is original script.\n",fileName);
    else if (s->scriptType == NEW_SCRIPT)
      writeLog( "# File type of %s is new script.\n",fileName);
    else
      writeLog( "# File type of %s is unknown.\n",fileName);

    s->fileName = fileName;

    s->start.x = startX;
    s->start.y = startY;
    s->start.rot = startRot;

    if ( s->odometryCorrection) {
      writeLog( "Odometry correction on.\n");
      s->start.x *= CORRECTION_FACTOR;
      s->start.y *= CORRECTION_FACTOR;
    }

    s->start = scriptPosition2RobotPosition( s->start);

    s->startTimeOfScript.hour = s->timeOfScript.hour;
    s->startTimeOfScript.minute = s->timeOfScript.minute;
    s->startTimeOfScript.second = s->timeOfScript.second;

    writeLog("# Script starts at %d:%d:%f with X:%.2f  Y:%.2f  Rot:%.2f\n",
	     s->startTimeOfScript.hour, s->startTimeOfScript.minute,
	     s->startTimeOfScript.second,
	     (float) s->start.x, (float) s->start.y, (float) s->start.rot);
    
    s->currentPos = s->start;

    s->fp = fp;

    s->newMarker = FALSE;

    s->markerCount = s->sonarCount = s->positionCount = s->laserCount = 0;

    s->bumpOccured = FALSE;

    s->newMapPosition = FALSE;

    gettimeofday( &(s->time), 0);
    s->startTimeVal = s->time;
    
    return(1);
  }
  else {
    fprintf(stderr, "Error %s is no script file.\n", fileName);
    fclose(fp);
    s->scriptType = WRONG_SCRIPT;
    return(0);
  }

}


int
closeScript(script *s) {
    if (s->scriptType == ORIG_SCRIPT || s->scriptType == CORR_SCRIPT) {
	fclose(s->fp);
	s->scriptType = WRONG_SCRIPT;
    }
    return(0);
}



static void
updateMovement( script *s, movement *delta, realPosition newPos)
{
  /* compute forward and sideward movement */
  *delta = movementBetweenRobotPoints(s->currentPos, newPos);
  
  delta->isNew = TRUE;
  delta->elapsedTime =
    scriptTimeDiff(s->timeOfScript, s->startTimeOfScript);
  
  /* switch to new position */
  s->positionCount++;
  s->currentPos = newPos;
  
  /* set the global measured position of the robot */
  measuredPosition = endPoint( measuredPosition, *delta);
  
  /* #define DUMP_STOP_POSITIONS  */
#ifdef DUMP_STOP_POSITIONS
    {
      static int alreadyStopped = FALSE;
      static float stopTime;

      if ( delta->forward == 0.0 && delta->sideward == 0.0 && delta->rotation == 0.0) {
	if ( alreadyStopped) {
	  realPosition pos = s->actionInfo->estimatedRobot.pos;
	  fprintf(stderr, "time %f\n", delta->elapsedTime - stopTime);
	  if ( delta->elapsedTime - stopTime > 30.0)
	    writeLog( "%f %f %f  #stop\n", 
		      pos.x, 
		      pos.y, 
		      pos.rot);
	  alreadyStopped = FALSE;
	} 
	else {
	  stopTime = delta->elapsedTime;
	  alreadyStopped = TRUE;
	}
      }
      else 
	alreadyStopped = FALSE;
    }
#endif
}

static void
updateSonar( script *s, sensing_PROXIMITY *sonars, float *sensor, int n)
{
  int i;

  s->sonarCount++;
  
  if ( robotType == PIONEER_ATRV || robotType == PIONEER_II || robotType == URBAN_ROBOT)
    for (i=0; i < n; i++)
      sonars->reading[i].dist = sensor[i];
  else
    for (i=0; i < n; i++)
      sonars->reading[i].dist = sensor[i] + ROB_RADIUS;
  
  sonars->elapsedTime =
    scriptTimeDiff(s->timeOfScript, s->startTimeOfScript);
  sonars->isNew = TRUE;
}


static void
updateImage( script* s,
	     sensing_VISION* image,
	     float* sensorValues,
	     int sizeX, int sizeY)
{
  unsigned int i, j;
  float *value = sensorValues;
  if (sizeX  == IMAGE_SIZE_X && sizeY == IMAGE_SIZE_Y){
    image->isNew = TRUE;
    image->sizeX = sizeX;
    image->sizeY = sizeY;


    for (j=0; j < image->sizeY; j++){
      for (i=0; i < image->sizeX; i++){
	image->pix[i][j] = round(*value++);
      }
      
    }
    s->imageCount++;
  }
}

static void
updateLaser( script* s,
	     sensing_PROXIMITY* frontLaser,
	     sensing_PROXIMITY* rearLaser,
	     float* sensorValues)
{
  int i;

  frontLaser->isNew = frontLaser->numberOfReadings > 0;
  rearLaser->isNew = rearLaser->numberOfReadings > 0;

  for (i=0; i < frontLaser->numberOfReadings &&  frontLaser->isNew; i++){
     frontLaser->reading[i].dist = sensorValues[i];
     frontLaser->isNew = (frontLaser->reading[i].dist >= 0);
  }
#ifdef DDD
  {
    static int scanNumber = 0;
    static positionWindow * posWin = NULL;
    static sampleSet samples;
    static int firstTime = TRUE;
    
    float avgAngle = 0.0;
    float avgDist = 0.0;
    int cnt = 0;
    int numReadings = frontLaser->numberOfReadings;
    realPosition robPos = {0,0,0};
    realPosition endPosition[360];
    movement tmpReading;
    probabilityGrid map;
    map.initialized = FALSE;
    tmpReading.sideward = tmpReading.rotation = 0.0;

    if ( firstTime) {

#define LIMITS 100

      firstTime = FALSE;
/*        samples = initializedSamples( 179, */
/*  				    -LIMITS, -LIMITS, */
/*  				    LIMITS, LIMITS); */
      
      samples = initializedSamples( 179,
				    0, 0, 
				    360, 360);
      
      
      posWin = createPositionWindow( &samples, "DELTAS", 500, 500, 1);

    }
    
    for (i=0; i < numReadings; i++){
      robPos.rot = normalizedAngle( frontLaser->reading[i].rot);
      tmpReading.forward = frontLaser->reading[i].dist;
      samples.sample[i].pos = endPoint( robPos, tmpReading);
      avgDist += frontLaser->reading[i].dist;
    }    
    avgDist /= 180;
    for (i=1; i < numReadings; i++){
      if (0) writeLog( "%f %f %d %d #%d_delta\n", endPosition[i].x - endPosition[i-1].x,
		endPosition[i].y - endPosition[i-1].y, i, scanNumber, scanNumber);
      samples.sample[i-1].pos.x = samples.sample[i].pos.x - samples.sample[i-1].pos.x;
      samples.sample[i-1].pos.y = samples.sample[i].pos.y - samples.sample[i-1].pos.y;
      if ( samples.sample[i-1].pos.x > 100 || samples.sample[i-1].pos.y > 100)
	samples.sample[i-1].pos.x = samples.sample[i-1].pos.y = 0;
      else if ( samples.sample[i-1].pos.x < -100 || samples.sample[i-1].pos.y < -100)
	samples.sample[i-1].pos.x = samples.sample[i-1].pos.y = 0;
      else {
	samples.sample[i-1].pos.x =  rad2Deg( normalizedAngle(atan2( samples.sample[i-1].pos.y, samples.sample[i-1].pos.x)));
	samples.sample[i-1].pos.y = samples.sample[i-1].pos.x;
	cnt ++;
	avgAngle += ((int) samples.sample[i-1].pos.x) % 90;
	writeLog( "%d %f %d %f #dir\n", i, samples.sample[i-1].pos.x,((int) samples.sample[i-1].pos.x) % 90,  avgDist);
      }
    }
    writeLog( "%f %f #extrdir\n", avgDist, avgAngle / cnt);
    EZX_SetColor(C_WHITE);
    EZX_FillRectangle(posWin->window, 0, 0, posWin->sizeX, posWin->sizeY);
    displayPositions( &samples, &map, NULL, posWin, FALSE, 0);

    scanNumber++;
  }
#endif  
  for (i=0; i < rearLaser->numberOfReadings &&  rearLaser->isNew; i++) {
    rearLaser->reading[i].dist = sensorValues[i+frontLaser->numberOfReadings];
    rearLaser->isNew = (rearLaser->reading[i].dist >= 0);
  }

  if (frontLaser->isNew)
    frontLaser->elapsedTime =
      scriptTimeDiff(s->timeOfScript, s->startTimeOfScript);

  if (rearLaser->isNew)
   rearLaser->elapsedTime =
    scriptTimeDiff(s->timeOfScript, s->startTimeOfScript);

  if (frontLaser->isNew || rearLaser->isNew)
    s->laserCount++;
}


bool
getReadings(char *line, float *sensor, int numberOfReadings)
{
  bool allRead = TRUE;
  int i = 0;
  char *p = line;
  
  while ((*p == ' ') && (*p != '\0') && (*p != '\n')) {
    p++;
  }


  while (allRead && (i < numberOfReadings)){
    allRead = (sscanf(p, "%e", &sensor[i++]) == 1); 
    if (allRead)
      { 
	do 	  
	  p++;
	while (*p != ' ' && (*p != '\0') && (*p != '\n'));
      }
    if (*p != ' ')
      allRead = FALSE;
  }
  if ( i != numberOfReadings) {
    fprintf( stderr, "Warning: not enough values: %d %d\n%s\n",
	     i, numberOfReadings, line);
    writeLog( "Warning: not enough values: %d %d\n", i, numberOfReadings);
  }

  if (0){
    fprintf(stderr, "%s\n", line);
    for (i = 0; i < numberOfReadings; i++)
      fprintf(stderr, "%f ", sensor[i]);
    fprintf(stderr, "\n");
  }
  return TRUE;
}



static void
processPositionReading(realPosition newPos, script *s, rawSensings *actualSensing)
{
  actualSensing->sonar.isNew = FALSE;
  actualSensing->frontLaser.isNew = FALSE;
  actualSensing->rearLaser.isNew = FALSE;

  /* This is a copy of the base status position. */
  /* Convert log angle to robot angle. */

    
  newPos = scriptPosition2RobotPosition( newPos);

  actualSensing->basePosition.x = newPos.x;
  actualSensing->basePosition.y = newPos.y;
  actualSensing->basePosition.rot = newPos.rot;

  updateMovement(s, &(actualSensing->delta), newPos);

  actualSensing->delta.isNew = TRUE;

  /* We compute the rotational velocity from the script. */
  {
#define NUMBER_OF_AVG 2
    static realPosition prevPos;
    static float prevTime = 0.0;
    static float rVel[NUMBER_OF_AVG];
    static int velPos = 0;
    static int numberOfVisits = 0;
    float rot = deg2Rad(fabs(prevPos.rot - newPos.rot));
    float tDiff = elapsedScriptTime - prevTime;
    int cnt;

    static float avgRVel = 0.0;

    if ( numberOfVisits++ == 0) {
      for (cnt = 0; cnt < NUMBER_OF_AVG; cnt++)
	avgRVel = 0.0;
    }

    if ( tDiff > 0.0) {
      rVel[velPos] = rot / tDiff;
    }

    if (numberOfVisits >= NUMBER_OF_AVG) {
      for (cnt = 0; cnt < NUMBER_OF_AVG; cnt++)
	avgRVel += rVel[cnt];
      avgRVel /= NUMBER_OF_AVG;
      actualSensing->rotVelocity = avgRVel;
    }

    prevPos = newPos;
    prevTime = elapsedScriptTime;
    velPos = (velPos + 1) % NUMBER_OF_AVG;
  }

}

int
getByteReadings( char *imageLine, float *sensor, int size){

  int i, cnt=0;
  char *origin;

  while( *imageLine == ' ')
    imageLine++;
  origin = imageLine;
  if (0) fprintf(stderr, "%s\n", imageLine);
  for (i = 0; i < size; i++){
    unsigned int val;
    sscanf(imageLine, "%2X", &val); 
    imageLine += 6;
    sensor[i] = val;
    if (0) {fprintf(stderr, "%2X", val);}
    cnt = (imageLine - origin);
  }
  if (0) fprintf(stderr, "\n");

  return (cnt == size * 6);
}


bool
readOrigScript(script* s, rawSensings* actualSensing)
{
  char line[BUFFLEN];
  char positionLine[BUFFLEN];
  char sonarLine[BUFFLEN];
  char laserLine[BUFFLEN];
  char imageLine[BUFFLEN];
  char *posMark, *sonarMark, *laserMark, *timeMark, *imageMark;
  int timeOffset;

  bool newPositionLine, newSonarLine, newLaserLine, newImageLine;
  float sensor[MAX_VALUESPERSCAN];
  bool eof, timeSkipped, returnValue;
  bool success = FALSE;
  int cnt;
  int markerCnt;
  bool newMarker = FALSE;

  realPosition newPos;

  struct timeval currentTime;

  scriptTime newTimeOfScript;

  float elapsedTime;
  float realTimeExpired;

  static bool firstTime = TRUE;

  if (firstTime) {
    firstTime = FALSE;
    gettimeofday( &(s->time), 0);
    s->startTimeVal = s->time;
  }

  if (s->scriptType == ORIG_SCRIPT) {
    posMark = ORIG_SCRIPTPOSMARK;
    sonarMark = ORIG_SCRIPTSONARMARK;
    laserMark = ORIG_SCRIPTLASERMARK;
    timeMark = ORIG_TIMEMARK;
    timeOffset =  ORIG_TIMEOFFSET;
  }
  else{
    posMark = NEW_SCRIPTPOSMARK;
    sonarMark = NEW_SCRIPTSONARMARK;
    laserMark = NEW_SCRIPTLASERMARK;
    timeMark = NEW_TIMEMARK;
    timeOffset =  NEW_TIMEOFFSET;
  }
  imageMark = NEW_IMAGE_MARK;



  /* skip to next sensing line */

/*   fprintf(stderr,"# Sc: %d    Pc: %d\n", s->sonarCount,s->positionCount);  */

  actualSensing->delta.isNew = actualSensing->sonar.isNew =
    actualSensing->frontLaser.isNew = actualSensing->rearLaser.isNew =
    actualSensing->image.isNew =  newPositionLine = newSonarLine =
    newLaserLine = newImageLine = eof = timeSkipped = s->newMarker = FALSE;
  
  returnValue = TRUE;
  
  cnt = 0;
  
  gettimeofday(&currentTime, 0);
  
  if (s->timeFactor > 0.0) {
    /* In the real time mode, even nonrelevant time counts since we want to
     * synchronize several LOCALIZE processes. */
    if ( s->realTime) {
      realTimeExpired = timeDiff(&( currentTime), &(s->startTimeVal)) / s->timeFactor;
      timeSkipped = elapsedScriptTime >= realTimeExpired;
    }
    else {
      realTimeExpired = fMax( timeDiff(&( currentTime), &(s->startTimeVal)) - nonRelevantTime, 0.0) / s->timeFactor;
    }
  }
  else {
    /* if timeFactor <= 0 we take every reading */
    realTimeExpired = 0.0;
  }

  nonRelevantTime = 0.0;

  /* Skip to the next reading line after the elapsed time.
     During this store the latest readings. */
  
  while (!eof && !timeSkipped){
    if (!(eof = (fgets(line,BUFFLEN,s->fp) == NULL))){
      /* #define CORRECTSCRIPT 1 */
#ifdef CORRECTSCRIPT
      {
	static FILE *newScriptFp;
	static bool firstTimeCorr = TRUE;
	realPosition mapPos;
	correctionParameter corrParams = s->actionInfo->correctionParam;

	if (firstTimeCorr){
	  firstTimeCorr = FALSE;
	  newScriptFp = fopen("newscript", "wt");
	}

	if (positionReadingLine(line)){
	  if (sscanf(&line[strlen(ORIG_SCRIPTPOSMARK)],"%f %f %f",
		     &newPos.x, &newPos.y, &newPos.rot) == 3) {

	    robotCoordinates2MapCoordinates( newPos.x, newPos.y, newPos.rot,
					     corrParams.x,
					     corrParams.y,
					     corrParams.rot,
					     corrParams.type,
					     &mapPos.x, &mapPos.y, &mapPos.rot);
	    mapPos.rot =  90 - rad2Deg( mapPos.rot);

	    fprintf(newScriptFp, "%s %f %f %f\n",
		    ORIG_SCRIPTPOSMARK,
		    mapPos.x,
		    mapPos.y,
		    mapPos.rot);
	      }
	}
	else
	  fprintf(newScriptFp, "%s\n", line);
      }
#endif

      if (0) writeLog( "%s", line);

      if ((s->scriptType == ORIG_SCRIPT && positionReadingLine(line))
	  ||
	  (s->scriptType == NEW_SCRIPT && newPositionReadingLine(line))){
	strcpy(positionLine, line);
	newPositionLine = TRUE;
	cnt++;
	/* 	  fprintf(stderr, "# new position during skip\n"); */
      }
      else if ((s->scriptType == ORIG_SCRIPT && sonarReadingLine(line)) ||
	       (s->scriptType == NEW_SCRIPT && newSonarReadingLine(line))){
	strcpy(sonarLine, line);
	newSonarLine = TRUE;
	cnt++;
	/* 	  fprintf(stderr, "# new sonar reading during skip\n"); */
      }
      else if ((s->scriptType == ORIG_SCRIPT && laserReadingLine(line)) ||
	       (s->scriptType == NEW_SCRIPT && newLaserReadingLine(line))) {
	strcpy(laserLine, line);
	newLaserLine = TRUE;
	cnt++;
	/* 	fprintf(stderr, "# new laser reading during skip\n"); */
      }
      else if (markerReadingLine(line)){
	if (strlen(line) > strlen(ORIG_SCRIPTMARKERMARK)){
	  if (sscanf(&line[strlen(ORIG_SCRIPTMARKERMARK)], "%d", &markerCnt))
	    newMarker = TRUE;
	  else
	    newMarker = FALSE;
	}
	else
	  newMarker = FALSE;
      }
      else if (newImageReadingLine(line)){
	strcpy(imageLine, line);
	newImageLine = TRUE;
	cnt++;
      }
      else if (bumperReadingLine(line)){
	if ( sscanf(&line[strlen(SCRIPTBUMPERMARK)],
		    "%f %f %f", &(s->bumpForward), &(s->bumpSideward), &(s->bumpRot)) < 3)
	  writeLog( "Wrong Bumper line: %s", line);
	else
	  s->bumpOccured = TRUE;
      }
      else if (detectionReadingLine(line)){
	char observer[50], detected[50];
	float dist, distUncertainty, angle, angleUncertainty, currentRotSpeed;
	int detectionFlag;
	if ( sscanf( &line[strlen(SCRIPT_DETECTION_MARK)],
		     "%s %s %f %f %f %f %f %d", observer, detected,
		     &dist, &distUncertainty, &angle, &angleUncertainty, &currentRotSpeed, &detectionFlag) < 8) {
	  fprintf( stderr, "Wrong Detection line: %s", line);
	  fprintf( stderr, "Detection: %s %s %f %f %f %f %f %d\n",
		   observer, detected,
		   dist, distUncertainty, angle, angleUncertainty, currentRotSpeed,
		   detectionFlag);
	}
	else {
	  scriptTime t = s->timeOfScript;
	  struct timeval detectionTime;
	  detectionTime.tv_sec =
	    (long) t.hour * 3600 + t.minute * 60 + floor(t.second);
	  detectionTime.tv_usec = (t.second - floor(t.second)) *
	    1000000;

	  fprintf( stderr, "%f %f ", dist, angle);
	  writeLog( "%f %f ", dist, angle);

#define ANGLE_UNCERTAINTY 2.2
#define DISTANCE_UNCERTAINTY 48.26

	  dist = fMax( 0.0, dist + randomGauss() * DISTANCE_UNCERTAINTY);  
	  angle += randomGauss() * ANGLE_UNCERTAINTY; 

	  fprintf( stderr, "%s: %f %f %d #DETECTION\n", detected, dist, angle, detectionFlag);
	  writeLog( "%s: %f %f %d #DETECTION\n", detected, dist, angle, detectionFlag);

	  if ( 1 || detectionFlag > 0)
	    sendDetectionToMultiLocalize( observer, detected,
					  dist, distUncertainty,
					  angle, angleUncertainty,
					  currentRotSpeed, detectionTime,
					  detectionFlag);
	}
      }
      else if (mapPositionReadingLine(line)){
	realPosition robPos;
	if ( sscanf(&line[strlen(SCRIPT_MAP_POSITION_MARK)],
		    "%f %f %f", &(s->mapPosition.x),
		    &(s->mapPosition.y), &(s->mapPosition.rot)) < 3)
	  writeLog( "Wrong map position line: %s", line);
	else {
	  s->newMapPosition = TRUE;
	}
      }
      else if (cnt > 0 && ((s->scriptType == ORIG_SCRIPT
			    && timeReadingLine(line))
			   ||
			   (s->scriptType == NEW_SCRIPT
			    && newTimeReadingLine(line)))) {
	if (s->scriptType == ORIG_SCRIPT){
	  if (sscanf(&line[strlen(ORIG_TIMEMARK)+9], "%d:%d:%f",
		     &newTimeOfScript.hour,
		     &newTimeOfScript.minute,
		     &newTimeOfScript.second) == 3){
	    elapsedScriptTime = scriptTimeDiff(newTimeOfScript, s->startTimeOfScript);
	    timeSkipped = elapsedScriptTime >= realTimeExpired;
	  }
	}
	else if (s->scriptType == NEW_SCRIPT){
	  long secs, usecs;
	  if (sscanf(&line[strlen(NEW_TIMEMARK)+NEW_TIMEOFFSET], "%ld %ld",
		     &secs, &usecs) == 2){
	    newTimeOfScript.hour = (secs % 86400) / 3600;
	    newTimeOfScript.minute = (secs % 3600) / 60;
	    newTimeOfScript.second = (secs % 60) + usecs/1000000.0;
	    /* fprintf(stderr, "**** Time: %2d:%2d:%2.2f\n ", newTimeOfScript.hour,
	       newTimeOfScript.minute, newTimeOfScript.second); */
	    elapsedScriptTime = scriptTimeDiff(newTimeOfScript, s->startTimeOfScript);
	    timeSkipped = elapsedScriptTime >= realTimeExpired;
	  }
	}
      }
    }
  }
  
  if (eof) {
    fprintf(stderr, "# End of file reached\n");
    getchar();
    return(FALSE);
  }
  
  /* Real time behaviour. LOCALIZE has to wait until the next sensor information
   * has arrived. We don't want to wait if the position of the robot does not change. */
  if (s->realTime) {
#ifdef DONT_SLEEP_IF_ROBOT_DOESNT_MOVE
    static bool positionHasChanged = FALSE;
    
    if ( newPositionLine) {
      realPosition newPos;
      if (sscanf(&positionLine[strlen(posMark)],"%f %f %f",
		 &newPos.x, &newPos.y, &newPos.rot) == 3){
	newPos = scriptPosition2RobotPosition( newPos);
	positionHasChanged = ( newPos.x != actualSensing->basePosition.x ||
			       newPos.y != actualSensing->basePosition.y ||
			       newPos.rot != actualSensing->basePosition.rot);
      }
    }
    if ( positionHasChanged) {
      usleep((scriptTimeDiff(newTimeOfScript, s->startTimeOfScript) - realTimeExpired)*1000000);
    }
#else
    /* If the time to sleep is too large, then we just keep on integrating. */
    if ( (( elapsedScriptTime - realTimeExpired) * s->timeFactor) < 0.2)
      swallowStatusReports( (elapsedScriptTime - realTimeExpired) * s->timeFactor);
#endif
  }
    
  s->timeOfScript.hour = newTimeOfScript.hour;
  s->timeOfScript.minute = newTimeOfScript.minute;
  s->timeOfScript.second = newTimeOfScript.second;

  gettimeofday(&(s->time), 0);


  if (newPositionLine){
    /*     fprintf(stderr, "Position : %s\n", positionLine); */

    if (sscanf(&positionLine[strlen(posMark)],"%f %f %f",
		&newPos.x, &newPos.y, &newPos.rot) == 3){
      
      if ( s->odometryCorrection) {
	
	/* Correction of wrong dead reckoning. */
	newPos.x *= CORRECTION_FACTOR;
	newPos.y *= CORRECTION_FACTOR;
      }
	  
      processPositionReading(newPos, s, actualSensing);
    }
    else {
      fprintf(stderr, "Error 1 while scanning script %s\n",s->fileName);
      fprintf(stderr, "Cannot scan:\n%s\n", line);
      actualSensing->delta.isNew = FALSE;
      returnValue = FALSE;
    }
  }

  if (returnValue && newMarker){
    s->newMarker = newMarker;
    s->markerCount = markerCnt;
    fprintf(stderr, "# Found marker: %d\n", markerCnt);
  }

  if (returnValue && newSonarLine){
    int cnt;
    /*     fprintf(stderr, "Sonar: %s\n", sonarLine); */

    
    if ( robotType == PIONEER_ATRV || robotType == PIONEER_II || robotType == URBAN_ROBOT)
      success = sscanf( &sonarLine[strlen(sonarMark)],
			"%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e",
			&sensor[0],&sensor[1],&sensor[2],&sensor[3],
			&sensor[4],&sensor[5],&sensor[6],&sensor[7],
			&sensor[8],&sensor[9],&sensor[10],&sensor[11],
			&sensor[12],&sensor[13],&sensor[14],&sensor[15],
			&sensor[16],&sensor[17],&sensor[18],&sensor[19],
			&sensor[20],&sensor[21],&sensor[22],&sensor[23])
	== MAX_NUMBER_OF_SONARS;
    else
      success = sscanf(&sonarLine[strlen(sonarMark)],
		       "%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e",
		       &sensor[23],&sensor[22],&sensor[21],&sensor[20],
		       &sensor[19],&sensor[18],&sensor[17],&sensor[16],
		       &sensor[15],&sensor[14],&sensor[13],&sensor[12],
		       &sensor[11],&sensor[10],&sensor[ 9],&sensor[ 8],
		       &sensor[ 7],&sensor[ 6],&sensor[ 5],&sensor[ 4],
		       &sensor[ 3],&sensor[ 2],&sensor[ 1],&sensor[ 0])
	== MAX_NUMBER_OF_SONARS;
    
    if ( success) {
      
      for ( cnt = 0; cnt < MAX_NUMBER_OF_SONARS; cnt++)
	sensor[cnt] = fMax( 0.0, sensor[cnt] - s->distanceOffset);
      
      updateSonar(s, &(actualSensing->sonar), sensor, numberOfSonars);
      
      actualSensing->sonar.isNew = TRUE;
    }
    else {
      fprintf(stderr, "Error 2 while scanning script %s\n",s->fileName);
      fprintf(stderr, "Cannot scan:\n%s\n",line);
      actualSensing->sonar.isNew = FALSE;
      returnValue = FALSE;
    }
  }
  if (returnValue && newImageLine){
    int sizeX, sizeY;
    unsigned int markLength = strlen(imageMark);

    if (sscanf( &imageLine[markLength], "%d %d", &sizeX, &sizeY)){
      
      /* Set the mark behind the :. */
      while ( (markLength < strlen(imageLine))
	      && (imageLine[markLength++] != ':'))
	;

      {
	bool gotReadings = FALSE; 
	if ( strlen(&imageLine[markLength]) < (unsigned int) 4 * sizeX * sizeY + 1){
	  if ( getReadings( &imageLine[markLength],
			    sensor, sizeX * sizeY))
	    gotReadings = TRUE;
	}
	else {
	  if ( getByteReadings( &imageLine[markLength],
				sensor, sizeX * sizeY))
	    gotReadings = TRUE;
	}

	if (gotReadings)
	  updateImage( s,
		       &(actualSensing->image),
		       sensor,
		       sizeX,
		       sizeY);
	else {
	  fprintf(stderr, "Error 3 while scanning script %s\n",s->fileName);
	  fprintf(stderr, "Cannot scan:\n%s\n",line);
	  actualSensing->image.isNew = FALSE;
	  returnValue = FALSE;
	}
      }
    }
  }
  
  if (returnValue && newLaserLine){

    /*     fprintf(stderr, "Laser: %s\n", laserLine);    */
    int frontSize, rearSize;
    unsigned int markLength = strlen(laserMark);
    int numberOfInts = sscanf( &laserLine[markLength], "%d %d:", &frontSize, &rearSize);

    if ( numberOfInts == 1)
      frontSize = rearSize = frontSize / 2;
    actualSensing->frontLaser.numberOfReadings = frontSize;
    actualSensing->rearLaser.numberOfReadings = rearSize;

    /* Set the mark behind the :. */
    while ( (markLength < strlen(laserLine))
	    && (laserLine[markLength++] != ':'))
      ;

    if ( getReadings( &laserLine[markLength],
		      sensor,
		      actualSensing->frontLaser.numberOfReadings +
		      actualSensing->rearLaser.numberOfReadings)) {
      updateLaser( s,
		   &(actualSensing->frontLaser),
		   &(actualSensing->rearLaser),
		   sensor);

    }
    else {
      fprintf(stderr, "Error 3 while scanning script %s\n",s->fileName);
      fprintf(stderr, "Cannot scan:\n%s\n",line);
      actualSensing->frontLaser.isNew = FALSE;
      actualSensing->rearLaser.isNew = FALSE;
      returnValue = FALSE;
    }
  }

  return returnValue;
}



bool
readScript(script* s, rawSensings* actualSensing)
{
  if (s->scriptType == ORIG_SCRIPT || s->scriptType == NEW_SCRIPT)
    return readOrigScript( s, actualSensing);
  else {
    fprintf( stderr, "Sorry corr script not implemented any longer.\n");
    fprintf( stderr, "Removed on Fri Jul  5 13:00:02 MET DST 1996\n");
    writeLog( "Sorry corr script not implemented any longer.\n");
    writeLog( "Removed on Fri Jul  5 13:00:02 MET DST 1996\n");
    closeLogAndExit(1);
    return FALSE;
  }
}


realPosition
startPosition(char *fileName, char *mapFileName, char *scriptFileName) {
    realPosition pos={0.0, 0.0, 0.0}, pos1;
    FILE *fp;
    bool found = FALSE;
    char tmpMapFileName[80], tmpScriptFileName[80], line[BUFFLEN];

    /* If no script file is given the position has to be estimated online. */
    if ( scriptFileName == NULL)
      scriptFileName = ONLINE_POSITION_MARKER;

    /* try to open file */
    if ((fp = fopen(fileName,"rt")) == NULL) {
	fprintf(stderr,"# Warning: Could not open file '%s'!\n",fileName);
	return(pos);
    }

    writeLog( "# Scanning %s:", fileName);
    fprintf(stderr, "# Scanning %s:", fileName);

    while (fgets(line,BUFFLEN,fp) != NULL &&
	   (sscanf(line, "%s %s %f %f %f", tmpMapFileName,
		  tmpScriptFileName, &pos1.x, &pos1.y, &pos1.rot) == 5) &&
	   !found)
	{
	    if ( ( strncmp(mapFileName, tmpMapFileName,
			strlen(mapFileName)) == 0) &
		( strncmp(scriptFileName, tmpScriptFileName,
			strlen(scriptFileName)) == 0)) {
		pos = pos1;
		found = TRUE;
	    }
	}

    fclose(fp);

    if (found) {
	pos.rot = deg2Rad(pos.rot);
	writeLog(
		 " start position is (%.2f, %.2f, %.2f)\n",
		 pos.x, pos.y, pos.rot);
	fprintf(stderr,
		" start position is (%.2f, %.2f, %.2f)\n",
		pos.x, pos.y, pos.rot);
    }
    else {
      writeLog(
	       " not found, assuming (%.2f, %.2f, %.2f)\n",
	       pos.x, pos.y, pos.rot);
      fprintf(stderr,
	      " not found, assuming (%.2f, %.2f, %.2f)\n",
	      pos.x, pos.y, pos.rot);
    }

    return(pos);
}



#endif
