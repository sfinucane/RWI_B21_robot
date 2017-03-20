
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/pantilt/panTilt_interface.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:26:54 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: panTilt_interface.c,v $
 * Revision 1.1  2002/09/14 15:26:54  rstone
 * *** empty log message ***
 *
 * Revision 1.6  1998/03/05 10:58:03  haehnel
 * only for Bonn (ifdef UNIBONN)
 *
 * Revision 1.5  1997/07/17 17:31:50  tyson
 * Added wiggle, square, Changes, fixed bugs, added features.  See the new Changes file for details
 *
 * Revision 1.4  1997/03/04 04:29:49  thrun
 * Sending periodic status update now. Also, removed some initial commands
 * that Ty just put in - since they make our pantilt unit sound
 * horrible (see panTilt_interface.c)
 *
 * Revision 1.3  1997/02/25 18:12:42  tyson
 * client lib for PANTILT and lots of little stuff
 *
 * Revision 1.2  1996/10/18 00:30:02  ws
 * Fixed two bugs (found by Stefan Waldherr)
 *
 * Revision 1.1.1.1  1996/09/22 16:46:29  rhino
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



#ifdef VMS
#include "vms.h"
#include "vmstime.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "Common.h"
/*#include "libc.h"*/
/*#include "LIB-math.h"*/
#define DECLARE_PANTILT_VARS
#include "panTilt_interface.h"
#include "devUtils.h"

/*****************************************************************************
 * Global variables 
 *****************************************************************************/
char failureString[80];

static int statusOK;
static float responseValue;
static char *waitForCommand = NULL;
static int maxPanPos, minPanPos, maxTiltPos, minTiltPos;
static int maxPanVel, minPanVel, maxTiltVel, minTiltVel;
static float panDegreesPerCount, tiltDegreesPerCount;

extern float global_pan_correction;

float global_pan_correction = 0.0;

#define TIMEOUT 1 /* In seconds */

      
DEGREES _180_to_180 (DEGREES value)
{
  for(;;) {
    if (value > 180.0)
      value -= 360.0;
    else if (value <= -180.0)
      value += 360.0;
    else
      return (value);
  }
}


#define NO_ARG -99999

/*****************************************************************************
 *
 * FUNCTION: int writeCommand
 *
 * DESCRIPTION: Write a command to the pan/tilt unit
 *
 * INPUTS:
 *
 * OUTPUTS: Returns TRUE if successful write, otherwise FALSE
 *
 * HISTORY: Merger of function by same name in rwibase_interface.c
 *          and function written by Conrad Poelman for the pan/tilt unit
 *
 *****************************************************************************/

static BOOLEAN writeCommand (char *command, int arg)
{
  char buffer[DEFAULT_LINE_LENGTH];
  
  if (panTilt_device.dev.fd == -1) {
    fprintf (stderr, "WriteCommand: Error: device is not initialized\n");
    return TRUE;
  }
  
  if (arg != NO_ARG) {
    sprintf (buffer, " %s%d ", command, arg);
  } else {
    sprintf (buffer, " %s ", command);
  }

#if 0  
  fprintf(stderr, "%s:%6d:%s() - [%s]\n",
	  __FILE__, __LINE__, __FUNCTION__, buffer);
#endif
  return ((int) writeN(&(panTilt_device.dev), buffer, strlen(buffer)));
}

BOOLEAN panTiltQuery (char *command, int arg)
{
  waitForCommand = command;
  writeCommand(command, arg);
  return WaitForResponse(&(panTilt_device.dev), &statusOK, TIMEOUT);
}

void panTiltParse (char *line)
{
  int i;
  char *restLine;

#if 0
  fprintf(stderr, "%s:%6d:%s() - line = [%s:%s]\n",
	  __FILE__, __LINE__, __FUNCTION__, waitForCommand, line);
#endif  

#if 1
  restLine = line;

  while (*restLine) {

    if (waitForCommand &&
	strncmp(waitForCommand, restLine, strlen(waitForCommand)) == 0) {

      statusOK = 1;
      restLine += strlen(waitForCommand);

      while (restLine[0] != '\0' && restLine[0] != '*') {
	restLine++;
      }

      while (restLine[0] != '\0') {
	if (isdigit(restLine[0]) ||
	    restLine[0] == '-' ||
	    restLine[0] == '.') {
	    responseValue = atof(&(restLine[0]));
	    break;
	  }
	  restLine++;
	}
    }

    else if (*restLine == '!') {
      if (restLine[1] == 'T') {
	strcpy(failureString, "Tilt axis limit reached");
      } else if (restLine[1] == 'P') {
	strcpy(failureString, "Pan axis limit reached");
      } else {
	strcpy(failureString, &(restLine[2]));
      }

      fprintf(stderr, "%s:%d:%s() - %s\n",
	      __FILE__, __LINE__, __FUNCTION__, failureString);

      restLine += 2;
    }

    else if (*restLine == '*') {
      if (!waitForCommand) statusOK = 1;
      restLine++;
    }

    else {
      restLine++;
    }
  }

#else

  if (waitForCommand == NULL ||
      strncmp(waitForCommand, line, strlen(waitForCommand)) == 0) {
    restLine = strpbrk(line, "*!");
    if (!restLine) {
      /* Handle error in parsing */
      exit (-1);
    } else {
      statusOK = (restLine[0] == '*');

      if (statusOK) {
	i = 1;
	while (restLine[i] != '\0') {
	  if (isdigit(restLine[i]) ||
	      restLine[i] == '-' ||
	      restLine[i] == '.') {
	    responseValue = atof(&(restLine[i]));
	    break;
	  }
	  i++;
	}
      } else if (restLine[1] == 'T') {
	strcpy(failureString, "Tilt axis limit reached");
      } else if (restLine[1] == 'P') {
	strcpy(failureString, "Pan axis limit reached");
      } else {
	strcpy(failureString, &(restLine[2]));
      }
    }
  }
#endif

}

static void initPanTilt(void)
{
#ifndef UNIBONN
  writeCommand("R", NO_ARG);
  sleep(30);
#endif
  writeCommand("I", NO_ARG);
  writeCommand("I", NO_ARG);
  writeCommand("EE", NO_ARG);	/* Enable echo */
  writeCommand("FT", NO_ARG);	/* Feedback Terse */
  writeCommand("LE", NO_ARG);	/* Enable position limits */
  if (panTiltQuery("PX", NO_ARG)) maxPanPos = (int)responseValue;
  if (panTiltQuery("PN", NO_ARG)) minPanPos = (int)responseValue;
  if (panTiltQuery("PU", NO_ARG)) maxPanVel = (int)responseValue;
  if (panTiltQuery("PL", NO_ARG)) minPanVel = (int)responseValue;
  if (panTiltQuery("PR", NO_ARG)) panDegreesPerCount = responseValue/3600.0;
  
  if (panTiltQuery("TX", NO_ARG)) maxTiltPos = (int)responseValue;
  if (panTiltQuery("TN", NO_ARG)) minTiltPos = (int)responseValue;
  if (panTiltQuery("TU", NO_ARG)) maxTiltVel = (int)responseValue;
  if (panTiltQuery("TL", NO_ARG)) minTiltVel = (int)responseValue;
  if (panTiltQuery("TR", NO_ARG)) tiltDegreesPerCount = responseValue/3600.0;
  /*  
  panTiltQuery("PL", minPanVel);
  panTiltQuery("PB", minPanVel);

  panTiltQuery("TL", minTiltVel);
  panTiltQuery("TB", minTiltVel);
  */
  /* Move in "regular" power mode. */
  writeCommand("PMR", NO_ARG);
  writeCommand("TMR", NO_ARG);
  
  /* Set low hold power mode to save power. */
  writeCommand("PHL", NO_ARG);
  writeCommand("THL", NO_ARG);
}

/*****************************************************************************
 *
 * FUNCTION: BOOLEAN PANTILT_init(void);
 *
 * DESCRIPTION: Open the panTilt device and initialize it.
 *
 * INPUTS:
 * PANTILT_PTR init - structure describing panTilt device.
 *
 * OUTPUTS: Returns true if device opened successfully, false otherwise.
 *
 * HISTORY:
 *
 *****************************************************************************/
BOOLEAN PANTILT_init(void)
{
  devInit();

  if (panTilt_device.dev.use_simulator) {
    /* set up to use the simulator */
    panTilt_device.dev.fd = connectSimulator(&panTilt_device.dev);
  } else {
    /* code to open the real device here. */
    connectTotty(&panTilt_device.dev);
  }
  if( panTilt_device.dev.fd == -1)
    return FALSE;
  else {
    m_setparms(panTilt_device.dev.fd, "x", "NONE", "8", 0, 0);
    connectDev(&panTilt_device.dev);
    initPanTilt();
    return TRUE;
  }
}

/*****************************************************************************
 *
 * FUNCTION: void PANTILT_outputHnd(int fd, long chars_available)
 *
 * DESCRIPTION: Handles output from the pan tilt device.
 *
 * INPUTS:
 * PANTILT_PTR init - structure describing panTilt device.
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/
void PANTILT_outputHnd(int fd, long chars_available)
{
  static LINE_BUFFER_PTR lineBuffer = NULL;
  if (!lineBuffer) lineBuffer = createLineBuffer(80, '\n', panTiltParse);
  processOutput(&(panTilt_device.dev), lineBuffer, chars_available);
}

/*****************************************************************************
 *
 * FUNCTION: BOOLEAN PANTILT_terminate(void);
 *
 * DESCRIPTION: Close the panTilt device.
 *
 * INPUTS:
 *
 * OUTPUTS: 
 *
 * HISTORY:
 *
 *****************************************************************************/

void PANTILT_terminate(void)
{
  close(panTilt_device.dev.fd);
  panTilt_device.dev.fd = -1;
}

/* Set the pan and tilt velocity, in degrees per second */
BOOLEAN PANTILT_setVelocity (float panVelocity, float tiltVelocity)
{
  int panVel, tiltVel;
  
  panVel = (int)(0.5 + panVelocity/panDegreesPerCount);
  tiltVel = (int)(0.5 + tiltVelocity/tiltDegreesPerCount);
  if (panVel < minPanVel || panVel > maxPanVel ||
      panVel < minPanVel || panVel > maxPanVel) {
    return FALSE;
  } else {
    return (panTiltQuery("PS", panVel) && panTiltQuery("TS", tiltVel));
  }
}

/* Set the pan and tilt acceleration */
BOOLEAN PANTILT_setAcceleration (float panAcceleration, float tiltAcceleration)
{
  int panVel, tiltVel;
  static int lastPanVel = 0, lastTiltVel= 0;
  static BOOLEAN last_ret = 0;
  panVel = (int) panAcceleration;
  tiltVel = (int) tiltAcceleration;
  if (panVel != lastPanVel || lastTiltVel != tiltVel){
    lastTiltVel = tiltVel;
    lastPanVel = panVel;
    last_ret = (panTiltQuery("PA", panVel) && panTiltQuery("TA", tiltVel));
  }
  return last_ret;
}

/* Return the position of the pan and tilt axes, in degrees */
BOOLEAN PANTILT_position (DEGREES *panAngle, DEGREES *tiltAngle)
{
  if (!panTiltQuery("PP", NO_ARG)) return FALSE;
  *panAngle = responseValue * panDegreesPerCount;
  *panAngle += global_pan_correction;
  
  if (!panTiltQuery("TP", NO_ARG)) return FALSE;
  *tiltAngle = responseValue * tiltDegreesPerCount;
  
  return TRUE;
}

/* Move the pan axis to the given (absolute) angle, in degrees */
BOOLEAN PANTILT_pan (DEGREES panAngle)
{
  int panPos;
  
  panAngle -= global_pan_correction;
  panAngle = _180_to_180(panAngle);

  panPos = (int)(panAngle/panDegreesPerCount);
  if (panPos < minPanPos || panPos > maxPanPos) {
    return FALSE;
  } else {
    writeCommand("I", NO_ARG);
#ifdef UNIBONN
    return (panTiltQuery("PP", panPos) && panTiltQuery("A", NO_ARG));
#else
    return (panTiltQuery("PP", panPos));
#endif
  }
}

/* Move the pan axis relative to its current angle, in degrees */
BOOLEAN PANTILT_panRelative (DEGREES panAngle)
{
  int panPosRel, panPos;
  
  panPosRel = (int)(panAngle/panDegreesPerCount);
  if (!panTiltQuery("PP", NO_ARG)) return FALSE;
  panPos = responseValue + panPosRel;
  if (panPos < minPanPos || panPos > maxPanPos) {
    return FALSE;
  } else {
    writeCommand("I", NO_ARG);
    return (panTiltQuery("PO", panPosRel) && panTiltQuery("A", NO_ARG));
  }
}

/* Move the tilt axis to the given (absolute) angle, in degrees */
BOOLEAN PANTILT_tilt (DEGREES tiltAngle)
{
  int tiltPos;
  
  tiltAngle = _180_to_180(tiltAngle);

  tiltPos = (int)(tiltAngle/tiltDegreesPerCount);
  if (tiltPos < minTiltPos || tiltPos > maxTiltPos) {
    return FALSE;
  } else {
    writeCommand("I", NO_ARG);
#ifdef UNIBONN
    return (panTiltQuery("TP", tiltPos) && panTiltQuery("A", NO_ARG));
#else
    return (panTiltQuery("TP", tiltPos));
#endif
  }
}

/* Move the tilt axis relative to its current angle, in degrees */
BOOLEAN PANTILT_tiltRelative (DEGREES tiltAngle)
{
  int tiltPosRel, tiltPos;
  
  tiltPosRel = (int)(tiltAngle/tiltDegreesPerCount);
  if (!panTiltQuery("TP", NO_ARG)) return FALSE;
  tiltPos = responseValue + tiltPosRel;
  if (tiltPos < minTiltPos || tiltPos > maxTiltPos) {
    return FALSE;
  } else {
    writeCommand("I", NO_ARG);
    return (panTiltQuery("TO", tiltPosRel) && panTiltQuery("A", NO_ARG));
  }
}

/* Move the pan and tilt axes simultaneously to the 
   given (absolute) angle, in degrees */
BOOLEAN PANTILT_move (DEGREES panAngle, DEGREES tiltAngle)
{
  int panPos, tiltPos;
  
  panAngle -= global_pan_correction;
  panAngle = _180_to_180(panAngle);

  tiltAngle = _180_to_180(tiltAngle);

  panPos = (int)(panAngle/panDegreesPerCount);
  tiltPos = (int)(tiltAngle/tiltDegreesPerCount);
  if (panPos < minPanPos || panPos > maxPanPos ||
      tiltPos < minTiltPos || tiltPos > maxTiltPos) {
    return FALSE;
  } else {
#ifdef UNIBONN
    writeCommand("S", NO_ARG);
    return (panTiltQuery("PP", panPos) && panTiltQuery("TP", tiltPos)
	    && panTiltQuery("A", NO_ARG));
#else
    writeCommand("I", NO_ARG);
    return (panTiltQuery("PP", panPos) && panTiltQuery("TP", tiltPos));
#endif
  }
}

/* Move the pan and tilt axes simultaneously to positions relative to
   the current positions, in degrees */
BOOLEAN PANTILT_moveRelative (DEGREES panAngle, DEGREES tiltAngle)
{
  int panPosRel, panPos, tiltPosRel, tiltPos;
  
  panPosRel = (int)(panAngle/panDegreesPerCount);
  if (!panTiltQuery("PP", NO_ARG)) return FALSE;
  panPos = responseValue + panPosRel;
  tiltPosRel = (int)(tiltAngle/tiltDegreesPerCount);
  if (!panTiltQuery("TP", NO_ARG)) return FALSE;
  tiltPos = responseValue + tiltPosRel;
  
  if (panPos < minPanPos || panPos > maxPanPos ||
      tiltPos < minTiltPos || tiltPos > maxTiltPos) {
    return FALSE;
  } else {
    writeCommand("S", NO_ARG);
    return (panTiltQuery("PO", panPosRel) && panTiltQuery("TO", tiltPosRel) &&
	    panTiltQuery("A", NO_ARG));
  }
}

/* Reset the pan and tilt axes, and restore the stored settings */
BOOLEAN PANTILT_reset(void)
{
  fprintf(stderr, "Resetting Pan/Tilt head: Please wait\n");
  writeCommand("DR", NO_ARG);	/* Restore stored defaults */
  return panTiltQuery("R", NO_ARG);
}

/* 
 * Return the limits of the pan and tilt axes (min/max angles, max velocity)
 */
void PANTILT_limits (DEGREES *minPanAngle, DEGREES *maxPanAngle,
		     DEGREES *minTiltAngle, DEGREES *maxTiltAngle,
		     float *maxPanVelocity, float *maxTiltVelocity)
{
  *minPanAngle = minPanPos*panDegreesPerCount + global_pan_correction;
  *maxPanAngle = maxPanPos*panDegreesPerCount + global_pan_correction;
  *minTiltAngle = minTiltPos*tiltDegreesPerCount;
  *maxTiltAngle = maxTiltPos*tiltDegreesPerCount;
  *maxPanVelocity = maxPanVel*panDegreesPerCount;
  *maxTiltVelocity = maxTiltVel*tiltDegreesPerCount;
}
