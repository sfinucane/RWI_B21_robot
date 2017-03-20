
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/detection-old/stuck.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:01 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: stuck.c,v $
 * Revision 1.1  2002/09/14 20:45:01  rstone
 * *** empty log message ***
 *
 * Revision 1.6  1999/01/27 16:33:43  fox
 * Nothing special.
 *
 * Revision 1.5  1998/09/12 21:26:48  fox
 * Final version of the museum.
 *
 * Revision 1.4  1998/09/05 00:25:29  fox
 * Works fine to detect people in the museum.
 *
 * Revision 1.3  1998/08/29 21:44:44  fox
 * Changed the detection report and improved stuck detection.
 *
 * Revision 1.2  1998/08/23 22:57:42  fox
 * First version of building maps of humans.
 *
 * Revision 1.1  1997/10/08 14:15:16  fox
 * Nothing special.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include "general.h"
#include "detection.h"
#include "allocate.h"
#include "function.h"
#include "laser.h"
#include "localTcx.h"
#include "graphics.h"

#include "DETECTION-messages.h"

/* Variables to set different moods. */

int numberOfSounds[] = { 1, 3, 4, 4};

char* happy[] = {"doobie-doobie-doo"};
char* first[] = {"pretty_crouded", "behind_1", "behind_2"};
char* second[] = {"some_space", "space", "horn2", "horn3"};
char* third[] = {"get_through", "in_my_way", "get_through1", "hey_blocking"};
char*** sounds = NULL; 

int currentMood = 0;
point positionAtLastSound = {0.0,0.0};

#define MIN_TIME_BETWEEN_SOUNDS 5.0
#define MIN_DIST_FOR_HAPPY 100.0

/* Variables to determine a stuck situation. */
#define STUCK_THRESHOLD 70.0

#define REPETITION_NUMBER 20
#define MAX_NUMBER_OF_IN_FRONT 10

int inFront[REPETITION_NUMBER];

void
stuckDetection( detectionStruct* obstacles)
{
  int i;
  int foundSomething = FALSE;
  int cnt, stuck;
  static int previousStuck = FALSE;
  static int inFrontCnt = 0;
  static int firstTime = TRUE;

  if ( 1 || targetPointGiven) {
    
    float minAngle = Deg2Rad( 70.0);
    float maxAngle = Deg2Rad( 290.0);

    if ( firstTime) {
      firstTime = FALSE;
      for ( i = 0; i < REPETITION_NUMBER; i++)
	inFront[i] = 0;
      setMood( 0);
    }
  
    for (i = 0; i < obstacles->numberOfAngles; i++) {
      if ( obstacles->angles[i] < minAngle || obstacles->angles[i] > maxAngle) {
	if ( obstacles->distances[i] > 0.0
	     &&
	     obstacles->distances[i] < STUCK_THRESHOLD) {
	  if (0) fprintf(stderr, "%f %f\n", obstacles->distances[i], Rad2Deg( obstacles->angles[i]));
	  foundSomething = TRUE;
	}
      }
    }
    
    inFront[inFrontCnt] = foundSomething;
  
    cnt = 0;
    for ( i = 0; i < REPETITION_NUMBER; i++) 
      if (inFront[i])
	cnt++;
  
    stuck = ( cnt > MAX_NUMBER_OF_IN_FRONT);
    
    if ( cnt > 0)
      fprintf(stderr, "inFrontCnt %d\n", cnt);
    
    if ( stuck) {
      fprintf(stderr, "S-T-U-C-K-!!!\n");
      for ( i = 0; i < REPETITION_NUMBER; i++)
	inFront[i] = 0;
      obstacles->foundSomething = FOUND_STUCK;
      sayStuck();
    }
    
    previousStuck = stuck;
    inFrontCnt = (inFrontCnt+1) % REPETITION_NUMBER;
  }

  /* Has the robot recovered from being disturbed? */
  if ( currentMood != 0 &&
       positionDist( positionAtLastSound.x, positionAtLastSound.y, 
		     robotPosition.x, robotPosition.y) > MIN_DIST_FOR_HAPPY) {
    currentMood = 0;
    setMood( currentMood);
    if ( sounds != NULL) {
      char* text = sounds[currentMood][randMax(numberOfSounds[currentMood] - 1)];
      triggerSound( text);
    }
  }
}




void
sayStuck()
{  
  char *text;
  
  static int firstTime = TRUE;

  float tExp = timeExpired( SOUND_TIMER);

  /* Initialize the sound structures. */
  if ( firstTime) {
    int i;

    resetTimer( SOUND_TIMER);
    
    sounds = (char***) malloc (NUMBER_OF_MOODS * sizeof(char**));

    sounds[0] = happy;
    sounds[1] = first;
    sounds[2] = second;
    sounds[3] = third;

    firstTime = FALSE;
  }

  if ( tExp > MIN_TIME_BETWEEN_SOUNDS) {
    
    if ( currentMood < NUMBER_OF_MOODS-1) {
      currentMood++;
      setMood( currentMood);
    }

    text = sounds[currentMood][randMax(numberOfSounds[currentMood] - 1)];
    triggerSound( text);

    resetTimer( SOUND_TIMER);
  }
  else 
    fprintf( stderr, "ignore: %f\n", tExp);

  positionAtLastSound.x = robotPosition.x;
  positionAtLastSound.y = robotPosition.y;
}


void
emergencyHandling( realPosition robPos, int emergency)
{
  static int emergencyOn = FALSE;
  static int firstTime = TRUE;
  static point positionAtEmergency;

  float timeSinceFirstEmergency;

#define MIN_EMERGENCY_TIME 8.0
#define MAX_EMERGENCY_TIME 30.0
#define EMERGENCY_THRESHOLD 70.0
#define MIN_MOTION_SINCE_EMERGENCY 40.0
  
  if ( firstTime) {
    firstTime = FALSE;
    resetTimer( EMERGENCY_TIMER);
  }

  timeSinceFirstEmergency = timeExpired( EMERGENCY_TIMER);

  /* Emergency handling. */
  if ( (! emergencyOn) && emergency) {
    
    printf( "\t\t\temergency!!!!\n");    
    
    if ( timeSinceFirstEmergency > MIN_EMERGENCY_TIME) {
      
      float closestKnownObstacle = 1000.0;
      int i, minAngle;
      
      for (i = 0; i < obstacles.numberOfAngles; i++) {
	if ( obstacles.measuredDistances[i] > 0.0) {
	  if ( obstacles.distances[i] == 0.0) {
	    if (0) printf( "%f %f\n", obstacles.distances[i], obstacles.measuredDistances[i]);
	    if ( obstacles.measuredDistances[i] < closestKnownObstacle) {
	      minAngle = i;
	      closestKnownObstacle = obstacles.measuredDistances[i];
	    }
	  }
	}
      }
      fprintf(stderr, "clos %f  %f\n", closestKnownObstacle, Rad2Deg(obstacles.angles[minAngle]));
      
      if ( closestKnownObstacle < EMERGENCY_THRESHOLD) {

	/* Ok. That's an emergency. */
	printf( "\t\t FOUND EMERGENCY!!\n");
	
	setBaseMode( BASE_ESCAPE_MODE);
	
	positionAtEmergency.x = robPos.x;
	positionAtEmergency.y = robPos.y;
	emergencyOn = TRUE;
	
      }
      else {
	printf( "all unknown %f.\n", closestKnownObstacle);
	
	/* We don't wait forever even if something unknown blocks the robot. */
	if ( timeSinceFirstEmergency > MAX_EMERGENCY_TIME) {

	  printf( "\t\t TIME PANIC!!\n");
	  
	  setBaseMode( BASE_ESCAPE_MODE);
	  
	  positionAtEmergency.x = robPos.x;
	  positionAtEmergency.y = robPos.y;
	  emergencyOn = TRUE;
	}
      }	  
    }
    else 
      printf(  "\t\t wait %f\n", timeSinceFirstEmergency);
  }
  else {

    resetTimer( EMERGENCY_TIMER);

    if ( emergencyOn) {
      
      /* No emergency. Wait until the robot has moved some meters. */
      if ( positionDist( robPos.x, robPos.y,
			 positionAtEmergency.x, positionAtEmergency.y) >
	   MIN_MOTION_SINCE_EMERGENCY) {
	
	/* Ok. We're done. */
	
	fprintf(stderr, "\t\t\t RESET TO NORMAL MODE!!\n");

	setBaseMode( BASE_DEFAULT_MODE);

	positionAtEmergency.x = robPos.x;
	positionAtEmergency.y = robPos.y;
	emergencyOn = FALSE;
	
	emergencyOn = FALSE;
      }    
      else
	fprintf( stderr, "wait for reset %f\n", 
		 positionDist( robPos.x, robPos.y,
			       positionAtEmergency.x, positionAtEmergency.y));
    }
  }
}

