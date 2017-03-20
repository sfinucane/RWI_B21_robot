
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliCleanUpTools.c,v $
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
 * $Log: colliCleanUpTools.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1996/09/24 07:13:16  rhino
 * Clean-up works fine now.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:04  rhino
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


#ifdef CLEANUP


#include "collisionIntern.h"
#include "colliCleanUp.h"


void
failSignal()
{
  putc(7, stderr);
  putc(7, stderr);
  putc(7, stderr);
}

void
stateTransitionSignal()
{
  putc(7, stderr);
  putc(7, stderr);
}

void
infoSignal()
{
  putc(7, stderr);
}

     
void
removeAllFarObjects()
{
   numberOfFarObjects = 0;
 }

void
removeAllFarTrashBins()
{
 numberOfFarTrashBins = 0;
}

void
removeFarObject()
{
  int i;
  
  numberOfFarObjects--;

  for ( i = 0; i <= numberOfFarObjects; i++)
    farObjects[i] = farObjects[i+1];

  actualFarObject.x = farObjects[i].x;
  actualFarObject.y = farObjects[i].y;
}

void
removeFarTrashBin()
{
  int i;
  
  numberOfFarTrashBins--;

  for ( i = 0; i <= numberOfFarTrashBins; i++)
    farTrashBins[i] = farTrashBins[i+1];

  actualFarTrashBin.x = farTrashBins[i].x;
  actualFarTrashBin.y = farTrashBins[i].y;
}


/***************************************************************************
 * Checks wether the clean up seqence accepts new objects from the vision.
 ***************************************************************************/
BOOLEAN
dontAcceptFarThings()
{
    return ( cleanUpState == GET_EXACT_OBJECT_POSITION ||
	     cleanUpState == GET_EXACT_TRASH_BIN_POSITION );
}

/***************************************************************************
 * Checks wether the clean up seqence accepts new objects from the vision.
 ***************************************************************************/
BOOLEAN
dontAcceptCloseThings() {
    return ( cleanUpState != GET_EXACT_OBJECT_POSITION &&
	     cleanUpState != GET_EXACT_TRASH_BIN_POSITION);
}


/***************************************************************************
 * Turns to the point with the given velocity.
 ***************************************************************************/
void
COLLI_turnToPoint( Point rPos,
		   float rRot,
		   Point object,
		   float rotateVelocity)
{
    float objectRot = compute_robot_angle_to_point( rPos, rRot, object);

    /* Set the velocities and rotate to the object. */
    BASE_TranslateVelocity (0.0);
    BASE_RotateVelocity ( rotateVelocity);
    BASE_TranslateCollisionVelocity (0.0);
    BASE_RotateCollisionVelocity ( rotateVelocity);
    BASE_Rotate(RAD_TO_DEG( objectRot));
    
    if (dumpInfo)
	fprintf( dumpFile, "Turn to the object by %f degrees.\n",
		 RAD_TO_DEG( objectRot));
    fprintf(stderr, "Turn to the object by %f degrees.\n",
	    RAD_TO_DEG( objectRot));
}


/***************************************************************************
 * Translates to the point with the given velocity until the robot is
 * <distance> centimeters away from the point.
 * If <checkForWayFree> is TRUE at each call it is checked wether
 * the direct way to the point is free. 
 ***************************************************************************/
int
COLLI_translateToPoint( Point rPos,
		       float rRot,
		       Point object,
		       float distance,
		       float translateVelocity,
		       float securityDist,
		       BOOLEAN checkForWayFree)
{
  static BOOLEAN alreadyInTranslation = FALSE;
  static BOOLEAN waitUntilWayFree = FALSE;
  BOOLEAN wayIsFree;
  BOOLEAN backUp;
  float translateDirection;
  int state;

 /* If the robot is too close it has to back up. This can be
   * derived from the sign of the objectdist. */
  float objectDist = compute_distance( rPos, object);
  objectDist -= ROB_RADIUS + distance;

  backUp = objectDist < 0.0;
  
  /* The angle to translate. */
  translateDirection = compute_angle_2p( rPos, object);
  
  if ( backUp)
    translateDirection += DEG_180;
  
  if ( checkForWayFree) {
    
    float targetDist;
    /* The distance to the next obstacle. */
    float collDist = computeCollisionDistance( rPos,
					      translateDirection,
					      translateVelocity,
					      0.0, /* No rotation */
					      securityDist,
					      0.0, /* No additional security dist */
					      0.0,  /* ARM PRAKTIKUM ???? */
					      &targetDist);
    wayIsFree = (collDist > fabs( objectDist));
  }
     
    
  else
    wayIsFree = TRUE;
  
  if (wayIsFree) {
    
    if ( alreadyInTranslation) {
      /* The command to translate has been sent to the robot. */
      if ( ! stillInTranslation()) {
	/* The point is reached. */
	if (dumpInfo)
	  fprintf(dumpFile, "Finished. Should stand %f cm in front of object.\n",
		  distance);
	fprintf(stderr, "Finished. Should stand %f cm in front of object.\n",
		distance);

	state = POINT_IS_REACHED;
      }
      else
	/* Not yet. */
	state = POINT_IS_NOT_YET_REACHED;
    }
    else {
      
      /* We have to start the translation. */
      
      /* Go backwards if the point is behind the robot. */
      if ( behindPoint( rPos, rRot, object)) 
	objectDist = -objectDist; 

      if ( colli_go_backward)
	objectDist = -objectDist;
	
      /* Set the velocities and translate to the object. */
      BASE_TranslateVelocity ( translateVelocity);
      BASE_TranslateCollisionVelocity ( translateVelocity);
      BASE_RotateVelocity ( 0.0);
      BASE_RotateCollisionVelocity ( 0.0);
      BASE_Translate( objectDist);
      
      alreadyInTranslation = TRUE;
      waitUntilWayFree = FALSE;
      
      if (dumpInfo)
	fprintf( dumpFile, "Translate to the object %f cm.\n", objectDist);
      fprintf(stderr, "Translate to the object %f cm.\n", objectDist);
      
      state = POINT_IS_NOT_YET_REACHED;
    }
  }
  
  /* The direct way is blocked. */
  else {
    
    BASE_TranslateHalt();
    
    /* It was blocked before. Just check how long we already wait. */
    if ( waitUntilWayFree) {
      if ( stillInTimeInterval())
	state = POINT_IS_NOT_YET_REACHED;
      else {
	/* We don't want to wait forever. */
	if (dumpInfo)
	  fprintf( dumpFile, "Way is blocked. Give up.\n");
	fprintf( stderr, "Way is blocked. Give up.\n");

	state = GAVE_UP;
      }
    }
    /* First time blocked. Set the timer. */
    else {
      if (dumpInfo)
	fprintf( dumpFile, "Way is blocked. dist: %f  objectdist: %f\n",
		distance, objectDist);
      fprintf( stderr, "Way is blocked. dist: %f  objectdist: %f\n",
	      distance, objectDist);
      setTimer( MAX_WAITING_TIME_FOR_WAY_FREE);
      state = POINT_IS_NOT_YET_REACHED;
    }

    alreadyInTranslation = FALSE;
    waitUntilWayFree = TRUE;
  }

  if (state == POINT_IS_REACHED || state == GAVE_UP)
	alreadyInTranslation = waitUntilWayFree = FALSE;

  return state;
}









#endif
