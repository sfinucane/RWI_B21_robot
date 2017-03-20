
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliArmExceptions.c,v $
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
 * $Log: colliArmExceptions.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.5  1998/05/13 07:19:39  fox
 * Minor changes.
 *
 * Revision 1.4  1997/11/12 17:07:28  fox
 * Removed some old arm stuff.
 *
 * Revision 1.3  1997/04/09 12:57:48  fox
 * Minor changes.
 *
 * Revision 1.2  1996/12/23 12:48:53  fox
 * First attempt to incorporate the tactiles.
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



#include "collisionIntern.h"

/* Exception-Handling Flags */
BOOLEAN emergencyStop = FALSE;
BOOLEAN rotatingAwayFlag = FALSE;
BOOLEAN haltingFlag = FALSE;
BOOLEAN rotPossibleFlag = FALSE;

/* Default mode is no. 2: the next free trajectory */
/* in target direction is taken. */
int rotateAwayNumber = 2;

/* how often the robot tried to rotate away */
int rotateAwayCounter = 0;        

/* this variables are needed to continue rotating-away in the desired */
/* direction after an unexpected event has occured */ 
float absoluteAngle = 0.0;
BOOLEAN rightRot = TRUE;


/**********************************************************************
 **********************************************************************
 *                     Collision functions                            *
 **********************************************************************
 **********************************************************************/

void keepOnHalting(Point rpos, float rrot);

static int
freeAgain(Point rpos, float rrot);

static void 
driveOn();


static int
rotateAwayModeIsPossible(Point rpos,
			 float rrot,
			 float* angle);
static int
rotateAwayModeIsPossible1(Point rpos,
			 float rrot,
			 float* angle);

/* This routine is callled when the robot is too close to an obstacle.
 * In this case the robot just turns in the direction that is closer 
 * to free space than the other. */
static int
rotateAwayModeIsPossible2(Point rpos,
			 float rrot,
			 float* angle);


static void
startRotatingAway(Point rpos, float rrot, float angle);

static void
keepOnRotatingAway(Point rpos, float rrot);

BOOLEAN isAngleReachable(Point rpos, float rrot,float *leftAngle,float *rightAngle );


/*  This procedure is called when an exception-handling is necessary.      *
 *  It sends a stop-command to the base and the haltingFlag is set to TRUE */
void
startHalting( Point rpos,
	     float rrot)		      
{
   BASE_TranslateCollisionAcceleration(ACTUAL_MODE->exception_trans_acceleration);
   BASE_RotateCollisionAcceleration( RAD_TO_DEG( ACTUAL_MODE->exception_rot_acceleration));
   BASE_TranslateHalt();
   BASE_RotateHalt();
   
   target_flag = TRUE;
   haltingFlag = TRUE;

   return;
}


/*  This function is called by collision.c when the haltingFlag is set TRUE. *
 *  It checks wether the way is free again. In this case, the robot can drive*
 *  on. Otherwise the routine waits until the robot has stopped completely,  *
 *  and the decides which exception-handling-method should be executed */    
void keepOnHalting(Point rpos,
		   float rrot)
{
   float angle;

   if (stillInTranslation() || stillInRotation())
   { 
      if(freeAgain(rpos, rrot))
      { 
	 driveOn();
	 haltingFlag = FALSE;
	 return;
      }
      else
	 return;
   }

   /* Now the robot is completely stopped */
   printf("KOH : The robot is now stopped completely .\n");
   
   haltingFlag = FALSE;

   if(rotateAwayModeIsPossible(rpos,rrot,&angle)  &&
      rotateAwayCounter <= ACTUAL_MODE->rotate_away_persistence)
   {

      SOUND_talk_text("rotate away");

      if (dumpInfo)
	 fprintf( dumpFile, "Rotate away");
      fprintf(stderr, "EH : Rotate away\n");

      rotateAwayCounter = rotateAwayCounter + 1;
      startRotatingAway(rpos,
			rrot,angle);
   }
   
   else  /* Rotation isn't possible */
   {
     putc(7,stderr);
     fprintf(stderr, "HELP!!!! Nothing possible!!!\n");
     putc(7,stderr);
   }
   return;
}
 
   


/* This function is called by keepOnHalting. It checks if the way is free *
 * again and returns the result */

int freeAgain(Point rpos, float rrot)
{
  float moveOnDist,targetDist; 
  moveOnDist = computeCollisionDistance(rpos, rrot, 
					ACTUAL_MODE->target_max_trans_speed, 0.0,  
					ROB_RADIUS+ACTUAL_MODE->security_dist, 
					0.0,0.0, 
					&targetDist); 
  
  return( moveOnDist > 2.0 * ACTUAL_MODE->min_dist); 
}



/* This function sends drive-on commands to the base after keepOnHalting has *
 * received the message that the way is free again */   
void driveOn()
{

  SOUND_talk_text("free again");
  SOUND_play_message(1);

  if (dumpInfo)
    fprintf( dumpFile, "free again.\n");
  fprintf(stderr, "DO : free again.\n");
  BASE_TranslateCollisionAcceleration(ACTUAL_MODE->target_trans_acceleration);
  BASE_RotateCollisionAcceleration(RAD_TO_DEG(ACTUAL_MODE->target_rot_acceleration));
  return;
}


/* This function is called by collision.c . It checks which exception-handling
   was started and calls the right function */
void
exceptionHandling( Point rpos,
		   float rrot)
{
  if ( haltingFlag )
    {
      keepOnHalting(rpos,rrot);
      COLLI_update_tcx_trajectory(rpos, rrot, 0.0 , 0.0, 0.0);
      return;
    }   
    
  if (rotatingAwayFlag)
    { 
      keepOnRotatingAway(rpos, rrot);
      COLLI_update_tcx_trajectory(rpos, rrot, 0.0, 1.0, 0.0);
      return;
    }
}



/* This procedure checks wether rotation with arm outside is possible  *
 * and reasonable. If this is the case the routine returns TRUE and    *
 * the angle the robot should rotate */
   
int rotateAwayModeIsPossible(Point rpos,float rrot,float* rotAngle)
{
   if (rotateAwayNumber==1)
     return rotateAwayModeIsPossible1(rpos,rrot,rotAngle);
   else
     return rotateAwayModeIsPossible2(rpos,rrot,rotAngle);
}



/* This routine calculates the angle the robot has to rotate in order to    *
 * get a maximum free way. If there is more than one maximum free way, this *
 * function chooses the direction nearest to the target. */

int rotateAwayModeIsPossible1(Point rpos,
			     float rrot,
			     float* rotAngle)
{ 
#define MIN_ROT 5.0
  int i; 
  float targetAngle, targetDist;
  float leftDist, rightDist, securityDist;
  float leftAngle,rightAngle,alpha1,beta1,leftDistAngle,rightDistAngle;
  float angle;
  float maxDistance;
  float delta1,delta2;
  float resultAngle;  
  
  maxDistance = 0.0;
  
  leftAngle = (collisionDistance(rpos,
				 rrot,
				 0.0,
				 -20.0,
				 &targetDist) * DEG_360) / MAX_RANGE;
  
  rightAngle = (collisionDistance(rpos,
				  rrot,
				  0.0,
				  20.0,
				  &targetDist) * DEG_360) / MAX_RANGE;

  /* Rotation is not possible */
  if((leftAngle <= DEG_TO_RAD(MIN_ROT)) && (rightAngle <= DEG_TO_RAD(MIN_ROT)))
  {
     printf("RP: Really, no rotation possible !\n");
     return 0;
  }

  securityDist = 2.0 * ACTUAL_MODE->min_dist;
  
  angle = targetAngle = compute_angle_2p(rpos, target);

    
  if ((armState == INSIDE) || ((leftAngle+rightAngle) >= DEG_360))
  {
     for (i=0; i<=180; i+=10) {	
	leftDist = computeCollisionDistance(
					    rpos, 
					    targetAngle + DEG_TO_RAD((float)i),
					    ACTUAL_MODE->target_max_trans_speed,
					    0.0, 
					    ROB_RADIUS+ACTUAL_MODE->security_dist,
					    0.0,0.0,
					    &targetDist);
	
	rightDist = computeCollisionDistance(
					     rpos,
					     targetAngle - DEG_TO_RAD((float)i),
					     ACTUAL_MODE->target_max_trans_speed,
					     0.0, 
					     ROB_RADIUS+ACTUAL_MODE->security_dist,
					     0.0,0.0,
					     &targetDist);
	    
	if (leftDist > maxDistance)
	{
	  maxDistance = leftDist;
	  angle = normed_angle(targetAngle + DEG_TO_RAD((float) i));
	}
	if (rightDist > maxDistance)
	{
	  maxDistance = rightDist;
	  angle = normed_angle(targetAngle - DEG_TO_RAD((float) i));
	}
     }
     if (maxDistance < securityDist)
     {
        printf("RP : Can not rotate away because maxDistance < securityDist \n");
	return 0;
     }
  }
     
  else     /* The arm is outside */
  {
     angle = rrot;    
     
     alpha1 = rrot + leftAngle; 
     norm_angle(&alpha1);
     beta1 = rrot -  rightAngle; 
     norm_angle(&beta1);
      
     for (i=0; i<=180; i+=10) {
	leftDistAngle = normed_angle(targetAngle - DEG_TO_RAD((float)i));
	rightDistAngle = normed_angle(targetAngle + DEG_TO_RAD((float)i));
	
	leftDist = computeCollisionDistance(
					    rpos, 
					    targetAngle - DEG_TO_RAD((float)i),
					    ACTUAL_MODE->target_max_trans_speed,
					    0.0, 
					    ROB_RADIUS+ACTUAL_MODE->security_dist,
					    0.0,0.0,
					    &targetDist);
	
	rightDist = computeCollisionDistance(
					     rpos,
					     targetAngle + DEG_TO_RAD((float)i),
					     ACTUAL_MODE->target_max_trans_speed,
					     0.0, 
					     ROB_RADIUS+ACTUAL_MODE->security_dist,
					     0.0,0.0,
					     &targetDist);
	if (alpha1>=beta1)
	{
	   if ((leftDist> maxDistance) &&
	       ((alpha1>=leftDistAngle) && (leftDistAngle>= beta1)))
	   {
	      maxDistance = leftDist;
	      angle = leftDistAngle;	      
	   }
	   if ((rightDist > maxDistance) &&
	       ((alpha1>=rightDistAngle) && (rightDistAngle>=beta1)))
	   {
	      maxDistance = rightDist;
	      angle = rightDistAngle;	      
	   }
	}
	else
	{
	   if ((leftDist> maxDistance) &&
	       ((alpha1>=leftDistAngle) || (leftDistAngle>= beta1)))
	   {
	      maxDistance = leftDist;
	      angle = leftDistAngle;	      
	   }
	   if ((rightDist > maxDistance) &&
	       ((alpha1>=rightDistAngle) || (rightDistAngle>=beta1)))
	   {
	      maxDistance = rightDist;
	      angle = rightDistAngle;	      
	   }	   
	}
     }
     if( maxDistance < securityDist)
     {
	printf("RP : Can not rotate away because maxDistance<securityDist \n");
	return 0;
     }
  }

  /* here we know that we are able to turn to the chosen angle "angle" */
  
  delta1 = rrot - angle;

  if (delta1<= 0.0)
     delta2 = delta1 + DEG_360;
  else
     delta2 = delta1 - DEG_360;
  
  if (fabs(delta1)<= fabs(delta2))
  {  /* We prefer delta1 as rotation angle because it is the smaller one */
     if (delta1 >= 0.0)
     {  /* We prefer turning right */
	if (delta1 <= rightAngle )
	{  /* It is possible to turn right */
	   resultAngle = delta1;
	}
	else
	{  /* It is not possible to turn right, therefore we turn left */
	   resultAngle = delta2;
	}
     }
     else
     {  /* We prefer turning left */
	if (fabs(delta1) <= leftAngle)
	{  /* It is possible to turn left */
	   resultAngle = delta1;
	}
	else
	{  /* It is not possible to turn left, therefore we turn right */
	   resultAngle = delta2;
	}
     }
  }
  else
  {  /* We prefer delta2 as rotation angle because it is the smaller one */
     if (delta2 >= 0.0)
     {  /* We prefer turning right */
	if (delta2 <= rightAngle )
	{  /* It is possible to turn right */
	   resultAngle = delta2;
	}
	else
	{  /* It is not possible to turn right, therefore we turn left */
	   resultAngle = delta1;
	}
     }
     else
     {  /* We prefer turning left */
	if (fabs(delta2) <= leftAngle)
	{  /* It is possible to turn left */
	   resultAngle = delta2;
	}
	else
	{  /* It is not possible to turn left, therefore we turn right */
	   resultAngle = delta1;
	}
     }
  }
  (*rotAngle) = resultAngle;   
  printf("Suggested rotation = %f \n",RAD_TO_DEG(resultAngle));
  return 1;
}



/* This function is called when rotate-away is chosen. It sends the commands
   that are nessessary to turn the robot by the angle calculated by rotateAway
   ModeIsPossible */
void
startRotatingAway(Point rpos, float rrot,float angle)
{
  rotatingAwayFlag = TRUE;
  absoluteAngle = normed_angle(rrot-angle); 
  if (angle >= 0.0)
     rightRot = TRUE;
  else
     rightRot = FALSE;
  BASE_TranslateCollisionAcceleration(ACTUAL_MODE->exception_trans_acceleration);
  BASE_RotateCollisionAcceleration( RAD_TO_DEG( ACTUAL_MODE->exception_rot_acceleration));
  target_flag = TRUE;
  BASE_TranslateVelocity (0.0);
  BASE_RotateVelocity ( RAD_TO_DEG( ACTUAL_MODE->exception_rot_velocity));
  BASE_TranslateCollisionVelocity (0.0);
  BASE_RotateCollisionVelocity ( RAD_TO_DEG( ACTUAL_MODE->exception_rot_velocity));
  
  BASE_Rotate((double)RAD_TO_DEG(angle));
  if (dumpInfo)
     fprintf( dumpFile, " (%f) ", RAD_TO_DEG(angle));
}




/* This routine calculates the angle the robot has to rotate so that *
 * he can drive the next free way in target direction. */
 
int rotateAwayModeIsPossible2(Point rpos,
			     float rrot,
			     float* rotAngle)
{ 
#define MIN_ROT 5.0
  int i; 
  float targetAngle, targetDist;
  float leftDist, rightDist, securityDist;
  float leftAngle,rightAngle,alpha1,beta1,leftDistAngle,rightDistAngle;
  float angle;
  float delta1,delta2;
  float resultAngle;  
  BOOLEAN foundAdmissibleAngle = FALSE;
 
  
  leftAngle = (collisionDistance(rpos,
				 rrot,
				 0.0,
				 -20.0,
				 &targetDist) * DEG_360) / MAX_RANGE;
  
  rightAngle = (collisionDistance(rpos,
				  rrot,
				  0.0,
				  20.0,
				  &targetDist) * DEG_360) / MAX_RANGE;

  /* Rotation is not possible */
  if((leftAngle <= DEG_TO_RAD(MIN_ROT)) && (rightAngle <= DEG_TO_RAD(MIN_ROT)))
  {
     printf("RP2: Really, no rotation possible !\n");
     return 0;
  }

  securityDist = 2.0 * ACTUAL_MODE->min_dist;
  
  angle = targetAngle = compute_angle_2p(rpos, target);

    
  if ((armState == INSIDE) || ((leftAngle+rightAngle) >= DEG_360))
  {
     for (i=0; i<=180; i+=10) {	
	leftDist = computeCollisionDistance(
					    rpos, 
					    targetAngle + DEG_TO_RAD((float)i),
					    ACTUAL_MODE->target_max_trans_speed,
					    0.0, 
					    ROB_RADIUS+ACTUAL_MODE->security_dist,
					    0.0,0.0,
					    &targetDist);
	
	rightDist = computeCollisionDistance(
					     rpos,
					     targetAngle - DEG_TO_RAD((float)i),
					     ACTUAL_MODE->target_max_trans_speed,
					     0.0, 
					     ROB_RADIUS+ACTUAL_MODE->security_dist,
					     0.0,0.0,
					     &targetDist);
	    
        if (leftDist > rightDist)
	{
	   if (leftDist > securityDist)
	   {
	      foundAdmissibleAngle = TRUE;
	      angle = normed_angle(targetAngle + DEG_TO_RAD((float) i));
	   }
        }
	else
	{
	   if (rightDist > securityDist)
	   {
	      foundAdmissibleAngle = TRUE;
	      angle = normed_angle(targetAngle - DEG_TO_RAD((float) i));
	   }
	}
	if (foundAdmissibleAngle)
	   break;
     }

     if (!foundAdmissibleAngle)
     {
        printf("RP2 : No angle is admissible !\n");
	return 0;
     }
  }
     
  else     /* The arm is outside */
  {
     
     alpha1 = rrot + leftAngle; 
     norm_angle(&alpha1);
     beta1 = rrot -  rightAngle; 
     norm_angle(&beta1);
      
     for (i=0; i<=180; i+=10) {
	leftDistAngle = normed_angle(targetAngle - DEG_TO_RAD((float)i));
	rightDistAngle = normed_angle(targetAngle + DEG_TO_RAD((float)i));
	
	leftDist = computeCollisionDistance(
					    rpos, 
					    targetAngle - DEG_TO_RAD((float)i),
					    ACTUAL_MODE->target_max_trans_speed,
					    0.0, 
					    ROB_RADIUS+ACTUAL_MODE->security_dist,
					    0.0,0.0,
					    &targetDist);
	
	rightDist = computeCollisionDistance(
					     rpos,
					     targetAngle + DEG_TO_RAD((float)i),
					     ACTUAL_MODE->target_max_trans_speed,
					     0.0, 
					     ROB_RADIUS+ACTUAL_MODE->security_dist,
					     0.0,0.0,
					     &targetDist);
	if (alpha1>=beta1)
	{
	   if (leftDist >= rightDist)
	   {  /* We prefer leftDistAngle */
	      if ((leftDist> securityDist) &&
		  ((alpha1>=leftDistAngle) && (leftDistAngle>= beta1)))
	      {
		 foundAdmissibleAngle = TRUE;
		 angle = leftDistAngle;	      
	      }
	      else
	      {
		 if ((rightDist > securityDist) &&
		     ((alpha1>=rightDistAngle) && (rightDistAngle>=beta1)))
		 {
		    foundAdmissibleAngle = TRUE;
		    angle = rightDistAngle;	      
		 }
	      }
	   }
	   else
	   {  /* We prefer rightDistAngle */
	      if ((rightDist > securityDist) &&
		  ((alpha1>=rightDistAngle) && (rightDistAngle>= beta1)))
	      {
		 foundAdmissibleAngle = TRUE;
		 angle = rightDistAngle;	      
	      }
	      else
	      {
		 if ((leftDist > securityDist) &&
		     ((alpha1>=leftDistAngle) && (leftDistAngle>=beta1)))
		 {
		    foundAdmissibleAngle = TRUE;
		    angle = leftDistAngle;	      
		 }
	      }
	   }	      
	}
	else /* alpha1 < beta1 */
	{
	   if (leftDist >= rightDist)
	   {  /* We prefer leftDistAngle */
	      if ((leftDist> securityDist) &&
		  ((alpha1>=leftDistAngle) || (leftDistAngle>= beta1)))
	      {
		 foundAdmissibleAngle = TRUE;
		 angle = leftDistAngle;	      
	      }
	      else
	      {
		 if ((rightDist > securityDist) &&
		     ((alpha1>=rightDistAngle) || (rightDistAngle>=beta1)))
		 {
		    foundAdmissibleAngle = TRUE;
		    angle = rightDistAngle;	      
		 }
	      }
	   }
	   else
	   {  /* We prefer rightDistAngle */
	      if ((rightDist > securityDist) &&
		  ((alpha1>=rightDistAngle) || (rightDistAngle>= beta1)))
	      {
		 foundAdmissibleAngle = TRUE;
		 angle = rightDistAngle;	      
	      }
	      else
	      {
		 if ((leftDist > securityDist) &&
		     ((alpha1>=leftDistAngle) || (leftDistAngle>=beta1)))
		 {
		    foundAdmissibleAngle = TRUE;
		    angle = leftDistAngle;	      
		 }
	      }
	   }
	}
        if (foundAdmissibleAngle)
	   break;
     }
     if(! foundAdmissibleAngle )
     {
	printf("RP : Can not rotate away because maxDistance<securityDist \n");
	return 0;
     }
  }

  /* here we know that we are able to turn to the chosen angle "angle" */
  
  delta1 = rrot - angle;

  if (delta1<= 0.0)
     delta2 = delta1 + DEG_360;
  else
     delta2 = delta1 - DEG_360;
  
  if (fabs(delta1)<= fabs(delta2))
  {  /* We prefer delta1 as rotation angle because it is the smaller one */
     if (delta1 >= 0.0)
     {  /* We prefer turning right */
	if (delta1 <= rightAngle )
	{  /* It is possible to turn right */
	   resultAngle = delta1;
	}
	else
	{  /* It is not possible to turn right, therefore we turn left */
	   resultAngle = delta2;
	}
     }
     else
     {  /* We prefer turning left */
	if (fabs(delta1) <= leftAngle)
	{  /* It is possible to turn left */
	   resultAngle = delta1;
	}
	else
	{  /* It is not possible to turn left, therefore we turn right */
	   resultAngle = delta2;
	}
     }
  }
  else
  {  /* We prefer delta2 as rotation angle because it is the smaller one */
     if (delta2 >= 0.0)
     {  /* We prefer turning right */
	if (delta2 <= rightAngle )
	{  /* It is possible to turn right */
	   resultAngle = delta2;
	}
	else
	{  /* It is not possible to turn right, therefore we turn left */
	   resultAngle = delta1;
	}
     }
     else
     {  /* We prefer turning left */
	if (fabs(delta2) <= leftAngle)
	{  /* It is possible to turn left */
	   resultAngle = delta2;
	}
	else
	{  /* It is not possible to turn left, therefore we turn right */
	   resultAngle = delta1;
	}
     }
  }
  (*rotAngle) = resultAngle;   
  printf("Suggested rotation = %f \n",RAD_TO_DEG(resultAngle));
  return 1;
}


/* While the robot is rotating-away this function is called to check if *
 * the chosen angle is still reachable */

BOOLEAN
isAngleReachable(Point rpos, float rrot,float *leftAngle,float *rightAngle)
{
   float targetDist,alpha1,beta1;
   
   (*leftAngle) = (collisionDistance(rpos,
				  rrot,
				  0.0,
				  -20.0,
				  &targetDist) * DEG_360) / MAX_RANGE;
   
   (*rightAngle) = (collisionDistance(rpos,
				   rrot,
				   0.0,
				   20.0,
				   &targetDist) * DEG_360) / MAX_RANGE;
       
   alpha1 = rrot + (*leftAngle); 
   norm_angle(&alpha1);
   beta1 = rrot -  (*rightAngle); 
   norm_angle(&beta1);
      
   if (rightRot)
   {
      if (rrot>=beta1)
      {
	 if ((beta1<=absoluteAngle) && (absoluteAngle<=rrot))
	    return TRUE;
	 else
	    return FALSE;
      }
      else
      {
	 if ((rrot>=absoluteAngle) || (absoluteAngle>=beta1))
	    return TRUE;
	 else
	    return FALSE;
      }
   }
   else
   {
      if (alpha1>=rrot)
      {
	 if ((alpha1>=absoluteAngle) && (absoluteAngle>=rrot))
	    return TRUE;
	 else
	    return FALSE;
      }
      else
      {
	 if ((rrot<=absoluteAngle) || (absoluteAngle<=alpha1))
	    return TRUE;
	 else
	    return FALSE;
      }
   }
}

      
/* This function controls the rotate-away-operation. It stops the robot
 * when the angle is not reachable. In the case that the angle is not
 * reachable because of a misreading  it restarts the rotation to the
 * old angle from the new position. Otherwise it decides wether a new
 * angle should be calculated or wether rollback should be executed
 * (compare rotateAwayNumber). */
void
keepOnRotatingAway(Point rpos, float rrot)
{
static BOOLEAN stoppedRotatingAway = FALSE;
float leftAngle,rightAngle,delta1,delta2,resultAngle;

   if (stoppedRotatingAway)
   {
      if (stillInRotation())
      {
	 if (isAngleReachable(rpos,rrot,&leftAngle,&rightAngle))
	 {
	    /* here we know that we are able to turn to the chosen angle "angle" */    
	    delta1 = rrot - absoluteAngle;
	    
	    if (delta1<= 0.0)
	       delta2 = delta1 + DEG_360;
	    else
	       delta2 = delta1 - DEG_360;
	    
	    if (fabs(delta1)<= fabs(delta2))
	    {  /* We prefer delta1 as rotation angle because it is the smaller one */
	       if (delta1 >= 0.0)
	       {  /* We prefer turning right */
		  if (delta1 <= rightAngle )
		  {  /* It is possible to turn right */
		     resultAngle = delta1;
		  }
		  else
		  {  /* It is not possible to turn right, therefore we turn left */
		     resultAngle = delta2;
		  }
	       }
	       else
	       {  /* We prefer turning left */
		  if (fabs(delta1) <= leftAngle)
		  {  /* It is possible to turn left */
		     resultAngle = delta1;
		  }
		  else
		  {  /* It is not possible to turn left, therefore we turn right */
		     resultAngle = delta2;
		  }
	       }
	    }
	    else
	    {  /* We prefer delta2 as rotation angle because it is the smaller one */
	       if (delta2 >= 0.0)
	       {  /* We prefer turning right */
		  if (delta2 <= rightAngle )
		  {  /* It is possible to turn right */
		     resultAngle = delta2;
	}
		  else
		  {  /* It is not possible to turn right, therefore we turn left */
		     resultAngle = delta1;
		  }
	       }
	       else
	       {  /* We prefer turning left */
		  if (fabs(delta2) <= leftAngle)
		  {  /* It is possible to turn left */
		     resultAngle = delta2;
		  }
		  else
		  {  /* It is not possible to turn left, therefore we turn right */
		     resultAngle = delta1;
		  }
	       }
	    }
	    startRotatingAway(rpos,rrot,resultAngle);   
	    stoppedRotatingAway = FALSE;
	    return;
	 }
	 else
	 {
	    return;
	 }
      }
      else
      {
	 stoppedRotatingAway = FALSE;
	 haltingFlag = TRUE;
	 rotatingAwayFlag = FALSE;
	 return;
      }
   }
   else
   {   
      if (!isAngleReachable(rpos,rrot,&leftAngle,&rightAngle))
      {
	 BASE_TranslateCollisionAcceleration(ACTUAL_MODE->exception_trans_acceleration);
	 BASE_RotateCollisionAcceleration( RAD_TO_DEG( ACTUAL_MODE->exception_rot_acceleration));
	 BASE_TranslateHalt();
	 BASE_RotateHalt();
	 
	 /* In the stop commands the target flag is set to FALSE. 
	  * We still want to be in the target mode.
	  */
	 target_flag = TRUE;
         stoppedRotatingAway = TRUE;
	 return;
      }
      else
      {
	 if (stillInRotation())
	    return;
	 else
	 {
	    rotatingAwayFlag = FALSE;
	    rotateAwayCounter = 0;
	    return;
	 }
      }
   }
}
      


 
/* Test-Functions to check the rotate-away functions */
void
printCurrentAngles(Point rpos, float rrot)
{ 
 
#define MIN_ROT 5.0
  int i; 
  float targetAngle, targetDist;
  float leftDist, rightDist, securityDist;
  float leftAngle,rightAngle,alpha1,beta1,leftDistAngle,rightDistAngle;
  float angle;
  float delta1,delta2;
  float resultAngle;  
  BOOLEAN foundAdmissibleAngle = FALSE;
 
  
  leftAngle = (collisionDistance(rpos,
				 rrot,
				 0.0,
				 -20.0,
				 &targetDist) * DEG_360) / MAX_RANGE;
  
  rightAngle = (collisionDistance(rpos,
				  rrot,
				  0.0,
				  20.0,
				  &targetDist) * DEG_360) / MAX_RANGE;

  /* Rotation is not possible */
  if((leftAngle <= DEG_TO_RAD(MIN_ROT)) && (rightAngle <= DEG_TO_RAD(MIN_ROT)))
  {
     printf("RP2: Really, no rotation possible !\n");
     rotPossibleFlag = FALSE;
     return;
  }

  printf("leftAngle = %f, rightAngle = %f, rrot = %f \n",RAD_TO_DEG(leftAngle)
	 ,RAD_TO_DEG(rightAngle),RAD_TO_DEG(rrot));
  
  securityDist = 2.0 * ACTUAL_MODE->min_dist;
  
  angle = targetAngle = 0.0;

    
  if ((armState == INSIDE) || ((leftAngle+rightAngle) >= DEG_360))
  {
     for (i=0; i<=180; i+=10) {	
	leftDist = computeCollisionDistance(
					    rpos, 
					    targetAngle + DEG_TO_RAD((float)i),
					    ACTUAL_MODE->target_max_trans_speed,
					    0.0, 
					    ROB_RADIUS+ACTUAL_MODE->security_dist,
					    0.0,0.0,
					    &targetDist);
	
	rightDist = computeCollisionDistance(
					     rpos,
					     targetAngle - DEG_TO_RAD((float)i),
					     ACTUAL_MODE->target_max_trans_speed,
					     0.0, 
					     ROB_RADIUS+ACTUAL_MODE->security_dist,
					     0.0,0.0,
					     &targetDist);
	    
        if (leftDist > rightDist)
	{
	   if (leftDist > securityDist)
	   {
	      foundAdmissibleAngle = TRUE;
	      angle = normed_angle(targetAngle + DEG_TO_RAD((float) i));
	   }
        }
	else
	{
	   if (rightDist > securityDist)
	   {
	      foundAdmissibleAngle = TRUE;
	      angle = normed_angle(targetAngle - DEG_TO_RAD((float) i));
	   }
	}
	if (foundAdmissibleAngle)
	   break;
     }

     if (!foundAdmissibleAngle)
     {
        printf("RP2 : No angle is admissible !\n");
	rotPossibleFlag = FALSE;
	return;
     }
  }
     
  else     /* The arm is outside */
  {
     
     alpha1 = rrot + leftAngle; 
     norm_angle(&alpha1);
     beta1 = rrot -  rightAngle; 
     norm_angle(&beta1);
      
     for (i=0; i<=180; i+=10) {
	leftDistAngle = normed_angle(targetAngle - DEG_TO_RAD((float)i));
	rightDistAngle = normed_angle(targetAngle + DEG_TO_RAD((float)i));
	
	leftDist = computeCollisionDistance(
					    rpos, 
					    targetAngle - DEG_TO_RAD((float)i),
					    ACTUAL_MODE->target_max_trans_speed,
					    0.0, 
					    ROB_RADIUS+ACTUAL_MODE->security_dist,
					    0.0,0.0,
					    &targetDist);
	
	rightDist = computeCollisionDistance(
					     rpos,
					     targetAngle + DEG_TO_RAD((float)i),
					     ACTUAL_MODE->target_max_trans_speed,
					     0.0, 
					     ROB_RADIUS+ACTUAL_MODE->security_dist,
					     0.0,0.0,
					     &targetDist);
	if (alpha1>=beta1)
	{
	   if (leftDist >= rightDist)
	   {  /* We prefer leftDistAngle */
	      if ((leftDist> securityDist) &&
		  ((alpha1>=leftDistAngle) && (leftDistAngle>= beta1)))
	      {
		 foundAdmissibleAngle = TRUE;
		 angle = leftDistAngle;	      
	      }
	      else
	      {
		 if ((rightDist > securityDist) &&
		     ((alpha1>=rightDistAngle) && (rightDistAngle>=beta1)))
		 {
		    foundAdmissibleAngle = TRUE;
		    angle = rightDistAngle;	      
		 }
	      }
	   }
	   else
	   {  /* We prefer rightDistAngle */
	      if ((rightDist > securityDist) &&
		  ((alpha1>=rightDistAngle) && (rightDistAngle>= beta1)))
	      {
		 foundAdmissibleAngle = TRUE;
		 angle = rightDistAngle;	      
	      }
	      else
	      {
		 if ((leftDist > securityDist) &&
		     ((alpha1>=leftDistAngle) && (leftDistAngle>=beta1)))
		 {
		    foundAdmissibleAngle = TRUE;
		    angle = leftDistAngle;	      
		 }
	      }
	   }	      
	}
	else /* alpha1 < beta1 */
	{
	   if (leftDist >= rightDist)
	   {  /* We prefer leftDistAngle */
	      if ((leftDist> securityDist) &&
		  ((alpha1>=leftDistAngle) || (leftDistAngle>= beta1)))
	      {
		 foundAdmissibleAngle = TRUE;
		 angle = leftDistAngle;	      
	      }
	      else
	      {
		 if ((rightDist > securityDist) &&
		     ((alpha1>=rightDistAngle) || (rightDistAngle>=beta1)))
		 {
		    foundAdmissibleAngle = TRUE;
		    angle = rightDistAngle;	      
		 }
	      }
	   }
	   else
	   {  /* We prefer rightDistAngle */
	      if ((rightDist > securityDist) &&
		  ((alpha1>=rightDistAngle) || (rightDistAngle>= beta1)))
	      {
		 foundAdmissibleAngle = TRUE;
		 angle = rightDistAngle;	      
	      }
	      else
	      {
		 if ((leftDist > securityDist) &&
		     ((alpha1>=leftDistAngle) || (leftDistAngle>=beta1)))
		 {
		    foundAdmissibleAngle = TRUE;
		    angle = leftDistAngle;	      
		 }
	      }
	   }
	}
        if (foundAdmissibleAngle)
	   break;
     }
     if(! foundAdmissibleAngle )
     {
	printf("RP : Can not rotate away because maxDistance<securityDist \n");
	rotPossibleFlag = FALSE;
	return;
     }
  }

  /* here we know that we are able to turn to the chosen angle "angle" */
  
  delta1 = rrot - angle;

  if (delta1<= 0.0)
     delta2 = delta1 + DEG_360;
  else
     delta2 = delta1 - DEG_360;
  
  if (fabs(delta1)<= fabs(delta2))
  {  /* We prefer delta1 as rotation angle because it is the smaller one */
     if (delta1 >= 0.0)
     {  /* We prefer turning right */
	if (delta1 <= rightAngle )
	{  /* It is possible to turn right */
	   resultAngle = delta1;
	}
	else
	{  /* It is not possible to turn right, therefore we turn left */
	   resultAngle = delta2;
	}
     }
     else
     {  /* We prefer turning left */
	if (fabs(delta1) <= leftAngle)
	{  /* It is possible to turn left */
	   resultAngle = delta1;
	}
	else
	{  /* It is not possible to turn left, therefore we turn right */
	   resultAngle = delta2;
	}
     }
  }
  else
  {  /* We prefer delta2 as rotation angle because it is the smaller one */
     if (delta2 >= 0.0)
     {  /* We prefer turning right */
	if (delta2 <= rightAngle )
	{  /* It is possible to turn right */
	   resultAngle = delta2;
	}
	else
	{  /* It is not possible to turn right, therefore we turn left */
	   resultAngle = delta1;
	}
     }
     else
     {  /* We prefer turning left */
	if (fabs(delta2) <= leftAngle)
	{  /* It is possible to turn left */
	   resultAngle = delta2;
	}
	else
	{  /* It is not possible to turn left, therefore we turn right */
	   resultAngle = delta1;
	}
     }
  }   
  printf("Suggested rotation = %f \n",RAD_TO_DEG(resultAngle));
  rotPossibleFlag = FALSE;
  return;
}

void
printCurrentAngles2(Point rpos,float rrot)
{    
#define MIN_ROT 5.0
  int i; 
  float targetAngle, targetDist;
  float leftDist, rightDist, securityDist;
  float leftAngle,rightAngle,alpha1,beta1,leftDistAngle,rightDistAngle;
  float angle;
  float maxDistance;
  float delta1,delta2;
  float resultAngle;  
  
  maxDistance = 0.0;
  
  leftAngle = (collisionDistance(rpos,
				 rrot,
				 0.0,
				 -20.0,
				 &targetDist) * DEG_360) / MAX_RANGE;
  
  rightAngle = (collisionDistance(rpos,
				  rrot,
				  0.0,
				  20.0,
				  &targetDist) * DEG_360) / MAX_RANGE;
  printf("RP: leftAngle = %f rightAngle = %f \n",RAD_TO_DEG(leftAngle),RAD_TO_DEG(rightAngle));
  printf("RP: rrot = %f \n",RAD_TO_DEG(rrot));  

  /* Rotation is not possible */
  if((leftAngle <= DEG_TO_RAD(MIN_ROT)) && (rightAngle <= DEG_TO_RAD(MIN_ROT)))
  {
     printf("RAP: Really No rotation possible !\n");
     rotPossibleFlag = FALSE;
     return;
  }

  securityDist = 2.0 * ACTUAL_MODE->min_dist;
  
  targetAngle = 0.0;
    
    
  if ((armState == INSIDE) || ((leftAngle+rightAngle) >= DEG_360))
  {
     angle = targetAngle;   
     for (i=0; i<=180; i+=10) {	
	leftDist = computeCollisionDistance(
					    rpos, 
					    targetAngle + DEG_TO_RAD((float)i),
					    ACTUAL_MODE->target_max_trans_speed,
					    0.0, 
					    ROB_RADIUS+ACTUAL_MODE->security_dist,
					    0.0,0.0,
					    &targetDist);
	
	rightDist = computeCollisionDistance(
					     rpos,
					     targetAngle - DEG_TO_RAD((float)i),
					     ACTUAL_MODE->target_max_trans_speed,
					     0.0, 
					     ROB_RADIUS+ACTUAL_MODE->security_dist,
					     0.0,0.0,
					     &targetDist);
	    
	if (leftDist > maxDistance) {
	   maxDistance = leftDist;
	   angle = normed_angle(targetAngle + DEG_TO_RAD((float) i));
	}
	if (rightDist > maxDistance) {
	   maxDistance = rightDist;
	   angle = normed_angle(targetAngle - DEG_TO_RAD((float) i));
	}
     
     }
     if (maxDistance < securityDist)
     {
        printf("RAP : Can not rotate away because (maxDistance= %f) and (securityDist = %f) \n",maxDistance,securityDist);
	rotPossibleFlag = FALSE;
	return;
     }
  }
     
  else     /* The arm is outside */
  {
     angle = rrot;    /* Falls keine Drehung moeglich bzw sinvoll ist, soll er sich nicht drehen. */
     
     alpha1 = rrot + leftAngle; 
     norm_angle(&alpha1);
     beta1 = rrot -  rightAngle; 
     norm_angle(&beta1);
      
     for (i=0; i<=180; i+=10) {
	leftDistAngle = normed_angle(targetAngle - DEG_TO_RAD((float)i));
	rightDistAngle = normed_angle(targetAngle + DEG_TO_RAD((float)i));
	
	leftDist = computeCollisionDistance(
					    rpos, 
					    targetAngle - DEG_TO_RAD((float)i),
					    ACTUAL_MODE->target_max_trans_speed,
					    0.0, 
					    ROB_RADIUS+ACTUAL_MODE->security_dist,
					    0.0,0.0,
					    &targetDist);
	
	rightDist = computeCollisionDistance(
					     rpos,
					     targetAngle + DEG_TO_RAD((float)i),
					     ACTUAL_MODE->target_max_trans_speed,
					     0.0, 
					     ROB_RADIUS+ACTUAL_MODE->security_dist,
					     0.0,0.0,
					     &targetDist);
	if (alpha1>=beta1)
	{
	   if ((leftDist> maxDistance) &&
	       ((alpha1>=leftDistAngle) && (leftDistAngle>= beta1)))
	   {
	      maxDistance = leftDist;
	      angle = leftDistAngle;	      
	   }
	   if ((rightDist > maxDistance) &&
	       ((alpha1>=rightDistAngle) && (rightDistAngle>=beta1)))
	   {
	      maxDistance = rightDist;
	      angle = rightDistAngle;	      
	   }
	}
	else
	{
	   if ((leftDist> maxDistance) &&
	       ((alpha1>=leftDistAngle) || (leftDistAngle>= beta1)))
	   {
	      maxDistance = leftDist;
	      angle = leftDistAngle;	      
	   }
	   if ((rightDist > maxDistance) &&
	       ((alpha1>=rightDistAngle) || (rightDistAngle>=beta1)))
	   {
	      maxDistance = rightDist;
	      angle = rightDistAngle;	      
	   }	   
	}
     }
     if( maxDistance < securityDist)
     {
	printf("RAP : Can not rotate away because maxDistance<securityDist \n");
	rotPossibleFlag = FALSE;
	return;
     }
  }

  /* here we know that we are able to turn to the chosen angle "angle" */
  
  delta1 = rrot - angle;

  if (delta1<= 0.0)
     delta2 = delta1 + DEG_360;
  else
     delta2 = delta1 - DEG_360;
  
  if (fabs(delta1)<= fabs(delta2))
  {  /* We prefer delta1 as rotation angle because it is the smaller one */
     if (delta1 >= 0.0)
     {  /* We prefer turning right */
	if (delta1 <= rightAngle )
	{  /* It is possible to turn right */
	   resultAngle = delta1;
	}
	else
	{  /* It is not possible to turn right, therefore we turn left */
	   resultAngle = delta2;
	}
     }
     else
     {  /* We prefer turning left */
	if (fabs(delta1) <= leftAngle)
	{  /* It is possible to turn left */
	   resultAngle = delta1;
	}
	else
	{  /* It is not possible to turn left, therefore we turn right */
	   resultAngle = delta2;
	}
     }
  }
  else
  {  /* We prefer delta2 as rotation angle because it is the smaller one */
     if (delta2 >= 0.0)
     {  /* We prefer turning right */
	if (delta2 <= rightAngle )
	{  /* It is possible to turn right */
	   resultAngle = delta2;
	}
	else
	{  /* It is not possible to turn right, therefore we turn left */
	   resultAngle = delta1;
	}
     }
     else
     {  /* We prefer turning left */
	if (fabs(delta2) <= leftAngle)
	{  /* It is possible to turn left */
	   resultAngle = delta2;
	}
	else
	{  /* It is not possible to turn left, therefore we turn right */
	   resultAngle = delta1;
	}
     }
  }
     
  printf("Suggested rotation = %f \n",RAD_TO_DEG(resultAngle));
  rotPossibleFlag = FALSE;
  return;
}














