
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliCleanUp.c,v $
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
 * $Log: colliCleanUp.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.2  1996/09/24 07:13:14  rhino
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
#include "colliCleanUpStates.h"


/********************************************************************
 * Resets all flags and sets the robot into the default mode. 
 **********************************************************************/
void
resetCleanUpStates()
{
  cleanUpState = NOT_RUNNING;

  /* All internal states are not active.\n*/
  /* PICK */
  randomWalkInternalState = Not_Active;
  approachFarObjectInternalState = Not_Active;
  getExactObjectPositionInternalState = Not_Active;
  approachCloseObjectInternalState = Not_Active;
  pickUpObjectInternalState = Not_Active;

  /* DROP */
  lookForTrashBinInternalState = Not_Active;
  approachFarTrashBinInternalState = Not_Active;
  getExactTrashBinPositionInternalState = Not_Active;
  approachCloseTrashBinInternalState = Not_Active;
  dropObjectInternalState = Not_Active;

  /* We delete all stored objects except the far trash bins. */
  numberOfCloseObjects = 0;
  numberOfFarObjects = 0; 
  numberOfCloseTrashBins = 0;
  /* numberOfFarTrashBins = 0; */
  
  /* Reset the default mode. */
  COLLI_SetMode( (double) DEFAULT_MODE);
}
  

/**********************************************************************
 * The robot starts to clean up the ground and to put the objects in
 * trash bins.
 **********************************************************************/
void
COLLI_StartCleaningUp()
{
  /* Reset the finite automata. */
  resetCleanUpStates();

  BASE_TranslateVelocity (0.0);
  BASE_RotateVelocity ( 0.0);
  BASE_TranslateHalt();
  BASE_RotateHalt();

  COLLI_GoForward();
  
  /* The robot starts by walking around randomly. */
  cleanUpState = RANDOM_WALK;
}


/**********************************************************************
 * Stop is.
 **********************************************************************/
void
COLLI_StopCleaningUp()
{
  /* Reset the finite automata. */
  resetCleanUpStates();

  numberOfFarObjects = numberOfFarTrashBins = 0;
  
  COLLI_GoForward();
  
  BASE_TranslateHalt();
  BASE_RotateHalt();

  target_flag = FALSE;
  
  cleanUpState = NOT_RUNNING;
}
    


/**********************************************************************
 * Checks wether the robot is in a sequence of the following modes.
 **********************************************************************/
BOOLEAN
inCleanUpSequence( Point rPos, float rRot)
{
    switch (cleanUpState) {
    case NOT_RUNNING :
	return CONTINUE_WITH_COLLISION_AVOIDANCE;
    case RANDOM_WALK :
	return randomWalkState( rPos, rRot);
    case APPROACH_FAR_OBJECT :
      return approachFarObjectState( rPos, rRot);
    case GET_EXACT_OBJECT_POSITION :
	return getExactObjectPositionState( rPos, rRot);
    case APPROACH_CLOSE_OBJECT :
	return approachCloseObjectState( rPos, rRot);
    case PICKUP_OBJECT :
	return pickUpObjectState( rPos, rRot);
    case LOOK_FOR_TRASH_BIN :
	return lookForTrashBinState( rPos, rRot);
    case APPROACH_FAR_TRASH_BIN :
	return approachFarTrashBinState( rPos, rRot);
    case GET_EXACT_TRASH_BIN_POSITION :
	return getExactTrashBinPositionState( rPos, rRot);
    case APPROACH_CLOSE_TRASH_BIN :
	return approachCloseTrashBinState( rPos, rRot);
    case DROP_OBJECT :
	return dropObjectState( rPos, rRot);
    default :
       return CONTINUE_WITH_COLLISION_AVOIDANCE;
    }
}


#endif
