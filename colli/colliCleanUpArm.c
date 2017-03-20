
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliCleanUpArm.c,v $
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
 * $Log: colliCleanUpArm.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.3  1997/04/09 12:57:48  fox
 * Minor changes.
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

/**********************************************************************/
/* The next functions communicate with the arm module. */
/**********************************************************************/
void ARM_moveToGround(void)
{
  if (use_arm) {
    tcxSendMsg(ARM, "ARM_pickup_at_ground_query", NULL);
    SOUND_talk_text("I pick it up."); 
    SOUND_play_message(16);
  }
  else {
    /* We ignore this command at the first time. The second
     * time we send the successful execution.
     * Second call should be performed by BASE.
     */
    static BOOLEAN firstTime = TRUE;
    if (firstTime) {
      /* SOUND_talk_text("I pick it up."); */
      firstTime = FALSE;
    }
    else {
      ARM_moveToGroundReady( TRUE);
      firstTime = TRUE;
    }
  }
}


/**********************************************************************/
void ARM_liftObject(void)
{
  if (use_arm) {
    tcxSendMsg(ARM, "ARM_lift_object_query", NULL);
    SOUND_talk_text("Got it."); 
    SOUND_play_message(9);
  }
  else {
    /* We ignore this command at the first time. The second
     * time we send the successful execution.
     * Second call should be performed by BASE.
     */
    static BOOLEAN firstTime = TRUE;
    if (firstTime) {
      /* SOUND_talk_text("Got it."); */
      firstTime = FALSE;
    }
    else {
      ARM_liftObjectReady( TRUE);
      firstTime = TRUE;
    }
  }
}

/**********************************************************************/
void ARM_dropObject(void)
{
  if (use_arm) {
    tcxSendMsg(ARM, "ARM_drop_object_query", NULL);
    SOUND_talk_text("Thank you."); 
    SOUND_play_message(19);
  }
  else {
    /* We ignore this command at the first time. The second
     * time we send the successful execution.
     * Second call should be performed by BASE.
     */
    static BOOLEAN firstTime = TRUE;
    if (firstTime) {
      /* SOUND_talk_text("Thank you."); */
      firstTime = FALSE;
    }
    else {
      ARM_dropObjectReady( TRUE);
      firstTime = TRUE;
    }
  }
}

/**********************************************************************/
void ARM_moveIn(void)
{
  if (use_arm) {
    tcxSendMsg(ARM, "ARM_move_in_query", NULL);
    SOUND_talk_text("That's it."); 
    SOUND_play_message(3);
  }
  else {
    /* We ignore this command at the first time. The second
     * time we send the successful execution.
     * Second call should be performed by BASE.
     */
    static BOOLEAN firstTime = TRUE;
    if (firstTime) {
      /* SOUND_talk_text("That's it."); */
      firstTime = FALSE;
    }
    else {
      ARM_moveInReady( TRUE);
      firstTime = TRUE;
    }
  }
}

/**********************************************************************
 *  The next functions react to successful execution of arm commands.
 **********************************************************************/

/* The arm is down and we translate the last centimeters. */
void ARM_moveToGroundReady( int success )
{
  fprintf( stderr,"Pickup ready (%d)!\n", success);

  armState = READY_TO_GRIP;
  COLLI_GoBackward();
}

/* The object is lifted. */
void ARM_liftObjectReady(int success)
{
  fprintf( stderr,"Lift ready (%d)!\n", success);

  armState = OBJECT_LIFTED;
}

/* The object is dropped. */
void ARM_dropObjectReady(int success)
{
  fprintf( stderr,"Drop ready (%d)!\n", success);

  armState = OBJECT_DROPPED;
}

/* The arm is inside agein. */
void ARM_moveInReady(int success)
{
  fprintf( stderr,"Move in ready (%d)!\n", success);

  armState = INSIDE;
  COLLI_GoForward();
}




#endif
