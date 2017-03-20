
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliCleanUpStates.h,v $
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
 * $Log: colliCleanUpStates.h,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:05  rhino
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


/*************************************************************
 * colliCleanUpPick.c
 *************************************************************/

/* Different internal states of the main states. */
extern int randomWalkInternalState;
extern int approachFarObjectInternalState;
extern int getExactObjectPositionInternalState;
extern int approachCloseObjectInternalState;
extern int pickUpObjectInternalState;


/* Values for the internal states. */
#define Not_Active 0
#define Active 1
#define Wait_Until_Rotation_Finished 2
#define Wait_Until_Translation_Finished 3
#define Wait_For_Coarse_Position 4
#define Wait_For_Exact_Position 5

#define Wait_Until_Object_Reached 6
#define Wait_Until_Trash_Bin_Reached 7
#define Wait_Until_Facing_Trash_Bin 8

#define Wait_Until_Arm_Down 9
#define Wait_Until_Object_Lifted 10
#define Wait_Until_Object_Dropped 11
#define Wait_Until_Arm_In 12

/* Values for the internal states. */

/***************************************************************************
 * The robot moves randomly and waits until the vision sends an object.
 ***************************************************************************/
BOOLEAN
randomWalkState( Point rPos, float rRot);
BOOLEAN
startRandomWalkState();
BOOLEAN
quitRandomWalkState();
BOOLEAN
randomWalkStateFailed();


/***************************************************************************
 * The robot approaches a far object.
 ***************************************************************************/
BOOLEAN
approachFarObjectState( Point rPos, float rRot);
BOOLEAN
startApproachFarObjectState();
BOOLEAN
quitApproachFarObjectState();
BOOLEAN
approachFarObjectStateFailed();



/***************************************************************************
 * Waits for the vision to send the exact position of an object. 
 ***************************************************************************/
BOOLEAN
getExactObjectPositionState( Point rPos, float rRot);
BOOLEAN
startGetExactObjectPositionState();
BOOLEAN
quitGetExactObjectPositionState();
BOOLEAN
getExactObjectPositionStateFailed();



/***************************************************************************
 * Approaches a close object.
 ***************************************************************************/
BOOLEAN
approachCloseObjectState( Point rPos, float rRot);
BOOLEAN
startApproachCloseObjectState( Point rPos, float rRot);
BOOLEAN
quitApproachCloseObjectState();
BOOLEAN
approachCloseObjectStateFailed();

/***************************************************************************
 * Picks up an object.
 ***************************************************************************/
BOOLEAN
pickUpObjectState( Point rPos, float rRot);
BOOLEAN
startPickUpObjectState();
BOOLEAN
quitPickUpObjectState();
BOOLEAN
pickUpObjectStateFailed();






/***************************************************************************
 *
 ***************************************************************************/


/*************************************************************
 * colliCleanUpDrop.c
 *************************************************************/

extern int lookForTrashBinInternalState;
extern int approachFarTrashBinInternalState;
extern int getExactTrashBinPositionInternalState;
extern int approachCloseTrashBinInternalState;
extern int dropObjectInternalState;

/***************************************************************************
 * The robot moves randomly ( the arm is outside) and look for trash bins.
 ***************************************************************************/
BOOLEAN
lookForTrashBinState( Point rPos, float rRot);
BOOLEAN
startLookForTrashBinState();
BOOLEAN
quitLookForTrashBinState();
BOOLEAN
lookForTrashBinStateFailed();

/***************************************************************************
 * Approaches far trash bin.
 ***************************************************************************/
BOOLEAN
approachFarTrashBinState( Point rPos, float rRot);
BOOLEAN
startApproachFarTrashBinState();
BOOLEAN
quitApproachFarTrashBinState();
BOOLEAN
approachFarTrashBinStateFailed();



/***************************************************************************
 * Waits for the vision to send exact position of a trash bin.
 ***************************************************************************/
BOOLEAN
getExactTrashBinPositionState( Point rPos, float rRot);
BOOLEAN
startGetExactTrashBinPositionState();
BOOLEAN
quitGetExactTrashBinPositionState();
BOOLEAN
getExactTrashBinPositionStateFailed();


/***************************************************************************
 *  Approaches a close trash bin.
 ***************************************************************************/
BOOLEAN
approachCloseTrashBinState( Point rPos, float rRot);
BOOLEAN
startApproachCloseTrashBinState( Point rPos, float rRot);
BOOLEAN
quitApproachCloseTrashBinState();
BOOLEAN
approachCloseTrashBinStateFailed();



/***************************************************************************
 * Drops an object into the trash bin.
 ***************************************************************************/
BOOLEAN
dropObjectState( Point rPos, float rRot);
BOOLEAN
startDropObjectState();
BOOLEAN
quitDropObjectState();
BOOLEAN
dropObjectStateFailed();








