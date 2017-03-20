
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/question.h,v $
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
 * $Log: question.h,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.9  1998/08/23 00:01:04  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.8  1998/07/01 10:46:02  fox
 * Final update of question sensor.
 *
 * Revision 1.7  1998/06/30 13:55:09  fox
 * Updated question sensor.
 *
 * Revision 1.6  1998/03/04 14:46:46  fox
 * This version should run.
 *
 * Revision 1.5  1998/01/22 13:06:21  fox
 * First version after selection-submission.
 *
 * Revision 1.4  1997/11/28 14:11:52  fox
 * Minor changes.
 *
 * Revision 1.3  1997/11/28 13:34:37  fox
 * Added questions.
 *
 * Revision 1.2  1997/11/26 15:47:45  fox
 * Added some structures for questions.
 *
 * Revision 1.1  1997/11/21 15:36:07  fox
 * Modifications in graphic
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#ifndef QUESTION_INCLUDE
#define QUESTION_INCLUDE

#include "general.h"
#include "sensings.h"
#include "probGrid.h"
#include "proximity.h"
#include "activeLocalize.h"
/* #include "BUTTONS-messages.h" */

/*********************************************************
 *********************************************************
 * The following functions and structures are mandatory for
 * each sensor.
 *********************************************************
 *********************************************************/


#define NUMBER_OF_ACTIONS_QUESTION 2

/* Action to be performed for the question. */
#define ASK_QUESTION             0
#define INTEGRATE_ANSWER         1

#define NO_COLOR -1
#define RED       0
#define GREEN     1
#define YELLOW    2
#define BLUE      3


void
initialize_QUESTION( char* fileName,
		     actionInformation* actionInfo,
		     sensingActionMask* actionMask,
		     sensingFunctions* handlers);


void
checkIfConsider_QUESTION( actionInformation* actionInfo,
			  sensingActionMask* mask);
void
setQuestion(actionInformation* actionInfo);

void
checkWhichActionsToPerform_QUESTION( actionInformation* actionInfo,
				     sensingActionMask* mask);

void
performActions_QUESTION( actionInformation* actionInfo,
			 sensingActionMask* mask);



/*********************************************************
 *********************************************************
 * Structures.
 *********************************************************
 *********************************************************/


/* This struct is used to get the probability of an answer given the
 * question.
 */
typedef struct {
  int numberOfAnswers;
  int numberOfExpectedAnswers;
  probability** prob;
} roomAnswerProbTable;


typedef struct {
  gridPosition from;
  gridPosition to;
} roomQuestionArea;

typedef struct {
  int redButton;
  int greenButton;
  int blueButton;
  int yellowButton;
} buttons;


/* This struct stores general information to integrate question information. */
typedef struct {
  positionProbabilityGrid* grid;
  int                      actualQuestion;
  bool                     questionIsAsked;
  buttons                  buttonLights;  
  sensing_QUESTION*        answer;
  abstractSensorVector*    abstractQuestion;
  int                      numberOfPossibleAnswers;
  roomAnswerProbTable      probOfRoomAnswerTableNormed;
  roomAnswerProbTable      probOfRoomAnswerTableDividedByPOfFeature;
} informationsFor_QUESTION;
   

/* This struct stores information needed to integrate the question information. */
typedef struct {
  int                  number;
  bool                 actualQuestion;
  char*                questionString;
  roomQuestionArea     positiveAnswerArea;
  roomAnswerProbTable* probOfAnswerTableDividedByPOfFeature;
  roomAnswerProbTable* probOfAnswerTableNormed;
  roomAnswerProbTable* probOfAnswerTable;
  float*               uninformedNormed;
  float*               uninformedDividedByPOfFeature;
} infoForRoomQuestionFeature;



/*********************************************************
 *********************************************************
 * Functions.
 *********************************************************
 *********************************************************/

#endif








