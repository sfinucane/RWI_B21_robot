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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/question.c,v $
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
 * $Log: question.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.16  1999/03/08 16:47:46  wolfram
 * Fixed problems with map, caused by the fact that it no longer accpets
 * correction parameters from LOCALIZE.  Improved the handling of sonars
 * with Pioneers.  Currently sonars do not work properly on a B21.
 *
 * Revision 1.15  1998/08/23 00:01:04  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.14  1998/08/20 00:23:02  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.13  1998/07/01 10:46:02  fox
 * Final update of question sensor.
 *
 * Revision 1.12  1998/06/30 13:55:08  fox
 * Updated question sensor.
 *
 * Revision 1.10  1998/03/04 14:46:45  fox
 * This version should run.
 *
 * Revision 1.5  1997/12/02 15:20:41  fox
 * Nothing remarkable.
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


#include "general.h"
#include "probGrid.h"
#include "allocate.h"
#include "function.h"
#include "selection.h"

#include "question.h"
#include "abstract.h"
#include "movement.h"
#include "file.h"
#include "script.h"
#include "probGridTools.h"
#include "SOUND-messages.h"
#include "BUTTONS-messages.h"

char* ROOM_QUESTION_TYPE = "ROOM_QUESTION";

BUTTONS_set_lights_type  buttonLights;

int Rhino = 1;                         /* 1 for work with Rhino */

#define NUMBER_OF_ANSWERS 2
#define NUMBER_OF_EXPECTED_ANSWERS 2
#define NUMBER_OF_QUESTIONS 11

#define ANSWER_NO  0                   /* possible answers */
#define ANSWER_YES 1

#define FLOOR 0                        /* question numbers */
#define A110  1                       
#define A111  2
#define A112  3
#define A113  4
#define A114  5
#define A117  6
#define A118  7
#define A119  8
#define A120  9
#define A121 10


/********************************************************************
 ********************************************************************
 * Forward declarations of internal functions.
 ********************************************************************
 ********************************************************************/

static void
askQuestion( informationsFor_QUESTION* questionInfo);

static void
integrateAnswer( informationsFor_QUESTION* info);

static void
initQuestionSensings( actionInformation* actionInfo,
		      sensingActionMask* actionMask);

static abstractSensorType
abstractQuestionSensor( actionInformation* actionInfo,
			int questionNumber);

static void
initializeRoomAnswerProbTables( informationsFor_QUESTION* info,
				actionInformation* actionInfo);   

probability
probOfAnswer(informationsFor_QUESTION* info,int answer);


/**************************************************************************
 **************************************************************************
 * Mandatory functions.
 **************************************************************************
 **************************************************************************/
  
void
initialize_QUESTION( char* fileName,
		     actionInformation* actionInfo,
		     sensingActionMask* actionMask,
		     sensingFunctions* handlers)
{

  /* This struct contains all relevant information for later integration
   * of sonar informations. */
  informationsFor_QUESTION* info = 
    (informationsFor_QUESTION*) malloc( sizeof( informationsFor_QUESTION));
  
  /*-------------------------------------------------------------------------
   * Initialize parameters
   *------------------------------------------------------------------------*/
  
  bool useQuestion = FALSE;

#ifdef CONSIDER_QUESTION
  useQuestion = TRUE;
#endif
  fileName = fileName;
  
  if ( useQuestion) {
    actionMask->numberOfActions[QUESTION] = NUMBER_OF_ACTIONS_QUESTION;
  }
  else 
    actionMask->numberOfActions[QUESTION] = 0;

  
  /*------------------------------------------------------------------------
   * Initialize the handlers.
   *-----------------------------------------------------------------------*/
  handlers->checkIfConsider[QUESTION]            = checkIfConsider_QUESTION;
  handlers->checkWhichActionsToPerform[QUESTION] = checkWhichActionsToPerform_QUESTION;
  handlers->performActions[QUESTION]             = performActions_QUESTION;

  /* Initialize the general information for all questions. */
  info->grid                       = &(actionInfo->positionProbs);
  info->actualQuestion             = -1;
  info->questionIsAsked            = FALSE;
  info->buttonLights.redButton     = 0;
  info->buttonLights.greenButton   = 0;
  info->buttonLights.blueButton    = 0;
  info->buttonLights.yellowButton  = 0;
  info->answer                     = &(actionInfo->actualSensings.answer);
  info->abstractQuestion           = &(actionInfo->abstractSensors[ABSTRACT_QUESTION]);
  info->numberOfPossibleAnswers    = NUMBER_OF_ANSWERS;

  initializeRoomAnswerProbTables( info,
				  actionInfo);    

   
  /*------------------------------------------------------------------------
   * Initialize the mask.
   *-----------------------------------------------------------------------*/
  actionMask->use[QUESTION] = useQuestion;

  /* Now place the struct in the global information struct. */
  actionInfo->info[QUESTION] = info;

  
  /* Initialze the global structures of question sensings. */
  initQuestionSensings( actionInfo,
			actionMask);
}


/**************************************************************************
 * Check whether the question readings shall be considered for integration.
 **************************************************************************/
void
checkIfConsider_QUESTION( actionInformation* actionInfo,
			  sensingActionMask* mask)
{
/*   fprintf(stderr, "\nhallo! here is checkIfConsider_QUESTION.\n"); */
  mask->consider[QUESTION] = mask->use[QUESTION];
  actionInfo = actionInfo;
}



/**************************************************************************
 * Set the actual Question
 **************************************************************************/
void
setQuestion(actionInformation* actionInfo)
{
   informationsFor_QUESTION* info =
    (informationsFor_QUESTION*) actionInfo->info[QUESTION];
 
   /*    fprintf(stderr, "\nhallo! here is setQuestion\n"); */

   info->actualQuestion = 2;
   
}


/**************************************************************************
 * Check which actions shall be performed on the position probability grid.
 **************************************************************************/

void
checkWhichActionsToPerform_QUESTION( actionInformation* actionInfo,
				     sensingActionMask* mask)
{
  double entropyDiff;
  bool considerInactive = 0;
  static abstractSensorVector questionSensor;
  static float firstCall = TRUE;
  
     
  informationsFor_QUESTION* info =
    (informationsFor_QUESTION*) actionInfo->info[QUESTION];
 
  /* fprintf(stderr,"\nhallo! Which actions to perform.\n"); */
  
  mask->perform[QUESTION][INTEGRATE_ANSWER] = FALSE;
  mask->perform[QUESTION][ASK_QUESTION]     = FALSE;
  

  /* zum Testen ohne Roboter info->actualSensings.answer.isNew auf TRUE setzen */
  if (!Rhino)
     actionInfo->actualSensings.answer.isNew = TRUE;
 
   if ( firstCall) 
   {    firstCall = FALSE;
        questionSensor.sensor = (abstractSensorType*)
                       malloc ( 1 * sizeof( abstractSensorType));
   } 
  
  
  questionSensor.numberOfSensors = 1;
  
  if (info->actualQuestion != -1) 
  {
     questionSensor.sensor[0] = actionInfo->abstractSensors[ABSTRACT_QUESTION].sensor[info->actualQuestion];
     
     /*       questionSensor.sensor[0] =
	      actionInfo->abstractSensors[ABSTRACT_QUESTION].sensor[2]; 
	      
	      computes difference in entropy before and after
	      asking the question */
     entropyDiff = computeExpectedEntropyDiff( &actionInfo->localMaxima,
	  				    &questionSensor,
                                            &actionInfo->positionProbs,
               				    NULL,
		  		            &entropyDiff,
			  		    considerInactive);

      fprintf( stderr, "\nEntropy Difference: %f \n", entropyDiff);      
   }


  /* nur aufrufen, wenn der erwartete Nutzen hochgenug ist */
  if (info->actualQuestion != -1  &&  info->questionIsAsked == FALSE  &&
      entropyDiff > 0.3)
  {
     mask->perform[QUESTION][ASK_QUESTION] = TRUE;
     /* set the Countdown to wait for an answer */   
     setTimer(6);
     fprintf(stderr, "perform askQuestion\n");
  }

  /* if there is an actual Question and no button is pressed and too much time
     has passed since the last question: forget the question */
  if (info->actualQuestion != -1  &&  !actionInfo->actualSensings.answer.isNew       && timeExpired(6) > 5 && entropyDiff > 0.3)
  {   info->actualQuestion = -1; 	
      info->questionIsAsked = FALSE;

       /* switch the Buttonlights off */
       buttonLights.red_light_status = 0;
       buttonLights.green_light_status = 0;
       fprintf(stderr,"\nButtonlights off\n");
       if (Rhino)
          tcxSendMsg( BUTTONS, "BUTTONS_set_lights", &buttonLights ); 

   }
     
  /* consider the buttons only, if there is an actual Question */
  if (info->actualQuestion == -1)
      actionInfo->actualSensings.answer.isNew = FALSE;

  /* if there is a new answer that should be considered: integrate Answer */
  if ( mask->consider[QUESTION] && actionInfo->actualSensings.answer.isNew) {
    mask->perform[QUESTION][INTEGRATE_ANSWER] = TRUE;
     }
}


/**************************************************************************
 * Perform the actions on  the position probability grid.
 **************************************************************************/
void
performActions_QUESTION( actionInformation* actionInfo,
			 sensingActionMask* mask)
{ 
   /* This structure contains all relevant information for the actions. */
  informationsFor_QUESTION* info =
    (informationsFor_QUESTION*) actionInfo->info[QUESTION];

  if ( mask->perform[QUESTION][ASK_QUESTION]) 
    askQuestion( info);
   
  if ( mask->perform[QUESTION][INTEGRATE_ANSWER]) 
  {  integrateAnswer( info);
     actionInfo->actualSensings.answer.isNew = FALSE;
  }	
}

/********************************************************************
 ********************************************************************
 * Global functions.
 ********************************************************************
 ********************************************************************/


/*------------------------------------------------------------------------
 * Sets the probabilities of getting an answer given the expected answer. *
 *-----------------------------------------------------------------------*/
static void
initializeRoomAnswerProbTables( informationsFor_QUESTION* questionInfo,
				actionInformation* actionInfo)
{
   int i,j;  
  
   actionInfo = actionInfo;

   /* initialize the normed ProbabilityTable */
      
   questionInfo->probOfRoomAnswerTableNormed.prob = (probability**)
     allocate2D( NUMBER_OF_ANSWERS, NUMBER_OF_EXPECTED_ANSWERS, PROBABILITY);

   
   questionInfo->probOfRoomAnswerTableNormed.numberOfAnswers = NUMBER_OF_ANSWERS;
   questionInfo->probOfRoomAnswerTableNormed.numberOfExpectedAnswers = NUMBER_OF_EXPECTED_ANSWERS;

   
   questionInfo->probOfRoomAnswerTableNormed.prob[ANSWER_NO][ANSWER_NO] = 0.95;
   questionInfo->probOfRoomAnswerTableNormed.prob[ANSWER_NO][ANSWER_YES]= 0.05;
   
   questionInfo->probOfRoomAnswerTableNormed.prob[ANSWER_YES][ANSWER_NO]= 0.05;
   questionInfo->probOfRoomAnswerTableNormed.prob[ANSWER_YES][ANSWER_YES]= 0.95;

   /* initialize the unnormed ProbabilityTable */

   questionInfo->probOfRoomAnswerTableDividedByPOfFeature.prob =(probability**)
     allocate2D( NUMBER_OF_ANSWERS, NUMBER_OF_EXPECTED_ANSWERS, PROBABILITY);

    
   /* p(answer) = p(answer|expectedAnswer) / p(answer) */
   for (i = 0; i < NUMBER_OF_ANSWERS; i++)
      for (j = 0; j < NUMBER_OF_EXPECTED_ANSWERS; j++)
      {  questionInfo->probOfRoomAnswerTableDividedByPOfFeature.prob[i][j] =
	    questionInfo->probOfRoomAnswerTableNormed.prob[i][j]/
	       probOfAnswer(questionInfo,i);
      }   

} 


/*------------------------------------------------------------------------
 * P(answer) 
 *-----------------------------------------------------------------------*/
probability
probOfAnswer(informationsFor_QUESTION* questionInfo,int answer)
{
   int i;
   probability prob = 0;
   
   for (i=0; i <  NUMBER_OF_EXPECTED_ANSWERS; i++)
      prob = prob + questionInfo->probOfRoomAnswerTableNormed.prob[answer][i];

   prob = prob/NUMBER_OF_EXPECTED_ANSWERS;

   return prob;
}



/*------------------------------------------------------------------------
 * Checks whether a point is in the given roomArea.
 *-----------------------------------------------------------------------*/
bool
pointInArea( gridPosition pt, roomQuestionArea area)
{
  if ((pt.x > area.from.x)  &&  (pt.x < area.to.x)  &&
      (pt.y > area.from.y)  &&  (pt.y < area.to.y)    )
       return TRUE;
  else
       return FALSE;
}

/*------------------------------------------------------------------------
 * Sets the boarders of a specific room area (given by 'questionNumber')
 *-----------------------------------------------------------------------*/
static roomQuestionArea
roomArea( int questionNumber, positionProbabilityGrid* grid)
{
  roomQuestionArea positiveAnswerArea;
  realPosition fromPosition = {0,0,0};
  realPosition toPosition   = {0,0,0};
  FILE *roomdata;
  char line[80];


  roomdata = fopen("roomdata.dat","r");
 
  if (roomdata == NULL) {
    fprintf(stderr,"\n No file roomdata!\n");
    
    fprintf(stderr, "cat the following data in the file roomdata.dat:\n");
    fprintf(stderr, "# Coordinates for rooms A110 to A114 and A117 to A121\n");
    fprintf(stderr, "# room from.x from.y to.x to.y\n");
    
    fprintf(stderr, "FLOOR  360  700  2610 1300\n");
    fprintf(stderr, "A110     0 1300   845 1920 \n");
    fprintf(stderr, "A111   845 1300  1280 1920\n");
    fprintf(stderr, "A112  1280 1300  1690 1920\n");
    fprintf(stderr, "A113  1700 1300  2120 1920\n");
    fprintf(stderr, "A114  2120 1300  2540 1920\n");
    fprintf(stderr, "A117  2120    0  2540  700\n");
    fprintf(stderr, "A118  1700    0  2110  700\n");
    fprintf(stderr, "A119  1280    0  1690  700\n");
    fprintf(stderr, "A120   860    0  1265  700\n");
    fprintf(stderr, "A121     0    0   860  700\n");
    
    exit(1);
  }
  
  /* gets all lines of file "roomdata" and gets the coordinates of the specific room */
  while(fgets(line,80,roomdata)!=NULL)
  {

     if(line[0] == '.')
	break;
     if(line[0] != '#')
     {
	/* if I have the line I'm looking for */
        if((questionNumber == FLOOR) && (strncmp(line,"FLOOR",5)==0))
	{  sscanf(line+5,"%f %f %f %f",&fromPosition.x,&fromPosition.y,
	                               &toPosition.x,&toPosition.y);

           positiveAnswerArea.from = gridPositionOfRealPosition(fromPosition,grid);
           positiveAnswerArea.to   = gridPositionOfRealPosition(toPosition,grid);
   	
        }
        if((questionNumber == A110) && (strncmp(line,"A110",4)==0))
	{  sscanf(line+4,"%f %f %f %f",&fromPosition.x,&fromPosition.y,
	                               &toPosition.x,&toPosition.y);

           positiveAnswerArea.from = gridPositionOfRealPosition(fromPosition,grid);
           positiveAnswerArea.to   = gridPositionOfRealPosition(toPosition,grid);
   	
        }
        if((questionNumber == A111) && (strncmp(line,"A111",4)==0))
	{  sscanf(line+4,"%f %f %f %f",&fromPosition.x,&fromPosition.y,
	                               &toPosition.x,&toPosition.y);

           positiveAnswerArea.from = gridPositionOfRealPosition(fromPosition,grid);
           positiveAnswerArea.to   = gridPositionOfRealPosition(toPosition,grid);
   	
        }

        if((questionNumber == A112) && (strncmp(line,"A112",4)==0))
	{  sscanf(line+4,"%f %f %f %f",&fromPosition.x,&fromPosition.y,
	                               &toPosition.x,&toPosition.y);

           positiveAnswerArea.from = gridPositionOfRealPosition(fromPosition,grid);
           positiveAnswerArea.to   = gridPositionOfRealPosition(toPosition,grid);
   	
        }
        if((questionNumber == A113) && (strncmp(line,"A113",4)==0))
	{  sscanf(line+4,"%f %f %f %f",&fromPosition.x,&fromPosition.y,
	                               &toPosition.x,&toPosition.y);

           positiveAnswerArea.from = gridPositionOfRealPosition(fromPosition,grid);
           positiveAnswerArea.to   = gridPositionOfRealPosition(toPosition,grid);
   	
        }
        if((questionNumber == A114) && (strncmp(line,"A114",4)==0))
	{  sscanf(line+4,"%f %f %f %f",&fromPosition.x,&fromPosition.y,
	                               &toPosition.x,&toPosition.y);

           positiveAnswerArea.from = gridPositionOfRealPosition(fromPosition,grid);
           positiveAnswerArea.to   = gridPositionOfRealPosition(toPosition,grid);
   	
        }
        if((questionNumber == A117) && (strncmp(line,"A117",4)==0))
	{  sscanf(line+4,"%f %f %f %f",&fromPosition.x,&fromPosition.y,
	                               &toPosition.x,&toPosition.y);

           positiveAnswerArea.from = gridPositionOfRealPosition(fromPosition,grid);
           positiveAnswerArea.to   = gridPositionOfRealPosition(toPosition,grid);
   	
        }
        if((questionNumber == A118) && (strncmp(line,"A118",4)==0))
	{  sscanf(line+4,"%f %f %f %f",&fromPosition.x,&fromPosition.y,
	                               &toPosition.x,&toPosition.y);

           positiveAnswerArea.from = gridPositionOfRealPosition(fromPosition,grid);
           positiveAnswerArea.to   = gridPositionOfRealPosition(toPosition,grid);
   	
        }
        if((questionNumber == A119) && (strncmp(line,"A119",4)==0))
	{  sscanf(line+4,"%f %f %f %f",&fromPosition.x,&fromPosition.y,
	                               &toPosition.x,&toPosition.y);

           positiveAnswerArea.from = gridPositionOfRealPosition(fromPosition,grid);
           positiveAnswerArea.to   = gridPositionOfRealPosition(toPosition,grid);
   	
        }
        if((questionNumber == A120) && (strncmp(line,"A120",4)==0))
	{  sscanf(line+4,"%f %f %f %f",&fromPosition.x,&fromPosition.y,
	                               &toPosition.x,&toPosition.y);

           positiveAnswerArea.from = gridPositionOfRealPosition(fromPosition,grid);
           positiveAnswerArea.to   = gridPositionOfRealPosition(toPosition,grid);
   	
        }
        if((questionNumber == A121) && (strncmp(line,"A121",4)==0))
	{  sscanf(line+4,"%f %f %f %f",&fromPosition.x,&fromPosition.y,
	                               &toPosition.x,&toPosition.y);

           positiveAnswerArea.from = gridPositionOfRealPosition(fromPosition,grid);
           positiveAnswerArea.to   = gridPositionOfRealPosition(toPosition,grid);
   	
        }
     }
  }

  fclose(roomdata);

  return positiveAnswerArea;
  
}

/*------------------------------------------------------------------------
 * returns probability to get this 'answer' at the actual 'position' 
 *-----------------------------------------------------------------------*/

static float
probabilityOfAnswerGivenPosition( int answer,
				  gridPosition position,
				  void* infoForFeature)
{
  infoForRoomQuestionFeature* info =
    (infoForRoomQuestionFeature*) infoForFeature;

  int expectedAnswer = pointInArea( position, info->positiveAnswerArea);

  return info->probOfAnswerTable->prob[answer][expectedAnswer]; 
}
  
static float
probabilityOfAnswerGivenExpected( int answer,
				  featureStruct* expectedAnswer,
				  void* infoForFeature)
{
  infoForRoomQuestionFeature* info = (infoForRoomQuestionFeature*) infoForFeature;
  
  return info->probOfAnswerTable->prob[answer][expectedAnswer->integrateNumber];
}

/*------------------------------------------------------------------------
 * Returns the expected answer to the question at the given position.
 *-----------------------------------------------------------------------*/
static featureStruct
expectedAnswer( gridPosition position,
		void* infoForFeature)
{
  infoForRoomQuestionFeature* info = (infoForRoomQuestionFeature*) infoForFeature;
  featureStruct fStruct;
  
   /* Default values for the feature struct. */
  fStruct.quality = 0;
  fStruct.numberOfQualities = 1;

  fStruct.integrateNumber = fStruct.feature =
    pointInArea( position, info->positiveAnswerArea);

  return fStruct;
}


/********************************************************************
 ********************************************************************
 * The local functions.
 ********************************************************************
 ********************************************************************/

/*------------------------------------------------------------------------
 *
 *-----------------------------------------------------------------------*/
void
sound_play( char *play )
{
 fprintf( stderr, "Playing \"%s\"\n", play );
 if (SOUND==NULL)
    fprintf( stderr, "SOUND not connected!\n");
 else
    tcxSendMsg( SOUND, "SOUND_play_query", &play );
 fprintf( stderr, "Sound is sent\n");
}


static void
askQuestion( informationsFor_QUESTION* questionInfo)
{

   int questionNumber;
   char questionString[80];

   /*   questionInfo->actualQuestion = 2;  // 2 == Wolfram */

   fprintf(stderr,"\nhere is askQuestion\n"); 

   questionNumber = questionInfo->actualQuestion; 

   if (questionNumber != -1)
   {        
       infoForRoomQuestionFeature* info = (infoForRoomQuestionFeature*)
           questionInfo->abstractQuestion->sensor[questionNumber].infoForFeatures;

       info->actualQuestion = TRUE;
       
       /*       fprintf(stderr,"\nquestionNumber: %d\n", questionNumber); */
      
       fprintf(stderr,"\nQuestion: %s\n",info->questionString); 

       sprintf(questionString,"%d",questionNumber);

       /* ask the question now! */
       if (Rhino)
         sound_play( questionString );

       /* switch the red and the green button on */
       buttonLights.red_light_status = 1;
       buttonLights.green_light_status = 1;
       fprintf(stderr,"\nButtonlights on\n");
       if (Rhino)
         tcxSendMsg( BUTTONS, "BUTTONS_set_lights", &buttonLights ); 

       
       /* question is asked, so set the Countdown to wait for an answer */
       /*   setTimer(6);*/

       questionInfo->questionIsAsked = TRUE;
       
    }
}


/*------------------------------------------------------------------------
 * Integrates an answer into the position probability grid.
 *-----------------------------------------------------------------------*/
static void
integrateAnswer( informationsFor_QUESTION* questionInfo)
{
  int x,y,plane;
  int questionNumber;
  /*float gridcell;*/
  gridPosition position;
  register probability **gridPlane;
  abstractSensorType* question = NULL;

  fprintf(stderr,"\nIntegrate the Answer\n"); 

  /* put the struct for the actual Question in "question" */
  for (questionNumber = 0; questionNumber < NUMBER_OF_QUESTIONS;
       questionNumber++)
  { infoForRoomQuestionFeature* info = (infoForRoomQuestionFeature*)
    questionInfo->abstractQuestion->sensor[questionNumber].infoForFeatures;
   
    if (info->actualQuestion)
       question = &(questionInfo->abstractQuestion->sensor[questionNumber]);
  }

  /* update the grid now */
  for (plane = 0; plane < questionInfo->grid->sizeZ; plane++)
    if ( questionInfo->grid->updatePlane[plane])
    {
      gridPlane = questionInfo->grid->prob[plane];
      
      for (x = 0; x < questionInfo->grid->sizeX; x++)
	for (y = 0; y < questionInfo->grid->sizeY; y++)
	  {
	    position.x = x;
	    position.y = y;
	    position.rot = plane;
       	    gridPlane[x][y] *=
	      question->probOfFeatureGivenPosition(questionInfo->answer->
						                 answerType,
						   position,
						   question->infoForFeatures);
          }
    }

  questionInfo->actualQuestion = -1;
  questionInfo->questionIsAsked = FALSE;

}  

/*------------------------------------------------------------------------
 * Extracts the answer from the given raw data. 
 *-----------------------------------------------------------------------*/
static int
extractedQuestionFeature( rawSensings* rawData,
			  int sensorNumber,
			  void* infoForFeature)
{
  infoForRoomQuestionFeature* info = (infoForRoomQuestionFeature*) infoForFeature;


  if ( rawData->answer.isNew)
  {  if ( info->actualQuestion) 
	{  fprintf(stderr, "Answer to Sensornumber %d is: %d\n",
		   sensorNumber, rawData->answer.answerType);

           /* switch the Buttonlights off */
	   buttonLights.red_light_status = 0;
           buttonLights.green_light_status = 0;
           fprintf(stderr,"\nButtonlights off\n");
           if (Rhino)
	     tcxSendMsg( BUTTONS, "BUTTONS_set_lights", &buttonLights ); 
 

	   /* return which button is pressed */
	   return rawData->answer.answerType; 
	}
   }
}


/*------------------------------------------------------------------------
 * Set the probability table to the normed probabilties.
 *-----------------------------------------------------------------------*/
static void
setNormedAnswerTab( void* infoForFeature,
		    float** uninformed)
{
  infoForRoomQuestionFeature* info = (infoForRoomQuestionFeature*) infoForFeature;

  info->probOfAnswerTable = info->probOfAnswerTableNormed;
  
  *uninformed = info->uninformedNormed;
}

/*------------------------------------------------------------------------
 * Set the probability table to the unnormed probabilties. This is necessary
 * for selective update and selection with considerInactive flag.
 *-----------------------------------------------------------------------*/
static void
setUnNormedAnswerTab( void* infoForFeature,
		      float** uninformed)
{
  infoForRoomQuestionFeature* info = (infoForRoomQuestionFeature*) infoForFeature;

  info->probOfAnswerTable = info->probOfAnswerTableDividedByPOfFeature;
  
  *uninformed = info->uninformedDividedByPOfFeature;
}

     
/*------------------------------------------------------------------------
 * Initializes the abstract question sensings.
 *-----------------------------------------------------------------------*/
static void
initQuestionSensings( actionInformation* actionInfo,
		      sensingActionMask* actionMask)
{
  int i;
  sensing_QUESTION* rawAnswer;
  abstractSensorVector* abstractQuestion;
   
  rawAnswer = &( actionInfo->actualSensings.answer);
  abstractQuestion = &( actionInfo->abstractSensors[ABSTRACT_QUESTION]);

  
  /*------------------------------------------------------------------------
   * Initialize the global structure of abstract sensors.
   *-----------------------------------------------------------------------*/
  if ( actionMask->use[QUESTION]) {
    abstractQuestion->numberOfSensors = NUMBER_OF_QUESTIONS;
    abstractQuestion->sensor  = (abstractSensorType*)
      malloc ( NUMBER_OF_QUESTIONS * sizeof( abstractSensorType));

    
    for ( i = 0; i < NUMBER_OF_QUESTIONS; i++) {
      abstractQuestion->sensor[i] = abstractQuestionSensor( actionInfo, i);
    }

    abstractQuestion->numberOfSensorsToBeUsed = 1;
    abstractQuestion->sensorOffsetType = NO_OFFSET;
    abstractQuestion->mask = (int*) allocate1D( abstractQuestion->numberOfSensors,
						INT);

    /* Pointers on information about integration. */
    abstractQuestion->integrate =
      &(actionMask->perform[QUESTION][INTEGRATE_ANSWER]);
    
    setFixedScanMask( abstractQuestion, 0);
  }
  else 
    abstractQuestion->numberOfSensors = 0;
}


static abstractSensorType
abstractQuestionSensor( actionInformation* actionInfo,
			int questionNumber)
{
  abstractSensorType sensor;
  infoForRoomQuestionFeature* info;
  informationsFor_QUESTION* questionInfo =
    (informationsFor_QUESTION*) actionInfo->info[QUESTION];
  static float* averageAnswerProb, *averageAnswerProbUnNormed;
  
  static float firstCall = TRUE;

  if ( firstCall) {
    firstCall = FALSE;
    averageAnswerProb = (float*) malloc( sizeof( float));
    averageAnswerProbUnNormed = (float*) malloc( sizeof( float));
    *averageAnswerProb = 1.0 / questionInfo->numberOfPossibleAnswers;
    *averageAnswerProbUnNormed = 1.0;
  }

  /* Information for one specific question. */
  info = (infoForRoomQuestionFeature*) malloc( sizeof(infoForRoomQuestionFeature));

  info->number = questionNumber;
  info->actualQuestion      = FALSE;

  info->questionString = (char*) malloc (20 * sizeof(char));
  switch(questionNumber)
  {
     case 0: info->questionString = "Bin ich im Flur?";       break;
     case 1: info->questionString = "Bin ich in A110?";       break;
     case 2: info->questionString = "Bin ich bei Wolfram?";   break;
     case 3: info->questionString = "Bin ich in A112?";       break;
     case 4: info->questionString = "Bin ich in A113?";       break;
     case 5: info->questionString = "Bin ich in A114?";       break;
     case 6: info->questionString = "Bin ich in A117?";       break;
     case 7: info->questionString = "Bin ich in A118?";       break;
     case 8: info->questionString = "Bin ich in A119?";       break;
     case 9: info->questionString = "Bin ich bei Dieter?";    break;
     case 10:info->questionString = "Bin ich in A120?";       break;
  }

  info->positiveAnswerArea  = roomArea( questionNumber,
					&(actionInfo->positionProbs));
  info->probOfAnswerTable   =
    &(questionInfo->probOfRoomAnswerTableDividedByPOfFeature);
  info->probOfAnswerTableDividedByPOfFeature   =
    &(questionInfo->probOfRoomAnswerTableDividedByPOfFeature);
  info->probOfAnswerTableNormed   =
    &(questionInfo->probOfRoomAnswerTableNormed);
  info->uninformedNormed   = averageAnswerProb;
  info->uninformedDividedByPOfFeature  = averageAnswerProbUnNormed;

  
  /* Offset of the sensors relative to the center of the robot.
   * Zero for questions. */
  sensor.integrationInfo.sensorOffset.forward        = 0.0;
  sensor.integrationInfo.sensorOffset.sideward        = 0.0;
  sensor.integrationInfo.sensorOffset.rotation        = 0.0;
  
  /* Link the specific sensor info into the abstract sensor type. */
  sensor.infoForFeatures    = info;
  sensor.sensorNumberOfType = questionNumber;
  sensor.numberOfFeatures   = questionInfo->numberOfPossibleAnswers;
  sensor.probOfFeatureGivenPosition = &probabilityOfAnswerGivenPosition;
  sensor.probOfFeatureGivenExpected = &probabilityOfAnswerGivenExpected;
  sensor.expectedFeature    = &expectedAnswer;

  sensor.setNormedFeatureProbs = &setNormedAnswerTab;
  sensor.setUnNormedFeatureProbs = &setUnNormedAnswerTab;

  sensor.type                  = ROOM_QUESTION_TYPE;
  sensor.extractedFeature      = &extractedQuestionFeature;
  
  return sensor;
}




