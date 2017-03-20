
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/sensings.c,v $
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
 * $Log: sensings.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.56  2000/03/06 20:00:46  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.55  1999/11/02 18:12:37  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.54  1999/10/21 17:30:45  fox
 * Fixed some bugs in real time scripts.
 *
 * Revision 1.53  1999/09/29 16:06:12  fox
 * Should work.
 *
 * Revision 1.52  1999/09/09 02:48:38  fox
 * Final version before germany.
 *
 * Revision 1.51  1999/09/06 16:36:04  fox
 * Many changes.
 *
 * Revision 1.50  1999/09/01 21:26:55  fox
 * Works almost perfectly :-)
 *
 * Revision 1.49  1999/09/01 00:02:58  fox
 * Getting closer. But still not close enough.
 *
 * Revision 1.48  1999/08/30 05:48:43  fox
 * Doesn't work!!
 *
 * Revision 1.47  1999/04/26 18:55:39  fox
 * Communication with sampling seems to work (no more stuck situations).
 *
 * Revision 1.46  1999/04/21 22:58:01  fox
 * First attempt to get samples from multi back.
 *
 * Revision 1.45  1999/04/21 14:06:01  fox
 * Just an intermediate version.
 *
 * Revision 1.44  1999/01/11 19:47:57  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.43  1998/11/17 23:26:28  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.42  1998/11/03 21:02:23  fox
 * Lasers will not be integrated if the robot rotates faster than
 * MAX_ROT_VEL_FOR_LASERS deg/sec.
 *
 * Revision 1.41  1998/10/02 15:16:43  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.40  1998/09/25 17:53:32  fox
 * Improved version of condensation.
 *
 * Revision 1.39  1998/09/25 04:02:59  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.38  1998/09/18 17:24:44  fox
 * Added skeleton files for condensation.
 *
 * Revision 1.37  1998/09/18 15:44:29  fox
 * First attempt to get a running system after museum.
 *
 * Revision 1.36  1998/08/20 00:23:02  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.35  1998/03/17 08:41:12  wolfram
 * First steps to add vision.
 *
 * Revision 1.34  1998/01/22 13:06:24  fox
 * First version after selection-submission.
 *
 * Revision 1.33  1997/12/19 11:30:15  fox
 * FIXED a bug I added.
 *
 * Revision 1.32  1997/12/02 15:20:43  fox
 * Nothing remarkable.
 *
 * Revision 1.31  1997/11/25 17:13:08  fox
 * Should work.
 *
 * Revision 1.30  1997/11/21 15:36:08  fox
 * Modifications in graphic
 *
 * Revision 1.29  1997/08/02 16:51:08  wolfram
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
 * Revision 1.28  1997/07/04 17:29:18  fox
 * Final version before holiday!!!
 *
 * Revision 1.27  1997/06/26 11:53:44  fox
 * Minor changes.
 *
 * Revision 1.26  1997/06/26 11:23:18  fox
 * Fixed a bug in normalize.
 *
 * Revision 1.25  1997/06/25 14:16:42  fox
 * Changed laser incorporation.
 *
 * Revision 1.24  1997/06/20 07:36:15  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.23  1997/05/26 08:47:58  fox
 * Last version before major changes.
 *
 * Revision 1.22  1997/04/30 12:25:42  fox
 * Some minor changes.
 *
 * Revision 1.21  1997/04/10 13:01:04  fox
 * Fixed a bug.
 *
 * Revision 1.20  1997/04/08 14:56:26  fox
 * Changed the reset function for new planes and the prob function.
 *
 * Revision 1.19  1997/04/02 08:57:35  fox
 * Display the robot right after sensor selection.
 *
 * Revision 1.18  1997/03/24 21:51:44  wolfram
 * Minor changes
 *
 * Revision 1.17  1997/03/24 06:55:29  wolfram
 * Cleaned up graphic.c and added a graphic window as a global variable
 * in graphic.c
 *
 * Revision 1.16  1997/03/18 18:45:31  fox
 * First executable version with a global scan mask for different sensor types.
 *
 * Revision 1.15  1997/03/14 17:58:24  fox
 * This version should run quite stable now.
 *
 * Revision 1.14  1997/01/29 12:23:14  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.13  1997/01/18 17:24:45  wolfram
 * Fixed two bugs
 *
 * Revision 1.12  1997/01/16 19:43:25  fox
 * And another bug ...
 *
 * Revision 1.11  1997/01/16 16:17:28  wolfram
 * Cells no longer in the map are removed from the list of local maxima
 *
 * Revision 1.10  1997/01/16 12:42:52  fox
 * Fixed a bug in scan masks.
 *
 * Revision 1.9  1997/01/10 15:19:25  fox
 * Improved several methods.
 *
 * Revision 1.8  1997/01/08 16:11:07  fox
 * Fixed a bug.
 *
 * Revision 1.7  1997/01/08 15:53:00  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.6  1997/01/03 10:09:48  fox
 * First version with exploration.
 *
 * Revision 1.5  1996/12/02 10:32:14  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.4  1996/11/22 10:30:25  wolfram
 * Added graphic for expected errors of movements
 *
 * Revision 1.3  1996/11/18 09:58:32  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.2  1996/11/15 17:44:09  ws
 * *** empty log message ***
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


#include "general.h"
#include "sensings.h"
#include "sonar.h"
#include "laser.h"
#include "movement.h"
#include "angle.h"
#include "question.h"
#include "vision.h"
#include "abstract.h"
#include "graphic.h"
#include "script.h"
#include "localTcx.h"
#include "condensation.h"
#include "activeInternal.h"

/* The main structure to call the different functions. */
sensingFunctions actionHandlers;

static void
updateSampleSetHistory( sampleSetHistory* history);

/***************************************************************************
 * Here the command line arguments are passed to the different modules.
 ***************************************************************************/
void
initialize_SENSINGS( char* fileName,
		    actionInformation* actionInfo,
		    sensingActionMask* actionMask)
{
  int sensing;
  int position;
  sampleSetHistory* history = &(actionInfo->samplesHistory);
  unsigned int action;
  
  initialize_SONAR( fileName, actionInfo, actionMask, &actionHandlers);
  initialize_LASER( fileName, actionInfo, actionMask, &actionHandlers);
  initialize_VISION( fileName, actionInfo, actionMask, &actionHandlers); 
  initialize_MOVEMENT( fileName, actionInfo, actionMask, &actionHandlers);
  initialize_ANGLE( fileName, actionInfo, actionMask, &actionHandlers);
  initialize_QUESTION( fileName, actionInfo, actionMask, &actionHandlers); 

  /* Set all movements to zero. */
  for ( sensing = 0; sensing < NUMBER_OF_SENSINGS; sensing++)
    for ( action = 0; action < actionMask->numberOfActions[sensing];
	  action++) {
      actionInfo->summedMovements[sensing][action] = 0.0;
      actionMask->perform[sensing][action] = FALSE;
    }

  /* Set the pointers of the sample history. */
  history->currentSampleSetNumber = -1;
  history->currentSampleSetPosition = 0;
  history->sizeOfHistory = SIZE_OF_SAMPLE_SET_HISTORY;
  history->actionInfo = (void*) actionInfo;
  history->actionMask = (void*) actionMask;

  for ( position = 0; position < history->sizeOfHistory; position++) {
    history->samples[position] = NULL;
    history->numberOfSamples[position] = 0;
    history->numberOfAllocatedSamples[position] = 0;
    history->timeStamp[position].tv_sec = 0;
    history->timeStamp[position].tv_usec = 0;
  }
}


/***************************************************************************
 * Checks wether a sensing should be considered for the integration into
 * the position probability grid.
 ***************************************************************************/
void
checkWhichSensingsToConsider( actionInformation* info, sensingActionMask* mask)
{
  int sensing;

  for ( sensing = 0; sensing < NUMBER_OF_SENSINGS; sensing++)
    actionHandlers.checkIfConsider[sensing]( info, mask);
}


/***************************************************************************
 * Checks wether an action should be performed on the position probability grid.
 ***************************************************************************/
void
checkWhichActionsToPerform( actionInformation* info, sensingActionMask* mask)
{
  int sensing;
  
  for ( sensing = 0; sensing < NUMBER_OF_SENSINGS; sensing++)
    actionHandlers.checkWhichActionsToPerform[sensing]( info, mask);

  checkWhichActionsToPerform_CONDENSATION( info, mask);
  checkWhichActionsToPerform_PROBGRID( info, mask);
}


/***************************************************************************
 * Performs the actions if they are set in the mask.
 ***************************************************************************/
void
performActions( actionInformation* info,
		sensingActionMask* mask)
{
  int sensing;

  /* Perform actions of the different sensors. */
  for ( sensing = 0; sensing < NUMBER_OF_SENSINGS; sensing++) {

    /* Movement is sensing no. 0 */
    if ( sensing == 1) {

      /* Set the global scan mask for all abstract sensors. */
      setGlobalScanMask( info->abstractSensors,
			 &(info->localMaxima),
			 &(info->positionProbs),
			 &(info->samples),
			 info->useProbGrid);
    }
    if ( mask->consider[sensing]) 
      actionHandlers.performActions[sensing]( info, mask);
      
    
    if ( info->useProbGrid && sensing > 0 && mask->consider[sensing]) {
      setTimer(GRAPHICS_TIMER);
      updateGlobalRobotWindow( info, mask);
      nonRelevantTime += timeExpired(GRAPHICS_TIMER);
    }
  }

  /* Normalize and update the estimated position of the robot. */
  if ( info->useProbGrid) {
    performActions_PROBGRID( info, mask);
  }    
  else {
    performActions_CONDENSATION( info, mask);
    setTimer(GRAPHICS_TIMER);
    updateGlobalRobotWindow( info, mask);
    nonRelevantTime += timeExpired(GRAPHICS_TIMER);
  }


  /* Perform the actions according to the state of the localization. */
  performActions_ACTIVE( info, mask);
  
#ifdef WRITE_MARKERS
  if ( mask->perform[LASER][INTEGRATE_LASER] &&
       numberOfLaserIntegrationsSinceStopped == 10)
    checkPosition( info, mask);
#endif
}



/*****************************************************************************
 * Accumulates the relative movements for each plane of the prob grid
 * and updates the summed movements for the actions.
 *****************************************************************************/
void
accumulateMovements( actionInformation* actionInfo,
		     sensingActionMask* mask)
{
  if ( actionInfo->actualSensings.delta.isNew) {

    int plane, sensing, max;
    unsigned int action;

    float deltaAbs;

    /* Check how often the robot has not moved. */
    if ( actionInfo->actualSensings.delta.forward == 0.0
	 && actionInfo->actualSensings.delta.sideward == 0.0
	 && actionInfo->actualSensings.delta.rotation == 0.0)
      actionInfo->actualSensings.noMoveCnt++;
    else {
      actionInfo->actualSensings.noMoveCnt = 0;
      
      /* Accumulate the relative movements for each plane of the prob grid. */
      if ( actionInfo->useProbGrid) {
	for ( plane = 0; plane < actionInfo->positionProbs.sizeZ; plane++) 
	  actionInfo->positionProbs.summedMovementOfPlane[plane] =
	    endPoint( actionInfo->positionProbs.summedMovementOfPlane[plane],
		      actionInfo->actualSensings.delta);
      }
      /* Condensation: accumulate the relative movement only once. */
      else {
	actionInfo->samples.summedMovement = 
	  endPoint( actionInfo->samples.summedMovement,
		    actionInfo->actualSensings.delta);
	
	actionInfo->tcxSamples.summedMovement = 
	  endPoint( actionInfo->tcxSamples.summedMovement,
		    actionInfo->actualSensings.delta);
	
	/* Update the sample set history. */
	updateSampleSetHistory( &(actionInfo->samplesHistory));
      }
	
      /* Update the summed movements for the actions. */
      deltaAbs = sqrt( fSqr( actionInfo->actualSensings.delta.forward)
		       + fSqr( actionInfo->actualSensings.delta.sideward));
      /* + fAbs( rad2Deg( actionInfo->actualSensings.delta.rotation));   */
      
      for ( sensing = 0; sensing < NUMBER_OF_SENSINGS; sensing++)
	for ( action = 0; action < mask->numberOfActions[sensing]; action++)
	  actionInfo->summedMovements[sensing][action] += deltaAbs;
    }
    
    /* Update the estimated position and the local maxima. */
    if ( actionInfo->useProbGrid) {
      for ( max = 0; max < actionInfo->localMaxima.numberOfCells; max++)
	actionInfo->localMaxima.cell[max].pos =
	  endPoint( actionInfo->localMaxima.cell[max].pos,
		    actionInfo->actualSensings.delta);
      
      removeCellsNotInGrid( &(actionInfo->localMaxima),
			    &(actionInfo->positionProbs));    
      
      if (actionInfo->localMaxima.numberOfCells > 0)
	actionInfo->estimatedRobot.pos = actionInfo->localMaxima.cell[0].pos;
      else
	actionInfo->estimatedRobot.pos =
	  endPoint( actionInfo->estimatedRobot.pos,
		    actionInfo->actualSensings.delta);
    }
    else
      actionInfo->estimatedRobot.pos =
	endPoint( actionInfo->estimatedRobot.pos,
		  actionInfo->actualSensings.delta);
  }
}

/*****************************************************************************
 * Proceed to the next position in the history ring. *
 *****************************************************************************/
static void
updateSampleSetHistory( sampleSetHistory* history)
{
  int s, sensing, action;
  actionInformation* actionInfo = (actionInformation*) history->actionInfo;
  sensingActionMask* actionMask = (sensingActionMask*) history->actionMask;
  float deltaAbs = sqrt( fSqr( actionInfo->actualSensings.delta.forward)
			 + fSqr( actionInfo->actualSensings.delta.sideward));

  /* Loop through the whole history.*/
  for ( s = 0; s < history->sizeOfHistory; s++) {

    /* Motion of the sample sets. */
    history->motionSinceSampleSet[s] =
      endPoint( history->motionSinceSampleSet[s],
		actionInfo->actualSensings.delta);

    /* Motion of the sensors/actions. */
    for ( sensing = 0; sensing < NUMBER_OF_SENSINGS; sensing++)
      for ( action = 0; action < actionMask->numberOfActions[sensing]; action++)
	history->summedMovement[s][sensing][action] += deltaAbs;
  }
}

/*****************************************************************************
 * Proceed to the next position in the history ring. *
 *****************************************************************************/
void
resetSampleSetToHistory( sampleSetHistory* history)
{
  int position;

  /* Set the pointers of the sample history. */
  history->currentSampleSetNumber = -1;
  history->currentSampleSetPosition = 0;

  for ( position = 0; position < history->sizeOfHistory; position++) {
    history->timeStamp[position].tv_sec = 0;
    history->timeStamp[position].tv_usec = 0;
  }
}


/*****************************************************************************
 * Proceed to the next position in the history ring. *
 *****************************************************************************/
int
addSampleSetToHistory( sampleSet* samples, sampleSetHistory* history,
		       int numberOfSamplesToBeSent)
{
  int sensing, action;
  int currentPosition, s;
  int numberOfSamples = samples->numberOfSamples;
  sampleType* dest;
  actionInformation* actionInfo = (actionInformation*) history->actionInfo;
  sensingActionMask* actionMask = (sensingActionMask*) history->actionMask;

  writeLog("%d %d #NORM\n", samples->alreadyNormalized, samples->alreadySampled);
  history->currentSampleSetNumber++;

  /* If the number of samples to be sent is smaller than zero then all
   * samples have to be sent. */
  if (numberOfSamplesToBeSent < 0 || numberOfSamplesToBeSent > numberOfSamples)
    numberOfSamplesToBeSent = numberOfSamples;
  
  history->minPosition.x   = samples->minX;
  history->minPosition.y   = samples->minY;
  history->minPosition.rot = -DEG_360;

  history->maxPosition.x   = samples->maxX;
  history->maxPosition.y   = samples->maxY;
  history->maxPosition.rot = DEG_360;
  
  history->currentSampleSetPosition =
    history->currentSampleSetNumber % history->sizeOfHistory;
  
  writeLog( "ADD %d %d (%f secs gap)\n", history->currentSampleSetNumber,
	    history->currentSampleSetPosition, timeDiff( &(history->timeStamp[(history->sizeOfHistory + history->currentSampleSetPosition - 1) % history->sizeOfHistory]), &(samples->timeStamp)));
	  
  currentPosition = history->currentSampleSetPosition;

  history->motionSinceSampleSet[currentPosition] = samples->summedMovement;

  /* Check whether we have to allocate new samples. */
  if ( history->numberOfAllocatedSamples[currentPosition] < numberOfSamplesToBeSent) {
    
    if ( history->numberOfAllocatedSamples[currentPosition] > 0) 
      free( (sampleType*) history->samples[currentPosition]);

    fprintf(stderr, "ALLOCATE %d samples\n", numberOfSamplesToBeSent);
    history->samples[currentPosition] = ( sampleType*) malloc( sizeof( sampleType) * numberOfSamplesToBeSent);
    history->numberOfAllocatedSamples[currentPosition] = numberOfSamplesToBeSent;
  }
  
  history->numberOfSamples[currentPosition]         = numberOfSamples;
  history->numberOfSamplesToBeSent[currentPosition] = numberOfSamplesToBeSent;
  history->timeStamp[currentPosition]               = samples->timeStamp;
  
  dest = history->samples[currentPosition];
    
  /* Copy the samples. */
  for ( s = 0; s < numberOfSamplesToBeSent; s++) 
    dest[s] = samples->sample[s];
    
  /* Motion of the sensors/actions. */
  for ( sensing = 0; sensing < NUMBER_OF_SENSINGS; sensing++)
    for ( action = 0; action < actionMask->numberOfActions[sensing]; action++) {
      history->summedMovement[history->currentSampleSetPosition][sensing][action] =
	actionInfo->summedMovements[sensing][action];
    }
  
  return history->currentSampleSetNumber;
}


/*****************************************************************************
 * Checks whether the sample set is not too old for replacement.
 *****************************************************************************/
bool
sampleSetUpToDate( int number, sampleSetHistory* history)
{
  if ( (history->currentSampleSetNumber - number) > MAX_DIST_IN_HISTORY) {
    writeLog( "# Sample set %d too old (%d).\n",
	      number, history->currentSampleSetNumber- number);
    return FALSE;
  }
  else {
    int positionOfNumber = history->currentSampleSetPosition
      - (history->currentSampleSetNumber - number);
    if ( positionOfNumber < 0)
      positionOfNumber += history->sizeOfHistory;
    
    if ( sqrt( fSqr( history->motionSinceSampleSet[positionOfNumber].x)
	       + fSqr( history->motionSinceSampleSet[positionOfNumber].y))
	 > MAX_DIST_TRAVELLED) {
      writeLog( "#Robot moved too far (%f) for replacement.\n",
		sqrt(fSqr(history->motionSinceSampleSet[positionOfNumber].x)
		     + fSqr(history->motionSinceSampleSet[positionOfNumber].y)));
      return FALSE;
    }
    else
      return TRUE;
  }
}


/*****************************************************************************
 * Replaces the motion information by the motion information stored in the sample
 * set history.
 *****************************************************************************/
void
setSampleSetHistory( LOCALIZE_updated_samples_ptr tcxSamples,
		     sampleSet* samples,
		     sampleSetHistory* history)
{
  int number = tcxSamples->numberOfSet;
  int positionOfNumber;
  int sensing, action, s, set;
  sampleType* dest;

  actionInformation* actionInfo = (actionInformation*) history->actionInfo;
  sensingActionMask* actionMask = (sensingActionMask*) history->actionMask;

  /* Look for the position in the history. */
  if ( history->currentSampleSetNumber < number) {
    fprintf( stderr, "ERROR: sample set number bigger than current sample set: %d %d\n",
	     history->currentSampleSetNumber, number);
    return;
  }
  else if ( (history->currentSampleSetNumber - number) > history->sizeOfHistory) {
    fprintf( stderr, "ERROR: sample too old %d %d. Resize SIZE_OF_SAMPLE_SET_HISTORY.\n",
	     history->currentSampleSetNumber, number);
    return;
  }

  fprintf( stderr, "Replace sample set %d by sample set %d.\n",
	   history->currentSampleSetNumber,
	   number);

  writeLog( "# Replace sample set %d by sample set %d at %f.\n",
	    history->currentSampleSetNumber, number, elapsedScriptTime);
  
  positionOfNumber = history->currentSampleSetPosition - (history->currentSampleSetNumber - number);
  if ( positionOfNumber < 0)
    positionOfNumber += history->sizeOfHistory;
  
  fprintf(stderr, "mot");
  writeLog( "# Motion: %f %f %f --> %f %f %f.\n",
	    samples->summedMovement.x, 
	    samples->summedMovement.y, 
	    rad2Deg(samples->summedMovement.rot), 
	    history->motionSinceSampleSet[positionOfNumber].x, 
	    history->motionSinceSampleSet[positionOfNumber].y, 
	    rad2Deg(history->motionSinceSampleSet[positionOfNumber].rot));
  
  /* Set the motion of the sample set. */
  samples->summedMovement = history->motionSinceSampleSet[positionOfNumber];
  samples->desiredNumberOfSamples = history->numberOfSamples[positionOfNumber];
  
  /* Copy the samples into the old sample set. */
  dest = history->samples[positionOfNumber];
  for ( s = 0; s < tcxSamples->numberOfSamples; s++) {
    dest[s].pos    = tcxSamples->replySamples[s].pos;
    dest[s].weight = tcxSamples->replySamples[s].weight;
  }
  
  /* Motion of the sensors/actions. */
  for ( sensing = 0; sensing < NUMBER_OF_SENSINGS; sensing++)
    for ( action = 0; action < actionMask->numberOfActions[sensing]; action++)
      actionInfo->summedMovements[sensing][action] =
	history->summedMovement[positionOfNumber][sensing][action];  
  
  /* Now set the history back to this replaced sample set. */
  if (0) {
    writeLog( "##I %d %d\n", number, history->currentSampleSetNumber);
    for ( set = number + 1; set <= history->currentSampleSetNumber; set++) {
      int currentPos = set % history->sizeOfHistory;
      writeLog( "##R %d %d\n", set, currentPos);
      if ( history->samples[currentPos] != NULL) {
	history->timeStamp[currentPos].tv_usec = 0;
	history->timeStamp[currentPos].tv_sec = 0;
      }
    }
  }
  
  history->currentSampleSetNumber = number;
  history->currentSampleSetPosition =
    history->currentSampleSetNumber % history->sizeOfHistory;
  
  fprintf(stderr, "replacement done");
  return;
}


/*****************************************************************************
 * Gets the sample set that is closest to the timeStamp.
 *****************************************************************************/
  int
getClosestSampleSet( sampleSetHistory* history, struct timeval timeStamp)
{
  if ( ! (timeStamp.tv_sec == 0 && timeStamp.tv_usec == 0)) {
    
    bool cycleClosed = FALSE;
    int historySize = history->sizeOfHistory;
    int stopMarker = (history->currentSampleSetPosition + 1) % historySize;
    int startPosition = stopMarker;
    
    /* This is the position of the oldest sample set */
    int currentPos = startPosition;
    
    while ( ! cycleClosed) {
      
      if ( history->samples[currentPos] != NULL) {
	
	float tDiff = timeDiff( & (history->timeStamp[currentPos]),
				& timeStamp);
	writeLog("%d --> %f\n", currentPos, tDiff);    
	
      }
      currentPos = (currentPos + 1) % historySize;
      cycleClosed = currentPos == stopMarker;
    }

    currentPos = startPosition;
    cycleClosed = FALSE;
    while ( ! cycleClosed) {
      
      if ( history->samples[currentPos] != NULL) {
	
	float tDiff = timeDiff( & (history->timeStamp[currentPos]),
				& timeStamp);
	writeLog("%d --> %f\n", currentPos, tDiff);    
	
	/* Found a newer sample set. */
	if ( tDiff > 0.0) {

	  /* If this is the oldes set then we cannot find a better one. */
	  if ( currentPos == startPosition)
	    return currentPos;

	  else {
	    float tDiffOfPrevious =
	      timeDiff( & (history->timeStamp[(historySize + currentPos - 1) % historySize]),
			& timeStamp);
	    
	    /* Is it close enough? */
	    if ( fAbs( tDiffOfPrevious) < 1.0 ||
		 ( tDiff > - tDiffOfPrevious))
	      /* We are interested in the previous one. */
	      return (historySize + currentPos - 1) % historySize;
	    else 
	      /* Return this one if it is closer. */
	      return (currentPos) % historySize;
	  }
	}
      }
      currentPos = (currentPos + 1) % historySize;
      cycleClosed = currentPos == stopMarker;
    }
  }
  
  if ( history->samples[history->currentSampleSetPosition] != NULL)
    return history->currentSampleSetPosition;
  else
    return -1;
}









