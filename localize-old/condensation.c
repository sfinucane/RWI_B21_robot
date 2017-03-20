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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/condensation.c,v $
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
 * $Log: condensation.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.63  2000/08/18 15:37:58  wolfram
 * Nothing special
 *
 * Revision 1.62  2000/03/06 20:00:42  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.61  2000/02/07 18:36:06  wolfram
 * Changed localTcx.c to maintain the robot position even if colliServer stops.
 * Fixed a memory leak.
 *
 * Revision 1.60  2000/01/26 22:56:42  fox
 * Might work. If anything weird happens, contact me.
 *
 * Revision 1.58  2000/01/11 09:45:54  fox
 * *** empty log message ***
 *
 * Revision 1.57  2000/01/10 19:11:52  fox
 * *** empty log message ***
 *
 * Revision 1.56  2000/01/10 19:04:20  fox
 * DON'T USE!
 *
 * Revision 1.55  2000/01/02 15:33:14  fox
 * Should work.
 *
 * Revision 1.54  1999/12/29 16:41:10  fox
 * Version before I merge generated scans with script scans.
 *
 * Revision 1.53  1999/12/29 15:07:19  fox
 * Version before I remove front and rear positions.
 *
 * Revision 1.52  1999/12/28 16:54:44  fox
 * Intermediate version. Might not work.
 *
 * Revision 1.51  1999/12/27 09:52:51  fox
 * *** empty log message ***
 *
 * Revision 1.50  1999/12/20 08:45:54  fox
 * Implementation of precomputation of p(x|o).
 *
 * Revision 1.49  1999/12/18 13:14:28  fox
 * Nothing special.
 *
 * Revision 1.48  1999/12/17 18:26:23  fox
 * Second version of precompuations.
 *
 * Revision 1.47  1999/12/10 15:20:16  wolfram
 * Improved version for on-line computation of expected distances
 *
 * Revision 1.46  1999/11/15 13:28:03  fox
 * *** empty log message ***
 *
 * Revision 1.45  1999/11/02 18:12:32  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.44  1999/10/21 17:30:42  fox
 * Fixed some bugs in real time scripts.
 *
 * Revision 1.43  1999/09/29 16:06:09  fox
 * Should work.
 *
 * Revision 1.42  1999/09/26 21:20:56  fox
 * Nothing special.
 *
 * Revision 1.41  1999/09/26 18:55:21  fox
 * Added scout robot.
 *
 * Revision 1.40  1999/09/09 02:48:37  fox
 * Final version before germany.
 *
 * Revision 1.39  1999/09/06 16:36:03  fox
 * Many changes.
 *
 * Revision 1.38  1999/09/03 22:22:39  fox
 * Changed hadnling of real time script. This version contains both.
 *
 * Revision 1.37  1999/09/03 13:43:35  fox
 * Nothing special.
 *
 * Revision 1.36  1999/09/01 21:26:54  fox
 * Works almost perfectly :-)
 *
 * Revision 1.35  1999/09/01 00:02:56  fox
 * Getting closer. But still not close enough.
 *
 * Revision 1.34  1999/08/30 05:48:41  fox
 * Doesn't work!!
 *
 * Revision 1.33  1999/08/27 22:22:32  fox
 * Better communication for multi robot localization.
 *
 * Revision 1.32  1999/07/13 23:08:01  fox
 * Some changes.
 *
 * Revision 1.31  1999/07/12 15:24:52  fox
 * Minor changes in Texas.
 *
 * Revision 1.30  1999/06/25 19:48:11  fox
 * Minor changs for the urbie.
 *
 * Revision 1.29  1999/06/24 00:21:49  fox
 * Some changes for the urbies.
 *
 * Revision 1.28  1999/05/18 15:15:19  fox
 * Added keywords to determine whether correction parameters should be sent to map and plan or not.
 *
 * Revision 1.27  1999/04/29 13:35:20  fox
 * Further adaptation to make multi localize run.
 *
 * Revision 1.26  1999/04/29 00:58:28  fox
 * Some minor changes for multi localize.
 *
 * Revision 1.25  1999/04/21 14:05:59  fox
 * Just an intermediate version.
 *
 * Revision 1.24  1999/04/18 19:00:09  fox
 * This is just to get the most recent version into the cmu bee tree. It shouldn't do any harm (hopefully!).
 *
 * Revision 1.23  1999/03/16 16:11:15  fox
 * Added more information to sample set message and created a new subscription
 * message for status, maps, robot position, and samples (maps and robot
 * positions not implemented yet).
 *
 * Revision 1.22  1999/03/12 00:41:48  fox
 * Minor changes.
 *
 * Revision 1.21  1999/02/17 19:42:22  fox
 * Enhanced gif utilities.
 *
 * Revision 1.20  1999/02/05 23:02:42  fox
 * Minor changes for samples.
 *
 * Revision 1.19  1999/02/01 21:52:22  fox
 * Added support for dumping gif files.
 *
 * Revision 1.18  1999/01/22 18:10:39  fox
 * Removed some hacks done for denver and the sampling paper.
 *
 * Revision 1.17  1999/01/22 17:48:01  fox
 * Final version of sampling paper and denver demo.
 *
 * Revision 1.16  1999/01/22 00:34:56  wolfram
 * Added database support in vision.c
 *
 * Revision 1.15  1999/01/14 23:39:29  wolfram
 * Added sensorMaxRange and standardDeviaton to the sensor model for lasers
 *
 * Revision 1.14  1999/01/11 19:47:47  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.12  1999/01/08 22:28:43  wolfram
 * Better integration of scanAlignment
 *
 * Revision 1.11  1998/11/24 23:05:25  fox
 * Implemented furhter routines for condensation and vision.
 *
 * Revision 1.10  1998/11/24 18:42:10  fox
 * First version of condensation with vision.
 *
 * Revision 1.9  1998/11/19 03:14:24  fox
 * Newest version with integrated mapping and localization. If MAP is connected,
 * then the correction parameters are NOT sent to PLAN.
 * Furthermore, the size of the onlinemap adapts to the partial maps sent by MAP.
 *
 * Revision 1.8  1998/11/17 23:26:17  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.7  1998/11/03 21:02:16  fox
 * Lasers will not be integrated if the robot rotates faster than
 * MAX_ROT_VEL_FOR_LASERS deg/sec.
 *
 * Revision 1.6  1998/10/29 03:44:59  fox
 * Nothing special.
 *
 * Revision 1.5  1998/10/19 18:29:54  fox
 * *** empty log message ***
 *
 * Revision 1.4  1998/10/02 15:16:36  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.3  1998/09/25 17:53:29  fox
 * Improved version of condensation.
 *
 * Revision 1.2  1998/09/25 04:02:53  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.1  1998/09/18 17:24:42  fox
 * Added skeleton files for condensation.
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/



#include <math.h>

#include "general.h"
#include "probGrid.h"
#include "function.h"
#include "file.h"
#include "sonar.h"
#include "laser.h"
#include "movement.h"
#include "angle.h"
#include "graphic.h"
#include "allocate.h"
#include "script.h"
#include "localTcx.h"
#include "condensation.h"
#include "proximityTools.h"
#include "communication.h"
#include "scanAlignment.h"

#define NUMBER_OF_SAMPLES_TOKEN              0
#define START_X_TOKEN                        1
#define START_Y_TOKEN                        2
#define START_ROT_TOKEN                      3
#define SET_START_POS_TOKEN                  4
#define SAMPLING_DISTANCE_NOISE_TOKEN        5
#define SAMPLING_ANGLE_NOISE_TOKEN           6
#define DUMP_SAMPLES_TOKEN                   7
#define USE_POSITION_TOKEN                   8
#define VARIABLE_SAMPLE_SIZE_TOKEN           9
#define MIN_NUMBER_OF_SAMPLES_TOKEN         10
#define SAMPLE_INTEGRATION_THRESHOLD_TOKEN  11
#define FRACTION_OF_UNIFORM_SAMPLES_TOKEN   12
#define LOAD_SAMPLES_FILE_TOKEN             13
#define SAMPLES_FILE_TOKEN                  14
#define SAMPLING_SIDE_DRIFT_TOKEN           15
#define SAMPLING_ROT_DRIFT_TOKEN            16
#define MULTI_LOCALIZE_TOKEN                17
#define RESET_SAMPLE_PROBABILITY_TOKEN      18

condensationParameters globalCondensationParameters;
extern informationsFor_COMMUNICATION communicationInfo;
int multiLocalize = FALSE;

#define NUMBER_KEY_WORD "#Number "
#define DIMENSION_KEY_WORD "#Dimension "
#define MIN_KEY_WORD "#Min "
#define MAX_KEY_WORD "#Max "
#define REF_POS_KEY_WORD "#RefPosition "
#define TIME_STAMP_KEY_WORD "#Time "
#define NUMBER_INTEGRATIONS_KEY_WORD "#Integrations "

/********************************************************************
 ********************************************************************
 * Forward declarations of internal functions.
 ********************************************************************
 ********************************************************************/


static sampleSet
readSamples( char* fileName);

static int
binarySearch( double* values, double searchVal, int numberOfValues);

static void
sampleAccordingToDistribution( sampleSet* samples, probabilityGrid* aPrioriProbs);

static void
lowVarianceResample( sampleSet* samples, probabilityGrid* aPrioriProbs);

static void
updateSampleInformation( actionInformation* info);

static void
dumpSamples( sampleSet* samples, char* robotName,
	     int numberOfSamples, realPosition refPos);

static void
checkAllocationOfAuxiliaryVariables( sampleSet* samples);

static float
normalizeWeights( sampleSet* samples);

static void
precomputePOfXGivenO( probabilityGrid* map,
		      probabilityGrid* aPrioriProbs,
		      expectedDistTable *distTab,
		      probability** distProbTable,
		      sensing_PROXIMITY* measuredFrontScan,
		      sensing_PROXIMITY* measuredRearScan);

/**************************************************************************
 **************************************************************************
 * Mandatory functions.
 **************************************************************************
 **************************************************************************/

void
initialize_CONDENSATION( char* fileName,
			 actionInformation* actionInfo,
			 sensingActionMask* actionMask, int argc, char** argv)
{
  realPosition startPos;
  int setStartPos = 0, z;

  token tok[NUMBER_OF_CONDENSATION_PARAMETERS];
  
  /**********************************************************
   * Initialize parameters
   **********************************************************/
  int numberOfSamples = 400;
  float integrateThreshold = 0.3;
  int variableSampleSize = FALSE;
  int minNumberOfSamples = numberOfSamples;
  float fractionOfUniformSamples = 0.0;
  char samplesFile[MAX_STRING_LENGTH];
  int loadSamplesFile = FALSE;
  globalCondensationParameters.distanceNoise = 0.05;
  globalCondensationParameters.angleNoise = 0.02;
  globalCondensationParameters.sideDrift = 0.02;
  globalCondensationParameters.rotDrift = 0.02;
  globalCondensationParameters.dumpSamples = FALSE;
  globalCondensationParameters.usePosition = TRUE;
  globalCondensationParameters.resetProbability = 0.0;
  globalCondensationParameters.samplesHistory = &(actionInfo->samplesHistory);
  
  actionInfo->proximityIntegrated = FALSE;

  setTokensInitialized( tok, NUMBER_OF_CONDENSATION_PARAMETERS);
  
  /**********************************************************
   * Get the parameters from the file.
   **********************************************************/
    
  tok[NUMBER_OF_SAMPLES_TOKEN].format   = INT_FORMAT;
  tok[NUMBER_OF_SAMPLES_TOKEN].variable = &numberOfSamples;
  tok[NUMBER_OF_SAMPLES_TOKEN].keyWord  = NUMBER_OF_SAMPLES_KEYWORD;
  
  tok[SET_START_POS_TOKEN].format   = INT_FORMAT;
  tok[SET_START_POS_TOKEN].variable = &(setStartPos);
  tok[SET_START_POS_TOKEN].keyWord  = SET_START_POS_KEYWORD;
  
  tok[USE_POSITION_TOKEN].format   = INT_FORMAT;
  tok[USE_POSITION_TOKEN].variable = &(globalCondensationParameters.usePosition);
  tok[USE_POSITION_TOKEN].keyWord  = USE_POSITION_KEYWORD;
  
  tok[START_X_TOKEN].format   = FLOAT_FORMAT;
  tok[START_X_TOKEN].variable = &(startPos.x);
  tok[START_X_TOKEN].keyWord  = START_X_KEYWORD;
  
  tok[START_Y_TOKEN].format   = FLOAT_FORMAT;
  tok[START_Y_TOKEN].variable = &(startPos.y);
  tok[START_Y_TOKEN].keyWord  = START_Y_KEYWORD;
  
  tok[START_ROT_TOKEN].format   = FLOAT_FORMAT;
  tok[START_ROT_TOKEN].variable = &(startPos.rot);
  tok[START_ROT_TOKEN].keyWord  = START_ROT_KEYWORD;
  
  tok[SAMPLING_DISTANCE_NOISE_TOKEN].format   = FLOAT_FORMAT;
  tok[SAMPLING_DISTANCE_NOISE_TOKEN].variable =
    &(globalCondensationParameters.distanceNoise);
  tok[SAMPLING_DISTANCE_NOISE_TOKEN].keyWord  = SAMPLING_DISTANCE_NOISE_KEYWORD;
  
  tok[SAMPLING_ANGLE_NOISE_TOKEN].format   = FLOAT_FORMAT;
  tok[SAMPLING_ANGLE_NOISE_TOKEN].variable =
    &(globalCondensationParameters.angleNoise);
  tok[SAMPLING_ANGLE_NOISE_TOKEN].keyWord  = SAMPLING_ANGLE_NOISE_KEYWORD;
  
  tok[SAMPLING_SIDE_DRIFT_TOKEN].format   = FLOAT_FORMAT;
  tok[SAMPLING_SIDE_DRIFT_TOKEN].variable =
    &(globalCondensationParameters.sideDrift);
  tok[SAMPLING_SIDE_DRIFT_TOKEN].keyWord  = SAMPLING_SIDE_DRIFT_KEYWORD;
  
  tok[SAMPLING_ROT_DRIFT_TOKEN].format   = FLOAT_FORMAT;
  tok[SAMPLING_ROT_DRIFT_TOKEN].variable =
    &(globalCondensationParameters.rotDrift);
  tok[SAMPLING_ROT_DRIFT_TOKEN].keyWord  = SAMPLING_ROT_DRIFT_KEYWORD;
  
  tok[DUMP_SAMPLES_TOKEN].format   = INT_FORMAT;
  tok[DUMP_SAMPLES_TOKEN].variable = &(globalCondensationParameters.dumpSamples);
  tok[DUMP_SAMPLES_TOKEN].keyWord  = DUMP_SAMPLES_KEYWORD;
  
  tok[VARIABLE_SAMPLE_SIZE_TOKEN].format   = INT_FORMAT;
  tok[VARIABLE_SAMPLE_SIZE_TOKEN].variable = &variableSampleSize;
  tok[VARIABLE_SAMPLE_SIZE_TOKEN].keyWord  = VARIABLE_SAMPLE_SIZE_KEYWORD;
  
  tok[MIN_NUMBER_OF_SAMPLES_TOKEN].format   = INT_FORMAT;
  tok[MIN_NUMBER_OF_SAMPLES_TOKEN].variable = &minNumberOfSamples;
  tok[MIN_NUMBER_OF_SAMPLES_TOKEN].keyWord  = MIN_NUMBER_OF_SAMPLES_KEYWORD;
  
  tok[SAMPLE_INTEGRATION_THRESHOLD_TOKEN].format   = FLOAT_FORMAT;
  tok[SAMPLE_INTEGRATION_THRESHOLD_TOKEN].variable = &integrateThreshold;
  tok[SAMPLE_INTEGRATION_THRESHOLD_TOKEN].keyWord  = SAMPLE_INTEGRATION_THRESHOLD_KEYWORD;

  tok[FRACTION_OF_UNIFORM_SAMPLES_TOKEN].format   = FLOAT_FORMAT;
  tok[FRACTION_OF_UNIFORM_SAMPLES_TOKEN].variable = &fractionOfUniformSamples;
  tok[FRACTION_OF_UNIFORM_SAMPLES_TOKEN].keyWord  = FRACTION_OF_UNIFORM_SAMPLES_KEYWORD;

  tok[SAMPLES_FILE_TOKEN].format   = STRING_FORMAT;
  tok[SAMPLES_FILE_TOKEN].variable = samplesFile;
  tok[SAMPLES_FILE_TOKEN].keyWord  = SAMPLES_FILE_KEYWORD;
  
  tok[LOAD_SAMPLES_FILE_TOKEN].format   = INT_FORMAT;
  tok[LOAD_SAMPLES_FILE_TOKEN].variable = &loadSamplesFile;
  tok[LOAD_SAMPLES_FILE_TOKEN].keyWord  = LOAD_SAMPLES_FILE_KEYWORD;
  
  tok[MULTI_LOCALIZE_TOKEN].format   = INT_FORMAT;
  tok[MULTI_LOCALIZE_TOKEN].variable = &multiLocalize;
  tok[MULTI_LOCALIZE_TOKEN].keyWord  = MULTI_LOCALIZE_KEYWORD;
  
  tok[RESET_SAMPLE_PROBABILITY_TOKEN].format   = FLOAT_FORMAT;
  tok[RESET_SAMPLE_PROBABILITY_TOKEN].variable = &globalCondensationParameters.resetProbability;
  tok[RESET_SAMPLE_PROBABILITY_TOKEN].keyWord  = RESET_SAMPLE_PROBABILITY_KEYWORD;
  
  readTokens( fileName, tok, NUMBER_OF_CONDENSATION_PARAMETERS, FALSE);

  if ( globalCondensationParameters.resetProbability > 0.0)
    writeLog( "#Reset sample set with probability %f.\n",
	      globalCondensationParameters.resetProbability);
  {
    if ( argc == 7) {
      fprintf(stderr, "WARNING: USE ALL SEVEN ARGUMENTS TO SET DEFAULT VALUES.\n");
      setStartPos = atoi(argv[2]);
      variableSampleSize = atoi(argv[3]);
      minNumberOfSamples = atoi(argv[4]);
      numberOfSamples    = atoi(argv[5]);
      integrateThreshold = atof(argv[6]);
    }
  }
  

  /**********************************************************
   * done.
   **********************************************************/

  if ( ! actionInfo->useProbGrid) {

    /* Set some default values for the prob grid. */
#define NUMBER_OF_DUMMY_PLANES 360
    actionInfo->positionProbs.angleResolution = DEG_360 / NUMBER_OF_DUMMY_PLANES;
    actionInfo->positionProbs.sizeX = 500;
    actionInfo->positionProbs.sizeY = 500;
    actionInfo->positionProbs.sizeZ = NUMBER_OF_DUMMY_PLANES;

    actionInfo->positionProbs.summedMovementOfPlane = (realPosition*)
      malloc( NUMBER_OF_DUMMY_PLANES * sizeof(realPosition));
    for ( z = 0; z < NUMBER_OF_DUMMY_PLANES; z++)
      actionInfo->positionProbs.summedMovementOfPlane[z].rot = deg2Rad(z);

    /* If online mapping and the map is initialized (read from a file) then
     * we can also set the position prob map. */
    if ( actionInfo->onlineMapping && actionInfo->onlineMap.initialized) {
      fprintf( stderr, "# Initialize a priori position probs with online map.\n");
      writeLog( "# Initialize a priori position probs with online map.\n");
      actionInfo->initialPositionProbs = invertedMap( &(actionInfo->onlineMap));
      setFreeSpace( &(actionInfo->initialPositionProbs));
    }

    if ( ! loadSamplesFile) {
      if ( actionMask->use[VISION]) {
	actionInfo->samples = 
	  initializedSamples( numberOfSamples,
			     0.0, 0.0,
			     actionInfo->visMap.maxRealX,
			     actionInfo->visMap.maxRealY);
      }
      else 
	actionInfo->samples = 
	  initializedSamples( numberOfSamples, 
			     0.0, 0.0, 
			     actionInfo->map.maxRealX, 
			     actionInfo->map.maxRealY);
    }
    else
      actionInfo->samples = readSamples( samplesFile);
      

    actionInfo->tcxSamples.allocatedSamples = 0;
    actionInfo->tcxSamples.sample = NULL;


    actionInfo->samples.integrateThreshold = integrateThreshold;
    actionInfo->samples.variableSampleSize = variableSampleSize;
    if ( actionInfo->samples.variableSampleSize)
      actionInfo->samples.minNumberOfSamples = minNumberOfSamples;
    else
      actionInfo->samples.minNumberOfSamples = actionInfo->samples.numberOfSamples;
    actionInfo->samples.fractionOfUniformSamples = fractionOfUniformSamples;

/*      gettimeofday( &(actionInfo->samples.timeStamp), NULL); */
/*      gettimeofday( &(actionInfo->samples.timeOfLastShift), NULL); */

    if ( ! loadSamplesFile)
      setUniformDistribution( &(actionInfo->samples), 
			    &(actionInfo->initialPositionProbs));
			    
    writeLog( "# Use [%d %d] samples with %.0f%% uniform samples.\n",
	      actionInfo->samples.minNumberOfSamples,
	      numberOfSamples,
	      actionInfo->samples.fractionOfUniformSamples * 100.0);

    writeLog( "# Dist noise %f, angle noise %f.\n",
	      globalCondensationParameters.distanceNoise,
	      globalCondensationParameters.angleNoise);

    writeLog( "# Side driftd %f, rot drift %f.\n",
	      globalCondensationParameters.sideDrift,
	      globalCondensationParameters.rotDrift);

    fprintf( stderr, "# Use [%d %d] samples with %.0f%% uniform samples.\n",
	      actionInfo->samples.minNumberOfSamples,
	      numberOfSamples,
	      actionInfo->samples.fractionOfUniformSamples * 100.0);

    fprintf( stderr, "# Dist noise %f, angle noise %f.\n",
	      globalCondensationParameters.distanceNoise,
	      globalCondensationParameters.angleNoise);

    fprintf( stderr, "# Side driftd %f, rot drift %f.\n",
	      globalCondensationParameters.sideDrift,
	      globalCondensationParameters.rotDrift);

    if ( setStartPos) {
      fprintf(stderr, "# Sample start position: %f %f %f\n",
	      startPos.x, startPos.y, startPos.rot);
      writeLog("# Sample start position: %f %f %f\n",
	       startPos.x, startPos.y, startPos.rot);
      startPos.rot = deg2Rad( startPos.rot);
      setSamplePosition( startPos, &(actionInfo->samples), 15.0);
    }

  }

}

/*****************************************************************************
 * Checks what has to be done with the grid independently from what kind of
 * sensings have been integrated.
 *****************************************************************************/
void
checkWhichActionsToPerform_CONDENSATION( actionInformation* info,
					 sensingActionMask* mask)
{
  mask->sampleDistribution = FALSE;

  /* #define DDD */
#ifdef DDD

  /*#define GENERATE_SCANS 1 */
#define GENERATE_SCANS 0
  
  /* Now precompute the sample sets for p(x | 0). */
  if ( GENERATE_SCANS ||
       (mask->use[LASER] && info->actualSensings.frontLaser.isNew)) {
    
    informationsFor_LASER* laserInfo = (informationsFor_LASER*) info->info[LASER];
    informationsFor_PROXIMITY* proximityInfo = &(laserInfo->general);
    probability** distProbTab = proximityInfo->distProbFunctionTableDividedByPOfFeature.prob;
    /* probability** distProbTab = proximityInfo->distProbFunctionTableNormed.prob; */
    expectedDistTable* expDistTab = &(proximityInfo->expectedDistances);
    
    static float previousDistance = 0.0;

    if ( GENERATE_SCANS || 
	 1 || fabs( previousDistance - info->actualSensings.distanceTraveled) > 0.1) {
      
      precomputePOfXGivenO( &(info->map), 
			    &(info->initialPositionProbs), 
			    expDistTab, distProbTab,  
			    &(info->actualSensings.frontLaser),  
			    &(info->actualSensings.rearLaser)); 

      previousDistance = info->actualSensings.distanceTraveled;
    }
  }
  return;
#else
#define GENERATE_SCANS 0
#endif
  
  if ( ! info->useProbGrid) {

    info->samples.replacedViaTcx = FALSE;
    
    if ( mask->consider[LASER] && mask->perform[LASER][INTEGRATE_LASER]) {
      mask->sampleDistribution = TRUE;
    }
    if ( mask->consider[SONAR] && mask->perform[SONAR][INTEGRATE_SONAR]) {
      mask->sampleDistribution = TRUE;
    }
    if ( mask->consider[MOVEMENT] && mask->perform[MOVEMENT][INTEGRATE_MOVEMENT]) {
      mask->sampleDistribution = TRUE;
    }
  }
}


/*****************************************************************************
 * This is mostly normalizing.
 *****************************************************************************/
void
performActions_CONDENSATION( actionInformation* info, sensingActionMask* mask)
{
  if ( ! info->useProbGrid) {

    if ( mask->sampleDistribution) {

      info->samples.desiredNumberOfSamples =
	info->samples.numberOfSamples;
      
      if ( ! info->samples.replacedViaTcx) {

	updateSampleInformation( info);
	
	computeCorrectionParam( info->actualSensings.basePosition,
				info->estimatedRobot.pos,
				&(info->correctionParam));
	
	writeLog( "%.2f %f %f %f %d #corr\n",
		  elapsedScriptTime,
		  info->correctionParam.x,
		  info->correctionParam.y,
		  info->correctionParam.rot,
		  info->correctionParam.type);
	
	writeLog( "%d #samples\n", info->samples.numberOfSamples);
	
	info->samples.alreadySampled = 3;
	
	if (1) displaySamples();
	info->samples.alreadySampled = FALSE;
	
	sampleAccordingToDistribution( &(info->samples), &(info->initialPositionProbs));
	
	/* Integrate the sample set into the history. */
	if (multiLocalize)
	  info->samples.numberOfSet = addSampleSetToHistory( &(info->samples),
							     &(info->samplesHistory),
							     MAX_NUMBER_OF_TCX_SAMPLES);
      }
      if ( globalCondensationParameters.dumpSamples != 0) 
	dumpSamples( &(info->samples), info->robotName,
		     globalCondensationParameters.dumpSamples,
		     info->measuredRobotPosition);
      
      /* Copy the sample set. */
      copySamples( &(info->samples), &(info->tcxSamples));
    }
  }
}


/********************************************************************
 ********************************************************************
 * Global functions.
 ********************************************************************
 ********************************************************************/

/* Shifts the samples according to the motion and to their probabilities. */
void
shiftSamples( sampleSet* samples, probabilityGrid* aPrioriProbs)
{
  int s;
  float distanceDeviation = globalCondensationParameters.distanceNoise;
  float angleDeviation = globalCondensationParameters.angleNoise;
  float mapResolution = aPrioriProbs->resolution;
  float sideDrift, rotDrift;
  polarMovement movementSinceLastShift;

  if ( samples->summedMovement.x == 0.0 &&
       samples->summedMovement.y == 0.0 &&
       samples->summedMovement.rot == 0.0) {
    writeLog( "No motion.\n");
    return;
  }
  
  /* Motion of the robot since last integration of probabilities. */
  movementSinceLastShift.distance =
    sqrt( fSqr(samples->summedMovement.x) + fSqr(samples->summedMovement.y));
  
  movementSinceLastShift.angle = (float) atan2( samples->summedMovement.y,
						samples->summedMovement.x);

  movementSinceLastShift.rotation = normalizedAngle(samples->summedMovement.rot);
  if ( movementSinceLastShift.rotation > DEG_180)
    movementSinceLastShift.rotation -= DEG_360;
  
  writeLog( "\n%f %f %f #mot\n", movementSinceLastShift.distance,
	    rad2Deg(movementSinceLastShift.angle),
	    rad2Deg(movementSinceLastShift.rotation));

  sideDrift = deg2Rad( movementSinceLastShift.distance * globalCondensationParameters.sideDrift);
  rotDrift  = deg2Rad( movementSinceLastShift.distance * globalCondensationParameters.rotDrift);

  /* Sometimes we want to reset the samples with probability resetProbability
   * after each meter of motion. */
  if ( globalCondensationParameters.resetProbability > 0.0) {
    
    float probOfReset = globalCondensationParameters.resetProbability
      * movementSinceLastShift.distance * 0.01;
    
    if ( RAND_ZERO_TO_ONE() < probOfReset) {
      writeLog( "%f #Reset sample set.\n", elapsedScriptTime);
      
      setUniformDistribution( samples, aPrioriProbs);

      /* Reset the movement. */
      samples->summedMovement.x = 0.0;
      samples->summedMovement.y = 0.0;
      samples->summedMovement.rot = 0.0;
      samples->alreadySampled = 2;
      samples->alreadyNormalized = TRUE;
      samples->timeStamp = samples->timeOfLastShift;

      resetSampleSetToHistory( globalCondensationParameters.samplesHistory);
      return;
    }
  }
  
  /* Consider the probability of the samples. */
  sampleAccordingToDistribution( samples, aPrioriProbs);
  
  /* Add some noise to the motion invormation and shift the samples accordingly. */
  for ( s = 0; s < samples->numberOfSamples; s++) {

    polarMovement noisyMove = movementSinceLastShift;
    
    noisyMove.distance *= 1.0 + distanceDeviation * randomGauss();
    noisyMove.angle    *= 1.0 + angleDeviation * randomGauss();
    noisyMove.angle    += sideDrift * randomGauss();
    noisyMove.rotation *= 1.0 + angleDeviation * randomGauss();
    noisyMove.rotation += rotDrift * randomGauss();

    samples->sample[s].pos = endPointOfPolar( samples->sample[s].pos,
					      noisyMove);
  }

  for ( s = 0; s < samples->numberOfSamples; s++) {

    /* Set the weight of all samples outside the map to zero. */
    if ((samples->sample[s].pos.x >= samples->maxX ||
	 samples->sample[s].pos.y >= samples->maxY)
	|| ( samples->sample[s].pos.x < samples->minX ||
	     samples->sample[s].pos.y < samples->minY))
      samples->sample[s].weight = 0.0;
    
    /* Consider the occupancy information. */
    else if ( globalCondensationParameters.usePosition && aPrioriProbs->initialized) {
      int x = samples->sample[s].pos.x / mapResolution;
      int y = samples->sample[s].pos.y / mapResolution;
      
      samples->sample[s].weight *= (double) aPrioriProbs->prob[x][y]; 
    }
  }

  /* Reset the movement. */
  samples->summedMovement.x = 0.0;
  samples->summedMovement.y = 0.0;
  samples->summedMovement.rot = 0.0;
  samples->alreadySampled = 2;
  samples->alreadyNormalized = TRUE;
  samples->timeStamp = samples->timeOfLastShift;
}



/*****************************************************************************
 * Initializes the samples with a Gaussian distribution around the center.
 *****************************************************************************/
void
setSamplePosition( realPosition center,
		   sampleSet* samples,
		   float deviation)
{
  float rotDeviation = deg2Rad( deviation) / 10.0;
  int s;

  /* Set all values to min probability. */
  for ( s = 0; s < samples->numberOfSamples; s++) {
    samples->sample[s].pos = center;

    /* x-deviation */
    /* sebastian: added factor 3.0 for the initial x-y noise */
    samples->sample[s].pos.x   += deviation    * 3.0 * randomGauss();
    samples->sample[s].pos.y   += deviation    * 3.0 * randomGauss();
    samples->sample[s].pos.rot += rotDeviation * randomGauss();
  }
}

/*****************************************************************************
 * Computes the mean of the samples by using their probabilities as weights.
 *****************************************************************************/
void
setGlobalSampleMean( sampleSet* samples)
{
  int s;
  double probSum = 0.0, distSum = 0.0;
  float avgSin = 0.0, avgCos = 0.0;
  double maxProb = 0.0;
  int maxSample = 0;
  
  samples->mean.x = 0.0;
  samples->mean.y = 0.0;
  samples->mean.rot = 0.0;

  for ( s = 0; s < samples->numberOfSamples; s++) {
    double prob = samples->sample[s].weight;

    probSum += prob;
    samples->mean.x += (float) prob * samples->sample[s].pos.x;
    samples->mean.y += (float) prob * samples->sample[s].pos.y;
    
    /* The angle is a bit more complicated. */
    avgSin += (float) prob * sin( samples->sample[s].pos.rot);
    avgCos += (float) prob * cos( samples->sample[s].pos.rot);
        
    if ( prob > maxProb) {
      maxProb = prob;
      maxSample = s;
    }
  }

  if ( probSum > 0.0) {
    samples->mean.rot = atan2( avgSin, avgCos);
    samples->mean.x /= (float) probSum;
    samples->mean.y /= (float) probSum;
  }
  else {
    samples->mean.rot = 0.0;
    samples->mean.x /= 0.0;
    samples->mean.y /= 0.0;
  }
    
  
  /* Compute the standard deviation of the samples. */
  for ( s = 0; s < samples->numberOfSamples; s++) {
    distSum += (float) samples->sample[s].weight *
      fSqr( distanceBetweenPoints( samples->mean, samples->sample[s].pos));
  }

  if ( probSum > 0.0)
    distSum /= (float) probSum;
  else
    distSum = 0.0;

  samples->stdDev = sqrt( distSum);

  /* If the std deviation is too large, we don't use the mean of the positoins
   * but the most likely position. */
#define STDDEV_THRESHOLD 100.0
  
  if ( samples->stdDev > STDDEV_THRESHOLD) {
    samples->mean = samples->sample[maxSample].pos;
    
    writeLog( "%f %f %f %f %f %f %d #meanMostLikely\n", elapsedScriptTime,
              samples->mean.x, samples->mean.y, rad2Deg(samples->mean.rot),
              samples->stdDev, probSum, samples->numberOfSamples);
  }
  else
    writeLog( "%f %f %f %f %f %f %d #mean\n", elapsedScriptTime,
              samples->mean.x, samples->mean.y, rad2Deg(samples->mean.rot),
              samples->stdDev, probSum, samples->numberOfSamples);
}




void
setLocalSampleMean( sampleSet* samples)
{
  static int firstTime = TRUE;
  static float** maxWeight = NULL;
  static realPosition** maxPos = NULL;
  static float** sumOfWeights = NULL;

#define CELL_SIZE 200.0
  int x, y, s;
  float maxCellWeight = 0.0;
    
  static float minX, minY;
  static float sizeX, sizeY;
  static int numX, numY;

  if ( firstTime ||
       (sizeX != (samples->maxX - samples->minX)) ||
       (sizeY != (samples->maxY - samples->minY))) {

    firstTime = FALSE;
    
    /* Reallocate the fields. */
    if ( sumOfWeights != NULL) {
      fprintf(stderr, "# Size has changed. Reallocate memory.\n");
      free2D( (void**) maxWeight, numX, FLOAT);
      free2D( (void**) maxPos, numX, FLOAT);
      free2D( (void**) sumOfWeights, numX, FLOAT);
    }
      
    /* Set the boarders. */
    minX  = samples->minX;
    minY  = samples->minY;
    sizeX = samples->maxX - samples->minX;
    sizeY = samples->maxY - samples->minY;
    numX  = sizeX / CELL_SIZE + 1;
    numY  = sizeY / CELL_SIZE + 1;

    maxWeight    = (float**) allocate2D( numX, numY, FLOAT);
    maxPos       = (realPosition**) allocate2D( numX, numY, REAL_POSITION);
    sumOfWeights = (float**) allocate2D( numX, numY, FLOAT);
  }

  /* Reset all sums to zero. */
  for ( x = 0; x < numX; x++)
    for ( y = 0; y < numY; y++) {
      maxWeight[x][y] = 0.0;
      sumOfWeights[x][y] = 0.0;
    }
  
  for ( s = 0; s < samples->numberOfSamples; s++) {

    sampleType sample = samples->sample[s];
    int x = ( sample.pos.x - minX) / CELL_SIZE;
    int y = ( sample.pos.y - minY) / CELL_SIZE;

    if ( x < numX && y < numY && x >= 0 && y >= 0) {
      
      /* Update the local information about samples within this cell. */
      if ( sample.weight > maxWeight[x][y]) {
	maxPos[x][y]    = sample.pos;
	maxWeight[x][y] = sample.weight;
      }
      sumOfWeights[x][y] += sample.weight;
    }
  }

  /* Now look for most likely cell. */
  for ( x = 0; x < numX; x++)
    for ( y = 0; y < numY; y++) 
      if ( sumOfWeights[x][y] > maxCellWeight) {
	samples->mean = maxPos[x][y];
	maxCellWeight = sumOfWeights[x][y];
      }
  
  samples->stdDev = 0.0;
  
  writeLog( "%f %f %f %f %f %d #bestGridCell\n", elapsedScriptTime,
	    samples->mean.x, samples->mean.y, rad2Deg(samples->mean.rot),
	    maxCellWeight, samples->numberOfSamples);

  /* Now average over an area around this best position. */
  { 
#define AREA_SIZE 50.0
    realPosition mean;
    float avgSin = 0.0, avgCos = 0.0;
    float probSum = 0.0;
    
    mean.x = mean.y = 0.0;
    mean.rot = 0.0;

    for (s = 0; s < samples->numberOfSamples; s++) {
      
      if (realPositionDistance(samples->sample[s].pos, samples->mean) < AREA_SIZE) {

	double prob = samples->sample[s].weight;
	
        mean.x += prob * samples->sample[s].pos.x;
        mean.y += prob * samples->sample[s].pos.y;
	
	/* The angle is a bit more complicated. */
	avgSin += (float) prob * sin( samples->sample[s].pos.rot);
	avgCos += (float) prob * cos( samples->sample[s].pos.rot);
	probSum += prob;
      }
    }    
    if (probSum > 0.0){
      mean.x /= probSum;
      mean.y /= probSum;
      mean.rot = atan2( avgSin, avgCos);
    }
    
    samples->mean = mean;
    maxCellWeight = probSum;
  }
  
  writeLog( "%f %f %f %f %f %d #localMean\n", elapsedScriptTime,
	    samples->mean.x, samples->mean.y, rad2Deg(samples->mean.rot),
	    maxCellWeight, samples->numberOfSamples);
}



/********************************************************************
 ********************************************************************
 ********************************************************************
 * The local functions.
 ********************************************************************
 ********************************************************************
 ********************************************************************/

void
deallocateSamples( sampleSet* samples)
{
  if ( samples->accumulatedProbs != NULL) 
    free1D( samples->accumulatedProbs, FLOAT);
  
  if ( samples->tmpPositions != NULL) 
    free1D( samples->tmpPositions, REAL_POSITION);

  free( ( sampleType*) samples->sample);
  free( ( sampleType*) samples->tmpSample);
}

sampleSet
initializedSamples( int numberOfSamples,
		    float minX, float minY,
		    float maxX, float maxY)
{
  sampleSet samples;

  samples.numberOfSamples = numberOfSamples;
  samples.desiredNumberOfSamples = numberOfSamples;
  samples.allocatedSamples = numberOfSamples;
  samples.fractionOfUniformSamples = 0.0;
  
  samples.maxX = maxX;
  samples.maxY = maxY;
  samples.minX = minX;
  samples.minY = minY;
  
  samples.summedMovement.x   = 0.0;
  samples.summedMovement.y   = 0.0;
  samples.summedMovement.rot = 0.0;
  
  samples.sample = ( sampleType*) malloc( sizeof( sampleType) * numberOfSamples);
  samples.tmpSample = ( sampleType*) malloc( sizeof( sampleType) * numberOfSamples);
  if ( samples.sample == NULL || samples.tmpSample == NULL) {
    fprintf( stderr,
	     "Error! Not enough memory for allocating %d samples.\n",
	     numberOfSamples);
    exit(0);
  }

  writeLog( "# Bounding box: [%f : %f] : [%f : %f]\n", minX, maxX, minY, maxY);
  
  samples.accumulatedProbs = (double*) allocate1D( numberOfSamples, DOUBLE);
  samples.tmpPositions     = (realPosition*) allocate1D( numberOfSamples, REAL_POSITION);
  
  return samples;
}

static sampleSet
readSamples( char* fileName)
{
  sampleSet samples;
  
  FILE *ifp;
    
#define NUMBER_KEY_WORD    "#Number "
#define DIMENSION_KEY_WORD "#Dimension "
#define MIN_KEY_WORD       "#Min "
#define MAX_KEY_WORD       "#Max "
  
  char line[BUFFLEN];
  int numberOfReadSamples = 0;
  int numberOfSamples = 0;
  float minX, minY, maxX, maxY;
  /* Scan for min and max values. */
  if ((ifp = fopen( fileName, "r")) == NULL) {
    fprintf(stderr, "Cannot open file %s.\n", fileName);
    exit(0);
  }


  fgets(line, BUFFLEN,ifp);
  if ( strncmp(line, NUMBER_KEY_WORD, strlen(NUMBER_KEY_WORD)) == 0) {
    unsigned int markLength = strlen( NUMBER_KEY_WORD);
    sscanf( &line[markLength], "%d", &numberOfSamples);
    fprintf(stderr, "File should contain %d samples.\n", numberOfSamples);
  }
  else
    fprintf( stderr, "Keyword %s expected.\n", NUMBER_KEY_WORD);
  
  fgets(line, BUFFLEN,ifp);
  if ( strncmp(line, DIMENSION_KEY_WORD, strlen(DIMENSION_KEY_WORD)) == 0) {
    unsigned int markLength = strlen( DIMENSION_KEY_WORD);
    int dimension;
    sscanf( &line[markLength], "%d", &dimension);
    fprintf(stderr, "Samples have dimension %d.\n", dimension);
    if ( dimension != 3) {
      fprintf( stderr, "Cannot read samples with dimension not equal 3.\n");
      exit(0);
    }
  }
  else
    fprintf( stderr, "Keyword %s expected.\n", DIMENSION_KEY_WORD);
  
  fgets(line, BUFFLEN,ifp);
  if ( strncmp(line, MIN_KEY_WORD, strlen(MIN_KEY_WORD)) == 0) {
    unsigned int markLength = strlen( MIN_KEY_WORD);
    sscanf( &line[markLength], "%f %f",
	    &minX, &minY);
  }
  else
    fprintf( stderr, "Keyword %s expected.\n", MIN_KEY_WORD);
  
  fgets(line, BUFFLEN,ifp);
  if ( strncmp(line, MAX_KEY_WORD, strlen(MAX_KEY_WORD)) == 0) {
    unsigned int markLength = strlen( MAX_KEY_WORD);
    sscanf( &line[markLength], "%f %f",
	    &maxX, &maxY);
  }
  else
    fprintf( stderr, "Keyword %s expected.\n", MAX_KEY_WORD);

  samples = initializedSamples( numberOfSamples, minX, minY, maxX, maxY);
  
  fprintf(stderr, "Start reading samples ... ");

  while (!feof(ifp) && (numberOfReadSamples < numberOfSamples)) {

    float tmpWeight;

    fgets(line, BUFFLEN,ifp);

    /* This should be a sample.     */
    if ( sscanf(line, "%f %f %f %f",
		&(samples.sample[numberOfReadSamples].pos.x),
		&(samples.sample[numberOfReadSamples].pos.y),
		&(samples.sample[numberOfReadSamples].pos.rot),
		&tmpWeight)  == 4){
      samples.sample[numberOfReadSamples].pos.rot = deg2Rad(samples.sample[numberOfReadSamples].pos.rot);
      samples.sample[numberOfReadSamples].weight = tmpWeight;
      numberOfReadSamples++;
    }
    else
      fprintf( stderr, "Error reading line %s", line);
  }

  fprintf(stderr, "done. Successfully read %d samples.\n", numberOfReadSamples);

  fclose(ifp);
  
  
  return samples;
}


/* Draw samples from the set according to the distribution and store
 * them in the same set again. Consider size of the set as well. */
static void
sampleAccordingToDistribution( sampleSet* samples, probabilityGrid* aPrioriProbs)
{
#define LOW_VARIANCE 1
#ifdef LOW_VARIANCE
  lowVarianceResample( samples, aPrioriProbs);
  return;
#endif
  
  if ( ! samples->alreadySampled) {

    double avgWeight;
    int s;
    float resolution = aPrioriProbs->resolution;
    float xOffset = samples->minX + 0.5 * aPrioriProbs->resolution;
    float yOffset = samples->minY + 0.5 * aPrioriProbs->resolution;
    int numberOfSamples = samples->numberOfSamples;
    
    bool addUniformSamples = samples->fractionOfUniformSamples > 0.0;
    float addUniformThreshold = 1.0 - samples->fractionOfUniformSamples;
    
    /* Set the new number of samples. */
    samples->desiredNumberOfSamples = iMax( samples->desiredNumberOfSamples,
					    samples->allocatedSamples);

    /* Allocate memory for the temporary structures. */
    checkAllocationOfAuxiliaryVariables( samples);

    /* Normalize the weights of the samples. */
    if ( normalizeWeights( samples) < 0.9) {
      writeLog( "No valid samples. Reset samples.\n");
      fprintf(stderr, "No valid samples. Reset samples.\n");
      setUniformDistribution( samples, aPrioriProbs);
      return;
    };
    
    /* Accumulate the probabilities for sampling. This is the current number
     * of active samples (samples->numberOfSamples). */
    samples->accumulatedProbs[0] = samples->sample[0].weight;
    for ( s = 1; s < samples->numberOfSamples; s++) {
      samples->accumulatedProbs[s] =
	samples->accumulatedProbs[s-1] + samples->sample[s].weight;
    }

    /* Now sample from the original samples according to the probabilities
     * in the accumulated probs. The resulting number of samples should be
     * samples->desiredNumberOfSamples. Store the result in the temporary struct. */
    for ( s = 0; s < samples->desiredNumberOfSamples; s++) {

      
      /* We sample from a uniform distribution with probability addUniformThreshold. */
      if ( addUniformSamples) {
	
	if ( RAND_ZERO_TO_ONE() > addUniformThreshold) {
	  
	  /* If the free space is defined as a list of cells then we sample from
	   * this list. */
	  if ( aPrioriProbs->freeSpace != NULL) {
	    
	    int index = RAND_ZERO_TO_ONE() * aPrioriProbs->numberOfFreeCells;

	    samples->tmpPositions[s].x   = aPrioriProbs->freeSpace[index].x * resolution + xOffset;
	    samples->tmpPositions[s].y   = aPrioriProbs->freeSpace[index].y * resolution + yOffset;
	    samples->tmpPositions[s].rot = RAND_ZERO_TO_ONE() * DEG_360;
	  }
	  /* Otherwise we sample from the whole region. */
	  else {
	    
	    static int first = TRUE;
	    static float sizeX, sizeY;
	    static float minX, minY;
	    
	    if ( first) {
	      first = FALSE;
	      sizeX = samples->maxX - samples->minX;
	      sizeY = samples->maxY - samples->minY;
	      minX  = samples->minX;
	      minY  = samples->minY;
	    }
	    
	    samples->tmpPositions[s].x   = minX + RAND_ZERO_TO_ONE() * sizeX;
	    samples->tmpPositions[s].y   = minY + RAND_ZERO_TO_ONE() * sizeY;
	    samples->tmpPositions[s].rot = RAND_ZERO_TO_ONE() * DEG_360;
	  }
	}
	else {
	  
	  double randVal = RAND_ZERO_TO_ONE();
	  int drawnSample = binarySearch( samples->accumulatedProbs, randVal,
					  samples->numberOfSamples);
	  samples->tmpPositions[s] = samples->sample[drawnSample].pos;
	}
      }
      else {
	
	double randVal = RAND_ZERO_TO_ONE();
	int drawnSample = binarySearch( samples->accumulatedProbs, randVal,
					samples->numberOfSamples);
	samples->tmpPositions[s] = samples->sample[drawnSample].pos;
      }
    }

    /* Now copy the positions back. */
    for ( s = 0; s < samples->desiredNumberOfSamples; s++) 
      samples->sample[s].pos = samples->tmpPositions[s];

    /* Everything is prepared. Set number of samples to desired. */
    samples->numberOfSamples = samples->desiredNumberOfSamples;
    
    /* Reset probabilities. */
    avgWeight = 1.0 / samples->numberOfSamples;
    for ( s = 0; s < samples->numberOfSamples; s++) 
      samples->sample[s].weight = avgWeight;

    samples->alreadySampled = TRUE;

    /* Set the desired number of samples back to the original size. */
    samples->desiredNumberOfSamples = numberOfSamples;
  }
}

/* Draw samples from the set according to the distribution and store
 * them in the same set again. Consider size of the set as well. */
static void
lowVarianceResample( sampleSet* samples, probabilityGrid* aPrioriProbs)
{
  if ( ! samples->alreadySampled) {

    double avgWeight;
    int s;
    float resolution = aPrioriProbs->resolution;
    float xOffset = samples->minX + 0.5 * aPrioriProbs->resolution;
    float yOffset = samples->minY + 0.5 * aPrioriProbs->resolution;
    int numberOfSamples = samples->numberOfSamples;
    
    bool addUniformSamples = samples->fractionOfUniformSamples > 0.0;
    float addUniformThreshold = 1.0 - samples->fractionOfUniformSamples;
    
    double increment = 1.0 / (double) samples->desiredNumberOfSamples;
    double currentThreshold  = increment * RAND_ZERO_TO_ONE();
    double currentSum;

    int currentSample = 0;
    
    /* Set the new number of samples. */
    samples->desiredNumberOfSamples = iMax( samples->desiredNumberOfSamples,
					    samples->allocatedSamples);

    /* Allocate memory for the temporary structures. */
    checkAllocationOfAuxiliaryVariables( samples);

    /* Normalize the weights of the samples. */
    if ( normalizeWeights( samples) == 0.0) {
      writeLog( "No valid samples. Reset samples.\n");
      fprintf(stderr, "No valid samples. Reset samples.\n");
      setUniformDistribution( samples, aPrioriProbs);
      return;
    }

    /* Initialize the first sum.*/
    currentSum = samples->sample[0].weight;

    /* Now sample from the original samples according to the probabilities
     * in the accumulated probs. The resulting number of samples should be
     * samples->desiredNumberOfSamples. Store the result in the temporary struct. */
    for ( s = 0; s < samples->desiredNumberOfSamples; s++, currentThreshold += increment) {

      /* fprintf( stderr, "%d %d %f %f #OUTER\n", s, currentSample, currentThreshold, currentSum); */
      
      while ( currentSum < currentThreshold && currentSample < samples->desiredNumberOfSamples-1) {
	currentSum += samples->sample[++currentSample].weight;
      }
	  
      /* We sample from a uniform distribution with probability addUniformThreshold. */
      if ( addUniformSamples) {
	
	if ( RAND_ZERO_TO_ONE() > addUniformThreshold) {
	  
	  /* If the free space is defined as a list of cells then we sample from
	   * this list. */
	  if ( aPrioriProbs->freeSpace != NULL) {
	    
	    int index = RAND_ZERO_TO_ONE() * aPrioriProbs->numberOfFreeCells;

	    samples->tmpPositions[s].x   = aPrioriProbs->freeSpace[index].x * resolution + xOffset;
	    samples->tmpPositions[s].y   = aPrioriProbs->freeSpace[index].y * resolution + yOffset;
	    samples->tmpPositions[s].rot = RAND_ZERO_TO_ONE() * DEG_360;
	  }
	  /* Otherwise we sample from the whole region. */
	  else {
	    
	    static int first = TRUE;
	    static float sizeX, sizeY;
	    static float minX, minY;
	    
	    if ( first) {
	      first = FALSE;
	      sizeX = samples->maxX - samples->minX;
	      sizeY = samples->maxY - samples->minY;
	      minX  = samples->minX;
	      minY  = samples->minY;
	    }
	    
	    samples->tmpPositions[s].x   = minX + RAND_ZERO_TO_ONE() * sizeX;
	    samples->tmpPositions[s].y   = minY + RAND_ZERO_TO_ONE() * sizeY;
	    samples->tmpPositions[s].rot = RAND_ZERO_TO_ONE() * DEG_360;
	  }
	}
	else {
	  if(0) fprintf( stderr, "[T%d ", s);
	  samples->tmpPositions[s] = samples->sample[currentSample].pos;
	  if (0) fprintf( stderr, "T]\n");
	}
      }
      else {
	samples->tmpPositions[s] = samples->sample[currentSample].pos;
      }
    }

    /* Now copy the positions back. */
    for ( s = 0; s < samples->desiredNumberOfSamples; s++) 
      samples->sample[s].pos = samples->tmpPositions[s];

    /* Everything is prepared. Set number of samples to desired. */
    samples->numberOfSamples = samples->desiredNumberOfSamples;
    
    /* Reset probabilities. */
    avgWeight = 1.0 / samples->numberOfSamples;
    for ( s = 0; s < samples->numberOfSamples; s++) 
      samples->sample[s].weight = avgWeight;

    samples->alreadySampled = TRUE;

    /* Set the desired number of samples back to the original size. */
    samples->desiredNumberOfSamples = numberOfSamples;
  }
}

/*****************************************************************************
 * Samples according to the distribution of the src samples. Number of desired
 * samples must already be set in the dest samples.
 *****************************************************************************/
void
generateSample( sampleSet* sampleSrc, sampleSet* sampleDest)
{
  if (sampleDest->allocatedSamples == 0) {



    *sampleDest = initializedSamples( sampleDest->allocatedSamples,
				      sampleSrc->minX, sampleSrc->minY,
				      sampleSrc->maxX, sampleSrc->maxY);
  }
  
  if ( sampleSrc->alreadySampled) {
    
    int s;
    
    /* Uniform distribution. */
    for ( s = 0; s < sampleDest->desiredNumberOfSamples; s++) {

      int drawnSample = randMax( sampleSrc->numberOfSamples-1);
      
      sampleDest->sample[s] = sampleSrc->sample[drawnSample];
    }      
  }
  else {
    fprintf(stderr, "Shouldn't come here.\n");
  }
}



/*****************************************************************************
 * Set the region of the samples.
 *****************************************************************************/
void
setSampleRegion( sampleSet* samples,
		 float minX, float minY,
		 float maxX, float maxY)
{
  samples->maxX = maxX;
  samples->maxY = maxY;
  samples->minX = minX;
  samples->minY = minY;
}


/*****************************************************************************
 * BinarySearch for smallest k for which c(k)>=r
  *****************************************************************************/
static int
binarySearch( double* values, double searchVal, int numberOfValues)
{
  int left = 0, right = numberOfValues - 1;
  int middle = numberOfValues / 2;
  
  if (values[left] > searchVal) return left;

  while(1) {
    if( values[middle] < searchVal) {
      if((right - middle) == 1)
	return right;
      left = middle;
      middle = middle + (right - middle) / 2;
    }
    else {
      if((middle - left) == 1)
	return values[left] < searchVal ? middle : left;
      right = middle;
      middle = middle - (middle - left) / 2;
    }
  }
}

static float averageSampleDistance( sampleSet* samples, realPosition refPos,
				    float* rotDist)
{
  float dist = 0.0;
  float sumOfWeights =0.0;
  int s;
  float refRot = refPos.rot;
  
  *rotDist = 0.0;
  
  for ( s = 0; s < samples->numberOfSamples; s++) {
    sumOfWeights += samples->sample[s].weight;
    dist += samples->sample[s].weight *
      distanceBetweenPoints( samples->sample[s].pos, refPos);

    *rotDist += angleDistance(samples->sample[s].pos.rot, refRot);

  }

  if ( sumOfWeights > 0) {
    dist /= sumOfWeights;
    *rotDist /= sumOfWeights;
  }
  else {
    dist = 0.0;
    *rotDist = 0.0;
  }
  
  return dist;
}


static float probabilityOfPosition( sampleSet* samples, realPosition refPos,
				    float maxXYDist, float maxRotDist)
{
  float prob = 0.0;
  float sumOfWeights =0.0;
  int s;

  for ( s = 0; s < samples->numberOfSamples; s++) {
    if ( ( distanceBetweenPoints( samples->sample[s].pos, refPos) < maxXYDist)
	 &&
	 ( angleDistance( samples->sample[s].pos.rot, refPos.rot) < maxRotDist)) {
      prob += samples->sample[s].weight;
    }
/*     else { */
/*       writeLog( "%f %f %f %f --> %f [%f %f] --> %f    %f %NOT\n", */
/* 		samples->sample[s].pos.x, refPos.x,  */
/* 		samples->sample[s].pos.y, refPos.y, */
/* 		distanceBetweenPoints( samples->sample[s].pos, refPos), */
/* 		samples->sample[s].pos.rot, refPos.rot, */
/* 		angleDistance( samples->sample[s].pos.rot, refPos.rot), */
/* 		samples->sample[s].weight); */
/*     } */
    sumOfWeights += samples->sample[s].weight;
  }

  if ( sumOfWeights > 0)
    prob /= sumOfWeights;
  else
    prob = 0.0;
  
  return prob;
}


static void
updateSampleInformation( actionInformation* info)
{
  unsigned int refPosFound;
  realPosition refPos = measuredRobotPosition( elapsedScriptTime, &refPosFound);

  normalizeWeights( &(info->samples));
  
  /* Set the estimated position of the robot to the mean of the
   * samples. */
  setLocalSampleMean( &(info->samples)); 

  if ( info->measuredRobotPosition.x != 0.0 || info->measuredRobotPosition.y != 0.0 || info->measuredRobotPosition.rot != 0.0)
    info->estimatedRobot.pos = info->measuredRobotPosition;
  else
    info->estimatedRobot.pos = info->samples.mean;
    
#define MAX_XY_DIST 50
#define MAX_ROT_DIST (deg2Rad(5))

  if (0) {refPosFound = TRUE;
  refPos = info->measuredRobotPosition;
  }
  fprintf(stderr, "######## %d %f %f %f\n", refPosFound,
	  refPos.x, refPos.y, refPos.rot);

  
  /* We got a reference position from a file. */
  if ( refPosFound && (refPos.x != 0.0 || refPos.y != 0.0 || refPos.rot != 0.0)) {
    float rotDist;
    float avgDist = averageSampleDistance( &(info->samples), refPos, &rotDist);
    float probOfRefPos = probabilityOfPosition( &(info->samples), refPos, MAX_XY_DIST, MAX_ROT_DIST);
    writeLog( "%f %f %f %f %f %f %f #distances\n", elapsedScriptTime, avgDist, probOfRefPos,
	      distanceBetweenPoints( info->samples.mean, refPos),  rad2Deg(rotDist),
	      refPos.x - info->samples.mean.x, refPos.y - info->samples.mean.y);
    if (0 && distanceBetweenPoints( info->samples.mean, refPos) > 150){
      fprintf(stderr, "# Distance too large, exiting\n");
      writeLog("# Distance too large, exiting\n");
      exit (0);
    }
  }
  
  if (communicationInfo.scr->newMarker){
    dumpDistanceToMarker( info->estimatedRobot.pos,
			  communicationInfo.scr->markerCount);

    if (info->simMap.initialized){
      (info->estimatedRobot.pos) =
	scanMatchingPosition(info->estimatedRobot.pos,
			     &(info->simMap), info);
      
      dumpDistanceToMarker( info->estimatedRobot.pos,
			    communicationInfo.scr->markerCount);
    }
  }
}


static void
dumpSamples( sampleSet* samples, char* robotName,
	     int numberOfSamples, realPosition refPos)
{
  char fileName[80];
  FILE* fp;
  int s;
  char *name = robotName;
  int numberOfSet = samples->numberOfSet;
  static int count = 0;
  if (robotName == NULL){
    numberOfSet = count;
    count++;
    name = "Samples";
  }
    
  if ( numberOfSamples < 0)
    numberOfSamples = samples->numberOfSamples;
  
  fprintf( stderr, "Dump %d samples no %d\n", numberOfSamples,
	   numberOfSet);
  writeLog( "Dump %d samples no %d\n", numberOfSamples,
	    numberOfSet);

  sprintf(fileName, "%s.%d", name, numberOfSet);
  fp = fopen( fileName, "w");

  fprintf( fp, "%s %d\n", NUMBER_KEY_WORD, numberOfSamples);
  fprintf( fp, "%s 3\n", DIMENSION_KEY_WORD);
  fprintf( fp, "%s %f %f %f\n", MIN_KEY_WORD,
	   samples->minX, samples->minY, -DEG_360);
  fprintf( fp, "%s %f %f %f\n", MAX_KEY_WORD,
	   samples->maxX, samples->maxY, DEG_360);
  fprintf( fp, "%s %f %f %f\n", REF_POS_KEY_WORD,
	   refPos.x, refPos.y, refPos.rot);
  fprintf( fp, "%s %ld %ld\n", TIME_STAMP_KEY_WORD,
	   samples->timeStamp.tv_sec, samples->timeStamp.tv_usec);
	   
  for ( s = 0; s < numberOfSamples; s++)
    fprintf( fp, "%f %f %f %f\n", 
	     samples->sample[s].pos.x,
	     samples->sample[s].pos.y,
	     samples->sample[s].pos.rot,
	     samples->sample[s].weight);
  fclose ( fp);
}
  


static void
checkAllocationOfAuxiliaryVariables( sampleSet* samples)
{
  static int numberOfLocalSamples = 0;
  
  if ( numberOfLocalSamples < samples->desiredNumberOfSamples ||
       numberOfLocalSamples < samples->numberOfSamples) {
    
    numberOfLocalSamples = iMax( samples->desiredNumberOfSamples,
				 samples->numberOfSamples);
    
    writeLog( "# Reallocate %d samples (%d %d)\n", numberOfLocalSamples,
	      samples->desiredNumberOfSamples, samples->numberOfSamples);

    fprintf( stderr, "# Reallocate %d samples (%d %d)\n", numberOfLocalSamples,
	      samples->desiredNumberOfSamples, samples->numberOfSamples);

    if ( samples->accumulatedProbs != NULL) 
      free1D( samples->accumulatedProbs, FLOAT);
    
    if ( samples->tmpPositions != NULL) 
      free1D( samples->tmpPositions, REAL_POSITION);
    
    samples->accumulatedProbs = (double*)
      allocate1D( numberOfLocalSamples, DOUBLE);

    samples->tmpPositions = (realPosition*)
      allocate1D( numberOfLocalSamples, REAL_POSITION);
  }
}

void
setUniformDistribution( sampleSet* samples, probabilityGrid* aPrioriProbs)
{
  int s;
  double avgWeight = 1.0 / samples->numberOfSamples;
  float sizeX = samples->maxX - samples->minX;
  float sizeY = samples->maxY - samples->minY;
  float resolution = aPrioriProbs->resolution;
  float xOffset = samples->minX + 0.5 * aPrioriProbs->resolution;
  float yOffset = samples->minY + 0.5 * aPrioriProbs->resolution;
  
  /* Set uniform distribution. */
  for ( s = 0; s < samples->numberOfSamples; s++) {
    
    /* If the free space is defined as a list of cells then we sample from
     * this list. */
    
    if ( aPrioriProbs->freeSpace != NULL) {

      int index = randMax( aPrioriProbs->numberOfFreeCells-1);

      samples->sample[s].pos.x   = aPrioriProbs->freeSpace[index].x * resolution + xOffset;
      samples->sample[s].pos.y   = aPrioriProbs->freeSpace[index].y * resolution + yOffset;
      samples->sample[s].pos.rot = RAND_ZERO_TO_ONE() * DEG_360;
      samples->sample[s].weight = avgWeight;
    }
    /* Otherwise we sample from the whole region. */
    else {
      
      samples->sample[s].pos.x   = samples->minX + RAND_ZERO_TO_ONE() * sizeX;
      samples->sample[s].pos.y   = samples->minY + RAND_ZERO_TO_ONE() * sizeY;
      samples->sample[s].pos.rot = RAND_ZERO_TO_ONE() * DEG_360;
      samples->sample[s].weight = avgWeight;
    }
  }
  samples->alreadySampled = TRUE;
  samples->alreadyNormalized = TRUE;
}

static float
normalizeWeights( sampleSet* samples)
{
  if ( ! samples->alreadyNormalized) {

    double sumOfWeights = 0.0;  
    double normalizeFactor;
    int s;

    for ( s = 0; s < samples->numberOfSamples; s++) 
      sumOfWeights += samples->sample[s].weight;

    if ( sumOfWeights > 0.0) {
      normalizeFactor = 1.0 / sumOfWeights;
      for ( s = 0; s < samples->numberOfSamples; s++) 
	samples->sample[s].weight *= normalizeFactor;
    }

    samples->alreadyNormalized = TRUE;

    return sumOfWeights;
  }
  else {
    return 1.0;
  }
}

void
copySamples( sampleSet* src, sampleSet* dest)
{
  int numberOfSamples = src->numberOfSamples;
  int s;
  
  if ( dest->allocatedSamples < src->numberOfSamples) {

    fprintf(stderr, "# Generate %d new samples.\n", numberOfSamples);
    writeLog( "# Generate %d new samples.\n", numberOfSamples);
    
    if ( dest->sample != NULL)
      free( ( sampleType*) dest->sample);

    dest->sample = ( sampleType*) malloc( sizeof( sampleType) * numberOfSamples);
    
    if ( dest->sample == NULL) {
      fprintf( stderr,
	       "Error! Not enough memory for allocating %d samples.\n",
	       numberOfSamples);
      exit(0);
    }
    
    dest->allocatedSamples = numberOfSamples;
  }
  
  dest->numberOfSamples        = numberOfSamples;
  dest->desiredNumberOfSamples = numberOfSamples;
  dest->numberOfSet            = src->numberOfSet;
  
  dest->maxX = src->maxX;
  dest->maxY = src->maxY;
  dest->minX = src->minX;
  dest->minY = src->minY;
  
  dest->summedMovement.x   = src->summedMovement.x;
  dest->summedMovement.y   = src->summedMovement.y;
  dest->summedMovement.rot = src->summedMovement.rot;

  dest->timeStamp          = src->timeStamp;
  
  /* Copy the samples. */
  for ( s = 0; s < numberOfSamples; s++) {
    dest->sample[s].pos    = src->sample[s].pos;
    dest->sample[s].weight = src->sample[s].weight;
  }
}

#define NUMBER_OF_SAMPLES_PER_POSITION 60000
#define NUMBER_OF_BEAMS                360
#define BEAM_SKIP                      10
#define NUMBER_OF_DIRECTIONS           360
   
#define NUMBER_OF_DISTANCE_FEATURES    5
#define NUMBER_OF_ANGLE_FEATURES       8
#define NUMBER_OF_AVERAGE_FEATURES     5

/* #define SMITHSONIAN */
#ifdef SMITHSONIAN
#define MIN_DIST 40.0
#define MAX_DIST 550.0

#define MIN_AVERAGE 500.0
#define MAX_AVERAGE 1100.0

#define MAXRANGE 2560

#else
#define MIN_DIST 30.0
#define MAX_DIST 300.0

#define MIN_AVERAGE 200.0
#define MAX_AVERAGE 320.0

#define MAXRANGE 1000

#endif
    
#ifdef ONE_SCANNER
#define MIN_ANGLE -1.0
#define MAX_ANGLE  1.0
#define INCOMPLETE_ANGLE_AREA 1
#else
#define MIN_ANGLE  0.0
#define MAX_ANGLE  DEG_360
#define INCOMPLETE_ANGLE_AREA 0
#endif


typedef struct {
  short int x;
  short int y;
  short int rot;
} smallGridPosition;

typedef struct gridSampleSet {
  int numberOfSamples;
  smallGridPosition* position;
  float* weight;
  int numberOfIntegrations;
} gridSampleSet;


typedef struct scanFeature {
  float posX;
  float posY;
  float distance;
  float angle;
  int   matrixDistance;
  int   matrixAngle;
  int   matrixAverage;
  float avgDistance;
} scanFeature;

gridSampleSet*
setForPreprocessing( int numberOfSamples,
		     probabilityGrid* map,
		     sensing_PROXIMITY* frontScan, sensing_PROXIMITY* rearScan)
{
  static int count;
  int s;

  position* freeSpace = map->freeSpace;
  int numberOfFreeCells = map->numberOfFreeCells;
  
  gridSampleSet* set = (gridSampleSet*) malloc( sizeof( gridSampleSet));

  smallGridPosition* position = ( smallGridPosition*) malloc( sizeof( smallGridPosition) * numberOfSamples);

  float* weight = ( float*) malloc( sizeof( float) * numberOfSamples);

  if ( set == NULL || position == NULL || weight == NULL) {
    fprintf(stderr, "Not enough memory for allocating set number %d.\n", count);
    return set;
  }
  
  set->position             = position;
  set->numberOfSamples      = numberOfSamples;
  set->weight               = weight;
  set->numberOfIntegrations = 0;
  
  /* Generate the positions. */
  for ( s = 0; s < numberOfSamples; s++) {
    int index = randMax( numberOfFreeCells-1);
    if ( index < 0 || index >= numberOfFreeCells) {
      fprintf(stderr, "Wrong INDEX %d %d\n", index, numberOfFreeCells);
      exit(0);
    }
    position[s].x   = freeSpace[index].x;
    position[s].y   = freeSpace[index].y;
    position[s].rot = randMax( 359);

    /* Sanity check. */
    if (0) if( position[s].x < 0 || position[s].x >= map->sizeX ||
	       position[s].y < 0 || position[s].y >= map->sizeY ||
	       position[s].rot < 0 || position[s].rot > 359) {
      fprintf(stderr, "Wrong random position: %d %d %d\n", position[s].x, position[s].y, position[s].rot);
      exit(0);
    }

    weight[s] = 0.0;
  }

  writeLog( "# Generated set no %d\n", count++);
  
  return set;
}

static scanFeature
extractedScanFeature( sensing_PROXIMITY* frontScan, sensing_PROXIMITY* rearScan,
		      int* frontBeamAngle, int* rearBeamAngle, 
		      float* cosinus, float* sinus,
		      float distanceResolution, float angleResolution, float averageResolution)
{
  scanFeature feature;
  int beam;
  float centerDistance, centerAngle, centerX = 0.0, centerY = 0.0;
  float distSum = 0.0;

  float realFrontCenterX = frontScan->offset.forward;
  float realFrontCenterY = 0.0;
  
#define ENDPOINT_THRESHOLD 20.0
  
  int count = 0;
  float lastUsedX = 0.0, lastUsedY = 0.0;
  
  /* Cut the beams. */
  for ( beam = 0; beam < frontScan->numberOfReadings; beam++) {
    if ( frontScan->reading[beam].dist > MAXRANGE)
      frontScan->reading[beam].dist = MAXRANGE;
    if ( rearScan != NULL)
      if ( rearScan->reading[beam].dist > MAXRANGE)
	rearScan->reading[beam].dist = MAXRANGE;
  }      

  /* Generate end points of front scan. */
  for ( beam = 0; beam < frontScan->numberOfReadings; beam++) {

    float endX = realFrontCenterX + cosinus[frontBeamAngle[beam]] * frontScan->reading[beam].dist;
    float endY = realFrontCenterY + sinus[frontBeamAngle[beam]] * frontScan->reading[beam].dist;

    distSum += frontScan->reading[beam].dist;
    
    if ( pointDistance( endX, endY, lastUsedX, lastUsedY) > ENDPOINT_THRESHOLD) {
      centerX += endX;
      centerY += endY;
      count++;
      lastUsedX = endX;
      lastUsedY = endY;
    }
  }

  if ( rearScan != NULL) {

    float realRearCenterX  = rearScan->offset.forward;
    float realRearCenterY  = 0.0;
    
    for ( beam = 0; beam < rearScan->numberOfReadings; beam++) {
      
      float endX = realRearCenterX + cosinus[rearBeamAngle[beam]] * rearScan->reading[beam].dist;
      float endY = realRearCenterY + sinus[rearBeamAngle[beam]] * rearScan->reading[beam].dist;
      
      distSum += rearScan->reading[beam].dist;
      
      if ( pointDistance( endX, endY, lastUsedX, lastUsedY) > ENDPOINT_THRESHOLD) {
	centerX += endX;
	centerY += endY;
	count++;
	lastUsedX = endX;
	lastUsedY = endY;
      }
    }
  }

  /* The center in x-y and polar coordinates. */
  centerX /= count;
  centerY /= count;
  
  feature.avgDistance = distSum / (frontScan->numberOfReadings +
				   ((rearScan != NULL ) ? rearScan->numberOfReadings : 0));

  centerDistance = sqrt( fSqr( centerX) + fSqr( centerY));
  centerAngle    = normalizedAngle( atan2( centerY, centerX));
  
  /* Check which feature this is. */
  if ( centerDistance > MAX_DIST)
    feature.matrixDistance = NUMBER_OF_DISTANCE_FEATURES - 1;
  else if ( centerDistance < MIN_DIST)
    feature.matrixDistance = 0;
  else
    feature.matrixDistance = 1 + (centerDistance - MIN_DIST) / distanceResolution;
  
  if (INCOMPLETE_ANGLE_AREA) {
    fprintf(stderr, "Oops. Angle area incomplete.\n");
    if ( centerAngle > MAX_ANGLE)
      feature.matrixAngle = NUMBER_OF_ANGLE_FEATURES - 1;
    else if ( centerAngle < MIN_ANGLE)
      feature.matrixAngle = 0;
    else
      feature.matrixAngle = 1 + (centerAngle - MIN_ANGLE) / angleResolution;
  }
  else {
    if ( feature.matrixDistance == 0)
      feature.matrixAngle = 0;
    else {
      feature.matrixAngle = (int) (centerAngle / DEG_360 * NUMBER_OF_ANGLE_FEATURES);
      if ( feature.matrixAngle >= NUMBER_OF_ANGLE_FEATURES)
	feature.matrixAngle = NUMBER_OF_ANGLE_FEATURES - 1;
    }
  }
  
  if ( feature.avgDistance > MAX_AVERAGE)
    feature.matrixAverage = NUMBER_OF_AVERAGE_FEATURES - 1;
  else if ( feature.avgDistance < MIN_AVERAGE)
    feature.matrixAverage = 0;
  else
    feature.matrixAverage = 1 + (feature.avgDistance - MIN_AVERAGE) / averageResolution;

  feature.posX     = centerX;
  feature.posY     = centerY;
  feature.distance = centerDistance;
  feature.angle    = centerAngle;

  writeLog(  "%f %f %f %d %d %d #FEAT\n", rad2Deg(centerAngle), centerDistance, 
	     feature.avgDistance, feature.matrixAngle, feature.matrixDistance, feature.matrixAverage);

  return feature;
}

static void
dumpFeatureDefinitions()
{
#define ANGLES_KEY_WORD "ANGLES"
#define DISTANCES_KEY_WORD "DISTANCES"
#define AVERAGES_KEY_WORD "AVERAGES"
#define BEAMS_KEY_WORD "BEAMS"

  FILE *ofp;
  char* fileName = "features.def";
  
  /* Scan for min and max values. */
  if ((ofp = fopen( fileName, "w")) == NULL) {
    fprintf( stderr, "Cannot open definition file %s.\n", fileName);
    exit(0);
  }
  
  fprintf( ofp, "%s %d %f %f\n", ANGLES_KEY_WORD, NUMBER_OF_ANGLE_FEATURES, MIN_ANGLE, MAX_ANGLE);
  fprintf( ofp, "%s %d %f %f\n", DISTANCES_KEY_WORD, NUMBER_OF_DISTANCE_FEATURES, MIN_DIST, MAX_DIST);
  fprintf( ofp, "%s %d %f %f\n", AVERAGES_KEY_WORD, NUMBER_OF_AVERAGE_FEATURES, MIN_AVERAGE, MAX_AVERAGE);
  fprintf( ofp, "%s %f\n",       BEAMS_KEY_WORD, (float) MAXRANGE);
  fprintf( ofp, "GENERATE %d\n", GENERATE_SCANS);

  fclose( ofp);
}
     

static void 
dumpDensitySet(gridSampleSet* set, int angle, int distance, int average, probabilityGrid* map)
{
  char fileName[BUFFLEN];
  int s;
  gzFile zFile;
  float dataVec[4];
  sprintf( fileName, "%d_%d_%d.gz", angle, distance, average);
  
  fprintf(stderr, "Dump %s.\n", fileName);
  if (( zFile = gzopen( fileName, "w")) == NULL) {
    fprintf(stderr, "ERROR: Could not open file %s!\n", fileName);
    exit(0);
  }
  
  gzprintf( zFile, "%s %d\n", NUMBER_KEY_WORD, set->numberOfSamples);
  gzprintf( zFile, "%s 3\n", DIMENSION_KEY_WORD);
  gzprintf( zFile, "%s %d\n", NUMBER_INTEGRATIONS_KEY_WORD, set->numberOfIntegrations);
  gzprintf( zFile, "%s %f %f %f\n", MIN_KEY_WORD, 0.0, 0.0, 0.0);
  gzprintf( zFile, "%s %f %f %f\n", MAX_KEY_WORD, map->maxRealX, map->maxRealY, DEG_360);
  gzprintf( zFile, "%s %f %f %f\n", REF_POS_KEY_WORD, 0.0, 0.0, 0.0);
  gzprintf( zFile, "%s %ld %ld\n", TIME_STAMP_KEY_WORD, (long int) 0, (long int) 0);
  
  for ( s = 0; s < set->numberOfSamples; s++) {
#ifdef OLD
    gzprintf( zFile, "%d %d %f %f\n", 
	      set->position[s].x * map->resolution,
	      set->position[s].y * map->resolution,
	      deg2Rad(set->position[s].rot),
	      set->weight[s]);
#else
    dataVec[0] = set->position[s].x * map->resolution;
    dataVec[1] = set->position[s].y * map->resolution;
    dataVec[2] = deg2Rad(set->position[s].rot);
    dataVec[3] = set->weight[s];
    gzwrite( zFile, dataVec, sizeof(float) * 4);
#endif
  }
  gzclose ( zFile);
}


#define INTEGRATE_SCANS
#ifdef INTEGRATE_SCANS

static void
precomputePOfXGivenO( probabilityGrid* map,
		      probabilityGrid* aPrioriProbs,
		      expectedDistTable *distTab,
		      probability** distProbTable,
		      sensing_PROXIMITY* measuredFrontScan,
		      sensing_PROXIMITY* measuredRearScan)
{
  if ( aPrioriProbs->freeSpace == NULL) {
    fprintf( stderr, "Precomputation only works with free space.\n");
    exit(0);
  }
  else {
    
    static bool firstTime = TRUE;
    
    static gridSampleSet* featureMatrix[NUMBER_OF_ANGLE_FEATURES][NUMBER_OF_DISTANCE_FEATURES][NUMBER_OF_AVERAGE_FEATURES];
    
    /* This is the matrix with the feature combinations. */
    float normalizeFactorScan = 0.0;
    smallGridPosition* positions;
    static float* weights;
    static float* scanWeights;
    int generateScans = GENERATE_SCANS;

    float distanceResolution = (MAX_DIST - MIN_DIST)   / (NUMBER_OF_DISTANCE_FEATURES - 2);  
    float angleResolution    = (MAX_ANGLE - MIN_ANGLE) / (NUMBER_OF_ANGLE_FEATURES - 2);
    float averageResolution  = (MAX_AVERAGE - MIN_AVERAGE) / (NUMBER_OF_AVERAGE_FEATURES - 2);
    scanFeature feature;
    
    int beam, s, distanceFeature, angleFeature, averageFeature;
    
    int posX, posY, posRot;
	
    static int frontBeamAngle[NUMBER_OF_BEAMS];
    static int rearBeamAngle[NUMBER_OF_BEAMS];

    /* Fast lookup of sinus and cosinus for the discrete directions. */
    static float cosinus[NUMBER_OF_DIRECTIONS];
    static float sinus[NUMBER_OF_DIRECTIONS];

    /* Lookup of the offset of the scan center if the robot looks into a certain direction. */
    static int frontOffsetX[NUMBER_OF_DIRECTIONS];
    static int frontOffsetY[NUMBER_OF_DIRECTIONS];
    static int rearOffsetX[NUMBER_OF_DIRECTIONS];
    static int rearOffsetY[NUMBER_OF_DIRECTIONS];
    
    static int pos = 0;
    
    int expectedFrontScanFeatures[NUMBER_OF_BEAMS];
    int expectedRearScanFeatures[NUMBER_OF_BEAMS];

    int scanRotation = 0, generateMoreScans = TRUE;
    float tmpScan[NUMBER_OF_DIRECTIONS];
    
    expectedDistance*** expDistTab = distTab->dist;
    
    /* -------------------------------------------------------------
     * Initialize the scan.
     ------------------------------------------------------------- */
    
    if ( firstTime) {

      firstTime = FALSE;
      
      scanWeights = (float*) malloc( NUMBER_OF_SAMPLES_PER_POSITION * sizeof(float));
      dumpFeatureDefinitions();
      
      for ( beam = 0; beam < NUMBER_OF_DIRECTIONS; beam++) {
	cosinus[beam] = cos( deg2Rad(beam));
	sinus[beam] = sin( deg2Rad(beam));
      }

      /* Initialization of feature matrix. */
      for ( angleFeature = 0; angleFeature < NUMBER_OF_ANGLE_FEATURES; angleFeature++) 
	
	for ( distanceFeature = 0; distanceFeature < NUMBER_OF_DISTANCE_FEATURES; distanceFeature++)
	  for ( averageFeature = 0; averageFeature < NUMBER_OF_AVERAGE_FEATURES; averageFeature++) 
	    featureMatrix[angleFeature][distanceFeature][averageFeature] = NULL;

	/* Fast lookup of beam angles in degree. */
	for ( beam = 0; beam < NUMBER_OF_BEAMS; beam++) {
	  frontBeamAngle[beam] = (270 + beam) % 360;
	  rearBeamAngle[beam] = (90 + beam) % 360;
	}
	
#define CONSIDER_OFFSET
#ifdef CONSIDER_OFFSET
      {
	float halfCellSize = map->resolution * 0.5;
	float cellSize     = map->resolution;
	float dx, dy, xOffset, yOffset;
	int p,fx, fy, rx, ry;
	int newNumberOfCells;
	
	/* Fast lookup of beam angles in degree. */
	for ( beam = 0; beam < NUMBER_OF_BEAMS; beam++) {
	  
	  /* The offset of the scanner in grid cells, given the robot orientation. */
	  dx = cosinus[beam] * measuredFrontScan->offset.forward;
	  xOffset = (dx > 0) ? halfCellSize : -halfCellSize;
	  frontOffsetX[beam] = (int) ((dx + xOffset) / cellSize);

	  dy = sinus[beam] * measuredFrontScan->offset.forward;
	  yOffset = (dy > 0) ? halfCellSize : -halfCellSize;
	  frontOffsetY[beam] = (int) ((dy + yOffset) / cellSize);
	
	  dx = cosinus[beam] * measuredRearScan->offset.forward;
	  xOffset = (dx > 0) ? halfCellSize : -halfCellSize;
	  rearOffsetX[beam] = (int) ((dx + xOffset) / cellSize);

	  dy = sinus[beam] * measuredRearScan->offset.forward;
	  yOffset = (dy > 0) ? halfCellSize : -halfCellSize;
	  rearOffsetY[beam] = (int) ((dy + yOffset) / cellSize);
	
	  if (0) fprintf(stderr, "%d: [%d %d] [%d %d]\n", beam,
			 frontOffsetX[beam], frontOffsetY[beam], rearOffsetX[beam], rearOffsetY[beam]);
	}
	
	/* Sanity check. */
	writeLog( "# Checking consistency ... ");
	fprintf( stderr, "# Checking consistency ... ");
	for ( p = 0, newNumberOfCells = 0; p < aPrioriProbs->numberOfFreeCells; p++) {

	  int cellOk = TRUE;
	  
	  for ( beam = 0; beam < NUMBER_OF_BEAMS; beam++) {
	    fx = aPrioriProbs->freeSpace[p].x + frontOffsetX[beam];
	    rx = aPrioriProbs->freeSpace[p].x + rearOffsetX[beam];
	    fy = aPrioriProbs->freeSpace[p].y + frontOffsetY[beam];
	    ry = aPrioriProbs->freeSpace[p].y + rearOffsetY[beam];
	    if ( fx >= aPrioriProbs->sizeX || fx < 0 ||
		 fy >= aPrioriProbs->sizeY || fy < 0 ||
		 rx >= aPrioriProbs->sizeX || rx < 0 ||
		 ry >= aPrioriProbs->sizeY || ry < 0 ||
		 aPrioriProbs->prob[fx][fy] < 0.9 ||
		 aPrioriProbs->prob[rx][ry] < 0.9) {
	      if ( cellOk)
		fprintf(stderr, "%d %d %d %d  : %d %d\n", fx, rx, fy, ry, aPrioriProbs->sizeX, aPrioriProbs->sizeY);
	      cellOk = FALSE;
	      /* exit(0); */
	    }
	  }
	  /* If it's not ok just copy the next one into it. */
	  if ( cellOk) {
	    if ( newNumberOfCells != p)
	      aPrioriProbs->freeSpace[newNumberOfCells] = aPrioriProbs->freeSpace[p];
	    newNumberOfCells++;
	  }
	}
	fprintf( stderr, "done, deleted %d cells.\n",
		  aPrioriProbs->numberOfFreeCells - newNumberOfCells);
	writeLog( "done, deleted %d cells.\n",
		  aPrioriProbs->numberOfFreeCells - newNumberOfCells);
	aPrioriProbs->numberOfFreeCells = newNumberOfCells;
      }
#else
      measuredFrontScan->offset.forward = 0.0;   
      measuredRearScan->offset.forward = 0.0;   
#endif
    }

    do {

      gridSampleSet* currentSet;
      
      /* Generate a scan at a randomly drawn position. */
      if ( generateScans) {
	
	int index = randMax( aPrioriProbs->numberOfFreeCells-1);
	
	posX   = aPrioriProbs->freeSpace[index].x;
	posY   = aPrioriProbs->freeSpace[index].y;
	posRot = randMax( 359);
	
	/* Generate the front scan for this position. */
	for ( beam = 0; beam < measuredFrontScan->numberOfReadings; beam++) {
	  
	  int beamRot = frontBeamAngle[beam];
	  int globalRot = (beamRot + posRot) % 360;
#ifdef CONSIDER_OFFSET
	  int distIndex =
	    distTab->dist[posX + frontOffsetX[posRot]][posY + frontOffsetY[posRot]][globalRot];
#else
	  int distIndex = distTab->dist[posX][posY][globalRot];
#endif
	  
	  measuredFrontScan->reading[beam].dist = distIndex * distTab->distanceResolution;
	  
	  expectedFrontScanFeatures[beam] = distIndex;
	}
	
	/* Generate the rear scan for this position. */
	for ( beam = 0; beam < measuredRearScan->numberOfReadings; beam++) {
	  
	  int beamRot = rearBeamAngle[beam];
	  int globalRot = (beamRot + posRot) % 360;
#ifdef CONSIDER_OFFSET
	  int distIndex = 
	    distTab->dist[posX + rearOffsetX[posRot]][posY + rearOffsetY[posRot]][globalRot];
#else
	  int distIndex = distTab->dist[posX][posY][globalRot];
#endif
	  measuredRearScan->reading[beam].dist = distIndex * distTab->distanceResolution;
	  
	  expectedRearScanFeatures[beam] = distIndex;
	}
      }
      /* Extract the distance features from the measured distances. */
      else {
	for ( beam = 0; beam < measuredFrontScan->numberOfReadings; beam++) {
	  expectedFrontScanFeatures[beam] = measuredFrontScan->reading[beam].dist / distTab->distanceResolution;
	  if ( measuredRearScan != NULL)
	    expectedRearScanFeatures[beam] = measuredRearScan->reading[beam].dist / distTab->distanceResolution;
	}

	/* Just for display. */
	posRot = 0;
	posX = map->sizeX / 2;
	posY = map->sizeY / 2;
      }
      
      feature = extractedScanFeature( measuredFrontScan, measuredRearScan,
				      frontBeamAngle, rearBeamAngle,
				      cosinus, sinus,
				      distanceResolution, angleResolution,
				      averageResolution);
      
      
      /* This is the sample set that represents the density of this feature. */
      if ( featureMatrix[feature.matrixAngle][feature.matrixDistance][feature.matrixAverage] == NULL) {
	
	writeLog( "%d %d %d #feat new\n",
		  feature.matrixAngle, feature.matrixDistance, feature.matrixAverage);
	
	featureMatrix[feature.matrixAngle][feature.matrixDistance][feature.matrixAverage] =
	  setForPreprocessing( NUMBER_OF_SAMPLES_PER_POSITION,
			       aPrioriProbs, measuredFrontScan, measuredRearScan);
	if ( featureMatrix[feature.matrixAngle][feature.matrixDistance][feature.matrixAverage] == NULL) {
	  for ( angleFeature = 0; angleFeature < NUMBER_OF_ANGLE_FEATURES; angleFeature++)
	    for ( distanceFeature = 0; distanceFeature < NUMBER_OF_DISTANCE_FEATURES; distanceFeature++)
	      for ( averageFeature = 0; averageFeature < NUMBER_OF_AVERAGE_FEATURES; averageFeature++)
		if ( featureMatrix[angleFeature][distanceFeature][averageFeature] != NULL)
		  dumpDensitySet( featureMatrix[angleFeature][distanceFeature][averageFeature],
				  angleFeature, distanceFeature, averageFeature, map);
	  exit(0);
	}
      }
      else	
	writeLog( "%d %d %d #feat\n",
		  feature.matrixAngle, feature.matrixDistance, feature.matrixAverage);
      
      currentSet = featureMatrix[feature.matrixAngle][feature.matrixDistance][feature.matrixAverage];
      positions = currentSet->position;
      weights = currentSet->weight;
      currentSet->numberOfIntegrations++;
      
      normalizeFactorScan = 0.0;
      
      /* -------------------------------------------------------------
       * We have the expected scan and the corresponding sample set.
       * Now we have to compute the likelihoods of the scan at each position
       * in the set. 
       * ------------------------------------------------------------- */
      fprintf(stderr, "[ %d ", pos++);
      
      for ( s = 0; s < NUMBER_OF_SAMPLES_PER_POSITION; s++) {
	
	int robRot = positions[s].rot;
	
#ifdef CONSIDER_OFFSET
	expectedDistance* distTabXY = expDistTab[positions[s].x + frontOffsetX[robRot]][positions[s].y + frontOffsetY[robRot]];
#else
	expectedDistance* distTabXY = expDistTab[positions[s].x][positions[s].y];
#endif
	
	scanWeights[s] = 1.0;
	
	/* Integrate the scan. */
	for ( beam = 0; beam < measuredFrontScan->numberOfReadings; beam += BEAM_SKIP) {
	  
	  int globalRot = (robRot + frontBeamAngle[beam]) % 360;	  
	  
	  scanWeights[s] *= distProbTable[distTabXY[globalRot]][expectedFrontScanFeatures[beam]];
	}
	
	if ( measuredRearScan != NULL) {	
	  
#ifdef CONSIDER_OFFSET
	  distTabXY = expDistTab[positions[s].x + rearOffsetX[robRot]][positions[s].y + rearOffsetY[robRot]];
#else
	  distTabXY = expDistTab[positions[s].x][positions[s].y];
#endif
	  
	  /* Integrate the scan. */
	  for ( beam = 0; beam < measuredRearScan->numberOfReadings; beam += BEAM_SKIP) {
	    
	    int globalRot = (robRot + rearBeamAngle[beam]) % 360;	  
	    scanWeights[s] *= distProbTable[distTabXY[globalRot]][expectedRearScanFeatures[beam]];
	  }
	}
	normalizeFactorScan += scanWeights[s];	  
      }
      
      /* Normalize the scan and the average sample set. */
      for ( s = 0; s < NUMBER_OF_SAMPLES_PER_POSITION; s++) {
	weights[s] += scanWeights[s] / normalizeFactorScan;
      }
      
      fprintf(stderr, "]");
      
      /* ----------------------------------------------------------------
       * Display the position. 
       * ------------------------------------------------------------- */
      if (0) {
	static gridWindow *window;
	static sampleSet displaySet;
	static positionWindow* posWindow;
	static int first = TRUE;

	if ( first) {
	  /* Display stuff. */
	  window = createMapWindow( map, "TEST", 0, 0, 1);
	  displaySet = initializedSamples( NUMBER_OF_SAMPLES_PER_POSITION, 
					   0.0, 0.0, 
					   map->maxRealX, 
					   map->maxRealY);
	  
	  posWindow = createPositionWindow( &displaySet, "Samples",
					    200, 0, 15);

	  first = FALSE;
	}
	
	{
	  float factor = (float) window->scale / window->gridResolution;
	  int robX = posX * map->resolution * factor;
	  int robY = (window->gridSizeY - posY * map->resolution / window->gridResolution) * window->scale;

#ifdef CONSIDER_OFFSET
	  int frontScannerX = posX + frontOffsetX[posRot];
	  int frontScannerY = posY + frontOffsetY[posRot];
#else
	  int frontScannerX = posX;
	  int frontScannerY = posY;
#endif
	  int frontScannerXWin = frontScannerX * map->resolution * factor;
	  int frontScannerYWin = (window->gridSizeY - frontScannerY * map->resolution / window->gridResolution) * window->scale;
	  
	  int cX = ((posX * map->resolution) + cos(feature.angle + deg2Rad(posRot)) * feature.distance) * factor;
	  int cY = (window->gridSizeY - (posY * map->resolution + sin(feature.angle + deg2Rad(posRot)) * feature.distance) / window->gridResolution) * window->scale;
	  int s;
	  displayMapWindow( map, window);
	  EZX_SetColor(C_BLUE);
	  EZX_SetLineWidth(1);

	  for ( s = 0; s < measuredFrontScan->numberOfReadings; s+= 2) {
	    
	    int globalRot = (posRot + frontBeamAngle[s]) % 360;
	    
	    int endX = map->resolution * frontScannerX + cosinus[globalRot] * 
	      (expectedFrontScanFeatures[s] *  distTab->distanceResolution);
	    
	    int endY = (window->gridSizeY - (map->resolution * frontScannerY + sinus[globalRot] *
					     (expectedFrontScanFeatures[s] *  distTab->distanceResolution)) / window->gridResolution) * window->scale;
	    endX *= factor;
	    EZX_DrawLine(window->window, frontScannerXWin, frontScannerYWin, endX, endY);
	  }
	  
	  if ( measuredRearScan != NULL) {

#ifdef CONSIDER_OFFSET
	    int rearScannerX = posX + rearOffsetX[posRot];
	    int rearScannerY = posY + rearOffsetY[posRot];
#else
	    int rearScannerX = posX;
	    int rearScannerY = posY;
#endif
	    
	    int rearScannerXWin = rearScannerX * map->resolution * factor;
	    int rearScannerYWin = (window->gridSizeY - rearScannerY * map->resolution / window->gridResolution) * window->scale;
	    
	    for ( s = 0; s < measuredRearScan->numberOfReadings; s+= 2) {
	      
	      int globalRot = (posRot + rearBeamAngle[s]) % 360;
	      
	      int endX = map->resolution * rearScannerX + cosinus[globalRot] * 
		(expectedRearScanFeatures[s] *  distTab->distanceResolution);
	      int endY = (window->gridSizeY - (map->resolution * rearScannerY + sinus[globalRot] *
					       (expectedRearScanFeatures[s] *  distTab->distanceResolution)) / window->gridResolution) * window->scale;
	      
	      endX *= factor;
	      EZX_DrawLine(window->window, rearScannerXWin, rearScannerYWin, endX, endY);
	    }
	    EZX_SetColor(C_LAWNGREEN);
	    EZX_FillCircle(window->window, rearScannerXWin, rearScannerYWin, 55 / (float) window->gridResolution * (float) window->scale);
	  }
	  
	  EZX_SetColor(C_YELLOW);
	  EZX_FillCircle(window->window, robX, robY, 135 / (float) window->gridResolution * (float) window->scale);
	  EZX_SetColor(C_LAWNGREEN);
	  EZX_FillCircle(window->window, frontScannerXWin, frontScannerYWin, 55 / (float) window->gridResolution * (float) window->scale);
	  EZX_SetColor(C_RED);
	  EZX_FillCircle(window->window, cX, cY, 5);
	  EZX_Flush();
	  
	  for ( s = 0; s < NUMBER_OF_SAMPLES_PER_POSITION; s++) {
	    displaySet.sample[s].pos.x = currentSet->position[s].x * map->resolution;
	    displaySet.sample[s].pos.y = currentSet->position[s].y * map->resolution;
	    displaySet.sample[s].weight = weights[s];
	  }
	  
	  displaySet.alreadySampled = FALSE;
	  displaySet.alreadyNormalized = FALSE;
	  
	  lowVarianceResample( &displaySet, aPrioriProbs);
	  
	  for ( s = 0; s < NUMBER_OF_SAMPLES_PER_POSITION; s++) {
	    displaySet.sample[s].pos.x += 10 * randomGauss();
	    displaySet.sample[s].pos.y += 10 * randomGauss();
	  }
	  displayPositions( &displaySet, map, NULL, posWindow, 1, 0);
	}
      }
      
      /* Dump the sample sets. */
      if ( pos % 5000 == 0)
	for ( angleFeature = 0; angleFeature < NUMBER_OF_ANGLE_FEATURES; angleFeature++)
	  for ( distanceFeature = 0; distanceFeature < NUMBER_OF_DISTANCE_FEATURES; distanceFeature++)
	    for ( averageFeature = 0; averageFeature < NUMBER_OF_AVERAGE_FEATURES; averageFeature++)
	      if ( featureMatrix[angleFeature][distanceFeature][averageFeature] != NULL)
		dumpDensitySet( featureMatrix[angleFeature][distanceFeature][averageFeature],
				angleFeature, distanceFeature, averageFeature, map);
      /* getchar(); */

      if ( ! generateScans) {

	if ( scanRotation == 0) {
	  for ( beam = 0; beam < measuredFrontScan->numberOfReadings; beam++) 
	    tmpScan[beam] = measuredFrontScan->reading[beam].dist;
	  for ( beam = 0; beam < measuredRearScan->numberOfReadings; beam++) 
	    tmpScan[beam + measuredFrontScan->numberOfReadings] = measuredRearScan->reading[beam].dist;
	}

	scanRotation += 20;

	/* ddd don't rotate scan */
	if ( 0 && scanRotation < 360) {
	  
	  /* Just rotate the scan. */
	  for ( beam = 0; beam < measuredFrontScan->numberOfReadings; beam++) 
	    measuredFrontScan->reading[beam].dist =
	      tmpScan[(beam + scanRotation) % NUMBER_OF_DIRECTIONS];

	  for ( beam = 0; beam < measuredRearScan->numberOfReadings; beam++) {
	    measuredRearScan->reading[beam].dist =
	      tmpScan[(measuredFrontScan->numberOfReadings + beam + scanRotation) % NUMBER_OF_DIRECTIONS];
	  }
	  generateMoreScans = TRUE;
	}
	else {
	  /* fprintf(stderr, "\nSCAN no. %d\n", pos / 18); */
	  generateMoreScans = FALSE;
	}
      }
      else
	generateMoreScans = TRUE;
    } while ( generateMoreScans);
  }
}








#else

void
insertIntoSet( int posX, int posY, int posRot, gridSampleSet* currentSet)
{
  if ( currentSet->numberOfIntegrations < currentSet->numberOfSamples) {
    int index = currentSet->numberOfIntegrations;
    currentSet->position[index].x = posX;
    currentSet->position[index].y = posY;
    currentSet->position[index].rot = posRot;
    currentSet->weight[index] = 1.0 / currentSet->numberOfSamples;
  }
  currentSet->numberOfIntegrations++;
}
  

/* A beam is given by an expected distance. */
  float
sampleFromBeam( int expectedDistIndex,
		float distanceResolution,
		int numberOfDistances,
		probability** distProbTable)
{
  static double** beamMatrix = NULL;
  double* beamProb;
  double randVal;
  int distIndex;

  if ( ! beamMatrix) {

    int distCnt, beamCnt;
    
    beamMatrix = (double**) allocate2D( numberOfDistances, numberOfDistances, DOUBLE);

    /* Beam counts over all possible expected distances. */
    for ( beamCnt = 0; beamCnt < numberOfDistances; beamCnt++) {
      float probSum = 0.0;
      double* beamProb = beamMatrix[beamCnt];
      for ( distCnt = 0; distCnt < numberOfDistances; distCnt++) {
	probSum += distProbTable[beamCnt][distCnt];
      }
      beamProb[0] = distProbTable[beamCnt][0] / probSum;
      for ( distCnt = 1; distCnt < numberOfDistances; distCnt++) {
	beamProb[distCnt] = beamProb[distCnt-1] + distProbTable[beamCnt][distCnt] / probSum;
      }
    }
  }

  beamProb = beamMatrix[expectedDistIndex];
  randVal = RAND_ZERO_TO_ONE();
  
  distIndex = binarySearch( beamProb, randVal, numberOfDistances);

  return ( distIndex * distanceResolution);
}

static void
precomputePOfXGivenO( probabilityGrid* map,
		      probabilityGrid* aPrioriProbs,
		      expectedDistTable *distTab,
		      probability** distProbTable,
		      sensing_PROXIMITY* measuredFrontScan,
		      sensing_PROXIMITY* measuredRearScan)
{

  if ( aPrioriProbs->freeSpace == NULL) {
    fprintf( stderr, "Precomputation only works with free space.\n");
    exit(0);
  }
  else {
    
    static bool firstTime = TRUE;
    
    static gridSampleSet* featureMatrix[NUMBER_OF_ANGLE_FEATURES][NUMBER_OF_DISTANCE_FEATURES][NUMBER_OF_AVERAGE_FEATURES];
    
    /* This is the matrix with the feature combinations. */
    float normalizeFactorScan = 0.0;
    smallGridPosition* positions;

    float distanceResolution = (MAX_DIST - MIN_DIST)   / (NUMBER_OF_DISTANCE_FEATURES - 2);  
    float angleResolution    = (MAX_ANGLE - MIN_ANGLE) / (NUMBER_OF_ANGLE_FEATURES - 2);
    float averageResolution  = (MAX_AVERAGE - MIN_AVERAGE) / (NUMBER_OF_AVERAGE_FEATURES - 2);
    scanFeature feature;
    
    int beam, s, distanceFeature, angleFeature, averageFeature;
    
    int posX, posY, posRot;
	
    static int frontBeamAngle[NUMBER_OF_BEAMS];
    static int rearBeamAngle[NUMBER_OF_BEAMS];

    /* Fast lookup of sinus and cosinus for the discrete directions. */
    static float cosinus[NUMBER_OF_DIRECTIONS];
    static float sinus[NUMBER_OF_DIRECTIONS];

    /* Lookup of the offset of the scan center if the robot looks into a certain direction. */
    static int frontOffsetX[NUMBER_OF_DIRECTIONS];
    static int frontOffsetY[NUMBER_OF_DIRECTIONS];
    static int rearOffsetX[NUMBER_OF_DIRECTIONS];
    static int rearOffsetY[NUMBER_OF_DIRECTIONS];
    
    static int pos = 0;
    
    int expectedFrontScanFeatures[NUMBER_OF_BEAMS];
    int expectedRearScanFeatures[NUMBER_OF_BEAMS];

    int scanRotation = 0, generateMoreScans = TRUE;
    float tmpScan[NUMBER_OF_DIRECTIONS];

    int posIndex = 0, rotIndex = 0;
    
    expectedDistance*** expDistTab = distTab->dist;
    
    /* -------------------------------------------------------------
     * Initialize the scan.
     ------------------------------------------------------------- */
    
    if ( firstTime) {

      firstTime = FALSE;
      
      dumpFeatureDefinitions();
      
      for ( beam = 0; beam < NUMBER_OF_DIRECTIONS; beam++) {
	cosinus[beam] = cos( deg2Rad(beam));
	sinus[beam] = sin( deg2Rad(beam));
      }

      /* Initialization of feature matrix. */
      for ( angleFeature = 0; angleFeature < NUMBER_OF_ANGLE_FEATURES; angleFeature++) 
	
	for ( distanceFeature = 0; distanceFeature < NUMBER_OF_DISTANCE_FEATURES; distanceFeature++)
	  for ( averageFeature = 0; averageFeature < NUMBER_OF_AVERAGE_FEATURES; averageFeature++) 
	    featureMatrix[angleFeature][distanceFeature][averageFeature] = NULL;

      /* Fast lookup of beam angles in degree. */
      for ( beam = 0; beam < NUMBER_OF_BEAMS; beam++) {
	frontBeamAngle[beam] = (270 + beam) % 360;
	rearBeamAngle[beam] = (90 + beam) % 360;
      }
	
#define CONSIDER_OFFSET
#ifdef CONSIDER_OFFSET
      {
	float halfCellSize = map->resolution * 0.5;
	float cellSize     = map->resolution;
	float dx, dy, xOffset, yOffset;
	int p,fx, fy, rx, ry;

	/* Fast lookup of beam angles in degree. */
	for ( beam = 0; beam < NUMBER_OF_BEAMS; beam++) {
	  
	  /* The offset of the scanner in grid cells, given the robot orientation. */
	  dx = cosinus[beam] * measuredFrontScan->offset.forward;
	  xOffset = (dx > 0) ? halfCellSize : -halfCellSize;
	  frontOffsetX[beam] = (int) ((dx + xOffset) / cellSize);

	  dy = sinus[beam] * measuredFrontScan->offset.forward;
	  yOffset = (dy > 0) ? halfCellSize : -halfCellSize;
	  frontOffsetY[beam] = (int) ((dy + yOffset) / cellSize);
	
	  dx = cosinus[beam] * measuredRearScan->offset.forward;
	  xOffset = (dx > 0) ? halfCellSize : -halfCellSize;
	  rearOffsetX[beam] = (int) ((dx + xOffset) / cellSize);

	  dy = sinus[beam] * measuredRearScan->offset.forward;
	  yOffset = (dy > 0) ? halfCellSize : -halfCellSize;
	  rearOffsetY[beam] = (int) ((dy + yOffset) / cellSize);
	
	  if (0) fprintf(stderr, "%d: [%d %d] [%d %d]\n", beam,
			 frontOffsetX[beam], frontOffsetY[beam], rearOffsetX[beam], rearOffsetY[beam]);
	}
	
	/* Sanity check. */
	writeLog( "# Checking consistency ... ");
	for ( p = 0; p < aPrioriProbs->numberOfFreeCells; p++) {
	  for ( beam = 0; beam < NUMBER_OF_BEAMS; beam++) {
	    fx = aPrioriProbs->freeSpace[p].x + frontOffsetX[beam];
	    rx = aPrioriProbs->freeSpace[p].x + rearOffsetX[beam];
	    fy = aPrioriProbs->freeSpace[p].y + frontOffsetY[beam];
	    ry = aPrioriProbs->freeSpace[p].y + rearOffsetY[beam];
	    if ( fx >= aPrioriProbs->sizeX || fx < 0 ||
		 fy >= aPrioriProbs->sizeY || fy < 0 ||
		 rx >= aPrioriProbs->sizeX || rx < 0 ||
		 ry >= aPrioriProbs->sizeY || ry < 0) {
	      fprintf(stderr, "%d %d %d %d  : %d %d\n", fx, rx, fy, ry, aPrioriProbs->sizeX, aPrioriProbs->sizeY);
	      exit(0);
	    }
	  }
	}
	writeLog( "done \n");
      }
#else
      measuredFrontScan->offset.forward = 0.0;   
      measuredRearScan->offset.forward = 0.0;   
#endif
    }

    do {
      
      gridSampleSet* currentSet;
      
      posX = aPrioriProbs->freeSpace[pos].x;
      posY = aPrioriProbs->freeSpace[pos].y;

      fprintf(stderr, "[ %d ", pos++);

      for ( posRot = 0; posRot < 360; posRot += 2) {
	
      /* Generate a scan at a randomly drawn position. */
/*        int index = randMax( aPrioriProbs->numberOfFreeCells-1); */
      
/*        posX   = aPrioriProbs->freeSpace[index].x; */
/*        posY   = aPrioriProbs->freeSpace[index].y; */
/*        posRot = randMax( 359); */

      
      
	/* Generate the front scan for this position. */
	for ( beam = 0; beam < measuredFrontScan->numberOfReadings; beam++) {
	
	int beamRot = frontBeamAngle[beam];
	int globalRot = (beamRot + posRot) % 360;
#ifdef CONSIDER_OFFSET
	int distIndex =
	  distTab->dist[posX + frontOffsetX[posRot]][posY + frontOffsetY[posRot]][globalRot];
#else
	int distIndex = distTab->dist[posX][posY][globalRot];
#endif
	
	/*  	measuredFrontScan->reading[beam].dist = sampleFromBeam( distIndex, */
	/*  								distTab->distanceResolution, */
	/*  								distTab->numberOfExpectedDistances, */
	/*  								distProbTable);  */
	measuredFrontScan->reading[beam].dist = distIndex * distTab->distanceResolution;
	
	expectedFrontScanFeatures[beam] = distIndex;
      }
      
      /* Generate the rear scan for this position. */
      for ( beam = 0; beam < measuredRearScan->numberOfReadings; beam++) {
	
	int beamRot = rearBeamAngle[beam];
	int globalRot = (beamRot + posRot) % 360;
#ifdef CONSIDER_OFFSET
	int distIndex = 
	  distTab->dist[posX + rearOffsetX[posRot]][posY + rearOffsetY[posRot]][globalRot];
#else
	int distIndex = distTab->dist[posX][posY][globalRot];
#endif
	/*  	measuredRearScan->reading[beam].dist = sampleFromBeam( distIndex, */
	/*  							       distTab->distanceResolution, */
	/*  							       distTab->numberOfExpectedDistances, */
	/*  							       distProbTable);  */
	
	measuredRearScan->reading[beam].dist = distIndex * distTab->distanceResolution;
	
	expectedRearScanFeatures[beam] = distIndex;
      }
      
      feature = extractedScanFeature( measuredFrontScan, measuredRearScan,
				      frontBeamAngle, rearBeamAngle,
				      cosinus, sinus,
				      distanceResolution, angleResolution,
				      averageResolution);
      

      /* This is the sample set that represents the density of this feature. */
      if ( featureMatrix[feature.matrixAngle][feature.matrixDistance][feature.matrixAverage] == NULL) {
	
	writeLog( "%d %d %d #feat new\n",
		  feature.matrixAngle, feature.matrixDistance, feature.matrixAverage);
	
	featureMatrix[feature.matrixAngle][feature.matrixDistance][feature.matrixAverage] =
	  setForPreprocessing( NUMBER_OF_SAMPLES_PER_POSITION,
			       aPrioriProbs, measuredFrontScan, measuredRearScan);

	if ( featureMatrix[feature.matrixAngle][feature.matrixDistance][feature.matrixAverage] == NULL) {
	  fprintf( stderr, "Error: Couldn't initialize feature set.\n");
	  for ( angleFeature = 0; angleFeature < NUMBER_OF_ANGLE_FEATURES; angleFeature++)
	    for ( distanceFeature = 0; distanceFeature < NUMBER_OF_DISTANCE_FEATURES; distanceFeature++)
	      for ( averageFeature = 0; averageFeature < NUMBER_OF_AVERAGE_FEATURES; averageFeature++)
		if ( featureMatrix[angleFeature][distanceFeature][averageFeature] != NULL)
		  dumpDensitySet( featureMatrix[angleFeature][distanceFeature][averageFeature],
				  angleFeature, distanceFeature, averageFeature, map);
	  exit(0);
	}
      }
      else	
	writeLog( "%d %d %d #feat\n",
		  feature.matrixAngle, feature.matrixDistance, feature.matrixAverage);
    
      currentSet = featureMatrix[feature.matrixAngle][feature.matrixDistance][feature.matrixAverage];

      insertIntoSet( posX, posY, posRot, currentSet);
    
      /* ----------------------------------------------------------------
       * Display the position. 
       * ------------------------------------------------------------- */
      if (0) {
	static gridWindow *window;
	static sampleSet displaySet;
	static positionWindow* posWindow;
	static int first = TRUE;
      
	if ( first) {
	  /* Display stuff. */
	  window = createMapWindow( map, "TEST", 0, 0, 1);
	  displaySet = initializedSamples( NUMBER_OF_SAMPLES_PER_POSITION, 
					   0.0, 0.0, 
					   map->maxRealX, 
					   map->maxRealY);
	
	  posWindow = createPositionWindow( &displaySet, "Samples",
					    200, 0, 15);
	
	  first = FALSE;
	}
      
	{
	  float factor = (float) window->scale / window->gridResolution;
	  int robX = posX * map->resolution * factor;
	  int robY = (window->gridSizeY - posY * map->resolution / window->gridResolution) * window->scale;
	
#ifdef CONSIDER_OFFSET
	  int frontScannerX = posX + frontOffsetX[posRot];
	  int frontScannerY = posY + frontOffsetY[posRot];
#else
	  int frontScannerX = posX;
	  int frontScannerY = posY;
#endif
	  int frontScannerXWin = frontScannerX * map->resolution * factor;
	  int frontScannerYWin = (window->gridSizeY - frontScannerY * map->resolution / window->gridResolution) * window->scale;
	
	  int cX = ((posX * map->resolution) + cos(feature.angle + deg2Rad(posRot)) * feature.distance) * factor;
	  int cY = (window->gridSizeY - (posY * map->resolution + sin(feature.angle + deg2Rad(posRot)) * feature.distance) / window->gridResolution) * window->scale;
	  int s;
	  displayMapWindow( map, window);
	  EZX_SetColor(C_BLUE);
	  EZX_SetLineWidth(1);
	
	  for ( s = 0; s < measuredFrontScan->numberOfReadings; s++) {
	  
	    int globalRot = (posRot + frontBeamAngle[s]) % 360;
	  
	    int endX = map->resolution * frontScannerX + cosinus[globalRot] * 
	      (expectedFrontScanFeatures[s] *  distTab->distanceResolution);
	  
	    int endY = (window->gridSizeY - (map->resolution * frontScannerY + sinus[globalRot] *
					     (expectedFrontScanFeatures[s] *  distTab->distanceResolution)) / window->gridResolution) * window->scale;
	    endX *= factor;
	    EZX_DrawLine(window->window, frontScannerXWin, frontScannerYWin, endX, endY);
	  }
	
	  if ( measuredRearScan != NULL) {
	  
#ifdef CONSIDER_OFFSET
	    int rearScannerX = posX + rearOffsetX[posRot];
	    int rearScannerY = posY + rearOffsetY[posRot];
#else
	    int rearScannerX = posX;
	    int rearScannerY = posY;
#endif
	  
	    int rearScannerXWin = rearScannerX * map->resolution * factor;
	    int rearScannerYWin = (window->gridSizeY - rearScannerY * map->resolution / window->gridResolution) * window->scale;
	  
	    for ( s = 0; s < measuredRearScan->numberOfReadings; s++) {
	    
	      int globalRot = (posRot + rearBeamAngle[s]) % 360;
	      
	      int endX = map->resolution * rearScannerX + cosinus[globalRot] * 
		(expectedRearScanFeatures[s] *  distTab->distanceResolution);
	      int endY = (window->gridSizeY - (map->resolution * rearScannerY + sinus[globalRot] *
					       (expectedRearScanFeatures[s] *  distTab->distanceResolution)) / window->gridResolution) * window->scale;
	      
	      endX *= factor;
	      EZX_DrawLine(window->window, rearScannerXWin, rearScannerYWin, endX, endY);
	    }
	    EZX_SetColor(C_LAWNGREEN);
	    EZX_FillCircle(window->window, rearScannerXWin, rearScannerYWin, 55 / (float) window->gridResolution * (float) window->scale);
	  }
	
	  EZX_SetColor(C_YELLOW);
	  EZX_FillCircle(window->window, robX, robY, 50 / (float) window->gridResolution * (float) window->scale);
	  EZX_SetColor(C_LAWNGREEN);
	  EZX_FillCircle(window->window, frontScannerXWin, frontScannerYWin, 35 / (float) window->gridResolution * (float) window->scale);
	  EZX_SetColor(C_RED);
	  EZX_FillCircle(window->window, cX, cY, 5);
	  EZX_Flush();
	  
	  for ( s = 0; s < currentSet->numberOfIntegrations; s++) {
	    displaySet.sample[s].pos.x = currentSet->position[s].x * map->resolution;
	    displaySet.sample[s].pos.y = currentSet->position[s].y * map->resolution;
	    displaySet.sample[s].weight = currentSet->weight[s];
	  }
	  
	  for ( s = 0; s < NUMBER_OF_SAMPLES_PER_POSITION; s++) {
	    displaySet.sample[s].pos.x += 10 * randomGauss();
	    displaySet.sample[s].pos.y += 10 * randomGauss();
	  }
	  displayPositions( &displaySet, map, NULL, posWindow, 1, 0);
	}
      }
    
      /* Dump the sample sets. */
      if ( pos == aPrioriProbs->numberOfFreeCells - 1) {
	fprintf( stderr, "DUMP\n\n");
	for ( angleFeature = 0; angleFeature < NUMBER_OF_ANGLE_FEATURES; angleFeature++)
	  for ( distanceFeature = 0; distanceFeature < NUMBER_OF_DISTANCE_FEATURES; distanceFeature++)
	    for ( averageFeature = 0; averageFeature < NUMBER_OF_AVERAGE_FEATURES; averageFeature++)
	      if ( featureMatrix[angleFeature][distanceFeature][averageFeature] != NULL)
		dumpDensitySet( featureMatrix[angleFeature][distanceFeature][averageFeature],
				angleFeature, distanceFeature, averageFeature, map);
      }
      /* getchar(); */

/*        } while (1);  */
      }
      fprintf(stderr, "]");
    } while (pos < aPrioriProbs->numberOfFreeCells); 
  }
}








#endif
