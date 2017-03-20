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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/sensings.h,v $
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
 * $Log: sensings.h,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.51  1999/11/02 18:12:37  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.50  1999/10/21 17:30:45  fox
 * Fixed some bugs in real time scripts.
 *
 * Revision 1.49  1999/09/09 02:48:38  fox
 * Final version before germany.
 *
 * Revision 1.48  1999/09/06 16:36:04  fox
 * Many changes.
 *
 * Revision 1.47  1999/09/01 00:02:58  fox
 * Getting closer. But still not close enough.
 *
 * Revision 1.46  1999/08/30 05:48:43  fox
 * Doesn't work!!
 *
 * Revision 1.45  1999/04/26 18:55:40  fox
 * Communication with sampling seems to work (no more stuck situations).
 *
 * Revision 1.44  1999/04/21 22:58:02  fox
 * First attempt to get samples from multi back.
 *
 * Revision 1.43  1999/04/21 14:06:01  fox
 * Just an intermediate version.
 *
 * Revision 1.42  1999/03/08 16:47:48  wolfram
 * Fixed problems with map, caused by the fact that it no longer accpets
 * correction parameters from LOCALIZE.  Improved the handling of sonars
 * with Pioneers.  Currently sonars do not work properly on a B21.
 *
 * Revision 1.41  1998/11/17 23:26:29  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.40  1998/11/03 21:02:23  fox
 * Lasers will not be integrated if the robot rotates faster than
 * MAX_ROT_VEL_FOR_LASERS deg/sec.
 *
 * Revision 1.39  1998/09/25 04:02:59  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.38  1998/09/19 13:07:44  wolfram
 * Added missing files
 *
 * Revision 1.37  1998/09/18 17:24:44  fox
 * Added skeleton files for condensation.
 *
 * Revision 1.36  1998/09/05 22:06:57  wolfram
 * Changes regarding vision!
 *
 * Revision 1.35  1998/08/31 22:29:24  wolfram
 * Several changes
 *
 * Revision 1.34  1998/08/24 07:39:51  wolfram
 * final version for Washington
 *
 * Revision 1.33  1998/08/23 00:01:05  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.32  1998/08/20 00:23:03  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.31  1998/06/30 13:55:11  fox
 * Updated question sensor.
 *
 * Revision 1.30  1998/03/17 08:41:13  wolfram
 * First steps to add vision.
 *
 * Revision 1.29  1998/02/12 15:47:24  derr
 * compiler warnings (unused variable ... ) removed
 *
 * Revision 1.28  1998/01/05 10:37:17  fox
 * Several changes especially in selection parts.
 *
 * Revision 1.27  1997/12/02 15:20:44  fox
 * Nothing remarkable.
 *
 * Revision 1.26  1997/11/28 13:34:38  fox
 * Added questions.
 *
 * Revision 1.25  1997/11/21 15:36:09  fox
 * Modifications in graphic
 *
 * Revision 1.24  1997/11/20 12:58:15  fox
 * Version with good sensor selection.
 *
 * Revision 1.23  1997/11/07 12:39:43  fox
 * Added some graphic features.
 *
 * Revision 1.22  1997/09/30 13:38:52  wolfram
 * Integration of angles is stopped after integration of any proximity sensor
 *
 * Revision 1.21  1997/09/11 20:01:05  fox
 * Final cmu version.
 *
 * Revision 1.20  1997/09/09 19:45:14  fox
 * Fixed a bug in finding local maxima and did several changes to active
 * localization.
 *
 * Revision 1.19  1997/08/22 04:16:41  fox
 * Final version before IJCAI.
 *
 * Revision 1.18  1997/06/25 14:16:43  fox
 * Changed laser incorporation.
 *
 * Revision 1.17  1997/06/20 07:36:15  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.16  1997/05/27 07:42:37  fox
 * Nothing special.
 *
 * Revision 1.15  1997/05/26 08:47:59  fox
 * Last version before major changes.
 *
 * Revision 1.14  1997/04/30 12:25:42  fox
 * Some minor changes.
 *
 * Revision 1.13  1997/04/24 21:25:44  wolfram
 * Ooops, fourth parameter of CUBE's is radius and not diameter!
 *
 * Revision 1.12  1997/04/02 08:57:35  fox
 * Display the robot right after sensor selection.
 *
 * Revision 1.11  1997/03/18 18:45:32  fox
 * First executable version with a global scan mask for different sensor types.
 *
 * Revision 1.10  1997/03/14 17:58:24  fox
 * This version should run quite stable now.
 *
 * Revision 1.9  1997/03/13 17:36:39  fox
 * Temporary version. Don't use!
 *
 * Revision 1.8  1997/02/11 11:04:11  wolfram
 * Faster Computation of expected distances, height of sensors is considered
 *
 * Revision 1.7  1997/01/29 12:23:15  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.6  1997/01/16 12:42:52  fox
 * Fixed a bug in scan masks.
 *
 * Revision 1.5  1997/01/07 09:55:49  fox
 * Added localMaxima to actionInfo.
 *
 * Revision 1.4  1997/01/03 10:09:48  fox
 * First version with exploration.
 *
 * Revision 1.3  1996/12/02 10:32:14  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.2  1996/11/15 17:44:09  ws
 * *** empty log message ***
 *
 * Revision 1.1.1.1  1996/09/22 16:46:33  rhino
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


#ifndef SENSINGS_INCLUDE
#define SENSINGS_INCLUDE

#include "general.h"
#include "correction.h"
#include "map.h"
#include "probGridTools.h"
#include "LOCALIZE-messages.h"


/*****************************************************
 * Different sensings to be included into the position
 * probability grid.
 *****************************************************/
#define MAX_NUMBER_OF_ACTIONS_PER_SENSING      5

#define MOVEMENT                0
#define SONAR                   1
#define ANGLE                   2
#define LASER                   3
#define QUESTION                4
#define VISION                  5
#define NUMBER_OF_SENSINGS      6

#ifndef SENSINGS_INCLUDE_SPECIAL_SCRIPT_DATA
#define SENSINGS_INCLUDE_SPECIAL_SCRIPT_DATA
typedef distanceScan sensing_PROXIMITY;
typedef distanceScan sensing_SONAR;
typedef movement sensing_MOVEMENT;
typedef answer sensing_QUESTION;
typedef cameraImage sensing_VISION;

#define NO_OFFSET 0
#define SCAN_OFFSET 1
#define BEAM_OFFSET 2

/* The actual sensings. */
typedef struct {
  realPosition                basePosition;
  sensing_MOVEMENT            delta;
  sensing_PROXIMITY           sonar;
  sensing_PROXIMITY           frontLaser;
  sensing_PROXIMITY           rearLaser;
  sensing_QUESTION            answer;
  sensing_VISION              image;
  float                       distanceTraveled;  
  int                         noMoveCnt;
  float                       transVelocity;
  float                       rotVelocity;
} rawSensings;
#endif

#define NUMBER_OF_ABSTRACT_SENSORS 4
#define ABSTRACT_SONAR          0
#define ABSTRACT_FRONT_LASER    1
#define ABSTRACT_REAR_LASER     2
#define ABSTRACT_QUESTION       3

extern char* SONAR_TYPE;
extern char* FRONT_LASER_TYPE;
extern char* REAR_LASER_TYPE;
extern char* ROOM_QUESTION_TYPE;

typedef struct {
  int feature;
  int quality;
  int integrateNumber;
  int numberOfQualities;
} featureStruct;


/* Virtual sensors based on features, not distances.
 * Computes the probability of observing the feature at a certain
 * position. */
typedef float (*probOfFeatureGivenPositionFunction) ( int feature,
						      gridPosition position,
						      void* infoForFeature);



/* Virtual sensors based on features, not distances.
 * Computes the probability of observing the feature given the
 * information about the expected feature. */
typedef float (*probOfFeatureGivenExpectedFunction) ( int feature,
						      featureStruct* expectedFeature,
						      void* infoForFeature);


/* Virtual sensors based on features, not distances. */
typedef int (*extractFeatureFunction) ( rawSensings* rawData,
					int sensorNumber,
					void* infoForFeature);

/* Virtual sensors based on features, not distances. */
typedef featureStruct (*expectedFeatureFunction) ( gridPosition position,
						   void* infoForFeature);

/* Sets the probability table used to compute probabilities of the features. */
typedef void (*setNormedFeatureProbsFunction) (void* infoForFeature,
					       float** uninformed);
typedef void (*setUnNormedFeatureProbsFunction) (void* infoForFeature,
						 float** uninformed);

/* This structure is needed to store the motion of the robot since
 * older sample sets. Important if an old sample set is updated by
 * the multi localize module and sent back to here. */
#define SIZE_OF_SAMPLE_SET_HISTORY 15

typedef struct sampleSetHistory {
  realPosition motionSinceSampleSet[SIZE_OF_SAMPLE_SET_HISTORY];
  float        summedMovement[SIZE_OF_SAMPLE_SET_HISTORY][NUMBER_OF_SENSINGS][MAX_NUMBER_OF_ACTIONS_PER_SENSING];
  int          numberOfSamples[SIZE_OF_SAMPLE_SET_HISTORY];
  int          numberOfSamplesToBeSent[SIZE_OF_SAMPLE_SET_HISTORY];
  int          numberOfAllocatedSamples[SIZE_OF_SAMPLE_SET_HISTORY];
  sampleType*  samples[SIZE_OF_SAMPLE_SET_HISTORY];
  struct timeval timeStamp[SIZE_OF_SAMPLE_SET_HISTORY];

  /* The boundaries of all sample sets are assumed to be the same. */
  realPosition minPosition;
  realPosition maxPosition;

  /* Necessary for navigation through the history. */
  int          currentSampleSetNumber;
  int          currentSampleSetPosition;
  int          sizeOfHistory;

  /* This points to the movement actually used for integration. */
  void*        actionInfo;
  void*        actionMask;
} sampleSetHistory;

/* Information need for the integration of an abstract sensor.
 * Notice that this struct is not sensor dependent. */
typedef struct {
  movement sensorOffset;
} abstractIntegrationInfo;


/* Abstract sensor type. Is the same for all proximity sensors but
 * could also be e.g. landmark information. */
typedef struct {
  char* type;
  int sensorNumberOfType;
  int numberOfFeatures;
  int measuredFeature;

  /* Sensor specific information to get prob. of features in probOfFeature(). */
  void* infoForFeatures;

  /* Computes the probability of a feature given position. */
  probOfFeatureGivenPositionFunction probOfFeatureGivenPosition;

  /* Computes the probability of a feature given the expected feature. */
  probOfFeatureGivenExpectedFunction probOfFeatureGivenExpected;

  /* The average probability for any feature to be measured. */
  float* uninformedFeatureProbability;

  /* Info for integration into grid. */
  abstractIntegrationInfo integrationInfo;

  /* Extracts an abstract feature from raw sensor data. */
  extractFeatureFunction extractedFeature;

  /* Returns the expected feature at a certain grid position. */
  expectedFeatureFunction expectedFeature;

  /* Sets the probability table used to compute probabilities of the features. */
  setNormedFeatureProbsFunction setNormedFeatureProbs;
  setUnNormedFeatureProbsFunction setUnNormedFeatureProbs;

} abstractSensorType;


/* The actual sensings of the abstract sensors. */
typedef struct {
  int numberOfSensors;
  abstractSensorType* sensor;
  int numberOfSensorsToBeUsed;
  int sensorOffsetType;
  int* mask;
  bool chooseOptimal;
  bool* integrate;
} abstractSensorVector;

/* The information needed to integrate the sensings. */
typedef struct {
  probabilityGrid            map;
  probabilityGrid            laserMap;
  probabilityGrid            sonarMap;
  probabilityGrid            initialPositionProbs;
  probabilityGrid            planMap;
  probabilityGrid            entropyMap;
  probabilityGrid            potentialMap;
  probabilityGrid            onlineMap;
  int                        useProbGrid;
  int                        onlineMapping;
  positionProbabilityGrid    positionProbs;
  sampleSet                  samples;
  sampleSet                  tcxSamples;
  sampleSetHistory           samplesHistory;
  visionMap                  visMap;
  visionMap                  visVarMap;
  visionMap                  varianceMap;
  visionMap                  varianceVarMap;
  simulatorMap               simMap;
  robot                      estimatedRobot;
  realPosition               measuredRobotPosition;
  realCellList               localMaxima;
  correctionParameter        correctionParam;
  rawSensings                actualSensings;
  abstractSensorVector       abstractSensors[NUMBER_OF_ABSTRACT_SENSORS];
  void*                      info[NUMBER_OF_SENSINGS];
  float                      summedMovements[NUMBER_OF_SENSINGS]
                                            [MAX_NUMBER_OF_ACTIONS_PER_SENSING];
  bool                       proximityIntegrated;
  char*                      logFileName;
  char*                      robotName;
} actionInformation;


/* For each sensing we store whether it should be considered for
 * integration and whether the actual information should be integrated
 * into the position probability grid. */
typedef struct {
  bool use[NUMBER_OF_SENSINGS];
  bool consider[NUMBER_OF_SENSINGS];
  bool perform[NUMBER_OF_SENSINGS][MAX_NUMBER_OF_ACTIONS_PER_SENSING];
  unsigned int   numberOfActions[NUMBER_OF_SENSINGS];
  bool normalizeGrid;
  bool sampleDistribution;
} sensingActionMask;



void
initialize_PROBGRID( char* fileName,
		     actionInformation* actionInfo);

void
initialize_MAP( char* fileName,
		actionInformation* actionInfo);

/*******************************************************
 * These functions are only used to call the special
 * functions for the different sensings.
 *******************************************************/

typedef void (*actionFunction) ( actionInformation*, sensingActionMask*);

typedef struct {
  actionFunction checkIfConsider[NUMBER_OF_SENSINGS];
  actionFunction checkWhichActionsToPerform[NUMBER_OF_SENSINGS];
  actionFunction performActions[NUMBER_OF_SENSINGS];
} sensingFunctions;


void
initialize_SENSINGS( char* fileName,
		    actionInformation* actionInfo,
		    sensingActionMask* mask);

void
accumulateMovements( actionInformation* actionInfo,
		     sensingActionMask* mask);

void
checkWhichSensingsToConsider( actionInformation* info,
			      sensingActionMask* mask);

void
checkWhichActionsToPerform( actionInformation* info,
			    sensingActionMask* mask);

void
performActions( actionInformation* actionInfo,
		sensingActionMask* mask);

void
resetSampleSetToHistory( sampleSetHistory* history);

int
addSampleSetToHistory( sampleSet* samples, sampleSetHistory* history,
		       int numberOfSamplesToBeSent);

#define MAX_DIST_IN_HISTORY 5
#define MAX_DIST_TRAVELLED  300
bool
sampleSetUpToDate( int number, sampleSetHistory* history);

void
setSampleSetHistory( LOCALIZE_updated_samples_ptr tcxSamples,
		     sampleSet* samples,
		     sampleSetHistory* history);

int
getClosestSampleSet( sampleSetHistory* history, struct timeval timeStamp);

#endif





