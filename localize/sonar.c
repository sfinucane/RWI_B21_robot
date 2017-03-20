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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/sonar.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:40 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: sonar.c,v $
 * Revision 1.1  2002/09/14 20:45:40  rstone
 * *** empty log message ***
 *
 * Revision 1.73  2000/02/01 09:29:35  wolfram
 * Fixed a bug causing LOCALIZE to crash when sonars are not used.
 *
 * Revision 1.72  1999/09/26 18:55:22  fox
 * Added scout robot.
 *
 * Revision 1.71  1999/08/27 22:22:34  fox
 * Better communication for multi robot localization.
 *
 * Revision 1.70  1999/07/13 23:08:02  fox
 * Some changes.
 *
 * Revision 1.69  1999/06/30 17:17:21  fox
 * Minor changes.
 *
 * Revision 1.68  1999/06/29 21:36:22  fox
 * Changes for new script reader.
 *
 * Revision 1.67  1999/06/25 19:48:14  fox
 * Minor changs for the urbie.
 *
 * Revision 1.66  1999/06/24 00:21:53  fox
 * Some changes for the urbies.
 *
 * Revision 1.65  1999/06/23 16:22:03  fox
 * Added robot type urban.
 *
 * Revision 1.64  1999/06/03 12:52:54  wolfram
 * Corrected sonar geometry for B21
 *
 * Revision 1.63  1999/03/25 14:27:32  wolfram
 * Added geometry for pioneer I
 *
 * Revision 1.62  1999/03/08 16:47:49  wolfram
 * Fixed problems with map, caused by the fact that it no longer accpets
 * correction parameters from LOCALIZE.  Improved the handling of sonars
 * with Pioneers.  Currently sonars do not work properly on a B21.
 *
 * Revision 1.61  1999/03/01 17:44:32  wolfram
 * Added support for Pioneer II.  Pioneer I will be added next
 *
 * Revision 1.60  1999/01/14 23:39:34  wolfram
 * Added sensorMaxRange and standardDeviaton to the sensor model for lasers
 *
 * Revision 1.59  1998/11/23 21:19:28  fox
 * Fixed some minor bugs.
 *
 * Revision 1.58  1998/11/23 19:45:09  fox
 * Latest version.
 *
 * Revision 1.57  1998/11/17 23:26:30  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.56  1998/10/29 03:45:05  fox
 * Nothing special.
 *
 * Revision 1.55  1998/10/23 20:52:53  fox
 * Nothing specatacular.
 *
 * Revision 1.54  1998/09/25 17:53:33  fox
 * Improved version of condensation.
 *
 * Revision 1.53  1998/09/25 04:02:59  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.52  1998/08/20 00:23:03  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.51  1998/06/12 10:16:46  fox
 * Implemented virutal sensor.
 *
 * Revision 1.50  1998/03/09 10:07:47  wolfram
 * slight changes
 *
 * Revision 1.49  1998/03/09 09:36:28  wolfram
 * LOCALIZE now checks the consistency of the various maps.
 *
 * Revision 1.48  1998/03/02 06:35:25  wolfram
 * Added output of sensor errror values. Small change in graphic.c
 *
 * Revision 1.47  1997/12/02 15:20:44  fox
 * Nothing remarkable.
 *
 * Revision 1.46  1997/11/20 17:10:15  wolfram
 * Fixed some accesses to not initialized memory using purify
 *
 * Revision 1.45  1997/10/31 15:27:21  wolfram
 * Offset of grid maps is set to zero
 *
 * Revision 1.44  1997/10/31 13:11:45  fox
 * Version for active sensing.
 *
 * Revision 1.43  1997/10/02 09:20:29  wolfram
 * Better initialization for angles
 *
 * Revision 1.42  1997/09/30 13:38:53  wolfram
 * Integration of angles is stopped after integration of any proximity sensor
 *
 * Revision 1.41  1997/09/29 16:29:42  wolfram
 * Angles now handle the situation where no sensor is given
 *
 * Revision 1.40  1997/09/29 10:45:25  wolfram
 * Added default values for all values in .ini file
 *
 * Revision 1.39  1997/09/26 17:02:11  wolfram
 * Intermediate version, do not use
 *
 * Revision 1.38  1997/09/09 19:45:14  fox
 * Fixed a bug in finding local maxima and did several changes to active
 * localization.
 *
 * Revision 1.37  1997/08/22 04:16:42  fox
 * Final version before IJCAI.
 *
 * Revision 1.36  1997/08/16 02:48:07  wolfram
 * DistTables for Laser and sonar are now computed from the sonarMap and
 * laserMap resp.
 *
 * Revision 1.35  1997/08/02 16:51:08  wolfram
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
 * Revision 1.34  1997/07/04 17:29:18  fox
 * Final version before holiday!!!
 *
 * Revision 1.33  1997/06/29 21:43:21  wolfram
 * Added initial distance probabilities to laser
 *
 * Revision 1.32  1997/06/27 20:53:00  wolfram
 * Intermediate version reading initial feature probs (sonar only)
 *
 * Revision 1.31  1997/06/27 16:26:31  fox
 * New model of the proximity sensors.
 *
 * Revision 1.30  1997/06/25 14:16:43  fox
 * Changed laser incorporation.
 *
 * Revision 1.29  1997/06/20 07:36:16  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.28  1997/06/03 11:49:24  fox
 * Museum version.
 *
 * Revision 1.27  1997/05/19 21:42:16  wolfram
 * Distance Probabilty Function is now read from a file
 *
 * Revision 1.26  1997/04/30 12:25:43  fox
 * Some minor changes.
 *
 * Revision 1.25  1997/04/08 14:56:26  fox
 * Changed the reset function for new planes and the prob function.
 *
 * Revision 1.24  1997/04/07 11:03:24  fox
 * Should be ok.
 *
 * Revision 1.23  1997/04/03 13:17:52  fox
 * Some minor changes.
 *
 * Revision 1.22  1997/03/19 17:52:44  fox
 * New laser parameters.
 *
 * Revision 1.21  1997/03/18 18:45:32  fox
 * First executable version with a global scan mask for different sensor types.
 *
 * Revision 1.20  1997/03/14 17:58:24  fox
 * This version should run quite stable now.
 *
 * Revision 1.19  1997/03/13 17:36:24  fox
 * Temporary version. Don't use!
 *
 * Revision 1.18  1997/03/03 12:59:23  wolfram
 * Initial position probabilities are computed out of the simulator map if
 * the simulator map is available
 *
 * Revision 1.17  1997/02/11 11:04:11  wolfram
 * Faster Computation of expected distances, height of sensors is considered
 *
 * Revision 1.16  1997/01/30 17:17:27  fox
 * New version with integrated laser.
 *
 * Revision 1.15  1997/01/29 12:23:15  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.14  1997/01/19 14:05:28  wolfram
 * Added counter for number of integrated readings
 *
 * Revision 1.13  1997/01/18 14:07:57  fox
 * Test version.
 *
 * Revision 1.12  1997/01/17 13:21:08  fox
 * Added timer for non relevant functions.
 *
 * Revision 1.11  1997/01/16 12:42:53  fox
 * Fixed a bug in scan masks.
 *
 * Revision 1.10  1997/01/07 09:38:06  fox
 * Changed the loading of the ini file such that all parameters have to be
 * given only if the corresponding feature has to be used.
 *
 * Revision 1.9  1997/01/06 16:31:19  wolfram
 * Added time stamp to sensing proximity
 *
 * Revision 1.8  1996/12/20 15:29:41  fox
 * Added four parameters.
 *
 * Revision 1.7  1996/12/19 14:33:29  wolfram
 * More sensitive computation of expected distance tables
 *
 * Revision 1.6  1996/12/09 10:12:01  wolfram
 * Fixed a bug in angle computation
 *
 * Revision 1.5  1996/12/02 10:32:15  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.4  1996/11/18 09:58:32  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.3  1996/11/15 17:44:09  ws
 * *** empty log message ***
 *
 * Revision 1.2  1996/10/24 09:56:57  fox
 * LOCALIZE also works in a write protected directory.
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
#include "probGrid.h"
#include "allocate.h"
#include "function.h"

#include "sonar.h"
#include "proximity.h"
#include "abstract.h"
#include "proximityTools.h"
#include "movement.h"
#include "file.h"
#include "script.h"
#include "graphic.h"
#include "probGridTools.h"

char* SONAR_TYPE = "SONAR";

/* These number are used to identify the different elements of
 * the token array. */
#define USE_SONAR_TOKEN                             0 
#define SONAR_COMPUTE_EXPECTED_DIST_TOKEN           1
#define SONAR_EXPECTED_DIST_FILE_TOKEN              2
#define NUMBER_OF_SONARS_TO_BE_USED_TOKEN           3
#define NUMBER_OF_FIRST_SONAR_TOKEN                 4
#define AAAI_SONARS_TOKEN                           5
#define SONAR_INTEGRATE_THRESHOLD_TOKEN             6
#define SONAR_CHOOSE_OPTIMAL_SENSOR_TOKEN           7
#define SONAR_NUMBER_OF_EXPECTED_DIST_ANGLES_TOKEN  8
#define SONAR_MAX_QUOTA_OF_PLANES_TOKEN             9
#define SONAR_MAX_FACTOR_TOKEN                     10
#define SONAR_COMBINE_SCANS_TOKEN                  11


#define SONAR_OPENING_ANGLE                         M_PI / 12
#define SONAR_NUMBER_OF_EXPECTED_DISTANCES 64
#define SONAR_NUMBER_OF_STDDEVS 4
/* the product of the latter two must be 256 */
#define SONAR_PROBFUNCTION_FILE_NAME "sonar"

#define SONAR_MAP_EXTENSION ".sonarmap"
#define SONAR_PROBFUNCTION_FILENAME "sonar"

static float
stdDevThreshold_SONAR[SONAR_NUMBER_OF_STDDEVS] = {50.0, 100.0, 500.0, MAXFLOAT};

float maxFactor_SONAR = 1.3;

static bool addressForNoIntegration = FALSE;

sonarParameters globalSonarParameters;

int numberOfSonars = 24;

float pioneerISonarOffsetForward[] =  {10.0,12.0,13.0,13.0,13.0,12.0,10.0,-13.0};
float pioneerISonarOffsetSideward[] = {9.0,3.0,1.5,0.0,-1.5,-3.0,-9.0,0.0};
float pioneerISonarAngle[]         =  {90.0,30.0,15.0,0.0,-15.0,-30.0,-90.0,180.0};   


float pioneerIISonarOffsetForward[] = {11.5, 15.5, 19.0, 21.0, 21.0, 19.0, 15.5, 11.5,  -11.5, -15.5, -19.0, -21.0, -21.0, -19.0, -15.5, -11.5}; 

float pioneerIISonarOffsetSideward[] = { 13.0, 11.5, 8.0, 2.5, -2.5, -8.0, -11.5, -13.0,  -13.0, -11.5, -8.0, -2.5, 2.5, 8.0, 11.5, 13.0};

float pioneerIISonarAngle[] = { 90.0, 50.0, 30.0, 10.0, -10.0, -30.0,
				-50.0, -90.0, -90.0, -130.0, -150.0, -170.0, 170.0, 150.0, 130.0, 90};

float urbanSonarOffsetForward[] = { 22.07, 21.52, 6.27, -9.73, -17.65, -17.65, -9.73, 6.27, 21.52, 22.07};

float urbanSonarOffsetSideward[] = { -4.39, -9.34, -17.71, -17.71, -17.71, 17.71, 17.71, 17.71, 9.34, 4.39};

float urbanSonarAngle[] = { 3.0, 11.0, 81.0, 90.0, 99.0, -99.0, -90.0, -81.0, -11.0, -3.0};


float *sonarOffsetForward;
float *sonarOffsetSideward;
float *sonarAngle;

/********************************************************************
 ********************************************************************
 * Forward declarations of internal functions.
 ********************************************************************
 ********************************************************************/

static void
initSonarHardwareSettings( int robotType);

static void
integrateSonars( informationsFor_SONAR* info);

static void
computeDistProbFunctionParameter_SONAR( sensorParameters
					*probFunctionParams);

static void
initSonarSensings( actionInformation* actionInfo,
		   sensingActionMask* actionMask,
		   int numberOfSonarsToBeUsed,
		   int numberOfFirstSonar);

static abstractSensorType
abstractSonarSensor( actionInformation* actionInfo,
		     int sonarNumber);

movement
sonarOffset( int num);

static void
initializeCombinedScan( abstractSensorVector* scan, combinedAbstractScan** combinedScan);

static bool
insertScan( abstractSensorVector* scan, realPosition scanPos, distanceScan* rawScan,
	    combinedAbstractScan* combinedScan);

static void
mergeScans( combinedAbstractScan* combinedScan, abstractSensorVector* merge, distanceScan* distances);

static void
initializeMergeScans( combinedAbstractScan* combinedScan,
		      abstractSensorVector** merge, distanceScan** distances);

/**************************************************************************
 **************************************************************************
 * Mandatory functions.
 **************************************************************************
 **************************************************************************/

void
initialize_SONAR( char* fileName,
		  actionInformation* actionInfo,
		  sensingActionMask* actionMask,
		  sensingFunctions* handlers)
{
  /* Local variables initialized in the parameter file. */
  char distFile[MAX_STRING_LENGTH];
  bool useSonar, computeExpectedDist, chooseOptimalSensor;
  int numberOfAngles;

  extern mapParameters globalMapParameters;
  
  int numberOfSonarsToBeUsed, numberOfFirstSonar;
  
  /* This struct contains all relevant information for later integration
   * of sonar informations. */
  informationsFor_SONAR* info = 
    (informationsFor_SONAR*) malloc( sizeof( informationsFor_SONAR));

  token tok[NUMBER_OF_SONAR_PARAMETERS];

  /*-------------------------------------------------------------------------
   * Initialize parameters
   *------------------------------------------------------------------------*/
  globalSonarParameters.useSonar = FALSE;
  useSonar = TRUE;
  computeExpectedDist = 0;
  sprintf(distFile, "%s%s", globalMapParameters.mapFileName, ".sonarDist");
  numberOfAngles = 360;
  chooseOptimalSensor = 0;
  numberOfSonarsToBeUsed = 12;
  numberOfFirstSonar = 0;
  globalSonarParameters.integrateThreshold = 40;
  globalSonarParameters.maxQuotaOfPlanes = 0.4;
  maxFactor_SONAR = 1.3;
  globalSonarParameters.aaaiSonars = 0;
  globalSonarParameters.numberOfScansToBeCombined = 1;
  
  setTokensInitialized(tok, NUMBER_OF_SONAR_PARAMETERS);
  
  /*-------------------------------------------------------------------------
   * Get the parameters from the file. 
   *------------------------------------------------------------------------*/
  tok[USE_SONAR_TOKEN].format   = INT_FORMAT;
  tok[USE_SONAR_TOKEN].variable = &useSonar;
  tok[USE_SONAR_TOKEN].keyWord  = USE_SONAR_KEYWORD;

  tok[SONAR_COMPUTE_EXPECTED_DIST_TOKEN].format   = INT_FORMAT;
  tok[SONAR_COMPUTE_EXPECTED_DIST_TOKEN].variable = &computeExpectedDist;
  tok[SONAR_COMPUTE_EXPECTED_DIST_TOKEN].keyWord  = SONAR_COMPUTE_EXPECTED_DIST_KEYWORD;
  
  tok[SONAR_EXPECTED_DIST_FILE_TOKEN].format   = STRING_FORMAT;
  tok[SONAR_EXPECTED_DIST_FILE_TOKEN].variable = distFile;
  tok[SONAR_EXPECTED_DIST_FILE_TOKEN].keyWord  = SONAR_EXPECTED_DIST_KEYWORD;
  
  tok[SONAR_NUMBER_OF_EXPECTED_DIST_ANGLES_TOKEN].format   = INT_FORMAT;
  tok[SONAR_NUMBER_OF_EXPECTED_DIST_ANGLES_TOKEN].variable = &numberOfAngles;
  tok[SONAR_NUMBER_OF_EXPECTED_DIST_ANGLES_TOKEN].keyWord  = SONAR_NUMBER_OF_EXPECTED_DIST_ANGLES_KEYWORD;
  
  tok[NUMBER_OF_SONARS_TO_BE_USED_TOKEN].format   = INT_FORMAT;
  tok[NUMBER_OF_SONARS_TO_BE_USED_TOKEN].variable = &numberOfSonarsToBeUsed;
  tok[NUMBER_OF_SONARS_TO_BE_USED_TOKEN].keyWord  = NUMBER_OF_SONARS_TO_BE_USED_KEYWORD;

  tok[NUMBER_OF_FIRST_SONAR_TOKEN].format   = INT_FORMAT;
  tok[NUMBER_OF_FIRST_SONAR_TOKEN].variable = &numberOfFirstSonar;
  tok[NUMBER_OF_FIRST_SONAR_TOKEN].keyWord  = NUMBER_OF_FIRST_SONAR_KEYWORD;
  
  tok[AAAI_SONARS_TOKEN].format   = INT_FORMAT;
  tok[AAAI_SONARS_TOKEN].variable = &(globalSonarParameters.aaaiSonars);
  tok[AAAI_SONARS_TOKEN].keyWord  = AAAI_SONARS_KEYWORD;

  tok[SONAR_CHOOSE_OPTIMAL_SENSOR_TOKEN].format   = INT_FORMAT;
  tok[SONAR_CHOOSE_OPTIMAL_SENSOR_TOKEN].variable = &chooseOptimalSensor;
  tok[SONAR_CHOOSE_OPTIMAL_SENSOR_TOKEN].keyWord  = SONAR_CHOOSE_OPTIMAL_SENSOR_KEYWORD;
  
  tok[SONAR_INTEGRATE_THRESHOLD_TOKEN].format   = FLOAT_FORMAT;
  tok[SONAR_INTEGRATE_THRESHOLD_TOKEN].variable = &(globalSonarParameters.integrateThreshold);
  tok[SONAR_INTEGRATE_THRESHOLD_TOKEN].keyWord  = SONAR_INTEGRATE_THRESHOLD_KEYWORD;
  
  tok[SONAR_MAX_QUOTA_OF_PLANES_TOKEN].format   = FLOAT_FORMAT;
  tok[SONAR_MAX_QUOTA_OF_PLANES_TOKEN].variable = &(globalSonarParameters.maxQuotaOfPlanes);
  tok[SONAR_MAX_QUOTA_OF_PLANES_TOKEN].keyWord  = SONAR_MAX_QUOTA_OF_PLANES_KEYWORD;

  tok[SONAR_MAX_FACTOR_TOKEN].format   = FLOAT_FORMAT;
  tok[SONAR_MAX_FACTOR_TOKEN].variable = &(maxFactor_SONAR);
  tok[SONAR_MAX_FACTOR_TOKEN].keyWord  = SONAR_MAX_FACTOR_KEYWORD;

  tok[SONAR_COMBINE_SCANS_TOKEN].format   = INT_FORMAT;
  tok[SONAR_COMBINE_SCANS_TOKEN].variable = &(globalSonarParameters.numberOfScansToBeCombined);
  tok[SONAR_COMBINE_SCANS_TOKEN].keyWord  = SONAR_COMBINE_SCANS_KEYWORD;

  readTokens( fileName, tok, NUMBER_OF_SONAR_PARAMETERS, FALSE); 
  
  /*-------------------------------------------------------------------------
   * done.
   *------------------------------------------------------------------------*/

  if ( useSonar) {
    globalSonarParameters.useSonar = TRUE;
    
    /*------------------------------------------------------------------------
     * Get the preprocessed tables to look up the probabilities of the measurements.
     *-----------------------------------------------------------------------*/
    
    computeDistProbFunctionParameter_SONAR(  &(globalSonarParameters.parameters));

    if ( ! actionInfo->onlineMapping) {
      
      if (!readGridMap( globalMapParameters.mapFileName,
			SONAR_MAP_EXTENSION,
			&actionInfo->sonarMap)){
	if (actionInfo->simMap.initialized){
	  computeGridMap( &actionInfo->simMap,
			  globalMapParameters.desiredResolution,
			  0.0,
			  globalSonarParameters.parameters.sensorHeight,
			  globalSonarParameters.parameters.sensorHeight,
			  &(actionInfo->sonarMap));
	  (void) writeGridMap( globalMapParameters.mapFileName,
			       SONAR_MAP_EXTENSION,
			       &(actionInfo->sonarMap));
	  
	  
	}
	else actionInfo->sonarMap = actionInfo->map;
      }
      actionInfo->sonarMap.offsetX = actionInfo->map.offsetX;
      actionInfo->sonarMap.offsetY = actionInfo->map.offsetY;
      actionInfo->sonarMap.initialized = TRUE;
      
      checkMapConsistency(actionInfo->map.sizeX, actionInfo->map.sizeY,
			  actionInfo->map.resolution,
			  actionInfo->sonarMap.sizeX,
			  actionInfo->sonarMap.sizeY,
			  actionInfo->sonarMap.resolution,
			  globalMapParameters.mapFileName,
			  GRID_MAP_EXTENSION, SONAR_MAP_EXTENSION);
      
      
      /*------------------------------------------------------------------------
       * Set the pointers to the structures provided in actionInfo.
       *-----------------------------------------------------------------------*/
      info->general.useProbGrid          = &(actionInfo->useProbGrid);
      info->general.grid                 = &(actionInfo->positionProbs);
      info->general.samples              = &(actionInfo->samples);
      info->general.scan                 = &(actionInfo->actualSensings.sonar);
      info->general.initialPositionProbs = &(actionInfo->initialPositionProbs);
      info->general.localMaxima          = &(actionInfo->localMaxima);
      info->general.numberOfIntegratedReadings = 0;
      info->general.map                  = &(actionInfo->sonarMap);
    }
    /* For online mapping the lasermap is just the online map. */
    else {
    
      info->general.useProbGrid          = &(actionInfo->useProbGrid);
      info->general.grid                 = &(actionInfo->positionProbs);
      info->general.samples              = &(actionInfo->samples);
      info->general.initialPositionProbs = &(actionInfo->initialPositionProbs);
      info->general.localMaxima          = &(actionInfo->localMaxima);
      info->general.numberOfIntegratedReadings = 0;
      info->general.scan                 = &(actionInfo->actualSensings.sonar);
      actionInfo->sonarMap               = actionInfo->map;
      info->general.map                  = &(actionInfo->sonarMap);
      info->general.simMap               = &(actionInfo->simMap);
    }
    
    info->general.onlineMapping        = actionInfo->onlineMapping;

    if ( actionInfo->onlineMapping && chooseOptimalSensor) {
      fprintf( stderr, "Sorry. Sensor filter not implemented for online mapping.\n");
      writeLog( "Sorry. Sensor filter not implemented for online mapping.\n");
      chooseOptimalSensor = FALSE;
    }
    
    actionInfo->abstractSensors[ABSTRACT_SONAR].chooseOptimal =
      chooseOptimalSensor;

    actionMask->numberOfActions[SONAR] = NUMBER_OF_ACTIONS_SONAR;

    info->general.abstractScan         = &(actionInfo->abstractSensors[ABSTRACT_SONAR]); 
    
    initializeDistProbTables( &(info->general),
			      actionInfo,
			      numberOfAngles,
			      globalSonarParameters.parameters,
			      &(actionInfo->sonarMap),
			      distFile,
			      computeExpectedDist);
    
    /* Set the resolution of the prob grid to the resolution of the
     * expected distances. */
    if ( ! actionInfo->useProbGrid) {
      actionInfo->positionProbs.positionResolution =
	info->general.expectedDistances.gridCellSize;
      writeLog( "#Sonar: don't use prob grid. Set resolution to %d cm.\n",
		actionInfo->positionProbs.positionResolution);
    }
    
  }
  else 
    actionMask->numberOfActions[SONAR] = 0;
  
  /*------------------------------------------------------------------------
   * Initialze the handlers.
   *-----------------------------------------------------------------------*/
  handlers->checkIfConsider[SONAR]            = checkIfConsider_SONAR;
  handlers->checkWhichActionsToPerform[SONAR] = checkWhichActionsToPerform_SONAR;
  handlers->performActions[SONAR]             = performActions_SONAR;
  
  /*------------------------------------------------------------------------
   * Initialze the mask.
   *-----------------------------------------------------------------------*/
  actionMask->use[SONAR] = useSonar;

  /* Now place the struct in the global information struct. */
  actionInfo->info[SONAR] = info;

  if (useSonar)
    initSonarHardwareSettings( robotType);

  numberOfSonarsToBeUsed = iMin( numberOfSonarsToBeUsed, numberOfSonars);
  
  /* Initialze the global structures of sonar sensings. */
  initSonarSensings( actionInfo,
		     actionMask,
		     numberOfSonarsToBeUsed,
		     numberOfFirstSonar);

}


/**************************************************************************
 * Check wether the sonar readings shall be considered for integration.
 **************************************************************************/
void
checkIfConsider_SONAR( actionInformation* actionInfo,
		       sensingActionMask* mask)
{
  /* Just check for the threshold. */
  mask->consider[SONAR] = mask->use[SONAR]
    && ( !mask->consider[ANGLE] || actionInfo->positionProbs.quotaOfPlanesToBeUpdated
	 < globalSonarParameters.maxQuotaOfPlanes);
}


/**************************************************************************
 * Check which actions shall be performed on the position probability grid.
 **************************************************************************/
void
checkWhichActionsToPerform_SONAR( actionInformation* info,
				  sensingActionMask* mask)
{
  
  /* Check wether the sonar reading has to be integrated.
   * This has to be done after enough movement and only with new
   * sonar readings. */
  if ( mask->consider[SONAR]
       && info->actualSensings.sonar.isNew
       && info->summedMovements[SONAR][INTEGRATE_SONAR]
       > globalSonarParameters.integrateThreshold) {
    mask->perform[SONAR][INTEGRATE_SONAR] = TRUE;
    mask->perform[MOVEMENT][INTEGRATE_MOVEMENT] = TRUE;
    info->summedMovements[SONAR][INTEGRATE_SONAR] = 0.0;
    info->summedMovements[MOVEMENT][INTEGRATE_MOVEMENT] = 0.0;  
  }
  else
    mask->perform[SONAR][INTEGRATE_SONAR] = FALSE;
}


/**************************************************************************
 * Perform the actions on  the position probability grid.
 **************************************************************************/
void
performActions_SONAR( actionInformation* actionInfo,
		      sensingActionMask* mask)
{
  /* This structure contains all relevant information for the actions. */
  informationsFor_SONAR* info =
    (informationsFor_SONAR*) actionInfo->info[SONAR];

  if ( mask->perform[SONAR][INTEGRATE_SONAR]) {
      
    if ( globalSonarParameters.numberOfScansToBeCombined > 1) {
      
      static combinedAbstractScan* combinedScan   = NULL;
      static abstractSensorVector* mergedAbstract = NULL;
      static distanceScan* mergedRaw              = NULL;
      static realPosition lastScanPosition = {0,0,0};
      
      movement motionSinceLastScan = movementBetweenRobotPoints( lastScanPosition,
								 actionInfo->actualSensings.basePosition);
      
      abstractSensorVector* scan = info->general.abstractScan;
      distanceScan rawScan = actionInfo->actualSensings.sonar;

      /* Allocate memory etc. */
      if ( ! combinedScan)
	initializeCombinedScan( scan, &combinedScan);
      if ( ! mergedAbstract || ! mergedRaw)
	initializeMergeScans( combinedScan, &mergedAbstract, &mergedRaw);
      
      /* Check whether the rotation of the robot is small enough to integrate the scan
       * into a larger structure. */
#define MAX_ROT_FOR_MERGE (deg2Rad(5))
#define MAX_SIDEWARD_FOR_MERGE (5)
      
      if( fabs( motionSinceLastScan.rotation) > MAX_ROT_FOR_MERGE ||
	  fabs( motionSinceLastScan.sideward) > MAX_SIDEWARD_FOR_MERGE) {
	
	fprintf(stderr, "Don't merge: %f %f %f\n",
		actionInfo->actualSensings.delta.forward,
		actionInfo->actualSensings.delta.sideward,
		actionInfo->actualSensings.delta.rotation);
	
	combinedScan->currentScanNumber = 0;
	
	integrateSonars( info);
	actionInfo->proximityIntegrated = TRUE;
      }

      /* Ok. Put it in the structure. */
      else {

	/* Insert the scan into the combined one. */
	if ( insertScan( scan,
			 actionInfo->actualSensings.basePosition, &rawScan,
			 combinedScan)) {
	  
	  /* The combined scan is complete. Merge the different scans. */
	  mergeScans( combinedScan, mergedAbstract, mergedRaw);

	  info->general.abstractScan = mergedAbstract;
	  integrateSonars( info);
	  info->general.abstractScan = scan;
	  
	  actionInfo->actualSensings.sonar = *mergedRaw;
	  updateGlobalRobotWindow( actionInfo, mask);
	  actionInfo->actualSensings.sonar = rawScan;
	  
	  actionInfo->proximityIntegrated = TRUE;
	}
	else {
	  if (1) mask->perform[SONAR][INTEGRATE_SONAR] = FALSE;
	}
      }
      lastScanPosition = actionInfo->actualSensings.basePosition;
    }
    else {
      integrateSonars( info);
      actionInfo->proximityIntegrated = TRUE;
    }
  }
}

/********************************************************************
 ********************************************************************
 * Global functions.
 ********************************************************************
 ********************************************************************/

float
sonarRot( int i) {
  static float* sonarRotTab;
  static bool firstCall = TRUE;

  if (!globalSonarParameters.useSonar) return 0.0;
  if (firstCall){
    int j;
    
    sonarRotTab = (float*) allocate1D( numberOfSonars, FLOAT);
    
    /* This is good old rhino in the US. */
    if ( robotType == B21_TWO_LASERS_ROBOT &&
	 globalSonarParameters.aaaiSonars) {
      for (j = 0; j < numberOfSonars; j++)      	
	sonarRotTab[j] = normalizedAngle( deg2Rad( (float) (j + 1) * 15.0));
    }
    
    /* Default sonar placement for the B21s. */
    else if ( robotType == B21_TWO_LASERS_ROBOT ||
	      robotType == B21_ONE_LASER_ROBOT) {
      for (j = 0; j < numberOfSonars; j++)      	
	sonarRotTab[j] = normalizedAngle( deg2Rad( (float) j * 15.0 + 7.5));
    }
    /* The pioneer angles. */
    else if ( robotType == PIONEER_ATRV ||
	      robotType == PIONEER_II ||
	      robotType == URBAN_ROBOT) {
      for (j = 0; j < numberOfSonars; j++)      	
	sonarRotTab[j] = normalizedAngle( deg2Rad( sonarAngle[j]));
    }
    else {
      fprintf( stderr, "Don't know the sonar settings of robotType %d (see file.h).\n",
	       robotType);
      for (j = 0; j < numberOfSonars; j++)      	
	sonarRotTab[j] = 0.0;
    }
    firstCall = FALSE;
  }
  
  return sonarRotTab[i];
}





movement
sonarOffset( int num)
{
  movement move;

  if (globalSonarParameters.useSonar){
    if (robotType == B21_TWO_LASERS_ROBOT || robotType == B21_ONE_LASER_ROBOT){
      move.forward = ROB_RADIUS * cos(sonarRot(num));
      move.sideward = -ROB_RADIUS * sin(sonarRot(num));
      move.rotation = sonarRot(num);
    }
    else if ( robotType == PIONEER_ATRV ||
	      robotType == PIONEER_II ||
	      robotType == URBAN_ROBOT) {
      move.forward  = sonarOffsetForward[num];
      move.sideward = sonarOffsetSideward[num];
    move.rotation = normalizedAngle( deg2Rad( sonarAngle[num]));
    }
  }
  else
    move.forward = move.sideward = move.rotation = 0.0;
  return move;
}


/********************************************************************
  returns the realPosition of sonar num given realPosition pos
  ******************************************************************/

realPosition
sonarPosition( realPosition pos, int num)
{

  if (robotType == B21_TWO_LASERS_ROBOT){
    pos.rot += sonarRot(num);
    pos.x += ROB_RADIUS * cos(pos.rot);
    pos.y += ROB_RADIUS * sin(pos.rot);
  }
  else if ( robotType == PIONEER_ATRV ||
	    robotType == PIONEER_II ||
	    robotType == URBAN_ROBOT) {
    pos = endPoint(pos, sonarOffset(num));
  }  
  return pos;
}


/********************************************************************
 ********************************************************************
 * The local functions.
 ********************************************************************
 ********************************************************************/


/***********************************************************************
 * Integrates the sonar probabilities in the grid.
 ***********************************************************************/
static void
integrateSonars( informationsFor_SONAR* info)
{
  /* Just for information. */
  static int doSonarCnt = 0;
  
  setTimer( 0);
    
  writeLog("#        Sonars no.%d ", ++doSonarCnt);
  writeLog( "... ");
  
  fprintf( stderr, "# *************************************************\n");
  fprintf( stderr,"#        Sonars no.%d ", doSonarCnt);
  fprintf( stderr, "... ");

  /*-------------------------------------------------------------------
   * This is the main function.
   *-------------------------------------------------------------------*/
  integrateDistScan( &( info->general));
  
  writeLog( "done (%.2f%c in %.2f secs).\n",
	    100.0 * info->general.grid->quotaOfValuesToBeUpdated,
	    '%', timeExpired(0));
  
  fprintf(stderr, "done (%.2f%c in %.2f secs).\n",
	  100.0 * info->general.grid->quotaOfValuesToBeUpdated,
	  '%', timeExpired(0));
  fprintf( stderr, "# *************************************************\n");
}



/***********************************************************************
 * Estimates the expected distance measured by an ultrasonic sensor    *
 * given a grid cell and a rotation.                                   *
 **********************************************************************/
distance
obstacleDistance_SONAR( probabilityGrid m,
			int x, int y, float rot,
			distance maxRange)
{
  return obstacleDistance( &m, x, y, rot, maxRange);
}


static void
initSonarSensings( actionInformation* actionInfo,
		   sensingActionMask* actionMask,
		   int numberOfSonarsToBeUsed,
		   int numberOfFirstSonar)
{
  int i;
  sensing_PROXIMITY* rawSonar = &( actionInfo->actualSensings.sonar);
  abstractSensorVector* abstractSonar = &( actionInfo->abstractSensors[ABSTRACT_SONAR]);

  /*------------------------------------------------------------------------
   * Initialze the global structure of raw sensors.
   *-----------------------------------------------------------------------*/
  rawSonar->numberOfReadings = numberOfSonars;
  rawSonar->reading          = (distanceReading*)
    malloc( numberOfSonars * sizeof( distanceReading));
  rawSonar->readingOffset          = (movement*)
    malloc( numberOfSonars * sizeof( movement));
  rawSonar->maxDistance      = SONAR_MAX_EXPECTED_DISTANCE;
  rawSonar->isNew            = FALSE;
  rawSonar->offset.forward = rawSonar->offset.sideward =
     rawSonar->offset.rotation = 0.0;
  for ( i = 0; i < numberOfSonars; i++){
    rawSonar->reading[i].rot = sonarRot(i);
    rawSonar->readingOffset[i] = sonarOffset(i);
  }

  /*------------------------------------------------------------------------
   * Initialze the global structure of abstract sensors.
   *-----------------------------------------------------------------------*/
  if ( actionMask->use[SONAR]) {
    abstractSonar->numberOfSensors = numberOfSonars;
    abstractSonar->sensor  = (abstractSensorType*)
      malloc ( numberOfSonars * sizeof( abstractSensorType));
    
    for ( i = 0; i < numberOfSonars; i++) {
      abstractSonar->sensor[i] = abstractSonarSensor( actionInfo, i);
    }
    
    abstractSonar->numberOfSensorsToBeUsed = numberOfSonarsToBeUsed;
    abstractSonar->mask = (int*) allocate1D( abstractSonar->numberOfSensors, INT);

    if (robotType == B21_ONE_LASER_ROBOT
	|| robotType == B21_TWO_LASERS_ROBOT) 
      abstractSonar->sensorOffsetType = NO_OFFSET;
    else
      abstractSonar->sensorOffsetType = BEAM_OFFSET;
    
    
    /* Pointers on information about integration. */
    abstractSonar->integrate =
      &(actionMask->perform[SONAR][INTEGRATE_SONAR]);
  
    setFixedScanMask( abstractSonar, numberOfFirstSonar);
  }
  else {
    /* Pointers on information about integration. */
    abstractSonar->integrate = &addressForNoIntegration;
    abstractSonar->numberOfSensors = 0;
  }
}



/***********************************************************************
 * Computes the sonar parameters
 **********************************************************************/

static void
computeDistProbFunctionParameter_SONAR( sensorParameters *params)
{
  params->numberOfStdDevs = SONAR_NUMBER_OF_STDDEVS;
  params->numberOfExpectedDistances =
    params->numberOfMeasuredDistances =
    SONAR_NUMBER_OF_EXPECTED_DISTANCES;
  params->sensorMaxRange = SONAR_MAX_EXPECTED_DISTANCE;
  params->openingAngle = SONAR_OPENING_ANGLE;
  params->sensorHeight = SONAR_HEIGHT;
  params->stdDevThreshold = stdDevThreshold_SONAR;
  params->maxFactor = maxFactor_SONAR;
  params->probFunctionFileName = SONAR_PROBFUNCTION_FILENAME;
}

/***********************************************************************
 * Converts a raw sonar reading into a sonar feature.
 **********************************************************************/
static int
extractedSonarFeature( rawSensings* rawData,
		       int sensorNumber,
		       void* infoForFeature)
{
  distance dist = rawData->sonar.reading[sensorNumber].dist;
  infoForProximityFeature* info = (infoForProximityFeature*) infoForFeature;
  distProbTable *distProbs = info->distProbFunctionTableNormed;

  return iMin( distProbs->numberOfMeasuredDistances - 1,
	       (int) round (dist / distProbs->distanceResolution));
}


/***********************************************************************
 * Returns a sonar sensor. This sensor is used to compute the a posteriori
 * entropy after having integrated such a sensor into the prob grid.
 **********************************************************************/
static abstractSensorType
abstractSonarSensor( actionInformation* actionInfo,
		     int sonarNumber)
{
  abstractSensorType sensor;
  infoForProximityFeature* info;
  informationsFor_SONAR* sonarInfo = (informationsFor_SONAR*) actionInfo->info[SONAR];
  static float* uninformedNormed;
  static float* uninformedUnNormed;
  static float firstCall = TRUE;
  
  info = (infoForProximityFeature*) malloc( sizeof(infoForProximityFeature));
  
  /* We don't compute this for each laser beam. */
  if (firstCall) {
    uninformedNormed =
      allocate1D( sonarInfo->general.distProbFunctionTableNormed.numberOfStdDevs, FLOAT);
    
    averageProbabilities( &(sonarInfo->general.distProbFunctionTableNormed),
			  uninformedNormed);
    uninformedUnNormed =
      allocate1D( sonarInfo->general.distProbFunctionTableDividedByPOfFeature.numberOfStdDevs, FLOAT);
    
    averageProbabilities( &(sonarInfo->general.distProbFunctionTableDividedByPOfFeature),
			  uninformedUnNormed);
    firstCall = FALSE;
  }

  info->grid                   = &(actionInfo->positionProbs);
  info->distTab                = &(sonarInfo->general.expectedDistances);
  info->samples                = &(actionInfo->samples);

  info->distProbFunctionTableNormed
    = &(sonarInfo->general.distProbFunctionTableNormed);
  info->distProbFunctionTableDividedByPOfFeature
    = &(sonarInfo->general.distProbFunctionTableDividedByPOfFeature);
  info->distProbFunctionTable = info->distProbFunctionTableDividedByPOfFeature;
  info->sensorRot              = sonarRot( sonarNumber);
  info->uninformedNormed       = uninformedNormed;
  info->uninformedDividedByPOfFeature = uninformedUnNormed;
  
  info->gridMap                = &(actionInfo->map);
  info->simMap                 = &(actionInfo->simMap);
  
  /* Offset of the sensors relative to the center of the robot. */
  if ( robotType == B21_ONE_LASER_ROBOT
       || robotType == B21_TWO_LASERS_ROBOT) {
    sensor.integrationInfo.sensorOffset.forward         = 0.0;
    sensor.integrationInfo.sensorOffset.sideward        = 0.0;
    sensor.integrationInfo.sensorOffset.rotation        = 0.0;
  }
  else if (robotType == PIONEER_II ||
	   robotType == PIONEER_ATRV ||
	   robotType == URBAN_ROBOT){
    sensor.integrationInfo.sensorOffset = sonarOffset(sonarNumber);
  }  
    
  sensor.type = SONAR_TYPE;
  sensor.sensorNumberOfType = sonarNumber;
  sensor.numberOfFeatures =
    sonarInfo->general.expectedDistances.numberOfExpectedDistances;
  sensor.probOfFeatureGivenPosition = &probabilityOfProximityFeatureGivenPosition;
  sensor.probOfFeatureGivenExpected = &probabilityOfProximityFeatureGivenExpected;
  sensor.expectedFeature  = &expectedProximityFeature;
  sensor.infoForFeatures  = info;
  sensor.extractedFeature = &extractedSonarFeature;
  sensor.uninformedFeatureProbability = uninformedUnNormed;

  sensor.setNormedFeatureProbs = &setNormedDistTab;
  sensor.setUnNormedFeatureProbs = &setUnNormedDistTab;
  
  return sensor;
}


/* This function sets the number and position of the sonars. */
static void
initSonarHardwareSettings( int robotType)
{
  if ( robotType == B21_ONE_LASER_ROBOT || robotType == B21_TWO_LASERS_ROBOT) {
    writeLog( "# Initialise sonar settings to robot type B21.\n");
    numberOfSonars = 24;
  }
  else if ( robotType == PIONEER_ATRV) {
    writeLog( "# Initialise sonar settings to robot type PIONEER_ATRV.\n");
    numberOfSonars = 7;
    sonarOffsetForward  = pioneerISonarOffsetForward;
    sonarAngle          = pioneerISonarAngle;
    sonarOffsetSideward = pioneerISonarOffsetSideward;
  }
  else if ( robotType == PIONEER_II) {
    writeLog( "# Initialise sonar settings to robot type PIONEER_II.\n");
    numberOfSonars = 16;
    sonarOffsetForward  = pioneerIISonarOffsetForward;
    sonarAngle          = pioneerIISonarAngle;
    sonarOffsetSideward = pioneerIISonarOffsetSideward;
  }
  else if ( robotType == URBAN_ROBOT) {
    writeLog( "# Initialise sonar settings to robot type URBAN_ROBOT.\n");
    numberOfSonars = 10;
    sonarOffsetForward  = urbanSonarOffsetForward;
    sonarAngle          = urbanSonarAngle;
    sonarOffsetSideward = urbanSonarOffsetSideward;
  }
  else {
    fprintf( stderr, "# Don't know sonar settings for robot type %d (see file.h).\n",
	     robotType);
    numberOfSonars = 24;
  }
}



static void
initializeCombinedScan( abstractSensorVector* scan, combinedAbstractScan** combinedScan)
{
  combinedAbstractScan* tmpScan = (combinedAbstractScan*) malloc ( sizeof( combinedAbstractScan));
  
  int numberOfSensors = scan->numberOfSensors;
  int s;
  int numberOfScans = globalSonarParameters.numberOfScansToBeCombined;
  
  fprintf(stderr, "# Initialize combined scan for %d scans containing %d sensors each.\n",
	  numberOfScans, numberOfSensors);

  writeLog( "# Initialize combined scan for %d scans containing %d sensors each.\n",
	  numberOfScans, numberOfSensors);

  tmpScan->numberOfScans = numberOfScans;
  tmpScan->sizeOfScans = numberOfSensors;
  tmpScan->currentScanNumber = 0;

  tmpScan->scan      = (abstractSensorVector*) malloc ( numberOfScans * sizeof( abstractSensorVector));
  tmpScan->position  = (realPosition*) malloc ( numberOfScans * sizeof( realPosition));
  tmpScan->distances = (distanceScan*) malloc ( numberOfScans * sizeof( distanceScan));
  
  for ( s = 0; s < numberOfScans; s++) {
    
    tmpScan->scan[s] = *scan;
    tmpScan->position[s].x = 0.0;
    tmpScan->position[s].y = 0.0;
    tmpScan->position[s].rot = 0.0;
    
    tmpScan->scan[s].sensor  = (abstractSensorType*)
      malloc ( numberOfSensors * sizeof( abstractSensorType));

    tmpScan->distances[s].reading       = (distanceReading*) malloc( numberOfSensors * sizeof(distanceReading));
    tmpScan->distances[s].readingOffset = (movement*) malloc( numberOfSensors * sizeof(movement));
  }
  *combinedScan = tmpScan;
}



static bool
insertScan( abstractSensorVector* scan, realPosition scanPos, distanceScan* rawScan,
	    combinedAbstractScan* combinedScan)
{
  int s;
  int numberOfSensors = scan->numberOfSensors;
  
  /* Copy the data into the combined scan. */
  if ( combinedScan->currentScanNumber < combinedScan->numberOfScans) {

    abstractSensorVector* currentScan = &(combinedScan->scan[combinedScan->currentScanNumber]);
    distanceScan* currentDists        = &(combinedScan->distances[combinedScan->currentScanNumber]);
    
    combinedScan->position[combinedScan->currentScanNumber] = scanPos;

    for ( s = 0; s < numberOfSensors; s++) {
      currentScan->sensor[s] = scan->sensor[s];
      currentDists->reading[s] = rawScan->reading[s];
    }
  }
  
  if ( combinedScan->currentScanNumber == combinedScan->numberOfScans-1) {
    combinedScan->currentScanNumber = 0;
    return TRUE;
  }
  else {
    (combinedScan->currentScanNumber) += 1;
    return TRUE;
  }
}
  

static void
initializeMergeScans( combinedAbstractScan* combinedScan,
		      abstractSensorVector** merge, distanceScan** distances)
{
  int numberOfSensors = combinedScan->numberOfScans * combinedScan->sizeOfScans;

  abstractSensorVector* mergeTmp = (abstractSensorVector*) malloc( sizeof( abstractSensorVector));
  distanceScan* distancesTmp = (distanceScan*) malloc( sizeof( distanceScan));
  
  mergeTmp->numberOfSensors = numberOfSensors;
  mergeTmp->numberOfSensorsToBeUsed = numberOfSensors;
  mergeTmp->sensorOffsetType = BEAM_OFFSET;
  
  mergeTmp->sensor  = (abstractSensorType*)
    malloc ( numberOfSensors * sizeof( abstractSensorType));
  
  
  mergeTmp->mask = (int*) allocate1D( numberOfSensors, INT);
  mergeTmp->chooseOptimal = FALSE;
  mergeTmp->integrate = combinedScan->scan[0].integrate;

  *merge = mergeTmp;

  distancesTmp->numberOfReadings = numberOfSensors;

  distancesTmp->readingOffset = (movement*) malloc( numberOfSensors * sizeof(movement));
  distancesTmp->reading       = (distanceReading*) malloc( numberOfSensors * sizeof(distanceReading));

  *distances = distancesTmp;
}


static void
mergeScans( combinedAbstractScan* combinedScan, abstractSensorVector* merge, distanceScan* distances)
{
  int scanNumber, beam, mergedBeam;
  realPosition finalPosition = combinedScan->position[combinedScan->numberOfScans-1];

  /* Get the offset between the last scan and each of the intermediate scans. */
  for ( scanNumber = 0, mergedBeam = 0; scanNumber < combinedScan->numberOfScans; scanNumber++) {

    movement scanOffset = movementBetweenRobotPoints( finalPosition, combinedScan->position[scanNumber]);

    /* This offset is added to each of the individual beams in the scan. */
    for ( beam = 0; beam < combinedScan->scan[scanNumber].numberOfSensors; beam++, mergedBeam++) {
      
      realPosition currentPos = {0.0,0.0,0.0};
      realPosition scanStart = endPoint( currentPos, scanOffset);
      realPosition beamStart = endPoint( scanStart,
					 combinedScan->scan[scanNumber].sensor[beam].integrationInfo.sensorOffset);

      merge->sensor[mergedBeam] = combinedScan->scan[scanNumber].sensor[beam];
      merge->sensor[mergedBeam].integrationInfo.sensorOffset =
	movementBetweenPoints( currentPos, beamStart);
      
      distances->reading[mergedBeam]       = combinedScan->distances[scanNumber].reading[beam];
      distances->readingOffset[mergedBeam] = merge->sensor[mergedBeam].integrationInfo.sensorOffset;
    }
  }
  distances->isNew = TRUE;
  distances->maxDistance = combinedScan->distances[0].maxDistance;
}
  

