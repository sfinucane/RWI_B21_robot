
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/laser.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 20:45:39 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: laser.c,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.95  2000/03/06 20:00:44  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.94  2000/01/26 22:56:43  fox
 * Might work. If anything weird happens, contact me.
 *
 * Revision 1.93  2000/01/19 16:21:47  fox
 * Version after I created density sample sets. Removed all defines so
 * that it should work properly. If you want to use zlib, set USE_ZLIB
 * in the Makefile.
 *
 * Revision 1.92  2000/01/10 19:04:20  fox
 * DON'T USE!
 *
 * Revision 1.91  2000/01/02 15:33:15  fox
 * Should work.
 *
 * Revision 1.90  1999/12/27 09:52:51  fox
 * *** empty log message ***
 *
 * Revision 1.89  1999/10/02 09:06:43  thrun
 * New robot type: XR4000.
 *
 * Revision 1.88  1999/09/26 18:55:22  fox
 * Added scout robot.
 *
 * Revision 1.87  1999/06/30 11:46:57  rhino
 * Added laser settings for PIONEER_II
 *
 * Revision 1.86  1999/06/25 19:48:12  fox
 * Minor changs for the urbie.
 *
 * Revision 1.85  1999/06/24 00:21:51  fox
 * Some changes for the urbies.
 *
 * Revision 1.84  1999/05/18 15:15:20  fox
 * Added keywords to determine whether correction parameters should be sent to map and plan or not.
 *
 * Revision 1.83  1999/04/29 13:35:20  fox
 * Further adaptation to make multi localize run.
 *
 * Revision 1.82  1999/04/29 00:58:28  fox
 * Some minor changes for multi localize.
 *
 * Revision 1.81  1999/03/08 16:47:42  wolfram
 * Fixed problems with map, caused by the fact that it no longer accpets
 * correction parameters from LOCALIZE.  Improved the handling of sonars
 * with Pioneers.  Currently sonars do not work properly on a B21.
 *
 * Revision 1.80  1999/03/01 17:44:30  wolfram
 * Added support for Pioneer II.  Pioneer I will be added next
 *
 * Revision 1.79  1999/02/17 19:42:23  fox
 * Enhanced gif utilities.
 *
 * Revision 1.78  1999/01/29 09:31:45  wolfram
 * Small changes
 *
 * Revision 1.77  1999/01/22 18:10:40  fox
 * Removed some hacks done for denver and the sampling paper.
 *
 * Revision 1.76  1999/01/22 17:48:05  fox
 * Final version of sampling paper and denver demo.
 *
 * Revision 1.75  1999/01/14 23:39:30  wolfram
 * Added sensorMaxRange and standardDeviaton to the sensor model for lasers
 *
 * Revision 1.74  1999/01/14 00:33:01  wolfram
 * Changes for vision
 *
 * Revision 1.73  1999/01/11 19:47:50  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.71  1998/11/23 19:45:07  fox
 * Latest version.
 *
 * Revision 1.70  1998/11/17 23:26:20  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.69  1998/11/03 21:02:18  fox
 * Lasers will not be integrated if the robot rotates faster than
 * MAX_ROT_VEL_FOR_LASERS deg/sec.
 *
 * Revision 1.68  1998/10/29 03:45:01  fox
 * Nothing special.
 *
 * Revision 1.67  1998/10/23 20:52:50  fox
 * Nothing specatacular.
 *
 * Revision 1.66  1998/10/02 15:16:39  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.65  1998/09/25 17:53:30  fox
 * Improved version of condensation.
 *
 * Revision 1.64  1998/09/25 04:02:56  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.63  1998/09/22 14:40:24  wolfram
 * Two stddevs for laser can more easily be configured.
 *
 * Revision 1.62  1998/09/18 15:44:26  fox
 * First attempt to get a running system after museum.
 *
 * Revision 1.61  1998/08/23 00:01:00  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.60  1998/08/20 00:22:59  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.59  1998/08/11 23:05:37  wolfram
 * Changes for the Smithsonian museum.
 *
 * WARNING: LOCALIZE now uses only one stddev for Lasers. Laser beams have 25 m
 * Maxrange now.
 * Please use ~wolfram/bee/data/localize/params/laser-0.1stdev with this version
 * of LOCALIZE
 *
 * Revision 1.58  1998/03/09 10:07:44  wolfram
 * slight changes
 *
 * Revision 1.57  1998/03/09 09:36:27  wolfram
 * LOCALIZE now checks the consistency of the various maps.
 *
 * Revision 1.56  1998/02/13 14:12:21  fox
 * Minor changes.
 *
 * Revision 1.55  1998/02/10 13:04:48  wolfram
 * LOCALIZE reads new scripts now! First alpha 0.0001-version
 *
 * Revision 1.54  1998/01/27 15:25:24  fox
 * Minor changes.
 *
 * Revision 1.53  1998/01/22 13:06:16  fox
 * First version after selection-submission.
 *
 * Revision 1.52  1998/01/06 15:11:20  fox
 * Added evaluation tools.
 *
 * Revision 1.51  1998/01/05 10:37:11  fox
 * Several changes especially in selection parts.
 *
 * Revision 1.50  1997/12/19 11:30:11  fox
 * FIXED a bug I added.
 *
 * Revision 1.49  1997/12/11 17:06:29  fox
 * Added some parameters.
 *
 * Revision 1.48  1997/12/02 15:20:39  fox
 * Nothing remarkable.
 *
 * Revision 1.47  1997/11/26 15:47:43  fox
 * Added some structures for questions.
 *
 * Revision 1.46  1997/11/25 17:12:54  fox
 * Should work.
 *
 * Revision 1.45  1997/11/21 15:36:05  fox
 * Modifications in graphic
 *
 * Revision 1.44  1997/11/20 12:58:11  fox
 * Version with good sensor selection.
 *
 * Revision 1.43  1997/10/31 15:27:19  wolfram
 * Offset of grid maps is set to zero
 *
 * Revision 1.42  1997/10/02 09:20:27  wolfram
 * Better initialization for angles
 *
 * Revision 1.41  1997/09/30 13:38:52  wolfram
 * Integration of angles is stopped after integration of any proximity sensor
 *
 * Revision 1.40  1997/09/29 10:45:23  wolfram
 * Added default values for all values in .ini file
 *
 * Revision 1.39  1997/09/26 17:02:09  wolfram
 * Intermediate version, do not use
 *
 * Revision 1.38  1997/08/22 04:16:38  fox
 * Final version before IJCAI.
 *
 * Revision 1.37  1997/08/16 02:48:05  wolfram
 * DistTables for Laser and sonar are now computed from the sonarMap and
 * laserMap resp.
 *
 * Revision 1.36  1997/08/02 16:51:03  wolfram
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
 * Revision 1.35  1997/07/04 17:29:14  fox
 * Final version before holiday!!!
 *
 * Revision 1.34  1997/06/29 21:43:20  wolfram
 * Added initial distance probabilities to laser
 *
 * Revision 1.33  1997/06/27 16:26:27  fox
 * New model of the proximity sensors.
 *
 * Revision 1.32  1997/06/25 14:16:39  fox
 * Changed laser incorporation.
 *
 * Revision 1.31  1997/06/20 07:36:10  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.30  1997/06/03 11:49:21  fox
 * Museum version.
 *
 * Revision 1.29  1997/05/26 08:47:49  fox
 * Last version before major changes.
 *
 * Revision 1.28  1997/05/19 21:42:14  wolfram
 * Distance Probabilty Function is now read from a file
 *
 * Revision 1.27  1997/04/30 12:25:40  fox
 * Some minor changes.
 *
 * Revision 1.26  1997/04/08 14:56:24  fox
 * Changed the reset function for new planes and the prob function.
 *
 * Revision 1.25  1997/04/07 11:03:22  fox
 * Should be ok.
 *
 * Revision 1.24  1997/04/03 15:01:02  wolfram
 * New parameters for lasers
 *
 * Revision 1.23  1997/04/03 13:29:04  wolfram
 * *** empty log message ***
 *
 * Revision 1.22  1997/04/03 13:27:41  wolfram
 * laser.c now includes abstract.h
 *
 * Revision 1.21  1997/03/19 17:52:42  fox
 * New laser parameters.
 *
 * Revision 1.20  1997/03/18 18:45:29  fox
 * First executable version with a global scan mask for different sensor types.
 *
 * Revision 1.19  1997/03/14 17:58:19  fox
 * This version should run quite stable now.
 *
 * Revision 1.18  1997/03/13 18:08:54  fox
 * Set raw laser offset.
 *
 * Revision 1.17  1997/03/13 17:35:43  fox
 * Temporary version. Don't use!
 *
 * Revision 1.16  1997/02/11 11:04:09  wolfram
 * Faster Computation of expected distances, height of sensors is considered
 *
 * Revision 1.15  1997/01/31 17:11:01  fox
 * Integrated laser reply.
 *
 * Revision 1.14  1997/01/30 17:17:24  fox
 * New version with integrated laser.
 *
 * Revision 1.13  1997/01/29 12:23:08  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.12  1997/01/19 14:05:26  wolfram
 * Added counter for number of integrated readings
 *
 * Revision 1.11  1997/01/16 12:42:50  fox
 * Fixed a bug in scan masks.
 *
 * Revision 1.10  1997/01/07 09:38:05  fox
 * Changed the loading of the ini file such that all parameters have to be
 * given only if the corresponding feature has to be used.
 *
 * Revision 1.9  1997/01/06 16:31:19  wolfram
 * Added time stamp to sensing proximity
 *
 * Revision 1.8  1996/12/31 09:19:23  fox
 * Last version before I switched exploration mode in PLAN to random
 * mode in COLLI.
 *
 * Revision 1.7  1996/12/20 15:29:38  fox
 * Added four parameters.
 *
 * Revision 1.6  1996/12/19 14:33:28  wolfram
 * More sensitive computation of expected distance tables
 *
 * Revision 1.5  1996/12/02 10:32:07  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.4  1996/11/18 09:58:30  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.3  1996/11/15 17:44:07  ws
 * *** empty log message ***
 *
 * Revision 1.2  1996/10/24 09:56:53  fox
 * LOCALIZE also works in a write protected directory.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:31  rhino
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
#include "laser.h"
#include "probGrid.h"
#include "allocate.h"
#include "function.h"
#include "laser.h"
#include "proximity.h"
#include "proximityTools.h"
#include "movement.h"
#include "file.h"
#include "abstract.h"

char* FRONT_LASER_TYPE = "FRONT_LASER";
char* REAR_LASER_TYPE = "REAR_LASER";

#define FRONT_LASER 0
#define REAR_LASER 1

int LASER_MAX_EXPECTED_DISTANCE = 1000;
int LASER_NUMBER_OF_STDDEVS = 1;

float FRONT_LASER_START_ANGLE = (DEG_270);
float REAR_LASER_START_ANGLE = (DEG_90);


/* These number are used to identify the different elements of
 * the token array. */
#define USE_LASER_TOKEN                             0 
#define LASER_COMPUTE_EXPECTED_DIST_TOKEN           1
#define LASER_EXPECTED_DIST_FILE_TOKEN              2
#define NUMBER_OF_LASERS_TO_BE_USED_TOKEN           3
#define NUMBER_OF_FIRST_LASER_TOKEN                 4
#define LASER_MAX_QUOTA_OF_PLANES_TOKEN             5
#define LASER_INTEGRATE_THRESHOLD_TOKEN             6
#define LASER_NUMBER_OF_EXPECTED_DIST_ANGLES_TOKEN  7
#define LASER_CHOOSE_OPTIMAL_SENSOR                 8
#define LASER_MAX_FACTOR_TOKEN                      9
#define NUMBER_OF_LASERS_FOR_CONVOLVE_TOKEN        10
#define MAX_NUMBER_OF_INTEGRATIONS_PER_STOP_TOKEN  11
/* NICKR */
#define LASER_CUT_DISTANCE_TOKEN                   12
#define MAX_ROT_VEL_FOR_LASERS_TOKEN               13
#define LASER_MAX_EXPECTED_DISTANCE_TOKEN          14
#define LASER_NUMBER_OF_STDDEVS_TOKEN              15

#define LASER_MAX_NUMBER_OF_STDDEVS 2
#define LASER_MAX_NUMBER_OF_EXPECTED_DISTANCES  256



#define LASER_PROBFUNCTION_FILE_NAME "laser"
#define LASER_MAP_EXTENSION ".lasermap"
#define LASER_PROBFUNCTION_FILENAME "laser"


float frontLaserOffset = 0.0;
float rearLaserOffset = 0.0;
float maxFactor_LASER = 1.3;

laserParameters globalLaserParameters;
int numberOfLaserIntegrationsSinceStopped = 0;

static bool addressForNoIntegration = FALSE;

/********************************************************************
 ********************************************************************
 * Forward declarations of internal functions.
 ********************************************************************
 ********************************************************************/

static void
initLaserHardwareSettings( int robotType);

static void
integrateLasers( informationsFor_LASER* info);

static void
computeDistProbFunctionParameter_LASER( sensorParameters *params);

static void
initLaserSensings( actionInformation* actionInfo,
		   sensingActionMask* actionMask,
		   int numberOfFirstLaser,
		   int laser);

static abstractSensorType
abstractLaserSensor( actionInformation* actionInfo,
		     int laserNumber,
		     int laser);

/**************************************************************************
 **************************************************************************
 * Mandatory functions.
 **************************************************************************
 **************************************************************************/

void
initialize_LASER( char* fileName,
		  actionInformation* actionInfo,
		  sensingActionMask* actionMask,
		  sensingFunctions* handlers)
{
  extern mapParameters globalMapParameters;
  /* Local variables initialized in the parameter file. */
  char distFile[MAX_STRING_LENGTH];
  bool useLaser, computeExpectedDist, chooseOptimalSensor;
  int numberOfAngles;
  int numberOfFirstLaser;

  /* NICKR */
  float cutDistance = 0.0;
  
  
  /* This struct contains all relevant information for later integration
   * of laser informations. */
  informationsFor_LASER* info = 
    (informationsFor_LASER*) malloc( sizeof( informationsFor_LASER));

  token tok[NUMBER_OF_LASER_PARAMETERS];


  /*-------------------------------------------------------------------------
   * Default values
   *------------------------------------------------------------------------*/

  useLaser = 1;
  computeExpectedDist = 0;
  sprintf(distFile, "%s%s", globalMapParameters.mapFileName, ".laserDist");
  numberOfAngles = 360;
  chooseOptimalSensor = 0;
  globalLaserParameters.numberOfLasersToBeUsed = 30;
  numberOfFirstLaser = 0;
  globalLaserParameters.integrateThreshold = 40;
  globalLaserParameters.maxQuotaOfPlanes = 0.4;
  globalLaserParameters.numberOfLasersForConvolve = 90;
  globalLaserParameters.maxNumberOfIntegrationsPerStop = 2;
  globalLaserParameters.maxRotVelForIntegration = 0.0;

  /* NICKR */
  globalLaserParameters.cutFeature = 0;
  maxFactor_LASER = 1.3;

  setTokensInitialized(tok, NUMBER_OF_LASER_PARAMETERS);
  
  /*-------------------------------------------------------------------------
   * Get the parameters from the file. 
   *------------------------------------------------------------------------*/

  tok[USE_LASER_TOKEN].format   = INT_FORMAT;
  tok[USE_LASER_TOKEN].variable = &useLaser;
  tok[USE_LASER_TOKEN].keyWord  = USE_LASER_KEYWORD;

  tok[LASER_COMPUTE_EXPECTED_DIST_TOKEN].format   = INT_FORMAT;
  tok[LASER_COMPUTE_EXPECTED_DIST_TOKEN].variable = &computeExpectedDist;
  tok[LASER_COMPUTE_EXPECTED_DIST_TOKEN].keyWord  = LASER_COMPUTE_EXPECTED_DIST_KEYWORD;
  
  tok[LASER_EXPECTED_DIST_FILE_TOKEN].format   = STRING_FORMAT;
  tok[LASER_EXPECTED_DIST_FILE_TOKEN].variable = distFile;
  tok[LASER_EXPECTED_DIST_FILE_TOKEN].keyWord  = LASER_EXPECTED_DIST_KEYWORD;
  
  tok[LASER_NUMBER_OF_EXPECTED_DIST_ANGLES_TOKEN].format   = INT_FORMAT;
  tok[LASER_NUMBER_OF_EXPECTED_DIST_ANGLES_TOKEN].variable = &numberOfAngles;
  tok[LASER_NUMBER_OF_EXPECTED_DIST_ANGLES_TOKEN].keyWord  = LASER_NUMBER_OF_EXPECTED_DIST_ANGLES_KEYWORD;
  
  tok[LASER_CHOOSE_OPTIMAL_SENSOR].format   = INT_FORMAT;
  tok[LASER_CHOOSE_OPTIMAL_SENSOR].variable = &chooseOptimalSensor;
  tok[LASER_CHOOSE_OPTIMAL_SENSOR].keyWord  = LASER_CHOOSE_OPTIMAL_SENSOR_KEYWORD;

  tok[NUMBER_OF_LASERS_TO_BE_USED_TOKEN].format   = INT_FORMAT;
  tok[NUMBER_OF_LASERS_TO_BE_USED_TOKEN].variable =
    &(globalLaserParameters.numberOfLasersToBeUsed);
  tok[NUMBER_OF_LASERS_TO_BE_USED_TOKEN].keyWord  = NUMBER_OF_LASERS_TO_BE_USED_KEYWORD;
  
  tok[NUMBER_OF_FIRST_LASER_TOKEN].format   = INT_FORMAT;
  tok[NUMBER_OF_FIRST_LASER_TOKEN].variable = &numberOfFirstLaser;
  tok[NUMBER_OF_FIRST_LASER_TOKEN].keyWord  = NUMBER_OF_FIRST_LASER_KEYWORD;
  
  tok[LASER_INTEGRATE_THRESHOLD_TOKEN].format   = FLOAT_FORMAT;
  tok[LASER_INTEGRATE_THRESHOLD_TOKEN].variable = &(globalLaserParameters.integrateThreshold);
  tok[LASER_INTEGRATE_THRESHOLD_TOKEN].keyWord  = LASER_INTEGRATE_THRESHOLD_KEYWORD;
  
  tok[LASER_MAX_QUOTA_OF_PLANES_TOKEN].format   = FLOAT_FORMAT;
  tok[LASER_MAX_QUOTA_OF_PLANES_TOKEN].variable = &(globalLaserParameters.maxQuotaOfPlanes);
  tok[LASER_MAX_QUOTA_OF_PLANES_TOKEN].keyWord  = LASER_MAX_QUOTA_OF_PLANES_KEYWORD;

  tok[LASER_MAX_FACTOR_TOKEN].format   = FLOAT_FORMAT;
  tok[LASER_MAX_FACTOR_TOKEN].variable = &(maxFactor_LASER);
  tok[LASER_MAX_FACTOR_TOKEN].keyWord  = LASER_MAX_FACTOR_KEYWORD;

  tok[NUMBER_OF_LASERS_FOR_CONVOLVE_TOKEN].format   = INT_FORMAT;
  tok[NUMBER_OF_LASERS_FOR_CONVOLVE_TOKEN].variable =
    &(globalLaserParameters.numberOfLasersForConvolve);
  tok[NUMBER_OF_LASERS_FOR_CONVOLVE_TOKEN].keyWord  = NUMBER_OF_LASERS_FOR_CONVOLVE_KEYWORD;
  
  tok[MAX_NUMBER_OF_INTEGRATIONS_PER_STOP_TOKEN].format   = INT_FORMAT;
  tok[MAX_NUMBER_OF_INTEGRATIONS_PER_STOP_TOKEN].variable =
    &(globalLaserParameters.maxNumberOfIntegrationsPerStop);
  tok[MAX_NUMBER_OF_INTEGRATIONS_PER_STOP_TOKEN].keyWord  = MAX_NUMBER_OF_INTEGRATIONS_PER_STOP_KEYWORD;
  
  /* NICKR */
  tok[LASER_CUT_DISTANCE_TOKEN].format   = FLOAT_FORMAT;
  tok[LASER_CUT_DISTANCE_TOKEN].variable = &cutDistance;
  tok[LASER_CUT_DISTANCE_TOKEN].keyWord  = LASER_CUT_DISTANCE_KEYWORD;
    
  tok[MAX_ROT_VEL_FOR_LASERS_TOKEN].format   = FLOAT_FORMAT;
  tok[MAX_ROT_VEL_FOR_LASERS_TOKEN].variable = &(globalLaserParameters.maxRotVelForIntegration);
  tok[MAX_ROT_VEL_FOR_LASERS_TOKEN].keyWord  = MAX_ROT_VEL_FOR_LASERS_KEYWORD;
    
  tok[LASER_MAX_EXPECTED_DISTANCE_TOKEN].format   = INT_FORMAT;
  tok[LASER_MAX_EXPECTED_DISTANCE_TOKEN].variable = &LASER_MAX_EXPECTED_DISTANCE;
  tok[LASER_MAX_EXPECTED_DISTANCE_TOKEN].keyWord  = LASER_MAX_EXPECTED_DISTANCE_KEYWORD;

  tok[LASER_NUMBER_OF_STDDEVS_TOKEN].format   = INT_FORMAT;
  tok[LASER_NUMBER_OF_STDDEVS_TOKEN].variable = &LASER_NUMBER_OF_STDDEVS;
  tok[LASER_NUMBER_OF_STDDEVS_TOKEN].keyWord  = LASER_NUMBER_OF_STDDEVS_KEYWORD;
  
  readTokens( fileName, tok, NUMBER_OF_LASER_PARAMETERS, FALSE); 

  
  /* Convert from degree to radiant. */
  globalLaserParameters.maxRotVelForIntegration =
    deg2Rad( globalLaserParameters.maxRotVelForIntegration);
  
  /*-------------------------------------------------------------------------
   * done.
   *------------------------------------------------------------------------*/

  initLaserHardwareSettings( robotType);
  
  if ( useLaser) {

    if ( useLaser == 2)
      globalLaserParameters.useRearLaser  = FALSE;
    else if ( useLaser == 3) 
      globalLaserParameters.useFrontLaser = FALSE;

    /*------------------------------------------------------------------------
     * Get the preprocessed tables to look up the probabilities of the measurements.
     *-----------------------------------------------------------------------*/

    computeDistProbFunctionParameter_LASER( &(globalLaserParameters.parameters));

    /* Only need maps is not mapping online. */
    if ( ! actionInfo->onlineMapping) {

      if (!readGridMap( globalMapParameters.mapFileName,
			LASER_MAP_EXTENSION,
			&actionInfo->laserMap)){
	if (actionInfo->simMap.initialized){
	  computeGridMap( &actionInfo->simMap,
			  globalMapParameters.desiredResolution,
			  0.0,
			  globalLaserParameters.parameters.sensorHeight,
			  globalLaserParameters.parameters.sensorHeight,
			  &(actionInfo->laserMap));
	  (void) writeGridMap( globalMapParameters.mapFileName,
			       LASER_MAP_EXTENSION,
			       &(actionInfo->laserMap));
	}
	else
	  actionInfo->laserMap = actionInfo->map;
      }
      actionInfo->laserMap.initialized = TRUE;
      actionInfo->laserMap.offsetX = actionInfo->map.offsetX;
      actionInfo->laserMap.offsetY = actionInfo->map.offsetY;
      
      checkMapConsistency(actionInfo->map.sizeX,actionInfo->map.sizeY,
			  actionInfo->map.resolution,
			  actionInfo->laserMap.sizeX,
			  actionInfo->laserMap.sizeY,
			  actionInfo->laserMap.resolution,
			  globalMapParameters.mapFileName,
			  GRID_MAP_EXTENSION, LASER_MAP_EXTENSION);
      
      /*------------------------------------------------------------------------
       * Set the pointers to the structures provided in actionInfo.
       *-----------------------------------------------------------------------*/
      info->general.useProbGrid          = &(actionInfo->useProbGrid);
      info->general.grid                 = &(actionInfo->positionProbs);
      info->general.samples              = &(actionInfo->samples);
      info->general.initialPositionProbs = &(actionInfo->initialPositionProbs);
      info->general.localMaxima          = &(actionInfo->localMaxima);
      info->general.numberOfIntegratedReadings = 0;
      info->general.map                  = &(actionInfo->laserMap);
      info->general.simMap               = &(actionInfo->simMap);
      info->general.scan                 = &(actionInfo->actualSensings.frontLaser);
    }
    /* For online mapping the lasermap is just the online map. */
    else {
      info->general.useProbGrid          = &(actionInfo->useProbGrid);
      info->general.grid                 = &(actionInfo->positionProbs);
      info->general.samples              = &(actionInfo->samples);
      info->general.initialPositionProbs = &(actionInfo->initialPositionProbs);
      info->general.localMaxima          = &(actionInfo->localMaxima);
      info->general.numberOfIntegratedReadings = 0;
      info->general.scan                 = &(actionInfo->actualSensings.frontLaser);
      actionInfo->laserMap               = actionInfo->onlineMap;
      info->general.map                  = &(actionInfo->laserMap);
    }
    
    info->general.onlineMapping        = actionInfo->onlineMapping;
    
    /* Values necessary to determine wether optimal sensor has to
     * be chosen from a scan. */
    if ( actionInfo->onlineMapping && chooseOptimalSensor) {
      fprintf( stderr, "# \tSorry. Sensor filter not implemented for online mapping.\n");
      writeLog( "#Sorry. Sensor filter not implemented for online mapping.\n");
      chooseOptimalSensor = FALSE;
    }
    
    actionInfo->abstractSensors[ABSTRACT_FRONT_LASER].chooseOptimal =
      chooseOptimalSensor;
    actionInfo->abstractSensors[ABSTRACT_REAR_LASER].chooseOptimal =
      chooseOptimalSensor;

    
    /* Set the pointers on the abstract scans. */
    info->general.abstractScan = &(actionInfo->abstractSensors[ABSTRACT_FRONT_LASER]); 
    info->abstractFrontScan    = &(actionInfo->abstractSensors[ABSTRACT_FRONT_LASER]);
    info->abstractRearScan     = &(actionInfo->abstractSensors[ABSTRACT_REAR_LASER]);

    initializeDistProbTables( &(info->general),
			      actionInfo,
			      numberOfAngles,
			      globalLaserParameters.parameters,
			      &(actionInfo->laserMap),
			      distFile,
			      computeExpectedDist);
    
    /* NICKR Set the maximal feature to be used. */
    if ( cutDistance > 0.0) {
      globalLaserParameters.cutFeature =
	cutDistance /
	globalLaserParameters.parameters.sensorMaxRange *
	globalLaserParameters.parameters.numberOfExpectedDistances;

      writeLog("# Cut laser at distance %f. Feature: %d.\n",
	       cutDistance, globalLaserParameters.cutFeature);
    }
    actionMask->numberOfActions[LASER] = NUMBER_OF_ACTIONS_LASER;
    
    /* Set the resolution of the prob grid to the resolution of the
     * expected distances. */
    if ( ! actionInfo->useProbGrid) {
      actionInfo->positionProbs.positionResolution =
	info->general.expectedDistances.gridCellSize;
      writeLog( "#Laser: don't use prob grid. Set resolution to %d cm.\n",
		actionInfo->positionProbs.positionResolution);
    }
  }
  else {
    globalLaserParameters.useFrontLaser = FALSE;
    globalLaserParameters.useRearLaser  = FALSE;;
    actionMask->numberOfActions[LASER] = 0;
  }
  
  /* Pointers on information about integration. */
  if ( globalLaserParameters.useFrontLaser)
    actionInfo->abstractSensors[ABSTRACT_FRONT_LASER].integrate =
      &(actionMask->perform[LASER][INTEGRATE_LASER]);
  else
    actionInfo->abstractSensors[ABSTRACT_FRONT_LASER].integrate =
      &addressForNoIntegration;

  if ( globalLaserParameters.useRearLaser)
    actionInfo->abstractSensors[ABSTRACT_REAR_LASER].integrate =
      &(actionMask->perform[LASER][INTEGRATE_LASER]);
  else
    actionInfo->abstractSensors[ABSTRACT_REAR_LASER].integrate =
      &addressForNoIntegration;


  /*------------------------------------------------------------------------
   * Initialze the handlers.
   *-----------------------------------------------------------------------*/
  handlers->checkIfConsider[LASER]            = checkIfConsider_LASER;
  handlers->checkWhichActionsToPerform[LASER] = checkWhichActionsToPerform_LASER;
  handlers->performActions[LASER]             = performActions_LASER;
  
  /*------------------------------------------------------------------------
   * Initialze the mask.
   *-----------------------------------------------------------------------*/
  actionMask->use[LASER] = useLaser;

  /* Now place the structs in the global information struct. */
  /* All the same apart from the scans. */
  actionInfo->info[LASER] = info;

  /* Initialze the global structures of laser sensings. */
  initLaserSensings( actionInfo,
		     actionMask,
		     numberOfFirstLaser,
		     FRONT_LASER);

  /* Initialze the global structures of laser sensings. */
  initLaserSensings( actionInfo,
		     actionMask,
		     numberOfFirstLaser,
		     REAR_LASER);

  /* Install the simulator map for online computation of the expected distances. */
  /*  if ( ! actionInfo->useProbGrid && actionInfo->simMap.initialized) {
    FILE* fp = fopen( actionInfo->simMap.fileName, "r");
    installSimMap( fp);
    fclose(fp);
  }
  */
}


/**************************************************************************
 * Check wether the laser readings shall be considered for integration.
 **************************************************************************/
void
checkIfConsider_LASER( actionInformation* actionInfo,
		       sensingActionMask* mask)
{
  /* Just check for the threshold. */
  if ( actionInfo->useProbGrid)
    mask->consider[LASER] = mask->use[LASER]
      && ( !mask->consider[ANGLE] || actionInfo->positionProbs.quotaOfPlanesToBeUpdated
	   < globalLaserParameters.maxQuotaOfPlanes);
  else
    mask->consider[LASER] = mask->use[LASER];
}


/**************************************************************************
 * Check which actions shall be performed on the position probability grid.
 **************************************************************************/
void
checkWhichActionsToPerform_LASER( actionInformation* info,
				  sensingActionMask* mask)
{
  static int numberOfIntegratedReadingsSinceConvolve = 0;

  mask->perform[LASER][INTEGRATE_LASER] = FALSE;

  if (! globalLaserParameters.useFrontLaser)
    info->actualSensings.frontLaser.isNew = FALSE;
  if (! globalLaserParameters.useRearLaser)
    info->actualSensings.rearLaser.isNew = FALSE;

  /* Check wether the laser reading has to be integrated.
   * This has to be done after enough movement and only with new
   * laser readings. */
  if ( mask->consider[LASER] && info->actualSensings.frontLaser.isNew) {
    
    /* Convolve the grid after each numberOfLasersForConvolve readings. */
    if ( globalLaserParameters.numberOfLasersForConvolve > 0) {

      informationsFor_PROXIMITY *proximityInfo = info->info[LASER];
      int laserReadings = proximityInfo->numberOfIntegratedReadings;
      
      if ( (info->localMaxima.numberOfCells == 1)
	   &&
	   (laserReadings - numberOfIntegratedReadingsSinceConvolve
	    > globalLaserParameters.numberOfLasersForConvolve)) {
	
	numberOfIntegratedReadingsSinceConvolve = laserReadings;	          
	mask->perform[MOVEMENT][CONVOLVE_POS_GRID] = TRUE;
	info->summedMovements[MOVEMENT][CONVOLVE_POS_GRID] = 0.0;
      }
    }

    /* Don't integrate if the robot rotates too fast. */
    if ( globalLaserParameters.maxRotVelForIntegration > 0.0 &&
	 (fabs(info->actualSensings.rotVelocity)
	  > globalLaserParameters.maxRotVelForIntegration)) {
      writeLog( "#Rotation (%f) too fast for laser integration.\n",
		rad2Deg(info->actualSensings.rotVelocity));
      return;
    }
    
    /* Has the robot moved enough? */
    if ( info->summedMovements[LASER][INTEGRATE_LASER]
	 > globalLaserParameters.integrateThreshold) {
      mask->perform[LASER][INTEGRATE_LASER] = TRUE;
      info->summedMovements[LASER][INTEGRATE_LASER] = 0.0;
      mask->perform[MOVEMENT][INTEGRATE_MOVEMENT] = TRUE;
      writeLog( "%f %f #las\n", info->summedMovements[LASER][INTEGRATE_LASER],
		globalLaserParameters.integrateThreshold);
      info->summedMovements[MOVEMENT][INTEGRATE_MOVEMENT] = 0.0;
      numberOfLaserIntegrationsSinceStopped = 0;
    }
    
    /* Lasers must not be integrated too often if the robot doesn't move. */
    else if ( info->actualSensings.noMoveCnt > 0) {
      if ( numberOfLaserIntegrationsSinceStopped < globalLaserParameters.maxNumberOfIntegrationsPerStop){
	writeLog("#nomove: %d\n", info->actualSensings.noMoveCnt);
	mask->perform[LASER][INTEGRATE_LASER] = TRUE;
	info->summedMovements[LASER][INTEGRATE_LASER] = 0.0;
	numberOfLaserIntegrationsSinceStopped++;      
      }
    }
    if ( mask->perform[LASER][INTEGRATE_LASER]) {
      info->abstractSensors[ABSTRACT_FRONT_LASER].numberOfSensorsToBeUsed =
	globalLaserParameters.numberOfLasersToBeUsed;
      info->abstractSensors[ABSTRACT_REAR_LASER].numberOfSensorsToBeUsed =
	globalLaserParameters.numberOfLasersToBeUsed;
    }
  }
}


/**************************************************************************
 * Perform the actions on  the position probability grid.
 **************************************************************************/
void
performActions_LASER( actionInformation* actionInfo,
		      sensingActionMask* mask)
{

  /* This structure contains all relevant information for the actions. */
  informationsFor_LASER* info =
    (informationsFor_LASER*) actionInfo->info[LASER];
  
  /* #define DDD */
#ifdef DDD
  return;
#endif
  
  if ( mask->perform[LASER][INTEGRATE_LASER]) {
    integrateLasers( info);
    actionInfo->proximityIntegrated = TRUE;
  }
}

/********************************************************************
 ********************************************************************
 * Global functions.
 ********************************************************************
 ********************************************************************/

float
laserRot( int laserReading, int laser) {

  static float laserRotTab[MAX_NUMBER_OF_LASERS][MAX_SIZE_OF_LASER_SCAN];
  static bool firstCall = TRUE;
  
  if (firstCall){

    int lCnt;
    float angleStep = DEG_180 / MAX_SIZE_OF_LASER_SCAN;
    
    for ( lCnt = 0; lCnt < MAX_SIZE_OF_LASER_SCAN; lCnt++) {
      
      laserRotTab[FRONT_LASER][lCnt] =
	normalizedAngle(FRONT_LASER_START_ANGLE + lCnt * angleStep);

      laserRotTab[REAR_LASER][lCnt]  =
	normalizedAngle(REAR_LASER_START_ANGLE + lCnt * angleStep);
    }
    firstCall = FALSE;
  }

  return laserRotTab[laser][laserReading];
}



/********************************************************************
 ********************************************************************
 * The local functions.
 ********************************************************************
 ********************************************************************/



/***********************************************************************
 * Integrates the laser probabilities in the grid.
 ***********************************************************************/
static void
integrateLasers( informationsFor_LASER* info)
{
  /* Just for information. */
  static int doLaserCnt = 0;

  setTimer( 0);
    
  writeLog("#        Lasers no.%d ", ++doLaserCnt);
  writeLog( "... ");
  fprintf( stderr, "# *************************************************\n");
  fprintf( stderr,"#        Lasers no.%d ", doLaserCnt);
  fprintf( stderr, "... ");

  /*-------------------------------------------------------------------
   * This is the main function.
   *-------------------------------------------------------------------*/

  if ( globalLaserParameters.useFrontLaser &&
       globalLaserParameters.useRearLaser) {

    /* #define TWO_LASERS_AT_ONCE  */
#ifdef TWO_LASERS_AT_ONCE
    informationsFor_LASER rearInfo = *info;
    informationsFor_LASER frontInfo = *info;
    
    frontInfo.general.abstractScan = info->abstractFrontScan; 
    rearInfo.general.abstractScan = info->abstractRearScan; 
    
    integrateTwoDistScans( &( frontInfo.general), &( rearInfo.general));
#else
    info->general.abstractScan = info->abstractFrontScan; 
    integrateDistScan( &( info->general));
    info->general.abstractScan = info->abstractRearScan;
    integrateDistScan( &( info->general));
#endif
  }
  else {
    
    if ( globalLaserParameters.useFrontLaser) {
      info->general.abstractScan = info->abstractFrontScan;       
      integrateDistScan( &( info->general));
    }
    if ( globalLaserParameters.useRearLaser) {
      info->general.abstractScan = info->abstractRearScan;
      integrateDistScan( &( info->general));
    }
  }

  if ( info->general.useProbGrid) {
    writeLog( "done (%.2f%c in %.2f secs).\n",
	      100.0 * info->general.grid->quotaOfValuesToBeUpdated,
	      '%', timeExpired(0));
    fprintf(stderr, "done (%.2f%c in %.2f secs).\n",
	    100.0 * info->general.grid->quotaOfValuesToBeUpdated,
	    '%', timeExpired(0));
    fprintf( stderr, "# *************************************************\n");
  }
  else {
    writeLog( "done in %.2f secs.\n",
	      timeExpired(0));
    fprintf(stderr, "done in %.2f secs.\n",
	    timeExpired(0));
    fprintf( stderr, "# *************************************************\n");
  }
}


distance
obstacleDistance_LASER( probabilityGrid map,
		       int x, int y, float rot,
		       distance maxRange)
{
  realPosition pos;
  
  pos.x = realCoordinateOfMapCoordinate(x, map.offsetX, map.resolution);
  pos.y = realCoordinateOfMapCoordinate(y, map.offsetY, map.resolution);
  pos.rot = rot;
  return obstacleDistanceReal(map, pos, maxRange);
}


static float *
laserStdDevThreshold( int numberOfStdDevs){
  float *stdDev;
  
  if ( numberOfStdDevs > LASER_MAX_NUMBER_OF_STDDEVS){
    fprintf(stderr,
	    "# Error: more than %d stdandard deviations are not allowed for lasers\n",
	    LASER_MAX_NUMBER_OF_STDDEVS);
    closeLogAndExit(1);
  }
  
  stdDev = (float *) allocate1D(numberOfStdDevs, FLOAT);
  
  switch (numberOfStdDevs){
  case 1:
    stdDev[0] = MAXFLOAT;
    break;
  case 2:
    stdDev[0] = 20;
    stdDev[1] = MAXFLOAT;
  }
  return stdDev;
}


static void
computeDistProbFunctionParameter_LASER( sensorParameters *params)
{
  params->numberOfStdDevs = LASER_NUMBER_OF_STDDEVS;
  params->numberOfExpectedDistances =
    params->numberOfMeasuredDistances =
    LASER_MAX_NUMBER_OF_EXPECTED_DISTANCES / params->numberOfStdDevs;
  if (params->numberOfStdDevs * params->numberOfMeasuredDistances
      !=
      LASER_MAX_NUMBER_OF_EXPECTED_DISTANCES){
    fprintf(stderr,
	   "# Error: %d is not admissible as number of standard deviations for laser\n",
	   params->numberOfStdDevs);
    closeLogAndExit(1);
  }
  
  params->sensorMaxRange = LASER_MAX_EXPECTED_DISTANCE;
  params->openingAngle = LASER_OPENING_ANGLE;
  params->sensorHeight = LASER_HEIGHT;
  params->probFunctionFileName = LASER_PROBFUNCTION_FILENAME;
  params->maxFactor = maxFactor_LASER;
  params->stdDevThreshold = laserStdDevThreshold(params->numberOfStdDevs);
  
}



/***********************************************************************
 * Converts a raw laser reading into a laser feature.
 **********************************************************************/
static int
extractedFrontLaserFeature( rawSensings* rawData,
			    int sensorNumber,
			    void* infoForFeature)
{
  distance dist = rawData->frontLaser.reading[sensorNumber].dist;
  infoForProximityFeature* info = (infoForProximityFeature*) infoForFeature;
  distProbTable *distProbs = info->distProbFunctionTableNormed;

  return iMin( distProbs->numberOfMeasuredDistances - 1,
	       (int) round (dist / distProbs->distanceResolution));
}

/***********************************************************************
 * Converts a raw sonar reading into a sonar feature.
 **********************************************************************/
static int
extractedRearLaserFeature( rawSensings* rawData,
			   int sensorNumber,
			   void* infoForFeature)
{
  distance dist = rawData->rearLaser.reading[sensorNumber].dist;
  infoForProximityFeature* info = (infoForProximityFeature*) infoForFeature;
  distProbTable *distProbs = info->distProbFunctionTableNormed;
  
  return iMin( distProbs->numberOfMeasuredDistances - 1,
	       (int) round (dist / distProbs->distanceResolution));
}


/***********************************************************************
 * Initializes the structures containing the raw laser data as well
 * as the laser data in abstract sensor structures.
 **********************************************************************/
static void
initLaserSensings( actionInformation* actionInfo,
		   sensingActionMask* actionMask,
		   int numberOfFirstLaser,
		   int laser)
{
  int i;
  sensing_PROXIMITY* rawLaser;
  abstractSensorVector* abstractLaser;

  if ( laser == FRONT_LASER) {
    rawLaser = &( actionInfo->actualSensings.frontLaser);
    rawLaser->offset.forward = frontLaserOffset;    
    abstractLaser = &( actionInfo->abstractSensors[ABSTRACT_FRONT_LASER]);
  }
  else {
    rawLaser = &( actionInfo->actualSensings.rearLaser);
    rawLaser->offset.forward = rearLaserOffset;
    abstractLaser = &( actionInfo->abstractSensors[ABSTRACT_REAR_LASER]);
  }
    
  /*------------------------------------------------------------------------
   * Initialze the global structure of raw sensors.
   *-----------------------------------------------------------------------*/
  rawLaser->numberOfReadings = MAX_SIZE_OF_LASER_SCAN;
  rawLaser->offset.sideward  = 0.0;
  rawLaser->offset.rotation  = 0.0;
  rawLaser->reading          = (distanceReading*)
    malloc( MAX_SIZE_OF_LASER_SCAN * sizeof( distanceReading));
  rawLaser->readingOffset          = (movement*)
    malloc( MAX_SIZE_OF_LASER_SCAN * sizeof( movement));
  if (rawLaser->reading == NULL || rawLaser->readingOffset == NULL){
    fprintf(stderr, "# Error: could not allocate laser readings\n");
    closeLogAndExit(0);
  }
			   
  rawLaser->maxDistance      = LASER_MAX_EXPECTED_DISTANCE;
  rawLaser->isNew            = FALSE;

  for ( i = 0; i < MAX_SIZE_OF_LASER_SCAN; i++){
    rawLaser->reading[i].rot = laserRot(i, laser);
    rawLaser->readingOffset[i] = rawLaser->offset;
    rawLaser->readingOffset[i].rotation = rawLaser->reading[i].rot;
  }
  
  /*------------------------------------------------------------------------
   * Initialze the global structure of abstract sensors.
   *-----------------------------------------------------------------------*/
  if ( actionMask->use[LASER]) {
    abstractLaser->numberOfSensors = MAX_SIZE_OF_LASER_SCAN;
    abstractLaser->sensor  = (abstractSensorType*)
      malloc ( MAX_SIZE_OF_LASER_SCAN * sizeof( abstractSensorType));
    
    for ( i = 0; i < MAX_SIZE_OF_LASER_SCAN; i++) {
      abstractLaser->sensor[i] = abstractLaserSensor( actionInfo, i, laser);
    }
    
    abstractLaser->numberOfSensorsToBeUsed =
      globalLaserParameters.numberOfLasersToBeUsed;
    abstractLaser->mask = (int*) allocate1D( abstractLaser->numberOfSensors, INT);
    abstractLaser->sensorOffsetType = SCAN_OFFSET;
    
    /* Pointers on information about integration. */
    if ( (laser == FRONT_LASER && globalLaserParameters.useFrontLaser) ||
       (laser == REAR_LASER && globalLaserParameters.useRearLaser))
      abstractLaser->integrate =
	&(actionMask->perform[LASER][INTEGRATE_LASER]);
    else
      abstractLaser->integrate = &addressForNoIntegration;
    
    setFixedScanMask( abstractLaser, numberOfFirstLaser);
  }
  else 
    abstractLaser->numberOfSensors = 0;
}


/***********************************************************************
 * Returns a laser sensor. This sensor is used to compute the a posteriori
 * entropy after having integrated such a sensor into the prob grid.
 **********************************************************************/
static abstractSensorType
abstractLaserSensor( actionInformation* actionInfo,
		     int laserNumber,
		     int laser)
{
  abstractSensorType sensor;
  infoForProximityFeature* info;
  informationsFor_LASER* laserInfo = (informationsFor_LASER*) actionInfo->info[LASER];
  static float* uninformedNormed;
  static float* uninformedUnNormed;
  static float firstCall = TRUE;

  info = (infoForProximityFeature*) malloc( sizeof(infoForProximityFeature));
  
  /* We don't compute this for each laser beam. */
  if (firstCall) {
    uninformedNormed =
      allocate1D( laserInfo->general.distProbFunctionTableNormed.numberOfStdDevs, FLOAT);
    
    averageProbabilities( &(laserInfo->general.distProbFunctionTableNormed),
			  uninformedNormed);
    uninformedUnNormed =
      allocate1D( laserInfo->general.distProbFunctionTableDividedByPOfFeature.numberOfStdDevs, FLOAT);
    
    averageProbabilities( &(laserInfo->general.distProbFunctionTableDividedByPOfFeature),
			  uninformedUnNormed);
    firstCall = FALSE;
  }

  info->grid                   = &(actionInfo->positionProbs);
  info->distTab                = &(laserInfo->general.expectedDistances);
  info->samples                = &(actionInfo->samples);

  info->distProbFunctionTableNormed
    = &(laserInfo->general.distProbFunctionTableNormed);
  info->distProbFunctionTableDividedByPOfFeature
    = &(laserInfo->general.distProbFunctionTableDividedByPOfFeature);
  info->distProbFunctionTable  = info->distProbFunctionTableDividedByPOfFeature;
  info->sensorRot              = laserRot( laserNumber, laser);
  info->uninformedNormed       = uninformedNormed;
  info->uninformedDividedByPOfFeature = uninformedUnNormed;

  info->gridMap                = &(actionInfo->laserMap);
  info->simMap                 = &(actionInfo->simMap);
  
  /* Offset of the sensors relative to the center of the robot. */
  sensor.integrationInfo.sensorOffset.sideward        = 0.0;
  sensor.integrationInfo.sensorOffset.rotation        = 0.0;
  
  sensor.sensorNumberOfType = laserNumber;
  sensor.numberOfFeatures   =
    laserInfo->general.expectedDistances.numberOfExpectedDistances;
  sensor.probOfFeatureGivenPosition = &probabilityOfProximityFeatureGivenPosition;
  sensor.probOfFeatureGivenExpected = &probabilityOfProximityFeatureGivenExpected;
  sensor.expectedFeature    = &expectedProximityFeature;
  sensor.infoForFeatures    = info;
  sensor.uninformedFeatureProbability = uninformedUnNormed;

  sensor.setNormedFeatureProbs = &setNormedDistTab;
  sensor.setUnNormedFeatureProbs = &setUnNormedDistTab;

  if ( laser == FRONT_LASER) {
    sensor.integrationInfo.sensorOffset.forward  = frontLaserOffset;
    sensor.type                  = FRONT_LASER_TYPE;
    sensor.extractedFeature      = &extractedFrontLaserFeature;
  }
  else {
    sensor.integrationInfo.sensorOffset.forward  = rearLaserOffset;
    sensor.type                  = REAR_LASER_TYPE;
    sensor.extractedFeature      = &extractedRearLaserFeature;
  }
  return sensor;
}


/* This function sets the number and position of the sonars. */
static void
initLaserHardwareSettings( int robotType)
{
  /* The front laser is always used. */
  globalLaserParameters.useFrontLaser = TRUE;
  
  if ( robotType == B21_ONE_LASER_ROBOT) {
    writeLog( "# Initialise laser settings to robot type B21_ONE_LASER_ROBOT.\n");
    fprintf( stderr, "# Initialise laser settings to robot type B21_ONE_LASER_ROBOT.\n");
    frontLaserOffset = B21_ONE_LASER_OFFSET;
    rearLaserOffset = 0.0;
    globalLaserParameters.useRearLaser = FALSE;
  }
  else if ( robotType == B21_TWO_LASERS_ROBOT) {
    writeLog( "# Initialise laser settings to robot type B21_TWO_LASERS_ROBOT.\n");
    fprintf( stderr, "# Initialise laser settings to robot type B21_TWO_LASERS_ROBOT.\n");
    frontLaserOffset = B21_TWO_LASERS_FRONT_LASER_OFFSET;
    rearLaserOffset = B21_TWO_LASERS_REAR_LASER_OFFSET;
    globalLaserParameters.useRearLaser = TRUE;
  }
  else if ( robotType == B18_ROBOT) {
    writeLog( "# Initialise laser settings to robot type B18_ROBOT.\n");
    fprintf( stderr, "# Initialise laser settings to robot type B18_ROBOT.\n");
    frontLaserOffset = B18_FRONT_LASER_OFFSET;
    rearLaserOffset = B18_REAR_LASER_OFFSET;
    globalLaserParameters.useRearLaser = TRUE;
  }
  else if ( robotType == PIONEER_ATRV) {
    writeLog( "# Initialise laser settings to robot type PIONEER_ATRV.\n");
    fprintf( stderr, "# Initialise laser settings to robot type PIONEER_ATRV.\n");
    frontLaserOffset = PIONEER_ATRV_LASER_OFFSET;
    rearLaserOffset = 0.0;
    globalLaserParameters.useRearLaser = FALSE;
  }
  else if ( robotType == URBAN_ROBOT) {
    writeLog( "# Initialise laser settings to robot type URBAN_ROBOT.\n");
    fprintf( stderr, "# Initialise laser settings to robot type URBAN_ROBOT.\n");
    frontLaserOffset = URBAN_ROBOT_LASER_OFFSET;
    rearLaserOffset = 0.0;
    globalLaserParameters.useRearLaser = FALSE;
  }
  else if ( robotType == PIONEER_II) {
    writeLog( "# Initialise laser settings to robot type PIONEER_II.\n");
    fprintf( stderr, "# Initialise laser settings to robot type PIONEER_II.\n");
    frontLaserOffset = PIONEER_II_LASER_OFFSET;
    rearLaserOffset = 0.0;
    globalLaserParameters.useRearLaser = FALSE;
  }
  else if ( robotType == SCOUT) {
    writeLog( "# Initialise laser settings to robot type Scout.\n");
    fprintf( stderr, "# Initialise laser settings to robot type Scout.\n");
    frontLaserOffset = SCOUT_LASER_OFFSET;
    rearLaserOffset = 0.0;
    globalLaserParameters.useRearLaser = FALSE;
  }
  else if ( robotType == XR4000) {
    writeLog( "# Initialise laser settings to robot type Xr400.\n");
    fprintf( stderr, "# Initialise laser settings to robot type Xr400.\n");
    frontLaserOffset = XR4000_LASER_OFFSET;
    rearLaserOffset = 0.0;
    globalLaserParameters.useRearLaser = FALSE;
  }
  else {
    fprintf( stderr, "Laser for robot type %d not supported.\n", robotType);
    closeLogAndExit( 0);
  }
}


