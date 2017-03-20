
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/angle.c,v $
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
 * $Log: angle.c,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.28  1999/12/16 16:13:58  fox
 * Several preparation changes for angles.
 *
 * Revision 1.27  1999/12/15 16:16:39  fox
 * First attempt to extract p(l | o).
 *
 * Revision 1.26  1998/03/09 09:36:26  wolfram
 * LOCALIZE now checks the consistency of the various maps.
 *
 * Revision 1.25  1997/11/27 18:11:16  fox
 * Several changes to make angles work better.
 *
 * Revision 1.24  1997/11/20 12:58:07  fox
 * Version with good sensor selection.
 *
 * Revision 1.23  1997/10/03 12:25:11  wolfram
 * slight changes
 *
 * Revision 1.22  1997/10/02 09:20:27  wolfram
 * Better initialization for angles
 *
 * Revision 1.21  1997/10/01 11:29:56  fox
 * Minor changes.
 *
 * Revision 1.20  1997/09/30 13:38:51  wolfram
 * Integration of angles is stopped after integration of any proximity sensor
 *
 * Revision 1.19  1997/09/29 16:29:41  wolfram
 * Angles now handle the situation where no sensor is given
 *
 * Revision 1.18  1997/09/29 10:45:22  wolfram
 * Added default values for all values in .ini file
 *
 * Revision 1.17  1997/09/26 17:02:07  wolfram
 * Intermediate version, do not use
 *
 * Revision 1.16  1997/09/21 08:03:45  wolfram
 * Fixed bugs in graphic.c
 *
 * Revision 1.15  1997/09/14 17:33:33  wolfram
 * Direction of walls is now [0:360], angle probabilities are computed
 * using the simulator map if it is available.
 *
 * Revision 1.14  1997/08/02 16:51:00  wolfram
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
 * Revision 1.13  1997/04/08 14:56:22  fox
 * Changed the reset function for new planes and the prob function.
 *
 * Revision 1.12  1997/03/19 17:52:40  fox
 * New laser parameters.
 *
 * Revision 1.11  1997/03/14 17:58:16  fox
 * This version should run quite stable now.
 *
 * Revision 1.10  1997/03/13 17:36:19  fox
 * Temporary version. Don't use!
 *
 * Revision 1.9  1997/01/29 12:23:00  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.8  1997/01/10 15:19:20  fox
 * Improved several methods.
 *
 * Revision 1.7  1997/01/07 09:38:04  fox
 * Changed the loading of the ini file such that all parameters have to be
 * given only if the corresponding feature has to be used.
 *
 * Revision 1.6  1996/12/20 15:29:35  fox
 * Added four parameters.
 *
 * Revision 1.5  1996/12/09 10:11:58  wolfram
 * Fixed a bug in angle computation
 *
 * Revision 1.4  1996/12/02 10:31:59  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.3  1996/11/18 09:58:28  ws
 * Added support for selecting the optimal sensor
 * Changed some variables to improve compatibility with C++
 *
 * Revision 1.2  1996/10/24 09:56:51  fox
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

#include "probGrid.h"
#include "angle.h"
#include "angleTools.h"
#include "function.h"
#include "graphic.h"
#include "file.h"
#include "movement.h"
#include "laser.h"
#include "sonar.h"
#include "map.h"

#define USE_ANGLE_TOKEN                     0
#define ANGLE_TOKEN                         1
#define UPDATE_THRESHOLD_TOKEN              2
#define CONVOLVE_THRESHOLD_TOKEN            3
#define ALIGNED_SONAR_READINGS_TOKEN        4
#define MIN_ALIGNED_SONAR_READINGS_TOKEN    5
#define MIN_QUOTA_OF_PLANES_FOR_ANGLE_TOKEN 6
#define ANGLE_KERNEL_SIZE_TOKEN             7
#define ANGLE_KERNEL_VALUES_TOKEN           8
#define INTEGRATE_ANGLE_GRID_THRESHOLD_TOKEN 9
#define ALIGNED_LASER_READINGS_TOKEN        10
#define MIN_ALIGNED_LASER_READINGS_TOKEN    11
#define ANGLE_USE_SONAR_TOKEN               12
#define ANGLE_USE_LASER_TOKEN               13

#define ANGLE_DEBUG


angleParameters globalAngleParameters;


/********************************************************************
 ********************************************************************
 * Forward declarations of internal functions.
 ********************************************************************
 ********************************************************************/

static void
updateAngleGrid( informationsFor_ANGLE* info);

static void
convolveAngleGrid( informationsFor_ANGLE* info); 

static void 
integrateAngleGrid( informationsFor_ANGLE* info);


#ifdef ANGLE_DEBUG
static angleProbTable angleGrid;
#endif

/**************************************************************************
 **************************************************************************
 * Mandatory functions.
 **************************************************************************
 **************************************************************************/

void
initialize_ANGLE( char* fileName,
		  actionInformation* actionInfo,
		  sensingActionMask* actionMask,
		  sensingFunctions* handlers)
{
  char angleFile[MAX_STRING_LENGTH],kernelString[MAX_STRING_LENGTH];
  int kernelSize;
  bool useAngle;
  extern mapParameters globalMapParameters;
  extern laserParameters globalLaserParameters;
  extern sonarParameters globalSonarParameters;
  
  /* This struct contains all relevant information for later integration
   * of movement informations. */
  informationsFor_ANGLE* info = 
    (informationsFor_ANGLE*) malloc( sizeof( informationsFor_ANGLE));

  token tok[NUMBER_OF_ANGLE_PARAMETERS];

  /*-------------------------------------------------------------------------
    Default values fo the ini-file parameters
    ------------------------------------------------------------------------*/
  useAngle = FALSE;
  sprintf(angleFile, "%s.angles", globalMapParameters.mapFileName);
  globalAngleParameters.updateThreshold = 5;
  globalAngleParameters.convolveThreshold = 100;
  info->useSonar = TRUE;  
  globalAngleParameters.alignedSonarReadings = 5;
  globalAngleParameters.minAlignedSonarReadings = 4;
  info->useLaser = TRUE;
  globalAngleParameters.alignedLaserReadings = 20;
  globalAngleParameters.minAlignedLaserReadings = 18; 
  globalAngleParameters.minQuotaOfPlanes = 0.4;
  kernelSize = 2;
  sprintf(kernelString, "0.6 0.2");
  globalAngleParameters.integrateThreshold = 10;
  setTokensInitialized(tok, NUMBER_OF_ANGLE_PARAMETERS);
  
  /*-------------------------------------------------------------------------
   * Get the parameters from the file. 
   *------------------------------------------------------------------------*/

  tok[USE_ANGLE_TOKEN].format   = INT_FORMAT;
  tok[USE_ANGLE_TOKEN].variable = &useAngle;
  tok[USE_ANGLE_TOKEN].keyWord  = USE_ANGLE_KEYWORD;
 
  tok[ANGLE_TOKEN].format   = STRING_FORMAT;
  tok[ANGLE_TOKEN].variable = angleFile;
  tok[ANGLE_TOKEN].keyWord  = ANGLE_KEYWORD;
  
  tok[UPDATE_THRESHOLD_TOKEN].format   = FLOAT_FORMAT;
  tok[UPDATE_THRESHOLD_TOKEN].variable = &(globalAngleParameters.updateThreshold);
  tok[UPDATE_THRESHOLD_TOKEN].keyWord  = UPDATE_ANGLE_GRID_THRESHOLD_KEYWORD;
  
  tok[CONVOLVE_THRESHOLD_TOKEN].format   = FLOAT_FORMAT;
  tok[CONVOLVE_THRESHOLD_TOKEN].variable = &(globalAngleParameters.convolveThreshold);
  tok[CONVOLVE_THRESHOLD_TOKEN].keyWord  = CONVOLVE_ANGLE_GRID_THRESHOLD_KEYWORD;
  
  tok[ANGLE_USE_SONAR_TOKEN].format   = INT_FORMAT;
  tok[ANGLE_USE_SONAR_TOKEN].variable = &(info->useSonar);
  tok[ANGLE_USE_SONAR_TOKEN].keyWord  = ANGLE_USE_SONAR_KEYWORD;
  
  tok[ALIGNED_SONAR_READINGS_TOKEN].format   = INT_FORMAT;
  tok[ALIGNED_SONAR_READINGS_TOKEN].variable = &(globalAngleParameters.alignedSonarReadings);
  tok[ALIGNED_SONAR_READINGS_TOKEN].keyWord  = ALIGNED_SONAR_READINGS_KEYWORD;
  
  tok[MIN_ALIGNED_SONAR_READINGS_TOKEN].format   = INT_FORMAT;
  tok[MIN_ALIGNED_SONAR_READINGS_TOKEN].variable = &(globalAngleParameters.minAlignedSonarReadings);
  tok[MIN_ALIGNED_SONAR_READINGS_TOKEN].keyWord  = MIN_ALIGNED_SONAR_READINGS_KEYWORD;

  tok[ANGLE_USE_LASER_TOKEN].format   = INT_FORMAT;
  tok[ANGLE_USE_LASER_TOKEN].variable = &(info->useLaser);
  tok[ANGLE_USE_LASER_TOKEN].keyWord  = ANGLE_USE_LASER_KEYWORD;
  
  tok[ALIGNED_LASER_READINGS_TOKEN].format   = INT_FORMAT;
  tok[ALIGNED_LASER_READINGS_TOKEN].variable = &(globalAngleParameters.alignedLaserReadings);
  tok[ALIGNED_LASER_READINGS_TOKEN].keyWord  = ALIGNED_LASER_READINGS_KEYWORD;
  
  tok[MIN_ALIGNED_LASER_READINGS_TOKEN].format   = INT_FORMAT;
  tok[MIN_ALIGNED_LASER_READINGS_TOKEN].variable = &(globalAngleParameters.minAlignedLaserReadings);
  tok[MIN_ALIGNED_LASER_READINGS_TOKEN].keyWord  = MIN_ALIGNED_LASER_READINGS_KEYWORD;

  tok[MIN_QUOTA_OF_PLANES_FOR_ANGLE_TOKEN].format   = FLOAT_FORMAT;
  tok[MIN_QUOTA_OF_PLANES_FOR_ANGLE_TOKEN].variable =
    &(globalAngleParameters.minQuotaOfPlanes);
  tok[MIN_QUOTA_OF_PLANES_FOR_ANGLE_TOKEN].keyWord  =
    MIN_QUOTA_OF_PLANES_FOR_ANGLE_KEYWORD;

  tok[ANGLE_KERNEL_SIZE_TOKEN].format   = INT_FORMAT;
  tok[ANGLE_KERNEL_SIZE_TOKEN].variable = &kernelSize;
  tok[ANGLE_KERNEL_SIZE_TOKEN].keyWord  = ANGLE_KERNEL_SIZE_KEYWORD;
   
  tok[ANGLE_KERNEL_VALUES_TOKEN].format   = MULTI_VALUE_FORMAT;
  tok[ANGLE_KERNEL_VALUES_TOKEN].variable = kernelString;
  tok[ANGLE_KERNEL_VALUES_TOKEN].keyWord  = ANGLE_KERNEL_VALUES_KEYWORD;
  
  tok[INTEGRATE_ANGLE_GRID_THRESHOLD_TOKEN].format   = INT_FORMAT;
  tok[INTEGRATE_ANGLE_GRID_THRESHOLD_TOKEN].variable =
    &(globalAngleParameters.integrateThreshold);
  tok[INTEGRATE_ANGLE_GRID_THRESHOLD_TOKEN].keyWord  = INTEGRATE_ANGLE_GRID_THRESHOLD_KEYWORD;
  
  readTokens( fileName, tok, NUMBER_OF_ANGLE_PARAMETERS, FALSE);
  

  /*-------------------------------------------------------------------------
   * done.
   *------------------------------------------------------------------------*/

  if ( useAngle){
    bool sensorAvailable;

    info->useLaser = info->useLaser &&
      (globalLaserParameters.useFrontLaser
			 || globalLaserParameters.useRearLaser);
    info->useSonar = info->useSonar
      && globalSonarParameters.useSonar;
    
    sensorAvailable = info->useLaser || info->useSonar;

    if (!sensorAvailable){
      char *error =
	"# Warning: Angles are not used since no sensor is specified!\n";
      fprintf(stderr, error);
      writeLog(error);
      useAngle = FALSE;
    }
  }

  if (useAngle){
    
    info->actionMask = actionMask;

    /*-------------------------------------------------------------------------
     * Get the angle probabities,
     *------------------------------------------------------------------------*/
    info->aPrioriAngleProbs =
      readAngleProbTable( angleFile);
    
    if ( info->aPrioriAngleProbs.prob == NULL) {
      expectedDistTable* expDist;
      
      if ( info->useLaser) {
	informationsFor_LASER* laserInfo =
	  (informationsFor_LASER*) actionInfo->info[LASER];
	expDist = &(laserInfo->general.expectedDistances);
      }
      else {
	informationsFor_SONAR* sonarInfo =
	  (informationsFor_SONAR*) actionInfo->info[SONAR];
	expDist = &(sonarInfo->general.expectedDistances);
      }
      
      info->aPrioriAngleProbs =
	preprocessedAngleProbabilities( &(actionInfo->map),
					&(actionInfo->simMap),
					&(actionInfo->initialPositionProbs),
					expDist, angleFile);
    }
    info->angleProbs =
      initializedAngleProbabilityGrid( &( info->aPrioriAngleProbs));

    info->angles.numberOfWalls = 0;

    if (info->useSonar){
      info->sonarScan              = &(actionInfo->actualSensings.sonar);
    }
    
    if (info->useLaser){
      info->frontLaserScan = &(actionInfo->actualSensings.frontLaser);
      info->rearLaserScan = &(actionInfo->actualSensings.rearLaser);
    }

    info->positionProbs          = &(actionInfo->positionProbs);
    
    /*-------------------------------------------------------------------------
     * Initialze the angle kernel.
     *------------------------------------------------------------------------*/
    info->angleKernel.size       = kernelSize;
    if ((info->angleKernel.element =
	 (float *) malloc( info->angleKernel.size * sizeof(float))) == NULL){
      char* error = "# Error: Could not allocate angle kernel\n";
      fprintf(stderr, error);
      writeLog(error);
      closeLogAndExit(-1);
    }
    getInitValuesFloat(kernelString, info->angleKernel.element, kernelSize);
    
    /*-------------------------------------------------------------------------
     * Initialze the mask.
     *------------------------------------------------------------------------*/
    actionMask->numberOfActions[ANGLE] = NUMBER_OF_ACTIONS_ANGLE;
  }
  else{
    info->useLaser = info->useSonar = FALSE;
    actionMask->numberOfActions[ANGLE] = 0;
  }  
  actionMask->use[ANGLE] = useAngle;
  
  /* Now place the struct in the global information struct. */
  actionInfo->info[ANGLE] = info;

  /*-------------------------------------------------------------------------
   * Initialze the handlers.
   *------------------------------------------------------------------------*/
  handlers->checkIfConsider[ANGLE]            = checkIfConsider_ANGLE;
  handlers->checkWhichActionsToPerform[ANGLE] = checkWhichActionsToPerform_ANGLE;
  handlers->performActions[ANGLE]             = performActions_ANGLE;

#ifdef ANGLE_DEBUG
  angleGrid.prob = NULL;
  angleGrid.numberOfAngles = 0;
#endif
}

/**************************************************************************
 **************************************************************************/
void
checkIfConsider_ANGLE( actionInformation* actionInfo,
		       sensingActionMask* mask)
{
  /* Just check for the threshold. */
  mask->consider[ANGLE] = mask->use[ANGLE]
    && actionInfo->positionProbs.quotaOfPlanesToBeUpdated
    > globalAngleParameters.minQuotaOfPlanes
    && !actionInfo->proximityIntegrated;
}

/**************************************************************************
 **************************************************************************/
void
checkWhichActionsToPerform_ANGLE( actionInformation* actionInfo,
				  sensingActionMask* mask)
{
  
  /* This structure contains all relevant information for the actions. */
  informationsFor_ANGLE* angleInfo =
    (informationsFor_ANGLE*) actionInfo->info[ANGLE];

  if ( mask->consider[ANGLE]) {

    /* Update the actual angle sensing. */
    updateWallInformations( angleInfo);
  
    /*-------------------------------------------------------------------------
     * Check wether the new angle should be integrated.
     *-------------------------------------------------------------------------*/
    if ( angleInfo->angles.isNew
	 && actionInfo->summedMovements[ANGLE][UPDATE_ANGLE_GRID]
	 > globalAngleParameters.updateThreshold) {
      mask->perform[ANGLE][UPDATE_ANGLE_GRID] = TRUE;
      actionInfo->summedMovements[ANGLE][UPDATE_ANGLE_GRID] = 0.0;
    }
    else
      mask->perform[ANGLE][UPDATE_ANGLE_GRID] = FALSE;
    
    /*-------------------------------------------------------------------------
     * After certain movement the angle probs have to be smoothed. 
     *-------------------------------------------------------------------------*/
    if (actionInfo->summedMovements[ANGLE][CONVOLVE_ANGLE_GRID]
	> globalAngleParameters.convolveThreshold) {
      mask->perform[ANGLE][CONVOLVE_ANGLE_GRID] = TRUE;
      actionInfo->summedMovements[ANGLE][CONVOLVE_ANGLE_GRID] = 0.0;
    }
    else
      mask->perform[ANGLE][CONVOLVE_ANGLE_GRID] = FALSE;
  
    /*-------------------------------------------------------------------------
     * Each INTEGRATE_SKIP'th reading will be integrated into the probGrid.
     *-------------------------------------------------------------------------*/
    if ( mask->perform[ANGLE][UPDATE_ANGLE_GRID]) {
      static int cnt = 0;
      cnt++; 
      mask->perform[ANGLE][INTEGRATE_ANGLE_GRID] =
	(cnt % globalAngleParameters.integrateThreshold == 0);
    }
    else
      mask->perform[ANGLE][INTEGRATE_ANGLE_GRID] = FALSE;
  }
  else {
    mask->perform[ANGLE][CONVOLVE_ANGLE_GRID]  = FALSE;
    mask->perform[ANGLE][INTEGRATE_ANGLE_GRID] = FALSE;
    mask->perform[ANGLE][UPDATE_ANGLE_GRID]    = FALSE;
  }
}

/**************************************************************************
 **************************************************************************/
void
performActions_ANGLE( actionInformation* actionInfo,
		      sensingActionMask* mask)
{
    /* This structure contains all relevant information for the actions. */
  informationsFor_ANGLE* info =
    (informationsFor_ANGLE*) actionInfo->info[ANGLE];
  
  info->angleProbs.isNew = FALSE;
  
  if ( mask->perform[ANGLE][UPDATE_ANGLE_GRID]) 
      updateAngleGrid( info);
  
  if ( mask->perform[ANGLE][CONVOLVE_ANGLE_GRID])  
      convolveAngleGrid( info); 
 
  if ( mask->perform[ANGLE][INTEGRATE_ANGLE_GRID]) 
      integrateAngleGrid( info);
}

/**************************************************************************
 **************************************************************************
 * Local functions.
 **************************************************************************
 **************************************************************************/



static void
updateAngleGrid( informationsFor_ANGLE* info)
{
  int angleCnt, wallCnt;
  static int cnt = 1;
    
  /* Just for easier access. */
  angleProbTable* grid = &(info->angleProbs);
  sensing_ANGLE angleSensing  = info->angles;

#ifdef ANGLE_DEBUG
  static bool firstTime = TRUE;
  static gridWindow *win;
  
  if ( firstTime) {
    firstTime = FALSE;
    angleGrid.prob =
      (probability*) malloc( grid->numberOfAngles * sizeof(probability));
    angleGrid.numberOfAngles = grid->numberOfAngles;
    for ( angleCnt = 0; angleCnt < grid->numberOfAngles; angleCnt++) 
      angleGrid.prob[angleCnt] = 1.0;
    win = createAngleWindow( grid, "Global Angle histogram",
			     400, 600);
  }
#endif
  
  if (angleSensing.numberOfWalls > 0) {
    wallCnt = angleSensing.bestWallIndex;
    
    if ( angleSensing.wall[wallCnt].isNew) {
      wall actualWall = angleSensing.wall[wallCnt];
      /* Found a wall. Integrate it. */
      grid->isNew = TRUE;
      
      fprintf( stderr, "# Updating wall no. %d at %f degree (certainty %f) %f %f.\n",
	       cnt, rad2Deg( angleSensing.bestWall.pos.rot),
	       angleSensing.bestWall.certainty,
	       angleSensing.bestWall.pos.x,  angleSensing.bestWall.pos.y);
      
      writeLog( "# Updating wall no. %d at %f degree (certainty %f).\n",
		cnt++, rad2Deg( angleSensing.bestWall.pos.rot),
		angleSensing.bestWall.certainty );
      
      for ( angleCnt = 0; angleCnt < grid->numberOfAngles; angleCnt++) {
	float angle;
	probability prob;
	/* We take the angle of the plane and add the relative angle of the
	 * wall. This will be the angle of the environment relative to
	 * the start position of the robot.    
	 */
	
	angle = normalizedAngle( angleCnt * grid->angleResolution
			     + info->positionProbs->summedMovementOfPlane[0].rot
			     + actualWall.pos.rot);
	prob = pow( probabilityOfAngle( angle, &(info->aPrioriAngleProbs)),
		    angleSensing.wall[wallCnt].certainty);
	grid->prob[angleCnt] *= prob;
#ifdef ANGLE_DEBUG
	angleGrid.prob[angleCnt] *= prob;
#endif
      }
    }
  }
  normalizeAngleGrid(grid->prob, grid->numberOfAngles);
  for ( angleCnt = 0; angleCnt < grid->numberOfAngles; angleCnt++)
    /* check for the threshold. */
    if ( grid->prob[angleCnt] < grid->minProb)
      grid->minProb = grid->prob[angleCnt];

#ifdef ANGLE_DEBUG
  displayAngleWindow( &angleGrid, win, 0.0);
#endif

  normalizeAngleGrid(angleGrid.prob, grid->numberOfAngles);
}


static void
convolveAngleGrid( informationsFor_ANGLE* info)
{    
  writeLog( "# Convolving angles ... ");
  fprintf( stderr, "# Convolving angles ... ");

  convolve1DTorus( info->angleProbs.prob,
		   info->angleProbs.numberOfAngles,
		   info->angleKernel);

#ifdef ANGLE_DEBUG
  convolve1DTorus( angleGrid.prob, angleGrid.numberOfAngles,
		   info->angleKernel);
#endif
  
  info->angleProbs.isNew = TRUE;
    writeLog( "done.\n");
  fprintf( stderr, "done.\n");
}


static void 
integrateAngleGrid( informationsFor_ANGLE* info)
{
  static int cnt = 0;
  int angleCnt;
  float angleResolutionRatio =
    info->positionProbs->angleResolution / info->angleProbs.angleResolution;

  fprintf( stderr, "# ----------------------------------------------\n");
  fprintf( stderr, "#        Angles no. %d ... ", ++cnt);
    
    writeLog( "# ----------------------------------------------\n");
    writeLog( "#        Angles no. %d ... ", cnt);
  
  for ( angleCnt = 0; angleCnt < info->positionProbs->sizeZ; angleCnt++) {

    int angleIndex =
      round( angleCnt * angleResolutionRatio)
      % info->angleProbs.numberOfAngles;

    /* If reading or movement is integrated we don't update all planes. */
    if ( 0 && ( info->actionMask->consider[MOVEMENT]
	 || (info->useSonar && info->actionMask->consider[SONAR])
    || (info->useLaser && info->actionMask->consider[LASER]))) {
      if ( info->positionProbs->updatePlane[angleCnt])
	info->positionProbs->normalizeFactorOfPlane[angleCnt] *=
	  info->angleProbs.prob[angleIndex];
    }
    /* Ohtherwise all planes are updated. */
    else 
      info->positionProbs->normalizeFactorOfPlane[angleCnt] *=
	info->angleProbs.prob[angleIndex];
    
    if ( info->positionProbs->normalizeFactorOfPlane[angleCnt] < MIN_NORM_FACTOR) {
	info->positionProbs->normalizeFactorOfPlane[angleCnt] = MIN_NORM_FACTOR;
    }
    
#define ANGLE_DEBUG
#ifdef ANGLE_DEBUG
    if ( info->positionProbs->updatePlane[angleCnt]) {
      writeLog("yes: ");
    }
    else
      writeLog( "no: ");
    
    writeLog("%d %g %g %g %g\n",
	     angleCnt,
	     info->positionProbs->normalizeFactorOfPlane[angleCnt]
	     * info->positionProbs->sumOfPlane[angleCnt],
	      
	     info->positionProbs->normalizeFactorOfPlane[angleCnt]
	     * info->positionProbs->maxProbabilityOfPlane[angleCnt],
	      
	     info->positionProbs->normalizeFactorOfPlane[angleCnt],
	     
	     info->angleProbs.prob[ round( rad2Deg( rotationOfPlane( angleCnt,
								     info->positionProbs))) % 360]);
#endif 
    
  }
  
  for ( angleCnt = 0; angleCnt < info->angleProbs.numberOfAngles; angleCnt++)
    info->angleProbs.prob[angleCnt] = 1.0;
  
  
  info->angleProbs.minProb = 1.0;
  info->angleProbs.isNew = FALSE;
  
  /* If only the angles are integrated we can change the numbers of
   * planes to be updated just by looking at the normalize factors. */
  if ( ! ( info->actionMask->consider[MOVEMENT] 
	   || (info->useSonar && info->actionMask->consider[SONAR])
	   || (info->useLaser && info->actionMask->consider[LASER]))) {
    normalizeFactorHasChangedUpdatePlanes( info->positionProbs);
  }
  
  writeLog( "done (quota: %g.\n", info->positionProbs->quotaOfPlanesToBeUpdated);
  writeLog( "# ----------------------------------------------\n");
  fprintf( stderr, "done (quota: %g).\n", info->positionProbs->quotaOfPlanesToBeUpdated);
  fprintf( stderr, "# ----------------------------------------------\n");
  

  info->angles.isNew = FALSE;
}
  















