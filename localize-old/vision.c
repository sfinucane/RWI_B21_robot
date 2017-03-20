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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/vision.c,v $
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
 * $Log: vision.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.32  2000/03/06 20:00:47  wolfram
 * Changed the code of the vision routines.
 *
 * Revision 1.31  1999/11/02 18:12:38  fox
 * Version of multi localize journal paper.
 *
 * Revision 1.30  1999/01/22 00:34:57  wolfram
 * Added database support in vision.c
 *
 * Revision 1.29  1999/01/14 00:33:04  wolfram
 * Changes for vision
 *
 * Revision 1.28  1999/01/11 19:47:57  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.26  1999/01/08 23:15:18  wolfram
 * slight changes
 *
 * Revision 1.25  1998/12/16 14:59:06  wolfram
 * First version without libGetDistance. Use with caution.
 *
 * Revision 1.24  1998/12/10 17:56:08  fox
 * Fixed a bug in displayPositions.
 *
 * Revision 1.23  1998/11/29 22:43:21  wolfram
 * Small changes
 *
 * Revision 1.22  1998/11/27 09:12:04  wolfram
 * Some changes for Frank
 *
 * Revision 1.21  1998/11/25 16:29:42  wolfram
 * Added higher resolution for vision maps. resolution is now read from file.
 * Couldn't integrate it consistently into graphic.c.
 *
 * Revision 1.20  1998/11/25 08:27:33  wolfram
 * Added slower display for overlay running on every XServer
 *
 * Revision 1.19  1998/11/24 23:05:27  fox
 * Implemented furhter routines for condensation and vision.
 *
 * Revision 1.18  1998/11/24 18:42:12  fox
 * First version of condensation with vision.
 *
 * Revision 1.17  1998/11/24 15:33:53  wolfram
 * nothing special
 *
 * Revision 1.16  1998/11/24 15:31:10  wolfram
 * Fixed a bug, added samples to vision
 *
 * Revision 1.15  1998/11/24 08:31:31  wolfram
 * Added linear transformation to the ceiling map
 *
 * Revision 1.14  1998/11/23 19:45:10  fox
 * Latest version.
 *
 * Revision 1.13  1998/11/16 08:35:27  wolfram
 * Cleaned up vision.c. It seems to work give appropriate maps!
 *
 * Revision 1.12  1998/10/26 22:16:18  wolfram
 * Added logScale option for plotting
 *
 * Revision 1.11  1998/09/25 04:03:00  fox
 * First version of CONDENSATION!!!!
 *
 * Revision 1.10  1998/09/05 22:06:57  wolfram
 * Changes regarding vision!
 *
 * Revision 1.9  1998/08/31 22:29:24  wolfram
 * Several changes
 *
 * Revision 1.8  1998/08/26 15:34:05  wolfram
 * Finished integration of vision
 *
 * Revision 1.7  1998/08/24 07:39:51  wolfram
 * final version for Washington
 *
 * Revision 1.6  1998/08/23 00:01:05  wolfram
 * Integration of vision and adaptation for the Smithsonian
 * Do not use this version!
 *
 * Revision 1.5  1998/08/20 00:23:04  wolfram
 * Intermediate version before Washington. Do not use!
 *
 * Revision 1.4  1998/08/19 16:33:59  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.3  1998/04/19 10:40:38  wolfram
 * Added graphical display to multi localization
 *
 * Revision 1.2  1998/03/17 13:58:42  wolfram
 * Nothing special.
 *
 * Revision 1.1  1998/03/17 08:40:46  wolfram
 * First steps to add vision.
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <values.h>

#include "general.h"
#include "probGrid.h"
#include "allocate.h"
#include "function.h"

#include "vision.h"
#include "question.h"
#include "abstract.h"
#include "movement.h"
#include "file.h"
#include "script.h"
#include "probGridTools.h"
#include "graphic.h"
#include "localTcx.h"

/* 
#include "featurebank.h"
#include "mypgm.c"
#include "featurebank.c"
#include "feature.c"
*/


#define VISION_MAP_EXTENSION ".visionmap.pgm"
#define VISION_VARIANCE_MAP_EXTENSION ".visionvariancemap.pgm"

#define BLUR_MEAN_EXTENSION ".blur-mean.pgm"
#define BLUR_VARIANCE_EXTENSION ".blur-variance.pgm"
#define BLSD_MEAN_EXTENSION ".blsd-mean.pgm"
#define BLSD_VARIANCE_EXTENSION ".blsd-variance.pgm"




#define NUMBER_OF_VISION_FEATURES 256

#define USE_VISION_TOKEN                             0 
#define VISION_INTEGRATE_THRESHOLD_TOKEN             1
#define VISION_MAX_QUOTA_OF_PLANES_TOKEN             2
#define VISION_CRUNCH_FACTOR_TOKEN                   3
#define VISION_INTEGRATIONS_PER_STOP_TOKEN           4


#define USE_DATABASE_FEATURE 0
#define NUMBER_OF_ROTATION_ANGLES         360
#define NUMBER_OF_PIXELS (IMAGE_SIZE_X*IMAGE_SIZE_Y)





static float **blurWeight;
static probability errorProb[NUMBER_OF_VISION_FEATURES];
visionParameters globalVisionParameters;


typedef struct {
  int x;
  int y;
} imagePosition;


static struct{
  int numberOfPixels[NUMBER_OF_VISION_FEATURES];
  imagePosition *pos[NUMBER_OF_VISION_FEATURES];
} featureList;

static struct{
  int numberOfPixels;
  imagePosition *pos;
} knownPositions;

static imagePosition rotationSourcePos[NUMBER_OF_ROTATION_ANGLES][NUMBER_OF_PIXELS];
static imagePosition correlationPos[NUMBER_OF_PIXELS];
static int numberOfCorrelationPositions = 0;


/* 
#define FEATURE_FILE_NAME "x1.pdm"

static FeatureBank *featureBank;
static Feature *feature;
*/ 

/********************************************************************
 ********************************************************************
 * Forward declarations of internal functions.
 ********************************************************************
 ********************************************************************/

static void
modifyVisionMap(visionMap map);

static probability**
visionProbabilityTable();

static void
computeVisionFeatureProbTable(visionFeatureProbTable *tab);

static void
integrateImage( informationsFor_VISION* info, actionInformation *actionInfo);

bool
readErrorProbs( probability *errorProb, int n);

static void
computeCorrelationPositions();

static void
computePixelIndex(visionMap map);

/**************************************************************************
 **************************************************************************
 * Mandatory functions.
 **************************************************************************
 **************************************************************************/

void
initialize_VISION( char* fileName,
		   actionInformation* actionInfo,
		   sensingActionMask* actionMask,
		   sensingFunctions* handlers)
{

  extern mapParameters globalMapParameters;

  
  /* This struct contains all relevant information for later integration
   * of vision informations. */
  
  informationsFor_VISION* info = 
    (informationsFor_VISION*) malloc( sizeof( informationsFor_VISION));
  sensing_VISION *rawImage = &(actionInfo->actualSensings.image);

  
  /*-------------------------------------------------------------------------
   * Initialize parameters
   *------------------------------------------------------------------------*/

  bool useVision = FALSE ;
  token tok[NUMBER_OF_VISION_PARAMETERS];

  globalVisionParameters.useVision = useVision;
  globalVisionParameters.integrateThreshold = 10.0;
  globalVisionParameters.maxQuotaOfPlanes = 1.1;
  globalVisionParameters.maxNumberOfIntegrationsPerStop = 0;
  globalVisionParameters.crunchFactor = 1.0;  
  globalVisionParameters.resolution = 10;
  
  rawImage->pix =
    (pixel **) allocate2D(IMAGE_SIZE_X, IMAGE_SIZE_Y, PIXEL);
  rawImage->sizeX = IMAGE_SIZE_X;
  rawImage->sizeY = IMAGE_SIZE_Y;
  rawImage->isNew = FALSE;

  setTokensInitialized(tok, NUMBER_OF_VISION_PARAMETERS);

  tok[USE_VISION_TOKEN].format   = INT_FORMAT;
  tok[USE_VISION_TOKEN].variable = &useVision;
  tok[USE_VISION_TOKEN].keyWord  = USE_VISION_KEYWORD;


  tok[VISION_INTEGRATE_THRESHOLD_TOKEN].format   = FLOAT_FORMAT;
  tok[VISION_INTEGRATE_THRESHOLD_TOKEN].variable =
    &globalVisionParameters.integrateThreshold;
  tok[VISION_INTEGRATE_THRESHOLD_TOKEN].keyWord  =
    VISION_INTEGRATE_THRESHOLD_KEYWORD;

  tok[VISION_MAX_QUOTA_OF_PLANES_TOKEN].format   = FLOAT_FORMAT;
  tok[VISION_MAX_QUOTA_OF_PLANES_TOKEN].variable =
    &globalVisionParameters.maxQuotaOfPlanes;
  tok[VISION_MAX_QUOTA_OF_PLANES_TOKEN].keyWord  =
    VISION_MAX_QUOTA_OF_PLANES_KEYWORD;

  tok[VISION_CRUNCH_FACTOR_TOKEN].format   = FLOAT_FORMAT;
  tok[VISION_CRUNCH_FACTOR_TOKEN].variable =
    &globalVisionParameters.crunchFactor;
  tok[VISION_CRUNCH_FACTOR_TOKEN].keyWord  = VISION_CRUNCH_FACTOR_KEYWORD;

  tok[VISION_INTEGRATIONS_PER_STOP_TOKEN].format   = INT_FORMAT;
  tok[VISION_INTEGRATIONS_PER_STOP_TOKEN].variable =
    &globalVisionParameters.maxNumberOfIntegrationsPerStop;
  tok[VISION_INTEGRATIONS_PER_STOP_TOKEN].keyWord  = VISION_INTEGRATIONS_PER_STOP_KEYWORD;

  readTokens( fileName, tok, NUMBER_OF_VISION_PARAMETERS, FALSE); 

  globalVisionParameters.useVision = info->useVision = useVision;
  
  /*------------------------------------------------------------------------
   * Initialze the handlers.
   *-----------------------------------------------------------------------*/
  
  handlers->checkIfConsider[VISION]            = checkIfConsider_VISION;
  handlers->checkWhichActionsToPerform[VISION] = checkWhichActionsToPerform_VISION;
  handlers->performActions[VISION]             = performActions_VISION;
  
  if (useVision) {
    computeVisionFeatureProbTable(&(info->probTab));
    computeCorrelationPositions();
    actionMask->numberOfActions[VISION] = NUMBER_OF_ACTIONS_VISION;
    
    
    /* Initialize the general information for all visions. */
    info->useProbGrid               = &(actionInfo->useProbGrid);
    info->grid                      = &(actionInfo->positionProbs);
    info->samples                   = &(actionInfo->samples);
    info->image                     = &(actionInfo->actualSensings.image);
    info->visMap                    = &(actionInfo->visMap);
    info->map                       = &(actionInfo->map);
    info->visVarMap                 = &(actionInfo->visVarMap);
    info->varianceMap               = &(actionInfo->varianceMap);
    info->varianceVarMap            = &(actionInfo->varianceVarMap);
    
    /*------------------------------------------------------------------------
     * Initialze the mask.
     *-----------------------------------------------------------------------*/
    actionMask->use[VISION] = useVision;
    
    if (readVisionMap(globalMapParameters.mapFileName,
		      VISION_MAP_EXTENSION,
		      &actionInfo->visMap,
		      &actionInfo->map)) {
      if (0) writeVisionMap(globalMapParameters.mapFileName,
			    ".aligned.pgm",
			    &actionInfo->visMap);
      if (0) modifyVisionMap(actionInfo->visMap);

      computePixelIndex(actionInfo->visMap);
      /* 
      featureBank = CreateFeatureBankFromFile(FEATURE_FILE_NAME,
					      round(100.0 /
					      actionInfo->visMap.resolution));
      feature = CreateFeatureFromFile(FEATURE_FILE_NAME);
      */

      actionInfo->visMap.initialized = TRUE;

      if ( ! readVisionMap(globalMapParameters.mapFileName,
			   VISION_VARIANCE_MAP_EXTENSION,
			   &actionInfo->visVarMap,
			   &actionInfo->map))
	actionInfo->visVarMap.initialized = FALSE;
      else
	actionInfo->visVarMap.initialized = TRUE;
      
      
      /* With the grid all maps have to have consistent resolutions. */
      if ( actionInfo->useProbGrid) {
	
	actionInfo->visMap.resolution =
	  actionInfo->visVarMap.resolution = actionInfo->map.resolution;

      }

      globalVisionParameters.resolution = actionInfo->visMap.resolution;

      fprintf(stderr, "Resolution: %d\n", globalVisionParameters.resolution);
      
      actionInfo->visMap.maxRealX = actionInfo->visVarMap.maxRealX =
	actionInfo->visMap.sizeX * actionInfo->visMap.resolution;
      actionInfo->visMap.maxRealY = actionInfo->visVarMap.maxRealY =
	actionInfo->visMap.sizeY * actionInfo->visMap.resolution;
    }
    else{
      actionInfo->visMap.initialized =
	actionInfo->visVarMap.initialized =
	actionMask->use[VISION] = FALSE;
      actionMask->numberOfActions[VISION] = 0;
    }
  }
  else{
    actionInfo->visMap.initialized =
      actionInfo->visVarMap.initialized = FALSE;      
    actionMask->use[VISION] = useVision = FALSE;
    actionMask->numberOfActions[VISION] = 0;
  }

  
  /* Now place the struct in the global information struct. */
  actionInfo->info[VISION] = info;
  
  if (useVision)
    if (!readErrorProbs(errorProb, NUMBER_OF_VISION_FEATURES)){
      fprintf(stderr,
	      "# Warning: could not read error probabilities. Not Using Vision\n");
      actionMask->use[VISION] = useVision = FALSE;
      actionMask->numberOfActions[VISION] = 0;
    }
}



  /**************************************************************************
 * Check whether the vision readings shall be considered for integration.
 **************************************************************************/
void
checkIfConsider_VISION( actionInformation* actionInfo,
			sensingActionMask* mask)
{
  /*printf("\nhallo! consider.\n");*/
  mask->consider[VISION] = mask->use[VISION];
  actionInfo=actionInfo;
}


/**************************************************************************
 * Check which actions shall be performed on the position probability grid.
 **************************************************************************/
void
checkWhichActionsToPerform_VISION( actionInformation* info,
				   sensingActionMask* mask)
{
  static int numberOfIntegratedReadingsSinceStop = 0;
  if (1) {
    mask->perform[VISION][INTEGRATE_VISION] = TRUE;
    return;
  }
  
  if (mask->consider[VISION] && info->actualSensings.image.isNew){
    if (info->summedMovements[VISION][INTEGRATE_VISION]
	> globalVisionParameters.integrateThreshold){
      mask->perform[VISION][INTEGRATE_VISION] = TRUE;
      info->summedMovements[VISION][INTEGRATE_VISION] = 0.0;
      mask->perform[MOVEMENT][INTEGRATE_MOVEMENT] = TRUE;
      info->summedMovements[MOVEMENT][INTEGRATE_MOVEMENT] = 0.0;
      numberOfIntegratedReadingsSinceStop = 0;
    }
    else if (info->actualSensings.noMoveCnt > 0){
      if ( numberOfIntegratedReadingsSinceStop <
	   globalVisionParameters.maxNumberOfIntegrationsPerStop){
	writeLog("#nomove: %d\n", info->actualSensings.noMoveCnt);
	mask->perform[VISION][INTEGRATE_VISION] = TRUE;
	info->summedMovements[VISION][INTEGRATE_VISION] = 0.0;
	mask->perform[MOVEMENT][INTEGRATE_MOVEMENT] = TRUE;
	info->summedMovements[MOVEMENT][INTEGRATE_MOVEMENT] = 0.0;
	numberOfIntegratedReadingsSinceStop++;
      }
    }
    else
      mask->perform[VISION][INTEGRATE_VISION] = FALSE;
  }
  else
    mask->perform[VISION][INTEGRATE_VISION] = FALSE;
}

/**************************************************************************
 * Perform the actions on  the position probability grid.
 **************************************************************************/
void
performActions_VISION( actionInformation* actionInfo,
		       sensingActionMask* mask)
{
   /* This structure contains all relevant information for the actions. */
  informationsFor_VISION* info =
    (informationsFor_VISION*) actionInfo->info[VISION];

  if ( mask->perform[VISION][INTEGRATE_VISION])
    integrateImage( info, actionInfo);
}



probability
probOfMeasuredVisionFeatureGivenExpected(pixel expected,
                                         pixel measured,
                                         pixel stddev)
{
  double s = (double) stddev;
  double e = (double) expected - (double) measured;
  /* change 3 to measurement variance. I use 3 to 6 */
  double v = 3*3 + s*s;
  probability p = exp(-0.5*e*e/v)/sqrt(DEG_360 * v);
  return p;
}



/* this is a linear transformation to have a better correlation
   between measured and expected features */

static void
modifyVisionMap(visionMap map)
{
  unsigned int x, y;
  /* 16.6355 + 1.10181*x first iteration */
  /* 17.9658 + 0.786483*x second iteration with condensation */
  /* 2.6675 + 1.13898*x third iteration */
  /* 16.4786 + 0.990514*x */
  
  for (x = 0; x < map.sizeX; x++)
    for (y = 0; y < map.sizeY; y++)
      if (map.pix[x][y] > 200)
	map.pix[x][y] = 255;
  
  return;
  
  {
    pixel ** pix = (pixel **) allocate2D(map.sizeX, map.sizeY, PIXEL);
    unsigned int x1, y1;
    
    for (x = 0; x < map.sizeX; x++)
      for (y = 0; y < map.sizeY; y++)
	pix[x][y] = map.pix[x][y];
    
    for (x = 1; x < map.sizeX-1; x++)
      for (y = 1; y < map.sizeY-1; y++)
	for (x1 = x-1; x1 < x+2; x1++)
	  for (y1 = y - 1; y1 < y+2; y1++)
	    if (map.pix[x1][y1] > 150 && map.pix[x1][y1] > pix[x][y] )
	      pix[x][y] = map.pix[x1][y1];
    
    for (x = 0; x < map.sizeX; x++)
      for (y = 0; y < map.sizeY; y++)
	map.pix[x][y] = pix[x][y];
    
    free2D((void **) pix, map.sizeX, PIXEL);
  }
  
}


static void
computePixelIndex(visionMap map)
{
  unsigned int count[NUMBER_OF_VISION_FEATURES];
  unsigned int x, y;
  
  for (x=0; x < NUMBER_OF_VISION_FEATURES; x++){
    count[x] = featureList.numberOfPixels[x] = 0;
  }
  for (x = 0; x < map.sizeX; x++)
    for (y = 0; y < map.sizeY; y++)
      count[map.pix[x][y]]++;
  
  for (x = 0; x < NUMBER_OF_VISION_FEATURES; x++){
    featureList.pos[x] = (imagePosition *)
      malloc(sizeof(imagePosition) * count[x]);
    if (count[x] > 0 && featureList.pos[x] == NULL){
      fprintf(stderr, "# Error: could not allocate feature list\n");
      closeLogAndExit(0);
    }
  }
  
  for (x = 0; x < map.sizeX; x++)
    for (y = 0; y < map.sizeY; y++){
      unsigned int p = map.pix[x][y];
      featureList.pos[p][featureList.numberOfPixels[p]].x = x;
      featureList.pos[p][featureList.numberOfPixels[p]].y = y;
      featureList.numberOfPixels[p]++;
    }

  if (0) 
    for (x = 0; x < NUMBER_OF_VISION_FEATURES; x++)
      fprintf(stderr, "Pixelcount: %d %d\n", (int) x,
	      (int) featureList.numberOfPixels[x]);
  if (1)
    { int y,z;
      knownPositions.numberOfPixels = 0;
      for (x = 1; x < NUMBER_OF_VISION_FEATURES; x++){
	knownPositions.numberOfPixels += count[x];
      }
      
      knownPositions.pos = (imagePosition *)
	malloc(sizeof(imagePosition) * knownPositions.numberOfPixels);
      if (knownPositions.pos == NULL || knownPositions.numberOfPixels == 0){
	fprintf(stderr, "# Error: could not allocate known positions\n");
	closeLogAndExit(0);
      }
      z = 0;
      for (x = 1; x < NUMBER_OF_VISION_FEATURES; x++)
	for (y = 0; y < count[x]; y++){
	  knownPositions.pos[z] = featureList.pos[x][y];
	  z++;
	}

      fprintf(stderr, "#### Known Area: %f\n",
	      ((float) knownPositions.numberOfPixels) / map.sizeX / map.sizeY);
    }
      
}

bool
readErrorProbs(probability *error, int n){
  FILE *fp;
  char *fileName = "error.dat";
  int x, y;
  char line[MAX_STRING_LENGTH];
  
  fp = fopen(fileName, "r");

  for (x = 0; x < n; x++)
    error[x] = 0.0;

  if (fp == NULL){
    fprintf(stderr, "# Error: could not open %s\n", fileName);
    return FALSE;
  }

  
  while (fgets(line, MAX_STRING_LENGTH, fp) != NULL){
    if (sscanf(line, "%d %d\n", &x, &y) == 2){
      if ((x >= 0) && (x < n))
	error[x] = y;
    }
    else{
      fprintf(stderr, "# Error: Could not read %s\n", fileName);
      return(FALSE);
    }
  }

  for (x = 0; x <  NUMBER_OF_VISION_FEATURES; x++)
    error[x] = 1.0 - sqrt(x / (float) NUMBER_OF_VISION_FEATURES);
  
  normalize1D(error, n, 1e-10);

  if (1) {
    for (x = 0; x < n; x++)
      if (error[x] != 0)
	error[x] *= NUMBER_OF_VISION_FEATURES;
      else
	error[x] = MINIMUM_PROBABILITY;
    
    for (x = 0; x < n; x++){
      float difference;
      difference = error[x] - 1;
      error[x] = 1 + globalVisionParameters.crunchFactor*difference;
    }
    normalize1D(error, n, 1e-10);
  }

  fprintf(stderr, "# Successfully read %s\n", fileName);

  return TRUE;
}


static void
computeCorrelationPositions(){
  int i = 0, x, y, rot;
  double centerX = IMAGE_SIZE_X * 0.5 - 0.5, centerY = IMAGE_SIZE_Y * 0.5 - 0.5;
  double radius = 0.5 * iMin(IMAGE_SIZE_X, IMAGE_SIZE_Y);

  for (x = 0; x < IMAGE_SIZE_X; x++)
    for (y = 0; y < IMAGE_SIZE_Y; y++){
      double deltaX = x - centerX;
      double deltaY = y - centerY;
      double dist = sqrt(deltaY * deltaY + deltaX * deltaX);
      if (dist <= radius){
	correlationPos[i].x = x;
	correlationPos[i].y = y;
	for (rot = 0; rot < NUMBER_OF_ROTATION_ANGLES; rot++){
	  int sourceX, sourceY;
	  double angle = atan2(deltaY, deltaX) - rot;
	  sourceX = round(centerX + dist * cos(angle));
	  sourceY = round(centerY + dist * sin(angle));
	  if (sourceX < 0 || sourceX >= IMAGE_SIZE_X
	      || sourceY < 0 || sourceY >= IMAGE_SIZE_Y){
	    fprintf(stderr, "Invalid value %d %d\n", sourceX, sourceY);
	    exit(0);
	  }
	  rotationSourcePos[rot][i].x = sourceX;
	  rotationSourcePos[rot][i].y = sourceY;
	}
	i++;
      }
    }
  numberOfCorrelationPositions = i;
}


static void
computeVisionFeatureProbTable(visionFeatureProbTable *tab){

  tab->numberOfExpectedFeatures =
    tab->numberOfMeasuredFeatures = NUMBER_OF_VISION_FEATURES;
  
  tab->prob = visionProbabilityTable();

}


static probability**
visionProbabilityTable(){
  probability ** f;
  double sum;
  int diff, var;
  probability minProb=1e30, maxProb=0.0;

  f = (probability **) allocate2D(NUMBER_OF_VISION_FEATURES,
				  NUMBER_OF_VISION_FEATURES,
				  PROBABILITY);
  for (diff = 0; diff < NUMBER_OF_VISION_FEATURES; diff++)
    for (var = 0; var < NUMBER_OF_VISION_FEATURES; var++)
      f[diff][var] = probOfMeasuredVisionFeatureGivenExpected(diff, 0, var)
	+ 1.0e-4;

  /* Normalization */

  for (var = 0; var < NUMBER_OF_VISION_FEATURES; var++){
    sum = 0;
    for (diff = 0; diff < NUMBER_OF_VISION_FEATURES; diff++)
      sum += f[diff][var];
    if (sum > 0)
      for (diff = 0; diff < NUMBER_OF_VISION_FEATURES; diff++)
	f[diff][var] /= sum;
  }
  

  /* Shifting so that the average value is 1 */
  for (var = 0; var < NUMBER_OF_VISION_FEATURES; var++)
    for (diff = 0; diff < NUMBER_OF_VISION_FEATURES; diff++)
      f[diff][var] *= NUMBER_OF_VISION_FEATURES;
  
  
  
  for (var = 0; var < NUMBER_OF_VISION_FEATURES; var++)
    for (diff = 0; diff < NUMBER_OF_VISION_FEATURES; diff++){
      if (f[diff][var] < minProb)
	minProb = f[diff][var];
      if (f[diff][var] >maxProb)
	maxProb = f[diff][var];
    }
  
  if (1){
    
    writeLog( "# Crunchfactor: %f\n:", globalVisionParameters.crunchFactor);
    fprintf(stderr, "# Crunchfactor: %f\n", globalVisionParameters.crunchFactor);
    
    for (diff = 0; diff < NUMBER_OF_VISION_FEATURES; diff++)
      for (var = 0; var < NUMBER_OF_VISION_FEATURES; var++){
	float difference;
	difference = f[diff][var] - 1;
	f[diff][var] = 1 + globalVisionParameters.crunchFactor*difference;
      }
  }


  maxProb = 0.0;
  minProb = 1.0e30;
  for (var = 0; var < NUMBER_OF_VISION_FEATURES; var++)
    for (diff = 0; diff < NUMBER_OF_VISION_FEATURES; diff++){
      if (f[diff][var] < minProb)
	minProb = f[diff][var];
      if (f[diff][var] >maxProb)
	maxProb = f[diff][var];
    }
  
  if (minProb != 0.0)
    fprintf(stderr, "# Maxfactor:  %f, max: %f min: %f\n", maxProb / minProb,
	    maxProb, minProb);

#define OUTPUT
#ifdef OUTPUT
  {
    FILE *fp;
    if ((fp = fopen("visprobfunc.gnu", "w")) != NULL){
      for (var = 0; var < NUMBER_OF_VISION_FEATURES; var++){
	for (diff = 0; diff < NUMBER_OF_VISION_FEATURES; diff++)
	  fprintf(fp, "%d %d %3e\n", diff, var, f[diff][var]);
	fprintf(fp,"\n");
	}
    }
    fclose(fp);
  }
#endif

  
  return f;
}

void
initBlurWeight(int sizeX, int sizeY, pixel standardDeviation)
{
  int x,y;
  double sum = 0;
  double variance = standardDeviation*standardDeviation;
  /* allocate space*/
  blurWeight = (float **) allocate2D(sizeX, sizeY, FLOAT);
  
  /* calculate weights*/
  for (x = 0; x < sizeX; x++)
    for (y = 0; y < sizeY; y++)
      {
	int dx = x-sizeX/2, dy = y-sizeY/2;
	double r = dx*dx+dy*dy;
	double weight = exp(-0.5*r/variance);
	blurWeight[x][y] = weight;
	sum += weight;
      }
  
  /* normalize*/
  if (sum != 0)
    for (x = 0; x < sizeX; x++){
      for (y = 0; y < sizeY; y++){
	blurWeight[x][y] /= sum;
	/* 	fprintf(stderr, "%d %d %.2e \n", x, y, blurWeight[x][y]); */
      }
    }
}


pixel
weightedStdDev(sensing_VISION *image)
{
  unsigned int x, y;
  double mean=0, squares=0;
  
  for (x = 0; x < image->sizeX; x++)
    for (y = 0; y < image->sizeY; y++)
      {
	double I = (double)image->pix[x][y];
	double wI = blurWeight[x][y] * I;
	mean += wI;
	squares += wI*I;
      }
  
  return (pixel) round(sqrt(squares-mean*mean));
}


pixel
centerValue(sensing_VISION *image)
{
  return image->pix[image->sizeX / 2][image->sizeY / 2];
}




void
ReadPGMSHeader(FILE *fp, int *num, int *width, int *height, int *max)
{
  char buf[200];
  fgets(buf, 200, fp); 
  fgets(buf, 200, fp); /* possible comment line */
  while(buf[0]=='#')
    fgets(buf, 200, fp);
  sscanf(buf,"%d %d %d\n",num,width,height);
  fgets(buf, 200, fp); /* get 255 or comment line */
  while(buf[0]=='#')
    fgets(buf, 200, fp);
  sscanf(buf,"%d\n",max);
}

int
ReadPGMStyleHeader(FILE *fp, const char* magic, int *width, int *height, int *max)
{
  char buf[1000];
  fscanf(fp, "%s\n", buf);
  if (strcmp(buf,magic)) return -1;
  
  /* munge comment lines */
  do
    if(fgets(buf, 1000, fp)==NULL) return -1;
  while(buf[0]=='#');
  
  if(sscanf(buf,"%d %d\n",width,height)!=2) return -1;
  
  /* munge comment lines */
  do
    if(fgets(buf, 1000, fp)==NULL) return -1;
  while(buf[0]=='#');
  
  if(sscanf(buf,"%d\n",max)!=1) return -1;
  
  return 0; /* success */
}

int *
ReadPIM(const char *filename, int *width, int *height)
{
  int max;
  FILE *fp;
  size_t size;
  int *data=NULL;
  
  fp = fopen(filename, "rb");
  if (fp==0) goto error;
  
  if(ReadPGMStyleHeader(fp,"PIM",width,height,&max)) goto error;
  
  size = (*width)*(*height);
  data = (int *)malloc(size*sizeof(int));
  if (data==NULL) goto error;
  
  if(fread(data, sizeof(int), size, fp)!=size) goto error;

  if (fclose(fp)) goto error;
  
  return data;
  
error:
  
  if (data!=NULL) free(data);
  return NULL;
}


void
rotateImageFast( sensing_VISION *source, sensing_VISION *destination, float rot){
  int rotIndex = round(normalizedAngle(rot));
  int i;

  
  if (rotIndex == 360) rotIndex = 0;
  
  for (i = 0; i < numberOfCorrelationPositions; i++){
    destination->pix[correlationPos[i].x][correlationPos[i].y] = 
      source->pix[rotationSourcePos[rotIndex][i].x][rotationSourcePos[rotIndex][i].y];
  }
}


void
rotateImage( sensing_VISION *source, sensing_VISION *destination, float rot){
  unsigned int x, y;
  int sourceX, sourceY;
  float centerX = destination->sizeX * 0.5 - 0.5;
  float centerY = destination->sizeY * 0.5 - 0.5;

  fprintf(stderr, "%f %f\n", (float) round(-0.7), (float) round(-1.1));
  
  for (x = 0; x < destination->sizeX; x++){
    for ( y= 0; y < destination->sizeY; y++){
      float deltaY = y - centerY;
      float deltaX = x - centerX;
      float dist = sqrt(deltaY * deltaY + deltaX * deltaX);
      float angle = atan2(deltaY, deltaX);
      angle -= rot;
      sourceX = round(centerX + dist * cos(angle));
      sourceY = round(centerY + dist * sin(angle));
      if (sourceX >= 0 && sourceY >= 0 && sourceX < (int) source->sizeX
	  && sourceY < (int) source->sizeY)
	destination->pix[x][y] = source->pix[sourceX][sourceY];
      else
	destination->pix[x][y] = (pixel) 255;
    }
  }
}
      

static double
sqr(double x){
  return x*x;
}





double
imageSquareDistance(sensing_VISION *image1, sensing_VISION *image2){

  double sum = 0.0;
  pixel **pix1 = image1->pix, **pix2 = image2->pix;
  int i;

  for (i = 0; i< numberOfCorrelationPositions; i++){
    imagePosition pos = correlationPos[i];
    sum += sqr(((double) pix1[pos.x][pos.y]) - pix2[pos.x][pos.y]);
  }

  return sqrt(sum);
}

double
imageCorrelation(sensing_VISION *image1, sensing_VISION *image2){
  int i;
  double sum1 = 0,sum2 = 0, average1, average2, stdDev1 = 0, stdDev2 = 0, cov12 = 0;
  double r;
  pixel **pix1 = image1->pix, **pix2 = image2->pix;
  
  for (i = 0; i< numberOfCorrelationPositions; i++){
    imagePosition pos = correlationPos[i];
    sum1 += pix1[pos.x][pos.y];
    sum2 += pix2[pos.x][pos.y];
  }
  
  average1 = sum1 / numberOfCorrelationPositions;
  average2 = sum2 / numberOfCorrelationPositions;
  
  for (i = 0; i< numberOfCorrelationPositions; i++){
    imagePosition pos = correlationPos[i];
    stdDev1 += sqr(pix1[pos.x][pos.y] - average1);
    stdDev2 += sqr(pix2[pos.x][pos.y] - average2);
    cov12 += (pix1[pos.x][pos.y] - average1)*(pix2[pos.x][pos.y] - average2);
  }
  
  stdDev1 = sqrt(stdDev1 / (numberOfCorrelationPositions - 1));
  stdDev2 = sqrt(stdDev2 / (numberOfCorrelationPositions - 1));

  if (stdDev1 < 1e-10){
    if (stdDev2 < 1e-10)
      r = 1.0;
    else
      r = 0.0;
  }
  else if (stdDev2 < 1e-10){
    if (stdDev1 < 1e-10)
      r = 1.0;
    else
      r = 0.0;
  }
  else{
    cov12 /= (numberOfCorrelationPositions - 1);
    r = cov12 / (stdDev1 * stdDev2);
  }
  
  return r;
}



void
zoomImage(sensing_VISION *image, sensing_VISION *zoomedImage, double zoomFactor)
{
  unsigned int x,y,startX, startY;

  double centerX = image->sizeX * 0.5 + 0.5;
  double centerY = image->sizeY * 0.5 + 0.5;

  for (x = 0; x < zoomedImage->sizeX; x++)
    for (y = 0; y < zoomedImage->sizeY; y++){
      startX = round(centerX + (x - centerX) / zoomFactor);
      startY = round(centerX + (y - centerY) / zoomFactor);
      zoomedImage->pix[x][y] = image->pix[startX][startY];
    }


}
  


void
getStoredImage(int gridX, int gridY, float rot, sensing_VISION *image){
  static FILE *databaseFp;
  static bool firstTime = TRUE;
  static int *imageIndex;
  long index;
  int offsetX = 35, offsetY = 52;
  static int width, height;
  pixel tiny[625];
  static long pgms_start = 0;
  static sensing_VISION databaseImage, tmpImage;
  bool imagePosition = TRUE;
  
  unsigned int i,y,x;

  if (firstTime)
    {
      int num, w, h, max;

      databaseFp = fopen("database.pgms","r");
      if (databaseFp == NULL){
	fprintf(stderr, "Error: could not open database.pgms\n");
	exit(0);
      }
      
      ReadPGMSHeader(databaseFp, &num, &w, &h, &max);

      if(w!=IMAGE_SIZE_X || h!=IMAGE_SIZE_Y){
	fprintf(stderr, "expected %d*%d images in database\n", (int) IMAGE_SIZE_X,
		(int) IMAGE_SIZE_Y);
	exit(0);
      }

      pgms_start = ftell(databaseFp);
      imageIndex = ReadPIM("database.pim",&width,&height);
      
      
      databaseImage.sizeX = IMAGE_SIZE_X;
      databaseImage.sizeY = IMAGE_SIZE_Y;
      databaseImage.pix = (pixel**) allocate2D(databaseImage.sizeX ,
					     databaseImage.sizeY, PIXEL);
      tmpImage = databaseImage;
      tmpImage.pix = (pixel**) allocate2D(databaseImage.sizeX ,
					     databaseImage.sizeY, PIXEL);
      firstTime = FALSE;
    }

  if (gridY < offsetY || gridX < offsetX){
    imagePosition = FALSE;
  }
  else {
    index = (gridY - offsetY) * width + gridX - offsetX;
    i=0;
    
    if (imageIndex[index] != -1){
      
      long dataBaseOffset = imageIndex[index]* ((long) 25);
      
      if(fseek(databaseFp,pgms_start+dataBaseOffset,SEEK_SET)){
	fprintf(stderr, "fseek error");
	exit(0);
      }
      
      if(fread(tiny, 1, 625, databaseFp)!=625) {
	fprintf(stderr, "pgms read error");
	exit(0);
      }
      
      i=0;
      for (y = 0; y< databaseImage.sizeY; y++)
	for (x = 0; x < databaseImage.sizeX; x++)
	  databaseImage.pix[x][y] = tiny[i++];
      
      (void) rotateImageFast(&databaseImage, image, -rot);
    }
    else {
      imagePosition = FALSE;
    }
  }

  if (! imagePosition)
    for (y = 0; y< databaseImage.sizeY; y++)
      for (x = 0; x < databaseImage.sizeX; x++)
	databaseImage.pix[x][y] = image->pix[x][y] = (pixel) 128;
  
}



pixel
standardDeviationValue(sensing_VISION *image)
{
  unsigned int x,y;
  double avg = 0.0;
  double stdDev = 0.0;
  
  for (x = 0; x < image->sizeX; x++)
    for (y = 0; y < image->sizeY; y++)
      avg += image->pix[x][y];

  avg = avg / image->sizeX /  image->sizeY;

  for (x = 0; x < image->sizeX; x++)
    for (y = 0; y < image->sizeY; y++)
      stdDev += fSqr(image->pix[x][y] -avg);
  
  stdDev = sqrt(stdDev / (image->sizeX * image->sizeY - 1));
  
  return (pixel) round(stdDev);
}


pixel
averageValue(sensing_VISION *image)
{
  unsigned int x,y;
  double avg = 0.0;
  pixel **pix = image->pix;

  for (x = 0; x < image->sizeX; x++)
    for (y = 0; y < image->sizeY; y++)
      if (pix[x][y] > 0){
	avg += pix[x][y];
      }
  return (pixel) round(avg / image->sizeX /  image->sizeY);
}

  

pixel
blurValue(sensing_VISION *image)
{
  unsigned int x,y;
  double value = 0;
  
  for (x = 0; x < image->sizeX; x++)
    for (y = 0; y < image->sizeY; y++)
      value += blurWeight[x][y] * image->pix[x][y];
  
  return (pixel) round(value);
}


pixel
lightFeature(sensing_VISION *image)
{
  unsigned int x, y;
  pixel feature = centerValue(image);
  if (feature <= 200)
    for (x = 0; x < image->sizeX; x++)
      for (y = 0; y < image->sizeY; y++){
	if (image->pix[x][y] > 200)
	  feature = 255;
      }
  else
    (feature = 255);

  return feature;
}


void
flipImage(sensing_VISION *image){
  pixel pix;
  unsigned int x, y;
  for (x = 0; x < image->sizeX; x++)
    for (y = 0; y < image->sizeY / 2; y++){
      pix = image->pix[x][y];
      image->pix[x][y] = image->pix[x][image->sizeY - y - 1];
      image->pix[x][image->sizeY - y - 1] = pix;
    }
}
  

pixel
imageFeature(sensing_VISION *image)
{
  
  return averageValue(image);
  /*  return FeatureConvolve2D(feature, image->pix, image->sizeX / 2, image->sizeY/2); */
  
}

/*------------------------------------------------------------------------
 * Integrates an image into the position probability grid.
 *-----------------------------------------------------------------------*/
static void
integrateImageIntoGrid( informationsFor_VISION* info, actionInformation *actionInfo)
{
  int plane;

  static int imageCnt = 0;
  static gridWindow *win;
  static gridWindow *imageWin;
  static visionMap        visMapSmall;
  static bool firstTime = TRUE;
  static mapProbability **mapProb;
  int cnt = 0;
  char * separator = "# -------------------------------------------------";
  
  register unsigned int x, y;
  visionMap visMap = *(info->visMap);
  visionMap visVarMap = *(info->visVarMap);
  pixel feature = (pixel) 0;
  sensing_VISION *image = info->image;
  pixel unknown = (pixel) 255;
  pixel variance = (pixel) 0; 
  

  float max = 0.0;
  float min = 1.0e30;
  probabilityGrid map;
  map = *(info->map);

  writeLog("#        Image no.%d ... ", imageCnt);
  fprintf( stderr, "%s\n", separator);
  fprintf( stderr,"#        Image no.%d ... ", imageCnt);
  
  
  if (firstTime){
    initBlurWeight(image->sizeX, image->sizeY, 6);
    mapProb = (mapProbability **) allocate2D(map.sizeX, map.sizeY, MAP_PROBABILITY);
    visMapSmall.sizeX = image->sizeX;
    visMapSmall.sizeY = image->sizeY;
    visMapSmall.resolution = visMap.resolution;
    visMapSmall.offsetX = visMapSmall.offsetY = 0;
    visMapSmall.pix = image->pix;
    imageWin = createVisionMapWindow( &visMapSmall, "Image", 0, 0, 6);
    win = createMapWindow( &map, "Image Probability Map", 0, 0, 2);
    if (1) modifyVisionMap(visMap);
    firstTime = FALSE;
  }
  
  map.prob = mapProb;

  displayVisionMapWindow(&visMapSmall, imageWin);
  
  feature = imageFeature(image);

  fprintf(stderr, "%d\n", (int) feature);
  for (x = 0; x < info->grid->sizeX; x++){
    int visMapX = round(x *  ((float) visMap.sizeX) / info->grid->sizeX);
    for (y = 0; y < info->grid->sizeY; y++){
      int visMapY = round(y * ((float) visMap.sizeY) / info->grid->sizeY);
      probability *p = &map.prob[x][y];
      *p= errorProb[abs((visMap.pix[visMapX][visMapY] - feature) / 2)];
      
      if (*p > max)
	max = *p;
      if (*p < min)
	min = *p;
    }
  }
  
  if (0) displayMapWindowMax(&map, win);
  
  for ( plane = 0; plane < info->grid->sizeZ; plane++) {
    unsigned int sizeX = info->grid->sizeX;
    unsigned int sizeY = info->grid->sizeY;
    
    if ( info->grid->updatePlane[plane]){
      probability ** gridPlane = info->grid->prob[plane];
      
      for ( x = 0; x < sizeX; x++) {
	register probability *cell, *prob;
	register pixel *visVarPix;
	probability minProb = info->grid->minimumProbability;
	cell = &gridPlane[x][0];
	prob = map.prob[x];
	visVarPix = visVarMap.pix[x];
	for (y = 0; y < sizeY; y++){
	  if ((*visVarPix != unknown) && (*cell > minProb)){
	    *cell++ *= *prob++;
	    cnt++;
	    visVarPix++;
	  }
	  else{
	    cell++;
	    prob++;
	    visVarPix++;
	  }
	}
	/* Just to avoid crash of the system. */
	if ( x % 5 == 0)
	  swallowStatusReports(DONT_WAIT);
      }
    }
  }
  fprintf(stderr, "%d %d", feature, variance);
  
  

  imageCnt++;
  image->isNew = FALSE;
  fprintf( stderr," %2.2f%%, done\n",
	   100.0 * cnt / (float) info->grid->sizeX /
	   info->grid->sizeX /
	   info->grid->sizeZ);
  writeLog(" %2.2f%%, done\n",
	   100.0 * cnt / (float) info->grid->sizeX /
	   info->grid->sizeX /
	   info->grid->sizeZ);
  
  writeLog("%s\n", separator);
  fprintf(stderr, "%s\n", separator);

  if (actionInfo->localMaxima.numberOfCells == 1){
    realPosition realPos = actionInfo->localMaxima.cell[0].pos;
    gridPosition gridPos =
      gridPositionOfRealPosition(realPos,
				 &(actionInfo->positionProbs));
    if (coordinateInGrid(gridPos.x, gridPos.y, &(actionInfo->positionProbs)))
      writeLog("%d %d %d %d %d #visionfeature (expected measured diff x y)\n",
	       (int) visMap.pix[gridPos.x][gridPos.y], (int) feature, (int)
	      visMap.pix[gridPos.x][gridPos.y] - (int) feature, gridPos.x, gridPos.y);
    
  }
  
  
  
}



void
dumpWindows(visionMap image)
{
  static int cnt=0;
  static long int skipCnt = 0;

  if (skipCnt %3 == 0){
    char command[256];
    
    if (skipCnt > 900 && skipCnt < 1400){
      sprintf(command, "xwd -id `xwininfo -name Samples | grep \"Window id\" | cut -d ' '  -f 4` | xwdtopnm | ppmtogif > images/samples%05d.gif", cnt);
      if(0)fprintf(stderr, "%s\n", command);
      if (1)system(command);
      sprintf(command, "images/image%05d.ppm", cnt);
      writeVisionMap(command, "", &image);
      cnt++;
    }
  }
  skipCnt++;
}





static void
integrateImageIntoSamples( informationsFor_VISION* info,
			   actionInformation *actionInfo)
{

  sampleSet *samples = &(actionInfo->samples);
  static double probSum = 0;
  static long integratedSamples = 0;
  int s;
  visionMap visMap = *(info->visMap);
  float resolution = visMap.resolution;
  pixel feature = (pixel) 0;
  char * separator = "# -------------------------------------------------";

  /* static variables */
  static bool firstTime = TRUE;
  static int imageCnt = 0;
  static visionMap imageMap, rotatedImageMap;
  static gridWindow *imageWin, *rotatedImageWin;
    
  sensing_VISION *image = info->image;
  static sensing_VISION rotatedImage;

  double max = 0.0, min = MAXFLOAT;
  int maxSample = 0;
  
  
  
  writeLog("#        Image no.%d ... ", imageCnt);
  writeLog("%s\n", separator);
  fprintf(stderr, "%s\n", separator);
  fprintf( stderr,"#        Image no.%d ... \n", imageCnt);

  
#define DISPLAY_IMAGE 1
  
  if (firstTime){
    unsigned int x, y; 
    initBlurWeight(image->sizeX, image->sizeY, 6);
    imageMap.sizeX = image->sizeX;
    imageMap.sizeY = image->sizeY;
    imageMap.resolution = visMap.resolution;
    imageMap.offsetX = imageMap.offsetY = 0;
    imageMap.pix = image->pix;
    rotatedImageMap = imageMap;
    rotatedImage = *image;
    rotatedImage.pix = rotatedImageMap.pix = (pixel**) allocate2D(rotatedImageMap.sizeX ,
								 rotatedImageMap.sizeY, PIXEL);
    
    for (x = 0; x < rotatedImageMap.sizeX; x++)
      for (y = 0; y< rotatedImageMap.sizeY; y++)
	rotatedImage.pix[x][y] = (pixel) 255;
    
    
    
    if (DISPLAY_IMAGE){
      imageWin = createVisionMapWindow( &imageMap, "Image", visMap.sizeX+10,
					0, 6);
      if (USE_DATABASE_FEATURE)
	rotatedImageWin = createVisionMapWindow( &rotatedImageMap,
						 "Rotated",
						 visMap.sizeX+10,
						 330, 6);
    }
    
    firstTime = FALSE;
  }


  if (0) {

    visionMap largeMap = visMap;
    unsigned int x, y;

    largeMap.sizeX = 6 * visMap.sizeX;
    largeMap.sizeY = 6 * visMap.sizeY;

    largeMap.pix = (pixel **) allocate2D(largeMap.sizeX, largeMap.sizeY, PIXEL);

    for (x = 0; x < largeMap.sizeX ; x++)
      for (y = 0; y < largeMap.sizeY; y++){
	largeMap.pix[x][y] = 0;
      }
    
    for (x = 0; x < visMap.sizeX ; x++)
      {
	fprintf(stderr, "\r%.2f",(float) x / visMap.sizeX);
	for (y = 0; y < visMap.sizeY; y++){
	  int x1, y1, x2, y2;
	  getStoredImage(x, y, 0, &rotatedImage);
	  if (0) displayVisionMapWindow(&rotatedImageMap, rotatedImageWin);
	  for (x1 = 6*x, x2=11; x1 < 6*x +6; x1++,x2++){
	    for (y1 = 6*y, y2 = 11; y1 < 6*y + 6; y1++,y2++){
	      largeMap.pix[x1][y1] = rotatedImage.pix[x2][y2];
	    }
	  }
	  
	}
      }
    writeVisionMap("largemap",".pgm", &largeMap);
    exit(0);
  }
  
  
  
  if (USE_DATABASE_FEATURE){
    for ( s = 0; s < samples->numberOfSamples; s++) {
      double* sampleProb = &(samples->sample[s].weight);
      realPosition pos = samples->sample[s].pos;
      int gridX = pos.x / resolution;
      int gridY = pos.y / resolution;
      
      getStoredImage(gridX, gridY, pos.rot, &rotatedImage);
      
      *sampleProb =  imageSquareDistance(image, &rotatedImage) / numberOfCorrelationPositions ;
      
      if (*sampleProb > max){
	max = *sampleProb;
      }
      if (*sampleProb < min){
	min = *sampleProb;
	maxSample = s;
      }
      
    }
    
    for ( s = 0; s < samples->numberOfSamples; s++) {
      double* sampleProb = &(samples->sample[s].weight);
      *sampleProb = exp(min - *sampleProb);
      if (*sampleProb < 1e-6)
	*sampleProb = 1e-6;
    }
    
    {
      realPosition pos = samples->sample[maxSample].pos;
      int gridX = pos.x / resolution;
      int gridY = pos.y / resolution;
      fprintf(stderr, "# Best sample: %d %f (worst %f)\n", maxSample,
	      (float) samples->sample[maxSample].weight,
	      (float) exp(min - max));
      getStoredImage(gridX, gridY, pos.rot, &rotatedImage);
      displayVisionMapWindow(&imageMap, imageWin);
      displayVisionMapWindow(&rotatedImageMap, rotatedImageWin);
    
    }
  }
  
  else {
    double sum = 0;
    int splitPoint;
    feature = imageFeature(image);
    if (0) for ( s = 0; s < samples->numberOfSamples; s+=10){
      realPosition pos = samples->sample[s].pos;
      int gridX = pos.x / resolution;
      int gridY = pos.y / resolution;
      if (gridX > 0 && gridX < (int) visMap.sizeX
	  && gridY > 0 && gridY < (int) visMap.sizeY)
	writeLog("%d %d #meas and exp feature\n", feature, visMap.pix[gridX][gridY]);
    }
#define FORWARD_QUOTA 0.7
    if (featureList.numberOfPixels[feature] == 0)
      splitPoint = samples->numberOfSamples;
    else
      splitPoint =  round(samples->numberOfSamples *  FORWARD_QUOTA);
    
    if (1)
      {
	for ( s = 0; s < splitPoint; s++) {
	  double* sampleProb = &(samples->sample[s].weight);
	  realPosition pos = samples->sample[s].pos;
	  int gridX = pos.x / resolution;
	  int gridY = pos.y / resolution;
	  if (gridX > 0 && gridX < (int) visMap.sizeX
	      && gridY > 0 && gridY < (int) visMap.sizeY)
	    if (visMap.pix[gridX][gridY] > 0)
	      *sampleProb =  (errorProb[abs((visMap.pix[gridX][gridY]
					     - feature))]);
	    else
	      *sampleProb = 1e-35;
	  else
	    *sampleProb = 1e-35;
	  sum += *sampleProb;
	  
	}
	

	probSum += sum;
	integratedSamples += splitPoint;

	fprintf(stderr, "Average Prob: %g\n", probSum / integratedSamples);
	if (splitPoint > 0 && sum != 0)
	  sum = sum / splitPoint;
	else
	  sum = 1e-35;
#define RANDOM 0
	if (RANDOM)
	  for (s = splitPoint; s < samples->numberOfSamples; s++) {
	    int sampleIndex;
	    imagePosition imagePos;
	    int posIndex;
	    sampleIndex = s;
	    
	    posIndex = randMax(knownPositions.numberOfPixels -1 );
	    imagePos =
	      knownPositions.pos[posIndex];
	    samples->sample[sampleIndex].pos.x = imagePos.x * resolution;
	    samples->sample[sampleIndex].pos.y = imagePos.y * resolution;
	    samples->sample[sampleIndex].pos.rot = randMax(10000) / 5000.0 * M_PI;
	    samples->sample[sampleIndex].weight = sum;
	  }    
	    
	else
	  for ( s = splitPoint; s < samples->numberOfSamples; s++) {
	    int sampleIndex;
	    int randomFeature;
	    imagePosition imagePos;
	    randomFeature = feature + 10.0 * randomGauss();
	    if (1)
	      sampleIndex = s;
	    else
	      sampleIndex = randMax(samples->numberOfSamples - 1);
	    if (randomFeature < 0 || randomFeature >= NUMBER_OF_VISION_FEATURES ||
		featureList.numberOfPixels[randomFeature] == 0)
	      randomFeature = feature;
	    
	    imagePos =
	      featureList.pos[randomFeature][randMax(featureList.numberOfPixels[randomFeature] -1)];
	    samples->sample[sampleIndex].pos.x = imagePos.x * resolution;
	    samples->sample[sampleIndex].pos.y = imagePos.y * resolution;
	    samples->sample[sampleIndex].pos.rot = randMax(10000) / 5000.0 * M_PI;
	    samples->sample[sampleIndex].weight = sum;
	    /* p(s) would be 0.00528068; */
	  }
	
	/* Normalization */
	sum = 0;
	for ( s = 0; s < samples->numberOfSamples; s++)
	  sum += samples->sample[s].weight;
	if (sum > 0){
	  sum = 1/sum;
	  for ( s = 0; s < samples->numberOfSamples; s++) {
	    samples->sample[s].weight *= sum;
	  }
	}
      }
  }
  
  imageCnt++;
  
  samples->alreadySampled = FALSE;
  writeLog("%s\n", separator);
  fprintf(stderr, "%s\n", separator);

  if (1) displayVisionMapWindow(&imageMap, imageWin);
  
  
  if (0 && DISPLAY_IMAGE){
    extern float elapsedScriptTime;
    realPosition measuredPos;
    int gridX = 0, gridY = 0;
    bool success = FALSE;
    measuredPos =  measuredRobotPosition( elapsedScriptTime, &success);
    gridX = measuredPos.x / resolution;
    gridY = measuredPos.y / resolution;
    getStoredImage(gridX, gridY, measuredPos.rot, &rotatedImage);
    if (0) displayVisionMapWindow(&imageMap, imageWin);
    if (0 && success)
      displayVisionMapWindow(&rotatedImageMap, rotatedImageWin);
  }
  else if (0){
    void displayVisionMapinPositionWindow(visionMap *visMap);
    if (1) displayVisionMapinPositionWindow(&imageMap);
  }
  if (0) dumpWindows(imageMap);
  

}



static void
integrateImage( informationsFor_VISION* info, actionInformation *actionInfo)
{
  if ( *(info->useProbGrid)) {
    integrateImageIntoGrid( info, actionInfo);
  }
  else
    integrateImageIntoSamples( info, actionInfo);

}












