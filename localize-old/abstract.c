
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/abstract.c,v $
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
 * $Log: abstract.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.52  2000/08/09 23:40:44  wolfram
 * Fixed the pain problem with MIN_WINDOW_SCALE
 *
 * Revision 1.51  1999/06/29 21:36:21  fox
 * Changes for new script reader.
 *
 * Revision 1.50  1999/03/08 16:47:38  wolfram
 * Fixed problems with map, caused by the fact that it no longer accpets
 * correction parameters from LOCALIZE.  Improved the handling of sonars
 * with Pioneers.  Currently sonars do not work properly on a B21.
 *
 * Revision 1.49  1999/01/22 17:48:00  fox
 * Final version of sampling paper and denver demo.
 *
 * Revision 1.48  1999/01/11 19:47:45  fox
 * Added several parameters to sampling. Number of samples is not fixed any
 * more.
 *
 * Revision 1.47  1998/11/23 19:45:06  fox
 * Latest version.
 *
 * Revision 1.46  1998/11/17 23:26:15  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.45  1998/11/03 21:02:15  fox
 * Lasers will not be integrated if the robot rotates faster than
 * MAX_ROT_VEL_FOR_LASERS deg/sec.
 *
 * Revision 1.44  1998/10/19 18:29:53  fox
 * *** empty log message ***
 *
 * Revision 1.43  1998/10/02 15:16:35  fox
 * Several improvements of condensation. Updated the function calls to the
 * new version of the distancServer.
 *
 * Revision 1.42  1998/09/18 17:24:41  fox
 * Added skeleton files for condensation.
 *
 * Revision 1.41  1998/09/18 15:44:24  fox
 * First attempt to get a running system after museum.
 *
 * Revision 1.40  1998/08/19 16:33:52  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.39  1998/06/12 10:16:21  fox
 * Implemented virutal sensor.
 *
 * Revision 1.38  1998/02/13 14:12:17  fox
 * Minor changes.
 *
 * Revision 1.37  1998/01/27 15:25:23  fox
 * Minor changes.
 *
 * Revision 1.36  1998/01/22 13:06:10  fox
 * First version after selection-submission.
 *
 * Revision 1.35  1998/01/08 12:19:46  wolfram
 * Minor changes for Sebastians new maps
 *
 * Revision 1.34  1998/01/05 10:37:08  fox
 * Several changes especially in selection parts.
 *
 * Revision 1.33  1997/12/02 15:20:33  fox
 * Nothing remarkable.
 *
 * Revision 1.32  1997/11/27 18:11:15  fox
 * Several changes to make angles work better.
 *
 * Revision 1.31  1997/11/25 17:12:39  fox
 * Should work.
 *
 * Revision 1.30  1997/11/21 15:36:01  fox
 * Modifications in graphic
 *
 * Revision 1.29  1997/11/20 12:58:06  fox
 * Version with good sensor selection.
 *
 * Revision 1.28  1997/11/07 12:39:37  fox
 * Added some graphic features.
 *
 * Revision 1.27  1997/10/31 13:14:35  fox
 * Removed active sensing comments.
 *
 * Revision 1.26  1997/10/31 13:11:39  fox
 * Version for active sensing.
 *
 * Revision 1.25  1997/10/01 11:29:55  fox
 * Minor changes.
 *
 * Revision 1.24  1997/09/11 20:01:04  fox
 * Final cmu version.
 *
 * Revision 1.23  1997/09/09 19:45:10  fox
 * Fixed a bug in finding local maxima and did several changes to active
 * localization.
 *
 * Revision 1.22  1997/08/22 04:16:37  fox
 * Final version before IJCAI.
 *
 * Revision 1.21  1997/08/19 18:18:45  fox
 * Last version before I change from information to entropy.
 *
 * Revision 1.20  1997/08/16 22:59:48  fox
 * Last version before I change selsection.
 *
 * Revision 1.19  1997/08/02 16:51:00  wolfram
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
 * Revision 1.18  1997/07/04 17:29:12  fox
 * Final version before holiday!!!
 *
 * Revision 1.17  1997/06/27 16:26:25  fox
 * New model of the proximity sensors.
 *
 * Revision 1.16  1997/06/26 11:23:16  fox
 * Fixed a bug in normalize.
 *
 * Revision 1.15  1997/06/25 14:16:37  fox
 * Changed laser incorporation.
 *
 * Revision 1.14  1997/06/20 07:36:06  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.13  1997/06/03 11:49:19  fox
 * Museum version.
 *
 * Revision 1.12  1997/05/27 07:42:28  fox
 * Nothing special.
 *
 * Revision 1.11  1997/05/26 10:31:55  fox
 * Added dump of rear laser and sonar data.
 *
 * Revision 1.10  1997/05/26 08:47:40  fox
 * Last version before major changes.
 *
 * Revision 1.9  1997/04/30 12:25:38  fox
 * Some minor changes.
 *
 * Revision 1.8  1997/04/10 13:01:02  fox
 * Fixed a bug.
 *
 * Revision 1.7  1997/04/08 14:56:21  fox
 * Changed the reset function for new planes and the prob function.
 *
 * Revision 1.6  1997/04/07 11:03:20  fox
 * Should be ok.
 *
 * Revision 1.5  1997/04/03 13:17:49  fox
 * Some minor changes.
 *
 * Revision 1.4  1997/04/02 08:57:32  fox
 * Display the robot right after sensor selection.
 *
 * Revision 1.3  1997/03/19 17:52:40  fox
 * New laser parameters.
 *
 * Revision 1.2  1997/03/18 18:45:28  fox
 * First executable version with a global scan mask for different sensor types.
 *
 * Revision 1.1  1997/03/13 17:36:18  fox
 * Temporary version. Don't use!
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
#include "abstract.h"
#include "proximity.h"
#include "proximityTools.h"
#include "movement.h"
#include "selection.h"
#include "graphic.h"
#include "function.h"
#include "script.h"
#include "activeInternal.h"
#include "laser.h"
#include "condensation.h"


static void
computeOptimalScanMaskSeveralMaxima( abstractSensorVector* scan,
				     realCellList* localMaxima,
				     positionProbabilityGrid* grid,
				     sampleSet* samples,
				     int useProbGrid);

static void
computeOptimalScanMaskOneMaximum( abstractSensorVector* scan,
				  realCellList* localMaxima,
				  positionProbabilityGrid* grid);


/********************************************************************
 ********************************************************************
 * Global functions.
 ********************************************************************
 ********************************************************************/



/**********************************************************************
 * integrates one reading to a given list of grid cells
 ********************************************************************/
void
integrateReadingIntoCellList( abstractSensorType* sensor,
			      int sensorOffsetType,
			      realCellList* cellList,
			      positionProbabilityGrid* grid)
{
  int i;
  float sum = 0.0;

  for (i = 0; i < cellList->numberOfCells; i++) {

    if ( cellList->inMap[i]) {
      
      gridPosition sensorPosition;
      bool consider = TRUE;

      if ( sensorOffsetType != NO_OFFSET) {
	realPosition sensPos =
	  endPoint( cellList->cell[i].pos, sensor->integrationInfo.sensorOffset);
	sensorPosition = gridPositionOfRealPosition( sensPos, grid);
	consider = coordinateInGrid( sensorPosition.x, sensorPosition.y, grid);
      }
      else
	sensorPosition = gridPositionOfRealPosition( cellList->cell[i].pos, grid);
      
      if ( consider)
	cellList->cell[i].prob *=
	  sensor->probOfFeatureGivenPosition( sensor->measuredFeature,
					   sensorPosition,
					   sensor->infoForFeatures);
      else
	cellList->cell[i].prob *=
	  1.0 / sensor->numberOfFeatures;
    }
    sum += cellList->cell[i].prob;
  }
  
  if (sum > 0.0){
     for (i = 0; i < cellList->numberOfCells; i++)
	cellList->cell[i].prob /= sum;
  }
}

  
/***********************************************************************
 * Sets the scan mask.
 ***********************************************************************/
void
setFixedScanMask( abstractSensorVector* abstract,
		  int indexOfFirstReading)
{
/* #define CUT_SENSOR_BEAMS */
#ifdef CUT_SENSOR_BEAMS
  extern laserParameters globalLaserParameters;
#endif 

  int i;
  int step = abstract->numberOfSensors / abstract->numberOfSensorsToBeUsed;

  abstract->mask[0] = indexOfFirstReading;
  
  for ( i = 1; i < abstract->numberOfSensorsToBeUsed; i++)
    abstract->mask[i] = (indexOfFirstReading + i*step) % abstract->numberOfSensors;

#ifdef CUT_SENSOR_BEAMS

  if ( globalLaserParameters.cutFeature > 0) { 
    int j = 0;

    for ( i = 0; i < abstract->numberOfSensorsToBeUsed; i++)
      if ( abstract->sensor[i].measuredFeature < globalLaserParameters.cutFeature) {
	abstract->mask[j++] = i;
      }
    abstract->numberOfSensorsToBeUsed = j;
  }
#endif
}


/***********************************************************************
 * Sets a random scan mask.
 ***********************************************************************/
void
setRandomScanMask( abstractSensorVector* abstract)
{
  int i;
  int sensor[MAX_SIZE_OF_SCAN];
  int num;

  if (0) writeLog( "# Randomly chosen sensors: ( ");
  for (i = 0; i < abstract->numberOfSensors; i++)
    sensor[i] = i;
  
  for (i = 0; i < abstract->numberOfSensorsToBeUsed; i++){
    num = randMax(abstract->numberOfSensors - i - 1);
    abstract->mask[i] = sensor[num];
    if (0) writeLog( "%d ",sensor[num]);
    sensor[num] = sensor[abstract->numberOfSensors - i - 1];
  }

  writeLog( ")\n");
}



/*-----------------------------------------------------------------
 * Set the corresponding values in the abstract sensors.
 *-----------------------------------------------------------------*/
void
updateAbstractSensors( rawSensings* actualSensings,
		       abstractSensorVector* abstractSensors)
{
  int abstractSens, sensor;

  /* Each abstract sensor type. */
  for ( abstractSens = 0; abstractSens < NUMBER_OF_ABSTRACT_SENSORS; abstractSens++) {

    /* Each reading is converted into a sensor. */
    for ( sensor = 0;
	  sensor < abstractSensors[abstractSens].numberOfSensors;
	  sensor++) {
      
      abstractSensorType* abstract =
	&(abstractSensors[abstractSens].sensor[sensor]);

      abstract->measuredFeature =
	abstract->extractedFeature( actualSensings,
				    abstract->sensorNumberOfType,
				    abstract->infoForFeatures);
    }
  }
}


/*-----------------------------------------------------------------
 * Sets the optimal scan mask for different kind of abstract sensors.
 *-----------------------------------------------------------------*/
void
setGlobalScanMask( abstractSensorVector* sensors,
		   realCellList* localMaxima,
		   positionProbabilityGrid* grid,
		   sampleSet* samples,
		   int useProbGrid)
{
  int abstractSens, i, sensCnt = 0;
  int scanStart = 0, scanEnd;
  abstractSensorVector globalScan;
  abstractSensorType sensor[MAX_SIZE_OF_SCAN];
  int mask[MAX_SIZE_OF_SCAN];
  
  globalScan.sensor = sensor;
  globalScan.mask   = mask;
  globalScan.numberOfSensors = 0;
  globalScan.numberOfSensorsToBeUsed = 0;

  /* Each abstract sensor type. */
  for ( abstractSens = 0;
	abstractSens < NUMBER_OF_ABSTRACT_SENSORS; abstractSens++) {

    abstractSensorVector scan = sensors[abstractSens];

    if ( (scan.chooseOptimal) && *(scan.integrate)) {
      globalScan.numberOfSensors += scan.numberOfSensors;
      globalScan.numberOfSensorsToBeUsed += scan.numberOfSensorsToBeUsed;
      
      for ( i = 0; i < scan.numberOfSensors; i++, sensCnt++)
	globalScan.sensor[sensCnt] = scan.sensor[i];
    }
  }

  /* Check wether anything has to be done here. */
  if ( globalScan.numberOfSensors == 0)
    return;

  /* Set the mask of the global scan. */
  setRandomScanMask( &globalScan);

  setOptimalScanMask( &globalScan,
		      localMaxima,
		      grid,
		      samples,
		      useProbGrid);
  
  /* Set the masks of the local scans. */
  for ( abstractSens = 0;
	abstractSens < NUMBER_OF_ABSTRACT_SENSORS; abstractSens++) {
    
    abstractSensorVector* scan = &(sensors[abstractSens]);
    if ( (scan->chooseOptimal) && *(scan->integrate)) {
      
      scanEnd = scanStart + scan->numberOfSensors;
      scan->numberOfSensorsToBeUsed =
	extractPartialScanMask( globalScan.mask,
				globalScan.numberOfSensorsToBeUsed,
				scan->mask,
				scanStart, scanEnd);

      scanStart += scan->numberOfSensors;
    }
  }
}



/**********************************************************************
 * computes a mask of the optimal sensors
 ********************************************************************/

#define MAX_NUMBER_OF_INFO_WINDOWS 10
#define SONAR_INFO_NUMBER 0
#define LASER_INFO_NUMBER 1
#define ALL_INFO_NUMBER 2
static void
displayInformation( double *information, int numberOfInformation,
		    double referenceValue, int windowNumber);

#define PROB_FACTOR 100.0

void
setOptimalScanMask( abstractSensorVector* scan,
		    realCellList* localMaxima,
		    positionProbabilityGrid* grid,
		    sampleSet* samples,
		    int useProbGrid)
{
  if ( useProbGrid) {
    
    if (localMaxima->numberOfCells <= 0)
      return; /* nothing has to be done */
    
    if ( localMaxima->numberOfCells == 1)
      computeOptimalScanMaskOneMaximum( scan, localMaxima, grid);
    
    /* Treat like one maximum */
    else if ( localMaxima->cell[0].prob / localMaxima->cell[1].prob > PROB_FACTOR)
      computeOptimalScanMaskOneMaximum( scan, localMaxima, grid);
    
    else
      computeOptimalScanMaskSeveralMaxima( scan, localMaxima, grid, samples, useProbGrid);
  }
  else { 
    computeOptimalScanMaskSeveralMaxima( scan, localMaxima, grid, samples, useProbGrid);
  }
}


double INFO_ONE_THRESHOLD = 0.00;
double INFO_SEVERAL_THRESHOLD = 0.000;

/**********************************************************************
 * Computes a mask of the optimal sensors if only one global maximum
 * is given in the probability distribution.
 ********************************************************************/
static void
computeOptimalScanMaskOneMaximum( abstractSensorVector* scan,
				  realCellList* localMaxima,
				  positionProbabilityGrid* grid)
{

  /* Treat the maxima as one and try to concentrate the probability in the center. */
  if ( localMaxima->cell[0].prob * localMaxima->originalSumOfProbs > 0.1) {
    
    double entropyDiff[MAX_SIZE_OF_SCAN];
    int num, cnt=0;

    /* We search around the global maximum and treat the other cells as local
     * maxima. */
    gridPosition maxPos = gridPositionOfRealPosition( localMaxima->cell[0].pos,
						      grid);
    gridPosition gridPos;
    realCellList cells;
    realCell tmpCell;
    int zCount, x, y;
    int deltaRot;
    int deltaPosX;
    int deltaPosY;
    
    probability orgSum = 0.0;

    cells.numberOfCells = 0;

#define MAX_NUMBER_OF_CELLS_IN_CUBE 1000
   
    deltaPosY = deltaPosX = 4;
    deltaRot = 3;

    /*-----------------------------------------------------------------------------
     * Collect all cells close to the global maximum.
     * ----------------------------------------------------------------------------*/
    for ( zCount = maxPos.rot - deltaRot; zCount <= maxPos.rot + deltaRot; zCount+=1) {

      gridPos.rot = torusPosition(zCount, grid->sizeZ);
      
      for ( x = maxPos.x - deltaPosX; x <= maxPos.x + deltaPosX; x+=1)
	
	if (x >= 0 && x < grid->sizeX) {
	  
	  gridPos.x = x;
	  
	  for ( y = maxPos.y - deltaPosY; y <= maxPos.y + deltaPosY; y+=1)
	    
	    if (y >= 0 && y < grid->sizeY) {
	      
	      gridPos.y = y;
	      tmpCell.prob = grid->prob[gridPos.rot][gridPos.x][gridPos.y];
	      
	      orgSum += grid->prob[gridPos.rot][gridPos.x][gridPos.y];
	      
	      if ( tmpCell.prob > grid->minimumProbability) {
		tmpCell.pos = realPositionOfGridPosition( gridPos, grid);
		insertRealLocalMaximum( tmpCell, &cells, MAX_NUMBER_OF_CELLS_IN_CUBE);
	      }
	    }
	}
    } 
    
    if ( globalActiveParameters.considerInactiveCells)
      cells.originalSumOfProbs = orgSum;
    else
      cells.originalSumOfProbs = 1.0;
    
    normalizeRealCellList( &cells);

    /*-----------------------------------------------------------------------------
     * Now compute the values for these cells.
     * ----------------------------------------------------------------------------*/
    if (globalActiveParameters.useMeasuredFeatures) {

      if ( globalActiveParameters.selectionMode == ENTROPY_SELECTION)
	computeEntropyDiff( &cells,
			    scan,
			    grid,
			    NULL,
			    entropyDiff,
			    globalActiveParameters.considerInactiveCells);
      else
	computeProbOfUnexpected( &cells,
				 scan,
				 grid,
				 entropyDiff,
				 globalActiveParameters.selectionMode,
				 globalActiveParameters.selectionThreshold);
    }
    else
      computeExpectedEntropyDiff( &cells,
				  scan,
				  grid,
				  NULL,
				  entropyDiff,
				  globalActiveParameters.considerInactiveCells);
    
    
    /* Count sensors above threshold. If we do aposteriori selection we don't want
     * to use max range readings. */
    if ( globalActiveParameters.useMeasuredFeatures) {

      int numberOfUnexpectedObstacles = 0;
      int numberOfExpectedObstacles = 0;
      int numberOfMaxRange = 0;
      
      for (num = 0; num < scan->numberOfSensors; num++) {
	
	/* Don't use max range. */
	if ( scan->sensor[num].measuredFeature ==
	     scan->sensor[num].numberOfFeatures -1) {
	  entropyDiff[num] = INFO_ONE_THRESHOLD - 0.001;
	  
	  numberOfMaxRange++;
	}
	
	else if ( entropyDiff[num] >= INFO_ONE_THRESHOLD) {

	  cnt++;

	  numberOfExpectedObstacles++;
	}
	else
	  numberOfUnexpectedObstacles++;
      }
      /* #define COUNT_UNEXPECTED */
#ifdef COUNT_UNEXPECTED
      writeLog( "%f %d %d %d %f #unexp\n", elapsedScriptTime,
		numberOfUnexpectedObstacles,
		numberOfExpectedObstacles, numberOfMaxRange,
		(float) numberOfUnexpectedObstacles / scan->numberOfSensors);
#endif
      
      displayInformation( entropyDiff, scan->numberOfSensors, INFO_ONE_THRESHOLD,
			  ALL_INFO_NUMBER+1);

      
      if (0) writeLog( "%d %f #cells\n",
		       cells.numberOfCells, grid->prob[maxPos.rot][maxPos.x][maxPos.y]);
      if (0) writeLog( "%d #above threshold (one)\n", cnt);
      
      if ( globalActiveParameters.minNumberOfSelectedSensors >= 0
	   &&
	   globalActiveParameters.maxNumberOfSelectedSensors > 0) {
	scan->numberOfSensorsToBeUsed =
	  iMax( globalActiveParameters.minNumberOfSelectedSensors,
		cnt);
	scan->numberOfSensorsToBeUsed =
	  iMin( globalActiveParameters.maxNumberOfSelectedSensors,
		scan->numberOfSensorsToBeUsed);
      }

#ifdef RANDOM_IF_NOT_ENOUGH
      if ( cnt < globalActiveParameters.minNumberOfSelectedSensors) {
	scan->numberOfSensorsToBeUsed =
	  2 * globalActiveParameters.minNumberOfSelectedSensors;
	setRandomScanMask( scan);
	fprintf( stderr, "Select %d random sensings.\n",
		 2 * globalActiveParameters.minNumberOfSelectedSensors);
	writeLog( "Select %d random sensings.\n",
		  2 * globalActiveParameters.minNumberOfSelectedSensors);
      }
      else 
#endif
	/* Set the scan mask. */
	setMaskAccordingToEvaluations( scan->mask, scan->numberOfSensors, 
				       entropyDiff, scan->numberOfSensorsToBeUsed, 
				       DESCENDING_ORDER);

    }
    else {
      if ( globalActiveParameters.minNumberOfSelectedSensors >= 0
	   &&
	   globalActiveParameters.maxNumberOfSelectedSensors > 0) {
	scan->numberOfSensorsToBeUsed =
	  iMax( globalActiveParameters.minNumberOfSelectedSensors,
		scan->numberOfSensorsToBeUsed);
	scan->numberOfSensorsToBeUsed =
	  iMin( globalActiveParameters.maxNumberOfSelectedSensors,
		scan->numberOfSensorsToBeUsed);
	
	if (0) writeLog("one: %d\n", scan->numberOfSensorsToBeUsed);
      }
      /* Set the scan mask. */
      setMaskAccordingToEvaluations( scan->mask, scan->numberOfSensors, 
				     entropyDiff, scan->numberOfSensorsToBeUsed, 
				     DESCENDING_ORDER);

    }
  }
}


static void
computeOptimalScanMaskSeveralMaxima( abstractSensorVector* scan,
				     realCellList* localMaxima,
				     positionProbabilityGrid* grid,
				     sampleSet* samples,
				     int useProbGrid)
{
  int cnt, num;
  double information[MAX_SIZE_OF_SCAN];
  realCellList* cells;

  /* When using probability grids we only use the local maxima of the belief state. */
  
  if ( useProbGrid) {
    
    cells = localMaxima;

    /* If the first two local maxima are too close to each other we treat
     * them as one. */
    if ( realPositionDistance ( localMaxima->cell[0].pos, 
				localMaxima->cell[1].pos) < 35.0
	 &&
	 angleDistance( localMaxima->cell[0].pos.rot,
			localMaxima->cell[0].pos.rot) < deg2Rad( 5.0)) {
      writeLog( "%f: Too close (%f %f %f: %f) (%f %f %f: %f). Treat as one.\n",
		elapsedScriptTime,
		localMaxima->cell[0].pos.x,
		localMaxima->cell[0].pos.y,
		rad2Deg(localMaxima->cell[0].pos.rot),
		localMaxima->cell[0].prob,
		localMaxima->cell[1].pos.x,
		localMaxima->cell[1].pos.y,
		rad2Deg(localMaxima->cell[1].pos.rot),
		localMaxima->cell[1].prob);
      computeOptimalScanMaskOneMaximum( scan, localMaxima, grid);
      return;
    }
  }
  /* Sample according to the distribution. */
  else {

#define NUM_SUB_SAMPLES 50
    static sampleSet subSample;
    static int firstTime = TRUE;
    static realCellList sampleCells;
    int s;
    
#define SUB_SAMPLE
#ifdef SUB_SAMPLE
    if (firstTime) {
      firstTime = FALSE;
      subSample.allocatedSamples = 0;
      subSample.desiredNumberOfSamples = NUM_SUB_SAMPLES;
    }
    
    if (1) {
      generateSample( samples, &subSample);
      
      sampleCells.numberOfCells = subSample.numberOfSamples;
      sampleCells.numberOfCellsInMap = subSample.numberOfSamples;
      
      for ( s = 0; s <  subSample.numberOfSamples; s++) {
	subSample.sample[s].weight = 1.0 / subSample.numberOfSamples;
      }
      
      for ( s = 0; s <  subSample.numberOfSamples; s++) {
	sampleCells.cell[s].pos = subSample.sample[s].pos;
	sampleCells.cell[s].prob = subSample.sample[s].weight;
	sampleCells.inMap[s] = TRUE;
      }
    }
    else {
      sampleCells.numberOfCells = 1;
      sampleCells.numberOfCellsInMap = 1;
      sampleCells.cell[0].pos = samples->mean;
      sampleCells.cell[0].prob = 1.0;
      sampleCells.inMap[0] = TRUE;
    }
#else
    int x, y, z;
    
    int numberOfCells = 0;
    fprintf(stderr, "MEAN %f %f %f\n", samples->mean.x, samples->mean.y, rad2Deg(samples->mean.rot));
    
    for ( x = -2; x < 3; x++) {
      for ( y = -2; y < 3; y++) {
	for ( z = -2; z < 3; z++) {
	  sampleCells.cell[numberOfCells].pos = samples->mean;
	  sampleCells.cell[numberOfCells].prob = 1.0 / 125;
	  sampleCells.inMap[numberOfCells] = TRUE;
	  sampleCells.cell[numberOfCells].pos.x += x * 5;
	  sampleCells.cell[numberOfCells].pos.y += y * 5;
	  sampleCells.cell[numberOfCells].pos.rot += deg2Rad(z * 5);
	  sampleCells.inMap[numberOfCells] = TRUE;
	  numberOfCells++;
	}
      }
    }
    
    sampleCells.numberOfCells = numberOfCells;
    sampleCells.numberOfCellsInMap = numberOfCells;
    
#endif
    cells = &sampleCells;
  }
  
  if ( globalActiveParameters.useMeasuredFeatures) {
    
    if ( globalActiveParameters.selectionMode == ENTROPY_SELECTION)
      computeEntropyDiff( cells,
			  scan,
			  grid,
			  NULL,
			  information,
			  globalActiveParameters.considerInactiveCells);
    else
      computeProbOfUnexpected( cells,
			       scan,
			       grid,
			       information,
			       globalActiveParameters.selectionMode,
			       globalActiveParameters.selectionThreshold);

    /* Count the sensors above threshold. This is only necessary if the
     * measurements are used. */

    cnt = 0;
    
    for (num = 0; num < scan->numberOfSensors; num++) {

      /* Don't use max range. */
      if ( scan->sensor[num].measuredFeature ==
	   scan->sensor[num].numberOfFeatures -1) {

	
	information[num] = INFO_SEVERAL_THRESHOLD - 0.001;
      }
	
      else if ( information[num] > INFO_SEVERAL_THRESHOLD) {
	cnt++;
      }
    }
    writeLog( "%d #above threshold (several)\n", cnt);
    
    if ( globalActiveParameters.minNumberOfSelectedSensors >= 0
	 &&
	 globalActiveParameters.maxNumberOfSelectedSensors > 0) {
      
      scan->numberOfSensorsToBeUsed =
	iMax( globalActiveParameters.minNumberOfSelectedSensors,
	      cnt);
      scan->numberOfSensorsToBeUsed =
	iMin( globalActiveParameters.maxNumberOfSelectedSensors,
	      scan->numberOfSensorsToBeUsed);
    }
  }
  else {
    
    computeExpectedEntropyDiff( cells,
				scan,
				grid,
				NULL,
				information,
				globalActiveParameters.considerInactiveCells);
    
    if ( globalActiveParameters.minNumberOfSelectedSensors >= 0
	 &&
	 globalActiveParameters.maxNumberOfSelectedSensors > 0) {
      scan->numberOfSensorsToBeUsed =
	iMax( globalActiveParameters.minNumberOfSelectedSensors,
	      scan->numberOfSensorsToBeUsed);
      scan->numberOfSensorsToBeUsed =
	iMin( globalActiveParameters.maxNumberOfSelectedSensors,
	      scan->numberOfSensorsToBeUsed);
    }
    if (1) writeLog("several: %d\n", scan->numberOfSensorsToBeUsed);
  }
  
  displayInformation( information, scan->numberOfSensors, INFO_SEVERAL_THRESHOLD,
		      ALL_INFO_NUMBER);

  /* Set the scan mask. */
  setMaskAccordingToEvaluations( scan->mask, scan->numberOfSensors, 
				 information, scan->numberOfSensorsToBeUsed, 
				 DESCENDING_ORDER);
}

static void
displayInformation( double *information, int numberOfInformation,
		    double referenceValue, int windowNumber)
{

  static bool windowCnt = 0;
  static gridWindow* win[MAX_NUMBER_OF_INFO_WINDOWS];
  static angleProbTable probTab[MAX_NUMBER_OF_INFO_WINDOWS];
  
  int i;

  if (1) return;
  
  if ( windowCnt == 0) {
    for ( i = 0; i < MAX_NUMBER_OF_INFO_WINDOWS; i++) {
      win[i] = NULL;
    }
  }

  if ( win[windowNumber] == NULL) {
    probTab[windowNumber].numberOfAngles = MAX_SIZE_OF_SCAN;
    probTab[windowNumber].prob = (probability*)	
      malloc( MAX_SIZE_OF_SCAN * sizeof(probability));
    win[windowNumber] = createAngleWindow( &(probTab[windowNumber]),
					   "Entropy histogram", 0, 400+ windowCnt*100);
    windowCnt++;
  }

  probTab[windowNumber].numberOfAngles = numberOfInformation;
  for (i = 0; i<numberOfInformation; i++)
    probTab[windowNumber].prob[i] = (probability) information[i];
    
  /* Display the window and set the reference value. */
  displayAngleWindow(&(probTab[windowNumber]), win[windowNumber], referenceValue);
}


