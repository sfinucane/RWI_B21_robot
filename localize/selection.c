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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/selection.c,v $
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
 * $Log: selection.c,v $
 * Revision 1.1  2002/09/14 20:45:40  rstone
 * *** empty log message ***
 *
 * Revision 1.16  2000/10/25 18:56:15  fox
 * At least human selection (MODE 2) works with the samples again.
 *
 * Revision 1.15  1999/03/08 16:47:47  wolfram
 * Fixed problems with map, caused by the fact that it no longer accpets
 * correction parameters from LOCALIZE.  Improved the handling of sonars
 * with Pioneers.  Currently sonars do not work properly on a B21.
 *
 * Revision 1.14  1999/01/22 17:48:11  fox
 * Final version of sampling paper and denver demo.
 *
 * Revision 1.13  1998/11/23 21:19:27  fox
 * Fixed some minor bugs.
 *
 * Revision 1.12  1998/11/17 23:26:27  fox
 * Incorporated the ability to localize the robot in maps that are changed
 * on the fly. Had to make several changes.
 * To perform online localization add command line parameter -mapping.
 * If this option is used, all maps will be set to the online map received
 * from the MAP module.
 *
 * Revision 1.11  1998/11/03 21:02:22  fox
 * Lasers will not be integrated if the robot rotates faster than
 * MAX_ROT_VEL_FOR_LASERS deg/sec.
 *
 * Revision 1.10  1998/10/19 18:29:57  fox
 * *** empty log message ***
 *
 * Revision 1.9  1998/08/19 16:33:58  fox
 * Minor changes. Position is dumped upon exit of tcxServer.
 *
 * Revision 1.8  1998/06/12 10:16:44  fox
 * Implemented virutal sensor.
 *
 * Revision 1.7  1998/01/29 16:46:53  fox
 * Removed some hacks.
 *
 * Revision 1.6  1998/01/22 13:06:23  fox
 * First version after selection-submission.
 *
 * Revision 1.5  1998/01/08 12:19:48  wolfram
 * Minor changes for Sebastians new maps
 *
 * Revision 1.4  1998/01/06 15:11:22  fox
 * Added evaluation tools.
 *
 * Revision 1.3  1998/01/05 10:37:16  fox
 * Several changes especially in selection parts.
 *
 * Revision 1.2  1997/12/11 17:06:32  fox
 * Added some parameters.
 *
 * Revision 1.1  1997/11/25 17:13:05  fox
 * Should work.
 *
 * Revision 1.13  1997/11/20 12:58:11  fox
 * Version with good sensor selection.
 *
 * Revision 1.12  1997/11/07 12:39:40  fox
 * Added some graphic features.
 *
 * Revision 1.11  1997/10/31 13:14:36  fox
 * Removed active sensing comments.
 *
 * Revision 1.9  1997/09/11 20:01:04  fox
 * Final cmu version.
 *
 * Revision 1.8  1997/09/09 19:45:12  fox
 * Fixed a bug in finding local maxima and did several changes to active
 * localization.
 *
 * Revision 1.7  1997/08/22 04:16:38  fox
 * Final version before IJCAI.
 *
 * Revision 1.6  1997/08/19 18:18:45  fox
 * Last version before I change from information to entropy.
 *
 * Revision 1.5  1997/07/04 17:29:14  fox
 * Final version before holiday!!!
 *
 * Revision 1.4  1997/06/25 14:16:39  fox
 * Changed laser incorporation.
 *
 * Revision 1.3  1997/06/20 07:36:09  fox
 * Intermediate test version before changing information.
 *
 * Revision 1.2  1997/06/03 11:49:21  fox
 * Museum version.
 *
 * Revision 1.1  1997/05/26 09:34:12  fox
 * Replaced entropy by information.
 *
 * Revision 1.8  1997/04/30 12:25:39  fox
 * Some minor changes.
 *
 * Revision 1.7  1997/04/03 13:17:50  fox
 * Some minor changes.
 *
 * Revision 1.6  1997/03/19 17:52:41  fox
 * New laser parameters.
 *
 * Revision 1.5  1997/03/18 18:45:28  fox
 * First executable version with a global scan mask for different sensor types.
 *
 * Revision 1.4  1997/03/17 18:41:12  fox
 * First version with sensor selection in cluttered environments.
 *
 * Revision 1.3  1997/03/14 17:58:17  fox
 * This version should run quite stable now.
 *
 * Revision 1.2  1997/03/13 17:36:20  fox
 * Temporary version. Don't use!
 *
 * Revision 1.1  1997/01/29 12:23:04  fox
 * First version of restructured LOCALIZE.
 *
 * Revision 1.21  1997/01/19 19:31:15  fox
 * yeah
 *
 * Revision 1.20  1997/01/19 18:56:55  wolfram
 * Again a bug ...
 *
 * Revision 1.19  1997/01/18 19:41:02  fox
 * Improved action selection.
 *
 * Revision 1.18  1997/01/16 19:43:20  fox
 * And another bug ...
 *
 * Revision 1.17  1997/01/13 16:54:12  fox
 * Nothing special.
 *
 * Revision 1.16  1997/01/10 15:19:21  fox
 * Improved several methods.
 *
 * Revision 1.15  1997/01/08 15:52:54  fox
 * Rotations are not any longer treated like movements.
 * Send parameters to PLAN.
 *
 * Revision 1.14  1996/12/20 15:29:35  fox
 * Added four parameters.
 *
 * Revision 1.13  1996/12/04 14:29:59  fox
 * ok
 *
 * Revision 1.12  1996/12/03 15:40:24  fox
 * ok
 *
 * Revision 1.11  1996/12/03 12:27:38  fox
 * Seems to work with the thin corridor.
 *
 * Revision 1.10  1996/12/02 18:46:25  fox
 * First version with the new expected distances.
 *
 * Revision 1.9  1996/12/02 10:32:00  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.8  1996/11/29 09:31:38  fox
 * ok
 *
 * Revision 1.7  1996/11/28 17:56:20  fox
 * *** empty log message ***
 *
 * Revision 1.6  1996/11/26 16:08:25  fox
 * Nothing special.
 *
 * Revision 1.5  1996/11/26 11:08:10  fox
 * Improved version.
 *
 * Revision 1.4  1996/11/25 19:35:39  fox
 * Test version for decisions of movements.
 *
 * Revision 1.3  1996/11/21 14:28:50  fox
 * Decides which action to perform next.
 *
 * Revision 1.2  1996/11/21 13:41:55  fox
 * First version to show the main structure to find the best movement of the
 * robot.
 *
 * Revision 1.1  1996/11/21 12:40:24  fox
 * Tools for bayesian reasoning on the grids.
 *
 *
 *
 *
 *
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/


#include "general.h"
#include "selection.h"
#include "movement.h"
#include "abstract.h"
#include "sensings.h"
#include "proximity.h"
#include "proximityTools.h"
#include "laser.h"
#include "allocate.h"

#define POSITION_OCCUPIED_THRESHOLD 0.8


static void
computeSensorPositions( realCellList* cellList,
			abstractSensorVector* sensors,
			positionProbabilityGrid* grid,
			probabilityGrid* occupancyProbs,
			float pOfPosition[MAX_NUMBER_OF_CELLS],
			bool considerPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS],
			gridPosition sensorPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS],
			bool considerInactive);

static void
computeSensorFeatures( realCellList* cellList,
		       abstractSensorType* sensor,
		       int sensorNumber,
		       float pOfPosition[MAX_NUMBER_OF_CELLS],
		       bool considerPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS],
		       gridPosition sensorPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS],		       
		       float pOfFeature[MAX_NUMBER_OF_FEATURES],
		       float pOfFeatureGivenPosition[MAX_NUMBER_OF_FEATURES][MAX_NUMBER_OF_CELLS]);

static gridPosition
sensorPositionOfRealPosition( realPosition pos, positionProbabilityGrid* grid);

static void
setUnNormedFeatureProbs( abstractSensorVector* scan);

static void
setNormedFeatureProbs( abstractSensorVector* scan);

static float
computeEntropy( float aprioriProbs[MAX_NUMBER_OF_CELLS],
		int numberOfActiveCells,
		int numberOfInactiveCells);

static distProbTable
createUnexpectedProbFunctionTable( abstractSensorType sensor, int mode);

/***********************************************************************
 * Computes the difference between the current entropy and the expected
 * entropy given the sensors. Also considers that the inactive cells
 * are not updated. Probabilities have to be centered around 1.0!!!!
 ***********************************************************************/
double
computeExpectedEntropyDiff( realCellList* cellList,
			    abstractSensorVector* sensors,
			    positionProbabilityGrid* grid,
			    probabilityGrid* occupancyProbs,
			    double* entropyDiff,
			    bool considerInactive)
{
  double averageDiff = 0.0;
  
  int sensorNumber; 
  int i;           /* index for the positions */
  int feature;     /* different features of the sensor */

  float pOfPosition[MAX_NUMBER_OF_CELLS]; 

  /* probability of a feature given the probabilities in the cell list */
  float pOfFeature[MAX_NUMBER_OF_FEATURES]; 
  /* probability of a feature given that the robot is in a cell */
  float pOfFeatureGivenPosition[MAX_NUMBER_OF_FEATURES][MAX_NUMBER_OF_CELLS]; 

  /* Check wether the position has to be considered. */
  bool considerPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS];

  /* grid position of the real position. */
  gridPosition sensorPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS];

  /* We must consider the inactive cells as well. */
  probability aprioriEntropy, aposterioriEnt;
  probability aposterioriProb[MAX_NUMBER_OF_CELLS];

  int numberOfInactiveCells;

  /*-------------------------------------------------------------- 
   * Use the original sum of the local maxima and consider the update
   * on the inactive cells as well.
   *--------------------------------------------------------------*/
  if ( considerInactive) {
    numberOfInactiveCells =
      grid->sizeX * grid->sizeY * grid->sizeZ - cellList->numberOfCells;
    setUnNormedFeatureProbs( sensors);
  }
  else {
    numberOfInactiveCells = 0;
    setNormedFeatureProbs( sensors);
  }
  
  /*-------------------------------------------------------------- 
   * Compute the grid positions of the sensors and check whether the
   * sensor has to be considered for the entropyDiff.
   *--------------------------------------------------------------*/
  computeSensorPositions( cellList,
			  sensors,
			  grid,
			  occupancyProbs,
			  pOfPosition,
			  considerPosition,
			  sensorPosition,
			  considerInactive);

  /*--------------------------------------------------------------
   * First compute the aPriori entropy under consideration of inactive cells. 
   *--------------------------------------------------------------*/
  aprioriEntropy = computeEntropy( pOfPosition,
				   cellList->numberOfCells,
				   numberOfInactiveCells);

  /*--------------------------------------------------------------
   * We compute for each sensor independently the mutual entropyDiff.
   *--------------------------------------------------------------*/
  for ( sensorNumber = 0; sensorNumber < sensors->numberOfSensors; sensorNumber++) {

    int numberOfFeatures = sensors->sensor[sensorNumber].numberOfFeatures;
    
    probability summedFeatureProb = 0.0;
    
    /* If we consider inactive cells then the probability of the features does
       not sum up to one but is 1 * numberOfFeatures in the average. */
    if ( considerInactive)
      entropyDiff[sensorNumber] = aprioriEntropy * numberOfFeatures;
    else
      entropyDiff[sensorNumber] = aprioriEntropy;      
    
    /*-------------------------------------------------------------- 
     * Compute the probabilities of the different features.
     *--------------------------------------------------------------*/
    computeSensorFeatures( cellList,
			   &(sensors->sensor[sensorNumber]),
			   sensorNumber,
			   pOfPosition,
			   considerPosition,
			   sensorPosition,
			   pOfFeature,
			   pOfFeatureGivenPosition);
    

    /* Simply integrate this feature. */
    for ( feature = 0; feature < numberOfFeatures; feature++) {
      
      summedFeatureProb += pOfFeature[feature];
      
      /* Compute the new probabilities after having integrated the
       * measured feature. */
      for ( i = 0; i < cellList->numberOfCells; i++) {
	aposterioriProb[i] =
	  pOfPosition[i] * pOfFeatureGivenPosition[feature][i] / pOfFeature[feature];
      }

      /* Now compute the normalize factor after integration and normalize the values. */
      aposterioriEnt = computeEntropy( aposterioriProb,
				       cellList->numberOfCells,
				       numberOfInactiveCells);
      
      entropyDiff[sensorNumber] -= pOfFeature[feature] * aposterioriEnt; 
    }
    averageDiff += entropyDiff[sensorNumber];
  }
  return averageDiff;
}


/***********************************************************************
 * Computes the difference between the current entropy and the expected
 * entropy given the measurements of the sensors. 
 ***********************************************************************/
void
computeEntropyDiff( realCellList* cellList,
		    abstractSensorVector* sensors,
		    positionProbabilityGrid* grid,
		    probabilityGrid* occupancyProbs,
		    double* entropyDiff,
		    bool considerInactive)
{
  int sensorNumber; 
  int i;           /* index for the positions */

  float pOfPosition[MAX_NUMBER_OF_CELLS]; 

  /* Check wether the position has to be considered. */
  bool considerPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS];

  /* grid position of the real position. */
  gridPosition sensorPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS];

  /* We must consider the inactive cells as well. */
  probability aprioriEntropy, aposterioriEnt;
  probability aposterioriProb[MAX_NUMBER_OF_CELLS];

  int numberOfInactiveCells;

  /*-------------------------------------------------------------- 
   * Use the original sum of the local maxima and consider the update
   * on the inactive cells as well.
   *--------------------------------------------------------------*/
  if ( considerInactive) {
    numberOfInactiveCells =
      grid->sizeX * grid->sizeY * grid->sizeZ - cellList->numberOfCells;
    setUnNormedFeatureProbs( sensors);
  }
  else {
    numberOfInactiveCells = 0;
    setNormedFeatureProbs( sensors);
  }
  
  /*-------------------------------------------------------------- 
   * Compute the grid positions of the sensors and check whether the
   * sensor has to be considered for the entropyDiff.
   *--------------------------------------------------------------*/
  computeSensorPositions( cellList,
			  sensors,
			  grid,
			  occupancyProbs,
			  pOfPosition,
			  considerPosition,
			  sensorPosition,
			  considerInactive);

  /*--------------------------------------------------------------
   * First compute the aPriori entropy under consideration of inactive cells. 
   *--------------------------------------------------------------*/
  aprioriEntropy = computeEntropy( pOfPosition,
				   cellList->numberOfCells,
				   numberOfInactiveCells);

#ifdef TEST
  {
    float sum = 0.0;
    for ( i = 0; i < cellList->numberOfCells; i++) {
      writeLog( "%d -- %f %f %f %f %d %d %d\n", i,
		cellList->cell[i].prob,
		cellList->cell[i].pos.x,
		cellList->cell[i].pos.y,
		cellList->cell[i].pos.rot,
		sensorPosition[0][i].x,
		sensorPosition[0][i].y,
		sensorPosition[0][i].rot);
      sum += cellList->cell[i].prob;
    }
    
    writeLog( "%d %f %f #celllist\n", cellList->numberOfCells,
	      sum, cellList->originalSumOfProbs);
  }
#endif
  
  /*--------------------------------------------------------------
   * We compute for each sensor independently the entropyDiff.
   *--------------------------------------------------------------*/
  for ( sensorNumber = 0; sensorNumber < sensors->numberOfSensors; sensorNumber++) {

      abstractSensorType* sensor = &(sensors->sensor[sensorNumber]);
      probability sumOfPositionProbs = 0.0;    
      probability pOfFeature = 0.0;
      
      /* Compute the new probabilities after having integrated the
       * measured feature. P(l) <-- p(s|l) * p(l) / p(s)
       * p(s) is computed in the loop and multiplied afterwards. */
      for ( i = 0; i < cellList->numberOfCells; i++) {
	
	sumOfPositionProbs += pOfPosition[i];
	
	if ( considerPosition[sensorNumber][i]) {
	  
	  aposterioriProb[i] = pOfPosition[i] *
	    sensor->probOfFeatureGivenPosition( sensor->measuredFeature,
						sensorPosition[sensorNumber][i],
						sensor->infoForFeatures);
	}
	else {
	  aposterioriProb[i] = pOfPosition[i] *
	    sensor->uninformedFeatureProbability
	    [sensor->expectedFeature( sensorPosition[sensorNumber][i],
				      sensor->infoForFeatures).quality];
	}
	pOfFeature += aposterioriProb[i];
      }

      /* Now add the inactive cells (p(s|incative cell) is assumed to be 1.0!!!). */
      pOfFeature += (1.0 - sumOfPositionProbs);

      /* Divide p(s) */
      for ( i = 0; i < cellList->numberOfCells; i++) {
	aposterioriProb[i] /= pOfFeature;
      }

      /* Now compute the new entropy. */
      aposterioriEnt = computeEntropy( aposterioriProb,
				       cellList->numberOfCells,
				       numberOfInactiveCells);
      
      entropyDiff[sensorNumber] = aprioriEntropy - aposterioriEnt;
  }
}


/***********************************************************************
 * Computes the difference between the current entropy and the expected
 * entropy given the measurements of the sensors. 
 ***********************************************************************/
void
computeProbOfUnexpected( realCellList* cellList,
			 abstractSensorVector* sensors,
			 positionProbabilityGrid* grid,
			 double* probOfUnexpected,
			 int mode,
			 float threshold)
{
  int sensorNumber; 
  int i;           /* index for the positions */
  static bool firstTime = TRUE;
  
  float pOfPosition[MAX_NUMBER_OF_CELLS]; 

  /* Check wether the position has to be considered. */
  bool considerPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS];

  /* grid position of the real position. */
  gridPosition sensorPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS];

  /*********************************************************
   * Probability table for unexpected obstacles of laser.
   *********************************************************/
  static distProbTable unexpectedProbFunctionTable;

  if ( firstTime) {
    
    if (  sensors->sensor[0].type != FRONT_LASER_TYPE
	  &&
	  sensors->sensor[0].type != REAR_LASER_TYPE) {
      fprintf(stderr, "Sorry. This kind of selection only allowed for laser.\n");
      return;
    }

    else {
      unexpectedProbFunctionTable =
	createUnexpectedProbFunctionTable( sensors->sensor[0], mode);

      firstTime = FALSE;
    }
  }

  /*-------------------------------------------------------------- 
   * Compute the grid positions of the sensors and check whether the
   * sensor has to be considered for the entropyDiff.
   *--------------------------------------------------------------*/
  computeSensorPositions( cellList,
			  sensors,
			  grid,
			  NULL,
			  pOfPosition,
			  considerPosition,
			  sensorPosition,
			  FALSE);

  
  /*--------------------------------------------------------------
   * We compute for each sensor independently the probability of
   * entropyDiff.
   *--------------------------------------------------------------*/
  for ( sensorNumber = 0; sensorNumber < sensors->numberOfSensors; sensorNumber++) {

    abstractSensorType* sensor = &(sensors->sensor[sensorNumber]);

    infoForProximityFeature* info = (infoForProximityFeature*)
      sensor->infoForFeatures;
    
    probOfUnexpected[sensorNumber] = threshold;
    probOfUnexpected[sensorNumber] = 0.0;

    for ( i = 0; i < cellList->numberOfCells; i++) {

      if ( considerPosition[sensorNumber][i]) {

	probOfUnexpected[sensorNumber] += pOfPosition[i] *
	  distProb_GivenPositionAndRot( sensor->measuredFeature,
					sensorPosition[sensorNumber][i],
					info->sensorRot,
					info->grid,
					info->distTab,
					&unexpectedProbFunctionTable);
      }
    }
  }
}


/*-------------------------------------------------------------- 
 * Computes the grid positions of the sensors and checks whether the
 * sensor has to be considered for the information.
 *--------------------------------------------------------------*/
static void
computeSensorPositions(  realCellList* cellList,
			 abstractSensorVector* sensors,
			 positionProbabilityGrid* grid,
			 probabilityGrid* occupancyProbs,
			 float pOfPosition[MAX_NUMBER_OF_CELLS],
			 bool considerPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS],
			 gridPosition sensorPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS],
			 bool considerInactive)
{
  int i, sensorNumber;
  
  /*-------------------------------------------------------------- 
   * Compute the grid positions of the sensors and check whether the
   * sensor has to be considered for the information.
   *--------------------------------------------------------------*/
  for ( i = 0; i < cellList->numberOfCells; i++) {

    if ( considerInactive)
      pOfPosition[i] = cellList->cell[i].prob * cellList->originalSumOfProbs;
    else
      pOfPosition[i] = cellList->cell[i].prob;
    
    for ( sensorNumber = 0; sensorNumber < sensors->numberOfSensors; sensorNumber++) {
      /* For each sensor and each position we must get the sensor position in
       * grid coordinates and check wether the sensor should be considered. */
      if ( ! cellList->inMap[i]) { 
	considerPosition[sensorNumber][i] = FALSE;
      }
      else if ( occupancyProbs != NULL &&
		( probOfRealCoordinate( cellList->cell[i].pos.x,
					cellList->cell[i].pos.y,
					occupancyProbs) 
		  > POSITION_OCCUPIED_THRESHOLD)) { 
	considerPosition[sensorNumber][i] = FALSE;
      }
      else { /* The cell is in the map and it is likely enough. */	
	
	if ( sensors->sensorOffsetType != NO_OFFSET) {
	  
	  realPosition sensPos =
	    endPoint( cellList->cell[i].pos,
		      sensors->sensor[sensorNumber].integrationInfo.sensorOffset);
	  
	  sensorPosition[sensorNumber][i] = sensorPositionOfRealPosition( sensPos, grid);

	  /* The position of the sensor might be outside the grid. Check this! */
	  considerPosition[sensorNumber][i] =
	    coordinateInGrid( sensorPosition[sensorNumber][i].x,
			      sensorPosition[sensorNumber][i].y,
			      grid);
	}
	else { /* No shift of the sensor necessary. */
	  
	  sensorPosition[sensorNumber][i] =
	    sensorPositionOfRealPosition( cellList->cell[i].pos, grid);
	  
	  /* Yes! consider it! */
	  considerPosition[sensorNumber][i] = 
	    coordinateInGrid( sensorPosition[sensorNumber][i].x,
			      sensorPosition[sensorNumber][i].y,
			      grid);
	}
      }
    }
  }
}


static void
computeSensorFeatures( realCellList* cellList,
		       abstractSensorType* sensor,
		       int sensorNumber,
		       float pOfPosition[MAX_NUMBER_OF_CELLS],
		       bool considerPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS],
		       gridPosition sensorPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS],		       
		       float pOfFeature[MAX_NUMBER_OF_FEATURES],
		       float pOfFeatureGivenPosition[MAX_NUMBER_OF_FEATURES][MAX_NUMBER_OF_CELLS])
{
  int feature, i;
  int bestQuality = 1000;
  int worstQuality = -1000;
  
  /* This is mostly to determine the expected feature and its quality. */
  featureStruct fStruct[MAX_NUMBER_OF_CELLS];

  probability sumOfPositionProbs = 0.0;
  
  /* Initialize the feature pobability. */
  for ( feature = 0; feature < sensor->numberOfFeatures; feature++) 
    pOfFeature[feature] = 0.0;
  
  /*--------------------------------------------------------------
   * Compute the expected features and their qualities.
   *--------------------------------------------------------------*/
  for ( i = 0; i < cellList->numberOfCells; i++) {
	
    if ( considerPosition[sensorNumber][i]) {

      fStruct[i] = sensor->expectedFeature( sensorPosition[sensorNumber][i],
					    sensor->infoForFeatures);

      if ( fStruct[i].quality > worstQuality) 
	worstQuality = fStruct[i].quality;
      if ( fStruct[i].quality < bestQuality) 
	bestQuality = fStruct[i].quality;
    }
  }

  /* Use the minimal quality. This only works for proximity sensors!!! */
  if ( bestQuality != worstQuality) {

    for ( i = 0; i < cellList->numberOfCells; i++) {
      if ( considerPosition[sensorNumber][i]) {
	if ( fStruct[i].quality < worstQuality) {
	  setQuality( &(fStruct[i]), worstQuality);
	}
      }
    }
  }
  
  /*--------------------------------------------------------------
   * Compute the probabilities of the features.
   *--------------------------------------------------------------*/
  for ( i = 0; i < cellList->numberOfCells; i++) {

    sumOfPositionProbs += pOfPosition[i];
    
    if ( considerPosition[sensorNumber][i]) {
      
      for ( feature = 0; feature < sensor->numberOfFeatures; feature++) {
	
	pOfFeatureGivenPosition[feature][i] =
	  sensor->probOfFeatureGivenExpected( feature,
					      &(fStruct[i]),
					      sensor->infoForFeatures);

	pOfFeature[feature] += pOfFeatureGivenPosition[feature][i] * pOfPosition[i];
      }
    }
    else {
      
      for ( feature = 0; feature < sensor->numberOfFeatures; feature++) {
	
	pOfFeatureGivenPosition[feature][i] = sensor->uninformedFeatureProbability[worstQuality];
	
	pOfFeature[feature] += pOfFeatureGivenPosition[feature][i] * pOfPosition[i];
      }
    }    
  }
  
  /* Now add the inactive cells (p(s|incative cell) is assumed to be 1.0!!!). */
  for ( feature = 0; feature < sensor->numberOfFeatures; feature++) 
    pOfFeature[feature] += (1.0 - sumOfPositionProbs);
}
  

static gridPosition
sensorPositionOfRealPosition( realPosition pos, positionProbabilityGrid* grid)
{
  gridPosition gridPos;
  int plane = planeOfRotation( pos.rot, grid);

  gridPos.x = ( pos.x - grid->offsetX) / grid->positionResolution;
  gridPos.y = ( pos.y - grid->offsetY) / grid->positionResolution;
  gridPos.rot = plane;

  return gridPos;
}


static void
setUnNormedFeatureProbs( abstractSensorVector* scan)
{
  int num;
  for (num = 0; num < scan->numberOfSensors; num++) 
    scan->sensor[num].setUnNormedFeatureProbs( scan->sensor[num].infoForFeatures,
					       &(scan->sensor[num].uninformedFeatureProbability));
}

static void
setNormedFeatureProbs( abstractSensorVector* scan)
{
  int num;
  for (num = 0; num < scan->numberOfSensors; num++) 
    scan->sensor[num].setNormedFeatureProbs( scan->sensor[num].infoForFeatures,
					     &(scan->sensor[num].uninformedFeatureProbability));
}

static float
computeEntropy( float prob[MAX_NUMBER_OF_CELLS],
		int numberOfActiveCells,
		int numberOfInactiveCells)
{
  int i;
  float entropy = 0.0;
  float sumOfActive = 0.0;
  
  for ( i = 0; i < numberOfActiveCells; i++) {
    entropy -= prob[i] * log( prob[i]);
    sumOfActive += prob[i];
  }
  /* Add the entropy of the incative cells. */
  if ( numberOfInactiveCells > 0 && sumOfActive < 1.0) {
    float sumOfInactive = 1.0 - sumOfActive;
    if ( sumOfInactive != 0.0)
      entropy -= sumOfInactive *
	log ( sumOfInactive / numberOfInactiveCells);
  }
  
  return entropy;
}

static float
computeAposterioriEntropy( float aposterioriProb[MAX_NUMBER_OF_CELLS],
			   int numberOfActiveCells,
			   int numberOfInactiveCells,
			   float aprioriSumOfActive,
			   float aposterioriSumOfActive)
{
  int i;
  float aprioriSumOfInactive = 1.0 - aprioriSumOfActive;
  float normalizeFactor = aposterioriSumOfActive + aprioriSumOfInactive;
  float aposterioriEnt = 0.0;

  aposterioriSumOfActive /= normalizeFactor;

  for ( i = 0; i < numberOfActiveCells; i++) {
    if (0) aposterioriProb[i] /= normalizeFactor;
    if ( aposterioriProb[i] > 0.0) 
      aposterioriEnt -= aposterioriProb[i] * log( aposterioriProb[i]);
  }

  /* Now add the entropy of the inactive cells. */
  if ( numberOfInactiveCells > 0) {

    float aposterioriSumOfInactive = 1.0 - aposterioriSumOfActive;

    if ( aposterioriSumOfInactive > 0.0)
      aposterioriEnt -= aposterioriSumOfInactive *
	log ( aposterioriSumOfInactive / numberOfInactiveCells);
  }

  return aposterioriEnt;
}



/*-------------------------------------------------------------- 
 * Cretes a prob function table for unexpected obstacles.
 *--------------------------------------------------------------*/
static distProbTable
createUnexpectedProbFunctionTable( abstractSensorType sensor, int mode)
{
  distProbTable table;

  infoForProximityFeature* info = (infoForProximityFeature*)
    sensor.infoForFeatures;
  expectedDistTable* expDist = info->distTab;
  char* fileName;
  
  int measuredCnt, expectedCnt, stdDevIndex, expectedIndex;
  FILE* fp;
  char line[MAX_STRING_LENGTH];
  int error;
  float tmp;
    
/* #define GENERATE	1 */
#ifdef GENERATE
  float stdDev = 5.0;
  float probFactor = 1.0 / gauss( 0, stdDev, 0);
  float maximumProbClose = 1.0;
  float maximumProbFar = 0.0;
  
#endif

  /* Create the table. */
  if ( mode == HUMAN_SELECTION)
    fileName = "../params/laser-human";
  else
    fileName = "laser-unexpected";

  fp = fopen(fileName, "rt");
  
  table.distanceResolution        = expDist->distanceResolution;
  table.numberOfExpectedDistances =
    table.numberOfMeasuredDistances = expDist->numberOfExpectedDistances;
  table.numberOfStdDevs = expDist->numberOfStdDevs;

  /* open file and read values */
  if (fp != NULL) {
      
    writeLog( "# Reading %s ...", fileName);

    /* read the first comment line */
    error = (fgets(line,MAX_STRING_LENGTH,fp) == NULL);
    if (!error && sscanf(&line[1], "%d %d", &expectedCnt, &measuredCnt) == 2){
      if (expectedCnt != table.numberOfExpectedDistances ||
	  measuredCnt != table.numberOfMeasuredDistances){
	fprintf(stderr, "# Wrong dimension in  %s\n", fileName);
	fclose(fp);
	exit(0);
      }
    }
      
    
    /***********************************************************************
     * Allocate memory.
     ***********************************************************************/
    table.prob = (float**)
      allocate2D( table.numberOfExpectedDistances * table.numberOfStdDevs,
		  table.numberOfMeasuredDistances,
		  FLOAT);

    /***********************************************************************
     * Fill in the probabilities.
     ***********************************************************************/
    for ( measuredCnt = 0;
	  measuredCnt < table.numberOfMeasuredDistances; measuredCnt++){
      
      for ( expectedCnt = 0;
	    expectedCnt < table.numberOfExpectedDistances; expectedCnt++) {

#ifdef GENERATE
	for (stdDevIndex = 0; stdDevIndex < table.numberOfStdDevs; stdDevIndex++){
	  expectedIndex = expectedCnt*table.numberOfStdDevs + stdDevIndex;
	  if ( measuredCnt <= expectedCnt)    
	    table.prob[expectedIndex][measuredCnt] =
	      fMin( maximumProbClose,
		    maximumProbClose -
		    probFactor * gauss( measuredCnt, stdDev, expectedCnt));
	  else
	    table.prob[expectedIndex][measuredCnt] = maximumProbFar;
	}
#endif
	  if ( (fgets(line, MAX_STRING_LENGTH,fp) != NULL) &&
	       sscanf(line, "%e", &tmp) == 1) {

	    for (stdDevIndex = 0; stdDevIndex < table.numberOfStdDevs; stdDevIndex++){
	      expectedIndex = expectedCnt*table.numberOfStdDevs + stdDevIndex;
	      table.prob[expectedIndex][measuredCnt] = tmp;
	    }
	  }
	  else{
	    fprintf(stderr, "%d %d -> %f\n", expectedCnt, measuredCnt, tmp);
	    fprintf( stderr, "Error: reading %s\n", fileName);
	    fclose(fp);
	    exit (0);
	  }
	}
	if (measuredCnt < table.numberOfMeasuredDistances - 1)
	  fgets(line,MAX_STRING_LENGTH,fp);
      }


#ifdef GENERATE    
      /* MAX_RANGE is expected. */
      for ( measuredCnt = 0;
	    measuredCnt < table.numberOfMeasuredDistances; measuredCnt++){
      
	for (stdDevIndex = 0; stdDevIndex < table.numberOfStdDevs; stdDevIndex++){
	  expectedIndex = (table.numberOfMeasuredDistances-1)*table.numberOfStdDevs + stdDevIndex;
	  
	  table.prob[expectedIndex][measuredCnt] = maximumProbClose;
	}
      }
#endif
  }
  else {
    fprintf( stderr, "Need file %s in parameter directory.\n", fileName);
    exit(0);
  }

  if (0) dumpDistTable( table, "select");
  
  return table;
}

  
