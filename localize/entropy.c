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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/entropy.c,v $
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
 * $Log: entropy.c,v $
 * Revision 1.1  2002/09/14 20:45:39  rstone
 * *** empty log message ***
 *
 * Revision 1.9  1997/05/26 08:47:44  fox
 * Last version before major changes.
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
#include "entropy.h"
#include "movement.h"
#include "abstract.h"
#include "proximity.h"
#include "proximityTools.h"

#define POSITION_OCCUPIED_THRESHOLD 0.8

static double
computeAPosterioriEntropiesAveraged( realCellList* cellList,
				     abstractSensorVector* sensors,
				     positionProbabilityGrid* grid,
				     probabilityGrid* occupancyProbs,
				     double* entropies);
static void
computeAPosterioriEntropiesOfMeasured( realCellList* cellList,
				       abstractSensorVector* sensors,
				       positionProbabilityGrid* grid,
				       probabilityGrid* occupancyProbs,
				       double* entropies);


double
computeAPosterioriEntropies( realCellList* cellList,
			     abstractSensorVector* sensors,
			     positionProbabilityGrid* grid,
			     probabilityGrid* occupancyProbs,
			     double* entropies,
			     bool useMeasuredFeatures)
{
  if ( useMeasuredFeatures == USE_MEASURED_FEATURES) {
    computeAPosterioriEntropiesOfMeasured( cellList, sensors,
					   grid, occupancyProbs, entropies);
    return 0.0;
  }
  else
    return computeAPosterioriEntropiesAveraged( cellList, sensors,
						grid, occupancyProbs, entropies);
}
    

/***********************************************************************
 * Computes the bayesian a posteriori error for a given probability
 * distribution (grid) represented by the cellList after having apllied
 * the sensor (sensor).
 ***********************************************************************/
static double
computeAPosterioriEntropiesAveraged( realCellList* cellList,
				     abstractSensorVector* sensors,
				     positionProbabilityGrid* grid,
				     probabilityGrid* occupancyProbs,
				     double* entropies)
{
  double averageAPosterioriEntropy = 0.0;
  
  int sensorNumber; 
  int i;           /* index for the positions */
  int feature;     /* different features of the sensor */

  /* probability of a feature given the probabilities in the cell list */
  float pOfFeature[MAX_NUMBER_OF_FEATURES]; 
  /* probability of a feature given that the robot is in a cell */
  float pOfFeatureGivenPosition[MAX_NUMBER_OF_FEATURES][MAX_NUMBER_OF_CELLS]; 

  /* Check wether the position has to be considered. */
  bool considerPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS];

  /* grid position of the real position. */
  gridPosition sensorPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS];

  /*-------------------------------------------------------------- 
   * Set the occupancy probabilities of the positions of the cells
   * in the map. Also compute the grid position of the sensors.
   *--------------------------------------------------------------*/
  for ( i = 0; i < cellList->numberOfCells; i++) {

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
	
	if ( sensors->sensor[sensorNumber].integrationInfo.sensorHasOffset) {
	  
	  realPosition sensPos =
	    endPoint( cellList->cell[i].pos,
		      sensors->sensor[sensorNumber].integrationInfo.sensorOffset);
	  
	  sensorPosition[sensorNumber][i] = gridPositionOfRealPosition( sensPos, grid);

	  /* The position of the sensor might be outside the grid. Check this! */
	  considerPosition[sensorNumber][i] =
	    coordinateInGrid( sensorPosition[sensorNumber][i].x,
			      sensorPosition[sensorNumber][i].y,
			      grid);
	}
	else { /* No shift of the sensor necessary. */

	  sensorPosition[sensorNumber][i] =
	    gridPositionOfRealPosition( cellList->cell[i].pos, grid);

	  /* Yes! consider it! */
	  considerPosition[sensorNumber][i] = TRUE;
	}
      }
    }
  }

  /*--------------------------------------------------------------
   * We check for each sensor independently the entropy and return
   * the minimal entropy.
   *--------------------------------------------------------------*/
  for ( sensorNumber = 0; sensorNumber < sensors->numberOfSensors; sensorNumber++) {

    double aPosterioriEntropy = 0.0;
    abstractSensorType* sensor = &(sensors->sensor[sensorNumber]);
    
    /*--------------------------------------------------------------
     * First compute the probabilities of the features.
     *--------------------------------------------------------------*/
    for ( feature = 0; feature < sensor->numberOfFeatures; feature++) {
      
      pOfFeature[feature] = 0.0;

      for ( i = 0; i < cellList->numberOfCells; i++) {

	if ( considerPosition[sensorNumber][i]) {

	  pOfFeatureGivenPosition[feature][i] =
	    sensor->probOfFeature( feature,
				   sensorPosition[sensorNumber][i],
				   sensor->infoForFeatures);
	}
	else
	  pOfFeatureGivenPosition[feature][i] = 1.0 / sensor->numberOfFeatures;
	
	pOfFeature[feature] +=
	  pOfFeatureGivenPosition[feature][i] * cellList->cell[i].prob;
      }
    }
    
    /*--------------------------------------------------------------
     * Now compute the bayesian a posteriori entropy. 
     *--------------------------------------------------------------*/
    for ( i = 0; i < cellList->numberOfCells; i++) {
      
      /* Now get the conditional probabilities of the different features */
      for ( feature = 0; feature < sensor->numberOfFeatures; feature++) {

	float factor = pOfFeatureGivenPosition[feature][i] * cellList->cell[i].prob;
	
	if ( factor != 0.0) {
/* 	  aPosterioriEntropy += factor * log( pOfFeature[feature] / factor); */
	  aPosterioriEntropy += factor * log( pOfFeature[feature] / pOfFeatureGivenPosition[feature][i]);
	}
      }
    }

    averageAPosterioriEntropy += aPosterioriEntropy;

    entropies[sensorNumber] = aPosterioriEntropy;

  }
  return averageAPosterioriEntropy / sensors->numberOfSensors;
}


/***********************************************************************
 * Computes the bayesian a posteriori error for a given probability
 * distribution (grid) represented by the cellList after having apllied
 * the sensor (sensor). The measured features are given and we don't have
 * to average over all possible features.
 ***********************************************************************/
static void
computeAPosterioriEntropiesOfMeasured( realCellList* cellList,
				       abstractSensorVector* sensors,
				       positionProbabilityGrid* grid,
				       probabilityGrid* occupancyProbs,
				       double* entropies)
{
  int sensorNumber; 
  int i;           /* index for the positions */

  /* Check wether the position has to be considered. */
  bool considerPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS];

  /* grid position of the real position. */
  gridPosition sensorPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS];

  /*-------------------------------------------------------------- 
   * Compute the grid position of the sensors.
   *--------------------------------------------------------------*/
  for ( i = 0; i < cellList->numberOfCells; i++) {

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
		  > POSITION_OCCUPIED_THRESHOLD)) 
	considerPosition[sensorNumber][i] = FALSE; 
      
      else { /* The cell is in the map and it is likely enough. */	
	
	if ( sensors->sensor[sensorNumber].integrationInfo.sensorHasOffset) {
	  
	  realPosition sensPos =
	    endPoint( cellList->cell[i].pos,
		      sensors->sensor[sensorNumber].integrationInfo.sensorOffset);
	  
	  sensorPosition[sensorNumber][i] = gridPositionOfRealPosition( sensPos, grid);
	  
	  /* The position of the sensor might be outside the grid. Check this! */
	  considerPosition[sensorNumber][i] =
	    coordinateInGrid( sensorPosition[sensorNumber][i].x,
			      sensorPosition[sensorNumber][i].y,
			      grid);
	}
	else { /* No shift of the sensor necessary. */

	  sensorPosition[sensorNumber][i] =
	    gridPositionOfRealPosition( cellList->cell[i].pos, grid);

	  /* Yes! consider it! */
	  considerPosition[sensorNumber][i] = TRUE;
	}
      }
    }
  }

  /*--------------------------------------------------------------
   * We check for each sensor independently the entropy and return
   * the minimal entropy.
   *--------------------------------------------------------------*/
  for ( sensorNumber = 0; sensorNumber < sensors->numberOfSensors; sensorNumber++) {

    abstractSensorType* sensor = &(sensors->sensor[sensorNumber]);
    probability aPosterioriProb[MAX_NUMBER_OF_CELLS];
    probability normalizeFactor = 0.0;

    entropies[sensorNumber] = 0.0;
	 
    /*--------------------------------------------------------------
     * Now compute the bayesian a posteriori error. 
     *--------------------------------------------------------------*/

    /* Compute the new probabilities after having integrated the
     * measured feature. */
    for ( i = 0; i < cellList->numberOfCells; i++) {

      if ( considerPosition[sensorNumber][i]) {
	aPosterioriProb[i] = cellList->cell[i].prob *
	  sensor->probOfFeature( sensor->measuredFeature,
				 sensorPosition[sensorNumber][i],
				 sensor->infoForFeatures); }
      else {
	aPosterioriProb[i] = cellList->cell[i].prob / sensor->numberOfFeatures;
      }
      normalizeFactor += aPosterioriProb[i];
    }

    /* Compute the entropy of the nomralized probabilities. */
    if ( normalizeFactor > 0.0) {
      for (i = 0; i < cellList->numberOfCells; i++) {
	if ( aPosterioriProb[i] > 0.0) {
	  aPosterioriProb[i] /= normalizeFactor;
/* 	  entropies[sensorNumber] -= aPosterioriProb[i] * */
/* 	    log(aPosterioriProb[i]); */
	  entropies[sensorNumber] -= aPosterioriProb[i] *
	    log(aPosterioriProb[i] / cellList->cell[i].prob);
	}
      }
    }
    else {
      /* Set the maximal entropy. */
/*       entropies[sensorNumber] = log( cellList->numberOfCells); */
      entropies[sensorNumber] = 0.0;
    }
  }
}



void
computeLikelihoods( realCellList* cellList,
		    abstractSensorVector* sensors,
		    positionProbabilityGrid* grid,
		    float* likelihoods)
{
  int sensorNumber; 
  int i;           /* index for the positions */

  /* Check wether the position has to be considered. */
  bool considerPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS];

  /* grid position of the real position. */
  gridPosition sensorPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS];

  /*-------------------------------------------------------------- 
   * Compute the grid position of the sensors.
   *--------------------------------------------------------------*/
  for ( i = 0; i < cellList->numberOfCells; i++) {

    for ( sensorNumber = 0; sensorNumber < sensors->numberOfSensors; sensorNumber++) {
      
      /* For each sensor and each position we must get the sensor position in
       * grid coordinates and check wether the sensor should be considered. */
      if ( ! cellList->inMap[i]) 
	considerPosition[sensorNumber][i] = FALSE; 
      else { /* The cell is in the map and it is likely enough. */	
	
	if ( sensors->sensor[sensorNumber].integrationInfo.sensorHasOffset) {
	  
	  realPosition sensPos =
	    endPoint( cellList->cell[i].pos,
		      sensors->sensor[sensorNumber].integrationInfo.sensorOffset);
	  
	  sensorPosition[sensorNumber][i] = gridPositionOfRealPosition( sensPos, grid);
	  
	  /* The position of the sensor might be outside the grid. Check this! */
	  considerPosition[sensorNumber][i] =
	    coordinateInGrid( sensorPosition[sensorNumber][i].x,
			      sensorPosition[sensorNumber][i].y,
			      grid);
	}
	else { /* No shift of the sensor necessary. */

	  sensorPosition[sensorNumber][i] =
	    gridPositionOfRealPosition( cellList->cell[i].pos, grid);

	  /* Yes! consider it! */
	  considerPosition[sensorNumber][i] = TRUE;
	}
      }
    }
  }

  /*--------------------------------------------------------------
   *--------------------------------------------------------------*/
  for ( sensorNumber = 0; sensorNumber < sensors->numberOfSensors; sensorNumber++) {

    abstractSensorType* sensor = &(sensors->sensor[sensorNumber]);

    likelihoods[sensorNumber] = 0.0;
    
    /*--------------------------------------------------------------
     * Now compute the bayesian a posteriori error. 
     *--------------------------------------------------------------*/

    /* Compute the new probabilities after having integrated the
     * measured feature. */
    for ( i = 0; i < cellList->numberOfCells; i++) {
      
      if ( considerPosition[sensorNumber][i]) 
	likelihoods[sensorNumber] += cellList->cell[i].prob *
	  sensor->probOfFeature( sensor->measuredFeature,
				 sensorPosition[sensorNumber][i],
				 sensor->infoForFeatures);
      else
	likelihoods[sensorNumber] += cellList->cell[i].prob / sensor->numberOfFeatures;
    }
  }
}


void
computeMaxLikelihoods( realCellList* cellList,
		       abstractSensorVector* sensors,
		       positionProbabilityGrid* grid,
		       float* likelihoods)
{
  int sensorNumber; 
  int i;           /* index for the positions */
  
  /* Check wether the position has to be considered. */
  bool considerPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS];
  
  /* grid position of the real position. */
  gridPosition sensorPosition[MAX_NUMBER_OF_SENSORS][MAX_NUMBER_OF_CELLS];
  
  /*-------------------------------------------------------------- 
   * Compute the grid position of the sensors.
   *--------------------------------------------------------------*/
  for ( i = 0; i < cellList->numberOfCells; i++) {
    
    for ( sensorNumber = 0; sensorNumber < sensors->numberOfSensors; sensorNumber++) {
      
      /* For each sensor and each position we must get the sensor position in
       * grid coordinates and check wether the sensor should be considered. */
      if ( ! cellList->inMap[i]) 
	considerPosition[sensorNumber][i] = FALSE; 
      else { /* The cell is in the map and it is likely enough. */	
	
	if ( sensors->sensor[sensorNumber].integrationInfo.sensorHasOffset) {
	  
	  realPosition sensPos =
	    endPoint( cellList->cell[i].pos,
		      sensors->sensor[sensorNumber].integrationInfo.sensorOffset);
	  
	  sensorPosition[sensorNumber][i] = gridPositionOfRealPosition( sensPos, grid);
	  
	  /* The position of the sensor might be outside the grid. Check this! */
	  considerPosition[sensorNumber][i] =
	    coordinateInGrid( sensorPosition[sensorNumber][i].x,
			      sensorPosition[sensorNumber][i].y,
			      grid);
	}
	else { /* No shift of the sensor necessary. */

	  sensorPosition[sensorNumber][i] =
	    gridPositionOfRealPosition( cellList->cell[i].pos, grid);

	  /* Yes! consider it! */
	  considerPosition[sensorNumber][i] = TRUE;
	}
      }
    }
  }

  /*--------------------------------------------------------------
   *--------------------------------------------------------------*/
  for ( sensorNumber = 0; sensorNumber < sensors->numberOfSensors; sensorNumber++) {

    abstractSensorType* sensor = &(sensors->sensor[sensorNumber]);

    likelihoods[sensorNumber] = 0.0;
    
    /*--------------------------------------------------------------
     * Now compute the maximal likelihood.
     *--------------------------------------------------------------*/

    for ( i = 0; i < cellList->numberOfCells; i++) {

      float tmp;
      
      if ( considerPosition[sensorNumber][i]) 
	tmp = cellList->cell[i].prob *
	  sensor->probOfFeature( sensor->measuredFeature,
				 sensorPosition[sensorNumber][i],
				 sensor->infoForFeatures);
      else
	tmp = cellList->cell[i].prob / sensor->numberOfFeatures;
      
      if ( tmp > likelihoods[sensorNumber])
	likelihoods[sensorNumber] = tmp;
    }
  }
}


/***********************************************************************
 * Computes the bayesian a posteriori error for a given probability
 * distribution (grid) represented by the cellList after having apllied
 * the sensor (sensor).
 ***********************************************************************/
double
pOfFeatures( abstractSensorVector* sensors,
	     positionProbabilityGrid* grid,
	     probabilityGrid* occupancyProbs,
	     double* entropies)
{
  double averageAPosterioriEntropy = 0.0;
  
  int x, y, z, cnt = 0; 
  int feature;     /* different features of the sensor */

  /* probability of a feature given the probabilities in the cell list */
  float pOfFeature[MAX_NUMBER_OF_FEATURES]; 

  /* grid position of the real position. */
  gridPosition pos;

  abstractSensorType* sensor = &(sensors->sensor[0]);

  fprintf(stderr, "start it\n");
  for ( feature = 0; feature < sensor->numberOfFeatures; feature++)
    pOfFeature[feature] = 0.0;
  
  /*-------------------------------------------------------------- 
   * Set the occupancy probabilities of the positions of the cells
   * in the map. Also compute the grid position of the sensors.
   *--------------------------------------------------------------*/
  for ( x = 0; x < grid->sizeX / 1; x++) {
    pos.x = x;
    fprintf(stderr, "x: %d\n", x);
    for ( y = 0; y < grid->sizeY / 1; y++) {
      pos.y = y;
      for ( z = 0; z < grid->sizeZ / 1; z++) {
	pos.rot = z;
	  
	if ( occupancyProbs->prob[x][y] > 0.5) {
	  cnt++;
	  for ( feature = 0; feature < sensor->numberOfFeatures; feature++)

	    pOfFeature[feature] += sensor->probOfFeature( feature,
							  pos,
							  sensor->infoForFeatures);
	  }
	else if (0) 
	  fprintf(stderr, "no %d %d %d  -> %f\n",
		  x, y, z, occupancyProbs->prob[x][y]);
      }
    }
  }

  for ( feature = 0; feature < sensor->numberOfFeatures; feature++)
    fprintf(stderr, "%d %f\n", feature, pOfFeature[feature] / cnt);
}

