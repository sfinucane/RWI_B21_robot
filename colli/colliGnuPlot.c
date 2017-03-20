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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/colli/colliGnuPlot.c,v $
 *****
 ***** Created by:      $Author: rstone $
 *****
 ***** Revision #:      $Revision: 1.1 $
 *****
 ***** Date of revision $Date: 2002/09/14 15:25:05 $
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *
 *                  ----- REVISION HISTORY -----
 *
 * $Log: colliGnuPlot.c,v $
 * Revision 1.1  2002/09/14 15:25:05  rstone
 * *** empty log message ***
 *
 * Revision 1.5  1998/08/26 23:23:40  fox
 * Final version before I changed exception handling.
 *
 * Revision 1.4  1998/08/18 16:24:23  fox
 * Added support for b18 robot.
 *
 * Revision 1.3  1998/05/15 08:29:33  fox
 * Removed error message for base command BASE_confirmCommand.
 *
 * Revision 1.2  1998/05/13 07:07:40  fox
 * Fixed some bugs I found due to graphical output.
 *
 * Revision 1.1.1.1  1996/09/22 16:46:04  rhino
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



#include <stdio.h>
#include "collisionIntern.h"


static int dumpStep = 0;

FILE* angleFile;
FILE* velocityFile;
FILE* distanceFile;
FILE* noSmoothFile;
FILE* smoothFile;

#define angFile "angles"
#define velFile "velocities"
#define distFile "distances"
#define noSmFile "nosmooth"
#define smFile "smooth"

#define DUMP_ANGLES 1
#define DUMP_VELOCITIES 1
#define DUMP_DISTANCES 1
#define DUMP_NOSMOOTH 1
#define DUMP_SMOOTH 1


static void
openDumpFiles( int cnt)
{
   char fName[80];

   if (DUMP_ANGLES) {
      sprintf( fName, "%s.dump.%d", angFile, cnt);
      angleFile    = fopen( fName, "w"); 
   }
   
   if (DUMP_VELOCITIES) {
      sprintf( fName, "%s.dump.%d", velFile, cnt); 
      velocityFile    = fopen( fName, "w");
   }
   
   if (DUMP_DISTANCES) {
      sprintf( fName, "%s.dump.%d", distFile, cnt); 
      distanceFile    = fopen( fName, "w");
   }
   
   if (DUMP_NOSMOOTH) {
      sprintf( fName, "%s.dump.%d", noSmFile, cnt); 
      noSmoothFile    = fopen( fName, "w");
   }

   if (DUMP_SMOOTH) {
      sprintf( fName, "%s.dump.%d", smFile, cnt); 
      smoothFile    = fopen( fName, "w");
   }

}

    
static void
closeFiles()
{
    if (DUMP_ANGLES) 
       fclose( angleFile); 
    if (DUMP_VELOCITIES) 
       fclose( velocityFile); 
    if (DUMP_DISTANCES) 
       fclose( distanceFile); 
}


static void
dump( VelocityCombination** combinations,
      float** values, float factor, int smooth, FILE* file,
      int numberOfRvels, int numberOfTvels )
{
   int i,j;
   
   fprintf( file, "# %d %d %f %d rv tv factor smooth\n", numberOfRvels, numberOfTvels, factor, smooth);
   
   fprintf( file, "# %f %f %f %f %f %f\n",
	    RAD_TO_DEG(actual_velocities.current_rvel),
	    actual_velocities.current_tvel,
	    RAD_TO_DEG(actual_velocities.min_rvel),
	    RAD_TO_DEG(actual_velocities.max_rvel),
	    actual_velocities.min_tvel, actual_velocities.max_tvel);
   
   for ( i = 0; i < numberOfRvels; i++) {
     
     for ( j = 0; j < numberOfTvels; j++) {
	 
       if ( values[i][j] == UNDEFINED) 
	 fprintf( file, "%f\t%f\t0.0\n", 
		  RAD_TO_DEG( combinations[i][j].rvel), 
		  combinations[i][j].tvel);
       else
	 fprintf( file, "%f\t%f\t%f\n", 
		  RAD_TO_DEG( combinations[i][j].rvel), 
		  combinations[i][j].tvel, values[i][j]); 
     }
     fprintf( file, "\n");
   } 
}

void COLLI_StartToDumpGnuPlot( double step)
{
  dumpStep = (int) step;
}



void
COLLI_DumpEvaluationFunction( VelocityCombination** combinations,
			      float** Velocities, float** Distances,
			      float** Angles,
			      float** noSmooth, float** smooth,
			      float velFactor,
			      float distFactor, float angFactor,
			      int smoothSize,
			      int numberOfRvels, int numberOfTvels)
{
  if ( dumpStep > 0) {
    
    static int cnt = 0;
    
    if (++cnt % dumpStep == 0) {
      
      openDumpFiles( cnt);
      
      if (DUMP_ANGLES) 
	dump( combinations, Angles, angFactor, smoothSize,
	      angleFile,
	      numberOfRvels, numberOfTvels);
      if (DUMP_VELOCITIES) 
	dump( combinations, Velocities, velFactor, smoothSize,
	      velocityFile,
	      numberOfRvels, numberOfTvels);
      if (DUMP_DISTANCES) 
	dump( combinations, Distances, distFactor, smoothSize,
	      distanceFile,
	      numberOfRvels, numberOfTvels);
      if (DUMP_NOSMOOTH)
	if ( noSmooth != NULL)
	  dump( combinations, noSmooth, distFactor, smoothSize,
		noSmoothFile,
		numberOfRvels, numberOfTvels);

      if (DUMP_SMOOTH)
	if ( smooth != NULL)
	  dump( combinations, smooth, distFactor, smoothSize,
		smoothFile,
		numberOfRvels, numberOfTvels);
	
      closeFiles();
    }
  }
}

#ifdef GNUPLOT

#define NUMBER_OF_GNUPLOT_RVELS 100
#define NUMBER_OF_GNUPLOT_TVELS 60

#define MIN_TVEL 0.0
#define MAX_TVEL 90.0
#define MIN_RVEL -DEG_90
#define MAX_RVEL DEG_90

#define MAX_VEL 90.0
#define MAX_ANG DEG_180
#define MAX_DIST MAX_RANGE



static void
generateGnuPlotVelocities( VelocityCombination** combinations,
			   float minR, float maxR,
			   float minT, float maxT,
			   int numberOfRvels,
			   int numberOfTvels)
{
  
  int i, j;
  
  float tvelStep = (maxT - minT) / (numberOfTvels - 1.0);
  float rvelStep = (maxR - minR) / (numberOfRvels - 1.0);
  
  for ( i = 0; i < numberOfRvels; i++) {
    for ( j = 0; j < numberOfTvels; j++) {
      combinations[i][j].tvel = minT + j * tvelStep;
      combinations[i][j].rvel = minR + i * rvelStep;
    }
  }
}
		    
		
/**********************************************************************
 * Evaluates the different combinations of tvel and rvel.
 * Stores the values in the three matrices.
 **********************************************************************/
static void
evaluateGnuPlotCombinations( Point rpos, float rrot,
			     VelocityCombination** combinations,
			     float** velocities,
			     float** distances,
			     float** angles,
			     float** smooth,
			     float** noSmooth,
			     int iDim,
			     int jDim)
{
  int i, j;
  float collDist, targetDist;
  
  /* We don't want the robot to move very slow. Thats why we set some combinations
   * to undefined. To determine the slow combinations we set the fraction of
   * velocities to be deleted. */
  
  for ( i = 0; i < iDim; i++) {
    
    for ( j = 0; j < jDim; j++) {
      
      float tvel = combinations[i][j].tvel;
      float rvel = combinations[i][j].rvel;

      if ( ! admissible( rpos,
			 rrot, 
			 tvel, 
			 rvel,
			 FALSE, FALSE,
			 &collDist,
			 &targetDist)) {

	velocities[i][j] = UNDEFINED;
	angles[i][j]     = UNDEFINED;
	distances[i][j]  = UNDEFINED;
	noSmooth[i][j]   = UNDEFINED;
	smooth[i][j]   = UNDEFINED;
      }
      else 
	{ 
	  velocities[i][j] = velocityEvaluation( tvel) / MAX_VEL;

	  angles[i][j]     = angleEvaluation( rpos,
					      rrot,
					      tvel,
					      rvel) / MAX_ANG;
	  

	  distances[i][j]  = distanceEvaluation( tvel,
						 rvel,
						 collDist,
						 targetDist) / MAX_DIST;

	  noSmooth[i][j]   =   VELOCITY_FACTOR * velocities[i][j] +
	    ANGLE_FACTOR * angles[i][j] +
	    DISTANCE_FACTOR * distances[i][j];
	  smooth[i][j]     = noSmooth[i][j];
	}
    }
  }
  /* Now let's smooth the histogram of the best values. */
  smoothGrid( smooth,
	      ACTUAL_MODE->number_of_rvels,
	      ACTUAL_MODE->number_of_tvels,
	      ACTUAL_MODE->smooth_width);
  

}

    
/**********************************************************************
 * Computes the best traectory to the target point. 
 **********************************************************************/
void
computeGnuPlotTrajectory(Point rpos, float rrot)
{
  static VelocityCombination** combinations;
  float** angles;
  float** distances;
  float** velocities;
  float** values;
  static float** smooth;
  static float** noSmooth;

  static int firstTime = TRUE;
  static int numberOfRVels, numberOfTVels;
  int i;

  dumpStep = 1;

  fprintf(stderr, "Dump gnuplot.\n");
  
  if ( firstTime) {
    firstTime = FALSE;

    /* Scale the number of velocities to have the right number in the
       dynamic window. */
    numberOfRVels = ACTUAL_MODE->number_of_rvels * (MAX_RVEL - MIN_RVEL) / 
      (ACCELERATION_ASSERTION_TIME * DEG_TO_RAD( rwi_base.rot_acceleration));

    numberOfTVels = ACTUAL_MODE->number_of_tvels * (MAX_TVEL) / 
      (ACCELERATION_ASSERTION_TIME * rwi_base.trans_acceleration);

    /* Let's create the smooth grid and the combinations by hand. */
    smooth = (float **) malloc(numberOfRVels * sizeof(float *));
    for (i = 0; i < numberOfRVels; i++) 
      smooth[i] = (float *) malloc( numberOfTVels * sizeof(float));
    
    /* Let's create the smooth grid and the combinations by hand. */
    noSmooth = (float **) malloc(numberOfRVels * sizeof(float *));
    for (i = 0; i < numberOfRVels; i++) 
      noSmooth[i] = (float *) malloc( numberOfTVels * sizeof(float));
    
    combinations = (VelocityCombination **)
      malloc( numberOfRVels * sizeof(VelocityCombination *));
    for (i = 0; i < numberOfRVels; i++) 
      combinations[i] = (VelocityCombination *)
	malloc( numberOfTVels * sizeof(VelocityCombination));
  }

  allocateEvaluations( &velocities, &distances, &angles, &values,
		       numberOfRVels, numberOfTVels);
  
  generateGnuPlotVelocities( combinations,
			     MIN_RVEL, MAX_RVEL,
			     MIN_TVEL, MAX_TVEL,
			     numberOfRVels,
			     numberOfTVels);

  /* Uses the evaluation function for each of the combinations.
   * Also checks wether there is one combination admissible
   * which includes translation. */
  evaluateGnuPlotCombinations( rpos, rrot,
			       combinations,
			       velocities,
			       distances,
			       angles,
			       noSmooth,
			       smooth,
			       numberOfRVels,
			       numberOfTVels);
  
  COLLI_DumpEvaluationFunction( combinations, 
				velocities, distances,
				angles,
				noSmooth, smooth,
				ACTUAL_MODE->velocity_factor,
				ACTUAL_MODE->distance_factor,
				ACTUAL_MODE->angle_factor,
				ACTUAL_MODE->smooth_width,
				numberOfRVels,
				numberOfTVels);

  
#ifdef TWO_VELOCITIES
  COLLI_SetMode(1); 
  
  /* Uses the evaluation function for each of the combinations.
   * Also checks wether there is one combination admissible
   * which includes translation. */
  evaluateGnuPlotCombinations( rpos, rrot,
			       combinations,
			       velocities,
			       distances,
			       angles,
			       numberOfRVels,
			       numberOfTVels);
  
  dumpStep = 1;
  COLLI_DumpEvaluationFunction( combinations, 
				velocities, distances,
				angles,
				ACTUAL_MODE->velocity_factor,
				ACTUAL_MODE->distance_factor,
				ACTUAL_MODE->angle_factor,
				ACTUAL_MODE->smooth_width,
				numberOfRVels,
				numberOfTVels);
  exit(1);
#endif
}

#endif




















