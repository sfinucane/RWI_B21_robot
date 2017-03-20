
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
 ***** Source file:     $Source: /usr/local/cvs/bee/src/localize/probFunction.c,v $
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
 * $Log: probFunction.c,v $
 * Revision 1.1  2002/09/14 17:06:29  rstone
 * *** empty log message ***
 *
 * Revision 1.5  1997/12/11 17:06:31  fox
 * Added some parameters.
 *
 * Revision 1.4  1997/01/18 18:19:22  wolfram
 * *** empty log message ***
 *
 * Revision 1.3  1996/12/02 10:32:10  wolfram
 * Expected distance file now includes distances + variances
 *
 * Revision 1.2  1996/10/24 12:07:12  fox
 * Fixed a bug.
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
#include "probFunction.h"
#include "proximityTools.h"
#include "function.h"

static void
testGlobalSonarProbFunctionParameters( float maxDistance,
				       float min,
				       float closeSigma,
				       float farSigma,
				       float closeFactor,
				       float farFactor)
{
  int x, y;
  int xStep, yStep;
  int numberOfExpectedDistances, numberOfMeasuredDistances;
  
  FILE* fp = fopen( DEFAULT_PROBABILITY_FILE, "w");

  /* The precomputed probability function for the sonars. */
  sonarProbabilityTable sonarProbabilities;
  
  /* This has to be given to the function. */
  expectedDistanceTable expDist;
  expDist.distanceResolution = MAX_EXPECTED_DISTANCE /
    (NUMBER_OF_EXPECTED_DISTANCES - 1);
  expDist.maxDistanceIndex = (NUMBER_OF_EXPECTED_DISTANCES - 1);

  setGlobalSonarProbFunctionParameters( min,
					closeSigma,
					farSigma,
					closeFactor,
					farFactor,
					maxDistance); 
  
  sonarProbabilities = preprocessedProbabilityFunctionTable( maxDistance,
							     &expDist);

  printf( "Write data into prob.fct ... \n");
  
  fprintf( fp, "#maxDistance        %f\n", maxDistance);
  fprintf( fp, "#min                %f\n", min);
  fprintf( fp, "#closeSigma         %f\n",closeSigma );
  fprintf( fp, "#farSigma           %f\n", farSigma);
  fprintf( fp, "#closeFactor        %f\n", closeFactor);
  fprintf( fp, "#farFactor          %f\n", farFactor);

  numberOfExpectedDistances = sonarProbabilities.expectedMaxDistanceIndex + 1;
  numberOfMeasuredDistances = sonarProbabilities.measuredMaxDistanceIndex + 1;
  
  xStep = iMax( 1, numberOfExpectedDistances / 40);
  yStep = iMax( 1, numberOfMeasuredDistances / 40);

  for ( x = 0; x < numberOfExpectedDistances; x += xStep) {
    for ( y = 0; y < numberOfMeasuredDistances; y += yStep) {
      fprintf( fp, "%f\n", sonarProbabilities.prob[x][y]);
    }
    if ( y != numberOfMeasuredDistances-1+yStep)
      fprintf( fp, "%f\n", sonarProbabilities.prob[x][numberOfMeasuredDistances-1]);
    fprintf( fp, "\n");
  }
  if ( x != numberOfExpectedDistances-1+xStep) {
    for ( y = 0; y < numberOfMeasuredDistances; y += yStep) 
      fprintf( fp, "%f\n", sonarProbabilities.prob[numberOfExpectedDistances-1][y]);
    if ( y != numberOfMeasuredDistances-1+yStep)
      fprintf( fp, "%f\n", sonarProbabilities.prob[numberOfExpectedDistances-1][numberOfMeasuredDistances-1]);
    }
  
  fclose( fp);
  printf( "done.\n");
  return;
 
}

int
main( int argc, char** argv)
{
  int nextArgument = 1;
  
  float maxDistance;
  float min;
  float closeSigma;
  float farSigma;
  float closeFactor;
  float farFactor;
 
  if ( argc != 7) {
    printf( "usage: probFunction <maxDistance> <minimum>\n");
    printf( "                    <closeSigma> <farSigma> <closeFactor> <farFactor>\n");
    closeLogAndExit( 1);
  }

  sscanf( argv[nextArgument++], "%f", &maxDistance);
  sscanf( argv[nextArgument++], "%f", &min);
  sscanf( argv[nextArgument++], "%f", &closeSigma);
  sscanf( argv[nextArgument++], "%f", &farSigma);
  sscanf( argv[nextArgument++], "%f", &closeFactor);
  sscanf( argv[nextArgument++], "%f", &farFactor);
  
  testGlobalSonarProbFunctionParameters( maxDistance,
					 min,
					 closeSigma,
					 farSigma,
					 closeFactor,
					 farFactor);
  closeLogAndExit(1);
}

